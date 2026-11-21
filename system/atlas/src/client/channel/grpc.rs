// SPDX-License-Identifier: MulanPSL-2.0
//! Typed protobuf communication used by generated contract bindings.

use super::{Channel, ChannelContract};
use crate::pb;
use anyhow::{Context, Result, ensure};
use futures_util::{Stream, StreamExt};
use prost::Message;
use std::pin::Pin;
use tonic::codegen::http::uri::PathAndQuery;

/// An initialized HTTP/2 connection and the authoritative Atlas method route.
#[derive(Clone)]
pub(super) struct Binding {
    /// Shared transport; execution ownership remains with each Channel.
    connection: tonic::transport::Channel,
    /// Method selected by the provider's Atlas declaration.
    path: PathAndQuery,
}

impl Binding {
    pub async fn connect<C: ChannelContract>(
        endpoint: &str,
        params: &pb::TransportParams,
        contract: &pb::ContractDescriptor,
    ) -> Result<Self> {
        let Some(pb::transport_params::Kind::Grpc(params)) = &params.kind else {
            anyhow::bail!("missing gRPC transport parameters");
        };
        ensure!(
            params.service_name == C::SERVICE
                || (!C::ID.is_empty() && contract.id != C::ID && C::matches_contract(contract)),
            "gRPC service does not match channel binding"
        );
        let route = format!("/{}/{}", params.service_name, C::METHOD);
        ensure!(
            params.method == C::METHOD || params.method == route,
            "gRPC method does not match channel binding"
        );
        let connection =
            tonic::transport::Endpoint::new(super::super::normalize_grpc_endpoint(endpoint))?
                .connect()
                .await
                .context("gRPC connection failed")?;
        Ok(Self {
            connection,
            path: route.parse()?,
        })
    }
}

type Messages<T> = Pin<Box<dyn Stream<Item = T> + Send>>;

async fn client<C: ChannelContract>(
    channel: &Channel<C>,
) -> Result<(tonic::client::Grpc<tonic::transport::Channel>, PathAndQuery)> {
    let binding = channel
        .execution
        .grpc
        .lock()
        .await
        .clone()
        .context("gRPC channel unavailable")?;
    let mut client = tonic::client::Grpc::new(binding.connection);
    client
        .ready()
        .await
        .map_err(|error| anyhow::anyhow!("gRPC not ready: {error}"))?;
    Ok((client, binding.path))
}

pub async fn unary<C, Req, Res>(channel: Channel<C>, request: Req) -> Result<Res>
where
    C: ChannelContract<Output = Res>,
    Req: Message + Default + 'static,
    Res: Message + Default + 'static,
{
    let (mut client, path) = client(&channel).await?;
    let response = client
        .unary(
            tonic::Request::new(request),
            path,
            tonic_prost::ProstCodec::<Req, Res>::default(),
        )
        .await?
        .into_inner();
    // A decoded synchronous response confirms completion before business interpretation.
    if !channel.is_async() {
        channel.execution.progress.lock().unwrap().terminal = true;
    }
    let (accepted, run_id) = C::execution(&response)?;
    channel.complete(accepted, run_id);
    Ok(response)
}

pub async fn server_stream<C, Req, Res>(
    channel: Channel<C>,
    request: Req,
) -> Result<Messages<Result<Res>>>
where
    C: ChannelContract,
    Req: Message + Default + 'static,
    Res: Message + Default + 'static,
{
    let (mut client, path) = client(&channel).await?;
    let response = client
        .server_streaming(
            tonic::Request::new(request),
            path,
            tonic_prost::ProstCodec::<Req, Res>::default(),
        )
        .await?;
    Ok(responses(channel, response.into_inner()))
}

pub async fn client_stream<C, Req, Res>(channel: Channel<C>, request: Messages<Req>) -> Result<Res>
where
    C: ChannelContract<Output = Res>,
    Req: Message + Default + 'static,
    Res: Message + Default + 'static,
{
    let (mut client, path) = client(&channel).await?;
    let request = requests(&channel, request);
    let response = client
        .client_streaming(
            tonic::Request::new(request),
            path,
            tonic_prost::ProstCodec::<Req, Res>::default(),
        )
        .await?
        .into_inner();
    channel.complete(true, String::new());
    Ok(response)
}

pub async fn bidirectional_stream<C, Req, Res>(
    channel: Channel<C>,
    request: Messages<Req>,
) -> Result<Messages<Result<Res>>>
where
    C: ChannelContract,
    Req: Message + Default + 'static,
    Res: Message + Default + 'static,
{
    let (mut client, path) = client(&channel).await?;
    let request = requests(&channel, request);
    let response = client
        .streaming(
            tonic::Request::new(request),
            path,
            tonic_prost::ProstCodec::<Req, Res>::default(),
        )
        .await?;
    Ok(responses(channel, response.into_inner()))
}

fn requests<C: ChannelContract, T: Send + 'static>(
    channel: &Channel<C>,
    input: Messages<T>,
) -> Messages<T> {
    let stop = channel.execution.stop_stream.subscribe();
    Box::pin(futures_util::stream::unfold(
        (input, stop),
        |(mut input, mut stop)| async move {
            if *stop.borrow() {
                return None;
            }
            let message = tokio::select! {
                _ = stop.changed() => return None,
                message = input.next() => message?,
            };
            Some((message, (input, stop)))
        },
    ))
}

fn responses<C: ChannelContract, T: Message + Default + 'static>(
    channel: Channel<C>,
    mut input: tonic::Streaming<T>,
) -> Messages<Result<T>> {
    let (sender, receiver) = tokio::sync::mpsc::channel(16);
    tokio::spawn(async move {
        let mut stop = channel.execution.stop_stream.subscribe();
        loop {
            let message = match input.message().await {
                Ok(Some(message)) => message,
                Ok(None) => {
                    channel.execution.progress.lock().unwrap().terminal = true;
                    return;
                }
                Err(error) => {
                    let error = channel.retained(&format!("gRPC stream failed: {error}"));
                    tokio::select! {
                        _ = stop.changed() => {},
                        _ = sender.send(Err(error)) => {},
                    }
                    return;
                }
            };
            if !*stop.borrow() && !sender.is_closed() {
                tokio::select! {
                    _ = stop.changed() => {},
                    _ = sender.send(Ok(message)) => {},
                }
            }
        }
    });
    Box::pin(tokio_stream::wrappers::ReceiverStream::new(receiver))
}
