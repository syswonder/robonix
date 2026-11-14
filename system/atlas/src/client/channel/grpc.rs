// SPDX-License-Identifier: MulanPSL-2.0
//! Bind gRPC routes and messages from deployment-generated descriptors.

use crate::pb;
use anyhow::{Context, Result, ensure};
use prost::Message;
use prost_reflect::{DescriptorPool, DynamicMessage, MessageDescriptor, MethodDescriptor};
use serde_json::Value;
use tonic::codec::{Codec, DecodeBuf, Decoder, EncodeBuf, Encoder};

/// A connected gRPC method and the generated types of its wire messages.
#[derive(Clone)]
pub(super) struct Binding {
    /// Reusable HTTP/2 connection; allocations still belong to individual executions.
    pub connection: tonic::transport::Channel,
    /// Input/output types and streaming flags from the generated descriptor.
    pub method: MethodDescriptor,
    /// Authoritative RPC route returned by Atlas.
    pub path: tonic::codegen::http::uri::PathAndQuery,
}

impl Binding {
    pub async fn connect(
        endpoint: &str,
        params: &pb::TransportParams,
        pool: &DescriptorPool,
        contract: &pb::ContractDescriptor,
    ) -> Result<Self> {
        let Some(pb::transport_params::Kind::Grpc(params)) = &params.kind else {
            anyhow::bail!("missing gRPC transport parameters");
        };
        let service = pool.get_service_by_name(&params.service_name).or_else(|| {
            // Legacy namespace Drivers use the shared lifecycle IDL and their own route.
            if contract.mode == "rpc" && contract.io_srv_type == "lifecycle/srv/Driver"
                && contract.id.ends_with("/driver")
                && params.method.rsplit('/').next() == Some("Driver")
            {
                pool.get_service_by_name("robonix.contracts.RobonixLifecycleDriver")
            } else { None }
        }).with_context(|| format!("missing generated gRPC service {}; load its descriptor with add_descriptor_set()", params.service_name))?;
        let name = params.method.rsplit('/').next().unwrap_or_default();
        let method = service
            .methods()
            .find(|m| m.name() == name)
            .with_context(|| format!("unknown gRPC method: {}", params.method))?;
        let route = format!("/{}/{}", params.service_name, method.name());
        ensure!(
            params.method == route || params.method == name,
            "gRPC method does not match service"
        );
        let mode = contract.mode.as_str();
        let flags = match mode {
            "rpc" => (false, false),
            "rpc_server_stream" | "topic_out" => (false, true),
            "rpc_client_stream" | "topic_in" => (true, false),
            "rpc_bidirectional_stream" => (true, true),
            _ => anyhow::bail!("unsupported gRPC contract mode: {mode}"),
        };
        ensure!(
            flags == (method.is_client_streaming(), method.is_server_streaming()),
            "gRPC method does not match contract mode: {mode}"
        );
        let connection =
            tonic::transport::Endpoint::new(super::super::normalize_grpc_endpoint(endpoint))?
                .connect()
                .await
                .context("gRPC connection failed")?;
        Ok(Self {
            connection,
            method,
            path: route.parse()?,
        })
    }

    pub async fn client(&self) -> Result<tonic::client::Grpc<tonic::transport::Channel>> {
        let mut client = tonic::client::Grpc::new(self.connection.clone());
        client
            .ready()
            .await
            .map_err(|e| anyhow::anyhow!("gRPC not ready: {e}"))?;
        Ok(client)
    }

    pub fn request(&self, value: &Value) -> Result<DynamicMessage> {
        DynamicMessage::deserialize(self.method.input(), value).context("invalid gRPC request")
    }

    pub fn codec(&self) -> MessageCodec {
        MessageCodec(self.method.output())
    }
}

pub(super) fn response(message: &DynamicMessage) -> Result<Value> {
    let options = prost_reflect::SerializeOptions::new()
        .use_proto_field_name(true)
        .skip_default_fields(false);
    let mut data = Vec::new();
    message.serialize_with_options(&mut serde_json::Serializer::new(&mut data), &options)?;
    Ok(serde_json::from_slice(&data)?)
}

/// Dynamic decoder bound to the response type of one generated method.
pub(super) struct MessageCodec(
    /// Wire message descriptor required to construct a dynamic response.
    MessageDescriptor,
);

impl Codec for MessageCodec {
    type Encode = DynamicMessage;
    type Decode = DynamicMessage;
    type Encoder = MessageEncoder;
    type Decoder = MessageDecoder;
    fn encoder(&mut self) -> Self::Encoder {
        MessageEncoder
    }
    fn decoder(&mut self) -> Self::Decoder {
        MessageDecoder(self.0.clone())
    }
}

/// Encode descriptor-validated request messages into protobuf frames.
pub(super) struct MessageEncoder;
impl Encoder for MessageEncoder {
    type Item = DynamicMessage;
    type Error = tonic::Status;
    fn encode(&mut self, item: Self::Item, dst: &mut EncodeBuf<'_>) -> Result<(), Self::Error> {
        item.encode(dst)
            .map_err(|e| tonic::Status::internal(e.to_string()))
    }
}

/// Decode protobuf frames with the generated response descriptor.
pub(super) struct MessageDecoder(
    /// Response shape selected by the RPC binding.
    MessageDescriptor,
);
impl Decoder for MessageDecoder {
    type Item = DynamicMessage;
    type Error = tonic::Status;
    fn decode(&mut self, src: &mut DecodeBuf<'_>) -> Result<Option<Self::Item>, Self::Error> {
        DynamicMessage::decode(self.0.clone(), src)
            .map(Some)
            .map_err(|e| tonic::Status::internal(e.to_string()))
    }
}
