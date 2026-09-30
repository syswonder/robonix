// SPDX-License-Identifier: MulanPSL-2.0
//! Incremental gRPC input adapter for the shared RTDL execution runtime.
use crate::execution::{Actor, Input};
use crate::pb::contracts::robonix_system_executor_execute_stream_server::RobonixSystemExecutorExecuteStream;
use crate::pb::executor::{ExecuteStreamInput, RtdlEvent};
use crate::service::ExecutorServiceImpl;
use tokio_stream::wrappers::ReceiverStream;
use tonic::{Request, Response, Status, Streaming};

#[tonic::async_trait]
impl RobonixSystemExecutorExecuteStream for ExecutorServiceImpl {
    type ExecuteStreamStream = ReceiverStream<Result<RtdlEvent, Status>>;

    /// Require START before handing incremental input to the common execution loop.
    async fn execute_stream(
        &self,
        request: Request<Streaming<ExecuteStreamInput>>,
    ) -> Result<Response<Self::ExecuteStreamStream>, Status> {
        let mut stream = request.into_inner();
        let first = stream
            .message()
            .await?
            .ok_or_else(|| Status::invalid_argument("missing START"))?;
        let mut input = Input::default();
        input.accept(first).map_err(Status::invalid_argument)?;
        Ok(Response::new(
            Actor::start(self.clone(), input, stream).await?,
        ))
    }
}
