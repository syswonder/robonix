// SPDX-License-Identifier: MulanPSL-2.0
// dispatch/grpc.rs — gRPC primitive / service dispatch
//
// TODO(executor owner): implement gRPC dispatch via Atlas endpoint resolution.
// `endpoint` here is an abstract_interface_id (e.g. "robonix/srv/navigation/navigate").
// Executor should resolve the concrete gRPC endpoint via Atlas QueryNodes and
// call the generated proto service.

use crate::pilot::TaskCallResult;

pub async fn execute(
    call_id: &str,
    name: &str,
    _args_json: &str,
    endpoint: &str,
) -> TaskCallResult {
    // Stub: gRPC dispatch not yet implemented.
    log::warn!(
        "[grpc] tool '{}' (interface '{}') — gRPC dispatch not yet implemented",
        name,
        endpoint
    );
    TaskCallResult {
        call_id: call_id.to_string(),
        tool_name: name.to_string(),
        success: false,
        output: String::new(),
        error: format!("gRPC dispatch not yet implemented for '{}'", name),
    }
}
