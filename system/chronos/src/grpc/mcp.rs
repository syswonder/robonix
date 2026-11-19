// SPDX-License-Identifier: MulanPSL-2.0

use std::net::IpAddr;

use rmcp::{
    ErrorData, RoleServer, ServerHandler,
    model::{
        CallToolRequestParams, CallToolResult, Content, JsonObject, ListToolsResult,
        PaginatedRequestParams, ServerCapabilities, ServerInfo, Tool,
    },
    service::RequestContext,
    transport::streamable_http_server::{
        StreamableHttpServerConfig, StreamableHttpService, session::local::LocalSessionManager,
    },
};
use serde_json::json;
use tower_http::limit::RequestBodyLimit;

use super::service::ChronosService;

pub(super) const CONTRACT_ID: &str = "robonix/system/chronos/get_wall_time";
pub(super) const DESCRIPTION: &str =
    "Get the current date and time in the Chronos process's local time zone.";
pub(super) const PATH: &str = "/mcp";
const MAX_REQUEST_BYTES: usize = 64 * 1024;

fn tool_name() -> &'static str {
    CONTRACT_ID
        .rsplit_once('/')
        .expect("qualified contract ID")
        .1
}

pub(super) fn wall_time_tool() -> Tool {
    let schema = JsonObject::from_iter([
        ("type".into(), json!("object")),
        ("properties".into(), json!({})),
        ("additionalProperties".into(), json!(false)),
    ]);
    Tool::new(tool_name(), DESCRIPTION, schema)
}

pub(super) fn http_service(
    service: ChronosService,
    advertised: IpAddr,
) -> RequestBodyLimit<StreamableHttpService<ChronosService, LocalSessionManager>> {
    let host = match advertised {
        IpAddr::V4(ip) => ip.to_string(),
        IpAddr::V6(ip) => format!("[{ip}]"),
    };
    let config = StreamableHttpServerConfig::default()
        .with_allowed_hosts([host])
        .with_stateful_mode(false)
        .with_json_response(true);
    let mcp = StreamableHttpService::new(move || Ok(service.clone()), Default::default(), config);
    RequestBodyLimit::new(mcp, MAX_REQUEST_BYTES)
}

impl ServerHandler for ChronosService {
    fn get_info(&self) -> ServerInfo {
        ServerInfo::new(ServerCapabilities::builder().enable_tools().build())
    }

    async fn list_tools(
        &self,
        _request: Option<PaginatedRequestParams>,
        _context: RequestContext<RoleServer>,
    ) -> Result<ListToolsResult, ErrorData> {
        Ok(ListToolsResult::with_all_items(vec![wall_time_tool()]))
    }

    async fn call_tool(
        &self,
        request: CallToolRequestParams,
        _context: RequestContext<RoleServer>,
    ) -> Result<CallToolResult, ErrorData> {
        if request.name != tool_name() {
            return Err(ErrorData::invalid_params("Unknown Chronos tool", None));
        }
        if request.arguments.is_some_and(|args| !args.is_empty()) {
            return Err(ErrorData::invalid_params(
                "This tool takes no arguments",
                None,
            ));
        }
        Ok(match self.wall_datetime().await {
            Ok(datetime) => CallToolResult::structured(json!({ "datetime": datetime })),
            Err(error) => CallToolResult::error(vec![Content::text(format!("{error:#}"))]),
        })
    }
}
