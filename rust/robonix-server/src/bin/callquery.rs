// SPDX-License-Identifier: MulanPSL-2.0

use anyhow::{Context, Result, bail};
use robonix_server::runtime::{RuntimeClient, RuntimeNode, call_query, wait_for_service};
use std::time::Duration as StdDuration;

struct Args {
    endpoint: String,
    requester_id: String,
    target: String,
    interface_id: String,
    request_json: String,
}

fn parse_args() -> Result<Args> {
    let mut endpoint = std::env::var("ROBONIX_META_GRPC_ENDPOINT")
        .unwrap_or_else(|_| "127.0.0.1:50051".to_string());
    let mut requester_id = std::env::var("ROBONIX_QUERY_REQUESTER_ID")
        .unwrap_or_else(|_| "example_callquery_client".to_string());
    let mut positionals = Vec::new();

    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--endpoint" => {
                endpoint = args
                    .next()
                    .context("missing value for --endpoint")?;
            }
            "--requester-id" => {
                requester_id = args
                    .next()
                    .context("missing value for --requester-id")?;
            }
            "-h" | "--help" => {
                print_usage();
                std::process::exit(0);
            }
            _ => positionals.push(arg),
        }
    }

    if positionals.len() < 2 || positionals.len() > 3 {
        print_usage();
        bail!("expected: callquery [--endpoint ADDR] [--requester-id ID] <target> <interface_id> [request_json]");
    }

    Ok(Args {
        endpoint,
        requester_id,
        target: positionals[0].clone(),
        interface_id: normalize_interface_id(&positionals[1]),
        request_json: positionals
            .get(2)
            .cloned()
            .unwrap_or_else(|| "".to_string()),
    })
}

fn print_usage() {
    eprintln!("usage: callquery [--endpoint ADDR] [--requester-id ID] <target> <interface_id> [request_json]");
}

fn normalize_interface_id(interface_id: &str) -> String {
    interface_id
        .trim()
        .trim_end_matches(".ridl")
        .trim_matches('/')
        .to_string()
}

fn pretty_string_response(value: &str) -> String {
    match serde_json::from_str::<serde_json::Value>(value) {
        Ok(json) => serde_json::to_string_pretty(&json).unwrap_or_else(|_| value.to_string()),
        Err(_) => value.to_string(),
    }
}

macro_rules! call_string_query {
    ($create_client:path, $request_from_string:path, $response_into_string:path, $node:expr, $runtime_client:expr, $requester_id:expr, $target:expr, $request_json:expr) => {{
        let client = $create_client($node, $runtime_client, $requester_id, $target)
            .await
            .with_context(|| format!("failed to resolve {}", stringify!($create_client)))?;
        wait_for_service(&client, StdDuration::from_secs(10)).await?;
        let request = $request_from_string($request_json.to_string());
        let response = call_query(&client, request, StdDuration::from_secs(10)).await?;
        Ok::<String, anyhow::Error>(pretty_string_response(&$response_into_string(response)))
    }};
}

async fn call_supported_query(
    interface_id: &str,
    node: &RuntimeNode,
    runtime_client: &mut RuntimeClient,
    requester_id: &str,
    target: &str,
    request_json: &str,
) -> Result<String> {
    match interface_id {
        "robonix/hal/base/status" => call_string_query!(
            robonix_server::generated::robonix_hal_base::status_query::create_resolved_client,
            robonix_server::generated::robonix_hal_base::status_query::request_from_string,
            robonix_server::generated::robonix_hal_base::status_query::response_into_string,
            node,
            runtime_client,
            requester_id,
            target,
            request_json
        ),
        "robonix/system/debug/ping" => call_string_query!(
            robonix_server::generated::robonix_system_debug::ping_query::create_resolved_client,
            robonix_server::generated::robonix_system_debug::ping_query::request_from_string,
            robonix_server::generated::robonix_system_debug::ping_query::response_into_string,
            node,
            runtime_client,
            requester_id,
            target,
            request_json
        ),
        // semantic_query, map_manager, model_manager, skill_library use robonix_msg types (Object[], etc.); use Python client
        "robonix/system/manager/task_manager" => call_string_query!(
            robonix_server::generated::robonix_system_manager::task_manager_query::create_resolved_client,
            robonix_server::generated::robonix_system_manager::task_manager_query::request_from_string,
            robonix_server::generated::robonix_system_manager::task_manager_query::response_into_string,
            node,
            runtime_client,
            requester_id,
            target,
            request_json
        ),
        "robonix/system/model/embeddings" => call_string_query!(
            robonix_server::generated::robonix_system_model::embeddings_query::create_resolved_client,
            robonix_server::generated::robonix_system_model::embeddings_query::request_from_string,
            robonix_server::generated::robonix_system_model::embeddings_query::response_into_string,
            node,
            runtime_client,
            requester_id,
            target,
            request_json
        ),
        "robonix/system/model/chat_completions" => call_string_query!(
            robonix_server::generated::robonix_system_model::chat_completions_query::create_resolved_client,
            robonix_server::generated::robonix_system_model::chat_completions_query::request_from_string,
            robonix_server::generated::robonix_system_model::chat_completions_query::response_into_string,
            node,
            runtime_client,
            requester_id,
            target,
            request_json
        ),
        "robonix/system/model/vision_completions" => call_string_query!(
            robonix_server::generated::robonix_system_model::vision_completions_query::create_resolved_client,
            robonix_server::generated::robonix_system_model::vision_completions_query::request_from_string,
            robonix_server::generated::robonix_system_model::vision_completions_query::response_into_string,
            node,
            runtime_client,
            requester_id,
            target,
            request_json
        ),
        "robonix/system/planning/task_plan" => call_string_query!(
            robonix_server::generated::robonix_system_planning::task_plan_query::create_resolved_client,
            robonix_server::generated::robonix_system_planning::task_plan_query::request_from_string,
            robonix_server::generated::robonix_system_planning::task_plan_query::response_into_string,
            node,
            runtime_client,
            requester_id,
            target,
            request_json
        ),
        _ => bail!(
            "unsupported query interface '{}' (semantic_query, map_manager, model_manager, skill_library use robonix_msg types; use Python client)",
            interface_id
        ),
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let rmw = std::env::var("RMW_IMPLEMENTATION").unwrap_or_else(|_| "(default)".to_string());
    eprintln!("RMW_IMPLEMENTATION={}", rmw);

    let args = parse_args()?;

    let node = RuntimeNode::new("callquery").context("failed to create ROS2 callquery node")?;
    node.start_background_spin()
        .context("failed to start ROS2 callquery executor")?;

    let mut runtime_client =
        RuntimeClient::connect_with_retry(&args.endpoint, 20, StdDuration::from_millis(250))
            .await
            .with_context(|| format!("failed to connect runtime endpoint '{}'", args.endpoint))?;

    let result = call_supported_query(
        &args.interface_id,
        &node,
        &mut runtime_client,
        &args.requester_id,
        &args.target,
        &args.request_json,
    )
    .await?;

    println!("{result}");
    Ok(())
}
