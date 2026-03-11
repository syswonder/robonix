// SPDX-License-Identifier: MulanPSL-2.0

use anyhow::{Context, Result};
use log::info;
use std::sync::Arc;
use std::time::Duration as StdDuration;

pub struct PingQueryRuntime {
    _node: Arc<crate::runtime::RuntimeNode>,
    _server: crate::runtime::QueryServerHandle<
        crate::generated::robonix_system_debug::ping_query::QueryService,
    >,
    _node_id: String,
}

impl PingQueryRuntime {
    pub async fn start(runtime_endpoint: &str) -> Result<Arc<Self>> {
        let node = crate::runtime::RuntimeNode::new("ping_query_runtime")
            .context("failed to create ping query runtime node")?;
        node.start_background_spin()
            .context("failed to start ping query runtime executor")?;

        let mut runtime_client = crate::runtime::RuntimeClient::connect_with_retry(
            runtime_endpoint,
            20,
            StdDuration::from_millis(250),
        )
        .await
        .context("failed to connect ping query runtime to registry")?;
        let node_id = runtime_client
            .register_node(
                "robonix-server",
                vec!["robonix/system/debug".to_string()],
                vec!["provide:robonix/system/debug/ping".to_string()],
            )
            .await
            .context("failed to register ping query runtime node")?;

        let server = crate::generated::robonix_system_debug::ping_query::create_registered_server(
            &node,
            &mut runtime_client,
            &node_id,
        )
        .await
        .context("failed to create ping query server")?;

        server.set_callback(
            |request: crate::generated::robonix_system_debug::ping_query::Request| {
                let payload =
                    crate::generated::robonix_system_debug::ping_query::request_string(&request);
                info!("ping query received: {}", payload);
                crate::generated::robonix_system_debug::ping_query::response_from_string(format!(
                    "pong:{payload}"
                ))
            },
        );

        let runtime = Arc::new(Self {
            _node: node,
            _server: server,
            _node_id: node_id.clone(),
        });

        info!("ping query runtime ready on node_id='{}'", node_id);
        Ok(runtime)
    }
}
