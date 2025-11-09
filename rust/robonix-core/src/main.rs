use ansi_term::Colour;
use futures_util::stream::StreamExt;
use robonix_core::core::RobonixCore;
use robonix_core::messages::{QueryRequest, QueryResponse, RegisterRequest, RegisterResponse};
use ros2_client::{
    AService, Context, Name, Node, NodeName, NodeOptions, ServiceMapping, ServiceTypeName,
    rustdds::{
        Duration, QosPolicies, QosPolicyBuilder,
        policy::{self, Deadline, Lifespan},
    },
};
use std::fmt;
use std::sync::Arc;
use std::time::Instant;
#[allow(unused_imports)]
use tracing::{Level, debug, error, info, warn};
use tracing_subscriber::fmt::FmtContext;
use tracing_subscriber::fmt::format::{FormatEvent, FormatFields, Writer};
use tracing_subscriber::registry::LookupSpan;

fn main() {
    // Initialize tracing with Linux-style format
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| {
                // Default: only show info+ for robonix_core, error+ for rustdds
                tracing_subscriber::EnvFilter::new("robonix_core=info,rustdds=error")
            }),
        )
        .with_target(false)
        .event_format(LinuxStyleFormatter)
        .init();

    info!("Robonix Core starting...");

    let mut node = create_node();
    let service_qos = create_qos();
    info!("Robonix Core node started");

    let core = Arc::new(RobonixCore::new());
    // Create registration service at /rbnx/srv/register
    let register_server = node
        .create_server::<AService<RegisterRequest, RegisterResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/srv", "register").unwrap(),
            &ServiceTypeName::new("robonix_core", "Register"),
            service_qos.clone(),
            service_qos.clone(),
        )
        .unwrap();

    info!("Registration service created at /rbnx/srv/register");

    // Create query service at /rbnx/srv/query
    let query_server = node
        .create_server::<AService<QueryRequest, QueryResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/srv", "query").unwrap(),
            &ServiceTypeName::new("robonix_core", "Query"),
            service_qos.clone(),
            service_qos.clone(),
        )
        .unwrap();

    info!("Query service created at /rbnx/srv/query");
    info!("Robonix Core ready. Waiting for requests...");

    // run it!
    smol::block_on(async {
        let core_clone1 = core.clone();
        let core_clone2 = core.clone();

        // Handle registration requests
        let register_task = smol::spawn(async move {
            let register_stream = register_server.receive_request_stream();
            register_stream
                .for_each(|result| async {
                    match result {
                        Ok((req_id, req)) => {
                            info!(
                                package = %req.package_name,
                                package_type = %req.package_type,
                                std_name = %req.std_name,
                                "Received registration request"
                            );
                            let resp = core_clone1.register(req).await;
                            let sr = register_server.async_send_response(req_id, resp).await;
                            if let Err(e) = sr {
                                error!("Send response error: {e:?}");
                            }
                        }
                        Err(e) => error!("Receive request error: {e:?}"),
                    }
                })
                .await;
        });

        // Handle query requests
        let query_task = smol::spawn(async move {
            let query_stream = query_server.receive_request_stream();
            query_stream
                .for_each(|result| async {
                    match result {
                        Ok((req_id, req)) => {
                            debug!(
                                std_name = %req.std_name,
                                "Received query request"
                            );
                            let resp = core_clone2.query(req).await;
                            let sr = query_server.async_send_response(req_id, resp).await;
                            if let Err(e) = sr {
                                error!("Send query response error: {e:?}");
                            }
                        }
                        Err(e) => error!("Receive query request error: {e:?}"),
                    }
                })
                .await;
        });

        // Wait for both tasks (both run indefinitely)
        futures_util::future::select(register_task, query_task).await;
    });
    // .count() here just converts Stream to ordinary Future.
    // It would return the count of requestes processed, if the stream would end.
} // main

fn create_qos() -> QosPolicies {
    let service_qos: QosPolicies = {
        QosPolicyBuilder::new()
            .history(policy::History::KeepLast { depth: 10 })
            .reliability(policy::Reliability::Reliable {
                max_blocking_time: Duration::from_millis(100),
            })
            .durability(policy::Durability::Volatile)
            .deadline(Deadline(Duration::INFINITE))
            .lifespan(Lifespan {
                duration: Duration::INFINITE,
            })
            .liveliness(policy::Liveliness::Automatic {
                lease_duration: Duration::INFINITE,
            })
            .build()
    };
    service_qos
}

fn create_node() -> Node {
    let context = Context::new().unwrap();
    context
        .new_node(
            NodeName::new("/rbnx", "core").unwrap(),
            NodeOptions::new().enable_rosout(true),
        )
        .unwrap()
}

// Linux-style log formatter
struct LinuxStyleFormatter;

impl<S, N> FormatEvent<S, N> for LinuxStyleFormatter
where
    S: tracing::Subscriber + for<'a> LookupSpan<'a>,
    N: for<'a> FormatFields<'a> + 'static,
{
    fn format_event(
        &self,
        ctx: &FmtContext<'_, S, N>,
        mut writer: Writer<'_>,
        event: &tracing::Event<'_>,
    ) -> fmt::Result {
        // Get timestamp in Linux kernel style [seconds.microseconds]
        // Use a static Instant for the start time - we'll initialize it on first use
        static START: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
        let start = START.get_or_init(Instant::now);
        let elapsed = start.elapsed();
        let secs = elapsed.as_secs();
        let micros = elapsed.subsec_micros();
        let timestamp = format!("[{}.{:05}]", secs, micros);

        // Get log level and format it as a single colored letter
        let level = *event.metadata().level();
        let (level_char, level_color) = match level {
            Level::ERROR => ('E', Colour::Red),
            Level::WARN => ('W', Colour::Yellow),
            Level::INFO => ('I', Colour::Green),
            Level::DEBUG => ('D', Colour::Cyan),
            Level::TRACE => ('T', Colour::White),
        };

        // Get process name
        let proc_name = std::env::current_exe()
            .ok()
            .and_then(|p| p.file_name().map(|n| n.to_string_lossy().into_owned()))
            .unwrap_or_else(|| "robonix-core".to_string());

        // Write Linux-style log entry: timestamp procname[pid]: LEVEL message
        write!(
            writer,
            "{} {}[{}]: {} ",
            timestamp,
            proc_name,
            std::process::id(),
            level_color.paint(level_char.to_string())
        )?;

        // Format the message and fields
        ctx.format_fields(writer.by_ref(), event)?;
        writeln!(writer)
    }
}
