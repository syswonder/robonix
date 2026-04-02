# robonix-sdk

Thin Rust gRPC client for robonix-atlas, wrapping the `RobonixRuntime` service defined in [`proto/robonix_runtime.proto`](../../proto/robonix_runtime.proto).

## Usage

Connect with a tonic endpoint string (`http://host:port`), then call the async helpers:

```rust
let mut client = robonix_sdk::RobonixClient::connect("http://127.0.0.1:50051").await?;

client
    .register_node("com.example.demo", "robonix/demo", "skill", include_str!("SKILL.md"))
    .await?;

client
    .declare_interface("com.example.demo", "out", vec!["ros2".into()], "{}")
    .await?;

let ch = client
    .negotiate_channel("com.example.consumer", "com.example.demo", "out", "ros2")
    .await?;
// ch.channel_id, ch.transport, ch.endpoint
```

See `RobonixClient` in `src/lib.rs` for `query_nodes`, `query_all_skills`, `release_channel`, etc.

## Schema

All request/response types and RPC names are defined in `proto/robonix_runtime.proto`.
