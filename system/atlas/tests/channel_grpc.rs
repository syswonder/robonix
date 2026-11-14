// SPDX-License-Identifier: MulanPSL-2.0
//! Cross-process coverage driven by robonix-api's shared provider fixture.
use anyhow::Result;
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb::Transport;
use serde_json::json;
use std::time::Duration;
use tokio_stream::StreamExt;

const ECHO: &str = "robonix/service/map/get_mode";
const NAV: &str = "robonix/service/navigation/navigate";

#[tokio::test]
#[ignore = "requires test_channel.py's live Atlas and gRPC fixtures"]
async fn grpc_channels() -> Result<()> {
    let atlas = AtlasClient::connect(std::env::var("ROBONIX_TEST_ATLAS")?).await?;
    atlas.add_descriptor_set(&std::fs::read(std::env::var("ROBONIX_TEST_DESCRIPTORS")?)?)?;
    let connect = |contract: &'static str| {
        let atlas = atlas.clone();
        async move {
            atlas
                .connect_channel("rust-channel-test", "grpc-test", contract, Transport::Grpc)
                .await
        }
    };
    let ch = connect(ECHO).await?;
    assert_eq!(ch.call(json!({})).await?["mode"], "mapping");
    assert!(ch.call(json!({})).await.is_err());
    ch.close().await?;
    assert!(atlas.get_channel(&ch.channel_id).is_none());
    let ch = connect(NAV).await?;
    assert_eq!(ch.call(json!({})).await?["accepted"], true);
    assert_eq!(ch.run_id().as_deref(), Some("grpc-run"));
    assert_eq!(ch.status().await?["state"], "RUNNING");
    assert_eq!(ch.cancel().await?["accepted"], true);
    assert_eq!(ch.status().await?["state"], "CANCELED");
    ch.close().await?;

    let ch = connect("robonix/service/speech/tts_stream").await?;
    let mut responses = ch.server_stream(json!({"text": "hello"})).await?;
    let mut count = 0;
    while let Some(response) = responses.next().await {
        let response = response?;
        assert_eq!(response["chunk"]["sequence"], count);
        assert_eq!(response["chunk"]["data"], "AQI=");
        count += 1;
    }
    assert_eq!(count, 3);
    ch.close().await?;
    let ch = connect("robonix/service/speech/wake_word").await?;
    let response = ch
        .client_stream(tokio_stream::iter([
            json!({"sequence": 4}),
            json!({"sequence": 7}),
        ]))
        .await?;
    assert_eq!(response["keyword"], "[4, 7]");
    ch.close().await?;
    let ch = connect("robonix/service/speech/asr_stream").await?;
    let mut responses = ch
        .bidirectional_stream(tokio_stream::iter([
            json!({"chunk": {"sequence": 4}}),
            json!({"chunk": {"sequence": 7}}),
        ]))
        .await?;
    assert_eq!(responses.next().await.unwrap()?["text"], "4");
    assert_eq!(responses.next().await.unwrap()?["text"], "7");
    assert!(responses.next().await.is_none());
    ch.close().await?;
    let ch = connect("robonix/primitive/audio/mic").await?;
    drop(ch.server_stream(json!({})).await?);
    ch.close().await?;
    let ch = connect("robonix/primitive/audio/speaker").await?;
    ch.client_stream(tokio_stream::iter([json!({"sequence": 1})]))
        .await?;
    ch.close().await?;

    let control = std::env::var("ROBONIX_TEST_CONTROL")?;
    std::fs::write(&control, r#"{"delay": 0.2}"#)?;
    let ch = connect(ECHO).await?;
    assert!(
        tokio::time::timeout(Duration::from_millis(5), ch.call(json!({})))
            .await
            .is_err()
    );
    assert!(atlas.clone().get_channel(&ch.channel_id).is_some());
    ch.close_with_timeout(Duration::from_secs(2)).await?;
    assert!(ch.is_closed());
    std::fs::write(&control, "{}")?;

    let ch = connect(ECHO).await?;
    assert!(ch.call(json!({"unknown_field": 1})).await.is_err());
    ch.close().await?;
    Ok(())
}
