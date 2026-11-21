// SPDX-License-Identifier: MulanPSL-2.0
//! Cross-process coverage driven by robonix-api's shared provider fixture.

#[allow(
    dead_code,
    unused_imports,
    unused_variables,
    clippy::all,
    rustdoc::broken_intra_doc_links,
    rustdoc::invalid_html_tags
)]
mod pb {
    include!(concat!(env!("OUT_DIR"), "/contract_proto_modules.rs"));
}

use anyhow::Result;
use pb::asr::{AsrAudioChunk, ServiceSpeechAsrStream};
use pb::audio::{AudioChunk, PrimitiveAudioMic, PrimitiveAudioSpeaker};
use pb::map::ServiceMapGetMode;
use pb::navigation::ServiceNavigationNavigate;
use pb::speech::ServiceSpeechWakeWord;
use pb::tts::ServiceSpeechTtsStream;
use robonix_atlas::client::AtlasClient;
use robonix_atlas::client::channel::{Channel, ChannelContract, validate_contract};
use robonix_atlas::pb::Transport;
use std::time::Duration;
use tokio_stream::StreamExt;

async fn connect<C: ChannelContract>(atlas: &AtlasClient) -> Result<Channel<C>> {
    atlas
        .connect_channel::<C>("rust-channel-test", "grpc-test", C::ID, Transport::Grpc)
        .await
}

#[tokio::test]
async fn grpc_channels() -> Result<()> {
    let atlas = AtlasClient::connect(std::env::var("ROBONIX_TEST_ATLAS")?).await?;
    let control = std::env::var("ROBONIX_TEST_CONTROL")?;
    let configure = |value: &str| std::fs::write(&control, value);
    let ch = connect::<ServiceMapGetMode>(&atlas).await?;
    let mut descriptor = atlas
        .clone()
        .query_contract(ServiceMapGetMode::ID)
        .await?
        .unwrap();
    validate_contract::<ServiceMapGetMode>(&descriptor)?;
    descriptor.mode = "rpc_server_stream".to_owned();
    assert!(validate_contract::<ServiceMapGetMode>(&descriptor).is_err());
    descriptor.mode = ServiceMapGetMode::MODE.to_owned();
    descriptor.io_srv_type = "navigation/srv/Navigate".to_owned();
    assert!(validate_contract::<ServiceMapGetMode>(&descriptor).is_err());
    descriptor.io_srv_type = ServiceMapGetMode::IDL.to_owned();
    descriptor.id = "robonix/service/map/another_get_mode".to_owned();
    assert!(validate_contract::<ServiceMapGetMode>(&descriptor).is_err());
    descriptor.id = "robonix/skill/channel_test/driver".to_owned();
    descriptor.io_srv_type = pb::lifecycle::LifecycleDriver::IDL.to_owned();
    validate_contract::<pb::lifecycle::LifecycleDriver>(&descriptor)?;
    descriptor.io_srv_type = ServiceMapGetMode::IDL.to_owned();
    assert!(validate_contract::<pb::lifecycle::LifecycleDriver>(&descriptor).is_err());
    descriptor.io_srv_type = pb::lifecycle::LifecycleDriver::IDL.to_owned();
    descriptor.id = pb::lifecycle::LifecycleDriver::ID.to_owned();
    validate_contract::<pb::lifecycle::LifecycleDriver>(&descriptor)?;
    assert!(
        atlas
            .get_channel::<ServiceNavigationNavigate>(&ch.channel_id)
            .is_err()
    );
    assert_eq!(ch.call(pb::map::GetModeRequest {}).await?.mode, "mapping");
    assert!(ch.call(pb::map::GetModeRequest {}).await.is_err());
    assert!(ch.status().await.is_err());
    ch.close().await?;
    assert!(
        atlas
            .get_channel::<ServiceMapGetMode>(&ch.channel_id)?
            .is_none()
    );
    assert!(
        atlas
            .connect_channel::<ServiceMapGetMode>(
                "rust-channel-test",
                "grpc-test",
                ServiceNavigationNavigate::ID,
                Transport::Grpc,
            )
            .await
            .is_err()
    );
    assert!(
        atlas
            .connect_channel::<ServiceMapGetMode>(
                "rust-channel-test",
                "grpc-test",
                ServiceMapGetMode::ID,
                Transport::Mcp,
            )
            .await
            .is_err()
    );

    let ch = connect::<ServiceNavigationNavigate>(&atlas).await?;
    assert!(
        ch.call(pb::navigation::NavigateRequest::default())
            .await?
            .accepted
    );
    assert_eq!(ch.run_id().as_deref(), Some("grpc-run"));
    assert_eq!(ch.status().await?.state, "RUNNING");
    assert!(ch.cancel().await?.accepted);
    assert_eq!(ch.status().await?.state, "CANCELED");
    ch.close().await?;

    configure(r#"{"reject": true}"#)?;
    let ch = connect::<ServiceNavigationNavigate>(&atlas).await?;
    assert!(!ch.call(Default::default()).await?.accepted);
    assert!(ch.is_terminal());
    ch.close().await?;
    configure(r#"{"run_id": ""}"#)?;
    let ch = connect::<ServiceNavigationNavigate>(&atlas).await?;
    ch.call(Default::default()).await?;
    assert_eq!(ch.run_id().as_deref(), Some(""));
    assert!(ch.cancel().await?.accepted);
    assert_eq!(ch.status().await?.state, "CANCELED");
    ch.close().await?;

    configure(r#"{"ignore_cancel": true}"#)?;
    let ch = connect::<ServiceNavigationNavigate>(&atlas).await?;
    ch.call(Default::default()).await?;
    assert!(
        ch.close_with_timeout(Duration::from_millis(100))
            .await
            .is_err()
    );
    assert!(!ch.is_closed());
    let ch = atlas
        .clone()
        .get_channel::<ServiceNavigationNavigate>(&ch.channel_id)?
        .unwrap();
    configure(r#"{"status_state": "INVALID"}"#)?;
    assert!(ch.status().await.is_err());
    assert!(!ch.is_terminal());
    configure(r#"{"status_error": true}"#)?;
    assert!(ch.status().await.is_err());
    assert!(!ch.is_terminal());
    configure(r#"{"finish": true}"#)?;
    assert_eq!(ch.status().await?.state, "SUCCEEDED");
    ch.close().await?;
    configure("{}")?;

    let ch = connect::<ServiceSpeechTtsStream>(&atlas).await?;
    let mut responses = ch
        .call(pb::tts::SynthesizeStreamRequest {
            text: "hello".to_owned(),
            ..Default::default()
        })
        .await?;
    let mut count = 0;
    while let Some(response) = responses.next().await {
        let chunk = response?.chunk.unwrap();
        assert_eq!(chunk.sequence, count);
        assert_eq!(chunk.data, [1, 2]);
        count += 1;
    }
    assert_eq!(count, 3);
    ch.close().await?;

    let chunks = || {
        tokio_stream::iter([4, 7].map(|sequence| AudioChunk {
            sequence,
            ..Default::default()
        }))
    };
    let ch = connect::<ServiceSpeechWakeWord>(&atlas).await?;
    let response = ch.call(Box::pin(chunks())).await?;
    assert_eq!(response.keyword, "[4, 7]");
    ch.close().await?;
    let ch = connect::<ServiceSpeechAsrStream>(&atlas).await?;
    let mut responses = ch
        .call(Box::pin(
            chunks().map(|chunk| AsrAudioChunk { chunk: Some(chunk) }),
        ))
        .await?;
    assert_eq!(responses.next().await.unwrap()?.text, "4");
    assert_eq!(responses.next().await.unwrap()?.text, "7");
    assert!(responses.next().await.is_none());
    ch.close().await?;

    let ch = connect::<PrimitiveAudioMic>(&atlas).await?;
    drop(ch.call(()).await?);
    ch.close().await?;
    let ch = connect::<PrimitiveAudioSpeaker>(&atlas).await?;
    ch.call(Box::pin(chunks())).await?;
    ch.close().await?;

    configure(r#"{"stream_count": 40}"#)?;
    let ch = connect::<PrimitiveAudioMic>(&atlas).await?;
    let responses = ch.call(()).await?;
    ch.close().await?;
    drop(responses);
    assert!(ch.is_closed());
    configure(r#"{"delay": 0.2}"#)?;
    let ch = connect::<ServiceMapGetMode>(&atlas).await?;
    assert!(
        tokio::time::timeout(Duration::from_millis(5), ch.call(Default::default()))
            .await
            .is_err()
    );
    assert!(
        atlas
            .clone()
            .get_channel::<ServiceMapGetMode>(&ch.channel_id)?
            .is_some()
    );
    ch.close_with_timeout(Duration::from_secs(2)).await?;
    assert!(ch.is_closed());
    configure("{}")?;

    let listener = std::net::TcpListener::bind("127.0.0.1:0")?;
    let endpoint = listener.local_addr()?.to_string();
    let mut registration = atlas.clone();
    registration
        .register_service("grpc-bad-binding", "robonix/service/map", "")
        .await?;
    registration
        .declare_capability(
            "grpc-bad-binding",
            ServiceMapGetMode::ID,
            Transport::Grpc,
            &endpoint,
            robonix_atlas::client::grpc_params(
                "robonix_contracts.proto",
                "wrong.Service",
                "GetMode",
            ),
        )
        .await?;
    let error = atlas
        .connect_channel::<ServiceMapGetMode>(
            "rust-channel-test",
            "grpc-bad-binding",
            ServiceMapGetMode::ID,
            Transport::Grpc,
        )
        .await
        .err()
        .unwrap();
    assert!(format!("{error:#}").contains("gRPC service does not match channel binding"));
    registration.unregister("grpc-bad-binding").await?;
    Ok(())
}
