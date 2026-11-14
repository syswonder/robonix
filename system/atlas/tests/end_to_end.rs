// SPDX-License-Identifier: MulanPSL-2.0

use robonix_atlas::{
    client::AtlasClient,
    pb,
    service::{AtlasRegistry, serve_atlas},
};
use std::sync::Arc;
use std::time::Duration;

#[tokio::test]
async fn timing_metadata_round_trips_over_grpc() {
    let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let address = listener.local_addr().unwrap();
    drop(listener);
    let server = tokio::spawn(serve_atlas(Arc::new(AtlasRegistry::default()), address));
    let mut atlas =
        AtlasClient::connect_with_retry(address.to_string(), 50, Duration::from_millis(10))
            .await
            .unwrap();
    let registration = pb::RegisterRequest {
        id: "camera".into(),
        namespace: "robonix/primitive/camera".into(),
        ..Default::default()
    };
    // Known zero delay must remain distinguishable from an omitted delay.
    for delay in [None, Some(0)] {
        atlas
            .inner()
            .register_primitive(registration.clone())
            .await
            .unwrap();
        let timing = pb::CapabilityTimeDescriptor {
            domain_id: "camera_clock".into(),
            timestamp_semantics: "exposure_midpoint".into(),
            pipeline_delay_ns: delay,
            pipeline_uncertainty_ns: 200,
            is_hardware_timestamped: true,
            supports_per_point_timestamp: false,
        };
        atlas
            .declare_capability_with_description_and_time(
                "camera",
                "robonix/primitive/camera/rgb",
                pb::Transport::Ros2,
                "/camera/rgb",
                pb::TransportParams {
                    kind: Some(pb::transport_params::Kind::Ros2(pb::Ros2Params::default())),
                },
                "",
                Some(timing.clone()),
            )
            .await
            .unwrap();
        let providers = atlas
            .query_primitives(
                "camera",
                "robonix/primitive/camera/rgb",
                "",
                pb::Transport::Ros2,
            )
            .await
            .unwrap();
        assert_eq!(
            providers[0].capabilities[0].time_descriptor,
            Some(timing.clone())
        );
        let (channel, endpoint, _, connected) = atlas
            .connect_capability_with_time(
                "consumer",
                "camera",
                "robonix/primitive/camera/rgb",
                pb::Transport::Ros2,
            )
            .await
            .unwrap();
        assert_eq!(endpoint, "/camera/rgb");
        assert_eq!(connected, Some(timing));
        atlas.disconnect_capability(&channel).await.unwrap();
    }
    atlas
        .inner()
        .register_primitive(registration)
        .await
        .unwrap();
    atlas
        .declare_capability_with_description(
            "camera",
            "robonix/primitive/camera/rgb",
            pb::Transport::Ros2,
            "/camera/rgb",
            pb::TransportParams {
                kind: Some(pb::transport_params::Kind::Ros2(pb::Ros2Params::default())),
            },
            "",
        )
        .await
        .unwrap();
    let (_, _, _, connected) = atlas
        .connect_capability_with_time(
            "consumer",
            "camera",
            "robonix/primitive/camera/rgb",
            pb::Transport::Ros2,
        )
        .await
        .unwrap();
    assert_eq!(connected, None);
    server.abort();
    let _ = server.await;
}
