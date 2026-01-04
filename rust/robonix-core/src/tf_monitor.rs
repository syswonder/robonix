// SPDX-License-Identifier: MulanPSL-2.0
// TF Monitor Module
//
// Monitors and parses ROS2 TF tree data

use futures_util::stream::StreamExt;
use log::debug;
use ros2_client::{
    Message, MessageTypeName, Name, Node, Subscription,
    rustdds::{
        Duration, QosPolicyBuilder,
        policy::{self, Reliability},
    },
};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::Mutex;

// TF2 message types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Header {
    pub stamp: Stamp,
    pub frame_id: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Stamp {
    pub sec: i32,
    pub nanosec: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Transform {
    pub translation: Vector3,
    pub rotation: Quaternion,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransformStamped {
    pub header: Header,
    pub child_frame_id: String,
    pub transform: Transform,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TFMessage {
    pub transforms: Vec<TransformStamped>,
}

impl Message for TFMessage {}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct TfFrame {
    pub frame_id: String,
    pub child_frames: Vec<String>,
    pub parent_frame: Option<String>,
    pub transform: Option<TfTransform>,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct TfTransform {
    pub translation: [f64; 3],
    pub rotation: [f64; 4], // quaternion [x, y, z, w]
}

#[derive(Serialize, Deserialize)]
pub struct TfTreeResponse {
    pub frames: Vec<TfFrame>,
    pub root_frames: Vec<String>,
}

pub struct TfTreeCache {
    pub frames: HashMap<String, TfFrame>,
    pub frame_last_seen: HashMap<String, std::time::Instant>,
    pub static_frames: std::collections::HashSet<String>,
    pub last_update: std::time::Instant,
}

impl Default for TfTreeCache {
    fn default() -> Self {
        Self {
            frames: HashMap::new(),
            frame_last_seen: HashMap::new(),
            static_frames: std::collections::HashSet::new(),
            last_update: std::time::Instant::now(),
        }
    }
}

pub struct TfMonitor {
    cache: Arc<Mutex<TfTreeCache>>,
}

impl TfMonitor {
    pub fn new() -> Self {
        Self {
            cache: Arc::new(Mutex::new(TfTreeCache::default())),
        }
    }

    pub fn get_cache(&self) -> Arc<Mutex<TfTreeCache>> {
        self.cache.clone()
    }

    pub async fn start_monitoring(
        &self,
        node: &mut Node,
    ) -> Result<(), Box<dyn std::error::Error>> {
        // Create QoS for /tf_static (static transforms - use TransientLocal)
        let tf_static_qos = QosPolicyBuilder::new()
            .history(policy::History::KeepLast { depth: 100 })
            .reliability(Reliability::Reliable {
                max_blocking_time: Duration::from_millis(100),
            })
            .durability(policy::Durability::TransientLocal)
            .build();

        // Create QoS for /tf (dynamic transforms - use Volatile to match publishers)
        let tf_qos = QosPolicyBuilder::new()
            .history(policy::History::KeepLast { depth: 100 })
            .reliability(Reliability::Reliable {
                max_blocking_time: Duration::from_millis(100),
            })
            .durability(policy::Durability::Volatile)
            .build();

        // Create /tf_static topic (static transforms)
        let tf_static_topic = node.create_topic(
            &Name::new("/", "tf_static")?,
            MessageTypeName::new("tf2_msgs", "TFMessage"),
            &tf_static_qos,
        )?;

        // Subscribe to /tf_static
        let tf_static_subscription: Subscription<TFMessage> =
            node.create_subscription(&tf_static_topic, None)?;

        // Create /tf topic (dynamic transforms)
        let tf_topic = node.create_topic(
            &Name::new("/", "tf")?,
            MessageTypeName::new("tf2_msgs", "TFMessage"),
            &tf_qos,
        )?;

        // Subscribe to /tf
        let tf_subscription: Subscription<TFMessage> = node.create_subscription(&tf_topic, None)?;

        let cache = self.cache.clone();

        // Spawn task to handle /tf_static messages
        let cache_static = cache.clone();
        tokio::spawn(async move {
            debug!("Subscribed to /tf_static topic");
            let mut stream = Box::pin(tf_static_subscription.async_stream());
            let mut message_count = 0;
            while let Some(result) = stream.next().await {
                match result {
                    Ok((msg, _msg_info)) => {
                        message_count += 1;
                        debug!(
                            "Received /tf_static message #{} with {} transforms",
                            message_count,
                            msg.transforms.len()
                        );
                        Self::process_tf_message(&msg, &cache_static, true).await;
                    }
                    Err(e) => {
                        debug!("Error receiving /tf_static message: {:?}", e);
                    }
                }
            }
        });

        // Spawn task to handle /tf messages
        let cache_dynamic = cache.clone();
        tokio::spawn(async move {
            debug!("Subscribed to /tf topic");
            let mut stream = Box::pin(tf_subscription.async_stream());
            let mut message_count = 0;
            while let Some(result) = stream.next().await {
                match result {
                    Ok((msg, _msg_info)) => {
                        message_count += 1;
                        // Only log first few messages, then every 1000 messages to avoid spam
                        if message_count <= 5 || message_count % 1000 == 0 {
                            let transform_names: Vec<String> = msg
                                .transforms
                                .iter()
                                .map(|t| format!("{} -> {}", t.header.frame_id, t.child_frame_id))
                                .collect();
                            debug!(
                                "Received /tf message #{} with {} transforms: {:?}",
                                message_count,
                                msg.transforms.len(),
                                transform_names
                            );
                        }
                        Self::process_tf_message(&msg, &cache_dynamic, false).await;
                    }
                    Err(e) => {
                        debug!("Error receiving /tf message: {:?}", e);
                    }
                }
            }
        });

        // Spawn task to periodically log TF tree statistics and clean up stale frames
        let cache_stats = cache.clone();
        tokio::spawn(async move {
            // Wait a bit before first check to allow some transforms to be collected
            tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;
            let mut interval = tokio::time::interval(tokio::time::Duration::from_secs(30));
            loop {
                interval.tick().await;
                let mut cache_guard = cache_stats.lock().await;
                let now = std::time::Instant::now();

                // Clean up frames that haven't been seen for a while
                // Static frames: use longer threshold (5 minutes) or don't clean at all
                // Dynamic frames: use shorter threshold (30 seconds)
                let dynamic_stale_threshold = std::time::Duration::from_secs(30);
                let static_stale_threshold = std::time::Duration::from_secs(300); // 5 minutes

                let frames_to_remove: Vec<String> = cache_guard
                    .frame_last_seen
                    .iter()
                    .filter(|(frame_id, last_seen)| {
                        let age = now.duration_since(**last_seen);
                        let is_static = cache_guard.static_frames.contains(*frame_id);
                        if is_static {
                            // For static frames, use longer threshold
                            age > static_stale_threshold
                        } else {
                            // For dynamic frames, use shorter threshold
                            age > dynamic_stale_threshold
                        }
                    })
                    .map(|(frame_id, _)| frame_id.clone())
                    .collect();

                for frame_id in &frames_to_remove {
                    debug!("Removing stale TF frame: {}", frame_id);

                    // Remove the frame itself
                    cache_guard.frames.remove(frame_id);
                    cache_guard.frame_last_seen.remove(frame_id);
                    cache_guard.static_frames.remove(frame_id);

                    // Remove references to this frame from other frames' child_frames lists
                    for frame in cache_guard.frames.values_mut() {
                        frame.child_frames.retain(|child| child != frame_id);
                    }

                    // Remove this frame as parent from other frames
                    for frame in cache_guard.frames.values_mut() {
                        if frame.parent_frame.as_ref() == Some(frame_id) {
                            frame.parent_frame = None;
                        }
                    }
                }

                let frame_count = cache_guard.frames.len();
                let frame_names: Vec<String> = cache_guard.frames.keys().cloned().collect();
                let mut sorted_frames = frame_names.clone();
                sorted_frames.sort();
                debug!(
                    "TF tree statistics: {} frames total. Frames: {:?}",
                    frame_count, sorted_frames
                );
            }
        });

        Ok(())
    }

    async fn process_tf_message(msg: &TFMessage, cache: &Arc<Mutex<TfTreeCache>>, is_static: bool) {
        let mut cache_guard = cache.lock().await;

        let total_frames_before = cache_guard.frames.len();
        let mut new_frames = Vec::new();

        for transform_stamped in &msg.transforms {
            let parent_frame = transform_stamped.header.frame_id.clone();
            let child_frame = transform_stamped.child_frame_id.clone();

            // Only log new frames to avoid spam
            let is_new_frame = !cache_guard.frames.contains_key(&child_frame);
            if is_new_frame {
                new_frames.push(format!("{} -> {}", parent_frame, child_frame));
            }

            // Extract transform data
            let tf_transform = TfTransform {
                translation: [
                    transform_stamped.transform.translation.x,
                    transform_stamped.transform.translation.y,
                    transform_stamped.transform.translation.z,
                ],
                rotation: [
                    transform_stamped.transform.rotation.x,
                    transform_stamped.transform.rotation.y,
                    transform_stamped.transform.rotation.z,
                    transform_stamped.transform.rotation.w,
                ],
            };

            // Update last seen time for both frames
            let now = std::time::Instant::now();
            cache_guard.frame_last_seen.insert(child_frame.clone(), now);
            cache_guard
                .frame_last_seen
                .insert(parent_frame.clone(), now);

            // Mark frames as static if they come from /tf_static
            if is_static {
                cache_guard.static_frames.insert(child_frame.clone());
                cache_guard.static_frames.insert(parent_frame.clone());
            }

            // Update or create child frame
            let child_frame_entry = cache_guard
                .frames
                .entry(child_frame.clone())
                .or_insert_with(|| TfFrame {
                    frame_id: child_frame.clone(),
                    child_frames: vec![],
                    parent_frame: Some(parent_frame.clone()),
                    transform: None,
                });
            child_frame_entry.parent_frame = Some(parent_frame.clone());
            child_frame_entry.transform = Some(tf_transform);

            // Update or create parent frame
            let parent_frame_entry = cache_guard
                .frames
                .entry(parent_frame.clone())
                .or_insert_with(|| TfFrame {
                    frame_id: parent_frame.clone(),
                    child_frames: vec![],
                    parent_frame: None,
                    transform: None,
                });
            if !parent_frame_entry.child_frames.contains(&child_frame) {
                parent_frame_entry.child_frames.push(child_frame);
            }
        }

        let total_frames_after = cache_guard.frames.len();
        if total_frames_after > total_frames_before {
            debug!(
                "TF cache updated: {} -> {} frames (added {} new frames: {:?})",
                total_frames_before,
                total_frames_after,
                total_frames_after - total_frames_before,
                new_frames
            );
        }
        cache_guard.last_update = std::time::Instant::now();
    }

    pub async fn get_tree(&self) -> TfTreeResponse {
        let cache = self.cache.lock().await;
        let frames_map = cache.frames.clone();

        // Find root frames (frames without parent)
        let root_frames: Vec<String> = frames_map
            .values()
            .filter(|f| f.parent_frame.is_none())
            .map(|f| f.frame_id.clone())
            .collect();

        // If no root frames found, default to "world"
        let root_frames = if root_frames.is_empty() {
            vec!["world".to_string()]
        } else {
            root_frames
        };

        let frames: Vec<TfFrame> = frames_map.into_values().collect();

        TfTreeResponse {
            frames,
            root_frames,
        }
    }
}
