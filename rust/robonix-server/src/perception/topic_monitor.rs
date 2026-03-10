// SPDX-License-Identifier: MulanPSL-2.0
// Topic Monitor Module
//
// Monitors ROS2 topics and their types

use log::debug;
use ros2_client::Node;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::sync::Arc;
use tokio::process::Command;
use tokio::sync::Mutex;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TopicInfo {
    pub name: String,
    pub message_type: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TopicsResponse {
    pub topics: Vec<TopicInfo>,
}

pub struct TopicMonitor {
    topics: Arc<Mutex<HashMap<String, TopicInfo>>>,
}

impl TopicMonitor {
    pub fn new() -> Self {
        Self {
            topics: Arc::new(Mutex::new(HashMap::new())),
        }
    }

    /// Get ROS2 setup.bash path, checking ROS_DISTRO environment variable first
    fn get_ros2_setup_path() -> String {
        let distro = std::env::var("ROS_DISTRO").unwrap_or_else(|_| "humble".to_string());
        format!("/opt/ros/{}/setup.bash", distro)
    }

    pub async fn get_topics(&self) -> TopicsResponse {
        // Refresh topics list on each call (on-demand discovery)
        // This avoids maintaining a periodic async task
        let _ = self.discover_topics().await; // Ignore errors, use cached data if discovery fails

        let topics = self.topics.lock().await;
        let mut topics_vec: Vec<TopicInfo> = topics.values().cloned().collect();
        // Sort by topic name
        topics_vec.sort_by(|a, b| a.name.cmp(&b.name));
        TopicsResponse { topics: topics_vec }
    }

    /// Discover topics using ROS2 command line tools
    /// Uses `ros2 topic list -t` to get all topics and their types in one command
    pub async fn discover_topics(&self) -> Result<(), Box<dyn std::error::Error>> {
        // Get list of topics with types using ros2 topic list -t
        // Source ROS2 environment first and add timeout to prevent hanging
        let setup_path = Self::get_ros2_setup_path();
        let output = tokio::time::timeout(
            tokio::time::Duration::from_secs(3),
            Command::new("bash")
                .arg("-c")
                .arg(format!(
                    "source {} && timeout 2 ros2 topic list -t",
                    setup_path
                ))
                .output(),
        )
        .await;

        let output = match output {
            Ok(Ok(output)) => output,
            Ok(Err(e)) => {
                return Err(format!("Failed to run ros2 topic list -t: {}", e).into());
            }
            Err(_) => {
                debug!("ros2 topic list -t timed out");
                return Ok(()); // Return empty result instead of error
            }
        };

        if !output.status.success() {
            let stderr = String::from_utf8_lossy(&output.stderr);
            debug!("ros2 topic list -t failed: {}", stderr);
            return Ok(()); // Return empty result instead of error
        }

        let topic_list = String::from_utf8_lossy(&output.stdout);

        // Parse output: each line is "topic_name [message_type]"
        // Example: "/amcl_pose [geometry_msgs/msg/PoseWithCovarianceStamped]"
        let mut topics_map = self.topics.lock().await;
        let mut current_topics_set = HashSet::new();

        for line in topic_list.lines() {
            let line = line.trim();
            if line.is_empty() {
                continue;
            }

            // Parse line: "topic_name [message_type1, message_type2, ...]" or just "topic_name"
            // Example: "/clock [builtin_interfaces/msg/Time, rosgraph_msgs/msg/Clock]"
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.is_empty() {
                continue;
            }

            let topic_name = parts[0].to_string();
            let message_type = if parts.len() >= 2 {
                // Join all parts after topic name (handles multiple types in brackets)
                // Example: "[builtin_interfaces/msg/Time," "rosgraph_msgs/msg/Clock]"
                let types_str = parts[1..].join(" ");
                // Remove brackets and trim
                types_str
                    .trim_matches(|c| c == '[' || c == ']')
                    .trim()
                    .to_string()
            } else {
                "unknown".to_string()
            };

            current_topics_set.insert(topic_name.clone());

            // Update or insert topic
            topics_map.insert(
                topic_name.clone(),
                TopicInfo {
                    name: topic_name.clone(),
                    message_type: message_type.clone(),
                },
            );
        }

        // Remove topics that no longer exist
        let topics_to_remove: Vec<String> = topics_map
            .keys()
            .filter(|topic_name| !current_topics_set.contains(*topic_name))
            .cloned()
            .collect();

        for topic_name in &topics_to_remove {
            debug!("Removing topic {} (no longer exists)", topic_name);
            topics_map.remove(topic_name);
        }

        debug!("Discovered {} topics", topics_map.len());
        drop(topics_map);

        Ok(())
    }

    /// Start monitoring topics by discovering them once
    /// No periodic task - topics are discovered on-demand when get_topics() is called
    pub async fn start_monitoring(
        &self,
        _node: &mut Node,
    ) -> Result<(), Box<dyn std::error::Error>> {
        // Initial discovery
        self.discover_topics().await?;
        Ok(())
    }
}

impl Clone for TopicMonitor {
    fn clone(&self) -> Self {
        Self {
            topics: self.topics.clone(),
        }
    }
}
