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
        let topics = self.topics.lock().await;
        let mut topics_vec: Vec<TopicInfo> = topics.values().cloned().collect();
        // Sort by topic name
        topics_vec.sort_by(|a, b| a.name.cmp(&b.name));
        TopicsResponse { topics: topics_vec }
    }

    /// Discover topics using ROS2 command line tools
    pub async fn discover_topics(&self) -> Result<(), Box<dyn std::error::Error>> {
        // Get list of topics using ros2 topic list
        // Source ROS2 environment first and add timeout to prevent hanging
        let setup_path = Self::get_ros2_setup_path();
        let output = tokio::time::timeout(
            tokio::time::Duration::from_secs(3),
            Command::new("bash")
                .arg("-c")
                .arg(format!(
                    "source {} && timeout 2 ros2 topic list",
                    setup_path
                ))
                .output(),
        )
        .await;

        let output = match output {
            Ok(Ok(output)) => output,
            Ok(Err(e)) => {
                return Err(format!("Failed to run ros2 topic list: {}", e).into());
            }
            Err(_) => {
                debug!("ros2 topic list timed out");
                return Ok(()); // Return empty result instead of error
            }
        };

        if !output.status.success() {
            let stderr = String::from_utf8_lossy(&output.stderr);
            debug!("ros2 topic list failed: {}", stderr);
            return Ok(()); // Return empty result instead of error
        }

        let topic_list = String::from_utf8_lossy(&output.stdout);
        // Get all topics (filter out empty lines)
        let topic_names: Vec<String> = topic_list
            .lines()
            .map(|s| s.trim().to_string())
            .filter(|s| !s.is_empty())
            .collect();

        debug!("Discovered {} topics", topic_names.len());

        // Get type for each topic
        let mut topics_map = self.topics.lock().await;

        // Create a set of currently existing topics for comparison
        let current_topics_set: HashSet<String> = topic_names.iter().cloned().collect();

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

        for topic_name in topic_names {
            // Skip if already exists
            if topics_map.contains_key(&topic_name) {
                continue;
            }

            // Get topic type using ros2 topic type
            // Source ROS2 environment first and add timeout to prevent hanging
            let setup_path = Self::get_ros2_setup_path();
            let topic_name_clone = topic_name.clone();
            let type_output = tokio::time::timeout(
                tokio::time::Duration::from_secs(2),
                Command::new("bash")
                    .arg("-c")
                    .arg(format!(
                        "source {} && timeout 1 ros2 topic type {}",
                        setup_path, &topic_name_clone
                    ))
                    .output(),
            )
            .await;

            let message_type = match type_output {
                Ok(Ok(output)) if output.status.success() => {
                    // Some topics have multiple types, collect all of them
                    let types: Vec<String> = String::from_utf8_lossy(&output.stdout)
                        .lines()
                        .map(|s| s.trim().to_string())
                        .filter(|s| !s.is_empty())
                        .collect();

                    if types.is_empty() {
                        "unknown".to_string()
                    } else {
                        types.join(", ")
                    }
                }
                _ => {
                    debug!(
                        "Failed to get type for topic {} (timeout or error)",
                        topic_name
                    );
                    "unknown".to_string()
                }
            };

            debug!("Topic {} has type {}", topic_name, message_type);

            let topic_name_clone = topic_name.clone();
            topics_map.insert(
                topic_name,
                TopicInfo {
                    name: topic_name_clone,
                    message_type: message_type.clone(),
                },
            );
        }
        drop(topics_map);

        Ok(())
    }

    /// Start monitoring topics by subscribing to them
    pub async fn start_monitoring(
        &self,
        _node: &mut Node,
    ) -> Result<(), Box<dyn std::error::Error>> {
        // First discover topics (this will also start monitoring for new topics)
        self.discover_topics().await?;

        // Spawn task to periodically discover new topics
        let monitor = Arc::new(self.clone());
        tokio::spawn(async move {
            let mut interval = tokio::time::interval(tokio::time::Duration::from_secs(5));
            loop {
                interval.tick().await;
                if let Err(e) = monitor.discover_topics().await {
                    debug!("Error discovering topics: {:?}", e);
                }
            }
        });

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
