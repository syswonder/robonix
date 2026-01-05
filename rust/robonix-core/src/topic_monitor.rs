// SPDX-License-Identifier: MulanPSL-2.0
// Topic Monitor Module
//
// Monitors ROS2 topics, their types, and publication frequencies

use log::debug;
use ros2_client::Node;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::sync::Arc;
use std::time::{Instant, SystemTime};
use tokio::process::Command;
use tokio::sync::Mutex;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TopicInfo {
    pub name: String,
    pub message_type: String,
    pub frequency: Option<f64>, // Hz
    pub last_message_time: Option<SystemTime>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TopicsResponse {
    pub topics: Vec<TopicInfo>,
}

pub struct TopicMonitor {
    topics: Arc<Mutex<HashMap<String, TopicInfo>>>,
    message_timestamps: Arc<Mutex<HashMap<String, Vec<Instant>>>>,
}

impl TopicMonitor {
    pub fn new() -> Self {
        Self {
            topics: Arc::new(Mutex::new(HashMap::new())),
            message_timestamps: Arc::new(Mutex::new(HashMap::new())),
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
        let topic_names: Vec<String> = topic_list
            .lines()
            .map(|s| s.trim().to_string())
            .filter(|s| !s.is_empty())
            .collect();

        debug!("Discovered {} topics", topic_names.len());

        // Get type for each topic and start monitoring new ones
        let mut topics_map = self.topics.lock().await;
        let mut new_topics = Vec::new();

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

            let topic_name_for_monitoring = topic_name.clone();
            topics_map.insert(
                topic_name,
                TopicInfo {
                    name: topic_name_for_monitoring.clone(),
                    message_type: message_type.clone(),
                    frequency: None,
                    last_message_time: None,
                },
            );

            // Track new topics to start monitoring (skip only if type is unknown)
            if message_type != "unknown" {
                new_topics.push(topic_name_for_monitoring.clone());
            }
        }
        drop(topics_map);

        // Start monitoring for newly discovered topics
        for topic_name in new_topics {
            let topic_name_clone = topic_name.clone();
            let timestamps = self.message_timestamps.clone();
            let topics = self.topics.clone();

            tokio::spawn(async move {
                Self::monitor_topic_frequency(topic_name_clone, timestamps, topics).await;
            });
        }

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

    async fn monitor_topic_frequency(
        topic_name: String,
        _timestamps: Arc<Mutex<HashMap<String, Vec<Instant>>>>,
        topics: Arc<Mutex<HashMap<String, TopicInfo>>>,
    ) {
        // Use ros2 topic hz to get frequency with timeout
        // Check less frequently to avoid too many processes
        let mut interval = tokio::time::interval(tokio::time::Duration::from_secs(5));
        loop {
            interval.tick().await;

            // Use timeout command to limit execution time (6 seconds)
            // This gives enough time for ros2 topic hz to collect messages and calculate frequency
            // Source ROS2 environment first
            let setup_path = Self::get_ros2_setup_path();
            let output = tokio::process::Command::new("bash")
                .arg("-c")
                .arg(format!(
                    "source {} && timeout 6 ros2 topic hz {} --window 10",
                    setup_path, &topic_name
                ))
                .output()
                .await;

            match output {
                Ok(output) => {
                    let stdout = String::from_utf8_lossy(&output.stdout);
                    let stderr = String::from_utf8_lossy(&output.stderr);
                    let combined = format!("{}\n{}", stdout, stderr);

                    // Parse frequency from output like "average rate: 10.000" or "average rate: 10.000 Hz"
                    // ros2 topic hz outputs multiple lines, get the last one for most accurate reading
                    let freq_lines: Vec<&str> = combined
                        .lines()
                        .filter(|line| line.contains("average rate"))
                        .collect();

                    if let Some(last_freq_line) = freq_lines.last() {
                        // Parse frequency value (format: "average rate: 10.000" or "average rate: 10.000 Hz")
                        if let Some(freq) = last_freq_line
                            .split_whitespace()
                            .nth(2)
                            .and_then(|s| s.parse::<f64>().ok())
                        {
                            let mut topics_guard = topics.lock().await;
                            if let Some(topic_info) = topics_guard.get_mut(&topic_name) {
                                topic_info.frequency = Some(freq);
                                topic_info.last_message_time = Some(SystemTime::now());
                            }
                        }
                    } else if combined.contains("does not appear to be published")
                        || combined.contains("no new messages")
                    {
                        // Topic exists but no messages received
                        let mut topics_guard = topics.lock().await;
                        if let Some(topic_info) = topics_guard.get_mut(&topic_name) {
                            topic_info.frequency = Some(0.0);
                        }
                    }
                }
                Err(e) => {
                    debug!("Failed to get frequency for topic {}: {:?}", topic_name, e);
                }
            }
        }
    }
}

impl Clone for TopicMonitor {
    fn clone(&self) -> Self {
        Self {
            topics: self.topics.clone(),
            message_timestamps: self.message_timestamps.clone(),
        }
    }
}
