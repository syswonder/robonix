// SPDX-License-Identifier: MulanPSL-2.0
// Image Monitor Module
//
// Monitors image topics and stores latest images for web UI display

use futures_util::stream::StreamExt;
use log::{info, trace, warn};
use ros2_client::{
    Message, MessageTypeName, Name, Node, Subscription,
    rustdds::{
        QosPolicyBuilder,
        policy::{self, Reliability},
    },
};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::SystemTime;
use tokio::fs;
use tokio::sync::Mutex;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImageTopicInfo {
    pub topic_name: String,
    pub message_type: String,
    pub last_update: Option<SystemTime>,
    pub image_paths: Vec<ImagePathInfo>, // Paths to recent images (up to 10, sorted by time, newest first)
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImagePathInfo {
    pub path: String,
    pub timestamp: u64, // Unix timestamp in seconds
}

// sensor_msgs/msg/Image message structure
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
pub struct ImageMsg {
    pub header: Header,
    pub height: u32,
    pub width: u32,
    pub encoding: String,
    pub is_bigendian: u8,
    pub step: u32,
    pub data: Vec<u8>,
}

impl Message for ImageMsg {}

pub struct ImageMonitor {
    image_topics: Arc<Mutex<HashMap<String, ImageTopicInfo>>>,
    image_storage_dir: PathBuf,
    subscriptions: Arc<Mutex<HashMap<String, Arc<Subscription<ImageMsg>>>>>,
}

impl ImageMonitor {
    pub fn new(storage_dir: PathBuf) -> Self {
        // Create storage directory if it doesn't exist
        if let Err(e) = std::fs::create_dir_all(&storage_dir) {
            warn!(
                "Failed to create image storage directory {:?}: {}",
                storage_dir, e
            );
        }

        Self {
            image_topics: Arc::new(Mutex::new(HashMap::new())),
            image_storage_dir: storage_dir,
            subscriptions: Arc::new(Mutex::new(HashMap::new())),
        }
    }

    /// Get all image topics
    pub async fn get_image_topics(&self) -> Vec<ImageTopicInfo> {
        let topics = self.image_topics.lock().await;
        topics.values().cloned().collect()
    }

    /// Update image topic info (called when a new image is received)
    pub async fn update_image(&self, topic_name: String, image_path: String) {
        let mut topics = self.image_topics.lock().await;
        let timestamp = SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs();

        if let Some(info) = topics.get_mut(&topic_name) {
            info.last_update = Some(SystemTime::now());
            // Replace with latest image (only keep one)
            info.image_paths = vec![ImagePathInfo {
                path: image_path,
                timestamp,
            }];
        } else {
            // New topic, add it
            topics.insert(
                topic_name.clone(),
                ImageTopicInfo {
                    topic_name: topic_name.clone(),
                    message_type: "sensor_msgs/msg/Image".to_string(),
                    last_update: Some(SystemTime::now()),
                    image_paths: vec![ImagePathInfo {
                        path: image_path,
                        timestamp,
                    }],
                },
            );
        }
    }

    /// Register an image topic and start subscribing (called when topic is discovered)
    pub async fn register_image_topic(
        &self,
        node: &mut Node,
        topic_name: String,
        message_type: String,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let mut subscriptions = self.subscriptions.lock().await;
        if subscriptions.contains_key(&topic_name) {
            // Already subscribed
            return Ok(());
        }

        // Register in topics map
        let mut topics = self.image_topics.lock().await;
        if !topics.contains_key(&topic_name) {
            topics.insert(
                topic_name.clone(),
                ImageTopicInfo {
                    topic_name: topic_name.clone(),
                    message_type: message_type.clone(),
                    last_update: None,
                    image_paths: Vec::new(),
                },
            );
        }
        drop(topics);

        // Create QoS for image topics
        let image_qos = QosPolicyBuilder::new()
            .history(policy::History::KeepLast { depth: 1 })
            .reliability(Reliability::BestEffort)
            .durability(policy::Durability::Volatile)
            .build();

        // Parse topic name (format: "/namespace/topic_name")
        let topic_name_parts: Vec<&str> = topic_name.split('/').filter(|s| !s.is_empty()).collect();
        let namespace = if topic_name_parts.len() > 1 {
            format!(
                "/{}",
                topic_name_parts[..topic_name_parts.len() - 1].join("/")
            )
        } else {
            "/".to_string()
        };
        let topic_part = topic_name_parts
            .last()
            .map(|s| s.to_string())
            .unwrap_or_else(|| topic_name.clone());

        // Create topic
        let image_topic = node.create_topic(
            &Name::new(&namespace, &topic_part)?,
            MessageTypeName::new("sensor_msgs", "Image"),
            &image_qos,
        )?;

        // Subscribe to topic
        let subscription: Subscription<ImageMsg> = node.create_subscription(&image_topic, None)?;
        let subscription_arc = Arc::new(subscription);
        subscriptions.insert(topic_name.clone(), subscription_arc.clone());
        drop(subscriptions);

        // Spawn task to handle image messages
        let image_topics_clone = self.image_topics.clone();
        let image_storage_dir_clone = self.image_storage_dir.clone();
        let topic_name_clone = topic_name.clone();
        tokio::spawn(async move {
            trace!("Subscribed to image topic: {}", topic_name_clone);
            let mut stream = Box::pin(subscription_arc.async_stream());
            let mut message_count = 0;
            while let Some(result) = stream.next().await {
                match result {
                    Ok((msg, _msg_info)) => {
                        message_count += 1;
                        if message_count <= 5 || message_count % 100 == 0 {
                            trace!(
                                "Received image from {}: {}x{}, encoding={}, data_len={}",
                                topic_name_clone,
                                msg.width,
                                msg.height,
                                msg.encoding,
                                msg.data.len()
                            );
                        }

                        // Convert image to JPEG and save
                        if let Err(e) = Self::process_image_message_internal(
                            &image_topics_clone,
                            &image_storage_dir_clone,
                            &topic_name_clone,
                            &msg,
                        )
                        .await
                        {
                            trace!("Failed to process image from {}: {}", topic_name_clone, e);
                        }
                    }
                    Err(e) => {
                        trace!("Error receiving image from {}: {:?}", topic_name_clone, e);
                    }
                }
            }
        });

        info!("Registered and subscribed to image topic: {}", topic_name);
        Ok(())
    }

    /// Process received image message: convert to JPEG and save (internal static method)
    async fn process_image_message_internal(
        image_topics: &Arc<Mutex<HashMap<String, ImageTopicInfo>>>,
        image_storage_dir: &PathBuf,
        topic_name: &str,
        msg: &ImageMsg,
    ) -> Result<(), String> {
        trace!(
            "Processing image from {}: {}x{}, encoding={}, data_len={}, step={}",
            topic_name,
            msg.width,
            msg.height,
            msg.encoding,
            msg.data.len(),
            msg.step
        );

        // Convert sensor_msgs/Image raw data to JPEG
        // The data field contains raw pixel data, not JPEG
        let jpeg_data = match Self::convert_image_to_jpeg(msg) {
            Ok(data) => {
                trace!(
                    "Successfully converted image to JPEG, size: {} bytes",
                    data.len()
                );
                data
            }
            Err(e) => {
                trace!(
                    "Failed to convert image to JPEG: {}, saving raw data instead",
                    e
                );
                // Fallback: save raw data (may not work in browser)
                msg.data.clone()
            }
        };

        // Save image file
        let sanitized_name = topic_name.replace('/', "_").replace("~", "_");
        let timestamp = SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs();
        let filename = format!("{}_{}.jpg", sanitized_name, timestamp);
        let filepath = image_storage_dir.join(&filename);

        trace!("Saving image to: {:?}", filepath);
        fs::write(&filepath, &jpeg_data).await.map_err(|e| {
            let err_msg = format!("Failed to write image file {:?}: {}", filepath, e);
            trace!("{}", err_msg);
            err_msg
        })?;

        // Verify file was written
        if let Ok(metadata) = fs::metadata(&filepath).await {
            trace!(
                "Image file saved successfully: {:?}, size: {} bytes",
                filepath,
                metadata.len()
            );
        } else {
            trace!(
                "Warning: Could not verify image file after write: {:?}",
                filepath
            );
        }

        let image_path = format!("images/{}", filename);
        trace!("Image path for API: {}", image_path);

        // Update image topic info (only keep latest one)
        let mut topics = image_topics.lock().await;
        let timestamp_secs = SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs();

        if let Some(info) = topics.get_mut(topic_name) {
            info.last_update = Some(SystemTime::now());
            // Replace with latest image (only keep one)
            info.image_paths = vec![ImagePathInfo {
                path: image_path.clone(),
                timestamp: timestamp_secs,
            }];
            trace!(
                "Updated image topic {} with new image: {}",
                topic_name, image_path
            );
        } else {
            topics.insert(
                topic_name.to_string(),
                ImageTopicInfo {
                    topic_name: topic_name.to_string(),
                    message_type: "sensor_msgs/msg/Image".to_string(),
                    last_update: Some(SystemTime::now()),
                    image_paths: vec![ImagePathInfo {
                        path: image_path.clone(),
                        timestamp: timestamp_secs,
                    }],
                },
            );
            trace!("Created new image topic entry: {}", topic_name);
        }

        Ok(())
    }

    /// Convert sensor_msgs/Image raw data to JPEG format
    fn convert_image_to_jpeg(msg: &ImageMsg) -> Result<Vec<u8>, String> {
        use image::{ImageBuffer, Rgb, RgbImage};

        let width = msg.width;
        let height = msg.height;
        let step = msg.step as usize;

        trace!(
            "Converting image: {}x{}, encoding={}, step={}, data_len={}",
            width,
            height,
            msg.encoding,
            step,
            msg.data.len()
        );

        // Handle different encodings
        // Check encoding (case-insensitive, handle variations)
        let encoding_lower = msg.encoding.to_lowercase();
        match encoding_lower.as_str() {
            "rgb8" | "bgr8" | "8uc3" => {
                // RGB8 or BGR8: 3 bytes per pixel
                let expected_size = (width as usize) * (height as usize) * 3;
                if msg.data.len() < expected_size {
                    return Err(format!(
                        "Image data too short: expected {}, got {}",
                        expected_size,
                        msg.data.len()
                    ));
                }

                let mut img: RgbImage = ImageBuffer::new(width, height);

                for y in 0..height {
                    for x in 0..width {
                        let idx = (y as usize) * step + (x as usize) * 3;
                        if idx + 2 < msg.data.len() {
                            let r = msg.data[idx];
                            let g = msg.data[idx + 1];
                            let b = msg.data[idx + 2];

                            // Handle BGR vs RGB (case-insensitive)
                            let (r, g, b) = if encoding_lower == "bgr8" || encoding_lower == "8uc3"
                            {
                                (b, g, r) // Swap R and B for BGR
                            } else {
                                (r, g, b)
                            };

                            img.put_pixel(x, y, Rgb([r, g, b]));
                        }
                    }
                }

                // Encode to JPEG
                let mut buffer = Vec::new();
                let mut cursor = std::io::Cursor::new(&mut buffer);
                img.write_to(&mut cursor, image::ImageFormat::Jpeg)
                    .map_err(|e| format!("Failed to encode JPEG: {}", e))?;

                Ok(buffer)
            }
            "rgba8" | "bgra8" | "8uc4" => {
                // RGBA8 or BGRA8: 4 bytes per pixel (R, G, B, A)
                // We'll ignore the alpha channel and convert to RGB
                let expected_size = (width as usize) * (height as usize) * 4;
                if msg.data.len() < expected_size {
                    return Err(format!(
                        "Image data too short: expected {}, got {}",
                        expected_size,
                        msg.data.len()
                    ));
                }

                let mut img: RgbImage = ImageBuffer::new(width, height);

                for y in 0..height {
                    for x in 0..width {
                        let idx = (y as usize) * step + (x as usize) * 4;
                        if idx + 3 < msg.data.len() {
                            let r = msg.data[idx];
                            let g = msg.data[idx + 1];
                            let b = msg.data[idx + 2];
                            // Alpha channel at idx + 3, we ignore it

                            // Handle BGRA vs RGBA (case-insensitive)
                            let (r, g, b) = if encoding_lower == "bgra8" || encoding_lower == "8uc4"
                            {
                                (b, g, r) // Swap R and B for BGRA
                            } else {
                                (r, g, b)
                            };

                            img.put_pixel(x, y, Rgb([r, g, b]));
                        }
                    }
                }

                // Encode to JPEG
                let mut buffer = Vec::new();
                let mut cursor = std::io::Cursor::new(&mut buffer);
                img.write_to(&mut cursor, image::ImageFormat::Jpeg)
                    .map_err(|e| format!("Failed to encode JPEG: {}", e))?;

                Ok(buffer)
            }
            "mono8" => {
                // Grayscale: 1 byte per pixel
                let expected_size = (width as usize) * (height as usize);
                if msg.data.len() < expected_size {
                    return Err(format!(
                        "Image data too short: expected {}, got {}",
                        expected_size,
                        msg.data.len()
                    ));
                }

                let mut img: ImageBuffer<Rgb<u8>, Vec<u8>> = ImageBuffer::new(width, height);

                for y in 0..height {
                    for x in 0..width {
                        let idx = (y as usize) * step + (x as usize);
                        if idx < msg.data.len() {
                            let gray = msg.data[idx];
                            img.put_pixel(x, y, Rgb([gray, gray, gray]));
                        }
                    }
                }

                // Encode to JPEG
                let mut buffer = Vec::new();
                let mut cursor = std::io::Cursor::new(&mut buffer);
                img.write_to(&mut cursor, image::ImageFormat::Jpeg)
                    .map_err(|e| format!("Failed to encode JPEG: {}", e))?;

                Ok(buffer)
            }
            "16uc1" | "mono16" => {
                // Depth image: 16-bit unsigned integer, 1 channel (2 bytes per pixel)
                // Convert to normalized 8-bit grayscale for visualization
                use byteorder::{BigEndian, ByteOrder, LittleEndian};

                let expected_size = (width as usize) * (height as usize) * 2;
                if msg.data.len() < expected_size {
                    return Err(format!(
                        "Image data too short: expected {}, got {}",
                        expected_size,
                        msg.data.len()
                    ));
                }

                let mut img: ImageBuffer<Rgb<u8>, Vec<u8>> = ImageBuffer::new(width, height);
                let is_bigendian = msg.is_bigendian != 0;

                // Find min and max for normalization (excluding 0 which is invalid depth)
                let mut min_val = u16::MAX;
                let mut max_val = 0u16;

                for y in 0..height {
                    for x in 0..width {
                        let idx = (y as usize) * step + (x as usize) * 2;
                        if idx + 1 < msg.data.len() {
                            let val = if is_bigendian {
                                BigEndian::read_u16(&msg.data[idx..idx + 2])
                            } else {
                                LittleEndian::read_u16(&msg.data[idx..idx + 2])
                            };
                            if val > 0 && val < min_val {
                                min_val = val;
                            }
                            if val > max_val {
                                max_val = val;
                            }
                        }
                    }
                }

                // Normalize and convert to RGB
                let range = if max_val > min_val {
                    max_val - min_val
                } else {
                    1
                };
                for y in 0..height {
                    for x in 0..width {
                        let idx = (y as usize) * step + (x as usize) * 2;
                        if idx + 1 < msg.data.len() {
                            let val = if is_bigendian {
                                BigEndian::read_u16(&msg.data[idx..idx + 2])
                            } else {
                                LittleEndian::read_u16(&msg.data[idx..idx + 2])
                            };

                            // Normalize to 0-255, with 0 (invalid depth) as black
                            let normalized = if val == 0 {
                                0u8
                            } else {
                                (((val - min_val) as f32 / range as f32) * 255.0) as u8
                            };

                            // Use grayscale visualization for depth
                            img.put_pixel(x, y, Rgb([normalized, normalized, normalized]));
                        }
                    }
                }

                // Encode to JPEG
                let mut buffer = Vec::new();
                let mut cursor = std::io::Cursor::new(&mut buffer);
                img.write_to(&mut cursor, image::ImageFormat::Jpeg)
                    .map_err(|e| format!("Failed to encode JPEG: {}", e))?;

                Ok(buffer)
            }
            "32fc1" => {
                // Depth image: 32-bit float, 1 channel (4 bytes per pixel)
                // Convert to normalized 8-bit grayscale for visualization
                use byteorder::{BigEndian, ByteOrder, LittleEndian};

                let expected_size = (width as usize) * (height as usize) * 4;
                if msg.data.len() < expected_size {
                    return Err(format!(
                        "Image data too short: expected {}, got {}",
                        expected_size,
                        msg.data.len()
                    ));
                }

                let mut img: ImageBuffer<Rgb<u8>, Vec<u8>> = ImageBuffer::new(width, height);
                let is_bigendian = msg.is_bigendian != 0;

                // Find min and max for normalization (excluding NaN and Inf)
                let mut min_val = f32::MAX;
                let mut max_val = f32::MIN;

                for y in 0..height {
                    for x in 0..width {
                        let idx = (y as usize) * step + (x as usize) * 4;
                        if idx + 3 < msg.data.len() {
                            let val = if is_bigendian {
                                BigEndian::read_f32(&msg.data[idx..idx + 4])
                            } else {
                                LittleEndian::read_f32(&msg.data[idx..idx + 4])
                            };
                            if val.is_finite() && val > 0.0 {
                                if val < min_val {
                                    min_val = val;
                                }
                                if val > max_val {
                                    max_val = val;
                                }
                            }
                        }
                    }
                }

                // Normalize and convert to RGB
                let range = if max_val > min_val {
                    max_val - min_val
                } else {
                    1.0
                };
                for y in 0..height {
                    for x in 0..width {
                        let idx = (y as usize) * step + (x as usize) * 4;
                        if idx + 3 < msg.data.len() {
                            let val = if is_bigendian {
                                BigEndian::read_f32(&msg.data[idx..idx + 4])
                            } else {
                                LittleEndian::read_f32(&msg.data[idx..idx + 4])
                            };

                            // Normalize to 0-255, with NaN/Inf/zero as black
                            let normalized = if !val.is_finite() || val <= 0.0 {
                                0u8
                            } else {
                                (((val - min_val) / range) * 255.0).clamp(0.0, 255.0) as u8
                            };

                            // Use grayscale visualization for depth
                            img.put_pixel(x, y, Rgb([normalized, normalized, normalized]));
                        }
                    }
                }

                // Encode to JPEG
                let mut buffer = Vec::new();
                let mut cursor = std::io::Cursor::new(&mut buffer);
                img.write_to(&mut cursor, image::ImageFormat::Jpeg)
                    .map_err(|e| format!("Failed to encode JPEG: {}", e))?;

                Ok(buffer)
            }
            _ => {
                trace!(
                    "Unsupported encoding: {}, cannot convert to JPEG",
                    msg.encoding
                );
                Err(format!(
                    "Unsupported encoding: {} (supported: rgb8, bgr8, rgba8, bgra8, mono8, 16UC1, mono16, 32FC1)",
                    msg.encoding
                ))
            }
        }
    }

    /// Get storage directory path
    pub fn get_storage_dir(&self) -> &Path {
        &self.image_storage_dir
    }

    /// Save image data to file and return relative path
    pub async fn save_image(&self, topic_name: &str, image_data: &[u8]) -> Result<String, String> {
        // Sanitize topic name for filename
        let sanitized_name = topic_name.replace('/', "_").replace("~", "_");

        // Create filename with timestamp
        let timestamp = SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs();
        let filename = format!("{}_{}.jpg", sanitized_name, timestamp);
        let filepath = self.image_storage_dir.join(&filename);

        // Write image data
        fs::write(&filepath, image_data)
            .await
            .map_err(|e| format!("Failed to write image file: {}", e))?;

        // Remove old images for this topic (keep only last 1)
        self.cleanup_old_images(&sanitized_name, 1).await;

        // Return relative path for API
        Ok(format!("images/{}", filename))
    }

    /// Clean up old images for a topic, keeping only the last N images
    async fn cleanup_old_images(&self, sanitized_topic_name: &str, keep_count: usize) {
        // Use blocking I/O for directory reading (simpler and acceptable for cleanup)
        let storage_dir = self.image_storage_dir.clone();
        let topic_name = sanitized_topic_name.to_string();

        tokio::task::spawn_blocking(move || {
            if let Ok(entries) = std::fs::read_dir(&storage_dir) {
                let mut entry_vec: Vec<(std::path::PathBuf, Option<std::time::SystemTime>)> =
                    entries
                        .filter_map(|e| e.ok())
                        .filter_map(|e| {
                            let path = e.path();
                            if path
                                .file_name()
                                .and_then(|n| n.to_str())
                                .map(|s| s.starts_with(&topic_name))
                                .unwrap_or(false)
                            {
                                let metadata = std::fs::metadata(&path).ok();
                                let modified = metadata.and_then(|m| m.modified().ok());
                                Some((path, modified))
                            } else {
                                None
                            }
                        })
                        .collect();

                // Sort by modified time (newest first)
                entry_vec.sort_by(|a, b| b.1.cmp(&a.1));

                // Remove all but the last N images
                for (path, _) in entry_vec.into_iter().skip(keep_count) {
                    if let Err(e) = std::fs::remove_file(&path) {
                        trace!("Failed to remove old image file: {}", e);
                    }
                }
            }
        })
        .await
        .ok();
    }
}
