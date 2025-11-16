// SPDX-License-Identifier: MulanPSL-2.0
// Spatial Map Module
//
// This module manages the spatial map (collection of pointcloud2 data).

use serde::{Deserialize, Serialize};

/// Spatial map entry containing pointcloud2 data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpatialMapEntry {
    pub frame_id: String,
    pub timestamp: u64,       // Unix timestamp in nanoseconds
    pub source_skill: String, // Which skill provided this map
    
    // PointCloud2 data
    pub height: u32,          // Height of point cloud (1 for unordered)
    pub width: u32,           // Width of point cloud (number of points for unordered)
    pub point_data: Vec<u8>,  // Actual point cloud data (binary blob)
    pub point_step: u32,      // Length of a point in bytes
    pub row_step: u32,        // Length of a row in bytes
    pub is_dense: bool,       // True if there are no invalid points
    pub is_bigendian: bool,   // Is this data bigendian?
    
    // Optional: topic name where this pointcloud was published (for reference)
    pub source_topic: Option<String>,
}

/// Spatial map (collection of pointcloud2 data)
#[derive(Debug, Clone)]
pub struct SpatialMap {
    entries: Vec<SpatialMapEntry>,
}

impl SpatialMap {
    pub fn new() -> Self {
        Self {
            entries: Vec::new(),
        }
    }

    pub fn add_entry(&mut self, entry: SpatialMapEntry) {
        self.entries.push(entry);
    }

    pub fn get_entries(&self) -> &[SpatialMapEntry] {
        &self.entries
    }

    pub fn get_latest_entry(&self) -> Option<&SpatialMapEntry> {
        self.entries.last()
    }

    pub fn clear(&mut self) {
        self.entries.clear();
    }
}

