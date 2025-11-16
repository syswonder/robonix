// SPDX-License-Identifier: MulanPSL-2.0
// Perception Module
//
// This module manages semantic maps and spatial maps (pointcloud2 data).
// It monitors skill registration and updates state when update_map is registered.

pub mod semantic_map;
pub mod spatial_map;

use ros2_client::Message;
use semantic_map::SemanticMap;
use serde::{Deserialize, Serialize};
use spatial_map::SpatialMap;
use std::sync::Arc;
use tokio::sync::RwLock;
use tracing::info;

/// Perception module state
pub struct PerceptionModule {
    semantic_map: Arc<RwLock<SemanticMap>>,
    spatial_map: Arc<RwLock<SpatialMap>>,
    is_updating_map: Arc<RwLock<bool>>,
}

impl PerceptionModule {
    pub fn new() -> Self {
        Self {
            semantic_map: Arc::new(RwLock::new(SemanticMap::new())),
            spatial_map: Arc::new(RwLock::new(SpatialMap::new())),
            is_updating_map: Arc::new(RwLock::new(false)),
        }
    }

    /// Check if a skill is registered and update state accordingly
    pub async fn on_skill_registered(&self, skill_name: &str) {
        if skill_name == "skl::update_map" {
            let mut updating = self.is_updating_map.write().await;
            *updating = true;
            info!("update_map skill registered - system is now continuously updating maps");
        }
    }

    /// Check if a skill is unregistered and update state accordingly
    pub async fn on_skill_unregistered(&self, skill_name: &str) {
        if skill_name == "skl::update_map" {
            let mut updating = self.is_updating_map.write().await;
            *updating = false;
            info!("update_map skill unregistered - system stopped updating maps");
        }
    }

    /// Get current map updating status
    pub async fn is_map_updating(&self) -> bool {
        *self.is_updating_map.read().await
    }

    /// Get semantic map (read-only)
    pub fn get_semantic_map(&self) -> Arc<RwLock<SemanticMap>> {
        self.semantic_map.clone()
    }

    /// Get spatial map (read-only)
    pub fn get_spatial_map(&self) -> Arc<RwLock<SpatialMap>> {
        self.spatial_map.clone()
    }

    /// Add entity to semantic map
    pub async fn add_entity(&self, entity: semantic_map::Entity) {
        let mut map = self.semantic_map.write().await;
        map.add_entity(entity);
    }

    /// Get entity by ID
    pub async fn get_entity(&self, id: &str) -> Option<semantic_map::Entity> {
        let map = self.semantic_map.read().await;
        map.get_entity(id).cloned()
    }

    /// Find entities by label
    pub async fn find_entities_by_label(&self, label: &str) -> Vec<semantic_map::Entity> {
        let map = self.semantic_map.read().await;
        map.find_entities_by_label(label)
            .iter()
            .map(|e| (*e).clone())
            .collect()
    }

    /// Find entities by path
    pub async fn find_entities_by_path(&self, path: &str) -> Vec<semantic_map::Entity> {
        let map = self.semantic_map.read().await;
        map.find_entities_by_path(path)
            .iter()
            .map(|e| (*e).clone())
            .collect()
    }

    /// Add spatial map entry
    pub async fn add_spatial_map_entry(&self, entry: spatial_map::SpatialMapEntry) {
        let mut map = self.spatial_map.write().await;
        map.add_entry(entry);
    }

    /// Get all spatial map entries
    pub async fn get_spatial_map_entries(&self) -> Vec<spatial_map::SpatialMapEntry> {
        let map = self.spatial_map.read().await;
        map.get_entries().to_vec()
    }
}

// Re-export types (use fully qualified paths in service messages to avoid conflicts)

// Service message types for map operations

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GetSemanticMapRequest {
    pub entity_id: Option<String>, // If None, return all entities
    pub label: Option<String>,     // Filter by label
    pub path: Option<String>,      // Filter by path
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GetSemanticMapResponse {
    pub success: bool,
    pub error_message: String,
    pub entities: Vec<semantic_map::Entity>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AddEntityRequest {
    pub entity: semantic_map::Entity,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AddEntityResponse {
    pub success: bool,
    pub error_message: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GetSpatialMapRequest {
    pub frame_id: Option<String>, // Filter by frame_id
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GetSpatialMapResponse {
    pub success: bool,
    pub error_message: String,
    pub entries: Vec<spatial_map::SpatialMapEntry>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AddSpatialMapEntryRequest {
    pub entry: spatial_map::SpatialMapEntry,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AddSpatialMapEntryResponse {
    pub success: bool,
    pub error_message: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GetMapStatusRequest {}
impl Message for GetMapStatusRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GetMapStatusResponse {
    pub is_updating: bool,
}
impl Message for GetMapStatusResponse {}

// Implement Message trait for service types
impl Message for GetSemanticMapRequest {}
impl Message for GetSemanticMapResponse {}
impl Message for AddEntityRequest {}
impl Message for AddEntityResponse {}
impl Message for GetSpatialMapRequest {}
impl Message for GetSpatialMapResponse {}
impl Message for AddSpatialMapEntryRequest {}
impl Message for AddSpatialMapEntryResponse {}

