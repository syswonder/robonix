// SPDX-License-Identifier: MulanPSL-2.0
// Perception Module
//
// This module manages semantic maps and spatial maps (pointcloud2).
// It monitors skill registration and updates state when update_map is registered.

use ros2_client::Message;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;
use tracing::info;

// Semantic Map Structures

/// Entity relation type
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum RelationType {
    ChildOf,        // For hierarchical structure (e.g., table-child_of->room)
    OnTop,          // Spatial relationship
    Inside,         // Spatial containment
    Near,           // Spatial proximity
    Custom(String), // Custom relation type
}

/// Entity relation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Relation {
    pub relation_type: RelationType,
    pub target_entity_id: String,
}

/// Entity texture/material type
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum TextureType {
    Wood,
    Plastic,
    Metal,
    Unknown,
}

/// 3D bounding box
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BoundingBox {
    pub scale_x: f64,
    pub scale_y: f64,
    pub scale_z: f64,
    pub yaw: f64,
}

/// 3D center point
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Point3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// Frame mapping to 3D space
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FrameMapping {
    pub center: Point3D,
    pub bbox: Option<BoundingBox>,
    pub texture: Option<TextureType>,
    pub frame_id: String,
}

/// Semantic entity node
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Entity {
    pub id: String,
    pub label: String,
    pub relations: Vec<Relation>,
    pub registered_skills: Vec<String>, // For controllable entities like robots
    pub registered_capabilities: Vec<String>,
    pub frame_mapping: Option<FrameMapping>,
}

impl Entity {
    /// Build entity path using child_of relations
    pub fn build_path(&self, entities: &HashMap<String, Entity>) -> String {
        let mut path = vec![self.label.clone()];
        let mut current_id = self.id.clone();

        // Traverse up the child_of chain
        loop {
            let entity = match entities.get(&current_id) {
                Some(e) => e,
                None => break,
            };

            // Find child_of relation
            let parent_relation = entity
                .relations
                .iter()
                .find(|r| matches!(r.relation_type, RelationType::ChildOf));

            match parent_relation {
                Some(rel) => {
                    if let Some(parent) = entities.get(&rel.target_entity_id) {
                        path.insert(0, parent.label.clone());
                        current_id = rel.target_entity_id.clone();
                    } else {
                        break;
                    }
                }
                None => break,
            }
        }

        format!("/{}", path.join("/"))
    }
}

/// Semantic map (graph of entities)
#[derive(Debug, Clone)]
pub struct SemanticMap {
    entities: HashMap<String, Entity>,
}

impl SemanticMap {
    pub fn new() -> Self {
        Self {
            entities: HashMap::new(),
        }
    }

    pub fn add_entity(&mut self, entity: Entity) {
        self.entities.insert(entity.id.clone(), entity);
    }

    pub fn get_entity(&self, id: &str) -> Option<&Entity> {
        self.entities.get(id)
    }

    pub fn get_entity_mut(&mut self, id: &str) -> Option<&mut Entity> {
        self.entities.get_mut(id)
    }

    pub fn remove_entity(&mut self, id: &str) -> Option<Entity> {
        self.entities.remove(id)
    }

    pub fn get_all_entities(&self) -> Vec<&Entity> {
        self.entities.values().collect()
    }

    pub fn find_entities_by_label(&self, label: &str) -> Vec<&Entity> {
        self.entities
            .values()
            .filter(|e| e.label == label)
            .collect()
    }

    pub fn find_entities_by_path(&self, path: &str) -> Vec<&Entity> {
        // Path format: /room/table/box
        let parts: Vec<&str> = path.trim_start_matches('/').split('/').collect();
        if parts.is_empty() {
            return Vec::new();
        }

        // Find root entity
        let root_label = parts[0];
        let mut candidates: Vec<&Entity> = self
            .entities
            .values()
            .filter(|e| {
                e.label == root_label
                    && !e
                        .relations
                        .iter()
                        .any(|r| matches!(r.relation_type, RelationType::ChildOf))
            })
            .collect();

        // Traverse down the path
        for part in parts.iter().skip(1) {
            let mut next_candidates = Vec::new();
            for candidate in &candidates {
                // Find child with matching label
                for rel in &candidate.relations {
                    if matches!(rel.relation_type, RelationType::ChildOf) {
                        if let Some(child) = self.entities.get(&rel.target_entity_id) {
                            if child.label == *part {
                                next_candidates.push(child);
                            }
                        }
                    }
                }
            }
            candidates = next_candidates;
        }

        candidates
    }
}

// Spatial Map Structures

/// Spatial map entry (pointcloud2 reference)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpatialMapEntry {
    pub frame_id: String,
    pub timestamp: u64,       // Unix timestamp in nanoseconds
    pub source_skill: String, // Which skill provided this map
}

/// Spatial map (collection of pointcloud2 references)
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

// Perception Module

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
    pub async fn add_entity(&self, entity: Entity) {
        let mut map = self.semantic_map.write().await;
        map.add_entity(entity);
    }

    /// Get entity by ID
    pub async fn get_entity(&self, id: &str) -> Option<Entity> {
        let map = self.semantic_map.read().await;
        map.get_entity(id).cloned()
    }

    /// Find entities by label
    pub async fn find_entities_by_label(&self, label: &str) -> Vec<Entity> {
        let map = self.semantic_map.read().await;
        map.find_entities_by_label(label)
            .iter()
            .map(|e| (*e).clone())
            .collect()
    }

    /// Find entities by path
    pub async fn find_entities_by_path(&self, path: &str) -> Vec<Entity> {
        let map = self.semantic_map.read().await;
        map.find_entities_by_path(path)
            .iter()
            .map(|e| (*e).clone())
            .collect()
    }

    /// Add spatial map entry
    pub async fn add_spatial_map_entry(&self, entry: SpatialMapEntry) {
        let mut map = self.spatial_map.write().await;
        map.add_entry(entry);
    }

    /// Get all spatial map entries
    pub async fn get_spatial_map_entries(&self) -> Vec<SpatialMapEntry> {
        let map = self.spatial_map.read().await;
        map.get_entries().to_vec()
    }
}

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
    pub entities: Vec<Entity>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AddEntityRequest {
    pub entity: Entity,
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
    pub entries: Vec<SpatialMapEntry>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AddSpatialMapEntryRequest {
    pub entry: SpatialMapEntry,
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
