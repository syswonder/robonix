// SPDX-License-Identifier: MulanPSL-2.0
// Semantic Map Module
//
// This module manages the semantic map (graph of entities).

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

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

