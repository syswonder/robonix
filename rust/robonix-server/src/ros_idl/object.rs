// SPDX-License-Identifier: MulanPSL-2.0
// Object ROS IDL Message Types (robonix_sdk/Object.msg)

use serde::{Deserialize, Serialize};

/// Time message (builtin_interfaces/Time.msg)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Time {
    pub sec: i32,
    pub nanosec: u32,
}

impl ros2_client::Message for Time {}

/// Dict message (robonix_sdk/Dict.msg)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Dict {
    pub keys: Vec<String>,
    pub values: Vec<String>,
}

impl ros2_client::Message for Dict {}

/// Point3D message (robonix_sdk/Point3D.msg)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Point3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl ros2_client::Message for Point3D {}

/// BoundingBox message (robonix_sdk/BoundingBox.msg)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BoundingBox {
    pub scale_x: f64,
    pub scale_y: f64,
    pub scale_z: f64,
    pub yaw: f64,
}

impl ros2_client::Message for BoundingBox {}

/// RelationType message (robonix_sdk/RelationType.msg)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RelationType {
    pub r#type: u8,          // CHILD_OF=0, ON_TOP=1, INSIDE=2, NEAR=3, CUSTOM=4
    pub custom_type: String, // Only used when type == CUSTOM
}

impl ros2_client::Message for RelationType {}

/// Relation message (robonix_sdk/Relation.msg)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Relation {
    pub relation_type: RelationType,
    pub target_entity_id: String,
}

impl ros2_client::Message for Relation {}

/// TextureType message (robonix_sdk/TextureType.msg)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TextureType {
    pub r#type: u8, // WOOD=0, PLASTIC=1, METAL=2, UNKNOWN=3
}

impl ros2_client::Message for TextureType {}

/// FrameMapping message (robonix_sdk/FrameMapping.msg)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FrameMapping {
    pub center: Point3D,
    pub bbox: Vec<BoundingBox>,
    pub texture: Vec<TextureType>,
    pub frame_id: String,
}

impl ros2_client::Message for FrameMapping {}

/// Object message (robonix_sdk/Object.msg)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Object {
    pub id: String,
    pub label: String,
    pub relations: Vec<Relation>,
    pub registered_skills: Vec<String>,
    pub registered_primitives: Vec<String>,
    pub frame_mapping: Vec<FrameMapping>,
}

impl ros2_client::Message for Object {}
