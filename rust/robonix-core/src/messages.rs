use ros2_client::Message;
use serde::{Deserialize, Serialize};

// Input/Output parameter specification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IOParameter {
    pub name: String,
    pub ros_type: String, // ROS message type (e.g., "geometry_msgs/msg/PoseStamped")
    pub channel: String,  // Topic or service name (e.g., "/piper/pose_goal")
}

// Configuration service specification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConfigService {
    pub service: String, // Service name (e.g., "/arm/configure")
    pub name: String,    // Configuration parameter name (e.g., "piper_arm_config_update")
}

// Registration service types based on srv/register.srv
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegisterRequest {
    pub provider_name: String,
    #[serde(rename = "type")]
    pub provider_type: String, // "cap" or "skl"
    pub std_name: String,
    pub description: String,
    pub code_path: String,
    pub input_names: Vec<String>,
    pub input_ros_types: Vec<String>,
    pub input_channels: Vec<String>,
    pub output_names: Vec<String>,
    pub output_ros_types: Vec<String>,
    pub output_channels: Vec<String>,
    pub config_services: Vec<String>,
    pub config_names: Vec<String>,
    pub dependencies: Vec<String>, // Required capabilities (for skills only)
}
impl Message for RegisterRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegisterResponse {
    pub success: bool,
    pub error_message: String,
}
impl Message for RegisterResponse {}

// Capability registration data
#[derive(Debug, Clone)]
pub struct Capability {
    pub provider_name: String,
    pub std_name: String,
    pub description: String,
    pub code_path: String,
    pub inputs: Vec<IOParameter>,
    pub outputs: Vec<IOParameter>,
    pub configs: Vec<ConfigService>,
}

// Skill registration data
#[derive(Debug, Clone)]
pub struct Skill {
    pub provider_name: String,
    pub std_name: String,
    pub description: String,
    pub code_path: String,
    pub inputs: Vec<IOParameter>,
    pub outputs: Vec<IOParameter>,
    pub configs: Vec<ConfigService>,
    pub dependencies: Vec<String>, // Required capabilities
}
