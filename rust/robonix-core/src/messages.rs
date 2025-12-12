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

// Skill registration data (legacy format, used for compatibility in skill_library)
#[derive(Debug, Clone)]
pub struct Skill {
    pub package_name: String,
    pub std_name: String,
    pub impl_id: String, // Implementation ID
    pub description: String,
    pub code_path: String,
    pub inputs: Vec<IOParameter>,
    pub outputs: Vec<IOParameter>,
    pub configs: Vec<ConfigService>,
}
