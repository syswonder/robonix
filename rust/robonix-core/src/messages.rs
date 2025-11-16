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

// Registration service types for capabilities and skills (based on srv/RegisterCapSkl.srv)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegisterCapSklRequest {
    pub package_name: String,
    pub package_type: String, // "cap" or "skl"
    pub std_name: String,
    pub impl_id: String, // Implementation ID (e.g., "algo01", "algo02"). If empty, defaults to "default"
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
    pub hostname: String,
    pub entity_name: String,
}
impl Message for RegisterCapSklRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegisterCapSklResponse {
    pub success: bool,
    pub error_message: String,
}
impl Message for RegisterCapSklResponse {}

// Backward compatibility aliases
pub type RegisterRequest = RegisterCapSklRequest;
pub type RegisterResponse = RegisterCapSklResponse;

// Capability registration data
#[derive(Debug, Clone)]
pub struct Capability {
    pub package_name: String,
    pub std_name: String,
    pub impl_id: String, // Implementation ID
    pub description: String,
    pub code_path: String,
    pub inputs: Vec<IOParameter>,
    pub outputs: Vec<IOParameter>,
    pub configs: Vec<ConfigService>,
}

// Skill registration data
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

// Query service types for capabilities and skills (based on srv/QueryCapSkl.srv)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QueryCapSklRequest {
    pub std_name: String,          // Standard name (e.g., "cap::vision.capture_rgb")
    pub impl_id: String,           // Implementation ID (optional). If empty, returns first match
    pub requirements: Vec<String>, // Optional requirements/filters
}
impl Message for QueryCapSklRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QueryCapSklResponse {
    pub success: bool,
    pub error_message: String,
    pub impl_id: String,           // Implementation ID of the returned capability/skill
    pub impl_ids: Vec<String>,      // All available implementation IDs for this std_name (if impl_id was empty)
    pub input_channels: Vec<String>,  // Input topic channels
    pub output_channels: Vec<String>, // Output topic channels
    pub input_names: Vec<String>,     // Input parameter names
    pub output_names: Vec<String>,    // Output parameter names
    pub input_types: Vec<String>,     // Input ROS message types
    pub output_types: Vec<String>,    // Output ROS message types
}
impl Message for QueryCapSklResponse {}

// Backward compatibility aliases
pub type QueryRequest = QueryCapSklRequest;
pub type QueryResponse = QueryCapSklResponse;

// Model type
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ModelType {
    LLM, // Large Language Model (text only)
    VLM, // Vision Language Model (text + vision)
}

// Model registration data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Model {
    pub model_id: String,
    pub model_name: String,
    pub model_type: ModelType,  // LLM or VLM
    pub provider: String,       // e.g., "openai", "anthropic", "local"
    pub api_endpoint: String,   // API endpoint URL
    pub api_key: Option<String>, // Optional API key
    pub description: String,
    pub capabilities: Vec<String>, // Supported capabilities
}

// Model registration request/response (based on srv/RegisterLLM.srv)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegisterModelRequest {
    pub model_id: String,
    pub model_name: String,
    pub model_type: ModelType, // LLM or VLM
    pub provider: String,
    pub api_endpoint: String,
    pub api_key: Option<String>,
    pub description: String,
    pub capabilities: Vec<String>,
}
impl Message for RegisterModelRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegisterModelResponse {
    pub success: bool,
    pub error_message: String,
}
impl Message for RegisterModelResponse {}

// Query model request/response (based on srv/QueryLLM.srv)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QueryModelRequest {
    pub model_id: Option<String>,   // If None, query all models
    pub model_type: Option<ModelType>, // Filter by model type (LLM or VLM)
    pub capability: Option<String>,  // Filter by capability
}
impl Message for QueryModelRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QueryModelResponse {
    pub success: bool,
    pub error_message: String,
    pub models: Vec<Model>,
}
impl Message for QueryModelResponse {}

// Ping service for testing concurrent service calls
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PingRequest {
    pub sequence: u32, // Sequence number for tracking
}
impl Message for PingRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PingResponse {
    pub success: bool,
    pub sequence: u32, // Echo back the sequence number
    pub timestamp: u64, // Server timestamp in nanoseconds
}
impl Message for PingResponse {}