use ros2_client::Message;
use serde::{Deserialize, Serialize};

// Registration service types based on srv/register.srv
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegisterRequest {
    pub provider_name: String,
    #[serde(rename = "type")]
    pub provider_type: String, // "cap" or "skl"
    pub std_name: String,
    pub description: String,
    pub input_topics: Vec<String>,
    pub output_topics: Vec<String>,
}
impl Message for RegisterRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegisterResponse {
    pub success: bool,
}
impl Message for RegisterResponse {}

// Capability registration data
#[derive(Debug, Clone)]
pub struct Capability {
    pub provider_name: String,
    pub std_name: String,
    pub description: String,
    pub input_topics: Vec<String>,
    pub output_topics: Vec<String>,
}

// Skill registration data
#[derive(Debug, Clone)]
pub struct Skill {
    pub provider_name: String,
    pub std_name: String,
    pub description: String,
    pub input_topics: Vec<String>,
    pub output_topics: Vec<String>,
}
