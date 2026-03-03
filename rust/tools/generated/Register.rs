// # Register.srv

#[derive(Debug, Serialize, Deserialize)]
pub struct Register {
  pub provider_name : String, 
  pub provider_type : String, 
  pub std_name : String, 
  pub description : String, 
  pub code_path : String, 
  pub input_names : Vec<String>, 
  pub input_ros_types : Vec<String>, 
  pub input_channels : Vec<String>, 
  pub output_names : Vec<String>, 
  pub output_ros_types : Vec<String>, 
  pub output_channels : Vec<String>, 
  pub config_services : Vec<String>, 
  pub config_names : Vec<String>, 
  pub dependencies : Vec<String>, 
}
