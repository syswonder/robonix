// Copyright 2020 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>
#include <limits>

#include "four_wheel_steering_controller/four_wheel_steering_controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "rcppmath/clamp.hpp"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";

constexpr auto POSITIVE_ZERO = 0.000001;
}  // namespace

namespace four_wheel_steering_controller
{
using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

inline double normalize(double z)
{
  return atan2(sin(z), cos(z));
}

FourWheelSteeringController::FourWheelSteeringController()
: controller_interface::ControllerInterface(), 
  command_struct_twist_{},
  open_loop_(true),
  track_(0.0), 
  wheel_radius_(0.0), 
  wheel_base_(0.0),   
  wheel_steering_y_offset_(0.0), 
  cmd_vel_timeout_(0.5), 
  base_frame_id_("base_link"), 
  odom_frame_id_("odom"), 
  enable_odom_tf_(true)
{
}

InterfaceConfiguration FourWheelSteeringController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (auto joint_name : {
      front_left_wheel_,
      front_right_wheel_,
      rear_left_wheel_,
      rear_right_wheel_
    })
  {
    std::stringstream ss;
    ss << joint_name << "/" << HW_IF_VELOCITY;
    conf_names.push_back(ss.str());
  }
  for (auto joint_name : {
      front_left_steering_,
      front_right_steering_,
      rear_left_steering_,
      rear_right_steering_
    })
  {
    std::stringstream ss;
    ss << joint_name << "/" << HW_IF_POSITION;
    conf_names.push_back(ss.str());
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration FourWheelSteeringController::state_interface_configuration() const
{
  std::vector<std::string> conf_names;
  const auto & motor_names = {
      front_left_wheel_,
      front_right_wheel_,
      rear_left_wheel_,
      rear_right_wheel_,
  };
  const auto & steering_names = {
      front_left_steering_,
      front_right_steering_,
      rear_left_steering_,
      rear_right_steering_
  };
  for (const auto & joint_name : motor_names) {
    std::stringstream ss;
    ss << joint_name << "/" << HW_IF_VELOCITY;
    conf_names.push_back(ss.str());
  }
  for (const auto & joint_name : steering_names) {
    std::stringstream ss;
    ss << joint_name << "/" << HW_IF_POSITION;
    conf_names.push_back(ss.str());
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type FourWheelSteeringController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  return ((controller_interface::return_type::OK == updateOdometry(time, period)) 
      && (controller_interface::return_type::OK == updateCommand(time, period)))
      ? controller_interface::return_type::OK : controller_interface::return_type::ERROR;
}

CallbackReturn FourWheelSteeringController::on_init()
{
  try
  {
    auto_declare<double>("publish_rate", 50.0);
    auto_declare<bool>("open_loop", true);
    auto_declare<int>("velocity_rolling_window_size", 10);
    auto_declare<double>("cmd_vel_timeout", 0.5);
    auto_declare<std::string>("odom_frame_id", "odom");
    auto_declare<std::string>("base_frame_id", "base_footprint");
    auto_declare<bool>("enable_odom_tf", true);
    auto_declare<bool>("wait_for_angle", true);
    auto_declare<bool>("stop_no_adjust_steering", true);
    auto_declare<double>("min_steering_diff", 0.05);

    auto_declare<std::string>("front_left_wheel", "fl_wheel_joint");
    auto_declare<std::string>("front_right_wheel", "fr_wheel_joint");
    auto_declare<std::string>("rear_left_wheel", "rl_wheel_joint");
    auto_declare<std::string>("rear_right_wheel", "rr_wheel_joint");
    auto_declare<std::string>("front_left_steering", "fl_steering_joint");
    auto_declare<std::string>("front_right_steering", "fr_steering_joint");
    auto_declare<std::string>("rear_left_steering", "rl_steering_joint");
    auto_declare<std::string>("rear_right_steering", "rr_steering_joint");
    auto_declare<std::string>("chassis_type", "all");

    auto_declare<double>("track", 0.423065);
    auto_declare<double>("wheel_radius", 0.1);
    auto_declare<double>("wheel_base", 0.490102);
    auto_declare<double>("wheel_steering_y_offset", 0.0316228);

    auto_declare<bool>("linear/x/has_velocity_limits", false);
    auto_declare<bool>("linear/x/has_acceleration_limits", false);
    auto_declare<double>("linear/x/max_velocity", NAN);
    auto_declare<double>("linear/x/min_velocity", NAN);
    auto_declare<double>("linear/x/max_acceleration", NAN);
    auto_declare<double>("linear/x/min_acceleration", NAN);

    auto_declare<bool>("angular/z/has_velocity_limits", false);
    auto_declare<bool>("angular/z/has_acceleration_limits", false);
    auto_declare<double>("angular/z/max_velocity", NAN);
    auto_declare<double>("angular/z/min_velocity", NAN);
    auto_declare<double>("angular/z/max_acceleration", NAN);
    auto_declare<double>("angular/z/min_acceleration", NAN);
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn FourWheelSteeringController::on_configure(const rclcpp_lifecycle::State &)
{
  // limit the publication on the topics /odom and /tf
  publish_rate_ = get_node()->get_parameter("publish_rate").as_double();
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);

  const std::string complete_ns = get_node()->get_namespace();
  std::size_t id = complete_ns.find_last_of("/");
  name_ = complete_ns.substr(id + 1);

  open_loop_ = get_node()->get_parameter("open_loop").as_bool();
  enable_odom_tf_ = get_node()->get_parameter("enable_odom_tf").as_bool();  
  wait_for_angle_ = get_node()->get_parameter("wait_for_angle").as_bool();
  stop_no_adjust_steering_ = get_node()->get_parameter("stop_no_adjust_steering").as_bool();
  min_steering_diff_ = get_node()->get_parameter("min_steering_diff").as_double();
  cmd_vel_timeout_ = get_node()->get_parameter("cmd_vel_timeout").as_double();
  odom_frame_id_ = get_node()->get_parameter("odom_frame_id").as_string();
  base_frame_id_ = get_node()->get_parameter("base_frame_id").as_string();

  front_left_wheel_ = get_node()->get_parameter("front_left_wheel").as_string();
  front_right_wheel_ = get_node()->get_parameter("front_right_wheel").as_string();
  rear_left_wheel_ = get_node()->get_parameter("rear_left_wheel").as_string();
  rear_right_wheel_ = get_node()->get_parameter("rear_right_wheel").as_string();
  front_left_steering_ = get_node()->get_parameter("front_left_steering").as_string();
  front_right_steering_ = get_node()->get_parameter("front_right_steering").as_string();
  rear_left_steering_ = get_node()->get_parameter("rear_left_steering").as_string();
  rear_right_steering_ = get_node()->get_parameter("rear_right_steering").as_string();
  chassis_type_ = get_node()->get_parameter("chassis_type").as_string();
  
  track_ = get_node()->get_parameter("track").as_double();
  wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
  wheel_base_ = get_node()->get_parameter("wheel_base").as_double();
  wheel_steering_y_offset_ = get_node()->get_parameter("wheel_steering_y_offset").as_double();

  limiter_lin_.has_velocity_limits_ = get_node()->get_parameter("linear/x/has_velocity_limits").as_bool();
  limiter_lin_.has_acceleration_limits_ = get_node()->get_parameter("linear/x/has_acceleration_limits").as_bool();
  limiter_lin_.max_velocity_ = get_node()->get_parameter("linear/x/max_velocity").as_double();
  limiter_lin_.min_velocity_ = get_node()->get_parameter("linear/x/min_velocity").as_double();
  limiter_lin_.max_acceleration_ = get_node()->get_parameter("linear/x/max_acceleration").as_double();
  limiter_lin_.min_acceleration_ = get_node()->get_parameter("linear/x/min_acceleration").as_double();

  limiter_ang_.has_velocity_limits_ = get_node()->get_parameter("angular/z/has_velocity_limits").as_bool();
  limiter_ang_.has_acceleration_limits_ = get_node()->get_parameter("angular/z/has_acceleration_limits").as_bool();
  limiter_ang_.max_velocity_ = get_node()->get_parameter("angular/z/max_velocity").as_double();
  limiter_ang_.min_velocity_ = get_node()->get_parameter("angular/z/min_velocity").as_double();
  limiter_ang_.max_acceleration_ = get_node()->get_parameter("angular/z/max_acceleration").as_double();
  limiter_ang_.min_acceleration_ = get_node()->get_parameter("angular/z/min_acceleration").as_double();

  odometry_.setVelocityRollingWindowSize(get_node()->get_parameter("velocity_rolling_window_size").as_int());
  odometry_.setWheelParams(track_-2*wheel_steering_y_offset_, wheel_steering_y_offset_, wheel_radius_, wheel_base_);
  odometry_.setUpdateFunction(chassis_type_);

  last1_cmd_ = std::make_shared<CommandTwist>();
  last1_cmd_->stamp = get_node()->get_clock()->now();

  last0_cmd_ = std::make_shared<CommandTwist>();
  last0_cmd_->stamp = get_node()->get_clock()->now();

  command_struct_twist_.stamp = get_node()->get_clock()->now();
  command_twist_.set(std::make_shared<CommandTwist>(command_struct_twist_));

  sub_command_twist_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
    DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg) -> void
    {
      if (!subscriber_is_active_)
      {
        RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
        return;
      }
      if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
      {
        RCLCPP_WARN_ONCE(
          get_node()->get_logger(),
          "Received TwistStamped with zero timestamp, setting it to current "
          "time, this message will only be shown once");
        msg->header.stamp = get_node()->get_clock()->now();
      }

      command_struct_twist_.stamp = msg->header.stamp;
      command_struct_twist_.lin_x = msg->twist.linear.x;
      command_struct_twist_.lin_y = msg->twist.linear.y;
      command_struct_twist_.ang = msg->twist.angular.z;
      command_twist_.set(std::make_shared<CommandTwist>(command_struct_twist_));
    });

  sub_command_twist_unstamped_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void
        {
          if (!subscriber_is_active_)
          {
            RCLCPP_WARN(
              get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
            return;
          }

          command_struct_twist_.stamp = get_node()->get_clock()->now();
          command_struct_twist_.lin_x = msg->linear.x;
          command_struct_twist_.lin_y = msg->linear.y;
          command_struct_twist_.ang = msg->angular.z;
          command_twist_.set(std::make_shared<CommandTwist>(command_struct_twist_));
        });

  // initialize odometry publisher and messasge
  odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odometry_publisher_);

  std::string controller_namespace = std::string(get_node()->get_namespace());

  if (controller_namespace == "/")
  {
    controller_namespace = "";
  }
  else
  {
    controller_namespace = controller_namespace.erase(0, 1) + "/";
  }

  const auto odom_frame_id = controller_namespace + odom_frame_id_;
  const auto base_frame_id = controller_namespace + base_frame_id_;

  auto & odometry_message = realtime_odometry_publisher_->msg_;
  odometry_message.header.frame_id = odom_frame_id;
  odometry_message.child_frame_id = base_frame_id;

  // initialize odom values zeros
  odometry_message.twist = geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);
  
  double pose_covariance_diagonal[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double twist_covariance_diagonal[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index)
  {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    odometry_message.pose.covariance[diagonal_index] = pose_covariance_diagonal[index];
    odometry_message.twist.covariance[diagonal_index] = twist_covariance_diagonal[index];
  }

  // initialize transform publisher and message
  odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_transform_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(odometry_transform_publisher_);
  
  // keeping track of odom and base_link transforms only
  auto & odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
  odometry_transform_message.transforms.resize(1);
  odometry_transform_message.transforms.front().header.frame_id = odom_frame_id;
  odometry_transform_message.transforms.front().child_frame_id = base_frame_id;

  previous_update_timestamp_ = get_node()->get_clock()->now();
  return CallbackReturn::SUCCESS;
}

CallbackReturn FourWheelSteeringController::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "On activate: Initialize Joints");

  // Initialize the joints
  const auto front_left_traction_result = get_traction(front_left_wheel_, front_left_traction_handle_);
  const auto front_right_traction_result = get_traction(front_right_wheel_, front_right_traction_handle_);
  const auto rear_left_traction_result = get_traction(rear_left_wheel_, rear_left_traction_handle_);
  const auto rear_right_traction_result = get_traction(rear_right_wheel_, rear_right_traction_handle_);  
  const auto front_left_steering_result = get_steering(front_left_steering_, front_left_steering_handle_);
  const auto front_right_steering_result = get_steering(front_right_steering_, front_right_steering_handle_);
  const auto rear_left_steering_result = get_steering(rear_left_steering_, rear_left_steering_handle_);
  const auto rear_right_steering_result = get_steering(rear_right_steering_, rear_right_steering_handle_);

  if (front_left_traction_result == CallbackReturn::ERROR 
  || front_right_traction_result == CallbackReturn::ERROR
  || rear_left_traction_result == CallbackReturn::ERROR
  || rear_right_traction_result == CallbackReturn::ERROR
  || front_left_steering_result == CallbackReturn::ERROR
  || front_right_steering_result == CallbackReturn::ERROR
  || rear_left_steering_result == CallbackReturn::ERROR
  || rear_right_steering_result == CallbackReturn::ERROR)
  {
    return CallbackReturn::ERROR;
  }

  if (front_left_traction_handle_ == nullptr 
  || front_right_traction_handle_ == nullptr
  || rear_left_traction_handle_ == nullptr
  || rear_right_traction_handle_ == nullptr
  || front_left_steering_handle_ == nullptr
  || front_right_steering_handle_ == nullptr
  || rear_left_steering_handle_ == nullptr
  || rear_right_steering_handle_ == nullptr)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Either steering or traction interfaces are non existent");
    return CallbackReturn::ERROR;
  }

  is_halted = false;
  subscriber_is_active_ = true;

  odometry_.init(get_node()->get_clock()->now());

  RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn FourWheelSteeringController::on_deactivate(const rclcpp_lifecycle::State &)
{
  subscriber_is_active_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn FourWheelSteeringController::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return CallbackReturn::ERROR;
  }

  command_twist_.set(std::make_shared<CommandTwist>());
  return CallbackReturn::SUCCESS;
}

CallbackReturn FourWheelSteeringController::on_error(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn FourWheelSteeringController::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

bool FourWheelSteeringController::reset()
{
  front_left_traction_handle_.reset();
  front_right_traction_handle_.reset();
  rear_left_traction_handle_.reset();
  rear_right_traction_handle_.reset();
  front_left_steering_handle_.reset();
  front_right_steering_handle_.reset();
  rear_left_steering_handle_.reset();
  rear_right_steering_handle_.reset();

  subscriber_is_active_ = false;  
  sub_command_twist_.reset();
  sub_command_twist_unstamped_.reset();

  command_twist_.set(nullptr);
  is_halted = false;
  return true;
}

void FourWheelSteeringController::halt()
{
  front_left_traction_handle_->velocity_command.get().set_value(0.0);
  front_right_traction_handle_->velocity_command.get().set_value(0.0);
  rear_left_traction_handle_->velocity_command.get().set_value(0.0);
  rear_right_traction_handle_->velocity_command.get().set_value(0.0);
  front_left_steering_handle_->position_command.get().set_value(0.0);
  front_right_steering_handle_->position_command.get().set_value(0.0);
  rear_left_steering_handle_->position_command.get().set_value(0.0);
  rear_right_steering_handle_->position_command.get().set_value(0.0);

  is_halted = true;
}

CallbackReturn FourWheelSteeringController::get_traction(const std::string & traction_joint_name, std::unique_ptr<TractionHandle> & joint)
{
  RCLCPP_INFO(get_node()->get_logger(), "Get Wheel Joint Instance");

  // Lookup the velocity state interface
  const auto state_handle = std::find_if(
    state_interfaces_.cbegin(), state_interfaces_.cend(),
    [&traction_joint_name](const auto & interface)
    {
      return interface.get_prefix_name() == traction_joint_name &&
             interface.get_interface_name() == HW_IF_VELOCITY;
    });
  if (state_handle == state_interfaces_.cend())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain joint state handle for %s", traction_joint_name.c_str());
    return CallbackReturn::ERROR;
  }

  // Lookup the velocity command interface
  const auto command_handle = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(),
    [&traction_joint_name](const auto & interface)
    {
      return interface.get_prefix_name() == traction_joint_name &&
             interface.get_interface_name() == HW_IF_VELOCITY;
    });
  if (command_handle == command_interfaces_.end())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain joint state handle for %s", traction_joint_name.c_str());
    return CallbackReturn::ERROR;
  }

  // Create the traction joint instance
   joint = std::unique_ptr<TractionHandle> {new TractionHandle{std::ref(*state_handle), std::ref(*command_handle)}};
  return CallbackReturn::SUCCESS;
}

CallbackReturn FourWheelSteeringController::get_steering(const std::string & steering_joint_name, std::unique_ptr<SteeringHandle> & joint)
{
  RCLCPP_INFO(get_node()->get_logger(), "Get Steering Joint Instance");

  // Lookup the velocity state interface
  const auto state_handle = std::find_if(
    state_interfaces_.cbegin(), state_interfaces_.cend(),
    [&steering_joint_name](const auto & interface)
    {
      return interface.get_prefix_name() == steering_joint_name &&
             interface.get_interface_name() == HW_IF_POSITION;
    });
  if (state_handle == state_interfaces_.cend())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain joint state handle for %s", steering_joint_name.c_str());
    return CallbackReturn::ERROR;
  }

  // Lookup the velocity command interface
  const auto command_handle = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(),
    [&steering_joint_name](const auto & interface)
    {
      return interface.get_prefix_name() == steering_joint_name &&
             interface.get_interface_name() == HW_IF_POSITION;
    });
  if (command_handle == command_interfaces_.end())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain joint state handle for %s", steering_joint_name.c_str());
    return CallbackReturn::ERROR;
  }

  // Create the steering joint instance  
  joint = std::unique_ptr<SteeringHandle> {new SteeringHandle{std::ref(*state_handle), std::ref(*command_handle)}};
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type FourWheelSteeringController::updateOdometry(const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // COMPUTE AND PUBLISH ODOMETRY
  double fl_speed = front_left_traction_handle_->velocity_state.get().get_value();    // in radians/s
  double fr_speed = front_right_traction_handle_->velocity_state.get().get_value();   // in radians/s
  double rl_speed = rear_left_traction_handle_->velocity_state.get().get_value();     // in radians/s
  double rr_speed = rear_right_traction_handle_->velocity_state.get().get_value();    // in radians/s

  if (std::isnan(fl_speed) || std::isnan(fr_speed)
   || std::isnan(rl_speed) || std::isnan(rr_speed))
  {
    return controller_interface::return_type::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "odom fl_speed: %.3f, fr_speed: %.3f, rl_speed: %f, rr_speed: %.3f", fl_speed, fr_speed, rl_speed, rr_speed);

  double fl_steering = front_left_steering_handle_->position_state.get().get_value(); // in radians
  double fr_steering = front_right_steering_handle_->position_state.get().get_value();// in radians
  double rl_steering = rear_left_steering_handle_->position_state.get().get_value();  // in radians
  double rr_steering = rear_right_steering_handle_->position_state.get().get_value(); // in radians

  if (std::isnan(fl_steering) || std::isnan(fr_steering)
   || std::isnan(rl_steering) || std::isnan(rr_steering))
  {
    return controller_interface::return_type::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "odom fl_steering: %.3f, fr_steering: %.3f, rl_steering: %.3f, rr_steering: %.3f", fl_steering, fr_steering, rl_steering, rr_steering);

  // Estimate linear and angular velocity using joint information
  odometry_.update(fl_speed, fr_speed, rl_speed, rr_speed, fl_steering, fr_steering, rl_steering, rr_steering, time);

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.getHeading());

  bool should_publish = false;
  try
  {
    if (previous_publish_timestamp_ + publish_period_ < time)
    {
      previous_publish_timestamp_ += publish_period_;
      should_publish = true;
    }
  }
  catch (const std::runtime_error &)
  {
    // Handle exceptions when the time source changes and initialize publish timestamp
    previous_publish_timestamp_ = time;
    should_publish = true;
  }

  if (should_publish)
  {
    if (realtime_odometry_publisher_->trylock())
    {
      auto & odometry_message = realtime_odometry_publisher_->msg_;
      odometry_message.header.stamp = time;
      odometry_message.pose.pose.position.x = odometry_.getX();
      odometry_message.pose.pose.position.y = odometry_.getY();
      odometry_message.pose.pose.orientation.x = orientation.x();
      odometry_message.pose.pose.orientation.y = orientation.y();
      odometry_message.pose.pose.orientation.z = orientation.z();
      odometry_message.pose.pose.orientation.w = orientation.w();
      odometry_message.twist.twist.linear.x = odometry_.getLinearX();
      odometry_message.twist.twist.linear.y = odometry_.getLinearY();
      odometry_message.twist.twist.angular.z = odometry_.getAngular();
      realtime_odometry_publisher_->unlockAndPublish();
    }
 
    if (enable_odom_tf_ && realtime_odometry_transform_publisher_->trylock())
    {
      auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
      transform.header.stamp = time;
      transform.transform.translation.x = odometry_.getX();
      transform.transform.translation.y = odometry_.getY();
      transform.transform.rotation.x = orientation.x();
      transform.transform.rotation.y = orientation.y();
      transform.transform.rotation.z = orientation.z();
      transform.transform.rotation.w = orientation.w();
      realtime_odometry_transform_publisher_->unlockAndPublish();
    }
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type FourWheelSteeringController::updateCommand(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Retreive current velocity command and time step:
  std::shared_ptr<CommandTwist> curr_cmd_twist = nullptr; 
  command_twist_.get(curr_cmd_twist);
  curr_cmd_twist = std::make_shared<CommandTwist>(*curr_cmd_twist);
  
  const auto dur = time - curr_cmd_twist->stamp;
  const double dt = dur.seconds();
  
  // Brake if cmd_vel has timeout:
  if (dt > cmd_vel_timeout_)
  {
    curr_cmd_twist->lin_x = 0.0;
    curr_cmd_twist->lin_y = 0.0;
    curr_cmd_twist->ang = 0.0;
  }
  
  RCLCPP_INFO(get_node()->get_logger(), "do cmd_vel(X:%.3f, Y:%.3f, W:%.3f)", curr_cmd_twist->lin_x, curr_cmd_twist->lin_y, curr_cmd_twist->ang);
  
  const double cmd_dt(period.seconds());

  const double angular_speed = odometry_.getAngular();
  const double steering_track = track_ - 2*wheel_steering_y_offset_;
  
  RCLCPP_INFO(get_node()->get_logger(), "angular_speed:%.3f wheel_radius_:%.3f", angular_speed, wheel_radius_);
  
  double vel_left_front = 0, vel_right_front = 0;
  double vel_left_rear = 0, vel_right_rear = 0;
  double front_left_steering = 0, front_right_steering = 0;
  double rear_left_steering = 0, rear_right_steering = 0;
  
  bool stop_cmd = false;
  
  {
    // Limit velocities and accelerations:
    limiter_lin_.limit(curr_cmd_twist->lin_x, last0_cmd_->lin_x, last1_cmd_->lin_x, cmd_dt);
    limiter_ang_.limit(curr_cmd_twist->ang, last0_cmd_->ang, last1_cmd_->ang, cmd_dt);
    last1_cmd_ = last0_cmd_;
    last0_cmd_ = curr_cmd_twist;
	  
  	// 收到速度cmd_vel(0,0,0)，仅停车，不复位舵角
  	if (fabs(curr_cmd_twist->lin_x) < POSITIVE_ZERO 
  	 && fabs(curr_cmd_twist->lin_y) < POSITIVE_ZERO 
  	 && fabs(curr_cmd_twist->ang) < POSITIVE_ZERO)
  	{
      vel_left_front = 0.0;
      vel_right_front = 0.0;
      vel_left_rear = 0.0;
      vel_right_rear = 0.0;
      
      front_left_steering = 0.0;
  		front_right_steering = 0.0;
  		rear_left_steering = 0.0;
  		rear_right_steering = 0.0;

      stop_cmd = true;
  	}
    // 横移 In-phase // All-Wheel Driving (crab motion)  螃蟹横着走
  	else if (fabs(curr_cmd_twist->ang) < POSITIVE_ZERO)
  	{
      // Compute wheels velocities:
      const double sign = copysign(1.0, curr_cmd_twist->lin_x);
      vel_left_front = sign * std::hypot(curr_cmd_twist->lin_x, curr_cmd_twist->lin_y) / wheel_radius_;
      vel_right_front = vel_left_front;
      vel_left_rear = vel_left_front;
      vel_right_rear = vel_right_front;
      
      double steering = (fabs(curr_cmd_twist->lin_x) < POSITIVE_ZERO) 
        ? copysign(M_PI_2, curr_cmd_twist->lin_x*curr_cmd_twist->lin_y)
        : atan(curr_cmd_twist->lin_y / curr_cmd_twist->lin_x);
      
      // Compute steering angles:
      front_left_steering = steering;
      front_right_steering = front_left_steering;
      rear_left_steering = front_left_steering;
      rear_right_steering = front_right_steering;      
  	}
  	// 自旋任务 x=0,y=0,w>0
  	else if (fabs(curr_cmd_twist->lin_x) < POSITIVE_ZERO 
  	 && fabs(curr_cmd_twist->lin_y) < POSITIVE_ZERO 
  	 && fabs(curr_cmd_twist->ang) > POSITIVE_ZERO)
  	{
      front_left_steering = -atan(wheel_base_ / steering_track);
      front_right_steering = atan(wheel_base_ / steering_track);
      rear_left_steering = -front_left_steering;
      rear_right_steering = -front_right_steering;
      
      vel_left_front = -curr_cmd_twist->ang * (std::hypot(wheel_base_/2, steering_track/2)+wheel_steering_y_offset_) / wheel_radius_;
      vel_right_front = curr_cmd_twist->ang * (std::hypot(wheel_base_/2, steering_track/2)+wheel_steering_y_offset_) / wheel_radius_;
      vel_left_rear = vel_left_front;
      vel_right_rear = vel_right_front;      
    }
    else // fabs(curr_cmd_twist->ang) > POSITIVE_ZERO
    {
      // 车体坐标系下的瞬时旋转中心ICR
      double ICR_Y = curr_cmd_twist->lin_x / curr_cmd_twist->ang;
      double ICR_X = -curr_cmd_twist->lin_y / curr_cmd_twist->ang;
      
      vel_left_front = curr_cmd_twist->ang * std::hypot(steering_track/2-ICR_Y, wheel_base_/2.0-ICR_X) / wheel_radius_;
      vel_right_front = curr_cmd_twist->ang * std::hypot(-steering_track/2-ICR_Y, wheel_base_/2.0-ICR_X) / wheel_radius_;
      vel_left_rear = curr_cmd_twist->ang * std::hypot(steering_track/2-ICR_Y, -wheel_base_/2.0-ICR_X) / wheel_radius_;
      vel_right_rear = curr_cmd_twist->ang * std::hypot(-steering_track/2-ICR_Y, -wheel_base_/2.0-ICR_X) / wheel_radius_;
      
      front_left_steering = normalize(atan2(steering_track/2-ICR_Y, wheel_base_/2.0-ICR_X) + M_PI_2);
      front_right_steering = normalize(atan2(-steering_track/2-ICR_Y, wheel_base_/2.0-ICR_X) + M_PI_2);
      rear_left_steering = normalize(atan2(steering_track/2-ICR_Y, -wheel_base_/2.0-ICR_X) + M_PI_2);      
      rear_right_steering = normalize(atan2(-steering_track/2-ICR_Y, -wheel_base_/2.0-ICR_X) + M_PI_2);
    }
    
    // 判断舵角是否到位，并输出控制量
    if (wait_for_angle_)
    {
      double fl_steering = front_left_steering_handle_->position_state.get().get_value(); // in radians
      double fr_steering = front_right_steering_handle_->position_state.get().get_value();// in radians
      double rl_steering = rear_left_steering_handle_->position_state.get().get_value();  // in radians
      double rr_steering = rear_right_steering_handle_->position_state.get().get_value(); // in radians
      
      if ((fabs(front_left_steering-fl_steering)>min_steering_diff_)
        ||(fabs(front_right_steering-fr_steering)>min_steering_diff_)
        ||(fabs(rear_left_steering-rl_steering)>min_steering_diff_)
        ||(fabs(rear_right_steering-rr_steering)>min_steering_diff_))
      {
        vel_left_front = 0.0;
        vel_right_front = 0.0;
        vel_left_rear = 0.0;
        vel_right_rear = 0.0;
      }
    }
  }
  
  RCLCPP_INFO(get_node()->get_logger(), "cmd_vel velocity fl: %.3f, fr: %.3f, rl: %.3f, rr: %.3f", vel_left_front, vel_right_front, vel_left_rear, vel_right_rear);
  RCLCPP_INFO(get_node()->get_logger(), "cmd_vel steering fl: %.3f, fr: %.3f, rl: %.3f, rr: %.3f", front_left_steering, front_right_steering, rear_left_steering, rear_right_steering);
  
  front_left_traction_handle_->velocity_command.get().set_value(vel_left_front);
  front_right_traction_handle_->velocity_command.get().set_value(vel_right_front);
  rear_left_traction_handle_->velocity_command.get().set_value(vel_left_rear);
  rear_right_traction_handle_->velocity_command.get().set_value(vel_right_rear);
  
  if (stop_no_adjust_steering_ && stop_cmd)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Receive a stop cmd without adjusting the steering wheel. stop_no_adjust_steering_: %d, stop_cmd: %d", stop_no_adjust_steering_, stop_cmd);
  }
  else  
  {
    front_left_steering_handle_->position_command.get().set_value(front_left_steering);
    front_right_steering_handle_->position_command.get().set_value(front_right_steering);
    rear_left_steering_handle_->position_command.get().set_value(rear_left_steering);
    rear_right_steering_handle_->position_command.get().set_value(rear_right_steering);
  }

  return controller_interface::return_type::OK;
}

}  // namespace four_wheel_steering_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  four_wheel_steering_controller::FourWheelSteeringController,
  controller_interface::ControllerInterface)
