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

#ifndef FOUR_WHEEL_STEERING_CONTROLLER__FOUR_WHEEL_STEERING_CONTROLLER_HPP_
#define FOUR_WHEEL_STEERING_CONTROLLER__FOUR_WHEEL_STEERING_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "four_wheel_steering_controller/odometry.hpp"
#include "four_wheel_steering_controller/speed_limiter.hpp"
#include "four_wheel_steering_controller/visibility_control.h"

namespace four_wheel_steering_controller
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class FourWheelSteeringController : public controller_interface::ControllerInterface
{
public:
  FOUR_WHEEL_STEERING_CONTROLLER_PUBLIC
  FourWheelSteeringController();
  
  FOUR_WHEEL_STEERING_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  FOUR_WHEEL_STEERING_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  FOUR_WHEEL_STEERING_CONTROLLER_PUBLIC
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  FOUR_WHEEL_STEERING_CONTROLLER_PUBLIC
  CallbackReturn on_init() override;

  FOUR_WHEEL_STEERING_CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  FOUR_WHEEL_STEERING_CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  FOUR_WHEEL_STEERING_CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  FOUR_WHEEL_STEERING_CONTROLLER_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  FOUR_WHEEL_STEERING_CONTROLLER_PUBLIC
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;
  
  FOUR_WHEEL_STEERING_CONTROLLER_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  
protected:
  struct TractionHandle
  {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_command;
  };
  struct SteeringHandle
  {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_command;
  };
  
  CallbackReturn get_traction(const std::string & traction_joint_name, std::unique_ptr<TractionHandle> & joint);
  CallbackReturn get_steering(const std::string & steering_joint_name, std::unique_ptr<SteeringHandle> & joint);

  std::unique_ptr<TractionHandle> front_left_traction_handle_;
  std::unique_ptr<TractionHandle> front_right_traction_handle_;
  std::unique_ptr<TractionHandle> rear_left_traction_handle_;
  std::unique_ptr<TractionHandle> rear_right_traction_handle_;
  std::unique_ptr<SteeringHandle> front_left_steering_handle_;
  std::unique_ptr<SteeringHandle> front_right_steering_handle_;
  std::unique_ptr<SteeringHandle> rear_left_steering_handle_;
  std::unique_ptr<SteeringHandle> rear_right_steering_handle_;

  std::string front_left_wheel_;
  std::string front_right_wheel_;
  std::string rear_left_wheel_;
  std::string rear_right_wheel_;
  std::string front_left_steering_;
  std::string front_right_steering_;
  std::string rear_left_steering_;
  std::string rear_right_steering_;
  std::string chassis_type_;

  /// Velocity command related:
  struct CommandTwist
  {
    rclcpp::Time stamp;

    double lin_x;
    double lin_y;
    double ang;

    CommandTwist() : stamp(0.0), lin_x(0.0), lin_y(0.0), ang(0.0) {}
  };
  
  bool subscriber_is_active_ = false;
  
  /// Twist command related:
  realtime_tools::RealtimeBox<std::shared_ptr<CommandTwist>> command_twist_{nullptr};
  CommandTwist command_struct_twist_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_command_twist_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_command_twist_unstamped_ = nullptr;
  
  /// Odometry related:
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> realtime_odometry_publisher_ = nullptr;
  
  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> realtime_odometry_transform_publisher_ = nullptr;
  
  Odometry odometry_;

  rclcpp::Time previous_update_timestamp_{0};

  // publish rate limiter
  double publish_rate_ = 50.0;
  rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
  rclcpp::Time previous_publish_timestamp_{0, 0, RCL_CLOCK_UNINITIALIZED};
  rclcpp::Time last_state_publish_time_{0, 0, RCL_CLOCK_UNINITIALIZED};

  std::string name_;
  bool open_loop_;

  double track_; /// Wheel separation (or track), distance between left and right wheels (from the midpoint of the wheel width)
  double wheel_radius_; /// Wheel radius (assuming it's the same for the left and right wheels)  
  double wheel_base_; /// Wheel base (distance between front and rear wheel)  
  double wheel_steering_y_offset_; /// Distance between a wheel joint (from the midpoint of the wheel width) and the associated steering joint: We consider that the distance is the same for every wheel
  double cmd_vel_timeout_; /// Timeout to consider cmd_vel commands old  
  std::string base_frame_id_; /// Frame to use for the robot base  
  std::string odom_frame_id_;
  bool enable_odom_tf_; /// Whether to publish odometry to tf or not  
  bool wait_for_angle_;
  double min_steering_diff_;
  bool stop_no_adjust_steering_;
  rclcpp::Time steering_wait_start_time_{0, 0, RCL_CLOCK_UNINITIALIZED};
  bool steering_waiting_ = false;

  /// Speed limiters:
  std::shared_ptr<CommandTwist> last1_cmd_;
  std::shared_ptr<CommandTwist> last0_cmd_;
  SpeedLimiter limiter_lin_;
  SpeedLimiter limiter_ang_;
  
  bool is_halted = false;
  bool use_stamped_vel_ = true;
  
  bool reset();
  void halt();

  controller_interface::return_type updateOdometry(const rclcpp::Time & time, const rclcpp::Duration & period);
  controller_interface::return_type updateCommand(const rclcpp::Time & time, const rclcpp::Duration & period);
};

}  // namespace four_wheel_steering_controller

#endif  // FOUR_WHEEL_STEERING_CONTROLLER__FOUR_WHEEL_STEERING_CONTROLLER_HPP_
