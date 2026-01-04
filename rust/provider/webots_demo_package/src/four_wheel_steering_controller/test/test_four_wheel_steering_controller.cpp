// Copyright 2021 Austrian Institute of Technology GmbH
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
#include <gmock/gmock.h>

#include <array>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <utility>

#include "four_wheel_steering_controller/four_wheel_steering_controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"


using CallbackReturn = four_wheel_steering_controller::CallbackReturn;
using rclcpp::Time;
using rclcpp::Duration;
using lifecycle_msgs::msg::State;
using four_wheel_steering_msgs::msg::FourWheelSteeringStamped;
using hardware_interface::LoanedStateInterface;
using hardware_interface::LoanedCommandInterface;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using testing::SizeIs;


class TestableFourWheelSteeringController : public four_wheel_steering_controller::
  FourWheelSteeringController
{
public:
  FourWheelSteeringStamped::SharedPtr getLastReceivedCmd()
  {
    FourWheelSteeringStamped::SharedPtr cmd;
    received_cmd_msg_ptr_.get(cmd);
    return cmd;
  }

  /**
  * @brief wait_for_cmd blocks until a new command message is received.
  * Requires that the executor is not spinned elsewhere between the
  *  message publication and the call to this function
  *
  * @return true if new cmd msg was received, false if timeout occurred
  */
  bool wait_for_cmd(
    rclcpp::Executor & executor,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds(500))
  {
    rclcpp::WaitSet wait_set;
    wait_set.add_subscription(sub_cmd_);
    if (wait_set.wait(timeout).kind() == rclcpp::WaitResultKind::Ready) {
      executor.spin_some();
      return true;
    }
    return false;
  }
};

class TestFourWheelSteeringController : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp() override
  {
    controller_ = std::make_unique<TestableFourWheelSteeringController>();

    pub_node = std::make_shared<rclcpp::Node>("cmd_publisher");
    cmd_publisher = pub_node->create_publisher<FourWheelSteeringStamped>(
      "/cmd_4ws",
      rclcpp::SystemDefaultsQoS());
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void send_cmd(double speed, double front_steering_angle)
  {
    int wait_count = 0;
    auto topic = cmd_publisher->get_topic_name();
    while (pub_node->count_subscribers(topic) == 0) {
      if (wait_count >= 5) {
        auto error_msg = std::string("publishing to ") + topic + " but can not find subscriber";
        throw std::runtime_error(error_msg);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      ++wait_count;
    }

    FourWheelSteeringStamped cmd;
    cmd.header.stamp = pub_node->get_clock()->now();
    cmd.data.speed = speed;
    cmd.data.front_steering_angle = front_steering_angle;
    cmd.data.rear_steering_angle = -front_steering_angle;
    cmd_publisher->publish(cmd);
  }

  void waitForSetup()
  {
    constexpr std::chrono::seconds TIMEOUT{2};
    auto clock = pub_node->get_clock();
    auto start = clock->now();
    while (cmd_publisher->get_subscription_count() <= 0) {
      if ((clock->now() - start) > TIMEOUT) {
        FAIL();
      }
      rclcpp::spin_some(pub_node);
    }
  }

  void assignResources()
  {
    std::vector<LoanedStateInterface> state_ifs;
    state_ifs.emplace_back(front_left_motor_vel_state_);
    state_ifs.emplace_back(front_right_motor_vel_state_);
    state_ifs.emplace_back(rear_left_motor_vel_state_);
    state_ifs.emplace_back(rear_right_motor_vel_state_);

    state_ifs.emplace_back(front_left_steering_state_);
    state_ifs.emplace_back(front_right_steering_state_);
    state_ifs.emplace_back(rear_left_steering_state_);
    state_ifs.emplace_back(rear_right_steering_state_);

    std::vector<LoanedCommandInterface> command_ifs;
    command_ifs.emplace_back(front_left_motor_vel_cmd_);
    command_ifs.emplace_back(front_right_motor_vel_cmd_);
    command_ifs.emplace_back(rear_left_motor_vel_cmd_);
    command_ifs.emplace_back(rear_right_motor_vel_cmd_);
    command_ifs.emplace_back(front_steering_cmd_);
    command_ifs.emplace_back(rear_steering_cmd_);

    controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
  }

  const std::string controller_name = "test_four_wheel_steering_controller";
  std::unique_ptr<TestableFourWheelSteeringController> controller_;

  std::vector<double> motor_state_ = {1.0, 1.0, 1.0, 1.0};
  std::vector<double> motor_cmd_ = {0.1, 0.1, 0.1, 0.1};
  std::vector<double> kingpin_state_ = {0.0, 0.0, 0.0, 0.0};
  std::vector<double> steering_cmd_ = {0.0, 0.0};

  hardware_interface::StateInterface front_left_motor_vel_state_{"front_left_motor", HW_IF_VELOCITY,
    &motor_state_[0]};
  hardware_interface::StateInterface front_right_motor_vel_state_{"front_right_motor",
    HW_IF_VELOCITY,
    &motor_state_[1]};
  hardware_interface::StateInterface rear_left_motor_vel_state_{"rear_left_motor", HW_IF_VELOCITY,
    &motor_state_[2]};
  hardware_interface::StateInterface rear_right_motor_vel_state_{"rear_right_motor", HW_IF_VELOCITY,
    &motor_state_[3]};

  hardware_interface::StateInterface front_left_steering_state_{"front_left_kingpin",
    HW_IF_POSITION,
    &kingpin_state_[0]};
  hardware_interface::StateInterface front_right_steering_state_{"front_right_kingpin",
    HW_IF_POSITION,
    &kingpin_state_[1]};
  hardware_interface::StateInterface rear_left_steering_state_{"rear_left_kingpin", HW_IF_POSITION,
    &kingpin_state_[2]};
  hardware_interface::StateInterface rear_right_steering_state_{"rear_right_kingpin",
    HW_IF_POSITION,
    &kingpin_state_[3]};

  hardware_interface::CommandInterface front_left_motor_vel_cmd_{"front_left_motor", HW_IF_VELOCITY,
    &motor_cmd_[0]};
  hardware_interface::CommandInterface front_right_motor_vel_cmd_{"front_right_motor",
    HW_IF_VELOCITY,
    &motor_cmd_[1]};
  hardware_interface::CommandInterface rear_left_motor_vel_cmd_{"rear_left_motor", HW_IF_VELOCITY,
    &motor_cmd_[2]};
  hardware_interface::CommandInterface rear_right_motor_vel_cmd_{"rear_right_motor", HW_IF_VELOCITY,
    &motor_cmd_[3]};

  hardware_interface::CommandInterface front_steering_cmd_{"front_steering", HW_IF_POSITION,
    &steering_cmd_[0]};
  hardware_interface::CommandInterface rear_steering_cmd_{"rear_steering",
    HW_IF_POSITION,
    &steering_cmd_[1]};

  rclcpp::Node::SharedPtr pub_node;
  rclcpp::Publisher<FourWheelSteeringStamped>::SharedPtr cmd_publisher;
};

TEST_F(TestFourWheelSteeringController, configure_fails_without_parameters)
{
  const auto ret = controller_->init(controller_name);
  ASSERT_EQ(ret, controller_interface::return_type::OK);
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(TestFourWheelSteeringController, correct_straight_driving)
{
  constexpr double THRESHOLD = 1e-12;

  const auto ret = controller_->init(controller_name);
  ASSERT_EQ(ret, controller_interface::return_type::OK);

  controller_->get_node()->set_parameter(rclcpp::Parameter("wheel_base", 1.5));
  controller_->get_node()->set_parameter(rclcpp::Parameter("wheel_track", 1.0));
  controller_->get_node()->set_parameter(rclcpp::Parameter("wheel_radius", 0.25));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  auto state = controller_->configure();
  assignResources();

  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  EXPECT_NEAR(1.0, front_left_motor_vel_state_.get_value(), THRESHOLD);
  EXPECT_NEAR(1.0, front_right_motor_vel_state_.get_value(), THRESHOLD);
  EXPECT_NEAR(1.0, rear_left_motor_vel_state_.get_value(), THRESHOLD);
  EXPECT_NEAR(1.0, rear_right_motor_vel_state_.get_value(), THRESHOLD);

  state = controller_->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());

  send_cmd(1.0, 0.0);
  ASSERT_TRUE(controller_->wait_for_cmd(executor));

  ASSERT_EQ(controller_->update(Time(0, 0), Duration(0, 0)), controller_interface::return_type::OK);
  EXPECT_NEAR(4.0, front_left_motor_vel_cmd_.get_value(), THRESHOLD);
  EXPECT_NEAR(4.0, front_right_motor_vel_cmd_.get_value(), THRESHOLD);
  EXPECT_NEAR(4.0, rear_left_motor_vel_cmd_.get_value(), THRESHOLD);
  EXPECT_NEAR(4.0, rear_right_motor_vel_cmd_.get_value(), THRESHOLD);

  // deactivated
  // wait so controller process the second point when deactivated
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  state = controller_->deactivate();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(controller_->update(Time(0, 0), Duration(0, 0)), controller_interface::return_type::OK);

  EXPECT_EQ(0.0, front_left_motor_vel_cmd_.get_value()) << "Wheels are halted on deactivate()";
  EXPECT_EQ(0.0, front_right_motor_vel_cmd_.get_value()) << "Wheels are halted on deactivate()";
  EXPECT_EQ(0.0, rear_left_motor_vel_cmd_.get_value()) << "Wheels are halted on deactivate()";
  EXPECT_EQ(0.0, rear_right_motor_vel_cmd_.get_value()) << "Wheels are halted on deactivate()";

  // cleanup
  state = controller_->cleanup();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());

  EXPECT_EQ(0.0, front_left_motor_vel_cmd_.get_value());
  EXPECT_EQ(0.0, front_right_motor_vel_cmd_.get_value());
  EXPECT_EQ(0.0, rear_left_motor_vel_cmd_.get_value());
  EXPECT_EQ(0.0, rear_right_motor_vel_cmd_.get_value());

  state = controller_->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  executor.cancel();
}
