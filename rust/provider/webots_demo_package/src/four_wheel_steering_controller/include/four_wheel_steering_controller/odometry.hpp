/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Irstea
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Irstea nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef FOUR_WHEEL_STEERING_CONTROLLER__ODOMETRY_HPP_
#define FOUR_WHEEL_STEERING_CONTROLLER__ODOMETRY_HPP_

#include <cmath>

#include "rclcpp/time.hpp"
#include "rcppmath/rolling_mean_accumulator.hpp"

namespace four_wheel_steering_controller
{
  /**
   * \brief The Odometry class handles odometry readings
   * (2D pose and velocity with related timestamp)
   */
  class Odometry
  {
  public:    
     using UpdateFunction = std::function<bool(double, double, double, double, double, double, double, double, const rclcpp::Time &)>;
  
    /**
     * \brief Constructor
     * Timestamp will get the current time value
     * Value will be set to zero
     * \param velocity_rolling_window_size Rolling window size used to compute the velocity mean
     */
    Odometry(size_t velocity_rolling_window_size = 10);

    /**
     * \brief Initialize the odometry
     * \param time Current time
     */
    void init(const rclcpp::Time & time);
    
    /**
     * \brief Updates the odometry class with latest wheels and steerings position
     * \param fl_speed front left wheel vehicle speed [rad/s]
     * \param fr_speed front right wheel vehicle speed [rad/s]
     * \param rl_speed rear left wheel vehicle speed [rad/s]
     * \param rr_speed rear right wheel vehicle speed [rad/s]
     * \param fl_steering front left steering position [rad]
     * \param fr_steering front right steering position [rad]
     * \param rl_steering rear left steering position [rad]
     * \param rr_steering rear right steering position [rad]
     * \param time      Current time
     * \return true if the odometry is actually updated
     */     
    UpdateFunction update;
    
    // all-四舵轮(全)
    bool update_all(double fl_speed, double fr_speed, double rl_speed, double rr_speed, double fl_steering, double fr_steering, double rl_steering, double rr_steering, const rclcpp::Time &time);
    
    // flrr-四舵轮(前左fl,后右rr)
    bool update_flrr(double fl_speed, double fr_speed, double rl_speed, double rr_speed, double fl_steering, double fr_steering, double rl_steering, double rr_steering, const rclcpp::Time &time);
    
    // frrl-四舵轮(前右fr,后左rl)
    bool update_frrl(double fl_speed, double fr_speed, double rl_speed, double rr_speed, double fl_steering, double fr_steering, double rl_steering, double rr_steering, const rclcpp::Time &time);

    void setUpdateFunction(std::string chassis_type);
    
    /**
     * \brief heading getter
     * \return heading [rad]
     */
    double getHeading() const
    {
      return heading_;
    }

    /**
     * \brief x position getter
     * \return x position [m]
     */
    double getX() const
    {
      return x_;
    }

    /**
     * \brief y position getter
     * \return y position [m]
     */
    double getY() const
    {
      return y_;
    }

    /**
     * \brief linear velocity getter norm
     * \return linear velocity [m/s]
     */
    double getLinear() const
    {
      return linear_;
    }

    /**
     * \brief linear velocity getter along X on the robot base link frame
     * \return linear velocity [m/s]
     */
    double getLinearX() const
    {
      return linear_x_;
    }

    /**
     * \brief linear velocity getter along Y on the robot base link frame
     * \return linear velocity [m/s]
     */
    double getLinearY() const
    {
      return linear_y_;
    }

    /**
     * \brief angular velocity getter
     * \return angular velocity [rad/s]
     */
    double getAngular() const
    {
      return angular_;
    }

    /**
     * \brief linear acceleration getter
     * \return linear acceleration [m/s²]
     */
    double getLinearAcceleration() const
    {      
        return linear_accel_acc_.getRollingMean();
    }

    /**
     * \brief linear jerk getter
     * \return linear jerk [m/s³]
     */
    double getLinearJerk() const
    {
        return linear_jerk_acc_.getRollingMean();
    }

    /**
     * \brief front steering velocity getter
     * \return front_steer_vel [m/s³]
     */
    double getFrontSteerVel() const
    {      
        return front_steer_vel_acc_.getRollingMean();
    }

    /**
     * \brief rear steering velocity getter
     * \return rear_steer_vel [m/s³]
     */
    double getRearSteerVel() const
    {      
      return rear_steer_vel_acc_.getRollingMean();
    }

    /**
     * \brief Sets the wheel parameters: radius and separation
     * \param steering_track          Seperation between left and right steering joints [m]
     * \param wheel_steering_y_offset Offest between the steering and wheel joints [m]
     * \param wheel_radius            Wheel radius [m]
     * \param wheel_base              Wheel base [m]
     */
    void setWheelParams(double steering_track, double wheel_steering_y_offset, double wheel_radius, double wheel_base);

    /**
     * \brief Velocity rolling window size setter
     * \param velocity_rolling_window_size Velocity rolling window size
     */
    void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

  private:
    using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;

    /**
     * \brief Integrates the velocities (linear on x and y and angular)
     * \param linear_x  Linear  velocity along x of the robot frame  [m] (linear  displacement, i.e. m/s * dt) computed by encoders
     * \param linear_y  Linear  velocity along y of the robot frame   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
     * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
     * \param pivot_turn Whether robot in pivot_turn state or not
     */
    void integrateXY(double linear_x, double linear_y, double angular, bool pivot_turn);

    /**
     * \brief Integrates the velocities (linear and angular) using 2nd order Runge-Kutta
     * \param linear  Linear  velocity   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
     * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
     */
    void integrateRungeKutta2(double linear, double angular);

    /**
     * \brief Integrates the velocities (linear and angular) using exact method
     * \param linear  Linear  velocity   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
     * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
     */
    void integrateExact(double linear, double angular);

    /**
     *  \brief Reset linear and angular accumulators
     */
    void resetAccumulators();

    /// Current timestamp:
    rclcpp::Time last_update_timestamp_;

    /// Current pose:
    double x_;        //   [m]
    double y_;        //   [m]
    double heading_;  // [rad]

    /// Current velocity:
    double linear_, linear_x_, linear_y_;  //   [m/s]
    double angular_; // [rad/s]

    /// Wheel kinematic parameters [m]:
    double steering_track_;
    double wheel_steering_y_offset_;
    double wheel_radius_;
    double wheel_base_;
    double diagonal_angle_; // 舵轮斜对角角度
    double diagonal_distance_; // 舵轮斜对角距离
    
    /// Previous wheel position/state [rad]:
    double wheel_old_pos_;

    /// Rolling mean accumulators for the linar and angular velocities:
    size_t velocity_rolling_window_size_;
    RollingMeanAccumulator linear_accel_acc_;
    RollingMeanAccumulator linear_jerk_acc_;
    RollingMeanAccumulator front_steer_vel_acc_;
    RollingMeanAccumulator rear_steer_vel_acc_;
    double linear_vel_prev_, linear_accel_prev_;
    double front_steer_vel_prev_, rear_steer_vel_prev_;
    
  };
} // namespace four_wheel_steering_controller

#endif  // FOUR_WHEEL_STEERING_CONTROLLER__ODOMETRY_HPP_
