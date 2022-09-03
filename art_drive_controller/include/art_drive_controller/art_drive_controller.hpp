// Copyright 2022 Helgen Technologies
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

/*
 * Author: Aldo Arriaga
 */

#ifndef ART_DRIVE_CONTROLLER__ART_DRIVE_CONTROLLER_HPP_
#define ART_DRIVE_CONTROLLER__ART_DRIVE_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "art_drive_controller/odometry.hpp"
#include "art_drive_controller/speed_limiter.hpp"
#include "art_drive_controller/steering_limiter.hpp"
#include "art_drive_controller/visibility_control.h"


namespace art_drive_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ArtDriveController : public controller_interface::ControllerInterface
{
  using Twist = geometry_msgs::msg::TwistStamped;

public:
  ART_DRIVE_CONTROLLER_PUBLIC
  DiffDriveController();

  ART_DRIVE_CONTROLLER_PUBLIC
  controller_interface::return_type init(const std::string & controller_name) override;

  ART_DRIVE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  ART_DRIVE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  ART_DRIVE_CONTROLLER_PUBLIC
  controller_interface::return_type update() override;

  ART_DRIVE_CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  ART_DRIVE_CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ART_DRIVE_CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ART_DRIVE_CONTROLLER_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  ART_DRIVE_CONTROLLER_PUBLIC
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  ART_DRIVE_CONTROLLER_PUBLIC
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

  CallbackReturn get_traction(
    const std::string & traction_joint_name, std::vector<TractionHandle> & joint);
  CallbackReturn get_steering(
    const std::string & steering_joint_name, std::vector<SteeringHandle> & joint);
  double convert_trans_rot_vel_to_steering_angle(double v, double omega, double wheelbase);
  std::tuple<double, double> twist_to_ackermann(double linear_command, double angular_command);


  std::string rear_left_wheel_joint_name_;
  std::string rear_right_wheel_joint_name_;
  std::string front_left_wheel_joint_name_;
  std::string front_right_wheel_joint_name_;

  std::string articulated_steering_joint_name_;
  std::string rear_steering_joint_name_;
  std::string front_steering_joint_name_;

  // HACK: put into vector to avoid initializing structs because they have no default constructors

  std::vector<TractionHandle> rear_left_wheel_joint_;
  std::vector<TractionHandle> rear_right_wheel_joint_;
  std::vector<TractionHandle> front_left_wheel_joint_;
  std::vector<TractionHandle> front_right_wheel_joint_;
  
  std::vector<SteeringHandle> articulated_steering_joint_;
  std::vector<SteeringHandle> rear_steering_joint_;
  std::vector<SteeringHandle> front_steering_joint_;

  struct WheelParams
  {
    double rear_axle_length = 0.0;
    double front_axle_length = 0.0;
    double rear_axle_wheelbase = 0.0;
    double front_axle_wheelbase = 0.0;
    double wheel_radius = 0.0;
  } wheel_params_;

  struct OdometryParams
  {
    bool open_loop = false;
    bool enable_odom_tf = false;
    bool odom_only_twist = false;  // for doing the pose integration in separate node
    std::string base_frame_id = "base_link";
    std::string odom_frame_id = "odom";
    std::array<double, 6> pose_covariance_diagonal;
    std::array<double, 6> twist_covariance_diagonal;
  } odom_params_;

  bool publish_ackermann_command_ = false;
  std::shared_ptr<rclcpp::Publisher<AckermannDrive>> ackermann_command_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<AckermannDrive>>
    realtime_ackermann_command_publisher_ = nullptr;

  Odometry odometry_;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
    realtime_odometry_publisher_ = nullptr;

  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_ =
    nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
    realtime_odometry_transform_publisher_ = nullptr;

  // Timeout to consider cmd_vel commands old
  std::chrono::milliseconds cmd_vel_timeout_{500};

  bool subscriber_is_active_ = false;
  rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    velocity_command_unstamped_subscriber_ = nullptr;

  realtime_tools::RealtimeBox<std::shared_ptr<Twist>> received_velocity_msg_ptr_{nullptr};

  std::queue<Twist> previous_commands_;  // last two commands

  // speed limiters
  SpeedLimiter limiter_linear_;
  SpeedLimiter limiter_angular_;

  bool publish_limited_velocity_ = false;
  std::shared_ptr<rclcpp::Publisher<Twist>> limited_velocity_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<Twist>> realtime_limited_velocity_publisher_ =
    nullptr;

  rclcpp::Time previous_update_timestamp_{0};

  // publish rate limiter
  double publish_rate_ = 50.0;
  rclcpp::Duration publish_period_{0, 0};
  rclcpp::Time previous_publish_timestamp_{0};

  bool is_halted = false;
  bool use_stamped_vel_ = true;

  bool reset();
  void halt();
};
}  // namespace diff_drive_controller
#endif  // ART_DRIVE_CONTROLLER__ART_DRIVE_CONTROLLER_HPP_
