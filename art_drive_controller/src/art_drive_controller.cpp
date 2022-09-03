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

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "art_drive_controller/art_drive_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
constexpr auto DEFAULT_ACKERMANN_OUT_TOPIC = "~/cmd_ackermann";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}  // namespace

namespace art_drive_controller
{
using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

ArtDriveController::ArtDriveController() : controller_interface::ControllerInterface() {}

controller_interface::return_type ArtDriveController::init(const std::string & controller_name)
{
  // initialize lifecycle node
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK)
  {
    return ret;
  }

  try
  {
    // with the lifecycle node being initialized, we can declare parameters
    auto_declare<std::string>("front_left_wheel_joint_name", std::string());
    auto_declare<std::string>("front_right_wheel_joint_name", std::string());
    auto_declare<std::string>("rear_left_wheel_joint_name", std::string());
    auto_declare<std::string>("rear_right_wheel_joint_name", std::string());

    auto_declare<std::string>("articulated_steering_joint_name", std::string());
    auto_declare<std::string>("rear_steering_joint_name", std::string());
    auto_declare<std::string>("front_steering_joint_name", std::string());

    auto_declare<double>("rear_axle_wheelbase", wheel_params_.rear_axle_wheelbase);
    auto_declare<double>("front_axle_wheelbase", wheel_params_.front_axle_wheelbase;
    auto_declare<double>("rear_axle_length", wheel_params_.rear_axle_length);
    auto_declare<double>("front_axle_length", wheel_params_.front_axle_length);
    auto_declare<double>("wheel_radius", wheel_params_.wheel_radius);

    auto_declare<std::string>("odom_frame_id", odom_params_.odom_frame_id);
    auto_declare<std::string>("base_frame_id", odom_params_.base_frame_id);
    auto_declare<std::vector<double>>("pose_covariance_diagonal", std::vector<double>());
    auto_declare<std::vector<double>>("twist_covariance_diagonal", std::vector<double>());
    auto_declare<bool>("open_loop", odom_params_.open_loop);
    auto_declare<bool>("enable_odom_tf", odom_params_.enable_odom_tf);
    auto_declare<bool>("odom_only_twist", odom_params_.odom_only_twist);    

    auto_declare<double>("cmd_vel_timeout", cmd_vel_timeout_.count() / 1000.0);//Need to define a safe timeout
    auto_declare<bool>("publish_limited_velocity", publish_limited_velocity_);
    auto_declare<bool>("publish_ackermann_command", publish_ackermann_command_);//perhaps it is not needed
    auto_declare<int>("velocity_rolling_window_size", 10);
    auto_declare<bool>("use_stamped_vel", use_stamped_vel_);

    auto_declare<bool>("linear.x.has_velocity_limits", false);
    auto_declare<bool>("linear.x.has_acceleration_limits", false);
    auto_declare<bool>("linear.x.has_jerk_limits", false);
    auto_declare<double>("linear.x.max_velocity", NAN);
    auto_declare<double>("linear.x.min_velocity", NAN);
    auto_declare<double>("linear.x.max_acceleration", NAN);
    auto_declare<double>("linear.x.min_acceleration", NAN);
    auto_declare<double>("linear.x.max_jerk", NAN);
    auto_declare<double>("linear.x.min_jerk", NAN);

    auto_declare<bool>("angular.z.has_velocity_limits", false);
    auto_declare<bool>("angular.z.has_acceleration_limits", false);
    auto_declare<bool>("angular.z.has_jerk_limits", false);
    auto_declare<double>("angular.z.max_velocity", NAN);
    auto_declare<double>("angular.z.min_velocity", NAN);
    auto_declare<double>("angular.z.max_acceleration", NAN);
    auto_declare<double>("angular.z.min_acceleration", NAN);
    auto_declare<double>("angular.z.max_jerk", NAN);
    auto_declare<double>("angular.z.min_jerk", NAN);

    auto_declare<double>("articulated_steering.max_position", NAN);
    auto_declare<double>("articulated_steering.min_position", NAN);
    auto_declare<double>("articulated_steering.max_velocity", NAN);
    auto_declare<double>("articulated_steering.min_velocity", NAN);
    auto_declare<double>("articulated_steering.max_acceleration", NAN);
    auto_declare<double>("articulated_steering.min_acceleration", NAN);

    auto_declare<bool>("rear_steering.active", false);
    auto_declare<double>("rear_steering.max_position", NAN);
    auto_declare<double>("rear_steering.min_position", NAN);
    auto_declare<double>("rear_steering.max_velocity", NAN);
    auto_declare<double>("rear_steering.min_velocity", NAN);
    auto_declare<double>("rear_steering.max_acceleration", NAN);
    auto_declare<double>("rear_steering.min_acceleration", NAN);

    auto_declare<bool>("front_steering.active", false);
    auto_declare<double>("front_steering.max_position", NAN);
    auto_declare<double>("front_steering.min_position", NAN);
    auto_declare<double>("front_steering.max_velocity", NAN);
    auto_declare<double>("front_steering.min_velocity", NAN);
    auto_declare<double>("front_steering.max_acceleration", NAN);
    auto_declare<double>("front_steering.min_acceleration", NAN);

    auto_declare<double>("publish_rate", publish_rate_);
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

InterfaceConfiguration ArtDriveController::command_interface_configuration() const
{
  InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = interface_configuration_type::INDIVIDUAL;
  //Wheels joint names
  command_interfaces_config.names.push_back(front_left_wheel_joint_name_ + "/" + HW_IF_VELOCITY);
  command_interfaces_config.names.push_back(front_right_wheel_joint_name_ + "/" + HW_IF_VELOCITY);
  command_interfaces_config.names.push_back(rear_left_wheel_joint_name_ + "/" + HW_IF_VELOCITY);
  command_interfaces_config.names.push_back(rear_right_wheel_joint_name_ + "/" + HW_IF_VELOCITY);

  //Steering joint names
  command_interfaces_config.names.push_back(articulated_steering_joint_name_ + "/" + HW_IF_POSITION);
  command_interfaces_config.names.push_back(rear_steering_joint_name_ + "/" + HW_IF_POSITION);
  command_interfaces_config.names.push_back(front_steering_joint_name_ + "/" + HW_IF_POSITION);

  return command_interfaces_config;
}

InterfaceConfiguration ArtDriveController::state_interface_configuration() const
{

  InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = interface_configuration_type::INDIVIDUAL;
  //Wheels joint names
  state_interfaces_config.names.push_back(front_left_wheel_joint_name_ + "/" + HW_IF_VELOCITY);
  state_interfaces_config.names.push_back(front_right_wheel_joint_name_ + "/" + HW_IF_VELOCITY);
  state_interfaces_config.names.push_back(rear_left_wheel_joint_name_ + "/" + HW_IF_VELOCITY);
  state_interfaces_config.names.push_back(rear_right_wheel_joint_name_ + "/" + HW_IF_VELOCITY);

  //Steering joint names
  state_interfaces_config.names.push_back(articulated_steering_joint_name_ + "/" + HW_IF_POSITION);
  state_interfaces_config.names.push_back(rear_steering_joint_name_ + "/" + HW_IF_POSITION);
  state_interfaces_config.names.push_back(front_steering_joint_name_ + "/" + HW_IF_POSITION);

  return state_interfaces_config;
}

// should we add time and period like in the tricycle controller?
controller_interface::return_type ArtDriveController::update() 
{
  auto logger = node_->get_logger();

  if (get_current_state().id() == State::PRIMARY_STATE_INACTIVE)
  {
    if (!is_halted)
    {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }

  const auto current_time = node_->get_clock()->now();

  //std::shared_ptr<Twist> last_command_msg;
  std::shared_ptr<TwistStamped> last_command_msg;
  received_velocity_msg_ptr_.get(last_command_msg);

  if (last_command_msg == nullptr)
  {
    RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  const auto dt = current_time - last_command_msg->header.stamp;
  // Brake if cmd_vel has timeout, override the stored command
  if (dt > cmd_vel_timeout_)
  {
    last_command_msg->twist.linear.x = 0.0;
    last_command_msg->twist.angular.z = 0.0;
  }

  // command may be limited further by SpeedLimit,
  // without affecting the stored twist command

  //Twist command = *last_command_msg;
  TwistStamped command = *last_command_msg;
  double & linear_command = command.twist.linear.x;
  double & angular_command = command.twist.angular.z;
  // The following is assuming that we have an encoder mounted on each wheel
  double rlws_read = rear_left_wheel_joint_[0].velocity_state.get().get_value();           // wheels speed in radians/s
  double rrws_read = rear_right_wheel_joint_[0].velocity_state.get().get_value(); 
  double flws_read = front_left_wheel_joint_[0].velocity_state.get().get_value(); 
  double frws_read = front_right_wheel_joint_[0].velocity_state.get().get_value();
  // We must have the angular position of each joint 
  double phi_read = articulated_steering_joint_[0].position_state.get().get_value(); // articulated joint angle in radians
  double xi_read = rear_steering_joint_[0].position_state.get().get_value();    // rear steering joint angle in radians
  double zeta_read = front_steering_joint_[0].position_state.get().get_value();  // front steering joint angle in radians

  // Apply (possibly new) multipliers:
  const auto wheels = wheel_params_;
  const double wheel_radius = wheels.radius;
  const double rear_axle_length = wheels.rear_axle_length;
  const double front_axle_length = wheels.front_axle_length;
  const double rear_axle_wheelbase = wheels.rear_axle_wheelbase;
  const double front_axle_wheelbase = wheels.front_axle_wheelbase;

  if (odom_params_.open_loop)
  {
    odometry_.updateOpenLoop(linear_command, angular_command, current_time); // current time or should we use an input parameter to the function like in the tricycle controller?
  }
  else
  {
    if (std::isnan(rlws_read) || 
        std::isnan(rrws_read) || 
        std::isnan(flws_read) ||
        std::isnan(frws_read) ||
        std::isnan(phi_read)  ||
        std::isnan(rrws_read) ||
        std::isnan(xi_read)   || 
        std::isnan(zeta_read))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Could not read feedback value");
      return controller_interface::return_type::ERROR;
    }
    odometry_.update(rlws_read, rws_read, flws_read, frws_read, alpha_read, current_time);
  }

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.getHeading());

  if (previous_publish_timestamp_ + publish_period_ < current_time)
  {
    previous_publish_timestamp_ += publish_period_;

    if (realtime_odometry_publisher_->trylock())
    {
      auto & odometry_message = realtime_odometry_publisher_->msg_;
      odometry_message.header.stamp = current_time;
      odometry_message.pose.pose.position.x = odometry_.getX();
      odometry_message.pose.pose.position.y = odometry_.getY();
      odometry_message.pose.pose.orientation.x = orientation.x();
      odometry_message.pose.pose.orientation.y = orientation.y();
      odometry_message.pose.pose.orientation.z = orientation.z();
      odometry_message.pose.pose.orientation.w = orientation.w();
      odometry_message.twist.twist.linear.x = odometry_.getLinear();
      odometry_message.twist.twist.angular.z = odometry_.getAngular();
      realtime_odometry_publisher_->unlockAndPublish();
    }

    if (odom_params_.enable_odom_tf && realtime_odometry_transform_publisher_->trylock())
    {
      auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
      transform.header.stamp = current_time;
      transform.transform.translation.x = odometry_.getX();
      transform.transform.translation.y = odometry_.getY();
      transform.transform.rotation.x = orientation.x();
      transform.transform.rotation.y = orientation.y();
      transform.transform.rotation.z = orientation.z();
      transform.transform.rotation.w = orientation.w();
      realtime_odometry_transform_publisher_->unlockAndPublish();
    }
  }

  const auto update_dt = current_time - previous_update_timestamp_;
  previous_update_timestamp_ = current_time;

  auto & last_command = previous_commands_.back().twist;
  auto & second_to_last_command = previous_commands_.front().twist;
  limiter_linear_.limit(
    linear_command, last_command.linear.x, second_to_last_command.linear.x, update_dt.seconds());
  limiter_angular_.limit(
    angular_command, last_command.angular.z, second_to_last_command.angular.z, update_dt.seconds());

  previous_commands_.pop();
  previous_commands_.emplace(command);

  //    Publish limited velocity
  if (publish_limited_velocity_ && realtime_limited_velocity_publisher_->trylock())
  {
    auto & limited_velocity_command = realtime_limited_velocity_publisher_->msg_;
    limited_velocity_command.header.stamp = current_time;
    limited_velocity_command.twist = command.twist;
    realtime_limited_velocity_publisher_->unlockAndPublish();
  }

  auto [psi_write, xi_write, zeta_write, ws_write] = twist_to_articulated(linear_command, angular_command, )

  // Compute wheels velocities:
  const double velocity_left =
    (linear_command - angular_command * wheel_separation / 2.0) / left_wheel_radius;
  const double velocity_right =
    (linear_command + angular_command * wheel_separation / 2.0) / right_wheel_radius;

  // Set wheels velocities:
  for (size_t index = 0; index < wheels.wheels_per_side; ++index)
  {
    registered_left_wheel_handles_[index].velocity.get().set_value(velocity_left);
    registered_right_wheel_handles_[index].velocity.get().set_value(velocity_right);
  }

  return controller_interface::return_type::OK;
}

CallbackReturn ArtDriveController::on_configure(const rclcpp_lifecycle::State &)
{
  auto logger = node_->get_logger();

  // update parameters
  left_wheel_names_ = node_->get_parameter("left_wheel_names").as_string_array();
  right_wheel_names_ = node_->get_parameter("right_wheel_names").as_string_array();

  if (left_wheel_names_.size() != right_wheel_names_.size())
  {
    RCLCPP_ERROR(
      logger, "The number of left wheels [%zu] and the number of right wheels [%zu] are different",
      left_wheel_names_.size(), right_wheel_names_.size());
    return CallbackReturn::ERROR;
  }

  if (left_wheel_names_.empty())
  {
    RCLCPP_ERROR(logger, "Wheel names parameters are empty!");
    return CallbackReturn::ERROR;
  }

  wheel_params_.separation = node_->get_parameter("wheel_separation").as_double();
  wheel_params_.wheels_per_side =
    static_cast<size_t>(node_->get_parameter("wheels_per_side").as_int());
  wheel_params_.radius = node_->get_parameter("wheel_radius").as_double();
  wheel_params_.separation_multiplier =
    node_->get_parameter("wheel_separation_multiplier").as_double();
  wheel_params_.left_radius_multiplier =
    node_->get_parameter("left_wheel_radius_multiplier").as_double();
  wheel_params_.right_radius_multiplier =
    node_->get_parameter("right_wheel_radius_multiplier").as_double();

  const auto wheels = wheel_params_;

  const double wheel_separation = wheels.separation_multiplier * wheels.separation;
  const double left_wheel_radius = wheels.left_radius_multiplier * wheels.radius;
  const double right_wheel_radius = wheels.right_radius_multiplier * wheels.radius;

  odometry_.setWheelParams(wheel_separation, left_wheel_radius, right_wheel_radius);
  odometry_.setVelocityRollingWindowSize(
    node_->get_parameter("velocity_rolling_window_size").as_int());

  odom_params_.odom_frame_id = node_->get_parameter("odom_frame_id").as_string();
  odom_params_.base_frame_id = node_->get_parameter("base_frame_id").as_string();

  auto pose_diagonal = node_->get_parameter("pose_covariance_diagonal").as_double_array();
  std::copy(
    pose_diagonal.begin(), pose_diagonal.end(), odom_params_.pose_covariance_diagonal.begin());

  auto twist_diagonal = node_->get_parameter("twist_covariance_diagonal").as_double_array();
  std::copy(
    twist_diagonal.begin(), twist_diagonal.end(), odom_params_.twist_covariance_diagonal.begin());

  odom_params_.open_loop = node_->get_parameter("open_loop").as_bool();
  odom_params_.enable_odom_tf = node_->get_parameter("enable_odom_tf").as_bool();

  cmd_vel_timeout_ = std::chrono::milliseconds{
    static_cast<int>(node_->get_parameter("cmd_vel_timeout").as_double() * 1000.0)};
  publish_limited_velocity_ = node_->get_parameter("publish_limited_velocity").as_bool();
  use_stamped_vel_ = node_->get_parameter("use_stamped_vel").as_bool();

  try
  {
    limiter_linear_ = SpeedLimiter(
      node_->get_parameter("linear.x.has_velocity_limits").as_bool(),
      node_->get_parameter("linear.x.has_acceleration_limits").as_bool(),
      node_->get_parameter("linear.x.has_jerk_limits").as_bool(),
      node_->get_parameter("linear.x.min_velocity").as_double(),
      node_->get_parameter("linear.x.max_velocity").as_double(),
      node_->get_parameter("linear.x.min_acceleration").as_double(),
      node_->get_parameter("linear.x.max_acceleration").as_double(),
      node_->get_parameter("linear.x.min_jerk").as_double(),
      node_->get_parameter("linear.x.max_jerk").as_double());
  }
  catch (const std::runtime_error & e)
  {
    RCLCPP_ERROR(node_->get_logger(), "Error configuring linear speed limiter: %s", e.what());
  }

  // TODO: set it in terms of the steering joints
  try
  {
    limiter_angular_ = SpeedLimiter(
      node_->get_parameter("angular.z.has_velocity_limits").as_bool(),
      node_->get_parameter("angular.z.has_acceleration_limits").as_bool(),
      node_->get_parameter("angular.z.has_jerk_limits").as_bool(),
      node_->get_parameter("angular.z.min_velocity").as_double(),
      node_->get_parameter("angular.z.max_velocity").as_double(),
      node_->get_parameter("angular.z.min_acceleration").as_double(),
      node_->get_parameter("angular.z.max_acceleration").as_double(),
      node_->get_parameter("angular.z.min_jerk").as_double(),
      node_->get_parameter("angular.z.max_jerk").as_double());
  }
  catch (const std::runtime_error & e)
  {
    RCLCPP_ERROR(node_->get_logger(), "Error configuring angular speed limiter: %s", e.what());
  }

  if (!reset())
  {
    return CallbackReturn::ERROR;
  }

  // left and right sides are both equal at this point
  wheel_params_.wheels_per_side = left_wheel_names_.size();

  if (publish_limited_velocity_)
  {
    limited_velocity_publisher_ =
      node_->create_publisher<Twist>(DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_limited_velocity_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<Twist>>(limited_velocity_publisher_);
  }

  const Twist empty_twist;
  received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));

  // Fill last two commands with default constructed commands
  previous_commands_.emplace(empty_twist);
  previous_commands_.emplace(empty_twist);

  // initialize command subscriber
  if (use_stamped_vel_)
  {
    velocity_command_subscriber_ = node_->create_subscription<Twist>(
      DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<Twist> msg) -> void {
        if (!subscriber_is_active_)
        {
          RCLCPP_WARN(node_->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }
        if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
        {
          RCLCPP_WARN_ONCE(
            node_->get_logger(),
            "Received TwistStamped with zero timestamp, setting it to current "
            "time, this message will only be shown once");
          msg->header.stamp = node_->get_clock()->now();
        }
        received_velocity_msg_ptr_.set(std::move(msg));
      });
  }
  else
  {
    velocity_command_unstamped_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void {
        if (!subscriber_is_active_)
        {
          RCLCPP_WARN(node_->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }

        // Write fake header in the stored stamped command
        std::shared_ptr<Twist> twist_stamped;
        received_velocity_msg_ptr_.get(twist_stamped);
        twist_stamped->twist = *msg;
        twist_stamped->header.stamp = node_->get_clock()->now();
      });
  }

  // initialize odometry publisher and messasge
  odometry_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>(
    DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      odometry_publisher_);

  auto & odometry_message = realtime_odometry_publisher_->msg_;
  odometry_message.header.frame_id = odom_params_.odom_frame_id;
  odometry_message.child_frame_id = odom_params_.base_frame_id;

  // limit the publication on the topics /odom and /tf
  publish_rate_ = node_->get_parameter("publish_rate").as_double();
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);
  previous_publish_timestamp_ = node_->get_clock()->now();

  // initialize odom values zeros
  odometry_message.twist =
    geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index)
  {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    odometry_message.pose.covariance[diagonal_index] = odom_params_.pose_covariance_diagonal[index];
    odometry_message.twist.covariance[diagonal_index] =
      odom_params_.twist_covariance_diagonal[index];
  }

  // initialize transform publisher and message
  odometry_transform_publisher_ = node_->create_publisher<tf2_msgs::msg::TFMessage>(
    DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_transform_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
      odometry_transform_publisher_);

  // keeping track of odom and base_link transforms only
  auto & odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
  odometry_transform_message.transforms.resize(1);
  odometry_transform_message.transforms.front().header.frame_id = odom_params_.odom_frame_id;
  odometry_transform_message.transforms.front().child_frame_id = odom_params_.base_frame_id;

  previous_update_timestamp_ = node_->get_clock()->now();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArtDriveController::on_activate(const rclcpp_lifecycle::State &)
{
  // Initialize wheels speeds and steering angles
  const auto rlw_result = get_traction(rear_left_wheel_joint_name_, rear_left_wheel_joint_);
  const auto rrw_result = get_traction(rear_right_wheel_joint_name_, rear_right_wheel_joint_);
  const auto flw_result = get_traction(front_left_wheel_joint_name_, front_left_wheel_joint_);
  const auto frw_result = get_traction(front_right_wheel_joint_name_, front_right_wheel_joint_);

  const auto as_result = get_steering(articulated_steering_joint_name_, steering_joint_);
  const auto rs_result = get_steering(rear_steering_joint_name_, rear_steering_joint_);
  const auto fs_result = get_steering(front_steering_joint_name_, front_steering_joint_);
  
  if (rlw_result  == CallbackReturn::ERROR || 
      rrw_result  == CallbackReturn::ERROR || 
      flw_result  == CallbackReturn::ERROR || 
      frw_result  == CallbackReturn::ERROR || 
      as_result   == CallbackReturn::ERROR || 
      rs_result   == CallbackReturn::ERROR || 
      fs_result   == CallbackReturn::ERROR )
  {
    return CallbackReturn::ERROR;
  }
  if (rear_left_wheel_joint_.empty()     || 
      rear_right_wheel_joint_.empty()    ||
      front_left_wheel_joint_.empty()    ||
      front_right_wheel_joint_.empty()   ||
      articulated_steering_joint.empty() ||
      rear_steering_joint_.empty()       ||
      front_steering_joint_.empty())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Either steering or wheels interfaces do not exist");
    return CallbackReturn::ERROR;
  }

  is_halted = false;
  subscriber_is_active_ = true;

  RCLCPP_DEBUG(node_->get_logger(), "Subscriber and publisher are now active.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArtDriveController::on_deactivate(const rclcpp_lifecycle::State &)
{
  subscriber_is_active_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArtDriveController::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return CallbackReturn::ERROR;
  }

  received_velocity_msg_ptr_.set(std::make_shared<Twist>());
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArtDriveController::on_error(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

bool ArtDriveController::reset()
{
  odometry_.resetOdometry();

  // release the old queue
  // TODO: check against the differential drive command. Which one is more intuitive?
  std::queue<AckermannDrive> empty_ackermann_drive;
  std::swap(previous_commands_, empty_ackermann_drive);

  rear_left_wheel_joint_.clear();
  rear_right_wheel_joint_.clear();
  front_left_wheel_joint_.clear();
  front_right_wheel_joint_.clear();

  articulated_steering_joint_.clear();
  rear_steering_joint_.clear();
  front_steering_joint_.clear();

  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();
  velocity_command_unstamped_subscriber_.reset();

  received_velocity_msg_ptr_.set(nullptr);
  is_halted = false;
  return true;
}

CallbackReturn ArtDriveController::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

void ArtDriveController::halt()
{
  front_left_wheel_joint_[0].velocity_command.get().set_value(0.0);
  front_right_wheel_joint_[0].velocity_command.get().set_value(0.0);
  rear_left_wheel_joint_[0].velocity_command.get().set_value(0.0);
  rear_right_wheel_joint_[0].velocity_command.get().set_value(0.0);

  articulated_steering_joint_[0].position_command.get().set_value(0.0);
  rear_steering_joint_[0].position_command.get().set_value(0.0);
  front_steering_joint_[0].position_command.get().set_value(0.0);
}

CallbackReturn ArtDriveController::get_traction(
  const std::string & traction_joint_name, std::vector<TractionHandle> & joint)
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
    RCLCPP_ERROR(
      get_node()->get_logger(), "Unable to obtain joint state handle for %s",
      traction_joint_name.c_str());
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
    RCLCPP_ERROR(
      get_node()->get_logger(), "Unable to obtain joint state handle for %s",
      traction_joint_name.c_str());
    return CallbackReturn::ERROR;
  }

  // Create the traction joint instance
  joint.emplace_back(TractionHandle{std::ref(*state_handle), std::ref(*command_handle)});
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArtDriveController::get_steering(
  const std::string & steering_joint_name, std::vector<SteeringHandle> & joint)
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
    RCLCPP_ERROR(
      get_node()->get_logger(), "Unable to obtain joint state handle for %s",
      steering_joint_name.c_str());
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
    RCLCPP_ERROR(
      get_node()->get_logger(), "Unable to obtain joint state handle for %s",
      steering_joint_name.c_str());
    return CallbackReturn::ERROR;
  }

  // Create the steering joint instance
  joint.emplace_back(SteeringHandle{std::ref(*state_handle), std::ref(*command_handle)});
  return CallbackReturn::SUCCESS;
}

double ArtDriveController::convert_trans_rot_vel_to_steering_angles(
  double Vx, double theta_dot, double wheelbase)
{
  if (theta_dot == 0 || Vx == 0)
  {
    return 0;
  }
  return std::atan(theta_dot * wheelbase / Vx);
}

std::tuple<double, double, double, double> ArtDriveController::twist_to_articulated(double Vx, double theta_dot)
{
  // using naming convention in http://users.isr.ist.utl.pt/~mir/cadeiras/robmovel/Kinematics.pdf
  double phi, xi, zeta, ws, R;

  if (theta_dot == 0 || Vx == 0)
  {
    phi, xi, zeta , ws = 0;
  }
  else if(theta_dot == 0 && abs(Vx) >__DBL_EPSILON__)
  { //straight motion
    phi = 0.0;
    xi = 0.0;
    zeta = 0.0;
    ws = Vx / (wheel_params_.radius);
  }
  else if (Vx == 0 && abs(theta_dot) > __DBL_EPSILON__)
  {  // the machine is spinning

    phi = theta_dot > 0 ? articulated_steering.max_position : articulated_steering.min_position;
    xi = theta_dot > 0 ? rear_stering.min_position : rear_steering.max_position;
    zeta = theta_dot > 0 ? front_steering.max_position : front_steering.min_position;

    ws = abs(theta_dot) * wheel_params_.wheelbase / wheel_params_.radius;
    return std::make_tuple(phi, xi, zeta, ws);
  }
  else if (abs(Vx) > __DBL_EPSILON__ && abs(theta_dot) > __DBL_EPSILON__)
  {
    alpha = convert_trans_rot_vel_to_steering_angles(Vx, theta_dot, wheel_params_.wheelbase);
    ws = Vx / (wheel_params_.radius * std::cos(alpha));
  }
  
  return std::make_tuple(phi, xi, zeta, ws);
}

}  // namespace diff_drive_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  art_drive_controller::ArtDriveController, controller_interface::ControllerInterface)
