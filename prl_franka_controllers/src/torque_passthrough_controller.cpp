#include "prl_franka_controllers/torque_passthrough_controller.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace prl_franka_controllers {

controller_interface::InterfaceConfiguration
TorquePassthroughController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (size_t i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
TorquePassthroughController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  // Add all state interfaces required by the FrankaRobotModel
  for (const auto &franka_robot_model_name :
       franka_robot_model_->get_state_interface_names()) {
    state_interfaces_config.names.push_back(franka_robot_model_name);
  }
  // Add the standard joint state interfaces (position, velocity, effort) for
  // each joint
  for (size_t i = 1; i <= num_joints_; ++i) {
    state_interfaces_config.names.push_back(arm_id_ + "_joint" +
                                            std::to_string(i) + "/position");
    state_interfaces_config.names.push_back(arm_id_ + "_joint" +
                                            std::to_string(i) + "/velocity");
    state_interfaces_config.names.push_back(arm_id_ + "_joint" +
                                            std::to_string(i) + "/effort");
  }
  return state_interfaces_config;
}

controller_interface::CallbackReturn TorquePassthroughController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<double>("k_p", 0.0);
    auto_declare<double>("k_i", 0.0);
    auto_declare<double>("k_q", 0.0);
    auto_declare<double>("k_v", 0.0);
    if (!get_node()->get_parameter("arm_id", arm_id_)) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to get arm_id parameter");
      return CallbackReturn::ERROR;
    }
    if (!get_node()->get_parameter("k_p", k_p_)) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to get k_p parameter");
      return CallbackReturn::ERROR;
    }
    if (!get_node()->get_parameter("k_i", k_i_)) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to get k_i parameter");
      return CallbackReturn::ERROR;
    }
    if (!get_node()->get_parameter("k_q", k_q_)) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to get k_q parameter");
      return CallbackReturn::ERROR;
    }
    if (!get_node()->get_parameter("k_v", k_v_)) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to get k_v parameter");
      return CallbackReturn::ERROR;
    }
    // Init vectors
    tau_bias_ = Eigen::VectorXd::Zero(num_joints_);
    tau_error_integral_ = Eigen::VectorXd::Zero(num_joints_);
    tau_ext_ = Eigen::VectorXd::Zero(num_joints_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Exception thrown during init stage: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TorquePassthroughController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {

  trajectory_sub_ =
      get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
          "/torque_passthrough_controller/torque_trajectory", 10,
          [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
            if (!msg->points.empty()) {
              for (auto &point : msg->points) {
                trajectory_fifo_.push_back(point);
              }
              RCLCPP_INFO(get_node()->get_logger(),
                          "Received %zu points, FIFO size: %zu",
                          msg->points.size(), trajectory_fifo_.size());
            }
          });

  torque_cmd_pub_ =
      get_node()->create_publisher<prl_franka_msgs::msg::TorqueData>(
          "/torque_passthrough_controller/torque_commanded", 10);
  pos_and_vel_pub_ =
      get_node()->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
          "/torque_passthrough_controller/pos_vel_data", 10);

  try {
    franka_robot_model_ =
        std::make_unique<franka_semantic_components::FrankaRobotModel>(
            arm_id_ + "/" + k_robot_model_interface_name,
            arm_id_ + "/" + k_robot_state_interface_name);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error creating model: %s",
                 e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TorquePassthroughController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // Initialize any necessary state or variables here
  if (franka_robot_model_) {
    franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  }
  // Build state interface map for quick lookup
  state_interface_map_.clear();
  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    state_interface_map_[state_interfaces_[i].get_name()] = i;
  }

  // Initial Bias estimation
  tau_bias_ = computeTauBias();
  tau_error_integral_.setZero(); // Reset integral
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type
TorquePassthroughController::update(const rclcpp::Time & /*time*/,
                                    const rclcpp::Duration & /*period*/) {
  tau_ext_ = computeExternalTorque();
  Eigen::VectorXd tau_des = Eigen::VectorXd::Zero(num_joints_);
  // get pos measured and vel measured
  Eigen::VectorXd pos_meas = Eigen::VectorXd::Zero(num_joints_);
  Eigen::VectorXd vel_meas = Eigen::VectorXd::Zero(num_joints_);
  for (size_t i = 0; i < num_joints_; ++i) {
    std::string pos_name =
        arm_id_ + "_joint" + std::to_string(i + 1) + "/position";
    std::string vel_name =
        arm_id_ + "_joint" + std::to_string(i + 1) + "/velocity";
    pos_meas(i) =
        state_interfaces_[state_interface_map_.at(pos_name)].get_value();
    vel_meas(i) =
        state_interfaces_[state_interface_map_.at(vel_name)].get_value();
  }
  Eigen::VectorXd pos_des = Eigen::VectorXd::Zero(num_joints_);
  Eigen::VectorXd vel_des = Eigen::VectorXd::Zero(num_joints_);
  Eigen::VectorXd tau_des_raw = Eigen::VectorXd::Zero(
      num_joints_); // For monitoring raw desired torque before feedback
  if (trajectory_fifo_.empty()) {
    for (size_t i = 0; i < command_interfaces_.size(); ++i) {
      command_interfaces_[i].set_value(
          0.0); // Set to zero when no trajectory is being executed
    }
    tau_bias_ =
        computeTauBias(); // Update bias when no trajectory is being executed
  } else {
    const auto &point =
        trajectory_fifo_.front(); // Get the first point in the FIFO
    for (size_t j = 0; j < point.effort.size(); ++j) {
      if (j < point.effort.size())
        tau_des(j) = point.effort[j];
      else
        tau_des(j) = 0.0;
      pos_des(j) = point.positions[j];
      vel_des(j) = point.velocities[j];
    }
    tau_des_raw = tau_des; // Store raw desired torque for monitoring
    tau_des += k_q_ * (pos_des - pos_meas) + k_v_ * (vel_des - vel_meas);
    Eigen::VectorXd tau_error(tau_des - tau_ext_);

    tau_error_integral_ += tau_error * dt_;
    Eigen::VectorXd torque_cmd = Eigen::VectorXd::Zero(num_joints_);
    torque_cmd = tau_des + k_p_ * tau_error + k_i_ * tau_error_integral_;

    for (size_t i = 0; i < command_interfaces_.size(); ++i) {
      command_interfaces_[i].set_value(torque_cmd(i));
    }
    trajectory_fifo_.pop_front(); // remove point after processing
  }

  // Publish torque command and external torque for monitoring
  if (torque_cmd_pub_) {
    prl_franka_msgs::msg::TorqueData msg;
    msg.torque_desired.resize(num_joints_);
    msg.torque_command.resize(num_joints_);
    msg.torque_external.resize(num_joints_);
    for (size_t i = 0; i < num_joints_; ++i) {
      msg.torque_desired[i] = tau_des_raw(i);
      msg.torque_command[i] = tau_des(i);
      msg.torque_external[i] = tau_ext_(i);
    }
    torque_cmd_pub_->publish(msg);
  }
  if (pos_and_vel_pub_) {
    trajectory_msgs::msg::JointTrajectoryPoint pos_vel_msg;
    pos_vel_msg.positions.resize(num_joints_);
    pos_vel_msg.velocities.resize(num_joints_);
    for (size_t i = 0; i < num_joints_; ++i) {
      pos_vel_msg.positions[i] = pos_des(i);
      pos_vel_msg.velocities[i] = vel_des(i);
    }
    pos_and_vel_pub_->publish(pos_vel_msg);
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn TorquePassthroughController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  trajectory_fifo_.clear();
  return CallbackReturn::SUCCESS;
}

Eigen::VectorXd TorquePassthroughController::computeExternalTorque() {
  // Get gravity compensation torques from the robot model
  std::array<double, 7> gravity_array =
      franka_robot_model_->getGravityForceVector();
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_gravity(
      gravity_array.data());
  // Get measured torque from robot state interfaces
  Eigen::VectorXd tau_measured = Eigen::VectorXd::Zero(num_joints_);
  for (size_t i = 0; i < num_joints_; ++i) {
    std::string name = arm_id_ + "_joint" + std::to_string(i + 1) + "/effort";
    tau_measured(i) =
        state_interfaces_[state_interface_map_.at(name)].get_value();
  }
  // Compute external torque estimation
  Eigen::VectorXd tau_ext = tau_measured - tau_gravity - tau_bias_;
  return tau_ext;
}

Eigen::VectorXd TorquePassthroughController::computeTauBias() {
  // Get initial gravity compensation torques from the robot
  std::array<double, 7> gravity_array =
      franka_robot_model_->getGravityForceVector();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity_vec(gravity_array.data());
  Eigen::VectorXd initial_tau_measured = Eigen::VectorXd::Zero(num_joints_);

  for (size_t i = 0; i < num_joints_; ++i) {
    // Interface name format: "arm_id_jointX/effort"
    std::string name = arm_id_ + "_joint" + std::to_string(i + 1) + "/effort";

    if (state_interface_map_.find(name) != state_interface_map_.end()) { //
      initial_tau_measured(i) =
          state_interfaces_[state_interface_map_[name]].get_value();
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Interface %s not found in map!",
                   name.c_str());
    }
  }

  return initial_tau_measured - gravity_vec;
}

} // namespace prl_franka_controllers

PLUGINLIB_EXPORT_CLASS(prl_franka_controllers::TorquePassthroughController,
                       controller_interface::ControllerInterface)
