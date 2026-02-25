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
    if (!get_node()->get_parameter("arm_id", arm_id_)) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to get arm_id parameter");
      return CallbackReturn::ERROR;
    }
    // Init vectors
    initial_tau_ext_ = Eigen::VectorXd::Zero(num_joints_);
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
      return CallbackReturn::ERROR;
    }
  }

  initial_tau_ext_ = initial_tau_measured - gravity_vec;
  tau_error_integral_.setZero(); // Reset integral
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type
TorquePassthroughController::update(const rclcpp::Time & /*time*/,
                                    const rclcpp::Duration & /*period*/) {
  tau_ext_ = computeExternalTorque();
  Eigen::VectorXd torque_cmd = Eigen::VectorXd::Zero(num_joints_);
  if (trajectory_fifo_.empty()) {
    for (size_t i = 0; i < command_interfaces_.size(); ++i) {
      command_interfaces_[i].set_value(torque_cmd(i));
    }
  } else {
    const auto &point = trajectory_fifo_.front(); // premier point
    Eigen::VectorXd tau_des(point.effort.size());
    for (size_t j = 0; j < point.effort.size(); ++j) {
      if (j < point.effort.size())
        tau_des(j) = point.effort[j];
      else
        tau_des(j) = 0.0;
    }
    tau_error_integral_ += (tau_des - tau_ext_) *
                           0.001; // Integrate error (assuming 1ms update rate)
    torque_cmd =
        tau_des + k_p_ * (tau_des - tau_ext_) + k_i_ * tau_error_integral_;

    for (size_t i = 0; i < command_interfaces_.size(); ++i) {
      command_interfaces_[i].set_value(torque_cmd(i));
    }
    trajectory_fifo_.pop_front(); // retirer le point joué
  }

  // Publish torque command and external torque for monitoring
  if (torque_cmd_pub_) {
    prl_franka_msgs::msg::TorqueData msg;
    msg.torque_command.resize(num_joints_);
    msg.torque_external.resize(num_joints_);
    for (size_t i = 0; i < num_joints_; ++i) {
      msg.torque_command[i] = torque_cmd(i);
      msg.torque_external[i] = tau_ext_(i);
    }
    torque_cmd_pub_->publish(msg);
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
  Eigen::VectorXd tau_ext = tau_measured - tau_gravity - initial_tau_ext_;
  return tau_ext;
}

} // namespace prl_franka_controllers

PLUGINLIB_EXPORT_CLASS(prl_franka_controllers::TorquePassthroughController,
                       controller_interface::ControllerInterface)
