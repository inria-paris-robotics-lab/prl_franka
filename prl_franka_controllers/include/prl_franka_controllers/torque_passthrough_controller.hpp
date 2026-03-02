#pragma once

#include <controller_interface/controller_interface.hpp>
#include <deque>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "prl_franka_msgs/msg/torque_data.hpp"
#include "prl_franka_msgs/msg/trajectory_torque.hpp"

#include "franka_semantic_components/franka_robot_model.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace prl_franka_controllers {

class TorquePassthroughController
    : public controller_interface::ControllerInterface {
public:
  TorquePassthroughController() = default;
  ~TorquePassthroughController() override = default;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  Eigen::VectorXd computeExternalTorque();
  Eigen::VectorXd computeTauBias();

private:
  std::string arm_id_;
  size_t num_joints_ = 7;
  std::unordered_map<std::string, size_t> state_interface_map_;

  double k_p_ = 0.0;
  double k_i_ = 0.0;
  double k_q_ = 0.0;
  double k_v_ = 0.0;
  double dt_ = 0.001; // Assuming a control loop running at 1 kHz
  std::array<double, 16> NE_T_EE = {1, 0, 0, 0, 0, 1, 0,    0,
                                    0, 0, 1, 0, 0, 0, 0.07, 1};
  Eigen::VectorXd tau_bias_, tau_error_integral_, tau_ext_;

  // Trajectory FIFO
  std::deque<trajectory_msgs::msg::JointTrajectoryPoint> trajectory_fifo_;

  // Subscriber / Publisher
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      trajectory_sub_;
  rclcpp::Publisher<prl_franka_msgs::msg::TorqueData>::SharedPtr
      torque_cmd_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr
      pos_and_vel_pub_;

  // Robot model
  std::unique_ptr<franka_semantic_components::FrankaRobotModel>
      franka_robot_model_;

  // Interface names for robot state and model
  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};
};

} // namespace prl_franka_controllers
