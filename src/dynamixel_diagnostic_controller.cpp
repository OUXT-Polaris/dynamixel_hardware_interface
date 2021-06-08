/**
 * @file dynamixel_diagnostic_controller.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief implementation of the dynamixal diagnostic controller class
 * @version 0.1
 * @date 2021-05-16
 * @copyright Copyright (c) OUXT Polaris 2021
 */

// Copyright (c) 2021 OUXT Polaris
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

#include <dynamixel_hardware_interface/dymanixel_diagnostic_controller.hpp>

namespace dynamixel_hardware_interface
{
controller_interface::return_type DynamixelDiagnosticController::init(
  const std::string & controller_name)
{
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }
  rclcpp::Parameter joints;
  auto node = get_node();
  node->declare_parameter<std::vector<std::string>>("joints", {});
  joints_ = node->get_parameter("joints").as_string_array();
  for (const auto & joint : joints_) {
    rclcpp::Parameter diagnostics;
    node->declare_parameter<std::vector<std::string>>(joint, {});
    if (!get_node()->get_parameter(joint, diagnostics)) {
      return controller_interface::return_type::ERROR;
    }
    std::vector<dynamixel_hardware_interface::DiagnosticsType> diag_list;
    const auto diagnostics_strings = diagnostics.as_string_array();
    for (const auto & diag_string : diagnostics_strings) {
      if (diag_string == "temperature") {
        diag_list.emplace_back(dynamixel_hardware_interface::DiagnosticsType::TEMPERATURE);
      }
    }
    diagnostics_[joint] = diag_list;
  }
  return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DynamixelDiagnosticController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  diag_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::SystemDefaultsQoS());
  diag_pub_realtime_ =
    std::make_shared<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>(
      diag_pub_);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

double DynamixelDiagnosticController::getValue(
  const std::string & joint_name, const std::string & interface_name)
{
  for (const auto & interface : state_interfaces_) {
    if (interface.get_name() == joint_name && interface.get_interface_name() == interface_name) {
      return interface.get_value();
    }
  }
  throw std::runtime_error(
    "state interface : " + interface_name + " does not exist in : " + joint_name);
}

controller_interface::return_type DynamixelDiagnosticController::update()
{
  if (diag_pub_realtime_->trylock()) {
    auto msg = diagnostic_msgs::msg::DiagnosticArray();
    msg.header.stamp = get_node()->get_clock()->now();
    for (const auto & joint : joints_) {
      const auto diagnostic_types = diagnostics_.at(joint);
      auto diag_msg = diagnostic_msgs::msg::DiagnosticStatus();
      diag_msg.name = "dynamixel_diagnostics";
      diag_msg.hardware_id = joint;
      diag_msg.level = diag_msg.OK;
      for (const auto & diag_type : diagnostic_types) {
        auto keyvalue_msg = diagnostic_msgs::msg::KeyValue();
        switch (diag_type) {
          case DiagnosticsType::TEMPERATURE:
            keyvalue_msg.key = "temperature";
            keyvalue_msg.value = std::to_string(getValue(joint, keyvalue_msg.key));
            break;
          default:
            throw std::runtime_error("diagnostic type is invalid");
            break;
        }
        diag_msg.values.emplace_back(keyvalue_msg);
        msg.status.emplace_back(diag_msg);
      }
    }
    diag_pub_realtime_->msg_ = msg;
    diag_pub_realtime_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}
}  // namespace dynamixel_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dynamixel_hardware_interface::DynamixelDiagnosticController,
  controller_interface::ControllerInterface)
