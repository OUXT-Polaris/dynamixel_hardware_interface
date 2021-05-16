/**
 * @file dynamixel_diagnostic_controller.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief implementation of the dynamixal diagnostic controller class
 * @version 0.1
 * @date 2021-05-16
 * @copyright Copyright (c) OUXT Polaris 2021
 */

// Copyright (c) 2019 OUXT Polaris
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
  if (get_node()->get_parameter("joints", joints)) {
    joints_ = joints.as_string_array();
    for (const auto & joint : joints_) {
      rclcpp::Parameter diagnostics;
      if (!get_node()->get_parameter("diagnostics/" + joint, diagnostics)) {
        return controller_interface::return_type::ERROR;
      }
      std::vector<dynamixel_hardware_interface::DiagnosticsType> diag_list;
      const auto diagnostics_strings = diagnostics.as_string_array();
      for (const auto & diag_string : diagnostics_strings) {
        if (diag_string == "tempelature") {
          diag_list.emplace_back(dynamixel_hardware_interface::DiagnosticsType::TEMPELATURE);
        }
      }
      diagnostics_[joint] = diag_list;
    }
  } else {
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DynamixelDiagnosticController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  for (const auto & joint : joints_) {
    diagnostic_updaters_[joint] = std::make_shared<diagnostic_updater::Updater>(node);
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type DynamixelDiagnosticController::update()
{
  return controller_interface::return_type::OK;
}
}  // namespace dynamixel_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dynamixel_hardware_interface::DynamixelDiagnosticController,
  controller_interface::ControllerInterface)
