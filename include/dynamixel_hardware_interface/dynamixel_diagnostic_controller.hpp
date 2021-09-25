/**
 * @file dynamixel_diagnostic_controller.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief definition of the dynamixal diagnostic controller class
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

#include <dynamixel_hardware_interface/visiblity_control.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <controller_interface/controller_interface.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <dynamixel_hardware_interface/constants.hpp>
#include <memory>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace dynamixel_hardware_interface
{
class DynamixelDiagnosticController : public controller_interface::ControllerInterface
{
public:
  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  controller_interface::return_type init(const std::string & controller_name) override;

  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
  }

  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    std::vector<std::string> interface_names;
    for (const auto & joint : joints_) {
      const auto diagnostic_types = diagnostics_.at(joint);
      for (const auto & diagnostic_type : diagnostic_types) {
        switch (diagnostic_type) {
          case DiagnosticsType::TEMPERATURE:
            interface_names.emplace_back(joint + "/temperature");
            break;
          default:
            break;
        }
      }
    }
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::INDIVIDUAL, interface_names};
  }

#if GALACTIC
  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init()
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
#endif

  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) override;

  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

#if GALACTIC
  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
#else
  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  controller_interface::return_type update() override;
#endif

private:
  std::vector<std::string> joints_;
  std::unordered_map<std::string, std::vector<dynamixel_hardware_interface::DiagnosticsType>>
    diagnostics_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>
    diag_pub_realtime_;
  double getValue(const std::string & joint_name, const std::string & interface);
};
}  // namespace dynamixel_hardware_interface
