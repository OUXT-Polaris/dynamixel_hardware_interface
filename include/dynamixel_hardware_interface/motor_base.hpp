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

#ifndef DYNAMIXEL_HARDWARE_INTERFACE__MOTOR_BASE_HPP_
#define DYNAMIXEL_HARDWARE_INTERFACE__MOTOR_BASE_HPP_

#include <dynamixel_sdk/dynamixel_sdk.h>

#include <dynamixel_hardware_interface/address_table_base.hpp>
#include <dynamixel_hardware_interface/constants.hpp>
#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_status_values.hpp>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace dynamixel_hardware_interface
{
struct Result
{
  const std::string description;
  const bool success;
  Result(const std::string & description, bool success)
  : description(description), success(success)
  {
  }
};

class MotorBase
{
public:
  const SupportedMotors motor_type;
  const std::string joint_name;
  const bool enable_dummy;
  const int baudrate;
  const uint8_t id;
  MotorBase() = delete;
  template<typename AddressTable>
  MotorBase(
    const SupportedMotors & motor_type, const std::string & joint_name, const bool enable_dummy,
    const AddressTable & table, int baudrate, uint8_t id,
    std::shared_ptr<dynamixel::PortHandler> port_handler,
    std::shared_ptr<dynamixel::PacketHandler> packet_handler)
  : motor_type(motor_type),
    joint_name(joint_name),
    enable_dummy(enable_dummy),
    baudrate(baudrate),
    id(id),
    port_handler_(port_handler),
    packet_handler_(packet_handler),
    joint_position_(std::numeric_limits<double>::quiet_NaN()),
    goal_position_(std::numeric_limits<double>::quiet_NaN())
  {
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("dynamixel_hardware_interface"), "start constructing motor instance");
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("dynamixel_hardware_interface"), "joint_name : " << joint_name);
    address_table_ = std::make_shared<AddressTableBase>(table);
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("dynamixel_hardware_interface"), "end constructing motor instance");
  }
  ~MotorBase();
  bool operationSupports(const Operation & operation);
  std::vector<Operation> getSupportedOperations();
  Result configure();
  Result torqueEnable(bool enable);
  Result setGoalPosition(double goal_position);
  double getJointPosition() const {return joint_position_;}
  double getGoalPosition() const {return goal_position_;}
  Result updateJointPosition();
  void appendStateInterfaces(std::vector<hardware_interface::StateInterface> & interfaces);
  void appendCommandInterfaces(std::vector<hardware_interface::CommandInterface> & interfaces);

private:
  Result getResult(int communication_result, uint8_t packet_error);
  uint16_t radianToPosition(double radian) const;
  double positionToRadian(const uint16_t position) const;
  std::shared_ptr<AddressTableBase> address_table_;
  std::shared_ptr<dynamixel::PortHandler> port_handler_;
  std::shared_ptr<dynamixel::PacketHandler> packet_handler_;
  double joint_position_;
  double goal_position_;
};
}  //  namespace dynamixel_hardware_interface

#endif  // DYNAMIXEL_HARDWARE_INTERFACE__MOTOR_BASE_HPP_
