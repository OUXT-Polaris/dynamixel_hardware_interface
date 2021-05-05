/**
 * @file motor_base.hpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief base class of the dynamixel motor
 * @version 0.1
 * @date 2021-05-01
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
/**
 * @brief Struct describes the command result.
 */
struct Result
{
  /**
   * @brief Description of the result.
   */
  const std::string description;
  /**
   * @brief If true, command execute successfully.
   */
  const bool success;
  /**
   * @brief Construct a new Result object.
   * @param description Description of the result.
   * @param success If true, command execute successfully.
   */
  Result(const std::string & description, bool success) : description(description), success(success)
  {
  }
};

/**
 * @brief Base class for controlling dynamixel motor.
 */
class MotorBase
{
public:
  /**
   * @brief Describe the type of the motor.
   */
  const SupportedMotors motor_type;
  /**
   * @brief Name of the joint which the motor is attaching to.
   */
  const std::string joint_name;
  /**
   * @brief If true, you can communicate with virtual dinamixel motor.
   */
  const bool enable_dummy;
  /**
   * @brief Baudrate of the serial communication.
   */
  const int baudrate;
  /**
   * @brief Id of the dynamixel motor.
   */
  const uint8_t id;
  /**
   * @brief Construct a new Motor Base object.
   * @tparam AddressTable address table type of the motor.
   * @param motor_type Type of the motor.
   * @param joint_name Name of the joint which the motor is attaching to.
   * @param enable_dummy If true, you can communicate with virtual dinamixel motor.
   * @param table address table of the motor.
   * @param baudrate Baudrate of the serial communication.
   * @param id Id of the dynamixel motor.
   * @param port_handler Port handler class of the dynamixel sdk.
   * @param packet_handler Packet handler class of the dynamixel sdk
   */
  template <typename AddressTable>
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
  /**
   * @brief Destroy the Motor Base object
   */
  ~MotorBase();
  /**
   * @brief Check the operation is support in your motor.
   * @param operation Operation which you want to execute.
   * @return true Operation supports.
   * @return false Operation does not support.
   */
  bool operationSupports(const Operation & operation);
  /**
   * @brief Get list of supported Operations in your motor.
   * @return std::vector<Operation> List of supported operations.
   */
  virtual std::vector<Operation> getSupportedOperations();
  /**
   * @brief Configure dynamixel motor.
   * @return Result result of the configuration.
   */
  virtual Result configure();
  /**
   * @brief Execute torqu_enabled command to the motor.
   * @param enable if true, enable torque.
   * @return Result result of the command.
   */
  virtual Result torqueEnable(bool enable);
  /**
   * @brief Execute goal_position command to the motor.
   * @param goal_position goal position angle in radian.
   * @return Result result of the command.
   */
  virtual Result setGoalPosition(double goal_position);
  /**
   * @brief Get current joint position of the motor.
   * @return double Current joint position of the motor in radian.
   */
  virtual double getJointPosition() const { return joint_position_; }
  /**
   * @brief Get current goal position of the motor.
   * @return double Current goal position of the motor in radian.
   */
  virtual double getGoalPosition() const { return goal_position_; }
  /**
   * @brief Execute update joint position command to the motor.
   * @return Result result of the command.
   */
  virtual Result updateJointPosition();
  /**
   * @brief Append state interface described in the URDF file.
   * @param interfaces List of state interface.
   */
  virtual void appendStateInterfaces(std::vector<hardware_interface::StateInterface> & interfaces);
  /**
   * @brief Append command interface described in the URDF file.
   * @param interfaces List of command interface.
   */
  virtual void appendCommandInterfaces(
    std::vector<hardware_interface::CommandInterface> & interfaces);

protected:
  /**
   * @brief Construct a new Motor Base object
   */
  MotorBase() = delete;
  Result getResult(int communication_result, uint8_t packet_error);
  template <typename T>
  T radianToPosition(double radian) const
  {
    T value;
    radianToPosition(radian, value);
    return value;
  }
  uint16_t radianToPosition(double radian) const;
  virtual double positionToRadian(const uint8_t position) const;
  virtual double positionToRadian(const uint16_t position) const;
  virtual double positionToRadian(const uint32_t position) const;
  virtual void radianToPosition(double radian, uint8_t & value) const
  {
    value = static_cast<uint8_t>((radian / M_PI) * 256);
  }
  virtual void radianToPosition(double radian, uint16_t & value) const
  {
    value = static_cast<uint16_t>((radian / M_PI) * 65536);
  }
  virtual void radianToPosition(double radian, uint32_t & value) const
  {
    value = static_cast<uint32_t>((radian / M_PI) * 4294967296);
  }
  std::shared_ptr<AddressTableBase> address_table_;
  std::shared_ptr<dynamixel::PortHandler> port_handler_;
  std::shared_ptr<dynamixel::PacketHandler> packet_handler_;
  double joint_position_;
  double goal_position_;
};
}  //  namespace dynamixel_hardware_interface

#endif  // DYNAMIXEL_HARDWARE_INTERFACE__MOTOR_BASE_HPP_
