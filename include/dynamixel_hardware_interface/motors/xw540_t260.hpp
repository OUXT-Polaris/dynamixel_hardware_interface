/**
 * @file xw540_t260.hpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief Class definition for the Dynamixel XW540-T260 motor.
 * @version 0.1
 * @date 2021-05-01
 * @sa https://emanual.robotis.com/docs/en/dxl/x/xw540-t260/
 * @copyright Copyright (c) OUXT Polaris 2021
 *
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

#ifndef DYNAMIXEL_HARDWARE_INTERFACE__MOTORS__XW540_T260_HPP_
#define DYNAMIXEL_HARDWARE_INTERFACE__MOTORS__XW540_T260_HPP_

#include <cmath>
#include <dynamixel_hardware_interface/address_table_base.hpp>
#include <dynamixel_hardware_interface/motor_base.hpp>
#include <limits>
#include <memory>
#include <string>

namespace dynamixel_hardware_interface
{
namespace address_tables
{
class XW540_T260 : public AddressTableBase
{
public:
  XW540_T260()
  : AddressTableBase(
      Address(64, PacketByteSize::ONE_BYTE), Address(116, PacketByteSize::FOUR_BYTE), Address(),
      Address(132, PacketByteSize::FOUR_BYTE), Address(128, PacketByteSize::FOUR_BYTE), Address(),
      Address(), Address(146, PacketByteSize::ONE_BYTE), Address(48, PacketByteSize::FOUR_BYTE),
      Address(52, PacketByteSize::FOUR_BYTE))
  {
  }
};
}  // namespace address_tables

namespace motors
{
class XW540_T260 : public MotorBase
{
public:
  explicit XW540_T260(
    const std::string joint_name, bool enable_dummy, int baudrate, uint8_t id,
    double max_joint_limit, double min_joint_limit,
    std::shared_ptr<dynamixel::PortHandler> port_handler,
    std::shared_ptr<dynamixel::PacketHandler> packet_handler)
  : MotorBase(
      SupportedMotors::XW540_T260, joint_name, enable_dummy, address_tables::XW540_T260(), baudrate,
      id, max_joint_limit, min_joint_limit, port_handler, packet_handler)
  {
  }

  double positionToRadian(const uint32_t position) const override
  {
    return static_cast<double>(position) / static_cast<double>(4096) * M_PI * 2;
  }

  void radianToPosition(double radian, uint32_t & value) const override
  {
    value = static_cast<uint32_t>((radian / M_PI) * 4096);
  }

  double valueToRpm(uint32_t value) const override
  {
    return 2.29 * 2 * M_PI / 60 * static_cast<double>(value);
  }

  double valueToTemperature(uint8_t value) const override { return static_cast<double>(value); }
};
}  // namespace motors
}  // namespace dynamixel_hardware_interface

#endif  // DYNAMIXEL_HARDWARE_INTERFACE__MOTORS__XW540_T260_HPP_
