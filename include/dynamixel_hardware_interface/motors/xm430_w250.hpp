/**
 * @file xm430_w350.hpp
 * @author Yutaka Kondo (yutaka.kondo@youtalk.jp)
 * @brief Class definition for the Dynamixel XM430-W350 motor.
 * @version 0.1
 * @date 2021-07-05
 * @sa https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/
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

#ifndef DYNAMIXEL_HARDWARE_INTERFACE__MOTORS__XM430_W350_HPP_
#define DYNAMIXEL_HARDWARE_INTERFACE__MOTORS__XM430_W350_HPP_

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
class XM430_W350 : public AddressTableBase
{
public:
  XM430_W350()
  : AddressTableBase(
      Address(64, PacketByteSize::ONE_BYTE), Address(116, PacketByteSize::FOUR_BYTE), Address(),
      Address(132, PacketByteSize::FOUR_BYTE), Address(128, PacketByteSize::FOUR_BYTE), Address(),
      Address(), Address(146, PacketByteSize::ONE_BYTE))
  {
  }
};
}  // namespace address_tables

namespace motors
{
class XM430_W350 : public MotorBase
{
public:
  explicit XM430_W350(
    const std::string joint_name, bool enable_dummy, int baudrate, uint8_t id,
    std::shared_ptr<dynamixel::PortHandler> port_handler,
    std::shared_ptr<dynamixel::PacketHandler> packet_handler)
  : MotorBase(
      SupportedMotors::XM430_W350, joint_name, enable_dummy, address_tables::XM430_W350(), baudrate,
      id, port_handler, packet_handler)
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

#endif  // DYNAMIXEL_HARDWARE_INTERFACE__MOTORS__XM430_W350_HPP_
