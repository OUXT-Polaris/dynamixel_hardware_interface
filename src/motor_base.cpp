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

#include <dynamixel_hardware_interface/motor_base.hpp>

namespace dynamixel_hardware_interface
{
MotorBase::~MotorBase()
{
}

Result MotorBase::getResult(int communication_result, uint8_t packet_error)
{
  if (communication_result != COMM_SUCCESS) {
    return Result(std::string(packet_handler_->getTxRxResult(communication_result)), false);
  }
  if (packet_error != 0) {
    return Result(std::string(packet_handler_->getRxPacketError(packet_error)), true);
  }
  return Result("", true);
}

Result MotorBase::torqueEnable(bool enable)
{
  if (!address_table_->addressExists(Operation::TORQUE_ENABLE)) {
    return Result("TORQUE_ENABLE operation does not support in " + motor_type, false);
  }
  uint8_t error = 0;
  const auto address = address_table_->getAddress(Operation::TORQUE_ENABLE);
  const auto result = packet_handler_->write1ByteTxRx(port_handler_.get(), id, address, enable, &error);
  return getResult(result, error);
}
}  //  namespace dynamixel_hardware_interface
