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

Result MotorBase::torqueEnable()
{
  if (!address_table_->addressExists(Operation::TORQUE_ENABLE)) {
    return Result("TORQUE_ENABLE operation does not support in " + motor_type, false);
  }
  // const auto address = address_table_->getAddress(Operation::TORQUE_ENABLE);
}
}  //  namespace dynamixel_hardware_interface
