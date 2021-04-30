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

#ifndef DYNAMIXEL_HARDWARE_INTERFACE__MOTORS__XW54_T260_HPP_
#define DYNAMIXEL_HARDWARE_INTERFACE__MOTORS__XW54_T260_HPP_

#include <dynamixel_hardware_interface/address_table_base.hpp>

#include <cmath>
#include <limits>

namespace dynamixel_hardware_interface
{
namespace address_tables
{
class XW54_T260 : public AddressTableBase
{
public:
  XW54_T260()
  : AddressTableBase(
      64,
      116,
      std::numeric_limits<uint16_t>::quiet_NaN(),
      132,
      128,
      std::numeric_limits<uint16_t>::quiet_NaN(),
      std::numeric_limits<uint16_t>::quiet_NaN(),
      146)
  {}
};
}  // namespace address_tables
}  // namespace dynamixel_hardware_interface

#endif  // DYNAMIXEL_HARDWARE_INTERFACE__MOTORS__XW54_T260_HPP_
