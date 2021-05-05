/**
 * @file visiblity_control.h
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief Header file to control visibility.
 * @version 0.1
 * @date 2021-05-01
 *
 * @copyright Copyright (c) OUXT Polaris 2021
 *
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

#ifndef DYNAMIXEL_HARDWARE_INTERFACE__VISIBLITY_CONTROL_H_
#define DYNAMIXEL_HARDWARE_INTERFACE__VISIBLITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define DYNAMIXEL_HARDWARE_INTERFACE_EXPORT __attribute__((dllexport))
#define DYNAMIXEL_HARDWARE_INTERFACE_IMPORT __attribute__((dllimport))
#else
#define DYNAMIXEL_HARDWARE_INTERFACE_EXPORT __declspec(dllexport)
#define DYNAMIXEL_HARDWARE_INTERFACE_IMPORT __declspec(dllimport)
#endif
#ifdef DYNAMIXEL_HARDWARE_INTERFACE_BUILDING_DLL
#define DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC DYNAMIXEL_HARDWARE_INTERFACE_EXPORT
#else
#define DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC DYNAMIXEL_HARDWARE_INTERFACE_IMPORT
#endif
#define DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC_TYPE DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
#define DYNAMIXEL_HARDWARE_INTERFACE_LOCAL
#else
#define DYNAMIXEL_HARDWARE_INTERFACE_EXPORT __attribute__((visibility("default")))
#define DYNAMIXEL_HARDWARE_INTERFACE_IMPORT
#if __GNUC__ >= 4
#define DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC __attribute__((visibility("default")))
#define DYNAMIXEL_HARDWARE_INTERFACE_LOCAL __attribute__((visibility("hidden")))
#else
#define DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
#define DYNAMIXEL_HARDWARE_INTERFACE_LOCAL
#endif
#define DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC_TYPE
#endif

#endif  // DYNAMIXEL_HARDWARE_INTERFACE__VISIBLITY_CONTROL_H_
