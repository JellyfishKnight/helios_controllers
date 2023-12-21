// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef CHASSIS_CONTROLLER__VISIBILITY_CONTROL_H_
#define CHASSIS_CONTROLLER__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CHASSIS_CONTROLLER_EXPORT __attribute__ ((dllexport))
    #define CHASSIS_CONTROLLER_IMPORT __attribute__ ((dllimport))
  #else
    #define CHASSIS_CONTROLLER_EXPORT __declspec(dllexport)
    #define CHASSIS_CONTROLLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef CHASSIS_CONTROLLER_BUILDING_DLL
    #define CHASSIS_CONTROLLER_PUBLIC CHASSIS_CONTROLLER_EXPORT
  #else
    #define CHASSIS_CONTROLLER_PUBLIC CHASSIS_CONTROLLER_IMPORT
  #endif
  #define CHASSIS_CONTROLLER_PUBLIC_TYPE CHASSIS_CONTROLLER_PUBLIC
  #define CHASSIS_CONTROLLER_LOCAL
#else
  #define CHASSIS_CONTROLLER_EXPORT __attribute__ ((visibility("default")))
  #define CHASSIS_CONTROLLER_IMPORT
  #if __GNUC__ >= 4
    #define CHASSIS_CONTROLLER_PUBLIC __attribute__ ((visibility("default")))
    #define CHASSIS_CONTROLLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CHASSIS_CONTROLLER_PUBLIC
    #define CHASSIS_CONTROLLER_LOCAL
  #endif
  #define CHASSIS_CONTROLLER_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // CHASSIS_CONTROLLER__VISIBILITY_CONTROL_H_