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

#ifndef PERCEPTION_LIDAR__VISIBILITY_CONTROL_H_
#define PERCEPTION_LIDAR__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PERCEPTION_LIDAR_EXPORT __attribute__ ((dllexport))
    #define PERCEPTION_LIDAR_IMPORT __attribute__ ((dllimport))
  #else
    #define PERCEPTION_LIDAR_EXPORT __declspec(dllexport)
    #define PERCEPTION_LIDAR_IMPORT __declspec(dllimport)
  #endif
  #ifdef PERCEPTION_LIDAR_BUILDING_DLL
    #define PERCEPTION_LIDAR_PUBLIC PERCEPTION_LIDAR_EXPORT
  #else
    #define PERCEPTION_LIDAR_PUBLIC PERCEPTION_LIDAR_IMPORT
  #endif
  #define PERCEPTION_LIDAR_PUBLIC_TYPE PERCEPTION_LIDAR_PUBLIC
  #define PERCEPTION_LIDAR_LOCAL
#else
  #define PERCEPTION_LIDAR_EXPORT __attribute__ ((visibility("default")))
  #define PERCEPTION_LIDAR_IMPORT
  #if __GNUC__ >= 4
    #define PERCEPTION_LIDAR_PUBLIC __attribute__ ((visibility("default")))
    #define PERCEPTION_LIDAR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PERCEPTION_LIDAR_PUBLIC
    #define PERCEPTION_LIDAR_LOCAL
  #endif
  #define PERCEPTION_LIDAR_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // PERCEPTION_LIDAR__VISIBILITY_CONTROL_H_
