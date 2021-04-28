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

#ifndef PERCEPTION_CAMERA__VISIBILITY_H_
#define PERCEPTION_CAMERA__VISIBILITY_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PERCEPTION_CAMERA_EXPORT __attribute__ ((dllexport))
    #define PERCEPTION_CAMERA_IMPORT __attribute__ ((dllimport))
  #else
    #define PERCEPTION_CAMERA_EXPORT __declspec(dllexport)
    #define PERCEPTION_CAMERA_IMPORT __declspec(dllimport)
  #endif
  #ifdef PERCEPTION_CAMERA_BUILDING_DLL
    #define PERCEPTION_CAMERA_PUBLIC PERCEPTION_CAMERA_EXPORT
  #else
    #define PERCEPTION_CAMERA_PUBLIC PERCEPTION_CAMERA_IMPORT
  #endif
  #define PERCEPTION_CAMERA_PUBLIC_TYPE PERCEPTION_CAMERA_PUBLIC
  #define PERCEPTION_CAMERA_LOCAL
#else
  #define PERCEPTION_CAMERA_EXPORT __attribute__ ((visibility("default")))
  #define PERCEPTION_CAMERA_IMPORT
  #if __GNUC__ >= 4
    #define PERCEPTION_CAMERA_PUBLIC __attribute__ ((visibility("default")))
    #define PERCEPTION_CAMERA_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PERCEPTION_CAMERA_PUBLIC
    #define PERCEPTION_CAMERA_LOCAL
  #endif
  #define PERCEPTION_CAMERA_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // PERCEPTION_CAMERA__VISIBILITY_H_
