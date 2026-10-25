/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, Wonik Robotics.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Wonik Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef AH_POSITION_EFFORT_CONTROLLER__VISIBILITY_CONTROL_H_
#define AH_POSITION_EFFORT_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define AH_POSITION_EFFORT_CONTROLLER_EXPORT __attribute__((dllexport))
#define AH_POSITION_EFFORT_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define AH_POSITION_EFFORT_CONTROLLER_EXPORT __declspec(dllexport)
#define AH_POSITION_EFFORT_CONTROLLER_IMPORT __declspec(dllimport)
#endif
#ifdef AH_POSITION_EFFORT_CONTROLLER_BUILDING_DLL
#define AH_POSITION_EFFORT_CONTROLLER_PUBLIC AH_POSITION_EFFORT_CONTROLLER_EXPORT
#else
#define AH_POSITION_EFFORT_CONTROLLER_PUBLIC AH_POSITION_EFFORT_CONTROLLER_IMPORT
#endif
#define AH_POSITION_EFFORT_CONTROLLER_PUBLIC_TYPE AH_POSITION_EFFORT_CONTROLLER_PUBLIC
#define AH_POSITION_EFFORT_CONTROLLER_LOCAL
#else
#define AH_POSITION_EFFORT_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define AH_POSITION_EFFORT_CONTROLLER_IMPORT
#if __GNUC__ >= 4
#define AH_POSITION_EFFORT_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#define AH_POSITION_EFFORT_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define AH_POSITION_EFFORT_CONTROLLER_PUBLIC
#define AH_POSITION_EFFORT_CONTROLLER_LOCAL
#endif
#define AH_POSITION_EFFORT_CONTROLLER_PUBLIC_TYPE
#endif

#endif // AH_POSITION_EFFORT_CONTROLLER__VISIBILITY_CONTROL_H_
