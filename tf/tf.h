/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROS_TF_H_
#define ROS_TF_H_

#include "geometry_msgs/TransformStamped.h"

namespace tf
{

static inline geometry_msgs::Quaternion createQuaternionFromYaw(double yaw)
{
  geometry_msgs::Quaternion q;
  q.x = 0;
  q.y = 0;
  q.z = sin(yaw * 0.5);
  q.w = cos(yaw * 0.5);
  return q;
}

static inline geometry_msgs::Quaternion createQuaternionFromRPY(double roll, double pitch, double yaw)
{
  geometry_msgs::Quaternion q;
  double halfYaw = yaw * 0.5;  
  double halfPitch = pitch * 0.5;  
  double halfRoll = roll * 0.5;  
  double cosYaw = cos(halfYaw);
  double sinYaw = sin(halfYaw);
  double cosPitch = cos(halfPitch);
  double sinPitch = sin(halfPitch);
  double cosRoll = cos(halfRoll);
  double sinRoll = sin(halfRoll);
  q.x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
  q.y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
  q.z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
  q.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
  return q;
}

}

#endif

