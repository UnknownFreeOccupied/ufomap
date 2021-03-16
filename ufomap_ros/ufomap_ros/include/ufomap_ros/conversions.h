/**
 * UFO ROS integration
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomap_ros
 * License: BSD 3
 *
 */

/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, D. Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef UFOMAP_ROS_CONVERSIONS_H
#define UFOMAP_ROS_CONVERSIONS_H

// UFO
#include <ufo/map/point_cloud.h>
#include <ufo/math/pose6.h>
#include <ufo/math/quaternion.h>
#include <ufo/math/vector3.h>

// ROS
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>

namespace ufomap_ros
{
// Point clouds
void rosToUfo(sensor_msgs::PointCloud2 const& cloud_in, ufo::map::PointCloud& cloud_out);

void rosToUfo(sensor_msgs::PointCloud2 const& cloud_in,
              ufo::map::PointCloudColor& cloud_out);

void ufoToRos(ufo::map::PointCloud const& cloud_in, sensor_msgs::PointCloud2& cloud_out);

void ufoToRos(ufo::map::PointCloudColor const& cloud_in,
              sensor_msgs::PointCloud2& cloud_out);

// Vector3/Point
void rosToUfo(geometry_msgs::Point const& point_in, ufo::math::Vector3& point_out);

void rosToUfo(geometry_msgs::Vector3 const& point_in, ufo::math::Vector3& point_out);

ufo::math::Vector3 rosToUfo(geometry_msgs::Point const& point);

void ufoToRos(ufo::math::Vector3 const& point_in, geometry_msgs::Point& point_out);

void ufoToRos(ufo::math::Vector3 const& point_in, geometry_msgs::Vector3& point_out);

geometry_msgs::Point ufoToRos(ufo::math::Vector3 const& point);

// Quaternion
void rosToUfo(geometry_msgs::Quaternion const& quaternion_in,
              ufo::math::Quaternion& quaternion_out);

ufo::math::Quaternion rosToUfo(const geometry_msgs::Quaternion& quaternion);

void ufoToRos(ufo::math::Quaternion const& quaternion_in,
              geometry_msgs::Quaternion& quaternion_out);

geometry_msgs::Quaternion ufoToRos(ufo::math::Quaternion const& quaternion);

// Transforms
void rosToUfo(geometry_msgs::Transform const& transform_in,
              ufo::math::Pose6& transform_out);

ufo::math::Pose6 rosToUfo(geometry_msgs::Transform const& transform);

void ufoToRos(ufo::math::Pose6 const& transform_in,
              geometry_msgs::Transform& transform_out);

geometry_msgs::Transform ufoToRos(ufo::math::Pose6 const& transform);
}  // namespace ufomap_ros

#endif  // UFOMAP_ROS_CONVERSIONS_H