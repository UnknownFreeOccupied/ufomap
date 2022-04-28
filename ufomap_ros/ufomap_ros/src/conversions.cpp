/*
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * License: BSD 3
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

// UFO ROS
#include <ufomap_ros/conversions.h>

namespace ufomap_ros
{
std::optional<sensor_msgs::PointField> getField(sensor_msgs::PointCloud2 const& cloud,
                                                std::string const& field_name)
{
	auto idx = sensor_msgs::getPointCloud2FieldIndex(cloud, field_name);
	return 0 > idx ? std::nullopt
	               : std::optional<sensor_msgs::PointField>{cloud.fields[idx]};
}

// Vector3/Point

void rosToUfo(geometry_msgs::Point const& point_in, ufo::math::Vector3& point_out)
{
	point_out.x() = point_in.x;
	point_out.y() = point_in.y;
	point_out.z() = point_in.z;
}

void rosToUfo(geometry_msgs::Vector3 const& point_in, ufo::math::Vector3& point_out)
{
	point_out.x() = point_in.x;
	point_out.y() = point_in.y;
	point_out.z() = point_in.z;
}

ufo::math::Vector3 rosToUfo(geometry_msgs::Point const& point)
{
	return ufo::math::Vector3(point.x, point.y, point.z);
}

ufo::math::Vector3 rosToUfo(geometry_msgs::Vector3 const& point)
{
	return ufo::math::Vector3(point.x, point.y, point.z);
}

void ufoToRos(ufo::math::Vector3 const& point_in, geometry_msgs::Point& point_out)
{
	point_out.x = point_in.x();
	point_out.y = point_in.y();
	point_out.z = point_in.z();
}

void ufoToRos(ufo::math::Vector3 const& point_in, geometry_msgs::Vector3& point_out)
{
	point_out.x = point_in.x();
	point_out.y = point_in.y();
	point_out.z = point_in.z();
}

geometry_msgs::Point ufoToRos(ufo::math::Vector3 const& point)
{
	geometry_msgs::Point point_out;
	ufoToRos(point, point_out);
	return point_out;
}

// Quaternion
void rosToUfo(geometry_msgs::Quaternion const& quaternion_in,
              ufo::math::Quaternion& quaternion_out)
{
	quaternion_out.x() = quaternion_in.x;
	quaternion_out.y() = quaternion_in.y;
	quaternion_out.z() = quaternion_in.z;
	quaternion_out.w() = quaternion_in.w;
}

ufo::math::Quaternion rosToUfo(geometry_msgs::Quaternion const& quaternion)
{
	return ufo::math::Quaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
}

void ufoToRos(ufo::math::Quaternion const& quaternion_in,
              geometry_msgs::Quaternion& quaternion_out)
{
	quaternion_out.x = quaternion_in.x();
	quaternion_out.y = quaternion_in.y();
	quaternion_out.z = quaternion_in.z();
	quaternion_out.w = quaternion_in.w();
}

geometry_msgs::Quaternion ufoToRos(ufo::math::Quaternion const& quaternion)
{
	geometry_msgs::Quaternion quaternion_out;
	ufoToRos(quaternion, quaternion_out);
	return quaternion_out;
}

// Transforms
void rosToUfo(geometry_msgs::Transform const& transform_in,
              ufo::math::Pose6& transform_out)
{
	rosToUfo(transform_in.translation, transform_out.translation());
	rosToUfo(transform_in.rotation, transform_out.rotation());
}

ufo::math::Pose6 rosToUfo(geometry_msgs::Transform const& transform)
{
	return ufo::math::Pose6(transform.translation.x, transform.translation.y,
	                        transform.translation.z, transform.rotation.w,
	                        transform.rotation.x, transform.rotation.y,
	                        transform.rotation.z);
}

void ufoToRos(ufo::math::Pose6 const& transform_in,
              geometry_msgs::Transform& transform_out)
{
	ufoToRos(transform_in.translation(), transform_out.translation);
	ufoToRos(transform_in.rotation(), transform_out.rotation);
}

geometry_msgs::Transform ufoToRos(ufo::math::Pose6 const& transform)
{
	geometry_msgs::Transform transform_out;
	ufoToRos(transform, transform_out);
	return transform_out;
}
}  // namespace ufomap_ros