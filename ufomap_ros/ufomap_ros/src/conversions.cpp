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

// UFO ROS
#include <ufomap_ros/conversions.h>

// ROS
#include <sensor_msgs/point_cloud2_iterator.h>

namespace ufomap_ros
{
void getFields(sensor_msgs::PointCloud2 const& cloud, bool& has_x, bool& has_y,
               bool& has_z, bool& has_rgb)
{
	has_x = false;
	has_y = false;
	has_z = false;
	has_rgb = false;

	for (auto const& field : cloud.fields) {
		if ("x" == field.name) {
			has_x = true;
		} else if ("y" == field.name) {
			has_y = true;
		} else if ("z" == field.name) {
			has_z = true;
		} else if ("rgb" == field.name) {
			has_rgb = true;
		} else if ("r" == field.name) {
			has_rgb = true;
		} else if ("g" == field.name) {
			has_rgb = true;
		} else if ("b" == field.name) {
			has_rgb = true;
		}
	}
}

void rosToUfo(sensor_msgs::PointCloud2 const& cloud_in, ufo::map::PointCloud& cloud_out)
{
	cloud_out.reserve(cloud_in.data.size() / cloud_in.point_step);

	bool has_x, has_y, has_z, has_rgb;
	getFields(cloud_in, has_x, has_y, has_z, has_rgb);

	if (!has_x || !has_y || !has_z) {
		throw std::runtime_error("cloud_in missing one or more of the xyz fields");
	}

	sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_in, "x");
	sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_in, "y");
	sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_in, "z");
	for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
		if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
			cloud_out.push_back(ufo::map::Point3(*iter_x, *iter_y, *iter_z));
		}
	}
}

void rosToUfo(sensor_msgs::PointCloud2 const& cloud_in,
              ufo::map::PointCloudColor& cloud_out)
{
	cloud_out.reserve(cloud_in.data.size() / cloud_in.point_step);

	bool has_x, has_y, has_z, has_rgb;
	getFields(cloud_in, has_x, has_y, has_z, has_rgb);

	if (!has_x || !has_y || !has_z) {
		throw std::runtime_error("cloud_in missing one or more of the xyz fields");
	}

	sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_in, "x");
	sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_in, "y");
	sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_in, "z");

	if (has_rgb) {
		sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_r(cloud_in, "r");
		sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_g(cloud_in, "g");
		sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_b(cloud_in, "b");

		for (; iter_x != iter_x.end();
		     ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
			if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z) &&
			    !std::isnan(*iter_r) && !std::isnan(*iter_g) && !std::isnan(*iter_b)) {
				cloud_out.push_back(
				    ufo::map::Point3Color(*iter_x, *iter_y, *iter_z, *iter_r, *iter_g, *iter_b));
			}
		}

	} else {
		// TODO: Should this throw?
		// throw std::runtime_error("cloud_in missing one or more of the rgb fields");

		for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
			if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
				cloud_out.push_back(ufo::map::Point3Color(*iter_x, *iter_y, *iter_z));
			}
		}
	}
}

void ufoToRos(ufo::map::PointCloud const& cloud_in, sensor_msgs::PointCloud2& cloud_out)
{
	bool has_x, has_y, has_z, has_rgb;
	getFields(cloud_out, has_x, has_y, has_z, has_rgb);

	sensor_msgs::PointCloud2Modifier cloud_out_modifier(cloud_out);
	cloud_out_modifier.setPointCloud2FieldsByString(1, "xyz");
	cloud_out_modifier.resize(cloud_in.size());

	sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_out, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_out, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_out, "z");

	for (size_t i = 0; i < cloud_in.size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
		*iter_x = cloud_in[i][0];
		*iter_y = cloud_in[i][1];
		*iter_z = cloud_in[i][2];
	}
}

void ufoToRos(ufo::map::PointCloudColor const& cloud_in,
              sensor_msgs::PointCloud2& cloud_out)
{
	bool has_x, has_y, has_z, has_rgb;
	getFields(cloud_out, has_x, has_y, has_z, has_rgb);

	sensor_msgs::PointCloud2Modifier cloud_out_modifier(cloud_out);
	cloud_out_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
	cloud_out_modifier.resize(cloud_in.size());

	sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_out, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_out, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_out, "z");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_out, "r");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_out, "g");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_out, "b");

	for (size_t i = 0; i < cloud_in.size();
	     ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
		*iter_x = cloud_in[i][0];
		*iter_y = cloud_in[i][1];
		*iter_z = cloud_in[i][2];
		*iter_r = cloud_in[i].getColor().r;
		*iter_g = cloud_in[i].getColor().g;
		*iter_b = cloud_in[i].getColor().b;
	}
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