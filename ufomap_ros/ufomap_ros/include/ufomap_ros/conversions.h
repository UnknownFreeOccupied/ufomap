/*!
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author Daniel Duberg (dduberg@kth.se)
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * @version 1.0
 * @date 2022-05-13
 *
 * @copyright Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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
#include <ufo/map/point.h>
#include <ufo/map/point_cloud.h>
#include <ufo/math/pose6.h>
#include <ufo/math/quaternion.h>
#include <ufo/math/vector3.h>

// ROS
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/point_cloud_conversion.h>

// STL
#include <optional>

namespace ufomap_ros
{
std::optional<sensor_msgs::PointField> getField(sensor_msgs::PointCloud2 const& cloud,
                                                std::string const& field_name);

// Point clouds
template <class P>
void rosToUfo(sensor_msgs::PointCloud2 const& cloud_in,
              ufo::map::PointCloudT<P>& cloud_out, bool filter_nan = true)
{
	if (0 == cloud_in.point_step || 0 == cloud_in.row_step || 0 == cloud_in.height) {
		throw std::runtime_error("cloud_in point_step, height, and/or row_step is zero");
	}

	// Get all fields
	auto x_field = getField(cloud_in, "x");
	auto y_field = getField(cloud_in, "y");
	auto z_field = getField(cloud_in, "z");
	auto rgb_field = getField(cloud_in, "rgb");
	auto label_field = getField(cloud_in, "label");
	if (!label_field) {
		label_field = getField(cloud_in, "l");
	}
	auto value_field = getField(cloud_in, "value");
	if (!value_field) {
		value_field = getField(cloud_in, "v");
	}

	// There must be x,y,z values
	if (!x_field || !y_field || !z_field) {  // FIXME: Need to check if the are consecutive?
		throw std::runtime_error("cloud_in missing one or more of the xyz fields");
	}

	// FIXME: Add different depending on the type
	// Perhaps make them variants?
	sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_in, "x");
	std::optional<sensor_msgs::PointCloud2ConstIterator<uint8_t>> iter_rgb;
	std::optional<sensor_msgs::PointCloud2ConstIterator<uint32_t>> iter_label;
	std::optional<sensor_msgs::PointCloud2ConstIterator<float>> iter_value;

	// Create optional iteraters if wanted and available
	if constexpr (std::is_base_of_v<ufo::map::RGBColor, P>) {
		if (rgb_field) {
			iter_rgb.emplace(cloud_in, rgb_field->name);
		}
	}
	if constexpr (std::is_base_of_v<ufo::map::SemanticPair, P>) {
		if (label_field) {
			iter_label.emplace(cloud_in, label_field->name);
		}
		if (value_field) {
			iter_value.emplace(cloud_in, value_field->name);
		}
	}

	// Preallocate
	size_t index = cloud_out.size();
	cloud_out.resize(index + (cloud_in.data.size() / cloud_in.point_step));

	// Copy data
	for (; iter_x.end() != iter_x; ++iter_x) {
		if (!filter_nan ||
		    (!std::isnan(iter_x[0]) && !std::isnan(iter_x[1]) && !std::isnan(iter_x[2]))) {
			cloud_out[index].x = iter_x[0];
			cloud_out[index].y = iter_x[1];
			cloud_out[index].z = iter_x[2];

			if constexpr (std::is_base_of_v<ufo::map::RGBColor, P>) {
				if (rgb_field) {
					cloud_out[index].red = (*iter_rgb)[0];
					cloud_out[index].green = (*iter_rgb)[1];
					cloud_out[index].blue = (*iter_rgb)[2];
				}
			}
			if constexpr (std::is_base_of_v<ufo::map::SemanticPair, P>) {
				cloud_out[index].label = iter_label ? *(*iter_label) : 0;
				cloud_out[index].value = iter_value ? *(*iter_value) : 0;
			}
			++index;
		}

		// Increment optional iterators
		if constexpr (std::is_base_of_v<ufo::map::RGBColor, P>) {
			if (rgb_field) {
				++(*iter_rgb);
			}
		}
		if constexpr (std::is_base_of_v<ufo::map::SemanticPair, P>) {
			if (iter_label) {
				++(*iter_label);
			}
			if (iter_value) {
				++(*iter_value);
			}
		}
	}

	// Resize
	cloud_out.resize(index);
}

template <class P>
void ufoToRos(ufo::map::PointCloudT<P> const& cloud_in,
              sensor_msgs::PointCloud2& cloud_out)
{
	sensor_msgs::PointCloud2Modifier cloud_out_modifier(cloud_out);
	if constexpr (std::is_base_of_v<ufo::map::RGBColor, P> &&
	              std::is_base_of_v<ufo::map::SemanticPair, P>) {
		// clang-format off
		cloud_out_modifier.setPointCloud2Fields(6, 
														"x",     1, sensor_msgs::PointField::FLOAT32,
														"y",     1, sensor_msgs::PointField::FLOAT32,
														"z",     1, sensor_msgs::PointField::FLOAT32,
														"rgb",   1, sensor_msgs::PointField::UINT32,
														"label", 1, sensor_msgs::PointField::UINT32,
														"value", 1, sensor_msgs::PointField::FLOAT32);
		// clang-format on
	} else if constexpr (std::is_base_of_v<ufo::map::RGBColor, P>) {
		cloud_out_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
	} else if constexpr (std::is_base_of_v<ufo::map::SemanticPair, P>) {
		// clang-format off
			cloud_out_modifier.setPointCloud2Fields(5, 
                            "x",     1, sensor_msgs::PointField::FLOAT32,
                            "y",     1, sensor_msgs::PointField::FLOAT32,
                            "z",     1, sensor_msgs::PointField::FLOAT32,
                            "label", 1, sensor_msgs::PointField::UINT32,
                            "value", 1, sensor_msgs::PointField::FLOAT32);
		// clang-format on
	} else {
		cloud_out_modifier.setPointCloud2FieldsByString(1, "xyz");
	}

	cloud_out_modifier.resize(cloud_in.size());

	sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_out, "x");
	std::optional<sensor_msgs::PointCloud2Iterator<uint8_t>> iter_rgb;
	std::optional<sensor_msgs::PointCloud2Iterator<uint32_t>> iter_label;
	std::optional<sensor_msgs::PointCloud2Iterator<float>> iter_value;

	// Create optional iterators
	if constexpr (std::is_base_of_v<ufo::map::RGBColor, P>) {
		iter_rgb.emplace(cloud_out, "rgb");
	}
	if constexpr (std::is_base_of_v<ufo::map::SemanticPair, P>) {
		iter_label.emplace(cloud_out, "label");
		iter_value.emplace(cloud_out, "value");
	}

	for (auto const& point : cloud_in) {
		iter_x[0] = point.x;
		iter_x[1] = point.y;
		iter_x[2] = point.z;
		++iter_x;
		if constexpr (std::is_base_of_v<ufo::map::RGBColor, P>) {
			(*iter_rgb)[0] = point.red;
			(*iter_rgb)[1] = point.green;
			(*iter_rgb)[2] = point.blue;
			++(*iter_rgb);
		}
		if constexpr (std::is_base_of_v<ufo::map::SemanticPair, P>) {
			*(*iter_label) = point.label;
			*(*iter_value) = point.value;
			++(*iter_label);
			++(*iter_value);
		}
	}
}

// Vector3/Point
template <typename T>
constexpr void rosToUfo(geometry_msgs::Point const& point_in,
                        ufo::math::Vector3<T>& point_out)
{
	point_out.x = point_in.x;
	point_out.y = point_in.y;
	point_out.z = point_in.z;
}

template <typename T>
constexpr void rosToUfo(geometry_msgs::Vector3 const& point_in,
                        ufo::math::Vector3<T>& point_out)
{
	point_out.x = point_in.x;
	point_out.y = point_in.y;
	point_out.z = point_in.z;
}

constexpr ufo::math::Vector3d rosToUfo(geometry_msgs::Point const& point)
{
	return ufo::math::Vector3d(point.x, point.y, point.z);
}

constexpr ufo::math::Vector3d rosToUfo(geometry_msgs::Vector3 const& point)
{
	return ufo::math::Vector3d(point.x, point.y, point.z);
}

template <typename T>
constexpr void ufoToRos(ufo::math::Vector3<T> const& point_in,
                        geometry_msgs::Point& point_out)
{
	point_out.x = point_in.x;
	point_out.y = point_in.y;
	point_out.z = point_in.z;
}

template <typename T>
constexpr void ufoToRos(ufo::math::Vector3<T> const& point_in,
                        geometry_msgs::Vector3& point_out)
{
	point_out.x = point_in.x;
	point_out.y = point_in.y;
	point_out.z = point_in.z;
}

template <typename P, typename T>
constexpr P ufoToRos(ufo::math::Vector3<T> const& point)
{
	P p;
	ufoToRos(point, p);
	return p;
}

// Quaternion
template <typename T>
constexpr void rosToUfo(geometry_msgs::Quaternion const& quaternion_in,
                        ufo::math::Quaternion<T>& quaternion_out)
{
	quaternion_out.x = quaternion_in.x;
	quaternion_out.y = quaternion_in.y;
	quaternion_out.z = quaternion_in.z;
	quaternion_out.w = quaternion_in.w;
}

constexpr ufo::math::Quaterniond rosToUfo(geometry_msgs::Quaternion const& quaternion)
{
	return ufo::math::Quaterniond(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
}

template <typename T>
constexpr void ufoToRos(ufo::math::Quaternion<T> const& quaternion_in,
                        geometry_msgs::Quaternion& quaternion_out)
{
	quaternion_out.x = quaternion_in.x;
	quaternion_out.y = quaternion_in.y;
	quaternion_out.z = quaternion_in.z;
	quaternion_out.w = quaternion_in.w;
}

template <typename T>
geometry_msgs::Quaternion ufoToRos(ufo::math::Quaternion<T> const& quaternion)
{
	geometry_msgs::Quaternion q;
	ufoToRos(quaternion, q);
	return q;
}

// Transforms/Pose
template <typename T>
constexpr void rosToUfo(geometry_msgs::Transform const& transform_in,
                        ufo::math::Pose6<T>& transform_out)
{
	rosToUfo(transform_in.translation, transform_out.translation);
	rosToUfo(transform_in.rotation, transform_out.rotation);
}

template <typename T>
constexpr void rosToUfo(geometry_msgs::Pose const& pose_in, ufo::math::Pose6<T>& pose_out)
{
	rosToUfo(pose_in.position, pose_out.translation);
	rosToUfo(pose_in.orientation, pose_out.rotation);
}

constexpr ufo::math::Pose6d rosToUfo(geometry_msgs::Transform const& transform)
{
	return ufo::math::Pose6d(transform.translation.x, transform.translation.y,
	                         transform.translation.z, transform.rotation.w,
	                         transform.rotation.x, transform.rotation.y,
	                         transform.rotation.z);
}

constexpr ufo::math::Pose6d rosToUfo(geometry_msgs::Pose const& pose)
{
	return ufo::math::Pose6d(pose.position.x, pose.position.y, pose.position.z,
	                         pose.orientation.w, pose.orientation.x, pose.orientation.y,
	                         pose.orientation.z);
}

template <typename T>
constexpr void ufoToRos(ufo::math::Pose6<T> const& transform_in,
                        geometry_msgs::Transform& transform_out)
{
	ufoToRos(transform_in.translation, transform_out.translation);
	ufoToRos(transform_in.rotation, transform_out.rotation);
}

template <typename T>
constexpr void ufoToRos(ufo::math::Pose6<T> const& pose_in, geometry_msgs::Pose& pose_out)
{
	ufoToRos(pose_in.translation, pose_out.position);
	ufoToRos(pose_in.rotation, pose_out.orientation);
}

template <typename G, typename T>
constexpr G ufoToRos(ufo::math::Pose6<T> const& transform)
{
	G g;
	ufoToRos(transform, g);
	return g;
}
}  // namespace ufomap_ros

#endif  // UFOMAP_ROS_CONVERSIONS_H