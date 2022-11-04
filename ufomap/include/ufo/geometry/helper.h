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

#ifndef UFO_GEOMETRY_HELPER_H
#define UFO_GEOMETRY_HELPER_H

// UFO
#include <ufo/geometry/aabb.h>
#include <ufo/geometry/aaebb.h>
#include <ufo/geometry/frustum.h>
#include <ufo/geometry/line_segment.h>
#include <ufo/geometry/obb.h>
#include <ufo/geometry/plane.h>
#include <ufo/geometry/point.h>
#include <ufo/geometry/ray.h>
#include <ufo/geometry/sphere.h>

namespace ufo::geometry
{
//
// Intersects line
//

constexpr bool intersectsLine(AABB const& aabb, Ray const& ray, float t_near,
                              float t_far) noexcept
{
	Point min = aabb.min();
	Point max = aabb.max();

	for (int i = 0; i < 3; ++i) {
		if (0 != ray.direction[i]) {
			float reciprocal_direction = 1.0 / ray.direction[i];
			float t1 = (min[i] - ray.origin[i]) * reciprocal_direction;
			float t2 = (max[i] - ray.origin[i]) * reciprocal_direction;

			if (t1 < t2) {
				t_near = std::max(t1, t_near);
				t_far = std::min(t2, t_far);
			} else {
				t_near = std::max(t2, t_near);
				t_far = std::min(t1, t_far);
			}

			if (t_near > t_far) {
				return false;
			}
		} else if (min[i] > ray.origin[i] || max[i] < ray.origin[i]) {
			return false;
		}
	}
	return true;
}

constexpr bool intersectsLine(AAEBB aaebb, Ray const& ray, float t_near,
                              float t_far) noexcept
{
	Point min = aaebb.min();
	Point max = aaebb.max();

	for (int i = 0; i < 3; ++i) {
		if (0 != ray.direction[i]) {
			float reciprocal_direction = 1.0 / ray.direction[i];
			float t1 = (min[i] - ray.origin[i]) * reciprocal_direction;
			float t2 = (max[i] - ray.origin[i]) * reciprocal_direction;

			if (t1 < t2) {
				t_near = std::max(t1, t_near);
				t_far = std::min(t2, t_far);
			} else {
				t_near = std::max(t2, t_near);
				t_far = std::min(t1, t_far);
			}

			if (t_near > t_far) {
				return false;
			}
		} else if (min[i] > ray.origin[i] || max[i] < ray.origin[i]) {
			return false;
		}
	}
	return true;
}

//
// Classify
//

constexpr float classify(AABB const& aabb, Plane const& plane) noexcept
{
	float r = std::abs(aabb.half_size.x * plane.normal.x) +
	          std::abs(aabb.half_size.y * plane.normal.y) +
	          std::abs(aabb.half_size.z * plane.normal.z);
	float d = Point::dot(plane.normal, aabb.center) + plane.distance;
	if (std::abs(d) < r) {
		return 0;
	} else if (d < 0) {
		return d + r;
	}
	return d - r;
}

constexpr float classify(AAEBB aaebb, Plane const& plane) noexcept
{
	float r = std::abs(aaebb.half_size * plane.normal.x) +
	          std::abs(aaebb.half_size * plane.normal.y) +
	          std::abs(aaebb.half_size * plane.normal.z);
	float d = Point::dot(plane.normal, aaebb.center) + plane.distance;
	if (std::abs(d) < r) {
		return 0;
	} else if (d < 0) {
		return d + r;
	}
	return d - r;
}

// constexpr float classify(OBB const& obb, Plane const& plane) noexcept
// {
// 	// FIXME: Check if correct
// 	Point normal = plane.normal * obb.rotation;
// 	float r = std::abs(obb.half_size.x() * normal.x()) +
// 	          std::abs(obb.half_size.y() * normal.y()) +
// 	          std::abs(obb.half_size.z() * normal.z());
// 	float d = Point::dot(plane.normal, obb.center) + plane.distance;
// 	if (std::abs(d) < r) {
// 		return 0;
// 	} else if (d < 0) {
// 		return d + r;
// 	}
// 	return d - r;
// }

//
// Get interval
//

constexpr std::pair<float, float> getInterval(AABB const& aabb, Point axis) noexcept
{
	Point i = aabb.min();
	Point a = aabb.max();

	Point vertex[8] = {Point(i.x, a.y, a.z), Point(i.x, a.y, i.z), Point(i.x, i.y, a.z),
	                   Point(i.x, i.y, i.z), Point(a.x, a.y, a.z), Point(a.x, a.y, i.z),
	                   Point(a.x, i.y, a.z), Point(a.x, i.y, i.z)};

	std::pair<float, float> result;
	result.first = result.second = Point::dot(axis, vertex[0]);

	for (int i = 1; i < 8; ++i) {
		float projection = Point::dot(axis, vertex[i]);
		result.first = std::min(result.first, projection);
		result.second = std::max(result.second, projection);
	}

	return result;
}

constexpr std::pair<float, float> getInterval(AAEBB aaebb, Point axis) noexcept
{
	Point i = aaebb.min();
	Point a = aaebb.max();

	Point vertex[8] = {Point(i.x, a.y, a.z), Point(i.x, a.y, i.z), Point(i.x, i.y, a.z),
	                   Point(i.x, i.y, i.z), Point(a.x, a.y, a.z), Point(a.x, a.y, i.z),
	                   Point(a.x, i.y, a.z), Point(a.x, i.y, i.z)};

	std::pair<float, float> result;
	result.first = result.second = Point::dot(axis, vertex[0]);

	for (int i = 1; i < 8; ++i) {
		float projection = Point::dot(axis, vertex[i]);
		result.first = std::min(result.first, projection);
		result.second = std::max(result.second, projection);
	}

	return result;
}

constexpr std::pair<float, float> getInterval(OBB const& obb, Point axis) noexcept
{
	Point vertex[8];

	Point C = obb.center;     // OBB Center
	Point E = obb.half_size;  // OBB Extents

	std::array<float, 9> obb_rot_matrix = obb.rotation.rotMatrix();

	Point A[] = {
	    // OBB Axis
	    Point(obb_rot_matrix[0], obb_rot_matrix[1], obb_rot_matrix[2]),
	    Point(obb_rot_matrix[3], obb_rot_matrix[4], obb_rot_matrix[5]),
	    Point(obb_rot_matrix[6], obb_rot_matrix[7], obb_rot_matrix[8]),
	};

	vertex[0] = C + A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
	vertex[1] = C - A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
	vertex[2] = C + A[0] * E[0] - A[1] * E[1] + A[2] * E[2];
	vertex[3] = C + A[0] * E[0] + A[1] * E[1] - A[2] * E[2];
	vertex[4] = C - A[0] * E[0] - A[1] * E[1] - A[2] * E[2];
	vertex[5] = C + A[0] * E[0] - A[1] * E[1] - A[2] * E[2];
	vertex[6] = C - A[0] * E[0] + A[1] * E[1] - A[2] * E[2];
	vertex[7] = C - A[0] * E[0] - A[1] * E[1] + A[2] * E[2];

	std::pair<float, float> result;
	result.first = result.second = Point::dot(axis, vertex[0]);

	for (int i = 1; i < 8; ++i) {
		float projection = Point::dot(axis, vertex[i]);
		result.first = std::min(result.first, projection);
		result.second = std::max(result.second, projection);
	}

	return result;
}

//
// Overlap on axis
//

constexpr bool overlapOnAxis(AABB const& aabb, OBB const& obb, Point axis) noexcept
{
	auto [a_min, a_max] = getInterval(aabb, axis);
	auto [b_min, b_max] = getInterval(obb, axis);
	return ((b_min <= a_max) && (a_min <= b_max));
}

constexpr bool overlapOnAxis(AAEBB aaebb, OBB const& obb, Point axis) noexcept
{
	auto [a_min, a_max] = getInterval(aaebb, axis);
	auto [b_min, b_max] = getInterval(obb, axis);
	return ((b_min <= a_max) && (a_min <= b_max));
}

constexpr bool overlapOnAxis(OBB const& obb_1, OBB const& obb_2, Point axis) noexcept
{
	auto [a_min, a_max] = getInterval(obb_1, axis);
	auto [b_min, b_max] = getInterval(obb_2, axis);
	return ((b_min <= a_max) && (a_min <= b_max));
}

}  // namespace ufo::geometry

#endif  // UFO_GEOMETRY_HELPER_H