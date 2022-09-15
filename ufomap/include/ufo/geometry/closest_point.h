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

#ifndef UFO_GEOMETRY_CLOSEST_POINT_H
#define UFO_GEOMETRY_CLOSEST_POINT_H

// UFO
#include <ufo/geometry/bounding_volume.h>

// STL
#include <algorithm>

namespace ufo::geometry
{
//
// AABB
//

constexpr Point closestPoint(AABB const& a, Point b) noexcept
{
	return b.clamp(a.min(), a.max());
}

//
// AAEBB
//

constexpr Point closestPoint(AAEBB a, Point b) noexcept
{
	return b.clamp(a.min(), a.max());
}

//
// Line segment
//

constexpr Point closestPoint(LineSegment const& line_segement, Point point) noexcept
{
	Point direction = line_segement.end - line_segement.start;
	float t = Point::dot(point - line_segement.start, direction) /
	          Point::dot(direction, direction);
	return line_segement.start + direction * std::clamp(t, 0.0f, 1.0f);
}

//
// OBB
//

constexpr Point closestPoint(OBB const& obb, Point point) noexcept
{
	Point result = obb.center;
	Point dir = point - obb.center;

	std::array<float, 9> obb_rot_matrix = obb.rotation.getRotMatrix();

	for (int i = 0; i < 3; ++i) {
		Point axis(obb_rot_matrix[i * 3], obb_rot_matrix[(i * 3) + 1],
		           obb_rot_matrix[(i * 3) + 2]);
		float distance = Point::dot(dir, axis);
		if (distance > obb.half_size[i]) {
			distance = obb.half_size[i];
		}
		if (distance < -obb.half_size[i])  // FIXME: Should this be else if?
		{
			distance = -obb.half_size[i];
		}
		result = result + (axis * distance);
	}
	return result;
}

//
// Plane
//

constexpr Point closestPoint(Plane const& plane, Point point) noexcept
{
	float distance = Point::dot(plane.normal, point) - plane.distance;
	return point - plane.normal * distance;
}

//
// Ray
//

constexpr Point closestPoint(Ray const& ray, Point point) noexcept
{
	float t = Point::dot(point - ray.origin, ray.direction);
	return ray.origin + ray.direction * std::max(t, 0.0f);
}

//
// Sphere
//

constexpr Point closestPoint(Sphere const& sphere, Point point) noexcept
{
	Point sphere_to_point = point - sphere.center;
	sphere_to_point.normalize();
	sphere_to_point *= sphere.radius;
	return sphere_to_point + sphere.center;
}
}  // namespace ufo::geometry

#endif  // UFO_GEOMETRY_CLOSEST_POINT_H