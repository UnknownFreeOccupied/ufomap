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

bool intersectsLine(AABB const& aabb, Ray const& ray, float t_near, float t_far);
inline bool intersectsLine(AAEBB const& aaebb, Ray const& ray, float t_near, float t_far)
{
	Point min = aaebb.getMin();
	Point max = aaebb.getMax();

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
// Closest point
//

Point closestPoint(AABB const& aabb, Point const& point);
inline Point closestPoint(AAEBB const& aaebb, Point const& point)
{
	Point min = aaebb.getMin();
	Point max = aaebb.getMax();
	return Point::clamp(point, min, max);
}
Point closestPoint(LineSegment const& line_segement, Point const& point);
Point closestPoint(OBB const& obb, Point const& point);
Point closestPoint(Plane const& plane, Point const& point);
Point closestPoint(Ray const& ray, Point const& point);
Point closestPoint(Sphere const& sphere, Point const& point);

//
// Classify
//

float classify(AABB const& aabb, Plane const& plane);
inline float classify(AAEBB const& aaebb, Plane const& plane)
{
	float r = std::abs(aaebb.halfSize() * plane.normal.x()) +
	          std::abs(aaebb.halfSize() * plane.normal.y()) +
	          std::abs(aaebb.halfSize() * plane.normal.z());
	float d = Point::dot(plane.normal, aaebb.center()) + plane.distance;
	if (std::abs(d) < r) {
		return 0.0f;
	} else if (d < 0.0f) {
		return d + r;
	}
	return d - r;
}
float classify(OBB const& obb, Plane const& plane);

//
// Get interval
//

std::pair<float, float> getInterval(AABB const& aabb, Point const& axis);
inline std::pair<float, float> getInterval(AAEBB const& aaebb, Point const& axis)
{
	Point i = aaebb.getMin();
	Point a = aaebb.getMax();

	Point vertex[8] = {Point(i.x(), a.y(), a.z()), Point(i.x(), a.y(), i.z()),
	                   Point(i.x(), i.y(), a.z()), Point(i.x(), i.y(), i.z()),
	                   Point(a.x(), a.y(), a.z()), Point(a.x(), a.y(), i.z()),
	                   Point(a.x(), i.y(), a.z()), Point(a.x(), i.y(), i.z())};

	std::pair<float, float> result;
	result.first = result.second = Point::dot(axis, vertex[0]);

	for (int i = 1; i < 8; ++i) {
		float projection = Point::dot(axis, vertex[i]);
		result.first = std::min(result.first, projection);
		result.second = std::max(result.second, projection);
	}

	return result;
}
std::pair<float, float> getInterval(OBB const& obb, Point const& axis);

//
// Overlap on axis
//

bool overlapOnAxis(AABB const& aabb, OBB const& obb, Point const& axis);
inline bool overlapOnAxis(AAEBB const& aaebb, OBB const& obb, Point const& axis)
{
	auto [a_min, a_max] = getInterval(aaebb, axis);
	auto [b_min, b_max] = getInterval(obb, axis);
	return ((b_min <= a_max) && (a_min <= b_max));
}
bool overlapOnAxis(OBB const& obb_1, OBB const& obb_2, Point const& axis);

}  // namespace ufo::geometry

#endif  // UFO_GEOMETRY_HELPER_H