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

#ifndef UFO_GEOMETRY_MINIMUM_DISTANCE_H
#define UFO_GEOMETRY_MINIMUM_DISTANCE_H

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
// AABB
//

constexpr float minDistance(AABB const& aabb_1, AABB const& aabb_2)
{
	Point a_min = aabb_1.min();
	Point a_max = aabb_1.max();
	Point b_min = aabb_2.min();
	Point b_max = aabb_2.max();

	float result = 0.0;
	for (size_t i = 0; i != 3; ++i) {
		float d_1 = std::fdim(b_max[i], a_min[i]);
		float d_2 = std::fdim(a_max[i], b_min[i]);
		result += (d_1 * d_1) + (d_2 * d_2);

		// if (a_min[i] > b_max[i]) {
		// 	float delta = b_max[i] - a_min[i];
		// 	result += delta * delta;
		// } else if (b_min[i] > a_max[i]) {
		// 	float delta = a_max[i] - b_min[i];
		// 	result += delta * delta;
		// }
		// else the projection intervals overlap.
	}

	return std::sqrt(result);
}

inline float minDistance(AABB const& aabb, AAEBB const& aaebb)
{
	Point a_min = aabb.min();
	Point a_max = aabb.max();
	Point b_min = aaebb.min();
	Point b_max = aaebb.max();

	float result = 0.0;
	for (size_t i = 0; i != 3; ++i) {
		float d_1 = std::fdim(b_max[i], a_min[i]);
		float d_2 = std::fdim(a_max[i], b_min[i]);
		result += (d_1 * d_1) + (d_2 * d_2);

		// if (a_min[i] > b_max[i]) {
		// 	float delta = b_max[i] - a_min[i];
		// 	result += delta * delta;
		// } else if (b_min[i] > a_max[i]) {
		// 	float delta = a_max[i] - b_min[i];
		// 	result += delta * delta;
		// }
		// else the projection intervals overlap.
	}

	return std::sqrt(result);
}

// constexpr float minDistance(AABB const& aabb, Frustum const& frustum)
// {
// 	// TODO: Implement
// }

// constexpr float minDistance(AABB const& aabb, LineSegment const& line_segment)
// {
// 	// TODO: Implement
// }

// constexpr float minDistance(AABB const& aabb, OBB const& obb)
// {
// 	// TODO: Implement
// }

// constexpr float minDistance(AABB const& aabb, Plane const& plane)
// {
// 	// TODO: Implement
// }

constexpr float minDistance(AABB const& aabb, Point const& point)
{
	Point min = aabb.min();
	Point max = aabb.max();
	Point p = Point::clamp(point, min, max);
	return point.distance(p);
}

// constexpr float minDistance(AABB const& aabb, Ray const& ray)
// {
// 	// TODO: Implement
// }

constexpr float minDistance(AABB const& aabb, Sphere const& sphere)
{
	return std::fdim(minDistance(aabb, sphere.center), sphere.radius);
}

//
// AAEBB
//

constexpr float minDistance(AAEBB const& aaebb, AABB const& aabb)
{
	return minDistance(aabb, aaebb);
}

constexpr float minDistance(AAEBB const& aaebb_1, AAEBB const& aaebb_2)
{
	Point a_min = aaebb_1.min();
	Point a_max = aaebb_1.max();
	Point b_min = aaebb_2.min();
	Point b_max = aaebb_2.max();

	float result = 0.0;
	for (size_t i = 0; i != 3; ++i) {
		float d_1 = std::fdim(b_max[i], a_min[i]);
		float d_2 = std::fdim(a_max[i], b_min[i]);
		result += (d_1 * d_1) + (d_2 * d_2);

		// if (a_min[i] > b_max[i]) {
		// 	float delta = b_max[i] - a_min[i];
		// 	result += delta * delta;
		// } else if (b_min[i] > a_max[i]) {
		// 	float delta = a_max[i] - b_min[i];
		// 	result += delta * delta;
		// }
		// else the projection intervals overlap.
	}

	return std::sqrt(result);
}

// constexpr float minDistance(AAEBB const& aaebb, Frustum const& frustum)
// {
// 	// TODO: Implement
// }

// constexpr float minDistance(AAEBB const& aaebb, LineSegment const& line_segment)
// {
// 	// TODO: Implement
// }

// constexpr float minDistance(AAEBB const& aaebb, OBB const& obb)
// {
// 	// TODO: Implement
// }

// constexpr float minDistance(AAEBB const& aaebb, Plane const& plane)
// {
// 	// TODO: Implement
// }

constexpr float minDistance(AAEBB const& aaebb, Point const& point)
{
	Point min = aaebb.min();
	Point max = aaebb.max();
	Point p = Point::clamp(point, min, max);
	return point.distance(p);
}

// constexpr float minDistance(AAEBB const& aaebb, Ray const& ray)
// {
// 	// TODO: Implement
}

constexpr float minDistance(AAEBB const& aaebb, Sphere const& sphere)
{
	return std::fdim(minDistance(aaebb, sphere.center), sphere.radius);
}

constexpr float minDistanceSquared(AAEBB const& aaebb, AABB const& aabb)
{
	Point a_min = aaebb.min();
	Point a_max = aaebb.max();
	Point b_min = aabb.min();
	Point b_max = aabb.max();

	float result = 0.0;
	for (size_t i = 0; i != 3; ++i) {
		float d_1 = std::fdim(b_max[i], a_min[i]);
		float d_2 = std::fdim(a_max[i], b_min[i]);
		result += (d_1 * d_1) + (d_2 * d_2);

		// if (a_min[i] > b_max[i]) {
		// 	float delta = b_max[i] - a_min[i];
		// 	result += delta * delta;
		// } else if (b_min[i] > a_max[i]) {
		// 	float delta = a_max[i] - b_min[i];
		// 	result += delta * delta;
		// }
		// else the projection intervals overlap.
	}

	return result;
}

constexpr float minDistanceSquared(AAEBB const& aaebb_1, AAEBB const& aaebb_2)
{
	// FIXME: Is this actually correct?
	// Point a;
	// for (size_t i = 0; i != 3; ++i) {
	// 	a[i] = std::max(0.0f, (aaebb_1.center[i] - aaebb_1.half_size) -
	// 	                          (aaebb_2.center[i] + aaebb_2.half_size));
	// }
	// float result = a.squaredNorm();
	// for (size_t i = 0; i != 3; ++i) {
	// 	a[i] = std::max(0.0f, (aaebb_2.center[i] - aaebb_2.half_size) -
	// 	                          (aaebb_1.center[i] + aaebb_1.half_size));
	// }
	// return result + a.squaredNorm();

	float hs = aaebb_1.half_size + aaebb_2.half_size;
	float result = 0.0f;
	for (size_t i = 0; i != 3; ++i) {
		float tmp = std::fdim(std::abs(aaebb_1[i] - aaebb_2[i]), hs);
		result += tmp * tmp;
	}
	return result;

	// float res = 0.0f;
	// for (size_t i = 0; i != 3; ++i) {
	// 	float tmp = std::max(0.0f, std::max((aaebb_1.center[i] - aaebb_1.half_size) -
	// 	                                        (aaebb_2.center[i] + aaebb_2.half_size),
	// 	                                    (aaebb_2.center[i] - aaebb_2.half_size) -
	// 	                                        (aaebb_1.center[i] + aaebb_1.half_size)));
	// 	res += tmp * tmp;
	// }
	// return res;

	// Point a = aaebb_1.getMin() - aaebb_2.getMax();
	// Point b = aaebb_2.getMin() - aaebb_1.getMax();

	// float result = 0.0;
	// for (size_t i = 0; i != 3; ++i) {
	// 	float d_1 = std::max(0.0f, a[i]);
	// 	float d_2 = std::max(0.0f, b[i]);
	// 	result += (d_1 * d_1) + (d_2 * d_2);
	// }

	// return result;
}
// float minDistanceSquared(AAEBB const& aaebb, Frustum const& frustum);
// float minDistanceSquared(AAEBB const& aaebb, LineSegment const& line_segment);
// float minDistanceSquared(AAEBB const& aaebb, OBB const& obb);
// float minDistanceSquared(AAEBB const& aaebb, Plane const& plane);

inline float minDistanceSquared(AAEBB const& aaebb, Point const& point)
{
	Point min = aaebb.getMin();
	Point max = aaebb.getMax();
	Point p = Point::clamp(point, min, max);
	return point.squaredDistance(p);
}

// float minDistanceSquared(AAEBB const& aaebb, Ray const& ray);
inline float minDistanceSquared(AAEBB const& aaebb, Sphere const& sphere)
{
	// FIXME: Implement better
	float dist = minDistance(aaebb, sphere);
	return dist * dist;
}

//
// Frustum
//

// constexpr float minDistance(Frustum const& frustum, AABB const& aabb);
// constexpr float minDistance(Frustum const& frustum, AAEBB const& aaebb);
// constexpr float minDistance(Frustum const& frustum_1, Frustum const& frustum_2);
// constexpr float minDistance(Frustum const& frustum, LineSegment const& line_segment);
// constexpr float minDistance(Frustum const& frustum, OBB const& obb);
// constexpr float minDistance(Frustum const& frustum, Plane const& plane);
// constexpr float minDistance(Frustum const& frustum, Point const& point);
// constexpr float minDistance(Frustum const& frustum, Ray const& ray);
// constexpr float minDistance(Frustum const& frustum, Sphere const& sphere);

//
// Line segment
//

// constexpr float minDistance(LineSegment const& line_segment, AABB const& aabb);
// constexpr float minDistance(LineSegment const& line_segment, AAEBB const& aaebb);
// constexpr float minDistance(LineSegment const& line_segment, Frustum const& frustum);
// constexpr float minDistance(LineSegment const& line_segment_1, LineSegment const&
// line_segment_2); 
// constexpr float minDistance(LineSegment const& line_segment, OBB const& obb);
// constexpr float minDistance(LineSegment const& line_segment, Plane const& plane);
// constexpr float minDistance(LineSegment const& line_segment, Point const& point);
// constexpr float minDistance(LineSegment const& line_segment, Ray const& ray);
// constexpr float minDistance(LineSegment const& line_segment, Sphere const& sphere);

//
// OBB
//

// constexpr float minDistance(OBB const& obb, AABB const& aabb);
// constexpr float minDistance(OBB const& obb, AAEBB const& aaebb);
// constexpr float minDistance(OBB const& obb, Frustum const& frustum);
// constexpr float minDistance(OBB const& obb, LineSegment const& line_segment);
// constexpr float minDistance(OBB const& obb_1, OBB const& obb_2);
// constexpr float minDistance(OBB const& obb, Plane const& plane);
// constexpr float minDistance(OBB const& obb, Point const& point);
// constexpr float minDistance(OBB const& obb, Ray const& ray);
// constexpr float minDistance(OBB const& obb, Sphere const& sphere);

//
// Plane
//

// constexpr float minDistance(Plane const& plane, AABB const& aabb);
// constexpr float minDistance(Plane const& plane, AAEBB const& aaebb);
// constexpr float minDistance(Plane const& plane, Frustum const& frustum);
// constexpr float minDistance(Plane const& plane, LineSegment const& line_segment);
// constexpr float minDistance(Plane const& plane, OBB const& obb);
// constexpr float minDistance(Plane const& plane_1, Plane const& plane_2);
// constexpr float minDistance(Plane const& plane, Point const& point);
// constexpr float minDistance(Plane const& plane, Ray const& ray);
// constexpr float minDistance(Plane const& plane, Sphere const& sphere);

//
// Point
//

float minDistance(Point const& point, AABB const& aabb);
float minDistance(Point const& point, AAEBB const& aaebb);
// float minDistance(Point const& point, Frustum const& frustum);
// float minDistance(Point const& point, LineSegment const& line_segment);
// float minDistance(Point const& point, OBB const& obb);
// float minDistance(Point const& point, Plane const& plane);
float minDistance(Point const& point_1, Point const& point_2);
// float minDistance(Point const& point, Ray const& ray);
float minDistance(Point const& point, Sphere const& sphere);

//
// Ray
//

// constexpr float minDistance(Ray const& ray, AABB const& aabb);
// constexpr float minDistance(Ray const& ray, AAEBB const& aaebb);
// constexpr float minDistance(Ray const& ray, Frustum const& frustum);
// constexpr float minDistance(Ray const& ray, LineSegment const& line_segment);
// constexpr float minDistance(Ray const& ray, OBB const& obb);
// constexpr float minDistance(Ray const& ray, Plane const& plane);
// constexpr float minDistance(Ray const& ray, Point const& point);
// constexpr float minDistance(Ray const& ray_1, Ray const& ray_2);
// constexpr float minDistance(Ray const& ray, Sphere const& sphere);

//
// Sphere
//

float minDistance(Sphere const& sphere, AABB const& aabb);
float minDistance(Sphere const& sphere, AAEBB const& aaebb);
// float minDistance(Sphere const& sphere, Frustum const& frustum);
// float minDistance(Sphere const& sphere, LineSegment const& line_segment);
// float minDistance(Sphere const& sphere, OBB const& obb);
// float minDistance(Sphere const& sphere, Plane const& plane);
float minDistance(Sphere const& sphere, Point const& point);
// float minDistance(Sphere const& sphere, Ray const& ray);
float minDistance(Sphere const& sphere_1, Sphere const& sphere_2);

}  // namespace ufo::geometry

#endif  // UFO_GEOMETRY_MINIMUM_DISTANCE_H