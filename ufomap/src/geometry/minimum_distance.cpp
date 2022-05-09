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

// UFO
#include <ufo/geometry/minimum_distance.h>

namespace ufo::geometry
{
//
// AABB
//

float minDistance(AABB const& aabb_1, AABB const& aabb_2)
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

// float minDistance(AABB const& aabb, AAEBB const& aaebb)
// {
// 	Point a_min = aabb.min();
// 	Point a_max = aabb.max();
// 	Point b_min = aaebb.min();
// 	Point b_max = aaebb.max();

// 	float result = 0.0;
// 	for (size_t i = 0; i != 3; ++i) {
// 		float d_1 = std::max(0.0f, a_min[i] - b_max[i]);
// 		float d_2 = std::max(0.0f, b_min[i] - a_max[i]);
// 		result += (d_1 * d_1) + (d_2 * d_2);

// 		// if (a_min[i] > b_max[i]) {
// 		// 	float delta = b_max[i] - a_min[i];
// 		// 	result += delta * delta;
// 		// } else if (b_min[i] > a_max[i]) {
// 		// 	float delta = a_max[i] - b_min[i];
// 		// 	result += delta * delta;
// 		// }
// 		// else the projection intervals overlap.
// 	}

// 	return std::sqrt(result);
// }

// float minDistance(AABB const& aabb, Frustum const& frustum)
// {
// 	// TODO: Implement
// }

// float minDistance(AABB const& aabb, LineSegment const& line_segment)
// {
// 	// TODO: Implement
// }

// float minDistance(AABB const& aabb, OBB const& obb)
// {
// 	// TODO: Implement
// }

// float minDistance(AABB const& aabb, Plane const& plane)
// {
// 	// TODO: Implement
// }

float minDistance(AABB const& aabb, Point const& point)
{
	Point min = aabb.min();
	Point max = aabb.max();
	Point p = Point::clamp(point, min, max);
	return point.distance(p);
}

// float minDistance(AABB const& aabb, Ray const& ray)
// {
// 	// TODO: Implement
// }

float minDistance(AABB const& aabb, Sphere const& sphere)
{
	return std::fdim(minDistance(aabb, sphere.center), sphere.radius);
}

//
// AAEBB
//

// float minDistance(AAEBB const& aaebb, AABB const& aabb)
// {
// 	return minDistance(aabb, aaebb);
// }

// float minDistance(AAEBB const& aaebb_1, AAEBB const& aaebb_2)
// {
// 	Point a_min = aaebb_1.min();
// 	Point a_max = aaebb_1.max();
// 	Point b_min = aaebb_2.min();
// 	Point b_max = aaebb_2.max();

// 	float result = 0.0;
// 	for (size_t i = 0; i != 3; ++i) {
// 		float d_1 = std::max(0.0f, a_min[i] - b_max[i]);
// 		float d_2 = std::max(0.0f, b_min[i] - a_max[i]);
// 		result += (d_1 * d_1) + (d_2 * d_2);

// 		// if (a_min[i] > b_max[i]) {
// 		// 	float delta = b_max[i] - a_min[i];
// 		// 	result += delta * delta;
// 		// } else if (b_min[i] > a_max[i]) {
// 		// 	float delta = a_max[i] - b_min[i];
// 		// 	result += delta * delta;
// 		// }
// 		// else the projection intervals overlap.
// 	}

// 	return std::sqrt(result);
// }

// float minDistance(AAEBB const& aaebb, Frustum const& frustum)
// {
// 	// TODO: Implement
// }

// float minDistance(AAEBB const& aaebb, LineSegment const& line_segment)
// {
// 	// TODO: Implement
// }

// float minDistance(AAEBB const& aaebb, OBB const& obb)
// {
// 	// TODO: Implement
// }

// float minDistance(AAEBB const& aaebb, Plane const& plane)
// {
// 	// TODO: Implement
// }

// float minDistance(AAEBB const& aaebb, Point const& point)
// {
// 	Point min = aaebb.min();
// 	Point max = aaebb.max();
// 	Point p = Point::clamp(point, min, max);
// 	return point.distance(p);
// }

// float minDistance(AAEBB const& aabb, Ray const& ray)
// {
// 	// TODO: Implement
// }

// float minDistance(AAEBB const& aaebb, Sphere const& sphere)
// {
// 	return std::max(0.0f, minDistance(aaebb, sphere.center) - sphere.radius);
// }

//
// Frustum
//

// float minDistance(Frustum const& frustum, AABB const& aabb)
// {
// 	// TODO: Enable
// 	return minDistance(aabb, frustum);
// }

// float minDistance(Frustum const& frustum, AAEBB const& aaebb)
// {
// 	// TODO: Enable
// 	return minDistance(aaebb, frustum);
// }

// float minDistance(Frustum const& frustum_1, Frustum const& frustum_2)
// {
// 	// TODO: Implement
// }

// float minDistance(Frustum const& frustum, LineSegment const& line_segment)
// {
// 	// TODO: Implement
// }

// float minDistance(Frustum const& frustum, OBB const& obb)
// {
// 	// TODO: Implement
// }

// float minDistance(Frustum const& frustum, Plane const& plane)
// {
// 	// TODO: Implement
// }

// float minDistance(Frustum const& frustum, Point const& point)
// {
// 	// TODO: Implement
// }

// float minDistance(Frustum const& frustum, Ray const& ray)
// {
// 	// TODO: Implement
// }

// float minDistance(Frustum const& frustum, Sphere const& sphere)
// {
// 	// TODO: Implement
// }

//
// Line segment
//

// float minDistance(LineSegment const& line_segment, AABB const& aabb)
// {
// 	// TODO: Enable
// 	return minDistance(aabb, line_segment);
// }

// float minDistance(LineSegment const& line_segment, AAEBB const& aaebb)
// {
// 	// TODO: Enable
// 	return minDistance(aaebb, line_segment);
// }

// float minDistance(LineSegment const& line_segment, Frustum const& frustum)
// {
// 	// TODO: Enable
// 	return minDistance(frustum, line_segment);
// }

// float minDistance(LineSegment const& line_segment_1, LineSegment const&
// line_segment_2)
// {
// 	// TODO: Implement
// }

// float minDistance(LineSegment const& line_segment, OBB const& obb)
// {
// 	// TODO: Implement
// }

// float minDistance(LineSegment const& line_segment, Plane const& plane)
// {
// 	// TODO: Implement
// }

// float minDistance(LineSegment const& line_segment, Point const& point)
// {
// 	// TODO: Implement
// }

// float minDistance(LineSegment const& line_segment, Ray const& ray)
// {
// 	// TODO: Implement
// }

// float minDistance(LineSegment const& line_segment, Sphere const& sphere)
// {
// 	// TODO: Implement
// }

//
// OBB
//

// float minDistance(OBB const& obb, AABB const& aabb)
// {
// 	// TODO: Enable
// 	return minDistance(aabb, obb);
// }

// float minDistance(OBB const& obb, AAEBB const& aaebb)
// {
// 	// TODO: Enable
// 	return minDistance(aaebb, obb);
// }

// float minDistance(OBB const& obb, Frustum const& frustum)
// {
// 	// TODO: Enable
// 	return minDistance(frustum, obb);
// }

// float minDistance(OBB const& obb, LineSegment const& line_segment)
// {
// 	// TODO: Enable
// 	return minDistance(line_segment, obb);
// }

// float minDistance(OBB const& obb_1, OBB const& obb_2)
// {
// 	// TODO: Implement
// }

// float minDistance(OBB const& obb, Plane const& plane)
// {
// 	// TODO: Implement
// }

// float minDistance(OBB const& obb, Point const& point)
// {
// 	// TODO: Implement
// }

// float minDistance(OBB const& obb, Ray const& ray)
// {
// 	// TODO: Implement
// }

// float minDistance(OBB const& obb, Sphere const& sphere)
// {
// 	// TODO: Implement
// }

//
// Plane
//

// float minDistance(Plane const& plane, AABB const& aabb)
// {
// 	// TODO: Enable
// 	return minDistance(aabb, plane);
// }

// float minDistance(Plane const& plane, AAEBB const& aaebb)
// {
// 	// TODO: Enable
// 	return minDistance(aaebb, plane);
// }

// float minDistance(Plane const& plane, Frustum const& frustum)
// {
// 	// TODO: Enable
// 	return minDistance(frustum, plane);
// }

// float minDistance(Plane const& plane, LineSegment const& line_segment)
// {
// 	// TODO: Enable
// 	return minDistance(line_segment, plane);
// }

// float minDistance(Plane const& plane, OBB const& obb)
// {
// 	// TODO: Enable
// 	return minDistance(obb, plane);
// }

// float minDistance(Plane const& plane_1, Plane const& plane_2)
// {
// 	// TODO: Implement
// }

// float minDistance(Plane const& plane, Point const& point)
// {
// 	// TODO: Implement
// }

// float minDistance(Plane const& plane, Ray const& ray)
// {
// 	// TODO: Implement
// }

// float minDistance(Plane const& plane, Sphere const& sphere)
// {
// 	// TODO: Implement
// }

//
// Point
//

float minDistance(Point const& point, AABB const& aabb)
{
	return minDistance(aabb, point);
}

float minDistance(Point const& point, AAEBB const& aaebb)
{
	return minDistance(aaebb, point);
}

// float minDistance(Point const& point, Frustum const& frustum)
// {
// 	// TODO: Enable
// 	return minDistance(frustum, point);
// }

// float minDistance(Point const& point, LineSegment const& line_segment)
// {
// 	// TODO: Enable
// 	return minDistance(line_segment, point);
// }

// float minDistance(Point const& point, OBB const& obb) {
// 	// TODO: Enable
// 	return minDistance(obb, point); }

// float minDistance(Point const& point, Plane const& plane)
// {
// 	// TODO: Enable
// 	return minDistance(plane, point);
// }

float minDistance(Point const& point_1, Point const& point_2)
{
	return point_1.distance(point_2);
}

// float minDistance(Point const& point, Ray const& ray)
// {
// 	// TODO: Implement
// }

float minDistance(Point const& point, Sphere const& sphere)
{
	return std::fdim(point.distance(sphere.center), sphere.radius);
}

//
// Ray
//

// float minDistance(Ray const& ray, AABB const& aabb)
// {
// 	// TODO: Enable
// 	return minDistance(aabb, ray);
// }

// float minDistance(Ray const& ray, AAEBB const& aaebb)
// {
// 	// TODO: Enable
// 	return minDistance(aaebb, ray);
// }

// float minDistance(Ray const& ray, Frustum const& frustum)
// {
// 	// TODO: Enable
// 	return minDistance(frustum, ray);
// }

// float minDistance(Ray const& ray, LineSegment const& line_segment)
// {
// 	// TODO: Enable
// 	return minDistance(line_segment, ray);
// }

// float minDistance(Ray const& ray, OBB const& obb)
// {
// 	// TODO: Enable
// 	return minDistance(obb, ray);
// }

// float minDistance(Ray const& ray, Plane const& plane)
// {
// 	// TODO: Enable
// 	return minDistance(plane, ray);
// }

// float minDistance(Ray const& ray, Point const& point)
// {
// 	// TODO: Enable
// 	return minDistance(point, ray);
// }

// float minDistance(Ray const& ray_1, Ray const& ray_2)
// {
// 	// TODO: Implement
// }

// float minDistance(Ray const& ray, Sphere const& sphere)
// {
// 	// TODO: Implement
// }

//
// Sphere
//

float minDistance(Sphere const& sphere, AABB const& aabb)
{
	return minDistance(aabb, sphere);
}

float minDistance(Sphere const& sphere, AAEBB const& aaebb)
{
	return minDistance(aaebb, sphere);
}

// float minDistance(Sphere const& sphere, Frustum const& frustum)
// {
// 	// TODO: Enable
// 	return minDistance(frustum, sphere);
// }

// float minDistance(Sphere const& sphere, LineSegment const& line_segment)
// {
// 	// TODO: Enable
// 	return minDistance(line_segment, sphere);
// }

// float minDistance(Sphere const& sphere, OBB const& obb)
// {
// 	// TODO: Enable
// 	return minDistance(obb, sphere);
// }

// float minDistance(Sphere const& sphere, Plane const& plane)
// {
// 	// TODO: Enable
// 	return minDistance(plane, sphere);
// }

float minDistance(Sphere const& sphere, Point const& point)
{
	return minDistance(point, sphere);
}

// float minDistance(Sphere const& sphere, Ray const& ray)
// {
// 	// TODO: Enable
// 	return minDistance(ray, sphere);
// }

float minDistance(Sphere const& sphere_1, Sphere const& sphere_2)
{
	return std::fdim(sphere_1.center.distance(sphere_2.center),
	                 sphere_1.radius + sphere_2.radius);
}

}  // namespace ufo::geometry