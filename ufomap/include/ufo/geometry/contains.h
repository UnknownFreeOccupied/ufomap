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

#ifndef UFO_GEOMETRY_CONTAINS_H
#define UFO_GEOMETRY_CONTAINS_H

// UFO
#include <ufo/geometry/aabb.h>
#include <ufo/geometry/aaebb.h>
#include <ufo/geometry/bounding_volume.h>
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

constexpr bool contains(AABB const& aabb_1, AABB const& aabb_2)
{
	Point min_1 = aabb_1.min();
	Point max_1 = aabb_1.max();
	Point min_2 = aabb_2.min();
	Point max_2 = aabb_2.max();
	return min_1.x <= min_2.x && min_1.y <= min_2.y && min_1.z <= min_2.z &&
	       max_1.x >= max_2.x && max_1.y >= max_2.y && max_1.z >= max_2.z;
}

constexpr bool contains(AABB const& aabb, AAEBB const& aaebb)
{
	Point min_1 = aabb.getMin();
	Point max_1 = aabb.getMax();
	Point min_2 = aaebb.getMin();
	Point max_2 = aaebb.getMax();
	return min_1.x <= min_2.x && min_1.y <= min_2.y && min_1.z <= min_2.z &&
	       max_1.x >= max_2.x && max_1.y >= max_2.y && max_1.z >= max_2.z;
}

// constexpr bool contains(AABB const& aabb, Frustum const& frustum)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(AABB const& aabb, LineSegment const& line_segment)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(AABB const& aabb, OBB const& obb)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(AABB const& aabb, Plane const& plane)
// {
// 	// TODO: Implement
// }

constexpr bool contains(AABB const& aabb, Point const& point)
{
	Point min = aabb.min();
	Point max = aabb.max();
	return min.x <= point.x && min.y <= point.y && min.z <= point.z && max.x >= point.x &&
	       max.y >= point.y && max.z >= point.z;
}

// constexpr bool contains(AABB const& aabb, Ray const& ray)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(AABB const& aabb, Sphere const& sphere)
// {
// 	// TODO: Implement
// }

//
// AAEBB
//

constexpr bool contains(AAEBB const& aaebb, AABB const& aabb)
{
	Point min_1 = aaebb.getMin();
	Point max_1 = aaebb.getMax();
	Point min_2 = aabb.getMin();
	Point max_2 = aabb.getMax();
	return min_1.x <= min_2.x && min_1.y <= min_2.y && min_1.z <= min_2.z &&
	       max_1.x >= max_2.x && max_1.y >= max_2.y && max_1.z >= max_2.z;
}

constexpr bool contains(AAEBB const& aaebb_1, AAEBB const& aaebb_2)
{
	if (aaebb_1.half_size < aaebb_2.half_size) {
		// FIXME: Check if this optimization is worth it and then apply for other
		return false;
	}

	Point min_1 = aaebb_1.getMin();
	Point max_1 = aaebb_1.getMax();
	Point min_2 = aaebb_2.getMin();
	Point max_2 = aaebb_2.getMax();
	return min_1.x <= min_2.x && min_1.y <= min_2.y && min_1.z <= min_2.z &&
	       max_1.x >= max_2.x && max_1.y >= max_2.y && max_1.z >= max_2.z;
}

// constexpr bool contains(AAEBB const& aaebb, Frustum const& frustum)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(AAEBB const& aaebb, LineSegment const& line_segment)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(AAEBB const& aaebb, OBB const& obb)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(AAEBB const& aaebb, Plane const& plane)
// {
// 	// TODO: Implement
// }

constexpr bool contains(AAEBB const& aaebb, Point const& point)
{
	Point min = aaebb.getMin();
	Point max = aaebb.getMax();
	return min.x <= point.x && min.y <= point.y && min.z <= point.z && max.x >= point.x &&
	       max.y >= point.y && max.z >= point.z;
}

// constexpr bool contains(AAEBB const& aaebb, Ray const& ray)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(AAEBB const& aaebb, Sphere const& sphere)
// {
// 	// TODO: Implement
// }

//
// Frustum
//

// constexpr bool contains(Frustum const& frustum, AABB const& aabb)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Frustum const& frustum, AAEBB const& aaebb)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Frustum const& frustum_1, Frustum const& frustum_2)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Frustum const& frustum, LineSegment const& line_segment)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Frustum const& frustum, OBB const& obb)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Frustum const& frustum, Plane const& plane)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Frustum const& frustum, Point const& point)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Frustum const& frustum, Ray const& ray)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Frustum const& frustum, Sphere const& sphere)
// {
// 	// TODO: Implement
// }

//
// Line segment
//

// constexpr bool contains(LineSegment const& line_segment, AABB const& aabb)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(LineSegment const& line_segment, AAEBB const& aaebb)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(LineSegment const& line_segment, Frustum const& frustum)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(LineSegment const& line_segment_1,
//                         LineSegment const& line_segment_2)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(LineSegment const& line_segment, OBB const& obb)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(LineSegment const& line_segment, Plane const& plane)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(LineSegment const& line_segment, Point const& point)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(LineSegment const& line_segment, Ray const& ray)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(LineSegment const& line_segment, Sphere const& sphere)
// {
// 	// TODO: Implement
// }

//
// OBB
//

// constexpr bool contains(OBB const& obb, AABB const& aabb)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(OBB const& obb, AAEBB const& aaebb)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(OBB const& obb, Frustum const& frustum)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(OBB const& obb, LineSegment const& line_segment)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(OBB const& obb_1, OBB const& obb_2)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(OBB const& obb, Plane const& plane)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(OBB const& obb, Point const& point)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(OBB const& obb, Ray const& ray)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(OBB const& obb, Sphere const& sphere)
// {
// 	// TODO: Implement
// }

//
// Plane
//

// constexpr bool contains(Plane const& plane, AABB const& aabb)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Plane const& plane, AAEBB const& aaebb)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Plane const& plane, Frustum const& frustum)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Plane const& plane, LineSegment const& line_segment)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Plane const& plane, OBB const& obb)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Plane const& plane_1, Plane const& plane_2)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Plane const& plane, Point const& point)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Plane const& plane, Ray const& ray)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Plane const& plane, Sphere const& sphere)
// {
// 	// TODO: Implement
// }

//
// Point
//

constexpr bool contains(Point const& point, AABB const& aabb)
{
	return point == aabb.center && 0 == aabb.half_size.x && 0 == aabb.half_size.y &&
	       0 == aabb.half_size.z;
}

constexpr bool contains(Point const& point, AAEBB const& aaebb)
{
	return point == aabb.center && 0 == aabb.half_size;
}

constexpr bool contains(Point const& point, Frustum const& frustum)
{
	return false;  // TODO: Check
}

constexpr bool contains(Point const& point, LineSegment const& line_segment)
{
	return point == line_segment.start && point == line_segment.end;
}

constexpr bool contains(Point const& point, OBB const& obb)
{
	return point == aabb.center && 0 == aabb.half_size.x && 0 == aabb.half_size.y &&
	       0 == aabb.half_size.z;
}

constexpr bool contains(Point const& point, Plane const& plane)
{
	return false;  // TODO: Check
}

constexpr bool contains(Point const& point_1, Point const& point_2)
{
	return point_1 == point_2;
}

constexpr bool contains(Point const& point, Ray const& ray) { return false; }

constexpr bool contains(Point const& point, Sphere const& sphere)
{
	return 0 == sphere.radius && point == sphere.center;
}

//
// Ray
//

// constexpr bool contains(Ray const& ray, AABB const& aabb)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Ray const& ray, AAEBB const& aaebb)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Ray const& ray, Frustum const& frustum)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Ray const& ray, LineSegment const& line_segment)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Ray const& ray, OBB const& obb)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Ray const& ray, Plane const& plane)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Ray const& ray, Point const& point)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Ray const& ray_1, Ray const& ray_2)
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Ray const& ray, Sphere const& sphere)
// {
// 	// TODO: Implement
// }

//
// Sphere
//

constexpr bool contains(Sphere const& sphere, AABB const& aabb)
{
	return contains(sphere, aabb.min()) && contains(sphere, aabb.max());
}

constexpr bool contains(Sphere const& sphere, AAEBB const& aaebb)
{
	return contains(sphere, aaebb.min()) && contains(sphere, aaebb.max());
}

// constexpr bool contains(Sphere const& sphere, Frustum const& frustum)
// {
// 	// TODO: Implement
// }

constexpr bool contains(Sphere const& sphere, LineSegment const& line_segment)
{
	return contains(sphere, line_segment.start) && contains(sphere, line_segment.end);
}

constexpr bool contains(Sphere const& sphere, OBB const& obb)
{
	return contains(sphere, obb.min()) && contains(sphere, obb.max());
}

constexpr bool contains(Sphere const& sphere, Plane const& plane) { return false; }

constexpr bool contains(Sphere const& sphere, Point const& point)
{
	return minDistance(sphere, point) <= sphere.radius;
}

constexpr bool contains(Sphere const& sphere, Ray const& ray) { return false; }

constexpr bool contains(Sphere const& sphere_1, Sphere const& sphere_2)
{
	return sphere_1.radius >= sphere_2.radius + sphere_1.center.distance(sphere_2.center);
}

//
// Bounding volume
//

template <class Geometry>
bool contains(BoundingVolume const& bv, Geometry const& geometry)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

template <class Geometry>
bool contains(Geometry const& geometry, BoundingVolume const& bv)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contains(BoundingVolume const& bv_1, BoundingVolume const& bv_2);
}  // namespace ufo::geometry

#endif  // UFO_GEOMETRY_CONTAINS_H