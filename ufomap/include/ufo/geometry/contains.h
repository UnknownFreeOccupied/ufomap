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

bool contains(AABB const& aabb_1, AABB const& aabb_2);
inline bool contains(AABB const& aabb, AAEBB const& aaebb)
{
	Point min_1 = aabb.getMin();
	Point max_1 = aabb.getMax();
	Point min_2 = aaebb.getMin();
	Point max_2 = aaebb.getMax();
	for (size_t i : {0, 1, 2}) {
		if (min_1[i] > min_2[i] || max_1[i] < max_2[i]) {
			return false;
		}
	}
	return true;
}
bool contains(AABB const& aabb, Frustum const& frustum);
bool contains(AABB const& aabb, LineSegment const& line_segment);
bool contains(AABB const& aabb, OBB const& obb);
bool contains(AABB const& aabb, Plane const& plane);
bool contains(AABB const& aabb, Point const& point);
bool contains(AABB const& aabb, Ray const& ray);
bool contains(AABB const& aabb, Sphere const& sphere);

//
// AAEBB
//

inline bool contains(AAEBB const& aaebb, AABB const& aabb)
{
	Point min_1 = aaebb.getMin();
	Point max_1 = aaebb.getMax();
	Point min_2 = aabb.getMin();
	Point max_2 = aabb.getMax();
	for (size_t i : {0, 1, 2}) {
		if (min_1[i] > min_2[i] || max_1[i] < max_2[i]) {
			return false;
		}
	}
	return true;
}
inline bool contains(AAEBB const& aaebb_1, AAEBB const& aaebb_2)
{
	Point min_1 = aaebb_1.getMin();
	Point max_1 = aaebb_1.getMax();
	Point min_2 = aaebb_2.getMin();
	Point max_2 = aaebb_2.getMax();
	for (size_t i : {0, 1, 2}) {
		if (min_1[i] > min_2[i] || max_1[i] < max_2[i]) {
			return false;
		}
	}
	return true;
}
inline bool contains(AAEBB const& aaebb, Frustum const& frustum)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}
inline bool contains(AAEBB const& aaebb, LineSegment const& line_segment)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}
inline bool contains(AAEBB const& aaebb, OBB const& obb)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}
inline bool contains(AAEBB const& aaebb, Plane const& plane)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}
inline bool contains(AAEBB const& aaebb, Point const& point)
{
	Point min = aaebb.getMin();
	Point max = aaebb.getMax();
	for (size_t i : {0, 1, 2}) {
		if (min[i] > point[i] || max[i] < point[i]) {
			return false;
		}
	}
	return true;
}
inline bool contains(AAEBB const& aaebb, Ray const& ray)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}
inline bool contains(AAEBB const& aaebb, Sphere const& sphere)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

//
// Frustum
//

bool contains(Frustum const& frustum, AABB const& aabb);
bool contains(Frustum const& frustum, AAEBB const& aaebb);
bool contains(Frustum const& frustum_1, Frustum const& frustum_2);
bool contains(Frustum const& frustum, LineSegment const& line_segment);
bool contains(Frustum const& frustum, OBB const& obb);
bool contains(Frustum const& frustum, Plane const& plane);
bool contains(Frustum const& frustum, Point const& point);
bool contains(Frustum const& frustum, Ray const& ray);
bool contains(Frustum const& frustum, Sphere const& sphere);

//
// Line segment
//

bool contains(LineSegment const& line_segment, AABB const& aabb);
bool contains(LineSegment const& line_segment, AAEBB const& aaebb);
bool contains(LineSegment const& line_segment, Frustum const& frustum);
bool contains(LineSegment const& line_segment_1, LineSegment const& line_segment_2);
bool contains(LineSegment const& line_segment, OBB const& obb);
bool contains(LineSegment const& line_segment, Plane const& plane);
bool contains(LineSegment const& line_segment, Point const& point);
bool contains(LineSegment const& line_segment, Ray const& ray);
bool contains(LineSegment const& line_segment, Sphere const& sphere);

//
// OBB
//

bool contains(OBB const& obb, AABB const& aabb);
bool contains(OBB const& obb, AAEBB const& aaebb);
bool contains(OBB const& obb, Frustum const& frustum);
bool contains(OBB const& obb, LineSegment const& line_segment);
bool contains(OBB const& obb_1, OBB const& obb_2);
bool contains(OBB const& obb, Plane const& plane);
bool contains(OBB const& obb, Point const& point);
bool contains(OBB const& obb, Ray const& ray);
bool contains(OBB const& obb, Sphere const& sphere);

//
// Plane
//

bool contains(Plane const& plane, AABB const& aabb);
bool contains(Plane const& plane, AAEBB const& aaebb);
bool contains(Plane const& plane, Frustum const& frustum);
bool contains(Plane const& plane, LineSegment const& line_segment);
bool contains(Plane const& plane, OBB const& obb);
bool contains(Plane const& plane_1, Plane const& plane_2);
bool contains(Plane const& plane, Point const& point);
bool contains(Plane const& plane, Ray const& ray);
bool contains(Plane const& plane, Sphere const& sphere);

//
// Point
//

bool contains(Point const& point, AABB const& aabb);
bool contains(Point const& point, AAEBB const& aaebb);
bool contains(Point const& point, Frustum const& frustum);
bool contains(Point const& point, LineSegment const& line_segment);
bool contains(Point const& point, OBB const& obb);
bool contains(Point const& point, Plane const& plane);
bool contains(Point const& point_1, Point const& point_2);
bool contains(Point const& point, Ray const& ray);
bool contains(Point const& point, Sphere const& sphere);

//
// Ray
//

bool contains(Ray const& ray, AABB const& aabb);
bool contains(Ray const& ray, AAEBB const& aaebb);
bool contains(Ray const& ray, Frustum const& frustum);
bool contains(Ray const& ray, LineSegment const& line_segment);
bool contains(Ray const& ray, OBB const& obb);
bool contains(Ray const& ray, Plane const& plane);
bool contains(Ray const& ray, Point const& point);
bool contains(Ray const& ray_1, Ray const& ray_2);
bool contains(Ray const& ray, Sphere const& sphere);

//
// Sphere
//

bool contains(Sphere const& sphere, AABB const& aabb);
bool contains(Sphere const& sphere, AAEBB const& aaebb);
bool contains(Sphere const& sphere, Frustum const& frustum);
bool contains(Sphere const& sphere, LineSegment const& line_segment);
bool contains(Sphere const& sphere, OBB const& obb);
bool contains(Sphere const& sphere, Plane const& plane);
bool contains(Sphere const& sphere, Point const& point);
bool contains(Sphere const& sphere, Ray const& ray);
bool contains(Sphere const& sphere_1, Sphere const& sphere_2);

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