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

#ifndef UFO_GEOMETRY_INTERSECTS_H
#define UFO_GEOMETRY_INTERSECTS_H

// UFO
#include <ufo/geometry/aabb.h>
#include <ufo/geometry/aaebb.h>
#include <ufo/geometry/bounding_volume.h>
#include <ufo/geometry/frustum.h>
#include <ufo/geometry/helper.h>
#include <ufo/geometry/line_segment.h>
#include <ufo/geometry/obb.h>
#include <ufo/geometry/plane.h>
#include <ufo/geometry/point.h>
#include <ufo/geometry/ray.h>
#include <ufo/geometry/sphere.h>
#include <ufo/geometry/triangle.h>

namespace ufo::geometry
{
//
// AABB
//

bool intersects(AABB const& aabb_1, AABB const& aabb_2);
inline bool intersects(AABB const& aabb, AAEBB const& aaebb)
{
	Point min_1 = aabb.getMin();
	Point max_1 = aabb.getMax();
	Point min_2 = aaebb.getMin();
	Point max_2 = aaebb.getMax();
	return min_1.x() <= max_2.x() && min_1.y() <= max_2.y() && min_1.z() <= max_2.z() &&
	       min_2.x() <= max_1.x() && min_2.y() <= max_1.y() && min_2.z() <= max_1.z();
}
bool intersects(AABB const& aabb, Frustum const& frustum);
bool intersects(AABB const& aabb, LineSegment const& line_segment);
bool intersects(AABB const& aabb, OBB const& obb);
bool intersects(AABB const& aabb, Plane const& plane);
bool intersects(AABB const& aabb, Point const& point);
bool intersects(AABB const& aabb, Ray const& ray);
bool intersects(AABB const& aabb, Sphere const& sphere);

//
// AAEBB
//

inline bool intersects(AAEBB const& aaebb, AABB const& aabb)
{
	return intersects(aabb, aaebb);
}
inline bool intersects(AAEBB const& aaebb_1, AAEBB const& aaebb_2)
{
	Point min_1 = aaebb_1.getMin();
	Point max_1 = aaebb_1.getMax();
	Point min_2 = aaebb_2.getMin();
	Point max_2 = aaebb_2.getMax();
	return min_1.x() <= max_2.x() && min_1.y() <= max_2.y() && min_1.z() <= max_2.z() &&
	       min_2.x() <= max_1.x() && min_2.y() <= max_1.y() && min_2.z() <= max_1.z();
}
inline bool intersects(AAEBB const& aaebb, Frustum const& frustum)
{
	// FIXME:
	for (int i = 0; i < 6; ++i) {
		float side = classify(aaebb, frustum.planes[i]);
		if (side < 0) {
			return false;
		}
	}
	return true;
}
inline bool intersects(AAEBB const& aaebb, LineSegment const& line_segment)
{
	Ray ray;
	ray.origin = line_segment.start;
	ray.direction = (line_segment.end - line_segment.start);
	float length = ray.direction.norm();
	ray.direction /= length;
	return intersectsLine(aaebb, ray, 0.0, length);
}
inline bool intersects(AAEBB const& aaebb, OBB const& obb)
{
	// ufo::geometry::OBB obb_2(aabb.center, aabb.half_size, ufo::math::Point(0, 0,
	// 0)); return intersects(obb, obb_2);

	std::array<float, 9> obb_rot_matrix;
	obb.rotation.toRotMatrix(obb_rot_matrix);

	Point test[15] = {Point(1, 0, 0),  // AABB axis 1
	                  Point(0, 1, 0),  // AABB axis 2
	                  Point(0, 0, 1),  // AABB axis 3
	                  Point(obb_rot_matrix[0], obb_rot_matrix[1], obb_rot_matrix[2]),
	                  Point(obb_rot_matrix[3], obb_rot_matrix[4], obb_rot_matrix[5]),
	                  Point(obb_rot_matrix[6], obb_rot_matrix[7], obb_rot_matrix[8])};

	for (int i = 0; i < 3; ++i) {  // Fill out rest of axis
		test[6 + i * 3 + 0] = Point::cross(test[i], test[3]);
		test[6 + i * 3 + 1] = Point::cross(test[i], test[4]);
		test[6 + i * 3 + 2] = Point::cross(test[i], test[5]);
	}

	for (int i = 0; i < 15; ++i) {
		if (!overlapOnAxis(aaebb, obb, test[i])) {
			return false;  // Seperating axis found
		}
	}

	return true;  // Seperating axis not found
}
inline bool intersects(AAEBB const& aaebb, Plane const& plane)
{
	float p_len = aaebb.halfSize() * std::abs(plane.normal.x()) +
	              aaebb.halfSize() * std::abs(plane.normal.y()) +
	              aaebb.halfSize() * std::abs(plane.normal.z());
	float distance = Point::dot(plane.normal, aaebb.center()) - plane.distance;
	return std::abs(distance) <= p_len;
}
inline bool intersects(AAEBB const& aaebb, Point const& point)
{
	Point min = aaebb.getMin();
	Point max = aaebb.getMax();
	if (point.x() < min.x() || point.y() < min.y() || point.z() < min.z() ||
	    point.x() > max.x() || point.y() > max.y() || point.z() > max.z()) {
		return false;
	}
	return true;
}
inline bool intersects(AAEBB const& aaebb, Ray const& ray)
{
	// TODO: infinity or max?
	return intersectsLine(aaebb, ray, 0.0, std::numeric_limits<float>::infinity());
}
inline bool intersects(AAEBB const& aaebb, Sphere const& sphere)
{
	Point closest_point = closestPoint(aaebb, sphere.center);
	float distance_squared = (sphere.center - closest_point).squaredNorm();
	float radius_squared = sphere.radius * sphere.radius;
	return distance_squared < radius_squared;
}
inline bool intersects(AAEBB const& aaebb, Triangle triangle)
{
	for (auto& point : triangle.points) {
		point -= aaebb.center();
	}

	std::array<Point, 3> f = {triangle.points[1] - triangle.points[0],
	                          triangle.points[2] - triangle.points[1],
	                          triangle.points[0] - triangle.points[2]};

	std::array<Point, 3> u = {Point(1, 0, 0), Point(0, 1, 0), Point(0, 0, 1)};

	std::array<Point, 9> axis = {
	    Point::cross(u[0], f[0]), Point::cross(u[0], f[1]), Point::cross(u[0], f[2]),
	    Point::cross(u[1], f[0]), Point::cross(u[1], f[1]), Point::cross(u[1], f[2]),
	    Point::cross(u[2], f[0]), Point::cross(u[2], f[1]), Point::cross(u[2], f[2])};

	std::array<Point, 9 + 3 + 1> all_axis = {axis[0],
	                                         axis[1],
	                                         axis[2],
	                                         axis[3],
	                                         axis[4],
	                                         axis[5],
	                                         axis[6],
	                                         axis[7],
	                                         axis[8],
	                                         u[0],
	                                         u[1],
	                                         u[2],
	                                         Point::cross(f[0], f[1])};

	for (auto a : all_axis) {
		std::array<float, 3> p = {Point::dot(triangle.points[0], a),
		                          Point::dot(triangle.points[1], a),
		                          Point::dot(triangle.points[2], a)};

		float r = aaebb.halfSize() * std::abs(Point::dot(u[0], a)) +
		          aaebb.halfSize() * std::abs(Point::dot(u[1], a)) +
		          aaebb.halfSize() * std::abs(Point::dot(u[2], a));

		if (std::max(-std::max({p[0], p[1], p[2]}), std::min({p[0], p[1], p[2]})) > r) {
			return false;
		}
	}

	return true;
}

//
// Frustum
//

bool intersects(Frustum const& frustum, AABB const& aabb);
bool intersects(Frustum const& frustum, AAEBB const& aaebb);
bool intersects(Frustum const& frustum_1, Frustum const& frustum_2);
bool intersects(Frustum const& frustum, LineSegment const& line_segment);
bool intersects(Frustum const& frustum, OBB const& obb);
bool intersects(Frustum const& frustum, Plane const& plane);
bool intersects(Frustum const& frustum, Point const& point);
bool intersects(Frustum const& frustum, Ray const& ray);
bool intersects(Frustum const& frustum, Sphere const& sphere);

//
// Line segment
//

bool intersects(LineSegment const& line_segment, AABB const& aabb);
bool intersects(LineSegment const& line_segment, AAEBB const& aaebb);
bool intersects(LineSegment const& line_segment, Frustum const& frustum);
bool intersects(LineSegment const& line_segment_1, LineSegment const& line_segment_2);
bool intersects(LineSegment const& line_segment, OBB const& obb);
bool intersects(LineSegment const& line_segment, Plane const& plane);
bool intersects(LineSegment const& line_segment, Point const& point);
bool intersects(LineSegment const& line_segment, Ray const& ray);
bool intersects(LineSegment const& line_segment, Sphere const& sphere);

//
// OBB
//

bool intersects(OBB const& obb, AABB const& aabb);
bool intersects(OBB const& obb, AAEBB const& aaebb);
bool intersects(OBB const& obb, Frustum const& frustum);
bool intersects(OBB const& obb, LineSegment const& line_segment);
bool intersects(OBB const& obb_1, OBB const& obb_2);
bool intersects(OBB const& obb, Plane const& plane);
bool intersects(OBB const& obb, Point const& point);
bool intersects(OBB const& obb, Ray const& ray);
bool intersects(OBB const& obb, Sphere const& sphere);

//
// Plane
//

bool intersects(Plane const& plane, AABB const& aabb);
bool intersects(Plane const& plane, AAEBB const& aaebb);
bool intersects(Plane const& plane, Frustum const& frustum);
bool intersects(Plane const& plane, LineSegment const& line_segment);
bool intersects(Plane const& plane, OBB const& obb);
bool intersects(Plane const& plane_1, Plane const& plane_2);
bool intersects(Plane const& plane, Point const& point);
bool intersects(Plane const& plane, Ray const& ray);
bool intersects(Plane const& plane, Sphere const& sphere);

//
// Point
//

bool intersects(Point const& point, AABB const& aabb);
bool intersects(Point const& point, AAEBB const& aaebb);
bool intersects(Point const& point, Frustum const& frustum);
bool intersects(Point const& point, LineSegment const& line_segment);
bool intersects(Point const& point, OBB const& obb);
bool intersects(Point const& point, Plane const& plane);
bool intersects(Point const& point_1, Point const& point_2);
bool intersects(Point const& point, Ray const& ray);
bool intersects(Point const& point, Sphere const& sphere);

//
// Ray
//

bool intersects(Ray const& ray, AABB const& aabb);
bool intersects(Ray const& ray, AAEBB const& aaebb);
bool intersects(Ray const& ray, Frustum const& frustum);
bool intersects(Ray const& ray, LineSegment const& line_segment);
bool intersects(Ray const& ray, OBB const& obb);
bool intersects(Ray const& ray, Plane const& plane);
bool intersects(Ray const& ray, Point const& point);
bool intersects(Ray const& ray_1, Ray const& ray_2);
bool intersects(Ray const& ray, Sphere const& sphere);

//
// Sphere
//

bool intersects(Sphere const& sphere, AABB const& aabb);
bool intersects(Sphere const& sphere, AAEBB const& aaebb);
bool intersects(Sphere const& sphere, Frustum const& frustum);
bool intersects(Sphere const& sphere, LineSegment const& line_segment);
bool intersects(Sphere const& sphere, OBB const& obb);
bool intersects(Sphere const& sphere, Plane const& plane);
bool intersects(Sphere const& sphere, Point const& point);
bool intersects(Sphere const& sphere, Ray const& ray);
bool intersects(Sphere const& sphere_1, Sphere const& sphere_2);

//
// Bounding volume
//

template <class Geometry>
bool intersects(BoundingVolume const& bounding_volume, Geometry const& geometry)
{
	for (auto const& bv : bounding_volume) {
		if (std::visit([&geometry](auto&& bv) -> bool { return intersects(bv, geometry); },
		               bv)) {
			return true;
		}
	}
	return false;
}

template <class Geometry>
bool intersects(Geometry const& geometry, BoundingVolume const& bounding_volume)
{
	return intersects(bounding_volume, geometry);
}

bool intersects(BoundingVolume const& bounding_volume_1,
                BoundingVolume const& bounding_volume_2);

}  // namespace ufo::geometry

#endif  // UFO_GEOMETRY_INTERSECTS_H