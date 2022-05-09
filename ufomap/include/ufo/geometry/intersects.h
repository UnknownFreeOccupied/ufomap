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
#include <ufo/geometry/bounding_volume.h>
#include <ufo/geometry/helper.h>

// STL
#include <algorithm>

namespace ufo::geometry
{
//
// Point
//

constexpr bool intersects(Point const& point, AABB const& aabb) noexcept
{
	Point min = aabb.min();
	Point max = aabb.max();
	return min.x <= point.x && min.y <= point.y && min.z <= point.z && max.x >= point.x &&
	       max.y >= point.y && max.z >= point.z;
}

constexpr bool intersects(Point const& point, AAEBB const& aaebb) noexcept
{
	Point min = aaebb.min();
	Point max = aaebb.max();
	return min.x <= point.x && min.y <= point.y && min.z <= point.z && max.x >= point.x &&
	       max.y >= point.y && max.z >= point.z;
}

constexpr bool intersects(Point const& point, Frustum const& frustum) noexcept
{
	for (std::size_t i = 0; i < 6; ++i) {
		const Point& normal = frustum[i].normal;
		const float& distance = frustum[i].distance;
		const float side = Point::dot(point, normal) + distance;
		if (0 > side) {
			return false;
		}
	}
	return true;
}

constexpr bool intersects(Point const& point, LineSegment const& line_segment) noexcept
{
	Point closest_point = closestPoint(line_segment, point);
	float distance_squared = (closest_point - point).squaredNorm();

	// FIXME: Almost equal?
	return 0 == distance_squared;
}

constexpr bool intersects(Point const& point, OBB const& obb) noexcept
{
	// FIXME: Implement look earlier. THIS IS WRONG!
	Point dir = point - obb.center;
	std::array<float, 9> obb_rot_matrix = obb.rotation.getRotMatrix();
	for (int i = 0; i < 3; ++i) {
		Point axis(obb_rot_matrix[i * 3], obb_rot_matrix[i * 3 + 1],
		           obb_rot_matrix[i * 3 + 2]);
		float distance = Point::dot(dir, axis);
		if (distance > obb.half_size[i]) {
			return false;
		}
		if (distance < -obb.half_size[i])  // FIXME: Should this be else if?
		{
			return false;
		}
	}
	return true;
}

constexpr bool intersects(Point const& point, Plane const& plane) noexcept
{
	return Point::dot(point, plane.normal) - plane.distance;
}

constexpr bool intersects(Point const& point_1, Point const& point_2) noexcept
{
	// FIXME: Almost equal?
	return point_1 == point_2;
}

constexpr bool intersects(Point const& point, Ray const& ray) noexcept
{
	if (ray.origin == point) {
		return true;
	}
	Point direction = point - ray.origin;
	direction.normalize();

	// FIXME: Almost equal?
	return 1.0 == Point::dot(direction, ray.direction);
}

constexpr bool intersects(Point const& point, Sphere const& sphere) noexcept
{
	return (point - sphere.center).squaredNorm() < (sphere.radius * sphere.radius);
}

//
// Sphere
//

constexpr bool intersects(Sphere const& sphere, AABB const& aabb) noexcept
{
	Point closest_point = closestPoint(aabb, sphere.center);
	float distance_squared = (sphere.center - closest_point).squaredNorm();
	float radius_squared = sphere.radius * sphere.radius;
	return distance_squared < radius_squared;
}

constexpr bool intersects(Sphere const& sphere, AAEBB const& aaebb) noexcept
{
	Point closest_point = closestPoint(aaebb, sphere.center);
	float distance_squared = (sphere.center - closest_point).squaredNorm();
	float radius_squared = sphere.radius * sphere.radius;
	return distance_squared < radius_squared;
}

constexpr bool intersects(Sphere const& sphere, Frustum const& frustum) noexcept
{
	for (std::size_t i = 0; i < 6; ++i) {
		const Point& normal = frustum[i].normal;
		const float& distance = frustum[i].distance;
		const float side = Point::dot(sphere.center, normal) + distance;
		if (side < -sphere.radius) {
			return false;
		}
	}
	return true;
}

constexpr bool intersects(Sphere const& sphere, LineSegment const& line_segment) noexcept
{
	Point closest_point = closestPoint(line_segment, sphere.center);
	float distance_squared = (sphere.center - closest_point).squaredNorm();
	return distance_squared <= (sphere.radius * sphere.radius);
}

constexpr bool intersects(Sphere const& sphere, OBB const& obb) noexcept
{
	Point closest_point = closestPoint(obb, sphere.center);
	float distance_squared = (sphere.center - closest_point).squaredNorm();
	return distance_squared < (sphere.radius * sphere.radius);
}

constexpr bool intersects(Sphere const& sphere, Plane const& plane) noexcept
{
	Point closest_point = closestPoint(plane, sphere.center);
	float distance_squared = (sphere.center - closest_point).squaredNorm();
	return distance_squared < (sphere.radius * sphere.radius);
}

constexpr bool intersects(Sphere const& sphere, Point const& point) noexcept
{
	return intersects(point, sphere);
}

constexpr bool intersects(Sphere const& sphere, Ray const& ray) noexcept
{
	Point e = sphere.center - ray.origin;
	float r_sq = sphere.radius * sphere.radius;
	float e_sq = e.squaredNorm();
	float a = Point::dot(e, ray.direction);
	return 0 <= r_sq - (e_sq - a * a);
}

constexpr bool intersects(Sphere const& sphere_1, Sphere const& sphere_2) noexcept
{
	float radius_sum = sphere_1.radius + sphere_2.radius;
	float distance_squared = (sphere_1.center - sphere_2.center).squaredNorm();
	return distance_squared < (radius_sum * radius_sum);
}

//
// AABB
//

constexpr bool intersects(AABB const& aabb_1, AABB const& aabb_2) noexcept
{
	Point min_1 = aabb_1.min();
	Point max_1 = aabb_1.max();
	Point min_2 = aabb_2.min();
	Point max_2 = aabb_2.max();
	return min_1.x <= max_2.x && min_1.y <= max_2.y && min_1.z <= max_2.z &&
	       min_2.x <= max_1.x && min_2.y <= max_1.y && min_2.z <= max_1.z;
}

constexpr bool intersects(AABB const& aabb, AAEBB const& aaebb) noexcept
{
	Point min_1 = aabb.min();
	Point max_1 = aabb.max();
	Point min_2 = aaebb.min();
	Point max_2 = aaebb.max();
	return min_1.x <= max_2.x && min_1.y <= max_2.y && min_1.z <= max_2.z &&
	       min_2.x <= max_1.x && min_2.y <= max_1.y && min_2.z <= max_1.z;
}

constexpr bool intersects(AABB const& aabb, Frustum const& frustum) noexcept
{
	return 0 <= classify(aabb, frustum.bottom) && 0 <= classify(aabb, frustum.far) &&
	       0 <= classify(aabb, frustum.left) && 0 <= classify(aabb, frustum.near) &&
	       0 <= classify(aabb, frustum.right) && 0 <= classify(aabb, frustum.top);
}

constexpr bool intersects(AABB const& aabb, LineSegment const& line_segment) noexcept
{
	Ray ray;
	ray.origin = line_segment.start;
	ray.direction = (line_segment.end - line_segment.start);
	float length = ray.direction.norm();
	ray.direction /= length;
	return intersectsLine(aabb, ray, 0.0, length);
}

constexpr bool intersects(AABB const& aabb, OBB const& obb) noexcept
{
	std::array<float, 9> obb_rot_matrix = obb.rotation.getRotMatrix();

	std::array<Point, 15> test = {
	    Point(1, 0, 0),  // AABB axis 1
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

	return std::all_of(std::cbegin(test), std::cend(test), [&aabb, &obb](auto const& t) {
		return overlapOnAxis(aabb, obb, t);
	});
}

constexpr bool intersects(AABB const& aabb, Plane const& plane) noexcept
{
	float p_len = aabb.half_size.x * std::abs(plane.normal.x) +
	              aabb.half_size.y * std::abs(plane.normal.y) +
	              aabb.half_size.z * std::abs(plane.normal.z);
	float distance = Point::dot(plane.normal, aabb.center) - plane.distance;
	return std::abs(distance) <= p_len;
}

constexpr bool intersects(AABB const& aabb, Point const& point) noexcept
{
	return intersects(point, aabb);
}

constexpr bool intersects(AABB const& aabb, Ray const& ray) noexcept
{
	// FIXME: infinity or max?
	return intersectsLine(aabb, ray, 0.0, std::numeric_limits<float>::infinity());
}

constexpr bool intersects(AABB const& aabb, Sphere const& sphere) noexcept
{
	return intersects(sphere, aabb);
}

//
// AAEBB
//

constexpr bool intersects(AAEBB const& aaebb, AABB const& aabb) noexcept
{
	return intersects(aabb, aaebb);
}

constexpr bool intersects(AAEBB const& aaebb_1, AAEBB const& aaebb_2) noexcept
{
	Point min_1 = aaebb_1.min();
	Point max_1 = aaebb_1.max();
	Point min_2 = aaebb_2.min();
	Point max_2 = aaebb_2.max();
	return min_1.x <= max_2.x && min_1.y <= max_2.y && min_1.z <= max_2.z &&
	       min_2.x <= max_1.x && min_2.y <= max_1.y && min_2.z <= max_1.z;
}

constexpr bool intersects(AAEBB const& aaebb, Frustum const& frustum) noexcept
{
	return 0 <= classify(aaebb, frustum.bottom) && 0 <= classify(aaebb, frustum.far) &&
	       0 <= classify(aaebb, frustum.left) && 0 <= classify(aaebb, frustum.near) &&
	       0 <= classify(aaebb, frustum.right) && 0 <= classify(aaebb, frustum.top);
}

constexpr bool intersects(AAEBB const& aaebb, LineSegment const& line_segment) noexcept
{
	Ray ray;
	ray.origin = line_segment.start;
	ray.direction = (line_segment.end - line_segment.start);
	float length = ray.direction.norm();
	ray.direction /= length;
	return intersectsLine(aaebb, ray, 0.0, length);
}

constexpr bool intersects(AAEBB const& aaebb, OBB const& obb) noexcept
{
	std::array<float, 9> obb_rot_matrix = obb.rotation.getRotMatrix();

	std::array<Point, 15> test = {
	    Point(1, 0, 0),  // AABB axis 1
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

	return std::all_of(std::cbegin(test), std::cend(test), [&aaebb, &obb](auto const& t) {
		return overlapOnAxis(aaebb, obb, t);
	});
}

constexpr bool intersects(AAEBB const& aaebb, Plane const& plane) noexcept
{
	float p_len = aaebb.half_size * std::abs(plane.normal.x) +
	              aaebb.half_size * std::abs(plane.normal.y) +
	              aaebb.half_size * std::abs(plane.normal.z);
	float distance = Point::dot(plane.normal, aaebb.center) - plane.distance;
	return std::abs(distance) <= p_len;
}

constexpr bool intersects(AAEBB const& aaebb, Point const& point) noexcept
{
	return intersects(point, aaebb);
}

constexpr bool intersects(AAEBB const& aaebb, Ray const& ray) noexcept
{
	// FIXME: infinity or max?
	return intersectsLine(aaebb, ray, 0.0, std::numeric_limits<float>::infinity());
}

constexpr bool intersects(AAEBB const& aaebb, Sphere const& sphere) noexcept
{
	return intersects(sphere, aaebb);
}

constexpr bool intersects(AAEBB const& aaebb, Triangle triangle) noexcept
{
	for (auto& point : triangle.points) {
		point -= aaebb.center;
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

		float r = aaebb.half_size * std::abs(Point::dot(u[0], a)) +
		          aaebb.half_size * std::abs(Point::dot(u[1], a)) +
		          aaebb.half_size * std::abs(Point::dot(u[2], a));

		if (std::max(-std::max({p[0], p[1], p[2]}), std::min({p[0], p[1], p[2]})) > r) {
			return false;
		}
	}

	return true;
}

//
// OBB
//

constexpr bool intersects(OBB const& obb, AABB const& aabb) noexcept
{
	return intersects(aabb, obb);
}

constexpr bool intersects(OBB const& obb, AAEBB const& aaebb) noexcept
{
	return intersects(aaebb, obb);
}

constexpr bool intersects(OBB const& obb, Frustum const& frustum) noexcept
{
	return 0 <= classify(obb, frustum.bottom) && 0 <= classify(obb, frustum.far) &&
	       0 <= classify(obb, frustum.left) && 0 <= classify(obb, frustum.near) &&
	       0 <= classify(obb, frustum.right) && 0 <= classify(obb, frustum.top);
}

constexpr bool intersects(OBB const& obb, LineSegment const& line_segment) noexcept
{
	Ray ray;
	ray.origin = line_segment.start;
	ray.direction = line_segment.end - line_segment.start;
	float line_length_squared = ray.direction.squaredNorm();
	if (line_length_squared < 0.0000001f) {
		return intersects(line_segment.start, obb);
	}
	ray.direction /= line_length_squared;  // Normalize

	// Begin ray casting

	Point p = obb.center - ray.origin;

	Point X(obb.rotation[0], 0, 0);
	Point Y(0, obb.rotation[1], 0);
	Point Z(0, 0, obb.rotation[2]);

	Point f(Point::dot(X, ray.direction), Point::dot(Y, ray.direction),
	        Point::dot(Z, ray.direction));

	Point e(Point::dot(X, p), Point::dot(Y, p), Point::dot(Z, p));

	float t[6] = {0, 0, 0, 0, 0, 0};
	for (int i = 0; i < 3; ++i) {
		if (0.0 == f[i])  // FIXME: Should be approximate equal
		{
			if (-e[i] - obb.half_size[i] > 0 || -e[i] + obb.half_size[i] < 0) {
				return false;
			}
			f[i] = 0.00001f;  // Avoid div by 0!
		}
		t[i * 2 + 0] = (e[i] + obb.half_size[i]) / f[i];  // tmin[x, y, z]
		t[i * 2 + 1] = (e[i] - obb.half_size[i]) / f[i];  // tmax[x, y, z]
	}

	float tmin = std::max(std::max(std::min(t[0], t[1]), std::min(t[2], t[3])),
	                      std::min(t[4], t[5]));
	float tmax = std::min(std::min(std::max(t[0], t[1]), std::max(t[2], t[3])),
	                      std::max(t[4], t[5]));

	// if tmax < 0, ray is intersecting AABB
	// but entire AABB is behing it's origin
	if (tmax < 0) {
		return false;
	}

	// if tmin > tmax, ray doesn't intersect AABB
	if (tmin > tmax) {
		return false;
	}

	// If tmin is < 0, tmax is closer
	float t_result = tmin;

	if (tmin < 0.0f) {
		t_result = tmax;
	}

	// End ray casting
	return t_result >= 0 && t_result * t_result <= line_length_squared;
}

constexpr bool intersects(OBB const& obb_1, OBB const& obb_2) noexcept
{
	std::array<float, 9> obb_1_rot_matrix = obb_1.rotation.getRotMatrix();
	std::array<float, 9> obb_2_rot_matrix = obb_2.rotation.getRotMatrix();

	std::array<Point, 15> test = {
	    Point(obb_1_rot_matrix[0], obb_1_rot_matrix[1], obb_1_rot_matrix[2]),
	    Point(obb_1_rot_matrix[3], obb_1_rot_matrix[4], obb_1_rot_matrix[5]),
	    Point(obb_1_rot_matrix[6], obb_1_rot_matrix[7], obb_1_rot_matrix[8]),
	    Point(obb_2_rot_matrix[0], obb_2_rot_matrix[1], obb_2_rot_matrix[2]),
	    Point(obb_2_rot_matrix[3], obb_2_rot_matrix[4], obb_2_rot_matrix[5]),
	    Point(obb_2_rot_matrix[6], obb_2_rot_matrix[7], obb_2_rot_matrix[8])};

	for (int i = 0; i < 3; ++i) {  // Fill out rest of axis
		test[6 + i * 3 + 0] = Point::cross(test[i], test[0]);
		test[6 + i * 3 + 1] = Point::cross(test[i], test[1]);
		test[6 + i * 3 + 2] = Point::cross(test[i], test[2]);
	}

	return std::all_of(std::cbegin(test), std::cend(test), [&obb_1, &obb_2](auto const& t) {
		return overlapOnAxis(obb_1, obb_2, t);
	});
}

constexpr bool intersects(OBB const& obb, Plane const& plane) noexcept
{
	Point rot[] = {
	    Point(obb.rotation[0], 0, 0),
	    Point(0, obb.rotation[1], 0),
	    Point(0, 0, obb.rotation[2]),
	};
	Point normal = plane.normal;

	// Project the half extents of the AABB onto the plane normal
	float p_len = obb.half_size.x * std::fabs(Point::dot(normal, rot[0])) +
	              obb.half_size.y * std::fabs(Point::dot(normal, rot[1])) +
	              obb.half_size.z * std::fabs(Point::dot(normal, rot[2]));
	// Find the distance from the center of the OBB to the plane
	float distance = Point::dot(plane.normal, obb.center) - plane.distance;
	// Intersection occurs if the distance falls within the projected side
	return std::abs(distance) <= p_len;
}

constexpr bool intersects(OBB const& obb, Point const& point) noexcept
{
	return intersects(point, obb);
}

constexpr bool intersects(OBB const& obb, Ray const& ray) noexcept
{
	Point p = obb.center - ray.origin;

	Point X(obb.rotation[0], 0, 0);
	Point Y(0, obb.rotation[1], 0);
	Point Z(0, 0, obb.rotation[2]);

	Point f(Point::dot(X, ray.direction), Point::dot(Y, ray.direction),
	        Point::dot(Z, ray.direction));

	Point e(Point::dot(X, p), Point::dot(Y, p), Point::dot(Z, p));

	float t[6] = {0, 0, 0, 0, 0, 0};
	for (int i = 0; i < 3; ++i) {
		if (0.0 == f[i])  // FIXME: Should be approximate equal?
		{
			if (-e[i] - obb.half_size[i] > 0 || -e[i] + obb.half_size[i] < 0) {
				return false;
			}
			f[i] = 0.00001f;  // Avoid div by 0!
		}

		t[i * 2 + 0] = (e[i] + obb.half_size[i]) / f[i];  // tmin[x, y, z]
		t[i * 2 + 1] = (e[i] - obb.half_size[i]) / f[i];  // tmax[x, y, z]
	}

	float tmin = std::max(std::max(std::min(t[0], t[1]), std::min(t[2], t[3])),
	                      std::min(t[4], t[5]));
	float tmax = std::min(std::min(std::max(t[0], t[1]), std::max(t[2], t[3])),
	                      std::max(t[4], t[5]));

	// if tmax < 0, ray is intersecting AABB
	// but entire AABB is behing it's origin
	if (tmax < 0) {
		return false;
	}
	// if tmin > tmax, ray doesn't intersect AABB
	if (tmin > tmax) {
		return false;
	}
	return true;
}

constexpr bool intersects(OBB const& obb, Sphere const& sphere) noexcept
{
	return intersects(sphere, obb);
}

//
// Frustum
//

constexpr bool intersects(Frustum const& frustum, AABB const& aabb) noexcept
{
	return intersects(aabb, frustum);
}

constexpr bool intersects(Frustum const& frustum, AAEBB const& aaebb) noexcept
{
	return intersects(aaebb, frustum);
}

// constexpr bool intersects(Frustum const& frustum_1, Frustum const& frustum_2) noexcept
// {
// TODO: Implement
// }

// constexpr bool intersects(Frustum const& frustum, LineSegment const& line_segment)
// noexcept
// {
// TODO: Implement
// }

constexpr bool intersects(Frustum const& frustum, OBB const& obb) noexcept
{
	return intersects(obb, frustum);
}

// constexpr bool intersects(Frustum const& frustum, Plane const& plane) noexcept
// {
// TODO: Implement
// }

constexpr bool intersects(Frustum const& frustum, Point const& point) noexcept
{
	return intersects(point, frustum);
}

// constexpr bool intersects(Frustum const& frustum, Ray const& ray) noexcept
// {
// TODO: Implement
// }

constexpr bool intersects(Frustum const& frustum, Sphere const& sphere) noexcept
{
	return intersects(sphere, frustum);
}

//
// Line segment
//

constexpr bool intersects(LineSegment const& line_segment, AABB const& aabb) noexcept
{
	return intersects(aabb, line_segment);
}

constexpr bool intersects(LineSegment const& line_segment, AAEBB const& aaebb) noexcept
{
	return intersects(aaebb, line_segment);
}

// constexpr bool intersects(LineSegment const& line_segment, Frustum const& frustum)
// noexcept
// {
// 	return intersects(frustum, line_segment);
// }

constexpr bool intersects(LineSegment const& line_segment_1,
                          LineSegment const& line_segment_2) noexcept
{
	// Plucker coordinates
	Point d_1 = line_segment_1.end - line_segment_1.start;
	Point d_2 = line_segment_2.end - line_segment_2.start;
	Point m_1 = Point::cross(line_segment_1.start, line_segment_1.end);
	Point m_2 = Point::cross(line_segment_1.start, line_segment_1.end);
	// FIXME: Almost equal?
	return 0 == Point::dot(d_1, m_2) + Point::dot(d_2, m_1);
}

constexpr bool intersects(LineSegment const& line_segment, OBB const& obb) noexcept
{
	return intersects(obb, line_segment);
}

constexpr bool intersects(LineSegment const& line_segment, Plane const& plane) noexcept
{
	Point ab = line_segment.end - line_segment.start;
	float n_A = Point::dot(plane.normal, line_segment.start);
	float n_AB = Point::dot(plane.normal, ab);
	if (0.0 == n_AB)  // FIXME: Almost equal?
	{
		return false;
	}
	float t = (plane.distance - n_A) / n_AB;
	return t >= 0.0 && t <= 1.0;
}

constexpr bool intersects(LineSegment const& line_segment, Point const& point) noexcept
{
	return intersects(point, line_segment);
}

// constexpr bool intersects(LineSegment const& line_segment, Ray const& ray) noexcept
// {
// TODO: Implement
// }

constexpr bool intersects(LineSegment const& line_segment, Sphere const& sphere) noexcept
{
	return intersects(sphere, line_segment);
}

//
// Plane
//

constexpr bool intersects(Plane const& plane, AABB const& aabb) noexcept
{
	return intersects(aabb, plane);
}

constexpr bool intersects(Plane const& plane, AAEBB const& aaebb) noexcept
{
	return intersects(aaebb, plane);
}

// constexpr bool intersects(Plane const& plane, Frustum const& frustum) noexcept
// {
// 	return intersects(frustum, plane);
// }

constexpr bool intersects(Plane const& plane, LineSegment const& line_segment) noexcept
{
	return intersects(line_segment, plane);
}

constexpr bool intersects(Plane const& plane, OBB const& obb) noexcept
{
	return intersects(obb, plane);
}

constexpr bool intersects(Plane const& plane_1, Plane const& plane_2) noexcept
{
	Point d = Point::cross(plane_1.normal, plane_2.normal);

	// FIXME: Almost not equal?
	return 0.0 != Point::dot(d, d);
}

constexpr bool intersects(Plane const& plane, Point const& point) noexcept
{
	return intersects(point, plane);
}

constexpr bool intersects(Plane const& plane, Ray const& ray) noexcept
{
	float nd = Point::dot(ray.direction, plane.normal);
	float pn = Point::dot(ray.origin, plane.normal);
	if (nd >= 0.0f) {
		return false;
	}
	float t = (plane.distance - pn) / nd;
	return t >= 0.0;
}

constexpr bool intersects(Plane const& plane, Sphere const& sphere) noexcept
{
	return intersects(sphere, plane);
}

//
// Ray
//

constexpr bool intersects(Ray const& ray, AABB const& aabb) noexcept
{
	return intersects(aabb, ray);
}

constexpr bool intersects(Ray const& ray, AAEBB const& aaebb) noexcept
{
	return intersects(aaebb, ray);
}

// constexpr bool intersects(Ray const& ray, Frustum const& frustum) noexcept
// {
// 	return intersects(frustum, ray);
// }

// constexpr bool intersects(Ray const& ray, LineSegment const& line_segment) noexcept
// {
// 	return intersects(line_segment, ray);
// }

constexpr bool intersects(Ray const& ray, OBB const& obb) noexcept
{
	return intersects(obb, ray);
}

constexpr bool intersects(Ray const& ray, Plane const& plane) noexcept
{
	return intersects(plane, ray);
}

constexpr bool intersects(Ray const& ray, Point const& point) noexcept
{
	return intersects(point, ray);
}

// constexpr bool intersects(Ray const& ray_1, Ray const& ray_2) noexcept
// {
// TODO: Implement
// }

constexpr bool intersects(Ray const& ray, Sphere const& sphere) noexcept
{
	return intersects(sphere, ray);
}

//
// Bounding volume
//

// template <class Geometry>
// constexpr bool intersects(BoundingVolume const& bounding_volume, Geometry const&
// geometry) noexcept
// {
// 	return std::any_of(
// 	    std::cbegin(bounding_volume), std::cend(bounding_volume),
// 	    [&geometry](auto const& bv) {
// 		    return std::visit(
// 		        [&geometry](auto const& bv) { return intersects(bv, geometry); }, bv);
// 	    });
// }

// template <class Geometry>
// constexpr bool intersects(Geometry const& geometry, BoundingVolume const&
// bounding_volume) noexcept
// {
// 	return intersects(bounding_volume, geometry);
// }

// constexpr bool intersects(BoundingVolume const& bounding_volume_1,
//                           BoundingVolume const& bounding_volume_2) noexcept
// {
// 	return std::any_of(
// 	    std::cbegin(bounding_volume_2),
// 	    std::cend(bounding_volume_2)[&bounding_volume_1](auto const& bv) {
// 		    return std::visit(
// 		        [&bounding_volume_1](auto const& bv) {
// 			        return intersects(bounding_volume_1, bv);
// 		        },
// 		        bv);
// 	    });
// }
}  // namespace ufo::geometry

#endif  // UFO_GEOMETRY_INTERSECTS_H