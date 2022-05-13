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

#ifndef UFO_GEOMETRY_CONTAINS_H
#define UFO_GEOMETRY_CONTAINS_H

// UFO
#include <ufo/geometry/bounding_volume.h>
#include <ufo/geometry/minimum_distance.h>

namespace ufo::geometry
{
//
// AABB
//

/*!
 * @brief Check if a contains b, that is b is within a.
 *
 * @param a,b The two geometry object to check.
 * @return Whether a contains b.
 */
constexpr bool contains(AABB const& a, AABB const& b) noexcept
{
	Point a_min = a.min();
	Point a_max = a.max();
	Point b_min = b.min();
	Point b_max = b.max();
	return a_min.x <= b_max.x && a_min.y <= b_max.y && a_min.z <= b_max.z &&
	       a_max.x >= b_min.x && a_max.y >= b_min.y && a_max.z >= b_min.z;
}

/*!
 * @brief Check if a contains b, that is b is within a.
 *
 * @param a,b The two geometry object to check.
 * @return Whether a contains b.
 */
constexpr bool contains(AABB const& a, AAEBB const& b) noexcept
{
	Point a_min = a.min();
	Point a_max = a.max();
	Point b_min = b.min();
	Point b_max = b.max();
	return a_min.x <= b_max.x && a_min.y <= b_max.y && a_min.z <= b_max.z &&
	       a_max.x >= b_min.x && a_max.y >= b_min.y && a_max.z >= b_min.z;
}

// constexpr bool contains(AABB const& a, Frustum const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(AABB const& a, LineSegment const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(AABB const& a, OBB const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(AABB const& a, Plane const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Check if a contains b, that is b is within a.
 *
 * @param a,b The two geometry object to check.
 * @return Whether a contains b.
 */
constexpr bool contains(AABB const& a, Point const& b) noexcept
{
	Point min = a.min();
	Point max = a.max();
	return min.x <= b.x && min.y <= b.y && min.z <= b.z && max.x >= b.x && max.y >= b.y &&
	       max.z >= b.z;
}

// constexpr bool contains(AABB const& a, Ray const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(AABB const& a, Sphere const& b) noexcept
// {
// 	// TODO: Implement
// }

//
// AAEBB
//

/*!
 * @brief Check if a contains b, that is b is within a.
 *
 * @param a,b The two geometry object to check.
 * @return Whether a contains b.
 */
constexpr bool contains(AAEBB const& a, AABB const& b) noexcept
{
	Point a_min = a.min();
	Point a_max = a.max();
	Point b_min = b.min();
	Point b_max = b.max();
	return a_min.x <= b_max.x && a_min.y <= b_max.y && a_min.z <= b_max.z &&
	       a_max.x >= b_min.x && a_max.y >= b_min.y && a_max.z >= b_min.z;
}

/*!
 * @brief Check if a contains b, that is b is within a.
 *
 * @param a,b The two geometry object to check.
 * @return Whether a contains b.
 */
constexpr bool contains(AAEBB const& a, AAEBB const& b) noexcept
{
	Point a_min = a.min();
	Point a_max = a.max();
	Point b_min = b.min();
	Point b_max = b.max();
	return a_min.x <= b_max.x && a_min.y <= b_max.y && a_min.z <= b_max.z &&
	       a_max.x >= b_min.x && a_max.y >= b_min.y && a_max.z >= b_min.z;
}

// constexpr bool contains(AAEBB const& a, Frustum const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(AAEBB const& a, LineSegment const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(AAEBB const& a, OBB const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(AAEBB const& a, Plane const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Check if a contains b, that is b is within a.
 *
 * @param a,b The two geometry object to check.
 * @return Whether a contains b.
 */
constexpr bool contains(AAEBB const& a, Point const& b) noexcept
{
	Point min = a.min();
	Point max = a.max();
	return min.x <= b.x && min.y <= b.y && min.z <= b.z && max.x >= b.x && max.y >= b.y &&
	       max.z >= b.z;
}

// constexpr bool contains(AAEBB const& a, Ray const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(AAEBB const& a, Sphere const& b) noexcept
// {
// 	// TODO: Implement
// }

//
// Frustum
//

// constexpr bool contains(Frustum const& a, AABB const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Frustum const& a, AAEBB const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Frustum const& a, Frustum const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Frustum const& a, LineSegment const& b)
// noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Frustum const& a, OBB const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Frustum const& a, Plane const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Frustum const& a, Point const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Frustum const& a, Ray const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Frustum const& a, Sphere const& b) noexcept
// {
// 	// TODO: Implement
// }

//
// Line segment
//

// constexpr bool contains(LineSegment const& a, AABB const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(LineSegment const& a, AAEBB const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(LineSegment const& a, Frustum const& b)
// noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(LineSegment const& a,
//                         LineSegment const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(LineSegment const& a, OBB const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(LineSegment const& a, Plane const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(LineSegment const& a, Point const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(LineSegment const& a, Ray const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(LineSegment const& a, Sphere const& b) noexcept
// {
// 	// TODO: Implement
// }

//
// OBB
//

// constexpr bool contains(OBB const& a, AABB const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(OBB const& a, AAEBB const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(OBB const& a, Frustum const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(OBB const& a, LineSegment const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(OBB const& a, OBB const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(OBB const& a, Plane const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(OBB const& a, Point const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(OBB const& a, Ray const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(OBB const& a, Sphere const& b) noexcept
// {
// 	// TODO: Implement
// }

//
// Plane
//

// constexpr bool contains(Plane const& a, AABB const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Plane const& a, AAEBB const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Plane const& a, Frustum const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Plane const& a, LineSegment const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Plane const& a, OBB const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Plane const& a, Plane const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Plane const& a, Point const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Plane const& a, Ray const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Plane const& a, Sphere const& b) noexcept
// {
// 	// TODO: Implement
// }

//
// Point
//

/*!
 * @brief Check if a contains b, that is b is within a.
 *
 * @param a,b The two geometry object to check.
 * @return Whether a contains b.
 */
constexpr bool contains(Point const& a, AABB const& b) noexcept
{
	return a == b.center && 0 == b.half_size.x && 0 == b.half_size.y && 0 == b.half_size.z;
}

/*!
 * @brief Check if a contains b, that is b is within a.
 *
 * @param a,b The two geometry object to check.
 * @return Whether a contains b.
 */
constexpr bool contains(Point const& a, AAEBB const& b) noexcept
{
	return a == b.center && 0 == b.half_size;
}

/*!
 * @brief Check if a contains b, that is b is within a.
 *
 * @param a,b The two geometry object to check.
 * @return Whether a contains b.
 */
constexpr bool contains(Point const& a, Frustum const& b) noexcept
{
	return false;  // FIXME: Check
}

/*!
 * @brief Check if a contains b, that is b is within a.
 *
 * @param a,b The two geometry object to check.
 * @return Whether a contains b.
 */
constexpr bool contains(Point const& a, LineSegment const& b) noexcept
{
	return a == b.start && a == b.end;
}

/*!
 * @brief Check if a contains b, that is b is within a.
 *
 * @param a,b The two geometry object to check.
 * @return Whether a contains b.
 */
constexpr bool contains(Point const& a, OBB const& b) noexcept
{
	return a == b.center && 0 == b.half_size.x && 0 == b.half_size.y && 0 == b.half_size.z;
}

/*!
 * @brief Check if a contains b, that is b is within a.
 *
 * @param a,b The two geometry object to check.
 * @return Whether a contains b.
 */
constexpr bool contains(Point const& a, Plane const& b) noexcept
{
	return false;  // FIXME: Check
}

/*!
 * @brief Check if a contains b, that is b is within a.
 *
 * @param a,b The two geometry object to check.
 * @return Whether a contains b.
 */
constexpr bool contains(Point const& a, Point const& b) noexcept { return a == b; }

/*!
 * @brief Check if a contains b, that is b is within a.
 *
 * @param a,b The two geometry object to check.
 * @return Whether a contains b.
 */
constexpr bool contains(Point const& a, Ray const& b) noexcept { return false; }

/*!
 * @brief Check if a contains b, that is b is within a.
 *
 * @param a,b The two geometry object to check.
 * @return Whether a contains b.
 */
constexpr bool contains(Point const& a, Sphere const& b) noexcept
{
	return 0 == b.radius && a == b.center;
}

//
// Ray
//

// constexpr bool contains(Ray const& a, AABB const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Ray const& a, AAEBB const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Ray const& a, Frustum const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Ray const& a, LineSegment const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Ray const& a, OBB const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Ray const& a, Plane const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Ray const& a, Point const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Ray const& a, Ray const& b) noexcept
// {
// 	// TODO: Implement
// }

// constexpr bool contains(Ray const& a, Sphere const& b) noexcept
// {
// 	// TODO: Implement
// }

//
// Sphere
//

/*!
 * @brief Check if a contains b, that is b is within a.
 *
 * @param a,b The two geometry object to check.
 * @return Whether a contains b.
 */
constexpr bool contains(Sphere const& a, Point const& b) noexcept
{
	return distance(a, b) <= a.radius;
}

/*!
 * @brief Check if a contains b, that is b is within a.
 *
 * @param a,b The two geometry object to check.
 * @return Whether a contains b.
 */
constexpr bool contains(Sphere const& a, AABB const& b) noexcept
{
	return contains(a, b.min()) && contains(a, b.max());
}

/*!
 * @brief Check if a contains b, that is b is within a.
 *
 * @param a,b The two geometry object to check.
 * @return Whether a contains b.
 */
constexpr bool contains(Sphere const& a, AAEBB const& b) noexcept
{
	return contains(a, b.min()) && contains(a, b.max());
}

// constexpr bool contains(Sphere const& a, Frustum const& b)
// {
// 	// TODO: Implement
// }

/*!
 * @brief Check if a contains b, that is b is within a.
 *
 * @param a,b The two geometry object to check.
 * @return Whether a contains b.
 */
constexpr bool contains(Sphere const& a, LineSegment const& b) noexcept
{
	return contains(a, b.start) && contains(a, b.end);
}

/*!
 * @brief Check if a contains b, that is b is within a.
 *
 * @param a,b The two geometry object to check.
 * @return Whether a contains b.
 */
constexpr bool contains(Sphere const& a, OBB const& b) noexcept
{
	return contains(a, b.min()) && contains(a, b.max());
}

/*!
 * @brief Check if a contains b, that is b is within a.
 *
 * @param a,b The two geometry object to check.
 * @return Whether a contains b.
 */
constexpr bool contains(Sphere const& a, Plane const& b) noexcept { return false; }

/*!
 * @brief Check if a contains b, that is b is within a.
 *
 * @param a,b The two geometry object to check.
 * @return Whether a contains b.
 */
constexpr bool contains(Sphere const& a, Ray const& b) noexcept { return false; }

/*!
 * @brief Check if a contains b, that is b is within a.
 *
 * @param a,b The two geometry object to check.
 * @return Whether a contains b.
 */
constexpr bool contains(Sphere const& a, Sphere const& b) noexcept
{
	return a.radius >= b.radius + a.center.distance(b.center);
}

//
// Bounding volume
//

// template <class Geometry>
// bool contains(BoundingVolume const& a, Geometry const& b) noexcept
// {
// 	throw std::logic_error("Function not yet implemented.");
// 	// TODO: Implement
// }

// template <class Geometry>
// bool contains(Geometry const& a, BoundingVolume const& b) noexcept
// {
// 	throw std::logic_error("Function not yet implemented.");
// 	// TODO: Implement
// }

// bool contains(BoundingVolume const& a, BoundingVolume const& b);
}  // namespace ufo::geometry

#endif  // UFO_GEOMETRY_CONTAINS_H