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
#include <ufo/geometry/bounding_volume.h>

namespace ufo::geometry
{

//
// AABB
//

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(AABB const& a, AABB const& b) noexcept
{
	Point a_min = a.min();
	Point a_max = a.max();
	Point b_min = b.min();
	Point b_max = b.max();

	float result = 0;
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

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(AABB const& a, AABB const& b) noexcept
{
	return std::sqrt(squaredDistance(a, b));
}

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(AABB const& a, AAEBB const& b) noexcept
{
	Point a_min = a.min();
	Point a_max = a.max();
	Point b_min = b.min();
	Point b_max = b.max();

	float result = 0;
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

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(AABB const& a, AAEBB const& b) noexcept
{
	return std::sqrt(squaredDistance(a, b));
}

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(AABB const& a, Frustum const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(AABB const& a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return std::sqrt(squaredDistance(a, b));
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(AABB const& a, LineSegment const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(AABB const& a, LineSegment const& b) noexcept
// {
// 	// FIXME: Enable
// 	return std::sqrt(squaredDistance(a, b));
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(AABB const& a, OBB const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(AABB const& a, OBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return std::sqrt(squaredDistance(a, b));
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(AABB const& a, Plane const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(AABB const& a, Plane const& b) noexcept
// {
// 	// FIXME: Enable
// 	return std::sqrt(squaredDistance(a, b));
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(AABB const& a, Point const& b) noexcept
{
	return b.squaredDistance(Point::clamp(b, a.min(), a.max()));
}

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(AABB const& a, Point const& b) noexcept
{
	return std::sqrt(squaredDistance(a, b));
}

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(AABB const& a, Ray const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(AABB const& a, Ray const& ray)
// {
// 	// FIXME: Enable
// 	return std::sqrt(squaredDistance(a, b));
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(AABB const& a, Sphere const& b) noexcept
{
	return std::fdim(distance(a, b.center), b.radius);
}

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(AABB const& a, Sphere const& b) noexcept
{
	// FIXME: Implement better
	auto dist = distance(a, b);
	return dist * dist;
}

//
// AAEBB
//

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(AAEBB const& a, AABB const& b) noexcept
{
	return squaredDistance(b, a);
}

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(AAEBB const& a, AABB const& b) noexcept
{
	return distance(b, a);
}

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(AAEBB const& a, AAEBB const& b) noexcept
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

	float hs = a.half_size + b.half_size;
	float result = 0;
	for (size_t i = 0; i != 3; ++i) {
		float tmp = std::fdim(std::abs(a.center[i] - b.center[i]), hs);
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

	// Point a = aaebb_1.min() - aaebb_2.max();
	// Point b = aaebb_2.min() - aaebb_1.max();

	// float result = 0.0;
	// for (size_t i = 0; i != 3; ++i) {
	// 	float d_1 = std::max(0.0f, a[i]);
	// 	float d_2 = std::max(0.0f, b[i]);
	// 	result += (d_1 * d_1) + (d_2 * d_2);
	// }

	// return result;
}

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(AAEBB const& a, AAEBB const& b) noexcept
{
	return std::sqrt(squaredDistance(a, b));
}

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(AAEBB const& a, Frustum const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(AAEBB const& a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(AAEBB const& a, LineSegment const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(AAEBB const& a, LineSegment const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(AAEBB const& a, OBB const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(AAEBB const& a, OBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(AAEBB const& a, Plane const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(AAEBB const& a, Plane const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(AAEBB const& a, Point const& b) noexcept
{
	return b.squaredDistance(Point::clamp(b, a.min(), a.max()));
}

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(AAEBB const& a, Point const& b) noexcept
{
	return b.distance(Point::clamp(b, a.min(), a.max()));
}

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(AAEBB const& a, Ray const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(AAEBB const& a, Ray const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(AAEBB const& a, Sphere const& b) noexcept
{
	return std::fdim(distance(a, b.center), b.radius);
}

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(AAEBB const& a, Sphere const& b) noexcept
{
	// FIXME: Implement better
	auto dist = distance(a, b);
	return dist * dist;
}

//
// Frustum
//

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Frustum const& a, AABB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Frustum const& a, AABB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Frustum const& a, AAEBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Frustum const& a, AAEBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Frustum const& a, Frustum const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Frustum const& a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Frustum const& a, LineSegment const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Frustum const& a, LineSegment const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Frustum const& a, OBB const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Frustum const& a, OBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Frustum const& a, Plane const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Frustum const& a, Plane const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Frustum const& a, Point const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Frustum const& a, Point const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Frustum const& a, Ray const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Frustum const& a, Ray const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Frustum const& a, Sphere const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Frustum const& a, Sphere const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

//
// Line segment
//

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(LineSegment const& a, AABB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(LineSegment const& a, AABB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(LineSegment const& a, AAEBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(LineSegment const& a, AAEBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(LineSegment const& a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(LineSegment const& a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(LineSegment const& a, LineSegment const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(LineSegment const& a, LineSegment const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(LineSegment const& a, OBB const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(LineSegment const& a, OBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(LineSegment const& a, Plane const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(LineSegment const& a, Plane const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(LineSegment const& a, Point const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(LineSegment const& a, Point const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(LineSegment const& a, Ray const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(LineSegment const& a, Ray const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(LineSegment const& a, Sphere const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(LineSegment const& a, Sphere const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

//
// OBB
//

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(OBB const& a, AABB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(OBB const& a, AABB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(OBB const& a, AAEBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(OBB const& a, AAEBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(OBB const& a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(OBB const& a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(OBB const& a, AAEBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(OBB const& a, LineSegment const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(OBB const& a, OBB const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(OBB const& a, OBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(OBB const& a, Plane const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(OBB const& a, Plane const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(OBB const& a, Point const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(OBB const& a, Point const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(OBB const& a, Ray const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(OBB const& a, Ray const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(OBB const& a, Sphere const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(OBB const& a, Sphere const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

//
// Plane
//

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Plane const& a, AABB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Plane const& a, AABB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Plane const& a, AAEBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Plane const& a, AAEBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Plane const& a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Plane const& a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Plane const& a, LineSegment const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Plane const& a, LineSegment const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Plane const& a, OBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Plane const& a, OBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Plane const& a, Plane const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Plane const& a, Plane const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Plane const& a, Point const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Plane const& a, Point const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Plane const& a, Ray const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Plane const& a, Ray const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Plane const& a, Sphere const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Plane const& a, Sphere const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

//
// Point
//

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(Point const& a, AABB const& b) noexcept
{
	return squaredDistance(b, a);
}

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(Point const& a, AABB const& b) noexcept
{
	return distance(b, a);
}

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(Point const& a, AAEBB const& b) noexcept
{
	return squaredDistance(b, a);
}

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(Point const& a, AAEBB const& b) noexcept
{
	return distance(b, a);
}

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Point const& a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Point const& a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Point const& a, LineSegment const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Point const& a, LineSegment const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Point const& a, OBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Point const& a, OBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Point const& a, Plane const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Point const& a, Plane const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(Point const& a, Point const& b) noexcept
{
	return a.squaredDistance(b);
}

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(Point const& a, Point const& b) noexcept
{
	return a.distance(b);
}

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Point const& a, Ray const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Point const& a, Ray const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(Point const& a, Sphere const& b) noexcept
{
	return std::fdim(a.distance(b.center), b.radius);
}

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(Point const& a, Sphere const& b) noexcept
{
	auto dist = distance(a, b);
	return dist * dist;
}

//
// Ray
//

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Ray const& a, AABB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Ray const& a, AABB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Ray const& a, AAEBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Ray const& a, AAEBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Ray const& a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Ray const& a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Ray const& a, LineSegment const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Ray const& a, LineSegment const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Ray const& a, OBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Ray const& a, OBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Ray const& a, Plane const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Ray const& a, Plane const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Ray const& a, Point const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Ray const& a, Point const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Ray const& a, Ray const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Ray const& a, Ray const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Ray const& a, AABB const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Ray const& a, Sphere const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

//
// Sphere
//

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(Sphere const& a, AABB const& b) noexcept
{
	return squaredDistance(b, a);
}

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(Sphere const& a, AABB const& b) noexcept
{
	return distance(b, a);
}

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(Sphere const& a, AAEBB const& b) noexcept
{
	return squaredDistance(b, a);
}

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(Sphere const& a, AAEBB const& b) noexcept
{
	return distance(b, a);
}

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Sphere const& a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Sphere const& a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Sphere const& a, LineSegment const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Sphere const& a, LineSegment const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Sphere const& a, OBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Sphere const& a, OBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Sphere const& a, Plane const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Sphere const& a, Plane const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(Sphere const& a, Point const& b) noexcept
{
	return distance(b, a);
}

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(Sphere const& a, Point const& b) noexcept
{
	return distance(b, a);
}

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Sphere const& a, Ray const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Sphere const& a, Ray const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(Sphere const& a, Sphere const& b) noexcept
{
	return std::fdim(a.center.distance(b.center), a.radius + b.radius);
}

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(Sphere const& a, Sphere const& b) noexcept
{
	auto dist = distance(a, b);
	return dist * dist;
}

}  // namespace ufo::geometry

#endif  // UFO_GEOMETRY_MINIMUM_DISTANCE_H