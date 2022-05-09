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

#ifndef UFO_GEOMETRY_FRUSTUM_H
#define UFO_GEOMETRY_FRUSTUM_H

// UFO
#include <ufo/geometry/plane.h>
#include <ufo/geometry/point.h>

// STL
#include <array>
#include <cmath>

namespace ufo::geometry
{
struct Frustum {
	Plane top;
	Plane bottom;
	Plane left;
	Plane right;
	Plane far;
	Plane near;

	constexpr Frustum() noexcept = default;

	constexpr Frustum(Point const& pos, Point const& target, Point const& up,
	                  float vertical_angle, float horizontal_angle, float near_distance,
	                  float far_distance) noexcept
	    : position(pos),
	      target(target),
	      up(up),
	      vertical_angle(vertical_angle),
	      horizontal_angle(horizontal_angle),
	      near_distance(near_distance),
	      far_distance(far_distance)
	{
		float ratio = horizontal_angle / vertical_angle;

		// TODO: Check if correct
		float tang = std::tan(vertical_angle * 0.5);
		float near_height = near_distance * tang;
		float near_width = near_height * ratio;
		float far_height = far_distance * tang;
		float far_width = far_height * ratio;

		Point Z = pos - target;
		Z.normalize();

		Point X = Point::cross(up, Z);
		X.normalize();

		Point Y = Point::cross(Z, X);

		Point nc = pos - Z * near_distance;
		Point fc = pos - Z * far_distance;

		Point near_top_left = nc + Y * near_height - X * near_width;
		Point near_top_right = nc + Y * near_height + X * near_width;
		Point near_bottom_left = nc - Y * near_height - X * near_width;
		Point near_bottom_right = nc - Y * near_height + X * near_width;

		Point far_top_left = fc + Y * far_height - X * far_width;
		Point far_top_right = fc + Y * far_height + X * far_width;
		Point far_bottom_left = fc - Y * far_height - X * far_width;
		Point far_bottom_right = fc - Y * far_height + X * far_width;

		top = Plane(near_top_right, near_top_left, far_top_left);
		bottom = Plane(near_bottom_left, near_bottom_right, far_bottom_right);
		left = Plane(near_top_left, near_bottom_left, far_bottom_left);
		right = Plane(near_bottom_right, near_top_right, far_bottom_right);
		near = Plane(near_top_left, near_top_right, near_bottom_right);
		far = Plane(far_top_right, far_top_left, far_bottom_left);
	}

	constexpr bool operator==(Frustum const& rhs) const noexcept
	{
		return rhs.top == top && rhs.bottom == bottom && rhs.left == left &&
		       rhs.right == right && rhs.far == far && rhs.near == near;
	}

	constexpr bool operator!=(Frustum const& rhs) const noexcept { return !(*this == rhs); }

	constexpr Plane operator[](std::size_t idx) const noexcept { return *(&top + idx); }

	constexpr Plane& operator[](std::size_t idx) noexcept { return *(&top + idx); }

	constexpr Point min() const noexcept
	{
		// TODO: Implement
		return Point();
	}

	constexpr Point max() const noexcept
	{
		// TODO: Implement
		return Point();
	}

 private:
	Point position;
	Point target;
	Point up;
	float vertical_angle = 0;
	float horizontal_angle = 0;
	float near_distance = 0;
	float far_distance = 0;
};
}  // namespace ufo::geometry

#endif  // UFO_GEOMETRY_FRUSTUM_H