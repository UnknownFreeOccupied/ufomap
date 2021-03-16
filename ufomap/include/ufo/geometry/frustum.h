/**
 * UFOGeometry - the geometry library used in UFO
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufogeometry
 * License: BSD 3
 *
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

// STD
#include <array>
#include <cmath>

namespace ufo::geometry
{
struct Frustum {
	std::array<Plane, 6> planes;

	Frustum() {}

	Frustum(Frustum const& frustum) : planes(frustum.planes) {}

	// TODO: Horizontal or vertical angle?
	Frustum(Point const& pos, Point const& target, Point const& up, double vertical_angle,
	        double horizontal_angle, double near_distance, double far_distance)
	{
		double ratio = horizontal_angle / vertical_angle;

		// TODO: Check if correct
		double tang = tan(vertical_angle * 0.5);
		double near_height = near_distance * tang;
		double near_width = near_height * ratio;
		double far_height = far_distance * tang;
		double far_width = far_height * ratio;

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

		top() = Plane(near_top_right, near_top_left, far_top_left);
		bottom() = Plane(near_bottom_left, near_bottom_right, far_bottom_right);
		left() = Plane(near_top_left, near_bottom_left, far_bottom_left);
		right() = Plane(near_bottom_right, near_top_right, far_bottom_right);
		near() = Plane(near_top_left, near_top_right, near_bottom_right);
		far() = Plane(far_top_right, far_top_left, far_bottom_left);
	}

	Plane const& top() const { return planes[0]; }

	Plane& top() { return planes[0]; }

	Plane const& bottom() const { return planes[1]; }

	Plane& bottom() { return planes[1]; }

	Plane const& left() const { return planes[2]; }

	Plane& left() { return planes[2]; }

	Plane const& right() const { return planes[3]; }

	Plane& right() { return planes[3]; }

	Plane const& near() const { return planes[4]; }

	Plane& near() { return planes[4]; }

	Plane const& far() const { return planes[5]; }

	Plane& far() { return planes[5]; }
};
}  // namespace ufo::geometry

#endif  // UFO_GEOMETRY_FRUSTUM_H