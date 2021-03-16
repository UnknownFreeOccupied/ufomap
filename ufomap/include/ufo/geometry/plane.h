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

#ifndef UFO_GEOMETRY_PLANE_H
#define UFO_GEOMETRY_PLANE_H

#include <ufo/geometry/point.h>

namespace ufo::geometry
{
struct Plane {
	Point normal;
	double distance;

	Plane() : normal(1.0, 0.0, 0.0) {}

	Plane(Plane const& plane) : normal(plane.normal), distance(plane.distance) {}

	Plane(Point const& normal, double distance) : normal(normal), distance(distance) {}

	Plane(Point const& v_1, Point const& v_2, Point const& v_3)
	{
		Point aux_1 = v_1 - v_2;
		Point aux_2 = v_3 - v_2;
		normal = aux_2.cross(aux_1);
		normal.normalize();
		distance = -normal.dot(v_2);
	}

	Plane& operator=(Plane const& rhs)
	{
		normal = rhs.normal;
		distance = rhs.distance;
		return *this;
	}
};
}  // namespace ufo::geometry

#endif  // UFO_GEOMETRY_PLANE_H