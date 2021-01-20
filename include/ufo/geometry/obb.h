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

#ifndef UFO_GEOMETRY_ORIENTED_BOUNDING_BOX_H
#define UFO_GEOMETRY_ORIENTED_BOUNDING_BOX_H

// UFO
#include <ufo/geometry/point.h>
#include <ufo/math/quaternion.h>

// STD
#include <stdexcept>

namespace ufo::geometry
{
struct OBB {
	Point center;
	Point half_size;
	math::Quaternion rotation;

	OBB() {}

	OBB(OBB const& obb)
	    : center(obb.center), half_size(obb.half_size), rotation(obb.rotation)
	{
	}

	OBB(Point const& center, Point const& half_size) : center(center), half_size(half_size)
	{
	}

	OBB(Point const& center, Point const& half_size, math::Quaternion const& rotation)
	    : center(center), half_size(half_size), rotation(rotation)
	{
	}

	OBB(Point const& center, Point const& half_size, Point const& rotation)
	    : center(center),
	      half_size(half_size),
	      rotation(rotation[0], rotation[1], rotation[2])
	{
	}

	Point getMin() const
	{
		Point rot_half_size = rotation.rotate(half_size);
		Point corners[8]{
		    Point(center.x() - rot_half_size.x(), center.y() - rot_half_size.y(),
		          center.z() - rot_half_size.z()),
		    Point(center.x() - rot_half_size.x(), center.y() - rot_half_size.y(),
		          center.z() + rot_half_size.z()),
		    Point(center.x() - rot_half_size.x(), center.y() + rot_half_size.y(),
		          center.z() - rot_half_size.z()),
		    Point(center.x() - rot_half_size.x(), center.y() + rot_half_size.y(),
		          center.z() + rot_half_size.z()),
		    Point(center.x() + rot_half_size.x(), center.y() - rot_half_size.y(),
		          center.z() - rot_half_size.z()),
		    Point(center.x() + rot_half_size.x(), center.y() - rot_half_size.y(),
		          center.z() + rot_half_size.z()),
		    Point(center.x() + rot_half_size.x(), center.y() + rot_half_size.y(),
		          center.z() - rot_half_size.z()),
		    Point(center.x() + rot_half_size.x(), center.y() + rot_half_size.y(),
		          center.z() + rot_half_size.z()),
		};

		Point minimum = corners[0];
		for (int i = 1; i < 8; ++i) {
			minimum.x() = std::min(minimum.x(), corners[i].x());
			minimum.y() = std::min(minimum.y(), corners[i].y());
			minimum.z() = std::min(minimum.z(), corners[i].z());
		}
		return minimum;
	}

	Point getMax() const
	{
		Point rot_half_size = rotation.rotate(half_size);
		Point corners[8]{
		    Point(center.x() - rot_half_size.x(), center.y() - rot_half_size.y(),
		          center.z() - rot_half_size.z()),
		    Point(center.x() - rot_half_size.x(), center.y() - rot_half_size.y(),
		          center.z() + rot_half_size.z()),
		    Point(center.x() - rot_half_size.x(), center.y() + rot_half_size.y(),
		          center.z() - rot_half_size.z()),
		    Point(center.x() - rot_half_size.x(), center.y() + rot_half_size.y(),
		          center.z() + rot_half_size.z()),
		    Point(center.x() + rot_half_size.x(), center.y() - rot_half_size.y(),
		          center.z() - rot_half_size.z()),
		    Point(center.x() + rot_half_size.x(), center.y() - rot_half_size.y(),
		          center.z() + rot_half_size.z()),
		    Point(center.x() + rot_half_size.x(), center.y() + rot_half_size.y(),
		          center.z() - rot_half_size.z()),
		    Point(center.x() + rot_half_size.x(), center.y() + rot_half_size.y(),
		          center.z() + rot_half_size.z()),
		};

		Point maximum = corners[0];
		for (int i = 1; i < 8; ++i) {
			maximum.x() = std::max(maximum.x(), corners[i].x());
			maximum.y() = std::max(maximum.y(), corners[i].y());
			maximum.z() = std::max(maximum.z(), corners[i].z());
		}
		return maximum;
	}
};
}  // namespace ufo::geometry

#endif  // UFO_GEOMETRY_ORIENTED_BOUNDING_BOX_H