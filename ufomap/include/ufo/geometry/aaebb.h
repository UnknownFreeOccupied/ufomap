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

#ifndef UFO_GEOMETRY_AXIS_ALIGNED_EQUAL_BOUNDING_BOX_H
#define UFO_GEOMETRY_AXIS_ALIGNED_EQUAL_BOUNDING_BOX_H

#include <ufo/geometry/point.h>

namespace ufo::geometry
{
struct AAEBB {
	std::array<float, 4> values;

	inline AAEBB() = default;

	inline AAEBB(Point const& center, float half_size)
	    : values{center[0], center[1], center[2], half_size}
	{
	}

	inline AAEBB(float center_x, float center_y, float center_z, float half_size)
	    : values{center_x, center_y, center_z, half_size}
	{
	}

	inline float operator[](size_t idx) const { return values[idx]; }
	inline float& operator[](size_t idx) { return values[idx]; }

	constexpr float x() const { return values[0]; }
	constexpr float& x() { return values[0]; }

	constexpr float y() const { return values[1]; }
	constexpr float& y() { return values[1]; }

	constexpr float z() const { return values[2]; }
	constexpr float& z() { return values[2]; }

	constexpr float halfSize() const { return values[3]; }
	constexpr float& halfSize() { return values[3]; }

	inline Point center() const { return Point(x(), y(), z()); }

	inline Point getMin() const
	{
		return Point(x() - halfSize(), y() - halfSize(), z() - halfSize());
	}

	inline Point getMax() const
	{
		return Point(x() + halfSize(), y() + halfSize(), z() + halfSize());
	}
};
}  // namespace ufo::geometry

#endif  // UFO_GEOMETRY_AXIS_ALIGNED_EQUAL_BOUNDING_BOX_H