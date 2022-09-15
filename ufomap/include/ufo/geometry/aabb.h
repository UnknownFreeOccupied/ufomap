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

#ifndef UFO_GEOMETRY_AXIS_ALIGNED_BOUNDING_BOX_H
#define UFO_GEOMETRY_AXIS_ALIGNED_BOUNDING_BOX_H

#include <ufo/geometry/point.h>

namespace ufo::geometry
{
/*!
 * @brief Something something
 * @author Daniel Duberg
 *
 */
struct AABB {
	// The center point of the AABB.
	Point center;
	// Half the size of a side of the AABB.
	Point half_size;

	constexpr AABB() noexcept = default;

	constexpr AABB(Point center, float half_size) noexcept
	    : center(center), half_size(half_size, half_size, half_size)
	{
	}

	constexpr AABB(Point min, Point max) noexcept : half_size((max - min) / 2.0)
	{
		center = min + half_size;
	}

	/*!
	 * @brief Compare two AABBs.
	 *
	 * @param lhs,rhs The AABBs to compare.
	 * @return Whether lhs and rhs are equal.
	 */
	friend constexpr bool operator==(AABB const& lhs, AABB const& rhs) noexcept
	{
		return lhs.center == rhs.center && lhs.half_size == rhs.half_size;
	}

	/*!
	 * @brief Compare two AABBs.
	 *
	 * @param lhs,rhs The AABBs to compare.
	 * @return Whether lhs and rhs are different.
	 */
	friend constexpr bool operator!=(AABB const& lhs, AABB const& rhs) noexcept
	{
		return !(lhs == rhs);
	}

	/*!
	 * @brief
	 *
	 * @return
	 */
	constexpr Point min() const noexcept { return center - half_size; }

	constexpr Point max() const noexcept { return center + half_size; }
};
}  // namespace ufo::geometry

#endif  // UFO_GEOMETRY_AXIS_ALIGNED_BOUNDING_BOX_H