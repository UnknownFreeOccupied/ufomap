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

#ifndef UFO_MAP_INTEGRATION_POINT_H
#define UFO_MAP_INTEGRATION_POINT_H

// UFO
#include <ufo/map/code.h>

// STL
#include <algorithm>
#include <initializer_list>
#include <vector>

namespace ufo::map
{

template <class P>
struct ValidPoint : P {
	bool valid = true;

	constexpr ValidPoint(P const& point, bool valid = true) : P(point), valid(valid) {}
};

template <class P>
struct IntegrationPointSmall {
	ValidPoint<P> point;
	Code code;

	constexpr IntegrationPointSmall(Code const code, P const& point,
	                                bool const valid = true)
	    : code(code), point(point, valid)
	{
	}

	[[nodiscard]] bool valid() const { return point.valid; }

	bool operator==(IntegrationPointSmall const& rhs) const
	{
		return code.code() == rhs.code.code();
	}

	bool operator!=(IntegrationPointSmall const& rhs) const { return !(*this == rhs); }

	bool operator<(IntegrationPointSmall const& rhs) const
	{
		return code.code() < rhs.code.code();
	}

	bool operator<=(IntegrationPointSmall const& rhs) const
	{
		return code.code() <= rhs.code.code();
	}

	bool operator>(IntegrationPointSmall const& rhs) const
	{
		return code.code() > rhs.code.code();
	}

	bool operator>=(IntegrationPointSmall const& rhs) const
	{
		return code.code() >= rhs.code.code();
	}
};

/*!
 * @brief
 *
 */
template <class P>
struct IntegrationPoint {
	Code code;
	std::vector<ValidPoint<P>> points;

	IntegrationPoint() = default;

	IntegrationPoint(Code const code) : code(code) {}

	IntegrationPoint(Code const code, P const& point, bool const valid = true)
	    : code(code), points(1, ValidPoint<P>(point, valid))
	{
	}

	IntegrationPoint(Code code, std::initializer_list<ValidPoint<P>> init)
	    : code(code), points(init)
	{
	}

	[[nodiscard]] bool valid() const
	{
		return std::any_of(std::cbegin(points), std::cend(points),
		                   [](auto const& p) { return p.valid; });
	}

	bool operator==(IntegrationPoint const& rhs) const
	{
		return code.code() == rhs.code.code();
	}

	bool operator!=(IntegrationPoint const& rhs) const { return !(*this == rhs); }

	bool operator<(IntegrationPoint const& rhs) const
	{
		return code.code() < rhs.code.code();
	}

	bool operator<=(IntegrationPoint const& rhs) const
	{
		return code.code() <= rhs.code.code();
	}

	bool operator>(IntegrationPoint const& rhs) const
	{
		return code.code() > rhs.code.code();
	}

	bool operator>=(IntegrationPoint const& rhs) const
	{
		return code.code() >= rhs.code.code();
	}
};
}  // namespace ufo::map

#endif  // UFO_MAP_INTEGRATION_POINT_H