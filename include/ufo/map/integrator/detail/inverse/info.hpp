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

#ifndef UFO_MAP_INTEGRATOR_DETAIL_INVERSE_INFO_HPP
#define UFO_MAP_INTEGRATOR_DETAIL_INVERSE_INFO_HPP

// UFO
#include <ufo/math/vec.hpp>

// STL
#include <cstddef>
#include <cstdint>
#include <limits>

namespace ufo::detail
{
template <std::size_t Dim>
struct InverseInfoTop {
	Vec<Dim, float> point{};
	float           distance{};
	std::uint32_t   first_lut{};
	std::uint32_t   last_lut{};
	std::uint32_t   first_child{};
	std::uint32_t   last_child{};
	bool            seen{};

	constexpr InverseInfoTop() = default;

	constexpr InverseInfoTop(Vec<Dim, float> const& point, float distance,
	                         std::uint32_t first_lut, std::uint32_t last_lut,
	                         std::uint32_t first_child, std::uint32_t last_child)
	    : point(point)
	    , distance(distance)
	    , first_lut(first_lut)
	    , last_lut(last_lut)
	    , first_child(first_child)
	    , last_child(last_child)
	{
	}
};

template <std::size_t Dim>
struct InverseInfoMiddle {
	Vec<Dim, float> point{};
	float           distance{};
	std::uint32_t   first_lut{};
	std::uint32_t   data = std::numeric_limits<std::uint32_t>::max() << 1;

	constexpr InverseInfoMiddle() = default;

	constexpr InverseInfoMiddle(Vec<Dim, float> const& point, float distance,
	                            std::uint32_t first_lut, std::uint32_t first_child)
	    : point(point), distance(distance), first_lut(first_lut), data(first_child << 1)
	{
	}

	[[nodiscard]] constexpr bool seen() const { return 0b1u == (0b1u & data); }

	constexpr void seen(bool v) { data = (~0b1u & data) | static_cast<std::uint32_t>(v); }

	[[nodiscard]] constexpr std::uint32_t firstChild() const { return data >> 1; }

	constexpr void firstChild(std::uint32_t first_child)
	{
		data = (0b1u & data) | (first_child << 1);
	}
};

template <std::size_t Dim>
struct InverseInfoBottom {
	Vec<Dim, float> point{};
	float           distance{};
	std::uint32_t   first_lut{};
	std::uint32_t   data{};

	constexpr InverseInfoBottom() = default;

	constexpr InverseInfoBottom(Vec<Dim, float> const& point, float distance,
	                            std::uint32_t first_lut, std::uint32_t first_lut_void)
	    : point(point), distance(distance), first_lut(first_lut), data(first_lut_void << 1)
	{
	}

	[[nodiscard]] constexpr bool seen() const { return 0b1u == (0b1u & data); }

	constexpr void seen(bool v) { data = (~0b1u & data) | static_cast<std::uint32_t>(v); }

	[[nodiscard]] constexpr std::uint32_t firstLutVoidRegion() const { return data >> 1; }

	constexpr void firstLutVoidRegion(std::uint32_t first_lut_void_region)
	{
		data = (0b1u & data) | (first_lut_void_region << 1);
	}
};
}  // namespace ufo::detail

#endif  // UFO_MAP_INTEGRATOR_DETAIL_INVERSE_INFO_HPP