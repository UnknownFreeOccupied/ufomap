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
#ifndef UFO_MAP_OCCUPANCY_BLOCK_HPP
#define UFO_MAP_OCCUPANCY_BLOCK_HPP

// UFO
#include <ufo/utility/create_array.hpp>

// STL
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>

namespace ufo
{
struct OccupancyElement {
	using logit_t = std::int8_t;

	logit_t      logit{};
	// TODO: Use a single std::uint8_t and shift instead of using bit-field
	std::uint8_t unknown  : 1;
	std::uint8_t free     : 1;
	std::uint8_t occupied : 1;

	OccupancyElement() noexcept                        = default;
	OccupancyElement(OccupancyElement const&) noexcept = default;

	OccupancyElement(logit_t logit, bool unknown, bool free, bool occupied) noexcept
	    : logit(logit)
	    , unknown(static_cast<std::uint8_t>(unknown))
	    , free(static_cast<std::uint8_t>(free))
	    , occupied(static_cast<std::uint8_t>(occupied))
	{
	}

	OccupancyElement& operator=(OccupancyElement const&) noexcept = default;
};

template <std::size_t BF>
struct OccupancyBlock {
	using logit_t = OccupancyElement::logit_t;

	std::array<OccupancyElement, BF> data;

	constexpr OccupancyBlock() = default;

	constexpr OccupancyBlock(logit_t logit, bool unknown, bool free, bool occupied)
	    : data(createArray<BF>(OccupancyElement(logit, unknown, free, occupied)))
	{
	}

	constexpr OccupancyBlock(OccupancyElement const& parent) : data(createArray<BF>(parent))
	{
	}

	constexpr void fill(OccupancyElement const& parent) { data.fill(parent); }

	[[nodiscard]] constexpr OccupancyElement& operator[](std::size_t pos)
	{
		assert(BF > pos);
		return data[pos];
	}

	[[nodiscard]] constexpr OccupancyElement const& operator[](std::size_t pos) const
	{
		assert(BF > pos);
		return data[pos];
	}
};
}  // namespace ufo
#endif  // UFO_MAP_OCCUPANCY_BLOCK_HPP