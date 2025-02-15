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
#include <ufo/math/math.hpp>
#include <ufo/utility/create_array.hpp>

// STL
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>

namespace ufo
{
struct OccupancyElement {
	using logit_t = std::int32_t;

	// 3 bits for unknown, free, occupied, 1 bit for sign, 1 bit because we want half, -1
	// because positive and negative should be the same
	static constexpr logit_t const MAX_VALUE = ipow(logit_t(2), 32 - 5) - 1;

	std::uint32_t value_and_indicators{};

	OccupancyElement() noexcept                        = default;
	OccupancyElement(OccupancyElement const&) noexcept = default;

	OccupancyElement(logit_t logit, bool unknown, bool free, bool occupied) noexcept
	    : value_and_indicators((static_cast<std::uint32_t>(logit + MAX_VALUE) << 3) |
	                           (static_cast<std::uint32_t>(unknown) << 2) |
	                           (static_cast<std::uint32_t>(free) << 1) |
	                           (static_cast<std::uint32_t>(occupied) << 0))
	{
	}

	OccupancyElement(std::uint32_t value, std::uint32_t indicators) noexcept
	    : value_and_indicators(value << 3 | indicators)
	{
	}

	OccupancyElement& operator=(OccupancyElement const&) noexcept = default;

	[[nodiscard]] logit_t logit() const noexcept
	{
		return static_cast<logit_t>(value_and_indicators >> 3) - MAX_VALUE;
	}

	void logit(logit_t v) noexcept
	{
		value_and_indicators =
		    (static_cast<std::uint32_t>(v + MAX_VALUE) << 3) | indicators();
	}

	[[nodiscard]] bool unknown() const noexcept { return (value_and_indicators >> 2) & 1u; }

	void unknown(bool v) noexcept
	{
		value_and_indicators =
		    (value_and_indicators & ~(std::uint32_t(1) << 2)) | (std::uint32_t(v) << 2);
	}

	[[nodiscard]] bool free() const noexcept { return (value_and_indicators >> 1) & 1u; }

	void free(bool v) noexcept
	{
		value_and_indicators =
		    (value_and_indicators & ~(std::uint32_t(1) << 1)) | (std::uint32_t(v) << 1);
	}

	[[nodiscard]] bool occupied() const noexcept
	{
		return (value_and_indicators >> 0) & 1u;
	}

	void occupied(bool v) noexcept
	{
		value_and_indicators =
		    (value_and_indicators & ~(std::uint32_t(1) << 0)) | (std::uint32_t(v) << 0);
	}

	[[nodiscard]] std::uint32_t value() const noexcept { return value_and_indicators >> 3; }

	[[nodiscard]] std::uint32_t indicators() const noexcept
	{
		return value_and_indicators & 0b111;
	}
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