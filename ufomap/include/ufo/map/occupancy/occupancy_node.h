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

#ifndef UFO_MAP_OCCUPANCY_NODE_H
#define UFO_MAP_OCCUPANCY_NODE_H

// UFO
#include <ufo/map/color/color_node.h>
#include <ufo/map/octree/octree_node.h>
#include <ufo/map/semantic/semantic_node.h>
#include <ufo/map/types.h>

// STL
#include <array>
#include <cstdint>
#include <iostream>
#include <type_traits>

namespace ufo::map
{
template <typename OccupancyType = float>
struct OccupancyNode {
	using occupancy_t = OccupancyType;

	// Data
	std::array<occupancy_t, 8> occupancy;

	// Indicators
	uint8_t contains_unknown;
	uint8_t contains_free;
	uint8_t contains_occupied;

	bool operator==(OccupancyNode const& rhs) const { return occupancy == rhs.occupancy; }

	bool operator!=(OccupancyNode const& rhs) const { return !(*this == rhs); }

	constexpr occupancy_t getOccupancy(std::size_t const index) const
	{
		return occupancy[index];
	}

	void setOccupancy(occupancy_t const value) { occupancy.fill(value); }

	constexpr void setOccupancy(std::size_t const index, occupancy_t const value)
	{
		occupancy[index] = value;
	}

	constexpr bool containsUnknown(std::size_t const index) const
	{
		return (contains_unknown >> index) & uint8_t(1);
	}

	constexpr bool containsFree(std::size_t const index) const
	{
		return (contains_free >> index) & uint8_t(1);
	}

	constexpr bool containsOccupied(std::size_t const index) const
	{
		return (contains_occupied >> index) & uint8_t(1);
	}

	constexpr void setContainsUnknown(bool const contains) const
	{
		contains_unknown = contains ? std::numeric_limits<uint8_t>::max() : uint8_t(0);
	}

	constexpr void setContainsUnknown(std::size_t const index, bool const contains) const
	{
		contains_unknown = (contains_unknown & ~(uint8_t(1) << index)) |
		                   (uint8_t(contains ? uint8_t(1) : uint8_t(0)) << index);
	}

	constexpr void setContainsFree(bool const contains) const
	{
		contains_free = contains ? std::numeric_limits<uint8_t>::max() : uint8_t(0);
	}

	constexpr void setContainsFree(std::size_t const index, bool const contains) const
	{
		contains_free = (contains_free & ~(uint8_t(1) << index)) |
		                (uint8_t(contains ? uint8_t(1) : uint8_t(0)) << index);
	}

	constexpr void setContainsOccupied(bool const contains) const
	{
		contains_occupied = contains ? std::numeric_limits<uint8_t>::max() : uint8_t(0);
	}

	constexpr void setContainsOccupied(std::size_t const index, bool const contains) const
	{
		contains_occupied = (contains_occupied & ~(uint8_t(1) << index)) |
		                    (uint8_t(contains ? uint8_t(1) : uint8_t(0)) << index);
	}
};
}  // namespace ufo::map

#endif  // UFO_MAP_OCCUPANCY_NODE_H