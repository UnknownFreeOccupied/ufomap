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
#include <ufo/algorithm/algorithm.h>
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
template <std::size_t N = 8>
struct OccupancyNode {
	// Data
	std::array<occupancy_t, N> occupancy;

	//
	// Size
	//

	[[nodiscard]] static constexpr std::size_t occupancySize() { return N; }

	//
	// Fill
	//

	void fill(OccupancyNode const parent, index_t const index)
	{
		if constexpr (1 == N) {
			occupancy = other.occupancy;
		} else {
			occupancy.fill(other.occupancy[index]);
		}
	}

	//
	// Is collapsible
	//

	[[nodiscard]] bool isCollapsible(OccupancyNode const parent, index_t const index) const
	{
		if constexpr (1 == N) {
			return occupancy == parent.occupancy;
		} else {
			return all_of(occupancy,
			              [p = parent.occupancy[index]](auto const x) { return x == p; });
		}
	}

	//
	// Get occupancy
	//

	[[nodiscard]] constexpr occupancy_t occupancyIndex(index_t const index) const
	{
		if constexpr (1 == N) {
			return occupancy[0];
		} else {
			return occupancy[index];
		}
	}

	//
	// Set occupancy
	//

	void setOccupancy(occupancy_t const value) { occupancy.fill(value); }

	void setOccupancyIndex(index_t const index, occupancy_t const value)
	{
		if constexpr (1 == N) {
			setOccupancy(value);
		} else {
			occupancy[index] = value;
		}
	}
};

template <std::size_t N>
struct ContainsOccupancy {
	// Indicators
	index_field_t contains_unknown;
	index_field_t contains_free;
	index_field_t contains_occupied;

	//
	// Fill
	//

	void fill(ContainsOccupancy const parent, index_t const index)
	{
		contains_unknown = (parent.contains_unknown >> index) & index_field_t(1)
		                       ? std::numeric_limits<index_field_t>::max()
		                       : 0;
		contains_free = (parent.contains_free >> index) & index_field_t(1)
		                    ? std::numeric_limits<index_field_t>::max()
		                    : 0;
		contains_occupied = (parent.contains_occupied >> index) & index_field_t(1)
		                        ? std::numeric_limits<index_field_t>::max()
		                        : 0;
	}

	//
	// Is collapsible
	//

	[[nodiscard]] bool isCollapsible(ContainsOccupancy const parent,
	                                 index_t const index) const
	{
		return true;  // Does not matter what this is...
	}

	//
	// Contains unknown
	//

	constexpr index_field_t containsUnknown() const { return contains_unknown; }

	constexpr index_field_t containsUnknown(index_field_t const indices) const
	{
		return contains_unknown & indices;
	}

	constexpr bool containsUnknownIndex(index_t const index) const
	{
		return (contains_unknown >> index) & index_field_t(1);
	}

	//
	// Contains free
	//

	constexpr index_field_t containsFree() const { return contains_free; }

	constexpr index_field_t containsFree(index_field_t const indices) const
	{
		return contains_free & indices;
	}

	constexpr bool containsFreeIndex(index_t const index) const
	{
		return (contains_free >> index) & index_field_t(1);
	}

	//
	// Contains occupied
	//

	constexpr index_field_t containsOccupied() const { return contains_occupied; }

	constexpr index_field_t containsOccupied(index_field_t const indices) const
	{
		return contains_occupied & indices;
	}

	constexpr bool containsOccupiedIndex(index_t const index) const
	{
		return (contains_occupied >> index) & index_field_t(1);
	}

	//
	// Set contains unknown
	//

	constexpr void setContainsUnknown() noexcept
	{
		contains_unknown = std::numeric_limits<index_field_t>::max();
	}

	constexpr void setContainsUnknown(index_field_t const indices) noexcept
	{
		contains_unknown |= indices;
	}

	constexpr void setContainsUnknownIndex(index_t const index) noexcept
	{
		contains_unknown |= index_field_t(1) << index;
	}

	//
	// Reset contains unknown
	//

	constexpr void resetContainsUnknown() noexcept { contains_unknown = 0; }

	constexpr void resetContainsUnknown(index_field_t const indices) noexcept
	{
		contains_unknown &= ~indices;
	}

	constexpr void resetContainsUnknownIndex(index_t const index) noexcept
	{
		contains_unknown &= ~(index_field_t(1) << index);
	}

	//
	// Set contains free
	//

	constexpr void setContainsFree() noexcept
	{
		contains_free = std::numeric_limits<index_field_t>::max();
	}

	constexpr void setContainsFree(index_field_t const indices) noexcept
	{
		contains_free |= indices;
	}

	constexpr void setContainsFreeIndex(index_t const index) noexcept
	{
		contains_free |= index_field_t(1) << index;
	}

	//
	// Reset contains free
	//

	constexpr void resetContainsFree() noexcept { contains_unknown = 0; }

	constexpr void resetContainsFree(index_field_t const indices) noexcept
	{
		contains_free &= ~indices;
	}

	constexpr void resetContainsFreeIndex(index_t const index) noexcept
	{
		contains_free &= ~(index_field_t(1) << index);
	}

	//
	// Set contains occupied
	//

	constexpr void setContainsOccupied() noexcept
	{
		contains_occupied = std::numeric_limits<index_field_t>::max();
	}

	constexpr void setContainsOccupied(index_field_t const indices) noexcept
	{
		contains_occupied |= indices;
	}

	constexpr void setContainsOccupiedIndex(index_t const index) noexcept
	{
		contains_occupied |= index_field_t(1) << index;
	}

	//
	// Reset contains occupied
	//

	constexpr void resetContainsOccupied() noexcept { contains_occupied = 0; }

	constexpr void resetContainsOccupied(index_field_t const indices) noexcept
	{
		contains_occupied &= ~indices;
	}

	constexpr void resetContainsOccupiedIndex(index_t const index) noexcept
	{
		contains_occupied &= ~(index_field_t(1) << index);
	}
};

template <>
struct ContainsOccupancy<1> {
	// Indicators
	index_field_t contains_unknown : 1;
	index_field_t contains_free : 1;
	index_field_t contains_occupied : 1;

	//
	// Fill
	//

	void fill(ContainsOccupancy const parent, index_t const)
	{
		contains_unknown = parent.contains_unknown ? 1 : 0;
		contains_free = parent.contains_free ? 1 : 0;
		contains_occupied = parent.contains_occupied ? 1 : 0;
	}

	//
	// Is collapsible
	//

	[[nodiscard]] bool isCollapsible(ContainsOccupancy const, index_t const) const
	{
		return true;  // Does not matter what this is...
	}

	//
	// Contains unknown
	//

	constexpr bool containsUnknown() const { return contains_unknown; }

	constexpr bool containsUnknown(index_field_t const) const { return contains_unknown; }

	constexpr bool containsUnknownIndex(index_t const) const { return contains_unknown; }

	//
	// Contains free
	//

	constexpr bool containsFree() const { return contains_free; }

	constexpr bool containsFree(index_field_t const) const { return contains_free; }

	constexpr bool containsFreeIndex(index_t const) const { return contains_free; }

	//
	// Contains occupied
	//

	constexpr bool containsOccupied() const { return contains_occupied; }

	constexpr bool containsOccupied(index_field_t const) const { return contains_occupied; }

	constexpr bool containsOccupiedIndex(index_t const) const { return contains_occupied; }

	//
	// Set contains unknown
	//

	constexpr void setContainsUnknown() noexcept { contains_unknown = 1; }

	constexpr void setContainsUnknown(index_field_t const) noexcept
	{
		contains_unknown = 1;
	}

	constexpr void setContainsUnknownIndex(index_t const) noexcept { contains_unknown = 1; }

	//
	// Reset contains unknown
	//

	constexpr void resetContainsUnknown() noexcept { contains_unknown = 0; }

	constexpr void resetContainsUnknown(index_field_t const) noexcept
	{
		contains_unknown = 0;
	}

	constexpr void resetContainsUnknownIndex(index_t const) noexcept
	{
		contains_unknown = 0;
	}

	//
	// Set contains free
	//

	constexpr void setContainsFree() noexcept { contains_free = 1; }

	constexpr void setContainsFree(index_field_t const) noexcept { contains_free = 1; }

	constexpr void setContainsFreeIndex(index_t const) noexcept { contains_free = 1; }

	//
	// Reset contains free
	//

	constexpr void resetContainsFree() noexcept { contains_unknown = 0; }

	constexpr void resetContainsFree(index_field_t const) noexcept { contains_free = 0; }

	constexpr void resetContainsFreeIndex(index_t const) noexcept { contains_free = 0; }

	//
	// Set contains occupied
	//

	constexpr void setContainsOccupied() noexcept { contains_occupied = 1; }

	constexpr void setContainsOccupied(index_field_t const) noexcept
	{
		contains_occupied = 1;
	}

	constexpr void setContainsOccupiedIndex(index_t const) noexcept
	{
		contains_occupied = 1;
	}

	//
	// Reset contains occupied
	//

	constexpr void resetContainsOccupied() noexcept { contains_occupied = 0; }

	constexpr void resetContainsOccupied(index_field_t const) noexcept
	{
		contains_occupied = 0;
	}

	constexpr void resetContainsOccupiedIndex(index_t const) noexcept
	{
		contains_occupied = 0;
	}
};
}  // namespace ufo::map

#endif  // UFO_MAP_OCCUPANCY_NODE_H