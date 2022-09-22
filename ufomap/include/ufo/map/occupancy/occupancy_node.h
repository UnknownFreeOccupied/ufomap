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

	void fill(OccupancyNode const parent, std::size_t const pos)
	{
		if constexpr (1 == N) {
			occupancy = other.occupancy;
		} else {
			occupancy.fill(other.occupancy[pos]);
		}
	}

	//
	// Is collapsible
	//

	[[nodiscard]] bool isCollapsible(OccupancyNode const parent,
	                                 std::size_t const pos) const
	{
		if constexpr (1 == N) {
			return occupancy == parent.occupancy;
		} else {
			return all_of(occupancy,
			              [p = parent.occupancy[pos]](auto const x) { return x == p; });
		}
	}
};

template <std::size_t N>
struct ContainsOccupancy {
	// Indicators
	IndexField contains_unknown;
	IndexField contains_free;
	IndexField contains_occupied;

	//
	// Fill
	//

	void fill(ContainsOccupancy const parent, index_t const index)
	{
		if (parent.contains_unknown[index]) {
			contains_unknown.set();
		} else {
			contains_unknown.reset();
		}
		if (parent.contains_free[index]) {
			contains_free.set();
		} else {
			contains_free.reset();
		}
		if (parent.contains_occupied[index]) {
			contains_occupied.set();
		} else {
			contains_occupied.reset();
		}
	}

	//
	// Is collapsible
	//

	[[nodiscard]] bool isCollapsible(ContainsOccupancy const parent,
	                                 std::size_t const pos) const
	{
		return true;  // Does not matter what this is...
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

	void fill(ContainsOccupancy const parent, std::size_t const)
	{
		contains_unknown = parent.contains_unknown;
		contains_free = parent.contains_free;
		contains_occupied = parent.contains_occupied;
	}

	//
	// Is collapsible
	//

	[[nodiscard]] bool isCollapsible(ContainsOccupancy const, std::size_t const) const
	{
		return true;  // Does not matter what this is...
	}
};
}  // namespace ufo::map

#endif  // UFO_MAP_OCCUPANCY_NODE_H