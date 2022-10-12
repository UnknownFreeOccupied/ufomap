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
#include <limits>
#include <type_traits>

namespace ufo::map
{
template <std::size_t N = 8>
struct OccupancyNode {
	//
	// Size
	//

	[[nodiscard]] static constexpr std::size_t occupancySize() { return N; }

	//
	// Data
	//

	[[nodiscard]] constexpr logit_t const* occupancyData() const noexcept
	{
		return occupancy_.data();
	}

	[[nodiscard]] constexpr logit_t* occupancyData() noexcept { return occupancy_.data(); }

	//
	// Iterators
	//

	[[nodiscard]] constexpr auto beginOccupancy() noexcept { return occupancy_.begin(); }

	[[nodiscard]] constexpr auto beginOccupancy() const noexcept
	{
		return occupancy_.begin();
	}

	[[nodiscard]] constexpr auto cbeginOccupancy() const noexcept
	{
		return occupancy_.cbegin();
	}

	[[nodiscard]] constexpr auto endOccupancy() noexcept { return occupancy_.end(); }

	[[nodiscard]] constexpr auto endOccupancy() const noexcept { return occupancy_.end(); }

	[[nodiscard]] constexpr auto cendOccupancy() const noexcept
	{
		return occupancy_.cend();
	}

	[[nodiscard]] constexpr auto rbeginOccupancy() noexcept { return occupancy_.rbegin(); }

	[[nodiscard]] constexpr auto rbeginOccupancy() const noexcept
	{
		return occupancy_.rbegin();
	}

	[[nodiscard]] constexpr auto crbeginOccupancy() const noexcept
	{
		return occupancy_.crbegin();
	}

	[[nodiscard]] constexpr auto rendOccupancy() noexcept { return occupancy_.rend(); }

	[[nodiscard]] constexpr auto rendOccupancy() const noexcept
	{
		return occupancy_.rend();
	}

	[[nodiscard]] constexpr auto crendOccupancy() const noexcept
	{
		return occupancy_.crend();
	}

	//
	// Fill
	//

	void fill(OccupancyNode const parent, index_t const index)
	{
		setOccupancy(parent.occupancy(index));
	}

	//
	// Is collapsible
	//

	[[nodiscard]] bool isCollapsible(OccupancyNode const parent, index_t const index) const
	{
		return all_of(occupancy_,
		              [p = parent.occupancy(index)](auto const x) { return x == p; });
	}

	//
	// Get occupancy
	//

	[[nodiscard]] constexpr logit_t occupancy(index_t const index) const
	{
		if constexpr (1 == N) {
			return occupancy_[0];
		} else {
			return occupancy_[index];
		}
	}

	//
	// Set occupancy
	//

	void setOccupancy(logit_t const value) { occupancy_.fill(value); }

	void setOccupancy(index_t const index, logit_t const value)
	{
		if constexpr (1 == N) {
			setOccupancy(value);
		} else {
			occupancy_[index] = value;
		}
	}

	//
	// Increase occupancy
	//

	void increaseOccupancy(logit_t const value)
	{
		for (auto& occ : occupancy_) {
			occ = std::numeric_limits<logit_t>::max() - value > occ
			          ? occ + value
			          : std::numeric_limits<logit_t>::max();
		}
	}

	void increaseOccupancy(index_t const index, logit_t const value)
	{
		if constexpr (1 == N) {
			occupancy_[0] = std::numeric_limits<logit_t>::max() - value > occupancy_[0]
			                    ? occupancy_[0] + value
			                    : std::numeric_limits<logit_t>::max();
		} else {
			occupancy_[index] = std::numeric_limits<logit_t>::max() - value > occupancy_[index]
			                        ? occupancy_[index] + value
			                        : std::numeric_limits<logit_t>::max();
		}
	}

	//
	// Decrease occupancy
	//

	void decreaseOccupancy(logit_t const value)
	{
		for (auto& occ : occupancy_) {
			occ = std::numeric_limits<logit_t>::lowest() + value < occ
			          ? occ - value
			          : std::numeric_limits<logit_t>::lowest();
		}
	}

	void decreaseOccupancy(index_t const index, logit_t const value)
	{
		if constexpr (1 == N) {
			occupancy_[0] = std::numeric_limits<logit_t>::lowest() + value < occupancy_[0]
			                    ? occupancy_[0] - value
			                    : std::numeric_limits<logit_t>::lowest();
		} else {
			occupancy_[index] =
			    std::numeric_limits<logit_t>::lowest() + value < occupancy_[index]
			        ? occupancy_[index] - value
			        : std::numeric_limits<logit_t>::lowest();
		}
	}

 private:
	// Data
	std::array<logit_t, N> occupancy_;
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

	[[nodiscard]] bool isCollapsible(ContainsOccupancy const, std::size_t const) const
	{
		return true;  // Does not matter what this is...
	}

	//
	// Get contains unknown
	//

	[[nodiscard]] constexpr bool containsUnknown(index_t const index) const
	{
		return contains_unknown[index];
	}

	//
	// Set contains unknown
	//

	constexpr void setContainsUnknown(bool const value)
	{
		if (value) {
			contains_unknown.set();
		} else {
			contains_unknown.reset();
		}
	}

	constexpr void setContainsUnknown(index_t const index, bool const value)
	{
		contains_unknown.set(index, value);
	}

	//
	// Get contains free
	//

	[[nodiscard]] constexpr bool containsFree(index_t const index) const
	{
		return contains_free[index];
	}

	//
	// Set contains free
	//

	constexpr void setContainsFree(bool const value)
	{
		if (value) {
			contains_free.set();
		} else {
			contains_free.reset();
		}
	}

	constexpr void setContainsFree(index_t const index, bool const value)
	{
		contains_free.set(index, value);
	}

	//
	// Get contains occupied
	//

	[[nodiscard]] constexpr bool containsOccupied(index_t const index) const
	{
		return contains_occupied[index];
	}

	//
	// Set contains occupied
	//

	constexpr void setContainsOccupied(bool const value)
	{
		if (value) {
			contains_occupied.set();
		} else {
			contains_occupied.reset();
		}
	}

	constexpr void setContainsOccupied(index_t const index, bool const value)
	{
		contains_occupied.set(index, value);
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

	//
	// Get contains unknown
	//

	[[nodiscard]] constexpr bool containsUnknown(index_t const index) const
	{
		return contains_unknown;
	}

	//
	// Set contains unknown
	//

	constexpr void setContainsUnknown(bool const value) { contains_unknown = value; }

	constexpr void setContainsUnknown(index_t const index, bool const value)
	{
		setContainsUnknown(value);
	}

	//
	// Get contains free
	//

	[[nodiscard]] constexpr bool containsFree(index_t const index) const
	{
		return contains_free;
	}

	//
	// Set contains free
	//

	constexpr void setContainsFree(bool const value) { contains_free = value; }

	constexpr void setContainsFree(index_t const index, bool const value)
	{
		setContainsFree(value);
	}

	//
	// Get contains occupied
	//

	[[nodiscard]] constexpr bool containsOccupied(index_t const index) const
	{
		return contains_occupied;
	}

	//
	// Set contains occupied
	//

	constexpr void setContainsOccupied(bool const value) { contains_occupied = value; }

	constexpr void setContainsOccupied(index_t const index, bool const value)
	{
		setContainsOccupied(value);
	}
};
}  // namespace ufo::map

#endif  // UFO_MAP_OCCUPANCY_NODE_H