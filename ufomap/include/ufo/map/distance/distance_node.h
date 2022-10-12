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

#ifndef UFO_MAP_DISTANCE_NODE_H
#define UFO_MAP_DISTANCE_NODE_H

// UFO
#include <ufo/algorithm/algorithm.h>
#include <ufo/map/types.h>

// STL
#include <array>

namespace ufo::map
{
template <std::size_t N = 8>
struct DistanceNode {
	//
	// Size
	//

	[[nodiscard]] static constexpr std::size_t distanceSize() { return N; }

	//
	// Data
	//

	[[nodiscard]] constexpr distance_t const* distanceData() const noexcept
	{
		return distance_.data();
	}

	[[nodiscard]] constexpr distance_t* distanceData() noexcept { return distance_.data(); }

	//
	// Iterators
	//

	[[nodiscard]] constexpr auto beginDistance() noexcept { return distance_.begin(); }

	[[nodiscard]] constexpr auto beginDistance() const noexcept
	{
		return distance_.begin();
	}

	[[nodiscard]] constexpr auto cbeginDistance() const noexcept
	{
		return distance_.cbegin();
	}

	[[nodiscard]] constexpr auto endDistance() noexcept { return distance_.end(); }

	[[nodiscard]] constexpr auto endDistance() const noexcept { return distance_.end(); }

	[[nodiscard]] constexpr auto cendDistance() const noexcept { return distance_.cend(); }

	[[nodiscard]] constexpr auto rbeginDistance() noexcept { return distance_.rbegin(); }

	[[nodiscard]] constexpr auto rbeginDistance() const noexcept
	{
		return distance_.rbegin();
	}

	[[nodiscard]] constexpr auto crbeginDistance() const noexcept
	{
		return distance_.crbegin();
	}

	[[nodiscard]] constexpr auto rendDistance() noexcept { return distance_.rend(); }

	[[nodiscard]] constexpr auto rendDistance() const noexcept { return distance_.rend(); }

	[[nodiscard]] constexpr auto crendDistance() const noexcept
	{
		return distance_.crend();
	}

	//
	// Fill
	//

	void fill(DistanceNode const parent, index_t const index)
	{
		setDistance(parent.distance(index));
	}

	//
	// Is collapsible
	//

	[[nodiscard]] constexpr bool isCollapsible(DistanceNode const parent,
	                                           index_t const index) const
	{
		return all_of(distance_,
		              [a = parent.distance(index)](auto const b) { return a == b; });
	}

	//
	// Get distance
	//

	[[nodiscard]] constexpr distance_t distance(index_t const index) const
	{
		if constexpr (1 == N) {
			return distance_[0];
		} else {
			return distance_[index];
		}
	}

	//
	// Set distance
	//

	void setDistance(distance_t const value) { distance_.fill(value); }

	void setDistance(index_t const index, distance_t const value)
	{
		if constexpr (1 == N) {
			setDistance(value);
		} else {
			distance_[index] = value;
		}
	}

 private:
	// Data
	std::array<distance_t, N> distance_;
};
}  // namespace ufo::map

#endif  // UFO_MAP_DISTANCE_NODE_H