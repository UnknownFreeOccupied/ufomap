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

#ifndef UFO_MAP_OCCUPANCY_PREDICATE_CONTAINS_OCCUPANCY_STATE_HPP
#define UFO_MAP_OCCUPANCY_PREDICATE_CONTAINS_OCCUPANCY_STATE_HPP

// UFO
#include <ufo/container/tree/predicate/filter.hpp>
#include <ufo/map/occupancy/propagation_criteria.hpp>
#include <ufo/map/occupancy/state.hpp>

namespace ufo::pred
{
template <ufo::OccupancyState State>
struct ContainOccupancyState {
};

using ContainUnknown  = ContainOccupancyState<ufo::OccupancyState::UNKNOWN>;
using ContainFree     = ContainOccupancyState<ufo::OccupancyState::FREE>;
using ContainOccupied = ContainOccupancyState<ufo::OccupancyState::OCCUPIED>;

template <ufo::OccupancyState State>
struct Filter<ContainOccupancyState<State>> {
	using Pred = ContainOccupancyState<State>;

	template <class Tree>
	static constexpr void init(Pred&, Tree const&)
	{
	}

	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool returnable(Pred const&, Tree const& t,
	                                               Node const& n)
	{
		if constexpr (ufo::OccupancyState::UNKNOWN == State) {
			return t.containsUnknown(n.index);
		} else if constexpr (ufo::OccupancyState::FREE == State) {
			return t.containsFree(n.index);
		} else if constexpr (ufo::OccupancyState::OCCUPIED == State) {
			return t.containsOccupied(n.index);
		}
	}

	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool traversable(Pred const& p, Tree const& t,
	                                                Node const& n)
	{
		if (OccupancyPropagationCriteria::NONE == t.occupancyPropagationCriteria()) {
			return true;
		}

		if constexpr (ufo::OccupancyState::UNKNOWN == State) {
			return t.containsUnknown(n.index);
		} else if constexpr (ufo::OccupancyState::FREE == State) {
			return t.containsFree(n.index);
		} else if constexpr (ufo::OccupancyState::OCCUPIED == State) {
			return t.containsOccupied(n.index);
		}
	}
};
}  // namespace ufo::pred

#endif  // UFO_MAP_OCCUPANCY_PREDICATE_CONTAINS_OCCUPANCY_STATE_HPP