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

#ifndef UFO_MAP_OCCUPANCY_PREDICATE_OCCUPANCY_STATES_HPP
#define UFO_MAP_OCCUPANCY_PREDICATE_OCCUPANCY_STATES_HPP

// UFO
#include <ufo/container/tree/index.hpp>
#include <ufo/container/tree/predicate/filter.hpp>
#include <ufo/map/occupancy/propagation_criteria.hpp>

namespace ufo::pred
{
struct OccupancyStates {
	constexpr OccupancyStates(bool unknown, bool free, bool occupied)
	    : unknown(unknown), free(free), occupied(occupied)
	{
	}

	bool unknown;
	bool free;
	bool occupied;
};

template <>
struct Filter<OccupancyStates> : public FilterBase<OccupancyStates> {
	using Pred = OccupancyStates;

	template <class Tree>
	static constexpr void init(Pred&, Tree const&)
	{
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool returnable(Pred const& p, Tree const& t,
	                                               TreeIndex const& n)
	{
		return (p.unknown && t.isUnknown(n)) || (p.free && t.isFree(n)) ||
		       (p.occupied && t.isOccupied(n));
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool traversable(Pred const& p, Tree const& t,
	                                                TreeIndex const& n)
	{
		if (OccupancyPropagationCriteria::NONE == t.occupancyPropagationCriteria()) {
			return true;
		}

		return (p.unknown && t.containsUnknown(n)) || (p.free && t.containsFree(n)) ||
		       (p.occupied && t.containsOccupied(n));
	}
};
}  // namespace ufo::pred

#endif  // UFO_MAP_OCCUPANCY_PREDICATE_OCCUPANCY_STATES_HPP