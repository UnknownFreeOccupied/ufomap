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

#ifndef UFO_MAP_OCCUPANCY_PREDICATE_OCCUPANCY_HPP
#define UFO_MAP_OCCUPANCY_PREDICATE_OCCUPANCY_HPP

// UFO
#include <ufo/container/tree/index.hpp>
#include <ufo/container/tree/predicate/filter.hpp>
#include <ufo/container/tree/predicate/predicate_compare.hpp>
#include <ufo/map/occupancy/propagation_criteria.hpp>

// STL
#include <cassert>

namespace ufo::pred
{
template <PredicateCompare PC = PredicateCompare::EQUAL>
struct Occupancy {
	using occupancy_t = float;
	using logit_t     = std::int8_t;

	Occupancy(occupancy_t occupancy) : occupancy(occupancy) {}

	occupancy_t occupancy;

 protected:
	logit_t logit;

	template <class T>
	friend class Filter;
};

using OccupancyE  = Occupancy<>;
using OccupancyNE = Occupancy<PredicateCompare::NOT_EQUAL>;
using OccupancyLE = Occupancy<PredicateCompare::LESS_EQUAL>;
using OccupancyGE = Occupancy<PredicateCompare::GREATER_EQUAL>;
using OccupancyL  = Occupancy<PredicateCompare::LESS>;
using OccupancyG  = Occupancy<PredicateCompare::GREATER>;

using OccupancyMin = OccupancyGE;
using OccupancyMax = OccupancyLE;

// TODO: Implement this
// pred::Filter::init(p, t);
// pred::Filter::returnable(p, t, n);
// pred::Filter::traversable(p, t, n);

// template <PredicateCompare PC>
// struct Filter<Occupancy<PC>> {
// 	using Pred = Occupancy<PC>;

// 	template <class Tree>
// 	static constexpr void init(Pred& p, Tree const& t)
// 	{
// 		p.logit = t.occupancyLogit(p.occupancy);
// 	}

// 	template <class Tree>
// 	[[nodiscard]] static constexpr bool returnable(Pred p, Tree const& t, TreeIndex n)
// 	{
// 		if constexpr (PredicateCompare::EQUAL == PC) {
// 			return t.occupancyLogit(n) == p.logit;
// 		} else if constexpr (PredicateCompare::NOT_EQUAL == PC) {
// 			return t.occupancyLogit(n) != p.logit;
// 		} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
// 			return t.occupancyLogit(n) <= p.logit;
// 		} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
// 			return t.occupancyLogit(n) >= p.logit;
// 		} else if constexpr (PredicateCompare::LESS == PC) {
// 			return t.occupancyLogit(n) < p.logit;
// 		} else if constexpr (PredicateCompare::GREATER == PC) {
// 			return t.occupancyLogit(n) > p.logit;
// 		}
// 	}

// 	template <class Tree>
// 	[[nodiscard]] static constexpr bool traversable(Pred p, Tree const& t, TreeIndex n)
// 	{
// 		switch (t.occupancyPropagationCriteria()) {
// 			case OccupancyPropagationCriteria::MIN:
// 				if constexpr (PredicateCompare::EQUAL == PC) {
// 					return t.occupancyLogit(n) <= p.logit;
// 				} else if constexpr (PredicateCompare::NOT_EQUAL == PC) {
// 					return true;
// 				} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
// 					return t.occupancyLogit(n) <= p.logit;
// 				} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
// 					return true;
// 				} else if constexpr (PredicateCompare::LESS == PC) {
// 					return t.occupancyLogit(n) < p.logit;
// 				} else if constexpr (PredicateCompare::GREATER == PC) {
// 					return true;
// 				}
// 			case OccupancyPropagationCriteria::MAX:
// 				if constexpr (PredicateCompare::EQUAL == PC) {
// 					return t.occupancyLogit(n) >= p.logit;
// 				} else if constexpr (PredicateCompare::NOT_EQUAL == PC) {
// 					return true;
// 				} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
// 					return true;
// 				} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
// 					return t.occupancyLogit(n) >= p.logit;
// 				} else if constexpr (PredicateCompare::LESS == PC) {
// 					return true;
// 				} else if constexpr (PredicateCompare::GREATER == PC) {
// 					return t.occupancyLogit(n) > p.logit;
// 				}
// 			case OccupancyPropagationCriteria::MEAN: return true;
// 			case OccupancyPropagationCriteria::ONLY_INDICATORS: return true;
// 			case OccupancyPropagationCriteria::NONE: return true;
// 		}
// 		assert(false);
// 		return true;
// 	}
// };

template <PredicateCompare PC>
struct Filter<Occupancy<PC>> : public FilterBase<Occupancy<PC>> {
	using Pred = Occupancy<PC>;

	template <class Tree>
	static constexpr void init(Pred& p, Tree const& t)
	{
		p.logit = t.occupancyLogit(p.occupancy);
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool returnable(Pred const& p, Tree const& t,
	                                               TreeIndex n)
	{
		if constexpr (PredicateCompare::EQUAL == PC) {
			return t.occupancyLogit(n) == p.logit;
		} else if constexpr (PredicateCompare::NOT_EQUAL == PC) {
			return t.occupancyLogit(n) != p.logit;
		} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
			return t.occupancyLogit(n) <= p.logit;
		} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
			return t.occupancyLogit(n) >= p.logit;
		} else if constexpr (PredicateCompare::LESS == PC) {
			return t.occupancyLogit(n) < p.logit;
		} else if constexpr (PredicateCompare::GREATER == PC) {
			return t.occupancyLogit(n) > p.logit;
		}
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool traversable(Pred const& p, Tree const& t,
	                                                TreeIndex n)
	{
		switch (t.occupancyPropagationCriteria()) {
			case OccupancyPropagationCriteria::MIN:
				if constexpr (PredicateCompare::EQUAL == PC) {
					return t.occupancyLogit(n) <= p.logit;
				} else if constexpr (PredicateCompare::NOT_EQUAL == PC) {
					return true;
				} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
					return t.occupancyLogit(n) <= p.logit;
				} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
					return true;
				} else if constexpr (PredicateCompare::LESS == PC) {
					return t.occupancyLogit(n) < p.logit;
				} else if constexpr (PredicateCompare::GREATER == PC) {
					return true;
				}
			case OccupancyPropagationCriteria::MAX:
				if constexpr (PredicateCompare::EQUAL == PC) {
					return t.occupancyLogit(n) >= p.logit;
				} else if constexpr (PredicateCompare::NOT_EQUAL == PC) {
					return true;
				} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
					return true;
				} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
					return t.occupancyLogit(n) >= p.logit;
				} else if constexpr (PredicateCompare::LESS == PC) {
					return true;
				} else if constexpr (PredicateCompare::GREATER == PC) {
					return t.occupancyLogit(n) > p.logit;
				}
			case OccupancyPropagationCriteria::MEAN: return true;
			case OccupancyPropagationCriteria::ONLY_INDICATORS: return true;
			case OccupancyPropagationCriteria::NONE: return true;
		}
		assert(false);
		return true;
	}
};
}  // namespace ufo::pred

#endif  // UFO_MAP_OCCUPANCY_PREDICATE_OCCUPANCY_HPP