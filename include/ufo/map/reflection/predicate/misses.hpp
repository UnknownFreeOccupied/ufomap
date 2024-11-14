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

#ifndef UFO_MAP_MISSES_PREDICATE_MISSES_HPP
#define UFO_MAP_MISSES_PREDICATE_MISSES_HPP

// UFO
#include <ufo/map/predicate/predicate.hpp>
#include <ufo/map/predicate/predicate_compare.hpp>
#include <ufo/map/types.hpp>

namespace ufo::pred
{
template <PredicateCompare PC = PredicateCompare::EQUAL>
struct Misses {
	count_t misses;

	Misses(count_t misses) : misses(misses) {}
};

using MissesE  = Misses<>;
using MissesNE = Misses<PredicateCompare::NOT_EQUAL>;
using MissesLE = Misses<PredicateCompare::LESS_EQUAL>;
using MissesGE = Misses<PredicateCompare::GREATER_EQUAL>;
using MissesL  = Misses<PredicateCompare::LESS>;
using MissesG  = Misses<PredicateCompare::GREATER>;

using MissesMin = MissesGE;
using MissesMax = MissesLE;

template <PredicateCompare PC>
struct InnerCheck<Misses<PC>> {
	using Pred = Misses<PC>;

	template <class Map, class Node>
	static inline bool apply(Pred p, Map const& m, Node const& n)
	{
		switch (m.missesPropagationCriteria()) {
			case PropagationCriteria::MIN:
				if constexpr (PredicateCompare::EQUAL == PC) {
					return m.misses(n.index()) <= p.misses;
				} else if constexpr (PredicateCompare::NOT_EQUAL == PC) {
					return true;
				} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
					return m.misses(n.index()) <= p.misses;
				} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
					return true;
				} else if constexpr (PredicateCompare::LESS == PC) {
					return m.misses(n.index()) < p.misses;
				} else if constexpr (PredicateCompare::GREATER == PC) {
					return true;
				}
			case PropagationCriteria::MAX:
				if constexpr (PredicateCompare::EQUAL == PC) {
					return m.misses(n.index()) >= p.misses;
				} else if constexpr (PredicateCompare::NOT_EQUAL == PC) {
					return true;
				} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
					return true;
				} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
					return m.misses(n.index()) >= p.misses;
				} else if constexpr (PredicateCompare::LESS == PC) {
					return true;
				} else if constexpr (PredicateCompare::GREATER == PC) {
					return m.misses(n.index()) > p.misses;
				}
			case PropagationCriteria::MEAN: return true;
			case PropagationCriteria::NONE: return true;
		}
	}
};

template <PredicateCompare PC>
struct ValueCheck<Misses<PC>> {
	using Pred = Misses<PC>;

	template <class Map, class Node>
	static inline bool apply(Pred const& p, Map const& m, Node n)
	{
		if constexpr (PredicateCompare::EQUAL == PC) {
			return m.misses(n.index()) == p.misses;
		} else if constexpr (PredicateCompare::NOT_EQUAL == PC) {
			return m.misses(n.index()) != p.misses;
		} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
			return m.misses(n.index()) <= p.misses;
		} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
			return m.misses(n.index()) >= p.misses;
		} else if constexpr (PredicateCompare::LESS == PC) {
			return m.misses(n.index()) < p.misses;
		} else if constexpr (PredicateCompare::GREATER == PC) {
			return m.misses(n.index()) > p.misses;
		}
	}
};
}  // namespace ufo::pred

#endif  // UFO_MAP_MISSES_PREDICATE_MISSES_HPP