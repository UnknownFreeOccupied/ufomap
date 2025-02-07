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

#ifndef UFO_MAP_PREDICATE_SURFEL_HPP
#define UFO_MAP_PREDICATE_SURFEL_HPP

// UFO
#include <ufo/map/node.hpp>
#include <ufo/map/predicate/predicate.hpp>

namespace ufo::pred
{
//
// Predicates
//

struct SurfelMap {
};

template <bool Negated = false>
struct HasSurfel {
};

template <bool Negated>
constexpr HasSurfel<!Negated> operator!(HasSurfel<Negated> const& p)
{
	return HasSurfel<!Negated>();
}

template <PredicateCompare PC = PredicateCompare::EQUAL>
struct NumSurfelPoints {
	constexpr NumSurfelPoints(std::size_t num_surfel_points)
	    : num_surfel_points(num_surfel_points)
	{
	}

	std::size_t num_surfel_points;
};

using NumSurfelPointsE  = NumSurfelPoints<>;
using NumSurfelPointsLE = NumSurfelPoints<PredicateCompare::LESS_EQUAL>;
using NumSurfelPointsGE = NumSurfelPoints<PredicateCompare::GREATER_EQUAL>;
using NumSurfelPointsL  = NumSurfelPoints<PredicateCompare::LESS>;
using NumSurfelPointsG  = NumSurfelPoints<PredicateCompare::GREATER>;

using NumSurfelPointsMin = NumSurfelPointsGE;
using NumSurfelPointsMax = NumSurfelPointsLE;

struct NumSurfelPointsInterval {
	constexpr NumSurfelPointsInterval(std::size_t min, std::size_t max) : min(min), max(max)
	{
	}

	NumSurfelPointsMin min;
	NumSurfelPointsMax max;
};

template <PredicateCompare PC = PredicateCompare::EQUAL>
struct SurfelPlanarity {
	constexpr SurfelPlanarity(double planarity) : planarity(planarity) {}

	double planarity;
};

using SurfelPlanarityE  = SurfelPlanarity<>;
using SurfelPlanarityLE = SurfelPlanarity<PredicateCompare::LESS_EQUAL>;
using SurfelPlanarityGE = SurfelPlanarity<PredicateCompare::GREATER_EQUAL>;
using SurfelPlanarityL  = SurfelPlanarity<PredicateCompare::LESS>;
using SurfelPlanarityG  = SurfelPlanarity<PredicateCompare::GREATER>;

using SurfelPlanarityMin = SurfelPlanarityGE;
using SurfelPlanarityMax = SurfelPlanarityLE;

struct SurfelPlanarityInterval {
	constexpr SurfelPlanarityInterval(double min, double max) : min(min), max(max) {}

	SurfelPlanarityMin min;
	SurfelPlanarityMax max;
};

//
// Predicate value/return check
//

template <>
struct ValueCheck<SurfelMap> {
	using Pred = SurfelMap;

	template <class Map, class Node>
	static constexpr bool apply(Pred, Map const&, Node)
	{
		return IsSurfelMap<Map>;
	}
};

template <class PredPost>
struct ValueCheck<Then<SurfelMap, PredPost>> {
	using Pred = Then<SurfelMap, PredPost>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (ValueCheck<SurfelMap>::apply(p.pre, m, n)) {
			return ValueCheck<PredPost>::apply(p.post, m, n);
		} else {
			return true;
		}
	}
};

template <bool Negated>
struct ValueCheck<HasSurfel<Negated>> {
	using Pred = HasSurfel<Negated>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const&, Map const& m, Node const& n)
	{
		if constexpr (Negated) {
			return !m.hasSurfel(n.index());
		} else {
			return m.hasSurfel(n.index());
		}
	}
};

template <PredicateCompare PC>
struct ValueCheck<NumSurfelPoints<PC>> {
	using Pred = NumSurfelPoints<PC>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (PredicateCompare::EQUAL == PC) {
			return m.getNumSurfelPoints(n.index()) == p.num_surfel_points;
		} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
			return m.getNumSurfelPoints(n.index()) <= p.num_surfel_points;
		} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
			return m.getNumSurfelPoints(n.index()) >= p.num_surfel_points;
		} else if constexpr (PredicateCompare::LESS == PC) {
			return m.getNumSurfelPoints(n.index()) < p.num_surfel_points;
		} else if constexpr (PredicateCompare::GREATER == PC) {
			return m.getNumSurfelPoints(n.index()) > p.num_surfel_points;
		} else {
			static_assert("Non-supported predicate comparison.");
		}
	}
};

template <>
struct ValueCheck<NumSurfelPointsInterval> {
	using Pred = NumSurfelPointsInterval;

	template <class Map, class Node>
	static inline bool apply(Pred const& p, Map const& m, Node const& n)
	{
		return ValueCheck<std::decay_t<decltype(p.min)>>::apply(p.min, m, n) &&
		       ValueCheck<std::decay_t<decltype(p.max)>>::apply(p.max, m, n);
	}
};

template <PredicateCompare PC>
struct ValueCheck<SurfelPlanarity<PC>> {
	using Pred = SurfelPlanarity<PC>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (PredicateCompare::EQUAL == PC) {
			return m.getSurfel(n.index()).getPlanarity() == p.planarity;
		} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
			return m.getSurfel(n.index()).getPlanarity() <= p.planarity;
		} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
			return m.getSurfel(n.index()).getPlanarity() >= p.planarity;
		} else if constexpr (PredicateCompare::LESS == PC) {
			return m.getSurfel(n.index()).getPlanarity() < p.planarity;
		} else if constexpr (PredicateCompare::GREATER == PC) {
			return m.getSurfel(n.index()).getPlanarity() > p.planarity;
		} else {
			static_assert("Non-supported predicate comparison.");
		}
	}
};

template <>
struct ValueCheck<SurfelPlanarityInterval> {
	using Pred = SurfelPlanarityInterval;

	template <class Map, class Node>
	static inline bool apply(Pred const& p, Map const& m, Node const& n)
	{
		return ValueCheck<std::decay_t<decltype(p.min)>>::apply(p.min, m, n) &&
		       ValueCheck<std::decay_t<decltype(p.max)>>::apply(p.max, m, n);
	}
};

//
// Predicate inner check
//

template <>
struct InnerCheck<SurfelMap> {
	using Pred = SurfelMap;

	template <class Map, class Node>
	static constexpr bool apply(Pred, Map const&, Node)
	{
		return IsSurfelMap<Map>;
	}
};

template <class PredPost>
struct InnerCheck<Then<SurfelMap, PredPost>> {
	using Pred = Then<SurfelMap, PredPost>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (InnerCheck<SurfelMap>::apply(p.pre, m, n)) {
			return InnerCheck<PredPost>::apply(p.post, m, n);
		} else {
			return true;
		}
	}
};

template <bool Negated>
struct InnerCheck<HasSurfel<Negated>> {
	using Pred = HasSurfel<Negated>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const&, Map const& m, Node n)
	{
		if constexpr (Negated) {
			return true;
		} else {
			return m.hasSurfel(n.index());
		}
	}
};

template <PredicateCompare PC>
struct InnerCheck<NumSurfelPoints<PC>> {
	using Pred = NumSurfelPoints<PC>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		// FIXME: Check how time step is propagated to determine

		if constexpr (PredicateCompare::EQUAL == PC) {
			return m.getNumSurfelPoints(n.index()) >= p.num_surfel_points;
		} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
			return true;
		} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
			return m.getNumSurfelPoints(n.index()) >= p.num_surfel_points;
		} else if constexpr (PredicateCompare::LESS == PC) {
			return true;
		} else if constexpr (PredicateCompare::GREATER == PC) {
			return m.getNumSurfelPoints(n.index()) > p.num_surfel_points;
		} else {
			static_assert("Non-supported predicate comparison.");
		}
	}
};

template <>
struct InnerCheck<NumSurfelPointsInterval> {
	using Pred = NumSurfelPointsInterval;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		return InnerCheck<std::decay_t<decltype(p.min)>>::apply(p.min, m, n) &&
		       InnerCheck<std::decay_t<decltype(p.max)>>::apply(p.max, m, n);
	}
};

template <PredicateCompare PC>
struct InnerCheck<SurfelPlanarity<PC>> {
	using Pred = SurfelPlanarity<PC>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		return true;
	}
};

template <>
struct InnerCheck<SurfelPlanarityInterval> {
	using Pred = SurfelPlanarityInterval;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		return InnerCheck<std::decay_t<decltype(p.min)>>::apply(p.min, m, n) &&
		       InnerCheck<std::decay_t<decltype(p.max)>>::apply(p.max, m, n);
	}
};

}  // namespace ufo::pred

#endif  // UFO_MAP_PREDICATE_SURFEL_HPP