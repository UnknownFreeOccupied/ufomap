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

#ifndef UFO_MAP_INTENSITY_PREDICATE_H
#define UFO_MAP_INTENSITY_PREDICATE_H

// UFO
#include <ufo/map/predicate/predicates.h>
#include <ufo/map/types.h>

namespace ufo::map::predicate
{
//
// Predicates
//

// REVIEW: Name?
struct IntensityMap {
};

template <PredicateCompare PC = PredicateCompare::EQUAL>
struct Intensity {
	constexpr Intensity(intensity_t intensity) : intensity(intensity) {}

	intensity_t intensity;
};

using IntensityE = Intensity<>;
using IntensityLE = Intensity<PredicateCompare::LESS_EQUAL>;
using IntensityGE = Intensity<PredicateCompare::GREATER_EQUAL>;
using IntensityL = Intensity<PredicateCompare::LESS>;
using IntensityG = Intensity<PredicateCompare::GREATER>;

using IntensityMin = IntensityGE;
using IntensityMax = IntensityLE;

struct IntensityInterval {
	constexpr IntensityInterval(intensity_t min, intensity_t max) : min(min), max(max) {}

	IntensityMin min;
	IntensityMax max;
};

//
// Predicate value/return check
//

template <>
struct PredicateValueCheck<IntensityMap> {
	using Pred = IntensityMap;

	template <class Map>
	static constexpr auto apply(Pred const&, Map const& m, Node const& n)
	    -> decltype(m.intensity(n), true)
	{
		return true;
	}

	static constexpr bool apply(...) { return false; }
};

template <class PredPost>
struct PredicateValueCheck<THEN<IntensityMap, PredPost>> {
	using Pred = THEN<IntensityMap, PredPost>;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (PredicateValueCheck<IntensityMap>::apply(p.pre, m, n)) {
			return PredicateValueCheck<PredPost>::apply(p.post, m, n);
		} else {
			return true;
		}
	}
};

template <PredicateCompare PC>
struct PredicateValueCheck<Intensity<PC>> {
	using Pred = Intensity<PC>;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (PredicateCompare::EQUAL == PC) {
			return m.intensity(n) == p.intensity;
		} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
			return m.intensity(n) <= p.intensity;
		} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
			return m.intensity(n) >= p.intensity;
		} else if constexpr (PredicateCompare::LESS == PC) {
			return m.intensity(n) < p.intensity;
		} else if constexpr (PredicateCompare::GREATER == PC) {
			return m.intensity(n) > p.intensity;
		} else {
			static_assert("Non-supported predicate comparison.");
		}
	}
};

template <>
struct PredicateValueCheck<IntensityInterval> {
	using Pred = IntensityInterval;

	template <class Map>
	static inline bool apply(Pred const& p, Map const& m, Node const& n)
	{
		return PredicateValueCheck<std::decay_t<decltype(p.min)>>::apply(p.min, m, n) &&
		       PredicateValueCheck<std::decay_t<decltype(p.max)>>::apply(p.max, m, n);
	}
};

//
// Predicate inner check
//

template <>
struct PredicateInnerCheck<IntensityMap> {
	using Pred = IntensityMap;

	template <class Map>
	static constexpr auto apply(Pred const&, Map const& m, Node const& n)
	    -> decltype(m.intensity(n), true)
	{
		return true;
	}

	static constexpr bool apply(...) { return false; }
};

template <class PredPost>
struct PredicateInnerCheck<THEN<IntensityMap, PredPost>> {
	using Pred = THEN<IntensityMap, PredPost>;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (PredicateInnerCheck<IntensityMap>::apply(p.pre, m, n)) {
			return PredicateInnerCheck<PredPost>::apply(p.post, m, n);
		} else {
			return true;
		}
	}
};

template <PredicateCompare PC>
struct PredicateInnerCheck<Intensity<PC>> {
	using Pred = Intensity<PC>;

	template <class Map>
	static inline bool apply(Pred const& p, Map const& m, Node const& n)
	{
		// FIXME: Check how intensity step is propagated to determine

		if constexpr (PredicateCompare::EQUAL == PC) {
			return m.intensity(n) >= p.intensity;
		} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
			return true;
		} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
			return m.intensity(n) >= p.intensity;
		} else if constexpr (PredicateCompare::LESS == PC) {
			return true;
		} else if constexpr (PredicateCompare::GREATER == PC) {
			return m.intensity(n) > p.intensity;
		} else {
			static_assert("Non-supported predicate comparison.");
		}
	}
};

template <>
struct PredicateInnerCheck<IntensityInterval> {
	using Pred = IntensityInterval;

	template <class Map>
	static inline bool apply(Pred const& p, Map const& m, Node const& n)
	{
		return PredicateInnerCheck<std::decay_t<decltype(p.min)>>::apply(p.min, m, n) &&
		       PredicateInnerCheck<std::decay_t<decltype(p.max)>>::apply(p.max, m, n);
	}
};

}  // namespace ufo::map::predicate

#endif  // UFO_MAP_INTENSITY_PREDICATE_H