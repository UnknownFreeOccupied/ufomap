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

#ifndef UFO_MAP_TIME_PREDICATE_H
#define UFO_MAP_TIME_PREDICATE_H

// UFO
#include <ufo/map/predicate/predicates.h>
#include <ufo/map/types.h>

namespace ufo::map::predicate
{
//
// Predicates
//

// REVIEW: Name?
struct TimeMap {
};

template <PredicateCompare PC = PredicateCompare::EQUAL>
struct Time {
	constexpr Time(time_t time) : time(time) {}

	time_t time;
};

using TimeE = Time<>;
using TimeLE = Time<PredicateCompare::LESS_EQUAL>;
using TimeGE = Time<PredicateCompare::GREATER_EQUAL>;
using TimeL = Time<PredicateCompare::LESS>;
using TimeG = Time<PredicateCompare::GREATER>;

using TimeMin = TimeGE;
using TimeMax = TimeLE;

struct TimeInterval {
	constexpr TimeInterval(time_t min, time_t max) : min(min), max(max) {}

	TimeMin min;
	TimeMax max;
};

//
// Predicate value/return check
//

template <>
struct PredicateValueCheck<TimeMap> {
	using Pred = TimeMap;

	template <class Map>
	static constexpr auto apply(Pred, Map const& m, Node n) -> decltype(m.time(n), true)
	{
		return true;  // TODO: Improve
	}

	static constexpr bool apply(...) { return false; }
};

template <class PredPost>
struct PredicateValueCheck<THEN<TimeMap, PredPost>> {
	using Pred = THEN<TimeMap, PredPost>;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (PredicateValueCheck<TimeMap>::apply(p.pre, m, n)) {
			return PredicateValueCheck<PredPost>::apply(p.post, m, n);
		} else {
			return true;
		}
	}
};

template <PredicateCompare PC>
struct PredicateValueCheck<Time<PC>> {
	using Pred = Time<PC>;

	template <class Map>
	static constexpr bool apply(Pred p, Map const& m, Node n)
	{
		if constexpr (PredicateCompare::EQUAL == PC) {
			return m.time(n) == p.time;
		} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
			return m.time(n) <= p.time;
		} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
			return m.time(n) >= p.time;
		} else if constexpr (PredicateCompare::LESS == PC) {
			return m.time(n) < p.time;
		} else if constexpr (PredicateCompare::GREATER == PC) {
			return m.time(n) > p.time;
		} else {
			static_assert("Non-supported predicate comparison.");
		}
	}
};

template <>
struct PredicateValueCheck<TimeInterval> {
	using Pred = TimeInterval;

	template <class Map>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		return PredicateValueCheck<std::decay_t<decltype(p.min)>>::apply(p.min, m, n) &&
		       PredicateValueCheck<std::decay_t<decltype(p.max)>>::apply(p.max, m, n);
	}
};

//
// Predicate inner check
//

template <>
struct PredicateInnerCheck<TimeMap> {
	using Pred = TimeMap;

	template <class Map>
	static constexpr auto apply(Pred, Map const& m, Node n) -> decltype(m.time(n), true)
	{
		return true;
	}

	static constexpr bool apply(...) { return false; }
};

template <class PredPost>
struct PredicateInnerCheck<THEN<TimeMap, PredPost>> {
	using Pred = THEN<TimeMap, PredPost>;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (PredicateInnerCheck<TimeMap>::apply(p.pre, m, n)) {
			return PredicateInnerCheck<PredPost>::apply(p.post, m, n);
		} else {
			return true;
		}
	}
};

template <PredicateCompare PC>
struct PredicateInnerCheck<Time<PC>> {
	using Pred = Time<PC>;

	template <class Map>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		// FIXME: Check how time step is propagated to determine

		if constexpr (PredicateCompare::EQUAL == PC) {
			return m.time(n) >= p.time;
		} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
			return true;
		} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
			return m.time(n) >= p.time;
		} else if constexpr (PredicateCompare::LESS == PC) {
			return true;
		} else if constexpr (PredicateCompare::GREATER == PC) {
			return m.time(n) > p.time;
		} else {
			static_assert("Non-supported predicate comparison.");
		}
	}
};

template <>
struct PredicateInnerCheck<TimeInterval> {
	using Pred = TimeInterval;

	template <class Map>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		return PredicateInnerCheck<std::decay_t<decltype(p.min)>>::apply(p.min, m, n) &&
		       PredicateInnerCheck<std::decay_t<decltype(p.max)>>::apply(p.max, m, n);
	}
};

}  // namespace ufo::map::predicate

#endif  // UFO_MAP_TIME_PREDICATE_H