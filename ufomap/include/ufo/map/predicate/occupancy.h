/*
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * License: BSD 3
 */

/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, D. Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
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

#ifndef UFO_MAP_PREDICATE_OCCUPANCY_H
#define UFO_MAP_PREDICATE_OCCUPANCY_H

// UFO
#include <ufo/map/predicate/predicates.h>
#include <ufo/map/types.h>

// STL
#include <functional>

namespace ufo::map::predicate
{
//
// Predicates
//

// REVIEW: Name?
struct OccupancyMap {
};

template <map::OccupancyState state>
struct OccupancyState {
};

using Unknown = OccupancyState<map::OccupancyState::UNKNOWN>;
using Free = OccupancyState<map::OccupancyState::FREE>;
using Occupied = OccupancyState<map::OccupancyState::OCCUPIED>;

struct OccupancyStates {
	OccupancyStates(bool unknown, bool free, bool occupied)
	    : unknown(unknown), free(free), occupied(occupied)
	{
	}

	bool unknown;
	bool free;
	bool occupied;
};

template <map::OccupancyState state>
struct ContainOccupancyState {
};

using ContainUnknown = ContainOccupancyState<map::OccupancyState::UNKNOWN>;
using ContainFree = ContainOccupancyState<map::OccupancyState::FREE>;
using ContainOccupied = ContainOccupancyState<map::OccupancyState::OCCUPIED>;

struct ContainOccupancyStates {
	ContainOccupancyStates(bool unknown, bool free, bool occupied)
	    : unknown(unknown), free(free), occupied(occupied)
	{
	}

	bool unknown;
	bool free;
	bool occupied;
};

template <PredicateCompare PC = PredicateCompare::EQUAL>
struct Occupancy {
	Occupancy(double occupancy) : occupancy(occupancy) {}

	constexpr void setOccupancy(double new_occupancy)
	{
		occupancy_modified = occupancy_modified || occupancy != new_occupancy;
		occupancy = new_occupancy;
	}

	constexpr double getOccupancy() const { return occupancy; }

 private:
	double occupancy;
	mutable float logit_f;
	mutable uint8_t logit_u8;
	mutable bool occupancy_modified = true;

	friend struct PredicateValueCheck<Occupancy<PC>>;
	friend struct PredicateInnerCheck<Occupancy<PC>>;
};

using OccupancyE = Occupancy<>;
using OccupancyLE = Occupancy<PredicateCompare::LESS_EQUAL>;
using OccupancyGE = Occupancy<PredicateCompare::GREATER_EQUAL>;
using OccupancyL = Occupancy<PredicateCompare::LESS>;
using OccupancyG = Occupancy<PredicateCompare::GREATER>;

using OccupancyMin = OccupancyGE;
using OccupancyMax = OccupancyLE;

struct OccupancyInterval {
	OccupancyInterval(double min, double max) : min(min), max(max) {}

	OccupancyMin min;
	OccupancyMax max;
};

//
// Predicate value/return check
//

template <>
struct PredicateValueCheck<OccupancyMap> {
	using Pred = OccupancyMap;

	template <class Map>
	static constexpr auto apply(Pred const&, Map const& m, Node const& n)
	    -> decltype(m.isUnknown(n), true)
	{
		return true;
	}

	static constexpr bool apply(...) { return false; }
};

template <class PredPost>
struct PredicateValueCheck<THEN<OccupancyMap, PredPost>> {
	using Pred = THEN<OccupancyMap, PredPost>;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (PredicateValueCheck<OccupancyMap>::apply(p.pre, m, n)) {
			return PredicateValueCheck<PredPost>::apply(p.post, m, n);
		} else {
			return true;
		}
	}
};

template <map::OccupancyState state>
struct PredicateValueCheck<OccupancyState<state>> {
	using Pred = OccupancyState<state>;

	template <class Map>
	static inline bool apply(Pred const&, Map const& m, Node const& n)
	{
		if constexpr (map::OccupancyState::UNKNOWN == state) {
			return m.isUnknown(n);
		} else if constexpr (map::OccupancyState::FREE == state) {
			return m.isFree(n);
		} else if constexpr (map::OccupancyState::OCCUPIED == state) {
			return m.isOccupied(n);
		} else {
			static_assert("Non-supported occupancy state.");
		}
	}
};

template <>
struct PredicateValueCheck<OccupancyStates> {
	using Pred = OccupancyStates;

	template <class Map>
	static inline bool apply(Pred const& p, Map const& m, Node const& n)
	{
		return (p.unknown && m.isUnknown(n)) || (p.free && m.isFree(n)) ||
		       (p.occupied && m.isOccupied(n));
	}
};

template <map::OccupancyState state>
struct PredicateValueCheck<ContainOccupancyState<state>> {
	using Pred = ContainOccupancyState<state>;

	template <class Map>
	static inline bool apply(Pred const&, Map const& m, Node const& n)
	{
		if constexpr (map::OccupancyState::UNKNOWN == state) {
			return m.containsUnknown(n);
		} else if constexpr (map::OccupancyState::FREE == state) {
			return m.containsFree(n);
		} else if constexpr (map::OccupancyState::OCCUPIED == state) {
			return m.containsOccupied(n);
		} else {
			static_assert("Non-supported occupancy state.");
		}
	}
};

template <>
struct PredicateValueCheck<ContainOccupancyStates> {
	using Pred = ContainOccupancyStates;

	template <class Map>
	static inline bool apply(Pred const& p, Map const& m, Node const& n)
	{
		return (p.unknown && m.containsUnknown(n)) || (p.free && m.containsFree(n)) ||
		       (p.occupied && m.containsOccupied(n));
	}
};

template <PredicateCompare PC>
struct PredicateValueCheck<Occupancy<PC>> {
	using Pred = Occupancy<PC>;

	template <class Map>
	static inline bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if (p.occupancy_modified) {
			if constexpr (std::is_same_v<uint8_t, typename Map::LogitType>) {
				p.logit_u8 = m.toOccupancyLogit(p.occupancy);
			} else {
				p.logit_f = m.toOccupancyLogit(p.occupancy);
			}
			p.occupancy_modified = false;
		}

		if constexpr (std::is_same_v<uint8_t, typename Map::LogitType>) {
			if constexpr (PredicateCompare::EQUAL == PC) {
				return m.getOccupancyLogit(n) == p.logit_u8;
			} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
				return m.getOccupancyLogit(n) <= p.logit_u8;
			} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
				return m.getOccupancyLogit(n) >= p.logit_u8;
			} else if constexpr (PredicateCompare::LESS == PC) {
				return m.getOccupancyLogit(n) < p.logit_u8;
			} else if constexpr (PredicateCompare::GREATER == PC) {
				return m.getOccupancyLogit(n) > p.logit_u8;
			} else {
				static_assert("Non-supported predicate comparison.");
			}
		} else {
			if constexpr (PredicateCompare::EQUAL == PC) {
				return m.getOccupancyLogit(n) == p.logit_f;
			} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
				return m.getOccupancyLogit(n) <= p.logit_f;
			} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
				return m.getOccupancyLogit(n) >= p.logit_f;
			} else if constexpr (PredicateCompare::LESS == PC) {
				return m.getOccupancyLogit(n) < p.logit_f;
			} else if constexpr (PredicateCompare::GREATER == PC) {
				return m.getOccupancyLogit(n) > p.logit_f;
			} else {
				static_assert("Non-supported predicate comparison.");
			}
		}
	}
};

template <>
struct PredicateValueCheck<OccupancyInterval> {
	using Pred = OccupancyInterval;

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
struct PredicateInnerCheck<OccupancyMap> {
	using Pred = OccupancyMap;

	template <class Map>
	static constexpr auto apply(Pred const&, Map const& m, Node const& n)
	    -> decltype(m.isOccupied(n), true)
	{
		return true;
	}

	static constexpr bool apply(...) { return false; }
};

template <class PredPost>
struct PredicateInnerCheck<THEN<OccupancyMap, PredPost>> {
	using Pred = THEN<OccupancyMap, PredPost>;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (PredicateInnerCheck<OccupancyMap>::apply(p.pre, m, n)) {
			return PredicateInnerCheck<PredPost>::apply(p.post, m, n);
		} else {
			return true;
		}
	}
};

template <map::OccupancyState state>
struct PredicateInnerCheck<OccupancyState<state>> {
	using Pred = OccupancyState<state>;

	template <class Map>
	static inline bool apply(Pred const&, Map const& m, Node const& n)
	{
		if constexpr (map::OccupancyState::UNKNOWN == state) {
			return m.containsUnknown(n);
		} else if constexpr (map::OccupancyState::FREE == state) {
			return m.containsFree(n);
		} else if constexpr (map::OccupancyState::OCCUPIED == state) {
			return m.containsOccupied(n);
		} else {
			static_assert("Non-supported occupancy state.");
		}
	}
};

template <>
struct PredicateInnerCheck<OccupancyStates> {
	using Pred = OccupancyStates;

	template <class Map>
	static inline bool apply(Pred const& p, Map const& m, Node const& n)
	{
		return (p.unknown && m.containsUnknown(n)) || (p.free && m.containsFree(n)) ||
		       (p.occupied && m.containsOccupied(n));
	}
};

template <map::OccupancyState state>
struct PredicateInnerCheck<ContainOccupancyState<state>> {
	using Pred = ContainOccupancyState<state>;

	template <class Map>
	static inline bool apply(Pred const&, Map const& m, Node const& n)
	{
		if constexpr (map::OccupancyState::UNKNOWN == state) {
			return m.containsUnknown(n);
		} else if constexpr (map::OccupancyState::FREE == state) {
			return m.containsFree(n);
		} else if constexpr (map::OccupancyState::OCCUPIED == state) {
			return m.containsOccupied(n);
		} else {
			static_assert("Non-supported occupancy state.");
		}
	}
};

template <>
struct PredicateInnerCheck<ContainOccupancyStates> {
	using Pred = ContainOccupancyStates;

	template <class Map>
	static inline bool apply(Pred const& p, Map const& m, Node const& n)
	{
		return (p.unknown && m.containsUnknown(n)) || (p.free && m.containsFree(n)) ||
		       (p.occupied && m.containsOccupied(n));
	}
};

template <PredicateCompare PC>
struct PredicateInnerCheck<Occupancy<PC>> {
	using Pred = Occupancy<PC>;

	template <class Map>
	static inline bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if (p.occupancy_modified) {
			if constexpr (std::is_same_v<uint8_t, typename Map::LogitType>) {
				p.logit_u8 = m.toOccupancyLogit(p.occupancy);
			} else {
				p.logit_f = m.toOccupancyLogit(p.occupancy);
			}
			p.occupancy_modified = false;
		}

		// FIXME: Check how occupancy is propagated to determine

		if constexpr (std::is_same_v<uint8_t, typename Map::LogitType>) {
			if constexpr (PredicateCompare::EQUAL == PC) {
				return m.getOccupancyLogit(n) >= p.logit_u8;
			} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
				return true;
			} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
				return m.getOccupancyLogit(n) >= p.logit_u8;
			} else if constexpr (PredicateCompare::LESS == PC) {
				return true;
			} else if constexpr (PredicateCompare::GREATER == PC) {
				return m.getOccupancyLogit(n) > p.logit_u8;
			} else {
				static_assert("Non-supported predicate comparison.");
			}
		} else {
			if constexpr (PredicateCompare::EQUAL == PC) {
				return m.getOccupancyLogit(n) >= p.logit_f;
			} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
				return true;
			} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
				return m.getOccupancyLogit(n) >= p.logit_f;
			} else if constexpr (PredicateCompare::LESS == PC) {
				return true;
			} else if constexpr (PredicateCompare::GREATER == PC) {
				return m.getOccupancyLogit(n) > p.logit_f;
			} else {
				static_assert("Non-supported predicate comparison.");
			}
		}
	}
};

template <>
struct PredicateInnerCheck<OccupancyInterval> {
	using Pred = OccupancyInterval;

	template <class Map>
	static inline bool apply(Pred const& p, Map const& m, Node const& n)
	{
		return PredicateInnerCheck<std::decay_t<decltype(p.min)>>::apply(p.min, m, n) &&
		       PredicateInnerCheck<std::decay_t<decltype(p.max)>>::apply(p.max, m, n);
	}
};
}  // namespace ufo::map::predicate

#endif  // UFO_MAP_PREDICATE_OCCUPANCY_H