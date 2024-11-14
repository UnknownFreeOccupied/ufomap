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

#ifndef UFO_MAP_COLOR_PREDICATE_ALPHA_HPP
#define UFO_MAP_COLOR_PREDICATE_ALPHA_HPP

// UFO
#include <ufo/map/color/color.hpp>
#include <ufo/map/predicate/predicate.hpp>
#include <ufo/map/predicate/predicate_compare.hpp>
#include <ufo/map/types.hpp>

// STL
#include <cassert>
#include <limits>

namespace ufo::pred
{
template <PredicateCompare PC = PredicateCompare::EQUAL>
struct Alpha {
	Alpha(color_t alpha) : alpha(alpha) {}

	float alphaf() const
	{
		return alpha / static_cast<float>(std::numeric_limits<color_t>::max());
	}

	void alphaf(float v)
	{
		assert(0.0f <= v && 1.0f >= v);
		alpha = static_cast<color_t>(std::numeric_limits<color_t>::max() * v);
	}

	color_t alpha;
};

using AlphaE  = Alpha<>;
using AlphaNE = Alpha<PredicateCompare::NOT_EQUAL>;
using AlphaLE = Alpha<PredicateCompare::LESS_EQUAL>;
using AlphaGE = Alpha<PredicateCompare::GREATER_EQUAL>;
using AlphaL  = Alpha<PredicateCompare::LESS>;
using AlphaG  = Alpha<PredicateCompare::GREATER>;

using AlphaMin = AlphaGE;
using AlphaMax = AlphaLE;

template <PredicateCompare PC>
struct InnerCheck<Alpha<PC>> {
	using Pred = Alpha<PC>;

	template <class Map, class Node>
	static constexpr bool apply(Pred p, Map const& m, Node const& n)
	{
		return true;
	}
};

template <PredicateCompare PC>
struct ValueCheck<Alpha<PC>> {
	using Pred = Alpha<PC>;

	template <class Map, class Node>
	static inline bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (PredicateCompare::EQUAL == PC) {
			return m.color(n.index()).alpha == p.alpha;
		} else if constexpr (PredicateCompare::NOT_EQUAL == PC) {
			return m.color(n.index()).alpha != p.alpha;
		} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
			return m.color(n.index()) <= p.alpha;
		} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
			return m.color(n.index()) >= p.alpha;
		} else if constexpr (PredicateCompare::LESS == PC) {
			return m.color(n.index()) < p.alpha;
		} else if constexpr (PredicateCompare::GREATER == PC) {
			return m.color(n.index()) > p.alpha;
		}
	}
};
}  // namespace ufo::pred

#endif  // UFO_MAP_COLOR_PREDICATE_ALPHA_HPP