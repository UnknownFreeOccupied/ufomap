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

#ifndef UFO_MAP_COLOR_PREDICATE_ALPHA_INTERVAL_HPP
#define UFO_MAP_COLOR_PREDICATE_ALPHA_INTERVAL_HPP

// UFO
#include <ufo/map/color/predicate/alpha.hpp>

namespace ufo::pred
{
template <bool Negated = false>
struct AlphaInterval {
	AlphaMin min;
	AlphaMax max;

	AlphaInterval(color_t min, color_t max) : min(min), max(max) {}

	float minAlphaf() const { return min.alphaf(); }

	void minAlphaf(float v) { min.alphaf(v); }

	float maxAlphaf() const { return max.alphaf(); }

	void maxAlphaf(float v) { max.alphaf(v); }
};

template <bool Negated>
AlphaInterval<!Negated> operator!(AlphaInterval<Negated> p)
{
	return {p.min, p.max};
}

template <bool Negated>
struct Init<AlphaInterval<Negated>> {
	using Pred = AlphaInterval<Negated>;

	template <class Map>
	static inline void apply(Pred& p, Map const& m)
	{
		Init<std::decay_t<decltype(p.min)>>::apply(p.min, m);
		Init<std::decay_t<decltype(p.max)>>::apply(p.max, m);
	}
};

template <bool Negated>
struct InnerCheck<AlphaInterval<Negated>> {
	using Pred = AlphaInterval<Negated>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (Negated) {
			return true;
		} else {
			return InnerCheck<std::decay_t<decltype(p.min)>>::apply(p.min, m, n) &&
			       InnerCheck<std::decay_t<decltype(p.max)>>::apply(p.max, m, n);
		}
	}
};

template <bool Negated>
struct ValueCheck<AlphaInterval<Negated>> {
	using Pred = AlphaInterval<Negated>;

	template <class Map, class Node>
	static inline bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (Negated) {
			return !ValueCheck<std::decay_t<decltype(p.min)>>::apply(p.min, m, n) ||
			       !ValueCheck<std::decay_t<decltype(p.max)>>::apply(p.max, m, n);
		} else {
			return ValueCheck<std::decay_t<decltype(p.min)>>::apply(p.min, m, n) &&
			       ValueCheck<std::decay_t<decltype(p.max)>>::apply(p.max, m, n);
		}
	}
};
}  // namespace ufo::pred

#endif  // UFO_MAP_COLOR_PREDICATE_ALPHA_INTERVAL_HPP