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

#ifndef UFO_MAP_INTENSITY_PREDICATE_INTENSITY_MAP_HPP
#define UFO_MAP_INTENSITY_PREDICATE_INTENSITY_MAP_HPP

// UFO
#include <ufo/map/intensity/is_intensity_map.hpp>
#include <ufo/map/predicate/predicate.hpp>

namespace ufo::pred
{
template <bool Negated = false>
struct IntensityMap {
 private:
	bool enabled;

	friend struct Init<IntensityMap>;
	friend struct InnerCheck<IntensityMap>;
	friend struct ValueCheck<IntensityMap>;
	friend struct ShaderInnerCheck<IntensityMap>;
	friend struct ShaderValueCheck<IntensityMap>;
};

template <bool Negated>
IntensityMap<!Negated> operator!(IntensityMap<Negated>)
{
	return {};
}

template <bool Negated>
struct Init<IntensityMap<Negated>> {
	using Pred = IntensityMap<Negated>;

	template <class Map>
	static inline void apply(Pred& p, Map const& m)
	{
		p.enabled = m.isMapTypesEnabled(MapType::INTENSITY);
	}
};

template <bool Negated>
struct InnerCheck<IntensityMap<Negated>> {
	using Pred = IntensityMap<Negated>;

	template <class Map, class Node>
	static constexpr bool apply(Pred p, Map const&, Node const&)
	{
		if constexpr (!Map::enableDisableUtilityPresent()) {
			if constexpr (Negated) {
				return !is_intensity_map_v<Map>;
			} else {
				return is_intensity_map_v<Map>;
			}
		} else {
			if constexpr (Negated) {
				return !p.enabled;
			} else {
				return p.enabled;
			}
		}
	}
};

template <bool Negated, class PredPost>
struct InnerCheck<Then<IntensityMap<Negated>, PredPost>> {
	using Pred = Then<IntensityMap<Negated>, PredPost>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (!Map::enableDisableUtilityPresent()) {
			if constexpr (InnerCheck<IntensityMap<Negated>>::apply(p.pre, m, n)) {
				return InnerCheck<PredPost>::apply(p.post, m, n);
			} else {
				return true;
			}
		} else {
			return !InnerCheck<IntensityMap<Negated>>::apply(p.pre, m, n) ||
			       InnerCheck<PredPost>::apply(p.post, m, n);
		}
	}
};

template <bool Negated>
struct ValueCheck<IntensityMap<Negated>> {
	using Pred = IntensityMap<Negated>;

	template <class Map, class Node>
	static constexpr bool apply(Pred p, Map const&, Node const& n)
	{
		if constexpr (!Map::enableDisableUtilityPresent()) {
			if constexpr (Negated) {
				return !is_intensity_map_v<Map>;
			} else {
				return is_intensity_map_v<Map>;
			}
		} else {
			if constexpr (Negated) {
				return !p.enabled;
			} else {
				return p.enabled;
			}
		}
	}
};

template <bool Negated, class PredPost>
struct ValueCheck<Then<IntensityMap<Negated>, PredPost>> {
	using Pred = Then<IntensityMap<Negated>, PredPost>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (!Map::enableDisableUtilityPresent()) {
			if constexpr (ValueCheck<IntensityMap<Negated>>::apply(p.pre, m, n)) {
				return ValueCheck<PredPost>::apply(p.post, m, n);
			} else {
				return true;
			}
		} else {
			return !ValueCheck<IntensityMap<Negated>>::apply(p.pre, m, n) ||
			       ValueCheck<PredPost>::apply(p.post, m, n);
		}
	}
};
}  // namespace ufo::pred

#endif  // UFO_MAP_INTENSITY_PREDICATE_INTENSITY_MAP_HPP