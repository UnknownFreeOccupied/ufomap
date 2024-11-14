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

#ifndef UFO_MAP_SEEN_EMPTY_PREDICATE_SEEN_EMPTY_HPP
#define UFO_MAP_SEEN_EMPTY_PREDICATE_SEEN_EMPTY_HPP

// UFO
#include <ufo/map/predicate/predicate.hpp>
#include <ufo/map/seen_empty/seen_empty_propagation_criteria.hpp>

namespace ufo::pred
{
template <bool Negated = false>
struct SeenEmpty {
};

template <bool Negated>
SeenEmpty<!Negated> operator!(SeenEmpty<Negated>)
{
	return {};
}

template <bool Negated>
struct InnerCheck<SeenEmpty<Negated>> {
	using Pred = SeenEmpty<Negated>;

	template <class Map, class Node>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		switch (m.seenEmptyPropagationCriteria()) {
			case SeenEmptyPropagationCriteria::ALL:
				if constexpr (Negated) {
					return !m.seenEmpty(n.index());
				} else {
					return true;
				}
			case SeenEmptyPropagationCriteria::ANY:
				if constexpr (Negated) {
					return true;
				} else {
					return m.seenEmpty(n.index());
				}
			case SeenEmptyPropagationCriteria::SOME: return true;
			case SeenEmptyPropagationCriteria::NONE: return true;
		}
	}
};

template <bool Negated>
struct ValueCheck<SeenEmpty<Negated>> {
	using Pred = SeenEmpty<Negated>;

	template <class Map, class Node>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		if constexpr (Negated) {
			return !m.seenEmpty(n.index());
		} else {
			return m.seenEmpty(n.index());
		}
	}
};
}  // namespace ufo::pred

#endif  // UFO_MAP_SEEN_EMPTY_PREDICATE_SEEN_EMPTY_HPP