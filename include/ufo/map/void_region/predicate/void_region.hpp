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

#ifndef UFO_MAP_VOID_REGION_PREDICATE_VOID_REGION_HPP
#define UFO_MAP_VOID_REGION_PREDICATE_VOID_REGION_HPP

// UFO
#include <ufo/container/tree/predicate/filter.hpp>
#include <ufo/utility/type_traits.hpp>

namespace ufo::pred
{
enum class VoidRegionType { PRIMARY = 1, SECONDARY = 2, ANY = 3, ALL = 4 };

template <VoidRegionType Type = VoidRegionType::PRIMARY, bool Negated = false>
struct VoidRegionT {
};

template <VoidRegionType Type, bool Negated>
VoidRegionT<Type, !Negated> operator!(VoidRegionT<Type, Negated>)
{
	return VoidRegionT<Type, !Negated>{};
}

using VoidRegion          = VoidRegionT<VoidRegionType::PRIMARY, false>;
using VoidRegionSecondary = VoidRegionT<VoidRegionType::SECONDARY, false>;
using VoidRegionAny       = VoidRegionT<VoidRegionType::ANY, false>;
using VoidRegionAll       = VoidRegionT<VoidRegionType::ALL, false>;

template <VoidRegionType Type, bool Negated>
struct Filter<VoidRegionT<Type, Negated>> {
	using Pred = VoidRegionT<Type, Negated>;

	template <class Tree>
	static constexpr void init(Pred&, Tree const&)
	{
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool returnable(Pred const& p, Tree const& t,
	                                               typename Tree::Node const& n)
	{
		if constexpr (VoidRegionType::PRIMARY == Type) {
			if constexpr (Negated) {
				return !t.voidRegion(n.index);
			} else {
				return t.voidRegion(n.index);
			}
		} else if constexpr (VoidRegionType::SECONDARY == Type) {
			if constexpr (Negated) {
				return !t.voidRegionSecondary(n.index);
			} else {
				return t.voidRegionSecondary(n.index);
			}
		} else if constexpr (VoidRegionType::ANY == Type) {
			if constexpr (Negated) {
				return !t.voidRegion(n.index) || !t.voidRegionSecondary(n.index);
			} else {
				return t.voidRegion(n.index) || t.voidRegionSecondary(n.index);
			}
		} else if constexpr (VoidRegionType::ALL == Type) {
			if constexpr (Negated) {
				return !t.voidRegion(n.index) && !t.voidRegionSecondary(n.index);
			} else {
				return t.voidRegion(n.index) && t.voidRegionSecondary(n.index);
			}
		} else {
			static_assert(dependent_false_v<Tree>, "Non-supported void region type");
		}
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool traversable(Pred const& p, Tree const& t,
	                                                typename Tree::Node const& n)
	{
		if constexpr (VoidRegionType::PRIMARY == Type) {
			if constexpr (Negated) {
				return !t.voidRegion(n.index);
			} else {
				return t.voidRegionContains(n.index);
			}
		} else if constexpr (VoidRegionType::SECONDARY == Type) {
			if constexpr (Negated) {
				return !t.voidRegionSecondary(n.index);
			} else {
				return t.voidRegionSecondaryContains(n.index);
			}
		} else if constexpr (VoidRegionType::ANY == Type) {
			if constexpr (Negated) {
				return !t.voidRegion(n.index) || !t.voidRegionSecondary(n.index);
			} else {
				return t.voidRegionContains(n.index) || t.voidRegionSecondaryContains(n.index);
			}
		} else if constexpr (VoidRegionType::ALL == Type) {
			if constexpr (Negated) {
				return !t.voidRegion(n.index) && !t.voidRegionSecondary(n.index);
			} else {
				return t.voidRegionContains(n.index) && t.voidRegionSecondaryContains(n.index);
			}
		} else {
			static_assert(dependent_false_v<Tree>, "Non-supported void region type");
		}
	}
};
}  // namespace ufo::pred

#endif  // UFO_MAP_VOID_REGION_PREDICATE_VOID_REGION_HPP
