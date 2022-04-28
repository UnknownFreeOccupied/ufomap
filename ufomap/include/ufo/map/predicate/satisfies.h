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

#ifndef UFO_MAP_PREDICATE_SATISFIES_H
#define UFO_MAP_PREDICATE_SATISFIES_H

// UFO
#include <ufo/map/predicate/predicates.h>

// STL
#include <functional>

namespace ufo::map::predicate
{
//
// Satisfies
//

template <class Fun, bool IsFunction>
struct SatisfiesImpl {
	SatisfiesImpl(Fun fun) : fun(fun) {}

	Fun* fun;
};

template <class Fun>
struct SatisfiesImpl<Fun, false> {
	SatisfiesImpl(Fun const& fun) : fun(fun) {}

	Fun fun;
};

template <class Fun, bool Negated = false>
struct Satisfies : SatisfiesImpl<Fun, std::is_function_v<Fun>> {
	using Base = SatisfiesImpl<Fun, std::is_function_v<Fun>>;

	Satisfies(Fun const& fun) : Base(fun) {}

	Satisfies(Base const& b) : Base(b) {}
};

template <class Fun, bool Negated = false>
struct SatisfiesInner : SatisfiesImpl<Fun, std::is_function_v<Fun>> {
	using Base = SatisfiesImpl<Fun, std::is_function_v<Fun>>;

	SatisfiesInner(Fun const& fun) : Base(fun) {}

	SatisfiesInner(Base const& b) : Base(b) {}
};

//
// Satisfied negate
//

template <class Fun, bool Negated>
constexpr Satisfies<Fun, !Negated> operator!(Satisfies<Fun, Negated> const& p)
{
	return Satisfies<Fun, !Negated>(p);
}

template <class Fun, bool Negated>
constexpr SatisfiesInner<Fun, !Negated> operator!(SatisfiesInner<Fun, Negated> const& p)
{
	return SatisfiesInner<Fun, !Negated>(p);
}

//
// Predicate value check
//

template <class Fun, bool Negated>
struct PredicateValueCheck<Satisfies<Fun, Negated>> {
	using Pred = Satisfies<Fun, Negated>;

	template <class Tree>
	static inline bool apply(Pred const& p, Tree const&, Node const& n)
	{
		if constexpr (Negated) {
			return !p.fun(n);
		} else {
			return p.fun(n);
		}
	}

	template <class Tree>
	static inline bool apply(Pred const& p, Tree const&, MinimalNode const& n)
	{
		if constexpr (Negated) {
			return !p.fun(n);
		} else {
			return p.fun(n);
		}
	}
};

template <class Fun, bool Negated>
struct PredicateValueCheck<SatisfiesInner<Fun, Negated>> {
	using Pred = SatisfiesInner<Fun, Negated>;

	template <class Tree>
	static inline constexpr bool apply(Pred const&, Tree const&, MinimalNode const&)
	{
		return true;
	}

	template <class Tree>
	static inline constexpr bool apply(Pred const&, Tree const&, Node const&)
	{
		return true;
	}
};

//
// Predicate inner check
//

template <class Fun, bool Negated>
struct PredicateInnerCheck<Satisfies<Fun, Negated>> {
	using Pred = Satisfies<Fun, Negated>;

	template <class Tree, class Node>
	static inline constexpr bool apply(Pred const&, Tree const&, Node const&)
	{
		return true;
	}
};

template <class Fun, bool Negated>
struct PredicateInnerCheck<SatisfiesInner<Fun, Negated>> {
	using Pred = SatisfiesInner<Fun, Negated>;

	template <class Tree, class Node>
	static inline bool apply(Pred const& p, Tree const&, Node const& n)
	{
		if constexpr (Negated) {
			return !p.fun(n);
		} else {
			return p.fun(n);
		}
	}
};
}  // namespace ufo::map::predicate

#endif  // UFO_MAP_PREDICATE_SATISFIES_H