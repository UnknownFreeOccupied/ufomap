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

#ifndef UFO_MAP_PREDICATE_PREDICATES_H
#define UFO_MAP_PREDICATE_PREDICATES_H

// STL
#include <tuple>

namespace ufo::map::predicate
{
//
// Predicate types
//

enum class PredicateType { RETURN_AND_INNER, RETURN, INNER };

//
// Predicate compare
//

enum class PredicateCompare { EQUAL, LESS_EQUAL, GREATER_EQUAL, LESS, GREATER };

//
// Predicate rounding
//

enum class PredicateRounding { NEAREST, FLOOR, CEIL };

//
// AND (&&)
//

template <class Pred1, class Pred2>
constexpr std::tuple<Pred1, Pred2> operator&&(Pred1 const& p1, Pred2 const& p2)
{
	return std::tuple<Pred1, Pred2>(p1, p2);
}

template <class... Preds1, class... Preds2>
constexpr std::tuple<Preds1..., Preds2...> operator&&(std::tuple<Preds1...> const& t1,
                                                      std::tuple<Preds2...> const& t2)
{
	return std::tuple_cat(t1, t2);
}

template <class... Preds, class Pred>
constexpr std::tuple<Preds..., Pred> operator&&(std::tuple<Preds...> const& t,
                                                Pred const& p)
{
	return std::tuple_cat(t, std::tie(p));
}

template <class Pred, class... Preds>
constexpr std::tuple<Pred, Preds...> operator&&(Pred const& p,
                                                std::tuple<Preds...> const& t)
{
	return std::tuple_cat(std::tie(p), t);
}

//
// OR (||)
//

template <class PredLeft, class PredRight>
struct OR {
	PredLeft left;
	PredRight right;
};

template <class PredLeft, class PredRight>
constexpr OR<PredLeft, PredRight> operator||(PredLeft const& p1, PredRight const& p2)
{
	return OR<PredLeft, PredRight>{p1, p2};
}

template <class... PredsLeft, class... PredsRight>
constexpr OR<std::tuple<PredsLeft...>, std::tuple<PredsRight...>> operator||(
    std::tuple<PredsLeft...> const& t1, std::tuple<PredsRight...> const& t2)
{
	return OR<std::tuple<PredsLeft...>, std::tuple<PredsRight...>>{t1, t2};
}

template <class... PredsLeft, class PredRight>
constexpr OR<std::tuple<PredsLeft...>, PredRight> operator||(
    std::tuple<PredsLeft...> const& t, PredRight const& p)
{
	return OR<std::tuple<PredsLeft...>, PredRight>{t, p};
}

template <class PredLeft, class... PredsRight>
constexpr OR<PredLeft, std::tuple<PredsRight...>> operator||(
    PredLeft const& p, std::tuple<PredsRight...> const& t)
{
	return OR<PredLeft, std::tuple<PredsRight...>>{p, t};
}

//
// THEN
//

template <class PredPre, class PredPost>
struct THEN {
	THEN(PredPre const& pre, PredPost const& post) : pre(pre), post(post) {}

	PredPre pre;
	PredPost post;
};

//
// True
//

struct TRUE {
};

//
// False
//

struct FALSE {
};

//
// Static assert check
//

template <bool Check, class... Ts>
struct static_assert_check : std::bool_constant<Check> {
};

// Helper variable template
template <bool Check, class... Ts>
inline constexpr bool static_assert_check_v = static_assert_check<Check, Ts...>::value;

//
// Predicate value/return check
//

template <class Predicate>
struct PredicateValueCheck {
	// REVIEW: Make better
	static_assert(static_assert_check_v<false, Predicate>,
	              "Not implemented for this Predicate.");
};

template <class... Preds>
struct PredicateValueCheck<std::tuple<Preds...>> {
	using Pred = std::tuple<Preds...>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		return std::apply(
		    [&m, &n](auto const&... preds) {
			    return (
			        (PredicateValueCheck<std::decay_t<decltype(preds)>>::apply(preds, m, n)) &&
			        ...);
		    },
		    p);
	}
};

template <class PredLeft, class PredRight>
struct PredicateValueCheck<OR<PredLeft, PredRight>> {
	using Pred = OR<PredLeft, PredRight>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		return PredicateValueCheck<PredLeft>::apply(p.left, m, n) ||
		       PredicateValueCheck<PredRight>::apply(p.right, m, n);
	}
};

template <class PredPre, class PredPost>
struct PredicateValueCheck<THEN<PredPre, PredPost>> {
	using Pred = THEN<PredPre, PredPost>;

	template <class Map, class Node>
	static constexpr auto apply(Pred const& p, Map const& m, Node const& n)
	    -> decltype(PredicateValueCheck<PredPre>::apply(p.pre, m, n), bool())
	{
		return !PredicateValueCheck<PredPre>::apply(p.pre, m, n) ||
		       PredicateValueCheck<PredPost>::apply(p.post, m, n);
	}

	static constexpr bool apply(...) { return true; }
};

template <>
struct PredicateValueCheck<TRUE> {
	using Pred = TRUE;

	template <class Map, class Node>
	static constexpr bool apply(Pred const&, Map const&, Node const&)
	{
		return true;
	}
};

template <>
struct PredicateValueCheck<FALSE> {
	using Pred = FALSE;

	template <class Map, class Node>
	static constexpr bool apply(Pred const&, Map const&, Node const&)
	{
		return false;
	}
};

//
// Predicate inner check
//

template <class Predicate>
struct PredicateInnerCheck {
	// REVIEW: Make better
	static_assert(static_assert_check_v<false, Predicate>,
	              "Not implemented for this Predicate.");
};

template <class... Preds>
struct PredicateInnerCheck<std::tuple<Preds...>> {
	using Pred = std::tuple<Preds...>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		return std::apply(
		    [&m, &n](auto const&... preds) {
			    return (
			        (PredicateInnerCheck<std::decay_t<decltype(preds)>>::apply(preds, m, n)) &&
			        ...);
		    },
		    p);
	}
};

template <class PredLeft, class PredRight>
struct PredicateInnerCheck<OR<PredLeft, PredRight>> {
	using Pred = OR<PredLeft, PredRight>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		return PredicateInnerCheck<PredLeft>::apply(p.left, m, n) ||
		       PredicateInnerCheck<PredRight>::apply(p.right, m, n);
	}
};

template <class PredPre, class PredPost>
struct PredicateInnerCheck<THEN<PredPre, PredPost>> {
	using Pred = THEN<PredPre, PredPost>;

	template <class Map, class Node>
	static constexpr auto apply(Pred const& p, Map const& m, Node const& n)
	    -> decltype(PredicateInnerCheck<PredPre>::apply(p.pre, m, n), bool())
	{
		return !PredicateInnerCheck<PredPre>::apply(p.pre, m, n) ||
		       PredicateInnerCheck<PredPost>::apply(p.post, m, n);
	}

	static constexpr bool apply(...) { return true; }
};

template <>
struct PredicateInnerCheck<TRUE> {
	using Pred = TRUE;

	template <class Map, class Node>
	static constexpr bool apply(Pred const&, Map const&, Node const&)
	{
		return true;
	}
};

template <>
struct PredicateInnerCheck<FALSE> {
	using Pred = FALSE;

	template <class Map, class Node>
	static constexpr bool apply(Pred const&, Map const&, Node const&)
	{
		return false;
	}
};

//
// Something
//

// template <typename T, typename Tuple>
// struct has_type;

template <typename T, typename T2>
struct has_type : std::false_type {
};

template <typename T>
struct has_type<T, std::tuple<>> : std::false_type {
};

template <typename T, typename L, typename R>
struct has_type<T, OR<L, R>> : std::false_type {
};

template <typename T, typename L, typename R>
struct has_type<T, THEN<L, R>> : std::false_type {
};

template <typename T, typename U, typename... Ts>
struct has_type<T, std::tuple<U, Ts...>> : has_type<T, std::tuple<Ts...>> {
};

template <typename T, typename... Ts>
struct has_type<T, std::tuple<T, Ts...>> : std::true_type {
};

template <typename T>
struct has_type<T, T> : std::true_type {
};

template <typename Predicate, typename Predicates>
using contains_predicate = typename has_type<Predicate, Predicates>::type;

}  // namespace ufo::map::predicate

#endif  // UFO_MAP_PREDICATE_PREDICATES_H