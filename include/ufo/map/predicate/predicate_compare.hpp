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

#ifndef UFO_MAP_PREDICATE_PREDICATE_COMPARE_HPP
#define UFO_MAP_PREDICATE_PREDICATE_COMPARE_HPP

// STL
#include <string>

namespace ufo::pred
{
enum class PredicateCompare {
	EQUAL,
	NOT_EQUAL,
	LESS_EQUAL,
	GREATER_EQUAL,
	LESS,
	GREATER
};

inline std::string enumToString(PredicateCompare PC)
{
	switch (PC) {
		case PredicateCompare::EQUAL: return "=";
		case PredicateCompare::NOT_EQUAL: return "~=";
		case PredicateCompare::LESS_EQUAL: return "<=";
		case PredicateCompare::GREATER_EQUAL: return ">=";
		case PredicateCompare::LESS: return "<";
		case PredicateCompare::GREATER: return ">";
	}
}

inline PredicateCompare switchSide(PredicateCompare PC)
{
	switch (PC) {
		case PredicateCompare::EQUAL: return PredicateCompare::EQUAL;
		case PredicateCompare::NOT_EQUAL: return PredicateCompare::NOT_EQUAL;
		case PredicateCompare::LESS_EQUAL: return PredicateCompare::GREATER_EQUAL;
		case PredicateCompare::GREATER_EQUAL: return PredicateCompare::LESS_EQUAL;
		case PredicateCompare::LESS: return PredicateCompare::GREATER;
		case PredicateCompare::GREATER: return PredicateCompare::LESS;
	}
}

template <template <PredicateCompare> class Pred, PredicateCompare PC, typename T>
Pred<PredicateCompare::EQUAL> operator==(Pred<PC> const& o, T&& t)
{
	return {std::forward<T>(t)};
}

template <template <PredicateCompare> class Pred, PredicateCompare PC, typename T>
Pred<PredicateCompare::NOT_EQUAL> operator!=(Pred<PC> const& o, T&& t)
{
	return {std::forward<T>(t)};
}

template <template <PredicateCompare> class Pred, PredicateCompare PC, typename T>
Pred<PredicateCompare::LESS_EQUAL> operator<=(Pred<PC> const& o, T&& t)
{
	return {std::forward<T>(t)};
}

template <template <PredicateCompare> class Pred, PredicateCompare PC, typename T>
Pred<PredicateCompare::GREATER_EQUAL> operator>=(Pred<PC> const& o, T&& t)
{
	return {std::forward<T>(t)};
}

template <template <PredicateCompare> class Pred, PredicateCompare PC, typename T>
Pred<PredicateCompare::LESS> operator<(Pred<PC> const& o, T&& t)
{
	return {std::forward<T>(t)};
}

template <template <PredicateCompare> class Pred, PredicateCompare PC, typename T>
Pred<PredicateCompare::GREATER> operator>(Pred<PC> const& o, T&& t)
{
	return {std::forward<T>(t)};
}

template <typename T, template <PredicateCompare> class Pred, PredicateCompare PC>
Pred<PredicateCompare::EQUAL> operator==(T&& t, Pred<PC> const& o)
{
	return {std::forward<T>(t)};
}

template <typename T, template <PredicateCompare> class Pred, PredicateCompare PC>
Pred<PredicateCompare::NOT_EQUAL> operator!=(T&& t, Pred<PC> const& o)
{
	return {std::forward<T>(t)};
}

template <typename T, template <PredicateCompare> class Pred, PredicateCompare PC>
Pred<PredicateCompare::GREATER_EQUAL> operator<=(T&& t, Pred<PC> const& o)
{
	return {std::forward<T>(t)};
}

template <typename T, template <PredicateCompare> class Pred, PredicateCompare PC>
Pred<PredicateCompare::LESS_EQUAL> operator>=(T&& t, Pred<PC> const& o)
{
	return {std::forward<T>(t)};
}

template <typename T, template <PredicateCompare> class Pred, PredicateCompare PC>
Pred<PredicateCompare::GREATER> operator<(T&& t, Pred<PC> const& o)
{
	return {std::forward<T>(t)};
}

template <typename T, template <PredicateCompare> class Pred, PredicateCompare PC>
Pred<PredicateCompare::LESS> operator>(T&& t, Pred<PC> const& o)
{
	return {std::forward<T>(t)};
}
}  // namespace ufo::pred

#endif  // UFO_MAP_PREDICATE_PREDICATE_COMPARE_HPP