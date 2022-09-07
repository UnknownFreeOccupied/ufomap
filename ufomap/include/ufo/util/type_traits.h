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

#ifndef UFO_UTIL_TYPE_TRAITS
#define UFO_UTIL_TYPE_TRAITS

// STL
#include <type_traits>

namespace ufo::util
{
template <std::size_t ArgNum, typename Arg, typename... Rest>
struct argument_helper_2 : argument_helper_2<ArgNum - 1, Rest...> {
};

template <typename Arg, typename... Rest>
struct argument_helper_2<0, Arg, Rest...> {
	using type = Arg;
};

template <std::size_t ArgNum, typename... Args>
using argument_helper_2_t = typename argument_helper_2<ArgNum, Args...>::type;

// Specialization for 'auto' argument type
template <std::size_t ArgNum, typename... T>
typename std::enable_if_t<ArgNum <= sizeof...(T)> argument_helper(T...);

// Specialization for function pointer
template <std::size_t ArgNum, typename Ret, typename... Args>
argument_helper_2_t<ArgNum, Args...> argument_helper(Ret (*)(Args...));

// Specialization for functor and lambda
template <std::size_t ArgNum, typename Ret, typename F, typename... Args>
argument_helper_2_t<ArgNum, Args...> argument_helper(Ret (F::*)(Args...));

// Specialization for functor and lambda
template <std::size_t ArgNum, typename Ret, typename F, typename... Args>
argument_helper_2_t<ArgNum, Args...> argument_helper(Ret (F::*)(Args...) const);

// Specialization for functor and lambda
template <std::size_t ArgNum, typename F>
decltype(argument_helper<ArgNum>(&F::operator())) argument_helper(F);

/*!
 * @brief Get the argument type ArgNum argument of function F.
 *
 */
template <typename F, std::size_t ArgNum>
using argument = decltype(argument_helper<ArgNum>(std::declval<F>()));

//
// Is pair
//

template <class>
struct is_pair : std::false_type {
};

template <class T, class U>
struct is_pair<std::pair<T, U>> : std::true_type {
};

template <class T>
inline constexpr bool is_pair_v = is_pair<T>::value;

//
// Base of template
//

template <template <typename...> class Base, typename Derived>
struct is_base_of_template_impl {
	template <typename... Ts>
	static constexpr std::true_type test(Base<Ts...> const*);
	static constexpr std::false_type test(...);
	using type = decltype(test(std::declval<Derived*>()));
};

template <template <typename...> class Base, typename Derived>
using is_base_of_template = typename is_base_of_template_impl<Base, Derived>::type;

template <template <typename...> class Base, typename Derived>
inline constexpr bool is_base_of_template_v = is_base_of_template<Base, Derived>::value;
}  // namespace ufo::util

#endif  // UFO_UTIL_TYPE_TRAITS