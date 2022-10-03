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

#ifndef UFO_MATH_EXTRA_H
#define UFO_MATH_EXTRA_H

// STL
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <type_traits>

namespace ufo::math
{
//
// Floating point
//

template <typename F>
[[nodiscard]] constexpr std::enable_if_t<std::is_floating_point_v<std::decay_t<F>>, F>
logit(F probability) noexcept
{
	return std::log(probability / (F(1) - probability));
}

template <typename F>
[[nodiscard]] constexpr std::enable_if_t<std::is_floating_point_v<std::decay_t<F>>, F>
logit(F probability, F min_logit, F max_logit) noexcept
{
	return std::clamp(logit(probability), min_logit, max_logit);
}

template <typename F>
[[nodiscard]] constexpr std::enable_if_t<std::is_floating_point_v<std::decay_t<F>>, F>
probability(F logit) noexcept
{
	return F(1) / (F(1) + std::exp(-logit));
}

//
// Signed
//

// template <typename S, typename F,
//           typename = std::enable_if_t<std::is_signed_v<S> && std::is_integral_v<S> &&
//                                       std::is_floating_point_v<F>>>
// [[nodiscard]] constexpr F convertLogit(S logit, F min_logit, F max_logit) noexcept
// {
// 	// TODO: Implement
// 	return ((logit * (max_logit - min_logit)) / std::numeric_limits<S>::max()) +
// min_logit;
// }

// template <typename S, typename F,
//           typename = std::enable_if_t<std::is_signed_v<S> && std::is_integral_v<S> &&
//                                       std::is_floating_point_v<F>>>
// [[nodiscard]] constexpr S convertLogit(F logit, F min_logit, F max_logit) noexcept
// {
// 	// TODO: Implement
// 	return std::llround((logit - min_logit) * std::numeric_limits<S>::max() /
// 	                    (max_logit - min_logit));
// }

// template <typename S, typename F,
//           typename = std::enable_if_t<std::is_signed_v<S> && std::is_integral_v<S> &&
//                                       std::is_floating_point_v<F>>>
// [[nodiscard]] constexpr S logit(F probability, F min_logit, F max_logit) noexcept
// {
// 	// TODO: Implement
// 	return convertLogit<S>(logit(probability, min_logit, max_logit), min_logit,
// max_logit);
// }

// template <typename S, typename F,
//           typename = std::enable_if_t<std::is_signed_v<S> && std::is_integral_v<S> &&
//                                       std::is_floating_point_v<F>>>
// [[nodiscard]] constexpr F probability(S logit, F min_logit, F max_logit) noexcept
// {
// 	// TODO: Implement
// 	return probability(convertLogit(logit, min_logit, max_logit));
// }

// // Call to get uint prob_hit / prob_miss
// template <typename S, typename F,
//           typename = std::enable_if_t<std::is_signed_v<S> && std::is_integral_v<S> &&
//                                       std::is_floating_point_v<F>>>
// [[nodiscard]] constexpr S logitChangeValue(F probability, F min_logit,
//                                            F max_logit) noexcept
// {
// 	// TODO: Implement
// 	return F(0.5) > probability ? logit<S>(F(0.5), min_logit, max_logit) -
// 	                                  logit<S>(probability, min_logit, max_logit)
// 	                            : logit<S>(probability, min_logit, max_logit) -
// 	                                  logit<S>(F(0.5), min_logit, max_logit);
// }

// template <typename S,
//           typename = std::enable_if_t<std::is_signed_v<S> && std::is_integral_v<S>>>
// [[nodiscard]] constexpr S increaseLogit(S cur, S inc) noexcept
// {
// 	// TODO: Implement
// 	return std::numeric_limits<S>::max() - cur > inc ? cur + inc
// 	                                                 : std::numeric_limits<S>::max();
// }

// template <typename S,
//           typename = std::enable_if_t<std::is_signed_v<S> && std::is_integral_v<S>>>
// [[nodiscard]] constexpr S decreaseLogit(S cur, S dec) noexcept
// {
// 	// TODO: Implement
// 	return cur - std::min(cur, dec);
// }

//
// Unsigned
//

template <
    typename U, typename F,
    typename = std::enable_if_t<std::is_unsigned_v<U> && std::is_floating_point_v<F>>>
[[nodiscard]] constexpr F convertLogit(U logit, F min_logit, F max_logit) noexcept
{
	return ((logit * (max_logit - min_logit)) / std::numeric_limits<U>::max()) + min_logit;
}

template <
    typename U, typename F,
    typename = std::enable_if_t<std::is_unsigned_v<U> && std::is_floating_point_v<F>>>
[[nodiscard]] constexpr U convertLogit(F logit, F min_logit, F max_logit) noexcept
{
	return std::llround((logit - min_logit) * std::numeric_limits<U>::max() /
	                    (max_logit - min_logit));
}

template <
    typename U, typename F,
    typename = std::enable_if_t<std::is_unsigned_v<U> && std::is_floating_point_v<F>>>
[[nodiscard]] constexpr U logit(F probability, F min_logit, F max_logit) noexcept
{
	return convertLogit<U>(logit(probability, min_logit, max_logit), min_logit, max_logit);
}

template <
    typename U, typename F,
    typename = std::enable_if_t<std::is_unsigned_v<U> && std::is_floating_point_v<F>>>
[[nodiscard]] constexpr F probability(U logit, F min_logit, F max_logit) noexcept
{
	return probability(convertLogit(logit, min_logit, max_logit));
}

// Call to get uint prob_hit / prob_miss
template <
    typename U, typename F,
    typename = std::enable_if_t<std::is_unsigned_v<U> && std::is_floating_point_v<F>>>
[[nodiscard]] constexpr U logitChangeValue(F probability, F min_logit,
                                           F max_logit) noexcept
{
	return F(0.5) > probability ? logit<U>(F(0.5), min_logit, max_logit) -
	                                  logit<U>(probability, min_logit, max_logit)
	                            : logit<U>(probability, min_logit, max_logit) -
	                                  logit<U>(F(0.5), min_logit, max_logit);
}

template <typename U, typename = std::enable_if_t<std::is_unsigned_v<U>>>
[[nodiscard]] constexpr U increaseLogit(U cur, U inc) noexcept
{
	return std::numeric_limits<U>::max() - cur > inc ? cur + inc
	                                                 : std::numeric_limits<U>::max();
}

template <typename U, typename = std::enable_if_t<std::is_unsigned_v<U>>>
[[nodiscard]] constexpr U decreaseLogit(U cur, U dec) noexcept
{
	return cur - std::min(cur, dec);
}
}  // namespace ufo::math

#endif  // UFO_MATH_EXTRA_H