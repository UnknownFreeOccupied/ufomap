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

#ifndef UFO_MATH_EXTRA_H
#define UFO_MATH_EXTRA_H

// STL
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace ufo::math
{
//
// Floating point
//

template <typename F, typename = std::enable_if_t<std::is_floating_point_v<F>>>
inline constexpr F logit(F probability)
{
	return std::log(probability / (F(1.0) - probability));
}

template <typename F, typename = std::enable_if_t<std::is_floating_point_v<F>>>
inline constexpr F logit(F probability, F min_logit, F max_logit)
{
	return std::clamp(logit(probability), min_logit, max_logit);
}

template <typename F, typename = std::enable_if_t<std::is_floating_point_v<F>>>
inline constexpr F probability(F logit)
{
	return F(1.0) / (F(1.0) + std::exp(-logit));
}

//
// Unsigned
//

template <
    typename U, typename F,
    typename = std::enable_if_t<std::is_unsigned_v<U> && std::is_floating_point_v<F>>>
inline constexpr F convertLogit(U logit, F min_logit, F max_logit)
{
	return ((logit * (max_logit - min_logit)) / std::numeric_limits<U>::max()) + min_logit;
}

template <
    typename U, typename F,
    typename = std::enable_if_t<std::is_unsigned_v<U> && std::is_floating_point_v<F>>>
inline constexpr U convertLogit(F logit, F min_logit, F max_logit)
{
	return std::llround((logit - min_logit) * std::numeric_limits<U>::max() /
	                    (max_logit - min_logit));
}

template <
    typename U, typename F,
    typename = std::enable_if_t<std::is_unsigned_v<U> && std::is_floating_point_v<F>>>
inline constexpr U logit(F probability, F min_logit, F max_logit)
{
	return convertLogit<U>(logit(probability, min_logit, max_logit), min_logit, max_logit);
}

template <
    typename U, typename F,
    typename = std::enable_if_t<std::is_unsigned_v<U> && std::is_floating_point_v<F>>>
inline constexpr F probability(U logit, F min_logit, F max_logit)
{
	return probability(convertLogit(logit, min_logit, max_logit));
}

// Call to get uint prob_hit / prob_miss
template <
    typename U, typename F,
    typename = std::enable_if_t<std::is_unsigned_v<U> && std::is_floating_point_v<F>>>
inline constexpr U logitChangeValue(F probability, F min_logit, F max_logit)
{
	probability = F(0.5) > probability ? probability : F(1.0) - probability;
	return logit<U>(F(0.5), min_logit, max_logit) -
	       logit<U>(probability, min_logit, max_logit);
}

template <typename U, typename = std::enable_if_t<std::is_unsigned_v<U>>>
constexpr U increaseLogit(U cur, U inc)
{
	return std::numeric_limits<U>::max() - cur > inc ? cur + inc
	                                                 : std::numeric_limits<U>::max();
}

template <typename U, typename = std::enable_if_t<std::is_unsigned_v<U>>>
constexpr U decreaseLogit(U cur, U dec)
{
	return cur - std::min(cur, dec);
}

//
// Old
//

// template <typename U, typename S,
//           typename = std::enable_if_t<std::is_unsigned_v<U> && std::is_signed_v<S>>>
// constexpr S convertLogit(U logit, U max_value, S min_logit, S max_logit)
// {
// 	return ((logit * (max_logit - min_logit)) / max_value) + min_logit;
// }

// template <typename T, typename C, typename = std::enable_if_t<std::is_unsigned_v<T>>,
//           typename = std::enable_if_t<std::is_floating_point_v<C>>>
// constexpr T convertLogit(C logit, T max_value, C min_logit, C max_logit)
// {
// 	return std::lround((logit - min_logit) * max_value / (max_logit - min_logit));
// }

// template <
//     typename U, typename F,
//     typename = std::enable_if_t<std::is_unsigned_v<U> && std::is_floating_point_v<F>>>
// constexpr U logit(F probability, U max_value, F min_logit, F max_logit)
// {
// 	return convertLogit(logit(probability, min_logit, max_logit), max_value, min_logit,
// 	                    max_logit);
// }

// template <
//     typename U, typename F,
//     typename = std::enable_if_t<std::is_unsigned_v<U> && std::is_floating_point_v<F>>>
// constexpr F probability(U logit, U max_value, F min_logit, F max_logit)
// {
// 	return probability(convertLogit(logit, max_value, min_logit, max_logit));
// }

// // Call to get uint prob_hit / prob_miss
// template <
//     typename U, typename F,
//     typename = std::enable_if_t<std::is_unsigned_v<U> && std::is_floating_point_v<F>>>
// constexpr U logitChangeValue(F probability, U max_value, F min_logit, F max_logit)
// {
// 	if (F(0.5) > probability) {
// 		return logit(F(0.5), max_value, min_logit, max_logit) -
// 		       logit(probability, max_value, min_logit, max_logit);
// 	} else {
// 		return logit(probability, max_value, min_logit, max_logit) -
// 		       logit(F(0.5), max_value, min_logit, max_logit);
// 	}
// }

// template <typename U, typename = std::enable_if_t<std::is_unsigned_v<U>>>
// constexpr U increaseLogit(U current_logit_value, U inc, U max_value)
// {
// 	return std::min(static_cast<uint64_t>(current_logit_value) + inc,
// 	                static_cast<uint64_t>(max_value));
// }

// template <typename U, typename = std::enable_if_t<std::is_unsigned_v<U>>>
// constexpr U decreaseLogit(U current_logit_value, U dec)
// {
// 	return std::max(static_cast<int64_t>(current_logit_value) - dec,
// 	                static_cast<int64_t>(0));
// }

}  // namespace ufo::math

#endif  // UFO_MATH_EXTRA_H