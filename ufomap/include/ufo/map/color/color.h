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

#ifndef UFO_MAP_COLOR_H
#define UFO_MAP_COLOR_H

// STL
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <iostream>
#include <type_traits>
#include <utility>
#include <vector>

namespace ufo::map
{
using RGBColorType = uint8_t;

/*!
 * @brief RGB color
 *
 */
struct RGBColor {
	RGBColorType red = 0;
	RGBColorType green = 0;
	RGBColorType blue = 0;

	constexpr RGBColor() = default;

	constexpr RGBColor(RGBColorType red, RGBColorType green, RGBColorType blue)
	    : red(red), green(green), blue(blue)
	{
	}

	constexpr bool operator==(RGBColor const& rhs) const
	{
		return rhs.red == red && rhs.green == green && rhs.blue == blue;
	}

	constexpr bool operator!=(RGBColor const& rhs) const { return !(*this == rhs); }

	[[nodiscard]] constexpr bool set() const { return 0 != red || 0 != green || 0 != blue; }

	constexpr void clear() { red = green = blue = 0; }

	template <class InputIt>
	static constexpr RGBColor average(InputIt first, InputIt last)
	{
		if constexpr (std::is_same_v<
		                  std::decay_t<typename std::iterator_traits<InputIt>::value_type>,
		                  RGBColor>) {
			unsigned r = 0;
			unsigned g = 0;
			unsigned b = 0;
			unsigned num = 0;
			for (; first != last; ++first) {
				if (!first->set()) {
					continue;
				}

				r += first->red;
				g += first->green;
				b += first->blue;
				++num;
			}
			return 0 == num ? RGBColor() : RGBColor(r / num, g / num, b / num);
		} else if constexpr (std::is_same_v<std::decay_t<typename std::iterator_traits<
		                                        InputIt>::value_type>,
		                                    std::pair<RGBColor, double>>) {
			unsigned r = 0;
			unsigned g = 0;
			unsigned b = 0;
			double weight = 0;
			for (; first != last; ++first) {
				if (!first->first.set()) {
					continue;
				}

				r += first->first.red;
				g += first->first.green;
				b += first->first.blue;
				weight += first->second;
			}
			return 0 == weight ? RGBColor() : RGBColor(r / weight, g / weight, b / weight);

		} else {
			static_assert(
			    std::is_same_v<std::decay_t<typename std::iterator_traits<InputIt>::value_type>,
			                   RGBColor> ||
			    std::is_same_v<std::decay_t<typename std::iterator_traits<InputIt>::value_type>,
			                   std::pair<RGBColor, double>>);
		}
	}

	static constexpr RGBColor average(std::initializer_list<RGBColor> colors)
	{
		return average(std::cbegin(colors), std::cend(colors));
	}

	static constexpr RGBColor average(
	    std::initializer_list<std::pair<RGBColor, double>> colors)
	{
		return average(std::cbegin(colors), std::cend(colors));
	}
};

}  // namespace ufo::map

#endif  // UFO_MAP_COLOR_H
