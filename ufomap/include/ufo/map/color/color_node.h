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

#ifndef UFO_MAP_COLOR_NODE_H
#define UFO_MAP_COLOR_NODE_H

// UFO
#include <ufo/algorithm/algorithm.h>
#include <ufo/map/color/color.h>

// STL
#include <array>

namespace ufo::map
{
template <bool Single = false>
struct ColorNode {
	// Data
	std::conditional_t<Single, color_t, std::array<color_t, 8>> red;
	std::conditional_t<Single, color_t, std::array<color_t, 8>> green;
	std::conditional_t<Single, color_t, std::array<color_t, 8>> blue;

	//
	// Fill
	//

	void fill(ColorNode const parent, index_t const index)
	{
		if constexpr (Single) {
			red = other.red;
			green = other.green;
			blue = other.blue;
		} else {
			red.fill(other.red[index]);
			green.fill(other.green[index]);
			blue.fill(other.blue[index]);
		}
	}

	//
	// Is collapsible
	//

	[[nodiscard]] constexpr bool isCollapsible(ColorNode const parent,
	                                           index_t const index) const
	{
		if constexpr (Single) {
			return red == parent.red && green == parent.green && blue == parent.blue;
		} else {
			return all_of(red, [r = parent.red[index]](auto const c) { return c == r; }) &&
			       all_of(green, [g = parent.green[index]](auto const c) { return c == g; }) &&
			       all_of(blue, [b = parent.blue[index]](auto const c) { return c == b; });
		}
	}

	constexpr RGBColor colorIndex(index_t const index) const
	{
		if constexpr (Single) {
			return RGBColor(red, green, blue);
		} else {
			return RGBColor(red[index], green[index], blue[index]);
		}
	}

	constexpr color_t redIndex(index_t const index) const
	{
		if constexpr (Single) {
			return red;
		} else {
			return red[index];
		}
	}

	constexpr color_t greenIndex(index_t const index) const
	{
		if constexpr (Single) {
			return green;
		} else {
			return green[index];
		}
	}

	constexpr color_t blueIndex(index_t const index) const
	{
		if constexpr (Single) {
			return blue;
		} else {
			return blue[index];
		}
	}

	void setColor(RGBColor const value)
	{
		if constexpr (Single) {
			red = value.red;
			green = value.green;
			blue = value.blue;
		} else {
			red.fill(value.red);
			green.fill(value.green);
			blue.fill(value.blue);
		}
	}

	constexpr void setColorIndex(index_t const index, RGBColor const value)
	{
		if constexpr (Single) {
			red = value.red;
			green = value.green;
			blue = value.blue;
		} else {
			red[index] = value.red;
			green[index] = value.green;
			blue[index] = value.blue;
		}
	}

	void setRed(color_t const value)
	{
		if constexpr (Single) {
			red = value;
		} else {
			red.fill(value);
		}
	}

	constexpr void setRedIndex(index_t const index, color_t const value)
	{
		if constexpr (Single) {
			red = value;
		} else {
			red[index] = value;
		}
	}

	void setGreen(color_t const value)
	{
		if constexpr (Single) {
			green = value;
		} else {
			green.fill(value);
		}
	}

	constexpr void setGreenIndex(index_t const index, color_t const value)
	{
		if constexpr (Single) {
			green = value;
		} else {
			green[index] = value;
		}
	}

	void setBlue(color_t const value)
	{
		if constexpr (Single) {
			blue = value;
		} else {
			blue.fill(value);
		}
	}

	constexpr void setBlueIndex(index_t const index, color_t const value)
	{
		if constexpr (Single) {
			blue = value;
		} else {
			blue[index] = value;
		}
	}
};
}  // namespace ufo::map

#endif  // UFO_MAP_COLOR_NODE_H