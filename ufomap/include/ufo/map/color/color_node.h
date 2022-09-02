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

	bool operator==(ColorNode const rhs) const
	{
		return red == rhs.red && green == rhs.green && blue == rhs.blue;
	}

	bool operator!=(ColorNode const rhs) const { return !(*this == rhs); }

	constexpr RGBColor getColor(index_t const index) const
	{
		if constexpr (Single) {
			return RGBColor(red, green, blue);
		} else {
			return RGBColor(red[index], green[index], blue[index]);
		}
	}

	constexpr color_t getRed(index_t const index) const
	{
		if constexpr (Single) {
			return red;
		} else {
			return red[index];
		}
	}

	constexpr color_t getGreen(index_t const index) const
	{
		if constexpr (Single) {
			return green;
		} else {
			return green[index];
		}
	}

	constexpr color_t getBlue(index_t const index) const
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

	constexpr void setColor(std::size_t const index, RGBColor const value)
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

	constexpr void setRed(std::size_t const index, color_t const value)
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

	constexpr void setGreen(std::size_t const index, color_t const value)
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

	constexpr void setBlue(std::size_t const index, color_t const value)
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