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
template <std::size_t N = 8>
struct ColorNode {
	// Data
	std::array<color_t, N> red;
	std::array<color_t, N> green;
	std::array<color_t, N> blue;

	//
	// Size
	//

	[[nodiscard]] static constexpr std::size_t colorSize() { return N; }

	//
	// Fill
	//

	void fill(ColorNode const parent, index_t const index)
	{
		if constexpr (1 == N) {
			setColor(parent.red[0], parent.green[0], parent.blue[0]);
		} else {
			setColor(parent.red[index], parent.green[index], parent.blue[index]);
		}
	}

	//
	// Is collapsible
	//

	[[nodiscard]] constexpr bool isCollapsible(ColorNode const parent,
	                                           index_t const index) const
	{
		if constexpr (1 == N) {
			return red[0] == parent.red[0] && green[0] == parent.green[0] &&
			       blue[0] == parent.blue[0];
		} else {
			return all_of(red, [r = parent.red[index]](auto const c) { return c == r; }) &&
			       all_of(green, [g = parent.green[index]](auto const c) { return c == g; }) &&
			       all_of(blue, [b = parent.blue[index]](auto const c) { return c == b; });
		}
	}

	//
	// Get color
	//

	[[nodiscard]] constexpr Color color(index_t const index) const
	{
		if constexpr (1 == N) {
			return Color(red[0], green[0], blue[0]);
		} else {
			return Color(red[index], green[index], blue[index]);
		}
	}

	//
	// Set color
	//

	void setColor(color_t const r, color_t const g, color_t const b)
	{
		red.fill(r);
		green.fill(g);
		blue.fill(b);
	}

	void setColor(Color const value) { setColor(value.red, value.green, value.blue); }

	void setColor(index_t const index, color_t const r, color_t const g, color_t const b)
	{
		if constexpr (1 == N) {
			setColor(r, g, b);
		} else {
			red[index] = r;
			green[index] = g;
			blue[index] = b;
		}
	}

	void setColor(index_t const index, Color const value)
	{
		setColor(index, value.red, value.green, value.blue);
	}

	//
	// Has color
	//

	[[nodiscard]] constexpr bool hasColor(index_t const index) const
	{
		if constexpr (1 == N) {
			return 0 != red[0] || 0 != green[0] || 0 != blue[0];
		} else {
			return 0 != red[index] || 0 != green[index] || 0 != blue[index];
		}
	}

	//
	// Clear color
	//

	void clearColor() { setColor(0, 0, 0); }

	void clearColor(index_t const index) { setColor(index, 0, 0, 0); }
};
}  // namespace ufo::map

#endif  // UFO_MAP_COLOR_NODE_H