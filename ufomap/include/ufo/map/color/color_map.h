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

#ifndef UFO_MAP_COLOR_MAP_H
#define UFO_MAP_COLOR_MAP_H

// UFO
#include <ufo/map/color/color_predicate.h>
#include <ufo/map/io.h>
#include <ufo/map/node.h>
#include <ufo/map/point.h>
#include <ufo/map/types.h>

// STL
#include <algorithm>
#include <array>
#include <deque>
#include <iostream>

namespace ufo::map
{
template <class Derived, std::size_t N>
class ColorMapBase
{
 public:
	//
	// Get color
	//

	[[nodiscard]] Color color(Node node) const
	{
		auto [index, offset] = derived().indexAndOffset(node);
		return Color(red_[index][offset], green_[index][offset], blue_[index][offset]);
	}

	[[nodiscard]] Color color(Code code) const
	{
		auto [index, offset] = derived().indexAndOffset(code);
		return Color(red_[index][offset], green_[index][offset], blue_[index][offset]);
	}

	[[nodiscard]] Color color(Key key) const { return color(derived().toCode(key)); }

	[[nodiscard]] Color color(Point coord, depth_t depth = 0) const
	{
		return color(derived().toCode(coord, depth));
	}

	[[nodiscard]] Color color(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return color(derived().toCode(x, y, z, depth));
	}

	//
	// Set color
	//

	Node setColor(Node node, Color color, bool propagate = true)
	{
		return setColor(node, color.red, color.green, color.blue, propagate);
	}

	Node setColor(Node node, color_t red, color_t green, color_t blue,
	              bool propagate = true)
	{
		return derived().apply(
		    node,
		    [&red_, &green_, &blue_, red, green, blue](index_t index, index_t offset) {
			    red_[index][offset] = red;
			    green_[index][offset] = green;
			    blue_[index][offset] = blue;
		    },
		    [&red_, &green_, &blue_, red, green, blue](index_t index) {
			    red_[index].fill(red);
			    green_[index].fill(green);
			    blue_[index].fill(blue);
		    },
		    propagate);
	}

	Node setColor(Code code, Color color, bool propagate = true)
	{
		return setColor(code, color.red, color.green, color.blue, propagate);
	}

	Node setColor(Code code, color_t red, color_t green, color_t blue,
	              bool propagate = true)
	{
		return derived().apply(
		    code,
		    [&red_, &green_, &blue_, red, green, blue](index_t index, index_t offset) {
			    red_[index][offset] = red;
			    green_[index][offset] = green;
			    blue_[index][offset] = blue;
		    },
		    [&red_, &green_, &blue_, red, green, blue](index_t index) {
			    red_[index].fill(red);
			    green_[index].fill(green);
			    blue_[index].fill(blue);
		    },
		    propagate);
	}

	Node setColor(Key key, Color color, bool propagate = true)
	{
		return setColor(derived().toCode(key), color, propagate);
	}

	Node setColor(Key key, color_t red, color_t green, color_t blue, bool propagate = true)
	{
		return setColor(derived().toCode(key), red, green, blue, propagate);
	}

	Node setColor(Point coord, Color color, bool propagate = true, depth_t depth = 0)
	{
		return setColor(derived().toCode(coord, depth), color, propagate);
	}

	Node setColor(Point coord, color_t red, color_t green, color_t blue,
	              bool propagate = true, depth_t depth = 0)
	{
		return setColor(derived().toCode(coord, depth), red, green, blue, propagate);
	}

	Node setColor(coord_t x, coord_t y, coord_t z, Color color, bool propagate = true,
	              depth_t depth = 0)
	{
		return setColor(derived().toCode(x, y, z, depth), color, propagate);
	}

	Node setColor(coord_t x, coord_t y, coord_t z, color_t red, color_t green, color_t blue,
	              bool propagate = true, depth_t depth = 0)
	{
		return setColor(derived().toCode(x, y, z, depth), red, green, blue, propagate);
	}

	//
	// Update color
	//

	template <class UnaryFunction>
	Node updateColor(Node node, UnaryFunction f, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [&red_, &green_, &blue_, f](index_t index, index_t offset) {
			    auto c =
			        f(Color(red_[index][offset], green_[index][offset], blue_[index][offset]));
			    red_[index][offset] = c.red;
			    green_[index][offset] = c.green;
			    blue_[index][offset] = c.blue;
		    },
		    [&red_, &green_, &blue_, f](index_t index) {
			    for (index_t i = 0; N != i; ++i) {
				    f(Color(red_[index][i], green_[index][i], blue_[index][i]));
				    red_[index][i] = c.red;
				    green_[index][i] = c.green;
				    blue_[index][i] = c.blue;
			    }
		    },
		    propagate);
	}

	template <class UnaryFunction>
	Node updateColor(Code code, UnaryFunction f, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [&red_, &green_, &blue_, f](index_t index, index_t offset) {
			    auto c =
			        f(Color(red_[index][offset], green_[index][offset], blue_[index][offset]));
			    red_[index][offset] = c.red;
			    green_[index][offset] = c.green;
			    blue_[index][offset] = c.blue;
		    },
		    [&red_, &green_, &blue_, f](index_t index) {
			    for (index_t i = 0; N != i; ++i) {
				    f(Color(red_[index][i], green_[index][i], blue_[index][i]));
				    red_[index][i] = c.red;
				    green_[index][i] = c.green;
				    blue_[index][i] = c.blue;
			    }
		    },
		    propagate);
	}

	template <class UnaryFunction>
	Node updateColor(Key key, UnaryFunction f, bool propagate = true)
	{
		return updateColor(derived().toCode(key), f, propagate);
	}

	template <class UnaryFunction>
	Node updateColor(Point coord, UnaryFunction f, bool propagate = true, depth_t depth = 0)
	{
		return updateColor(derived().toCode(coord, depth), f, propagate);
	}

	template <class UnaryFunction>
	Node updateColor(coord_t x, coord_t y, coord_t z, UnaryFunction f,
	                 bool propagate = true, depth_t depth = 0)
	{
		return updateColor(derived().toCode(x, y, z, depth), f, propagate);
	}

	//
	// Has color
	//

	[[nodiscard]] bool hasColor(Node node) const
	{
		auto [index, offset] = derived().indexAndOffset(node);
		return 0 != red_[index][offset] || 0 != green_[index][offset] ||
		       0 != blue_[index][offset];
	}

	[[nodiscard]] bool hasColor(Code code) const
	{
		auto [index, offset] = derived().indexAndOffset(code);
		return 0 != red_[index][offset] || 0 != green_[index][offset] ||
		       0 != blue_[index][offset];
	}

	[[nodiscard]] bool hasColor(Key key) const { return hasColor(derived().toCode(key)); }

	[[nodiscard]] bool hasColor(Point coord, depth_t depth = 0) const
	{
		return hasColor(derived().toCode(coord, depth));
	}

	[[nodiscard]] bool hasColor(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return hasColor(derived().toCode(x, y, z, depth));
	}

	//
	// Clear color
	//

	void clearColor(Node node, bool propagate = true)
	{
		setColor(node, 0, 0, 0, propagate);
	}

	void clearColor(Code code, bool propagate = true)
	{
		setColor(code, 0, 0, 0, propagate);
	}

	void clearColor(Key key, bool propagate = true)
	{
		clearColor(derived().toCode(key), propagate);
	}

	void clearColor(Point coord, bool propagate = true, depth_t depth = 0)
	{
		clearColor(derived().toCode(coord, depth), propagate);
	}

	void clearColor(coord_t x, coord_t y, coord_t z, bool propagate = true,
	                depth_t depth = 0)
	{
		clearColor(derived().toCode(x, y, z, depth), propagate);
	}

 protected:
	//
	// Constructors
	//

	ColorMapBase() = default;

	ColorMapBase(ColorMapBase const&) = default;

	ColorMapBase(ColorMapBase&&) = default;

	template <class Derived2>
	ColorMapBase(ColorMapBase<Derived2, N> const& other)
	    : red_(other.red_), green_(other.green_), blue_(other.blue_)
	{
	}

	//
	// Assignment operator
	//

	ColorMapBase& operator=(ColorMapBase const&) = default;

	ColorMapBase& operator=(ColorMapBase&&) = default;

	template <class Derived2>
	ColorMapBase& operator=(ColorMapBase<Derived2, N> const& rhs)
	{
		red_ = rhs.red_;
		green_ = rhs.green_;
		blue_ = rhs.blue_;
		return *this;
	}

	//
	// Swap
	//

	void swap(ColorMapBase& other) noexcept
	{
		std::swap(red_, other.red_);
		std::swap(green_, other.green_);
		std::swap(blue_, other.blue_);
	}

	//
	// Derived
	//

	[[nodiscard]] constexpr Derived& derived() { return *static_cast<Derived*>(this); }

	[[nodiscard]] constexpr Derived const& derived() const
	{
		return *static_cast<Derived const*>(this);
	}

	//
	// Initilize root
	//

	void initRoot()
	{
		auto index = derived().rootIndex();
		auto offset = derived().rootOffset();
		red_[index][offset] = 0;
		green_[index][offset] = 0;
		blue_[index][offset] = 0;
	}

	//
	// Resize
	//

	void resize(std::size_t count)
	{
		red_.resize(count);
		green_.resize(count);
		blue_.resize(count);
	}

	//
	// Fill
	//

	void fill(index_t index, index_t parent_index, index_t parent_offset)
	{
		red_[index].fill(red_[parent_index][parent_offset]);
		green_[index].fill(green_[parent_index][parent_offset]);
		blue_[index].fill(blue_[parent_index][parent_offset]);
	}

	//
	// Clear
	//

	void clear(index_t index) {}

	//
	// Update node
	//

	void updateNode(index_t index, index_t offset, index_t children_index)
	{
		double red = 0;
		double green = 0;
		double blue = 0;

		std::size_t num = 0;
		for (std::size_t i = 0; N != i; ++i) {
			if (0 != red_[children_index][i] || 0 != green_[children_index][i] ||
			    0 != blue_[children_index][i]) {
				++num;
				red += red_[children_index][i];
				green += green_[children_index][i];
				blue += blue_[children_index][i];
			}
		}

		if (0 == num) {
			red_[index][offset] = 0;
			green_[index][offset] = 0;
			blue_[index][offset] = 0;
		} else {
			red_[index][offset] = red / num;
			green_[index][offset] = green / num;
			blue_[index][offset] = blue / num;
		}
	}

	//
	// Is collapsible
	//

	[[nodiscard]] bool isCollapsible(index_t index) const
	{
		return std::all_of(std::begin(red_[index]) + 1, std::end(red_[index]),
		                   [r = red_[index].front()](auto c) { return c == r; }) &&
		       std::all_of(std::begin(green_[index]) + 1, std::end(green_[index]),
		                   [g = green_[index].front()](auto c) { return c == g; }) &&
		       std::all_of(std::begin(blue_[index]) + 1, std::end(blue_[index]),
		                   [b = blue_[index].front()](auto c) { return c == b; });
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr MapType mapType() noexcept { return MapType::COLOR; }

	[[nodiscard]] static constexpr bool canReadData(MapType mt) noexcept
	{
		return mapType() == mt;
	}

	template <class InputIt>
	std::size_t serializedSize(InputIt first, InputIt last) const
	{
		return std::distance(first, last) * 3 * N * sizeof(color_t);
	}

	template <class OutputIt>
	void readNodes(ReadBuffer& in, OutputIt first, OutputIt last)
	{
		for (; first != last; ++first) {
			if (first->offsets.all()) {
				in.read(red_[first->index].data(), N * sizeof(color_t));
				in.read(green_[first->index].data(), N * sizeof(color_t));
				in.read(blue_[first->index].data(), N * sizeof(color_t));
			} else {
				std::array<color_t, N> red;
				std::array<color_t, N> green;
				std::array<color_t, N> blue;
				in.read(red.data(), N * sizeof(color_t));
				in.read(green.data(), N * sizeof(color_t));
				in.read(blue.data(), N * sizeof(color_t));
				for (index_t i = 0; N != i; ++i) {
					if (first->offsets[i]) {
						red_[first->index][i] = red[i];
						green_[first->index][i] = green[i];
						blue_[first->index][i] = blue[i];
					}
				}
			}
		}
	}

	template <class InputIt>
	void writeNodes(WriteBuffer& out, InputIt first, InputIt last) const
	{
		out.reserve(out.size() + serializedSize(first, last));
		for (; first != last; ++first) {
			out.write(red_[*first].data(), N * sizeof(color_t));
			out.write(green_[*first].data(), N * sizeof(color_t));
			out.write(blue_[*first].data(), N * sizeof(color_t));
		}
	}

 protected:
	// Data
	std::deque<std::array<color_t, N>> red_;
	std::deque<std::array<color_t, N>> green_;
	std::deque<std::array<color_t, N>> blue_;

	template <class Derived2, std::size_t N2>
	friend class ColorMapBase;
};
}  // namespace ufo::map

#endif  // UFO_MAP_COLOR_MAP_H