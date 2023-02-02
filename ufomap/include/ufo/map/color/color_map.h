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

	[[nodiscard]] Color colorUnsafe(Index index) const
	{
		return color_[index.index][index.offset];
	}

	[[nodiscard]] Color color(Node node) const
	{
		return colorUnsafe(derived().index(node));
	}

	[[nodiscard]] Color color(Code code) const
	{
		return colorUnsafe(derived().index(code));
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

	void setColorUnsafe(Index index, Color color)
	{
		// TODO: Implement
	}

	void setColorUnsafe(Index index, color_t red, color_t green, color_t blue)
	{
		setColorUnsafe(index, Color(red, green, blue));
	}

	void setColorUnsafe(IndexFam index, Color color)
	{
		// TODO: Implement
	}

	void setColorUnsafe(IndexFam index, color_t red, color_t green, color_t blue)
	{
		setColorUnsafe(index, Color(red, green, blue));
	}

	Node setColor(Node node, Color color, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [&color_, color](Index index) { color_[index.index][index.offset] = color; },
		    [&color_, color](index_t index) { color_[index].fill(color); }, propagate);
	}

	Node setColor(Node node, color_t red, color_t green, color_t blue,
	              bool propagate = true)
	{
		return setColor(node, Color(red, green, blue), propagate);
	}

	Node setColor(Code code, Color color, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [&color_, color](Index index) { color_[index.index][index.offset] = color; },
		    [&color_, color](index_t index) { color_[index].fill(color); }, propagate);
	}

	Node setColor(Code code, color_t red, color_t green, color_t blue,
	              bool propagate = true)
	{
		return setColor(code, Color(red, green, blue), propagate);
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
	void updateColorUnsafe(Index index, UnaryFunction f)
	{
		// TODO: Implement
	}

	template <class UnaryFunction>
	void updateColorUnsafe(IndexFam index, UnaryFunction f)
	{
		// TODO: Implement
	}

	template <class UnaryFunction>
	Node updateColor(Node node, UnaryFunction f, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [&color_, f](Index index) {
			    color_[index.index][index.offset] = f(color_[index.index][index.offset]);
		    },
		    [&color_, f](index_t index) {
			    for (offset_t i = 0; N != i; ++i) {
				    color_[index][i] = f(color_[index][i]);
			    }
		    },
		    propagate);
	}

	template <class UnaryFunction>
	Node updateColor(Code code, UnaryFunction f, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [&color_, f](Index index) {
			    color_[index.index][index.offset] = f(color_[index.index][index.offset]);
		    },
		    [&color_, f](index_t index) {
			    for (offset_t i = 0; N != i; ++i) {
				    color_[index][i] = f(color_[index][i]);
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

	[[nodiscard]] bool hasColorUnsafe(Index index) const
	{
		return color_[index.index][index.offset].isSet();
	}

	[[nodiscard]] bool hasColor(Node node) const
	{
		return hasColorUnsafe(derived().index(node));
	}

	[[nodiscard]] bool hasColor(Code code) const
	{
		return hasColorUnsafe(derived().index(code));
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

	void clearColorUnsafe(Index index) { setColor(index, Color()); }

	void clearColorUnsafe(IndexFam index) { setColor(index, Color()); }

	void clearColor(Node node, bool propagate = true)
	{
		setColor(node, Color(), propagate);
	}

	void clearColor(Code code, bool propagate = true)
	{
		setColor(code, Color(), propagate);
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
	ColorMapBase(ColorMapBase<Derived2, N> const& other) : color_(other.color_)
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
		color_ = rhs.color_;
		return *this;
	}

	//
	// Swap
	//

	void swap(ColorMapBase& other) noexcept { std::swap(color_, other.color_); }

	//
	// Derived
	//

	[[nodiscard]] constexpr Derived& derived() { return *static_cast<Derived*>(this); }

	[[nodiscard]] constexpr Derived const& derived() const
	{
		return *static_cast<Derived const*>(this);
	}

	//
	// Allocate node block
	//

	void allocateNodeBlock() { color_.emplace_back(); }

	//
	// Initilize root
	//

	void initRoot()
	{
		auto index = derived().rootIndex();
		color_[index.index][index.offset].clear();
	}

	//
	// Resize
	//

	void resize(std::size_t count) { color_.resize(count); }

	//
	// Fill
	//

	void fill(index_t index, Index parent) { color_[index].fill(colorUnsafe(parent)); }

	//
	// Clear
	//

	void clear(index_t index) {}

	//
	// Update node
	//

	void updateNode(Index index, index_t children_index)
	{
		std::uint16_t red{}, green{}, blue{}, num{};
		for (std::size_t i{}; N != i; ++i) {
			red += color_[children_index][i].red;
			green += color_[children_index][i].green;
			blue += color_[children_index][i].blue;
			num += color_[children_index][i].isSet();
		}

		num = num ? num : 1;
		color_[index.index][index.offset].red = red / num;
		color_[index.index][index.offset].green = green / num;
		color_[index.index][index.offset].blue = blue / num;
	}

	//
	// Is collapsible
	//

	[[nodiscard]] bool isCollapsible(index_t index) const
	{
		return std::all_of(std::cbegin(color_[index]) + 1, std::cend(color_[index]),
		                   [c = color_[index].front()](auto e) { return c == e; });
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
		return std::distance(first, last) * N * sizeof(Color);
	}

	template <class OutputIt>
	void readNodes(ReadBuffer& in, OutputIt first, OutputIt last)
	{
		for (; first != last; ++first) {
			if (first->offsets.all()) {
				in.read(color_[first->index].data(), N * sizeof(Color));
			} else {
				std::array<Color, N> color;
				in.read(color.data(), N * sizeof(Color));
				for (index_t i = 0; N != i; ++i) {
					if (first->offsets[i]) {
						color_[first->index][i] = color[i];
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
			out.write(color_[*first].data(), N * sizeof(Color));
		}
	}

 protected:
	// Data
	std::deque<std::array<Color, N>> color_;

	template <class Derived2, std::size_t N2, bool ThreadSafe2>
	friend class ColorMapBase;
};
}  // namespace ufo::map

#endif  // UFO_MAP_COLOR_MAP_H