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
#include <ufo/map/color/color_node.h>
#include <ufo/map/color/color_predicate.h>
#include <ufo/map/io.h>
#include <ufo/map/node.h>
#include <ufo/map/point.h>
#include <ufo/map/types.h>

// STL
#include <iostream>

namespace ufo::map
{
template <class Derived>
class ColorMapBase
{
 public:
	//
	// Get color
	//

	[[nodiscard]] Color color(Node node) const
	{
		auto const& data = derived().leafNode(node);
		auto index = node.index();
		return Color(data.red[index], data.green[index], data.blue[index]);
	}

	[[nodiscard]] Color color(Code code) const
	{
		auto [node, depth] = derived().leafNodeAndDepth(code);
		auto index = code.index(depth);
		return Color(node.red[index], node.green[index], node.blue[index]);
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

	void setColor(Node node, Color color, bool propagate = true)
	{
		setColor(node, color.red, color.green, color.blue, propagate);
	}

	void setColor(Node node, color_t red, color_t green, color_t blue,
	              bool propagate = true)
	{
		derived().apply(
		    node,
		    [red, green, blue](auto& node, index_t index) {
			    node.red[index] = red;
			    node.green[index] = green;
			    node.blue[index] = blue;
		    },
		    [red, green, blue](auto& node) {
			    node.red.fill(red);
			    node.green.fill(green);
			    node.blue.fill(blue);
		    },
		    propagate);
	}

	void setColor(Code code, Color color, bool propagate = true)
	{
		setColor(code, color.red, color.green, color.blue, propagate);
	}

	void setColor(Code code, color_t red, color_t green, color_t blue,
	              bool propagate = true)
	{
		derived().apply(
		    code,
		    [red, green, blue](auto& node, index_t index) {
			    node.red[index] = red;
			    node.green[index] = green;
			    node.blue[index] = blue;
		    },
		    [red, green, blue](auto& node) {
			    node.red.fill(red);
			    node.green.fill(green);
			    node.blue.fill(blue);
		    },
		    propagate);
	}

	void setColor(Key key, Color color, bool propagate = true)
	{
		setColor(derived().toCode(key), color, propagate);
	}

	void setColor(Key key, color_t red, color_t green, color_t blue, bool propagate = true)
	{
		setColor(derived().toCode(key), red, green, blue, propagate);
	}

	void setColor(Point coord, Color color, bool propagate = true, depth_t depth = 0)
	{
		setColor(derived().toCode(coord, depth), color, propagate);
	}

	void setColor(Point coord, color_t red, color_t green, color_t blue,
	              bool propagate = true, depth_t depth = 0)
	{
		setColor(derived().toCode(coord, depth), red, green, blue, propagate);
	}

	void setColor(coord_t x, coord_t y, coord_t z, Color color, bool propagate = true,
	              depth_t depth = 0)
	{
		setColor(derived().toCode(x, y, z, depth), color, propagate);
	}

	void setColor(coord_t x, coord_t y, coord_t z, color_t red, color_t green, color_t blue,
	              bool propagate = true, depth_t depth = 0)
	{
		setColor(derived().toCode(x, y, z, depth), red, green, blue, propagate);
	}

	//
	// Update color
	//

	template <class UnaryFunction>
	void updateColor(Node node, UnaryFunction f, bool propagate = true)
	{
		derived().apply(
		    node,
		    [f](auto& node, index_t index) {
			    auto c = f(Color(node.red[index], node.green[index], node.blue[index]));
			    node.red[index] = c.red;
			    node.green[index] = c.green;
			    node.blue[index] = c.blue;
		    },
		    [f](auto& node) {
			    for (index_t i = 0; node.red.size() != i; ++i) {
				    auto c = f(Color(node.red[i], node.green[i], node.blue[i]));
				    node.red[i] = c.red;
				    node.green[i] = c.green;
				    node.blue[i] = c.blue;
			    }
		    },
		    propagate);
	}

	template <class UnaryFunction>
	void updateColor(Code code, UnaryFunction f, bool propagate = true)
	{
		derived().apply(
		    code,
		    [f](auto& node, index_t index) {
			    auto c = f(Color(node.red[index], node.green[index], node.blue[index]));
			    node.red[index] = c.red;
			    node.green[index] = c.green;
			    node.blue[index] = c.blue;
		    },
		    [f](auto& node) {
			    for (index_t i = 0; node.red.size() != i; ++i) {
				    auto c = f(Color(node.red[i], node.green[i], node.blue[i]));
				    node.red[i] = c.red;
				    node.green[i] = c.green;
				    node.blue[i] = c.blue;
			    }
		    },
		    propagate);
	}

	template <class UnaryFunction>
	void updateColor(Key key, UnaryFunction f, bool propagate = true)
	{
		updateColor(derived().toCode(key), f, propagate);
	}

	template <class UnaryFunction>
	void updateColor(Point coord, UnaryFunction f, bool propagate = true, depth_t depth = 0)
	{
		updateColor(derived().toCode(coord, depth), f, propagate);
	}

	template <class UnaryFunction>
	void updateColor(coord_t x, coord_t y, coord_t z, UnaryFunction f,
	                 bool propagate = true, depth_t depth = 0)
	{
		updateColor(derived().toCode(x, y, z, depth), f, propagate);
	}

	//
	// Has color
	//

	[[nodiscard]] bool hasColor(Node node) const
	{
		auto const& data = derived().leafNode(node);
		auto index = node.index();
		return 0 != data.red[index] || 0 != data.green[index] || 0 != data.blue[index];
	}

	[[nodiscard]] bool hasColor(Code code) const
	{
		auto [node, depth] = derived().leafNodeAndDepth(code);
		auto index = code.index(depth);
		return 0 != node.red[index] || 0 != node.green[index] || 0 != node.blue[index];
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
	ColorMapBase(ColorMapBase<Derived2> const&)
	{
	}

	//
	// Assignment operator
	//

	ColorMapBase& operator=(ColorMapBase const&) = default;

	ColorMapBase& operator=(ColorMapBase&&) = default;

	template <class Derived2>
	ColorMapBase& operator=(ColorMapBase<Derived2> const&)
	{
		return *this;
	}

	//
	// Swap
	//

	void swap(ColorMapBase&) noexcept {}

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
		derived().root().red[index] = 0;
		derived().root().green[index] = 0;
		derived().root().blue[index] = 0;
	}

	//
	// Update node
	//

	template <std::size_t N, class InputIt>
	void updateNode(ColorNode<N>& node, IndexField const indices, InputIt first,
	                InputIt last)
	{
		if (indices.all()) {
			for (index_t i = 0; first != last; ++first, ++i) {
				Color color = meanColor(*first);
				node.red[i] = color.red;
				node.green[i] = color.green;
				node.blue[i] = color.blue;
			}
		} else {
			for (index_t i = 0; first != last; ++first, ++i) {
				if (indices[i]) {
					Color color = meanColor(*first);
					node.red[i] = color.red;
					node.green[i] = color.green;
					node.blue[i] = color.blue;
				}
			}
		}
	}

	template <std::size_t N>
	Color meanColor(ColorNode<N> node) const
	{
		double red = 0;
		double green = 0;
		double blue = 0;

		std::size_t num = 0;
		for (std::size_t i = 0; N != i; ++i) {
			if (0 != node.red[i] || 0 != node.green[i] || 0 != node.blue[i]) {
				++num;
				red += node.red[i];
				green += node.green[i];
				blue += node.blue[i];
			}
		}
		return 0 == num ? Color() : Color(red / num, green / num, blue / num);
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr DataIdentifier dataIdentifier() noexcept
	{
		return DataIdentifier::COLOR;
	}

	[[nodiscard]] static constexpr bool canReadData(DataIdentifier identifier) noexcept
	{
		return dataIdentifier() == identifier;
	}

	template <class InputIt>
	[[nodiscard]] static constexpr std::size_t numData() noexcept
	{
		using value_type = typename std::iterator_traits<InputIt>::value_type;
		using node_type = typename value_type::node_type;
		return node_type::colorSize();
	}

	template <class InputIt>
	std::size_t serializedSize(InputIt first, InputIt last) const
	{
		return 3 * std::iterator_traits<InputIt>::value_type::colorSize() *
		       std::distance(first, last) * sizeof(color_t);
	}

	template <class OutputIt>
	void readNodes(std::istream& in, OutputIt first, OutputIt last)
	{
		constexpr std::uint8_t const N = numData<OutputIt>();
		constexpr auto const S = 3 * N;
		auto const num_data = std::distance(first, last) * S;
		auto data = std::make_unique<color_t[]>(num_data);
		in.read(reinterpret_cast<char*>(data.get()),
		        num_data * sizeof(typename decltype(data)::element_type));
		for (auto d = data.get(); first != last; ++first, d += S) {
			if (first->index_field.all()) {
				std::copy(d, d + N, first->node.red.data());
				std::copy(d + N, d + (2 * N), first->node.green.data());
				std::copy(d + (2 * N), d + (3 * N), first->node.blue.data());
			} else {
				for (index_t i = 0; N != i; ++i) {
					if (first->index_field[i]) {
						first->node.red[i] = *(d + i);
						first->node.green[i] = *(d + N + i);
						first->node.blue[i] = *(d + (2 * N) + i);
					}
				}
			}
		}
	}

	template <class OutputIt>
	void readNodes(ReadBuffer&, OutputIt, OutputIt)
	{
		// TODO: Implement
	}

	template <class InputIt>
	void writeNodes(std::ostream& out, InputIt first, InputIt last) const
	{
		constexpr std::uint8_t const N = 8;  // FIXME: numData<InputIt>();
		constexpr auto const S = 3 * N;
		auto const num_data = std::distance(first, last) * S;
		auto data = std::make_unique<color_t[]>(num_data);
		for (auto d = data.get(); first != last; ++first) {
			d = copy(first->red, d);
			d = copy(first->green, d);
			d = copy(first->blue, d);
		}
		out.write(reinterpret_cast<char const*>(data.get()),
		          num_data * sizeof(typename decltype(data)::element_type));
	}

	template <class InputIt>
	void writeNodes(WriteBuffer& out, InputIt first, InputIt last) const
	{
		out.reserve(out.size() + serializedSize(first, last));
		for (; first != last; ++first) {
			out.write(first->red.data(), first->red.size() * sizeof(color_t));
			out.write(first->green.data(), first->green.size() * sizeof(color_t));
			out.write(first->blue.data(), first->blue.size() * sizeof(color_t));
		}
	}
};
}  // namespace ufo::map

#endif  // UFO_MAP_COLOR_MAP_H