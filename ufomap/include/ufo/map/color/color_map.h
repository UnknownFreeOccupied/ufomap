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
class ColorMap
{
 public:
	//
	// Get color
	//

	[[nodiscard]] Color color(Node node) const
	{
		return derived().leafNode(node).color(node.index());
	}

	[[nodiscard]] Color color(Code code) const
	{
		auto [node, depth] = derived().leafNodeAndDepth(code);
		return node.color(code.index(depth));
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
			    node.setColor(index, red, green, blue);
		    },
		    [red, green, blue](auto& node) { node.setColor(red, green, blue); }, propagate);
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
			    node.setColor(index, red, green, blue);
		    },
		    [red, green, blue](auto& node) { node.setColor(red, green, blue); }, propagate);
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
	// Has color
	//

	[[nodiscard]] bool hasColor(Node node) const
	{
		return derived().leafNode(node).hasColor(node.index());
	}

	[[nodiscard]] bool hasColor(Code code) const
	{
		auto [node, depth] = derived().leafNodeAndDepth(code);
		return node.hasColor(code.index(depth));
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
		derived().apply(
		    node, [](auto& node, index_t index) { node.clearColor(index); },
		    [](auto& node) { node.clearColor(); }, propagate);
	}

	void clearColor(Code code, bool propagate = true)
	{
		derived().apply(
		    code, [](auto& node, index_t index) { node.clearColor(index); },
		    [](auto& node) { node.clearColor(); }, propagate);
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

	ColorMap() = default;

	ColorMap(ColorMap const&) = default;

	ColorMap(ColorMap&&) = default;

	template <class Derived2>
	ColorMap(ColorMap<Derived2> const&)
	{
	}

	//
	// Assignment operator
	//

	ColorMap& operator=(ColorMap const&) = default;

	ColorMap& operator=(ColorMap&&) = default;

	template <class Derived2>
	ColorMap& operator=(ColorMap<Derived2> const&)
	{
		return *this;
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

	void initRoot() { derived().root().clearColor(derived().rootIndex()); }

	//
	// Update node
	//

	template <std::size_t N, class InputIt>
	void updateNode(ColorNode<N>& node, IndexField indices, InputIt first, InputIt last)
	{
		if constexpr (1 == N) {
			node.setColor(mean(first, last, [](ColorNode<N> child) { return child.red[0]; }),
			              mean(first, last, [](ColorNode<N> child) { return child.green[0]; }),
			              mean(first, last, [](ColorNode<N> child) { return child.blue[0]; }));
		} else {
			for (index_t i = 0; first != last; ++first, ++i) {
				if (indices[i]) {
					node.setColor(i, mean(first->red), mean(first->green), mean(first->blue));
				}
			}
		}
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

	template <class OutputIt>
	void readNodes(std::istream& in, OutputIt first, std::size_t num_nodes)
	{
		std::uint8_t n;
		in.read(reinterpret_cast<char*>(&n), sizeof(n));
		std::size_t const s = 3 * n;
		num_nodes *= s;

		auto data = std::make_unique<color_t[]>(num_nodes);
		in.read(reinterpret_cast<char*>(data.get()),
		        num_nodes * sizeof(typename decltype(data)::element_type));

		auto const d = data.get();
		if constexpr (1 == numData<OutputIt>()) {
			if (1 == n) {
				for (std::size_t i = 0; i != num_nodes; ++first, i += 3) {
					first->node.setColor(*(d + i), *(d + i + 1), *(d + i + 2));
				}
			} else {
				for (std::size_t i = 0; i != num_nodes; ++first, i += 24) {
					first->node.setColor(mean(d + i, d + i + 8), mean(d + i + 8, d + i + 16),
					                     mean(d + i + 16, d + i + 24));
				}
			}
		} else {
			if (1 == n) {
				for (std::size_t i = 0; i != num_nodes; ++first, i += 3) {
					if (first->index_field.all()) {
						first->node.setColor(*(d + i), *(d + i + 1), *(d + i + 2));
					} else {
						for (std::size_t index = 0; numData<OutputIt>() != index; ++index) {
							if (first->index_field[index]) {
								first->node.setColor(index, *(d + i), *(d + i + 1), *(d + i + 2));
							}
						}
					}
				}
			} else {
				for (std::size_t i = 0; i != num_nodes; ++first, i += 24) {
					if (first->index_field.all()) {
						std::copy(d + i, d + i + 8, first->node.red.data());
						std::copy(d + i + 8, d + i + 16, first->node.green.data());
						std::copy(d + i + 16, d + i + 24, first->node.blue.data());
					} else {
						for (std::size_t index = 0; numData<OutputIt>() != index; ++index) {
							if (first->index_field[index]) {
								first->node.setColor(index, *(d + i + index), *(d + i + index + 8),
								                     *(d + i + index + 16));
							}
						}
					}
				}
			}
		}
	}

	template <class InputIt>
	void writeNodes(std::ostream& out, InputIt first, std::size_t num_nodes) const
	{
		constexpr std::uint8_t const n = numData<InputIt>();
		constexpr std::size_t const s = 3 * n;
		num_nodes *= s;

		auto data = std::make_unique<color_t[]>(num_nodes);
		auto d = data.get();
		for (std::size_t i = 0; i != num_nodes; ++first, i += s) {
			std::copy(std::cbegin(first->node.red), std::cend(first->node.red), d + i);
			std::copy(std::cbegin(first->node.green), std::cend(first->node.green), d + i + n);
			std::copy(std::cbegin(first->node.blue), std::cend(first->node.blue),
			          d + i + (2 * n));
		}

		out.write(reinterpret_cast<char const*>(&n), sizeof(n));
		out.write(reinterpret_cast<char const*>(data.get()),
		          num_nodes * sizeof(typename decltype(data)::element_type));
	}
};
}  // namespace ufo::map

#endif  // UFO_MAP_COLOR_MAP_H