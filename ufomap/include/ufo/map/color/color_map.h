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

#ifndef UFO_MAP_COLOR_MAP_BASE_H
#define UFO_MAP_COLOR_MAP_BASE_H

// UFO
#include <ufo/map/color/color_node.h>
#include <ufo/map/color/color_predicate.h>

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

	[[nodiscard]] constexpr RGBColor color(Node node) const
	{
		return color(derived().leafNode(node), node.index());
	}

	[[nodiscard]] constexpr RGBColor color(Code code) const
	{
		return color(derived().leafNode(code), node.index());
	}

	[[nodiscard]] constexpr RGBColor color(Key key) const
	{
		return color(Derived::toCode(key));
	}

	[[nodiscard]] constexpr RGBColor color(Point3 coord, depth_t depth = 0) const
	{
		return color(derived().toCode(coord, depth));
	}

	[[nodiscard]] constexpr RGBColor color(coord_t x, coord_t y, coord_t z,
	                                       depth_t depth = 0) const
	{
		return color(derived().toCode(x, y, z, depth));
	}

	//
	// Set color
	//

	constexpr void setColor(Node node, RGBColor color, bool propagate = true)
	{
		derived().apply(
		    node,
		    [color](auto& node, index_t const index) {
			    ColorMapBase::setColor(node, index, color);
		    },
		    [color](auto& node) { ColorMapBase::setColor(node, color); }, propagate);
	}

	constexpr void setColor(Code code, RGBColor color, bool propagate = true)
	{
		derived().apply(
		    code,
		    [color](auto& node, index_t const index) {
			    ColorMapBase::setColor(node, index, color);
		    },
		    [color](auto& node) { ColorMapBase::setColor(node, color); }, propagate);
	}

	constexpr void setColor(Key key, RGBColor color, bool propagate = true)
	{
		setColor(Derived::toCode(key), color, propagate);
	}

	constexpr void setColor(Point3 coord, RGBColor color, bool propagate = true,
	                        depth_t depth = 0)
	{
		setColor(derived().toCode(coord, depth), color, propagate);
	}

	constexpr void setColor(coord_t x, coord_t y, coord_t z, RGBColor color,
	                        bool propagate = true, depth_t depth = 0)
	{
		setColor(derived().toCode(x, y, z, depth), color, propagate);
	}

	//
	// Clear color
	//

	constexpr void clearColor(Node node, bool propagate = true)
	{
		derived().apply(
		    node,
		    [](auto& node, index_t const index) { ColorMapBase::clearColor(node, index); },
		    [](auto& node) { ColorMapBase::clearColor(node); }, propagate);
	}

	constexpr void clearColor(Code code, bool propagate = true)
	{
		derived().apply(
		    code,
		    [](auto& node, index_t const index) { ColorMapBase::clearColor(node, index); },
		    [](auto& node) { ColorMapBase::clearColor(node); }, propagate);
	}

	constexpr void clearColor(Key key, bool propagate = true)
	{
		clearColor(Derived::toCode(key), propagate);
	}

	constexpr void clearColor(Point3 coord, bool propagate = true, depth_t depth = 0)
	{
		clearColor(derived().toCode(coord, depth), propagate);
	}

	constexpr void clearColor(coord_t x, coord_t y, coord_t z, bool propagate = true,
	                          depth_t depth = 0)
	{
		clearColor(derived().toCode(x, y, z, depth), propagate);
	}

 protected:
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

	void initRoot() { clearColor(derived().getRoot(), derived().getRootIndex()); }

	//
	// Get color
	//

	template <bool Single>
	static constexpr RGBColor getColor(ColorNode<Single> const& node,
	                                   index_t const index) noexcept
	{
		return node.getColor(index);
	}

	//
	// Set color
	//

	template <bool Single>
	static constexpr void setColor(ColorNode<Single>& node, RGBColor const color) noexcept
	{
		node.setColor(color);
	}

	template <bool Single>
	static constexpr void setColor(ColorNode<Single>& node, index_t const index,
	                               RGBColor const color) noexcept
	{
		node.setColor(index, color);
	}

	//
	// Clear color
	//

	template <bool Single>
	static constexpr void clearColor(ColorNode<Single>& node) noexcept
	{
		setColor(node, RGBColor());
	}

	template <bool Single>
	static constexpr void clearColor(ColorNode<Single>& node, index_t const index) noexcept
	{
		setColor(node, index, RGBColor());
	}

	//
	// Update node
	//

	template <bool Single, class T>
	void updateNode(ColorNode<Single>& node, index_field_t const indices, T const& children)
	{
		std::array<color_t, children.size()> reds;
		std::array<color_t, children.size()> greens;
		std::array<color_t, children.size()> blues;
		if constexpr (Single) {
			for (std::size_t i = 0; children.size() != i; ++i) {
				reds[i] = getRed(children[i], 0);
				greens[i] = getGreen(children[i], 0);
				blues[i] = getBlues(children[i], 0);
			}
			node.setRed(average(std::cbegin(reds), std::cend(reds)));
			node.setGreen(average(std::cbegin(greens), std::cend(greens)));
			node.setBlue(average(std::cbegin(blues), std::cend(blues)));
		} else {
			for (index_t index = 0; children.size() != index; ++index) {
				if ((indices >> index) & index_field_t(1)) {
					for (std::size_t i = 0; children.size() != i; ++i) {
						reds[i] = getRed(children[index], i);
						greens[i] = getGreen(children[index], i);
						blues[i] = getBlue(children[index], i);
					}
					node.setRed(index, average(std::cbegin(reds), std::cend(reds)));
					node.setGreen(index, average(std::cbegin(greens), std::cend(greens)));
					node.setBlue(index, average(std::cbegin(blues), std::cend(blues)));
				}
			}
		}
	}

	template <class InputIt>
	static constexpr color_t average(InputIt first, InputIt last)
	{
		return std::reduce(first, last, 0.0) / double(std::distance(first, last));
	}

	//
	// Input/output (read/write)
	//

	static constexpr DataIdentifier getDataIdentifier() noexcept
	{
		return DataIdentifier::COLOR;
	}

	static constexpr bool canReadData(DataIdentifier identifier) noexcept
	{
		return getDataIdentifier() == identifier;
	}

	template <class InputIt>
	static constexpr uint8_t isSingle() noexcept
	{
		using typename std::iterator_traits<InputIt>::value_type;
		using typename value_type::node_type;
		if constexpr (std::is_base_of_v(ColorNode<true>, node_type)) {
			return 1;
		} else {
			return 0;
		}
	}

	template <class InputIt>
	void readNodes(std::istream& in, InputIt first, InputIt last)
	{
		uint8_t single;
		in.read(reinterpret_cast<char*>(&single), sizeof(single));

		std::size_t num_nodes = 3 * std::distance(first, last);
		if (!single) {
			num_nodes *= 8;
		}

		auto data = std::make_unique<color_t[]>(num_nodes);
		in.read(reinterpret_cast<char*>(data.get()),
		        num_nodes * sizeof(typename decltype(data)::element_type));

		if constexpr (isSingle<InputIt>()) {
			if (single) {
				for (std::size_t i = 0; first != last; ++first) {
					setRed(first->node, data[i++]);
					setGreen(first->node, data[i++]);
					setBlue(first->node, data[i++]);
				}
			} else {
				for (auto d_first = data.get(); first != last; ++first) {
					auto d_last = std::next(d_first, 8);
					first->node.setRed(average(d_first, d_last));
					d_first = d_last;
					d_last = std::next(d_first, 8);
					first->node.setGreen(average(d_first, d_last));
					d_first = d_last;
					d_last = std::next(d_first, 8);
					first->node.setBlue(average(d_first, d_last));
					d_first = d_last;
					d_last = std::next(d_first, 8);
				}
			}
		} else {
			if (single) {
				for (std::size_t i = 0; first != last; ++first) {
					first->node.setRed(data[i++]);
					first->node.setGreen(data[i++]);
					first->node.setBlue(data[i++]);
				}
			} else {
				for (std::size_t i = 0; first != last; ++first) {
					if (std::numeric_limits<index_field_t>::max() == first->index_field) {
						auto d_first = &(data[i]);
						auto d_last = std::next(d_first, first->node.red.size());
						std::copy(d_first, d_last, first->node.red.data());
						d_first = d_last;
						d_last = std::next(d_first, first->node.green.size());
						std::copy(d_first, d_last, first->node.green.data());
						d_first = d_last;
						d_last = std::next(d_first, first->node.blue.size());
						std::copy(d_first, d_last, first->node.blue.data());
						i += first->node.red.size() + first->node.green.size() +
						     first->node.blue.size();
					} else {
						for (index_t index = 0; first->node.red.size() != index; ++index) {
							if ((first.index_field >> index) & index_field_t(1)) {
								first->node.setRed(index, data[i++]);
								first->node.setGreen(index, data[i++]);
								first->node.setBlue(index, data[i++]);
							} else {
								i += 3;
							}
						}
					}
				}
			}
		}
	}

	template <class InputIt>
	void writeNodes(std::ostream& out, InputIt first, InputIt last)
	{
		constexpr uint8_t const single = isSingle<InputIt>();
		out.write(reinterpret_cast<char const*>(&single), sizeof(single));

		std::size_t num_nodes = 3 * std::distance(first, last);
		if constexpr (!single) {
			num_nodes *= 8;
		}

		auto data = std::make_unique<color_t[]>(num_nodes);
		if constexpr (single) {
			for (std::size_t i = 0; first != last; ++first) {
				data[i++] = first->node.red;
				data[i++] = first->node.green;
				data[i++] = first->node.blue;
			}
		} else {
			for (auto d = data.get(); first != last; ++first) {
				d = std::copy(std::cbegin(first->node.red), std::cend(first->node.red), d);
				d = std::copy(std::cbegin(first->node.green), std::cend(first->node.green), d);
				d = std::copy(std::cbegin(first->node.blue), std::cend(first->node.blue), d);
			}
		}

		out.write(reinterpret_cast<char const*>(data.get()),
		          num_nodes * sizeof(typename decltype(data)::element_type));
	}
};
}  // namespace ufo::map

#endif  // UFO_MAP_COLOR_MAP_BASE_H