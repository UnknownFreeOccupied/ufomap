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
#include <ufo/map/predicate/color.h>

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

	constexpr RGBColor getColor(Node node) const
	{
		return getColor(derived().getLeafNode(node), node.index());
	}

	constexpr RGBColor getColor(Code code) const
	{
		return getColor(derived().getLeafNode(code), node.index());
	}

	constexpr RGBColor getColor(Key key) const { return getColor(Derived::toCode(key)); }

	constexpr RGBColor getColor(Point3 coord, depth_t depth = 0) const
	{
		return getColor(derived().toCode(coord, depth));
	}

	constexpr RGBColor getColor(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return getColor(derived().toCode(x, y, z, depth));
	}

	//
	// Set color
	//

	constexpr void setColor(Node& node, RGBColor color, bool propagate = true)
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

	constexpr void clearColor(Node& node, bool propagate = true)
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

	constexpr void setColor(Point3 coord, bool propagate = true, depth_t depth = 0)
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

	constexpr Derived& derived() { return *static_cast<Derived*>(this); }

	constexpr Derived const& derived() const { return *static_cast<Derived const*>(this); }

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
	static constexpr void clearColor(ColorNodoe<Single>& node, index_t const index) noexcept
	{
		setColor(node, index, RGBColor());
	}

	//
	// Update node
	//

	template <bool Single>
	constexpr void updateNode(ColorNode<Single> const&) noexcept
	{
	}

	template <bool Single, class T>
	void updateNode(ColorNode<Single>& node, index_field_t const indices, T const& children)
	{
		if constexpr (Single) {
			std::array<RGBColor, children.size()> colors;
			for (std::size_t i = 0; children.size() != i; ++i) {
				colors[i] = getColor(children[i], 0);
			}
			setColor(node, RGBColor::average(std::cbegin(colors), std::cend(colors)));
		} else {
			for (index_t index = 0; children.size() != index; ++index) {
				if ((indices >> index) & index_field_t(1)) {
					std::array<RGBColor, children.size()> colors;
					for (std::size_t i = 0; children.size() != i; ++i) {
						colors[i] = getColor(children[index], i);
					}
					setColor(node, index,
					         RGBColor::average(std::cbegin(colors), std::cend(colors)));
				}
			}
		}
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
		// TODO: Implement

		auto const num_nodes = std::distance(first, last);

		auto data = std::make_unique<RGBColor[]>(num_nodes);
		in.read(reinterpret_cast<char*>(data.get()),
		        num_nodes * sizeof(typename decltype(data)::element_type));

		for (std::size_t i = 0; num_nodes != i; ++i, std::advance(first, 1)) {
			setColor(*first, data[i]);
		}
	}

	template <class InputIt>
	void writeNodes(std::ostream& out, InputIt first, InputIt last)
	{
		// TODO: Implement

		auto const num_nodes = std::distance(first, last);

		auto data = std::make_unique<RGBColor[]>(num_nodes);
		for (std::size_t i = 0; num_nodes != i; ++i, std::advance(first, 1)) {
			data[i] = getColor(*first);
		}

		out.write(reinterpret_cast<char const*>(data.get()),
		          num_nodes * sizeof(typename decltype(data)::element_type));
	}
};
}  // namespace ufo::map

#endif  // UFO_MAP_COLOR_MAP_BASE_H