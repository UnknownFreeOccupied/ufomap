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
		return getColor(derived().getLeafNode(node));
	}

	constexpr RGBColor getColor(Code code) const
	{
		return getColor(derived().getLeafNode(code));
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
		    node, [color](ColorNode& node) { ColorMapBase::setColor(node, color); },
		    propagate);
	}

	constexpr void setColor(Code code, RGBColor color, bool propagate = true)
	{
		derived().apply(
		    code, [color](ColorNode& node) { ColorMapBase::setColor(node, color); },
		    propagate);
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
		    node, [](ColorNode& node) { ColorMapBase::clearColor(node); }, propagate);
	}

	constexpr void clearColor(Code code, bool propagate = true)
	{
		derived().apply(
		    code, [](ColorNode& node) { ColorMapBase::clearColor(node); }, propagate);
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

	void initRoot() { clearColor(derived().getRoot()); }

	//
	// Get color
	//

	static constexpr RGBColor& getColor(ColorNode& node) noexcept { return node.color; }

	static constexpr RGBColor getColor(ColorNode node) noexcept { return node.color; }

	//
	// Set color
	//

	static constexpr void setColor(ColorNode& node, RGBColor color) noexcept
	{
		node.color = color;
	}

	//
	// Clear color
	//

	static constexpr void clearColor(ColorNode& node) noexcept { getColor(node).clear(); }

	//
	// Update node
	//

	constexpr void updateNode(ColorNode) noexcept {}

	template <class T>
	void updateNode(ColorNode& node, T const& children)
	{
		std::array<RGBColor, children.size()> colors;
		for (std::size_t i = 0; children.size() != i; ++i) {
			colors[i] = getColor(children[i]);
		}

		setColor(node, RGBColor::average(std::cbegin(colors), std::cend(colors)));
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
	void readNodes(std::istream& in, InputIt first, InputIt last)
	{
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