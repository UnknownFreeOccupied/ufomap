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

#ifndef UFO_MAP_COLOR_MAP_HPP
#define UFO_MAP_COLOR_MAP_HPP

// UFO
#include <ufo/map/block.hpp>
#include <ufo/map/color/block.hpp>
#include <ufo/map/type.hpp>
#include <ufo/utility/bit_set.hpp>
#include <ufo/utility/io/buffer.hpp>
#include <ufo/vision/color.hpp>
#include <ufo/vision/image.hpp>

// STL
#include <algorithm>
#include <array>
#include <iomanip>
#include <iostream>
#include <type_traits>
#include <vector>

namespace ufo
{
template <class Derived, class Tree>
class ColorMap
{
	template <class Derived2, class Tree2>
	friend class ColorMap;

	static constexpr auto const BF  = Tree::branchingFactor();
	static constexpr auto const Dim = Tree::dimensions();

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                        Tags                                         |
	|                                                                                     |
	**************************************************************************************/

	static constexpr MapType const Type = MapType::COLOR;

	// Container
	using Index    = typename Tree::Index;
	using Node     = typename Tree::Node;
	using Code     = typename Tree::Code;
	using Key      = typename Tree::Key;
	using Point    = typename Tree::Point;
	using Coord    = typename Tree::Coord;
	using coord_t  = typename Tree::coord_t;
	using depth_t  = typename Tree::depth_t;
	using offset_t = typename Tree::offset_t;
	using length_t = typename Tree::length_t;
	using pos_t    = typename Tree::pos_t;

	using color_t = typename Color::value_type;

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                       Access                                        |
	|                                                                                     |
	**************************************************************************************/

	template <class NodeType>
	[[nodiscard]] Color color(NodeType node) const
	{
		Index n = derived().index(node);
		return colorBlock(n.pos)[n.offset];
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Modifiers                                      |
	|                                                                                     |
	**************************************************************************************/

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	void colorSet(NodeType node, Color value, bool propagate = true)
	{
		auto node_f = [this, value](Index node) {
			colorBlock(node.pos)[node.offset] = value;
		};

		auto block_f = [this, value](pos_t block) { colorBlock(block) = value; };

		auto update_f = [this](Index node, pos_t children) {
			onPropagateChildren(node, children);
		};

		derived().recursParentFirst(node, node_f, block_f, update_f, propagate);
	}

	template <class NodeType>
	void colorSet(NodeType node, color_t red, color_t green, color_t blue,
	              color_t alpha     = std::numeric_limits<color_t>::max(),
	              bool    propagate = true)
	{
		colorSet(node, Color(red, green, blue, alpha), propagate);
	}

	//
	// Update color
	//

	template <class NodeType, class UnaryOp,
	          std::enable_if_t<std::is_invocable_r_v<Color, UnaryOp, Index>, bool> = true>
	void colorUpdate(NodeType node, UnaryOp unary_op, bool propagate = true)
	{
		auto node_f = [this, unary_op](Index node) {
			colorBlock(node.pos)[node.offset] = unary_op(node);
		};

		auto block_f = [this, unary_op](pos_t block) {
			for (std::size_t i{}; BF > i; ++i) {
				colorBlock(block)[i] = unary_op(Index(block, i));
			}
		};

		auto update_f = [this](Index node, pos_t children) {
			onPropagateChildren(node, children);
		};

		derived().recursLeaves(node, node_f, block_f, update_f, propagate);
	}

	template <class NodeType>
	void colorUpdate(NodeType node, Color color, float weight = 1.0f, bool propagate = true)
	{
		assert(0.0f <= weight && 1.0f >= weight);

		float red        = weight * color.red;
		float green      = weight * color.green;
		float blue       = weight * color.blue;
		float alpha      = weight * color.alpha;
		float weight_inv = 1.0f - weight;

		colorUpdate(
		    node,
		    [&](Index node) {
			    Color c = colorBlock(node.pos)[node.offset];
			    c.red   = weight_inv * c.red + red;
			    c.green = weight_inv * c.green + green;
			    c.blue  = weight_inv * c.blue + blue;
			    c.alpha = weight_inv * c.alpha + alpha;
			    return c;
		    },
		    propagate);
	}

	template <class NodeType>
	void colorClear(NodeType node, bool propagate = true)
	{
		colorSet(node, Color(0), propagate);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Lookup                                        |
	|                                                                                     |
	**************************************************************************************/

	template <class NodeType>
	[[nodiscard]] bool colorEmpty(NodeType node) const
	{
		Index n = derived().index(node);
		return colorBlock(n.pos)[n.offset].empty();
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Propagation                                     |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] constexpr ColorBlendingMode colorPropagationBlendingMode() const noexcept
	{
		return prop_mode_;
	}

	void colorSetPropagationBlendingMode(ColorBlendingMode prop_mode, bool propagate = true)
	{
		if (colorPropagationBlendingMode() == prop_mode) {
			return;
		}

		prop_mode_ = prop_mode;

		// Set all inner nodes to modified
		// // FIXME: Possible to optimize this to only set the ones with children
		// derived().setModified();

		// if (propagate) {
		// 	derived().propagateModified();
		// }
	}

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	ColorMap() { onInitRoot(); }

	ColorMap(ColorMap const&) = default;

	ColorMap(ColorMap&&) = default;

	template <class Derived2, class Tree2>
	ColorMap(ColorMap<Derived2, Tree2> const& other)
	// TODO: Implement
	{
	}

	template <class Derived2, class Tree2>
	ColorMap(ColorMap<Derived2, Tree2>&& other)
	// TODO: Implement
	{
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Destructor                                      |
	|                                                                                     |
	**************************************************************************************/

	~ColorMap() = default;

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	ColorMap& operator=(ColorMap const&) = default;

	ColorMap& operator=(ColorMap&&) = default;

	template <class Derived2, class Tree2>
	ColorMap& operator=(ColorMap<Derived2, Tree2> const& rhs)
	{
		// TODO: Implement
		return *this;
	}

	template <class Derived2, class Tree2>
	ColorMap& operator=(ColorMap<Derived2, Tree2>&& rhs)
	{
		// TODO: Implement
		return *this;
	}

	//
	// Swap
	//

	friend void swap(ColorMap& lhs, ColorMap& rhs) noexcept
	{
		// TODO: Implement
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Derived                                       |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] constexpr Derived& derived() { return *static_cast<Derived*>(this); }

	[[nodiscard]] constexpr Derived const& derived() const
	{
		return *static_cast<Derived const*>(this);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Block                                        |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] ColorBlock<BF>& colorBlock(pos_t pos)
	{
		return derived().template data<ColorBlock<BF>>(pos);
	}

	[[nodiscard]] ColorBlock<BF> const& colorBlock(pos_t pos) const
	{
		return derived().template data<ColorBlock<BF>>(pos);
	}

	/**************************************************************************************
	|                                                                                     |
	|                              Functions Derived expects                              |
	|                                                                                     |
	**************************************************************************************/

	void onInitRoot() { colorBlock(0) = Color(0); }

	void onInitChildren(Index node, pos_t children)
	{
		colorBlock(children) = colorBlock(node.pos)[node.offset];
	}

	void onPropagateChildren(Index node, pos_t children)
	{
		colorBlock(node.pos)[node.offset] =
		    Color::blend(colorBlock(children).data, colorPropagationBlendingMode());
	}

	[[nodiscard]] bool onIsPrunable(pos_t block) const
	{
		using std::begin;
		using std::end;
		return std::all_of(begin(colorBlock(block).data) + 1, cend(colorBlock(block).data),
		                   [v = colorBlock(block).data.front()](auto e) { return v == e; });
	}

	void onPruneChildren(Index /* node */, pos_t /* children */) {}

	[[nodiscard]] std::size_t onSerializedSize(
	    std::vector<std::pair<pos_t, BitSet<BF>>> const& /* nodes */,
	    std::size_t num_nodes) const
	{
		return num_nodes * sizeof(Color);
	}

	void onRead(ReadBuffer& in, std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes)
	{
		for (auto [block, offset] : nodes) {
			auto& cb = colorBlock(block);

			if (offset.all()) {
				in.read(cb);
			} else {
				for (std::size_t i{}; BF > i; ++i) {
					if (offset[i]) {
						in.read(cb[i]);
					}
				}
			}
		}
	}

	void onWrite(WriteBuffer&                                     out,
	             std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes) const
	{
		for (auto [block, offset] : nodes) {
			auto const& cb = colorBlock(block);

			if (offset.all()) {
				out.write(cb);
			} else {
				for (std::size_t i{}; BF > i; ++i) {
					if (offset[i]) {
						out.write(cb[i]);
					}
				}
			}
		}
	}

	void onDotFile(std::ostream& out, Index node) const
	{
		Color    c     = color(node);
		unsigned red   = c.red;
		unsigned green = c.green;
		unsigned blue  = c.blue;
		unsigned alpha = c.alpha;

		out << "RGBA: <font color='#" << std::hex << std::setfill('0') << std::setw(2) << red
		    << std::setw(2) << green << std::setw(2) << blue << "'> #" << std::uppercase
		    << std::setw(2) << red << std::setw(2) << green << std::setw(2) << blue
		    << std::setw(2) << alpha << "</font>" << std::nouppercase << std::dec;
		// out << color_[node.pos][node.offset];

		// auto color_box = "▀";  // "█"; // ▄, ▌, ▐

		// out << std::hex << std::setfill('0') << "RGBA: #" << std::setw(2) << red
		//     << std::setw(2) << green << std::setw(2) << blue << std::setw(2) << alpha
		//     << "<font color='#" << std::setw(2) << red << std::setw(2) << green
		//     << std::setw(2) << blue << "'> " << color_box << "</font>" << std::dec;
	}

 protected:
	// Propagation
	ColorBlendingMode prop_mode_ = ColorBlendingMode::MEAN_ALPHA;
};

template <std::size_t Dim, std::size_t BF>
struct map_block<ColorMap, Dim, BF> {
	using type = ColorBlock<BF>;
};
}  // namespace ufo

#endif  // UFO_MAP_COLOR_MAP_HPP