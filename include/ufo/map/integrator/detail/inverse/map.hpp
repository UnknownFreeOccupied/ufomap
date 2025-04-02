/*!
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author Daniel Duberg (dduberg@kth.se), Ramona Häuselmann (ramonaha@kth.se)
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

#ifndef UFO_MAP_INTEGRATOR_DETAIL_INVERSE_MAP_HPP
#define UFO_MAP_INTEGRATOR_DETAIL_INVERSE_MAP_HPP

// UFO
#include <ufo/container/tree/container.hpp>
#include <ufo/container/tree/tree.hpp>
#include <ufo/map/block.hpp>
#include <ufo/map/integrator/detail/inverse/block.hpp>
#include <ufo/map/type.hpp>
#include <ufo/math/transform3.hpp>
#include <ufo/utility/bit_set.hpp>
#include <ufo/utility/io/buffer.hpp>
#include <ufo/utility/macros.hpp>
#include <ufo/utility/type_traits.hpp>

// STL
#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <iostream>
#include <limits>
#include <numeric>
#include <string_view>
#include <type_traits>
#include <vector>

namespace ufo::detail
{
template <class Derived, class Tree>
class InverseMap
{
 private:
	template <class Derived2, class Tree2>
	friend class InverseMap;

	static constexpr auto const BF  = Tree::branchingFactor();
	static constexpr auto const Dim = Tree::dimensions();

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                        Tags                                         |
	|                                                                                     |
	**************************************************************************************/

	static constexpr MapType const Type = MapType::NONE;

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

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                       Access                                        |
	|                                                                                     |
	**************************************************************************************/

	// NOTE: It is bad practice to return reference since it can destory tree structure.
	// This is a special map type only intended to be used in the integrator
	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] std::atomic_uint_fast32_t& inverseCount(NodeType node)
	{
		Index n = derived().index(node);
		return inverseBlock(n.pos)[n.offset].count;
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] std::uint_fast32_t inverseCount(NodeType node) const
	{
		Index n = derived().index(node);
		return inverseBlock(n.pos)[n.offset].count;
	}

	// NOTE: It is bad practice to return reference since it can destory tree structure.
	// This is a special map type only intended to be used in the integrator
	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] std::atomic_uint_fast32_t& inverseIndex(NodeType node)
	{
		Index n = derived().index(node);
		return inverseBlock(n.pos)[n.offset].index;
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] std::uint_fast32_t inverseIndex(NodeType node) const
	{
		Index n = derived().index(node);
		return inverseBlock(n.pos)[n.offset].index;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Modifiers                                      |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Implement

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	InverseMap() { onInitRoot(); }

	InverseMap(InverseMap const& other) = default;

	InverseMap(InverseMap&& other) = default;

	template <class Derived2, class Tree2>
	InverseMap(InverseMap<Derived2, Tree2> const& /* other */)
	{
	}

	template <class Derived2, class Tree2>
	InverseMap(InverseMap<Derived2, Tree2>&& /* other */)
	{
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Destructor                                      |
	|                                                                                     |
	**************************************************************************************/

	~InverseMap() = default;

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	InverseMap& operator=(InverseMap const& rhs) = default;

	InverseMap& operator=(InverseMap&& rhs) = default;

	template <class Derived2, class Tree2>
	InverseMap& operator=(InverseMap<Derived2, Tree2> const& /* rhs */)
	{
		return *this;
	}

	template <class Derived2, class Tree2>
	InverseMap& operator=(InverseMap<Derived2, Tree2>&& /* rhs */)
	{
		return *this;
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

	[[nodiscard]] InverseBlock<BF>& inverseBlock(pos_t pos)
	{
		return derived().template data<InverseBlock<BF>>(pos);
	}

	[[nodiscard]] InverseBlock<BF> const& inverseBlock(pos_t pos) const
	{
		return derived().template data<InverseBlock<BF>>(pos);
	}

	/**************************************************************************************
	|                                                                                     |
	|                              Functions Derived expects                              |
	|                                                                                     |
	**************************************************************************************/

	void onInitRoot() { inverseBlock(0) = InverseBlock<BF>{}; }

	void onInitChildren(Index node, pos_t children)
	{
		inverseBlock(children).fill(inverseBlock(node.pos)[node.offset]);
	}

	void onPropagateChildren(Index node, pos_t children)
	{
		auto const& children_block = inverseBlock(children);
		auto&       v              = inverseBlock(node.pos)[node.offset];

		std::uint_fast32_t count{};
		for (std::size_t i{}; BF > i; ++i) {
			if (0u < children_block[i].count) {
				++count;
			}
		}
		v.count = count;
	}

	[[nodiscard]] bool onIsPrunable(pos_t /* block */) const { return false; }

	void onPruneChildren(Index /* node */, pos_t /* children */) {}

	[[nodiscard]] constexpr std::size_t onSerializedSize(
	    std::vector<std::pair<pos_t, BitSet<BF>>> const& /* nodes */,
	    std::size_t /* num_nodes */) const
	{
		return 0;
	}

	void onRead(ReadBuffer& /* in */,
	            std::vector<std::pair<pos_t, BitSet<BF>>> const& /* nodes */)
	{
	}

	void onWrite(WriteBuffer& /* out */,
	             std::vector<std::pair<pos_t, BitSet<BF>>> const& /* nodes */)
	{
	}

	void onDotFile(std::ostream& /* out */, Index /* node */) const {}
};

}  // namespace ufo::detail

namespace ufo
{
template <std::size_t Dim, std::size_t BF>
struct map_block<detail::InverseMap, Dim, BF> {
	using type = detail::InverseBlock<BF>;
};
}  // namespace ufo

#endif  // UFO_MAP_INTEGRATOR_DETAIL_INVERSE_MAP_HPP