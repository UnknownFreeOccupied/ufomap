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

#ifndef UFO_MAP_COST_MAP_HPP
#define UFO_MAP_COST_MAP_HPP

// UFO
#include <ufo/container/tree/container.hpp>
#include <ufo/container/tree/tree.hpp>
#include <ufo/map/cost/block.hpp>
#include <ufo/map/cost/predicate.hpp>
#include <ufo/map/cost/propagation_criteria.hpp>
#include <ufo/map/map/map_block.hpp>
#include <ufo/map/type.hpp>
#include <ufo/math/transform3.hpp>
#include <ufo/utility/bit_set.hpp>
#include <ufo/utility/io/buffer.hpp>
#include <ufo/utility/macros.hpp>
#include <ufo/utility/type_traits.hpp>

// STL
#include <array>
#include <iostream>
#include <limits>
#include <string_view>
#include <type_traits>

namespace ufo
{
template <class Derived, class Tree>
class CostMap
{
 private:
	template <class Derived2, class Tree2>
	friend class CostMap;

	static constexpr auto const BF  = Tree::branchingFactor();
	static constexpr auto const Dim = Tree::dimensions();

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                        Tags                                         |
	|                                                                                     |
	**************************************************************************************/

	static constexpr MapType const Type = MapType::COST;

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

	// Cost
	using cost_t = typename CostBlock<BF>::cost_t;

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                       Access                                        |
	|                                                                                     |
	**************************************************************************************/

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] cost_t cost(NodeType node) const
	{
		Index n = derived().index(node);
		return costBlock(n.pos)[n.offset].cost;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Modifiers                                      |
	|                                                                                     |
	**************************************************************************************/

	/**
	 * @brief
	 *
	 * @note If `NodeType` is `Index`, only propagates up to `node` and it does not set
	 * modified if propagation is `false`.
	 *
	 * @tparam NodeType Should be of type `Node`, `Code`, `Key`, or `Coord`
	 * @param node
	 * @param value
	 * @param propagate
	 */
	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	void costSet(NodeType node, cost_t value, bool propagate = true)
	{
		CostElement elem(value);

		auto node_f  = [this, elem](Index node) { costBlock(node.pos)[node.offset] = elem; };
		auto block_f = [this, elem](pos_t pos) { costBlock(pos).fill(elem); };

		if constexpr (std::is_same_v<Index, std::decay_t<NodeType>>) {
			if (propagate) {
				derived().recursParentFirst(node, node_f, block_f);
			} else {
				derived().recursLeaves(node, node_f, block_f);
			}
		} else {
			if (propagate) {
				auto propagate_f = [this](Index node, pos_t children) {
					onPropagateChildren(node, children);
				};

				derived().recursLeaves(derived().code(node), node_f, block_f, propagate_f,
				                       propagate);
			} else {
				derived().recursParentFirst(derived().code(node), node_f, block_f);
			}
		}
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	void costUpdate(NodeType node, cost_t change, bool propagate = true)
	{
		costUpdate(
		    node,
		    [this, change](Index node) {
			    return CostBlock(node.pos)[node.offset].cost + change;
		    },
		    propagate);
	}

	template <class NodeType, class UnaryOp,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool>       = true,
	          std::enable_if_t<std::is_invocable_r_v<cost_t, UnaryOp, Index>, bool> = true>
	void costUpdate(NodeType node, UnaryOp unary_op, bool propagate = true)
	{
		auto node_f = [this, unary_op](Index node) {
			cost_t value = unary_op(node);

			auto& cb = CostBlock(node.pos)[node.offset];
			cb.cost  = value;
		};

		auto block_f = [this, node_f](pos_t pos) {
			for (std::size_t i{}; BF > i; ++i) {
				node_f(Index(pos, i));
			}
		};

		auto propagate_f = [this](Index node, pos_t children) {
			onPropagateChildren(node, children);
		};

		if constexpr (std::is_same_v<Index, std::decay_t<NodeType>>) {
			if (propagate) {
				derived().recursLeaves(node, node_f, block_f, propagate_f);
			} else {
				derived().recursLeaves(node, node_f, block_f);
			}
		} else {
			derived().recursLeaves(derived().code(node), node_f, block_f, propagate_f,
			                       propagate);
		}
	}

	//
	// Propagation criteria
	//

	[[nodiscard]] constexpr CostPropagationCriteria costPropagationCriteria() const noexcept
	{
		return prop_criteria_;
	}

	void costSetPropagationCriteria(CostPropagationCriteria prop_criteria,
	                                bool                    propagate = true)
	{
		if (CostPropagationCriteria() == prop_criteria) {
			return;
		}

		prop_criteria_ = prop_criteria;

		// Set all inner nodes to modified
		// FIXME: Possible to optimize this to only set the ones with children
		derived().setModified();

		if (propagate) {
			derived().propagateModified();
		}
	}

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	CostMap() { onInitRoot(); }

	CostMap(CostMap const& other) = default;

	CostMap(CostMap&& other) = default;

	template <class Derived2, class Tree2>
	CostMap(CostMap<Derived2, Tree2> const& other) : prop_criteria_(other.prop_criteria_)
	{
	}

	template <class Derived2, class Tree2>
	CostMap(CostMap<Derived2, Tree2>&& other)
	    : prop_criteria_(std::move(other.prop_criteria_))
	{
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Destructor                                      |
	|                                                                                     |
	**************************************************************************************/

	~CostMap() = default;

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	CostMap& operator=(CostMap const& rhs) = default;

	CostMap& operator=(CostMap&& rhs) = default;

	template <class Derived2, class Tree2>
	CostMap& operator=(CostMap<Derived2, Tree2> const& rhs)
	{
		prop_criteria_ = rhs.prop_criteria_;
		return *this;
	}

	template <class Derived2, class Tree2>
	CostMap& operator=(CostMap<Derived2, Tree2>&& rhs)
	{
		prop_criteria_ = std::move(rhs.prop_criteria_);
		return *this;
	}

	//
	// Swap
	//

	void swap(CostMap& other) noexcept { std::swap(prop_criteria_, other.prop_criteria_); }

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

	[[nodiscard]] CostBlock<BF>& costBlock(pos_t pos)
	{
		return derived().template data<CostBlock<BF>>(pos);
	}

	[[nodiscard]] CostBlock<BF> const& costBlock(pos_t pos) const
	{
		return derived().template data<CostBlock<BF>>(pos);
	}

	/**************************************************************************************
	|                                                                                     |
	|                              Functions Derived expect                               |
	|                                                                                     |
	**************************************************************************************/

	void onInitRoot()
	{
		auto& block = costBlock(0);

		cost_t value{};
		block[0].cost = value;
	}

	void onInitChildren(Index node, pos_t children)
	{
		costBlock(children).fill(costBlock(node.pos)[node.offset]);
	}

	void onPruneChildren(Index node, pos_t children)
	{
		auto& v = costBlock(node.pos)[node.offset];
	}

	void onPropagateChildren(Index node, pos_t children)
	{
		auto const& children_block = costBlock(children);

		auto& cb = costBlock(node.pos)[node.offset];

		cost_t value;
		switch (prop_criteria_) {
			case CostPropagationCriteria::MAX:
				value = std::numeric_limits<cost_t>::lowest();
				for (std::size_t i{}; BF > i; ++i) {
					value = UFO_MAX(value, children_block[i].cost);
				}
				break;
			case CostPropagationCriteria::MIN:
				value = std::numeric_limits<cost_t>::max();
				for (std::size_t i{}; BF > i; ++i) {
					value = UFO_MIN(value, children_block[i].cost);
				}
				break;
			case CostPropagationCriteria::MEAN: {
				cost_t v = 0;
				for (std::size_t i{}; BF > i; ++i) {
					v += static_cast<int>(children_block[i].cost);
				}
				value = static_cast<cost_t>(v / static_cast<int>(BF));
				break;
			}
			case CostPropagationCriteria::NONE: return;
		}

		cb.cost = value;
	}

	//
	// Is prunable
	//

	[[nodiscard]] bool prunable(pos_t block) const
	{
		using std::begin;
		using std::end;
		return std::all_of(
		    begin(costBlock(block).data) + 1, end(costBlock(block).data),
		    [v = costBlock(block)[0].cost](auto const& e) { return v == e.cost; });
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr std::size_t serializedSizeNode() noexcept
	{
		return sizeof(cost_t);
	}

	[[nodiscard]] constexpr std::size_t serializedSize(
	    std::vector<std::pair<pos_t, BitSet<BF>>> const& /* nodes */,
	    std::size_t num_nodes) const
	{
		return num_nodes * serializedSizeNode();
	}

	void onRead(ReadBuffer& in, std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes)
	{
		for (auto [block, offset] : nodes) {
			auto& cb = costBlock(block);

			if (offset.all()) {
				std::array<cost_t, BF> cost;
				in.read(cost);
				for (offset_t i{}; BF > i; ++i) {
					cb[i] = CostElement(cost[i]);
				}
			} else {
				for (offset_t i{}; BF > i; ++i) {
					if (offset[i]) {
						cost_t value;
						in.read(value);
						cb[i] = CostElement(value);
					}
				}
			}
		}
	}

	void onWrite(WriteBuffer& out, std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes)
	{
		for (auto [block, offset] : nodes) {
			auto const& cb = CostBlock(block);

			if (offset.all()) {
				std::array<cost_t, BF> cost;
				for (offset_t i{}; BF > i; ++i) {
					cost[i] = cb[i].cost;
				}
				out.write(cost);
			} else {
				for (offset_t i{}; BF > i; ++i) {
					if (offset[i]) {
						out.write(cb[i].cost);
					}
				}
			}
		}
	}

	//
	// Dot file info
	//

	void onDotFileInfo(std::ostream& out, Index node) const
	{
		out << "Cost: " << cost(node);
	}

 private:
	// Propagation criteria
	CostPropagationCriteria prop_criteria_ = CostPropagationCriteria::MAX;
};

template <std::size_t Dim, std::size_t BF>
struct map_block<CostMap, Dim, BF> {
	using type = CostBlock<BF>;
};
}  // namespace ufo

#endif  // UFO_MAP_COST_MAP_HPP