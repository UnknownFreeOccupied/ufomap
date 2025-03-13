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

#ifndef UFO_MAP_VOID_REGION_MAP_HPP
#define UFO_MAP_VOID_REGION_MAP_HPP

// UFO
#include <ufo/map/block.hpp>
#include <ufo/map/type.hpp>
#include <ufo/map/void_region/block.hpp>
#include <ufo/map/void_region/predicate.hpp>
#include <ufo/utility/bit_set.hpp>
#include <ufo/utility/io/buffer.hpp>

// STL
#include <cstddef>
#include <ostream>
#include <type_traits>
#include <utility>
#include <vector>

namespace ufo
{
template <class Derived, class Tree>
class VoidRegionMap
{
	template <class Derived2, class Tree2>
	friend class VoidRegionMap;

	static constexpr auto const BF  = Tree::branchingFactor();
	static constexpr auto const Dim = Tree::dimensions();

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                        Tags                                         |
	|                                                                                     |
	**************************************************************************************/

	static constexpr MapType const Type = MapType::VOID_REGION;

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
	|                                   Primary Access                                    |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] bool voidRegion() const { return voidRegion(derived().index()); }

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool voidRegion(NodeType node) const
	{
		Index n = derived().index(node);
		return voidRegionBlock(n.pos).primary(n.offset);
	}

	[[nodiscard]] bool voidRegionAny(pos_t block) const
	{
		assert(derived().valid(block));
		return voidRegionBlock(block).primaryAny();
	}

	[[nodiscard]] bool voidRegionAll(pos_t block) const
	{
		assert(derived().valid(block));
		return voidRegionBlock(block).primaryAll();
	}

	[[nodiscard]] bool voidRegionNone(pos_t block) const
	{
		assert(derived().valid(block));
		return voidRegionBlock(block).primaryNone();
	}

	/**************************************************************************************
	|                                                                                     |
	|                                 Primary Modifiers                                   |
	|                                                                                     |
	**************************************************************************************/

	void voidRegionSet(bool value, bool propagate = true)
	{
		voidRegionSet(derived().index(), value, propagate);
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	void voidRegionSet(NodeType node, bool value, bool propagate = true)
	{
		if (value) {
			auto node_f = [this, value](Index node) {
				voidRegionBlock(node.pos).primarySet(node.offset);
			};

			auto block_f = [this, value](pos_t block) {
				voidRegionBlock(block).primaryInit(true);
			};

			auto update_f = [this](Index node, pos_t children) {
				onPropagateChildren(node, children);
			};

			derived().recursParentFirst(node, node_f, block_f, update_f, propagate);
		} else {
			auto node_f = [this, value](Index node) {
				voidRegionBlock(node.pos).primaryReset(node.offset);
			};

			auto block_f = [this, value](pos_t block) {
				voidRegionBlock(block).primaryInit(false);
			};

			auto update_f = [this](Index node, pos_t children) {
				onPropagateChildren(node, children);
			};

			derived().recursParentFirst(node, node_f, block_f, update_f, propagate);
		}
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_r_v<bool, UnaryOp, Index>, bool> = true>
	void voidRegionUpdate(UnaryOp unary_op, bool propagate = true)
	{
		voidRegionUpdate(derived().index(), unary_op, propagate);
	}

	template <class NodeType, class UnaryOp,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool>     = true,
	          std::enable_if_t<std::is_invocable_r_v<bool, UnaryOp, Index>, bool> = true>
	void voidRegionUpdate(NodeType node, UnaryOp unary_op, bool propagate = true)
	{
		auto node_f = [this, unary_op](Index node) {
			voidRegionBlock(node.pos).primarySet(node.offset, unary_op(node));
		};

		auto block_f = [this, unary_op](pos_t block) {
			for (std::size_t i{}; BF > i; ++i) {
				voidRegionBlock(block).primarySet(i, unary_op(Index(block, i)));
			}
		};

		auto update_f = [this](Index node, pos_t children) {
			onPropagateChildren(node, children);
		};

		derived().recursLeaves(node, node_f, block_f, update_f, propagate);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                   Primary Lookup                                    |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] bool voidRegionContains() const
	{
		return voidRegionContains(derived().index());
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool voidRegionContains(NodeType node) const
	{
		Index n = derived().index(node);
		return voidRegionBlock(n.pos).primaryContains(n.offset);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                  Secondary Access                                   |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] bool voidRegionSecondary() const
	{
		return voidRegionSecondary(derived().index());
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool voidRegionSecondary(NodeType node) const
	{
		Index n = derived().index(node);
		return voidRegionBlock(n.pos).secondary(n.offset);
	}

	[[nodiscard]] bool voidRegionSecondaryAny(pos_t block) const
	{
		assert(derived().valid(block));
		return voidRegionBlock(block).secondaryAny();
	}

	[[nodiscard]] bool voidRegionSecondaryAll(pos_t block) const
	{
		assert(derived().valid(block));
		return voidRegionBlock(block).secondaryAll();
	}

	[[nodiscard]] bool voidRegionSecondaryNone(pos_t block) const
	{
		assert(derived().valid(block));
		return voidRegionBlock(block).secondaryNone();
	}

	/**************************************************************************************
	|                                                                                     |
	|                                 Secondary Modifiers                                 |
	|                                                                                     |
	**************************************************************************************/

	void voidRegionSecondarySet(bool value, bool propagate = true)
	{
		voidRegionSecondarySet(derived().index(), value, propagate);
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	void voidRegionSecondarySet(NodeType node, bool value, bool propagate = true)
	{
		if (value) {
			auto node_f = [this, value](Index node) {
				voidRegionBlock(node.pos).secondarySet(node.offset);
			};

			auto block_f = [this, value](pos_t block) {
				voidRegionBlock(block).secondaryInit(true);
			};

			auto update_f = [this](Index node, pos_t children) {
				onPropagateChildren(node, children);
			};

			derived().recursParentFirst(node, node_f, block_f, update_f, propagate);
		} else {
			auto node_f = [this, value](Index node) {
				voidRegionBlock(node.pos).secondaryReset(node.offset);
			};

			auto block_f = [this, value](pos_t block) {
				voidRegionBlock(block).secondaryInit(false);
			};

			auto update_f = [this](Index node, pos_t children) {
				onPropagateChildren(node, children);
			};

			derived().recursParentFirst(node, node_f, block_f, update_f, propagate);
		}
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_r_v<bool, UnaryOp, Index>, bool> = true>
	void voidRegionSecondaryUpdate(UnaryOp unary_op, bool propagate = true)
	{
		voidRegionSecondaryUpdate(derived().index(), unary_op, propagate);
	}

	template <class NodeType, class UnaryOp,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool>     = true,
	          std::enable_if_t<std::is_invocable_r_v<bool, UnaryOp, Index>, bool> = true>
	void voidRegionSecondaryUpdate(NodeType node, UnaryOp unary_op, bool propagate = true)
	{
		auto node_f = [this, unary_op](Index node) {
			voidRegionBlock(node.pos).secondarySet(node.offset, unary_op(node));
		};

		auto block_f = [this, unary_op](pos_t block) {
			for (std::size_t i{}; BF > i; ++i) {
				voidRegionBlock(block).secondarySet(i, unary_op(Index(block, i)));
			}
		};

		auto update_f = [this](Index node, pos_t children) {
			onPropagateChildren(node, children);
		};

		derived().recursLeaves(node, node_f, block_f, update_f, propagate);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                  Secondary Lookup                                   |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] bool voidRegionSecondaryContains() const
	{
		return voidRegionSecondaryContains(derived().index());
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool voidRegionSecondaryContains(NodeType node) const
	{
		Index n = derived().index(node);
		return voidRegionBlock(n.pos).secondaryContains(n.offset);
	}

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	VoidRegionMap() { onInitRoot(); }

	VoidRegionMap(VoidRegionMap const&) = default;

	VoidRegionMap(VoidRegionMap&&) = default;

	template <class Derived2, class Tree2>
	VoidRegionMap(VoidRegionMap<Derived2, Tree2> const& /* other */)
	{
	}

	template <class Derived2, class Tree2>
	VoidRegionMap(VoidRegionMap<Derived2, Tree2>&& /* other */)
	{
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Destructor                                      |
	|                                                                                     |
	**************************************************************************************/

	~VoidRegionMap() = default;

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	VoidRegionMap& operator=(VoidRegionMap const&) = default;

	VoidRegionMap& operator=(VoidRegionMap&&) = default;

	template <class Derived2, class Tree2>
	VoidRegionMap& operator=(VoidRegionMap<Derived2, Tree2> const& /* rhs */)
	{
		return *this;
	}

	template <class Derived2, class Tree2>
	VoidRegionMap& operator=(VoidRegionMap<Derived2, Tree2>&& /* rhs */)
	{
		return *this;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Swap                                         |
	|                                                                                     |
	**************************************************************************************/

	friend void swap(VoidRegionMap& /* lhs */, VoidRegionMap& /* rhs */) noexcept {}

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

	[[nodiscard]] VoidRegionBlock<BF>& voidRegionBlock(pos_t pos)
	{
		return derived().template data<VoidRegionBlock<BF>>(pos);
	}

	[[nodiscard]] VoidRegionBlock<BF> const& voidRegionBlock(pos_t pos) const
	{
		return derived().template data<VoidRegionBlock<BF>>(pos);
	}

	/**************************************************************************************
	|                                                                                     |
	|                              Functions Derived expects                              |
	|                                                                                     |
	**************************************************************************************/

	void onInitRoot() { voidRegionBlock(0) = {}; }

	void onInitChildren(Index node, pos_t children)
	{
		voidRegionBlock(children) = VoidRegionBlock(voidRegionBlock(node.pos), node.offset);
	}

	void onPropagateChildren(Index node, pos_t children)
	{
		auto&       vrb  = voidRegionBlock(node.pos);
		auto const& cvrb = voidRegionBlock(children);
		vrb.primarySet(node.offset, cvrb.primary_void_region.all());
		vrb.primaryContains(node.offset) = cvrb.primary_contains_void_region.any();
		vrb.secondarySet(node.offset, cvrb.secondary_void_region.all());
		vrb.secondaryContains(node.offset) = cvrb.secondary_contains_void_region.any();
	}

	[[nodiscard]] bool onIsPrunable(pos_t block) const
	{
		auto const& vrb = voidRegionBlock(block);
		return (vrb.primaryNone() || vrb.primaryAll()) &&
		       (vrb.secondaryNone() || vrb.secondaryAll());
	}

	void onPruneChildren(Index node, pos_t /* children */)
	{
		auto& vrb                          = voidRegionBlock(node.pos);
		vrb.primaryContains(node.offset)   = vrb.primary(node.offset);
		vrb.secondaryContains(node.offset) = vrb.secondary(node.offset);
	}

	[[nodiscard]] std::size_t onSerializedSize(
	    std::vector<std::pair<pos_t, BitSet<BF>>> const& /* nodes */,
	    std::size_t num_nodes) const
	{
		return num_nodes * (sizeof(VoidRegionBlock<BF>::primary_void_region) +
		                    sizeof(VoidRegionBlock<BF>::secondary_void_region));
	}

	void onRead(ReadBuffer& in, std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes)
	{
		for (auto [block, offset] : nodes) {
			auto& vrb = voidRegionBlock(block);

			BitSet<BF> pvr;
			BitSet<BF> svr;
			in.read(pvr);
			in.read(svr);
			vrb.primary_void_region = (vrb.primary_void_region & ~offset) | (pvr & offset);
			vrb.primary_contains_void_region = vrb.primary_void_region;
			vrb.secondary_void_region = (vrb.secondary_void_region & ~offset) | (svr & offset);
			vrb.secondary_contains_void_region = vrb.secondary_void_region;
		}
	}

	void onWrite(WriteBuffer&                                     out,
	             std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes) const
	{
		for (auto [block, _] : nodes) {
			out.write(voidRegionBlock(block).primary_void_region);
			out.write(voidRegionBlock(block).secondary_void_region);
		}
	}

	void onDotFile(std::ostream& out, Index node) const
	{
		if (voidRegion(node)) {
			out << "Void region: <font color='green'><b>true</b></font>";
		} else {
			out << "Void region: <font color='red'>false</font>";
		}
		if (voidRegionSecondary(node)) {
			out << "Void region 2nd: <font color='green'><b>true</b></font>";
		} else {
			out << "Void region 2nd: <font color='red'>false</font>";
		}
	}
};

template <std::size_t Dim, std::size_t BF>
struct map_block<VoidRegionMap, Dim, BF> {
	using type = VoidRegionBlock<BF>;
};
}  // namespace ufo

#endif  // UFO_MAP_VOID_REGION_MAP_HPP