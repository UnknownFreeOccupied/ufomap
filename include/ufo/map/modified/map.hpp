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

#ifndef UFO_MAP_MODIFIED_MAP_HPP
#define UFO_MAP_MODIFIED_MAP_HPP

// UFO
#include <ufo/container/tree/container.hpp>
#include <ufo/container/tree/tree.hpp>
#include <ufo/execution/execution.hpp>
#include <ufo/map/block.hpp>
#include <ufo/map/header.hpp>
#include <ufo/map/modified/block.hpp>
#include <ufo/map/modified/predicate.hpp>
#include <ufo/map/type.hpp>
#include <ufo/map/utility.hpp>
#include <ufo/utility/create_array.hpp>
#include <ufo/utility/enum.hpp>
#include <ufo/utility/type_traits.hpp>

// STL
#include <algorithm>
#include <ostream>
#include <type_traits>
#include <vector>

namespace ufo
{
template <class Derived, class Tree>
class ModifiedMap
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

	static constexpr MapType const Type = MapType::MODIFIED;

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

	/*!
	 * @brief Check if the root of the tree.
	 *
	 * @return Whether the root of the tree is in a modified state.
	 */
	[[nodiscard]] bool modified() const { return modified(derived().index()); }

	/*!
	 * @brief Check if a node of the tree is in a modified state (i.e., the node
	 * or one of its children has been modified).
	 *
	 * @param node The node to check.
	 * @return Whether the node is in a modified state.
	 */
	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool modified(NodeType node) const
	{
		Index n = derived().index(node);
		return modifiedBlock(n.pos)[n.offset];
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Modifiers                                      |
	|                                                                                     |
	**************************************************************************************/

	void modifiedSet(bool value) { return value ? modifiedSet() : modifiedReset(); }

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	void modifiedSet(NodeType node, bool value)
	{
		return value ? modifiedSet(node) : modifiedReset(node);
	}

	void modifiedSet() { modifiedSet(derived().index()); }

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	void modifiedSet(NodeType node)
	{
		auto node_f = [this](Index node) { modifiedBlock(node.pos).set(node.offset); };

		auto block_f = [this](pos_t block) { modifiedBlock(block) = true; };

		auto update_f = [this](Index node, pos_t children) {
			modifiedBlock(node.pos).set(node.offset, modifiedBlock(children).any());
		};

		derived().recursParentFirst(node, node_f, block_f, update_f,
		                            !modified(derived().parent(node)));
	}

	void modifiedReset() { modifiedReset(derived().index()); }

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	void modifiedReset(NodeType node)
	{
		auto node_f = [this](Index node) { modifiedBlock(node.pos).reset(node.offset); };

		auto block_f = [this](pos_t block) { modifiedBlock(block) = false; };

		auto update_f = [this](Index /* node */, pos_t /* children */) {};

		derived().recursParentFirst(node, node_f, block_f, update_f, false);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Propagate                                      |
	|                                                                                     |
	**************************************************************************************/

	/*!
	 * @brief Propagate modified information up the tree.
	 *
	 * @param reset_modified Whether propagated node's modified state should be reset
	 * @param prune Whether the tree should be pruned also
	 */
	void modifiedPropagate(MapType map_types = MapType::ALL, bool reset_modified = true,
	                       bool prune = true)
	{
		modifiedPropagate(derived().index(), map_types, reset_modified, prune);
	}

	void modifiedPropagate(pos_t block, MapType map_types = MapType::ALL,
	                       bool reset_modified = true, bool prune = true)
	{
		assert(derived().valid(block));

		auto m = reset_modified ? modifiedBlock(block).modified.exchange(0)
		                        : modifiedBlock(block).modified.load();

		if (0 == m) {
			return;
		}

		for (std::size_t i{}; BF > i; ++i) {
			auto n = Index(block, i);
			if (0u == (m & (1u << i)) || derived().isLeaf(n)) {
				continue;
			}

			auto c = derived().children(n);

			modifiedPropagate(c, map_types, reset_modified, prune);
			derived().onPropagateChildren(n, c, map_types);

			if (prune && derived().onIsPrunable(c)) {
				derived().pruneChildren(n, c);
			}
		}
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	void modifiedPropagate(NodeType node, MapType map_types = MapType::ALL,
	                       bool reset_modified = true, bool prune = true)
	{
		assert(derived().valid(node));

		auto n = derived().index(node);

		if (!modified(n)) {
			return;
		}

		if (reset_modified) {
			modifiedBlock(n.pos).reset(n.offset);
		}

		if (derived().isLeaf(n)) {
			return;
		}

		auto c = derived().children(n);

		modifiedPropagate(c, map_types, reset_modified, prune);
		derived().onPropagateChildren(n, c, map_types);

		if (prune && derived().onIsPrunable(c)) {
			derived().pruneChildren(n, c);
		}
	}

	template <
	    class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void modifiedPropagate(ExecutionPolicy&& policy, MapType map_types = MapType::ALL,
	                       bool reset_modified = true, bool prune = true)
	{
		modifiedPropagate(std::forward<ExecutionPolicy>(policy), derived().index(), map_types,
		                  reset_modified, prune);
	}

	template <
	    class ExecutionPolicy, class NodeType,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true,
	    std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool>           = true>
	void modifiedPropagate(ExecutionPolicy&& policy, NodeType node,
	                       MapType map_types = MapType::ALL, bool reset_modified = true,
	                       bool prune = true)
	{
		// TODO: Optimize
		modifiedPropagate(node, map_types, reset_modified, prune);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Create                                        |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add proper guards for the templates

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	Index modifiedCreate(NodeType node)
	{
		assert(derived().valid(node));

		Code code         = derived().code(node);
		auto wanted_depth = derived().depth(code);
		auto cur_node     = derived().index();
		auto cur_depth    = derived().depth();
		while (wanted_depth < cur_depth) {
			auto child = derived().createChild(cur_node, code.offset(--cur_depth));
			modifiedBlock(cur_node.pos).set(cur_node.offset);
			cur_node = child;
		}

		modifiedBlock(cur_node.pos).set(cur_node.offset);

		return cur_node;
	}

	template <class InputIt, class OutputIt>
	OutputIt modifiedCreate(InputIt first, InputIt last, OutputIt d_first)
	{
		Index node = derived().index();
		Code  code = derived().code();

		return std::transform(first, last, d_first, [this, &node, &code](auto const& x) {
			Code    e            = derived().code(x);
			depth_t wanted_depth = derived().depth(e);
			depth_t depth        = Code::depthWhereEqual(code, e);
			code                 = e;

			node = derived().ancestor(node, depth);
			for (; wanted_depth < depth; --depth) {
				auto child = derived().createChild(node, code.offset(depth - 1));
				//  NOTE: Important that modified is set after creating the
				//  children, otherwise all children will be modified
				modifiedBlock(node.pos).set(node.offset);
				node = child;
			}

			//  NOTE: Important that modified is set after creating the
			//  children, otherwise all children will be modified
			modifiedBlock(node.pos).set(node.offset);

			return node;
		});
	}

	template <class InputIt>
	std::vector<Index> modifiedCreate(InputIt first, InputIt last)
	{
		std::vector<Index> nodes;
		modifiedCreate(first, last, std::back_inserter(nodes));
		return nodes;
	}

	template <
	    class ExecutionPolicy, class RandomIt1, class RandomIt2,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	RandomIt2 modifiedCreate(ExecutionPolicy&& policy, RandomIt1 first, RandomIt1 last,
	                         RandomIt2 d_first)
	{
		std::cout << "Hellu from second modifiedCreate\n";
		return transform(std::forward<ExecutionPolicy>(policy), first, last, d_first,
		                 [this](auto const& x) {
			                 thread_local Index node = derived().index();

			                 // NOTE: `node` can be from last call to `create` (if the same
			                 // thread still persists), so we need to check if the node is valid
			                 // (i.e., has not been deleted). If it has been deleted, we set it
			                 // to the root node.
			                 // FIXME: Note sure if `valid` is thread safe
			                 node          = derived().valid(node) ? node : derived().index();
			                 Code cur_code = derived().code(node);

			                 Code    code         = derived().code(x);
			                 depth_t wanted_depth = derived().depth(code);
			                 depth_t depth        = Code::depthWhereEqual(code, cur_code);

			                 node = derived().ancestor(node, depth);
			                 for (; wanted_depth < depth; --depth) {
				                 auto child = derived().createChildThreadSafe(
				                     node, code.offset(depth - 1));
				                 //  NOTE: Important that modified is set after creating the
				                 //  children, otherwise all children will be modified
				                 modifiedBlock(node.pos).set(node.offset);
				                 node = child;
			                 }

			                 //  NOTE: Important that modified is set after creating the
			                 //  children, otherwise all children will be modified
			                 modifiedBlock(node.pos).set(node.offset);

			                 return node;
		                 });

		// return transform(std::forward<ExecutionPolicy>(policy), first, last, d_first,
		//                  [this](auto const& x) {
		// 	                 Code code         = derived().code(x);
		// 	                 auto wanted_depth = derived().depth(code);
		// 	                 auto cur_node     = derived().index();
		// 	                 auto cur_depth    = derived().depth();
		// 	                 while (wanted_depth < cur_depth) {
		// 		                 auto child =
		// 		                     derived().createChildThreadSafe(cur_node,
		// code.offset(--cur_depth)); modifiedBlock(cur_node.pos).set(cur_node.offset);
		// 		                 cur_node = child;
		// 	                 }
		// 	                 return cur_node;
		//                  });
	}

	template <
	    class ExecutionPolicy, class RandomIt,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	std::vector<Index> modifiedCreate(ExecutionPolicy&& policy, RandomIt first,
	                                  RandomIt last)
	{
		__block std::vector<Index> nodes(std::distance(first, last));
		modifiedCreate(std::forward<ExecutionPolicy>(policy), first, last, nodes.begin());
		return nodes;
	}

	template <class Range, class OutputIt,
	          std::enable_if_t<!Tree::template is_node_type_v<Range>, bool> = true>
	OutputIt modifiedCreate(Range const& r, OutputIt d_first)
	{
		using std::begin;
		using std::end;
		return modifiedCreate(begin(r), end(r), d_first);
	}

	template <class Range,
	          std::enable_if_t<!Tree::template is_node_type_v<Range>, bool> = true>
	std::vector<Index> modifiedCreate(Range const& r)
	{
		std::vector<Index> nodes;
		modifiedCreate(r, std::back_inserter(nodes));
		return nodes;
	}

	template <
	    class ExecutionPolicy, class Range, class RandomIt,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true,
	    std::enable_if_t<!Tree::template is_node_type_v<Range>, bool>             = true>
	RandomIt modifiedCreate(ExecutionPolicy&& policy, Range const& r, RandomIt d_first)
	{
		std::cout << "Hellu from modifiedCreate\n";
		using std::begin;
		using std::end;
		return modifiedCreate(std::forward<ExecutionPolicy>(policy), begin(r), end(r),
		                      d_first);
	}

	template <
	    class ExecutionPolicy, class Range,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true,
	    std::enable_if_t<!Tree::template is_node_type_v<Range>, bool>             = true>
	std::vector<Index> modifiedCreate(ExecutionPolicy&& policy, Range const& r)
	{
		using std::size;
		__block std::vector<Index> nodes(size(r));
		modifiedCreate(std::forward<ExecutionPolicy>(policy), r, nodes.begin());
		return nodes;
	}

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	ModifiedMap() { onInitRoot(); }

	ModifiedMap(ModifiedMap const&) = default;

	ModifiedMap(ModifiedMap&&) = default;

	template <class Derived2, class Tree2>
	ModifiedMap(ModifiedMap<Derived2, Tree2> const& /* other */)
	{
	}

	template <class Derived2, class Tree2>
	ModifiedMap(ModifiedMap<Derived2, Tree2>&& /* other */)
	{
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Destructor                                      |
	|                                                                                     |
	**************************************************************************************/

	~ModifiedMap() = default;

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	ModifiedMap& operator=(ModifiedMap const&) = default;

	ModifiedMap& operator=(ModifiedMap&&) = default;

	template <class Derived2, class Tree2>
	ModifiedMap& operator=(ModifiedMap<Derived2, Tree2> const& /* rhs */)
	{
		return *this;
	}

	template <class Derived2, class Tree2>
	ModifiedMap& operator=(ModifiedMap<Derived2, Tree2>&& /* rhs */)
	{
		return *this;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                         Swap                                        |
	|                                                                                     |
	**************************************************************************************/

	friend void swap(ModifiedMap& /* lhs */, ModifiedMap& /* rhs */) noexcept {}

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

	[[nodiscard]] ModifiedBlock<BF>& modifiedBlock(pos_t pos)
	{
		return derived().template data<ModifiedBlock<BF>>(pos);
	}

	[[nodiscard]] ModifiedBlock<BF> const& modifiedBlock(pos_t pos) const
	{
		return derived().template data<ModifiedBlock<BF>>(pos);
	}

	/**************************************************************************************
	|                                                                                     |
	|                              Functions Derived expects                              |
	|                                                                                     |
	**************************************************************************************/

	void onInitRoot() { modifiedBlock(0) = false; }

	void onInitChildren(Index node, pos_t children)
	{
		modifiedBlock(children) = modifiedBlock(node.pos)[node.offset];
	}

	void onPropagateChildren(Index /* node */, pos_t /* children */) {}

	[[nodiscard]] bool onIsPrunable(pos_t block) const
	{
		using T = typename ModifiedBlock<BF>::value_type;
		T m     = modifiedBlock(block).modified.load();
		return T(0) == m || ModifiedBlock<BF>::ALL_SET == m;
	}

	void onPruneChildren(Index /* node */, pos_t /* children */) {}

	[[nodiscard]] std::size_t onSerializedSize(
	    std::vector<std::pair<pos_t, BitSet<BF>>> const& /* nodes */,
	    std::size_t num_nodes) const
	{
		return num_nodes * sizeof(ModifiedBlock<BF>::modified);
	}

	void onRead(ReadBuffer& in, std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes)
	{
		for (auto [block, offset] : nodes) {
			auto& mb = modifiedBlock(block);

			typename ModifiedBlock<BF>::value_type m;
			in.read(m);
			mb.modified = m & offset.data();
		}
	}

	void onWrite(WriteBuffer&                                     out,
	             std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes) const
	{
		for (auto [block, _] : nodes) {
			out.write(modifiedBlock(block).modified.load());
		}
	}

	void onDotFile(std::ostream& out, Index node) const
	{
		if (modified(node)) {
			out << "Modified: <font color='green'><b>true</b></font>";
		} else {
			out << "Modified: <font color='red'>false</font>";
		}
	}
};

template <std::size_t Dim, std::size_t BF>
struct map_block<ModifiedMap, Dim, BF> {
	using type = ModifiedBlock<BF>;
};
}  // namespace ufo

#endif  // UFO_MAP_MODIFIED_MAP_HPP