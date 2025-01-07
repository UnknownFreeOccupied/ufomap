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

#ifndef UFO_TREE_TREE_HPP
#define UFO_TREE_TREE_HPP

// UFO
#include <ufo/container/tree/container.hpp>
#include <ufo/container/tree/tree.hpp>
#include <ufo/execution/execution.hpp>
#include <ufo/map/tree/block.hpp>
#include <ufo/map/tree/map_utility.hpp>
#include <ufo/map/type.hpp>
#include <ufo/utility/create_array.hpp>
#include <ufo/utility/enum.hpp>

// STL
#include <algorithm>
#include <array>
#include <atomic>
#include <mutex>
#include <ostream>
#include <vector>

namespace ufo
{
template <class Derived, std::size_t Dim, mu_t Utility, class... Blocks>
class MapTree
    : public Tree<
          Derived, Dim, true,
          MapBlock<Dim, std::size_t(1) << Dim, 0 != (MapUtility::WITH_CENTER & Utility)>,
          Blocks...>
{
 protected:
	using Base =
	    Tree<Derived, Dim, true,
	         MapBlock<Dim, std::size_t(1) << Dim, 0 != (MapUtility::WITH_CENTER & Utility)>,
	         Blocks...>;

	//
	// Friends
	//

	template <class, std::size_t, mu_t, class...>
	friend class MapTree;

	// First base friends :)
	friend Base;

	static constexpr auto const BF = Base::branchingFactor();

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                        Tags                                         |
	|                                                                                     |
	**************************************************************************************/

	// UFO stuff
	using Index    = typename Base::Index;
	using Node     = typename Base::Node;
	using Code     = typename Base::Code;
	using Key      = typename Base::Key;
	using Point    = typename Base::Point;
	using Coord    = typename Base::Coord;
	using Bounds   = typename Base::Bounds;
	using coord_t  = typename Base::coord_t;
	using depth_t  = typename Base::depth_t;
	using offset_t = typename Base::offset_t;
	using length_t = typename Base::length_t;
	using pos_t    = typename Base::pos_t;
	using mt_t     = std::underlying_type_t<MapType>;

	/**************************************************************************************
	|                                                                                     |
	|                                      Modified                                       |
	|                                                                                     |
	**************************************************************************************/

	/*!
	 * @brief Check if the tree is in a modified state (i.e., at least one node has been
	 * modified).
	 *
	 * @return Whether the tree is in a modified state.
	 */
	[[nodiscard]] bool modified() const { return modified(Base::index()); }

	/*!
	 * @brief Check if a node of the tree is in a modified state (i.e., the node
	 * or one of its children has been modified).
	 *
	 * @param node The node to check.
	 * @return Whether the node is in a modified state.
	 */
	template <class NodeType,
	          std::enable_if_t<Base::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool modified(NodeType node) const
	{
		Index n = Base::index(node);
		return Base::treeBlock(n).modified(n.offset);
	}

	/*!
	 * @brief Set every node to the modified state.
	 */
	Index modifiedSet() { return modifiedSet(Base::code()); }

	/*!
	 * @brief Set a node to the modified state.
	 *
	 * @details This will also set all the node's ancestors to the modified state as well as
	 * all its children.
	 *
	 * @param node The node to set to the modified state.
	 */
	template <class NodeType,
	          std::enable_if_t<Base::template is_node_type_v<NodeType>, bool> = true>
	Index modifiedSet(NodeType node)
	{
		Code c = Base::code(node);

		assert(Base::valid(c));

		auto node_f = [this](Index node) {
			// NOTE: the order of the checks in the if-statement is important,
			// modifiedSet needs to be first so that pure leaf nodes also get set modified
			if (0 == Base::treeBlock(node.pos).modifiedSet(node.offset) &&
			    0 < Base::depth(node)) {
				modifiedBlockAdd(node.pos);
			}
		};

		auto block_f = [this](pos_t block) {
			// NOTE: the order of the checks in the if-statement is important,
			// modifiedSet needs to be first so that pure leaf nodes also get set modified
			if (0 == Base::treeBlock(block).modifiedFill() && 0 < Base::depth(block)) {
				modifiedBlockAdd(block);
			}
		};

		auto wanted_depth = Base::depth(c);
		auto cur_node     = Base::index();
		auto cur_depth    = Base::depth();
		auto prev_node    = cur_node;
		while (wanted_depth < cur_depth) {
			cur_node = Base::createChild(cur_node, c.offset(--cur_depth));
			node_f(prev_node);
			prev_node = cur_node;
		}
		Base::recursParentFirst(cur_node, node_f, block_f);

		return cur_node;
	}

	/*!
	 * @brief Set the nodes in the range [ `first`, `last` ) to the modified state.
	 *
	 * @details This will also set all the ancestors to the modified state as well as
	 * all children.
	 *
	 * @param first,last The range of nodes.
	 * @return The nodes.
	 */
	template <class InputIt>
	std::vector<Index> modifiedSet(InputIt first, InputIt last)
	{
		// TODO: Implement
		return modifiedSet(execution::seq, first, last);
	}

	/*!
	 * @brief Set the nodes in the range [ `first`, `last` ) to the modified state.
	 *
	 * @details This will also set all the ancestors to the modified state as well as
	 * all children.
	 *
	 * @param policy The execution policy to use.
	 * @param first,last The range of nodes.
	 * @return The nodes.
	 */
	template <
	    class ExecutionPolicy, class RandomIt,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	std::vector<Index> modifiedSet(ExecutionPolicy&& policy, RandomIt first, RandomIt last)
	{
		std::vector<Index> nodes = Base::create(policy, first, last);

		// Add root node once
		auto root = Base::index();
		if (0 == Base::treeBlock(root.pos).modifiedSet(root.offset)) {
			auto depth                                    = Base::depth();
			auto p                                        = modified_block_[depth].create();
			modified_block_[depth].template get<pos_t>(p) = root.pos;
		}

		if constexpr (execution::is_stl_v<ExecutionPolicy>) {
			// TODO: Implement

			auto node_f = [this](Index node) {
				auto depth = Base::depth(node);

				auto idx = std::uint64_t(1) << node.offset;
				auto old = Base::treeBlock(node.pos).modifiedSet(node.offset);

				if (0 == depth || 0 != (old & idx)) {
					return;
				}

				if (0 == old) {
					pos_t p = modified_block_[depth].createThreadSafe();
					modified_block_[depth].template get<pos_t>(p) = node.pos;
				}
			};

			// TODO: Implement
			// auto block_f = [this, &pos](pos_t block) {
			// 	if (0 == Base::treeBlock(block).modifiedFill() && 0 < Base::depth(block)) {
			// 		modifiedBlockAdd(block);
			// 	}
			// };

			auto fun = [this, root_depth = Base::depth(), node_f /* , block_f */](Index node) {
				auto depth = Base::depth(node);

				if (0 == depth) {
					auto idx = std::uint64_t(1) << node.offset;
					auto old = Base::treeBlock(node.pos).modifiedSet(node.offset);
					if (old & idx) {
						return;
					}
				} else if (!modified(node)) {
					Base::recursParentFirst(node, node_f /* , block_f */);
				} else {
					return;
				}

				for (++depth; root_depth > depth; ++depth) {
					node = Base::parent(node);

					auto idx = std::uint64_t(1) << node.offset;
					auto old = Base::treeBlock(node.pos).modifiedSet(node.offset);
					if (0 != (old & idx)) {
						return;
					}

					if (0 == old) {
						pos_t p = modified_block_[depth].createThreadSafe();
						modified_block_[depth].template get<pos_t>(p) = node.pos;
					}
				}
			};

			std::for_each(execution::toSTL(policy), nodes.begin(), nodes.end(), fun);
		}
#if defined(UFO_PAR_GCD)
		else if constexpr (execution::is_gcd_v<ExecutionPolicy>) {
			// TODO: Implement
			static_assert(dependent_false_v<ExecutionPolicy>,
			              "Not implemented for the execution policy");
		}
#endif
#if defined(UFO_PAR_TBB)
		else if constexpr (execution::is_tbb_v<ExecutionPolicy>) {
			// TODO: Implement
			static_assert(dependent_false_v<ExecutionPolicy>,
			              "Not implemented for the execution policy");
		}
#endif
		else if constexpr (execution::is_omp_v<ExecutionPolicy>) {
			// TODO: Implement
			static_assert(dependent_false_v<ExecutionPolicy>,
			              "Not implemented for the execution policy");
		}

		return nodes;
	}

	/*!
	 * @brief Set the nodes in the range to the modified state.
	 *
	 * @details This will also set all the ancestors to the modified state as well as
	 * all children.
	 *
	 * @param r The range of nodes.
	 * @return The nodes.
	 */
	template <class Range,
	          // FIXME: Change to is_range_type_v
	          std::enable_if_t<not Base::template is_node_type_v<Range>, bool> = true>
	std::vector<Index> modifiedSet(Range const& r)
	{
		using std::begin;
		using std::end;
		return modifiedSet(begin(r), end(r));
	}

	/*!
	 * @brief Set the nodes in the range to the modified state.
	 *
	 * @details This will also set all the ancestors to the modified state as well as
	 * all children.
	 *
	 * @param policy The execution policy to use.
	 * @param r The range of nodes.
	 * @return The nodes.
	 */
	template <
	    class ExecutionPolicy, class Range,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true,
	    // FIXME: Change to is_range_type_v
	    std::enable_if_t<not Base::template is_node_type_v<Range>, bool> = true>
	std::vector<Index> modifiedSet(ExecutionPolicy&& policy, Range const& r)
	{
		using std::begin;
		using std::end;
		return modifiedSet(std::forward<ExecutionPolicy>(policy), begin(r), end(r));
	}

	/*!
	 * @brief Reset every node from the modified state.
	 */
	void modifiedReset()
	{
		// TODO: Implement
		modifiedReset(execution::seq);
	}

	/*!
	 * @brief Reset every node from the modified state.
	 *
	 * @param policy The execution policy to use.
	 */
	template <
	    class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void modifiedReset(ExecutionPolicy&& policy)
	{
		if constexpr (execution::is_stl_v<ExecutionPolicy>) {
			for (auto& mb : modified_block_) {
				std::for_each(execution::toSTL(policy), mb.template begin<pos_t>(),
				              mb.template end<pos_t>(),
				              [this](pos_t block) { Base::treeBlock(block).modifiedClear(); });
				mb.clear();
			}
		}
#if defined(UFO_PAR_GCD)
		else if constexpr (execution::is_gcd_v<ExecutionPolicy>) {
			// TODO: Implement
			// for (auto& mb : modified_block_) {
			// 	dispatch_apply(mb.size(), dispatch_get_global_queue(0, 0), ^(std::size_t row) {
			// 		for (std::size_t col{}; col < cols; ++col) {
			// 			rays(row, col) = fun(row, col);
			// 		}
			// 	});
			// }
			static_assert(dependent_false_v<ExecutionPolicy>,
			              "Not implemented for the execution policy");
		}
#endif
#if defined(UFO_PAR_TBB)
		else if constexpr (execution::is_tbb_v<ExecutionPolicy>) {
			// TODO: Implement
			static_assert(dependent_false_v<ExecutionPolicy>,
			              "Not implemented for the execution policy");
		}
#endif
		else if constexpr (execution::is_omp_v<ExecutionPolicy>) {
			// TODO: Implement

			for (auto& mb : modified_block_) {
#pragma omp parallel for
				for (pos_t block : mb.template iter<pos_t>()) {
					Base::treeBlock(block).modifiedClear();
				}
				mb.clear();
			}

			static_assert(dependent_false_v<ExecutionPolicy>,
			              "Not implemented for the execution policy");
		} else {
			static_assert(dependent_false_v<ExecutionPolicy>,
			              "Not implemented for the execution policy");
		}
	}

	/*!
	 * @brief Reset the node and all its children from the modified state.
	 *
	 * @details The ancestors of the node will also be reset from the modified state if they
	 * have no children in the modified state anymore.
	 *
	 * @param node The node to reset from the modified state.
	 */
	void modifiedReset(Index node) { modifiedReset(Base::code(node)); }

	/*!
	 * @brief Reset the node and all its children from the modified state.
	 *
	 * @details The ancestors of the node will also be reset from the modified state if they
	 * have no children in the modified state anymore.
	 *
	 * @param node The node to reset from the modified state.
	 */
	void modifiedReset(Node node) { modifiedReset(Base::code(node)); }

	/*!
	 * @brief Reset the node and all its children from the modified state.
	 *
	 * @details The ancestors of the node will also be reset from the modified state if they
	 * have no children in the modified state anymore.
	 *
	 * @param node The node to reset from the modified state.
	 */
	void modifiedReset(Code node)
	{
		assert(Base::valid(node));

		auto node_f = [this](Index node) {
			if (noneModified(node.pos)) {
				return;
			}

			Base::treeBlock(node.pos).modifiedReset(node.offset);
			if (noneModified(node.pos)) {
				modifiedBlockErase(node.pos);
			}
		};

		auto block_f = [this](pos_t block) {
			if (noneModified(block)) {
				return;
			}

			Base::treeBlock(block).modifiedClear();
			modifiedBlockErase(block);
		};

		auto                                         wanted_depth = Base::depth(node);
		auto                                         cur_node     = Base::index();
		auto                                         root_depth   = Base::depth();
		auto                                         cur_depth    = root_depth;
		std::array<Index, Base::maxNumDepthLevels()> trail;
		while (wanted_depth < cur_depth) {
			if (!modified(cur_node)) {
				return;
			}

			trail[cur_depth] = cur_node;
			cur_node         = Base::createChild(cur_node, node.offset(--cur_depth));
		}

		if (!modified(cur_node)) {
			return;
		}

		Base::recurs(cur_node, node_f, block_f,
		             [this](Index node) { return !modified(node); });

		while (cur_depth < root_depth) {
			cur_node = trail[++cur_depth];
			Base::treeBlock(cur_node.pos).modifiedReset(cur_node.offset);
			if (anyModified(cur_node.pos)) {
				return;
			}
			modifiedBlockErase(cur_node.pos);
		}
	}

	/*!
	 * @brief Reset the node and all its children from the modified state.
	 *
	 * @details The ancestors of the node will also be reset from the modified state if they
	 * have no children in the modified state anymore.
	 *
	 * @param node The node to reset from the modified state.
	 */
	void modifiedReset(Key node) { modifiedReset(Base::code(node)); }

	/*!
	 * @brief Reset the node and all its children from the modified state.
	 *
	 * @details The ancestors of the node will also be reset from the modified state if they
	 * have no children in the modified state anymore.
	 *
	 * @param node The node to reset from the modified state.
	 */
	void modifiedReset(Coord node) { modifiedReset(Base::code(node)); }

	/**************************************************************************************
	|                                                                                     |
	|                                      Propagate                                      |
	|                                                                                     |
	**************************************************************************************/

	void propagate(mt_t map_types = to_underlying(MapType::ALL), bool prune = false)
	{
		propagate(Base::index(), false, map_types, prune);
	}

	void propagate(Index node, bool to_root = true,
	               mt_t map_types = to_underlying(MapType::ALL), bool prune = false)
	{
		if (to_root) {
			return propagate(Base::code(node), to_root, map_types, prune);
		}

		Base::recurs(
		    node,
		    [this, map_types, prune](Index node) {
			    propagateChildren(node, map_types, prune);
		    },
		    [](Index node) { return false; });
	}

	void propagate(Node node, bool to_root = true,
	               mt_t map_types = to_underlying(MapType::ALL), bool prune = false)
	{
		if (to_root) {
			propagate(Base::code(node), to_root, map_types, prune);
		} else {
			propagate(Base::index(node), to_root, map_types, prune);
		}
	}

	void propagate(Code node, bool to_root = true,
	               mt_t map_types = to_underlying(MapType::ALL), bool prune = false)
	{
		if (!to_root) {
			return propagate(Base::index(node), to_root, map_types, prune);
		}

		auto                                         wanted_depth = Base::depth(node);
		auto                                         cur_node     = Base::index();
		auto                                         root_depth   = Base::depth();
		auto                                         cur_depth    = root_depth;
		std::array<Index, Base::maxNumDepthLevels()> trail;
		while (wanted_depth < cur_depth) {
			trail[cur_depth] = cur_node;
			if (Base::isLeaf(cur_node)) {
				break;
			}
			cur_node = Base::child(cur_node, node.offset(--cur_depth));
		}

		propagate(cur_node, false, map_types, prune);

		while (cur_depth < root_depth) {
			propagateChildren(trail[++cur_depth], map_types, prune);
		}
	}

	void propagate(Key node, bool to_root = true,
	               mt_t map_types = to_underlying(MapType::ALL), bool prune = false)
	{
		propagate(Base::code(node), to_root, map_types, prune);
	}

	void propagate(Coord node, bool to_root = true,
	               mt_t map_types = to_underlying(MapType::ALL), bool prune = false)
	{
		propagate(Base::code(node), to_root, map_types, prune);
	}

	/*!
	 * @brief Propagate modified information up the tree.
	 *
	 * @param reset_modified Whether propagated node's modified state should be reset
	 * @param prune Whether the tree should be pruned also
	 */
	void propagateModified(mt_t map_types      = to_underlying(MapType::ALL),
	                       bool reset_modified = true, bool prune = false)
	{
		// TODO: Implement
		propagateModified(execution::seq, map_types, reset_modified, prune);
	}

	template <
	    class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void propagateModified(ExecutionPolicy&& policy,
	                       mt_t              map_types = to_underlying(MapType::ALL),
	                       bool reset_modified = true, bool prune = false)
	{
		auto fun = [this, map_types, reset_modified, prune](pos_t block) {
			for (std::size_t i{}; BF > i; ++i) {
				Index node(block, i);
				if (Base::isLeaf(node) || !modified(node)) {
					continue;
				}
				propagateChildren(node, map_types, prune);
			}
			if (reset_modified) {
				Base::treeBlock(block).modifiedClear();
			}
		};

		if constexpr (execution::is_stl_v<ExecutionPolicy>) {
			for (std::size_t i = 1; modified_block_.size() > i; ++i) {
				std::for_each(execution::toSTL(policy),
				              modified_block_[i].template begin<pos_t>(),
				              modified_block_[i].template end<pos_t>(), fun);
			}

			if (reset_modified) {
				for (auto& mb : modified_block_) {
					mb.clear();
				}
			}
		}
#if defined(UFO_PAR_GCD)
		else if constexpr (execution::is_gcd_v<ExecutionPolicy>) {
			// TODO: Implement
			static_assert(dependent_false_v<ExecutionPolicy>,
			              "Not implemented for the execution policy");
		}
#endif
#if defined(UFO_PAR_TBB)
		else if constexpr (execution::is_tbb_v<ExecutionPolicy>) {
			// TODO: Implement
			static_assert(dependent_false_v<ExecutionPolicy>,
			              "Not implemented for the execution policy");
		}
#endif
		else if constexpr (execution::is_omp_v<ExecutionPolicy>) {
			for (auto& mb : modified_block_) {
				if constexpr (execution::is_seq_v<ExecutionPolicy>) {
					for (pos_t block : mb.template iter<pos_t>()) {
						fun(block);
					}
				} else if constexpr (execution::is_unseq_v<ExecutionPolicy>) {
#pragma omp simd
					for (pos_t block : mb.template iter<pos_t>()) {
						fun(block);
					}
				} else if constexpr (execution::is_par_v<ExecutionPolicy>) {
#pragma omp parallel for
					for (pos_t block : mb.template iter<pos_t>()) {
						fun(block);
					}
				} else if constexpr (execution::is_par_unseq_v<ExecutionPolicy>) {
#pragma omp parallel for simd
					for (pos_t block : mb.template iter<pos_t>()) {
						fun(block);
					}
				}

				if (reset_modified) {
					mb.clear();
				}
			}
		} else {
			static_assert(dependent_false_v<ExecutionPolicy>,
			              "Not implemented for the execution policy");
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Prune                                        |
	|                                                                                     |
	**************************************************************************************/

	/*!
	 * @brief
	 *
	 * @note Setting `lossy` to `true` is the same as calling `eraseChildren()`.
	 *
	 * @param lossy
	 */
	void prune(bool lossy = false) { prune(Base::index(), lossy); }

	/*!
	 * @brief
	 *
	 * @note Setting `lossy` to `true` is the same as calling `eraseChildren(node)`.
	 *
	 * @param node
	 * @param lossy
	 */
	void prune(Index node, bool lossy = false)
	{
		if (lossy) {
			Base::eraseChildren(node);
			return;
		}

		if (Base::isLeaf(node)) {
			return;
		}

		auto node_f = [this](Index node) {
			auto children = Base::children(node);
			if (derived().isPrunable(children)) {
				Base::eraseChildren(node);
			}
		};

		recurs(node, node_f, [](Index) { return false; });
	}

	void prune(Node node, bool lossy = false) { prune(Base::index(node), lossy); }

	void prune(Code node, bool lossy = false) { prune(Base::index(node), lossy); }

	void prune(Key node, bool lossy = false) { prune(Base::code(node), lossy); }

	void prune(Coord node, bool lossy = false) { prune(Base::code(node), lossy); }

	/**************************************************************************************
	|                                                                                     |
	|                                         I/O                                         |
	|                                                                                     |
	**************************************************************************************/

	// [[nodiscard]] static bool isMap(std::filesystem::path const& file)
	// {
	// 	std::ifstream f = openFileIn(file);
	// 	return isMap(f);
	// }

	// [[nodiscard]] static bool isMap(std::istream& in)
	// {
	// 	auto        g = in.tellg();
	// 	std::string line;
	// 	std::getline(in, line);
	// 	in.seekg(g);
	// 	// TODO: Implement
	// }

	// [[nodiscard]] static bool isMap(ReadBuffer& in)
	// {
	// 	// TODO: Implement
	// }

	// void read(std::filesystem::path const& file, mt_t map_types =
	// to_underlying(MapType::ALL),
	//           bool propagate = true)
	// {
	// 	std::ifstream f = openFileIn(file);
	// 	read(f, map_types, propagate);
	// }

	// void read(std::istream& in, mt_t map_types = to_underlying(MapType::ALL), bool
	// propagate = true)
	// {
	// 	readData(in, readHeader(in), map_types, propagate);
	// }

	// void read(ReadBuffer& in, mt_t map_types = to_underlying(MapType::ALL), bool
	// propagate = true)
	// {
	// 	readData(in, readHeader(in), map_types, propagate);
	// }

	// [[nodiscard]] FileHeader readHeader(std::filesystem::path const& file) const
	// {
	// 	return {file};
	// }

	// [[nodiscard]] FileHeader readHeader(std::istream& in) const { return {in}; }

	// [[nodiscard]] FileHeader readHeader(ReadBuffer& in) const { return {in}; }

	// void readData(std::istream& in, FileHeader const& header, mt_t map_types =
	// to_underlying(MapType::ALL),
	//               bool propagate = true)
	// {
	// 	// TODO: Implement

	// 	if (treeType() != header.tree_type) {
	// 		// TODO: Make better error message
	// 		throw std::runtime_error("Trying to read wrong tree type");
	// 	}

	// 	if (size() != header.leaf_size || depthLevels() != header.depth_levels) {
	// 		clear(header.leaf_size, header.depth_levels);
	// 	}

	// 	readNodes(in, retrieveNodes(in), header.compressed, map_types);

	// 	if (propagate) {
	// 		propagateModified();
	// 	}
	// }

	// void readData(ReadBuffer& in, FileHeader const& header, mt_t map_types =
	// to_underlying(MapType::ALL),
	//               bool propagate = true)
	// {
	// 	// TODO: Implement

	// 	if (treeType() != header.tree_type) {
	// 		// TODO: Make better error message
	// 		throw std::runtime_error("Trying to read wrong tree type");
	// 	}

	// 	if (size() != header.leaf_size || depthLevels() != header.depth_levels) {
	// 		clear(header.leaf_size, header.depth_levels);
	// 	}

	// 	readNodes(in, retrieveNodes(in), header.compressed, map_types);

	// 	if (propagate) {
	// 		propagateModified();
	// 	}
	// }

	// [[nodiscard]] FileHeader header() const
	// {
	// 	// TODO: Implement
	// }

	// template <class Pred, CompressionAlgorithm CAlg,
	//           std::enable_if_t<pred::is_pred_v<Pred, Map, Node>, bool> = true>
	// void write(
	//     std::filesystem::path const& file, Pred const& pred = pred::True(),
	//     Compressor<CAlg> const& compressor = Compressor<CompressionAlgorithm::NONE>(),
	//     mt_t                    map_types  = to_underlying(MapType::ALL)) const
	// {
	// 	std::ofstream f = openFileOut(file);
	// 	write(f, pred, compressor, map_types);
	// }

	// template <class Pred, CompressionAlgorithm CAlg,
	//           std::enable_if_t<pred::is_pred_v<Pred, Map, Node>, bool> = true>
	// void write(
	//     std::ostream& out, Pred const& pred = pred::True(),
	//     Compressor<CAlg> const& compressor = Compressor<CompressionAlgorithm::NONE>(),
	//     mt_t                    map_types  = to_underlying(MapType::ALL)) const
	// {
	// 	// TODO: Implement
	// 	auto [tree, blocks] = retrieveTreeAndNodes<Node>(pred);
	// 	writeTreeAndBlocks(out, tree, blocks, compressor, map_types);
	// }

	// template <class Pred, CompressionAlgorithm CAlg,
	//           std::enable_if_t<pred::is_pred_v<Pred, Map, Node>, bool> = true>
	// void write(
	//     WriteBuffer& out, Pred const& pred = pred::True(),
	//     Compressor<CAlg> const& compressor = Compressor<CompressionAlgorithm::NONE>(),
	//     mt_t                    map_types  = to_underlying(MapType::ALL)) const
	// {
	// 	// TODO: Implement
	// 	auto [tree, blocks] = retrieveTreeAndNodes<Node>(pred);
	// 	writeTreeAndBlocks(out, tree, blocks, compressor, map_types);
	// }

	// template <class Pred, CompressionAlgorithm CAlg,
	//           std::enable_if_t<pred::is_pred_v<Pred, Map, Node>, bool> = true>
	// Buffer write(
	//     Pred const&             pred       = pred::True(),
	//     Compressor<CAlg> const& compressor = Compressor<CompressionAlgorithm::NONE>(),
	//     mt_t                    map_types  = to_underlying(MapType::ALL)) const
	// {
	// 	Buffer buffer;
	// 	write(buffer, pred, compressor, map_types);
	// 	return buffer;
	// }

	// template <CompressionAlgorithm CAlg>
	// void writeModified(
	//     std::filesystem::path const& file,
	//     Compressor<CAlg> const&      compressor =
	//     Compressor<CompressionAlgorithm::NONE>(), mt_t                         map_types
	//     = to_underlying(MapType::ALL)) const
	// {
	// 	std::ofstream f = openFileOut(file);
	// 	writeModified(f, compressor, map_types);
	// }

	// template <CompressionAlgorithm CAlg>
	// void writeModified(
	//     std::ostream&           out,
	//     Compressor<CAlg> const& compressor = Compressor<CompressionAlgorithm::NONE>(),
	//     mt_t                    map_types  = to_underlying(MapType::ALL)) const
	// {
	// 	auto [tree, blocks] = retrieveModifiedTreeAndNodes();
	// 	writeTreeAndBlocks(out, tree, blocks, compressor, map_types);
	// }

	// template <CompressionAlgorithm CAlg>
	// void writeModified(
	//     WriteBuffer&            out,
	//     Compressor<CAlg> const& compressor = Compressor<CompressionAlgorithm::NONE>(),
	//     mt_t                    map_types  = to_underlying(MapType::ALL)) const
	// {
	// 	auto [tree, blocks] = retrieveModifiedTreeAndNodes();
	// 	writeTreeAndBlocks(out, tree, blocks, compressor, map_types);
	// }

	// template <CompressionAlgorithm CAlg>
	// Buffer writeModified(
	//     Compressor<CAlg> const& compressor = Compressor<CompressionAlgorithm::NONE>(),
	//     mt_t                    map_types  = to_underlying(MapType::ALL)) const
	// {
	// 	Buffer buffer;
	// 	writeModified(buffer, compressor, map_types);
	// 	return buffer;
	// }

	/**************************************************************************************
	|                                                                                     |
	|                                     Statistics                                      |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Implement

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	MapTree() = default;

	MapTree(length_t leaf_node_length, depth_t num_depth_levels)
	    : Base(leaf_node_length, num_depth_levels)
	{
	}

	MapTree(MapTree const&) = default;
	MapTree(MapTree&&)      = default;

	template <class Derived2, mu_t Utility2, class... Blocks2>
	MapTree(MapTree<Derived2, Dim, Utility2, Blocks2...> const& other)
	    : Base(static_cast<Base const&>(other)), modified_block_(other.modified_block_)
	{
	}

	template <class Derived2, mu_t Utility2, class... Blocks2>
	MapTree(MapTree<Derived2, Dim, Utility2, Blocks2...>&& other)
	    : Base(static_cast<Base&&>(other))
	    , modified_block_(std::move(other.modified_block_))
	{
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Destructor                                      |
	|                                                                                     |
	**************************************************************************************/

	~MapTree() = default;

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	MapTree& operator=(MapTree const&) = default;

	MapTree& operator=(MapTree&&) = default;

	template <class Derived2, mu_t Utility2, class... Blocks2>
	MapTree& operator=(MapTree<Derived2, Dim, Utility2, Blocks2...> const& rhs)
	{
		Base::operator=(static_cast<Base const&>(rhs));
		modified_block_ = rhs.modified_block_;

		return *this;
	}

	template <class Derived2, mu_t Utility2, class... Blocks2>
	MapTree& operator=(MapTree<Derived2, Dim, Utility2, Blocks2...>&& rhs)
	{
		Base::operator=(static_cast<Base&&>(rhs));
		modified_block_ = std::move(rhs.modified_block_);

		return *this;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                         Swap                                        |
	|                                                                                     |
	**************************************************************************************/

	void swap(MapTree& other)
	{
		using std::swap;
		static_cast<Base&>(*this).swap(static_cast<Base&>(other));
		swap(modified_block_, other.modified_block_);
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
	|                              Functions Derived Expects                              |
	|                                                                                     |
	**************************************************************************************/

	/*!
	 * @brief Init the root.
	 *
	 * @details Since "Tree" in ufocontainer inits it, we do not need to do anything here.
	 *
	 */
	void onInitRoot() {}

	/*!
	 * @brief Inits the `children` of `node`.
	 *
	 * @details Since "Tree" in ufocontainer does everything needed, we do not need to do
	 * anything here.
	 *
	 * @param node The node to init the children for
	 * @param children The children to init
	 */
	void onInitChildren(Index /* node */, pos_t /* children */) {}

	/*!
	 * @brief Prunes the `children` of `node`.
	 *
	 * @details Since "Tree" in ufocontainer does everything needed, we do not need to do
	 * anything here.
	 *
	 * @param node The node to prune the children of
	 * @param children The children to prune
	 */
	void onPruneChildren(Index /* node */, pos_t /* children */) {}

	/**************************************************************************************
	|                                                                                     |
	|                                     Update node                                     |
	|                                                                                     |
	**************************************************************************************/

	void propagateChildren(Index node, mt_t map_types, bool prune)
	{
		derived().onPropagateChildren(node, Base::children(node), map_types);
		// TODO: Implement
		// if (prune) {
		// 	this->prune(node);
		// }
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Recurs                                        |
	|                                                                                     |
	**************************************************************************************/

	using Base::recursParentFirst;

	using Base::recursChildrenFirst;

	using Base::recursLeaves;

	template <class NodeFun, class UpdateFun,
	          std::enable_if_t<std::is_invocable_v<NodeFun, Index>, bool>          = true,
	          std::enable_if_t<std::is_invocable_v<UpdateFun, Index, pos_t>, bool> = true>
	void recursLeaves(Code code, NodeFun node_f, UpdateFun update_f, bool propagate)
	{
		assert(Base::valid(code));

		if (propagate) {
			// Propagate up to root
			auto trail = Base::createTrail(code);

			depth_t depth = Base::depth(code);
			auto    node  = trail[depth];

			Base::recursLeaves(node, node_f, update_f);

			depth_t root_depth = Base::depth();
			for (++depth; root_depth >= depth; ++depth) {
				node = trail[depth];
				update_f(node, Base::children(node));
			}
		} else {
			// Propagate only to code
			auto node = modifiedSet(code);
			Base::recursLeaves(node, node_f, update_f);
		}
	}

	template <class NodeFun, class BlockFun, class UpdateFun,
	          std::enable_if_t<std::is_invocable_v<NodeFun, Index>, bool>          = true,
	          std::enable_if_t<std::is_invocable_v<BlockFun, pos_t>, bool>         = true,
	          std::enable_if_t<std::is_invocable_v<UpdateFun, Index, pos_t>, bool> = true>
	void recursLeaves(Code code, NodeFun node_f, BlockFun block_f, UpdateFun update_f,
	                  bool propagate)
	{
		assert(Base::valid(code));

		if (propagate) {
			// Propagate up to root
			auto trail = Base::createTrail(code);

			depth_t depth = Base::depth(code);
			auto    node  = trail[depth];

			Base::recursLeaves(node, node_f, block_f, update_f);

			depth_t root_depth = Base::depth();
			for (++depth; root_depth >= depth; ++depth) {
				node = trail[depth];
				update_f(node, Base::children(node));
			}

		} else {
			// Propagate only to code
			auto node = modifiedSet(code);
			Base::recursLeaves(node, node_f, block_f, update_f);
		}
	}

	template <class NodeFun, class BlockFun,
	          std::enable_if_t<std::is_invocable_v<NodeFun, Index>, bool> = true>
	void recursParentFirst(Code code, NodeFun node_f)
	{
		assert(Base::valid(code));

		// Propagate only to code
		auto node = modifiedSet(code);
		Base::recursParentFirst(node, node_f);
	}

	template <class NodeFun, class BlockFun,
	          std::enable_if_t<std::is_invocable_v<NodeFun, Index>, bool>  = true,
	          std::enable_if_t<std::is_invocable_v<BlockFun, pos_t>, bool> = true>
	void recursParentFirst(Code code, NodeFun node_f, BlockFun block_f)
	{
		assert(Base::valid(code));

		// Propagate only to code
		auto node = modifiedSet(code);
		Base::recursParentFirst(node, node_f, block_f);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Modified                                       |
	|                                                                                     |
	**************************************************************************************/

	void modifiedBlockAdd(pos_t block, depth_t depth)
	{
		assert(Base::valid(block));
		assert(Base::depth(block) == depth);
		// TODO: assert(0 < depth);

		pos_t p                                       = modified_block_[depth].create();
		modified_block_[depth].template get<pos_t>(p) = block;
	}

	void modifiedBlockAdd(pos_t block) { modifiedBlockAdd(block, Base::depth(block)); }

	void modifiedBlockErase(pos_t block, depth_t depth)
	{
		assert(Base::valid(block));
		assert(Base::depth(block) == depth);

		auto& vec = modified_block_[depth];
		vec.erase(std::remove(vec.begin(), vec.end(), block), vec.end());
	}

	void modifiedBlockErase(pos_t block) { modifiedBlockErase(block, Base::depth(block)); }

	/**************************************************************************************
	|                                                                                     |
	|                                         I/O                                         |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Implement

	/**************************************************************************************
	|                                                                                     |
	|                                      Dot file                                       |
	|                                                                                     |
	**************************************************************************************/

	void onDotFileInfo(std::ostream& out, Index node) const
	{
		if (modified(node)) {
			out << "Modified: <font color='green'><b>true</b></font>";
		} else {
			out << "Modified: <font color='red'>false</font>";
		}
		out << "<br/>Center: " << Base::center(node);
		out << "<br/>Depth: " << Base::depth(node) << " | Length: " << Base::length(node);
	}

 private:
	// TODO: Need to remove from this when a block gets erased
	// TODO: Do not need to add blocks at depth level 0
	std::array<TreeContainer<pos_t>, Base::maxNumDepthLevels()> modified_block_;

	std::mutex modified_mutex_;
};
}  // namespace ufo

#endif  // UFO_TREE_TREE_HPP