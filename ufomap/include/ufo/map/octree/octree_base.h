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

#ifndef UFO_MAP_OCTREE_BASE_H
#define UFO_MAP_OCTREE_BASE_H

// UFO
#include <ufo/algorithm/algorithm.h>
#include <ufo/map/code.h>
#include <ufo/map/io.h>
#include <ufo/map/key.h>
#include <ufo/map/node.h>
#include <ufo/map/octree/octree_iterator.h>
#include <ufo/map/octree/octree_node.h>
#include <ufo/map/octree/octree_predicate.h>
#include <ufo/map/point.h>
#include <ufo/map/predicate/predicates.h>
#include <ufo/map/predicate/spatial.h>
#include <ufo/map/types.h>
#include <ufo/math/util.h>
#include <ufo/util/iterator_wrapper.h>
#include <ufo/util/type_traits.h>

// STL
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <deque>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace ufo::map
{
// Utilizing curiously recurring template pattern (CRTP)
template <class Derived, class Data, class InnerData = void, bool ReuseNodes = false,
          bool LockLess = false, bool CountNodes = true>
class OctreeBase
{
 private:
	using LeafNode = OctreeLeafNode<Data>;
	using InnerNode = OctreeInnerNode<LeafNode, InnerData>;
	using typename InnerNode::InnerNodeBlock;
	using typename InnerNode::LeafNodeBlock;

	friend Derived;

 public:
	using const_iterator = IteratorWrapper<Derived, Node>;
	using const_query_iterator = const_iterator;
	using const_bounding_volume_iterator = IteratorWrapper<Derived, NodeBV>;
	using const_bounding_volume_query_iterator = const_bounding_volume_iterator;
	using const_query_nearest_iterator = IteratorWrapper<Derived, NearestNode>;

	using Query = util::IteratorWrapper<const_query_iterator>;
	using QueryBV = util::IteratorWrapper<const_bounding_volume_query_iterator>;
	using QueryNearest = util::IteratorWrapper<const_query_nearest_iterator>;

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                       Octree                                        |
	|                                                                                     |
	**************************************************************************************/

	//
	// Clear
	//

	/*!
	 * @brief Erases the map. After this call, the map contains only the root node.
	 *
	 * @param prune Whether the memory should be cleared.
	 */
	void clear(bool prune = false) { clear(size(), depthLevels(), prune); }

	/*!
	 * @brief Erases the map and changes the leaf node size and the number of depth levels.
	 * After this call, the map contains only the root node.
	 *
	 * @param leaf_size The new leaf node size.
	 * @param depth_levels The new number of depth levels.
	 * @param prune Whether the memory should be cleared.
	 */
	void clear(node_size_t leaf_size, depth_t depth_levels, bool prune = false)
	{
		deleteChildren(root(), rootDepth(), prune);
		setNodeSizeAndDepthLevels(leaf_size, depth_levels);
		derived().initRoot();
	}

	//
	// Automatic pruning
	//

	// FIXME: Come up with better names

	/*!
	 * @brief Check if autonomaic pruning is enabled.
	 *
	 * @return Whether automatic pruning is enabled.
	 */
	[[nodiscard]] constexpr bool automaticPruning() const noexcept
	{
		return automatic_prune_;
	}

	/*!
	 * @brief Set (/turn on) automatic pruning.
	 */
	constexpr void setAutomaticPruning() noexcept { automatic_prune_ = true; }

	/*!
	 * @brief Reset (/turn off) automatic pruning.
	 */
	constexpr void resetAutomaticPruning() noexcept { automatic_prune_ = false; }

	//
	// Depth levels
	//

	/*!
	 * @brief The number of depth levels the octree currently have.
	 *
	 * @return The number of depth levels the octree currently have.
	 */
	[[nodiscard]] constexpr depth_t depthLevels() const noexcept { return depth_levels_; }

	/*!
	 * @brief The minimum depth levels an octree can have.
	 *
	 * @return The minimum depth levels an octree can have.
	 */
	[[nodiscard]] static constexpr depth_t minDepthLevels() { return 3; }

	/*!
	 * @brief The maximum depth levels an octree can have.
	 *
	 * @return The maximum depth levels an octree can have.
	 */
	[[nodiscard]] static constexpr depth_t maxDepthLevels() { return 22; }

	//
	// Size
	//

	/*!
	 * @brief Get the node size at a specific depth.
	 *
	 * @param depth The depth.
	 * @return The node size at the depth.
	 */
	[[nodiscard]] constexpr node_size_t size(depth_t depth = 0) const
	{
		return node_size_[depth];
	}

	//
	// Volume
	//

	/*!
	 * @brief The volume of the octree.
	 *
	 * @note This is the same as `size() * size() * size()`.
	 *
	 * @return The volume of the octree.
	 */
	[[nodiscard]] constexpr node_size_t volume() const
	{
		auto const s = size();
		return s * s * s;
	}

	//
	// Center
	//

	/*!
	 * @return The center of the octree.
	 */
	[[nodiscard]] Point center() const { return Point(0, 0, 0); }

	//
	// Bounding volume
	//

	/*!
	 * @return Minimum bounding volume convering the whole octree.
	 */
	[[nodiscard]] geometry::AAEBB boundingVolume() const
	{
		return geometry::AAEBB(center(), size(rootDepth() - 1));
	}

	//
	// Within
	//

	/*!
	 * @brief Check if a coordinate is within the octree bounds.
	 *
	 * @param coord The coordinate.
	 * @return Whether the coordinate is within the octree bounds.
	 */
	[[nodiscard]] constexpr bool isWithin(Point coord) const
	{
		return isWithin(coord.x, coord.y, coord.z);
	}

	/*!
	 * @brief Check if a coordinate is within the octree bounds.
	 *
	 * @param x,y,z The coordinate.
	 * @return Whether the coordinate is within the octree bounds.
	 */
	[[nodiscard]] constexpr bool isWithin(coord_t x, coord_t y, coord_t z) const
	{
		auto const max = size(rootDepth() - 1);
		auto const min = -max;
		return min <= x && min <= y && min <= z && max >= x && max >= y && max >= z;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Leaf                                         |
	|                                                                                     |
	**************************************************************************************/

	//
	// Pure leaf
	//

	/*!
	 * @brief Check if a node is a pure leaf node (i.e., can never have children).
	 *
	 * @param node The node to check.
	 * @return Whether the node is a pure leaf node.
	 */
	[[nodiscard]] static constexpr bool isPureLeaf(Node node) noexcept
	{
		return 0 == node.depth();
	}

	/*!
	 * @brief Check if a node corresponding to a code is a pure leaf node (i.e., can never
	 * have children).
	 *
	 * @note Only have to check if the depth of the code is 0.
	 *
	 * @param code The code of the node to check.
	 * @return Whether the node is a pure leaf node.
	 */
	[[nodiscard]] static constexpr bool isPureLeaf(Code code) noexcept
	{
		return 0 == code.depth();
	}

	/*!
	 * @brief Check if a node corresponding to a key is a pure leaf node (i.e., can never
	 * have children).
	 *
	 * @note Only have to check if the depth of the key is 0.
	 *
	 * @param key The key of the node to check.
	 * @return Whether the node is a pure leaf node.
	 */
	[[nodiscard]] static constexpr bool isPureLeaf(Key key) noexcept
	{
		return 0 == key.depth();
	}

	/*!
	 * @brief Check if a node corresponding to a coordinate at a specified depth is a
	 * pure leaf node (i.e., can never have children).
	 *
	 * @note Only have to check if the depth is 0.
	 *
	 * @param coord The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a pure leaf node.
	 */
	[[nodiscard]] static constexpr bool isPureLeaf(Point coord, depth_t depth = 0) noexcept
	{
		return 0 == depth;
	}

	/*!
	 * @brief Check if a node corresponding to a coordinate at a specified depth is a
	 * pure leaf node (i.e., can never have children).
	 *
	 * @note Only have to check if the depth is 0.
	 *
	 * @param x,y,z The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a pure leaf node.
	 */
	[[nodiscard]] static constexpr bool isPureLeaf(coord_t x, coord_t y, coord_t z,
	                                               depth_t depth = 0) noexcept
	{
		return 0 == depth;
	}

	//
	// Leaf
	//

	/*!
	 * @brief Check if a node is a leaf node (i.e., has no children).
	 *
	 * @param node The node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] constexpr bool isLeaf(Node node) const
	{
		return isPureLeaf(node) || innerNode(node).leaf[node.index()];
	}

	/*!
	 * @brief Check if a node corresponding to a code is a leaf node (i.e., has no
	 * children).
	 *
	 * @param code The code of the node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] bool isLeaf(Code code) const
	{
		if (isPureLeaf(code)) {
			return true;
		}
		auto [node, depth] = leafNodeAndDepth(code);
		return node.leaf[code.index(depth)];
	}

	/*!
	 * @brief Check if a node corresponding to a key is a leaf node (i.e., has no children).
	 *
	 * @param key The key of the node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] bool isLeaf(Key key) const
	{
		return isPureLeaf(key) || isLeaf(toCode(key));
	}

	/*!
	 * @brief Check if a node corresponding to a coordinate at a specified depth is a leaf
	 * node (i.e., has no children).
	 *
	 * @param coord The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] bool isLeaf(Point coord, depth_t depth = 0) const
	{
		return isPureLeaf(coord, depth) || isLeaf(toCode(coord, depth));
	}

	/*!
	 * @brief Check if a node corresponding to a coordinate at a specified depth is a leaf
	 * node (i.e., has no children).
	 *
	 * @param x,y,z The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] bool isLeaf(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return isPureLeaf(x, y, z, depth) || isLeaf(toCode(x, y, z, depth));
	}

	//
	// Parent
	//

	/*!
	 * @brief Check if a node is a parent (i.e., has children).
	 *
	 * @param node The node to check.
	 * @return Whether the node is a parent.
	 */
	[[nodiscard]] constexpr bool isParent(Node node) const { return !isLeaf(node); }

	/*!
	 * @brief Check if a node corresponding to a code is a parent (i.e., has
	 * children).
	 *
	 * @param code The code of the node to check.
	 * @return Whether the node is a parent.
	 */
	[[nodiscard]] constexpr bool isParent(Code code) const { return !isLeaf(code); }

	/*!
	 * @brief Check if a node corresponding to a key is a parent (i.e., has
	 * children).
	 *
	 * @param key The key of the node to check.
	 * @return Whether the node is a parent.
	 */
	[[nodiscard]] constexpr bool isParent(Key key) const { return !isLeaf(key); }

	/*!
	 * @brief Check if a node corresponding to a coordinate at a specified depth is a parent
	 * (i.e., has children).
	 *
	 * @param coord The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a parent.
	 */
	[[nodiscard]] constexpr bool isParent(Point coord, depth_t depth = 0) const
	{
		return !isLeaf(coord, depth);
	}

	/*!
	 * @brief Check if a node corresponding to a coordinate at a specified depth is a parent
	 * (i.e., has children).
	 *
	 * @param x,y,z The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a parent.
	 */
	[[nodiscard]] constexpr bool isParent(coord_t x, coord_t y, coord_t z,
	                                      depth_t depth = 0) const
	{
		return !isLeaf(x, y, z, depth);
	}

	// TODO: Add comment
	[[nodiscard]] constexpr bool exists(Node node) const
	{
		node = operator()(node);
		return node.isReal();
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Modified                                       |
	|                                                                                     |
	**************************************************************************************/

	//
	// Modified
	//

	/*!
	 * @brief Check if the octree is in a modified state (i.e., at least one node has been
	 * modified).
	 *
	 * @return Whether the octree is in a modified state.
	 */
	[[nodiscard]] constexpr bool isModified() const { return isModified(root()); }

	/*!
	 * @brief Check if a node of the octree is in a modified state (i.e., the node
	 * or one of its children has been modified).
	 *
	 * @param node The node to check.
	 * @return Whether the node is in a modified state.
	 */
	[[nodiscard]] constexpr bool isModified(Node node) const
	{
		return leafNode(node).modified[node.index()];
	}

	/*!
	 * @brief Check if a node corresponding to a code is in a modified state (i.e., the node
	 * or one of its children has been modified).
	 *
	 * @param code The code of the node to check.
	 * @return Whether the node is in a modified state.
	 */
	[[nodiscard]] constexpr bool isModified(Code code) const
	{
		auto [node, depth] = leafNodeAndDepth(code);
		return node.modified[code.index(depth)];
	}

	/*!
	 * @brief Check if a node corresponding to a key is in a modified state (i.e., the node
	 * or one of its children has been modified).
	 *
	 * @param key The key of the node to check.
	 * @return Whether the node is in a modified state.
	 */
	[[nodiscard]] constexpr bool isModified(Key key) const
	{
		return isModified(toCode(key));
	}

	/*!
	 * @brief Check if a node corresponding to a coordinate at a specified depth is in a
	 * modified state (i.e., the node or one of its children has been modified).
	 *
	 * @param coord The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is in a modified state.
	 */
	[[nodiscard]] constexpr bool isModified(Point coord, depth_t depth = 0) const
	{
		return isModified(toCode(coord, depth));
	}

	/*!
	 * @brief Check if a node corresponding to a coordinate at a specified depth is in a
	 * modified state (i.e., the node or one of its children has been modified).
	 *
	 * @param x,y,z The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is in a modified state.
	 */
	[[nodiscard]] constexpr bool isModified(coord_t x, coord_t y, coord_t z,
	                                        depth_t depth = 0) const
	{
		return isModified(toCode(x, y, z, depth));
	}

	//
	// Set modified
	//

	/*!
	 * @brief Set all nodes down to and including 'min_depth' to modified state.
	 *
	 * @param min_depth The minimum depth to set nodes to the modified state.
	 */
	void setModified(depth_t min_depth = 0)
	{
		if (rootDepth() >= min_depth) {
			setModified(root(), rootIndex(), rootDepth(), min_depth);
		}
	}

	/*!
	 * @brief Set a node and all its children down to and including 'min_depth' to the
	 * modified state. This will also set the parents of the node to the modified state.
	 *
	 * @param node The node.
	 * @param min_depth The minimum depth to set nodes to the modified state.
	 */
	void setModified(Node node, depth_t min_depth = 0)
	{
		if (rootDepth() < min_depth) {
			return;
		}

		if (node.depth() < min_depth) {
			setModifiedParents(node.code().toDepth(min_depth - 1));
		} else {
			if (isPureLeaf(node)) {
				setModified(leafNode(node), node.index());
			} else {
				setModified(innerNode(node), node.index(), node.depth(), min_depth);
			}
			setModifiedParents(node.code().toDepth(min_depth));
		}
	}

	/*!
	 * @brief Set a node, corresponding to a code, and all its children down to and
	 * including 'min_depth' to the modified state. This will also set the parents of the
	 * node to the modified state.
	 *
	 * @param code The code of the node.
	 * @param min_depth The minimum depth to set nodes to the modified state.
	 */
	void setModified(Code code, depth_t min_depth = 0)
	{
		if (rootDepth() < min_depth) {
			return;
		}

		if (code.depth() < min_depth) {
			setModifiedParents(code.toDepth(min_depth - 1));
		} else {
			if (isPureLeaf(code)) {
				auto [node, depth] = leafNodeAndDepth(code);
				setModified(node, code.index(depth));
			} else {
				auto [node, depth] = innerNodeAndDepth(code);
				setModified(node, code.index(depth), depth, min_depth);
			}
			setModifiedParents(code.toDepth(min_depth));
		}
	}

	/*!
	 * @brief Set a node, corresponding to a key, and all its children down to and including
	 * 'min_depth' to the modified state. This will also set the parents of the node to the
	 * modified state.
	 *
	 * @param key The key of the node.
	 * @param min_depth The minimum depth to set nodes to the modified state.
	 */
	void setModified(Key key, depth_t min_depth = 0)
	{
		setModified(toCode(key), min_depth);
	}

	/*!
	 * @brief Set a node, corresponding to a coordinate and a specified depth, and all its
	 * children down to and including 'min_depth' to the modified state. This will also set
	 * the parents of the node to the modified state.
	 *
	 * @param coord The coordinate of the node.
	 * @param min_depth The minimum depth to set nodes to the modified state.
	 * @param depth The depth of the node.
	 */
	void setModified(Point coord, depth_t min_depth = 0, depth_t depth = 0)
	{
		setModified(toCode(coord, depth), min_depth);
	}

	/*!
	 * @brief Set a node, corresponding to a coordinate and a specified depth, and all its
	 * children down to and including 'min_depth' to the modified state. This will also set
	 * the parents of the node to the modified state.
	 *
	 * @param x,y,z The coordinate of the node.
	 * @param min_depth The minimum depth to set nodes to the modified state.
	 * @param depth The depth of the node.
	 */
	void setModified(coord_t x, coord_t y, coord_t z, depth_t min_depth = 0,
	                 depth_t depth = 0)
	{
		setModified(toCode(x, y, z, depth), min_depth);
	}

	//
	// Reset modified
	//

	/*!
	 * @brief Reset all nodes up to and including 'max_depth' from modified state.
	 *
	 * @note Only use this if you know what you are doing or if you do *not* plan to query
	 * for information in the octree. This function does not propagate information up the
	 * tree, hence queries will not function as expected. If you want to make queries then
	 * you should use 'propagateModified' instead.
	 *
	 * @param max_depth The maximum depth to reset nodes from the modified state.
	 */
	void resetModified(depth_t max_depth = maxDepthLevels())
	{
		resetModified(root(), rootIndex(), rootDepth(), max_depth);
	}

	/*!
	 * @brief Reset a node and its children up to and including 'max_depth' from modified
	 * state.
	 *
	 * @note Only use this if you know what you are doing or if you do *not* plan to query
	 * for information in the octree. This function does not propagate information up the
	 * tree, hence queries will not function as expected. If you want to make queries then
	 * you should use 'propagateModified' instead.
	 *
	 * @param node The node.
	 * @param max_depth The maximum depth to reset nodes from the modified state.
	 */
	void resetModified(Node node, depth_t max_depth = maxDepthLevels())
	{
		if (isPureLeaf(node)) {
			resetModified(leafNode(node), node.index());
		} else {
			resetModified(innerNode(node), node.index(), node.depth(), max_depth);
		}
	}

	/*!
	 * @brief Reset a node, corresponding to a code, and its children up to and including
	 * 'max_depth' from modified state.
	 *
	 * @note Only use this if you know what you are doing or if you do *not* plan to query
	 * for information in the octree. This function does not propagate information up the
	 * tree, hence queries will not function as expected. If you want to make queries then
	 * you should use 'propagateModified' instead.
	 *
	 * @param code The code of the node.
	 * @param max_depth The maximum depth to reset nodes from the modified state.
	 */
	void resetModified(Code code, depth_t max_depth = maxDepthLevels())
	{
		if (isPureLeaf(code)) {
			auto [node, depth] = leafNodeAndDepth(code);
			if (code.depth() == depth) {
				resetModified(node, code.index(depth));
			}
		} else {
			auto [node, depth] = innerNodeAndDepth(code);
			if (code.depth() == depth) {
				resetModified(node, code.index(depth), depth, max_depth);
			}
		}
	}

	/*!
	 * @brief Reset a node, corresponding to a key, and its children up to and including
	 * 'max_depth' from modified state.
	 *
	 * @note Only use this if you know what you are doing or if you do *not* plan to query
	 * for information in the octree. This function does not propagate information up the
	 * tree, hence queries will not function as expected. If you want to make queries then
	 * you should use 'propagateModified' instead.
	 *
	 * @param key The key of the node.
	 * @param max_depth The maximum depth to reset nodes from the modified state.
	 */
	void resetModified(Key key, depth_t max_depth = maxDepthLevels())
	{
		resetModified(toCode(key), max_depth);
	}

	/*!
	 * @brief Reset a node, corresponding to a coordinate and a specified depth, and its
	 * children up to and including 'max_depth' from modified state.
	 *
	 * @note Only use this if you know what you are doing or if you do *not* plan to query
	 * for information in the octree. This function does not propagate information up the
	 * tree, hence queries will not function as expected. If you want to make queries then
	 * you should use 'propagateModified' instead.
	 *
	 * @param coord The coordinate of the node.
	 * @param max_depth The maximum depth to reset nodes from the modified state.
	 * @param depth The depth of the node.
	 */
	void resetModified(Point coord, depth_t max_depth = maxDepthLevels(), depth_t depth = 0)
	{
		resetModified(toCode(coord, depth), max_depth);
	}

	/*!
	 * @brief Reset a node, corresponding to a coordinate and a specified depth, and its
	 * children up to and including 'max_depth' from modified state.
	 *
	 * @note Only use this if you know what you are doing or if you do *not* plan to query
	 * for information in the octree. This function does not propagate information up the
	 * tree, hence queries will not function as expected. If you want to make queries then
	 * you should use 'propagateModified' instead.
	 *
	 * @param x,y,z The coordinate of the node.
	 * @param max_depth The maximum depth to reset nodes from the modified state.
	 * @param depth The depth of the node.
	 */
	void resetModified(coord_t x, coord_t y, coord_t z,
	                   depth_t max_depth = maxDepthLevels(), depth_t depth = 0)
	{
		resetModified(toCode(x, y, z, depth), max_depth);
	}

	//
	// Propagate
	//

	/*!
	 * @brief Propagate modified information up the octree to and including 'max_depth'.
	 *
	 * @param keep_modified Whether propagated node's modified state should be reset.
	 * @param max_depth The maximum depth to propagate information.
	 */
	void propagateModified(bool keep_modified = false, depth_t max_depth = maxDepthLevels())
	{
		propagateModified(root(), rootIndex(), rootDepth(), keep_modified, max_depth);
	}

	/*!
	 * @brief Propagate a node's children modified information up to and including
	 * 'max_depth'.
	 *
	 * @param node The node.
	 * @param keep_modified Whether propagated node's modified state should be reset.
	 * @param max_depth The maximum depth to propagate information.
	 */
	void propagateModified(Node node, bool keep_modified = false,
	                       depth_t max_depth = maxDepthLevels())
	{
		if (isPureLeaf(node)) {
			propagateModified(leafNode(node), node.index(), keep_modified);
		} else {
			propagateModified(innerNode(node), node.index(), node.depth(), keep_modified,
			                  max_depth);
		}
	}

	/*!
	 * @brief Propagate a node's, corresponding to a code, children modified information up
	 * to and including 'max_depth'.
	 *
	 * @param code The code of the node.
	 * @param keep_modified Whether propagated node's modified state should be reset.
	 * @param max_depth The maximum depth to propagate information.
	 */
	void propagateModified(Code code, bool keep_modified = false,
	                       depth_t max_depth = maxDepthLevels())
	{
		if (isPureLeaf(code)) {
			auto [node, depth] = leafNodeAndDepth(code);
			if (code.depth() == depth) {
				propagateModified(node, code.index(depth), keep_modified);
			}
		} else {
			auto [node, depth] = innerNodeAndDepth(code);
			if (code.depth() == depth) {
				propagateModified(node, code.index(depth), depth, keep_modified, max_depth);
			}
		}
	}

	/*!
	 * @brief Propagate a node's, corresponding to a key, children modified information up
	 * to and including 'max_depth'.
	 *
	 * @param key The key of the node.
	 * @param keep_modified Whether propagated node's modified state should be reset.
	 * @param max_depth The maximum depth to propagate information.
	 */
	void propagateModified(Key key, bool keep_modified = false,
	                       depth_t max_depth = maxDepthLevels())
	{
		propagateModified(toCode(key), keep_modified, max_depth);
	}

	/*!
	 * @brief Propagate a node's, corresponding to a coordinate and a specified depth,
	 * children modified information up to and including 'max_depth'.
	 *
	 * @param coord The coordinate of the node.
	 * @param keep_modified Whether propagated node's modified state should be reset.
	 * @param max_depth The maximum depth to propagate information.
	 * @param depth The depth of the node.
	 */
	void propagateModified(Point coord, bool keep_modified = false,
	                       depth_t max_depth = maxDepthLevels(), depth_t depth = 0)
	{
		propagateModified(toCode(coord, depth), depth, keep_modified, max_depth);
	}

	/*!
	 * @brief Propagate a node's, corresponding to a coordinate and a specified depth,
	 * children modified information up to and including 'max_depth'.
	 *
	 * @param x,y,z The coordinate of the node.
	 * @param keep_modified Whether propagated node's modified state should be reset.
	 * @param max_depth The maximum depth to propagate information.
	 * @param depth The depth of the node.
	 */
	void propagateModified(coord_t x, coord_t y, coord_t z, bool keep_modified = false,
	                       depth_t max_depth = maxDepthLevels(), depth_t depth = 0)
	{
		propagateModified(toCode(x, y, z, depth), keep_modified, max_depth);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Root                                         |
	|                                                                                     |
	**************************************************************************************/

	//
	// Is
	//

	/*!
	 * @brief Check if a node is the root node.
	 *
	 * @param node The node.
	 * @return Whether the node is the root node.
	 */
	[[nodiscard]] bool isRoot(Node node) const { return isRoot(node.code()); }

	/*!
	 * @brief Check if a node, corresponding to a code, is the root node.
	 *
	 * @param code The code of the node.
	 * @return Whether the node is the root node.
	 */
	[[nodiscard]] bool isRoot(Code code) const { return rootCode() == code; }

	/*!
	 * @brief Check if a node, corresponding to a key, is the root node.
	 *
	 * @param key The key of the node.
	 * @return Whether the node is the root node.
	 */
	[[nodiscard]] bool isRoot(Key key) const { return isRoot(toCode(key)); }

	/*!
	 * @brief Check if a node, corresponding to a coordinate at a specific depth, is the
	 * root node.
	 *
	 * @param coord The coordinate of the node.
	 * @param depth The depth of the node.
	 * @return Whether the node is the root node.
	 */
	[[nodiscard]] bool isRoot(Point coord, depth_t depth = 0) const
	{
		return isRoot(toCode(coord, depth));
	}

	/*!
	 * @brief Check if a node, corresponding to a coordinate at a specific depth, is the
	 * root node.
	 *
	 * @param x,y,z The coordinate of the node.
	 * @param depth The depth of the node.
	 * @return Whether the node is the root node.
	 */
	[[nodiscard]] bool isRoot(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return isRoot(toCode(x, y, z, depth));
	}

	//
	// Get
	//

	/*!
	 * @brief Get the root node.
	 *
	 * @return The root node.
	 */
	[[nodiscard]] constexpr Node rootNode() const
	{
		return Node(const_cast<InnerNode*>(&root()), rootCode(), rootDepth());
	}

	/*!
	 * @brief Get the root node with bounding volume.
	 *
	 * @return The root node with bounding volume.
	 */
	[[nodiscard]] constexpr NodeBV rootNodeBV() const
	{
		return NodeBV(rootNode(), rootBoundingVolume());
	}

	//
	// Code
	//

	/*!
	 * @brief Get the code for the root node.
	 *
	 * @return The root node code.
	 */
	[[nodiscard]] constexpr Code rootCode() const
	{
		return Code(rootIndex() << 3 * rootDepth(), rootDepth());
	}

	//
	// Depth
	//

	/*!
	 * @brief Get the depth of the root node.
	 *
	 * @return The depth of the root node.
	 */
	[[nodiscard]] constexpr depth_t rootDepth() const noexcept { return depthLevels() - 1; }

	//
	// Size
	//

	/*!
	 * @brief Get the size of the root node.
	 *
	 * @note This is the same as `size()`.
	 *
	 * @return The size of the root node.
	 */
	[[nodiscard]] constexpr node_size_t rootSize() const noexcept { return size(); }

	//
	// Center
	//

	/*!
	 * @brief Get the center of the root node.
	 *
	 * @note This is the same as `center()`.
	 *
	 * @return The center of the root node.
	 */
	[[nodiscard]] constexpr Point rootCenter() const noexcept { return center(); }

	//
	// Bounding volume
	//

	/*!
	 * @brief Get the bounding volume of the root node.
	 *
	 * @note This is the same as `boundingVolmue()`.
	 *
	 * @return The bounding volume of the root node.
	 */
	[[nodiscard]] constexpr geometry::AAEBB rootBoundingVolume() const noexcept
	{
		return boundingVolume();
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Node                                         |
	|                                                                                     |
	**************************************************************************************/

	//
	// Size
	//

	/*!
	 * @brief Get the size of a node.
	 *
	 * @param node The node.
	 * @return The size of the node.
	 */
	[[nodiscard]] constexpr node_size_t size(Node node) const { return size(node.depth()); }

	//
	// Center
	//

	/*!
	 * @brief The center of a node.
	 *
	 * @param node The node.
	 * @return The center of the node.
	 */
	template <class NodeType>
	[[nodiscard]] Point center(NodeType const& node) const
	{
		if constexpr (std::is_same_as_v<NodeBV, NodeType>) {
			return node.center();
		} else {
			return coord(node.code());
		}
	}

	//
	// Bounding volume
	//

	/*!
	 * @brief Bounding volume for a node.
	 *
	 * @param node The node
	 * @return Bounding volume for the node.
	 */
	template <class NodeType>
	[[nodiscard]] geometry::AAEBB boundingVolume(NodeType const& node) const
	{
		if constexpr (std::is_same_as_v<NodeBV, NodeType>) {
			return node.boundingVolume();

		} else {
			return geometry::AAEBB(center(node), size(node) / 2);
		}
	}

	//
	// At
	//

	/*!
	 * @brief Get the node corresponding to a code with bounds checking.
	 *
	 * @note The node can be higher up the tree than the specified depth. This happens if
	 * the node at a higher depth has no children. If it is neccessary that the node is at
	 * the specified depth, then the corresponding 'createNodeChecked' function can be used.
	 * The data inside the nodes returned by this function and 'createNodeChecked' will be
	 * the same, so it is only neccessary to use 'createNodeChecked' if you intend to alter
	 * what the node stores.
	 *
	 * @param code The code.
	 * @return The node.
	 */
	[[nodiscard]] std::optional<Node> at(Code code) const
	{
		return code.depth() <= rootDepth() ? std::optional<Node>(operator()(code))
		                                   : std::nullopt;
	}

	/*!
	 * @brief Get the node corresponding to a key with bounds checking.
	 *
	 * @note The node can be higher up the tree than the specified depth. This happens if
	 * the node at a higher depth has no children. If it is neccessary that the node is at
	 * the specified depth, then the corresponding 'createNodeChecked' function can be used.
	 * The data inside the nodes returned by this function and 'createNodeChecked' will be
	 * the same, so it is only neccessary to use 'createNodeChecked' if you intend to alter
	 * what the node stores.
	 *
	 * @param key The key.
	 * @return The node.
	 */
	[[nodiscard]] std::optional<Node> at(Key key) const { return at(toCode(key)); }

	/*!
	 * @brief Get the node corresponding to a coordinate at a specific depth with bounds
	 * checking.
	 *
	 * @note The node can be higher up the tree than the specified depth. This happens if
	 * the node at a higher depth has no children. If it is neccessary that the node is at
	 * the specified depth, then the corresponding 'createNodeChecked' function can be used.
	 * The data inside the nodes returned by this function and 'createNodeChecked' will be
	 * the same, so it is only neccessary to use 'createNodeChecked' if you intend to alter
	 * what the node stores.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The node.
	 */
	[[nodiscard]] std::optional<Node> at(Point coord, depth_t depth = 0) const
	{
		if (auto code = toCodeChecked(coord, depth)) {
			return std::optional<Node>(*code);
		} else {
			return std::nullopt;
		}
	}

	/*!
	 * @brief Get the node corresponding to a coordinate at a specific depth with bounds
	 * checking.
	 *
	 * @note The node can be higher up the tree than the specified depth. This happens if
	 * the node at a higher depth has no children. If it is neccessary that the node is at
	 * the specified depth, then the corresponding 'createNodeChecked' function can be used.
	 * The data inside the nodes returned by this function and 'createNodeChecked' will be
	 * the same, so it is only neccessary to use 'createNodeChecked' if you intend to alter
	 * what the node stores.
	 *
	 * @param x,y,z The coordinate.
	 * @param depth The depth.
	 * @return The node.
	 */
	[[nodiscard]] std::optional<Node> at(coord_t x, coord_t y, coord_t z,
	                                     depth_t depth = 0) const
	{
		return at(Point(x, y, z), depth);
	}

	//
	// Function call operator
	//

	// TODO: Add comment
	[[nodiscard]] Node operator()(Node node) const
	{
		if (node.depth() == node.realDepth()) {
			return node;
		}

		// TODO: Implement correct
		InnerNode const* node = &innerNode(node);
		depth_t depth = node.realDepth();
		depth_t min_depth = std::max(node.depth(), depth_t(1));
		Code code = node.code();
		while (min_depth != depth && isParent(*node)) {
			node = &innerChild(*node, code.index(depth));
			--depth;
		}

		return 0 == code.depth() && isParent(*node)
		           ? Node(const_cast<LeafNode*>(&leafChild(*node, code.index())), code, 0)
		           : Node(const_cast<LeafNode*>(node), code, depth);
	}

	/*!
	 * @brief Get the node corresponding to a code.
	 *
	 * @note The node can be higher up the tree than the specified depth. This happens if
	 * the node at a higher depth has no children. If it is neccessary that the node is at
	 * the specified depth, then the corresponding 'createNode' function can be used. The
	 * data inside the nodes returned by this function and 'createNode' will be the same, so
	 * it is only neccessary to use 'createNode' if you intend to alter what the node
	 * stores.
	 *
	 * @param code The code.
	 * @return The node.
	 */
	[[nodiscard]] Node operator()(Code code) const
	{
		auto [node, depth] = leafNodeAndDepth(code);
		return Node(const_cast<LeafNode*>(&node), code, depth);
	}

	/*!
	 * @brief Get the node corresponding to a key.
	 *
	 * @note The node can be higher up the tree than the specified depth. This happens if
	 * the node at a higher depth has no children. If it is neccessary that the node is at
	 * the specified depth, then the corresponding 'createNode' function can be used. The
	 * data inside the nodes returned by this function and 'createNode' will be the same, so
	 * it is only neccessary to use 'createNode' if you intend to alter what the node
	 * stores.
	 *
	 * @param key The key.
	 * @return The node.
	 */
	[[nodiscard]] Node operator()(Key key) const { return operator()(toCode(key)); }

	/*!
	 * @brief Get the node corresponding to a coordinate at a specific depth.
	 *
	 * @note The node can be higher up the tree than the specified depth. This happens if
	 * the node at a higher depth has no children. If it is neccessary that the node is at
	 * the specified depth, then the corresponding 'createNode' function can be used. The
	 * data inside the nodes returned by this function and 'createNode' will be the same, so
	 * it is only neccessary to use 'createNode' if you intend to alter what the node
	 * stores.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The node.
	 */
	[[nodiscard]] Node operator()(Point coord, depth_t depth = 0) const
	{
		return operator()(toCode(coord, depth));
	}

	/*!
	 * @brief Get the node corresponding to a coordinate at a specific depth.
	 *
	 * @note The node can be higher up the tree than the specified depth. This happens if
	 * the node at a higher depth has no children. If it is neccessary that the node is at
	 * the specified depth, then the corresponding 'createNode' function can be used. The
	 * data inside the nodes returned by this function and 'createNode' will be the same, so
	 * it is only neccessary to use 'createNode' if you intend to alter what the node
	 * stores.
	 *
	 * @param x,y,z The coordinate.
	 * @param depth The depth.
	 * @return The node.
	 */
	[[nodiscard]] Node operator()(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return operator()(toCode(x, y, z, depth));
	}

	//
	// Sibling
	//

	/*!
	 * @brief Get the sibling of a node.
	 *
	 * @param node The node.
	 * @param sibling_index The index of the sibling.
	 * @return The sibling.
	 */
	[[nodiscard]] Node sibling(Node node, index_t sibling_index) const
	{
		node = operator()(node);
		return Node(node.data(), node.code().sibling(sibling_index), node.realDepth());
	}

	/*!
	 * @brief Get the sibling of a node.
	 *
	 * @param node The node.
	 * @param sibling_index The index of the sibling.
	 * @return The sibling.
	 */
	[[nodiscard]] NodeBV sibling(NodeBV const& node, index_t sibling_index) const
	{
		geometry::AAEBB aaebb(
		    siblingCenter(node.center(), node.halfSize(), node.index(), sibling_index),
		    node.halfSize());
		return NodeBV(sibling(static_cast<Node>(node), sibling_index), aaebb);
	}

	/*!
	 * @brief Get the sibling of a node with bounds checking.
	 *
	 * @param node The node.
	 * @param sibling_index The index of the sibling.
	 * @return The sibling.
	 */
	template <class Node>
	[[nodiscard]] Node siblingChecked(Node const& node, index_t sibling_index) const
	{
		if (!isRoot(node)) {
			throw std::out_of_range("Node has no siblings");
		} else if (7 < sibling_index) {
			throw std::out_of_range("sibling_index out of range");
		}
		return sibling(node, sibling_index);
	}

	//
	// Child
	//

	/*!
	 * @brief Get a child of a node.
	 *
	 * @param node The node.
	 * @param child_index The index of the child.
	 * @return The child.
	 */
	[[nodiscard]] Node child(Node node, index_t child_index) const
	{
		node = operator()(node);
		return isLeaf(node)
		           ? Node(node.data(), node.code().child(child_index), node.realDepth())
		           : Node(&child(innerNode(node), node.index(), node.depth()),
		                  node.code.child(child_index), node.realDepth() - 1);
	}

	/*!
	 * @brief Get a child of a node.
	 *
	 * @param node The node.
	 * @param child_index The index of the child.
	 * @return The child.
	 */
	[[nodiscard]] NodeBV child(NodeBV const& node, index_t child_index) const
	{
		auto const child_half_size = node.halfSize() / 2;
		geometry::AAEBB child_aaebb(childCenter(node.center(), child_half_size, child_index),
		                            child_half_size);

		return NodeBV(child(static_cast<Node>(node), child_index), child_aaebb);
	}

	/*!
	 * @brief Get a child of a node with bounds checking.
	 *
	 * @param node The node.
	 * @param child_index The index of the child.
	 * @return The child.
	 */
	template <class Node>
	[[nodiscard]] Node childChecked(Node const& node, index_t child_index) const
	{
		if (!isParent(node)) {
			throw std::out_of_range("Node has no children");
		} else if (7 < child_index) {
			throw std::out_of_range("child_index out of range");
		}
		return child(node, child_index);
	}

	//
	// Parent
	//

	/*!
	 * @brief Get the parent of a node.
	 *
	 * @param node The node.
	 * @return The parent.
	 */
	[[nodiscard]] Node parent(Node node) const
	{
		return node.depth() == node.realDepth() ? operator()(node.code().parent())
		                                        : Node(node.data(), node.code.parent(),
		                                               node.realDepth());
	}

	/*!
	 * @brief Get the parent of a node.
	 *
	 * @param node The node.
	 * @return The parent.
	 */
	[[nodiscard]] NodeBV parent(NodeBV const& node) const
	{
		return toNodeBV(parent(static_cast<Node>(node)));
	}

	/*!
	 * @brief Get the parent of a node with bounds checking.
	 *
	 * @param node The node.
	 * @return The parent.
	 */
	template <class Node>
	[[nodiscard]] Node parentChecked(Node const& node) const
	{
		if (rootDepth() <= node.depth()) {
			throw std::out_of_range("Node has no parent");
		}
		return parent(node);
	}

	//
	// NodeBV
	//

	/*!
	 * @brief Convert a node to a node with bounding volume.
	 *
	 * @param node The node.
	 * @return The node with bounding volume.
	 */
	[[nodiscard]] NodeBV toNodeBV(Node node) { return NodeBV(node, boundingVolume(node)); }

	/**************************************************************************************
	|                                                                                     |
	|                                     Conversion                                      |
	|                                                                                     |
	**************************************************************************************/

	//
	// Code
	//

	/*!
	 * @brief Convert a key to a code.
	 *
	 * @param key The key to convert.
	 * @return The code corresponding to the key.
	 */
	[[nodiscard]] static constexpr Code toCode(Key key) noexcept { return Code(key); }

	/*!
	 * @brief Convert a coordinate at a specific depth to a code.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The code corresponding to the coordinate at the specific depth.
	 */
	[[nodiscard]] constexpr Code toCode(Point coord, depth_t depth = 0) const noexcept
	{
		return toCode(toKey(coord, depth));
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a code.
	 *
	 * @param x,y,z The coordinate.
	 * @param depth The depth.
	 * @return The code corresponding to the coordinate at the specific depth.
	 */
	[[nodiscard]] constexpr Code toCode(coord_t x, coord_t y, coord_t z,
	                                    depth_t depth = 0) const noexcept
	{
		return toCode(toKey(x, y, z, depth));
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a code with bounds check.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The code corresponding to the coordinate at the specific depth.
	 */
	[[nodiscard]] constexpr std::optional<Code> toCodeChecked(
	    Point coord, depth_t depth = 0) const noexcept
	{
		std::optional<key_t> key = toKeyChecked(coord, depth);
		return key ? std::optional<Code>(toCode(*key)) : std::nullopt;
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a code with bounds check.
	 *
	 * @param x,y,z The coordinate.
	 * @param depth The depth.
	 * @return The code corresponding to the coordinate at the specific depth.
	 */
	[[nodiscard]] constexpr std::optional<Code> toCodeChecked(
	    coord_t x, coord_t y, coord_t z, depth_t depth = 0) const noexcept
	{
		return toCodeChecked(Point(x, y, z), depth);
	}

	//
	// Key
	//

	/*!
	 * @brief Convert a code to a key.
	 *
	 * @param code The code to convert.
	 * @return The key corresponding to the code.
	 */
	[[nodiscard]] static constexpr Key toKey(Code code) noexcept { return code; }

	/*!
	 * @brief COnvert a coordinate at a specific depth to a key.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The corresponding key.
	 */
	[[nodiscard]] constexpr Key toKey(Point coord, depth_t depth = 0) const noexcept
	{
		return Key(toKey(coord.x, depth), toKey(coord.y, depth), toKey(coord.z, depth),
		           depth);
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a key.
	 *
	 * @param x,y,z The coordinate.
	 * @param depth The depth.
	 * @return The corresponding key.
	 */
	[[nodiscard]] constexpr Key toKey(coord_t x, coord_t y, coord_t z,
	                                  depth_t depth = 0) const noexcept
	{
		return Key(toKey(x, depth), toKey(y, depth), toKey(z, depth), depth);
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a key with bounds check.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The corresponding key.
	 */
	[[nodiscard]] constexpr std::optional<Key> toKeyChecked(
	    Point coord, depth_t depth = 0) const noexcept
	{
		return rootDepth() >= depth && isWithin(coord)
		           ? std::optional<Key>(toKey(coord, depth))
		           : std::nullopt;
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a key with bounds check.
	 *
	 * @param x,y,z The coordinate.
	 * @param depth The depth.
	 * @return The corresponding key.
	 */
	[[nodiscard]] constexpr std::optional<Key> toKeyChecked(
	    coord_t x, coord_t y, coord_t z, depth_t depth = 0) const noexcept
	{
		return toKeyChecked(Point(x, y, z), depth);
	}

	//
	// Coordinate
	//

	/*!
	 * @brief Convert a code to a coordinate.
	 *
	 * @param code The code.
	 * @return The corresponding coordinate.
	 */
	[[nodiscard]] constexpr Point toCoord(Code code) const noexcept
	{
		return toCoord(toKey(code));
	}

	/*!
	 * @brief Convert a key to a coordinate.
	 *
	 * @param key The key.
	 * @return The corresponding coordinate.
	 */
	[[nodiscard]] constexpr Point toCoord(Key key) const noexcept
	{
		return Point(toCoord(key[0], key.depth()), toCoord(key[1], key.depth()),
		             toCoord(key[2], key.depth()));
	}

	/*!
	 * @brief Convert a code to a coordinate with bounds check.
	 *
	 * @param Code The code.
	 * @return The corresponding coordinate.
	 */
	[[nodiscard]] constexpr std::optional<Point> toCoordChecked(Code code) const noexcept
	{
		return toCoordChecked(toKey(code));
	}

	/*!
	 * @brief Convert a key to a coordinate with bounds check.
	 *
	 * @param key The key.
	 * @return The corresponding coordinate.
	 */
	[[nodiscard]] constexpr std::optional<Point> toCoordChecked(Key key) const noexcept
	{
		return rootDepth() >= key.depth() ? std::optional<Point>(toCoord(key)) : std::nullopt;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Traverse                                       |
	|                                                                                     |
	**************************************************************************************/

	/*!
	 * @brief Depth first traversal of the octree, starting at the root node. The function
	 * 'f' will be called for each node traverse. If 'f' returns false then the children of
	 * the node will also be traverse, otherwise they will not.
	 *
	 * @param f The callback function to be called for each node traversed.
	 */
	template <class UnaryFunction>
	void traverse(UnaryFunction f) const
	{
		if constexpr (std::is_same_as_v<
		                  Node, util::argument<UnaryFunction, 0>>) {  // FIXME: Should it be
			                                                            // const also?
			traverseRecurs(rootNode(), f);
		} else {
			traverseRecurs(rootNodeBV(), f);
		}
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node. The function 'f'
	 * will be called for each node traverse. If 'f' returns false then the children of the
	 * node will also be traverse, otherwise they will not.
	 *
	 * @param node The node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 */
	template <class UnaryFunction>
	void traverse(Node node, UnaryFunction f) const
	{
		if constexpr (std::is_same_v<util::argument<UnaryFunction, 0>,
		                             Node>) {  // FIXME: Should it be const also?
			traverseRecurs(node, f);
		} else {
			traverseRecurs(NodeBV(node, boundingVolume(node)), f);
		}
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node corresponding to the
	 * code. The function 'f' will be called for each node traverse. If 'f' returns false
	 * then the children of the node will also be traverse, otherwise they will not.
	 *
	 * @param code The code to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 */
	template <class UnaryFunction>
	void traverse(Code code, UnaryFunction f) const
	{
		traverse(operator()(code), f);
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node corresponding to the
	 * key. The function 'f' will be called for each node traverse. If 'f' returns false
	 * then the children of the node will also be traverse, otherwise they will not.
	 *
	 * @param key The key to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 */
	template <class UnaryFunction>
	void traverse(Key key, UnaryFunction f) const
	{
		traverse(toCode(key), f);
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node corresponding to the
	 * coordinate at a specified depth. The function 'f' will be called for each node
	 * traverse. If 'f' returns false then the children of the node will also be traverse,
	 * otherwise they will not.
	 *
	 * @param coord The coord to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 * @param depth The depth of the node.
	 */
	template <class UnaryFunction>
	void traverse(Point coord, UnaryFunction f, depth_t depth = 0) const
	{
		traverse(toCode(coord, depth), f);
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node corresponding to the
	 * coordinate at a specified depth. The function 'f' will be called for each node
	 * traverse. If 'f' returns false then the children of the node will also be traverse,
	 * otherwise they will not.
	 *
	 * @param x,y,z The coord to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 * @param depth The depth of the node.
	 */
	template <class UnaryFunction>
	void traverse(coord_t x, coord_t y, coord_t z, UnaryFunction f, depth_t depth = 0) const
	{
		traverse(toCode(x, y, z, depth), f);
	}

	/*! TODO: Update info for all nearest
	 * @brief Traverse the octree in the orderDepth first traversal of the octree, starting
	 * at the root node. The function 'f' will be called for each node traverse. If 'f'
	 * returns false then the children of the node will also be traverse, otherwise they
	 * will not.
	 *
	 * @param f The callback function to be called for each node traversed.
	 */
	template <class Geometry, class UnaryFunction>
	void traverseNearest(Geometry const& g, UnaryFunction f) const
	{
		traverseNearestRecurs(rootNodeBV(), g, f);
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node. The function 'f'
	 * will be called for each node traverse. If 'f' returns false then the children of the
	 * node will also be traverse, otherwise they will not.
	 *
	 * @param node The node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 */
	template <class Geometry, class UnaryFunction>
	void traverseNearest(Node node, Geometry const& g, UnaryFunction f) const
	{
		traverseNearestRecurs(NodeBV(node, boundingVolume(node)), g, f);
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node corresponding to the
	 * code. The function 'f' will be called for each node traverse. If 'f' returns false
	 * then the children of the node will also be traverse, otherwise they will not.
	 *
	 * @param code The code to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 */
	template <class Geometry, class UnaryFunction>
	void traverseNearest(Code code, Geometry const& g, UnaryFunction f) const
	{
		traverseNearest(operator()(code), g, f);
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node corresponding to the
	 * key. The function 'f' will be called for each node traverse. If 'f' returns false
	 * then the children of the node will also be traverse, otherwise they will not.
	 *
	 * @param key The key to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 */
	template <class Geometry, class UnaryFunction>
	void traverseNearest(Key key, Geometry const& g, UnaryFunction f) const
	{
		traverseNearest(toCode(key), g, f);
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node corresponding to the
	 * coordinate at a specified depth. The function 'f' will be called for each node
	 * traverse. If 'f' returns false then the children of the node will also be traverse,
	 * otherwise they will not.
	 *
	 * @param coord The coord to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 * @param depth The depth of the node.
	 */
	template <class Geometry, class UnaryFunction>
	void traverseNearest(Point coord, Geometry const& g, UnaryFunction f,
	                     depth_t depth = 0) const
	{
		traverseNearest(toCode(coord, depth), g, f);
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node corresponding to the
	 * coordinate at a specified depth. The function 'f' will be called for each node
	 * traverse. If 'f' returns false then the children of the node will also be traverse,
	 * otherwise they will not.
	 *
	 * @param x,y,z The coord to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 * @param depth The depth of the node.
	 */
	template <class Geometry, class UnaryFunction>
	void traverseNearest(coord_t x, coord_t y, coord_t z, Geometry const& g,
	                     UnaryFunction f, depth_t depth = 0) const
	{
		traverseNearest(toCode(x, y, z, depth), g, f);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Query                                        |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	//
	// Query
	//

	template <class Predicates>
	[[nodiscard]] Query query(Predicates&& predicates, bool create_nodes = false) const
	{
		return Query(beginQuery(std::forward<Predicates>(predicates), create_nodes),
		             endQuery());
	}

	template <class Predicates>
	[[nodiscard]] Query query(Node node, Predicates&& predicates,
	                          bool create_nodes = false) const
	{
		return Query(beginQuery(node, std::forward<Predicates>(predicates), create_nodes),
		             endQuery());
	}

	template <class Predicates>
	[[nodiscard]] Query query(Code code, Predicates&& predicates,
	                          bool create_nodes = false) const
	{
		return Query(beginQuery(code, std::forward<Predicates>(predicates), create_nodes),
		             endQuery());
	}

	template <class Predicates>
	[[nodiscard]] Query query(Key key, Predicates&& predicates,
	                          bool create_nodes = false) const
	{
		return query(toCode(key), std::forward<Predicates>(predicates), create_nodes);
	}

	template <class Predicates>
	[[nodiscard]] Query query(Point coord, depth_t depth, Predicates&& predicates,
	                          bool create_nodes = false) const
	{
		return query(toCode(coord, depth), std::forward<Predicates>(predicates),
		             create_nodes);
	}

	template <class Predicates>
	[[nodiscard]] Query query(coord_t x, coord_t y, coord_t z, depth_t depth,
	                          Predicates&& predicates, bool create_nodes = false) const
	{
		return query(toCode(x, y, z, depth), std::forward<Predicates>(predicates),
		             create_nodes);
	}

	//
	// Query bounding volume
	//

	template <class Predicates>
	[[nodiscard]] QueryBV queryBV(Predicates&& predicates, bool create_nodes = false) const
	{
		return QueryBV(beginQueryBV(std::forward<Predicates>(predicates), create_nodes),
		               endQueryBV());
	}

	template <class Predicates>
	[[nodiscard]] QueryBV queryBV(Node node, Predicates&& predicates,
	                              bool create_nodes = false) const
	{
		return QueryBV(beginQueryBV(node, std::forward<Predicates>(predicates), create_nodes),
		               endQueryBV());
	}

	template <class Predicates>
	[[nodiscard]] QueryBV queryBV(Code code, Predicates&& predicates,
	                              bool create_nodes = false) const
	{
		return QueryBV(beginQueryBV(code, std::forward<Predicates>(predicates), create_nodes),
		               endQueryBV());
	}

	template <class Predicates>
	[[nodiscard]] QueryBV queryBV(Key key, Predicates&& predicates,
	                              bool create_nodes = false) const
	{
		return queryBV(toCode(key), std::forward<Predicates>(predicates), create_nodes);
	}

	template <class Predicates>
	[[nodiscard]] QueryBV queryBV(Point coord, depth_t depth, Predicates&& predicates,
	                              bool create_nodes = false) const
	{
		return queryBV(toCode(coord, depth), std::forward<Predicates>(predicates),
		               create_nodes);
	}

	template <class Predicates>
	[[nodiscard]] QueryBV queryBV(coord_t x, coord_t y, coord_t z, depth_t depth,
	                              Predicates&& predicates, bool create_nodes = false) const
	{
		return queryBV(toCode(x, y, z, depth), std::forward<Predicates>(predicates),
		               create_nodes);
	}

	//
	// Query nearest
	//

	template <class Geometry, class Predicates>
	[[nodiscard]] QueryNearest queryNearest(Geometry&& geometry, Predicates&& predicates,
	                                        bool create_nodes = false) const
	{
		return QueryNearest(
		    beginQueryNearest(std::forward<Geometry>(geometry),
		                      std::forward<Predicates>(predicates), create_nodes),
		    endQueryNearest());
	}

	template <class Geometry, class Predicates>
	[[nodiscard]] QueryNearest queryNearest(Node node, Geometry&& geometry,
	                                        Predicates&& predicates,
	                                        bool create_nodes = false) const
	{
		return QueryNearest(
		    beginQueryNearest(node, std::forward<Geometry>(geometry),
		                      std::forward<Predicates>(predicates), create_nodes),
		    endQueryNearest());
	}

	template <class Geometry, class Predicates>
	[[nodiscard]] QueryNearest queryNearest(Code code, Geometry&& geometry,
	                                        Predicates&& predicates,
	                                        bool create_nodes = false) const
	{
		return QueryNearest(
		    beginQueryNearest(code, std::forward<Geometry>(geometry),
		                      std::forward<Predicates>(predicates), create_nodes),
		    endQueryNearest());
	}

	template <class Geometry, class Predicates>
	[[nodiscard]] QueryNearest queryNearest(Key key, Geometry&& geometry,
	                                        Predicates&& predicates,
	                                        bool create_nodes = false) const
	{
		return queryNearest(toCode(key), std::forward<Geometry>(geometry),
		                    std::forward<Predicates>(predicates), create_nodes);
	}

	template <class Geometry, class Predicates>
	[[nodiscard]] QueryNearest queryNearest(Point coord, depth_t depth, Geometry&& geometry,
	                                        Predicates&& predicates,
	                                        bool create_nodes = false) const
	{
		return queryNearest(toCode(coord, depth), std::forward<Geometry>(geometry),
		                    std::forward<Predicates>(predicates), create_nodes);
	}

	template <class Geometry, class Predicates>
	[[nodiscard]] QueryNearest queryNearest(coord_t x, coord_t y, coord_t z, depth_t depth,
	                                        Geometry&& geometry, Predicates&& predicates,
	                                        bool create_nodes = false) const
	{
		return queryNearest(toCode(x, y, z, depth), std::forward<Geometry>(geometry),
		                    std::forward<Predicates>(predicates), create_nodes);
	}

	//
	// Query to output
	//

	template <class Predicates, class OutputIt>
	OutputIt query(Predicates&& predicates, OutputIt d_first,
	               bool create_nodes = false) const
	{
		if constexpr (std::is_same_v<typename OutputIt::value_type, Node>) {
			return std::copy(beginQuery(std::forward<Predicates>(predicates), create_nodes),
			                 endQuery(), d_first);
		} else {
			return std::copy(beginQueryBV(std::forward<Predicates>(predicates), create_nodes),
			                 endQueryBV(), d_first);
		}
	}

	template <class Predicates, class OutputIt>
	OutputIt query(Node node, Predicates&& predicates, OutputIt d_first,
	               bool create_nodes = false) const
	{
		if constexpr (std::is_same_v<typename OutputIt::value_type, Node>) {
			return std::copy(
			    beginQuery(node, std::forward<Predicates>(predicates), create_nodes),
			    endQuery(), d_first);
		} else {
			return std::copy(
			    beginQueryBV(node, std::forward<Predicates>(predicates), create_nodes),
			    endQueryBV(), d_first);
		}
	}

	template <class Predicates, class OutputIt>
	OutputIt query(Code code, Predicates&& predicates, OutputIt d_first,
	               bool create_nodes = false) const
	{
		if constexpr (std::is_same_v<typename OutputIt::value_type, Node>) {
			return std::copy(
			    beginQuery(code, std::forward<Predicates>(predicates), create_nodes),
			    endQuery(), d_first);
		} else {
			return std::copy(
			    beginQueryBV(code, std::forward<Predicates>(predicates), create_nodes),
			    endQueryBV(), d_first);
		}
	}

	template <class Predicates, class OutputIt>
	OutputIt query(Key key, Predicates&& predicates, OutputIt d_first,
	               bool create_nodes = false) const
	{
		return query(toCode(key), std::forward<Predicates>(predicates), d_first,
		             create_nodes);
	}

	template <class Predicates, class OutputIt>
	OutputIt query(Point coord, depth_t depth, Predicates&& predicates, OutputIt d_first,
	               bool create_nodes = false) const
	{
		return query(toCode(coord, depth), std::forward<Predicates>(predicates), d_first,
		             create_nodes);
	}

	template <class Predicates, class OutputIt>
	OutputIt query(coord_t x, coord_t y, coord_t z, depth_t depth, Predicates&& predicates,
	               OutputIt d_first, bool create_nodes = false) const
	{
		return query(toCode(x, y, z, depth), std::forward<Predicates>(predicates), d_first,
		             create_nodes);
	}

	template <class Predicates, class OutputIt>
	OutputIt queryK(std::size_t k, Predicates&& predicates, OutputIt d_first,
	                bool create_nodes = false) const
	{
		std::size_t count = 0;
		if constexpr (std::is_same_v<typename OutputIt::value_type, Node>) {
			for (auto it = beginQuery(std::forward<Predicates>(predicates), create_nodes);
			     count < k && it != endQuery(); ++it, ++count) {
				*d_first++ = *it;
			}
		} else {
			for (auto it = beginQueryBV(std::forward<Predicates>(predicates), create_nodes);
			     count < k && it != endQueryBV(); ++it, ++count) {
				*d_first++ = *it;
			}
		}
		return d_first;
	}

	template <class Predicates, class OutputIt>
	OutputIt queryK(Node node, std::size_t k, Predicates&& predicates, OutputIt d_first,
	                bool create_nodes = false) const
	{
		std::size_t count = 0;
		if constexpr (std::is_same_v<typename OutputIt::value_type, Node>) {
			for (auto it = beginQuery(node, std::forward<Predicates>(predicates), create_nodes);
			     count < k && it != endQuery(); ++it, ++count) {
				*d_first++ = *it;
			}
		} else {
			for (auto it =
			         beginQueryBV(node, std::forward<Predicates>(predicates), create_nodes);
			     count < k && it != endQueryBV(); ++it, ++count) {
				*d_first++ = *it;
			}
		}
		return d_first;
	}

	template <class Predicates, class OutputIt>
	OutputIt queryK(Code code, std::size_t k, Predicates&& predicates, OutputIt d_first,
	                bool create_nodes = false) const
	{
		std::size_t count = 0;
		if constexpr (std::is_same_v<typename OutputIt::value_type, Node>) {
			for (auto it = beginQuery(code, std::forward<Predicates>(predicates), create_nodes);
			     count < k && it != endQuery(); ++it, ++count) {
				*d_first++ = *it;
			}
		} else {
			for (auto it =
			         beginQueryBV(code, std::forward<Predicates>(predicates), create_nodes);
			     count < k && it != endQueryBV(); ++it, ++count) {
				*d_first++ = *it;
			}
		}
		return d_first;
	}

	template <class Predicates, class OutputIt>
	OutputIt queryK(Key key, std::size_t k, Predicates&& predicates, OutputIt d_first,
	                bool create_nodes = false) const
	{
		return queryK(toCode(key), k, std::forward<Predicates>(predicates), d_first,
		              create_nodes);
	}

	template <class Predicates, class OutputIt>
	OutputIt queryK(Point coord, depth_t depth, std::size_t k, Predicates&& predicates,
	                OutputIt d_first, bool create_nodes = false) const
	{
		return queryK(toCode(coord, depth), k, std::forward<Predicates>(predicates), d_first,
		              create_nodes);
	}

	template <class Predicates, class OutputIt>
	OutputIt queryK(coord_t x, coord_t y, coord_t z, depth_t depth, std::size_t k,
	                Predicates&& predicates, OutputIt d_first,
	                bool create_nodes = false) const
	{
		return queryK(toCode(x, y, z, depth), k, std::forward<Predicates>(predicates),
		              d_first, create_nodes);
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearest(Geometry&& geometry, Predicates&& predicates, OutputIt d_first,
	                      double epsilon = 0.0, bool create_nodes = false) const
	{
		return std::copy(
		    beginQueryNearest(std::forward<Geometry>(geometry),
		                      std::forward<Predicates>(predicates), epsilon, create_nodes),
		    endQueryNearest(), d_first);
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearest(Node node, Geometry&& geometry, Predicates&& predicates,
	                      OutputIt d_first, double epsilon = 0.0,
	                      bool create_nodes = false) const
	{
		return std::copy(
		    beginQueryNearest(node, std::forward<Geometry>(geometry),
		                      std::forward<Predicates>(predicates), epsilon, create_nodes),
		    endQueryNearest(), d_first);
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearest(Code code, Geometry&& geometry, Predicates&& predicates,
	                      OutputIt d_first, double epsilon = 0.0,
	                      bool create_nodes = false) const
	{
		return std::copy(
		    beginQueryNearest(code, std::forward<Geometry>(geometry),
		                      std::forward<Predicates>(predicates), epsilon, create_nodes),
		    endQueryNearest(), d_first);
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearest(Key key, Geometry&& geometry, Predicates&& predicates,
	                      OutputIt d_first, double epsilon = 0.0,
	                      bool create_nodes = false) const
	{
		return queryNearest(toCode(key), std::forward<Geometry>(geometry),
		                    std::forward<Predicates>(predicates), d_first, epsilon,
		                    create_nodes);
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearest(Point coord, depth_t depth, Geometry&& geometry,
	                      Predicates&& predicates, OutputIt d_first, double epsilon = 0.0,
	                      bool create_nodes = false) const
	{
		return queryNearest(toCode(coord, depth), std::forward<Geometry>(geometry),
		                    std::forward<Predicates>(predicates), d_first, epsilon,
		                    create_nodes);
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearest(coord_t x, coord_t y, coord_t z, depth_t depth,
	                      Geometry&& geometry, Predicates&& predicates, OutputIt d_first,
	                      double epsilon = 0.0, bool create_nodes = false) const
	{
		return queryNearest(toCode(x, y, z, depth), std::forward<Geometry>(geometry),
		                    std::forward<Predicates>(predicates), d_first, epsilon,
		                    create_nodes);
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearestK(std::size_t k, Geometry&& geometry, Predicates&& predicates,
	                       OutputIt d_first, double epsilon = 0.0,
	                       bool create_nodes = false) const
	{
		std::size_t count = 0;
		for (auto it = beginQueryNearest(std::forward<Geometry>(geometry),
		                                 std::forward<Predicates>(predicates), epsilon,
		                                 create_nodes);
		     count < k && it != endQueryNearest(); ++it, ++count) {
			*d_first++ = *it;
		}
		return d_first;
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearestK(Node node, std::size_t k, Geometry&& geometry,
	                       Predicates&& predicates, OutputIt d_first, double epsilon = 0.0,
	                       bool create_nodes = false) const
	{
		std::size_t count = 0;
		for (auto it = beginQueryNearest(node, std::forward<Geometry>(geometry),
		                                 std::forward<Predicates>(predicates), epsilon,
		                                 create_nodes);
		     count < k && it != endQueryNearest(); ++it, ++count) {
			*d_first++ = *it;
		}
		return d_first;
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearestK(Code code, std::size_t k, Geometry&& geometry,
	                       Predicates&& predicates, OutputIt d_first, double epsilon = 0.0,
	                       bool create_nodes = false) const
	{
		std::size_t count = 0;
		for (auto it = beginQueryNearest(code, std::forward<Geometry>(geometry),
		                                 std::forward<Predicates>(predicates), epsilon,
		                                 create_nodes);
		     count < k && it != endQueryNearest(); ++it, ++count) {
			*d_first++ = *it;
		}
		return d_first;
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearestK(Key key, std::size_t k, Geometry&& geometry,
	                       Predicates&& predicates, OutputIt d_first, double epsilon = 0.0,
	                       bool create_nodes = false) const
	{
		return queryNearestK(toCode(key), k, std::forward<Geometry>(geometry),
		                     std::forward<Predicates>(predicates), d_first, epsilon,
		                     create_nodes);
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearestK(Point coord, depth_t depth, std::size_t k, Geometry&& geometry,
	                       Predicates&& predicates, OutputIt d_first, double epsilon = 0.0,
	                       bool create_nodes = false) const
	{
		return queryNearestK(toCode(coord, depth), k, std::forward<Geometry>(geometry),
		                     std::forward<Predicates>(predicates), d_first, epsilon,
		                     create_nodes);
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearestK(coord_t x, coord_t y, coord_t z, depth_t depth, std::size_t k,
	                       Geometry&& geometry, Predicates&& predicates, OutputIt d_first,
	                       double epsilon = 0.0, bool create_nodes = false) const
	{
		return queryNearestK(toCode(x, y, z, depth), k, std::forward<Geometry>(geometry),
		                     std::forward<Predicates>(predicates), d_first, epsilon,
		                     create_nodes);
	}

	//
	// Query iterator
	//

	template <class Predicates>
	[[nodiscard]] const_query_iterator beginQuery(Predicates&& predicates,
	                                              bool create_nodes = false) const
	{
		if (create_nodes) {
			if constexpr (predicate::contains_spatial_predicate_v<std::decay_t<Predicates>>) {
				return const_query_iterator(
				    new Iterator<Node, false, Derived, NodeBV, std::decay_t<Predicates>>(
				        &derived(), rootNodeBV(), std::forward<Predicates>(predicates)));
			} else {
				return const_query_iterator(
				    new Iterator<Node, false, Derived, Node, std::decay_t<Predicates>>(
				        &derived(), rootNode(), std::forward<Predicates>(predicates)));
			}
		} else {
			if constexpr (predicate::contains_spatial_predicate_v<std::decay_t<Predicates>>) {
				return const_query_iterator(
				    new Iterator<Node, true, Derived, NodeBV, std::decay_t<Predicates>>(
				        &derived(), rootNodeBV(), std::forward<Predicates>(predicates)));
			} else {
				return const_query_iterator(
				    new Iterator<Node, true, Derived, Node, std::decay_t<Predicates>>(
				        &derived(), rootNode(), std::forward<Predicates>(predicates)));
			}
		}
	}

	template <class Predicates>
	[[nodiscard]] const_query_iterator beginQuery(Node node, Predicates&& predicates,
	                                              bool create_nodes = false) const
	{
		if (create_nodes) {
			if constexpr (predicate::contains_spatial_predicate_v<std::decay_t<Predicates>>) {
				return const_query_iterator(
				    new Iterator<Node, false, Derived, NodeBV, std::decay_t<Predicates>>(
				        &derived(), toNodeBV(node), std::forward<Predicates>(predicates)));
			} else {
				return const_query_iterator(
				    new Iterator<Node, false, Derived, Node, std::decay_t<Predicates>>(
				        &derived(), node, std::forward<Predicates>(predicates)));
			}
		} else {
			if constexpr (predicate::contains_spatial_predicate_v<std::decay_t<Predicates>>) {
				return const_query_iterator(
				    new Iterator<Node, true, Derived, NodeBV, std::decay_t<Predicates>>(
				        &derived(), toNodeBV(node), std::forward<Predicates>(predicates)));
			} else {
				return const_query_iterator(
				    new Iterator<Node, true, Derived, Node, std::decay_t<Predicates>>(
				        &derived(), node, std::forward<Predicates>(predicates)));
			}
		}
	}

	template <class Predicates>
	[[nodiscard]] const_query_iterator beginQuery(Code code, Predicates&& predicates,
	                                              bool create_nodes = false) const
	{
		return beginQuery(operator()(code), std::forward<Predicates>(predicates),
		                  create_nodes);
	}

	template <class Predicates>
	[[nodiscard]] const_query_iterator beginQuery(Key key, Predicates&& predicates,
	                                              bool create_nodes = false) const
	{
		return beginQuery(toCode(key), std::forward<Predicates>(predicates), create_nodes);
	}

	template <class Predicates>
	[[nodiscard]] const_query_iterator beginQuery(Point coord, depth_t depth,
	                                              Predicates&& predicates,
	                                              bool create_nodes = false) const
	{
		return beginQuery(toCode(coord, depth), std::forward<Predicates>(predicates),
		                  create_nodes);
	}

	template <class Predicates>
	[[nodiscard]] const_query_iterator beginQuery(coord_t x, coord_t y, coord_t z,
	                                              depth_t depth, Predicates&& predicates,
	                                              bool create_nodes = false) const
	{
		return beginQuery(toCode(x, y, z, depth), std::forward<Predicates>(predicates),
		                  create_nodes);
	}

	[[nodiscard]] const_query_iterator endQuery() const
	{
		return const_query_iterator(new Iterator<Node, true, Derived>(rootNode()));
	}

	template <class Predicates>
	[[nodiscard]] const_bounding_volume_query_iterator beginQueryBV(
	    Predicates&& predicates, bool create_nodes = false) const
	{
		if (create_nodes) {
			return const_bounding_volume_query_iterator(
			    new Iterator<NodeBV, false, Derived, NodeBV, Predicates>(
			        &derived(), rootNodeBV(), std::forward<Predicates>(predicates)));
		} else {
			return const_bounding_volume_query_iterator(
			    new Iterator<NodeBV, true, Derived, NodeBV, Predicates>(
			        &derived(), rootNodeBV(), std::forward<Predicates>(predicates)));
		}
	}

	template <class Predicates>
	[[nodiscard]] const_bounding_volume_query_iterator beginQueryBV(
	    Node node, Predicates&& predicates, bool create_nodes = false) const
	{
		if (create_nodes) {
			return const_bounding_volume_query_iterator(
			    new Iterator<NodeBV, false, Derived, NodeBV, Predicates>(
			        &derived(), toNodeBV(node), std::forward<Predicates>(predicates)));
		} else {
			return const_bounding_volume_query_iterator(
			    new Iterator<NodeBV, true, Derived, NodeBV, Predicates>(
			        &derived(), toNodeBV(node), std::forward<Predicates>(predicates)));
		}
	}

	template <class Predicates>
	[[nodiscard]] const_bounding_volume_query_iterator beginQueryBV(
	    Code code, Predicates&& predicates, bool create_nodes = false) const
	{
		return beginQueryBV(operator()(code), std::forward<Predicates>(predicates),
		                    create_nodes);
	}

	template <class Predicates>
	[[nodiscard]] const_bounding_volume_query_iterator beginQueryBV(
	    Key key, Predicates&& predicates, bool create_nodes = false) const
	{
		return beginQueryBV(toCode(key), std::forward<Predicates>(predicates), create_nodes);
	}

	template <class Predicates>
	[[nodiscard]] const_bounding_volume_query_iterator beginQueryBV(
	    Point coord, depth_t depth, Predicates&& predicates,
	    bool create_nodes = false) const
	{
		return beginQueryBV(toCode(coord, depth), std::forward<Predicates>(predicates),
		                    create_nodes);
	}

	template <class Predicates>
	[[nodiscard]] const_bounding_volume_query_iterator beginQueryBV(
	    coord_t x, coord_t y, coord_t z, depth_t depth, Predicates&& predicates,
	    bool create_nodes = false) const
	{
		return beginQueryBV(toCode(x, y, z, depth), std::forward<Predicates>(predicates),
		                    create_nodes);
	}

	[[nodiscard]] const_bounding_volume_query_iterator endQueryBV() const
	{
		return const_bounding_volume_query_iterator(
		    new Iterator<NodeBV, true, Derived, NodeBV>(rootNodeBV()));
	}

	template <class Geometry, class Predicates>
	[[nodiscard]] const_query_nearest_iterator beginQueryNearest(
	    Geometry&& geometry, Predicates&& predicates, double epsilon = 0.0,
	    bool create_nodes = false) const
	{
		if (create_nodes) {
			return const_query_nearest_iterator(new NearestIterator<false>(
			    &derived(), rootNodeBV(), std::forward<Geometry>(geometry),
			    std::forward<Predicates>(predicates), epsilon));
		} else {
			return const_query_nearest_iterator(new NearestIterator<true>(
			    &derived(), rootNodeBV(), std::forward<Geometry>(geometry),
			    std::forward<Predicates>(predicates), epsilon));
		}
	}

	template <class Geometry, class Predicates>
	[[nodiscard]] const_query_nearest_iterator beginQueryNearest(
	    Node node, Geometry&& geometry, Predicates&& predicates, double epsilon = 0.0,
	    bool create_nodes = false) const
	{
		if (create_nodes) {
			return const_query_nearest_iterator(new NearestIterator<false>(
			    &derived(), toNodeBV(node), std::forward<Geometry>(geometry),
			    std::forward<Predicates>(predicates), epsilon));
		} else {
			return const_query_nearest_iterator(new NearestIterator<true>(
			    &derived(), toNodeBV(node), std::forward<Geometry>(geometry),
			    std::forward<Predicates>(predicates), epsilon));
		}
	}

	template <class Geometry, class Predicates>
	[[nodiscard]] const_query_nearest_iterator beginQueryNearest(
	    Code code, Geometry&& geometry, Predicates&& predicates, double epsilon = 0.0,
	    bool create_nodes = false) const
	{
		return beginQueryNearest(operator()(code), std::forward<Geometry>(geometry),
		                         std::forward<Predicates>(predicates), epsilon, create_nodes);
	}

	template <class Geometry, class Predicates>
	[[nodiscard]] const_query_nearest_iterator beginQueryNearest(
	    Key key, Geometry&& geometry, Predicates&& predicates, double epsilon = 0.0,
	    bool create_nodes = false) const
	{
		return beginQueryNearest(toCode(key), std::forward<Geometry>(geometry),
		                         std::forward<Predicates>(predicates), epsilon, create_nodes);
	}

	template <class Geometry, class Predicates>
	[[nodiscard]] const_query_nearest_iterator beginQueryNearest(
	    Point coord, depth_t depth, Geometry&& geometry, Predicates&& predicates,
	    double epsilon = 0.0, bool create_nodes = false) const
	{
		return beginQueryNearest(toCode(coord, depth), std::forward<Geometry>(geometry),
		                         std::forward<Predicates>(predicates), epsilon, create_nodes);
	}

	template <class Geometry, class Predicates>
	[[nodiscard]] const_query_nearest_iterator beginQueryNearest(
	    coord_t x, coord_t y, coord_t z, depth_t depth, Geometry&& geometry,
	    Predicates&& predicates, double epsilon = 0.0, bool create_nodes = false) const
	{
		return beginQueryNearest(toCode(x, y, z, depth), std::forward<Geometry>(geometry),
		                         std::forward<Predicates>(predicates), epsilon, create_nodes);
	}

	[[nodiscard]] const_query_nearest_iterator endQueryNearest() const
	{
		return const_query_nearest_iterator(new NearestIterator<true, Derived>());
	}

	//
	// "Normal" iterator
	//

	[[nodiscard]] const_iterator begin(bool create_nodes = false) const
	{
		return beginQuery(predicate::TRUE{}, create_nodes);
	}

	[[nodiscard]] const_iterator begin(Node node, bool create_nodes = false) const
	{
		return beginQuery(node, predicate::TRUE{}, create_nodes);
	}

	[[nodiscard]] const_iterator begin(Code code, bool create_nodes = false) const
	{
		return beginQuery(code, predicate::TRUE{}, create_nodes);
	}

	[[nodiscard]] const_iterator begin(Key key, bool create_nodes = false) const
	{
		return begin(toCode(key), create_nodes);
	}

	[[nodiscard]] const_iterator begin(Point coord, depth_t depth,
	                                   bool create_nodes = false) const
	{
		return begin(toCode(coord, depth), create_nodes);
	}

	[[nodiscard]] const_iterator begin(coord_t x, coord_t y, coord_t z, depth_t depth,
	                                   bool create_nodes = false) const
	{
		return begin(toCode(x, y, z, depth), create_nodes);
	}

	[[nodiscard]] const_iterator end() const { return endQuery(predicate::TRUE{}); }

	[[nodiscard]] const_bounding_volume_iterator beginBV(bool create_nodes = false) const
	{
		return beginQueryBV(predicate::TRUE{}, create_nodes);
	}

	[[nodiscard]] const_iterator beginBV(Node node, bool create_nodes = false) const
	{
		return beginQueryBV(node, predicate::TRUE{}, create_nodes);
	}

	[[nodiscard]] const_iterator beginBV(Code code, bool create_nodes = false) const
	{
		return beginQueryBV(code, predicate::TRUE{}, create_nodes);
	}

	[[nodiscard]] const_iterator beginBV(Key key, bool create_nodes = false) const
	{
		return beginBV(toCode(key), create_nodes);
	}

	[[nodiscard]] const_iterator beginBV(Point coord, depth_t depth,
	                                     bool create_nodes = false) const
	{
		return beginBV(toCode(coord, depth), create_nodes);
	}

	[[nodiscard]] const_iterator beginBV(coord_t x, coord_t y, coord_t z, depth_t depth,
	                                     bool create_nodes = false) const
	{
		return beginBV(toCode(x, y, z, depth), create_nodes);
	}

	[[nodiscard]] const_bounding_volume_iterator endBV() const
	{
		return endQueryBV(predicate::TRUE{});
	}

	/**************************************************************************************
	|                                                                                     |
	|                                         I/O                                         |
	|                                                                                     |
	**************************************************************************************/

	void read(std::filesystem::path const& path, bool propagate = true)
	{
		std::ifstream file;
		file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		file.imbue(std::locale());
		file.open(path, std::ios_base::in | std::ios_base::binary);

		read(file, propagate);
	}

	void read(std::istream& in, bool propagate = true)
	{
		readData(in, readHeader(in), propagate);
	}

	void readData(std::istream& in, FileHeader const& header, bool propagate = true)
	{
		if (size() != header.leaf_size || depthLevels() != header.depth_levels) {
			clear(header.leaf_size, header.depth_levels);
		}

		auto nodes = readNodes(in);

		derived().readNodes(in, std::begin(nodes), nodes.size(), header.compressed);

		if (propagate) {
			propagateModified();
		}
	}

	void write(std::filesystem::path const& path, depth_t min_depth = 0,
	           bool compress = false, int compression_acceleration_level = 1,
	           int compression_level = 0) const
	{
		write(path, predicate::TRUE(), min_depth, compress, compression_acceleration_level,
		      compression_level);
	}

	void write(std::ostream& out, depth_t min_depth = 0, bool compress = false,
	           int compression_acceleration_level = 1, int compression_level = 0) const
	{
		write(out, predicate::TRUE(), min_depth, compress, compression_acceleration_level,
		      compression_level);
	}

	template <class Predicates,
	          typename = std::enable_if_t<!std::is_scalar_v<std::decay_t<Predicates>>>>
	void write(std::filesystem::path const& path, Predicates&& predicates,
	           depth_t min_depth = 0, bool compress = false,
	           int compression_acceleration_level = 1, int compression_level = 0) const
	{
		std::ofstream file;
		file.exceptions(std::ofstream::failbit | std::ofstream::badbit);
		file.imbue(std::locale());
		file.open(path, std::ios_base::out | std::ios_base::binary);

		write(file, std::forward<Predicates>(predicates), min_depth, compress,
		      compression_acceleration_level, compression_level);
	}

	template <class Predicates,
	          typename = std::enable_if_t<!std::is_scalar_v<std::decay_t<Predicates>>>>
	void write(std::ostream& out, Predicates&& predicates, depth_t min_depth = 0,
	           bool compress = false, int compression_acceleration_level = 1,
	           int compression_level = 0) const
	{
		auto [tree_structure, nodes] =
		    data(predicate::Leaf(min_depth) && std::forward<Predicates>(predicates));
		write(out, tree_structure, std::cbegin(nodes), nodes.size(), compress,
		      compression_acceleration_level, compression_level);
	}

	void writeModifiedAndPropagate(std::filesystem::path const& filename,
	                               bool compress = false,
	                               int compression_acceleration_level = 1,
	                               int compression_level = 0)
	{
		std::ofstream file;
		file.exceptions(std::ofstream::failbit | std::ofstream::badbit);
		file.imbue(std::locale());
		file.open(filename, std::ios_base::out | std::ios_base::binary);

		writeModifiedAndPropagate(file, compress, compression_acceleration_level,
		                          compression_level);
	}

	void writeModifiedAndPropagate(std::ostream& out, bool compress = false,
	                               int compression_acceleration_level = 1,
	                               int compression_level = 0)
	{
		auto [tree_structure, nodes] = modifiedData<true>();
		write(out, tree_structure, std::cbegin(nodes), nodes.size(), compress,
		      compression_acceleration_level, compression_level);
	}

	void writeModifiedAndReset(std::filesystem::path const& filename, bool compress = false,
	                           int compression_acceleration_level = 1,
	                           int compression_level = 0)
	{
		std::ofstream file;
		file.exceptions(std::ofstream::failbit | std::ofstream::badbit);
		file.imbue(std::locale());
		file.open(filename, std::ios_base::out | std::ios_base::binary);

		writeModifiedAndReset(file, compress, compression_acceleration_level,
		                      compression_level);
	}

	void writeModifiedAndReset(std::ostream& out, bool compress = false,
	                           int compression_acceleration_level = 1,
	                           int compression_level = 0)
	{
		auto [tree_structure, nodes] = modifiedData<false>();
		write(out, tree_structure, std::cbegin(nodes), nodes.size(), compress,
		      compression_acceleration_level, compression_level);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Statistics                                      |
	|                                                                                     |
	**************************************************************************************/

	/*!
	 * @return Number of inner nodes in the octree.
	 */
	[[nodiscard]] constexpr std::size_t numInnerNodes() const noexcept
	{
		return num_inner_nodes_;
	}

	/*!
	 * @return Number of inner leaf nodes in the octree.
	 */
	[[nodiscard]] constexpr std::size_t numInnerLeafNodes() const noexcept
	{
		return num_inner_leaf_nodes_;
	}

	/*!
	 * @return Number of leaf nodes in the octree.
	 */
	[[nodiscard]] constexpr std::size_t numLeafNodes() const noexcept
	{
		return num_leaf_nodes_;
	}

	/*!
	 * @return Number of nodes in the octree.
	 */
	[[nodiscard]] constexpr std::size_t numNodes() const noexcept
	{
		return numInnerNodes() + numInnerLeafNodes() + numLeafNodes();
	}

	/*!
	 * @brief This is lower bound memory usage of an inner node.
	 *
	 * @note Additional data accessed by pointers inside an inner node are not counted.
	 *
	 * @return Memory usage of a single inner node.
	 */
	[[nodiscard]] constexpr std::size_t memoryInnerNode() const noexcept
	{
		return sizeof(InnerNode) / 8;
	}

	/*!
	 * @brief This is lower bound memory usage of an inner leaf node.
	 *
	 * @note Additional data accessed by pointers inside an inner leaf node are not counted.
	 *
	 * @return Memory usage of a single inner leaf node.
	 */
	[[nodiscard]] constexpr std::size_t memoryInnerLeafNode() const noexcept
	{
		return sizeof(InnerNode) / 8;
	}

	/*!
	 * @brief This is lower bound memory usage of a leaf node.
	 *
	 * @note Additional data accessed by pointers inside a leaf node are not counted.
	 *
	 * @return Memory usage of a single leaf node.
	 */
	[[nodiscard]] constexpr std::size_t memoryLeafNode() const noexcept
	{
		return sizeof(LeafNode) / 8;
	}

	/*!
	 * @brief Lower bound memory usage for all nodes.
	 *
	 * @note Does not account for pointed to data inside nodes.
	 *
	 * @return Memory usage of the octree.
	 */
	[[nodiscard]] std::size_t memoryUsage() const noexcept
	{
		return (numInnerNodes() * memoryInnerNode()) +
		       (numInnerLeafNodes() * memoryInnerLeafNode()) +
		       (numLeafNodes() * memoryLeafNode());
	}

	/*!
	 * @return Number of allocated inner nodes.
	 */
	[[nodiscard]] constexpr std::size_t numInnerNodesAllocated() const noexcept
	{
		return num_allocated_inner_nodes_;
	}

	/*!
	 * @return Number of allocated inner leaf nodes.
	 */
	[[nodiscard]] constexpr std::size_t numInnerLeafNodesAllocated() const noexcept
	{
		return num_allocated_inner_leaf_nodes_;
	}

	/*!
	 * @return Number of allocated leaf nodes.
	 */
	[[nodiscard]] constexpr std::size_t numLeafNodesAllocated() const noexcept
	{
		return num_allocated_leaf_nodes_;
	}

	/*!
	 * @return Number of allocated nodes.
	 */
	[[nodiscard]] constexpr std::size_t numNodesAllocated() const noexcept
	{
		return numInnerNodesAllocated() + numInnerLeafNodesAllocated() +
		       numLeafNodesAllocated();
	}

	/*!
	 * @brief Lower bound memory usage for all allocated nodes.
	 *
	 * @note Does not account for pointed to data inside nodes.
	 *
	 * @return Memory usage of the allocated octree.
	 */
	[[nodiscard]] std::size_t memoryUsageAllocated() const noexcept
	{
		return (numInnerNodesAllocated() * memoryInnerNode()) +
		       (numInnerLeafNodesAllocated() * memoryInnerLeafNode()) +
		       (numLeafNodesAllocated() * memoryLeafNode());
	}

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	OctreeBase(node_size_t leaf_node_size = 0.1, depth_t depth_levels = 16,
	           bool automatic_pruning = true)
	    : automatic_prune_(automatic_pruning)
	{
		setNodeSizeAndDepthLevels(leaf_node_size, depth_levels);

		init();
	}

	OctreeBase(OctreeBase const& other)
	    : depth_levels_(other.depth_levels_),
	      max_value_(other.max_value_),
	      node_size_(other.node_size_),
	      node_size_factor_(other.node_size_factor_),
	      automatic_prune_(other.automatic_prune_)
	{
		init();
	}

	OctreeBase(OctreeBase&& other)
	    : depth_levels_(std::move(other.depth_levels_)),
	      max_value_(std::move(other.max_value_)),
	      root_(std::move(other.root_)),
	      node_size_(std::move(other.node_size_)),
	      node_size_factor_(std::move(other.node_size_factor_)),
	      automatic_prune_(std::move(other.automatic_prune_))
	{
		num_inner_nodes_.store(other.num_inner_nodes_);
		num_inner_leaf_nodes_.store(other.num_inner_leaf_nodes_);
		num_leaf_nodes_.store(other.num_leaf_nodes_);

		num_allocated_inner_nodes_.store(other.num_allocated_inner_nodes_);
		num_allocated_inner_leaf_nodes_.store(other.num_allocated_inner_leaf_nodes_);
		num_allocated_leaf_nodes_.store(other.num_allocated_leaf_nodes_);

		init();
	}

	template <class Derived2, class Data2, class InnerData2, bool ReuseNodes2,
	          bool LockLess2, bool CountNodes2>
	OctreeBase(OctreeBase<Derived2, Data2, InnerData2, ReuseNodes2, LockLess2,
	                      CountNodes2> const& other)
	    : depth_levels_(other.depth_levels_),
	      max_value_(other.max_value_),
	      node_size_(other.node_size_),
	      node_size_factor_(other.node_size_factor_),
	      automatic_prune_(other.automatic_prune_)
	{
		init();
	}

	//
	// Init
	//

	void init()
	{
		for (auto& a_l : children_locks_) {
			a_l.clear();
		}
	}

	//
	// Destructor
	//

	~OctreeBase() { deleteChildren(root(), rootDepth(), true); }

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	OctreeBase& operator=(OctreeBase const& rhs)
	{
		// TODO: Should this clear?
		clear(rhs.size(), rhs.depthLevels());

		depth_levels_ = rhs.depth_levels_;
		max_value_ = rhs.max_value_;
		node_size_ = rhs.node_size_;
		node_size_factor_ = rhs.node_size_factor_;
		automatic_prune_ = rhs.automatic_prune_;
		return *this;
	}

	OctreeBase& operator=(OctreeBase&& rhs)
	{
		// TODO: Should this clear?
		clear(rhs.size(), rhs.depthLevels(), true);

		depth_levels_ = std::move(rhs.depth_levels_);
		max_value_ = std::move(rhs.max_value_);
		root_ = std::move(rhs.root_);
		node_size_ = std::move(rhs.node_size_);
		node_size_factor_ = std::move(rhs.node_size_factor_);
		automatic_prune_ = std::move(rhs.automatic_prune_);
		num_inner_nodes_.store(rhs.num_inner_nodes_);
		num_inner_leaf_nodes_.store(rhs.num_inner_leaf_nodes_);
		num_leaf_nodes_.store(rhs.num_leaf_nodes_);
		num_allocated_inner_nodes_.store(rhs.num_allocated_inner_nodes_);
		num_allocated_inner_leaf_nodes_.store(rhs.num_allocated_inner_leaf_nodes_);
		num_allocated_leaf_nodes_.store(rhs.num_allocated_leaf_nodes_);
		return *this;
	}

	template <class Derived2, class Data2, class InnerData2, bool ReuseNodes2,
	          bool LockLess2, bool CountNodes2>
	OctreeBase& operator=(OctreeBase<Derived2, Data2, InnerData2, ReuseNodes2, LockLess2,
	                                 CountNodes2> const& rhs)
	{
		// TODO: Should this clear?
		clear(rhs.size(), rhs.depthLevels());

		depth_levels_ = rhs.depth_levels_;
		max_value_ = rhs.max_value_;
		node_size_ = rhs.node_size_;
		node_size_factor_ = rhs.node_size_factor_;
		automatic_prune_ = rhs.automatic_prune_;
		return *this;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                         Swap                                        |
	|                                                                                     |
	**************************************************************************************/

	void swap(OctreeBase& other)
	{
		std::swap(depth_levels_, other.depth_levels_);
		std::swap(max_value_, other.max_value_);
		std::swap(root_, other.root_);
		std::swap(node_size_, other.node_size_);
		std::swap(node_size_factor_, other.node_size_factor_);
		std::swap(automatic_prune_, other.automatic_prune_);
		// TODO: Check locks
		std::swap(free_inner_blocks_, other.free_inner_blocks_);
		std::swap(free_leaf_blocks_, other.free_leaf_blocks_);

		num_inner_nodes_ = other.num_inner_nodes_.exchange(num_inner_nodes_);
		num_inner_leaf_nodes_ = other.num_inner_leaf_nodes_.exchange(num_inner_leaf_nodes_);
		num_leaf_nodes_ = other.num_leaf_nodes_.exchange(num_leaf_nodes_);
		num_allocated_inner_nodes_ =
		    other.num_allocated_inner_nodes_.exchange(num_allocated_inner_nodes_);
		num_allocated_inner_leaf_nodes_ =
		    other.num_allocated_inner_leaf_nodes_.exchange(num_allocated_inner_leaf_nodes_);
		num_allocated_leaf_nodes_ =
		    other.num_allocated_leaf_nodes_.exchange(num_allocated_leaf_nodes_);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Derived                                       |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	[[nodiscard]] constexpr Derived& derived() { return *static_cast<Derived*>(this); }

	[[nodiscard]] constexpr Derived const& derived() const
	{
		return *static_cast<Derived const*>(this);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Octree                                        |
	|                                                                                     |
	**************************************************************************************/

	void setNodeSizeAndDepthLevels(node_size_t const leaf_node_size,
	                               depth_t const depth_levels)
	{
		if (minDepthLevels() > depth_levels || maxDepthLevels() < depth_levels) {
			throw std::invalid_argument("depth_levels have to be in range [" +
			                            std::to_string(+minDepthLevels()) + ".." +
			                            std::to_string(+maxDepthLevels()) + "], '" +
			                            std::to_string(+depth_levels) + "' was supplied.");
		}

		depth_levels_ = depth_levels;
		max_value_ = std::pow(2, depth_levels - 1);

		// For increased precision
		node_size_[0] = leaf_node_size;
		int const s = node_size_.size();
		for (int i = 1; s != i; ++i) {
			node_size_[i] = std::ldexp(leaf_node_size, i);
		}

		std::transform(std::cbegin(node_size_), std::cend(node_size_),
		               std::begin(node_size_factor_), [](auto const n) { return 1.0 / n; });
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Root                                         |
	|                                                                                     |
	**************************************************************************************/

	/*!
	 * @brief Initilize the root node.
	 */
	void initRoot()
	{
		root().leaf.set();
		root().modified.reset();
	}

	/*!
	 * @brief Get the root node.
	 *
	 * @return The root node.
	 */
	[[nodiscard]] constexpr InnerNode const& root() const noexcept { return root_; }

	/*!
	 * @brief Get the root node.
	 *
	 * @return The root node.
	 */
	[[nodiscard]] constexpr InnerNode& root() noexcept { return root_; }

	/*!
	 * @brief Get the root node's index field.
	 *
	 * @return The root node's index field.
	 */
	[[nodiscard]] constexpr IndexField rootIndexField() const noexcept
	{
		return IndexField(1);
	}

	/*!
	 * @brief Get the root node index.
	 *
	 * @return The root node index.
	 */
	[[nodiscard]] constexpr index_t rootIndex() const noexcept { return index_t(0); }

	/**************************************************************************************
	|                                                                                     |
	|                                        Apply                                        |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	template <class BinaryFunction, class UnaryFunction,
	          typename = std::enable_if_t<std::is_copy_constructible_v<BinaryFunction> &&
	                                      std::is_copy_constructible_v<UnaryFunction>>>
	void apply(Node node, BinaryFunction f, UnaryFunction f2, bool const propagate)
	{
		if (node.depth() == node.realDepth()) {
			if (isLeaf(node)) {
				f(leafNode(node), node.index());
			} else {
				applyAllRecurs(innerNode(node), node.index(), node.depth(), f, f2);
			}

			if (leafNode(node).modified.none()) {
				setModifiedParents(node.code());
			}

			leafNode(node).modified.set(node.index());
		} else {
			applyRecurs(innerNode(node), node.realDepth(), node.code(), f, f2);
		}

		if (propagate) {
			propagateModified();
		}
	}

	template <class BinaryFunction, class UnaryFunction,
	          typename = std::enable_if_t<std::is_copy_constructible_v<BinaryFunction> &&
	                                      std::is_copy_constructible_v<UnaryFunction>>>
	void apply(Code code, BinaryFunction f, UnaryFunction f2, bool const propagate)
	{
		// FIXME: Should this be here?
		if (code.depth() > rootDepth()) {
			return;
		}

		applyRecurs(root(), rootDepth(), code, f, f2);

		if (propagate) {
			propagateModified();
		}
	}

	template <class BinaryFunction, class UnaryFunction,
	          typename = std::enable_if_t<std::is_copy_constructible_v<BinaryFunction> &&
	                                      std::is_copy_constructible_v<UnaryFunction>>>
	void applyRecurs(InnerNode& node, depth_t depth, Code code, BinaryFunction f,
	                 UnaryFunction f2)
	{
		auto index = code.index(depth);
		auto child_index = code.index(depth - 1);

		if (code.depth() == depth) {
			if (node.leaf[index]) {
				f(static_cast<LeafNode&>(node), index);
			} else {
				applyAllRecurs(node, index, depth, f, f2);
			}
		} else if (1 == depth) {
			// TODO: Implement
			createLeafChildren(node, ...);
			LeafNode& child = leafChild(node, code.index(0));
			f(child);
			setModified(child, true);
		} else {
			// TODO: Implement
			createInnerChildren(node, depth);
			applyRecurs(innerChild(node, code.index(depth - 1)), depth - 1, code, f, f2);
		}

		node.modified.set(index);
	}

	template <class BinaryFunction, class UnaryFunction,
	          typename = std::enable_if_t<std::is_copy_constructible_v<BinaryFunction> &&
	                                      std::is_copy_constructible_v<UnaryFunction>>>
	void applyAllRecurs(InnerNode& node, index_t index, depth_t depth, BinaryFunction f,
	                    UnaryFunction f2)
	{
		if (1 == depth) {
			auto& children = leafChild(node, index);
			f2(children);
			children.modified.set();
		} else {
			auto& children = innerChild(node, index);
			if (children.leaf.all()) {
				f2(children);
			} else {
				for (index_t i = 0; 8 != i; ++i) {
					if (children.leaf[i]) {
						f(static_cast<LeafNode&>(children), i);
					} else {
						applyAllRecurs(innerChild(children, i), i, depth - 1, f, f2);
					}
				}
			}
			children.modified.set();
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Traverse                                       |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comment

	template <class NodeType, class UnaryFunction>
	void traverseRecurs(NodeType const& node, UnaryFunction f) const
	{
		if (f(node) || isLeaf(node)) {
			return;
		}

		node = child(node, 0);
		for (index_t index = 0; 8 != index; ++index) {
			traverseRecurs(sibling(node, index), f);
		}
	}

	template <class Geometry, class UnaryFunction>
	void traverseNearestRecurs(NodeBV const& node, Geometry const& g, UnaryFunction f) const
	{
		// TODO: Implement
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Conversion                                      |
	|                                                                                     |
	**************************************************************************************/

	//
	// Key
	//

	/*!
	 * @brief Convert a coordinate at a specific depth to a key.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The corresponding key.
	 */
	[[nodiscard]] constexpr key_t toKey(coord_t coord, depth_t depth = 0) const
	{
		// FIXME: Make it possible to make a cloud of coordinates
		key_t val = std::floor(node_size_factor_[0] * coord);
		return ((val + max_value_) >> depth) << depth;
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a key with bounds check.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The corresponding key.
	 */
	[[nodiscard]] constexpr std::optional<key_t> toKeyChecked(coord_t coord,
	                                                          depth_t depth = 0) const
	{
		auto min = -size(rootDepth() - 1);
		auto max = -min;
		return min <= coord && max >= coord ? std::optional<key_t>(toKey(coord, depth))
		                                    : std::nullopt;
	}

	//
	// Coordinate
	//

	/*!
	 * @brief Convert a key to a coordinate at a specific depth.
	 *
	 * @param key The key.
	 * @param depth The depth of the coordinate.
	 * @return The corresponding coordinate.
	 */
	[[nodiscard]] constexpr coord_t toCoord(key_t key, depth_t depth = 0) const
	{
		constexpr auto sub64 = std::minus<int_fast64_t>{};
		return depth_levels_ == depth
		           ? 0
		           : (std::floor(sub64(key, max_value_) / static_cast<coord_t>(1U << depth)) +
		              coord_t(0.5)) *
		                 size(depth);
		//  FIXME: Look at
		// return rootDepth() == depth
		//            ? 0
		//            : ((sub64(key, max_value_) >> depth) + coord_t(0.5)) * size(depth);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                    Access nodes                                     |
	|                                                                                     |
	**************************************************************************************/

	//
	// Get node
	//

	[[nodiscard]] constexpr LeafNode const& leafNode(Node node) const
	{
		if (node.isReal()) {
			return *static_cast<LeafNode*>(node.data());
		} else {
			// TODO: Implement
		}
	}

	[[nodiscard]] constexpr LeafNode& leafNode(Node node)
	{
		if (node.isReal()) {
			return *static_cast<LeafNode*>(node.data());
		} else {
			// TODO: Implement
		}
	}

	[[nodiscard]] LeafNode const& leafNode(Code code) const
	{
		InnerNode const* node = &root();
		depth_t depth = rootDepth();
		depth_t min_depth = std::max(code.depth(), depth_t(1));
		while (min_depth != depth && node->isParentIndex(code.index(depth))) {
			node = &innerNode(*node, code.index(depth));
			--depth;
		}

		return 0 == code.depth() && node->isParentIndex(code.index(depth))
		           ? leafNode(*node, code.index(1))
		           : *node;
	}

	[[nodiscard]] LeafNode& leafNode(Code code)
	{
		InnerNode* node = &root();
		depth_t depth = rootDepth();
		depth_t min_depth = std::max(code.depth(), depth_t(1));
		while (min_depth != depth && node->isParentIndex(code.index(depth))) {
			node = &innerNode(*node, code.index(depth));
			--depth;
		}

		return 0 == code.depth() && node->isParentIndex(code.index(depth))
		           ? leafNode(*node, code.index(1))
		           : *node;
	}

	[[nodiscard]] constexpr InnerNode const& innerNode(Node node) const
	{
		if (node.isReal()) {
			return *static_cast<InnerNode*>(node.data());
		} else {
			// TODO: Implement
		}
	}

	[[nodiscard]] constexpr InnerNode& innerNode(Node& node)
	{
		if (node.isReal()) {
			return *static_cast<InnerNode*>(node.data());
		} else {
			// TODO: Implement
		}
	}

	[[nodiscard]] InnerNode const& innerNode(Code code) const
	{
		assert(0 != code.depth());

		InnerNode const* node = &root();
		depth_t depth = rootDepth();
		depth_t min_depth = code.depth();
		while (min_depth != depth && isParent(*node)) {
			node = &innerChild(*node, code.index(depth));
			--depth;
		}

		return *node;
	}

	[[nodiscard]] InnerNode& innerNode(Code code)
	{
		assert(0 != code.depth());

		InnerNode* node = &root();
		depth_t depth = rootDepth();
		depth_t min_depth = code.depth();
		while (min_depth != depth && isParent(*node)) {
			node = &innerChild(*node, code.index(depth));
			--depth;
		}

		return *node;
	}

	[[nodiscard]] std::pair<LeafNode const&, depth_t> leafNodeAndDepth(Code code) const
	{
		InnerNode const* node = &root();
		depth_t depth = rootDepth();
		depth_t min_depth = std::max(code.depth(), depth_t(1));
		while (min_depth != depth && isParent(*node)) {
			node = &innerChild(*node, code.index(depth));
			--depth;
		}

		return 0 == code.depth() && isParent(*node)
		           ? std::make_pair<LeafNode const&, depth_t>(leafChild(*node, code.index()),
		                                                      0)
		           : std::make_pair<LeafNode const&, depth_t>(*node, depth);
	}

	[[nodiscard]] std::pair<LeafNode&, depth_t> leafNodeAndDepth(Code code)
	{
		InnerNode* node = &root();
		depth_t depth = rootDepth();
		depth_t min_depth = std::max(code.depth(), depth_t(1));
		while (min_depth != depth && isParent(*node)) {
			node = &innerChild(*node, code.index(depth));
			--depth;
		}

		return 0 == code.depth() && isParent(*node)
		           ? std::make_pair<LeafNode&, depth_t>(leafChild(*node, code.index(0)), 0)
		           : std::make_pair<LeafNode&, depth_t>(*node, depth);
	}

	[[nodiscard]] std::pair<InnerNode const&, depth_t> innerNodeAndDepth(Code code) const
	{
		InnerNode const* node = &root();
		depth_t depth = rootDepth();
		depth_t min_depth = std::max(code.depth(), depth_t(1));
		while (min_depth != depth && isParent(*node)) {
			node = &innerChild(*node, code.index(depth));
			--depth;
		}

		return {*node, depth};
	}

	[[nodiscard]] std::pair<InnerNode&, depth_t> innerNodeAndDepth(Code code)
	{
		InnerNode* node = &root();
		depth_t depth = rootDepth();
		depth_t min_depth = std::max(code.depth(), depth_t(1));
		while (min_depth != depth && isParent(*node)) {
			node = &innerChild(*node, code.index(depth));
			--depth;
		}

		return {*node, depth};
	}

	//
	// Create node
	//

	[[nodiscard]] LeafNode& createLeafNode(Code code, bool set_modified = false)
	{
		InnerNode* node = &root();
		if (set_modified) {
			node->setModifiedIndex(rootIndex());
		}
		auto const min_depth = std::max(depth_t(1), code.depth());
		for (depth_t depth = rootDepth(); min_depth < depth; --depth) {
			auto const index = code.index(depth);
			if (node->isLeafIndex(index)) {
				createInnerChildren(*node, index, depth);
			}
			node = &innerChild(*node, index);
			if (set_modified) {
				node->setModifiedIndex(code.index(depth - 1));
			}
		}

		if (0 == code.depth()) {
			auto const index = code.index(1);
			if (node->isLeafIndex(index)) {
				createLeafChildren(*node, index);
			}
			LeafNode& n = leafChild(*node, index);
			if (set_modified) {
				n.setModifiedIndex(code.index(0));
			}
			return n;
		} else {
			return *node;
		}
	}

	[[nodiscard]] InnerNode& createInnerNode(Code code, bool set_modified = false)
	{
		InnerNode* node = &root();
		if (set_modified) {
			node->setModifiedIndex(rootIndex());
		}
		auto const min_depth = std::max(depth_t(1), code.depth());
		for (depth_t depth = rootDepth(); min_depth < depth; --depth) {
			auto const index = code.index(depth);
			if (node->isLeafIndex(index)) {
				createInnerChildren(*node, index, depth);
			}
			node = &innerChild(*node, index);
			if (set_modified) {
				node->setModifiedIndex(code.index(depth - 1));
			}
		}
		return *node;
	}

	//
	// Get children
	//

	[[nodiscard]] static constexpr LeafNodeBlock const& leafChildren(InnerNode const& node)
	{
		return *node.leaf_children;
	}

	[[nodiscard]] static constexpr LeafNodeBlock& leafChildren(InnerNode& node)
	{
		return *node.leaf_children;
	}

	[[nodiscard]] static constexpr InnerNodeBlock const& innerChildren(
	    InnerNode const& node)
	{
		return *node.inner_children;
	}

	[[nodiscard]] static constexpr InnerNodeBlock& innerChildren(InnerNode& node)
	{
		return *node.inner_children;
	}

	[[nodiscard]] static constexpr LeafNode const& leafChild(InnerNode const& parent,
	                                                         index_t child_index)
	{
		return leafChildren(parent)[child_index];
	}

	[[nodiscard]] static constexpr LeafNode& leafChild(InnerNode& parent,
	                                                   index_t child_index)
	{
		return leafChildren(parent)[child_index];
	}

	[[nodiscard]] static constexpr InnerNode const& innerChild(InnerNode const& parent,
	                                                           index_t child_index)
	{
		return innerChildren(parent)[child_index];
	}

	[[nodiscard]] static constexpr InnerNode& innerChild(InnerNode& parent,
	                                                     index_t child_index)
	{
		return innerChildren(parent)[child_index];
	}

	[[nodiscard]] static constexpr LeafNode const& child(InnerNode const& parent,
	                                                     index_t child_index,
	                                                     depth_t parent_depth)
	{
		return 1 == parent_depth ? leafChild(parent, child_index)
		                         : innerChild(parent, child_index);
	}

	[[nodiscard]] static constexpr LeafNode& child(InnerNode& parent, index_t child_index,
	                                               depth_t parent_depth)
	{
		return 1 == parent_depth ? leafChild(parent, child_index)
		                         : innerChild(parent, child_index);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Center                                        |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	//
	// Child center
	//

	[[nodiscard]] static constexpr Point childCenter(Point parent_center,
	                                                 node_size_t child_half_size,
	                                                 index_t child_index)
	{
		parent_center[0] += child_index & index_t(1) ? child_half_size : -child_half_size;
		parent_center[1] += child_index & index_t(2) ? child_half_size : -child_half_size;
		parent_center[2] += child_index & index_t(4) ? child_half_size : -child_half_size;
		return parent_center;
	}

	//
	// Sibling center
	//

	[[nodiscard]] static constexpr Point siblingCenter(Point center, node_size_t half_size,
	                                                   index_t index, index_t sibling_index)
	{
		index_t const temp = index ^ sibling_index;
		node_size_t const size = 2 * half_size;
		if (temp & index_t(1)) {
			center[0] += sibling_index & index_t(1) ? size : -size;
		}
		if (temp & index_t(2)) {
			center[1] += sibling_index & index_t(2) ? size : -size;
		}
		if (temp & index_t(4)) {
			center[2] += sibling_index & index_t(4) ? size : -size;
		}
		return center;
	}

	//
	// Parent center
	//

	[[nodiscard]] static constexpr Point parentCenter(Point child_center,
	                                                  node_size_t child_half_size,
	                                                  index_t child_index)
	{
		child_center[0] -= child_index & index_t(1) ? child_half_size : -child_half_size;
		child_center[1] -= child_index & index_t(2) ? child_half_size : -child_half_size;
		child_center[2] -= child_index & index_t(4) ? child_half_size : -child_half_size;
		return child_center;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Leaf                                         |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Implement

	/**************************************************************************************
	|                                                                                     |
	|                                      Modified                                       |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Implement

	//
	// Set modified
	//

	void setModified(LeafNode& node, index_t index) { node.modified.set(index); }

	void setModified(InnerNode& node, index_t index, depth_t depth, depth_t min_depth)
	{
		if (min_depth <= depth) {
			node.modified.set(index);
		}

		if (min_depth < depth) {
			if (1 == depth) {
				leafChild(node, index).modified.set();
			} else {
				setModifiedRecurs(innerChild(node, index), depth - 1, min_depth);
			}
		}
	}

	void setModifiedRecurs(InnerNode& node, depth_t depth, depth_t min_depth)
	{
		node.modified.set();

		if (node.leaf.all() || depth == min_depth) {
			return;
		}

		if (1 == depth) {
			// All are allocated so we can just set all to modified, even if they are leaves
			for (auto& child : leafChildren(node)) {
				child.modified.set();
			}
		} else {
			for (index_t i = 0; 8 != i; ++i) {
				if (!node.leaf[i]) {
					setModifiedRecurs(innerNode(node, i), depth - 1, min_depth);
				}
			}
		}
	}

	//
	// Reset modified
	//

	void resetModified(LeafNode& node, index_t index) { node.modified.reset(index); }

	void resetModified(InnerNode& node, index_t index, depth_t depth, depth_t max_depth)
	{
		if (node.leaf[index] || !node.modified[index]) {
			if (depth <= max_depth) {
				node.modified.reset(index);
			}
			return;
		}

		if (1 == depth) {
			leafChild(node, index).modified.reset();
		} else {
			resetModifiedRecurs(innerChild(node, index), depth - 1, max_depth);
		}

		if (depth <= max_depth) {
			node.modified.reset(index);
		}
	}

	void resetModifiedRecurs(InnerNode& node, depth_t depth, depth_t max_depth)
	{
		IndexField modified_parents = node.modified & ~node.leaf;

		if (modified_parents.none()) {
			if (depth <= max_depth) {
				node.modified.reset();
			}
			return;
		}

		if (1 == depth) {
			// All are allocated so we can just resset all, even if they are leaves
			for (auto& child : leafChildren(node)) {
				child.modified.reset();
			}
		} else {
			for (index_t i = 0; 8 != i; ++i) {
				if (modified_parents[i]) {
					resetModifiedRecurs(innerNode(node, i), depth - 1, max_depth);
				}
			}
		}

		if (depth <= max_depth) {
			node.modified.reset();
		}
	}

	//
	// Set parents modified
	//

	void setModifiedParents(Code code)
	{
		setModifiedParentsRecurs(root(), rootDepth(), code);
	}

	// NOTE: Assumes code has depth higher then depth
	void setModifiedParentsRecurs(InnerNode& node, depth_t depth, Code code)
	{
		auto index = code.index(depth);
		node.modified.set(index);
		if (code.depth() < depth - 1 && !node.leaf[index]) {
			setModifiedParentsRecurs(innerChild(node, index), depth - 1, code);
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Propagate                                      |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	template <bool KeepModified>
	void updateNode(InnerNode& node, IndexField indices, depth_t const depth)
	{
		if (1 == depth) {
			derived().updateNode(node, indices, std::cbegin(leafChildren(node)),
			                     std::cend(leafChildren(node)));
		} else {
			derived().updateNode(node, indices, std::cbegin(innerChildren(node)),
			                     std::cend(innerChildren(node)));
		}

		prune(node, indices, depth);
	}

	void propagateModified(LeafNode& node, index_t index, bool keep_modified)
	{
		if (!keep_modified) {
			node.modified.reset(index);
		}
	}

	void propagateModified(InnerNode& node, index_t index, depth_t depth,
	                       bool keep_modified, depth_t max_depth)
	{
		if (!node.modified[index]) {
			return;  // Nothing modified here
		}

		if (1 == depth) {
			if (!keep_modified) {
				leafChild(node, index).modified.reset();
			}
		} else {
			if (keep_modified) {
				propagateModifiedRecurs<true>(innerChild(node, index), depth - 1, max_depth);
			} else {
				propagateModifiedRecurs<false>(innerChild(node, index), depth - 1, max_depth);
			}
		}
	}

	template <bool KeepModified>
	void propagateModifiedRecurs(InnerNode& node, depth_t depth, depth_t max_depth)
	{
		IndexField modified_parent = node.modified & ~node.leaf;

		if (modified_parent.none()) {
			if constexpr (!KeepModified) {
				if (depth <= max_depth) {
					node.modified.reset();
				}
			}
			return;
		}

		if (1 == depth) {
			if constexpr (!KeepModified) {
				for (std::size_t index = 0; 8 != index; ++index) {
					if (!modified_parent[index]) {
						continue;
					}
					leafChild(node, index).modified.reset();
				}
			}
		} else {
			for (std::size_t index = 0; 8 != index; ++index) {
				if (!modified_parent[index]) {
					continue;
				}
				propagateModifiedRecurs<KeepModified>(innerChild(node, index), depth - 1,
				                                      max_depth);
			}
		}

		if (depth <= max_depth) {
			updateNode<KeepModified>(node, modified_parent, depth);
			if constexpr (!KeepModified) {
				node.modified.reset();
			}
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Prune                                        |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	// If all children are the same as the parent they can be pruned
	// NOTE: Only call with nodes that have children
	[[nodiscard]] static IndexField isCollapsible(InnerNode const& node,
	                                              IndexField const indices,
	                                              depth_t const depth)
	{
		IndexField collapsible;
		for (std::size_t index = 0; 8 != index; ++index) {
			if (!indices[index]) {
				continue;
			}
			if (1 == depth) {
				if (leafChild(node, index).isCollapsible(node, index)) {
					collapsible.set(index);
				}
			} else {
				if (innerChild(node, index).isCollapsible(node, index)) {
					collapsible.set(index);
				}
			}
		}
		return collapsible;
	}

	// NOTE: Only call with nodes that have children
	IndexField prune(InnerNode& node, IndexField indices, depth_t depth)
	{
		indices = isCollapsible(node, indices, depth);
		if (indices.all()) {
			deleteChildren(node, indices, depth);
		}
		return indices;
	}

	IndexField pruneRecurs(InnerNode& node, depth_t depth)
	{
		if (node.isAllLeaf()) {
			return node.isLeaf();
		}

		if (1 == depth) {
			return node.isLeaf() | prune(node, node.isParent(), depth);
		}

		IndexField prunable;
		for (std::size_t index = 0; 8 != index; ++index) {
			if (node.isLeafIndex(index)) {
				continue;
			}

			auto& child = innerChild(node, index);
			if (child.isParent() == pruneRecurs(child, depth - 1)) {
				prunable.set(index);
			}
		}

		return node.isLeaf() | (prunable.none() ? prunable : prune(node, prunable, depth));
	}

	/**************************************************************************************
	|                                                                                     |
	|                                 Create/delete nodes                                 |
	|                                                                                     |
	**************************************************************************************/

	//
	// Create
	//

	inline void allocateLeafChildren(InnerNode& node)
	{
		node.leaf_children = new LeafNodeBlock();
		num_allocated_leaf_nodes_ += 8 * 8;
		num_allocated_inner_leaf_nodes_ -= 1 * 8;
		num_allocated_inner_nodes_ += 1 * 8;
	}

	inline void allocateInnerChildren(InnerNode& node)
	{
		node.inner_children = new InnerNodeBlock();
		// Got 8 * 8 new and 1 * 8 is made into a inner node
		num_allocated_inner_leaf_nodes_ += 7 * 8;
		num_allocated_inner_nodes_ += 1 * 8;
	}

	inline void takeExistingLeafChildren(InnerNode& node)
	{
		node.leaf_children = free_leaf_blocks_.top();
		free_leaf_blocks_.pop();
	}

	inline void takeExistingInnerChildren(InnerNode& node)
	{
		node.leaf_children = free_inner_blocks_.top();
		free_inner_blocks_.pop();
	}

	void createLeafChildren(InnerNode& node) { createLeafChildren(node, leaf(node)); }

	void createLeafChildren(InnerNode& node, IndexField indices)
	{
		indices &= leaf(node);
		if (indices.none()) {
			return;
		}

		if constexpr (!LockLess) {
			if (!lockIfLeaf(node, indices, 0)) {
				return;
			}
		}

		if constexpr (ReuseNodes) {
			if constexpr (!LockLess) {
				if (lockIfNonEmptyLeaves()) {
					takeExistingLeafChildren(node);
					unlockLeaves();
				} else {
					allocateLeafChildren(node);
				}
			} else {
				if (free_leaf_blocks_.empty()) {
					allocateLeafChildren(node);
				} else {
					takeExistingLeafChildren(node);
				}
			}
		} else if (!node.leaf_children) {
			allocateLeafChildren(node);
		}

		std::size_t num = 0;
		for (index_t index = 0; 8 != index; ++index) {
			if (!indices[index]) {
				continue;
			}
			++num;
			leafChildren(node, index).fill(node, index);
		}

		num_leaf_nodes_ += 8 * num;
		num_inner_leaf_nodes_ -= 1 * num;
		num_inner_nodes_ += 1 * num;

		resetLeaf(node, indices);
		if constexpr (!LockLess) {
			unlockChildren(0);
		}
	}

	void createLeafChildrenIndex(InnerNode& node, index_t index)
	{
		// TODO: Implement
	}

	void createInnerChildren(InnerNode& node, depth_t depth)
	{
		createInnerChildren(node, leaf(node), depth);
	}

	void createInnerChildren(InnerNode& node, IndexField indices, depth_t depth)
	{
		indices &= leaf(node);
		if (indices.none()) {
			return;
		}

		if constexpr (!LockLess) {
			if (!lockIfLeaf(node, indices, depth)) {
				return;
			}
		}

		if constexpr (ReuseNodes) {
			if constexpr (!LockLess) {
				if (lockIfNonEmptyInner()) {
					takeExistingInnerChildren(node);
					unlockInner();
				} else {
					allocateInnerChildren(node);
				}
			} else {
				if (free_inner_blocks_.empty()) {
					allocateInnerChildren(node);
				} else {
					takeExistingInnerChildren(node);
				}
			}
		} else if (!node.inner_children) {
			allocateInnerChildren(node);
		}

		std::size_t num = 0;
		for (index_t index = 0; 8 != index; ++index) {
			if (!indices[index]) {
				continue;
			}
			++num;
			innerChildren(node, index).fill(node, index);
		}

		num_inner_leaf_nodes_ += 7 * num;
		num_inner_nodes_ += 1 * num;

		resetLeaf(node, indices);
		if constexpr (!LockLess) {
			unlockChildren(depth);
		}
	}

	void createInnerChildrenIndex(InnerNode& node, index_t index, depth_t depth)
	{
		// TODO: Implement
	}

	//
	// Delete
	//

	inline void deallocateLeafChildren(InnerNode& node)
	{
		delete &leafChildren(node);
		num_allocated_leaf_nodes_ -= 8 * 8;
		num_allocated_inner_leaf_nodes_ += 1 * 8;
		num_allocated_inner_nodes_ -= 1 * 8;
	}

	inline void deallocateInnerChildren(InnerNode& node)
	{
		delete &innerChildren(node);
		// Remove 8*8 and 1*8 inner node is made into a inner leaf node
		num_allocated_inner_leaf_nodes_ -= 7 * 8;
		num_allocated_inner_nodes_ -= 1 * 8;
	}

	inline void addExistingLeafChildren(InnerNode& node)
	{
		free_leaf_blocks_.push(&leafChildren(node));
	}

	inline void addExistingInnerChildren(InnerNode& node)
	{
		free_inner_blocks_.push(*innerChildren(node));
	}

	void deleteLeafChildren(InnerNode& node, bool manual_pruning = false)
	{
		deleteLeafChildren(node, ~leaf(node), manual_pruning);
	}

	void deleteLeafChildren(InnerNode& node, IndexField indices,
	                        bool manual_pruning = false)
	{
		IndexField new_leaf = indices & ~leaf(node);
		if (new_leaf.none()) {
			return;
		}

		indices |= leaf(node);

		setLeaf(node, indices);

		std::size_t num = 0;
		for (index_t index = 0; 8 != index; ++index) {
			if (new_leaf[index]) {
				++num;
			}
		}
		num_leaf_nodes_ -= 8 * num;
		num_inner_leaf_nodes_ += 1 * num;
		num_inner_nodes_ -= 1 * num;

		if (!indices.all()) {
			// Can only remove if all nodes will be leaves
			return;
		}

		if constexpr (ReuseNodes) {
			if (!manual_pruning && !automaticPruning()) {
				if constexpr (!LockLess) {
					lockLeaves();
					addExistingLeafChildren(node);
					unlockLeaves();
				} else {
					addExistingLeafChildren(node);
				}
			} else {
				deallocateLeafChildren(node);
			}
		} else {
			if (!manual_pruning && !automaticPruning()) {
				return;
			}
			deallocateLeafChildren(node);
		}

		node.leaf_children = nullptr;
	}

	void deleteChildren(InnerNode& node, depth_t const depth, bool manual_pruning = false)
	{
		deleteChildren(node, ~leaf(node), depth, manual_pruning);
	}

	void deleteChildren(InnerNode& node, IndexField indices, depth_t const depth,
	                    bool manual_pruning = false)
	{
		IndexField new_leaf = indices & ~leaf(node);
		if (new_leaf.none()) {
			return;
		}

		indices |= leaf(node);

		setLeaf(node, indices);

		std::size_t num = 0;
		for (index_t index = 0; 8 != index; ++index) {
			if (new_leaf[index]) {
				++num;
			}
		}
		num_inner_leaf_nodes_ -= 7 * num;
		num_inner_nodes_ -= 1 * num;

		if (!indices.all()) {
			// Can only remove if all nodes will be leaves
			return;
		}

		if constexpr (ReuseNodes) {
			if (!manual_pruning && !automaticPruning()) {
				if constexpr (!LockLess) {
					lockInner();
					addExistingInnerChildren(node);
					unlockInner();
				} else {
					addExistingInnerChildren(node);
				}
			} else {
				deallocateInnerChildren(node);
			}
		} else {
			if (!manual_pruning && !automaticPruning()) {
				return;
			}

			deallocateInnerChildren(node);
		}

		node.inner_children = nullptr;
	}

	//
	// (Un)lock children
	//

	[[nodiscard]] bool tryLockChildren(depth_t depth)
	{
		return !children_locks_[depth].test_and_set(std::memory_order_acquire);
	}

	void lockChildren(depth_t depth)
	{
		while (!tryLockChildren(depth))
			;
	}

	[[nodiscard]] bool lockIfLeaf(InnerNode const& node, depth_t depth)
	{
		do {
			if (isParent(node)) {
				return false;
			}
		} while (!tryLockChildren(depth));

		if (isParent(node)) {
			unlockChildren(depth);
			return false;
		}

		return true;
	}

	void unlockChildren(depth_t depth)
	{
		children_locks_[depth].clear(std::memory_order_release);
	}

	//
	// Leaf/inner lock
	//

	[[nodiscard]] bool tryLockLeaves()
	{
		return !free_leaf_block_lock_.test_and_set(std::memory_order_acquire);
	}

	void lockLeaves()
	{
		while (!tryLockLeaves()) {
		}
	}

	[[nodiscard]] bool lockIfNonEmptyLeaves()
	{
		do {
			if (free_leaf_blocks_.empty()) {
				return false;
			}
		} while (!tryLockLeaves());

		if (free_leaf_blocks_.empty()) {
			unlockLeaves();
			return false;
		}
		return true;
	}

	void unlockLeaves() { free_leaf_block_lock_.clear(std::memory_order_release); }

	[[nodiscard]] bool tryLockInner()
	{
		return !free_inner_block_lock_.test_and_set(std::memory_order_acquire);
	}

	void lockInner()
	{
		while (!tryLockInner()) {
		}
	}

	[[nodiscard]] bool lockIfNonEmptyInner()
	{
		do {
			if (free_inner_blocks_.empty()) {
				return false;
			}
		} while (!tryLockInner());

		if (free_inner_blocks_.empty()) {
			unlockInner();
			return false;
		}
		return true;
	}

	void unlockInner() { free_inner_block_lock_.clear(std::memory_order_release); }

	/**************************************************************************************
	|                                                                                     |
	|                                         I/O                                         |
	|                                                                                     |
	**************************************************************************************/

	struct NodeAndIndices {
		std::reference_wrapper<LeafNode> node;
		IndexField indices;

		NodeAndIndices() = default;

		NodeAndIndices(LeafNode& node, IndexField indices) : node(node), indices(indices) {}

		NodeAndIndices(NodeAndIndices const&) = default;

		NodeAndIndices(NodeAndIndices&&) = default;

		NodeAndIndices& operator=(NodeAndIndices const&) = default;

		NodeAndIndices& operator=(NodeAndIndices&&) = default;
	};

	struct ConstNodeAndIndices {
		std::reference_wrapper<LeafNode const> node;
		IndexField indices;

		ConstNodeAndIndices() = default;

		ConstNodeAndIndices(LeafNode const& node, IndexField indices)
		    : node(node), indices(indices)
		{
		}

		ConstNodeAndIndices(ConstNodeAndIndices const&) = default;

		ConstNodeAndIndices(ConstNodeAndIndices&&) = default;

		ConstNodeAndIndices& operator=(ConstNodeAndIndices const&) = default;

		ConstNodeAndIndices& operator=(ConstNodeAndIndices&&) = default;
	};

	[[nodiscard]] FileOptions fileOptions(bool const compress) const
	{
		FileOptions options;
		options.compressed = compress;
		options.leaf_size = size();
		options.depth_levels = depthLevels();
		return options;
	}

	[[nodiscard]] std::vector<NodeAndIndices> readNodes(std::istream& in)
	{
		auto tree = readTreeStructure(in);
		std::uint64_t num_nodes = readNumNodes(in);
		return retrieveNodes(tree, num_nodes);
	}

	[[nodiscard]] std::unique_ptr<IndexField[]> readTreeStructure(std::istream& in)
	{
		std::uint64_t num;
		in.read(reinterpret_cast<char*>(&num), sizeof(num));

		auto tree = std::make_unique<IndexField[]>(num);
		in.read(reinterpret_cast<char*>(tree.get()),
		        num * sizeof(typename decltype(tree)::element_type));

		return tree;
	}

	[[nodiscard]] std::uint64_t readNumNodes(std::istream& in)
	{
		std::uint64_t num_nodes;
		in.read(reinterpret_cast<char*>(&num_nodes), sizeof(num_nodes));
		return num_nodes;
	}

	[[nodiscard]] std::vector<NodeAndIndices> retrieveNodes(
	    std::unique_ptr<IndexField[]> const& tree, std::uint64_t num_nodes)
	{
		std::vector<NodeAndIndices> nodes;
		nodes.reserve(num_nodes);

		if (tree[0].any()) {
			nodes.emplace_back(root(), tree[0]);
			setModified(root());
		} else if (tree[1].any()) {
			createInnerChildren(root(), rootIndex());
			retrieveNodesRecurs(innerChildren(root(), rootIndex()), tree[1], rootDepth() - 1,
			                    std::next(tree.get(), 2), nodes);
		}

		return nodes;
	}

	IndexField const* retrieveNodesRecurs(LeafNodeBlock& node, IndexField indices,
	                                      IndexField const* tree,
	                                      std::vector<NodeAndIndices>& nodes)
	{
		for (index_t i = 0; 8 != i; ++i) {
			if (!indices[i]) {
				continue;
			}

			IndexField const valid_return = *tree++;
			nodes.emplace_back(node[i], valid_return);
			node[i].modified = valid_return;
		}

		return tree;
	}

	IndexField const* retrieveNodesRecurs(InnerNodeBlock& node, IndexField indices,
	                                      depth_t const depth, IndexField const* tree,
	                                      std::vector<NodeAndIndices>& nodes)
	{
		for (index_t i = 0; 8 != i; ++i) {
			if (!indices[i]) {
				continue;
			}

			IndexField const valid_return = *tree++;
			IndexField const valid_inner = *tree++;

			if (valid_return.any()) {
				nodes.emplace_back(node[i], valid_return);
			}

			if (valid_inner.any()) {
				if (1 == depth) {
					createLeafChildren(node[i], ...);
					// TODO: Implement
				} else {
					createInnerChildren(node[i], ...);
					// TODO: Implement
				}
			}

			setModified(node[i], valid_return | valid_inner);
		}

		return tree;
	}

	IndexField const* retrieveNodesRecurs(InnerNode& node, IndexField indices,
	                                      depth_t const depth, IndexField const* tree,
	                                      std::vector<NodeAndIndices>& nodes)
	{
		for (index_t i = 0; 8 != i; ++i) {
			if (!indices[i]) {
				continue;
			}

			IndexField const children_valid_return = *tree++;

			if (1 == depth) {
				if (children_valid_return.none()) {
					continue;
				}

				// TODO: Implement
			} else {
				IndexField const children_valid_inner = *tree++;

				if (children_valid_return.none() && children_valid_inner.none()) {
					continue;
				}

				// TODO: Implement
			}
		}

		return tree;

		if (1 == depth) {
			if (children_valid_return.none()) } else {
		}

		// TODO: Implement

		if (1 == depth) {
			if (children_valid_return.none()) {
				return tree;
			}

			node.modified = children_valid_return;

			createLeafChildren(node);

			for (std::size_t i = 0; 8 != i; ++i) {
				if (children_valid_return[i]) {
					nodes.push_back(
					    std::ref(leafChild(node, i)));  // TODO: push_back or emplace_back?
				}
			}
		} else {
			IndexField const child_valid_inner = *indicators++;

			if (children_valid_return.none() && child_valid_inner.none()) {
				return indicators;
			}

			node.modified = ...;

			createInnerChildren(node, depth);

			for (std::size_t i = 0; 8 != i; ++i) {
				if (children_valid_return[i]) {
					nodes.push_back(std::ref(static_cast<LeafNode&>(
					    innerChild(node, i))));  // TODO: push_back or emplace_back?
				} else if (child_valid_inner[i]) {
					indicators =
					    retrieveNodesRecurs(indicators, nodes, innerChild(node, i), depth - 1);
				}
			}
		}

		return indicators;
	}

	template <class Predicates>
	[[nodiscard]] std::pair<std::vector<IndexField>, std::vector<ConstNodeAndIndices>> data(
	    Predicates const& predicates) const
	{
		std::vector<IndexField> tree;
		std::vector<ConstNodeAndIndices> nodes;

		std::conditional_t<predicate::contains_spatial_predicate_v<Predicates>, NodeBV, Node>
		    root = rootNodeBV();

		bool valid_return =
		    predicate::PredicateValueCheck<Predicates>::apply(predicates, derived(), root);
		bool valid_inner = !valid_return && predicate::PredicateInnerCheck<Predicates>::apply(
		                                        predicates, derived(), root);

		tree.emplace_back(valid_return ? 1U : 0U);
		tree.emplace_back(valid_inner ? 1U : 0U);

		if (valid_return) {
			nodes.emplace_back(root(), 1U);
		} else if (valid_inner) {
			dataRecurs(child(root, 0), predicates, tree, nodes);
			if (nodes.empty()) {
				//  Nothing was added
				tree.clear();
			}
		}

		return {std::move(tree), std::move(nodes)};
	}

	template <class Predicates, class NodeType>
	void dataRecurs(NodeType const& node, Predicates const& predicates,
	                std::vector<IndexField>& tree,
	                std::vector<ConstNodeAndIndices>& nodes) const
	{
		IndexField valid_return;

		if (0 == node.depth()) {
			for (index_t i = 0; 8 != i; ++i) {
				if (predicate::PredicateValueCheck<Predicates>::apply(predicates, derived(),
				                                                      sibling(node, i))) {
					valid_return[i] = true;
				}
			}

			tree.push_back(valid_return);

			if (valid_return) {
				nodes.emplace_back(leafNode(node), valid_return);
			}
		} else {
			IndexField valid_inner;
			for (index_t i = 0; 8 != i; ++i) {
				auto s = sibling(node, i);

				if (predicate::PredicateValueCheck<Predicates>::apply(predicates, derived(), s)) {
					valid_return[i] = true;
				} else if (predicate::PredicateInnerCheck<Predicates>::apply(predicates,
				                                                             derived(), s)) {
					valid_inner[i] = true;
				}
			}

			tree.push_back(valid_return);
			tree.push_back(valid_inner);

			auto cur_tree_size = tree.size();
			auto cur_nodes_size = nodes.size();

			if (valid_return.any()) {
				nodes.emplace_back(leafNode(node), valid_return);
			}

			if (valid_inner.any()) {
				for (index_t i = 0; 8 != i; ++i) {
					if (valid_inner[i]) {
						auto s = sibling(node, i);
						dataRecurs(child(s, 0), predicates, tree, nodes);
					}
				}
			}

			if (nodes.size() == cur_nodes_size) {
				tree.resize(cur_tree_size);
				tree[tree.size() - 1] = 0;
				tree[tree.size() - 2] = 0;
			}
		}
	}

	template <bool Propagate>
	[[nodiscard]] std::pair<std::vector<IndexField>, std::vector<ConstNodeAndIndices>>
	modifiedData()
	{
		std::vector<IndexField> tree;
		std::vector<ConstNodeAndIndices> nodes;

		InnerNode& root = root();
		depth_t depth = rootDepth();

		IndexField valid_return = root.leaf & root.modified;
		IndexField valid_inner = root.modified;

		tree.push_back(valid_return);
		tree.push_back(valid_inner);

		if (valid_return) {
			nodes.emplace_back(root, valid_return);
			if constexpr (Propagate) {
				propagate(root, valid_return);
			}
		} else if (valid_inner) {
			modifiedDataRecurs<Propagate>(innerChildren(root), valid_inner, depth - 1, tree,
			                              nodes);
			if constexpr (Propagate) {
				propagate(root, valid_inner, depth);
			}
			if (nodes.empty()) {
				//  Nothing was added
				tree.clear();
			}
		}

		root.modified.reset();

		return {std::move(tree), std::move(nodes)};  // FIXME: Check if RVO
	}

	template <bool Propagate>
	void modifiedDataRecurs(LeafNodeBlock& node_block, IndexField indices,
	                        std::vector<IndexField>& tree,
	                        std::vector<ConstNodeAndIndices>& nodes)
	{
		for (index_t i = 0; node_block.size() != i; ++i) {
			if (indices[i]) {
				continue;
			}

			IndexField m = node_block[i].modified;
			tree.push_back(m);

			if (m.none()) {
				continue;
			}

			nodes.emplace_back(node_block[i], m);

			if constexpr (Propagate) {
				propagate(node_block[i], m);
			}

			node_block[i].modified.reset();
		}
	}

	template <bool Propagate>
	void modifiedDataRecurs(InnerNodeBlock& node_block, IndexField indices,
	                        depth_t const depth, std::vector<IndexField>& tree,
	                        std::vector<ConstNodeAndIndices>& nodes)
	{
		for (index_t i = 0; node_block.size() != i; ++i) {
			if (indices[i]) {
				continue;
			}

			IndexField m = node_block[i].modified;
			IndexField l = node_block[i].leaf;

			IndexField valid_return = m & l;
			IndexField valid_inner = m & ~l;

			tree.push_back(valid_return);
			tree.push_back(valid_inner);

			auto const cur_tree_size = tree.size();
			auto const cur_nodes_size = nodes.size();

			if (valid_return.any()) {
				nodes.emplace_back(node_block[i], valid_return);
			}

			if (valid_inner.any()) {
				if (1 == depth) {
					modifiedDataRecurs<Propagate>(leafChildren(node_block[i]), valid_inner, tree,
					                              nodes);
				} else {
					modifiedDataRecurs<Propagate>(innerChildren(node_block[i]), valid_inner,
					                              depth - 1, tree, nodes);
				}
			}

			if constexpr (Propagate) {
				propagate(node_block[i], m);
			}

			node_block[i].modified.reset();

			if (nodes.size() == cur_nodes_size) {
				tree.resize(cur_tree_size);
				tree[tree.size() - 1] = 0;
				tree[tree.size() - 2] = 0;
			}
		}
	}

	template <class InputIt>
	void write(std::ostream& out, std::vector<IndexField> const& tree, InputIt first,
	           std::size_t num_nodes, bool compress, int compression_acceleration_level,
	           int compression_level) const
	{
		writeHeader(out, fileOptions(compress));
		writeTreeStructure(out, tree);
		writeNumNodes(out, num_nodes);
		writeNodes(out, first, num_nodes, compress, compression_acceleration_level,
		           compression_level);
	}

	void writeTreeStructure(std::ostream& out, std::vector<IndexField> const& tree) const
	{
		std::uint64_t num = tree.size();
		out.write(reinterpret_cast<char const*>(&num), sizeof(num));
		out.write(reinterpret_cast<char const*>(tree.data()),
		          num * sizeof(typename decltype(tree)::value_type));
	}

	void writeNumNodes(std::ostream& out, std::uint64_t const num_nodes) const
	{
		out.write(reinterpret_cast<char const*>(&num_nodes), sizeof(num_nodes));
	}

	template <class InputIt>
	void writeNodes(std::ostream& out, InputIt first, std::size_t num_nodes,
	                bool const compress, int const compression_acceleration_level,
	                int const compression_level) const
	{
		derived().writeNodes(out, first, num_nodes, compress, compression_acceleration_level,
		                     compression_level);
	}

 protected:
	// The number of depth levels
	depth_t depth_levels_;
	// The maximum coordinate value the octree can store
	key_t max_value_;

	// The root of the octree
	InnerNode root_;

	// Stores the node size at a given depth, where the depth is the index
	std::array<node_size_t, maxDepthLevels()> node_size_;
	// Reciprocal of the node size at a given depth, where the depth is the index
	std::array<node_size_t, maxDepthLevels()> node_size_factor_;

	// Automatic pruning
	bool automatic_prune_ = true;

	// Locks to support parallel insertion, one per depth level
	std::array<std::atomic_flag, maxDepthLevels()> children_locks_;

	// Free inner node blocks that can be used instead of allocating new
	std::stack<InnerNodeBlock*> free_inner_blocks_;
	// Free leaf node blocks that can be used instead of allocating new
	std::stack<LeafNodeBlock*> free_leaf_blocks_;

	// Lock for accessing `free_inner_blocks_`
	std::atomic_flag free_inner_block_lock_ = ATOMIC_FLAG_INIT;
	// Lock for accessing `free_leaf_blocks_`
	std::atomic_flag free_leaf_block_lock_ = ATOMIC_FLAG_INIT;

	//
	// Memory
	//

	// Current number of inner nodes
	std::atomic_size_t num_inner_nodes_ = 0;
	// Current number of inner leaf nodes
	std::atomic_size_t num_inner_leaf_nodes_ = 1;
	// Current number of leaf nodes
	std::atomic_size_t num_leaf_nodes_ = 0;

	// Current number of allocated inner nodes
	std::atomic_size_t num_allocated_inner_nodes_ = 0;
	// Current number of allocated inner leaf nodes
	std::atomic_size_t num_allocated_inner_leaf_nodes_ = 1 * 8;
	// Current number of allocated leaf nodes
	std::atomic_size_t num_allocated_leaf_nodes_ = 0;
};

}  // namespace ufo::map

#endif  // UFO_MAP_OCTREE_BASE_H