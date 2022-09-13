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
#include <ufo/map/code/code.h>
#include <ufo/map/io.h>
#include <ufo/map/key.h>
#include <ufo/map/octree/iterator.h>
#include <ufo/map/octree/node.h>
#include <ufo/map/octree/octree_node.h>
#include <ufo/map/octree/octree_predicate.h>
#include <ufo/map/octree/query.h>
#include <ufo/map/point.h>
#include <ufo/map/predicate/predicates.h>
#include <ufo/map/predicate/spatial.h>
#include <ufo/map/types.h>
#include <ufo/math/util.h>
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
	// Minimum number of depth levels
	static constexpr depth_t MIN_DEPTH_LEVELS = 3;
	// Maximum number of depth levels
	static constexpr depth_t MAX_DEPTH_LEVELS = 22;

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
	void clear(bool prune = false) { clear(nodeSize(), depthLevels(), prune); }

	/*!
	 * @brief Erases the map and changes the leaf node size and the number of depth levels.
	 * After this call, the map contains only the root node.
	 *
	 * @param new_leaf_size The new leaf node size.
	 * @param new_depth_levels The new number of depth levels.
	 * @param prune Whether the memory should be cleared.
	 */
	void clear(node_size_t new_leaf_size, depth_t new_depth_levels, bool prune = false)
	{
		deleteChildren(root(), rootDepth(), prune);
		setNodeSizeAndDepthLevels(new_leaf_size, new_depth_levels);
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
	[[nodiscard]] static constexpr depth_t minDepthLevels() { return MIN_DEPTH_LEVELS; }

	/*!
	 * @brief The maximum depth levels an octree can have.
	 *
	 * @return The maximum depth levels an octree can have.
	 */
	[[nodiscard]] static constexpr depth_t maxDepthLevels() { return MAX_DEPTH_LEVELS; }

	//
	// Size
	//

	/*!
	 * @brief The size the octree covers.
	 *
	 * @note This is the same as `nodeSize(rootDepth())`.
	 *
	 * @return The size the octree covers.
	 */
	[[nodiscard]] constexpr node_size_t size() const noexcept
	{
		return nodeSize(rootDepth());
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
	[[nodiscard]] constexpr node_size_t volume() const noexcept
	{
		auto const s = size();
		return s * s * s;
	}

	//
	// Node size
	//

	/*!
	 * @brief Get the node size at a specific depth.
	 *
	 * @param depth The depth.
	 * @return The node size at the depth.
	 */
	[[nodiscard]] constexpr node_size_t nodeSize(depth_t depth = 0) const noexcept
	{
		return node_size_[depth];
	}

	//
	// Center
	//

	/*!
	 * @return The center of the octree.
	 */
	[[nodiscard]] Point3 center() const { return Point3(0, 0, 0); }

	//
	// Bounding volume
	//

	/*!
	 * @return Minimum bounding volume convering the whole octree.
	 */
	[[nodiscard]] geometry::AAEBB boundingVolume() const
	{
		return geometry::AAEBB(center(), nodeSize(rootDepth() - 1));
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
	[[nodiscard]] constexpr bool isWithin(Point3 coord) const
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
		auto const max = nodeSize(rootDepth() - 1);
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
	[[nodiscard]] static constexpr bool isPureLeaf(Point3 coord, depth_t depth = 0) noexcept
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
		return isPureLeaf(node) || innerNode(node).isLeafIndex(node.index());
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
		return isPureLeaf(node) || innerNode(code).isLeafIndex(code.index());
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
	[[nodiscard]] bool isLeaf(Point3 coord, depth_t depth = 0) const
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
	[[nodiscard]] constexpr bool isParent(Point3 coord, depth_t depth = 0) const
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

	/**************************************************************************************
	|                                                                                     |
	|                                      Modified                                       |
	|                                                                                     |
	**************************************************************************************/

	//
	// Modified
	//

	[[nodiscard]] constexpr bool isModified() const { return isModified(root()); }

	[[nodiscard]] constexpr bool isModified(Node node) const
	{
		return leafNode(node).isModifiedIndex(node.index());
	}

	[[nodiscard]] constexpr bool isModified(Code code) const
	{
		auto [n, d] = nodeAndDepth(code);
		return n->isModifiedIndex(code.index(d));
	}

	[[nodiscard]] constexpr bool isModified(Key key) const
	{
		return isModified(toCode(key));
	}

	[[nodiscard]] constexpr bool isModified(Point3 coord, depth_t depth = 0) const
	{
		return isModified(toCode(coord, depth));
	}

	[[nodiscard]] constexpr bool isModified(coord_t x, coord_t y, coord_t z,
	                                        depth_t depth = 0) const
	{
		return isModified(toCode(x, y, z, depth));
	}

	//
	// Set modified
	//

	void setModified(depth_t min_depth = 0)
	{
		if (rootDepth() >= min_depth) {
			setModified(root(), rootIndex(), rootDepth(), min_depth);
		}
	}

	void setModified(Node node, depth_t min_depth = 0)
	{
		if (rootDepth() < min_depth) {
			return;
		} else if (node.depth() < min_depth) {
			setModifiedParents(node.code().toDepth(min_depth - 1));
			return;
		} else if (isPureLeaf(node)) {
			leafNode(node).setModifiedIndex(node.index());
		} else {
			setModified(innerNode(node), node.index(), node.depth(), min_depth);
		}
		setModifiedParents(node.code().toDepth(min_depth));
	}

	void setModified(Code code, depth_t min_depth = 0)
	{
		if (rootDepth() < min_depth) {
			return;
		} else if (code.depth() < min_depth) {
			setModifiedParents(code.toDepth(min_depth - 1));
			return;
		} else if (isPureLeaf(code)) {
			createLeafNode(code).setModifiedIndex(code.index());
		} else {
			setModified(createInnerNode(code), code.index(), code.depth(), min_depth);
		}
		setModifiedParents(code.toDepth(min_depth));
	}

	void setModified(Key key, depth_t min_depth = 0)
	{
		setModified(toCode(key), min_depth);
	}

	void setModified(Point3 coord, depth_t min_depth = 0, depth_t depth = 0)
	{
		setModified(toCode(coord, depth), min_depth);
	}

	void setModified(coord_t x, coord_t y, coord_t z, depth_t min_depth = 0,
	                 depth_t depth = 0)
	{
		setModified(toCode(x, y, z, depth), min_depth);
	}

	//
	// Reset modified
	//

	void resetModified(depth_t max_depth = maxDepthLevels())
	{
		resetModified(root(), rootIndex(), rootDepth(), max_depth);
	}

	void resetModified(Node node, depth_t max_depth = maxDepthLevels())
	{
		if (isLeaf(node)) {
			leafNode(node).resetModifiedIndex(node.index());
		} else {
			resetModified(innerNode(node), node.index(), node.depth(), max_depth);
		}
	}

	void resetModified(Code code, depth_t max_depth = maxDepthLevels())
	{
		if (isPureLeaf(code)) {
			leafNode(code).resetModifiedIndex(code.index());
		} else {
			resetModified(innerNode(code), code.index(), code.depth(), max_depth);
		}
	}

	void resetModified(Key key, depth_t max_depth = maxDepthLevels())
	{
		resetModified(toCode(key), max_depth);
	}

	void resetModified(Point3 coord, depth_t depth = 0,
	                   depth_t max_depth = maxDepthLevels())
	{
		resetModified(toCode(coord, depth), max_depth);
	}

	void resetModified(coord_t x, coord_t y, coord_t z, depth_t depth = 0,
	                   depth_t max_depth = maxDepthLevels())
	{
		resetModified(toCode(x, y, z, depth), max_depth);
	}

	//
	// Propagate
	//

	void propagateModified(bool keep_modified = false, depth_t max_depth = maxDepthLevels())
	{
		propagateModified(root(), rootIndex(), rootDepth(), keep_modified, max_depth);
	}

	void propagateModified(Node node, bool keep_modified = false,
	                       depth_t max_depth = maxDepthLevels())
	{
		if (isPureLeaf(node)) {
			propagateModified(leafNode(node), node.index(), keep_modified);
		} else {
			propagateModified(innerNode(node), node.index() node.depth(), keep_modified,
			                  max_depth);
		}
	}

	void propagateModified(Code code, bool keep_modified = false,
	                       depth_t max_depth = maxDepthLevels())
	{
		if (isPureLeaf(code)) {
			propagateModified(leafNode(code), code.index(), keep_modified);
		} else {
			propagateModified(innerNode(code), code.index() code.depth(), keep_modified,
			                  max_depth);
		}
	}

	void propagateModified(Key key, bool keep_modified = false,
	                       depth_t max_depth = maxDepthLevels())
	{
		propagateModified(toCode(key), keep_modified, max_depth);
	}

	void propagateModified(Point3 coord, depth_t depth = 0, bool keep_modified = false,
	                       depth_t max_depth = maxDepthLevels())
	{
		propagateModified(toCode(coord, depth), depth, keep_modified, max_depth);
	}

	void propagateModified(coord_t x, coord_t y, coord_t z, depth_t depth = 0,
	                       bool keep_modified = false, depth_t max_depth = maxDepthLevels())
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

	[[nodiscard]] bool isRoot(Node node) const { return isRoot(node.code()); }

	[[nodiscard]] bool isRoot(Code code) const { return rootCode() == code; }

	[[nodiscard]] bool isRoot(Key key) const { return isRoot(toCode(key)); }

	[[nodiscard]] bool isRoot(Point3 coord, depth_t depth = 0) const
	{
		return isRoot(toCode(coord, depth));
	}

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
		return Node(const_cast<InnerNode*>(&root()), rootCode());
	}

	/*!
	 * @brief Get the root node with bounding volume.
	 *
	 * @return The root node with bounding volume.
	 */
	[[nodiscard]] constexpr NodeBV rootNodeBV() const
	{
		return NodeBV(rootNode(), boundingVolume());
	}

	//
	// Code
	//

	/*!
	 * @brief Get the code for the root node.
	 *
	 * @return The root node code.
	 */
	[[nodiscard]] constexpr Code rootCode() const { return Code(rootIndex(), rootDepth()); }

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
	[[nodiscard]] constexpr Point3 rootCenter() const noexcept { return center(); }

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
	[[nodiscard]] constexpr node_size_t nodeSize(Node node) const noexcept
	{
		return nodeSize(node.depth());
	}

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
	[[nodiscard]] Point3 nodeCenter(NodeType const& node) const
	{
		if constexpr (std::is_same_v<NodeBV, NodeType>) {
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
	[[nodiscard]] geometry::AAEBB nodeBoundingVolume(NodeType const& node) const
	{
		if constexpr (std::is_same_v<NodeBV, NodeType>) {
			return node.boundingVolume();

		} else {
			return geometry::AAEBB(nodeCenter(node), nodeSize(node) / 2);
		}
	}

	//
	// Find
	//

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
	[[nodiscard]] Node findNode(Code code) const
	{
		auto [n, d] = nodeAndDepth(code);
		return Node(const_cast<LeafNode*>(n), code.parent(d));
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
	[[nodiscard]] Node findNode(Key key) const { return findNode(toCode(key)); }

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
	[[nodiscard]] Node findNode(Point3 coord, depth_t depth = 0) const
	{
		return findNode(toCode(coord, depth));
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
	[[nodiscard]] Node findNode(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return findNode(toCode(x, y, z, depth));
	}

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
	[[nodiscard]] std::optional<Node> findNodeChecked(Code code) const
	{
		return code.depth() <= rootDepth() ? std::optional<Node>(findNode(code))
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
	[[nodiscard]] std::optional<Node> findNodeChecked(Key key) const
	{
		return findNodeChecked(toCode(key));
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
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The node.
	 */
	[[nodiscard]] std::optional<Node> findNodeChecked(Point3 coord, depth_t depth = 0) const
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
	[[nodiscard]] std::optional<Node> findNodeChecked(coord_t x, coord_t y, coord_t z,
	                                                  depth_t depth = 0) const
	{
		return findNodeChecked(Point3(x, y, z), depth);
	}

	//
	// Function call operator
	//

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
	 * @note Same as the corresponding `findNode` function.
	 *
	 * @param code The code.
	 * @return The node.
	 */
	[[nodiscard]] Node operator()(Code code) const { return findNode(code); }

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
	 * @note Same as the corresponding `findNode` function.
	 *
	 * @param key The key.
	 * @return The node.
	 */
	[[nodiscard]] Node operator()(Key key) const { return findNode(key); }

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
	 * @note Same as the corresponding `findNode` function.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The node.
	 */
	[[nodiscard]] Node operator()(Point3 coord, depth_t depth = 0) const
	{
		return findNode(coord, depth);
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
	 * @note Same as the corresponding `findNode` function.
	 *
	 * @param x,y,z The coordinate.
	 * @param depth The depth.
	 * @return The node.
	 */
	[[nodiscard]] Node operator()(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return findNode(x, y, z, depth);
	}

	//
	// Create
	//

	// TODO: Add comments

	Node createNode(Code code)
	{
		// FIXME: Handle wrong codes

		InnerNode* node = &root();
		auto const min_depth = std::max(depth_t(1), code.depth());
		for (depth_t depth = rootDepth(); min_depth < depth; --depth) {
			auto const index = code.index(depth);
			if (node->leafIndex(index)) {
				createInnerChildren(*node, index, depth);
			}
			node = &innerChild(*node, index, code.index(depth - 1));
		}

		if (0 == code.depth()) {
			auto const index = code.index();
			if (node->leafIndex(index)) {
				createLeafChildren(*node, index);
			}
			return Node(&leafChild(*node, code.index()), code);
		} else {
			return Node(node, code);
		}
	}

	Node createNode(Key key) { return createNode(toCode(key)); }

	Node createNode(Point3 coord, depth_t depth = 0)
	{
		return createNode(toCode(coord, depth));
	}

	Node createNode(coord_t x, coord_t y, coord_t z, depth_t depth = 0)
	{
		return createNode(toCode(x, y, z, depth));
	}

	std::optional<Node> createNodeChecked(Code code)
	{
		// TODO: Implement
	}

	std::optional<Node> createNodeChecked(Key key)
	{
		// TODO: Implement
	}

	std::optional<Node> createNodeChecked(Point3 coord, depth_t depth = 0)
	{
		// TODO: Implement
	}

	std::optional<Node> createNodeChecked(coord_t x, coord_t y, coord_t z,
	                                      depth_t depth = 0)
	{
		// TODO: Implement
	}

	//
	// Sibling
	//

	// TODO: Add comments

	[[nodiscard]] Node nodeSibling(Node node, index_t sibling_index) const
	{
		return Node(node.data(), node.code().sibling(sibling_index));
	}

	[[nodiscard]] NodeBV nodeSibling(NodeBV const& node, index_t sibling_index) const
	{
		geometry::AAEBB aaebb(
		    siblingCenter(node.center(), node.halfSize(), node.index(), sibling_index),
		    node.halfSize());
		return NodeBV(node.data(), node.code().sibling(sibling_index), aaebb);
	}

	template <class Node>
	[[nodiscard]] Node nodeSiblingChecked(Node const& node, index_t sibling_index) const
	{
		if (!root(node)) {
			throw std::out_of_range("Node has no siblings");
		} else if (7 < sibling_index) {
			throw std::out_of_range("sibling_index out of range");
		}
		return nodeSibling(node, sibling_index);
	}

	//
	// Child
	//

	// TODO: Add comments

	[[nodiscard]] Node nodeChild(Node node, index_t child_index) const
	{
		auto& child = children(innerNode(node), node.index(), node.depth() - 1);
		return Node(child, node.code().child(child_index));
	}

	[[nodiscard]] NodeBV nodeChild(NodeBV const& node, index_t child_index) const
	{
		auto& child = children(innerNode(node), node.index(), node.depth() - 1);

		auto const child_half_size = node.halfSize() / 2;
		geometry::AAEBB child_aaebb(childCenter(node.center(), child_half_size, child_index),
		                            child_half_size);

		return NodeBV(&child, node.code().child(child_index), child_aaebb);
	}

	template <class Node>
	[[nodiscard]] Node nodeChildChecked(Node const& node, index_t child_index) const
	{
		if (!parent(node)) {
			throw std::out_of_range("Node has no children");
		} else if (7 < child_index) {
			throw std::out_of_range("child_index out of range");
		}
		return nodeChild(node, child_index);
	}

	//
	// Parent
	//

	// TODO: Add comments

	[[nodiscard]] Node nodeParent(Node node) const
	{
		// TODO: Implement
	}

	[[nodiscard]] NodeBV nodeParent(NodeBV const& node) const
	{
		// TODO: Implement
	}

	template <class Node>
	[[nodiscard]] Node nodeParentChecked(Node const& node) const
	{
		if (rootDepth() <= node.depth()) {
			throw std::out_of_range("Node has no parent");
		}
		return nodeParent(node);
	}

	//
	// NodeBV
	//

	// TODO: Add comment

	NodeBV toNodeBV(Node node) { return NodeBV(node, boundingVolume(node)); }

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
	[[nodiscard]] constexpr Code toCode(Point3 coord, depth_t depth = 0) const noexcept
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
	    Point3 coord, depth_t depth = 0) const noexcept
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
		return toCodeChecked(Point3(x, y, z), depth);
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
	[[nodiscard]] constexpr Key toKey(Point3 coord, depth_t depth = 0) const noexcept
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
	    Point3 coord, depth_t depth = 0) const noexcept
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
		return toKeyChecked(Point3(x, y, z), depth);
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
	[[nodiscard]] constexpr Point3 toCoord(Code code) const noexcept
	{
		return toCoord(toKey(code));
	}

	/*!
	 * @brief Convert a key to a coordinate.
	 *
	 * @param key The key.
	 * @return The corresponding coordinate.
	 */
	[[nodiscard]] constexpr Point3 toCoord(Key key) const noexcept
	{
		return Point3(toCoord(key[0], key.depth()), toCoord(key[1], key.depth()),
		              toCoord(key[2], key.depth()));
	}

	/*!
	 * @brief Convert a code to a coordinate with bounds check.
	 *
	 * @param Code The code.
	 * @return The corresponding coordinate.
	 */
	[[nodiscard]] constexpr std::optional<Point3> toCoordChecked(Code code) const noexcept
	{
		return toCoordChecked(toKey(code));
	}

	/*!
	 * @brief Convert a key to a coordinate with bounds check.
	 *
	 * @param key The key.
	 * @return The corresponding coordinate.
	 */
	[[nodiscard]] constexpr std::optional<Point3> toCoordChecked(Key key) const noexcept
	{
		return rootDepth() >= key.depth() ? std::optional<Point3>(toCoord(key))
		                                  : std::nullopt;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Traverse                                       |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	// If f returns true then children will not be visited
	template <class UnaryFunction>
	void traverse(UnaryFunction f) const
	{
		if constexpr (std::is_same_v<util::argument<UnaryFunction, 0>,
		                             Node>) {  // FIXME: Should it be const also?
			traverseRecurs(rootNode(), f);
		} else {
			traverseRecurs(rootNodeBV(), f);
		}
	}

	template <class UnaryFunction>
	void traverse(Node node, UnaryFunction f) const
	{
		if constexpr (std::is_same_v<util::argument<UnaryFunction, 0>,
		                             Node>) {  // FIXME: Should it be const also?
			traverseRecurs(node, f);
		} else {
			traverseRecurs(nodeBV(node), f);
		}
	}

	template <class UnaryFunction>
	void traverse(Code code, UnaryFunction f) const
	{
		// FIXME: Correct behaviour?
		if (Node node = node(code); node.depth() == code.depth()) {
			traverse(node, f);
		}
	}

	template <class UnaryFunction>
	void traverse(Key key, UnaryFunction f) const
	{
		traverse(code(key), f);
	}

	template <class UnaryFunction>
	void traverse(Point3 coord, depth_t depth, UnaryFunction f) const
	{
		traverse(code(coord, depth), f);
	}

	template <class UnaryFunction>
	void traverse(coord_t x, coord_t y, coord_t z, depth_t depth, UnaryFunction f) const
	{
		traverse(code(x, y, z, depth), f);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Query                                        |
	|                                                                                     |
	**************************************************************************************/

	//
	// Query
	//

	template <class Predicates>
	Query<const_query_iterator> query(Predicates&& predicates) const
	{
		return Query<const_query_iterator>(beginQuery(std::forward<Predicates>(predicates)),
		                                   endQuery());
	}

	template <class Predicates>
	Query<const_query_iterator> query(Node node, Predicates&& predicates) const
	{
		return Query<const_query_iterator>(
		    beginQuery(node, std::forward<Predicates>(predicates)), endQuery());
	}

	template <class Predicates>
	Query<const_query_iterator> query(Code code, Predicates&& predicates) const
	{
		return Query<const_query_iterator>(
		    beginQuery(code, std::forward<Predicates>(predicates)), endQuery());
	}

	template <class Predicates>
	Query<const_query_iterator> query(Key key, Predicates&& predicates) const
	{
		return query(code(key), std::forward<Predicates>(predicates));
	}

	template <class Predicates>
	Query<const_query_iterator> query(Point3 coord, depth_t depth,
	                                  Predicates&& predicates) const
	{
		return query(code(coord, depth), std::forward<Predicates>(predicates));
	}

	template <class Predicates>
	Query<const_query_iterator> query(coord_t x, coord_t y, coord_t z, depth_t depth,
	                                  Predicates&& predicates) const
	{
		return query(code(x, y, z, depth), std::forward<Predicates>(predicates));
	}

	//
	// Query bounding volume
	//

	template <class Predicates>
	Query<const_bounding_volume_query_iterator> queryBV(Predicates&& predicates) const
	{
		return Query<const_bounding_volume_query_iterator>(
		    beginQueryBV(std::forward<Predicates>(predicates)), endQueryBV());
	}

	template <class Predicates>
	Query<const_bounding_volume_query_iterator> queryBV(Node node,
	                                                    Predicates&& predicates) const
	{
		return Query<const_bounding_volume_query_iterator>(
		    beginQueryBV(node, std::forward<Predicates>(predicates)), endQueryBV());
	}

	template <class Predicates>
	Query<const_bounding_volume_query_iterator> queryBV(Code code,
	                                                    Predicates&& predicates) const
	{
		return Query<const_bounding_volume_query_iterator>(
		    beginQueryBV(code, std::forward<Predicates>(predicates)), endQueryBV());
	}

	template <class Predicates>
	Query<const_bounding_volume_query_iterator> queryBV(Key key,
	                                                    Predicates&& predicates) const
	{
		return queryBV(code(key), std::forward<Predicates>(predicates));
	}

	template <class Predicates>
	Query<const_bounding_volume_query_iterator> queryBV(Point3 coord, depth_t depth,
	                                                    Predicates&& predicates) const
	{
		return queryBV(code(coord, depth), std::forward<Predicates>(predicates));
	}

	template <class Predicates>
	Query<const_bounding_volume_query_iterator> queryBV(coord_t x, coord_t y, coord_t z,
	                                                    depth_t depth,
	                                                    Predicates&& predicates) const
	{
		return queryBV(code(x, y, z, depth), std::forward<Predicates>(predicates));
	}

	//
	// Query nearest
	//

	template <class Geometry, class Predicates>
	Query<const_query_nearest_iterator> queryNearest(Geometry&& geometry,
	                                                 Predicates&& predicates) const
	{
		return Query<const_query_nearest_iterator>(
		    beginQueryNearest(std::forward<Geometry>(geometry),
		                      std::forward<Predicates>(predicates)),
		    endQueryNearest());
	}

	template <class Geometry, class Predicates>
	Query<const_query_nearest_iterator> queryNearest(Node node, Geometry&& geometry,
	                                                 Predicates&& predicates) const
	{
		return Query<const_query_nearest_iterator>(
		    beginQueryNearest(node, std::forward<Geometry>(geometry),
		                      std::forward<Predicates>(predicates)),
		    endQueryNearest());
	}

	template <class Geometry, class Predicates>
	Query<const_query_nearest_iterator> queryNearest(Code code, Geometry&& geometry,
	                                                 Predicates&& predicates) const
	{
		return Query<const_query_nearest_iterator>(
		    beginQueryNearest(code, std::forward<Geometry>(geometry),
		                      std::forward<Predicates>(predicates)),
		    endQueryNearest());
	}

	template <class Geometry, class Predicates>
	Query<const_query_nearest_iterator> queryNearest(Key key, Geometry&& geometry,
	                                                 Predicates&& predicates) const
	{
		return queryNearest(code(key), std::forward<Geometry>(geometry),
		                    std::forward<Predicates>(predicates));
	}

	template <class Geometry, class Predicates>
	Query<const_query_nearest_iterator> queryNearest(Point3 coord, depth_t depth,
	                                                 Geometry&& geometry,
	                                                 Predicates&& predicates) const
	{
		return queryNearest(code(coord, depth), std::forward<Geometry>(geometry),
		                    std::forward<Predicates>(predicates));
	}

	template <class Geometry, class Predicates>
	Query<const_query_nearest_iterator> queryNearest(coord_t x, coord_t y, coord_t z,
	                                                 depth_t depth, Geometry&& geometry,
	                                                 Predicates&& predicates) const
	{
		return queryNearest(code(x, y, z, depth), std::forward<Geometry>(geometry),
		                    std::forward<Predicates>(predicates));
	}

	//
	// Query to output
	//

	template <class Predicates, class OutputIt>
	OutputIt query(Predicates&& predicates, OutputIt d_first) const
	{
		if constexpr (std::is_same_v<typename OutputIt::value_type, Node>) {
			return std::copy(beginQuery(std::forward<Predicates>(predicates)), endQuery(),
			                 d_first);
		} else {
			return std::copy(beginQueryBV(std::forward<Predicates>(predicates)), endQueryBV(),
			                 d_first);
		}
	}

	template <class Predicates, class OutputIt>
	OutputIt query(Node node, Predicates&& predicates, OutputIt d_first) const
	{
		if constexpr (std::is_same_v<typename OutputIt::value_type, Node>) {
			return std::copy(beginQuery(node, std::forward<Predicates>(predicates)), endQuery(),
			                 d_first);
		} else {
			return std::copy(beginQueryBV(node, std::forward<Predicates>(predicates)),
			                 endQueryBV(), d_first);
		}
	}

	template <class Predicates, class OutputIt>
	OutputIt query(Code code, Predicates&& predicates, OutputIt d_first) const
	{
		if constexpr (std::is_same_v<typename OutputIt::value_type, Node>) {
			return std::copy(beginQuery(code, std::forward<Predicates>(predicates)), endQuery(),
			                 d_first);
		} else {
			return std::copy(beginQueryBV(code, std::forward<Predicates>(predicates)),
			                 endQueryBV(), d_first);
		}
	}

	template <class Predicates, class OutputIt>
	OutputIt query(Key key, Predicates&& predicates, OutputIt d_first) const
	{
		return query(code(key), std::forward<Predicates>(predicates), d_first);
	}

	template <class Predicates, class OutputIt>
	OutputIt query(Point3 coord, depth_t depth, Predicates&& predicates,
	               OutputIt d_first) const
	{
		return query(code(coord, depth), std::forward<Predicates>(predicates), d_first);
	}

	template <class Predicates, class OutputIt>
	OutputIt query(coord_t x, coord_t y, coord_t z, depth_t depth, Predicates&& predicates,
	               OutputIt d_first) const
	{
		return query(code(x, y, z, depth), std::forward<Predicates>(predicates), d_first);
	}

	template <class Predicates, class OutputIt>
	OutputIt queryK(std::size_t k, Predicates&& predicates, OutputIt d_first) const
	{
		std::size_t count = 0;
		if constexpr (std::is_same_v<typename OutputIt::value_type, Node>) {
			for (auto it = beginQuery(std::forward<Predicates>(predicates));
			     count < k && it != endQuery(); ++it, ++count) {
				*d_first++ = *it;
			}
		} else {
			for (auto it = beginQueryBV(std::forward<Predicates>(predicates));
			     count < k && it != endQueryBV(); ++it, ++count) {
				*d_first++ = *it;
			}
		}
		return d_first;
	}

	template <class Predicates, class OutputIt>
	OutputIt queryK(Node node, std::size_t k, Predicates&& predicates,
	                OutputIt d_first) const
	{
		std::size_t count = 0;
		if constexpr (std::is_same_v<typename OutputIt::value_type, Node>) {
			for (auto it = beginQuery(node, std::forward<Predicates>(predicates));
			     count < k && it != endQuery(); ++it, ++count) {
				*d_first++ = *it;
			}
		} else {
			for (auto it = beginQueryBV(node, std::forward<Predicates>(predicates));
			     count < k && it != endQueryBV(); ++it, ++count) {
				*d_first++ = *it;
			}
		}
		return d_first;
	}

	template <class Predicates, class OutputIt>
	OutputIt queryK(Code code, std::size_t k, Predicates&& predicates,
	                OutputIt d_first) const
	{
		std::size_t count = 0;
		if constexpr (std::is_same_v<typename OutputIt::value_type, Node>) {
			for (auto it = beginQuery(code, std::forward<Predicates>(predicates));
			     count < k && it != endQuery(); ++it, ++count) {
				*d_first++ = *it;
			}
		} else {
			for (auto it = beginQueryBV(code, std::forward<Predicates>(predicates));
			     count < k && it != endQueryBV(); ++it, ++count) {
				*d_first++ = *it;
			}
		}
		return d_first;
	}

	template <class Predicates, class OutputIt>
	OutputIt queryK(Key key, std::size_t k, Predicates&& predicates, OutputIt d_first) const
	{
		return queryK(code(key), k, std::forward<Predicates>(predicates), d_first);
	}

	template <class Predicates, class OutputIt>
	OutputIt queryK(Point3 coord, depth_t depth, std::size_t k, Predicates&& predicates,
	                OutputIt d_first) const
	{
		return queryK(code(coord, depth), k, std::forward<Predicates>(predicates), d_first);
	}

	template <class Predicates, class OutputIt>
	OutputIt queryK(coord_t x, coord_t y, coord_t z, depth_t depth, std::size_t k,
	                Predicates&& predicates, OutputIt d_first) const
	{
		return queryK(code(x, y, z, depth), k, std::forward<Predicates>(predicates), d_first);
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearest(Geometry&& geometry, Predicates&& predicates, OutputIt d_first,
	                      double epsilon = 0.0) const
	{
		return std::copy(beginQueryNearest(std::forward<Geometry>(geometry),
		                                   std::forward<Predicates>(predicates), epsilon),
		                 endQueryNearest(), d_first);
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearest(Node node, Geometry&& geometry, Predicates&& predicates,
	                      OutputIt d_first, double epsilon = 0.0) const
	{
		return std::copy(beginQueryNearest(node, std::forward<Geometry>(geometry),
		                                   std::forward<Predicates>(predicates), epsilon),
		                 endQueryNearest(), d_first);
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearest(Code code, Geometry&& geometry, Predicates&& predicates,
	                      OutputIt d_first, double epsilon = 0.0) const
	{
		return std::copy(beginQueryNearest(code, std::forward<Geometry>(geometry),
		                                   std::forward<Predicates>(predicates), epsilon),
		                 endQueryNearest(), d_first);
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearest(Key key, Geometry&& geometry, Predicates&& predicates,
	                      OutputIt d_first, double epsilon = 0.0) const
	{
		return queryNearest(code(key), std::forward<Geometry>(geometry),
		                    std::forward<Predicates>(predicates), d_first, epsilon);
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearest(Point3 coord, depth_t depth, Geometry&& geometry,
	                      Predicates&& predicates, OutputIt d_first,
	                      double epsilon = 0.0) const
	{
		return queryNearest(code(coord, depth), std::forward<Geometry>(geometry),
		                    std::forward<Predicates>(predicates), d_first, epsilon);
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearest(coord_t x, coord_t y, coord_t z, depth_t depth,
	                      Geometry&& geometry, Predicates&& predicates, OutputIt d_first,
	                      double epsilon = 0.0) const
	{
		return queryNearest(code(x, y, z, depth), std::forward<Geometry>(geometry),
		                    std::forward<Predicates>(predicates), d_first, epsilon);
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearestK(std::size_t k, Geometry&& geometry, Predicates&& predicates,
	                       OutputIt d_first, double epsilon = 0.0) const
	{
		std::size_t count = 0;
		for (auto it = beginQueryNearest(std::forward<Geometry>(geometry),
		                                 std::forward<Predicates>(predicates), epsilon);
		     count < k && it != endQueryNearest(); ++it, ++count) {
			*d_first++ = *it;
		}
		return d_first;
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearestK(Node node, std::size_t k, Geometry&& geometry,
	                       Predicates&& predicates, OutputIt d_first,
	                       double epsilon = 0.0) const
	{
		std::size_t count = 0;
		for (auto it = beginQueryNearest(node, std::forward<Geometry>(geometry),
		                                 std::forward<Predicates>(predicates), epsilon);
		     count < k && it != endQueryNearest(); ++it, ++count) {
			*d_first++ = *it;
		}
		return d_first;
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearestK(Code code, std::size_t k, Geometry&& geometry,
	                       Predicates&& predicates, OutputIt d_first,
	                       double epsilon = 0.0) const
	{
		std::size_t count = 0;
		for (auto it = beginQueryNearest(code, std::forward<Geometry>(geometry),
		                                 std::forward<Predicates>(predicates), epsilon);
		     count < k && it != endQueryNearest(); ++it, ++count) {
			*d_first++ = *it;
		}
		return d_first;
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearestK(Key key, std::size_t k, Geometry&& geometry,
	                       Predicates&& predicates, OutputIt d_first,
	                       double epsilon = 0.0) const
	{
		return queryNearestK(code(key), k, std::forward<Geometry>(geometry),
		                     std::forward<Predicates>(predicates), d_first, epsilon);
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearestK(Point3 coord, depth_t depth, std::size_t k, Geometry&& geometry,
	                       Predicates&& predicates, OutputIt d_first,
	                       double epsilon = 0.0) const
	{
		return queryNearestK(code(coord, depth), k, std::forward<Geometry>(geometry),
		                     std::forward<Predicates>(predicates), d_first, epsilon);
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearestK(coord_t x, coord_t y, coord_t z, depth_t depth, std::size_t k,
	                       Geometry&& geometry, Predicates&& predicates, OutputIt d_first,
	                       double epsilon = 0.0) const
	{
		return queryNearestK(code(x, y, z, depth), k, std::forward<Geometry>(geometry),
		                     std::forward<Predicates>(predicates), d_first, epsilon);
	}

	//
	// Query iterator
	//

	template <class Predicates>
	const_query_iterator beginQuery(Predicates&& predicates) const
	{
		if constexpr (predicate::contains_spatial_predicate_v<std::decay_t<Predicates>>) {
			return const_query_iterator(
			    new Iterator<Node, Derived, NodeBV, std::decay_t<Predicates>>(
			        &derived(), rootNodeBV(), std::forward<Predicates>(predicates)));
		} else {
			return const_query_iterator(
			    new Iterator<Node, Derived, Node, std::decay_t<Predicates>>(
			        &derived(), rootNode(), std::forward<Predicates>(predicates)));
		}
	}

	template <class Predicates>
	const_query_iterator beginQuery(Node node, Predicates&& predicates) const
	{
		if constexpr (predicate::contains_spatial_predicate_v<std::decay_t<Predicates>>) {
			return const_query_iterator(
			    new Iterator<Node, Derived, NodeBV, std::decay_t<Predicates>>(
			        &derived(), nodeBV(node), std::forward<Predicates>(predicates)));
		} else {
			return const_query_iterator(
			    new Iterator<Node, Derived, Node, std::decay_t<Predicates>>(
			        &derived(), node, std::forward<Predicates>(predicates)));
		}
	}

	template <class Predicates>
	const_query_iterator beginQuery(Code code, Predicates&& predicates) const
	{
		// TODO: Implement
	}

	template <class Predicates>
	const_query_iterator beginQuery(Key key, Predicates&& predicates) const
	{
		return beginQuery(code(key), std::forward<Predicates>(predicates));
	}

	template <class Predicates>
	const_query_iterator beginQuery(Point3 coord, depth_t depth,
	                                Predicates&& predicates) const
	{
		return beginQuery(code(coord, depth), std::forward<Predicates>(predicates));
	}

	template <class Predicates>
	const_query_iterator beginQuery(coord_t x, coord_t y, coord_t z, depth_t depth,
	                                Predicates&& predicates) const
	{
		return beginQuery(code(x, y, z, depth), std::forward<Predicates>(predicates));
	}

	const_query_iterator endQuery() const
	{
		return const_query_iterator(new Iterator<Node, Derived>(rootNode()));
	}

	template <class Predicates>
	const_bounding_volume_query_iterator beginQueryBV(Predicates&& predicates) const
	{
		return const_bounding_volume_query_iterator(
		    new Iterator<NodeBV, Derived, NodeBV, Predicates>(
		        &derived(), rootNodeBV(), std::forward<Predicates>(predicates)));
	}

	template <class Predicates>
	const_bounding_volume_query_iterator beginQueryBV(Node node,
	                                                  Predicates&& predicates) const
	{
		return const_bounding_volume_query_iterator(
		    new Iterator<NodeBV, Derived, NodeBV, Predicates>(
		        &derived(), nodeBV(node), std::forward<Predicates>(predicates)));
	}

	template <class Predicates>
	const_bounding_volume_query_iterator beginQueryBV(Code code,
	                                                  Predicates&& predicates) const
	{
		// TODO: Implement
	}

	template <class Predicates>
	const_bounding_volume_query_iterator beginQueryBV(Key key,
	                                                  Predicates&& predicates) const
	{
		return beginQueryBV(code(key), std::forward<Predicates>(predicates));
	}

	template <class Predicates>
	const_bounding_volume_query_iterator beginQueryBV(Point3 coord, depth_t depth,
	                                                  Predicates&& predicates) const
	{
		return beginQueryBV(code(coord, depth), std::forward<Predicates>(predicates));
	}

	template <class Predicates>
	const_bounding_volume_query_iterator beginQueryBV(coord_t x, coord_t y, coord_t z,
	                                                  depth_t depth,
	                                                  Predicates&& predicates) const
	{
		return beginQueryBV(code(x, y, z, depth), std::forward<Predicates>(predicates));
	}

	const_bounding_volume_query_iterator endQueryBV() const
	{
		return const_bounding_volume_query_iterator(
		    new Iterator<NodeBV, Derived, NodeBV>(rootNodeBV()));
	}

	template <class Geometry, class Predicates>
	const_query_nearest_iterator beginQueryNearest(Geometry&& geometry,
	                                               Predicates&& predicates,
	                                               double epsilon = 0.0) const
	{
		return const_query_nearest_iterator(
		    new NearestIterator(&derived(), rootNodeBV(), std::forward<Geometry>(geometry),
		                        std::forward<Predicates>(predicates), epsilon));
	}

	template <class Geometry, class Predicates>
	const_query_nearest_iterator beginQueryNearest(Node node, Geometry&& geometry,
	                                               Predicates&& predicates,
	                                               double epsilon = 0.0) const
	{
		return const_query_nearest_iterator(
		    new NearestIterator(&derived(), nodeBV(node), std::forward<Geometry>(geometry),
		                        std::forward<Predicates>(predicates), epsilon));
	}

	template <class Geometry, class Predicates>
	const_query_nearest_iterator beginQueryNearest(Code code, Geometry&& geometry,
	                                               Predicates&& predicates,
	                                               double epsilon = 0.0) const
	{
		// TODO: Implement
	}

	template <class Geometry, class Predicates>
	const_query_nearest_iterator beginQueryNearest(Key key, Geometry&& geometry,
	                                               Predicates&& predicates,
	                                               double epsilon = 0.0) const
	{
		return beginQueryNearest(code(key), std::forward<Geometry>(geometry),
		                         std::forward<Predicates>(predicates), epsilon);
	}

	template <class Geometry, class Predicates>
	const_query_nearest_iterator beginQueryNearest(Point3 coord, depth_t depth,
	                                               Geometry&& geometry,
	                                               Predicates&& predicates,
	                                               double epsilon = 0.0) const
	{
		return beginQueryNearest(code(coord, depth), std::forward<Geometry>(geometry),
		                         std::forward<Predicates>(predicates), epsilon);
	}

	template <class Geometry, class Predicates>
	const_query_nearest_iterator beginQueryNearest(coord_t x, coord_t y, coord_t z,
	                                               depth_t depth, Geometry&& geometry,
	                                               Predicates&& predicates,
	                                               double epsilon = 0.0) const
	{
		return beginQueryNearest(code(x, y, z, depth), std::forward<Geometry>(geometry),
		                         std::forward<Predicates>(predicates), epsilon);
	}

	const_query_nearest_iterator endQueryNearest() const
	{
		return const_query_nearest_iterator(new NearestIterator<Derived>());
	}

	//
	// "Normal" iterator
	//

	const_iterator begin() const { return beginQuery(predicate::TRUE{}); }

	const_iterator begin(Node node) const { return beginQuery(node, predicate::TRUE{}); }

	const_iterator begin(Code code) const { return beginQuery(code, predicate::TRUE{}); }

	const_iterator begin(Key key) const { return begin(code(key)); }

	const_iterator begin(Point3 coord, depth_t depth) const
	{
		return begin(code(coord, depth));
	}

	const_iterator begin(coord_t x, coord_t y, coord_t z, depth_t depth) const
	{
		return begin(code(x, y, z, depth));
	}

	const_iterator end() const { return endQuery(predicate::TRUE{}); }

	const_bounding_volume_iterator beginBV() const
	{
		return beginQueryBV(predicate::TRUE{});
	}

	const_iterator beginBV(Node node) const
	{
		return beginQueryBV(node, predicate::TRUE{});
	}

	const_iterator beginBV(Code code) const
	{
		return beginQueryBV(code, predicate::TRUE{});
	}

	const_iterator beginBV(Key key) const { return beginBV(code(key)); }

	const_iterator beginBV(Point3 coord, depth_t depth) const
	{
		return beginBV(code(coord, depth));
	}

	const_iterator beginBV(coord_t x, coord_t y, coord_t z, depth_t depth) const
	{
		return beginBV(code(x, y, z, depth));
	}

	const_bounding_volume_iterator endBV() const { return endQueryBV(predicate::TRUE{}); }

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

		auto nodes = nodes(in);

		derived().readNodes(in, std::begin(nodes), std::end(nodes), header.compressed);

		if (propagate) {
			propagate();
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
		write(out, tree_structure, std::cbegin(nodes), std::cend(nodes), compress,
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
		write(out, tree_structure, std::cbegin(nodes), std::cend(nodes), compress,
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
		write(out, tree_structure, std::cbegin(nodes), std::cend(nodes), compress,
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
	//
	// Set leaf node size and depth levels
	//

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

	//
	// Initilize root
	//

	void initRoot()
	{
		setLeaf(root());
		resetModified(root());
	}

	//
	// To key
	//

	/*!
	 * @brief Convert a coordinate at a specific depth to a key.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The corresponding key.
	 */
	[[nodiscard]] constexpr key_t toKey(coord_t coord, depth_t depth = 0) const noexcept
	{
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
	[[nodiscard]] constexpr std::optional<key_t> toKeyChecked(
	    coord_t coord, depth_t depth = 0) const noexcept
	{
		auto min = -nodeSize(rootDepth() - 1);
		auto max = -min;
		return min <= coord && max >= coord ? std::optional<key_t>(toKey(coord, depth))
		                                    : std::nullopt;
	}

	//
	// To coordinate
	//

	/*!
	 * @brief Convert a key to a coordinate at a specific depth.
	 *
	 * @param key The key.
	 * @param depth The depth of the coordinate.
	 * @return The corresponding coordinate.
	 */
	[[nodiscard]] constexpr coord_t toCoord(key_t key, depth_t depth = 0) const noexcept
	{
		constexpr auto sub64 = std::minus<int_fast64_t>{};
		return depth_levels_ == depth
		           ? 0
		           : (std::floor(sub64(key, max_value_) / static_cast<coord_t>(1U << depth)) +
		              coord_t(0.5)) *
		                 nodeSize(depth);

		// // FIXME: Check if correct
		// return depth_levels_ == depth
		//            ? 0
		//            : (((key >> depth) << depth) - max_value_) * node_size_;
	}

	//
	// Get node
	//

	[[nodiscard]] constexpr LeafNode const& leafNode(Node node) const
	{
		return *static_cast<LeafNode*>(node.data());
	}

	[[nodiscard]] constexpr LeafNode& leafNode(Node node)
	{
		return *static_cast<LeafNode*>(node.data());
	}

	[[nodiscard]] LeafNode const& leafNode(Code code) const
	{
		InnerNode const* node = &root();
		depth_t depth = rootDepth();
		depth_t min_depth = std::max(code.depth(), depth_t(1));
		while (min_depth != depth && isParent(*node)) {
			--depth;
			node = &innerNode(*node, code.index(depth));
		}

		return 0 == code.depth() && isParent(*node) ? leafNode(*node, code.index()) : *node;
	}

	[[nodiscard]] LeafNode& leafNode(Code code)
	{
		InnerNode* node = &root();
		depth_t depth = rootDepth();
		depth_t min_depth = std::max(code.depth(), depth_t(1));
		while (min_depth != depth && isParent(*node)) {
			--depth;
			node = &innerNode(*node, code.index(depth));
		}

		return 0 == code.depth() && isParent(*node) ? leafNode(*node, code.index()) : *node;
	}

	[[nodiscard]] constexpr InnerNode const& innerNode(Node node) const
	{
		return *static_cast<InnerNode*>(node.data());
	}

	[[nodiscard]] constexpr InnerNode& innerNode(Node& node)
	{
		return *static_cast<InnerNode*>(node.data());
	}

	[[nodiscard]] InnerNode const& innerNode(Code code) const
	{
		assert(0 != code.depth());

		InnerNode const* node = &root();
		depth_t depth = rootDepth();
		depth_t min_depth = code.depth();
		while (min_depth != depth && isParent(*node)) {
			--depth;
			node = &innerNode(*node, code.index(depth));
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
			--depth;
			node = &innerNode(*node, code.index(depth));
		}

		return *node;
	}

	[[nodiscard]] std::pair<LeafNode const*, depth_t> nodeAndDepth(Code code) const
	{
		InnerNode const* node = &root();
		depth_t depth = rootDepth();
		depth_t min_depth = std::max(code.depth(), depth_t(1));
		while (min_depth != depth && isParent(*node)) {
			--depth;
			node = &innerNode(*node, code.index(depth));
		}

		return 0 == code.depth() && isParent(*node) ? {&leafNode(*node, code.index()), 0}
		                                            : {node, depth};
	}

	[[nodiscard]] std::pair<LeafNode*, depth_t> nodeAndDepth(Code code)
	{
		InnerNode* node = &root();
		depth_t depth = rootDepth();
		depth_t min_depth = std::max(code.depth(), depth_t(1));
		while (min_depth != depth && isParent(*node)) {
			--depth;
			node = &innerNode(*node, code.index(depth));
		}

		return 0 == code.depth() && isParent(*node) ? {&leafNode(*node, code.index(0)), 0}
		                                            : {node, depth};
	}

	//
	// Create node
	//

	[[nodiscard]] LeafNode const& createLeafNode(Code code) const
	{
		// TODO: Implement
	}

	[[nodiscard]] LeafNode& createLeafNode(Code code)
	{
		// TODO: Implement
	}

	[[nodiscard]] InnerNode const& createInnerNode(Code code) const
	{
		// TODO: Implement
	}

	[[nodiscard]] InnerNode& createInnerNode(Code code)
	{
		// TODO: Implement
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

	[[nodiscard]] static constexpr LeafNode const& leafChild(InnerNode const& node,
	                                                         index_t index)
	{
		return leafChildren(node)[index];
	}

	[[nodiscard]] static constexpr LeafNode& leafChild(InnerNode& node, index_t index)
	{
		return leafChildren(node)[index];
	}

	[[nodiscard]] static constexpr InnerNode const& innerChild(InnerNode const& node,
	                                                           index_t index)
	{
		return innerChildren(node)[index];
	}

	[[nodiscard]] static constexpr InnerNode& innerChild(InnerNode& node, index_t index)
	{
		return innerChildren(node)[index];
	}

	[[nodiscard]] static constexpr LeafNode const& child(InnerNode const& node,
	                                                     index_t index, depth_t depth)
	{
		return 1 == depth ? leafChild(node, index) : innerChild(node, index);
	}

	[[nodiscard]] static constexpr LeafNode& child(InnerNode& node, index_t index,
	                                               depth_t depth)
	{
		return 1 == depth ? leafChild(node, index) : innerChild(node, index);
	}

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

	template <class Derived2, class DataType2, bool ReuseNodes2, bool LockLess2,
	          bool CountNodes2>
	OctreeBase(
	    OctreeBase<Derived2, DataType2, ReuseNodes2, LockLess2, CountNodes2> const& other)
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
		clear(rhs.nodeSize(), rhs.depthLevels());

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
		clear(rhs.nodeSize(), rhs.depthLevels(), true);

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

	template <class Derived2, class DataType2, bool ReuseNodes2, bool LockLess2,
	          bool CountNodes2>
	OctreeBase& operator=(
	    OctreeBase<Derived2, DataType2, ReuseNodes2, LockLess2, CountNodes2> const& rhs)
	{
		// TODO: Should this clear?
		clear(rhs.nodeSize(), rhs.depthLevels());

		depth_levels_ = rhs.depth_levels_;
		max_value_ = rhs.max_value_;
		node_size_ = rhs.node_size_;
		node_size_factor_ = rhs.node_size_factor_;
		automatic_prune_ = rhs.automatic_prune_;
		return *this;
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
	|                                        Root                                         |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	[[nodiscard]] constexpr InnerNode const& root() const noexcept { return root_; }

	[[nodiscard]] constexpr InnerNode& root() noexcept { return root_; }

	[[nodiscard]] constexpr index_field_t rootIndexField() const noexcept
	{
		return index_field_t(1);
	}

	[[nodiscard]] constexpr index_t rootIndex() const noexcept { return index_t(0); }

	/**************************************************************************************
	|                                                                                     |
	|                                        Apply                                        |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	template <class NodeType, class BinaryFunction, class UnaryFunction,
	          typename = std::enable_if_t<std::is_copy_constructible_v<BinaryFunction> &&
	                                      std::is_copy_constructible_v<UnaryFunction>>>
	void apply(NodeType const& node, BinaryFunction f, UnaryFunction f2,
	           bool const propagate)
	{
		if (leaf(node)) {
			f(leafNode(node), node.index());
		} else {
			applyAllRecurs(innerNode(node), node.depth(), f, f2);
		}

		if (!modified(node)) {
			setModifiedParents(node);
			setModified(leafNode(node), ...);
		}

		if (propagate) {
			updateModifiedNodes();
		}
	}

	template <class BinaryFunction, UnaryFunction,
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
			updateModifiedNodes();
		}
	}

	template <class BinaryFunction, class UnaryFunction,
	          typename = std::enable_if_t<std::is_copy_constructible_v<BinaryFunction> &&
	                                      std::is_copy_constructible_v<UnaryFunction>>>
	void applyRecurs(InnerNode& node, depth_t depth, Code code, BinaryFunction f,
	                 UnaryFunction2 f2)
	{
		if (code.depth() == depth) {
			if (leaf(node)) {
				f(static_cast<LeafNode&>(node));
			} else {
				applyAllRecurs(node, depth, f, f2);
			}
		} else if (1 == depth) {
			createLeafChildren(node);
			LeafNode& child = leafChild(node, code.index(0));
			f(child);
			setModified(child, true);
		} else {
			createInnerChildren(node, depth);
			applyRecurs(innerChild(node, code.index(depth - 1)), depth - 1, code, f, f2);
		}

		setModified(node, true);
	}

	template <class BinaryFunction, class UnaryFunction,
	          typename = std::enable_if_t<std::is_copy_constructible_v<BinaryFunction> &&
	                                      std::is_copy_constructible_v<UnaryFunction>>>
	void applyAllRecurs(InnerNode& node, depth_t depth, BinaryFunction f, UnaryFunction f2)
	{
		if (1 == depth) {
			auto& children = leafChildren(node);
			f2(children);
			setModified(children, true);
		} else {
			auto& children = innerChildren(node);
			if (leaf(children)) {
				f2(children);
			} else {
				for (InnerNode& child : innerChildren(node)) {
					if (leaf(child)) {
						f(static_cast<LeafNode&>(child));
					} else {
						applyAllRecurs(child, depth - 1, f, f2);
					}
				}
			}
			setModified(children, true);
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Traverse                                       |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comment

	template <class NodeType, class UnaryFunction>
	void traverseRecurs(NodeType node, UnaryFunction f) const
	{
		if (f(node) || isLeaf(node)) {
			return;
		}

		node = nodeChild(node, 0);
		for (index_t index = 0; 8 != index; ++index) {
			traverseRecurs(nodeSibling(node, index), f);
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                    Access nodes                                     |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Implement

	/**************************************************************************************
	|                                                                                     |
	|                                       Center                                        |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	//
	// Child center
	//

	[[nodiscard]] static constexpr Point3 childCenter(Point3 parent_center,
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

	[[nodiscard]] static constexpr Point3 siblingCenter(Point3 center,
	                                                    node_size_t half_size,
	                                                    index_t index,
	                                                    index_t sibling_index)
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

	[[nodiscard]] static constexpr Point3 parentCenter(Point3 child_center,
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

	void setModified(InnerNode& node, index_t index, depth_t depth, depth_t min_depth)
	{
		node.setModifiedIndex(index);

		if (min_depth < depth) {
			if (1 == depth) {
				leafChild(node, index).setModified();
			} else {
				setModifiedRecurs(innerChild(node, index), depth - 1, min_depth);
			}
		}
	}

	void setModifiedRecurs(InnerNode& node, depth_t depth, depth_t min_depth)
	{
		node.setModified();

		if (node.isAllLeaf() || depth == min_depth) {
			return;
		}

		if (1 == depth) {
			// All are allocated so we can just set all to modified, even if they are leaves
			for (auto& child : leafChildren(node)) {
				child.setModified();
			}
		} else {
			for (index_t index = 0; 8 != index; ++index) {
				if (node.isParentIndex(index)) {
					setModifiedRecurs(innerNode(node, index), depth - 1, min_depth);
				}
			}
		}
	}

	//
	// Reset modified
	//

	void resetModified(InnerNode& node, index_t index, depth_t depth, depth_t max_depth)
	{
		// TODO: Implement
	}

	void resetModifiedRecurs(InnerNode& node, depth_t depth, depth_t max_depth)
	{
		// TODO: Implement
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
		node.setModifiedIndex(index);
		if (code.depth() < depth - 1 && node.isParentIndex(index)) {
			setModifiedParentsRecurs(innerChild(node, index), code.index(depth - 1), depth - 1,
			                         code);
		}
	}

	//
	// Reset Children modified
	//

	void resetModifiedRecurs(InnerNode& node, depth_t depth, depth_t max_depth)
	{
		if (node.isNoneModified()) {
			// Already clear
			return;
		}

		if (depth <= max_depth) {
			node.resetModified();
		}

		if (node.isAllLeaf()) {
			return;
		}

		if (1 == depth) {
			// All are allocated so we can just resset all, even if they are leaves
			for (auto& child : leafChildren(node)) {
				child.resetModified();
			}
		} else {
			for (index_t index = 0; 8 != index; ++index) {
				if (node.isParentIndex(index)) {
					resetModifiedRecurs(innerNode(node, index), depth - 1, max_depth);
				}
			}
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Propagate                                      |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	template <bool KeepModified>
	void updateNode(InnerNode& node, index_field_t indices, depth_t const depth)
	{
		if (1 == depth) {
			derived().updateNode(node, indices, leafChildren(node));
		} else {
			derived().updateNode(node, indices, innerChildren(node));
		}

		prune(node, indices, depth);
	}

	void propagateModified(LeafNode& node, index_t index, bool keep_modified)
	{
		if (!keep_modified) {
			node.resetModifiedIndex(index);
		}
	}

	void propagateModified(InnerNode& node, index_t index, depth_t depth,
	                       bool keep_modified, depth_t max_depth)
	{
		if (!node.isModifiedIndex(index)) {
			return;  // Nothing modified here
		}

		if (1 == depth) {
			if (!keep_modified) {
				leafChild(node, index).resetModified();
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
		index_field_t modified_parent = node.isModified() & node.isParent();

		if (index_field_t(0) == modified_parent) {
			if constexpr (!KeepModified) {
				node.resetModified();
			}
			return;
		}

		if (1 == depth) {
			if constexpr (!KeepModified) {
				for (index_t index = 0; 8 != index; ++index) {
					if (index_field_t(0) == (modified_parent >> index) & index_field_t(1)) {
						continue;
					}
					leafChild(node, index).resetModified();
				}
			}
		} else {
			for (index_t index = 0; 8 != index; ++index) {
				if (index_field_t(0) == (modified_parent >> index) & index_field_t(1)) {
					continue;
				}
				propagateModifiedRecurs<KeepModified>(innerChild(node, index), depth - 1,
				                                      max_depth);
			}
		}

		if (depth <= max_depth) {
			updateNode<KeepModified>(node, modified_parent, depth);
			if constexpr (!KeepModified) {
				node.resetModified();
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
	[[nodiscard]] static index_field_t isCollapsible(InnerNode const& node,
	                                                 index_field_t const indices,
	                                                 depth_t const depth)
	{
		index_field_t collapsible = 0;
		for (index_t index = 0; 8 != index; ++index) {
			if (0 == (indices >> index) & index_field_t(1)) {
				continue;
			}
			if (1 == depth) {
				if (leafChild(node, index).isCollapsible(node, index)) {
					collapsible |= index_field_t(1) << index;
				}
			} else {
				if (innerChild(node, index).isCollapsible(node, index)) {
					collapsible |= index_field_t(1) << index;
				}
			}
		}
		return collapsible;
	}

	// NOTE: Only call with nodes that have children
	index_field_t prune(InnerNode& node, index_field_t indices, depth_t depth)
	{
		indices = isCollapsible(node, indices, depth);
		if (indices) {
			deleteChildren(node, indices, depth);
		}
		return indices;
	}

	index_field_t pruneRecurs(InnerNode& node, depth_t depth)
	{
		if (node.isAllLeaf()) {
			return node.isLeaf();
		}

		if (1 == depth) {
			return node.isLeaf() | prune(node, node.isParent(), depth);
		}

		index_field_t prunable = 0;
		for (index_t index = 0; 8 != index; ++index) {
			if (node.isLeafIndex(index)) {
				continue;
			}

			auto& child = innerChild(node, index);
			if (child.isParent() == pruneRecurs(child, depth - 1)) {
				prunable |= index_field_t(1) << index;
			}
		}

		return node.isLeaf() | (0 == prunable ? prunable : prune(node, prunable, depth));
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

	void createLeafChildren(InnerNode& node, index_field_t indices)
	{
		indices &= leaf(node);
		if (0U == indices) {
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
			if (0U == 1U & (indices >> index)) {
				continue;
			}
			++num;
			leafChildren(node, index).fill(node, index);
		}

		num_leaf_nodes += 8 * num;
		num_inner_leaf_nodes_ -= 1 * num;
		num_inner_nodes_ += 1 * num;

		resetLeaf(node, indices);
		if constexpr (!LockLess) {
			unlockChildren(0);
		}
	}

	void createInnerChildren(InnerNode& node, depth_t depth)
	{
		createInnerChildren(node, leaf(node), depth);
	}

	void createInnerChildren(InnerNode& node, index_field_t indices, depth_t depth)
	{
		indices &= leaf(node);
		if (0U == indices) {
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
			if (0U == 1U & (indices >> index)) {
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

	void deleteLeafChildren(InnerNode& node, index_field_t indices,
	                        bool manual_pruning = false)
	{
		index_field_t new_leaf = indices & ~leaf(node);
		if (0 == new_leaf) {
			return;
		}

		indices |= leaf(node);

		setLeaf(node, indices);

		std::size_t num = 0;
		for (index_t index = 0; 8 != index; ++index) {
			if (1U & (new_leaf >> index)) {
				++num;
			}
		}
		num_leaf_nodes_ -= 8 * num;
		num_inner_leaf_nodes_ += 1 * num;
		num_inner_nodes_ -= 1 * num;

		if (std::numeric_limits<index_field_t>::max() != indices) {
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

	void deleteChildren(InnerNode& node, index_field_t indices, depth_t const depth,
	                    bool manual_pruning = false)
	{
		index_field_t new_leaf = indices & ~leaf(node);
		if (0 == new_leaf) {
			return;
		}

		indices |= leaf(node);

		setLeaf(node, indices);

		std::size_t num = 0;
		for (index_t index = 0; 8 != index; ++index) {
			if (1U & (new_leaf >> index)) {
				++num;
			}
		}
		num_inner_leaf_nodes_ -= 7 * num;
		num_inner_nodes_ -= 1 * num;

		if (std::numeric_limits<index_field_t>::max() != indices) {
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

	bool tryLockChildren(depth_t depth)
	{
		return !children_locks_[depth].test_and_set(std::memory_order_acquire);
	}

	void lockChildren(depth_t depth)
	{
		while (!tryLockChildren(depth))
			;
	}

	bool lockIfLeaf(InnerNode const& node, depth_t depth)
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

	bool tryLockLeaves()
	{
		return !free_leaf_block_lock_.test_and_set(std::memory_order_acquire);
	}

	void lockLeaves()
	{
		while (!tryLockLeaves()) {
		}
	}

	bool lockIfNonEmptyLeaves()
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

	bool tryLockInner()
	{
		return !free_inner_block_lock_.test_and_set(std::memory_order_acquire);
	}

	void lockInner()
	{
		while (!tryLockInner()) {
		}
	}

	bool lockIfNonEmptyInner()
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
		index_field_t indices;

		NodeAndIndices() = default;

		NodeAndIndices(LeafNode& node, index_field_t indices) : node(node), indices(indices)
		{
		}
	};

	template <bool Reference>
	struct ConstNodeAndIndices {
		std::conditional_t<Reference, std::reference_wrapper<LeafNode const>, LeafNode> node;
		index_field_t indices;

		ConstNodeAndIndices() = default;

		ConstNodeAndIndices(LeafNode const& node, index_field_t indices)
		    : node(node), indices(indices)
		{
		}
	};

	FileOptions fileOptions(bool const compress) const
	{
		FileOptions options;
		options.compressed = compress;
		options.leaf_size = nodeSize();
		options.depth_levels = depthLevels();
		return options;
	}

	std::vector<NodeAndIndices> readNodes(std::istream& in)
	{
		auto [tree_structure, size] = readTreeStructure(in);
		std::uint64_t num_nodes = readNumNodes(in);
		return retrieveNodes(tree_structure, size);
	}

	std::pair<std::unique_ptr<std::uint8_t[]>, std::uint64_t> readTreeStructure(
	    std::istream& in)
	{
		std::uint64_t num;
		in.read(reinterpret_cast<char*>(&num), sizeof(num));

		auto tree_structure = std::make_unique<std::uint8_t[]>(num);
		in.read(reinterpret_cast<char*>(tree_structure.get()), num * sizeof(std::uint8_t));

		return {std::move(tree_structure), num};
	}

	std::uint64_t readNumNodes(std::istream& in)
	{
		std::uint64_t num_nodes;
		in.read(reinterpret_cast<char*>(&num_nodes), sizeof(num_nodes));
		return num_nodes;
	}

	std::vector<NodeAndIndices> retrieveNodes(
	    std::unique_ptr<std::uint8_t[]> const& tree_structure,
	    std::uint64_t const tree_structure_size, std::uint64_t num_nodes)
	{
		std::vector<NodeAndIndices> nodes;
		nodes.reserve(num_nodes);

		if (tree_structure[0]) {
			nodes.emplace_back(root(), tree_structure[0]);
			setModified(root());
		} else if (tree_structure[1]) {
			createInnerChildren(root(), rootIndex());
			retrieveNodesRecurs(innerChildren(root(), rootIndex()), tree_structure[1],
			                    rootDepth() - 1, std::next(tree_structure.get(), 2), nodes);
		}

		return nodes;
	}

	std::uint8_t const* retrieveNodesRecurs(LeafNodeBlock& node, index_field_t indices,
	                                        std::uint8_t const* tree_structure,
	                                        std::vector<NodeAndIndices>& nodes)
	{
		for (index_t index = 0; 8 != index; ++index) {
			if (std::uint8_t(0) == std::uint8_t(1) & (indices >> index)) {
				continue;
			}

			index_field_t const valid_return = *tree_structure++;
			nodes.emplace_back(node[index], valid_return);
			setModified(node[index], valid_return);
		}

		return tree_structure;
	}

	std::uint8_t const* retrieveNodesRecurs(InnerNodeBlock& node, index_field_t indices,
	                                        depth_t const depth,
	                                        std::uint8_t const* tree_structure,
	                                        std::vector<NodeAndIndices>& nodes)
	{
		for (index_t index = 0; 8 != index; ++index) {
			if (std::uint8_t(0) == std::uint8_t(1) & (indices >> index)) {
				continue;
			}

			index_field_t const valid_return = *tree_structure++;
			index_field_t const valid_inner = *tree_structure++;

			if (valid_return) {
				nodes.emplace_back(node[index], valid_return);
			}

			if (valid_inner) {
				if (1 == depth) {
					createLeafChildren(node[index], ...);
					// TODO: Implement
				} else {
					createInnerChildren(node[index], ...);
					// TODO: Implement
				}
			}

			setModified(node[index], valid_return | valid_inner);
		}

		return tree_structure;
	}

	std::uint8_t const* retrieveNodesRecurs(InnerNode& node, index_field_t indices,
	                                        depth_t const depth,
	                                        std::uint8_t const* tree_structure,
	                                        std::vector<NodeAndIndices>& nodes)
	{
		for (index_t index = 0; 8 != index; ++index) {
			if (std::uint8_t(0) == std::uint8_t(1) & (indices >> index)) {
				continue;
			}

			index_field_t const children_valid_return = *tree_structure++;

			if (1 == depth) {
				if (0 == children_valid_return) {
					continue;
				}

				// TODO: Implement
			} else {
				index_field_t const children_valid_inner = *tree_structure++;

				if (0 == children_valid_return && 0 == children_valid_inner) {
					continue;
				}

				// TODO: Implement
			}
		}

		return tree_structure;

		if (1 == depth) {
			if (0 == children_valid_return) } else {
		}

		// TODO: Implement

		if (1 == depth) {
			if (0 == children_valid_return) {
				return tree_structure;
			}

			setModified(node, children_valid_return);

			createLeafChildren(node);

			for (std::size_t i = 0; 8 != i; ++i) {
				if ((children_valid_return >> i) & 1U) {
					nodes.push_back(
					    std::ref(leafChild(node, i)));  // TODO: push_back or emplace_back?
				}
			}
		} else {
			std::uint8_t const child_valid_inner = *indicators++;

			if (0 == children_valid_return && 0 == child_valid_inner) {
				return indicators;
			}

			setModified(node, true);

			createInnerChildren(node, depth);

			for (std::size_t i = 0; 8 != i; ++i) {
				if ((children_valid_return >> i) & 1U) {
					nodes.push_back(std::ref(static_cast<LeafNode&>(
					    innerChild(node, i))));  // TODO: push_back or emplace_back?
				} else if ((child_valid_inner >> i) & 1U) {
					indicators =
					    retrieveNodesRecurs(indicators, nodes, innerChild(node, i), depth - 1);
				}
			}
		}

		return indicators;
	}

	template <class Predicates>
	std::pair<std::vector<std::uint8_t>, std::vector<ConstNodeAndIndices>> data(
	    Predicates const& predicates) const
	{
		std::vector<std::uint8_t> tree_structure;
		std::vector<ConstNodeAndIndices> nodes;

		std::conditional_t<predicate::contains_spatial_predicate_v<Predicates>, NodeBV, Node>
		    root = rootNodeBV();

		bool valid_return =
		    predicate::PredicateValueCheck<Predicates>::apply(predicates, derived(), root);
		bool valid_inner = !valid_return && predicate::PredicateInnerCheck<Predicates>::apply(
		                                        predicates, derived(), root);

		tree_structure.push_back(valid_return ? 1U : 0U);
		tree_structure.push_back(valid_inner ? 1U : 0U);

		if (valid_return) {
			nodes.emplace_back(root(), 1U);
		} else if (valid_inner) {
			dataRecurs(child(root, 0), predicates, tree_structure, nodes);
			if (nodes.empty()) {
				//  Nothing was added
				tree_structure.clear();
			}
		}

		return {std::move(tree_structure), std::move(nodes)};
	}

	template <class Predicates, class NodeType>
	void dataRecurs(NodeType const& node, Predicates const& predicates,
	                std::vector<std::uint8_t>& tree_structure,
	                std::vector<ConstNodeAndIndices>& nodes) const
	{
		std::uint8_t valid_return = 0;

		if (0 == node.depth()) {
			for (index_t index = 0; 8 != index; ++index) {
				if (predicate::PredicateValueCheck<Predicates>::apply(predicates, derived(),
				                                                      sibling(node, index))) {
					valid_return |= std::uint8_t(1) << index;
				}
			}

			tree_structure.push_back(valid_return);

			if (valid_return) {
				nodes.emplace_back(leafNode(node), valid_return);
			}
		} else {
			std::uint8_t valid_inner = 0;
			for (index_t index = 0; 8 != index; ++index) {
				auto s = sibling(node, index);

				if (predicate::PredicateValueCheck<Predicates>::apply(predicates, derived(), s)) {
					valid_return |= std::uint8_t(1) << index;
				} else if (predicate::PredicateInnerCheck<Predicates>::apply(predicates,
				                                                             derived(), s)) {
					valid_inner |= std::uint8_t(1) << index;
				}
			}

			tree_structure.push_back(valid_return);
			tree_structure.push_back(valid_inner);

			auto cur_tree_structure_size = tree_structure.size();
			auto cur_nodes_size = nodes.size();

			if (valid_return) {
				nodes.emplace_back(leafNode(node), valid_return);
			}

			if (valid_inner) {
				for (index_t index = 0; 8 != index; ++index) {
					if (std::uint8_t(1) & (valid_inner >> index)) {
						auto s = sibling(node, index);
						dataRecurs(child(s, 0), predicates, tree_structure, nodes);
					}
				}
			}

			if (nodes.size() == cur_nodes_size) {
				tree_structure.resize(cur_tree_structure_size);
				tree_structure[tree_structure.size() - 1] = 0;
				tree_structure[tree_structure.size() - 2] = 0;
			}
		}
	}

	template <bool Propagate>
	std::pair<std::vector<std::uint8_t>, std::vector<ConstNodeAndIndices>> modifiedData()
	{
		std::vector<std::uint8_t> tree_structure;
		std::vector<ConstNodeAndIndices> nodes;

		InnerNode& root = root();
		depth_t depth = rootDepth();

		index_field_t valid_return = leaf(root) & modified(root);
		index_field_t valid_inner = modified(root);

		tree_structure.push_back(valid_return);
		tree_structure.push_back(valid_inner);

		if (valid_return) {
			nodes.emplace_back(root, valid_return);
			if constexpr (Propagate) {
				propagate(root, valid_return);
			}
		} else if (valid_inner) {
			modifiedDataRecurs<Propagate>(innerChildren(root), valid_inner, depth - 1,
			                              tree_structure, nodes);
			if constexpr (Propagate) {
				propagate(root, valid_inner, depth);
			}
			if (nodes.empty()) {
				//  Nothing was added
				tree_structure.clear();
			}
		}

		resetModified(root, valid_inner);

		return {std::move(tree_structure), std::move(nodes)};  // FIXME: Check if RVO
	}

	template <bool Propagate>
	void modifiedDataRecurs(LeafNodeBlock& nodes, index_field_t indices,
	                        std::vector<std::uint8_t>& tree_structure,
	                        std::vector<ConstNodeAndIndices>& nodes)
	{
		for (index_t index = 0; nodes.size() != index; ++index) {
			if (index_field_t(0) == index_field_t(1) & (indices >> index)) {
				continue;
			}

			index_field_t m = modified(nodes[index]);
			tree_structure.push_back(m);

			if (index_field_t(0) == m) {
				continue;
			}

			nodes.emplace_back(nodes[index], m);

			if constexpr (Propagate) {
				propagate(nodes[index], m);
			}

			resetModified(nodes[index]);
		}
	}

	template <bool Propagate>
	void modifiedDataRecurs(InnerNodeBlock& nodes, index_field_t indices,
	                        depth_t const depth, std::vector<std::uint8_t>& tree_structure,
	                        std::vector<ConstNodeAndIndices>& nodes)
	{
		for (index_t index = 0; nodes.size() != index; ++index) {
			if (index_field_t(0) == index_field_t(1) & (indices >> index)) {
				continue;
			}

			index_field_t m = modified(nodes[index]);
			index_field_t l = leaf(nodes[index]);

			index_field_t valid_return = m & l;
			index_field_t valid_inner = m & ~l;

			tree_structure.push_back(valid_return);
			tree_structure.push_back(valid_inner);

			auto const cur_tree_structure_size = tree_structure.size();
			auto const cur_nodes_size = nodes.size();

			if (valid_return) {
				nodes.emplace_back(nodes[index], valid_return);
			}

			if (valid_inner) {
				if (1 == depth) {
					modifiedDataRecurs<Propagate>(leafChildren(nodes[index]), valid_inner,
					                              tree_structure, nodes);
				} else {
					modifiedDataRecurs<Propagate>(innerChildren(nodes[index]), valid_inner,
					                              depth - 1, tree_structure, nodes);
				}
			}

			if constexpr (Propagate) {
				propagate(nodes[index], m);
			}

			resetModified(nodes[index]);

			if (nodes.size() == cur_nodes_size) {
				tree_structure.resize(cur_tree_structure_size);
				tree_structure[tree_structure.size() - 1] = 0;
				tree_structure[tree_structure.size() - 2] = 0;
			}
		}
	}

	template <class InputIt>
	void write(std::ostream& out, std::vector<std::uint8_t> const& tree_structure,
	           InputIt first, InputIt last, bool compress,
	           int compression_acceleration_level, int compression_level) const
	{
		writeHeader(out, fileOptions(compress));
		writeTreeStructure(out, tree_structure);
		writeNumNodes(out, std::distance(first, last));
		writeNodes(out, first, last, compress, compression_acceleration_level,
		           compression_level);
	}

	void writeTreeStructure(std::ostream& out, std::vector<std::uint8_t> const& tree) const
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
	void writeNodes(std::ostream& out, InputIt first, InputIt last, bool const compress,
	                int const compression_acceleration_level,
	                int const compression_level) const
	{
		derived().writeNodes(out, first, last, compress, compression_acceleration_level,
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