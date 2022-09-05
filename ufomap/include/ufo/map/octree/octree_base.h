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
#include <ufo/map/octree/query.h>
#include <ufo/map/point.h>
#include <ufo/map/predicate/octree.h>
#include <ufo/map/predicate/predicates.h>
#include <ufo/map/predicate/spatial.h>
#include <ufo/map/types.h>
#include <ufo/math/util.h>
#include <ufo/util/type_traits.h>

// STL
#include <algorithm>
#include <array>
#include <atomic>
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
// Utilizing Curiously recurring template pattern (CRTP)
template <class Derived, class DataType, bool ReuseNodes = false, bool LockLess = false>
class OctreeBase
{
 private:
	// Minimum number of depth levels
	static constexpr depth_t MIN_DEPTH_LEVELS = 2;
	// Maximum number of depth levels
	static constexpr depth_t MAX_DEPTH_LEVELS = 21;

	using LeafNode = OctreeLeafNode<DataType>;
	using InnerNode = OctreeInnerNode<LeafNode>;
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
	//
	// Reserve
	//

	void reserve(std::size_t num_leaf_nodes, std::size_t num_inner_nodes)
	{
		// TODO: Implement
		// free_inner_blocks_
		// free_leaf_blocks_
	}

	//
	// Clear
	//

	/*!
	 * @brief Erases the map. After this call, the map contains only the default constructed
	 * root node.
	 *
	 * @param prune If the memory should be cleared.
	 */
	void clear(bool prune = false) { clear(size(), depthLevels(), prune); }

	/*!
	 * @brief Erases the map and change the resolution and the number of depth levels. After
	 * this call, the map contains only the default constructed root node.
	 *
	 * @param new_resolution The new resolution.
	 * @param new_depth_levels The new number of depth levels.
	 * @param prune If the memory should be cleared.
	 */
	void clear(node_size_t new_leaf_size, depth_t new_depth_levels, bool prune = false)
	{
		deleteChildren(root(), rootIndex(), rootDepth(), prune);

		// FIXME: Should this be first?
		setLeafSizeAndDepthLevels(new_leaf_size, new_depth_levels);

		derived().initRoot();
	}

	//
	// Size
	//

	/*!
	 * @brief Get the resolution (node size) of the octree at a specific depth.
	 *
	 * @param depth The depth of the resolution.
	 * @return The resolution (node size) of the octree.
	 */
	[[nodiscard]] constexpr node_size_t size(depth_t depth = 0) const noexcept
	{
		return size_[depth];
	}

	[[nodiscard]] constexpr node_size_t size(Node node) const noexcept
	{
		return size(node.depth());
	}

	[[nodiscard]] node_size_t size(Code code) const { return size(code.depth()); }

	// TODO: Call it size instead of resolution? and have size(node)?

	//
	// Automatic pruning
	//

	/*!
	 * @brief Check if autonomaic pruning is enabled.
	 *
	 * @return Whether automatic pruning is enabled.
	 */
	[[nodiscard]] constexpr bool automaticPruning() const noexcept
	{
		return automatic_pruning_enabled_;
	}

	/*!
	 * @brief Set automatic pruning.
	 *
	 * @param enable New state of the automatic pruning.
	 */
	constexpr void automaticPruning(bool enable) noexcept
	{
		automatic_pruning_enabled_ = enable;
	}

	//
	// Depth levels
	//

	/*!
	 * @brief The octree depth levels.
	 *
	 * @return The number of octree depth levels.
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
	// Get min coordinate octree can store
	//

	/*!
	 * @return The minimum coordinate the octree can store.
	 */
	[[nodiscard]] Point3 min() const
	{
		auto half_size = -getNodeSize(depthLevels() - 1);
		return Point3(half_size, half_size, half_size);
	}

	template <class NodeType>
	[[nodiscard]] Point3 min(NodeType const& node) const
	{
		if constexpr (std::is_same_v<NodeBV, NodeType>) {
			return node.min();
		} else {
			return min(node.code());
		}
	}

	[[nodiscard]] Point3 min(Code code) const
	{
		// TODO: Implement
	}

	[[nodiscard]] Point3 min(Key key) const
	{
		// TODO: Implement
	}

	[[nodiscard]] Point3 min(Point3 coord, depth_t depth = 0) const
	{
		// TODO: Implement
	}

	[[nodiscard]] Point3 min(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		// TODO: Implement
	}

	//
	// Get max coordinate octree can store
	//

	/*!
	 * @return The maximum coordinate the octree can store.
	 */
	[[nodiscard]] Point3 max() const
	{
		auto half_size = getNodeSize(depthLevels() - 1);
		return Point3(half_size, half_size, half_size);
	}

	template <class NodeType>
	[[nodiscard]] Point3 max(NodeType const& node) const
	{
		if constexpr (std::is_same_v<NodeBV, NodeType>) {
			return node.max();
		} else {
			return max(node.code());
		}
	}

	[[nodiscard]] Point3 max(Code code) const
	{
		// TODO: Implement
	}

	[[nodiscard]] Point3 max(Key key) const
	{
		// TODO: Implement
	}

	[[nodiscard]] Point3 max(Point3 coord, depth_t depth = 0) const
	{
		// TODO: Implement
	}

	[[nodiscard]] Point3 max(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		// TODO: Implement
	}

	//
	// Center
	//

	/*!
	 * @return The center of the octree.
	 */
	[[nodiscard]] Point3 center() const { return Point3(0, 0, 0); }

	template <class NodeType>
	[[nodiscard]] Point3 center(NodeType const& node) const
	{
		if constexpr (std::is_same_v<NodeBV, NodeType>) {
			return node.center();
		} else {
			return center(node.code());
		}
	}

	[[nodiscard]] Point3 center(Code code) const
	{
		// TODO: Implement
	}

	[[nodiscard]] Point3 center(Key key) const
	{
		// TODO: Implement
	}

	[[nodiscard]] Point3 center(Point3 coord, depth_t depth = 0) const
	{
		// TODO: Implement
	}

	[[nodiscard]] Point3 center(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		// TODO: Implement
	}

	//
	// Bounding volume
	//

	/*!
	 * @return Minimum bounding volume convering the whole octree.
	 */
	[[nodiscard]] geometry::AAEBB boundingVolume() const
	{
		return geometry::AAEBB(center(), getNodeSize(depthLevels() - 1));
	}

	template <class NodeType>
	[[nodiscard]] geometry::AAEBB boundingVolume(NodeType const& node) const
	{
		if constexpr (std::is_same_v<NodeBV, NodeType>) {
			return node.boundingVolume();

		} else {
			return boundingVolume(node.code());
		}
	}

	[[nodiscard]] geometry::AAEBB boundingVolume(Code code) const
	{
		// TODO: Implement
	}

	[[nodiscard]] geometry::AAEBB boundingVolume(Key key) const
	{
		return boundingVolume(toCode(key));
	}

	[[nodiscard]] geometry::AAEBB boundingVolume(Point3 coord, depth_t depth = 0) const
	{
		return boundingVolume(toCode(coord, depth));
	}

	[[nodiscard]] geometry::AAEBB boundingVolume(coord_t x, coord_t y, coord_t z,
	                                             depth_t depth = 0) const
	{
		return boundingVolume(toCode(x, y, z, depth));
	}

	//
	// Is within
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
		auto max = nodeSize(depthLevels() - 1);
		auto min = -max;
		return min <= x && min <= y && min <= z && max >= x && max >= y && max >= z;
	}

	//
	// Memory
	//

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
	 * @brief This is lower bound memeory usage of an inner node.
	 *
	 * @note Additional data accessed by pointers inside an inner node are not counted.
	 *
	 * @return Memory usage of a single inner node.
	 */
	[[nodiscard]] constexpr std::size_t memoryInnerNode() const noexcept
	{
		return sizeof(InnerNode);
	}

	/*!
	 * @brief This is lower bound memeory usage of an inner leaf node.
	 *
	 * @note Additional data accessed by pointers inside an inner leaf node are not counted.
	 *
	 * @return Memory usage of a single inner leaf node.
	 */
	[[nodiscard]] constexpr std::size_t memoryInnerLeafNode() const noexcept
	{
		return sizeof(InnerNode);
	}

	/*!
	 * @brief This is lower bound memeory usage of a leaf node.
	 *
	 * @note Additional data accessed by pointers inside a leaf node are not counted.
	 *
	 * @return Memory usage of a single leaf node.
	 */
	[[nodiscard]] constexpr std::size_t memoryLeafNode() const noexcept
	{
		return sizeof(LeafNode);
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
	 * @return Number of nodes in the octree.
	 */
	[[nodiscard]] constexpr std::size_t size() const noexcept { return numNodes(); }

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
	 * @brief Lower bound memory usage for all nodes.
	 *
	 * @note Does not account for pointed to data inside nodes.
	 *
	 * @return Memory usage of the octree.
	 */
	[[nodiscard]] std::size_t memoryUsageAllocated() const noexcept
	{
		return (numInnerNodesAllocated() * memoryInnerNode()) +
		       (numInnerLeafNodesAllocated() * memoryInnerLeafNode()) +
		       (numLeafNodesAllocated() * memoryLeafNode());
	}

	/*!
	 * @return Number of nodes in the octree.
	 */
	[[nodiscard]] constexpr std::size_t sizeAllocated() const noexcept
	{
		return numNodesAllocated();
	}

	//
	// Pure leaf
	//

	/*!
	 * @brief Check if node is a pure leaf node (can never have children).
	 *
	 * @param node The node to check.
	 * @return Whether the node is a pure leaf node.
	 */
	[[nodiscard]] static constexpr bool pureLeaf(Node node) noexcept
	{
		return 0 == node.depth();
	}

	/*!
	 * @brief Check if the node corresponding to the code is a pure leaf node (can never
	 * have children).
	 *
	 * @note Only have to check if the depth of the code is 0.
	 *
	 * @param code The code of the node to check.
	 * @return Whether the node is a pure leaf node.
	 */
	[[nodiscard]] static constexpr bool pureLeaf(Code code) noexcept
	{
		return 0 == code.depth();
	}

	/*!
	 * @brief Check if the node corresponding to the key is a pure leaf node (can never have
	 * children).
	 *
	 * @note Only have to check if the depth of the key is 0.
	 *
	 * @param key The key of the node to check.
	 * @return Whether the node is a pure leaf node.
	 */
	[[nodiscard]] static constexpr bool pureLeaf(Key key) noexcept
	{
		return 0 == key.depth();
	}

	/*!
	 * @brief Check if the node corresponding to the coordinate at the specified depth is a
	 * pure leaf node (can never have children).
	 *
	 * @note Only have to check if the depth is 0.
	 *
	 * @param coord The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a pure leaf node.
	 */
	[[nodiscard]] static constexpr bool pureLeaf(Point3 coord, depth_t depth = 0) noexcept
	{
		return 0 == depth;
	}

	/*!
	 * @brief Check if the node corresponding to the coordinate at the specified depth is a
	 * pure leaf node (can never have children).
	 *
	 * @note Only have to check if the depth is 0.
	 *
	 * @param x,y,z The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a pure leaf node.
	 */
	[[nodiscard]] static constexpr bool pureLeaf(coord_t x, coord_t y, coord_t z,
	                                             depth_t depth = 0) noexcept
	{
		return 0 == depth;
	}

	//
	// Leaf
	//

	/*!
	 * @brief Check if node is a leaf node (has no children).
	 *
	 * @param node The node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] constexpr bool leaf(Node node) const
	{
		return leaf(getLeafNode(node), node.index());
	}

	/*!
	 * @brief Check if the node corresponding to the code is a leaf node (has no children).
	 *
	 * @param code The code of the node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] constexpr bool leaf(Code code) const
	{
		return leaf(getLeafNode(code), code.index());
	}

	/*!
	 * @brief Check if the node corresponding to the key is a leaf node (has no children).
	 *
	 * @param key The key of the node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] constexpr bool leaf(Key key) const
	{
		return pureLeaf(key) || leaf(toCode(key));
	}

	/*!
	 * @brief Check if the node corresponding to the coordinate is a leaf node (has no
	 * children).
	 *
	 * @param coord The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] constexpr bool leaf(Point3 coord, depth_t depth = 0) const
	{
		return pureLeaf(coord, depth) || leaf(toCode(coord, depth));
	}

	/*!
	 * @brief Check if the node corresponding to the coordinate is a leaf node (has no
	 * children).
	 *
	 * @param x,y,z The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] constexpr bool leaf(coord_t x, coord_t y, coord_t z,
	                                  depth_t depth = 0) const
	{
		return pureLeaf(x, y, z, depth) || leaf(toCode(x, y, z, depth));
	}

	//
	// Parent
	//

	/*!
	 * @brief Check if node is a parent, i.e., has children.
	 *
	 * @param node The node to check.
	 * @return Whether the node is a parent.
	 */
	[[nodiscard]] constexpr bool parent(Node node) const { return !leaf(node); }

	/*!
	 * @brief Check if the node corresponding to the code is a parent, i.e., has
	 * children.
	 *
	 * @param code The code of the node to check.
	 * @return Whether the node is a parent.
	 */
	[[nodiscard]] constexpr bool parent(Code code) const { return !leaf(code); }

	/*!
	 * @brief Check if the node corresponding to the key is a parent, i.e., has
	 * children.
	 *
	 * @param key The key of the node to check.
	 * @return Whether the node is a parent.
	 */
	[[nodiscard]] constexpr bool parent(Key key) const { return !leaf(key); }

	/*!
	 * @brief Check if the node corresponding to the coordinate is a parent, i.e., has
	 * children.
	 *
	 * @param coord The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a parent.
	 */
	[[nodiscard]] constexpr bool parent(Point3 coord, depth_t depth = 0) const
	{
		return !leaf(coord, depth);
	}

	/*!
	 * @brief Check if the node corresponding to the coordinate is a parent, i.e., has
	 * children.
	 *
	 * @param x,y,z The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a parent.
	 */
	[[nodiscard]] constexpr bool parent(coord_t x, coord_t y, coord_t z,
	                                    depth_t depth = 0) const
	{
		return !leaf(x, y, z, depth);
	}

	//
	// Code
	//

	/*!
	 * @brief Convert a key to a code.
	 *
	 * @param key The key to convert.
	 * @return The code corresponding to the key.
	 */
	[[nodiscard]] static constexpr Code code(Key key) noexcept { return Code(key); }

	/*!
	 * @brief Convert a coordinate at a specific depth to a code.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The code corresponding to the coordinate at the specific depth.
	 */
	[[nodiscard]] constexpr Code code(Point3 coord, depth_t depth = 0) const noexcept
	{
		return code(key(coord, depth));
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a code.
	 *
	 * @param x,y,z The coordinate.
	 * @param depth The depth.
	 * @return The code corresponding to the coordinate at the specific depth.
	 */
	[[nodiscard]] constexpr Code code(coord_t x, coord_t y, coord_t z,
	                                  depth_t depth = 0) const noexcept
	{
		return code(key(x, y, z, depth));
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a code with bounds check.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The code corresponding to the coordinate at the specific depth.
	 */
	[[nodiscard]] constexpr std::optional<Code> codeChecked(
	    Point3 coord, depth_t depth = 0) const noexcept
	{
		std::optional<Key::key_t> key = keyChecked(coord, depth);
		return key ? std::optional<Code>(code(*key)) : std::nullopt;
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a code with bounds check.
	 *
	 * @param x,y,z The coordinate.
	 * @param depth The depth.
	 * @return The code corresponding to the coordinate at the specific depth.
	 */
	[[nodiscard]] constexpr std::optional<Code> codeChecked(
	    coord_t x, coord_t y, coord_t z, depth_t depth = 0) const noexcept
	{
		return codeChecked(Point3(x, y, z), depth);
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
	[[nodiscard]] static constexpr Key key(Code code) noexcept { return code; }

	/*!
	 * @brief COnvert a coordinate at a specific depth to a key.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The corresponding key.
	 */
	constexpr Key key(Point3 coord, depth_t depth = 0) const noexcept
	{
		return Key(key(coord.x, depth), key(coord.y, depth), key(coord.z, depth), depth);
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a key.
	 *
	 * @param x,y,z The coordinate.
	 * @param depth The depth.
	 * @return The corresponding key.
	 */
	constexpr Key key(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const noexcept
	{
		return Key(key(x, depth), key(y, depth), key(z, depth), depth);
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a key with bounds check.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The corresponding key.
	 */
	constexpr std::optional<Key> keyChecked(Point3 coord, depth_t depth = 0) const noexcept
	{
		return depthLevels() >= depth && isWithin(coord)
		           ? std::optional<Key>(key(coord, depth))
		           : std::nullopt;
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a key with bounds check.
	 *
	 * @param x,y,z The coordinate.
	 * @param depth The depth.
	 * @return The corresponding key.
	 */
	constexpr std::optional<Key> keyChecked(coord_t x, coord_t y, coord_t z,
	                                        depth_t depth = 0) const noexcept
	{
		return keyChecked(Point3(x, y, z), depth);
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
	constexpr Point3 coord(Code code) const noexcept { return coord(key(code)); }

	/*!
	 * @brief Convert a key to a coordinate.
	 *
	 * @param key The key.
	 * @return The corresponding coordinate.
	 */
	constexpr Point3 coord(Key key) const noexcept
	{
		return Point3(coord(key[0], key.depth()), coord(key[1], key.depth()),
		              coord(key[2], key.depth()));
	}

	/*!
	 * @brief Convert a code to a coordinate with bounds check.
	 *
	 * @param Code The code.
	 * @return The corresponding coordinate.
	 */
	constexpr std::optional<Point3> coordChecked(Code code) const noexcept
	{
		return coordChecked(toKey(code));
	}

	/*!
	 * @brief Convert a key to a coordinate with bounds check.
	 *
	 * @param key The key.
	 * @return The corresponding coordinate.
	 */
	constexpr std::optional<Point3> coordChecked(Key key) const noexcept
	{
		return key.depth() <= depthLevels() ? std::optional<Point3>(coord(key))
		                                    : std::nullopt;
	}

	//
	// Root
	//

	/*!
	 * @brief Get the root node.
	 *
	 * @return The root node.
	 */
	constexpr Node rootNode() const
	{
		return Node(const_cast<InnerNode*>(&root()), rootCode());
	}

	/*!
	 * @brief Get the root node with bounding volume.
	 *
	 * @return The root node with bounding volume.
	 */
	constexpr NodeBV rootNodeBV() const { return NodeBV(rootNode(), boundingVolume()); }

	/*!
	 * @brief Get the code for the root node.
	 *
	 * @return The root node code.
	 */
	constexpr Code rootCode() const { return Code(rootIndex(), rootDepth()); }

	constexpr depth_t rootDepth() const { return depthLevels(); }

	//
	// Node
	//

	// TODO: Move here

	//
	// NodeBV
	//

	// TODO: Move here

	//
	// Create node
	//

	Node createNode(Code code)
	{
		// FIXME: Handle wrong codes

		InnerNode* node = &root();
		auto const min_depth = std::max(depth_t(1), code.depth());
		for (depth_t depth = rootDepth(); min_depth < depth; --depth) {
			auto const index = code.index(depth);
			if (leaf(*node, index)) {
				createInnerChildren(*node, index, depth);
			}
			node = &innerChild(*node, index, code.index(depth - 1));
		}

		if (0 == code.depth()) {
			auto const index = code.index();
			if (leaf(*node, index)) {
				createLeafChildren(*node, index);
			}
			return Node(&leafChild(*node, code.index()), code);
		} else {
			return Node(node, code);
		}
	}

	Node createNode(Key key) { return createNode(code(key)); }

	Node createNode(Point3 coord, depth_t depth = 0)
	{
		return createNode(code(coord, depth));
	}

	Node createNode(coord_t x, coord_t y, coord_t z, depth_t depth = 0)
	{
		return createNode(code(x, y, z, depth));
	}

	//
	// Create bv node
	//

	NodeBV createNodeBV(Code code) { return nodeBV(createNode(code)); }

	NodeBV createNodeBV(Key key) { return createNodeBV(code(key)); }

	NodeBV createNodeBV(Point3 coord, depth_t depth = 0)
	{
		return createNodeBV(code(coord, depth));
	}

	NodeBV createNodeBV(coord_t x, coord_t y, coord_t z, depth_t depth = 0)
	{
		return createNodeBV(code(x, y, z, depth));
	}

	//
	// Get node
	//

	/*!
	 * @brief Get the corresponding node for the code.
	 *
	 * @param code The code for the node.
	 * @return The node.
	 */
	Node node(Code code) const
	{
		auto const& [node, depth] = nodeAndDepth(code);
		return Node(const_cast<LeafNode*>(&node), code.toDepth(depth));
	}

	/*!
	 * @brief Get the corresponding node for the key.
	 *
	 * @param key The key for the node.
	 * @return The node.
	 */
	Node node(Key key) const { return node(code(key)); }

	/*!
	 * @brief Get the corresponding node for the coordinate at a specific depth.
	 *
	 * @param coord The coordinate for the node.
	 * @param depth The depth.
	 * @return The node.
	 */
	Node node(Point3 coord, depth_t depth = 0) const { return node(code(coord, depth)); }

	/*!
	 * @brief Get the corresponding node for the coordinate at a specific depth.
	 *
	 * @param x,y,z The coordinate for the node.
	 * @param depth The depth.
	 * @return The node.
	 */
	Node node(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return node(code(x, y, z, depth));
	}

	/*!
	 * @brief Get the corresponding node for the code with bounds check.
	 *
	 * @param code The code for the node.
	 * @return The node.
	 */
	std::optional<Node> nodeChecked(Code code) const
	{
		return code.depth() <= depthLevels() ? std::optional<Node>(node(code)) : std::nullopt;
	}

	/*!
	 * @brief Get the corresponding node for the key with bounds check.
	 *
	 * @param key The key for the node.
	 * @return The node.
	 */
	std::optional<Node> nodeChecked(Key key) const { return nodeChecked(code(key)); }

	/*!
	 * @brief Get the corresponding node for the coordinate at a specific depth with bounds
	 * check.
	 *
	 * @param coord The coordinate for the node.
	 * @param depth The depth.
	 * @return The node.
	 */
	std::optional<Node> nodeChecked(Point3 coord, depth_t depth = 0) const
	{
		if (auto code = codeChecked(coord, depth)) {
			return std::optional<Node>(*code);
		} else {
			return std::nullopt;
		}
	}

	/*!
	 * @brief Get the corresponding node for the coordinate at a specific depth with bounds
	 * check.
	 *
	 * @param x,y,z The coordinate for the node.
	 * @param depth The depth.
	 * @return The node.
	 */
	std::optional<Node> nodeChecked(coord_t x, coord_t y, coord_t z,
	                                depth_t depth = 0) const
	{
		return nodeChecked(Point3(x, y, z), depth);
	}

	//
	// Get bounding volume node
	//

	NodeBV nodeBV(Node node) const { return NodeBV(node, boundingVolume(node)); }

	NodeBV nodeBV(Code code) const { return nodeBV(node(code)); }

	NodeBV nodeBV(Key key) const { return nodeBV(code(key)); }

	NodeBV nodeBV(Point3 coord, depth_t depth = 0) const
	{
		return nodeBV(code(coord, depth));
	}

	NodeBV nodeBV(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return nodeBV(code(x, y, z, depth));
	}

	/*!
	 * @brief Get the corresponding node for the code with bounds check.
	 *
	 * @param code The code for the node.
	 * @return The node.
	 */
	std::optional<NodeBV> nodeBVChecked(Code code) const
	{
		return code.depth() <= depthLevels() ? std::optional<NodeBV>(nodeBV(code))
		                                     : std::nullopt;
	}

	/*!
	 * @brief Get the corresponding node for the key with bounds check.
	 *
	 * @param key The key for the node.
	 * @return The node.
	 */
	std::optional<NodeBV> nodeBVChecked(Key key) const { return nodeBVChecked(code(key)); }

	/*!
	 * @brief Get the corresponding node for the coordinate at a specific depth with bounds
	 * check.
	 *
	 * @param coord The coordinate for the node.
	 * @param depth The depth.
	 * @return The node.
	 */
	std::optional<NodeBV> nodeBVChecked(Point3 coord, depth_t depth = 0) const
	{
		if (auto code = codeChecked(coord, depth)) {
			return std::optional<NodeBV>(*code);
		} else {
			return std::nullopt;
		}
	}

	/*!
	 * @brief Get the corresponding node for the coordinate at a specific depth with bounds
	 * check.
	 *
	 * @param x,y,z The coordinate for the node.
	 * @param depth The depth.
	 * @return The node.
	 */
	std::optional<NodeBV> nodeBVChecked(coord_t x, coord_t y, coord_t z,
	                                    depth_t depth = 0) const
	{
		return nodeBVChecked(Point3(x, y, z), depth);
	}

	//
	// Child
	//

	// TODO: Continue from here

	[[nodiscard]] Node child(Node node, index_t child_index) const
	{
		auto& child = children(innerNode(node), node.index(), node.depth() - 1);
		return Node(child, node.code().child(child_index));
	}

	[[nodiscard]] NodeBV child(NodeBV const& node, index_t child_index) const
	{
		auto& child = children(innerNode(node), node.index(), node.depth() - 1);

		auto const child_half_size = node.halfSize() / 2;
		geometry::AAEBB child_aaebb(childCenter(node.center(), child_half_size, child_index),
		                            child_half_size);

		return NodeBV(&child, node.code().child(child_index), child_aaebb);
	}

	template <class Node>
	[[nodiscard]] Node childChecked(Node const& node, index_t child_index) const
	{
		if (!parent(node)) {
			throw std::out_of_range("Node has no children");
		} else if (7 < child_index) {
			throw std::out_of_range("child_index out of range");
		}
		return child(node, child_index);
	}

	//
	// Sibling
	//

	[[nodiscard]] Node sibling(Node node, index_t sibling_index) const
	{
		return Node(node.data(), node.code().sibling(sibling_index));
	}

	[[nodiscard]] NodeBV sibling(NodeBV const& node, index_t sibling_index) const
	{
		geometry::AAEBB aaebb(
		    siblingCenter(node.center(), node.halfSize(), node.index(), sibling_index),
		    node.halfSize());
		return NodeBV(node.data(), node.code().sibling(sibling_index), aaebb);
	}

	template <class Node>
	[[nodiscard]] Node siblingChecked(Node const& node, index_t sibling_index) const
	{
		if (!root(node)) {
			throw std::out_of_range("Node has no siblings");
		} else if (7 < sibling_index) {
			throw std::out_of_range("sibling_index out of range");
		}
		return sibling(node, sibling_index);
	}

	//
	// Modified
	//

	[[nodiscard]] constexpr bool modified(Node node) const
	{
		return modified(leafNode(node));
	}

	[[nodiscard]] constexpr bool modified(Code code) const
	{
		return modified(leafNode(code));
	}

	[[nodiscard]] constexpr bool modified(Key key) const { return modified(code(key)); }

	[[nodiscard]] constexpr bool modified(Point3 coord, depth_t depth = 0) const
	{
		return mdified(code(coord, depth));
	}

	[[nodiscard]] constexpr bool modified(coord_t x, coord_t y, coord_t z,
	                                      depth_t depth = 0) const
	{
		return modified(code(x, y, z, depth));
	}

	//
	// Is root
	//

	[[nodiscard]] bool root(Node node) const { return root(node.code()); }

	[[nodiscard]] bool root(Code code) const { return rootCode() == code; }

	[[nodiscard]] bool root(Key key) const { return root(code(key)); }

	[[nodiscard]] bool root(Point3 coord, depth_t depth = 0) const
	{
		return root(code(coord, depth));
	}

	[[nodiscard]] bool root(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return root(code(x, y, z, depth));
	}

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
			        &derived(), getRootNodeBV(), std::forward<Predicates>(predicates)));
		} else {
			return const_query_iterator(
			    new Iterator<Node, Derived, Node, std::decay_t<Predicates>>(
			        &derived(), getRootNode(), std::forward<Predicates>(predicates)));
		}
	}

	template <class Predicates>
	const_query_iterator beginQuery(Node node, Predicates&& predicates) const
	{
		if constexpr (predicate::contains_spatial_predicate_v<std::decay_t<Predicates>>) {
			return const_query_iterator(
			    new Iterator<Node, Derived, NodeBV, std::decay_t<Predicates>>(
			        &derived(), getNodeBV(node), std::forward<Predicates>(predicates)));
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
		return const_query_iterator(new Iterator<Node, Derived>(getRootNode()));
	}

	template <class Predicates>
	const_bounding_volume_query_iterator beginQueryBV(Predicates&& predicates) const
	{
		return const_bounding_volume_query_iterator(
		    new Iterator<NodeBV, Derived, NodeBV, Predicates>(
		        &derived(), getRootNodeBV(), std::forward<Predicates>(predicates)));
	}

	template <class Predicates>
	const_bounding_volume_query_iterator beginQueryBV(Node node,
	                                                  Predicates&& predicates) const
	{
		return const_bounding_volume_query_iterator(
		    new Iterator<NodeBV, Derived, NodeBV, Predicates>(
		        &derived(), getNodeBV(node), std::forward<Predicates>(predicates)));
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
		    new Iterator<NodeBV, Derived, NodeBV>(getRootNodeBV()));
	}

	template <class Geometry, class Predicates>
	const_query_nearest_iterator beginQueryNearest(Geometry&& geometry,
	                                               Predicates&& predicates,
	                                               double epsilon = 0.0) const
	{
		return const_query_nearest_iterator(
		    new NearestIterator(&derived(), getRootNodeBV(), std::forward<Geometry>(geometry),
		                        std::forward<Predicates>(predicates), epsilon));
	}

	template <class Geometry, class Predicates>
	const_query_nearest_iterator beginQueryNearest(Node node, Geometry&& geometry,
	                                               Predicates&& predicates,
	                                               double epsilon = 0.0) const
	{
		return const_query_nearest_iterator(
		    new NearestIterator(&derived(), getNodeBV(node), std::forward<Geometry>(geometry),
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

	//
	// Traverse
	//

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

	//
	// Propagate
	//

	void propagate(bool keep_modified = false, depth_t max_depth = maxDepthLevels())
	{
		propagate(root(), rootIndex(), rootDepth(), keep_modified, max_depth);
	}

	void propagate(Node node, bool keep_modified = false,
	               depth_t max_depth = maxDepthLevels())
	{
		if (0 == node.depth()) {
			// TODO: What to do?
		} else {
			propagate(innerNode(node), node.index() node.depth(), keep_modified, max_depth);
		}
	}

	void propagate(Code code, bool keep_modified = false,
	               depth_t max_depth = maxDepthLevels())
	{
		if (0 == code.depth()) {
			// TODO: What to do?
		} else {
			propagate(innerNode(code), code.index() code.depth(), keep_modified, max_depth);
		}
	}

	void propagate(Key key, bool keep_modified = false,
	               depth_t max_depth = maxDepthLevels())
	{
		propagate(code(key), keep_modified, max_depth);
	}

	void propagate(Point3 coord, depth_t depth = 0, bool keep_modified = false,
	               depth_t max_depth = maxDepthLevels())
	{
		propagate(code(coord, depth), depth, keep_modified, max_depth);
	}

	void propagate(coord_t x, coord_t y, coord_t z, depth_t depth = 0,
	               bool keep_modified = false, depth_t max_depth = maxDepthLevels())
	{
		propagate(code(x, y, z, depth), keep_modified, max_depth);
	}

	//
	// Set modified
	//

	void setModified(depth_t min_depth = 0)
	{
		if (depthLevels() >= min_depth) {
			setModifiedRecurs(root(), rootDepth(), min_depth);
		}
	}

	void setModified(Node node, depth_t min_depth = 0)
	{
		if (node.depth() >= min_depth) {
			if (0 == node.depth()) {
				setModified(leafNode(node), true);
			} else {
				setModifiedRecurs(innerNode(node), node.depth(), min_depth);
			}
			setModifiedParentsRecurs(root(), rootDepth(), node.code());
		} else {
			setModifiedParentsRecurs(root(), rootDepth(), node.code().toDepth(min_depth));
		}
	}

	void setModified(Code code, depth_t min_depth = 0)
	{
		if (code.depth() >= min_depth) {
			if (0 == code.depth()) {
				setModified(leafNode(code), true);
			} else {
				setModifiedRecurs(getInnerNode(code), code.depth(), min_depth);
			}
			setModifiedParentsRecurs(getRoot(), rootDepth(), code);
		} else {
			setModifiedParentsRecurs(getRoot(), rootDepth(), code.toDepth(min_depth));
		}
	}

	void setModified(Key key, depth_t min_depth = 0) { setModified(code(key), min_depth); }

	void setModified(Point3 coord, depth_t depth = 0, depth_t min_depth = 0)
	{
		setModified(code(coord, depth), min_depth);
	}

	void setModified(coord_t x, coord_t y, coord_t z, depth_t depth = 0,
	                 depth_t min_depth = 0)
	{
		setModified(code(x, y, z, depth), min_depth);
	}

	//
	// Reset modified
	//

	void resetModified(depth_t max_depth = maxDepthLevels())
	{
		resetModifiedRecurs(root(), rootDepth(), max_depth);
	}

	void resetModified(Node node, depth_t max_depth = maxDepthLevels())
	{
		if (isLeaf(node)) {
			setModified(leafNode(node), false);
		} else {
			resetModifiedRecurs(innerNode(node), node.depth(), max_depth);
		}
	}

	void resetModified(Code code, depth_t max_depth = maxDepthLevels())
	{
		if (0 == code.depth()) {
			setModified(leafNode(code), false);
		} else {
			resetModifiedRecurs(innerNode(code), code.depth(), max_depth);
		}
	}

	void resetModified(Key key, depth_t max_depth = maxDepthLevels())
	{
		resetModified(code(key), max_depth);
	}

	void resetModified(Point3 coord, depth_t depth = 0,
	                   depth_t max_depth = maxDepthLevels())
	{
		resetModified(code(coord, depth), max_depth);
	}

	void resetModified(coord_t x, coord_t y, coord_t z, depth_t depth = 0,
	                   depth_t max_depth = maxDepthLevels())
	{
		resetModified(code(x, y, z, depth), max_depth);
	}

	//
	// Input/output (read/write)
	//

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
		writeHeader(out, fileOptions(compress));
		writeData(out, std::forward<Predicates>(predicates), min_depth, compress,
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
		writeHeader(out, getFileOptions(compress));
		writeModifiedData<true>(out, compress, compression_acceleration_level,
		                        compression_level);
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
		writeHeader(out, fileOptions(compress));
		writeModifiedData<false>(out, compress, compression_acceleration_level,
		                         compression_level);
	}

 protected:
	//
	// Constructors
	//

	OctreeBase(resolution_t resolution = 0.1, depth_t depth_levels = 16,
	           bool automatic_pruning = true)
	    : automatic_pruning_enabled_(automatic_pruning)
	{
		setResolutionAndDepthLevels(resolution, depth_levels);

		init();
	}

	OctreeBase(OctreeBase const& other)
	    : depth_levels_(other.depth_levels_),
	      max_value_(other.max_value_),
	      resolution_(other.resolution_),
	      resolution_factor_(other.resolution_factor_),
	      automatic_pruning_enabled_(other.automatic_pruning_enabled_)
	{
		init();
	}

	OctreeBase(OctreeBase&& other)
	    : depth_levels_(std::move(other.depth_levels_)),
	      max_value_(std::move(other.max_value_)),
	      root_(std::move(other.root_)),
	      resolution_(std::move(other.resolution_)),
	      resolution_factor_(std::move(other.resolution_factor_)),
	      automatic_pruning_enabled_(std::move(other.automatic_pruning_enabled_))
	{
		num_inner_nodes_.store(other.num_inner_nodes_);
		num_inner_leaf_nodes_.store(other.num_inner_leaf_nodes_);
		num_leaf_nodes_.store(other.num_leaf_nodes_);

		num_allocated_inner_nodes_.store(other.num_allocated_inner_nodes_);
		num_allocated_inner_leaf_nodes_.store(other.num_allocated_inner_leaf_nodes_);
		num_allocated_leaf_nodes_.store(other.num_allocated_leaf_nodes_);

		init();
	}

	//
	// Assignment operator
	//

	OctreeBase& operator=(OctreeBase const& rhs)
	{
		// TODO: Should this clear?
		clear(rhs.resolution(), rhs.depthLevels());

		depth_levels_ = rhs.depth_levels_;
		max_value_ = rhs.max_value_;
		resolution_ = rhs.resolution_;
		resolution_factor_ = rhs.resolution_factor_;
		automatic_pruning_enabled_ = rhs.automatic_pruning_enabled_;
		return *this;
	}

	OctreeBase& operator=(OctreeBase&& rhs)
	{
		// TODO: Should this clear?
		clear(rhs.resolution(), rhs.depthLevels(), true);

		depth_levels_ = std::move(rhs.depth_levels_);
		max_value_ = std::move(rhs.max_value_);
		root_ = std::move(rhs.root_);
		resolution_ = std::move(rhs.resolution_);
		resolution_factor_ = std::move(rhs.resolution_factor_);
		automatic_pruning_enabled_ = std::move(rhs.automatic_pruning_enabled_);
		num_inner_nodes_.store(rhs.num_inner_nodes_);
		num_inner_leaf_nodes_.store(rhs.num_inner_leaf_nodes_);
		num_leaf_nodes_.store(rhs.num_leaf_nodes_);
		num_allocated_inner_nodes_.store(rhs.num_allocated_inner_nodes_);
		num_allocated_inner_leaf_nodes_.store(rhs.num_allocated_inner_leaf_nodes_);
		num_allocated_leaf_nodes_.store(rhs.num_allocated_leaf_nodes_);
		return *this;
	}

	//
	// Destructor
	//

	~OctreeBase() { deleteChildren(getRoot(), depthLevels(), true); }

	//
	// Derived
	//

	constexpr Derived& derived() { return *static_cast<Derived*>(this); }

	constexpr Derived const& derived() const { return *static_cast<Derived const*>(this); }

	//
	// Init
	//

	void init()
	{
		size_t i = 0;
		for (auto& a_l : children_locks_) {
			a_l.clear();
		}
	}

	//
	// Set resolution and depth levels
	//

	void setResolutionAndDepthLevels(resolution_t const resolution,
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

		std::generate(std::begin(resolution_), std::end(resolution_),
		              [n = resolution]() mutable {
			              auto const c = n;
			              n *= 2.0;
			              return c;
		              });

		std::transform(std::begin(resolution_), std::end(resolution_),
		               std::begin(resolution_factor_), [](auto n) { return 1.0 / n; });
	}

	//
	// Initilize root
	//

	void initRoot()
	{
		setIsLeaf(getRoot(), true);
		setModified(getRoot(), false);
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
	constexpr Key::key_t toKey(coord_t coord, depth_t depth = 0) const noexcept
	{
		Key::key_t val = std::floor(resolution_factor_[0] * coord);
		return ((val + max_value_) >> depth) << depth;
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a key with bounds check.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The corresponding key.
	 */
	constexpr std::optional<Key::key_t> toKeyChecked(coord_t coord,
	                                                 depth_t depth = 0) const noexcept
	{
		auto min = -getNodeSize(depthLevels() - 1);
		auto max = -min;
		return min <= coord && max >= coord ? std::optional<Key::key_t>(toKey(coord, depth))
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
	constexpr coord_t toCoord(Key::key_t key, depth_t depth = 0) const noexcept
	{
		constexpr auto sub64 = std::minus<int_fast64_t>{};
		return depth_levels_ == depth
		           ? 0
		           : (std::floor(sub64(key, max_value_) / static_cast<coord_t>(1U << depth)) +
		              coord_t(0.5)) *
		                 getNodeSize(depth);

		// // FIXME: Check if correct
		// return depth_levels_ == depth
		//            ? 0
		//            : (((key >> depth) << depth) - max_value_) * resolution_;
	}

	//
	// Traverse
	//

	template <class UnaryFunction, class NodeType>
	void traverseRecurs(NodeType const& node, UnaryFunction f) const
	{
		if (f(node) || isLeaf(node)) {
			return;
		}

		auto child = getNodeChild(node, 0);
		for (std::size_t i = 0; i != 8; ++i) {
			traverseRecurs(getNodeSibling(child, i), f);
		}
	}

	//
	// Apply
	//

	template <class UnaryFunction, class UnaryFunction2,
	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction>>,
	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction2>>>
	void apply(Node node, UnaryFunction f, UnaryFunction2 f2, bool const propagate)
	{
		if (isLeaf(node)) {
			f(getLeafNode(node), node.index());
		} else {
			applyAllRecurs(getInnerNode(node), node.depth(), f, f2);
		}

		if (!isModified(node)) {
			// Update all parents
			setModifiedParents(node);

			setModified(getLeafNode(node), true);
		}

		if (propagate) {
			updateModifiedNodes();
		}
	}

	template <class UnaryFunction, UnaryFunction2,
	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction>>,
	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction2>>>
	void apply(Code code, UnaryFunction f, UnaryFunction2 f2, bool const propagate)
	{
		// FIXME: Should this be here?
		if (code.depth() > depthLevels()) {
			return;
		}

		applyRecurs(getRoot(), depthLevels(), code, f, f2);

		if (propagate) {
			updateModifiedNodes();
		}
	}

	template <class UnaryFunction, class UnaryFunction2,
	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction>>,
	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction2>>>
	void applyRecurs(InnerNode& node, depth_t depth, Code code, UnaryFunction f,
	                 UnaryFunction2 f2)
	{
		if (code.depth() == depth) {
			if (isLeaf(node)) {
				f(static_cast<LeafNode&>(node));
			} else {
				applyAllRecurs(node, depth, f, f2);
			}
		} else if (1 == depth) {
			createLeafChildren(node);
			LeafNode& child = getLeafChild(node, code.indexAtDepth(0));
			f(child);
			setModified(child, true);
		} else {
			createInnerChildren(node, depth);
			applyRecurs(getInnerChild(node, code.indexAtDepth(depth - 1)), depth - 1, code, f,
			            f2);
		}

		setModified(node, true);
	}

	template <class UnaryFunction, class UnaryFunction2,
	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction>>,
	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction2>>>
	void applyAllRecurs(InnerNode& node, depth_t depth, UnaryFunction f, UnaryFunction2 f2)
	{
		if (1 == depth) {
			auto& children = getLeafChildren(node);
			f2(children);
			setModified(children, true);
		} else {
			auto& children = getInnerChildren(node);
			if (isLeaf(children)) {
				f2(children);
			} else {
				for (InnerNode& child : getInnerChildren(node)) {
					if (isLeaf(child)) {
						f(static_cast<LeafNode&>(child));
					} else {
						applyAllRecurs(child, depth - 1, f, f2);
					}
				}
			}
			setModified(children, true);
		}
	}

	//
	// Get root
	//

	constexpr InnerNode const& getRoot() const noexcept { return root_; }

	constexpr InnerNode& getRoot() noexcept { return root_; }

	//
	// Get root index
	//

	constexpr index_t getRootIndex() const noexcept { return 0; }

	//
	// Get root depth
	//

	constexpr depth_t getRootDepth() const noexcept { return depthLevels(); }

	//
	// Get node
	//

	constexpr LeafNode const& getLeafNode(Node node) const
	{
		return *static_cast<LeafNode*>(node.data());
	}

	constexpr LeafNode& getLeafNode(Node node)
	{
		return *static_cast<LeafNode*>(node.data());
	}

	LeafNode const& getLeafNode(Code code) const
	{
		LeafNode const* node = &getRoot();
		depth_t depth = depthLevels();
		while (code.depth() != depth && isParent(*node)) {
			--depth;
			node = 0 == depth ? &getLeafChild(static_cast<InnerNode const&>(*node),
			                                  code.indexAtDepth(depth))
			                  : &getInnerChild(static_cast<InnerNode const&>(*node),
			                                   code.indexAtDepth(depth));
		}
		return *node;
	}

	LeafNode& getLeafNode(Code code)
	{
		LeafNode* node = &getRoot();
		depth_t depth = depthLevels();
		while (code.depth() != depth && isParent(*node)) {
			--depth;
			node = 0 == depth ? &getLeafChild(static_cast<InnerNode const&>(*node),
			                                  code.indexAtDepth(depth))
			                  : &getInnerChild(static_cast<InnerNode const&>(*node),
			                                   code.indexAtDepth(depth));
		}
		return *node;
	}

	constexpr InnerNode const& getInnerNode(Node node) const
	{
		return *static_cast<InnerNode*>(node.data());
	}

	constexpr InnerNode& getInnerNode(Node& node)
	{
		return *static_cast<InnerNode*>(node.data());
	}

	InnerNode const& getInnerNode(Code code) const
	{
		// TODO: Implement correctly
		return static_cast<InnerNode const&>(getLeafNode(code));
	}

	InnerNode& getInnerNode(Code code)
	{
		// TODO: Implement correctly
		return static_cast<InnerNode&>(getLeafNode(code));
	}

	std::pair<LeafNode const&, depth_t> getNodeAndDepth(Code code) const
	{
		LeafNode const* node = &getRoot();
		depth_t depth = depthLevels();
		while (code.depth() != depth && isParent(*node)) {
			--depth;
			node = 0 == depth ? &getLeafChild(static_cast<InnerNode const&>(*node),
			                                  code.indexAtDepth(depth))
			                  : &getInnerChild(static_cast<InnerNode const&>(*node),
			                                   code.indexAtDepth(depth));
		}
		return {*node, depth};
	}

	std::pair<LeafNode&, depth_t> getNodeAndDepth(Code code)
	{
		LeafNode* node = &getRoot();
		depth_t depth = depthLevels();
		while (code.depth() != depth && isParent(*node)) {
			--depth;
			node = 0 == depth ? &getLeafChild(static_cast<InnerNode const&>(*node),
			                                  code.indexAtDepth(depth))
			                  : &getInnerChild(static_cast<InnerNode const&>(*node),
			                                   code.indexAtDepth(depth));
		}
		return {*node, depth};
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

	//
	// Create children
	//

	void createLeafChildren(InnerNode& node)
	{
		if constexpr (!LockLess) {
			if (!lockIfLeaf(node, 0)) {
				return;
			}
		}

		// TODO: Implement

		setIsLeaf(node, false);
		if constexpr (!LockLess) {
			unlockChildren(0);
		}
	}

	void createLeafChildren(InnerNode& node, index_t index)
	{
		if constexpr (!LockLess) {
			if (!lockIfLeaf(node, 0)) {
				return;
			}
		}

		// TODO: Implement

		setIsLeaf(node, index, false);
		if constexpr (!LockLess) {
			unlockChildren(0);
		}
	}

	void createInnerChildren(InnerNode& node, depth_t depth)
	{
		if constexpr (!LockLess) {
			if (!lockIfLeaf(node, depth)) {
				return;
			}
		}

		// TODO: Implement

		setIsLeaf(node, false);
		if constexpr (!LockLess) {
			unlockChildren(depth);
		}
	}

	void createInnerChildren(InnerNode& node, index_t index, depth_t depth)
	{
		if constexpr (!LockLess) {
			if (!lockIfLeaf(node, depth)) {
				return;
			}
		}

		// TODO: Implement

		setIsLeaf(node, index, false);
		if constexpr (!LockLess) {
			unlockChildren(depth);
		}
	}

	void createLeafChildren(InnerNode& node)
	{
		if constexpr (!LockLess) {
			if (!lockIfLeaf(node, 0)) {
				return;
			}
		}

		if constexpr (ReuseNodes) {
			if constexpr (!LockLess) {
				if (lockIfNonEmptyLeaves()) {
					// Take existing children
					node.leaf_children = free_leaf_blocks_.top();
					free_leaf_blocks_.pop();

					unlockLeaves();
				} else {
					// Allocate children
					node.leaf_children = new LeafNodeBlock();
					num_allocated_leaf_nodes_ += 8;
					num_allocated_inner_leaf_nodes_ -= 1;
					num_allocated_inner_nodes_ += 1;
				}
			} else {
				if (!free_leaf_blocks_.empty()) {
					// Take existing children
					node.leaf_children = free_leaf_blocks_.top();
					free_leaf_blocks_.pop();
				} else {
					// Allocate children
					node.leaf_children = new LeafNodeBlock();
					num_allocated_leaf_nodes_ += 8;
					num_allocated_inner_leaf_nodes_ -= 1;
					num_allocated_inner_nodes_ += 1;
				}
			}
		} else {
			if (!node.leaf_children) {
				// Allocate children
				node.leaf_children = new LeafNodeBlock();
				num_allocated_leaf_nodes_ += 8;
				num_allocated_inner_leaf_nodes_ -= 1;
				num_allocated_inner_nodes_ += 1;
			}
		}

		num_leaf_nodes_ += 8;
		num_inner_leaf_nodes_ -= 1;
		num_inner_nodes_ += 1;

		getLeafChildren(node).fill(static_cast<LeafNode&>(node));

		setIsLeaf(node, false);
		if constexpr (!LockLess) {
			unlockChildren(0);
		}
	}

	void createInnerChildren(InnerNode& node, depth_t depth)
	{
		if constexpr (!LockLess) {
			if (!lockIfLeaf(node, depth)) {
				return;
			}
		}

		if constexpr (ReuseNodes) {
			if constexpr (!LockLess) {
				if (lockIfNonEmptyInner()) {
					// Take existing children
					node.inner_children = free_inner_blocks_.top();
					free_inner_blocks_.pop();

					unlockInner();
				} else {
					// Allocate children
					node.inner_children = new InnerNodeBlock();
					// Get 8 new and 1 is made into a inner node
					num_allocated_inner_leaf_nodes_ += 7;
					num_allocated_inner_nodes_ += 1;
				}
			} else {
				if (!free_inner_blocks_.empty()) {
					// Take existing children
					node.inner_children = free_inner_blocks_.top();
					free_inner_blocks_.pop();
				} else {
					// Allocate children
					node.inner_children = new InnerNodeBlock();
					// Get 8 new and 1 is made into a inner node
					num_allocated_inner_leaf_nodes_ += 7;
					num_allocated_inner_nodes_ += 1;
				}
			}
		} else {
			if (!node.inner_children) {
				// Allocate children
				node.inner_children = new InnerNodeBlock();
				// Get 8 new and 1 is made into a inner node
				num_allocated_inner_leaf_nodes_ += 7;
				num_allocated_inner_nodes_ += 1;
			}
		}

		num_inner_leaf_nodes_ += 7;
		num_inner_nodes_ += 1;

		for (InnerNode& child : getInnerChildren(node)) {
			static_cast<LeafNode&>(child) = static_cast<LeafNode&>(node);
		}

		setIsLeaf(node, false);
		if constexpr (!LockLess) {
			unlockChildren(depth);
		}
	}

	//
	// Delete children
	//

	void deleteLeafChildren(InnerNode& node, bool manual_pruning = false)
	{
		// TODO: Implement
	}

	void deleteLeafChildren(InnerNode& node, index_t const index,
	                        bool manual_pruning = false)
	{
		// TODO: Implement
	}

	void deleteChildren(InnerNode& node, depth_t const depth, bool manual_pruning = false)
	{
		// TODO: Implement
	}

	void deleteChildren(InnerNode& node, index_t const index, depth_t const depth,
	                    bool manual_pruning = false)
	{
		// TODO: Implement
	}

	void deleteLeafChildren(InnerNode& node, bool manual_pruning = false)
	{
		// FIXME: Do I need to take lock here?

		if (!isLeaf(node)) {
			setIsLeaf(node, true);
			num_leaf_nodes_ -= 8;
			num_inner_leaf_nodes_ += 1;
			num_inner_nodes_ -= 1;
		}

		if (nullptr == node.leaf_children) {
			return;
		}

		if constexpr (ReuseNodes) {
			if (!manual_pruning && !automaticPruning()) {
				if constexpr (!LockLess) {
					lockLeaves();
					free_leaf_blocks_.push(&getLeafChildren(node));
					unlockLeaves();
				} else {
					free_leaf_blocks_.push(&getLeafChildren(node));
				}
			} else {
				delete &getLeafChildren(node);
				num_allocated_leaf_nodes_ -= 8;
				num_allocated_inner_leaf_nodes_ += 1;
				num_allocated_inner_nodes_ -= 1;
			}

		} else {
			if (!manual_pruning && !automaticPruning()) {
				return;
			}

			delete &getLeafChildren(node);
			num_allocated_leaf_nodes_ -= 8;
			num_allocated_inner_leaf_nodes_ += 1;
			num_allocated_inner_nodes_ -= 1;
		}

		node.leaf_children = nullptr;
	}

	void deleteChildren(InnerNode& node, depth_t depth, bool manual_pruning = false)
	{
		// FIXME: Do I need to take lock here?
		if (1 == depth) {
			deleteLeafChildren(node, manual_pruning);
			return;
		}

		if (!isLeaf(node)) {
			setIsLeaf(node, true);
			num_inner_leaf_nodes_ -= 7;
			num_inner_nodes_ -= 1;

			// Need to call it here such that the number of nodes are correctly calculated
			for (InnerNode& child : getInnerChildren(node)) {
				deleteChildren(child, depth - 1, true);
			}
		}

		if (nullptr == node.inner_children) {
			return;
		}

		if constexpr (ReuseNodes) {
			if (!manual_pruning && !automaticPruning()) {
				if constexpr (!LockLess) {
					lockInner();
					free_inner_blocks_.push(&getInnerChildren(node));
					unlockInner();
				} else {
					free_inner_blocks_.push(&getInnerChildren(node));
				}
			} else {
				delete &getInnerChildren(node);
				// Remove 8 and 1 inner node is made into a inner leaf node
				num_allocated_inner_leaf_nodes_ -= 7;
				num_allocated_inner_nodes_ -= 1;
			}

		} else {
			if (!manual_pruning && !automaticPruning()) {
				return;
			}

			delete &getInnerChildren(node);
			// Remove 8 and 1 inner node is made into a inner leaf node
			num_allocated_inner_leaf_nodes_ -= 7;
			num_allocated_inner_nodes_ -= 1;
		}

		node.inner_children = nullptr;
	}

	//
	// Get children
	//

	static constexpr LeafNodeBlock& leafChildren(InnerNode const& node)
	{
		return *node.leaf_children;
	}

	static constexpr InnerNodeBlock& innerChildren(InnerNode const& node)
	{
		return *node.inner_children;
	}

	static constexpr LeafNode& leafChildren(InnerNode const& node, index_t index)
	{
		return getLeafChildrenBlock(node)[index];
	}

	static constexpr InnerNode& innerChildren(InnerNode const& node, index_t index)
	{
		return getInnerChildrenBlock(node)[index];
	}

	static constexpr LeafNode& children(InnerNode const& node, index_t index, depth_t depth)
	{
		return 1 == depth ? getLeafChildren(node, index) : getInnerChildren(node, index);
	}

	//
	// Get child center
	//

	static constexpr Point3 childCenter(Point3 parent_center, resolution_t child_half_size,
	                                    index_t child_index)
	{
		parent_center[0] += child_index & 1U ? child_half_size : -child_half_size;
		parent_center[1] += child_index & 2U ? child_half_size : -child_half_size;
		parent_center[2] += child_index & 4U ? child_half_size : -child_half_size;
		return parent_center;
	}

	//
	// Get sibling center
	//

	static constexpr Point3 siblingCenter(Point3 center, resolution_t half_size,
	                                      index_t index, index_t sibling_index)
	{
		index_t const temp = index ^ sibling_index;
		resolution_t const size = 2 * half_size;
		if (temp & 1U) {
			center[0] += sibling_index & 1U ? size : -size;
		}
		if (temp & 2U) {
			center[1] += sibling_index & 2U ? size : -size;
		}
		if (temp & 4U) {
			center[2] += sibling_index & 4U ? size : -size;
		}
		return center;
	}

	//
	// Get parent center
	//

	static constexpr Point3 parentCenter(Point3 child_center, resolution_t child_half_size,
	                                     index_t child_index)
	{
		child_center[0] -= child_index & 1U ? child_half_size : -child_half_size;
		child_center[1] -= child_index & 2U ? child_half_size : -child_half_size;
		child_center[2] -= child_index & 4U ? child_half_size : -child_half_size;
		return child_center;
	}

	//
	// Is leaf
	//

	static constexpr bool isLeaf(LeafNode const& node, index_t index) noexcept
	{
		return node.isLeaf(index);
	}

	static constexpr bool isAllLeaf(LeafNode const& node) noexcept
	{
		return node.isAllLeaf();
	}

	static constexpr bool isAnyLeaf(LeafNode const& node) noexcept
	{
		return node.isAnyLeaf();
	}

	static constexpr bool isNoneLeaf(LeafNode const& node) noexcept
	{
		return node.isNoneLeaf();
	}

	//
	// Set is leaf
	//

	static constexpr void setIsLeaf(LeafNode& node, bool is_leaf) noexcept
	{
		node.setLeaf(is_leaf);
	}

	static constexpr void setIsLeaf(LeafNode& node, index_t index, bool is_leaf) noexcept
	{
		node.setLeaf(index, is_leaf);
	}

	//
	// Is parent
	//

	static constexpr bool isParent(LeafNode const& node, index_t index) noexcept
	{
		return !isLeaf(node, index);
	}

	static constexpr bool isAllParent(LeafNode const& node) noexcept
	{
		return isNoneLeaf(node);
	}

	static constexpr bool isAnyParent(LeafNode const& node) noexcept
	{
		return !isAllLeaf(node);
	}

	static constexpr bool isNoneParent(LeafNode const& node) noexcept
	{
		return isAllLeaf(node);
	}

	//
	// Is modified
	//

	static constexpr bool modified(LeafNode const& node) noexcept
	{
		return node.modified();
	}

	static constexpr bool modified(LeafNode const& node, index_t index) noexcept
	{
		return node.modified(index);
	}

	static constexpr bool allModified(LeafNode const& node) noexcept
	{
		return node.allModified();
	}

	static constexpr bool anyModified(LeafNode const& node) noexcept
	{
		return node.anyModified();
	}

	static constexpr bool noneModified(LeafNode const& node) noexcept
	{
		return node.noneModified();
	}

	//
	// Set modified
	//

	static constexpr void setModified(LeafNode& node) noexcept { node.setModified(); }

	static constexpr void setModified(LeafNode& node, index_t index) noexcept
	{
		node.setModified(index);
	}

	//
	// Reset modified
	//

	static constexpr void resetModified(LeafNode& node) noexcept { node.resetModified(); }

	static constexpr void resetModified(LeafNode& node, index_t index) noexcept
	{
		node.resetModified(index);
	}

	//
	// Is node collapsible
	//

	// If all children are the same as the parent they can be pruned
	// NOTE: Only call with nodes that have children
	static bool isNodeCollapsible(InnerNode const& node, depth_t depth) noexcept
	{
		return 1 == depth ? all_of(getLeafChildren(node),
		                           [node](LeafNode const& child) { return node == child; })
		                  : all_of(getInnerChildren(node), [node](LeafNode const& child) {
			                    return isLeaf(child) && node == child;
		                    });
	}

	//
	// Update nodes
	//

	void updateNodes(LeafNode& nodes) { derived().updateNode(nodes, nodes.modified); }

	void updateNodes(InnerNode& nodes, depth_t const depth)
	{
		if (1 == depth) {
			derived().updateNode(nodes, nodes.modified, getLeafChildrenBlock(nodes));
		} else {
			derived().updateNode(nodes, nodes.modified, getInnerChildrenBlock(nodes));
		}

		pruneNodes(nodes, nodes.modified, depth);
	}

	template <bool KeepModified>
	void updateModifiedNodesRecurs(InnerNode& node, depth_t depth, depth_t max_depth)
	{
		if (isLeaf(node)) {
			if (depth <= max_depth) {
				derived().updateNode(node);
				if constexpr (!KeepModified) {
					setModified(node, false);
				}
			}
			return;
		}

		if (1 == depth) {
			for (auto& child : getLeafChildrenBlock(node)) {
				if (isAnyModified(child)) {
					updateNode(child);
					if constexpr (!KeepModified) {
						setModified(child, false);
					}
				}
			}
		} else {
			for (auto& child : getInnerChildren(node)) {
				if (isModified(child)) {
					updateModifiedNodesRecurs<KeepModified>(child, depth - 1, max_depth);
				}
			}
		}

		if (depth <= max_depth) {
			updateNode(node, depth);
			if constexpr (!KeepModified) {
				setModified(node, false);
			}
		}
	}

	//
	// Set parents modified
	//

	void setModifiedParents(Node& node)
	{
		if (node.depth() >= depthLevels()) {
			return;
		}
		setModifiedParentsRecurs(getRoot(), depthLevels(), node.code());
	}

	// NOTE: Assumes code has depth higher then depth
	void setModifiedParentsRecurs(InnerNode& node, depth_t depth, Code code)
	{
		setModified(node, true);
		if (code.depth() < depth - 1) {
			setModifiedParentsRecurs(getInnerChild(node, code.indexAtDepth(depth - 1)),
			                         depth - 1, code);
		}
	}

	//
	// Set modified
	//

	void setModifiedRecurs(InnerNode& node, depth_t depth, depth_t min_depth)
	{
		setModified(node, true);

		if (isLeaf(node) || depth == min_depth) {
			return;
		}

		if (1 == depth) {
			for (auto& child : getLeafChildren(node)) {
				setModified(child, true);
			}
		} else {
			for (auto& child : getInnerChildren(node)) {
				setModifiedRecurs(child, depth - 1, min_depth);
			}
		}
	}

	//
	// Clear Children modified
	//

	void clearModifiedRecurs(InnerNode& node, depth_t depth, depth_t max_depth)
	{
		if (!isModified(node)) {
			// Already clear
			return;
		}

		setModified(node, depth > max_depth);

		if (isLeaf(node)) {
			return;
		}

		if (1 == depth) {
			for (auto& child : getLeafChildren(node)) {
				setModified(child, false);
			}
		} else {
			for (auto& child : getInnerChildren(node)) {
				clearModifiedRecurs(child, depth - 1, max_depth);
			}
		}
	}

	//
	// Prune
	//

	// NOTE: Only call with nodes that have children
	bool pruneNode(InnerNode& node, depth_t depth)
	{
		return isNodeCollapsible(node, depth) ? deleteChildren(node, depth), true : false;
	}

	bool pruneRecurs(InnerNode& node, depth_t depth)
	{
		if (isLeaf(node)) {
			return true;
		}

		if (1 == depth) {
			return pruneNode(node, depth);
		}

		bool prunable = all_of(getInnerChildren(node), [this, depth](InnerNode& child) {
			return pruneRecurs(child, depth - 1);
		});

		return !prunable || pruneNode(node, depth);
	}

	//
	// Input/output (read/write)
	//

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
		options.leaf_size = size();
		options.depth_levels = depthLevels();
		return options;
	}

	std::vector<NodeAndIndices> readNodes(std::istream& in)
	{
		auto [tree_structure, size] = readTreeStructure(in);
		return retrieveNodes(tree_structure, size);
	}

	std::pair<std::unique_ptr<uint8_t[]>, uint64_t> readTreeStructure(std::istream& in)
	{
		uint64_t num;
		in.read(reinterpret_cast<char*>(&num), sizeof(num));

		auto tree_structure = std::make_unique<uint8_t[]>(num);
		in.read(reinterpret_cast<char*>(tree_structure.get()), num * sizeof(uint8_t));

		return {std::move(tree_structure), num};
	}

	std::vector<NodeAndIndices> retrieveNodes(
	    std::unique_ptr<uint8_t[]> const& tree_structure,
	    uint64_t const tree_structure_size)
	{
		uint64_t total_nodes = 0;
		for (size_t i = 0; i < tree_structure_size; i += 2) {
			for (size_t b = 0; 8 != b; ++b) {
				if ((tree_structure[i] >> b) & 1U) {
					++total_nodes;
				}
			}
		}

		std::vector<NodeAndIndices> nodes;
		nodes.reserve(total_nodes);

		if (0 != tree_structure[0]) {
			nodes.emplace_back(root(), tree_structure[0]);
			setModified(root());
		} else if (0 != tree_structure[1]) {
			retrieveNodesRecurs(std::next(tree_structure.get(), 2), nodes, root(),
			                    tree_structure[1], rootDepth());
		}

		return nodes;
	}

	uint8_t* retrieveNodesRecurs(uint8_t* tree_structure,
	                             std::vector<NodeAndIndices>& nodes, InnerNode& node,
	                             index_field_t indices, depth_t const depth)
	{
		// TODO: Implement

		index_field_t const children_valid_return = *tree_structure++;

		if (1 == depth) {
			if (0 == children_valid_return) {
				return tree_structure;
			}

			setModified(node, children_valid_return);

			createLeafChildren(node);

			for (std::size_t i = 0; 8 != i; ++i) {
				if ((children_valid_return >> i) & 1U) {
					nodes.push_back(
					    std::ref(getLeafChild(node, i)));  // TODO: push_back or emplace_back?
				}
			}
		} else {
			uint8_t const child_valid_inner = *indicators++;

			if (0 == children_valid_return && 0 == child_valid_inner) {
				return indicators;
			}

			setModified(node, true);

			createInnerChildren(node, depth);

			for (std::size_t i = 0; 8 != i; ++i) {
				if ((children_valid_return >> i) & 1U) {
					nodes.push_back(std::ref(static_cast<LeafNode&>(
					    getInnerChild(node, i))));  // TODO: push_back or emplace_back?
				} else if ((child_valid_inner >> i) & 1U) {
					indicators =
					    retrieveNodesRecurs(indicators, nodes, getInnerChild(node, i), depth - 1);
				}
			}
		}

		return indicators;
	}

	template <class Predicates>
	void writeData(std::ostream& out, Predicates&& predicates, depth_t min_depth,
	               bool const compress, int const compression_acceleration_level,
	               int const compression_level) const
	{
		auto [tree_structure, nodes] =
		    data(predicate::Leaf(min_depth) && std::forward<Predicates>(predicates));

		writeTreeStructure(out, tree_structure);

		derived().writeNodes(out, std::cbegin(nodes), std::cend(nodes), compress,
		                     compression_acceleration_level, compression_level);
	}

	template <bool Propagate>
	void writeModifiedData(std::ostream& out, bool compress,
	                       int compression_acceleration_level, int compression_level)
	{
		auto [tree_structure, nodes] = modifiedData<Propagate>();

		writeTreeStructure(out, tree_structure);

		derived().writeNodes(out, std::cbegin(nodes), std::cend(nodes), compress,
		                     compression_acceleration_level, compression_level);
	}

	template <class Predicates>
	std::pair<std::vector<uint8_t>, std::vector<ConstNodeAndIndices>> data(
	    Predicates const& predicates) const
	{
		std::vector<uint8_t> tree_structure;
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
			dataRecurs(root, tree_structure, nodes, predicates);
			if (nodes.empty()) {
				//  Nothing was added
				tree_structure.clear();
			}
		}

		return {std::move(tree_structure), std::move(nodes)};
	}

	template <class Predicates, class NodeType>
	void dataRecurs(NodeType const& node, std::vector<uint8_t>& tree_structure,
	                std::vector<ConstNodeAndIndices>& nodes,
	                Predicates const& predicates) const
	{
		// TODO: Implement

		uint8_t child_valid_return = 0;

		if (1 == node.depth()) {
			for (std::size_t i = 0; 8 != i; ++i) {
				if (predicate::PredicateValueCheck<Predicates>::apply(predicates, derived(),
				                                                      getNodeChild(node, i))) {
					child_valid_return |= 1U << i;
				}
			}

			indicators.push_back(child_valid_return);

			if (0 == child_valid_return) {
				return;
			}

			for (std::size_t i = 0; 8 != i; ++i) {
				if ((child_valid_return >> i) & 1U) {
					nodes.push_back(getLeafChild(getInnerNode(node), i));
				}
			}
		} else {
			auto cur_indicators_size = indicators.size();
			auto cur_nodes_size = nodes.size();

			uint8_t child_valid_inner = 0;
			for (size_t i = 0; 8 != i; ++i) {
				auto child = getNodeChild(node, i);

				if (predicate::PredicateValueCheck<Predicates>::apply(predicates, derived(),
				                                                      child)) {
					child_valid_return |= 1U << i;
				} else if (predicate::PredicateInnerCheck<Predicates>::apply(predicates,
				                                                             derived(), child)) {
					child_valid_inner |= 1U << i;
				}
			}

			indicators.push_back(child_valid_return);
			indicators.push_back(child_valid_inner);

			if (0 == child_valid_return && 0 == child_valid_inner) {
				return;
			}

			for (size_t i = 0; 8 != i; ++i) {
				if ((child_valid_return >> i) & 1U) {
					nodes.push_back(getInnerChild(getInnerNode(node), i));
				} else if ((child_valid_inner >> i) & 1U) {
					dataRecurs(getNodeChild(node, i), indicators, nodes, predicates);
				}
			}

			if (nodes.size() == cur_nodes_size) {
				indicators.resize(cur_indicators_size + 2);
				indicators.back() = 0U;
				*std::prev(std::end(indicators), 2) = 0U;
			}
		}
	}

	template <bool Propagate>
	std::pair<std::vector<uint8_t>, std::vector<ConstNodeAndIndices>> modifiedData()
	{
		std::vector<uint8_t> tree_structure;
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
	                        std::vector<uint8_t>& tree_structure,
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
	                        depth_t const depth, std::vector<uint8_t>& tree_structure,
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

	void writeTreeStructure(std::ostream& out,
	                        std::vector<uint8_t> const& tree_structure) const
	{
		uint64_t num = tree_structure.size();
		out.write(reinterpret_cast<char*>(&num), sizeof(num));
		out.write(reinterpret_cast<char const*>(tree_structure.data()),
		          num * sizeof(uint8_t));
	}

 protected:
	// The number of depth levels
	depth_t depth_levels_;
	// The maximum coordinate value the octree can store
	Key::key_t max_value_;

	// The root
	InnerNode root_;

	// Stores the node size at a given depth, where the depth is the index
	std::array<node_size_t, maxDepthLevels()> size_;
	// Reciprocal of the node size at a given depth, where the depth is the index
	std::array<node_size_t, maxDepthLevels()> size_factor_;

	// Automatic pruning
	bool automatic_pruning_enabled_ = true;

	// Locks to support parallel insertion, one per level
	std::array<std::atomic_flag, maxDepthLevels() + 1> children_locks_;

	std::stack<InnerNodeBlock*> free_inner_blocks_;
	std::stack<LeafNodeBlock*> free_leaf_blocks_;

	std::atomic_flag free_inner_block_lock_ = ATOMIC_FLAG_INIT;
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
	std::atomic_size_t num_allocated_inner_leaf_nodes_ = 1;
	// Current number of allocated leaf nodes
	std::atomic_size_t num_allocated_leaf_nodes_ = 0;
};

}  // namespace ufo::map

#endif  // UFO_MAP_OCTREE_BASE_H