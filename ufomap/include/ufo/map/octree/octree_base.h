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
#include <ufo/map/code.h>
#include <ufo/map/io.h>
#include <ufo/map/iterator/octree.h>
#include <ufo/map/key.h>
#include <ufo/map/octree/node.h>
#include <ufo/map/octree/octree_node.h>
#include <ufo/map/octree/query.h>
#include <ufo/map/point.h>
#include <ufo/map/predicate/octree.h>
#include <ufo/map/predicate/predicates.h>
#include <ufo/map/predicate/spatial.h>
#include <ufo/map/types.h>

// STL
#include <algorithm>
#include <array>
#include <atomic>
#include <execution>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>

namespace ufo::map
{
template <class Derived,  // This is the derived (CRTP)...
          class DataType, class Indicators = OctreeIndicators>
class OctreeBase
{
	// // Must be able to cast InnerNode <-> LeafNode and slice InnerNode -> LeafNode
	// static_assert(std::is_base_of_v<LeafNode, InnerNode>);

 private:
	// Minimum number of depth levels
	static constexpr const Depth MIN_DEPTH_LEVELS = 2;
	// Maximum number of depth levels
	static constexpr const Depth MAX_DEPTH_LEVELS = 21;

 protected:
	using LeafNode = OctreeLeafNode<DataType, Indicators>;
	using InnerNode = OctreeInnerNode<LeafNode>;

 public:
	using const_iterator = IteratorWrapper<Derived, Node>;
	using const_query_iterator = const_iterator;
	using const_query_nearest_iterator = IteratorWrapper<Derived, std::pair<Node, double>>;

 public:
	//
	// Clear
	//

	/*!
	 * @brief Erases the map. After this call, the map contains only the default constructed
	 * root node.
	 *
	 * @param prune If the memory should be cleared.
	 */
	void clear(bool prune = false) { clear(resolution(), depthLevels(), prune); }

	/*!
	 * @brief Erases the map. After this call, the map contains only the default constructed
	 * root node.
	 *
	 * @tparam ExecutionPolicy The execution policy to use.
	 * @param policy The execution policy to use.
	 * @param prune If the memory should be cleared.
	 */
	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void clear(ExecutionPolicy policy, bool prune = false)
	{
		clear(policy, resolution(), depthLevels(), prune);
	}

	/*!
	 * @brief Erases the map and change the resolution and the number of depth levels. After
	 * this call, the map contains only the default constructed root node.
	 *
	 * @param new_resolution The new resolution.
	 * @param new_depth_levels The new number of depth levels.
	 * @param prune If the memory should be cleared.
	 */
	void clear(double new_resolution, Depth new_depth_levels, bool prune = false)
	{
		if (root_) {
			deleteChildren(getRootImpl(), depthLevels(), prune);
		}

		// FIXME: Should this be first?
		setResolutionAndDepthLevels(new_resolution, new_depth_levels);

		initRoot();
	}

	/*!
	 * @brief Erases the map and change the resolution and the number of depth levels. After
	 * this call, the map contains only the default constructed root node.
	 *
	 * @tparam ExecutionPolicy The execution policy to use.
	 * @param new_resolution The new resolution.
	 * @param new_depth_levels The new number of depth levels.
	 * @param prune If the memory should be cleared.
	 */
	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void clear(ExecutionPolicy policy, double new_resolution, Depth new_depth_levels,
	           bool prune = false)
	{
		if (root_) {
			deleteChildren(policy, getRootImpl(), depthLevels(), prune);
		}

		// FIXME: Should this be first?
		setResolutionAndDepthLevels(new_resolution, new_depth_levels);

		initRoot();
	}

	//
	// Get node size
	//

	/*!
	 * @brief Get the size of a node at a specific depth.
	 *
	 * @param depth The depth.
	 * @return The size of a node at the depth.
	 */
	[[nodiscard]] constexpr double nodeSize(Depth depth) const { return node_size_[depth]; }

	//
	// Resolution
	//

	/*!
	 * @brief Get the resolution (leaf node size) of the octree.
	 *
	 * @return The resolution (leaf node size) of the octree.
	 */
	[[nodiscard]] constexpr double resolution() const noexcept { return nodeSize(0); }

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
	// Parallel execution depth
	//

	/*!
	 * @brief Return the depth where parallel execution begins.
	 *
	 * @return The depth parallel execution begins.
	 */
	[[nodiscard]] constexpr Depth parallelExecutionDepth() const noexcept
	{
		return parallel_depth_;
	}

	/*!
	 * @brief Set the depth when parallel execution begins.
	 *
	 * @param new_parallel_depth The new depth where parallel execution begins.
	 */
	constexpr void parallelExecutionDepth(Depth new_parallel_depth) noexcept
	{
		parallel_depth_ = new_parallel_depth;
	}

	//
	// Depth levels
	//

	/*!
	 * @brief The octree depth levels.
	 *
	 * @return The number of octree depth levels.
	 */
	[[nodiscard]] constexpr Depth depthLevels() const noexcept { return depth_levels_; }

	/*!
	 * @brief The minimum depth levels an octree can have.
	 *
	 * @return The minimum depth levels an octree can have.
	 */
	[[nodiscard]] static constexpr Depth minDepthLevels() { return MIN_DEPTH_LEVELS; }

	/*!
	 * @brief The maximum depth levels an octree can have.
	 *
	 * @return The maximum depth levels an octree can have.
	 */
	[[nodiscard]] static constexpr Depth maxDepthLevels() { return MAX_DEPTH_LEVELS; }

	//
	// Get min/max coordinate octree can store
	//

	/*!
	 * @return The minimum coordinate the octree can store.
	 */
	[[nodiscard]] Point3 min() const
	{
		auto half_size = -nodeSize(depthLevels() - 1);
		return Point3(half_size, half_size, half_size);
	}

	/*!
	 * @return The maximum coordinate the octree can store.
	 */
	[[nodiscard]] Point3 max() const
	{
		auto half_size = nodeSize(depthLevels() - 1);
		return Point3(half_size, half_size, half_size);
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
		return geometry::AAEBB(center(), nodeSize(depthLevels() - 1));
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
		auto min = -nodeSize(depthLevels() - 1);
		auto max = -min;
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
		return num_inner_nodes_ + num_inner_leaf_nodes_;
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
		return numInnerNodes() + numLeafNodes();
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
	[[nodiscard]] virtual std::size_t memoryUsage() const noexcept
	{
		return (numInnerNodes() * memoryInnerNode()) + (numLeafNodes() * memoryLeafNode());
	}

	/*!
	 * @return Number of nodes in the octree.
	 */
	[[nodiscard]] constexpr std::size_t size() const noexcept
	{
		return numInnerNodes() + numLeafNodes();
	}

	/*!
	 * @return Number of allocated inner nodes.
	 */
	[[nodiscard]] constexpr std::size_t numInnerNodesAllocated() const noexcept
	{
		return num_allocated_inner_nodes_ + num_allocated_inner_leaf_nodes_;
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
		return numInnerNodesAllocated() + numLeafNodesAllocated();
	}

	/*!
	 * @brief Lower bound memory usage for all nodes.
	 *
	 * @note Does not account for pointed to data inside nodes.
	 *
	 * @return Memory usage of the octree.
	 */
	[[nodiscard]] virtual std::size_t memoryUsageAllocated() const noexcept
	{
		return (numInnerNodesAllocated() * memoryInnerNode()) +
		       (numLeafNodesAllocated() * memoryLeafNode());
	}

	/*!
	 * @return Number of nodes in the octree.
	 */
	[[nodiscard]] constexpr std::size_t sizeAllocated() const noexcept
	{
		return numInnerNodesAllocated() + numLeafNodesAllocated();
	}

	//
	// Is pure leaf
	//

	/*!
	 * @brief Check if node is a pure leaf node (can never have children).
	 *
	 * @param node The node to check.
	 * @return Whether the node is a pure leaf node.
	 */
	[[nodiscard]] static constexpr bool isPureLeaf(Node node) noexcept
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
	[[nodiscard]] static constexpr bool isPureLeaf(Code code) noexcept
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
	[[nodiscard]] static constexpr bool isPureLeaf(Key key) noexcept
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
	[[nodiscard]] static constexpr bool isPureLeaf(Point3 coord, Depth depth = 0) noexcept
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
	[[nodiscard]] static constexpr bool isPureLeaf(coord_t x, coord_t y, coord_t z,
	                                               Depth depth = 0) noexcept
	{
		return 0 == depth;
	}

	//
	// Is leaf
	//

	/*!
	 * @brief Check if node is a leaf node (has no children).
	 *
	 * @param node The node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] static constexpr bool isLeaf(Node node) noexcept
	{
		return isPureLeaf(node) || isLeaf(innerNode(node));
	}

	/*!
	 * @brief Check if the node corresponding to the code is a leaf node (has no children).
	 *
	 * @param code The code of the node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] constexpr bool isLeaf(Code code) noexcept
	{
		return isPureLeaf(code) || isLeaf(innerNode(code));
	}

	/*!
	 * @brief Check if the node corresponding to the key is a leaf node (has no children).
	 *
	 * @param key The key of the node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] constexpr bool isLeaf(Key key) noexcept
	{
		return isPureLeaf(key) || isLeaf(innerNode(toCode(key)));
	}

	/*!
	 * @brief Check if the node corresponding to the coordinate is a leaf node (has no
	 * children).
	 *
	 * @param coord The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] constexpr bool isLeaf(Point3 coord, Depth depth = 0) noexcept
	{
		return isPureLeaf(coord, depth) || isLeaf(innerNode(toCode(coord, depth)));
	}

	/*!
	 * @brief Check if the node corresponding to the coordinate is a leaf node (has no
	 * children).
	 *
	 * @param x,y,z The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] constexpr bool isLeaf(coord_t x, coord_t y, coord_t z, Depth depth = 0) noexcept
	{
		return isPureLeaf(x, y, z, depth) || isLeaf(innerNode(toCode(x, y, z, depth)));
	}

	//
	// Is parent
	//

	/*!
	 * @brief Check if node is a parent, i.e., has children.
	 *
	 * @param node The node to check.
	 * @return Whether the node is a parent.
	 */
	[[nodiscard]] static constexpr bool isParent(Node node) noexcept
	{
		return !isLeaf(node);
	}

	/*!
	 * @brief Check if the node corresponding to the code is a parent, i.e., has
	 * children.
	 *
	 * @param code The code of the node to check.
	 * @return Whether the node is a parent.
	 */
	[[nodiscard]] static constexpr bool isParent(Code code) noexcept
	{
		return !isLeaf(code);
	}

	/*!
	 * @brief Check if the node corresponding to the key is a parent, i.e., has
	 * children.
	 *
	 * @param key The key of the node to check.
	 * @return Whether the node is a parent.
	 */
	[[nodiscard]] static constexpr bool isParent(Key key) noexcept { return !isLeaf(key); }

	/*!
	 * @brief Check if the node corresponding to the coordinate is a parent, i.e., has
	 * children.
	 *
	 * @param coord The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a parent.
	 */
	[[nodiscard]] constexpr bool isParent(Point3 coord, Depth depth = 0) noexcept
	{
		return !isLeaf(coord, depth);
	}

	/*!
	 * @brief Check if the node corresponding to the coordinate is a parent, i.e., has
	 * children.
	 *
	 * @param x,y,z The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a parent.
	 */
	[[nodiscard]] constexpr bool isParent(coord_t x, coord_t y, coord_t z,
	                                      Depth depth = 0) noexcept
	{
		return !isLeaf(x, y, z, depth);
	}

	//
	// To code
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
	[[nodiscard]] constexpr Code toCode(Point3 coord, Depth depth = 0) const noexcept
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
	                                    Depth depth = 0) const noexcept
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
	    Point3 coord, Depth depth = 0) const noexcept
	{
		std::optional<Key::KeyType> key = toKeyChecked(coord, depth);
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
	    coord_t x, coord_t y, coord_t z, Depth depth = 0) const noexcept
	{
		return toCodeChecked(Point3(x, y, z), depth);
	}

	//
	// To key
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
	constexpr Key toKey(Point3 coord, Depth depth = 0) const noexcept
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
	constexpr Key toKey(coord_t x, coord_t y, coord_t z, Depth depth = 0) const noexcept
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
	constexpr std::optional<Key> toKeyChecked(Point3 coord, Depth depth = 0) const noexcept
	{
		return depthLevels() >= depth && isWithin(coord)
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
	constexpr std::optional<Key> toKeyChecked(coord_t x, coord_t y, coord_t z,
	                                          Depth depth = 0) const noexcept
	{
		return toKeyChecked(Point3(x, y, z), depth);
	}

	//
	// To coordinate
	//

	/*!
	 * @brief Convert a code to a coordinate.
	 *
	 * @param code The code.
	 * @return The corresponding coordinate.
	 */
	constexpr Point3 toCoord(Code code) const noexcept { return toCoord(toKey(code)); }

	/*!
	 * @brief Convert a key to a coordinate.
	 *
	 * @param key The key.
	 * @return The corresponding coordinate.
	 */
	constexpr Point3 toCoord(Key key) const noexcept
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
	constexpr std::optional<Point3> toCoordChecked(Code code) const noexcept
	{
		return toCoordChecked(toKey(code));
	}

	/*!
	 * @brief Convert a key to a coordinate with bounds check.
	 *
	 * @param key The key.
	 * @return The corresponding coordinate.
	 */
	constexpr std::optional<Point3> toCoordChecked(Key key) const noexcept
	{
		return key.depth() <= depthLevels() ? std::optional<Point3>(toCoord(key))
		                                    : std::nullopt;
	}

	//
	// Get root
	//

	/*!
	 * @brief Get the root node.
	 *
	 * @return The root node.
	 */
	constexpr Node getRoot() { return Node(&getRootImpl(), getRootCode()); }

	/*!
	 * @brief Get the root node.
	 *
	 * @return The root node.
	 */
	constexpr Node getRoot() const
	{
		// FIXME: Look at
		return Node(const_cast<InnerNode*>(&getRootImpl()), getRootCode());
	}

	/*!
	 * @brief Get the root node with bounding volume.
	 *
	 * @return The root node with bounding volume.
	 */
	constexpr NodeBV getRootBV() { return NodeBV(getRoot(), boundingVolume()); }

	/*!
	 * @brief Get the root node with bounding volume.
	 *
	 * @return The root node with bounding volume.
	 */
	constexpr NodeBV getRootBV() const
	{
		// FIXME: Look at
		return NodeBV(getRoot(), boundingVolume());
	}

	/*!
	 * @brief Get the code for the root node.
	 *
	 * @return The root node code.
	 */
	constexpr Code getRootCode() const { return Code(0, depthLevels()); }

	//
	// Get node
	//

	/*!
	 * @brief Get the corresponding node for the code.
	 *
	 * @param code The code for the node.
	 * @return The node.
	 */
	Node getNode(Code code) const
	{
		Node node = getRoot();

		while (isParent(node) && code.depth() != node.depth()) {
			node = nodeChild(node, code.indexAtDepth(node.depth() - 1));
		}

		return node;
	}

	/*!
	 * @brief Get the corresponding node for the key.
	 *
	 * @param key The key for the node.
	 * @return The node.
	 */
	Node getNode(Key key) const { return getNode(toCode(key)); }

	/*!
	 * @brief Get the corresponding node for the coordinate at a specific depth.
	 *
	 * @param coord The coordinate for the node.
	 * @param depth The depth.
	 * @return The node.
	 */
	Node getNode(Point3 coord, Depth depth = 0) const
	{
		return getNode(toCode(coord, depth));
	}

	/*!
	 * @brief Get the corresponding node for the coordinate at a specific depth.
	 *
	 * @param x,y,z The coordinate for the node.
	 * @param depth The depth.
	 * @return The node.
	 */
	Node getNode(coord_t x, coord_t y, coord_t z, Depth depth = 0) const
	{
		return getNode(toCode(x, y, z, depth));
	}

	/*!
	 * @brief Get the corresponding node for the code with bounds check.
	 *
	 * @param code The code for the node.
	 * @return The node.
	 */
	std::optional<Node> getNodeChecked(Code code) const
	{
		return code.depth() <= depthLevels() ? std::optional<Node>(getNode(code))
		                                     : std::nullopt;
	}

	/*!
	 * @brief Get the corresponding node for the key with bounds check.
	 *
	 * @param key The key for the node.
	 * @return The node.
	 */
	std::optional<Node> getNodeChecked(Key key) const
	{
		return getNodeChecked(toCode(key));
	}

	/*!
	 * @brief Get the corresponding node for the coordinate at a specific depth with bounds
	 * check.
	 *
	 * @param coord The coordinate for the node.
	 * @param depth The depth.
	 * @return The node.
	 */
	std::optional<Node> getNodeChecked(Point3 coord, Depth depth = 0) const
	{
		if (auto code = toCodeChecked(coord, depth)) {
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
	std::optional<Node> getNodeChecked(coord_t x, coord_t y, coord_t z, Depth depth = 0) const
	{
		return getNodeChecked(Point3(x, y, z), depth);
	}

	//
	// FIXME: Get bounding volume node
	//

	// NodeBV getNodeBV(Code code) const
	// {
	// 	NodeBV node(getRootNodeBV());

	// 	while (isParent(node) && code.depth() != node.depth()) {
	// 		node = getChild(node, code.indexAtDepth(node.depth() - 1));
	// 	}

	// 	return node;
	// }

	// NodeBV getNodeBV(Key key) const { return getNodeBV(toCode(key)); }

	// NodeBV getNodeBV(Point3 coord, Depth depth = 0) const
	// {
	// 	return getNodeBV(toCode(coord, depth));
	// }

	// NodeBV getNodeBV(coord_t x, coord_t y, coord_t z, Depth depth = 0) const
	// {
	// 	return getNodeBV(toCode(x, y, z, depth));
	// }

	//
	// Node bounding volume
	//

	constexpr geometry::AAEBB nodeBoundingVolume(Node const& node) const noexcept
	{
		return geometry::AAEBB(nodeCenter(node), nodeSize(node) / 2);
	}

	static constexpr geometry::AAEBB nodeBoundingVolume(NodeBV const& node) noexcept
	{
		return node.boundingVolume();
	}

	constexpr Point3 nodeCenter(Node const& node) const noexcept
	{
		return toCoord(node.code());
	}

	static constexpr Point3 nodeCenter(NodeBV const& node) noexcept
	{
		return node.center();
	}

	constexpr Point3 nodeMin(Node const& node) const noexcept
	{
		return nodeCenter(node) - (nodeSize(node) / 2);
	}

	static constexpr Point3 nodeMin(NodeBV const& node) noexcept { return node.min(); }

	constexpr Point3 nodeMax(Node const& node) const noexcept
	{
		return nodeCenter(node) + (nodeSize(node) / 2);
	}

	static constexpr Point3 nodeMax(NodeBV const& node) noexcept { return node.max(); }

	constexpr double nodeSize(Node const& node) const noexcept
	{
		return nodeSize(node.depth());
	}

	static constexpr double nodeSize(NodeBV const& node) noexcept { return node.size(); }

	constexpr coord_t nodeX(Node const& node) const noexcept
	{
		return toCoord(node.code().toKey(0), node.depth());
	}

	static constexpr coord_t nodeX(NodeBV const& node) noexcept { return node.x(); }

	constexpr coord_t nodeY(Node const& node) const noexcept
	{
		return toCoord(node.code().toKey(1), node.depth());
	}

	static constexpr coord_t nodeY(NodeBV const& node) noexcept { return node.y(); }

	constexpr coord_t nodeZ(Node const& node) const noexcept
	{
		return toCoord(node.code().toKey(2), node.depth());
	}

	static constexpr coord_t nodeZ(NodeBV const& node) noexcept { return node.z(); }

	//
	// Node child
	//

	static Node nodeChild(Node const& node, size_t child_idx)
	{
		LeafNode& child = getChild(innerNode(node), node.depth() - 1, child_idx);

		return Node(&child, node.code().child(child_idx));
	}

	static NodeBV nodeChild(NodeBV const& node, size_t child_idx)
	{
		LeafNode& child_node = getChild(innerNode(node), node.depth() - 1, child_idx);

		double child_half_size = node.halfSize() / 2;
		geometry::AAEBB child_aaebb(childCenter(node.center(), child_half_size, child_idx),
		                            child_half_size);

		return NodeBV(&child_node, node.code().child(child_idx), child_aaebb);
	}

	template <class Node>
	static Node nodeChildChecked(Node const& node, size_t child_idx)
	{
		if (!isParent(node)) {
			throw std::out_of_range("Node has no children");
		} else if (7 < child_idx) {
			throw std::out_of_range("child_idx out of range");
		}
		return nodeChild(node, child_idx);
	}

	//
	// Get Sibling
	//

	static Node nodeSibling(Node const& node, size_t sibling_idx)
	{
		auto cur_idx = node.code().indexAtDepth(node.depth());

		if (sibling_idx == cur_idx) {
			return node;
		}

		LeafNode* sibling;
		int idx_diff = static_cast<int>(sibling_idx) - static_cast<int>(cur_idx);
		if (isPureLeaf(node)) {
			sibling = const_cast<LeafNode*>(std::next(&leafNode(node), idx_diff));
		} else {
			sibling = static_cast<LeafNode*>(
			    const_cast<InnerNode*>(std::next(&innerNode(node), idx_diff)));
		}

		return Node(sibling, node.code().sibling(sibling_idx));
	}

	static NodeBV nodeSibling(NodeBV const& node, size_t sibling_idx)
	{
		auto cur_idx = node.code().indexAtDepth(node.depth());

		if (sibling_idx == cur_idx) {
			return node;
		}

		LeafNode* sibling;
		int idx_diff = static_cast<int>(sibling_idx) - static_cast<int>(cur_idx);
		if (isPureLeaf(node)) {
			sibling = const_cast<LeafNode*>(std::next(&leafNode(node), idx_diff));
		} else {
			sibling = static_cast<LeafNode*>(
			    const_cast<InnerNode*>(std::next(&innerNode(node), idx_diff)));
		}

		geometry::AAEBB sibling_aaebb(
		    siblingCenter(node.center(), node.halfSize(), cur_idx, sibling_idx),
		    node.halfSize());

		return NodeBV(sibling, node.code().sibling(sibling_idx), sibling_aaebb);
	}

	template <class Node>
	Node nodeSiblingChecked(Node const& node, size_t sibling_idx)
	{
		if (!isRoot(node)) {
			throw std::out_of_range("Node has no siblings");
		} else if (7 < sibling_idx) {
			throw std::out_of_range("sibling_idx out of range");
		}
		return nodeSibling(node, sibling_idx);
	}

	//
	// Modified
	//

	static constexpr bool isModified(Node const& node)
	{
		return isModified(leafNode(node));
	}

	constexpr bool isModified(Code code) const { return isModified(leafNode(code)); }

	constexpr bool isModified(Key key) const { return isModified(toCode(key)); }

	constexpr bool isModified(Point3 coord, Depth depth = 0) const
	{
		return isModified(toCode(coord, depth));
	}

	constexpr bool isModified(coord_t x, coord_t y, coord_t z, Depth depth = 0) const
	{
		return isModified(toCode(x, y, z, depth));
	}

	//
	// Is root
	//

	bool isRoot(Node const& node) const { return isRoot(node.code()); }

	bool isRoot(Code code) const { return getRootCode() == code; }

	bool isRoot(Key key) const { return isRoot(toCode(key)); }

	bool isRoot(Point3 coord, Depth depth = 0) const
	{
		return isRoot(toCode(coord, depth));
	}

	bool isRoot(coord_t x, coord_t y, coord_t z, Depth depth = 0) const
	{
		return isRoot(toCode(x, y, z, depth));
	}

	//
	// Create node
	//

	Node createNode(Code code)
	{
		InnerNode* node = &getRootImpl();
		auto const min_depth = std::max(static_cast<Depth>(1), code.depth());
		Depth cur_depth = depthLevels();
		for (; min_depth < cur_depth; --cur_depth) {
			if (isLeaf(*node)) {
				createInnerChildren(*node, cur_depth, code.indexAtDepth(cur_depth));
			}
			node = &getInnerChild(*node, code.indexAtDepth(cur_depth - 1));
		}

		if (0 == code.depth()) {
			if (isLeaf(*node)) {
				createLeafChildren(*node, code.indexAtDepth(1));
			}
			return Node(&getLeafChild(*node, code.indexAtDepth(0)), code);
		} else {
			return Node(node, code);
		}
	}

	Node createNode(Key key) { return createNode(toCode(key)); }

	Node createNode(Point3 coord, Depth depth = 0)
	{
		return createNode(toCode(coord, depth));
	}

	Node createNode(coord_t x, coord_t y, coord_t z, Depth depth = 0)
	{
		return createNode(toCode(x, y, z, depth));
	}

	//
	// Create bv node
	//

	NodeBV createNodeBV(Code code)
	{
		// FIXME: Handle wrong codes

		NodeBV node(getRoot());

		while (code.depth() != node.depth()) {
			if (isLeaf(node)) {
				createChildren(innerNode(node), node.depth(),
				               node.code().indexAtDepth(node.depth()));
			}
			node = nodeChild(node, code.indexAtDepth(node.depth() - 1));
		}

		return node;
	}

	NodeBV createNodeBV(Key key) { return createNodeBV(toCode(key)); }

	NodeBV createNodeBV(Point3 coord, Depth depth = 0)
	{
		return createNodeBV(toCode(coord, depth));
	}

	NodeBV createNodeBV(coord_t x, coord_t y, coord_t z, Depth depth = 0)
	{
		return createNodeBV(toCode(x, y, z, depth));
	}

	//
	// Query
	//

	template <class Predicates>
	Query<const_query_iterator> query(Predicates const& predicates) const
	{
		return Query<const_query_iterator>(beginQuery(predicates), endQuery());
	}

	template <class Geometry, class Predicates>
	Query<const_query_nearest_iterator> queryNearest(Geometry const& geometry,
	                                                 Predicates const& predicates) const
	{
		return Query<const_query_nearest_iterator>(beginQueryNearest(geometry, predicates),
		                                           endQueryNearest());
	}

	template <class Predicates, class OutputIt>
	OutputIt query(Predicates const& predicates, OutputIt d_first) const
	{
		return std::copy(beginQuery(predicates), endQuery(predicates), d_first);
	}

	template <class ExecutionPolicy, class Predicates, class OutputIt,
	          typename = std::enable_if_t<
	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
	OutputIt query(ExecutionPolicy policy, Predicates const& predicates,
	               OutputIt d_first) const
	{
		return std::copy(policy, beginQuery(predicates), endQuery(predicates), d_first);
	}

	template <class Predicates, class OutputIt>
	OutputIt queryK(size_t k, Predicates const& predicates, OutputIt d_first) const
	{
		size_t count = 0;
		for (auto it = beginQuery(predicates); count < k && it != endQuery(); ++it, ++count) {
			*d_first++ = *it;
		}
		return d_first;
	}

	template <class ExecutionPolicy, class Predicates, class OutputIt,
	          typename = std::enable_if_t<
	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
	OutputIt queryK(ExecutionPolicy policy, size_t k, Predicates const& predicates,
	                OutputIt d_first) const
	{
		size_t count = 0;
		for (auto it = beginQuery(predicates); count < k && it != endQuery(); ++it, ++count) {
			*d_first++ = *it;
		}
		return d_first;
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearest(Geometry const& geometry, Predicates const& predicates,
	                      OutputIt d_first, float epsilon = 0.0f) const
	{
		return std::copy(beginQueryNearest(geometry, predicates, epsilon),
		                 endQueryNearest(geometry, predicates), d_first);
	}

	template <class ExecutionPolicy, class Geometry, class Predicates, class OutputIt,
	          typename = std::enable_if_t<
	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
	OutputIt queryNearest(ExecutionPolicy policy, Geometry const& geometry,
	                      Predicates const& predicates, OutputIt d_first,
	                      float epsilon = 0.0f) const
	{
		return std::copy(policy, beginQueryNearest(geometry, predicates, epsilon),
		                 endQueryNearest(geometry, predicates), d_first);
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearestK(size_t k, Geometry const& geometry, Predicates const& predicates,
	                       OutputIt d_first, float epsilon = 0.0f) const
	{
		size_t count = 0;
		for (auto it = beginQueryNearest(geometry, predicates, epsilon);
		     count < k && it != endQueryNearest(); ++it, ++count) {
			*d_first++ = *it;
		}
		return d_first;
	}

	template <class ExecutionPolicy, class Geometry, class Predicates, class OutputIt,
	          typename = std::enable_if_t<
	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
	OutputIt queryNearestK(ExecutionPolicy policy, size_t k, Geometry const& geometry,
	                       Predicates const& predicates, OutputIt d_first,
	                       float epsilon = 0.0f) const
	{
		size_t count = 0;
		for (auto it = beginQueryNearest(geometry, predicates, epsilon);
		     count < k && it != endQueryNearest(); ++it, ++count) {
			*d_first++ = *it;
		}
		return d_first;
	}

	//
	// Query iterator
	//

	template <class Predicates>
	const_query_iterator beginQuery(Predicates const& predicates) const
	{
		return const_query_iterator(
		    new Iterator(dynamic_cast<Derived const*>(this), getRoot(), predicates));
	}

	const_query_iterator endQuery() const
	{
		return const_query_iterator(new Iterator<Derived>(getRoot()));
	}

	template <class Geometry, class Predicates>
	const_query_nearest_iterator beginQueryNearest(Geometry const& geometry,
	                                               Predicates const& predicates,
	                                               float epsilon = 0.0f) const
	{
		return const_query_nearest_iterator(new NearestIterator(
		    dynamic_cast<Derived const*>(this), getRoot(), geometry, predicates, epsilon));
	}

	const_query_nearest_iterator endQueryNearest() const
	{
		return const_query_nearest_iterator(new NearestIterator<Derived>());
	}

	//
	// "Normal" iterator
	//

	const_iterator begin() const { return beginQuery(predicate::TRUE{}); }

	const_iterator end() const { return endQuery(predicate::TRUE{}); }

	//
	// Traverse
	//

	// If f returns true then children will not be visited
	template <class UnaryFunction>
	void traverse(UnaryFunction f)
	{
		auto root = getRoot();
		if (!f(root)) {
			traverseRecurs(f, root);
		}
	}

	template <class UnaryFunction>
	void traverse(UnaryFunction f) const
	{
		auto root = getRoot();
		if (!f(root)) {
			traverseRecurs(f, root);
		}
	}

	//
	// Update modified nodes
	//

	void updateModifiedNodes(bool keep_modified = false, Depth max_depth = maxDepthLevels())
	{
		if (keep_modified) {
			updateModifiedNodesRecurs<true>(max_depth, getRootImpl(), depthLevels());
		} else {
			updateModifiedNodesRecurs<false>(max_depth, getRootImpl(), depthLevels());
		}
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void updateModifiedNodes(ExecutionPolicy policy, bool keep_modified = false,
	                         Depth max_depth = maxDepthLevels())
	{
		if (keep_modified) {
			updateModifiedNodesRecurs<true>(policy, max_depth, getRootImpl(), depthLevels());
		} else {
			updateModifiedNodesRecurs<false>(policy, max_depth, getRootImpl(), depthLevels());
		}
	}

	void updateModifiedNodes(Node const& node, bool keep_modified = false,
	                         Depth max_depth = maxDepthLevels())
	{
		// TODO: Implement
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void updateModifiedNodes(ExecutionPolicy policy, Node const& node,
	                         bool keep_modified = false, Depth max_depth = maxDepthLevels())
	{
		// TODO: Implement
		// if ()

		// updateModifiedNodesRecurs(policy, max_depth, )
	}

	void updateModifiedNodes(Code code, bool keep_modified = false,
	                         Depth max_depth = maxDepthLevels())
	{
		// TODO: Implement
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void updateModifiedNodes(ExecutionPolicy policy, Code code, bool keep_modified = false,
	                         Depth max_depth = maxDepthLevels())
	{
		// TODO: Implement
	}

	void updateModifiedNodes(Key key, bool keep_modified = false,
	                         Depth max_depth = maxDepthLevels())
	{
		updateModifiedNodes(toCode(key), keep_modified, max_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void updateModifiedNodes(ExecutionPolicy policy, Key key, bool keep_modified = false,
	                         Depth max_depth = maxDepthLevels())
	{
		updateModifiedNodes(policy, toCode(key), keep_modified, max_depth);
	}

	void updateModifiedNodes(Point3 coord, Depth depth = 0, bool keep_modified = false,
	                         Depth max_depth = maxDepthLevels())
	{
		updateModifiedNodes(toCode(coord, depth), depth, keep_modified, max_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void updateModifiedNodes(ExecutionPolicy policy, Point3 coord, Depth depth = 0,
	                         bool keep_modified = false, Depth max_depth = maxDepthLevels())
	{
		updateModifiedNodes(policy, toCode(coord, depth), keep_modified, max_depth);
	}

	void updateModifiedNodes(double x, double y, double z, Depth depth = 0,
	                         bool keep_modified = false, Depth max_depth = maxDepthLevels())
	{
		updateModifiedNodes(toCode(x, y, z, depth), keep_modified, max_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void updateModifiedNodes(ExecutionPolicy policy, double x, double y, double z,
	                         Depth depth = 0, bool keep_modified = false,
	                         Depth max_depth = maxDepthLevels())
	{
		updateModifiedNodes(policy, toCode(x, y, z, depth), keep_modified, max_depth);
	}

	//
	// Set modified
	//

	void setModified(Depth min_depth = 0)
	{
		if (depthLevels() >= min_depth) {
			setModifiedRecurs(getRootImpl(), depthLevels(), min_depth);
		}
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setModified(ExecutionPolicy policy, Depth min_depth = 0)
	{
		// FIXME: Update
		if (depthLevels() >= min_depth) {
			setModifiedRecurs(getRootImpl(), depthLevels(), min_depth);
		}
	}

	void setModified(Node const& node, Depth min_depth = 0)
	{
		if (node.depth() >= min_depth) {
			if (0 == node.depth()) {
				leafNode(node).modified = true;
			} else {
				setModifiedRecurs(innerNode(node), node.depth(), min_depth);
			}
			setModifiedParentsRecurs(getRootImpl(), depthLevels(), node.code());
		} else {
			setModifiedParentsRecurs(getRootImpl(), depthLevels(),
			                         node.code().toDepth(min_depth));
		}
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setModified(ExecutionPolicy policy, Node const& node, Depth min_depth = 0)
	{
		if (node.depth() >= min_depth) {
			if (0 == node.depth()) {
				leafNode(node).modified = true;
			} else {
				// FIXME: Update
				setModifiedRecurs(innerNode(node), node.depth(), min_depth);
			}
			setModifiedParentsRecurs(getRootImpl(), depthLevels(), node.code());
		} else {
			setModifiedParentsRecurs(getRootImpl(), depthLevels(),
			                         node.code().toDepth(min_depth));
		}
	}

	void setModified(Code code, Depth min_depth = 0)
	{
		if (code.depth() >= min_depth) {
			if (0 == code.depth()) {
				leafNode(code).modified = true;
			} else {
				setModifiedRecurs(innerNode(code), code.depth(), min_depth);
			}
			setModifiedParentsRecurs(getRootImpl(), depthLevels(), code);
		} else {
			setModifiedParentsRecurs(getRootImpl(), depthLevels(), code.toDepth(min_depth));
		}
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setModified(ExecutionPolicy policy, Code code, Depth min_depth = 0)
	{
		if (code.depth() >= min_depth) {
			if (0 == code.depth()) {
				leafNode(code).modified = true;
			} else {
				// FIXME: Update
				setModifiedRecurs(innerNode(code), code.depth(), min_depth);
			}
			setModifiedParentsRecurs(getRootImpl(), depthLevels(), code);
		} else {
			setModifiedParentsRecurs(getRootImpl(), depthLevels(), code.toDepth(min_depth));
		}
	}

	void setModified(Key key, Depth min_depth = 0) { setModified(toCode(key), min_depth); }

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setModified(ExecutionPolicy policy, Key key, Depth min_depth = 0)
	{
		setModified(policy, toCode(key), min_depth);
	}

	void setModified(Point3 coord, Depth depth = 0, Depth min_depth = 0)
	{
		setModified(toCode(coord, depth), min_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setModified(ExecutionPolicy policy, Point3 coord, Depth depth = 0,
	                 Depth min_depth = 0)
	{
		setModified(policy, toCode(coord, depth), min_depth);
	}

	void setModified(double x, double y, double z, Depth depth = 0, Depth min_depth = 0)
	{
		setModified(toCode(x, y, z, depth), min_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setModified(ExecutionPolicy policy, double x, double y, double z, Depth depth = 0,
	                 Depth min_depth = 0)
	{
		setModified(policy, toCode(x, y, z, depth), min_depth);
	}

	//
	// Clear modified
	//

	void clearModified(Depth max_depth = maxDepthLevels())
	{
		clearModifiedRecurs(getRootImpl(), depthLevels(), max_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void clearModified(ExecutionPolicy policy, Depth max_depth = maxDepthLevels())
	{
		// FIXME: Update
		clearModifiedRecurs(getRootImpl(), depthLevels(), max_depth);
	}

	void clearModified(Node const& node, Depth max_depth = maxDepthLevels())
	{
		if (isLeaf(node)) {
			leafNode(node).modified = false;
		} else {
			clearModifiedRecurs(innerNode(node), node.depth(), max_depth);
		}
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void clearModified(ExecutionPolicy policy, Node const& node,
	                   Depth max_depth = maxDepthLevels())
	{
		// FIXME: Update
		if (isPureLeaf(node)) {
			leafNode(node).modified = false;
		} else {
			clearModifiedRecurs(innerNode(node), node.depth(), max_depth);
		}
	}

	void clearModified(Code code, Depth max_depth = maxDepthLevels())
	{
		if (0 == code.depth()) {
			leafNode(code).modified = false;
		} else {
			clearModifiedRecurs(innerNode(code), code.depth(), max_depth);
		}
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void clearModified(ExecutionPolicy policy, Code code,
	                   Depth max_depth = maxDepthLevels())
	{
		// FIXME: Update
		if (0 == code.depth()) {
			leafNode(code).modified = false;
		} else {
			clearModifiedRecurs(innerNode(code), code.depth(), max_depth);
		}
	}

	void clearModified(Key key, Depth max_depth = maxDepthLevels())
	{
		clearModified(toCode(key), max_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void clearModified(ExecutionPolicy policy, Key key, Depth max_depth = maxDepthLevels())
	{
		clearModified(policy, toCode(key), max_depth);
	}

	void clearModified(Point3 coord, Depth depth = 0, Depth max_depth = maxDepthLevels())
	{
		clearModified(toCode(coord, depth), max_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void clearModified(ExecutionPolicy policy, Point3 coord, Depth depth = 0,
	                   Depth max_depth = maxDepthLevels())
	{
		clearModified(policy, toCode(coord, depth), max_depth);
	}

	void clearModified(double x, double y, double z, Depth depth = 0,
	                   Depth max_depth = maxDepthLevels())
	{
		clearModified(toCode(x, y, z, depth), max_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void clearModified(ExecutionPolicy policy, double x, double y, double z,
	                   Depth depth = 0, Depth max_depth = maxDepthLevels())
	{
		clearModified(policy, toCode(x, y, z, depth), max_depth);
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] bool canMerge(std::filesystem::path const& filename) const
	{
		std::ifstream file;
		file.exceptions(std::ifstream::failbit | std::ifstream::badbit);

		file.open(filename, std::ios_base::in | std::ios_base::binary);
		return canMerge(file);
	}

	[[nodiscard]] bool canMerge(std::istream& in_stream) const
	{
		auto pos = in_stream.tellg();
		FileInfo header = readHeader(in_stream);
		in_stream.seekg(pos);

		double res;
		Depth depth_levels;
		std::istringstream(header.at("resolution").at(0)) >> res;
		uint32_t tmp;
		std::istringstream(header.at("depth_levels").at(0)) >> tmp;
		depth_levels = tmp;

		return resolution() == res && depthLevels() == depth_levels;
	}

	void read(std::filesystem::path const& filename, bool propagate = true)
	{
		std::ifstream file;
		file.exceptions(std::ifstream::failbit | std::ifstream::badbit);

		file.open(filename, std::ios_base::in | std::ios_base::binary);

		read(file, propagate);
	}

	void read(std::istream& in_stream, bool propagate = true)
	{
		if (!correctFileType(in_stream)) {
			throw std::runtime_error("Trying to read non-UFOMap file");
		}

		FileInfo header = readHeader(in_stream);

		readData(in_stream, header, propagate);
	}

	void readData(std::istream& in_stream, FileInfo const& header, bool propagate = true)
	{
		double res;
		Depth depth_levels;

		std::istringstream(header.at("resolution").at(0)) >> res;
		uint32_t tmp;
		std::istringstream(header.at("depth_levels").at(0)) >> tmp;
		depth_levels = tmp;

		if (resolution() != res || depthLevels() != depth_levels) {
			clear(res, depth_levels);
		}

		uint8_t compressed;
		in_stream.read(reinterpret_cast<char*>(&compressed), sizeof(compressed));
		uint64_t uncompressed_data_size;
		in_stream.read(reinterpret_cast<char*>(&uncompressed_data_size),
		               sizeof(uncompressed_data_size));

		if (UINT8_MAX == compressed) {
			std::stringstream data(std::ios_base::in | std::ios_base::out |
			                       std::ios_base::binary);
			data.exceptions(std::stringstream::failbit | std::stringstream::badbit);

			decompressData(in_stream, data, uncompressed_data_size);

			std::vector<LeafNode*> nodes = getNodes(data);
			readNodes(data, header, nodes);
		} else {
			std::vector<LeafNode*> nodes = getNodes(in_stream);
			readNodes(in_stream, header, nodes);
		}

		if (propagate) {
			updateModifiedNodes();
		}
	}

	void write(std::filesystem::path const& filename, Depth min_depth = 0,
	           bool compress = false, int compression_acceleration_level = 1,
	           int compression_level = 0) const
	{
		write(filename, predicate::TRUE(), min_depth, compress,
		      compression_acceleration_level, compression_level);
	}

	void write(std::ostream& out_stream, Depth min_depth = 0, bool compress = false,
	           int compression_acceleration_level = 1, int compression_level = 0) const
	{
		write(out_stream, predicate::TRUE(), min_depth, compress,
		      compression_acceleration_level, compression_level);
	}

	template <class Predicates, typename = std::enable_if_t<!std::is_scalar_v<Predicates>>>
	void write(std::filesystem::path const& filename, Predicates const& predicates,
	           Depth min_depth = 0, bool compress = false,
	           int compression_acceleration_level = 1, int compression_level = 0) const
	{
		std::ofstream file;
		file.exceptions(std::ofstream::failbit | std::ofstream::badbit);

		file.open(filename, std::ios_base::out | std::ios_base::binary);

		write(file, predicates, min_depth, compress, compression_acceleration_level,
		      compression_level);
	}

	template <class Predicates, typename = std::enable_if_t<!std::is_scalar_v<Predicates>>>
	void write(std::ostream& out_stream, Predicates const& predicates, Depth min_depth = 0,
	           bool compress = false, int compression_acceleration_level = 1,
	           int compression_level = 0) const
	{
		writeHeader(out_stream, getFileInfo());
		writeData(out_stream, predicates, min_depth, compress, compression_acceleration_level,
		          compression_level);
	}

 protected:
	//
	// Constructors
	//

	OctreeBase(double resolution = 0.1, Depth depth_levels = 16,
	           bool automatic_pruning = true)
	    : depth_levels_(depth_levels),
	      max_value_(std::pow(2, depth_levels - 1)),
	      automatic_pruning_enabled_(automatic_pruning)
	{
		// FIXME: Should this be first?
		setResolutionAndDepthLevels(resolution, depth_levels);

		init();
	}

	OctreeBase(OctreeBase const& other)
	    : depth_levels_(other.depth_levels_),
	      max_value_(other.max_value_),
	      node_size_(other.node_size_),
	      node_size_factor_(other.node_size_factor_),
	      automatic_pruning_enabled_(other.automatic_pruning_enabled_),
	      parallel_depth_(other.parallel_depth_)
	{
		printf("OctreeBase copy constructor\n");
		// TODO: Implement

		// num_inner_nodes_ = other.num_inner_nodes_;
		// num_inner_leaf_nodes_ = other.num_inner_leaf_nodes_;
		// num_leaf_nodes_ = other.num_leaf_nodes_;

		init();
	}

	template <class D1, class D2, class I>
	OctreeBase(OctreeBase<D1, D2, I> const& other)
	    : depth_levels_(other.depth_levels_),
	      max_value_(other.max_value_),
	      node_size_(other.node_size_),
	      node_size_factor_(other.node_size_factor_),
	      automatic_pruning_enabled_(other.automatic_pruning_enabled_),
	      parallel_depth_(other.parallel_depth_)
	{
		printf("OctreeBase template copy constructor\n");
		// TODO: Implement

		// num_inner_nodes_ = other.num_inner_nodes_;
		// num_inner_leaf_nodes_ = other.num_inner_leaf_nodes_;
		// num_leaf_nodes_ = other.num_leaf_nodes_;

		init();
	}

	OctreeBase(OctreeBase&& other)
	    : depth_levels_(std::move(other.depth_levels_)),
	      max_value_(std::move(other.max_value_)),
	      root_(std::move(other.root_)),
	      node_size_(std::move(other.node_size_)),
	      node_size_factor_(std::move(other.node_size_factor_)),
	      automatic_pruning_enabled_(std::move(other.automatic_pruning_enabled_)),
	      parallel_depth_(std::move(other.parallel_depth_))
	{
		printf("OctreeBase move constructor\n");

		num_inner_nodes_.store(other.num_inner_nodes_);
		num_inner_leaf_nodes_.store(other.num_inner_leaf_nodes_);
		num_leaf_nodes_.store(other.num_leaf_nodes_);

		num_allocated_inner_nodes_.store(other.num_allocated_inner_nodes_);
		num_allocated_inner_leaf_nodes_.store(other.num_allocated_inner_leaf_nodes_);
		num_allocated_leaf_nodes_.store(other.num_allocated_leaf_nodes_);

		init();
	}

	//
	// Init
	//

	void init()
	{
		size_t i = 0;
		for (auto& a_l : children_locks_) {
			for (auto& l : a_l) {
				l.clear();
			}
		}
	}

	//
	// Set resolution and depth levels
	//

	void setResolutionAndDepthLevels(double const resolution, Depth const depth_levels)
	{
		if (minDepthLevels() > depth_levels || maxDepthLevels() < depth_levels) {
			throw std::invalid_argument("depth_levels can be minimum " +
			                            std::to_string(+minDepthLevels()) + " and maximum " +
			                            std::to_string(+maxDepthLevels()) + ", '" +
			                            std::to_string(+depth_levels) + "' was supplied.");
		}

		depth_levels_ = depth_levels;
		max_value_ = std::pow(2, depth_levels - 1);

		std::generate(std::begin(node_size_), std::end(node_size_),
		              [n = resolution]() mutable {
			              auto c = n;
			              n *= 2;
			              return c;
		              });

		std::transform(std::begin(node_size_), std::end(node_size_),
		               std::begin(node_size_factor_), [](auto n) { return 1 / n; });
	}

	OctreeBase& operator=(OctreeBase const& rhs)
	{
		printf("OctreeBase copy assignment\n");

		// TODO: Should this clear?
		clear(rhs.resolution(), rhs.depthLevels());

		depth_levels_ = rhs.depth_levels_;
		max_value_ = rhs.max_value_;
		node_size_ = rhs.node_size_;
		node_size_factor_ = rhs.node_size_factor_;
		automatic_pruning_enabled_ = rhs.automatic_pruning_enabled_;
		parallel_depth_ = rhs.parallel_depth_;
		// num_inner_nodes_.store(rhs.num_inner_nodes_);
		// num_inner_leaf_nodes_.store(rhs.num_inner_leaf_nodes_);
		// num_leaf_nodes_.store(rhs.num_leaf_nodes_);
		return *this;
	}

	template <class D1, class D2, class I>
	OctreeBase& operator=(OctreeBase<D1, D2, I> const& rhs)
	{
		printf("OctreeBase template copy assignment\n");

		// TODO: Should this clear?
		clear(rhs.resolution(), rhs.depthLevels());

		depth_levels_ = rhs.depth_levels_;
		max_value_ = rhs.max_value_;
		node_size_ = rhs.node_size_;
		node_size_factor_ = rhs.node_size_factor_;
		automatic_pruning_enabled_ = rhs.automatic_pruning_enabled_;
		parallel_depth_ = rhs.parallel_depth_;
		// num_inner_nodes_.store(rhs.num_inner_nodes_);
		// num_inner_leaf_nodes_.store(rhs.num_inner_leaf_nodes_);
		// num_leaf_nodes_.store(rhs.num_leaf_nodes_);
		return *this;
	}

	OctreeBase& operator=(OctreeBase&& rhs)
	{
		printf("OctreeBase move assignment\n");

		// TODO: Should this clear?
		clear(rhs.resolution(), rhs.depthLevels(), true);

		depth_levels_ = std::move(rhs.depth_levels_);
		max_value_ = std::move(rhs.max_value_);
		root_ = std::move(rhs.root_);
		node_size_ = std::move(rhs.node_size_);
		node_size_factor_ = std::move(rhs.node_size_factor_);
		automatic_pruning_enabled_ = std::move(rhs.automatic_pruning_enabled_);
		parallel_depth_ = std::move(rhs.parallel_depth_);
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

	virtual ~OctreeBase()
	{
		if (root_) {
			deleteChildren(getRootImpl(), depthLevels(), true);
		}
	}

	//
	// Initilize root
	//

	virtual void initRoot()
	{
		printf("OctreeBase initRoot\n");
		if (!root_) {
			root_ = std::make_unique<InnerNode>();
		}
		root_->is_leaf = true;
		root_->modified = false;
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
	constexpr Key::KeyType toKey(coord_t coord, Depth depth = 0) const noexcept
	{
		Key::KeyType val = std::floor(node_size_factor_[0] * coord);
		return ((val + max_value_) >> depth) << depth;
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a key with bounds check.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The corresponding key.
	 */
	constexpr std::optional<Key::KeyType> toKeyChecked(coord_t coord,
	                                                   Depth depth = 0) const noexcept
	{
		auto min = -nodeSize(depthLevels() - 1);
		auto max = -min;
		return min <= coord && max >= coord ? std::optional<Key::KeyType>(toKey(coord, depth))
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
	constexpr coord_t toCoord(Key::KeyType key, Depth depth = 0) const noexcept
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
		//            : (((key >> depth) << depth) - max_value_) * resolution_;
	}

	//
	// Traverse
	//

	template <class UnaryFunction>
	void traverseRecurs(UnaryFunction f, Node const& node)
	{
		if (isLeaf(node)) {
			return;
		}

		for (size_t i = 0; i != 8; ++i) {
			Node child = nodeChild(node, i);
			if (!f(child)) {
				traverseRecurs(f, child);
			}
		}
	}

	template <class UnaryFunction>
	void traverseRecurs(UnaryFunction f, Node const& node) const
	{
		if (isLeaf(node)) {
			return;
		}

		for (size_t i = 0; i != 8; ++i) {
			Node child = nodeChild(node, i);
			if (!f(child)) {
				traverseRecurs(f, child);
			}
		}
	}

	//
	// Apply
	//

	template <class UnaryFunction,
	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction>>>
	void apply(Node& node, UnaryFunction f, bool propagate)
	{
		if (isLeaf(node)) {
			f(leafNode(node));
		} else {
			applyAllRecurs(innerNode(node), node.depth(), f);
		}

		if (!isModified(node)) {
			// Update all parents
			setModifiedParents(node);

			leafNode(node).modified = true;
		}

		if (propagate) {
			updateModifiedNodes();
		}
	}

	template <class ExecutionPolicy, class UnaryFunction,
	          typename = std::enable_if_t<
	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>,
	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction>>>
	void apply(ExecutionPolicy policy, Node& node, UnaryFunction f, bool propagate)
	{
		if (isLeaf(node)) {
			f(leafNode(node));
		} else {
			applyAllRecurs(policy, innerNode(node), node.depth(), f);
		}

		if (!isModified(node)) {
			// Update all parents
			setModifiedParents(node);

			leafNode(node).modified = true;
		}

		if (propagate) {
			updateModifiedNodes(policy);
		}
	}

	template <class UnaryFunction,
	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction>>>
	void apply(Code code, UnaryFunction f, bool propagate)
	{
		// FIXME: Should this be here?
		if (code.depth() > depthLevels()) {
			return;
		}

		applyRecurs(getRootImpl(), depthLevels(), code, f);

		if (propagate) {
			updateModifiedNodes();
		}
	}

	template <class ExecutionPolicy, class UnaryFunction,
	          typename = std::enable_if_t<
	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>,
	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction>>>
	void apply(ExecutionPolicy policy, Code code, UnaryFunction f, bool propagate)
	{
		// FIXME: Should this be here?
		if (code.depth() > depthLevels()) {
			return;
		}

		applyRecurs(policy, getRootImpl(), depthLevels(), code, f);

		if (propagate) {
			updateModifiedNodes(policy);
		}
	}

	template <class UnaryFunction,

	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction>>>
	void applyRecurs(InnerNode& node, Depth depth, Code code, UnaryFunction f)
	{
		if (code.depth() == depth) {
			if (isLeaf(node)) {
				f(static_cast<LeafNode&>(node));
			} else {
				applyAllRecurs(node, depth, f);
			}
		} else if (1 == depth) {
			createLeafChildren(node, code.indexAtDepth(depth));
			LeafNode& child = getLeafChild(node, code.indexAtDepth(0));
			f(child);
			child.modified = true;
		} else {
			createInnerChildren(node, depth, code.indexAtDepth(depth));
			applyRecurs(getInnerChild(node, code.indexAtDepth(depth - 1)), depth - 1, code, f);
		}

		node.modified = true;
	}

	template <class ExecutionPolicy, class UnaryFunction,
	          typename = std::enable_if_t<
	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>,
	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction>>>
	void applyRecurs(ExecutionPolicy policy, InnerNode& node, Depth depth, Code code,
	                 UnaryFunction f)
	{
		if (code.depth() == depth) {
			if (isLeaf(node)) {
				f(static_cast<LeafNode&>(node));
			} else {
				applyAllRecurs(policy, node, depth, f);
			}
		} else if (1 == depth) {
			createLeafChildren(node, code.indexAtDepth(depth));
			LeafNode& child = getLeafChild(node, code.indexAtDepth(0));
			f(child);
			child.modified = true;
		} else {
			createInnerChildren(node, depth, code.indexAtDepth(depth));
			applyRecurs(policy, getInnerChild(node, code.indexAtDepth(depth - 1)), depth - 1,
			            code, f);
		}

		node.modified = true;
	}

	template <class UnaryFunction,
	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction>>>
	void applyAllRecurs(InnerNode& node, Depth depth, UnaryFunction f)
	{
		if (1 == depth) {
			for (LeafNode& child : getLeafChildren(node)) {
				f(child);
				child.modified = true;
			}
		} else {
			for (InnerNode& child : getInnerChildren(node)) {
				if (isLeaf(child)) {
					f(static_cast<LeafNode&>(child));
				} else {
					applyAllRecurs(child, depth - 1, f);
				}
				child.modified = true;
			}
		}
	}

	template <class ExecutionPolicy, class UnaryFunction,
	          typename = std::enable_if_t<
	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>,
	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction>>>
	void applyAllRecurs(ExecutionPolicy policy, InnerNode& node, Depth depth,
	                    UnaryFunction f)
	{
		if (1 == depth) {
			std::for_each(policy, std::begin(getLeafChildren(node)),
			              std::end(getLeafChildren(node)), [&f](LeafNode& child) {
				              f(child);
				              child.modified = true;
			              });
		} else {
			std::for_each(policy, std::begin(getInnerChildren(node)),
			              std::end(getInnerChildren(node)), [&](InnerNode& child) {
				              if (isLeaf(child)) {
					              f(static_cast<LeafNode&>(child));
				              } else {
					              applyAllRecurs(child, depth - 1, f);
				              }
				              child.modified = true;
			              });
		}
	}

	//
	// Get root impl
	//

	constexpr InnerNode const& getRootImpl() const noexcept { return *root_; }

	constexpr InnerNode& getRootImpl() noexcept { return *root_; }

	//
	// Get node
	//

	static constexpr LeafNode const& leafNode(Node const& node)
	{
		return *static_cast<LeafNode const*>(node.data());
	}

	static constexpr LeafNode& leafNode(Node& node)
	{
		return *static_cast<LeafNode*>(node.data());
	}

	LeafNode const& leafNode(Code code) const
	{
		return leafNodeRecurs(getRootImpl(), depthLevels(), code);
	}

	LeafNode& leafNode(Code code)
	{
		return const_cast<LeafNode&>(std::as_const(*this).leafNode(code));
	}

	LeafNode const& leafNodeRecurs(InnerNode const& node, Depth depth, Code code) const
	{
		if (code.depth() == depth || isLeaf(node)) {
			return node;
		} else if (1 == depth) {
			return getLeafChild(node, code.indexAtDepth(0));
		} else {
			return leafNodeRecurs(getInnerChild(node, code.indexAtDepth(depth - 1)), depth - 1,
			                      code);
		}
	}

	static constexpr InnerNode const& innerNode(Node const& node)
	{
		return static_cast<InnerNode const&>(leafNode(node));
	}

	static constexpr InnerNode& innerNode(Node& node)
	{
		return static_cast<InnerNode&>(leafNode(node));
	}

	InnerNode const& innerNode(Code code) const
	{
		return static_cast<InnerNode const&>(leafNode(code));
	}

	InnerNode& innerNode(Code code)
	{
		return const_cast<InnerNode&>(std::as_const(*this).innerNode(code));
	}

	//
	// Delete node
	//

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void deleteNodeChildrenRecurs(ExecutionPolicy policy, InnerNode& node, Depth depth,
	                              Code code)
	{
		// FIXME: What happens when no children?
		if (isLeaf(node)) {
			return;
		}

		if (code.depth() == depth) {
			deleteChildren(policy, node, depth);
		} else if (1 == depth) {
			deleteLeafChildren(node);
		} else {
			deleteNodeChildrenRecurs(policy, getInnerChild(node, code.indexAtDepth(depth - 1)),
			                         depth - 1, code);
		}

		node.modified = true;
	}

	//
	// (Un)lock children
	//

	bool tryLockChildren(Depth depth, size_t index)
	{
		return !children_locks_[depth][index].test_and_set(std::memory_order_acquire);
	}

	void lockChildren(Depth depth, size_t index)
	{
		while (!tryLockChildren(depth, index))
			;
	}

	bool lockIfLeaf(InnerNode const& node, Depth depth, size_t index)
	{
		do {
			if (isParent(node)) {
				return false;
			}
		} while (!tryLockChildren(depth, index));

		if (isParent(node)) {
			unlockChildren(depth, index);
			return false;
		}

		return true;
	}

	void unlockChildren(Depth depth, size_t index)
	{
		children_locks_[depth][index].clear(std::memory_order_release);
	}

	//
	// Create children
	//

	bool createLeafChildren(InnerNode& node, size_t index)
	{
		if (!lockIfLeaf(node, 0, index)) {
			return false;
		}

		// if (isParent(node)) {
		// 	return false;
		// }

		if (!node.children) {
			// Allocate children
			node.children = new std::array<LeafNode, 8>();
			num_allocated_leaf_nodes_ += 8;
			num_allocated_inner_leaf_nodes_ -= 1;
			num_allocated_inner_nodes_ += 1;
		}
		num_leaf_nodes_ += 8;
		num_inner_leaf_nodes_ -= 1;
		num_inner_nodes_ += 1;

		getLeafChildren(node).fill(static_cast<LeafNode&>(node));

		node.is_leaf = false;
		unlockChildren(0, index);
		return true;
	}

	bool createInnerChildren(InnerNode& node, Depth depth, size_t index)
	{
		if (!lockIfLeaf(node, depth, index)) {
			return false;
		}

		// if (isParent(node)) {
		// 	return false;
		// }

		if (!node.children) {
			// Allocate children
			node.children = new std::array<InnerNode, 8>();
			// Get 8 new and 1 is made into a inner node
			num_allocated_inner_leaf_nodes_ += 7;
			num_allocated_inner_nodes_ += 1;
		}
		num_inner_leaf_nodes_ += 7;
		num_inner_nodes_ += 1;

		for (InnerNode& child : getInnerChildren(node)) {
			static_cast<LeafNode&>(child) = static_cast<LeafNode&>(node);
		}

		node.is_leaf = false;
		unlockChildren(depth, index);
		return true;
	}

	bool createChildren(InnerNode& node, Depth depth, size_t index)
	{
		return (1 == depth) ? createLeafChildren(node, index)
		                    : createInnerChildren(node, depth, index);
	}

	//
	// Delete children
	//

	void deleteLeafChildren(InnerNode& node, bool manual_pruning = false)
	{
		// FIXME: Do I need to take lock here?

		if (!node.is_leaf) {
			num_leaf_nodes_ -= 8;
			num_inner_leaf_nodes_ += 1;
			num_inner_nodes_ -= 1;
		}

		node.is_leaf = true;

		if (nullptr == node.children || (!manual_pruning && !automaticPruning())) {
			return;
		}

		delete &getLeafChildren(node);
		num_allocated_leaf_nodes_ -= 8;
		num_allocated_inner_leaf_nodes_ += 1;
		num_allocated_inner_nodes_ -= 1;
		node.children = nullptr;
	}

	void deleteChildren(InnerNode& node, Depth depth, bool manual_pruning = false)
	{
		// FIXME: Do I need to take lock here?
		if (1 == depth) {
			deleteLeafChildren(node, manual_pruning);
			return;
		}

		if (!node.is_leaf) {
			num_inner_leaf_nodes_ -= 7;
			num_inner_nodes_ -= 1;
		}

		node.is_leaf = true;

		if (nullptr == node.children || (!manual_pruning && !automaticPruning())) {
			return;
		}

		for (InnerNode& child : getInnerChildren(node)) {
			deleteChildren(child, depth - 1, true);
		}

		delete &getInnerChildren(node);
		// Remove 8 and 1 inner node is made into a inner leaf node
		num_allocated_inner_leaf_nodes_ -= 7;
		num_allocated_inner_nodes_ -= 1;
		node.children = nullptr;
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void deleteChildren(ExecutionPolicy policy, InnerNode& node, Depth depth,
	                    bool manual_pruning = false)
	{
		// FIXME: Do I need to take lock here?
		if (1 == depth) {
			deleteLeafChildren(node, manual_pruning);
			return;
		}

		if (!node.is_leaf) {
			num_inner_leaf_nodes_ -= 7;
			num_inner_nodes_ -= 1;
		}

		node.is_leaf = true;

		if (nullptr == node.children || (!manual_pruning && !automaticPruning())) {
			return;
		}

		// Manual pruning is true in case automatic_pruning_enabled_ changes between
		// calls
		if (parallelExecutionDepth() == depth) {
			std::for_each(
			    policy, std::begin(getInnerChildren(node)), std::end(getInnerChildren(node)),
			    [this, depth](InnerNode& child) { deleteChildren(child, depth - 1, true); });
		} else {
			for (InnerNode& child : getInnerChildren(node)) {
				deleteChildren(policy, child, depth - 1, true);
			}
		}

		delete &getInnerChildren(node);
		// Remove 8 and 1 inner node is made into a inner leaf node
		num_allocated_inner_leaf_nodes_ -= 7;
		num_allocated_inner_nodes_ -= 1;
		node.children = nullptr;
	}

	//
	// Get children
	//

	static constexpr std::array<LeafNode, 8>& getLeafChildren(InnerNode const& node)
	{
		return *static_cast<std::array<LeafNode, 8>*>(node.children);
	}

	static constexpr std::array<InnerNode, 8>& getInnerChildren(InnerNode const& node)
	{
		return *static_cast<std::array<InnerNode, 8>*>(node.children);
	}

	static constexpr LeafNode& getLeafChild(InnerNode const& node, unsigned int const idx)
	{
		return getLeafChildren(node)[idx];
	}

	static constexpr InnerNode& getInnerChild(InnerNode const& node, unsigned int const idx)
	{
		return getInnerChildren(node)[idx];
	}

	static constexpr LeafNode& getChild(InnerNode const& node, Depth const child_depth,
	                                    unsigned int const idx)
	{
		return 0 == child_depth ? getLeafChild(node, idx) : getInnerChild(node, idx);
	}

	//
	// Get child center
	//

	static Point3 childCenter(Point3 parent_center, float child_half_size,
	                          unsigned int child_idx)
	{
		parent_center[0] += ((child_idx & 1U) ? child_half_size : -child_half_size);
		parent_center[1] += ((child_idx & 2U) ? child_half_size : -child_half_size);
		parent_center[2] += ((child_idx & 4U) ? child_half_size : -child_half_size);
		return parent_center;
	}

	//
	// Get sibling center
	//

	static Point3 siblingCenter(Point3 sibling_center, float half_size,
	                            unsigned int sibling_idx, unsigned int new_idx)
	{
		sibling_center[0] += ((sibling_idx & 1U) ? -half_size : half_size);
		sibling_center[1] += ((sibling_idx & 2U) ? -half_size : half_size);
		sibling_center[2] += ((sibling_idx & 4U) ? -half_size : half_size);
		sibling_center[0] += ((new_idx & 1U) ? half_size : -half_size);
		sibling_center[1] += ((new_idx & 2U) ? half_size : -half_size);
		sibling_center[2] += ((new_idx & 4U) ? half_size : -half_size);
		return sibling_center;
	}

	//
	// Get parent center
	//

	static Point3 getParentCenter(Point3 child_center, float child_half_size,
	                              unsigned int child_idx)
	{
		child_center[0] -= ((child_idx & 1U) ? child_half_size : -child_half_size);
		child_center[1] -= ((child_idx & 2U) ? child_half_size : -child_half_size);
		child_center[2] -= ((child_idx & 4U) ? child_half_size : -child_half_size);
		return child_center;
	}

	//
	// Is leaf
	//

	static constexpr bool isLeaf(LeafNode const& node) noexcept { return node.is_leaf; }

	//
	// Is parent
	//

	static constexpr bool isParent(LeafNode const& node) noexcept { return !isLeaf(node); }

	//
	// Is modified
	//

	static constexpr bool isModified(LeafNode const& node) noexcept
	{
		return node.modified;
	}

	//
	// Is node collapsible
	//

	// If all children are the same as the parent they can be pruned
	// NOTE: Only call with nodes that have children
	static bool isNodeCollapsible(InnerNode const& node, Depth depth) noexcept
	{
		if (1 == depth) {
			return std::all_of(std::cbegin(getLeafChildren(node)),
			                   std::cend(getLeafChildren(node)),
			                   [&node](LeafNode const& child) {
				                   return static_cast<LeafNode const&>(node) == child;
			                   });
		} else {
			return std::all_of(std::cbegin(getInnerChildren(node)),
			                   std::cend(getInnerChildren(node)),
			                   [](InnerNode const& child) { return isLeaf(child); }) &&
			       std::all_of(std::cbegin(getInnerChildren(node)),
			                   std::cend(getInnerChildren(node)),
			                   [&node](InnerNode const& child) {
				                   return static_cast<LeafNode const&>(node) ==
				                          static_cast<LeafNode const&>(child);
			                   });
		}
	}

	//
	// Update nodes
	//

	template <bool KeepModified>
	void updateModifiedNodesRecurs(Depth max_depth, InnerNode& node, Depth depth)
	{
		if (isLeaf(node)) {
			if (depth <= max_depth) {
				updateNodeIndicators(node);
				if constexpr (!KeepModified) {
					node.modified = false;
				}
			}
			return;
		}

		if (1 == depth) {
			for (auto& child : getLeafChildren(node)) {
				if (isModified(child)) {
					updateNodeIndicators(child);
					if constexpr (!KeepModified) {
						child.modified = false;
					}
				}
			}
		} else {
			for (auto& child : getInnerChildren(node)) {
				if (isModified(child)) {
					updateModifiedNodesRecurs<KeepModified>(max_depth, child, depth - 1);
				}
			}
		}

		if (depth <= max_depth) {
			updateNode(node, depth);
			updateNodeIndicators(node, depth);
			pruneNode(node, depth);
			if constexpr (!KeepModified) {
				node.modified = false;
			}
		}
	}

	template <bool KeepModified, class ExecutionPolicy,
	          typename = std::enable_if_t<
	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
	void updateModifiedNodesRecurs(ExecutionPolicy policy, Depth max_depth, InnerNode& node,
	                               Depth depth)
	{
		if (!isModified(node)) {
			return;
		}

		if (isParent(node)) {
			if (1 == depth) {
				for (auto& child : getLeafChildren(node)) {
					updateNodeIndicators(child);
					if constexpr (!KeepModified) {
						child.modified = false;
					}
				}
			} else {
				if (parallelExecutionDepth() == depth) {
					std::for_each(
					    policy, std::begin(getInnerChildren(node)),
					    std::end(getInnerChildren(node)), [this, max_depth, depth](auto& child) {
						    // Sequential execution for rest in case parallel depth changes
						    // in between
						    updateModifiedNodesRecurs<KeepModified>(max_depth, child, depth - 1);
					    });
				} else {
					for (auto& child : getInnerChildren(node)) {
						updateModifiedNodesRecurs<KeepModified>(policy, max_depth, child, depth - 1);
					}
				}
			}
		}

		if (depth <= max_depth) {
			if (isParent(node)) {
				updateNode(node, depth);
				updateNodeIndicators(node, depth);
				pruneNode(node, depth);
			} else {
				updateNodeIndicators(node);
			}
			if constexpr (!KeepModified) {
				node.modified = false;
			}
		}
	}

	virtual void updateNode(InnerNode& node, Depth depth) = 0;

	virtual void updateNodeIndicators(LeafNode& node) = 0;

	virtual void updateNodeIndicators(InnerNode& node, Depth depth) = 0;

	//
	// Set parents modified
	//

	void setModifiedParents(Node& node)
	{
		if (node.depth() >= depthLevels()) {
			return;
		}
		setModifiedParentsRecurs(getRootImpl(), depthLevels(), node.code());
	}

	// NOTE: Assumes code has depth higher then depth
	void setModifiedParentsRecurs(InnerNode& node, Depth depth, Code code)
	{
		node.modified = true;
		if (code.depth() < depth - 1) {
			setModifiedParentsRecurs(getInnerChild(node, code.indexAtDepth(depth - 1)),
			                         depth - 1, code);
		}
	}

	//
	// Set modified
	//

	void setModifiedRecurs(InnerNode& node, Depth depth, Depth min_depth)
	{
		node.modified = true;

		if (isLeaf(node) || depth == min_depth) {
			return;
		}

		if (1 == depth) {
			for (auto& child : getLeafChildren(node)) {
				child.modified = true;
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

	void clearModifiedRecurs(InnerNode& node, Depth depth, Depth max_depth)
	{
		if (!isModified(node)) {
			// Already clear
			return;
		}

		node.modified = depth > max_depth;

		if (isLeaf(node)) {
			return;
		}

		if (1 == depth) {
			for (auto& child : getLeafChildren(node)) {
				child.modified = false;
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
	bool pruneNode(InnerNode& node, Depth depth)
	{
		if (isNodeCollapsible(node, depth)) {
			deleteChildren(node, depth);
			return true;
		} else {
			return false;
		}
	}

	bool pruneRecurs(InnerNode& node, Depth depth)
	{
		if (isLeaf(node)) {
			return true;
		}

		if (1 == depth) {
			return pruneNode(node, depth);
		}

		bool prunable = std::all_of(
		    std::begin(getInnerChildren(node)), std::end(getInnerChildren(node)),
		    [this, depth](InnerNode& child) { return pruneRecurs(child, depth - 1); });

		return !prunable || pruneNode(node, depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	bool pruneRecurs(ExecutionPolicy policy, InnerNode& node, Depth depth)
	{
		if (isLeaf(node)) {
			return true;
		}

		if (1 == depth) {
			return pruneNode(node, depth);
		}

		bool prunable;
		if (parallelExecutionDepth() == depth) {
			prunable = std::all_of(
			    policy, std::begin(getInnerChildren(node)), std::end(getInnerChildren(node)),
			    [this, depth](InnerNode& child) { return pruneRecurs(child, depth - 1); });
		} else {
			prunable = std::all_of(std::begin(getInnerChildren(node)),
			                       std::end(getInnerChildren(node)),
			                       [this, policy, depth](InnerNode& child) {
				                       return pruneRecurs(policy, child, depth - 1);
			                       });
		}

		return !prunable || pruneNode(node, depth);
	}

	//
	// Input/output (read/write)
	//

	FileInfo getFileInfo() const
	{
		FileInfo info;
		info["resolution"].push_back(std::to_string(resolution()));
		info["depth_levels"].push_back(std::to_string(static_cast<uint32_t>(depthLevels())));
		addFileInfo(info);
		return info;
	}

	virtual void addFileInfo(FileInfo& info) const {}

	std::vector<LeafNode*> getNodes(std::istream& in_stream)
	{
		return getNodes(getIndicators(in_stream));
	}

	std::vector<uint8_t> getIndicators(std::istream& in_stream)
	{
		uint8_t compressed;
		in_stream.read(reinterpret_cast<char*>(&compressed), sizeof(compressed));

		uint64_t uncompressed_data_size;
		in_stream.read(reinterpret_cast<char*>(&uncompressed_data_size),
		               sizeof(uncompressed_data_size));

		if (UINT8_MAX == compressed) {
			std::stringstream data(std::ios_base::in | std::ios_base::out |
			                       std::ios_base::binary);
			data.exceptions(std::stringstream::failbit | std::stringstream::badbit);

			decompressData(in_stream, data, uncompressed_data_size);

			uint64_t num;
			data.read(reinterpret_cast<char*>(&num), sizeof(num));

			std::vector<uint8_t> indicators(num);
			data.read(reinterpret_cast<char*>(indicators.data()),
			          sizeof(typename std::decay_t<decltype(indicators)>::value_type) * num);

			return indicators;
		} else {
			uint64_t num;
			in_stream.read(reinterpret_cast<char*>(&num), sizeof(num));

			std::vector<uint8_t> indicators(num);
			in_stream.read(
			    reinterpret_cast<char*>(indicators.data()),
			    sizeof(typename std::decay_t<decltype(indicators)>::value_type) * num);

			return indicators;
		}
	}

	std::vector<LeafNode*> getNodes(std::vector<uint8_t> const& indicators)
	{
		uint64_t total_nodes = 0;
		for (size_t i = 0; i < indicators.size(); i += 2) {
			for (size_t b = 0; 8 != b; ++b) {
				total_nodes += (indicators[i] >> b) & 1U ? 1 : 0;
			}
		}

		std::vector<LeafNode*> nodes(total_nodes);

		bool valid_return = 0 != indicators[0];
		bool valid_inner = 0 != indicators[1];

		if (valid_return) {
			nodes[0] = static_cast<LeafNode*>(&getRootImpl());
			getRootImpl().modified = true;
		} else if (valid_inner) {
			size_t idx = 2;
			auto nodes_it = nodes.begin();
			getNodesRecurs(indicators, idx, nodes_it, getRoot());
		}

		return nodes;
	}

	void getNodesRecurs(std::vector<uint8_t> const& indicators, size_t& idx,
	                    typename std::vector<LeafNode*>::iterator& nodes_it, Node node)
	{
		uint8_t child_valid_return = indicators[idx++];
		uint8_t child_valid_inner = indicators[idx++];

		if (0 == child_valid_return && 0 == child_valid_inner) {
			return;
		}

		InnerNode& inner_node = innerNode(node);
		inner_node.modified = true;

		if (1 == node.depth()) {
			createLeafChildren(inner_node, node.code().indexAtDepth(1));

			auto child = nodeChild(node, 0);
			for (size_t i = 0; i != 8; ++i) {
				if ((child_valid_return >> i) & 1U) {
					child = nodeSibling(child, i);
					*nodes_it++ = &leafNode(child);
				}
			}
		} else {
			createInnerChildren(inner_node, node.depth(),
			                    node.code().indexAtDepth(node.depth()));

			auto child = nodeChild(node, 0);
			for (size_t i = 0; i != 8; ++i) {
				if ((child_valid_return >> i) & 1U) {
					child = nodeSibling(child, i);
					*nodes_it++ = &leafNode(child);
				} else if ((child_valid_inner >> i) & 1U) {
					child = nodeSibling(child, i);
					getNodesRecurs(indicators, idx, nodes_it, child);
				}
			}
		}
	}

	virtual void readNodes(std::istream& in_stream, FileInfo const& header,
	                       std::vector<LeafNode*> const& nodes)
	{
		size_t num_fields = header.at("fields").size();
		if (header.at("size").size() != num_fields ||
		    header.at("type").size() != num_fields) {
			return;
		}

		for (size_t i = 0; i != header.at("fields").size(); ++i) {
			std::string field = header.at("fields")[i];
			uint64_t size = static_cast<uint64_t>(std::stoi(header.at("size")[i]));
			char type = header.at("type")[i][0];

			uint64_t num;
			in_stream.read(reinterpret_cast<char*>(&num), sizeof(num));

			if (!readNodes(in_stream, nodes, field, type, size, num)) {
				// Skip forward
				in_stream.seekg(size * num, std::istream::cur);
			}
		}
	}

	virtual bool readNodes(std::istream& in_stream, std::vector<LeafNode*> const& nodes,
	                       std::string const& field, char type, uint64_t size,
	                       uint64_t num) = 0;

	template <class Predicates>
	void writeData(std::ostream& out_stream, Predicates const& predicates, Depth min_depth,
	               bool compress, int compression_acceleration_level,
	               int compression_level) const
	{
		uint8_t compressed = compress ? UINT8_MAX : 0U;
		out_stream.write(reinterpret_cast<char*>(&compressed), sizeof(compressed));

		auto size_pos = out_stream.tellp();
		uint64_t uncompressed_data_size = 0;
		out_stream.write(reinterpret_cast<char*>(&uncompressed_data_size),
		                 sizeof(uncompressed_data_size));

		auto data_pos = out_stream.tellp();

		auto [indicators, nodes] = data(predicate::Leaf(min_depth) && predicates);

		if (compress) {
			std::stringstream data(std::ios_base::in | std::ios_base::out |
			                       std::ios_base::binary);
			data.exceptions(std::stringstream::failbit | std::stringstream::badbit);

			writeIndicators(data, indicators, compress, compression_acceleration_level,
			                compression_level);
			writeNodes(data, nodes, compress, compression_acceleration_level,
			           compression_level);

			uncompressed_data_size = data.tellp();

			compressData(data, out_stream, uncompressed_data_size,
			             compression_acceleration_level, compression_level);
		} else {
			writeIndicators(out_stream, indicators, compress, compression_acceleration_level,
			                compression_level);
			writeNodes(out_stream, nodes, compress, compression_acceleration_level,
			           compression_level);

			uncompressed_data_size = out_stream.tellp() - data_pos;
		}

		auto end_pos = out_stream.tellp();

		out_stream.seekp(size_pos);
		out_stream.write(reinterpret_cast<char*>(&uncompressed_data_size),
		                 sizeof(uncompressed_data_size));

		out_stream.seekp(end_pos);
	}

	template <class Predicates>
	std::pair<std::vector<uint8_t>, std::vector<LeafNode const*>> data(
	    Predicates const& predicates) const
	{
		std::vector<uint8_t> indicators;
		std::vector<LeafNode const*> nodes;

		auto root = getRoot();

		Derived const* derived = dynamic_cast<Derived const*>(this);

		bool valid_return =
		    predicate::PredicateValueCheck<Predicates>::apply(predicates, *derived, root);
		bool valid_inner = !valid_return && predicate::PredicateInnerCheck<Predicates>::apply(
		                                        predicates, *derived, root);

		indicators.push_back(valid_return ? UINT8_MAX : 0U);
		indicators.push_back(valid_inner ? UINT8_MAX : 0U);

		if (valid_return) {
			nodes.push_back(&leafNode(root));
		} else if (valid_inner) {
			dataRecurs(indicators, nodes, predicates, derived, root);
			if (nodes.empty()) {
				//  Nothing was added
				indicators.clear();
			}
		}

		return {indicators, nodes};
	}

	template <class Predicates>
	void dataRecurs(std::vector<uint8_t>& indicators, std::vector<LeafNode const*>& nodes,
	                Predicates const& predicates, Derived const* derived,
	                Node const& node) const
	{
		auto cur_indicators_size = indicators.size();
		auto cur_nodes_size = nodes.size();

		auto child = nodeChild(node, 0);

		uint8_t child_valid_return = 0;
		uint8_t child_valid_inner = 0;
		for (size_t i = 0; 8 != i; ++i) {
			child = nodeSibling(child, i);

			if (predicate::PredicateValueCheck<Predicates>::apply(predicates, *derived,
			                                                      child)) {
				child_valid_return |= 1U << i;
			} else if (predicate::PredicateInnerCheck<Predicates>::apply(predicates, *derived,
			                                                             child)) {
				child_valid_inner |= 1U << i;
			}
		}

		indicators.push_back(child_valid_return);
		indicators.push_back(child_valid_inner);

		for (size_t i = 0; 8 != i; ++i) {
			if ((child_valid_return >> i) & 1U) {
				child = nodeSibling(child, i);
				nodes.push_back(&leafNode(child));
			} else if ((child_valid_inner >> i) & 1U) {
				child = nodeSibling(child, i);
				dataRecurs(indicators, nodes, predicates, derived, child);
			}
		}

		if (nodes.size() == cur_nodes_size) {
			indicators.resize(cur_indicators_size);
			indicators.push_back(0U);
			indicators.push_back(0U);
		}
	}

	void writeIndicators(std::ostream& out_stream, std::vector<uint8_t> const& indicators,
	                     bool compress, int compression_acceleration_level,
	                     int compression_level) const
	{
		uint8_t compressed = compress ? UINT8_MAX : 0U;
		out_stream.write(reinterpret_cast<char*>(&compressed), sizeof(compressed));

		auto size_pos = out_stream.tellp();
		uint64_t uncompressed_data_size = 0;
		out_stream.write(reinterpret_cast<char*>(&uncompressed_data_size),
		                 sizeof(uncompressed_data_size));

		auto data_pos = out_stream.tellp();

		uint64_t num = indicators.size();
		if (compress) {
			std::stringstream data(std::ios_base::in | std::ios_base::out |
			                       std::ios_base::binary);
			data.exceptions(std::stringstream::failbit | std::stringstream::badbit);

			data.write(reinterpret_cast<char*>(&num), sizeof(num));
			data.write(reinterpret_cast<char const*>(indicators.data()),
			           sizeof(typename std::decay_t<decltype(indicators)>::value_type) * num);

			uncompressed_data_size = data.tellp();

			compressData(data, out_stream, uncompressed_data_size,
			             compression_acceleration_level, compression_level);
		} else {
			out_stream.write(reinterpret_cast<char*>(&num), sizeof(num));
			out_stream.write(
			    reinterpret_cast<char const*>(indicators.data()),
			    sizeof(typename std::decay_t<decltype(indicators)>::value_type) * num);

			uncompressed_data_size = out_stream.tellp() - data_pos;
		}

		auto end_pos = out_stream.tellp();

		out_stream.seekp(size_pos);
		out_stream.write(reinterpret_cast<char*>(&uncompressed_data_size),
		                 sizeof(uncompressed_data_size));

		out_stream.seekp(end_pos);
	}

	virtual void writeNodes(std::ostream& out_stream,
	                        std::vector<LeafNode const*> const& nodes, bool compress,
	                        int compression_acceleration_level,
	                        int compression_level) const = 0;

 protected:
	Depth depth_levels_;      // The number of depth levels
	Key::KeyType max_value_;  // The maximum coordinate value the octree can store

	std::unique_ptr<InnerNode> root_;  // The root of the octree

	// std::array<Blocks<InnerNode>, maxDepthLevels() - 1> inner_nodes_;
	// Blocks<LeafNode> leaf_nodes_;

	// Stores the size of a node at a given depth, where the depth is the index
	std::array<double, maxDepthLevels()> node_size_;
	// Reciprocal of the node size at a given depth, where the depth is the index
	std::array<double, maxDepthLevels()> node_size_factor_;

	// Automatic pruning
	bool automatic_pruning_enabled_ = true;

	// Depth at which parallel execution happens
	Depth parallel_depth_ = 12;

	// Locks to support parallel insertion, one per level and child
	std::array<std::array<std::atomic_flag, 8>, maxDepthLevels() + 1>
	    children_locks_;  // FIXME: Initialize to ATOMIC_FLAG_INIT

	// Memory
	std::atomic_size_t num_inner_nodes_ = 0;       // Current number of inner nodes
	std::atomic_size_t num_inner_leaf_nodes_ = 1;  // Current number of inner leaf nodes
	std::atomic_size_t num_leaf_nodes_ = 0;        // Current number of leaf nodes

	std::atomic_size_t num_allocated_inner_nodes_ = 0;
	std::atomic_size_t num_allocated_inner_leaf_nodes_ = 1;
	std::atomic_size_t num_allocated_leaf_nodes_ = 0;

	template <class D1, class D2, class I>
	friend class OctreeBase;
};

}  // namespace ufo::map

#endif  // UFO_MAP_OCTREE_BASE_H