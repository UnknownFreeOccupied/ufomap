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
#include <ufo/map/iterator/octree.h>
#include <ufo/map/key.h>
#include <ufo/map/octree/memory/pointer.h>
#include <ufo/map/octree/node.h>
#include <ufo/map/octree/octree_indicators.h>
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
// Utilizing Curiously recurring template pattern (CRTP)
template <class Derived, class DataType, class Indicators = OctreeIndicators,
          bool LockLess = false, bool ReuseNodes = false,
          template <typename, typename, bool, bool> typename MemoryModel =
              ufo::map::OctreePointerMemory>
class OctreeBase : public MemoryModel<DataType, Indicators, LockLess, ReuseNodes>
{
 private:
	// Minimum number of depth levels
	static constexpr depth_t MIN_DEPTH_LEVELS = 2;
	// Maximum number of depth levels
	static constexpr depth_t MAX_DEPTH_LEVELS = 21;

	friend Derived;

	using MM = MemoryModel<DataType, Indicators, LockLess, ReuseNodes>;

 public:
	using LeafNode = typename MM::LeafNode;
	using InnerNode = typename MM::InnerNode;

 public:
	using const_iterator = IteratorWrapper<Derived, Node>;
	using const_query_iterator = const_iterator;
	using const_bounding_volume_iterator = IteratorWrapper<Derived, NodeBV>;
	using const_bounding_volume_query_iterator = const_bounding_volume_iterator;
	using const_query_nearest_iterator = IteratorWrapper<Derived, NearestNode>;

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
	void clear(double new_resolution, depth_t new_depth_levels, bool prune = false)
	{
		this->deleteChildren(this->getRoot(), depthLevels(), prune);

		// FIXME: Should this be first?
		setResolutionAndDepthLevels(new_resolution, new_depth_levels);

		derived().initRoot();
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
	void clear(ExecutionPolicy policy, double new_resolution, depth_t new_depth_levels,
	           bool prune = false)
	{
		this->deleteChildren(policy, this->getRoot(), depthLevels(), prune);

		// FIXME: Should this be first?
		setResolutionAndDepthLevels(new_resolution, new_depth_levels);

		derived().initRoot();
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
	[[nodiscard]] constexpr double getNodeSize(depth_t depth) const
	{
		return node_size_[depth];
	}

	[[nodiscard]] double getNodeSize(Node node) const { return getNodeSize(node.depth()); }

	[[nodiscard]] double getNodeSize(NodeBV const& node) const { return node.size(); }

	//
	// Resolution
	//

	/*!
	 * @brief Get the resolution (leaf node size) of the octree.
	 *
	 * @return The resolution (leaf node size) of the octree.
	 */
	[[nodiscard]] constexpr double resolution() const noexcept { return getNodeSize(0); }

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
	[[nodiscard]] constexpr depth_t parallelExecutionDepth() const noexcept
	{
		return parallel_depth_;
	}

	/*!
	 * @brief Set the depth when parallel execution begins.
	 *
	 * @param new_parallel_depth The new depth where parallel execution begins.
	 */
	constexpr void parallelExecutionDepth(depth_t new_parallel_depth) noexcept
	{
		parallel_depth_ = new_parallel_depth;
	}

	//
	// depth_t levels
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
	// Get min/max coordinate octree can store
	//

	/*!
	 * @return The minimum coordinate the octree can store.
	 */
	[[nodiscard]] Point3 getMin() const
	{
		auto half_size = -getNodeSize(depthLevels() - 1);
		return Point3(half_size, half_size, half_size);
	}

	[[nodiscard]] Point3 getMin(Node node) const
	{
		return getCenter(node) - (getNodeSize(node) / 2);
	}

	[[nodiscard]] Point3 getMin(NodeBV const& node) const { return node.min(); }

	/*!
	 * @return The maximum coordinate the octree can store.
	 */
	[[nodiscard]] Point3 getMax() const
	{
		auto half_size = getNodeSize(depthLevels() - 1);
		return Point3(half_size, half_size, half_size);
	}

	[[nodiscard]] Point3 getMax(Node node) const
	{
		return getCenter(node) + (getNodeSize(node) / 2);
	}

	[[nodiscard]] Point3 getMax(NodeBV const& node) const { return node.max(); }

	//
	// Get center
	//

	/*!
	 * @return The center of the octree.
	 */
	[[nodiscard]] Point3 getCenter() const { return Point3(0, 0, 0); }

	/*!
	 * @return The center of the node
	 */
	[[nodiscard]] Point3 getCenter(Node node) const { return toCoord(node.code()); }

	/*!
	 * @return The center of the node
	 */
	[[nodiscard]] Point3 getCenter(NodeBV const& node) const { return node.center(); }

	//
	// Bounding volume
	//

	/*!
	 * @return Minimum bounding volume convering the whole octree.
	 */
	[[nodiscard]] geometry::AAEBB getBoundingVolume() const
	{
		return geometry::AAEBB(getCenter(), getNodeSize(depthLevels() - 1));
	}

	/*!
	 * @return Bounding volume of the node.
	 */
	[[nodiscard]] geometry::AAEBB getBoundingVolume(Node node) const
	{
		return geometry::AAEBB(getCenter(node), getNodeSize(node) / 2);
	}

	/*!
	 * @return Bounding volume of the node.
	 */
	[[nodiscard]] geometry::AAEBB getBoundingVolume(NodeBV const& node) const
	{
		return node.getBoundingVolume();
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
		auto min = -getNodeSize(depthLevels() - 1);
		auto max = -min;
		return min <= x && min <= y && min <= z && max >= x && max >= y && max >= z;
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
	[[nodiscard]] static constexpr bool isPureLeaf(Point3 coord, depth_t depth = 0) noexcept
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
	                                               depth_t depth = 0) noexcept
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
	[[nodiscard]] bool isLeaf(Node node) const
	{
		return isPureLeaf(node) || isLeaf(this->getInnerNode(node));
	}

	/*!
	 * @brief Check if the node corresponding to the code is a leaf node (has no children).
	 *
	 * @param code The code of the node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] bool isLeaf(Code code) const
	{
		return isPureLeaf(code) || isLeaf(getInnerNode(code));
	}

	/*!
	 * @brief Check if the node corresponding to the key is a leaf node (has no children).
	 *
	 * @param key The key of the node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] bool isLeaf(Key key) const
	{
		return isPureLeaf(key) || isLeaf(getInnerNode(toCode(key)));
	}

	/*!
	 * @brief Check if the node corresponding to the coordinate is a leaf node (has no
	 * children).
	 *
	 * @param coord The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] bool isLeaf(Point3 coord, depth_t depth = 0) const
	{
		return isPureLeaf(coord, depth) || isLeaf(getInnerNode(toCode(coord, depth)));
	}

	/*!
	 * @brief Check if the node corresponding to the coordinate is a leaf node (has no
	 * children).
	 *
	 * @param x,y,z The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] bool isLeaf(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return isPureLeaf(x, y, z, depth) || isLeaf(getInnerNode(toCode(x, y, z, depth)));
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
	[[nodiscard]] bool isParent(Node node) const { return !isLeaf(node); }

	/*!
	 * @brief Check if the node corresponding to the code is a parent, i.e., has
	 * children.
	 *
	 * @param code The code of the node to check.
	 * @return Whether the node is a parent.
	 */
	[[nodiscard]] bool isParent(Code code) const { return !isLeaf(code); }

	/*!
	 * @brief Check if the node corresponding to the key is a parent, i.e., has
	 * children.
	 *
	 * @param key The key of the node to check.
	 * @return Whether the node is a parent.
	 */
	[[nodiscard]] bool isParent(Key key) const { return !isLeaf(key); }

	/*!
	 * @brief Check if the node corresponding to the coordinate is a parent, i.e., has
	 * children.
	 *
	 * @param coord The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a parent.
	 */
	[[nodiscard]] bool isParent(Point3 coord, depth_t depth = 0) const
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
	[[nodiscard]] bool isParent(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
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
		std::optional<Key::key_t> key = toKeyChecked(coord, depth);
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
	constexpr Key toKey(Point3 coord, depth_t depth = 0) const noexcept
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
	constexpr Key toKey(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const noexcept
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
	constexpr std::optional<Key> toKeyChecked(Point3 coord,
	                                          depth_t depth = 0) const noexcept
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
	                                          depth_t depth = 0) const noexcept
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

	using MM::getRootNode;

	/*!
	 * @brief Get the root node.
	 *
	 * @return The root node.
	 */
	constexpr Node getRootNode() const { return this->getRootNode(getRootCode()); }

	/*!
	 * @brief Get the root node with bounding volume.
	 *
	 * @return The root node with bounding volume.
	 */
	constexpr NodeBV getRootNodeBV() const
	{
		return NodeBV(getRootNode(), getBoundingVolume());
	}

	/*!
	 * @brief Get the code for the root node.
	 *
	 * @return The root node code.
	 */
	constexpr Code getRootCode() const { return Code(0, depthLevels()); }

	//
	// Create node
	//

	using MM::createNode;

	Node createNode(Code code) { return this->createNode(code, depthLevels()); }

	Node createNode(Key key) { return createNode(toCode(key)); }

	Node createNode(Point3 coord, depth_t depth = 0)
	{
		return createNode(toCode(coord, depth));
	}

	Node createNode(coord_t x, coord_t y, coord_t z, depth_t depth = 0)
	{
		return createNode(toCode(x, y, z, depth));
	}

	//
	// Create bv node
	//

	NodeBV createNodeBV(Code code)
	{
		return NodeBV(createNode(code),
		              geometry::AAEBB(toCoord(code), getNodeSize(code.depth()) / 2));
	}

	NodeBV createNodeBV(Key key) { return createNodeBV(toCode(key)); }

	NodeBV createNodeBV(Point3 coord, depth_t depth = 0)
	{
		return createNodeBV(toCode(coord, depth));
	}

	NodeBV createNodeBV(coord_t x, coord_t y, coord_t z, depth_t depth = 0)
	{
		return createNodeBV(toCode(x, y, z, depth));
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
	Node getNode(Code code) const
	{
		auto const& [node, depth] = getNodeAndDepth(code);
		return Node(const_cast<LeafNode*>(&node), code.toDepth(depth));
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
	Node getNode(Point3 coord, depth_t depth = 0) const
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
	Node getNode(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
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
	std::optional<Node> getNodeChecked(Point3 coord, depth_t depth = 0) const
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
	std::optional<Node> getNodeChecked(coord_t x, coord_t y, coord_t z,
	                                   depth_t depth = 0) const
	{
		return getNodeChecked(Point3(x, y, z), depth);
	}

	//
	// Get bounding volume node
	//

	NodeBV getNodeBV(Code code) const
	{
		Node node = getNode(code);
		code = code.toDepth(node.depth());
		return NodeBV(node, geometry::AAEBB(toCoord(code), getNodeSize(code.depth()) / 2));
	}

	NodeBV getNodeBV(Key key) const { return getNodeBV(toCode(key)); }

	NodeBV getNodeBV(Point3 coord, depth_t depth = 0) const
	{
		return getNodeBV(toCode(coord, depth));
	}

	NodeBV getNodeBV(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return getNodeBV(toCode(x, y, z, depth));
	}

	/*!
	 * @brief Get the corresponding node for the code with bounds check.
	 *
	 * @param code The code for the node.
	 * @return The node.
	 */
	std::optional<NodeBV> getNodeBVChecked(Code code) const
	{
		return code.depth() <= depthLevels() ? std::optional<NodeBV>(getNodeBV(code))
		                                     : std::nullopt;
	}

	/*!
	 * @brief Get the corresponding node for the key with bounds check.
	 *
	 * @param key The key for the node.
	 * @return The node.
	 */
	std::optional<NodeBV> getNodeBVChecked(Key key) const
	{
		return getNodeBVChecked(toCode(key));
	}

	/*!
	 * @brief Get the corresponding node for the coordinate at a specific depth with bounds
	 * check.
	 *
	 * @param coord The coordinate for the node.
	 * @param depth The depth.
	 * @return The node.
	 */
	std::optional<NodeBV> getNodeBVChecked(Point3 coord, depth_t depth = 0) const
	{
		if (auto code = toCodeChecked(coord, depth)) {
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
	std::optional<NodeBV> getNodeBVChecked(coord_t x, coord_t y, coord_t z,
	                                       depth_t depth = 0) const
	{
		return getNodeBVChecked(Point3(x, y, z), depth);
	}

	//
	// Node child
	//

	Node getNodeChild(Node node, size_t child_idx) const
	{
		LeafNode const* data = 1 == node.depth()
		                           ? &this->getLeafChild(this->getInnerNode(node), child_idx)
		                           : &static_cast<LeafNode const&>(this->getInnerChild(
		                                 this->getInnerNode(node), child_idx));

		return Node(const_cast<LeafNode*>(data), node.code().child(child_idx));
	}

	NodeBV getNodeChild(NodeBV const& node, size_t child_idx) const
	{
		double child_half_size = node.halfSize() / 2;
		geometry::AAEBB child_aaebb(childCenter(node.center(), child_half_size, child_idx),
		                            child_half_size);

		LeafNode const* data = 1 == node.depth()
		                           ? &this->getLeafChild(this->getInnerNode(node), child_idx)
		                           : &static_cast<LeafNode const&>(this->getInnerChild(
		                                 this->getInnerNode(node), child_idx));

		return NodeBV(const_cast<LeafNode*>(data), node.code().child(child_idx), child_aaebb);
	}

	template <class Node>
	Node getNodeChildChecked(Node const& node, size_t child_idx) const
	{
		if (!isParent(node)) {
			throw std::out_of_range("Node has no children");
		} else if (7 < child_idx) {
			throw std::out_of_range("child_idx out of range");
		}
		return getNodeChild(node, child_idx);
	}

	//
	// Get Sibling
	//

	Node getNodeSibling(Node node, size_t sibling_idx) const
	{
		auto cur_idx = node.code().indexAtDepth(node.depth());

		if (sibling_idx == cur_idx) {
			return node;
		}

		LeafNode* sibling;
		int idx_diff = static_cast<int>(sibling_idx) - static_cast<int>(cur_idx);
		if (isPureLeaf(node)) {
			sibling = const_cast<LeafNode*>(std::next(&this->getLeafNode(node), idx_diff));
		} else {
			sibling = const_cast<InnerNode*>(std::next(&this->getInnerNode(node), idx_diff));
		}

		return Node(sibling, node.code().sibling(sibling_idx));
	}

	NodeBV getNodeSibling(NodeBV const& node, size_t sibling_idx) const
	{
		auto cur_idx = node.code().indexAtDepth(node.depth());

		if (sibling_idx == cur_idx) {
			return node;
		}

		LeafNode* sibling;
		int idx_diff = static_cast<int>(sibling_idx) - static_cast<int>(cur_idx);
		if (isPureLeaf(node)) {
			sibling = const_cast<LeafNode*>(std::next(&this->getLeafNode(node), idx_diff));
		} else {
			sibling = const_cast<InnerNode*>(std::next(&this->getInnerNode(node), idx_diff));
		}

		geometry::AAEBB sibling_aaebb(
		    siblingCenter(node.center(), node.halfSize(), cur_idx, sibling_idx),
		    node.halfSize());

		return NodeBV(sibling, node.code().sibling(sibling_idx), sibling_aaebb);
	}

	template <class Node>
	Node getNodeSiblingChecked(Node const& node, size_t sibling_idx) const
	{
		if (!isRoot(node)) {
			throw std::out_of_range("Node has no siblings");
		} else if (7 < sibling_idx) {
			throw std::out_of_range("sibling_idx out of range");
		}
		return getNodeSibling(node, sibling_idx);
	}

	//
	// Modified
	//

	bool isModified(Node node) const { return isModified(this->getLeafNode(node)); }

	constexpr bool isModified(Code code) const { return isModified(getLeafNode(code)); }

	constexpr bool isModified(Key key) const { return isModified(toCode(key)); }

	constexpr bool isModified(Point3 coord, depth_t depth = 0) const
	{
		return isModified(toCode(coord, depth));
	}

	constexpr bool isModified(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return isModified(toCode(x, y, z, depth));
	}

	//
	// Is root
	//

	bool isRoot(Node node) const { return isRoot(node.code()); }

	bool isRoot(Code code) const { return getRootCode() == code; }

	bool isRoot(Key key) const { return isRoot(toCode(key)); }

	bool isRoot(Point3 coord, depth_t depth = 0) const
	{
		return isRoot(toCode(coord, depth));
	}

	bool isRoot(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return isRoot(toCode(x, y, z, depth));
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
	Query<const_bounding_volume_query_iterator> queryBV(Predicates&& predicates) const
	{
		return Query<const_bounding_volume_query_iterator>(
		    beginQueryBV(std::forward<Predicates>(predicates)), endQueryBV());
	}

	template <
	    class Geometry, class Predicates,
	    std::enable_if_t<not std::is_invocable_r_v<double, Geometry, NodeBV>, bool> = true>
	Query<const_query_nearest_iterator> queryNearest(Geometry&& geometry,
	                                                 Predicates&& predicates) const
	{
		return Query<const_query_nearest_iterator>(
		    beginQueryNearest(std::forward<Geometry>(geometry),
		                      std::forward<Predicates>(predicates)),
		    endQueryNearest());
	}

	template <class SquaredDistance, class Predicates,
	          std::enable_if_t<std::is_invocable_r_v<double, SquaredDistance, NodeBV>,
	                           bool> = true>
	Query<const_query_nearest_iterator> queryNearest(SquaredDistance&& sq_dist,
	                                                 Predicates&& predicates) const
	{
		return Query<const_query_nearest_iterator>(
		    beginQueryNearest(std::forward<SquaredDistance>(sq_dist),
		                      std::forward<Predicates>(predicates)),
		    endQueryNearest());
	}

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

	template <class ExecutionPolicy, class Predicates, class OutputIt,
	          typename = std::enable_if_t<
	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
	OutputIt query(ExecutionPolicy policy, Predicates&& predicates, OutputIt d_first) const
	{
		if constexpr (std::is_same_v<typename OutputIt::value_type, Node>) {
			return std::copy(policy, beginQuery(std::forward<Predicates>(predicates)),
			                 endQuery(), d_first);
		} else {
			return std::copy(policy, beginQueryBV(std::forward<Predicates>(predicates)),
			                 endQueryBV(), d_first);
		}
	}

	template <class Predicates, class OutputIt>
	OutputIt queryK(size_t k, Predicates&& predicates, OutputIt d_first) const
	{
		size_t count = 0;
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

	template <class ExecutionPolicy, class Predicates, class OutputIt,
	          typename = std::enable_if_t<
	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
	OutputIt queryK(ExecutionPolicy policy, size_t k, Predicates&& predicates,
	                OutputIt d_first) const
	{
		return queryK(k, std::forward<Predicates>(predicates), d_first);
	}

	template <
	    class Geometry, class Predicates, class OutputIt,
	    std::enable_if_t<not std::is_invocable_r_v<double, Geometry, NodeBV>, bool> = true>
	OutputIt queryNearest(Geometry&& geometry, Predicates&& predicates, OutputIt d_first,
	                      double epsilon = 0.0) const
	{
		return std::copy(
		    beginQueryNearest(geometry, std::forward<Predicates>(predicates), epsilon),
		    endQueryNearest(geometry, std::forward<Predicates>(predicates)), d_first);
	}

	template <class SquaredDistance, class Predicates, class OutputIt,
	          std::enable_if_t<std::is_invocable_r_v<double, SquaredDistance, NodeBV>,
	                           bool> = true>
	OutputIt queryNearest(SquaredDistance&& sq_dist, Predicates&& predicates,
	                      OutputIt d_first, double epsilon = 0.0) const
	{
		return std::copy(
		    beginQueryNearest(sq_dist, std::forward<Predicates>(predicates), epsilon),
		    endQueryNearest(sq_dist, std::forward<Predicates>(predicates)), d_first);
	}

	template <
	    class ExecutionPolicy, class Geometry, class Predicates, class OutputIt,
	    std::enable_if_t<not std::is_invocable_r_v<double, Geometry, NodeBV>, bool> = true,
	    typename =
	        std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
	OutputIt queryNearest(ExecutionPolicy policy, Geometry&& geometry,
	                      Predicates&& predicates, OutputIt d_first,
	                      double epsilon = 0.0) const
	{
		return std::copy(policy,
		                 beginQueryNearest(std::forward<Geometry>(geometry),
		                                   std::forward<Predicates>(predicates), epsilon),
		                 endQueryNearest(), d_first);
	}

	template <
	    class ExecutionPolicy, class SquaredDistance, class Predicates, class OutputIt,
	    std::enable_if_t<std::is_invocable_r_v<double, SquaredDistance, NodeBV>, bool> =
	        true,
	    typename =
	        std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
	OutputIt queryNearest(ExecutionPolicy policy, SquaredDistance&& sq_dist,
	                      Predicates&& predicates, OutputIt d_first,
	                      double epsilon = 0.0) const
	{
		return std::copy(policy,
		                 beginQueryNearest(std::forward<SquaredDistance>(sq_dist),
		                                   std::forward<Predicates>(predicates), epsilon),
		                 endQueryNearest(), d_first);
	}

	template <
	    class Geometry, class Predicates, class OutputIt,
	    std::enable_if_t<not std::is_invocable_r_v<double, Geometry, NodeBV>, bool> = true>
	OutputIt queryNearestK(size_t k, Geometry&& geometry, Predicates&& predicates,
	                       OutputIt d_first, double epsilon = 0.0) const
	{
		size_t count = 0;
		for (auto it = beginQueryNearest(std::forward<Geometry>(geometry),
		                                 std::forward<Predicates>(predicates), epsilon);
		     count < k && it != endQueryNearest(); ++it, ++count) {
			*d_first++ = *it;
		}
		return d_first;
	}

	template <class SquaredDistance, class Predicates, class OutputIt,
	          std::enable_if_t<std::is_invocable_r_v<double, SquaredDistance, NodeBV>,
	                           bool> = true>
	OutputIt queryNearestK(size_t k, SquaredDistance&& sq_dist, Predicates&& predicates,
	                       OutputIt d_first, double epsilon = 0.0) const
	{
		size_t count = 0;
		for (auto it = beginQueryNearest(std::forward<SquaredDistance>(sq_dist),
		                                 std::forward<Predicates>(predicates), epsilon);
		     count < k && it != endQueryNearest(); ++it, ++count) {
			*d_first++ = *it;
		}
		return d_first;
	}

	template <
	    class ExecutionPolicy, class Geometry, class Predicates, class OutputIt,
	    std::enable_if_t<not std::is_invocable_r_v<double, Geometry, NodeBV>, bool> = true,
	    typename =
	        std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
	OutputIt queryNearestK(ExecutionPolicy policy, size_t k, Geometry&& geometry,
	                       Predicates&& predicates, OutputIt d_first,
	                       double epsilon = 0.0) const
	{
		size_t count = 0;
		for (auto it = beginQueryNearest(std::forward<Geometry>(geometry),
		                                 std::forward<Predicates>(predicates), epsilon);
		     count < k && it != endQueryNearest(); ++it, ++count) {
			*d_first++ = *it;
		}
		return d_first;
	}

	template <
	    class ExecutionPolicy, class SquaredDistance, class Predicates, class OutputIt,
	    std::enable_if_t<std::is_invocable_r_v<double, SquaredDistance, NodeBV>, bool> =
	        true,
	    typename =
	        std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
	OutputIt queryNearestK(ExecutionPolicy policy, size_t k, SquaredDistance&& sq_dist,
	                       Predicates&& predicates, OutputIt d_first,
	                       double epsilon = 0.0) const
	{
		size_t count = 0;
		for (auto it = beginQueryNearest(std::forward<SquaredDistance>(sq_dist),
		                                 std::forward<Predicates>(predicates), epsilon);
		     count < k && it != endQueryNearest(); ++it, ++count) {
			*d_first++ = *it;
		}
		return d_first;
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

	const_bounding_volume_query_iterator endQueryBV() const
	{
		return const_bounding_volume_query_iterator(
		    new Iterator<NodeBV, Derived, NodeBV>(getRootNodeBV()));
	}

	template <
	    class Geometry, class Predicates,
	    std::enable_if_t<not std::is_invocable_r_v<double, Geometry, NodeBV>, bool> = true>
	const_query_nearest_iterator beginQueryNearest(Geometry&& geometry,
	                                               Predicates&& predicates,
	                                               double epsilon = 0.0) const
	{
		return const_query_nearest_iterator(
		    new NearestIterator(&derived(), getRootNodeBV(), std::forward<Geometry>(geometry),
		                        std::forward<Predicates>(predicates), epsilon));
	}

	template <class SquaredDistance, class Predicates,
	          std::enable_if_t<std::is_invocable_r_v<double, SquaredDistance, NodeBV>,
	                           bool> = true>
	const_query_nearest_iterator beginQueryNearest(SquaredDistance&& sq_dist,
	                                               Predicates&& predicates,
	                                               double epsilon = 0.0) const
	{
		return const_query_nearest_iterator(new NearestIterator(
		    &derived(), getRootNodeBV(), std::forward<SquaredDistance>(sq_dist),
		    std::forward<Predicates>(predicates), epsilon));
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

	const_bounding_volume_iterator beginBV() const
	{
		return beginQueryBV(predicate::TRUE{});
	}

	const_bounding_volume_iterator endBV() const { return endQueryBV(predicate::TRUE{}); }

	//
	// Traverse
	//

	// If f returns true then children will not be visited
	template <class UnaryFunction>
	void traverse(UnaryFunction f)
	{
		if constexpr (std::is_same_v<util::argument<UnaryFunction, 0>, Node>) {
			traverseRecurs(f, getRootNode());
		} else {
			traverseRecurs(f, getRootNodeBV());
		}
	}

	template <class UnaryFunction>
	void traverse(UnaryFunction f) const
	{
		if constexpr (std::is_same_v<util::argument<UnaryFunction, 0>, Node>) {
			traverseRecurs(f, getRootNode());
		} else {
			traverseRecurs(f, getRootNodeBV());
		}
	}

	//
	// Update modified nodes
	//

	void updateModifiedNodes(bool keep_modified = false,
	                         depth_t max_depth = maxDepthLevels())
	{
		if (keep_modified) {
			updateModifiedNodesRecurs<true>(max_depth, this->getRoot(), depthLevels());
		} else {
			updateModifiedNodesRecurs<false>(max_depth, this->getRoot(), depthLevels());
		}
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void updateModifiedNodes(ExecutionPolicy policy, bool keep_modified = false,
	                         depth_t max_depth = maxDepthLevels())
	{
		if (keep_modified) {
			updateModifiedNodesRecurs<true>(policy, max_depth, this->getRoot(), depthLevels());
		} else {
			updateModifiedNodesRecurs<false>(policy, max_depth, this->getRoot(), depthLevels());
		}
	}

	void updateModifiedNodes(Node const& node, bool keep_modified = false,
	                         depth_t max_depth = maxDepthLevels())
	{
		// TODO: Implement
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void updateModifiedNodes(ExecutionPolicy policy, Node const& node,
	                         bool keep_modified = false,
	                         depth_t max_depth = maxDepthLevels())
	{
		// TODO: Implement
		// if ()

		// updateModifiedNodesRecurs(policy, max_depth, )
	}

	void updateModifiedNodes(Code code, bool keep_modified = false,
	                         depth_t max_depth = maxDepthLevels())
	{
		// TODO: Implement
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void updateModifiedNodes(ExecutionPolicy policy, Code code, bool keep_modified = false,
	                         depth_t max_depth = maxDepthLevels())
	{
		// TODO: Implement
	}

	void updateModifiedNodes(Key key, bool keep_modified = false,
	                         depth_t max_depth = maxDepthLevels())
	{
		updateModifiedNodes(toCode(key), keep_modified, max_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void updateModifiedNodes(ExecutionPolicy policy, Key key, bool keep_modified = false,
	                         depth_t max_depth = maxDepthLevels())
	{
		updateModifiedNodes(policy, toCode(key), keep_modified, max_depth);
	}

	void updateModifiedNodes(Point3 coord, depth_t depth = 0, bool keep_modified = false,
	                         depth_t max_depth = maxDepthLevels())
	{
		updateModifiedNodes(toCode(coord, depth), depth, keep_modified, max_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void updateModifiedNodes(ExecutionPolicy policy, Point3 coord, depth_t depth = 0,
	                         bool keep_modified = false,
	                         depth_t max_depth = maxDepthLevels())
	{
		updateModifiedNodes(policy, toCode(coord, depth), keep_modified, max_depth);
	}

	void updateModifiedNodes(double x, double y, double z, depth_t depth = 0,
	                         bool keep_modified = false,
	                         depth_t max_depth = maxDepthLevels())
	{
		updateModifiedNodes(toCode(x, y, z, depth), keep_modified, max_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void updateModifiedNodes(ExecutionPolicy policy, double x, double y, double z,
	                         depth_t depth = 0, bool keep_modified = false,
	                         depth_t max_depth = maxDepthLevels())
	{
		updateModifiedNodes(policy, toCode(x, y, z, depth), keep_modified, max_depth);
	}

	//
	// Set modified
	//

	void setModified(depth_t min_depth = 0)
	{
		if (depthLevels() >= min_depth) {
			setModifiedRecurs(this->getRoot(), depthLevels(), min_depth);
		}
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setModified(ExecutionPolicy policy, depth_t min_depth = 0)
	{
		// FIXME: Update
		if (depthLevels() >= min_depth) {
			setModifiedRecurs(this->getRoot(), depthLevels(), min_depth);
		}
	}

	void setModified(Node node, depth_t min_depth = 0)
	{
		if (node.depth() >= min_depth) {
			if (0 == node.depth()) {
				setModified(this->getLeafNode(node), true);
			} else {
				setModifiedRecurs(this->getInnerNode(node), node.depth(), min_depth);
			}
			setModifiedParentsRecurs(this->getRoot(), depthLevels(), node.code());
		} else {
			setModifiedParentsRecurs(this->getRoot(), depthLevels(),
			                         node.code().toDepth(min_depth));
		}
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setModified(ExecutionPolicy policy, Node node, depth_t min_depth = 0)
	{
		if (node.depth() >= min_depth) {
			if (0 == node.depth()) {
				setModified(this->getLeafNode(node), true);
			} else {
				// FIXME: Update
				setModifiedRecurs(this->getInnerNode(node), node.depth(), min_depth);
			}
			setModifiedParentsRecurs(this->getRoot(), depthLevels(), node.code());
		} else {
			setModifiedParentsRecurs(this->getRoot(), depthLevels(),
			                         node.code().toDepth(min_depth));
		}
	}

	void setModified(Code code, depth_t min_depth = 0)
	{
		if (code.depth() >= min_depth) {
			if (0 == code.depth()) {
				setModified(getLeafNode(code), true);
			} else {
				setModifiedRecurs(getInnerNode(code), code.depth(), min_depth);
			}
			setModifiedParentsRecurs(this->getRoot(), depthLevels(), code);
		} else {
			setModifiedParentsRecurs(this->getRoot(), depthLevels(), code.toDepth(min_depth));
		}
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setModified(ExecutionPolicy policy, Code code, depth_t min_depth = 0)
	{
		if (code.depth() >= min_depth) {
			if (0 == code.depth()) {
				setModified(getLeafNode(code), true);
			} else {
				// FIXME: Update
				setModifiedRecurs(getInnerNode(code), code.depth(), min_depth);
			}
			setModifiedParentsRecurs(this->getRoot(), depthLevels(), code);
		} else {
			setModifiedParentsRecurs(this->getRoot(), depthLevels(), code.toDepth(min_depth));
		}
	}

	void setModified(Key key, depth_t min_depth = 0)
	{
		setModified(toCode(key), min_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setModified(ExecutionPolicy policy, Key key, depth_t min_depth = 0)
	{
		setModified(policy, toCode(key), min_depth);
	}

	void setModified(Point3 coord, depth_t depth = 0, depth_t min_depth = 0)
	{
		setModified(toCode(coord, depth), min_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setModified(ExecutionPolicy policy, Point3 coord, depth_t depth = 0,
	                 depth_t min_depth = 0)
	{
		setModified(policy, toCode(coord, depth), min_depth);
	}

	void setModified(double x, double y, double z, depth_t depth = 0, depth_t min_depth = 0)
	{
		setModified(toCode(x, y, z, depth), min_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setModified(ExecutionPolicy policy, double x, double y, double z,
	                 depth_t depth = 0, depth_t min_depth = 0)
	{
		setModified(policy, toCode(x, y, z, depth), min_depth);
	}

	//
	// Clear modified
	//

	void clearModified(depth_t max_depth = maxDepthLevels())
	{
		clearModifiedRecurs(this->getRoot(), depthLevels(), max_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void clearModified(ExecutionPolicy policy, depth_t max_depth = maxDepthLevels())
	{
		// FIXME: Update
		clearModifiedRecurs(this->getRoot(), depthLevels(), max_depth);
	}

	void clearModified(Node node, depth_t max_depth = maxDepthLevels())
	{
		if (isLeaf(node)) {
			setModified(this->getLeafNode(node), false);
		} else {
			clearModifiedRecurs(this->getInnerNode(node), node.depth(), max_depth);
		}
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void clearModified(ExecutionPolicy policy, Node node,
	                   depth_t max_depth = maxDepthLevels())
	{
		// FIXME: Update
		if (isPureLeaf(node)) {
			setModified(this->getLeafNode(node), false);
		} else {
			clearModifiedRecurs(this->getInnerNode(node), node.depth(), max_depth);
		}
	}

	void clearModified(Code code, depth_t max_depth = maxDepthLevels())
	{
		if (0 == code.depth()) {
			setModified(getLeafNode(code), false);
		} else {
			clearModifiedRecurs(getInnerNode(code), code.depth(), max_depth);
		}
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void clearModified(ExecutionPolicy policy, Code code,
	                   depth_t max_depth = maxDepthLevels())
	{
		// FIXME: Update
		if (0 == code.depth()) {
			setModified(getLeafNode(code), false);
		} else {
			clearModifiedRecurs(getInnerNode(code), code.depth(), max_depth);
		}
	}

	void clearModified(Key key, depth_t max_depth = maxDepthLevels())
	{
		clearModified(toCode(key), max_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void clearModified(ExecutionPolicy policy, Key key,
	                   depth_t max_depth = maxDepthLevels())
	{
		clearModified(policy, toCode(key), max_depth);
	}

	void clearModified(Point3 coord, depth_t depth = 0,
	                   depth_t max_depth = maxDepthLevels())
	{
		clearModified(toCode(coord, depth), max_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void clearModified(ExecutionPolicy policy, Point3 coord, depth_t depth = 0,
	                   depth_t max_depth = maxDepthLevels())
	{
		clearModified(policy, toCode(coord, depth), max_depth);
	}

	void clearModified(double x, double y, double z, depth_t depth = 0,
	                   depth_t max_depth = maxDepthLevels())
	{
		clearModified(toCode(x, y, z, depth), max_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void clearModified(ExecutionPolicy policy, double x, double y, double z,
	                   depth_t depth = 0, depth_t max_depth = maxDepthLevels())
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
		file.imbue(std::locale());
		file.open(filename, std::ios_base::in | std::ios_base::binary);

		return canMerge(file);
	}

	[[nodiscard]] bool canMerge(std::istream& in_stream) const
	{
		auto pos = in_stream.tellg();
		FileInfo header = readHeader(in_stream);
		in_stream.seekg(pos);

		double res;
		std::istringstream(header.at("resolution").at(0)) >> res;
		depth_t depth_levels;
		std::istringstream(header.at("depth_levels").at(0)) >> depth_levels;

		return resolution() == res && depthLevels() == depth_levels;
	}

	void read(std::filesystem::path const& filename, bool propagate = true)
	{
		std::ifstream file;
		file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		file.imbue(std::locale());
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
		std::istringstream(header.at("resolution").at(0)) >> res;
		depth_t depth_levels;
		std::istringstream(header.at("depth_levels").at(0)) >> depth_levels;

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
			data.imbue(std::locale());

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

	void write(std::filesystem::path const& filename, depth_t min_depth = 0,
	           bool compress = false, int compression_acceleration_level = 1,
	           int compression_level = 0) const
	{
		write(filename, predicate::TRUE(), min_depth, compress,
		      compression_acceleration_level, compression_level);
	}

	void write(std::ostream& out_stream, depth_t min_depth = 0, bool compress = false,
	           int compression_acceleration_level = 1, int compression_level = 0) const
	{
		write(out_stream, predicate::TRUE(), min_depth, compress,
		      compression_acceleration_level, compression_level);
	}

	template <class Predicates,
	          typename = std::enable_if_t<!std::is_scalar_v<std::decay_t<Predicates>>>>
	void write(std::filesystem::path const& filename, Predicates&& predicates,
	           depth_t min_depth = 0, bool compress = false,
	           int compression_acceleration_level = 1, int compression_level = 0) const
	{
		std::ofstream file;
		file.exceptions(std::ofstream::failbit | std::ofstream::badbit);
		file.imbue(std::locale());
		file.open(filename, std::ios_base::out | std::ios_base::binary);

		write(file, std::forward<Predicates>(predicates), min_depth, compress,
		      compression_acceleration_level, compression_level);
	}

	template <class Predicates,
	          typename = std::enable_if_t<!std::is_scalar_v<std::decay_t<Predicates>>>>
	void write(std::ostream& out_stream, Predicates&& predicates, depth_t min_depth = 0,
	           bool compress = false, int compression_acceleration_level = 1,
	           int compression_level = 0) const
	{
		writeHeader(out_stream, getFileInfo());
		writeData(out_stream, std::forward<Predicates>(predicates), min_depth, compress,
		          compression_acceleration_level, compression_level);
	}
	void writeAndUpdateModified(std::filesystem::path const& filename,
	                            bool compress = false,
	                            int compression_acceleration_level = 1,
	                            int compression_level = 0)
	{
		std::ofstream file;
		file.exceptions(std::ofstream::failbit | std::ofstream::badbit);
		file.imbue(std::locale());
		file.open(filename, std::ios_base::out | std::ios_base::binary);

		writeAndUpdateModified(file, compress, compression_acceleration_level,
		                       compression_level);
	}

	void writeAndUpdateModified(std::ostream& out_stream, bool compress = false,
	                            int compression_acceleration_level = 1,
	                            int compression_level = 0)
	{
		writeHeader(out_stream, getFileInfo());
		writeAndUpdateModifiedData(out_stream, compress, compression_acceleration_level,
		                           compression_level);
	}

 protected:
	//
	// Constructors
	//

	OctreeBase(double resolution = 0.1, depth_t depth_levels = 16,
	           bool automatic_pruning = true)
	    : automatic_pruning_enabled_(automatic_pruning)
	{
		setResolutionAndDepthLevels(resolution, depth_levels);
	}

	OctreeBase(OctreeBase const& other)
	    : depth_levels_(other.depth_levels_),
	      max_value_(other.max_value_),
	      node_size_(other.node_size_),
	      node_size_factor_(other.node_size_factor_),
	      automatic_pruning_enabled_(other.automatic_pruning_enabled_),
	      parallel_depth_(other.parallel_depth_)
	{
		// TODO: Implement

		// num_inner_nodes_ = other.num_inner_nodes_;
		// num_inner_leaf_nodes_ = other.num_inner_leaf_nodes_;
		// num_leaf_nodes_ = other.num_leaf_nodes_;
	}

	OctreeBase(OctreeBase&& other)
	    : depth_levels_(std::move(other.depth_levels_)),
	      max_value_(std::move(other.max_value_)),
	      node_size_(std::move(other.node_size_)),
	      node_size_factor_(std::move(other.node_size_factor_)),
	      automatic_pruning_enabled_(std::move(other.automatic_pruning_enabled_)),
	      parallel_depth_(std::move(other.parallel_depth_))
	{
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
		// TODO: Should this clear?
		clear(rhs.resolution(), rhs.depthLevels(), true);

		depth_levels_ = std::move(rhs.depth_levels_);
		max_value_ = std::move(rhs.max_value_);
		node_size_ = std::move(rhs.node_size_);
		node_size_factor_ = std::move(rhs.node_size_factor_);
		automatic_pruning_enabled_ = std::move(rhs.automatic_pruning_enabled_);
		parallel_depth_ = std::move(rhs.parallel_depth_);
		return *this;
	}

	//
	// Destructor
	//

	~OctreeBase() { this->deleteChildren(this->getRoot(), depthLevels(), true); }

	//
	// Derived
	//

	constexpr Derived& derived() { return *static_cast<Derived*>(this); }

	constexpr Derived const& derived() const { return *static_cast<Derived const*>(this); }

	//
	// Set resolution and depth levels
	//

	void setResolutionAndDepthLevels(double const resolution, depth_t const depth_levels)
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
			              auto const c = n;
			              n *= 2.0;
			              return c;
		              });

		std::transform(std::begin(node_size_), std::end(node_size_),
		               std::begin(node_size_factor_), [](auto n) { return 1.0 / n; });
	}

	//
	// Initilize root
	//

	void initRoot()
	{
		setIsLeaf(this->getRoot(), true);
		setModified(this->getRoot(), false);
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
		Key::key_t val = std::floor(node_size_factor_[0] * coord);
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
	void traverseRecurs(UnaryFunction f, NodeType const& node)
	{
		if (f(node) || isLeaf(node)) {
			return;
		}

		for (size_t i = 0; i != 8; ++i) {
			traverseRecurs(f, getNodeChild(node, i));
		}
	}

	template <class UnaryFunction, class NodeType>
	void traverseRecurs(UnaryFunction f, NodeType const& node) const
	{
		if (f(node) || isLeaf(node)) {
			return;
		}

		for (size_t i = 0; i != 8; ++i) {
			traverseRecurs(f, getNodeChild(node, i));
		}
	}

	//
	// Apply
	//

	template <class UnaryFunction,
	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction>>>
	void apply(Node node, UnaryFunction f, bool propagate)
	{
		if (isLeaf(node)) {
			f(this->getLeafNode(node));
		} else {
			applyAllRecurs(this->getInnerNode(node), node.depth(), f);
		}

		if (!isModified(node)) {
			// Update all parents
			setModifiedParents(node);

			setModified(this->getLeafNode(node), true);
		}

		if (propagate) {
			updateModifiedNodes();
		}
	}

	template <class ExecutionPolicy, class UnaryFunction,
	          typename = std::enable_if_t<
	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>,
	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction>>>
	void apply(ExecutionPolicy policy, Node node, UnaryFunction f, bool propagate)
	{
		if (isLeaf(node)) {
			f(this->getLeafNode(node));
		} else {
			applyAllRecurs(policy, this->getInnerNode(node), node.depth(), f);
		}

		if (!isModified(node)) {
			// Update all parents
			setModifiedParents(node);

			setModified(this->getLeafNode(node), true);
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

		applyRecurs(this->getRoot(), depthLevels(), code, f);

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

		applyRecurs(policy, this->getRoot(), depthLevels(), code, f);

		if (propagate) {
			updateModifiedNodes(policy);
		}
	}

	template <class UnaryFunction,

	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction>>>
	void applyRecurs(InnerNode& node, depth_t depth, Code code, UnaryFunction f)
	{
		if (code.depth() == depth) {
			if (isLeaf(node)) {
				f(static_cast<LeafNode&>(node));
			} else {
				applyAllRecurs(node, depth, f);
			}
		} else if (1 == depth) {
			this->createChildren(node, depth);
			LeafNode& child = this->getLeafChild(node, code.indexAtDepth(0));
			f(child);
			setModified(child, true);
		} else {
			this->createChildren(node, depth);
			applyRecurs(this->getInnerChild(node, code.indexAtDepth(depth - 1)), depth - 1,
			            code, f);
		}

		setModified(node, true);
	}

	template <class ExecutionPolicy, class UnaryFunction,
	          typename = std::enable_if_t<
	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>,
	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction>>>
	void applyRecurs(ExecutionPolicy policy, InnerNode& node, depth_t depth, Code code,
	                 UnaryFunction f)
	{
		if (code.depth() == depth) {
			if (isLeaf(node)) {
				f(static_cast<LeafNode&>(node));
			} else {
				applyAllRecurs(policy, node, depth, f);
			}
		} else if (1 == depth) {
			this->createChildren(node, depth);
			LeafNode& child = this->getLeafChild(node, code.indexAtDepth(0));
			f(child);
			setModified(child, true);
		} else {
			this->createChildren(node, depth);
			applyRecurs(policy, this->getInnerChild(node, code.indexAtDepth(depth - 1)),
			            depth - 1, code, f);
		}

		setModified(node, true);
	}

	template <class UnaryFunction,
	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction>>>
	void applyAllRecurs(InnerNode& node, depth_t depth, UnaryFunction f)
	{
		if (1 == depth) {
			for (LeafNode& child : this->getLeafChildren(node)) {
				f(child);
				setModified(child, true);
			}
		} else {
			for (InnerNode& child : this->getInnerChildren(node)) {
				if (isLeaf(child)) {
					f(static_cast<LeafNode&>(child));
				} else {
					applyAllRecurs(child, depth - 1, f);
				}
				setModified(child, true);
			}
		}
	}

	template <class ExecutionPolicy, class UnaryFunction,
	          typename = std::enable_if_t<
	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>,
	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction>>>
	void applyAllRecurs(ExecutionPolicy policy, InnerNode& node, depth_t depth,
	                    UnaryFunction f)
	{
		if (1 == depth) {
			for_each(policy, this->getLeafChildren(node), [&f](LeafNode& child) {
				f(child);
				setModified(child, true);
			});
		} else {
			for_each(policy, this->getInnerChildren(node), [&](InnerNode& child) {
				if (isLeaf(child)) {
					f(static_cast<LeafNode&>(child));
				} else {
					applyAllRecurs(child, depth - 1, f);
				}
				setModified(child, true);
			});
		}
	}

	//
	// Get node
	//

	using MM::getLeafNode;

	LeafNode const& getLeafNode(Code code) const
	{
		LeafNode const* node = &this->getRoot();
		depth_t depth = depthLevels();
		while (code.depth() != depth || isParent(node)) {
			--depth;
			node = 0 == depth ? &this->getLeafChild(node, code.indexAtDepth(depth))
			                  : &this->getInnerChild(node, code.indexAtDepth(depth));
		}
		return *node;
	}

	LeafNode& getLeafNode(Code code)
	{
		LeafNode* node = &this->getRoot();
		depth_t depth = depthLevels();
		while (code.depth() != depth || isParent(node)) {
			--depth;
			node = 0 == depth ? &this->getLeafChild(node, code.indexAtDepth(depth))
			                  : &this->getInnerChild(node, code.indexAtDepth(depth));
		}
		return *node;
	}

	using MM::getInnerNode;

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
		LeafNode const* node = &this->getRoot();
		depth_t depth = depthLevels();
		while (code.depth() != depth || isParent(node)) {
			--depth;
			node = 0 == depth ? &this->getLeafChild(node, code.indexAtDepth(depth))
			                  : &this->getInnerChild(node, code.indexAtDepth(depth));
		}
		return {*node, depth};
	}

	std::pair<LeafNode&, depth_t> getNodeAndDepth(Code code)
	{
		LeafNode* node = &this->getRoot();
		depth_t depth = depthLevels();
		while (code.depth() != depth || isParent(node)) {
			--depth;
			node = 0 == depth ? &this->getLeafChild(node, code.indexAtDepth(depth))
			                  : &this->getInnerChild(node, code.indexAtDepth(depth));
		}
		return {*node, depth};
	}

	//
	// Delete node
	//

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void deleteNodeChildrenRecurs(ExecutionPolicy policy, InnerNode& node, depth_t depth,
	                              Code code)
	{
		// FIXME: What happens when no children?
		if (isLeaf(node)) {
			return;
		}

		if (code.depth() == depth) {
			this->deleteChildren(policy, node, depth, automaticPruning());
		} else {
			deleteNodeChildrenRecurs(policy,
			                         this->getInnerChild(node, code.indexAtDepth(depth - 1)),
			                         depth - 1, code);
		}

		setModified(node, true);
	}

	//
	// Get child center
	//

	static constexpr Point3 childCenter(Point3 parent_center, double child_half_size,
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

	static constexpr Point3 siblingCenter(Point3 sibling_center, double half_size,
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

	static constexpr Point3 getParentCenter(Point3 child_center, double child_half_size,
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
	// Set is leaf
	//

	static constexpr void setIsLeaf(LeafNode& node, bool is_leaf) noexcept
	{
		node.is_leaf = is_leaf;
	}

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
	// Set is modified
	//

	static constexpr void setModified(LeafNode& node, bool modified) noexcept
	{
		node.modified = modified;
	}

	//
	// Is node collapsible
	//

	// If all children are the same as the parent they can be pruned
	// NOTE: Only call with nodes that have children
	bool isNodeCollapsible(InnerNode const& node, depth_t depth) const noexcept
	{
		return 1 == depth ? all_of(this->getLeafChildren(node),
		                           [node = static_cast<LeafNode const&>(node)](auto&& child) {
			                           return node == child;
		                           })
		                  : all_of(this->getInnerChildren(node),
		                           [node = static_cast<LeafNode const&>(node)](auto&& child) {
			                           return isLeaf(child) &&
			                                  node == static_cast<LeafNode const&>(child);
		                           });
	}

	//
	// Update nodes
	//

	template <bool KeepModified>
	void updateModifiedNodesRecurs(depth_t max_depth, InnerNode& node, depth_t depth)
	{
		if (isLeaf(node)) {
			if (depth <= max_depth) {
				derived().updateNodeIndicators(node);
				if constexpr (!KeepModified) {
					setModified(node, false);
				}
			}
			return;
		}

		if (1 == depth) {
			for (auto& child : this->getLeafChildren(node)) {
				if (isModified(child)) {
					derived().updateNodeIndicators(child);
					if constexpr (!KeepModified) {
						setModified(child, false);
					}
				}
			}
		} else {
			for (auto& child : this->getInnerChildren(node)) {
				if (isModified(child)) {
					updateModifiedNodesRecurs<KeepModified>(max_depth, child, depth - 1);
				}
			}
		}

		if (depth <= max_depth) {
			derived().updateNode(node, depth);
			derived().updateNodeIndicators(node, depth);
			pruneNode(node, depth);
			if constexpr (!KeepModified) {
				setModified(node, false);
			}
		}
	}

	template <bool KeepModified, class ExecutionPolicy,
	          typename = std::enable_if_t<
	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
	void updateModifiedNodesRecurs(ExecutionPolicy policy, depth_t max_depth,
	                               InnerNode& node, depth_t depth)
	{
		if (!isModified(node)) {
			return;
		}

		if (isParent(node)) {
			if (1 == depth) {
				for (auto& child : this->getLeafChildren(node)) {
					derived().updateNodeIndicators(child);
					if constexpr (!KeepModified) {
						setModified(child, false);  // TODO: Create function
					}
				}
			} else {
				if (parallelExecutionDepth() == depth) {
					for_each(policy, this->getInnerChildren(node),
					         [this, max_depth, depth](auto& child) {
						         // Sequential execution for rest in case parallel depth changes
						         // in between
						         updateModifiedNodesRecurs<KeepModified>(max_depth, child, depth - 1);
					         });
				} else {
					for (auto& child : this->getInnerChildren(node)) {
						updateModifiedNodesRecurs<KeepModified>(policy, max_depth, child, depth - 1);
					}
				}
			}
		}

		if (depth <= max_depth) {
			if (isParent(node)) {
				derived().updateNode(node, depth);
				derived().updateNodeIndicators(node, depth);
				pruneNode(node, depth);
			} else {
				derived().updateNodeIndicators(node);
			}
			if constexpr (!KeepModified) {
				setModified(node, false);  // TODO: Create function
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
		setModifiedParentsRecurs(this->getRoot(), depthLevels(), node.code());
	}

	// NOTE: Assumes code has depth higher then depth
	void setModifiedParentsRecurs(InnerNode& node, depth_t depth, Code code)
	{
		setModified(node, true);
		if (code.depth() < depth - 1) {
			setModifiedParentsRecurs(this->getInnerChild(node, code.indexAtDepth(depth - 1)),
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
			for (auto& child : this->getLeafChildren(node)) {
				setModified(child, true);
			}
		} else {
			for (auto& child : this->getInnerChildren(node)) {
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
			for (auto& child : this->getLeafChildren(node)) {
				setModified(child, false);
			}
		} else {
			for (auto& child : this->getInnerChildren(node)) {
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
		if (isNodeCollapsible(node, depth)) {
			this->deleteChildren(node, depth, automaticPruning());
			return true;
		} else {
			return false;
		}
	}

	bool pruneRecurs(InnerNode& node, depth_t depth)
	{
		if (isLeaf(node)) {
			return true;
		}

		if (1 == depth) {
			return pruneNode(node, depth);
		}

		bool prunable = all_of(this->getInnerChildren(node), [this, depth](InnerNode& child) {
			return pruneRecurs(child, depth - 1);
		});

		return !prunable || pruneNode(node, depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	bool pruneRecurs(ExecutionPolicy policy, InnerNode& node, depth_t depth)
	{
		if (isLeaf(node)) {
			return true;
		}

		if (1 == depth) {
			return pruneNode(node, depth);
		}

		bool prunable;
		if (parallelExecutionDepth() == depth) {
			prunable = all_of(
			    policy, this->getInnerChildren(node),
			    [this, depth](InnerNode& child) { return pruneRecurs(child, depth - 1); });
		} else {
			prunable =
			    all_of(this->getInnerChildren(node), [this, policy, depth](InnerNode& child) {
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
		info["depth_levels"].push_back(std::to_string(depthLevels()));
		derived().addFileInfo(info);
		return info;
	}

	std::vector<LeafNode*> getNodes(std::istream& in_stream)
	{
		auto [indicators, size] = getIndicators(in_stream);
		return getNodes(indicators, size);
	}

	std::pair<std::unique_ptr<uint8_t[]>, uint64_t> getIndicators(std::istream& in_stream)
	{
		uint64_t num;
		in_stream.read(reinterpret_cast<char*>(&num), sizeof(num));

		auto indicators = std::make_unique<uint8_t[]>(num);
		in_stream.read(reinterpret_cast<char*>(indicators.get()), sizeof(uint8_t) * num);

		return {std::move(indicators), num};
	}

	std::vector<LeafNode*> getNodes(std::unique_ptr<uint8_t[]> const& indicators,
	                                uint64_t const indicators_size)
	{
		uint64_t total_nodes = 0;
		for (size_t i = 0; i < indicators_size; i += 2) {
			for (size_t b = 0; 8 != b; ++b) {
				if ((indicators[i] >> b) & 1U) {
					++total_nodes;
				}
			}
		}

		std::vector<LeafNode*> nodes;
		nodes.reserve(total_nodes);

		bool valid_return = 0 != indicators[0];
		bool valid_inner = 0 != indicators[1];

		if (valid_return) {
			nodes.push_back(static_cast<LeafNode*>(&this->getRoot()));
			setModified(this->getRoot(), true);
		} else if (valid_inner) {
			getNodesRecurs(std::next(indicators.get(), 2), nodes, this->getRoot(),
			               depthLevels());
		}

		return nodes;
	}

	uint8_t* getNodesRecurs(uint8_t* indicators, std::vector<LeafNode*>& nodes,
	                        InnerNode& node, depth_t const depth)
	{
		uint8_t const child_valid_return = *indicators++;

		if (1 == depth) {
			if (0 == child_valid_return) {
				return indicators;
			}

			setModified(node, true);

			this->createChildren(node, depth);

			for (std::size_t i = 0; 8 != i; ++i) {
				if ((child_valid_return >> i) & 1U) {
					nodes.push_back(&this->getLeafChild(node, i));
				}
			}
		} else {
			uint8_t const child_valid_inner = *indicators++;

			if (0 == child_valid_return && 0 == child_valid_inner) {
				return indicators;
			}

			setModified(node, true);

			this->createChildren(node, depth);

			for (size_t i = 0; i != 8; ++i) {
				if ((child_valid_return >> i) & 1U) {
					nodes.push_back(&this->getInnerChild(node, i));
				} else if ((child_valid_inner >> i) & 1U) {
					indicators =
					    getNodesRecurs(indicators, nodes, this->getInnerChild(node, i), depth - 1);
				}
			}
		}

		return indicators;
	}

	void readNodes(std::istream& in_stream, FileInfo const& header,
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

			uint64_t data_size;
			in_stream.read(reinterpret_cast<char*>(&data_size), sizeof(data_size));

			if (!derived().readNodes(in_stream, nodes, field, type, size)) {
				// Skip forward
				in_stream.seekg(data_size, std::istream::cur);
			}
		}
	}

	template <class Predicates>
	void writeData(std::ostream& out_stream, Predicates&& predicates, depth_t min_depth,
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

		auto [indicators, nodes] =
		    data(predicate::Leaf(min_depth) && std::forward<Predicates>(predicates));

		if (compress) {
			std::stringstream data(std::ios_base::in | std::ios_base::out |
			                       std::ios_base::binary);
			data.exceptions(std::stringstream::failbit | std::stringstream::badbit);
			data.imbue(std::locale());

			writeIndicators(data, indicators);
			derived().writeNodes(data, nodes);

			uncompressed_data_size = data.tellp();

			compressData(data, out_stream, uncompressed_data_size,
			             compression_acceleration_level, compression_level);
		} else {
			writeIndicators(out_stream, indicators);
			derived().writeNodes(out_stream, nodes);

			uncompressed_data_size = out_stream.tellp() - data_pos;
		}

		auto end_pos = out_stream.tellp();

		out_stream.seekp(size_pos);
		out_stream.write(reinterpret_cast<char*>(&uncompressed_data_size),
		                 sizeof(uncompressed_data_size));

		out_stream.seekp(end_pos);
	}

	void writeAndUpdateModifiedData(std::ostream& out_stream, bool compress,
	                                int compression_acceleration_level,
	                                int compression_level)
	{
		uint8_t compressed = compress ? UINT8_MAX : 0U;
		out_stream.write(reinterpret_cast<char*>(&compressed), sizeof(compressed));

		auto size_pos = out_stream.tellp();
		uint64_t uncompressed_data_size = 0;
		out_stream.write(reinterpret_cast<char*>(&uncompressed_data_size),
		                 sizeof(uncompressed_data_size));

		auto data_pos = out_stream.tellp();

		auto [indicators, nodes] = modifiedData();

		if (compress) {
			std::stringstream data(std::ios_base::in | std::ios_base::out |
			                       std::ios_base::binary);
			data.exceptions(std::stringstream::failbit | std::stringstream::badbit);
			data.imbue(std::locale());

			writeIndicators(data, indicators);
			derived().writeNodes(data, nodes);

			uncompressed_data_size = data.tellp();

			compressData(data, out_stream, uncompressed_data_size,
			             compression_acceleration_level, compression_level);
		} else {
			writeIndicators(out_stream, indicators);
			derived().writeNodes(out_stream, nodes);

			uncompressed_data_size = out_stream.tellp() - data_pos;
		}

		auto end_pos = out_stream.tellp();

		out_stream.seekp(size_pos);
		out_stream.write(reinterpret_cast<char*>(&uncompressed_data_size),
		                 sizeof(uncompressed_data_size));

		out_stream.seekp(end_pos);
	}

	template <class Predicates>
	std::pair<std::vector<uint8_t>, std::vector<LeafNode>> data(
	    Predicates const& predicates) const
	{
		std::vector<uint8_t> indicators;
		std::vector<LeafNode> nodes;

		std::conditional_t<predicate::contains_spatial_predicate_v<Predicates>, NodeBV, Node>
		    root = getRootNodeBV();

		bool valid_return =
		    predicate::PredicateValueCheck<Predicates>::apply(predicates, derived(), root);
		bool valid_inner = !valid_return && predicate::PredicateInnerCheck<Predicates>::apply(
		                                        predicates, derived(), root);

		indicators.push_back(valid_return ? UINT8_MAX : 0U);
		indicators.push_back(valid_inner ? UINT8_MAX : 0U);

		if (valid_return) {
			nodes.push_back(this->getLeafNode(root));
		} else if (valid_inner) {
			dataRecurs(indicators, nodes, predicates, root);
			if (nodes.empty()) {
				//  Nothing was added
				indicators.clear();
			}
		}

		return {indicators, nodes};
	}

	template <class Predicates, class NodeType>
	void dataRecurs(std::vector<uint8_t>& indicators, std::vector<LeafNode>& nodes,
	                Predicates const& predicates, NodeType const& node) const
	{
		auto cur_indicators_size = indicators.size();
		auto cur_nodes_size = nodes.size();

		uint8_t child_valid_return = 0;

		if (1 == getDepth(node)) {
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
					nodes.push_back(this->getLeafChild(this->getInnerNode(node), i));
				}
			}
		} else {
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
					nodes.push_back(this->getInnerChild(this->getInnerNode(node), i));
				} else if ((child_valid_inner >> i) & 1U) {
					dataRecurs(indicators, nodes, predicates, getNodeChild(node, i));
				}
			}

			if (nodes.size() == cur_nodes_size) {
				indicators.resize(cur_indicators_size + 2);
				indicators.back() = 0U;
				*std::prev(std::end(indicators), 2) = 0U;
			}
		}
	}

	std::pair<std::vector<uint8_t>, std::vector<LeafNode>> modifiedData()
	{
		std::vector<uint8_t> indicators;
		std::vector<LeafNode> nodes;

		InnerNode& root = this->getRoot();

		bool valid_return = isLeaf(root) && isModified(root);
		bool valid_inner = isModified(root);

		indicators.push_back(valid_return ? UINT8_MAX : 0U);
		indicators.push_back(valid_inner ? UINT8_MAX : 0U);

		if (valid_return) {
			nodes.push_back(root);
			derived().updateNodeIndicators(root);
		} else if (valid_inner) {
			modifiedDataRecurs(indicators, nodes, root, depthLevels());
			derived().updateNode(root, depthLevels());
			derived().updateNodeIndicators(root, depthLevels());
			pruneNode(root, depthLevels());
			if (nodes.empty()) {
				//  Nothing was added
				indicators.clear();
			}
		}

		setModified(root, false);

		return {indicators, nodes};
	}

	void modifiedDataRecurs(std::vector<uint8_t>& indicators, std::vector<LeafNode>& nodes,
	                        InnerNode& node, depth_t depth)
	{
		auto cur_indicators_size = indicators.size();
		auto cur_nodes_size = nodes.size();

		uint8_t child_valid_return = 0;
		uint8_t child_valid_inner = 0;
		if (1 == depth) {
			for (size_t i = 0; 8 != i; ++i) {
				auto& child = this->getLeafChild(node, i);
				if (isModified(child)) {
					child_valid_return |= 1U << i;
				}
			}
		} else {
			for (size_t i = 0; 8 != i; ++i) {
				auto& child = this->getInnerChild(node, i);
				if (isModified(child)) {
					if (isLeaf(child)) {
						child_valid_return |= 1U << i;
					} else {
						child_valid_inner |= 1U << i;
					}
				}
			}
		}

		indicators.push_back(child_valid_return);
		indicators.push_back(child_valid_inner);

		if (0 == child_valid_return && 0 == child_valid_inner) {
			return;
		}

		if (1 == depth) {
			for (size_t i = 0; 8 != i; ++i) {
				auto& child = this->getLeafChild(node, i);
				if (isModified(child)) {
					nodes.push_back(child);
					derived().updateNodeIndicators(child);
					setModified(child, false);
				}
			}
		} else {
			for (size_t i = 0; 8 != i; ++i) {
				auto& child = this->getInnerChild(node, i);
				if (isModified(child)) {
					if (isLeaf(child)) {
						nodes.push_back(child);
						derived().updateNodeIndicators(node, depth);
					} else {
						modifiedDataRecurs(indicators, nodes, child, depth - 1);
						derived().updateNode(child, depth - 1);
						derived().updateNodeIndicators(child, depth - 1);
						pruneNode(child, depth - 1);
					}
					setModified(child, false);
				}
			}
		}

		if (nodes.size() == cur_nodes_size) {
			indicators.resize(cur_indicators_size + 2);
			indicators.back() = 0U;
			*std::prev(std::end(indicators), 2) = 0U;
		}
	}

	void writeIndicators(std::ostream& out_stream,
	                     std::vector<uint8_t> const& indicators) const
	{
		uint64_t num = indicators.size();
		out_stream.write(reinterpret_cast<char*>(&num), sizeof(num));
		out_stream.write(reinterpret_cast<char const*>(indicators.data()),
		                 sizeof(uint8_t) * num);
	}

 protected:
	// The number of depth levels
	depth_t depth_levels_;
	// The maximum coordinate value the octree can store
	Key::key_t max_value_;

	// Stores the size of a node at a given depth, where the depth is the index
	std::array<double, maxDepthLevels()> node_size_;
	// Reciprocal of the node size at a given depth, where the depth is the index
	std::array<double, maxDepthLevels()> node_size_factor_;

	// Automatic pruning
	bool automatic_pruning_enabled_ = true;

	// depth_t at which parallel execution happens
	depth_t parallel_depth_ = 12;
};

}  // namespace ufo::map

#endif  // UFO_MAP_OCTREE_BASE_H