/*
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * License: BSD 3
 */

/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, D. Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
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
	static constexpr const DepthType MIN_DEPTH_LEVELS = 2;
	// Maximum number of depth levels
	static constexpr const DepthType MAX_DEPTH_LEVELS = 21;

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

	void clear(bool prune = false) { clear(getResolution(), getTreeDepthLevels(), prune); }

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void clear(ExecutionPolicy policy, bool prune = false)
	{
		clear(policy, getResolution(), getTreeDepthLevels(), prune);
	}

	void clear(float new_resolution, DepthType new_depth_levels, bool prune = false)
	{
		if (getMinTreeDepthLevels() > new_depth_levels ||
		    getMaxTreeDepthLevels() < new_depth_levels) {
			throw std::invalid_argument(
			    "depth_levels can be minimum " + std::to_string(getMinTreeDepthLevels()) +
			    " and maximum " + std::to_string(getMaxTreeDepthLevels()) + ", '" +
			    std::to_string(+new_depth_levels) + "' was supplied.");
		}

		if (root_) {
			deleteChildren(getRoot(), getTreeDepthLevels(), prune);
		}

		depth_levels_ = new_depth_levels;
		max_value_ = std::pow(2, getTreeDepthLevels() - 1);

		resolution_ = new_resolution;
		resolution_factor_ = 1.0 / new_resolution;

		nodes_half_sizes_[0] = resolution_ / 2.0;
		nodes_half_sizes_[1] = resolution_;
		for (std::size_t i = 2; i <= getTreeDepthLevels(); ++i) {
			nodes_half_sizes_[i] = nodes_half_sizes_[i - 1] * 2.0;
		}

		initRoot();
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void clear(ExecutionPolicy policy, float new_resolution, DepthType new_depth_levels,
	           bool prune = false)
	{
		if (getMinTreeDepthLevels() > new_depth_levels ||
		    getMaxTreeDepthLevels() < new_depth_levels) {
			throw std::invalid_argument(
			    "depth_levels can be minimum " + std::to_string(getMinTreeDepthLevels()) +
			    " and maximum " + std::to_string(getMaxTreeDepthLevels()) + ", '" +
			    std::to_string(+new_depth_levels) + "' was supplied.");
		}

		if (root_) {
			deleteChildren(policy, getRoot(), getTreeDepthLevels(), prune);
		}

		depth_levels_ = new_depth_levels;
		max_value_ = std::pow(2, getTreeDepthLevels() - 1);

		resolution_ = new_resolution;
		resolution_factor_ = 1.0 / new_resolution;

		nodes_half_sizes_[0] = resolution_ / 2.0;
		nodes_half_sizes_[1] = resolution_;
		for (std::size_t i = 2; i <= getTreeDepthLevels(); ++i) {
			nodes_half_sizes_[i] = nodes_half_sizes_[i - 1] * 2.0;
		}

		initRoot();
	}

	//
	// Resolution
	//

	[[nodiscard]] constexpr float getResolution() const noexcept { return resolution_; }

	//
	// Automatic pruning
	//

	[[nodiscard]] constexpr bool isAutomaticPruningEnabled() const noexcept
	{
		return automatic_pruning_enabled_;
	}

	constexpr void enableAutomaticPruning(bool enable) noexcept
	{
		automatic_pruning_enabled_ = enable;
	}

	//
	// Parallel execution depth
	//

	constexpr DepthType getParallelExecutionDepth() const noexcept
	{
		return parallel_depth_;
	}

	constexpr void setParallelExecutionDepth(DepthType new_parallel_depth) noexcept
	{
		parallel_depth_ = new_parallel_depth;
	}

	//
	// Tree depth levels
	//

	constexpr DepthType getTreeDepthLevels() const noexcept { return depth_levels_; }

	static constexpr DepthType getMinTreeDepthLevels() noexcept { return MIN_DEPTH_LEVELS; }

	static constexpr DepthType getMaxTreeDepthLevels() noexcept { return MAX_DEPTH_LEVELS; }

	//
	// To code
	//

	Code toCode(Key key) const noexcept { return Code(key); }

	Code toCode(Point3 coord, DepthType depth = 0) const noexcept
	{
		return toCode(toKey(coord, depth));
	}

	Code toCode(float x, float y, float z, DepthType depth = 0) const noexcept
	{
		return toCode(toKey(x, y, z, depth));
	}

	std::optional<Code> toCodeChecked(Point3 coord, DepthType depth = 0) const noexcept
	{
		std::optional<KeyType> key = toKeyChecked(coord, depth);
		return key.has_value() ? std::optional<Code>(toCode(key)) : std::nullopt;
	}

	std::optional<Code> toCodeChecked(float x, float y, float z,
	                                  DepthType depth = 0) const noexcept
	{
		return toCodeChecked(Point3(x, y, z), depth);
	}

	//
	// To key
	//

	Key toKey(Code code) const noexcept { return code.toKey(); }

	KeyType toKey(float coord, DepthType depth = 0) const noexcept
	{
		KeyType val = std::floor(resolution_factor_ * coord);
		return ((val + max_value_) >> depth) << depth;

		// static constexpr auto add64 = std::plus<int_fast64_t>{};
		// KeyType val = add64(std::floor(resolution_factor_ * coord), max_value_);
		// return (val >> depth) << depth;
	}

	Key toKey(Point3 coord, DepthType depth = 0) const noexcept
	{
		return Key(toKey(coord[0], depth), toKey(coord[1], depth), toKey(coord[2], depth),
		           depth);
	}

	Key toKey(float x, float y, float z, DepthType depth = 0) const noexcept
	{
		return Key(toKey(x, depth), toKey(y, depth), toKey(z, depth), depth);
	}

	std::optional<KeyType> toKeyChecked(float coord, DepthType depth = 0) const noexcept
	{
		if (getMin()[0] > coord || getMax()[0] < coord) {
			return std::nullopt;
		}
		return toKey(coord, depth);
	}

	std::optional<Key> toKeyChecked(Point3 coord, DepthType depth = 0) const noexcept
	{
		std::optional<KeyType> x = toKeyChecked(coord[0], depth);
		if (!x.has_value()) {
			return std::nullopt;
		}
		std::optional<KeyType> y = toKeyChecked(coord[1], depth);
		if (!y.has_value()) {
			return std::nullopt;
		}
		std::optional<KeyType> z = toKeyChecked(coord[2], depth);
		if (!z.has_value()) {
			return std::nullopt;
		}
		return Key(x.value(), y.value(), z.value(), depth);
	}

	std::optional<Key> toKeyChecked(float x, float y, float z,
	                                DepthType depth = 0) const noexcept
	{
		return toKeyChecked(Point3(x, y, z), depth);
	}

	//
	// To coordinate
	//

	Point3 toCoord(Code code) const noexcept { return toCoord(toKey(code)); }

	float toCoord(KeyType key, DepthType depth = 0) const noexcept
	{
		static constexpr auto sub64 = std::minus<int_fast64_t>{};
		return depth_levels_ == depth
		           ? 0.0f
		           : (std::floor(sub64(key, max_value_) / static_cast<float>(1U << depth)) +
		              0.5f) *
		                 getNodeSize(depth);
	}

	Point3 toCoord(Key key) const noexcept
	{
		return Point3(toCoord(key[0], key.getDepth()), toCoord(key[1], key.getDepth()),
		              toCoord(key[2], key.getDepth()));
	}

	Point3 toCoord(Key key, DepthType depth) const noexcept
	{
		return Point3(toCoord(key[0], depth), toCoord(key[1], depth), toCoord(key[2], depth));
	}

	std::optional<Point3> toCoordChecked(Key key, DepthType depth) const noexcept
	{
		if (key.getDepth() > depth) {
			return std::nullopt;
		}
		return toCoord(key, depth);
	}

	//
	// Is inside
	//

	bool isInside(Point3 point) const noexcept { return inBBX(point, getMin(), getMax()); }

	static bool inBBX(Point3 point, Point3 bbx_min, Point3 bbx_max) noexcept
	{
		return bbx_min.x() <= point.x() && bbx_max.x() >= point.x() &&
		       bbx_min.y() <= point.y() && bbx_max.y() >= point.y() &&
		       bbx_min.z() <= point.z() && bbx_max.z() >= point.z();
	}

	//
	// Move line into octree bounds
	//

	std::optional<std::pair<Point3, Point3>> moveLineInside(Point3 origin,
	                                                        Point3 end) const noexcept
	{
		Point3 bbx_min = getMin();
		Point3 bbx_max = getMax();

		if ((origin[0] < bbx_min[0] && end[0] < bbx_min[0]) ||
		    (origin[0] > bbx_max[0] && end[0] > bbx_max[0]) ||
		    (origin[1] < bbx_min[1] && end[1] < bbx_min[1]) ||
		    (origin[1] > bbx_max[1] && end[1] > bbx_max[1]) ||
		    (origin[2] < bbx_min[2] && end[2] < bbx_min[2]) ||
		    (origin[2] > bbx_max[2] && end[2] > bbx_max[2])) {
			return std::nullopt;
		}

		if (inBBX(origin, bbx_min, bbx_max) && inBBX(end, bbx_min, bbx_max)) {
			return std::optional<std::pair<Point3, Point3>>(std::in_place, origin, end);
		}

		int hits = 0;
		std::array<Point3, 2> hit;
		for (int i = 0; i < 3 && hits < 2; ++i) {
			if (getIntersection(origin[i] - bbx_min[i], end[i] - bbx_min[i], origin, end,
			                    &hit[hits]) &&
			    inBBX(hit[hits], i, bbx_min, bbx_max)) {
				++hits;
			}
		}
		for (int i = 0; i < 3 && hits < 2; ++i) {
			if (getIntersection(origin[i] - bbx_max[i], end[i] - bbx_max[i], origin, end,
			                    &hit[hits]) &&
			    inBBX(hit[hits], i, bbx_min, bbx_max)) {
				++hits;
			}
		}

		switch (hits) {
			case 1:
				if (inBBX(origin, bbx_min, bbx_max)) {
					end = hit[0];
				} else {
					origin = hit[0];
				}
				break;
			case 2:
				if (((origin - hit[0]).squaredNorm() + (end - hit[1]).squaredNorm()) <=
				    ((origin - hit[1]).squaredNorm() + (end - hit[0]).squaredNorm())) {
					origin = hit[0];
					end = hit[1];
				} else {
					origin = hit[1];
					end = hit[0];
				}
		}

		return std::optional<std::pair<Point3, Point3>>(std::in_place, origin, end);
	}

	//
	// Get node size
	//

	constexpr float getNodeSize(DepthType depth) const
	{
		return getNodeHalfSize(depth + 1);
	}

	constexpr float getNodeHalfSize(DepthType depth) const
	{
		return nodes_half_sizes_[depth];
	}

	//
	// Is pure leaf
	//

	static constexpr bool isPureLeaf(MinimalNode const& node)
	{
		return 0 == node.getDepth();
	}

	//
	// Is leaf
	//

	static constexpr bool isLeaf(MinimalNode const& node)
	{
		return isPureLeaf(node) || isLeaf(getInnerNode(node));
	}

	//
	// Has children
	//

	static constexpr bool hasChildren(MinimalNode const& node) { return !isLeaf(node); }

	//
	// Get child
	//

	static Node getChild(Node const& node, size_t child_idx)
	{
		LeafNode& child = getChild(getInnerNode(node), node.getDepth() - 1, child_idx);

		float child_half_size = node.getHalfSize() / 2.0;
		geometry::AAEBB child_aaebb(
		    getChildCenter(node.getCenter(), child_half_size, child_idx), child_half_size);

		return Node(child_aaebb, &child, node.getCode().getChild(child_idx));
	}

	static Node getChildChecked(Node const& node, size_t child_idx)
	{
		if (!hasChildren(node)) {
			throw std::out_of_range("Node has no children");
		} else if (7 < child_idx) {
			throw std::out_of_range("child_idx out of range");
		}
		return getChild(node, child_idx);
	}

	static MinimalNode getChild(MinimalNode const& node, size_t child_idx)
	{
		LeafNode& child = getChild(getInnerNode(node), node.getDepth() - 1, child_idx);

		return MinimalNode(&child, node.getCode().getChild(child_idx));
	}

	static MinimalNode getChildChecked(MinimalNode const& node, size_t child_idx)
	{
		if (!hasChildren(node)) {
			throw std::out_of_range("MinimalNode has no children");
		} else if (7 < child_idx) {
			throw std::out_of_range("child_idx out of range");
		}
		return getChild(node, child_idx);
	}

	//
	// Get Sibling
	//

	static Node getSibling(Node const& node, size_t sibling_idx)
	{
		auto cur_idx = node.getCode().getChildIdx(node.getDepth());

		if (sibling_idx == cur_idx) {
			return node;
		}

		LeafNode* sibling;
		int idx_diff = static_cast<int>(sibling_idx) - static_cast<int>(cur_idx);
		if (isPureLeaf(node)) {
			sibling = const_cast<LeafNode*>(std::next(&getLeafNode(node), idx_diff));
		} else {
			sibling = static_cast<LeafNode*>(
			    const_cast<InnerNode*>(std::next(&getInnerNode(node), idx_diff)));
		}

		geometry::AAEBB sibling_aaebb(
		    getSiblingCenter(node.getCenter(), node.getHalfSize(), cur_idx, sibling_idx),
		    node.getHalfSize());

		return Node(sibling_aaebb, sibling, node.getCode().getSibling(sibling_idx));
	}

	Node getSiblingChecked(Node const& node, size_t sibling_idx)
	{
		if (!isRoot(node)) {
			throw std::out_of_range("Node has no siblings");
		} else if (7 < sibling_idx) {
			throw std::out_of_range("sibling_idx out of range");
		}
		return getSibling(node, sibling_idx);
	}

	static MinimalNode getSibling(MinimalNode const& node, size_t sibling_idx)
	{
		auto cur_idx = node.getCode().getChildIdx(node.getDepth());

		if (sibling_idx == cur_idx) {
			return node;
		}

		LeafNode* sibling;
		int idx_diff = static_cast<int>(sibling_idx) - static_cast<int>(cur_idx);
		if (isPureLeaf(node)) {
			sibling = const_cast<LeafNode*>(std::next(&getLeafNode(node), idx_diff));
		} else {
			sibling = static_cast<LeafNode*>(
			    const_cast<InnerNode*>(std::next(&getInnerNode(node), idx_diff)));
		}

		return MinimalNode(sibling, node.getCode().getSibling(sibling_idx));
	}

	MinimalNode getSiblingChecked(MinimalNode const& node, size_t sibling_idx)
	{
		if (!isRoot(node)) {
			throw std::out_of_range("MinimalNode has no siblings");
		} else if (7 < sibling_idx) {
			throw std::out_of_range("sibling_idx out of range");
		}
		return getSibling(node, sibling_idx);
	}

	//
	// Modified
	//

	static constexpr bool isModified(MinimalNode const& node)
	{
		return getLeafNode(node).modified;
	}

	//
	// Is root
	//

	bool isRoot(MinimalNode const& node) { return getRootMinimalNode() == node; }

	//
	// Get min/max coordinate octree can store
	//

	/**
	 * @return The minimum point the octree can store
	 */
	Point3 getMin() const
	{
		float half_size = -getNodeHalfSize(getTreeDepthLevels());
		return Point3(half_size, half_size, half_size);
	}

	/**
	 * @return The maximum point the octree can store
	 */
	Point3 getMax() const
	{
		float half_size = getNodeHalfSize(getTreeDepthLevels());
		return Point3(half_size, half_size, half_size);
	}

	//
	// Get root
	//

	Node getRootNode() noexcept
	{
		return Node(getRootBoundingVolume(), &getRoot(), getRootCode());
	}

	Node getRootNode() const noexcept
	{
		// TODO: Look at
		return Node(getRootBoundingVolume(), const_cast<InnerNode*>(&getRoot()),
		            getRootCode());
	}

	MinimalNode getRootMinimalNode() noexcept
	{
		return MinimalNode(&getRoot(), getRootCode());
	}

	MinimalNode getRootMinimalNode() const noexcept
	{
		// TODO: Look at
		return MinimalNode(const_cast<InnerNode*>(&getRoot()), getRootCode());
	}

	Code getRootCode() const noexcept { return Code(0, getTreeDepthLevels()); }

	geometry::AAEBB getRootBoundingVolume() const noexcept
	{
		return geometry::AAEBB(geometry::Point(0, 0, 0),
		                       getNodeHalfSize(getTreeDepthLevels()));
	}

	//
	// Get node
	//

	Node getNode(Code code) const
	{
		// FIXME: Handle wrong codes

		Node node(getRootNode());

		while (hasChildren(node) && code.getDepth() != node.getDepth()) {
			node = getChild(node, code.getChildIdx(node.getDepth() - 1));
		}

		return node;
	}

	Node getNode(Key key) const { return getNode(toCode(key)); }

	Node getNode(Point3 coord, DepthType depth = 0) const
	{
		return getNode(toCode(coord, depth));
	}

	Node getNode(float x, float y, float z, DepthType depth = 0) const
	{
		return getNode(toCode(x, y, z, depth));
	}

	//
	// Get minimal node
	//

	MinimalNode getMinimalNode(Code code) const
	{
		// FIXME: Handle wrong codes

		MinimalNode node(getRootMinimalNode());

		while (hasChildren(node) && code.getDepth() != node.getDepth()) {
			node = getChild(node, code.getChildIdx(node.getDepth() - 1));
		}

		return node;
	}

	MinimalNode getMinimalNode(Key key) const { return getMinimalNode(toCode(key)); }

	MinimalNode getMinimalNode(Point3 coord, DepthType depth = 0) const
	{
		return getMinimalNode(toCode(coord, depth));
	}

	MinimalNode getMinimalNode(float x, float y, float z, DepthType depth = 0) const
	{
		return getMinimalNode(toCode(x, y, z, depth));
	}

	//
	// Create node
	//

	Node createNode(Code code)
	{
		// FIXME: Handle wrong codes

		Node node(getRootNode());

		while (code.getDepth() != node.getDepth()) {
			if (isLeaf(node)) {
				createChildren(getInnerNode(node), node.getDepth(),
				               node.getCode().getChildIdx(node.getDepth()));
			}
			node = getChild(node, code.getChildIdx(node.getDepth() - 1));
		}

		return node;
	}

	Node createNode(Key key) { return createNode(toCode(key)); }

	Node createNode(Point3 coord, DepthType depth = 0)
	{
		return createNode(toCode(coord, depth));
	}

	Node createNode(float x, float y, float z, DepthType depth = 0)
	{
		return createNode(toCode(x, y, z, depth));
	}

	//
	// Create minimal node
	//

	MinimalNode createMinimalNode(Code code)
	{
		InnerNode* node = &getRoot();
		auto const min_depth = std::max(static_cast<DepthType>(1), code.getDepth());
		DepthType cur_depth = getTreeDepthLevels();
		for (; min_depth < cur_depth; --cur_depth) {
			if (isLeaf(*node)) {
				createInnerChildren(*node, cur_depth, code.getChildIdx(cur_depth));
			}
			node = &getInnerChild(*node, code.getChildIdx(cur_depth - 1));
		}

		if (0 == code.getDepth()) {
			if (isLeaf(*node)) {
				createLeafChildren(*node, code.getChildIdx(1));
			}
			return MinimalNode(&getLeafChild(*node, code.getChildIdx(0)), code);
		} else {
			return MinimalNode(node, code);
		}
	}

	MinimalNode createMinimalNode(Key key) { return createMinimalNode(toCode(key)); }

	MinimalNode createMinimalNode(Point3 coord, DepthType depth = 0)
	{
		return createMinimalNode(toCode(coord, depth));
	}

	MinimalNode createMinimalNode(float x, float y, float z, DepthType depth = 0)
	{
		return createMinimalNode(toCode(x, y, z, depth));
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
		return query(std::execution::seq, predicates, d_first);
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
		return queryK(std::execution::seq, k, predicates, d_first);
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
		return queryNearest(std::execution::seq, geometry, predicates, d_first, epsilon);
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
		return queryNearestK(std::execution::seq, k, geometry, predicates, d_first, epsilon);
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
		    new Iterator(dynamic_cast<Derived const*>(this), getRootNode(), predicates));
	}

	const_query_iterator endQuery() const
	{
		return const_query_iterator(new Iterator<Derived>(getRootNode()));
	}

	template <class Geometry, class Predicates>
	const_query_nearest_iterator beginQueryNearest(Geometry const& geometry,
	                                               Predicates const& predicates,
	                                               float epsilon = 0.0f) const
	{
		return const_query_nearest_iterator(
		    new NearestIterator(dynamic_cast<Derived const*>(this), getRootNode(), geometry,
		                        predicates, epsilon));
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
		auto root = getRootNode();
		if (!f(root)) {
			traverseRecurs(f, root);
		}
	}

	template <class UnaryFunction>
	void traverse(UnaryFunction f) const
	{
		auto root = getRootNode();
		if (!f(root)) {
			traverseRecurs(f, root);
		}
	}

	//
	// Memory
	//

	/**
	 * @return std::size_t number of inner nodes in the tree
	 */
	constexpr std::size_t getNumInnerNodes() const noexcept
	{
		return num_inner_nodes_ + num_inner_leaf_nodes_;
	}

	/**
	 * @return std::size_t number of leaf nodes in the tree
	 */
	constexpr std::size_t getNumLeafNodes() const noexcept { return num_leaf_nodes_; }

	/**
	 * @brief This is lower bound memeory usage of an inner node.
	 *
	 * Note: Additional data accessed by pointers inside an inner node are not counted.
	 *
	 * @return std::size_t memory usage of a single inner node
	 */
	constexpr std::size_t memoryUsageInnerNode() const noexcept
	{
		return sizeof(InnerNode);
	}

	/**
	 * @brief This is lower bound memeory usage of a leaf node.
	 *
	 * Note: Additional data accessed by pointers inside a leaf node are not counted.
	 *
	 * @return std::size_t memory usage of a single leaf node
	 */
	constexpr std::size_t memoryUsageLeafNode() const noexcept { return sizeof(LeafNode); }

	/**
	 * @brief Lower bound memory usage for all nodes.
	 *
	 * Note: Does not account for pointed to data inside nodes.
	 *
	 * @return std::size_t memory usage of the octree
	 */
	virtual std::size_t memoryUsage() const noexcept
	{
		return (getNumInnerNodes() * memoryUsageInnerNode()) +
		       (getNumLeafNodes() * memoryUsageLeafNode());
	}

	/**
	 * @return std::size_t number of nodes in the tree
	 */
	constexpr std::size_t size() const noexcept
	{
		return getNumInnerNodes() + getNumLeafNodes();
	}

	/**
	 * @return std::size_t number of inner nodes in the tree
	 */
	constexpr std::size_t getNumAllocatedInnerNodes() const noexcept
	{
		return num_allocated_inner_nodes_ + num_allocated_inner_leaf_nodes_;
	}

	/**
	 * @return std::size_t number of leaf nodes in the tree
	 */
	constexpr std::size_t getNumAllocatedLeafNodes() const noexcept
	{
		return num_allocated_leaf_nodes_;
	}

	/**
	 * @brief Lower bound memory usage for all nodes.
	 *
	 * Note: Does not account for pointed to data inside nodes.
	 *
	 * @return std::size_t memory usage of the octree
	 */
	virtual std::size_t memoryUsageAllocated() const noexcept
	{
		return (getNumAllocatedInnerNodes() * memoryUsageInnerNode()) +
		       (getNumAllocatedLeafNodes() * memoryUsageLeafNode());
	}

	/**
	 * @return std::size_t number of nodes in the tree
	 */
	constexpr std::size_t sizeAllocated() const noexcept
	{
		return getNumAllocatedInnerNodes() + getNumAllocatedLeafNodes();
	}

	//
	// Update modified nodes
	//

	void updateModifiedNodes(bool keep_modified = false,
	                         DepthType max_depth = getMaxTreeDepthLevels())
	{
		if (keep_modified) {
			updateModifiedNodesRecurs<true>(max_depth, getRoot(), getTreeDepthLevels());
		} else {
			updateModifiedNodesRecurs<false>(max_depth, getRoot(), getTreeDepthLevels());
		}
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void updateModifiedNodes(ExecutionPolicy policy, bool keep_modified = false,
	                         DepthType max_depth = getMaxTreeDepthLevels())
	{
		if (keep_modified) {
			updateModifiedNodesRecurs<true>(policy, max_depth, getRoot(), getTreeDepthLevels());
		} else {
			updateModifiedNodesRecurs<false>(policy, max_depth, getRoot(),
			                                 getTreeDepthLevels());
		}
	}

	void updateModifiedNodes(MinimalNode const& node, bool keep_modified = false,
	                         DepthType max_depth = getMaxTreeDepthLevels())
	{
		// TODO: Implement
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void updateModifiedNodes(ExecutionPolicy policy, MinimalNode const& node,
	                         bool keep_modified = false,
	                         DepthType max_depth = getMaxTreeDepthLevels())
	{
		// TODO: Implement
		// if ()

		// updateModifiedNodesRecurs(policy, max_depth, )
	}

	void updateModifiedNodes(Code code, bool keep_modified = false,
	                         DepthType max_depth = getMaxTreeDepthLevels())
	{
		// TODO: Implement
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void updateModifiedNodes(ExecutionPolicy policy, Code code, bool keep_modified = false,
	                         DepthType max_depth = getMaxTreeDepthLevels())
	{
		// TODO: Implement
	}

	void updateModifiedNodes(Key key, bool keep_modified = false,
	                         DepthType max_depth = getMaxTreeDepthLevels())
	{
		updateModifiedNodes(toCode(key), keep_modified, max_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void updateModifiedNodes(ExecutionPolicy policy, Key key, bool keep_modified = false,
	                         DepthType max_depth = getMaxTreeDepthLevels())
	{
		updateModifiedNodes(policy, toCode(key), keep_modified, max_depth);
	}

	void updateModifiedNodes(Point3 coord, DepthType depth = 0, bool keep_modified = false,
	                         DepthType max_depth = getMaxTreeDepthLevels())
	{
		updateModifiedNodes(toCode(coord, depth), depth, keep_modified, max_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void updateModifiedNodes(ExecutionPolicy policy, Point3 coord, DepthType depth = 0,
	                         bool keep_modified = false,
	                         DepthType max_depth = getMaxTreeDepthLevels())
	{
		updateModifiedNodes(policy, toCode(coord, depth), keep_modified, max_depth);
	}

	void updateModifiedNodes(double x, double y, double z, DepthType depth = 0,
	                         bool keep_modified = false,
	                         DepthType max_depth = getMaxTreeDepthLevels())
	{
		updateModifiedNodes(toCode(x, y, z, depth), keep_modified, max_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void updateModifiedNodes(ExecutionPolicy policy, double x, double y, double z,
	                         DepthType depth = 0, bool keep_modified = false,
	                         DepthType max_depth = getMaxTreeDepthLevels())
	{
		updateModifiedNodes(policy, toCode(x, y, z, depth), keep_modified, max_depth);
	}

	//
	// Set modified
	//

	void setModified(DepthType min_depth = 0)
	{
		setModifiedRecurs(getRoot(), getTreeDepthLevels(), min_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setModified(ExecutionPolicy policy, DepthType min_depth = 0)
	{
		// FIXME: Update
		setModifiedRecurs(getRoot(), getTreeDepthLevels(), min_depth);
	}

	void setModified(MinimalNode const& node, DepthType min_depth = 0)
	{
		// TODO: Implement
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setModified(ExecutionPolicy policy, MinimalNode const& node,
	                 DepthType min_depth = 0)
	{
		// TODO: Implement
	}

	void setModified(Code code, DepthType min_depth = 0)
	{
		// TODO: Implement
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setModified(ExecutionPolicy policy, Code code, DepthType min_depth = 0)
	{
		// TODO: Implement
	}

	void setModified(Key key, DepthType min_depth = 0)
	{
		setModified(toCode(key), min_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setModified(ExecutionPolicy policy, Key key, DepthType min_depth = 0)
	{
		setModified(policy, toCode(key), min_depth);
	}

	void setModified(Point3 coord, DepthType depth = 0, DepthType min_depth = 0)
	{
		setModified(toCode(coord, depth), min_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setModified(ExecutionPolicy policy, Point3 coord, DepthType depth = 0,
	                 DepthType min_depth = 0)
	{
		setModified(policy, toCode(coord, depth), min_depth);
	}

	void setModified(double x, double y, double z, DepthType depth = 0,
	                 DepthType min_depth = 0)
	{
		setModified(toCode(x, y, z, depth), min_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setModified(ExecutionPolicy policy, double x, double y, double z,
	                 DepthType depth = 0, DepthType min_depth = 0)
	{
		setModified(policy, toCode(x, y, z, depth), min_depth);
	}

	//
	// Clear modified
	//

	void clearModified(DepthType max_depth = getMaxTreeDepthLevels())
	{
		clearModifiedRecurs(getRoot(), getTreeDepthLevels(), max_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void clearModified(ExecutionPolicy policy,
	                   DepthType max_depth = getMaxTreeDepthLevels())
	{
		// FIXME: Update
		clearModifiedRecurs(getRoot(), getTreeDepthLevels(), max_depth);
	}

	void clearModified(MinimalNode const& node,
	                   DepthType max_depth = getMaxTreeDepthLevels())
	{
		if (isLeaf(node)) {
			getLeafNode(node).modified = false;
		} else {
			clearModifiedRecurs(getInnerNode(node), node.getDepth(), max_depth);
		}
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void clearModified(ExecutionPolicy policy, MinimalNode const& node,
	                   DepthType max_depth = getMaxTreeDepthLevels())
	{
		// FIXME: Update
		if (isPureLeaf(node)) {
			getLeafNode(node).modified = false;
		} else {
			clearModifiedRecurs(getInnerNode(node), node.getDepth(), max_depth);
		}
	}

	void clearModified(Code code, DepthType max_depth = getMaxTreeDepthLevels())
	{
		if (0 == code.getDepth()) {
			getLeafNode(code).modified = false;
		} else {
			clearModifiedRecurs(getInnerNode(code), code.getDepth(), max_depth);
		}
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void clearModified(ExecutionPolicy policy, Code code,
	                   DepthType max_depth = getMaxTreeDepthLevels())
	{
		// FIXME: Update
		if (0 == code.getDepth()) {
			getLeafNode(code).modified = false;
		} else {
			clearModifiedRecurs(getInnerNode(code), code.getDepth(), max_depth);
		}
	}

	void clearModified(Key key, DepthType max_depth = getMaxTreeDepthLevels())
	{
		clearModified(toCode(key), max_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void clearModified(ExecutionPolicy policy, Key key,
	                   DepthType max_depth = getMaxTreeDepthLevels())
	{
		clearModified(policy, toCode(key), max_depth);
	}

	void clearModified(Point3 coord, DepthType depth = 0,
	                   DepthType max_depth = getMaxTreeDepthLevels())
	{
		clearModified(toCode(coord, depth), max_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void clearModified(ExecutionPolicy policy, Point3 coord, DepthType depth = 0,
	                   DepthType max_depth = getMaxTreeDepthLevels())
	{
		clearModified(policy, toCode(coord, depth), max_depth);
	}

	void clearModified(double x, double y, double z, DepthType depth = 0,
	                   DepthType max_depth = getMaxTreeDepthLevels())
	{
		clearModified(toCode(x, y, z, depth), max_depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void clearModified(ExecutionPolicy policy, double x, double y, double z,
	                   DepthType depth = 0, DepthType max_depth = getMaxTreeDepthLevels())
	{
		clearModified(policy, toCode(x, y, z, depth), max_depth);
	}

	//
	// Input/output (read/write)
	//

	bool canMerge(std::filesystem::path const& filename) const
	{
		std::ifstream file;
		file.exceptions(std::ifstream::failbit | std::ifstream::badbit);

		file.open(filename, std::ios_base::in | std::ios_base::binary);
		return canMerge(file);
	}

	bool canMerge(std::istream& in_stream) const
	{
		auto pos = in_stream.tellg();
		FileInfo header = readHeader(in_stream);
		in_stream.seekg(pos);

		float resolution;
		DepthType depth_levels;
		std::istringstream(header.at("resolution").at(0)) >> resolution;
		uint32_t tmp;
		std::istringstream(header.at("depth_levels").at(0)) >> tmp;
		depth_levels = tmp;

		return getResolution() == resolution && getTreeDepthLevels() == depth_levels;
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
		float resolution;
		DepthType depth_levels;

		std::istringstream(header.at("resolution").at(0)) >> resolution;
		uint32_t tmp;
		std::istringstream(header.at("depth_levels").at(0)) >> tmp;
		depth_levels = tmp;

		if (getResolution() != resolution || getTreeDepthLevels() != depth_levels) {
			clear(resolution, depth_levels);
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

	void write(std::filesystem::path const& filename, DepthType min_depth = 0,
	           bool compress = false, int compression_acceleration_level = 1,
	           int compression_level = 0) const
	{
		write(filename, predicate::TRUE(), min_depth, compress,
		      compression_acceleration_level, compression_level);
	}

	void write(std::ostream& out_stream, DepthType min_depth = 0, bool compress = false,
	           int compression_acceleration_level = 1, int compression_level = 0) const
	{
		write(out_stream, predicate::TRUE(), min_depth, compress,
		      compression_acceleration_level, compression_level);
	}

	template <class Predicates, typename = std::enable_if_t<!std::is_scalar_v<Predicates>>>
	void write(std::filesystem::path const& filename, Predicates const& predicates,
	           DepthType min_depth = 0, bool compress = false,
	           int compression_acceleration_level = 1, int compression_level = 0) const
	{
		std::ofstream file;
		file.exceptions(std::ofstream::failbit | std::ofstream::badbit);

		file.open(filename, std::ios_base::out | std::ios_base::binary);

		write(file, predicates, min_depth, compress, compression_acceleration_level,
		      compression_level);
	}

	template <class Predicates, typename = std::enable_if_t<!std::is_scalar_v<Predicates>>>
	void write(std::ostream& out_stream, Predicates const& predicates,
	           DepthType min_depth = 0, bool compress = false,
	           int compression_acceleration_level = 1, int compression_level = 0) const
	{
		writeHeader(out_stream, getFileInfo());
		writeData(out_stream, predicates, min_depth, compress, compression_acceleration_level,
		          compression_level);
	}

 protected:
	//
	// Constructors
	//

	OctreeBase(float resolution = 0.1, DepthType depth_levels = 16,
	           bool automatic_pruning = true)
	    : resolution_(resolution),
	      resolution_factor_(1.0 / resolution),
	      depth_levels_(depth_levels),
	      max_value_(std::pow(2, depth_levels - 1)),
	      automatic_pruning_enabled_(automatic_pruning)
	{
		printf("OctreeBase constructor\n");
		if (getMinTreeDepthLevels() > depth_levels ||
		    getMaxTreeDepthLevels() < depth_levels) {
			throw std::invalid_argument("'depth_levels' has to be [" +
			                            std::to_string(getMinTreeDepthLevels()) + ", " +
			                            std::to_string(getMaxTreeDepthLevels()) + "], '" +
			                            std::to_string(+depth_levels) + "' was supplied.");
		}

		// Precompute sizes
		nodes_half_sizes_[0] = resolution_ / 2.0;
		nodes_half_sizes_[1] = resolution_;
		for (size_t i = 2; i <= depth_levels_; ++i) {
			nodes_half_sizes_[i] = nodes_half_sizes_[i - 1] * 2.0;
		}

		init();
	}

	OctreeBase(OctreeBase const& other)
	    : resolution_(other.resolution_),
	      resolution_factor_(other.resolution_factor_),
	      depth_levels_(other.depth_levels_),
	      max_value_(other.max_value_),
	      nodes_half_sizes_(other.nodes_half_sizes_),
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
	    : resolution_(other.resolution_),
	      resolution_factor_(other.resolution_factor_),
	      depth_levels_(other.depth_levels_),
	      max_value_(other.max_value_),
	      nodes_half_sizes_(other.nodes_half_sizes_),
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
	    : resolution_(std::move(other.resolution_)),
	      resolution_factor_(std::move(other.resolution_factor_)),
	      depth_levels_(std::move(other.depth_levels_)),
	      max_value_(std::move(other.max_value_)),
	      root_(std::move(other.root_)),
	      nodes_half_sizes_(std::move(other.nodes_half_sizes_)),
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

	void init()
	{
		size_t i = 0;
		for (auto& a_l : children_locks_) {
			for (auto& l : a_l) {
				l.clear();
			}
		}
	}

	OctreeBase& operator=(OctreeBase const& rhs)
	{
		printf("OctreeBase copy assignment\n");

		// TODO: Should this clear?
		clear(rhs.resolution_, rhs.depth_levels_);

		resolution_ = rhs.resolution_;
		resolution_factor_ = rhs.resolution_factor_;
		depth_levels_ = rhs.depth_levels_;
		max_value_ = rhs.max_value_;
		nodes_half_sizes_ = rhs.nodes_half_sizes_;
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
		clear(rhs.resolution_, rhs.depth_levels_);

		resolution_ = rhs.resolution_;
		resolution_factor_ = rhs.resolution_factor_;
		depth_levels_ = rhs.depth_levels_;
		max_value_ = rhs.max_value_;
		nodes_half_sizes_ = rhs.nodes_half_sizes_;
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
		clear(rhs.resolution_, rhs.depth_levels_, true);

		resolution_ = std::move(rhs.resolution_);
		resolution_factor_ = std::move(rhs.resolution_factor_);
		depth_levels_ = std::move(rhs.depth_levels_);
		max_value_ = std::move(rhs.max_value_);
		root_ = std::move(rhs.root_);
		nodes_half_sizes_ = std::move(rhs.nodes_half_sizes_);
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
			deleteChildren(getRoot(), getTreeDepthLevels(), true);
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
	// Traverse
	//

	template <class UnaryFunction>
	void traverseRecurs(UnaryFunction f, MinimalNode const& node)
	{
		if (isLeaf(node)) {
			return;
		}

		for (size_t i = 0; i != 8; ++i) {
			MinimalNode child = getChild(node, i);
			if (!f(child)) {
				traverseRecurs(f, child);
			}
		}
	}

	template <class UnaryFunction>
	void traverseRecurs(UnaryFunction f, MinimalNode const& node) const
	{
		if (isLeaf(node)) {
			return;
		}

		for (size_t i = 0; i != 8; ++i) {
			MinimalNode child = getChild(node, i);
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
	void apply(MinimalNode& node, UnaryFunction f, bool propagate)
	{
		if (isLeaf(node)) {
			f(getLeafNode(node));
		} else {
			applyAllRecurs(getInnerNode(node), node.getDepth(), f);
		}

		if (!isModified(node)) {
			// Update all parents
			setParentsModified(node);

			getLeafNode(node).modified = true;
		}

		if (propagate) {
			updateModifiedNodes();
		}
	}

	template <class ExecutionPolicy, class UnaryFunction,
	          typename = std::enable_if_t<
	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>,
	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction>>>
	void apply(ExecutionPolicy policy, MinimalNode& node, UnaryFunction f, bool propagate)
	{
		if (isLeaf(node)) {
			f(getLeafNode(node));
		} else {
			applyAllRecurs(policy, getInnerNode(node), node.getDepth(), f);
		}

		if (!isModified(node)) {
			// Update all parents
			setParentsModified(node);

			getLeafNode(node).modified = true;
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
		if (code.getDepth() > getTreeDepthLevels()) {
			return;
		}

		applyRecurs(getRoot(), getTreeDepthLevels(), code, f);

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
		if (code.getDepth() > getTreeDepthLevels()) {
			return;
		}

		applyRecurs(policy, getRoot(), getTreeDepthLevels(), code, f);

		if (propagate) {
			updateModifiedNodes(policy);
		}
	}

	template <class UnaryFunction,

	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction>>>
	void applyRecurs(InnerNode& node, DepthType depth, Code code, UnaryFunction f)
	{
		if (code.getDepth() == depth) {
			if (isLeaf(node)) {
				f(static_cast<LeafNode&>(node));
			} else {
				applyAllRecurs(node, depth, f);
			}
		} else if (1 == depth) {
			createLeafChildren(node, code.getChildIdx(depth));
			LeafNode& child = getLeafChild(node, code.getChildIdx(0));
			f(child);
			child.modified = true;
		} else {
			createInnerChildren(node, depth, code.getChildIdx(depth));
			applyRecurs(getInnerChild(node, code.getChildIdx(depth - 1)), depth - 1, code, f);
		}

		node.modified = true;
	}

	template <class ExecutionPolicy, class UnaryFunction,
	          typename = std::enable_if_t<
	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>,
	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction>>>
	void applyRecurs(ExecutionPolicy policy, InnerNode& node, DepthType depth, Code code,
	                 UnaryFunction f)
	{
		if (code.getDepth() == depth) {
			if (isLeaf(node)) {
				f(static_cast<LeafNode&>(node));
			} else {
				applyAllRecurs(policy, node, depth, f);
			}
		} else if (1 == depth) {
			createLeafChildren(node, code.getChildIdx(depth));
			LeafNode& child = getLeafChild(node, code.getChildIdx(0));
			f(child);
			child.modified = true;
		} else {
			createInnerChildren(node, depth, code.getChildIdx(depth));
			applyRecurs(policy, getInnerChild(node, code.getChildIdx(depth - 1)), depth - 1,
			            code, f);
		}

		node.modified = true;
	}

	template <class UnaryFunction,
	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction>>>
	void applyAllRecurs(InnerNode& node, DepthType depth, UnaryFunction f)
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
	void applyAllRecurs(ExecutionPolicy policy, InnerNode& node, DepthType depth,
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
	// Move line inside octree bounds helper
	//

	static bool getIntersection(float d_1, float d_2, Point3 p_1, Point3 p_2,
	                            Point3* hit) noexcept
	{
		if (0 <= (d_1 * d_2)) {
			return false;
		}
		*hit = p_1 + (p_2 - p_1) * (-d_1 / (d_2 - d_1));
		return true;
	}

	static bool inBBX(Point3 point, int axis, Point3 bbx_min, Point3 bbx_max) noexcept
	{
		if (0 == axis && point[2] > bbx_min[2] && point[2] < bbx_max[2] &&
		    point[1] > bbx_min[1] && point[1] < bbx_max[1]) {
			return true;
		}
		if (1 == axis && point[2] > bbx_min[2] && point[2] < bbx_max[2] &&
		    point[0] > bbx_min[0] && point[0] < bbx_max[0]) {
			return true;
		}
		if (2 == axis && point[0] > bbx_min[0] && point[0] < bbx_max[0] &&
		    point[1] > bbx_min[1] && point[1] < bbx_max[1]) {
			return true;
		}
		return false;
	}

	//
	// Get root
	//

	constexpr InnerNode const& getRoot() const noexcept { return *root_; }

	constexpr InnerNode& getRoot() noexcept { return *root_; }

	//
	// Get node
	//

	static constexpr LeafNode const& getLeafNode(MinimalNode const& node)
	{
		return *static_cast<LeafNode const*>(node.getData());
	}

	static constexpr LeafNode& getLeafNode(MinimalNode& node)
	{
		return *static_cast<LeafNode*>(node.getData());
	}

	LeafNode const& getLeafNode(Code code) const
	{
		return getLeafNodeRecurs(getRoot(), getTreeDepthLevels(), code);
	}

	LeafNode& getLeafNode(Code code)
	{
		return const_cast<LeafNode&>(std::as_const(*this).getLeafNode(code));
	}

	LeafNode const& getLeafNodeRecurs(InnerNode const& node, DepthType depth,
	                                  Code code) const
	{
		if (code.getDepth() == depth || isLeaf(node)) {
			return node;
		} else if (1 == depth) {
			return getLeafChild(node, code.getChildIdx(0));
		} else {
			return getLeafNodeRecurs(getInnerChild(node, code.getChildIdx(depth - 1)),
			                         depth - 1, code);
		}
	}

	static constexpr InnerNode const& getInnerNode(MinimalNode const& node)
	{
		return static_cast<InnerNode const&>(getLeafNode(node));
	}

	static constexpr InnerNode& getInnerNode(MinimalNode& node)
	{
		return static_cast<InnerNode&>(getLeafNode(node));
	}

	InnerNode const& getInnerNode(Code code) const
	{
		return static_cast<InnerNode const&>(getLeafNode(code));
	}

	InnerNode& getInnerNode(Code code)
	{
		return const_cast<InnerNode&>(std::as_const(*this).getInnerNode(code));
	}

	//
	// Delete node
	//

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void deleteNodeChildrenRecurs(ExecutionPolicy policy, InnerNode& node, DepthType depth,
	                              Code code)
	{
		// FIXME: What happens when no children?
		if (isLeaf(node)) {
			return;
		}

		if (code.getDepth() == depth) {
			deleteChildren(policy, node, depth);
		} else if (1 == depth) {
			deleteLeafChildren(node);
		} else {
			deleteNodeChildrenRecurs(policy, getInnerChild(node, code.getChildIdx(depth - 1)),
			                         depth - 1, code);
		}

		node.modified = true;
	}

	//
	// (Un)lock children
	//

	bool tryLockChildren(DepthType depth, size_t index)
	{
		return !children_locks_[depth][index].test_and_set(std::memory_order_acquire);
	}

	void lockChildren(DepthType depth, size_t index)
	{
		while (!tryLockChildren(depth, index))
			;
	}

	bool lockIfLeaf(InnerNode const& node, DepthType depth, size_t index)
	{
		do {
			if (hasChildren(node)) {
				return false;
			}
		} while (!tryLockChildren(depth, index));

		if (hasChildren(node)) {
			unlockChildren(depth, index);
			return false;
		}

		return true;
	}

	void unlockChildren(DepthType depth, size_t index)
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

		// if (hasChildren(node)) {
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

	bool createInnerChildren(InnerNode& node, DepthType depth, size_t index)
	{
		if (!lockIfLeaf(node, depth, index)) {
			return false;
		}

		// if (hasChildren(node)) {
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

	bool createChildren(InnerNode& node, DepthType depth, size_t index)
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

		if (nullptr == node.children || (!manual_pruning && !isAutomaticPruningEnabled())) {
			return;
		}

		delete &getLeafChildren(node);
		num_allocated_leaf_nodes_ -= 8;
		num_allocated_inner_leaf_nodes_ += 1;
		num_allocated_inner_nodes_ -= 1;
		node.children = nullptr;
	}

	void deleteChildren(InnerNode& node, DepthType depth, bool manual_pruning = false)
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

		if (nullptr == node.children || (!manual_pruning && !isAutomaticPruningEnabled())) {
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
	void deleteChildren(ExecutionPolicy policy, InnerNode& node, DepthType depth,
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

		if (nullptr == node.children || (!manual_pruning && !isAutomaticPruningEnabled())) {
			return;
		}

		// Manual pruning is true in case automatic_pruning_enabled_ changes between
		// calls
		if (getParallelExecutionDepth() == depth) {
			std::for_each(policy, std::begin(getInnerChildren(node)),
			              std::end(getInnerChildren(node)), [this, depth](InnerNode& child) {
				              deleteChildren(std::execution::seq, child, depth - 1, true);
			              });
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

	static constexpr LeafNode& getChild(InnerNode const& node, DepthType const child_depth,
	                                    unsigned int const idx)
	{
		return 0 == child_depth ? getLeafChild(node, idx) : getInnerChild(node, idx);
	}

	//
	// Get child center
	//

	static Point3 getChildCenter(Point3 parent_center, float child_half_size,
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

	static Point3 getSiblingCenter(Point3 sibling_center, float half_size,
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
	// Has children
	//

	static constexpr bool hasChildren(LeafNode const& node) noexcept
	{
		return !isLeaf(node);
	}

	//
	// Is node collapsible
	//

	// If all children are the same as the parent they can be pruned
	// NOTE: Only call with nodes that have children
	static bool isNodeCollapsible(InnerNode const& node, DepthType depth) noexcept
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
	void updateModifiedNodesRecurs(DepthType max_depth, InnerNode& node, DepthType depth)
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
				if (child.modified) {
					updateNodeIndicators(child);
					if constexpr (!KeepModified) {
						child.modified = false;
					}
				}
			}
		} else {
			for (auto& child : getInnerChildren(node)) {
				if (child.modified) {
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
	void updateModifiedNodesRecurs(ExecutionPolicy policy, DepthType max_depth,
	                               InnerNode& node, DepthType depth)
	{
		if (!node.modified) {
			return;
		}

		if (hasChildren(node)) {
			if (1 == depth) {
				for (auto& child : getLeafChildren(node)) {
					updateNodeIndicators(child);
					if constexpr (!KeepModified) {
						child.modified = false;
					}
				}
			} else {
				if (getParallelExecutionDepth() == depth) {
					std::for_each(policy, std::begin(getInnerChildren(node)),
					              std::end(getInnerChildren(node)),
					              [this, max_depth, depth](auto& child) {
						              // Sequential execution for rest in case parallel depth changes
						              // in between
						              updateModifiedNodesRecurs<KeepModified>(
						                  std::execution::seq, max_depth, child, depth - 1);
					              });
				} else {
					for (auto& child : getInnerChildren(node)) {
						updateModifiedNodesRecurs<KeepModified>(policy, max_depth, child, depth - 1);
					}
				}
			}
		}

		if (depth <= max_depth) {
			if (hasChildren(node)) {
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

	virtual void updateNode(InnerNode& node, DepthType depth) = 0;

	virtual void updateNodeIndicators(LeafNode& node) = 0;

	virtual void updateNodeIndicators(InnerNode& node, DepthType depth) = 0;

	//
	// Set parents modified
	//

	void setParentsModified(MinimalNode& node)
	{
		if (node.getDepth() >= getTreeDepthLevels()) {
			return;
		}
		setParentsModifiedRecurs(getRoot(), getTreeDepthLevels(), node.getCode());
	}

	// NOTE: Assumes code has depth higher then depth
	void setParentsModifiedRecurs(InnerNode& node, DepthType depth, Code code)
	{
		node.modified = true;
		if (code.getDepth() < depth - 1) {
			setParentsModifiedRecurs(getInnerChild(node, code.getChildIdx(depth - 1)),
			                         depth - 1, code);
		}
	}

	//
	// Set modified
	//

	void setModifiedRecurs(InnerNode& node, DepthType depth, DepthType min_depth)
	{
		if (depth < min_depth) {
			return;
		}

		node.modified = true;

		if (depth == min_depth) {
			return;
		}

		if (isLeaf(node)) {
			return;
		}

		if (1 == depth) {
			for (LeafNode& child : getLeafChildren(node)) {
				child.modified = true;
			}
		} else {
			for (InnerNode& child : getInnerChildren(node)) {
				setModifiedRecurs(child, depth - 1, min_depth);
			}
		}
	}

	//
	// Clear Children modified
	//

	void clearModifiedRecurs(InnerNode& node, DepthType depth, DepthType max_depth)
	{
		if (!node.modified) {
			// Already clear
			return;
		}

		if (depth <= max_depth) {
			node.modified = false;
		}

		if (isLeaf(node)) {
			return;
		}

		if (1 == depth) {
			for (LeafNode& child : getLeafChildren(node)) {
				child.modified = false;
			}
		} else {
			for (InnerNode& child : getInnerChildren(node)) {
				clearModifiedRecurs(child, depth - 1, max_depth);
			}
		}
	}

	//
	// Prune
	//

	// NOTE: Only call with nodes that have children
	bool pruneNode(InnerNode& node, DepthType depth)
	{
		if (isNodeCollapsible(node, depth)) {
			deleteChildren(node, depth);
			return true;
		} else {
			return false;
		}
	}

	bool pruneRecurs(InnerNode& node, DepthType depth)
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
	bool pruneRecurs(ExecutionPolicy policy, InnerNode& node, DepthType depth)
	{
		if (isLeaf(node)) {
			return true;
		}

		if (1 == depth) {
			return pruneNode(node, depth);
		}

		bool prunable;
		if (getParallelExecutionDepth() == depth) {
			prunable =
			    std::all_of(policy, std::begin(getInnerChildren(node)),
			                std::end(getInnerChildren(node)), [this, depth](InnerNode& child) {
				                return pruneRecurs(std::execution::seq, child, depth - 1);
			                });
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
		info["resolution"].push_back(std::to_string(getResolution()));
		info["depth_levels"].push_back(
		    std::to_string(static_cast<uint32_t>(getTreeDepthLevels())));
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
			nodes[0] = static_cast<LeafNode*>(&getRoot());
			getRoot().modified = true;
		} else if (valid_inner) {
			size_t idx = 2;
			auto nodes_it = nodes.begin();
			getNodesRecurs(indicators, idx, nodes_it, getRootMinimalNode());
		}

		return nodes;
	}

	void getNodesRecurs(std::vector<uint8_t> const& indicators, size_t& idx,
	                    typename std::vector<LeafNode*>::iterator& nodes_it,
	                    MinimalNode node)
	{
		uint8_t child_valid_return = indicators[idx++];
		uint8_t child_valid_inner = indicators[idx++];

		if (0 == child_valid_return && 0 == child_valid_inner) {
			return;
		}

		InnerNode& inner_node = getInnerNode(node);
		inner_node.modified = true;

		if (1 == node.getDepth()) {
			createLeafChildren(inner_node, node.getCode().getChildIdx(1));

			auto child = getChild(node, 0);
			for (size_t i = 0; i != 8; ++i) {
				if ((child_valid_return >> i) & 1U) {
					child = getSibling(child, i);
					*nodes_it++ = &getLeafNode(child);
				}
			}
		} else {
			createInnerChildren(inner_node, node.getDepth(),
			                    node.getCode().getChildIdx(node.getDepth()));

			auto child = getChild(node, 0);
			for (size_t i = 0; i != 8; ++i) {
				if ((child_valid_return >> i) & 1U) {
					child = getSibling(child, i);
					*nodes_it++ = &getLeafNode(child);
				} else if ((child_valid_inner >> i) & 1U) {
					child = getSibling(child, i);
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
	void writeData(std::ostream& out_stream, Predicates const& predicates,
	               DepthType min_depth, bool compress, int compression_acceleration_level,
	               int compression_level) const
	{
		uint8_t compressed = compress ? UINT8_MAX : 0U;
		out_stream.write(reinterpret_cast<char*>(&compressed), sizeof(compressed));

		auto size_pos = out_stream.tellp();
		uint64_t uncompressed_data_size = 0;
		out_stream.write(reinterpret_cast<char*>(&uncompressed_data_size),
		                 sizeof(uncompressed_data_size));

		auto data_pos = out_stream.tellp();

		auto [indicators, nodes] = getData(predicate::Leaf(min_depth) && predicates);

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
	std::pair<std::vector<uint8_t>, std::vector<LeafNode const*>> getData(
	    Predicates const& predicates) const
	{
		std::vector<uint8_t> indicators;
		std::vector<LeafNode const*> nodes;

		auto root = getRootNode();

		Derived const* derived = dynamic_cast<Derived const*>(this);

		bool valid_return =
		    predicate::PredicateValueCheck<Predicates>::apply(predicates, *derived, root);
		bool valid_inner = !valid_return && predicate::PredicateInnerCheck<Predicates>::apply(
		                                        predicates, *derived, root);

		indicators.push_back(valid_return ? UINT8_MAX : 0U);
		indicators.push_back(valid_inner ? UINT8_MAX : 0U);

		if (valid_return) {
			nodes.push_back(&getLeafNode(root));
		} else if (valid_inner) {
			getDataRecurs(indicators, nodes, predicates, derived, root);
			if (nodes.empty()) {
				//  Nothing was added
				indicators.clear();
			}
		}

		return {indicators, nodes};
	}

	template <class Predicates>
	void getDataRecurs(std::vector<uint8_t>& indicators,
	                   std::vector<LeafNode const*>& nodes, Predicates const& predicates,
	                   Derived const* derived, Node const& node) const
	{
		auto cur_indicators_size = indicators.size();
		auto cur_nodes_size = nodes.size();

		auto child = getChild(node, 0);

		uint8_t child_valid_return = 0;
		uint8_t child_valid_inner = 0;
		for (size_t i = 0; 8 != i; ++i) {
			child = getSibling(child, i);

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
				child = getSibling(child, i);
				nodes.push_back(&getLeafNode(child));
			} else if ((child_valid_inner >> i) & 1U) {
				child = getSibling(child, i);
				getDataRecurs(indicators, nodes, predicates, derived, child);
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
	float resolution_;         // The voxel size of the leaf nodes
	float resolution_factor_;  // Reciprocal of the resolution
	DepthType depth_levels_;   // The maximum depth of the octree
	KeyType max_value_;        // The maximum coordinate value the octree can store

	std::unique_ptr<InnerNode> root_;  // The root of the octree

	// std::array<Blocks<InnerNode>, getMaxTreeDepthLevels() - 1> inner_nodes_;
	// Blocks<LeafNode> leaf_nodes_;

	// Stores the half size of a node at a given depth, where the depth is the index
	std::array<float, getMaxTreeDepthLevels() + 1> nodes_half_sizes_;

	// Automatic pruning
	bool automatic_pruning_enabled_ = true;

	// Depth at which parallel execution happens
	DepthType parallel_depth_ = 12;

	// Locks to support parallel insertion, one per level and child
	std::array<std::array<std::atomic_flag, 8>, getMaxTreeDepthLevels() + 1>
	    children_locks_;  // TODO: Initialize to ATOMIC_FLAG_INIT

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