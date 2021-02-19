/**
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * License: BSD 3
 *
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

#ifndef UFO_MAP_ITERATOR_OCTREE_H
#define UFO_MAP_ITERATOR_OCTREE_H

#include <ufo/geometry/bounding_volume.h>
#include <ufo/geometry/collision_checks.h>
#include <ufo/map/code.h>
#include <ufo/map/types.h>

#include <array>
#include <type_traits>

namespace ufo::map
{
template <typename TREE, typename DATA_TYPE, typename INNER_NODE, typename LEAF_NODE,
          bool ONLY_LEAF>
class OctreeIterator
{
 protected:
	struct IteratorNode {
		// Pointer to the actual node
		LEAF_NODE const* node;
		// The index of the node from its parent
		unsigned int index;
		// The AABB for the node
		ufo::geometry::AABB aabb;
		// Indicates if this node is completely inside the bounding volume.
		// Meaning its children does not have to do any intersection checks.
		bool inside;
	};

 public:
	OctreeIterator() {}

	OctreeIterator(TREE const* tree, ufo::geometry::BoundingVolume const& bounding_volume,
	               DepthType min_depth = 0)
	    : tree_(tree), bounding_volume_(bounding_volume), min_depth_(min_depth)
	{
	}

	OctreeIterator(TREE const* tree, INNER_NODE const& root,
	               ufo::geometry::BoundingVolume const& bounding_volume,
	               unsigned int min_depth = 0)
	    : OctreeIterator(tree, bounding_volume, min_depth)
	{
		init(root);
	}

	OctreeIterator(OctreeIterator const& other)
	    : tree_(other.tree_),
	      bounding_volume_(other.bounding_volume_),
	      path_(other.path_),
	      current_depth_(other.current_depth_),
	      min_depth_(other.min_depth_),
	      max_depth_(other.max_depth_)
	{
	}

	OctreeIterator& operator=(OctreeIterator const& rhs)
	{
		tree_ = rhs.tree_;
		bounding_volume_ = rhs.bounding_volume_;
		path_ = rhs.path_;
		current_depth_ = rhs.current_depth_;
		min_depth_ = rhs.min_depth_;
		max_depth_ = rhs.max_depth_;
		return *this;
	}

	bool operator==(OctreeIterator const& rhs) const
	{
		// TODO: Can be improved
		return rhs.tree_ == tree_;  // && rhs.path_ == path_;
	}

	bool operator!=(OctreeIterator const& rhs) const { return !(*this == rhs); }

	// Postfix increment
	OctreeIterator operator++(int)
	{
		OctreeIterator result = *this;
		++(*this);
		return result;
	}

	// Prefix increment
	OctreeIterator& operator++()
	{
		increment();
		return *this;
	}

	DATA_TYPE const* operator->() const { return &(path_[current_depth_].node->value); }

	DATA_TYPE const& operator*() const { return path_[current_depth_].node->value; }

	double getSize() const { return tree_->getNodeSize(getDepth()); }

	double getHalfSize() const { return tree_->getNodeHalfSize(getDepth()); }

	ufo::geometry::AABB getBoundingVolume() const { return path_[current_depth_].aabb; }

	// bool completelyInsideBoundingVolume() const { return path_[current_depth_].inside; }

	DepthType getDepth() const { return current_depth_; }

	Point3 getCenter() const { return path_[current_depth_].aabb.center; }

	Code getCode(DepthType depth = 0) const
	{
		// TODO: Improve
		Code code = tree_->getRootCode();
		for (int d = code.getDepth() - 1;
		     static_cast<int>(getDepth()) <= d && static_cast<int>(depth) <= d; --d) {
			code = code.getChild(path_[d].index);
		}
		return code;
	}

	double getX() const { return path_[current_depth_].aabb.center[0]; }

	double getY() const { return path_[current_depth_].aabb.center[1]; }

	double getZ() const { return path_[current_depth_].aabb.center[2]; }

	bool isPureLeaf() const { return 0 == getDepth(); }

	bool isLeaf() const { return tree_->isLeaf(path_[getDepth()].node, getDepth()); }

	bool hasChildren() const { return !isLeaf(); }

 public:
	// iterator traits
	using difference_type = std::ptrdiff_t;  // What should this be?
	using value_type = DATA_TYPE;
	using pointer = DATA_TYPE const*;
	using reference = DATA_TYPE const&;
	using iterator_category = std::forward_iterator_tag;

 protected:
	void init(INNER_NODE const& root)
	{
		current_depth_ = tree_->getTreeDepthLevels();
		max_depth_ = current_depth_;

		IteratorNode node;
		node.node = &root;
		node.index = 7;  // Root has no siblings
		node.aabb.center = Point3(0, 0, 0);
		double s = tree_->getNodeHalfSize(current_depth_);
		node.aabb.half_size = Point3(s, s, s);
		node.inside = bounding_volume_.empty();

		// Prefill AABBs
		for (size_t depth = min_depth_; depth < current_depth_; ++depth) {
			s = tree_->getNodeHalfSize(depth);
			path_[depth].aabb.half_size = Point3(s, s, s);
		}

		if (validNode(node, current_depth_) && current_depth_ >= min_depth_) {
			path_[current_depth_] = node;
			if (!validReturnNode()) {
				operator++();
			}
		} else {
			// Same as end iterator
			tree_ = nullptr;
		}
	}

	bool hasMore() const { return getDepth() <= max_depth_; }

	virtual bool validNode(IteratorNode& node, DepthType) const
	{
		if (node.inside) {
			return true;
		}

		// Bounding volume can only contain one otherwise we cannot make sure the node is
		// completely inside and we therefore only do the simple intersection test instead
		// if (0 < depth && 1 == bounding_volume_.size()) {
		// 	// Check if the node is completely inside the bounding volume
		// 	double hs = node.aabb.half_size[0];
		// 	Point3 const& c = node.aabb.center;
		// 	std::array<Point3, 8> corners = {Point3(-hs, -hs, -hs), Point3(-hs, -hs, hs),
		// 	                                 Point3(-hs, hs, -hs),  Point3(-hs, hs, hs),
		// 	                                 Point3(hs, -hs, -hs),  Point3(hs, -hs, hs),
		// 	                                 Point3(hs, hs, -hs),   Point3(hs, hs, hs)};
		// 	bool one_inside = false;
		// 	node.inside = true;
		// 	for (Point3 const& offset : corners) {
		// 		if (!bounding_volume_.intersects(c + offset)) {
		// 			// The node is not completely inside the bounding volume
		// 			node.inside = false;
		// 			break;
		// 		}
		// 		one_inside = true;
		// 	}

		// 	if (one_inside) {
		// 		return true;
		// 	}
		// }
		return bounding_volume_.intersects(node.aabb);
	}

	virtual bool validReturnNode() const
	{
		if constexpr (ONLY_LEAF) {
			return min_depth_ == getDepth() || isLeaf();
		} else {
			return true;
		}
	}

	void increment()
	{
		if (!hasMore()) {
			tree_ = nullptr;
		} else {
			// Skip forward to next valid return node
			do {
				singleIncrement();
			} while (hasMore() && !validReturnNode());

			if (!hasMore()) {
				tree_ = nullptr;
			}
		}
	}

	virtual void singleIncrement()
	{
		if (getDepth() <= min_depth_ || isLeaf()) {
			// Move to next leaf if possible
			++path_[current_depth_].index;
		} else {
			// Current node has children
			--current_depth_;
			path_[current_depth_].index = 0;
		}

		if (!getNextNode()) {
			// Move up in the tree until valid node is found
			for (++current_depth_; current_depth_ < max_depth_; ++current_depth_) {
				++path_[current_depth_].index;
				if (getNextNode()) {
					return;
				}
			}

			if (current_depth_ == max_depth_) {
				++current_depth_;
			}
		}
	}

	bool getNextNode()
	{
		DepthType depth = current_depth_;
		DepthType parent_depth = depth + 1;
		Point3 const& p_center = path_[parent_depth].aabb.center;
		double hs = path_[depth].aabb.half_size[0];
		for (; 8 > path_[depth].index; ++path_[depth].index) {
			INNER_NODE const& inner_node =
			    static_cast<INNER_NODE const&>(*path_[parent_depth].node);
			path_[depth].node = &tree_->getChild(inner_node, depth, path_[depth].index);
			path_[depth].aabb.center = tree_->getChildCenter(p_center, hs, path_[depth].index);
			path_[depth].inside = path_[parent_depth].inside;

			if (validNode(path_[depth], depth)) {
				return true;
			}
		}
		return false;
	}

 protected:
	TREE const* tree_ = nullptr;
	ufo::geometry::BoundingVolume bounding_volume_;
	std::array<IteratorNode, TREE::getMaxDepthLevels()> path_;
	DepthType current_depth_;
	DepthType min_depth_;
	DepthType max_depth_;
};
}  // namespace ufo::map

#endif  // UFO_MAP_ITERATOR_OCTREE_H