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

#ifndef UFO_MAP_ITERATOR_NEAREST_OCTREE_H
#define UFO_MAP_ITERATOR_NEAREST_OCTREE_H

#include <ufo/geometry/bounding_volume.h>
#include <ufo/geometry/collision_checks.h>
#include <ufo/map/types.h>

#include <queue>
#include <type_traits>
#include <vector>

namespace ufo::map
{
template <typename TREE, typename DATA_TYPE, typename INNER_NODE, typename LEAF_NODE,
          bool ONLY_LEAF>
class OctreeNearestIterator
{
 protected:
	struct IteratorNode {
		// Pointer to the actual node
		LEAF_NODE const* node;
		// Squared distance to coordinate
		double squared_distance = 0;
		// The center of the node
		Point3 center;
		// The depth of the node
		DepthType depth;
	};

 public:
	OctreeNearestIterator() {}

	OctreeNearestIterator(TREE const* tree,
	                      ufo::geometry::BoundingVolume const& bounding_volume,
	                      Point3 const& coordinate, DepthType min_depth = 0)
	    : tree_(tree),
	      bounding_volume_(bounding_volume),
	      coordinate_(coordinate),
	      min_depth_(min_depth)
	{
	}

	OctreeNearestIterator(TREE const* tree, INNER_NODE const& root,
	                      ufo::geometry::BoundingVolume const& bounding_volume,
	                      Point3 const& coordinate, DepthType min_depth = 0)
	    : OctreeNearestIterator(tree, bounding_volume, coordinate, min_depth)
	{
		init(root);
	}

	OctreeNearestIterator(OctreeNearestIterator const& other)
	    : tree_(other.tree_),
	      bounding_volume_(other.bounding_volume_),
	      coordinate_(other.coordinate_),
	      container_(other.container_),
	      min_depth_(other.min_depth_)
	{
	}

	OctreeNearestIterator& operator=(OctreeNearestIterator const& rhs)
	{
		tree_ = rhs.tree_;
		bounding_volume_ = rhs.bounding_volume_;
		coordinate_ = rhs.coordinate_;
		container_ = rhs.container_;
		min_depth_ = rhs.min_depth_;
		return *this;
	}

	bool operator==(OctreeNearestIterator const& rhs) const
	{
		return rhs.tree_ == tree_;
		// return (rhs.tree_ == tree_ && rhs.container_.size() == container_.size() &&
		//         (container_.empty() || (rhs.container_.top() == container_.top())));
	}

	bool operator!=(OctreeNearestIterator const& rhs) const { return !(*this == rhs); }

	// Postfix increment
	OctreeNearestIterator operator++(int)
	{
		OctreeNearestIterator result = *this;
		++(*this);
		return result;
	}

	// Prefix increment
	OctreeNearestIterator& operator++()
	{
		increment();
		return *this;
	}

	DATA_TYPE const* operator->() const { return &(container_.top().node->value); }

	DATA_TYPE const& operator*() const { return container_.top().node->value; }

	double getDistance() const { return std::sqrt(getSquaredDistance()); }

	double getSquaredDistance() const { return container_.top().squared_distance; }

	double getSize() const { return tree_->getNodeSize(getDepth()); }

	double getHalfSize() const { return tree_->getNodeHalfSize(getDepth()); }

	ufo::geometry::AABB getAABB() const
	{
		return ufo::geometry::AABB(getCenter(), getHalfSize());
	}

	DepthType getDepth() const { return container_.top().depth; }

	Point3 const& getCenter() const { return container_.top().center; }

	double getX() const { return getCenter().x(); }

	double getY() const { return getCenter().y(); }

	double getZ() const { return getCenter().z(); }

	bool isPureLeaf() const { return 0 == getDepth(); }

	bool isLeaf() const { return tree_->isLeaf(container_.top().node, getDepth()); }

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
		IteratorNode node;
		node.node = &root;
		node.depth = tree_->getTreeDepthLevels();
		node.center = Point3(0, 0, 0);

		ufo::geometry::AABB node_aabb(node.center, tree_->getNodeHalfSize(node.depth));
		if (validNode(node, node_aabb) && node.depth >= min_depth_) {
			node.squared_distance = squaredDistance(node_aabb);
			container_.push(node);
			if (!validReturnNode()) {
				operator++();
			}
		} else {
			// Same as end iterator
			tree_ = nullptr;
		}
	}

	// Nearest
	double squaredDistance(ufo::geometry::AABB const& aabb) const
	{
		Point3 closest_point = Point3::clamp(coordinate_, aabb.getMin(), aabb.getMax());
		return (coordinate_ - closest_point).squaredNorm();
	}

	virtual bool validNode(IteratorNode const&, ufo::geometry::AABB const& node_aabb) const
	{
		return bounding_volume_.empty() || bounding_volume_.intersects(node_aabb);
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
		if (container_.empty()) {
			tree_ = nullptr;
		} else {
			// Skip forward to next valid node
			do {
				singleIncrement();
			} while (!container_.empty() && !validReturnNode());

			if (container_.empty()) {
				tree_ = nullptr;
			}
		}
	}

	void singleIncrement()
	{
		IteratorNode top = container_.top();
		container_.pop();

		if (getDepth() <= min_depth_ || isLeaf()) {
			return;
		}

		// We know it is a inner node
		INNER_NODE const& inner_node = static_cast<INNER_NODE const&>(*top.node);

		IteratorNode node;
		node.depth = top.depth - 1;
		double child_half_size = tree_->getNodeHalfSize(node.depth);
		for (size_t idx = 0; 8 > idx; ++idx) {
			node.node = &tree_->getChild(inner_node, node.depth, idx);
			node.center = tree_->getChildCenter(top.center, child_half_size, idx);

			ufo::geometry::AABB node_aabb(node.center, child_half_size);
			if (validNode(node, node_aabb)) {
				node.squared_distance = squaredDistance(node_aabb);
				container_.push(node);
			}
		}
	}

 protected:
	TREE const* tree_ = nullptr;
	ufo::geometry::BoundingVolume bounding_volume_;
	Point3 coordinate_;

	struct Compare {
		bool operator()(IteratorNode const& left, IteratorNode const& right)
		{
			return left.squared_distance > right.squared_distance;
		}
	};

	std::priority_queue<IteratorNode, std::vector<IteratorNode>, Compare> container_;
	DepthType min_depth_;
};
}  // namespace ufo::map

#endif  // UFO_MAP_ITERATOR_NEAREST_OCTREE_H