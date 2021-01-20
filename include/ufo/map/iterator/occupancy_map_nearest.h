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

#ifndef UFO_MAP_ITERATOR_NEAREST_OCCUPANCY_MAP_H
#define UFO_MAP_ITERATOR_NEAREST_OCCUPANCY_MAP_H

#include <ufo/map/iterator/octree_nearest.h>

namespace ufo::map
{
template <typename TREE, typename DATA_TYPE, typename INNER_NODE, typename LEAF_NODE,
          bool ONLY_LEAF>
class OccupancyMapNearestIterator
    : public OctreeNearestIterator<TREE, DATA_TYPE, INNER_NODE, LEAF_NODE, ONLY_LEAF>
{
 private:
	using Base = OctreeNearestIterator<TREE, DATA_TYPE, INNER_NODE, LEAF_NODE, ONLY_LEAF>;
	using IteratorNode = typename Base::IteratorNode;

 public:
	OccupancyMapNearestIterator() {}

	OccupancyMapNearestIterator(TREE const* tree, INNER_NODE const& root,
	                            ufo::geometry::BoundingVolume const& bounding_volume,
	                            Point3 const& coordinate, bool occupied_space = true,
	                            bool free_space = true, bool unknown_space = false,
	                            bool contains = false, DepthType min_depth = 0)
	    : Base(tree, bounding_volume, coordinate, min_depth),
	      occupied_space_(occupied_space),
	      free_space_(free_space),
	      unknown_space_(unknown_space),
	      contains_(contains)
	{
		Base::init(root);
	}

	OccupancyMapNearestIterator(OccupancyMapNearestIterator const& other)
	    : Base(other),
	      occupied_space_(other.occupied_space_),
	      free_space_(other.free_space_),
	      unknown_space_(other.unknown_space_),
	      contains_(other.contains_)
	{
	}

	OccupancyMapNearestIterator& operator=(OccupancyMapNearestIterator const& rhs)
	{
		Base::operator=(rhs);
		occupied_space_ = rhs.occupied_space_;
		free_space_ = rhs.free_space_;
		unknown_space_ = rhs.unknown_space_;
		contains_ = rhs.contains_;
		return *this;
	}

	bool operator==(OccupancyMapNearestIterator const& rhs) const
	{
		return Base::operator==(rhs);
	}

	bool operator!=(OccupancyMapNearestIterator const& rhs) const
	{
		return Base::operator!=(rhs);
	}

	// Postfix increment
	OccupancyMapNearestIterator operator++(int)
	{
		OccupancyMapNearestIterator result = *this;
		++(*this);
		return result;
	}

	// Prefix increment
	OccupancyMapNearestIterator& operator++()
	{
		Base::increment();
		return *this;
	}

 public:
	bool isOccupied() const { return isOccupied(Base::container_.top()); }

	bool isFree() const { return isFree(Base::container_.top()); }

	bool isUnknown() const { return isUnknown(Base::container_.top()); }

	bool containsOccupied() const
	{
		return containsOccupied(Base::container_.top(), Base::getDepth());
	}

	bool containsFree() const
	{
		return containsFree(Base::container_.top(), Base::getDepth());
	}

	bool containsUnknown() const
	{
		return containsUnknown(Base::container_.top(), Base::getDepth());
	}

	double getOccupancy() const
	{
		return Base::tree_->getOccupancy(*Base::container_.top().node);
	}

 protected:
	bool isOccupied(IteratorNode const& node) const
	{
		return Base::tree_->isOccupied(*node.node);
	}

	bool isFree(IteratorNode const& node) const { return Base::tree_->isFree(*node.node); }

	bool isUnknown(IteratorNode const& node) const
	{
		return Base::tree_->isUnknown(*node.node);
	}

	bool containsOccupied(IteratorNode const& node) const
	{
		return Base::tree_->containsOccupied(*node.node, node.depth);
	}

	bool containsFree(IteratorNode const& node) const
	{
		return Base::tree_->containsFree(*node.node, node.depth);
	}

	bool containsUnknown(IteratorNode const& node) const
	{
		return Base::tree_->containsUnknown(*node.node, node.depth);
	}

	virtual bool validNode(IteratorNode const& node,
	                       ufo::geometry::AABB const& node_aabb) const override
	{
		if (!Base::validNode(node, node_aabb)) {
			return false;
		}

		if (contains_ || Base::min_depth_ != node.depth) {
			return (occupied_space_ && containsOccupied(node)) ||
			       (unknown_space_ && containsUnknown(node)) ||
			       (free_space_ && containsFree(node));
		}

		return (occupied_space_ && isOccupied(node)) || (unknown_space_ && isUnknown(node)) ||
		       (free_space_ && isFree(node));
	}

	virtual bool validReturnNode() const override
	{
		if (!Base::validReturnNode()) {
			return false;
		}

		if constexpr (!ONLY_LEAF) {
			if (contains_) {
				return (occupied_space_ && containsOccupied()) ||
				       (unknown_space_ && containsUnknown()) || (free_space_ && containsFree());
			}
		}

		return (occupied_space_ && isOccupied()) || (unknown_space_ && isUnknown()) ||
		       (free_space_ && isFree());
	}

 protected:
	bool occupied_space_;
	bool free_space_;
	bool unknown_space_;

	bool contains_;
};
}  // namespace ufo::map

#endif  // UFO_MAP_ITERATOR_NEAREST_OCCUPANCY_MAP_H