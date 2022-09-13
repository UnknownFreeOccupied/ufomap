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

#ifndef UFO_MAP_OCTREE_OCTREE_NODE_H
#define UFO_MAP_OCTREE_OCTREE_NODE_H

// UFO
#include <ufo/map/types.h>

// STL
#include <array>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace ufo::map
{
template <class Data>
struct OctreeLeafNode : Data {
	// Indicates whether this node has to be updated (get information from children and/or
	// update indicators). Useful when propagating information up the tree
	index_field_t modified;

	//
	// Fill
	//

	void fill(OctreeLeafNode const& other, index_t index)
	{
		if (index_field_t(1) & (other.modified >> index)) {
			setModified();
		} else {
			resetModified();
		}

		Data::fill(other, index);
	}

	//
	// Is collapsible
	//

	[[nodiscard]] constexpr bool isCollapsible(OctreeLeafNode const& parent,
	                                           index_t index) const
	{
		return Data::isCollapsible(parent, index);
	}

	//
	// Is modified
	//

	constexpr index_field_t isModified() const noexcept { return modified; }

	constexpr index_field_t isModified(index_field_t indices) const noexcept
	{
		return modified & indices;
	}

	constexpr bool isModifiedIndex(index_t index) const noexcept
	{
		return (modified >> index) & index_field_t(1);
	}

	constexpr bool isAllModified() const noexcept
	{
		return std::numeric_limits<index_field_t>::max() == modified;
	}

	constexpr bool isAnyModified() const noexcept { return 0 != modified; }

	constexpr bool isNoneModified() const noexcept { return 0 == modified; }

	//
	// Set modified
	//

	constexpr void setModified() noexcept
	{
		modified = std::numeric_limits<index_field_t>::max();
	}

	constexpr void setModified(index_field_t index_field) noexcept
	{
		modified |= index_field;
	}

	constexpr void setModifiedIndex(index_t index) noexcept
	{
		modified |= index_field_t(1) << index;
	}

	//
	// Reset modified
	//

	constexpr void resetModified() noexcept { modified = 0; }

	constexpr void resetModified(index_field_t index_field) noexcept
	{
		modified &= ~index_field;
	}

	constexpr void resetModifiedIndex(index_t index) noexcept
	{
		modified &= ~(index_field_t(1) << index);
	}
};

struct Leaf {
	// Indicates whether this is a leaf node (has no children) or not. If true then the
	// children are not valid and should not be accessed
	index_field_t leaf;

	//
	// Fill
	//

	void fill(Leaf const other, index_t const index)
	{
		if (index_field_t(1) & (other.leaf >> index)) {
			setLeaf();
		} else {
			resetLeaf();
		}
	}

	//
	// Is collapsible
	//

	[[nodiscard]] constexpr bool isCollapsible(Leaf const, index_t const) const
	{
		return isAllLeaf();
	}

	//
	// Is leaf
	//

	constexpr index_field_t isLeaf() const noexcept { return leaf; }

	constexpr index_field_t isLeaf(index_field_t indices) const noexcept
	{
		return leaf & indices;
	}

	constexpr bool isLeafIndex(index_t index) const noexcept
	{
		return (leaf >> index) & index_field_t(1);
	}

	constexpr bool isAllLeaf() const noexcept
	{
		return std::numeric_limits<index_field_t>::max() == leaf;
	}

	constexpr bool isAnyLeaf() const noexcept { return 0 != leaf; }

	constexpr bool isNoneLeaf() const noexcept { return 0 == leaf; }

	//
	// Is parent
	//

	constexpr index_field_t isParent() const noexcept { return ~isLeaf(); }

	constexpr index_field_t isParent(index_field_t indices) const noexcept
	{
		return ~isLeaf(indices);
	}

	constexpr bool isParentIndex(index_t index) const noexcept
	{
		return !isLeafIndex(index);
	}

	constexpr bool isAllParent() const noexcept { return isNoneLeaf(); }

	constexpr bool isAnyParent() const noexcept { return !isAllLeaf(); }

	constexpr bool isNoneParent() const noexcept { return !isAnyLeaf(); }

	//
	// Set leaf
	//

	constexpr void setLeaf() noexcept { leaf = std::numeric_limits<index_field_t>::max(); }

	constexpr void setLeaf(index_field_t indices) noexcept { leaf |= indices; }

	constexpr void setLeafIndex(index_t index) noexcept
	{
		leaf |= index_field_t(1) << index;
	}

	//
	// Reset leaf
	//

	constexpr void resetLeaf() noexcept { leaf = 0; }

	constexpr void resetLeaf(index_field_t indices) noexcept { leaf &= ~indices; }

	constexpr void resetLeafIndex(index_t index) noexcept
	{
		leaf &= ~(index_field_t(1) << index);
	}
};

template <class LeafNode, class InnerData>
struct OctreeInnerNode : LeafNode, Leaf, InnerData {
	using InnerNodeBlock = std::array<OctreeInnerNode, 8>;
	using LeafNodeBlock = std::array<LeafNode, 8>;

	// Pointer to children
	union {
		InnerNodeBlock* inner_children = nullptr;
		LeafNodeBlock* leaf_children;
	};

	//
	// Fill
	//

	void fill(OctreeInnerNode const& other, index_t index)
	{
		LeafNode::fill(other, index);
		Leaf::fill(other, index);
		InnerData::fill(other, index);
	}

	//
	// Is collapsible
	//

	[[nodiscard]] constexpr bool isCollapsible(OctreeInnerNode const& parent,
	                                           index_t index) const
	{
		return Leaf::isCollapsible(parent, index) && LeafNode::isCollapsible(parent, index) &&
		       InnerData::isCollapsible(parent, index);
	}
};

template <class LeafNode>
struct OctreeInnerNode<LeafNode, void> : LeafNode, Leaf {
	using InnerNodeBlock = std::array<OctreeInnerNode, 8>;
	using LeafNodeBlock = std::array<LeafNode, 8>;

	// Pointer to children
	union {
		InnerNodeBlock* inner_children = nullptr;
		LeafNodeBlock* leaf_children;
	};

	//
	// Fill
	//

	void fill(OctreeInnerNode const& other, index_t index)
	{
		Leaf::fill(other, index);
		LeafNode::fill(other, index);
	}

	//
	// Is collapsible
	//

	[[nodiscard]] constexpr bool isCollapsible(OctreeInnerNode const& parent,
	                                           index_t index) const
	{
		return Leaf::isCollapsible(parent, index) && LeafNode::isCollapsible(parent, index);
	}
};
}  // namespace ufo::map

#endif  // UFO_MAP_OCTREE_OCTREE_NODE_H