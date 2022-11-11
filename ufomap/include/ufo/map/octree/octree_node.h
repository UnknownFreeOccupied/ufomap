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
#include <ufo/map/code.h>
#include <ufo/map/types.h>

// STL
#include <array>
#include <atomic>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace ufo::map
{

template <typename... Nodes>
struct OctreeNode : Nodes... {
	//
	// Fill
	//

	void fill(OctreeNode const& parent, index_t const index)
	{
		(Nodes::fill(parent, index), ...);
	}

	//
	// Is collapsible
	//

	[[nodiscard]] bool isCollapsible() const { return (Nodes::isCollapsible() && ...); }
};

template <class Data, bool Reuse, bool Track>
struct OctreeLeafNode : Data {
	// Indicates whether this node has to be updated (get information from children and/or
	// update indicators). Useful when propagating information up the tree
	IndexField modified;

	//
	// Fill
	//

	void fill(OctreeLeafNode const& other, index_t index)
	{
		if (other.modified[index]) {
			modified.set();
		} else {
			modified.reset();
		}

		Data::fill(other, index);
	}

	//
	// Clear
	//

	constexpr void clear() {}

	//
	// Is collapsible
	//

	[[nodiscard]] bool isCollapsible() const { return Data::isCollapsible(); }
};

template <class Data>
struct OctreeLeafNode<Data, true, true> : Data {
	// Code for this node
	Code code;

	// Indicates whether this node has to be updated (get information from children and/or
	// update indicators). Useful when propagating information up the tree
	IndexField modified;

	//
	// Fill
	//

	void fill(OctreeLeafNode const& other, index_t index)
	{
		if (other.modified[index]) {
			modified.set();
		} else {
			modified.reset();
		}

		Data::fill(other, index);
		code = other.code.child(index);
	}

	//
	// Clear
	//

	constexpr void clear() { code = Code(); }

	//
	// Is collapsible
	//

	[[nodiscard]] bool isCollapsible() const { return Data::isCollapsible(); }
};

template <class Data>
struct OctreeLeafNode<Data, false, true> {
	// Indicates whether this node has to be updated (get information from children and/or
	// update indicators). Useful when propagating information up the tree
	IndexField modified;

	// Indicates whether this node actual is part of the octree
	bool exists = false;

	//
	// Fill
	//

	void fill(OctreeLeafNode const& other, index_t index)
	{
		if (other.modified[index]) {
			modified.set();
		} else {
			modified.reset();
		}

		Data::fill(other, index);
		exists = true;
	}

	//
	// Clear
	//

	void clear() { exists = false; }

	//
	// Is collapsible
	//

	[[nodiscard]] bool isCollapsible() const { return Data::isCollapsible(); }
};

struct Leaf {
	// Indicates whether this is a leaf node (has no children) or not. If true then the
	// children are not valid and should not be accessed
	IndexField leaf;

	//
	// Fill
	//

	void fill(Leaf const parent, index_t const index)
	{
		if (parent.leaf[index]) {
			leaf.set();
		} else {
			leaf.reset();
		}
	}

	//
	// Is collapsible
	//

	[[nodiscard]] constexpr bool isCollapsible() const { return leaf.all(); }
};

template <class LeafNode, class InnerData, bool Lock>
struct OctreeInnerNode : LeafNode, Leaf, InnerData {
	using InnerNodeBlock = std::array<OctreeInnerNode, 8>;
	using LeafNodeBlock = std::array<LeafNode, 8>;

	// Pointer to children
	// union {
	// 	InnerNodeBlock* inner_children = nullptr;
	// 	LeafNodeBlock* leaf_children;
	// };

	union {
		std::uint32_t inner_children = std::numeric_limits<std::uint32_t>::max();
		std::uint32_t leaf_children;
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

	[[nodiscard]] bool isCollapsible() const
	{
		return Leaf::isCollapsible() && LeafNode::isCollapsible() &&
		       InnerData::isCollapsible();
	}
};

template <class LeafNode, class InnerData>
struct OctreeInnerNode<LeafNode, InnerData, true> : LeafNode, Leaf, InnerData {
	using InnerNodeBlock = std::array<OctreeInnerNode, 8>;
	using LeafNodeBlock = std::array<LeafNode, 8>;

	// Lock
	std::atomic_flag lock;

	// Pointer to children
	// union {
	// 	InnerNodeBlock* inner_children = nullptr;
	// 	LeafNodeBlock* leaf_children;
	// };

	union {
		std::uint32_t inner_children = std::numeric_limits<std::uint32_t>::max();
		std::uint32_t leaf_children;
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

	[[nodiscard]] bool isCollapsible() const
	{
		return Leaf::isCollapsible() && LeafNode::isCollapsible() &&
		       InnerData::isCollapsible();
	}
};

template <class LeafNode>
struct OctreeInnerNode<LeafNode, void, false> : LeafNode, Leaf {
	using InnerNodeBlock = std::array<OctreeInnerNode, 8>;
	using LeafNodeBlock = std::array<LeafNode, 8>;

	// Pointer to children
	// union {
	// 	InnerNodeBlock* inner_children = nullptr;
	// 	LeafNodeBlock* leaf_children;
	// };

	union {
		std::uint32_t inner_children = std::numeric_limits<std::uint32_t>::max();
		std::uint32_t leaf_children;
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

	[[nodiscard]] bool isCollapsible() const
	{
		return Leaf::isCollapsible() && LeafNode::isCollapsible();
	}
};

template <class LeafNode>
struct OctreeInnerNode<LeafNode, void, true> : LeafNode, Leaf {
	using InnerNodeBlock = std::array<OctreeInnerNode, 8>;
	using LeafNodeBlock = std::array<LeafNode, 8>;

	// Lock
	std::atomic_flag lock;

	// Pointer to children
	// union {
	// 	InnerNodeBlock* inner_children = nullptr;
	// 	LeafNodeBlock* leaf_children;
	// };

	union {
		std::uint32_t inner_children = std::numeric_limits<std::uint32_t>::max();
		std::uint32_t leaf_children;
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

	[[nodiscard]] bool isCollapsible() const
	{
		return Leaf::isCollapsible() && LeafNode::isCollapsible();
	}
};
}  // namespace ufo::map

#endif  // UFO_MAP_OCTREE_OCTREE_NODE_H