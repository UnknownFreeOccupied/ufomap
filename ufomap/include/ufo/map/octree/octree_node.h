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

// STL
#include <array>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace ufo::map
{
template <class Data>
struct OctreeLeafNode : Data {
	// Indicates whether this is a leaf node (has no children) or not. If true then the
	// children are not valid and should not be accessed
	uint8_t is_leaf;
	// Indicates whether this node has to be updated (get information from children and/or
	// update indicators). Useful when propagating information up the tree
	uint8_t modified;

	bool operator==(OctreeLeafNode const& rhs) const
	{
		return static_cast<Data const&>(rhs) == static_cast<Data const&>(*this);
	}

	bool operator!=(OctreeLeafNode const& rhs) const { return !(*this == rhs); }

	constexpr bool isLeaf(std::size_t const index) const
	{
		return (is_leaf >> index) & uint8_t(1);
	}

	constexpr bool isAllLeaf() const
	{
		return std::numeric_limits<uint8_t>::max() == is_leaf;
	}

	constexpr bool isAnyLeaf() const { return 0 != is_leaf; }

	constexpr bool isNoneLeaf() const { return 0 == is_leaf; }

	constexpr bool isModified(std::size_t const index) const
	{
		return (modified >> index) & uint8_t(1);
	}

	constexpr bool isAllModified() const
	{
		return std::numeric_limits<uint8_t>::max() == modified;
	}

	constexpr bool isAnyModified() const { return 0 != modified; }

	constexpr bool isNoneModified() const { return 0 == modified; }

	constexpr void setLeaf(bool const value) const
	{
		is_leaf = value ? std::numeric_limits<uint8_t>::max() : uint8_t(0);
	}

	constexpr void setLeaf(std::size_t const index, bool const value) const
	{
		is_leaf = (is_leaf & ~(uint8_t(1) << index)) |
		          (uint8_t(value ? uint8_t(1) : uint8_t(0)) << index);
	}

	constexpr void setModified(bool const value) const
	{
		modified = value ? std::numeric_limits<uint8_t>::max() : uint8_t(0);
	}

	constexpr void setModified(std::size_t const index, bool const value) const
	{
		modified = (modified & ~(uint8_t(1) << index)) |
		           (uint8_t(value ? uint8_t(1) : uint8_t(0)) << index);
	}
};

template <class LeafNode>
struct OctreeInnerNode : LeafNode {
	using InnerNodeBlock = std::array<OctreeInnerNode, 8>;
	using LeafNodeBlock = std::array<LeafNode, 8>;

	// Pointer to children
	union {
		InnerNodeBlock* inner_children = nullptr;
		LeafNodeBlock* leaf_children;
	};
};
}  // namespace ufo::map

#endif  // UFO_MAP_OCTREE_OCTREE_NODE_H