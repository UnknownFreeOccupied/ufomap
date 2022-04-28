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

#ifndef UFO_MAP_OCTREE_OCTREE_NODE_H
#define UFO_MAP_OCTREE_OCTREE_NODE_H

// STL
#include <cstdint>
#include <iostream>
#include <type_traits>

namespace ufo::map
{
struct OctreeIndicators {
	// Indicates whether this is a leaf node (has no children) or not. If true then the
	// children are not valid and should not be accessed
	uint8_t is_leaf : 1;
	// Indicates whether this node has to be updated (get information from children and/or
	// update indicators). Useful when propagating information up the tree
	uint8_t modified : 1;
};

template <typename Data, class Indicators>
struct OctreeDataLeafNode : Indicators {
	Data value;

	bool operator==(OctreeDataLeafNode const& rhs) const noexcept
	{
		return static_cast<Data const&>(rhs.value) == value;
	}
	bool operator!=(OctreeDataLeafNode const& rhs) const noexcept
	{
		return static_cast<Data const&>(rhs.value) != value;
	}
};

template <class Data, class Indicators>
struct OctreeLeafNode : Data, Indicators {
	bool operator==(OctreeLeafNode const& rhs) const noexcept
	{
		return static_cast<Data const&>(rhs) == static_cast<Data const&>(*this);
	}
	bool operator!=(OctreeLeafNode const& rhs) const noexcept
	{
		return static_cast<Data const&>(rhs) != static_cast<Data const&>(*this);
	}
};

template <class LeafNode>
struct OctreeInnerNode : LeafNode {
	// Pointer to children
	void* children = nullptr;
};

//
// Is octree leaf node
//

// template <class>
// struct is_octree_leaf_node_impl : std::false_type {
// };

// template <class D, class I>
// struct is_octree_leaf_node_impl<OctreeLeafNode<D, I>> : std::true_type {
// };

// template <class D, class I>
// struct is_octree_leaf_node
//     : is_octree_leaf_node_impl<std::remove_cv_t<D>, std::remove_cv_t<I>> {
// };

// // Helper variable
// template <class D, class I>
// inline constexpr bool is_octree_leaf_node_v = is_octree_leaf_node<D, I>::value;

// //
// // Is octree inner node
// //

// template <class>
// struct is_octree_inner_node_impl : std::false_type {
// };

// template <class L>
// struct is_octree_inner_node_impl<OctreeInnerNode<L>> : std::true_type {
// };

// template <class L>
// struct is_octree_inner_node : is_octree_inner_node_impl<std::remove_cv_t<L>> {
// };  // TODO: remove_cv_t or decay_t?

// // Helper variable template
// template <class L>
// inline constexpr bool is_octree_inner_node_v = is_octree_inner_node<L>::value;
}  // namespace ufo::map

#endif  // UFO_MAP_OCTREE_OCTREE_NODE_H