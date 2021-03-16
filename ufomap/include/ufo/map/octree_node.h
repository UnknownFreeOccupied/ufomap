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

#ifndef UFO_MAP_OCTREE_NODE_H
#define UFO_MAP_OCTREE_NODE_H

// STD
#include <iostream>

namespace ufo::map
{
// Have DEPTH in case it is of type OctreeInnerNode
template <typename T>
struct OctreeLeafNode {
	T value;

	/**
	 * @brief Write the data from this node to stream s
	 *
	 * @param s The stream to write the data to
	 * @return std::ostream&
	 */
	std::ostream& writeData(std::ostream& s) const
	{
		return value.writeData(s);
	}

	/**
	 * @brief Read the data for this node from stream s
	 *
	 * @param s The stream to read the data from
	 * @return std::istream&
	 */
	std::istream& readData(std::istream& s)
	{
		return value.readData(s);
	}
};

template <typename T>
struct OctreeInnerNodeBase : T {
	// Note: Important that the bools are before the children because of alignment

	// Indicates whether this is a leaf node (has no children) or not. If true then the
	// children are not valid and should not be accessed
	bool is_leaf = true;

	// Pointer to children
	void* children = nullptr;

	// void setChildExists(std::size_t index, bool value)
	// {
	// 	// Source:
	// 	//
	// https://stackoverflow.com/questions/47981/how-do-you-set-clear-and-toggle-a-single-bit
	// 	if (value) {
	// 		child_exists |= 1U << index;
	// 	} else {
	// 		child_exists &= ~(1U << index);
	// 	}
	// }

	// bool childExists(std::size_t index) const
	// {
	// 	// Source:
	// 	//
	// https://stackoverflow.com/questions/523724/c-c-check-if-one-bit-is-set-in-i-e-int-variable
	// 	return child_exist & (1U << index);
	// }
};

template <typename T>
using OctreeInnerNode = OctreeInnerNodeBase<OctreeLeafNode<T>>;
}  // namespace ufo::map

#endif  // UFO_MAP_OCTREE_NODE_H