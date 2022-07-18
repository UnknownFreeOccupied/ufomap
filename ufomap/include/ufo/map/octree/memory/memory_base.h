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

#ifndef UFO_MAP_OCTREE_MEMORY_BASE_H
#define UFO_MAP_OCTREE_MEMORY_BASE_H

// UFO
#include <ufo/map/types.h>

// STL
#include <array>
#include <atomic>

namespace ufo::map
{

template <class LeafNode, class InnerNode, bool LockLess>
class MemoryBase
{
 public:
	//
	// Memory
	//

	/*!
	 * @return Number of inner nodes in the octree.
	 */
	[[nodiscard]] constexpr std::size_t numInnerNodes() const noexcept
	{
		return num_inner_nodes_;
	}

	/*!
	 * @return Number of inner leaf nodes in the octree.
	 */
	[[nodiscard]] constexpr std::size_t numInnerLeafNodes() const noexcept
	{
		return num_inner_leaf_nodes_;
	}

	/*!
	 * @return Number of leaf nodes in the octree.
	 */
	[[nodiscard]] constexpr std::size_t numLeafNodes() const noexcept
	{
		return num_leaf_nodes_;
	}

	/*!
	 * @return Number of nodes in the octree.
	 */
	[[nodiscard]] constexpr std::size_t numNodes() const noexcept
	{
		return numInnerNodes() + numInnerLeafNodes() + numLeafNodes();
	}

	/*!
	 * @brief This is lower bound memeory usage of an inner node.
	 *
	 * @note Additional data accessed by pointers inside an inner node are not counted.
	 *
	 * @return Memory usage of a single inner node.
	 */
	[[nodiscard]] constexpr std::size_t memoryInnerNode() const noexcept
	{
		return sizeof(InnerNode);
	}

	/*!
	 * @brief This is lower bound memeory usage of an inner leaf node.
	 *
	 * @note Additional data accessed by pointers inside an inner leaf node are not counted.
	 *
	 * @return Memory usage of a single inner leaf node.
	 */
	[[nodiscard]] constexpr std::size_t memoryInnerLeafNode() const noexcept
	{
		return sizeof(InnerNode);
	}

	/*!
	 * @brief This is lower bound memeory usage of a leaf node.
	 *
	 * @note Additional data accessed by pointers inside a leaf node are not counted.
	 *
	 * @return Memory usage of a single leaf node.
	 */
	[[nodiscard]] constexpr std::size_t memoryLeafNode() const noexcept
	{
		return sizeof(LeafNode);
	}

	/*!
	 * @brief Lower bound memory usage for all nodes.
	 *
	 * @note Does not account for pointed to data inside nodes.
	 *
	 * @return Memory usage of the octree.
	 */
	[[nodiscard]] std::size_t memoryUsage() const noexcept
	{
		return (numInnerNodes() * memoryInnerNode()) +
		       (numInnerLeafNodes() * memoryInnerLeafNode()) +
		       (numLeafNodes() * memoryLeafNode());
	}

	/*!
	 * @return Number of nodes in the octree.
	 */
	[[nodiscard]] constexpr std::size_t size() const noexcept { return numNodes(); }

	/*!
	 * @return Number of allocated inner nodes.
	 */
	[[nodiscard]] constexpr std::size_t numInnerNodesAllocated() const noexcept
	{
		return num_allocated_inner_nodes_;
	}

	/*!
	 * @return Number of allocated inner leaf nodes.
	 */
	[[nodiscard]] constexpr std::size_t numInnerLeafNodesAllocated() const noexcept
	{
		return num_allocated_inner_leaf_nodes_;
	}

	/*!
	 * @return Number of allocated leaf nodes.
	 */
	[[nodiscard]] constexpr std::size_t numLeafNodesAllocated() const noexcept
	{
		return num_allocated_leaf_nodes_;
	}

	/*!
	 * @return Number of allocated nodes.
	 */
	[[nodiscard]] constexpr std::size_t numNodesAllocated() const noexcept
	{
		return numInnerNodesAllocated() + numInnerLeafNodesAllocated() +
		       numLeafNodesAllocated();
	}

	/*!
	 * @brief Lower bound memory usage for all nodes.
	 *
	 * @note Does not account for pointed to data inside nodes.
	 *
	 * @return Memory usage of the octree.
	 */
	[[nodiscard]] std::size_t memoryUsageAllocated() const noexcept
	{
		return (numInnerNodesAllocated() * memoryInnerNode()) +
		       (numInnerLeafNodesAllocated() * memoryInnerLeafNode()) +
		       (numLeafNodesAllocated() * memoryLeafNode());
	}

	/*!
	 * @return Number of nodes in the octree.
	 */
	[[nodiscard]] constexpr std::size_t sizeAllocated() const noexcept
	{
		return numNodesAllocated();
	}

 protected:
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
	// Lock free leaf blocks
	//

	bool tryLockFreeLeafBlocks()
	{
		if constexpr (!LockLess) {
			return !free_leaf_block_lock_.test_and_set(std::memory_order_acquire);
		} else {
			return true;
		}
	}

	void lockFreeLeafBlocks()
	{
		if constexpr (!LockLess) {
			while (!tryLockFreeLeafBlocks())
				;
		}
	}

	template <class T>
	bool lockIfNonEmptyFreeLeafBlocks(T const& free_leaf_blocks)
	{
		if constexpr (!LockLess) {
			do {
				if (free_leaf_blocks.empty()) {
					return false;
				}
			} while (!tryLockFreeLeafBlocks());

			if (free_leaf_blocks.empty()) {
				unlockFreeLeafBlocks();
				return false;
			}
			return true;
		} else {
			return !free_leaf_blocks.empty();
		}
	}

	//
	// Unlock free leaf blocks
	//

	void unlockFreeLeafBlocks()
	{
		if constexpr (!LockLess) {
			free_leaf_block_lock_.clear(std::memory_order_release);
		}
	}

	//
	// Lock free inner blocks
	//

	bool tryLockFreeInnerBlocks()
	{
		if constexpr (!LockLess) {
			return !free_inner_block_lock_.test_and_set(std::memory_order_acquire);
		} else {
			return true;
		}
	}

	void lockFreeInnerBlocks()
	{
		if constexpr (!LockLess) {
			while (!tryLockFreeInnerBlocks())
				;
		}
	}

	template <class T>
	bool lockIfNonEmptyFreeInnerBlocks(T const& free_inner_blocks)
	{
		if constexpr (!LockLess) {
			do {
				if (free_inner_blocks.empty()) {
					return false;
				}
			} while (!tryLockFreeInnerBlocks());

			if (free_inner_blocks.empty()) {
				unlockFreeInnerBlocks();
				return false;
			}
			return true;
		} else {
			return !free_inner_blocks.empty();
		}
	}

	//
	// Unlock free inner blocks
	//

	void unlockFreeInnerBlocks()
	{
		if constexpr (!LockLess) {
			free_inner_block_lock_.clear(std::memory_order_release);
		}
	}

 protected:
	//
	// Locks for accessing free blocks
	//

	std::atomic_flag free_inner_block_lock_ = ATOMIC_FLAG_INIT;
	std::atomic_flag free_leaf_block_lock_ = ATOMIC_FLAG_INIT;

	//
	// Memory
	//

	// Current number of inner nodes
	std::atomic_size_t num_inner_nodes_ = 0;
	// Current number of inner leaf nodes
	std::atomic_size_t num_inner_leaf_nodes_ = 1;
	// Current number of leaf nodes
	std::atomic_size_t num_leaf_nodes_ = 0;

	// Current number of allocated inner nodes
	std::atomic_size_t num_allocated_inner_nodes_ = 0;
	// Current number of allocated inner leaf nodes
	std::atomic_size_t num_allocated_inner_leaf_nodes_ = 1;
	// Current number of allocated leaf nodes
	std::atomic_size_t num_allocated_leaf_nodes_ = 0;
};
}  // namespace ufo::map

#endif  // UFO_MAP_OCTREE_MEMORY_BASE_H