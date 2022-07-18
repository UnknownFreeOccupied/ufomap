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

#ifndef UFO_MAP_OCTREE_MEMORY_INDEX_H
#define UFO_MAP_OCTREE_MEMORY_INDEX_H

// UFO
#include <ufo/map/octree/memory/index_node.h>
#include <ufo/map/octree/memory/memory_base.h>
#include <ufo/map/octree/node.h>
#include <ufo/map/octree/octree_node.h>

// STL
#include <array>
#include <atomic>
#include <mutex>
#include <shared_mutex>
#include <stack>
#include <type_traits>
#include <vector>

namespace ufo::map
{
template <class DataType, class Indicators, bool LockLess, bool ReuseNodes>
class OctreeIndexMemory
    : public MemoryBase<OctreeLeafNode<DataType, Indicators>,
                        OctreeIndexInnerNode<OctreeLeafNode<DataType, Indicators>>,
                        LockLess>
{
 public:
	using LeafNode = OctreeLeafNode<DataType, Indicators>;
	using InnerNode = OctreeIndexInnerNode<LeafNode>;

	using LeafNodeBlock = typename InnerNode::LeafChildrenBlock;
	using InnerNodeBlock = typename InnerNode::InnerChildrenBlock;

	using index_t = typename OctreeIndexInnerNode<LeafNode>::index_type;

 public:
	OctreeIndexMemory()
	{
		inner_blocks_.reserve(10000000);
		leaf_blocks_.reserve(10000000);
	}

	//
	// Reserve
	//

	void reserve(std::size_t num_leaf_nodes, std::size_t num_inner_nodes)
	{
		// TODO: Implement
	}

 protected:
	//
	// Get root
	//

	constexpr InnerNode const& getRoot() const noexcept
	{
		if constexpr (LockLess) {
			return inner_blocks_[0][0];
		} else {
			std::shared_lock lock(inner_blocks_mutex_);
			return inner_blocks_[0][0];
		}
	}

	constexpr InnerNode& getRoot() noexcept
	{
		if constexpr (LockLess) {
			return inner_blocks_[0][0];
		} else {
			std::shared_lock lock(inner_blocks_mutex_);
			return inner_blocks_[0][0];
		}
	}

	//
	// Get root node
	//

	/*!
	 * @brief Get the root node.
	 *
	 * @return The root node.
	 */
	constexpr Node getRootNode(Code root_code) const { return Node(0, 0, root_code); }

	//
	// Create node
	//

	Node createNode(Code code, depth_t root_depth)
	{
		if (code.depth() == root_depth) {
			return getRootNode(code);
		}

		InnerNode* parent_node = &getRoot();
		auto const parent_depth = code.depth() + 1;
		for (auto depth = root_depth; parent_depth < depth; --depth) {
			createChildren(*parent_node, depth);
			parent_node = &getInnerChild(*parent_node, code.indexAtDepth(depth - 1));
		}

		createChildren(*parent_node, parent_depth);
		return Node(getChildrenIndex(*parent_node), code.indexAtDepth(code.depth()), code);
	}

	//
	// Get leaf node
	//

	LeafNode const& getLeafNode(Node node) const
	{
		if (0 != node.depth()) {
			return getInnerNode(node);
		}

		auto index = node.dataIndex();
		if constexpr (LockLess) {
			return leaf_blocks_[index.block_index_][index.child_index_];
		} else {
			std::shared_lock lock(leaf_blocks_mutex_);
			return leaf_blocks_[index.block_index_][index.child_index_];
		}
	}

	LeafNode& getLeafNode(Node node)
	{
		if (0 != node.depth()) {
			return getInnerNode(node);
		}

		auto index = node.dataIndex();
		if constexpr (LockLess) {
			return leaf_blocks_[index.block_index_][index.child_index_];
		} else {
			std::shared_lock lock(leaf_blocks_mutex_);
			return leaf_blocks_[index.block_index_][index.child_index_];
		}
	}

	//
	// Get inner node
	//

	InnerNode const& getInnerNode(Node node) const
	{
		if (0 == node.depth()) {
			return getLeafNode(node);
		}

		auto index = node.dataIndex();
		if constexpr (LockLess) {
			return inner_blocks_[index.block_index_][index.child_index_];
		} else {
			std::shared_lock lock(inner_blocks_mutex_);
			return inner_blocks_[index.block_index_][index.child_index_];
		}
	}

	InnerNode& getInnerNode(Node node)
	{
		if (0 == node.depth()) {
			return getLeafNode(node);
		}

		auto index = node.dataIndex();
		if constexpr (LockLess) {
			return inner_blocks_[index.block_index_][index.child_index_];
		} else {
			std::shared_lock lock(inner_blocks_mutex_);
			return inner_blocks_[index.block_index_][index.child_index_];
		}
	}

	//
	// Get leaf children
	//

	LeafNodeBlock const& getLeafChildren(InnerNode const& node) const
	{
		if constexpr (LockLess) {
			return leaf_blocks_[node.children_index];
		} else {
			std::shared_lock lock(leaf_blocks_mutex_);
			return leaf_blocks_[node.children_index];
		}
	}

	LeafNodeBlock& getLeafChildren(InnerNode& node)
	{
		if constexpr (LockLess) {
			return leaf_blocks_[node.children_index];
		} else {
			std::shared_lock lock(leaf_blocks_mutex_);
			return leaf_blocks_[node.children_index];
		}
	}

	//
	// Get inner children
	//

	InnerNodeBlock const& getInnerChildren(InnerNode const& node) const
	{
		if constexpr (LockLess) {
			return inner_blocks_[node.children_index];
		} else {
			std::shared_lock lock(inner_blocks_mutex_);
			return inner_blocks_[node.children_index];
		}
	}

	InnerNodeBlock& getInnerChildren(InnerNode& node)
	{
		if constexpr (LockLess) {
			return inner_blocks_[node.children_index];
		} else {
			std::shared_lock lock(inner_blocks_mutex_);
			return inner_blocks_[node.children_index];
		}
	}

	//
	// Get leaf child
	//

	LeafNode const& getLeafChild(InnerNode const& node, unsigned int const idx) const
	{
		return getLeafChildren(node)[idx];
	}

	LeafNode& getLeafChild(InnerNode& node, unsigned int const idx)
	{
		return getLeafChildren(node)[idx];
	}

	//
	// Get inner child
	//

	InnerNode const& getInnerChild(InnerNode const& node, unsigned int const idx) const
	{
		return getInnerChildren(node)[idx];
	}

	InnerNode& getInnerChild(InnerNode& node, unsigned int const idx)
	{
		return getInnerChildren(node)[idx];
	}

	//
	// Get child
	//

	LeafNode const& getChild(InnerNode const& node, unsigned int const idx,
	                         depth_t depth) const
	{
		return 1 == depth ? getLeafChild(node, idx)
		                  : static_cast<LeafNode const&>(getInnerChild(node, idx));
	}

	LeafNode& getChild(InnerNode& node, unsigned int const idx, depth_t depth)
	{
		return 1 == depth ? getLeafChild(node, idx)
		                  : static_cast<LeafNode&>(getInnerChild(node, idx));
	}

	//
	// Create children
	//

	void createChildren(InnerNode& node, depth_t depth)
	{
		std::unique_lock inner_lock{inner_blocks_mutex_, std::defer_lock};
		std::unique_lock leaf_lock{leaf_blocks_mutex_, std::defer_lock};

		if constexpr (!LockLess) {
			if (1 == depth) {
				leaf_lock.lock();
			} else {
				inner_lock.lock();
			}
		}

		if (this->isParent(node)) {
			return;
		}

		if (1 == depth) {
			createLeafChildren(node);
		} else {
			createInnerChildren(node);
		}

		this->setIsLeaf(node, false);
	}

	//
	// Delete children
	//

	void deleteChildren(InnerNode& node, depth_t depth, bool prune)
	{
		if (this->isLeaf(node)) {
			return;
		}

		this->setIsLeaf(node, true);

		if (1 == depth) {
			deleteLeafChildren(node, prune);
		} else {
			for (InnerNode& child : getInnerChildren(node)) {
				deleteChildren(child, depth - 1, prune);
			}

			deleteInnerChildren(node, prune);
		}
	}

	template <class T>
	void deleteChildren(T t, InnerNode& node, depth_t depth, bool prune)
	{
		// TODO: Implement
		deleteChildren(node, depth, prune);
	}

 private:
	//
	// Get children index
	//

	constexpr index_t getChildrenIndex(InnerNode const& node) const
	{
		return node.children_index;
	}

	//
	// Get leaf children lock free
	//

	LeafNodeBlock& getLeafChildrenLockFree(InnerNode& node)
	{
		return leaf_blocks_[node.children_index];
	}

	//
	// Get inner children lock free
	//

	InnerNodeBlock& getInnerChildrenLockFree(InnerNode& node)
	{
		return inner_blocks_[node.children_index];
	}

	//
	// Create leaf children
	//

	void createLeafChildren(InnerNode& node)
	{
		if (std::numeric_limits<index_t>::max() != node.children_index) {
			// Use existing nodes
		} else if (this->lockIfNonEmptyFreeLeafBlocks(free_leaf_blocks_)) {
			// Take existing children
			node.children_index = free_leaf_blocks_.top();
			free_leaf_blocks_.pop();
			this->unlockFreeLeafBlocks();
		} else {
			// Allocate children
			node.children_index = leaf_blocks_.size();
			leaf_blocks_.emplace_back();
			this->num_allocated_leaf_nodes_ += 8;
			this->num_allocated_inner_leaf_nodes_ -= 1;
			this->num_allocated_inner_nodes_ += 1;
		}

		getLeafChildrenLockFree(node).fill(static_cast<LeafNode&>(node));

		this->num_leaf_nodes_ += 8;
		this->num_inner_leaf_nodes_ -= 1;
		this->num_inner_nodes_ += 1;
	}

	//
	// Create inner children
	//

	void createInnerChildren(InnerNode& node)
	{
		if (std::numeric_limits<index_t>::max() != node.children_index) {
			// Use existing nodes
		} else if (this->lockIfNonEmptyFreeInnerBlocks(free_inner_blocks_)) {
			// Take existing children
			node.children_index = free_inner_blocks_.top();
			free_inner_blocks_.pop();
			this->unlockFreeInnerBlocks();
		} else {
			// Allocate children
			node.children_index = inner_blocks_.size();
			inner_blocks_.emplace_back();
			// Get 8 new and 1 is made into a inner node
			this->num_allocated_inner_leaf_nodes_ += 7;
			this->num_allocated_inner_nodes_ += 1;
		}

		for (InnerNode& child : getInnerChildrenLockFree(node)) {
			static_cast<LeafNode&>(child) = static_cast<LeafNode&>(node);
		}

		this->num_inner_leaf_nodes_ += 7;
		this->num_inner_nodes_ += 1;
	}

	//
	// Delete leaf children
	//

	void deleteLeafChildren(InnerNode& node, bool prune)
	{
		this->num_leaf_nodes_ -= 8;
		this->num_inner_leaf_nodes_ += 1;
		this->num_inner_nodes_ -= 1;

		if (prune) {
			this->lockFreeLeafBlocks();
			free_leaf_blocks_.push(node.children_index);
			this->unlockFreeLeafBlocks();
			this->num_allocated_leaf_nodes_ -= 8;
			this->num_allocated_inner_leaf_nodes_ += 1;
			this->num_allocated_inner_nodes_ -= 1;
			node.children_index = std::numeric_limits<index_t>::max();
		} else {
			if constexpr (ReuseNodes) {
				this->lockFreeLeafBlocks();
				free_leaf_blocks_.push(node.children_index);
				this->unlockFreeLeafBlocks();
				node.children_index = std::numeric_limits<index_t>::max();
			}
		}
	}

	//
	// Delete inner children
	//

	void deleteInnerChildren(InnerNode& node, bool prune)
	{
		this->num_inner_leaf_nodes_ -= 7;
		this->num_inner_nodes_ -= 1;

		if (prune) {
			this->lockFreeInnerBlocks();
			free_inner_blocks_.push(node.children_index);
			this->unlockFreeInnerBlocks();
			// Remove 8 and 1 inner node is made into a inner leaf node
			this->num_allocated_inner_leaf_nodes_ -= 7;
			this->num_allocated_inner_nodes_ -= 1;
			node.children_index = std::numeric_limits<index_t>::max();
		} else {
			if constexpr (ReuseNodes) {
				this->lockFreeInnerBlocks();
				free_inner_blocks_.push(node.children_index);
				this->unlockFreeInnerBlocks();
				node.children_index = std::numeric_limits<index_t>::max();
			}
		}
	}

 private:
	std::vector<InnerNodeBlock> inner_blocks_ = {InnerNodeBlock()};
	std::vector<LeafNodeBlock> leaf_blocks_;

	mutable std::shared_mutex inner_blocks_mutex_;
	mutable std::shared_mutex leaf_blocks_mutex_;

	std::stack<index_t, std::vector<index_t>> free_inner_blocks_;
	std::stack<index_t, std::vector<index_t>> free_leaf_blocks_;
};
}  // namespace ufo::map

#endif  // UFO_MAP_OCTREE_MEMORY_INDEX_H