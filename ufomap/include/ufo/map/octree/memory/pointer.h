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

#ifndef UFO_MAP_OCTREE_MEMORY_POINTER_H
#define UFO_MAP_OCTREE_MEMORY_POINTER_H

// UFO
#include <ufo/map/octree/memory/memory_base.h>
#include <ufo/map/octree/memory/pointer_node.h>
#include <ufo/map/octree/node.h>
#include <ufo/map/octree/octree_node.h>

// STL
#include <array>
#include <atomic>
#include <stack>
#include <type_traits>
#include <vector>

namespace ufo::map
{
template <class DataType, class Indicators, bool LockLess, bool ReuseNodes>
class OctreePointerMemory
    : public MemoryBase<OctreeLeafNode<DataType, Indicators>,
                        OctreePointerInnerNode<OctreeLeafNode<DataType, Indicators>>,
                        LockLess>
{
 public:
	using LeafNode = OctreeLeafNode<DataType, Indicators>;
	using InnerNode = OctreePointerInnerNode<LeafNode>;

	using LeafNodeBlock = typename InnerNode::LeafChildrenBlock;
	using InnerNodeBlock = typename InnerNode::InnerChildrenBlock;

 public:
	//
	// Reserve
	//

	void reserve(std::size_t num_leaf_nodes, std::size_t num_inner_nodes)
	{
		if constexpr (ReuseNodes) {
			// TODO: Implement
		}
	}

 protected:
	//
	// Get root
	//

	constexpr InnerNode const& getRoot() const noexcept { return *root_; }

	constexpr InnerNode& getRoot() noexcept { return *root_; }

	//
	// Get root node
	//

	/*!
	 * @brief Get the root node.
	 *
	 * @return The root node.
	 */
	constexpr Node getRootNode(Code root_code) const
	{
		return Node(const_cast<InnerNode*>(&getRoot()), root_code);
	}

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
		return Node(&getChild(*parent_node, code.indexAtDepth(code.depth()), parent_depth),
		            code);
	}

	//
	// Get leaf node
	//

	LeafNode const& getLeafNode(Node node) const
	{
		return *static_cast<LeafNode*>(node.data());
	}

	LeafNode& getLeafNode(Node node) { return *static_cast<LeafNode*>(node.data()); }

	//
	// Get inner node
	//

	InnerNode const& getInnerNode(Node node) const
	{
		return *static_cast<InnerNode*>(node.data());
	}

	InnerNode& getInnerNode(Node node) { return *static_cast<InnerNode*>(node.data()); }

	//
	// Get leaf children
	//

	LeafNodeBlock const& getLeafChildren(InnerNode const& node) const
	{
		return *node.leaf_children;
	}

	LeafNodeBlock& getLeafChildren(InnerNode& node) { return *node.leaf_children; }

	//
	// Get inner children
	//

	InnerNodeBlock const& getInnerChildren(InnerNode const& node) const
	{
		return *node.inner_children;
	}

	InnerNodeBlock& getInnerChildren(InnerNode& node) { return *node.inner_children; }

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
		if constexpr (!LockLess) {
			if (!this->lockIfLeaf(node, depth)) {
				return;
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

		if constexpr (!LockLess) {
			this->unlockChildren(depth);
		}
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
	// Create leaf children
	//

	void createLeafChildren(InnerNode& node)
	{
		if constexpr (!ReuseNodes) {
			if (!node.leaf_children) {
				// Allocated children
				node.leaf_children = new LeafNodeBlock();
				this->num_allocated_leaf_nodes_ += 8;
				this->num_allocated_inner_leaf_nodes_ -= 1;
				this->num_allocated_inner_nodes_ += 1;
			}
		} else {
			if (this->lockIfNonEmptyFreeLeafBlocks(free_leaf_blocks_)) {
				// Take existing children
				node.leaf_children = free_leaf_blocks_.top();
				free_leaf_blocks_.pop();
				this->unlockFreeLeafBlocks();
			} else {
				// Allocate children
				node.leaf_children = new LeafNodeBlock();
				this->num_allocated_leaf_nodes_ += 8;
				this->num_allocated_inner_leaf_nodes_ -= 1;
				this->num_allocated_inner_nodes_ += 1;
			}
		}

		getLeafChildren(node).fill(static_cast<LeafNode&>(node));

		this->num_leaf_nodes_ += 8;
		this->num_inner_leaf_nodes_ -= 1;
		this->num_inner_nodes_ += 1;
	}

	//
	// Create inner children
	//

	void createInnerChildren(InnerNode& node)
	{
		if constexpr (!ReuseNodes) {
			if (!node.inner_children) {
				// Allocate children
				node.inner_children = new InnerNodeBlock();
				// Get 8 new and 1 is made into a inner node
				this->num_allocated_inner_leaf_nodes_ += 7;
				this->num_allocated_inner_nodes_ += 1;
			}
		} else {
			if (this->lockIfNonEmptyFreeInnerBlocks(free_inner_blocks_)) {
				// Take existing children
				node.inner_children = free_inner_blocks_.top();
				free_inner_blocks_.pop();
				this->unlockFreeInnerBlocks();
			} else {
				// Allocate children
				node.inner_children = new InnerNodeBlock();
				// Get 8 new and 1 is made into a inner node
				this->num_allocated_inner_leaf_nodes_ += 7;
				this->num_allocated_inner_nodes_ += 1;
			}
		}

		for (InnerNode& child : getInnerChildren(node)) {
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
			delete &getLeafChildren(node);
			this->num_allocated_leaf_nodes_ -= 8;
			this->num_allocated_inner_leaf_nodes_ += 1;
			this->num_allocated_inner_nodes_ -= 1;
			node.leaf_children = nullptr;
		} else {
			if constexpr (ReuseNodes) {
				this->lockFreeLeafBlocks();
				free_leaf_blocks_.push(&getLeafChildren(node));
				this->unlockFreeLeafBlocks();
				node.leaf_children = nullptr;
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
			delete &getInnerChildren(node);
			// Remove 8 and 1 inner node is made into a inner leaf node
			this->num_allocated_inner_leaf_nodes_ -= 7;
			this->num_allocated_inner_nodes_ -= 1;
			node.inner_children = nullptr;
		} else {
			if constexpr (ReuseNodes) {
				this->lockFreeInnerBlocks();
				free_inner_blocks_.push(&getInnerChildren(node));
				this->unlockFreeInnerBlocks();
				node.inner_children = nullptr;
			}
		}
	}

	//
	// Lock children
	//

	bool tryLockChildren(depth_t depth)
	{
		if constexpr (!LockLess) {
			return !children_locks_[depth].test_and_set(std::memory_order_acquire);
		} else {
			return true;
		}
	}

	void lockChildren(depth_t depth)
	{
		if constexpr (!LockLess) {
			while (!tryLockChildren(depth))
				;
		}
	}

	bool lockIfLeaf(InnerNode const& node, depth_t depth)
	{
		if constexpr (!LockLess) {
			do {
				if (this->isParent(node)) {
					return false;
				}
			} while (!tryLockChildren(depth));

			if (this->isParent(node)) {
				unlockChildren(depth);
				return false;
			}

			return true;
		} else {
			return this->isLeaf(node);
		}
	}

	//
	// Unlock children
	//

	void unlockChildren(depth_t depth)
	{
		if constexpr (!LockLess) {
			children_locks_[depth].clear(std::memory_order_release);
		}
	}

 private:
	std::unique_ptr<InnerNode> root_ = std::make_unique<InnerNode>();

	// Locks to support parallel insertion, one per level
	std::array<std::atomic_flag, 21> children_locks_ = {
	    {ATOMIC_FLAG_INIT, ATOMIC_FLAG_INIT, ATOMIC_FLAG_INIT, ATOMIC_FLAG_INIT,
	     ATOMIC_FLAG_INIT, ATOMIC_FLAG_INIT, ATOMIC_FLAG_INIT, ATOMIC_FLAG_INIT,
	     ATOMIC_FLAG_INIT, ATOMIC_FLAG_INIT, ATOMIC_FLAG_INIT, ATOMIC_FLAG_INIT,
	     ATOMIC_FLAG_INIT, ATOMIC_FLAG_INIT, ATOMIC_FLAG_INIT, ATOMIC_FLAG_INIT,
	     ATOMIC_FLAG_INIT, ATOMIC_FLAG_INIT, ATOMIC_FLAG_INIT, ATOMIC_FLAG_INIT,
	     ATOMIC_FLAG_INIT}};

	//
	// Only used when nodes are reused
	//

	std::conditional_t<ReuseNodes,
	                   std::stack<InnerNodeBlock*, std::vector<InnerNodeBlock*>>,
	                   std::array<bool, 0>>
	    free_inner_blocks_;
	std::conditional_t<ReuseNodes, std::stack<LeafNodeBlock*, std::vector<LeafNodeBlock*>>,
	                   std::array<bool, 0>>
	    free_leaf_blocks_;
};
}  // namespace ufo::map

#endif  // UFO_MAP_OCTREE_MEMORY_POINTER_H