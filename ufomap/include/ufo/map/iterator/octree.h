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

#ifndef UFO_MAP_ITERATOR_OCTREE_H
#define UFO_MAP_ITERATOR_OCTREE_H

// UFO
#include <ufo/geometry/bounding_volume.h>
#include <ufo/geometry/minimum_distance.h>
#include <ufo/geometry/point.h>
#include <ufo/map/code/code.h>
#include <ufo/map/octree/node.h>
#include <ufo/map/predicate/octree.h>
#include <ufo/map/predicate/predicates.h>
#include <ufo/map/types.h>

// STL
#include <cstddef>   // For std::ptrdiff_t
#include <iterator>  // For std::forward_iterator_tag
#include <queue>
#include <set>
#include <stack>
#include <type_traits>
// #include <memory>
// #include <utility>

namespace ufo::map
{
template <class Tree, typename T>
class IteratorBase
{
 public:
	// Tags
	using iterator_category = std::forward_iterator_tag;
	using difference_type = std::ptrdiff_t;
	using value_type = T;
	using pointer = value_type*;
	using reference = value_type&;
	using const_pointer = value_type const*;
	using const_reference = value_type const&;

	constexpr IteratorBase() = default;

	constexpr IteratorBase(Tree const* tree) : tree_(tree) {}

	virtual IteratorBase& next() = 0;

	virtual IteratorBase* copy() = 0;

	// virtual reference data() = 0;
	virtual const_reference data() const = 0;

	constexpr Tree const* tree() const { return tree_; }

	virtual bool equal(IteratorBase const& other) const = 0;

 protected:
	template <class NodeType>
	constexpr depth_t depth(NodeType const& node) const
	{
		return node.depth();
	}

	template <class NodeType>
	constexpr Code code(NodeType const& node) const
	{
		return node.code();
	}

	template <class NodeType>
	constexpr std::size_t indexAtDepth(NodeType const& node) const
	{
		return code(node).indexAtDepth(depth(node));
	}

	template <class NodeType>
	constexpr bool isParent(NodeType const& node) const
	{
		return tree_->isParent(node);
	}

	template <class NodeType>
	constexpr NodeType child(NodeType const& node, unsigned int idx) const
	{
		return tree_->getNodeChild(node, idx);
	}

	template <class NodeType>
	constexpr NodeType sibling(NodeType const& node, unsigned int idx) const
	{
		return tree_->getNodeSibling(node, idx);
	}

	template <class NodeType, class Predicates>
	constexpr bool validInnerNode(NodeType const& node, Predicates const& predicates) const
	{
		return isParent(node) && validInnerNodeOnlyLeaves(node, predicates);
	}

	template <class NodeType, class Predicates>
	constexpr bool validInnerNodeOnlyLeaves(NodeType const& node,
	                                        Predicates const& predicates) const
	{
		return predicate::PredicateInnerCheck<Predicates>::apply(predicates, *tree_, node);
	}

	template <class NodeType, class Predicates>
	constexpr bool validReturnNode(NodeType const& node, Predicates const& predicates) const
	{
		return predicate::PredicateValueCheck<Predicates>::apply(predicates, *tree_, node);
	}

 protected:
	// The UFOMap
	Tree const* tree_ = nullptr;
};

template <class Tree, typename T>
class IteratorWrapper
{
 private:
	using Base = IteratorBase<Tree, T>;

 public:
	// Tags
	using iterator_category = typename Base::iterator_category;
	using difference_type = typename Base::difference_type;
	using value_type = typename Base::value_type;
	using pointer = typename Base::pointer;
	using reference = typename Base::reference;
	using const_pointer = typename Base::const_pointer;
	using const_reference = typename Base::const_reference;

	IteratorWrapper(Base* it_base) : it_base_(it_base) {}

	IteratorWrapper(IteratorWrapper const& other) : it_base_(other.it_base_->copy()) {}

	IteratorWrapper& operator++()
	{
		it_base_->next();
		return *this;
	}

	IteratorWrapper operator++(int)
	{
		IteratorWrapper result(it_base_->copy());
		++(*this);
		return result;
	}

	// pointer operator->() { return &(it_base_->data()); }

	// reference operator*() { return it_base_->data(); }

	const_pointer operator->() const { return &(it_base_->data()); }

	const_reference operator*() const { return it_base_->data(); }

	friend bool operator==(IteratorWrapper const& lhs, IteratorWrapper const& rhs)
	{
		return lhs.it_base_->equal(*(rhs.it_base_));
	}

	friend bool operator!=(IteratorWrapper const& lhs, IteratorWrapper const& rhs)
	{
		return !(lhs == rhs);
	}

 private:
	std::unique_ptr<Base> it_base_;
};

template <class BaseNodeType, class Tree, class NodeType = Node,
          class Predicates = predicate::TRUE>
class Iterator : public IteratorBase<Tree, BaseNodeType>
{
 private:
	static constexpr bool OnlyLeaves =
	    predicate::contains_always_predicate_v<predicate::Leaf, Predicates>;

	using Base = IteratorBase<Tree, BaseNodeType>;

 public:
	// Tags
	using typename Base::const_pointer;
	using typename Base::const_reference;
	using typename Base::difference_type;
	using typename Base::iterator_category;
	using typename Base::pointer;
	using typename Base::reference;
	using typename Base::value_type;

	Iterator(NodeType const& root)
	    : node_(root), predicates_(predicate::TRUE()), valid_inner_(false)
	{
	}

	Iterator(NodeType const& root, Predicates const& predicates)
	    : node_(root), predicates_(predicates), valid_inner_(false)
	{
	}

	Iterator(Tree const* tree, NodeType const& root, Predicates const& predicates)
	    : Base(tree), node_(root), predicates_(predicates), valid_inner_(false)
	{
		init();
	}

	Iterator& next() override
	{
		increment();
		return *this;
	}

	Iterator* copy() override { return new Iterator(*this); }

	// reference data() override { return node_; }

	const_reference data() const override { return node_; }

	bool equal(Base const& other) const override
	{
		return other.tree() == this->tree() && other.data() == data();
	}

 private:
	void init()
	{
		if constexpr (OnlyLeaves) {
			if (this->validReturnNode(node_, predicates_)) {
				valid_inner_ = false;
			} else {
				valid_inner_ = this->validInnerNodeOnlyLeaves(node_, predicates_);
				if (valid_inner_) {
					increment();
				} else {
					this->tree_ = nullptr;
				}
			}
		} else {
			valid_inner_ = this->validInnerNode(node_, predicates_);

			if (!this->validReturnNode(node_, predicates_)) {
				if (valid_inner_) {
					increment();
				} else {
					this->tree_ = nullptr;
				}
			}
		}
	}

	void increment()
	{
		// Skip forward to next valid return node
		while (this->tree_ && !singleIncrement()) {
		}
	}

	// Return true if valid return node
	bool singleIncrement()
	{
		// Go down the tree
		if (valid_inner_) {
			if constexpr (OnlyLeaves) {
				auto current = this->child(node_, 0);
				for (unsigned int idx = 0; idx != 8; ++idx) {
					current = this->sibling(current, idx);

					if (this->validReturnNode(current, predicates_)) {
						parents_.push(node_);
						node_ = current;
						valid_inner_ = false;
						return true;
					} else if (this->validInnerNodeOnlyLeaves(current, predicates_)) {
						parents_.push(node_);
						node_ = current;
						valid_inner_ = true;
						return false;
					}
				}
			} else {
				auto current = this->child(node_, 0);

				for (unsigned int idx = 0; idx != 8; ++idx) {
					current = this->sibling(current, idx);

					bool valid_inner = this->validInnerNode(current, predicates_);

					if (this->validReturnNode(current, predicates_)) {
						parents_.push(node_);
						node_ = current;
						valid_inner_ = valid_inner;
						return true;
					} else if (valid_inner) {
						parents_.push(node_);
						node_ = current;
						return false;
					}
				}
			}
		}

		// Go diagonal and up in the tree
		while (this->depth(node_) != this->tree_->depthLevels()) {
			for (auto idx = this->indexAtDepth(node_) + 1; 8 != idx; ++idx) {
				node_ = this->sibling(node_, idx);

				if constexpr (OnlyLeaves) {
					if (this->validReturnNode(node_, predicates_)) {
						valid_inner_ = false;
						return true;
					} else if (this->validInnerNodeOnlyLeaves(node_, predicates_)) {
						valid_inner_ = true;
						return false;
					}
				} else {
					valid_inner_ = this->validInnerNode(node_, predicates_);

					if (this->validReturnNode(node_, predicates_)) {
						return true;
					} else if (valid_inner_) {
						return false;
					}
				}
			}
			node_ = parents_.top();
			parents_.pop();
			valid_inner_ = true;
		}

		this->tree_ = nullptr;
		valid_inner_ = false;
		return false;
	}

 private:
	// Predicates
	Predicates predicates_;

	// If current is valid inner node
	bool valid_inner_;

	// Current node
	NodeType node_;

	// Parents
	std::stack<NodeType, std::vector<NodeType>> parents_;
};

struct NearestNode {
	NodeBV node;
	float squared_distance;

	constexpr NearestNode(NodeBV const& node, float squared_distance)
	    : node(node), squared_distance(squared_distance)
	{
	}

	friend constexpr bool operator==(NearestNode const& a, NearestNode const& b)
	{
		return a.node == b.node;
	}

	friend constexpr bool operator!=(NearestNode const& a, NearestNode const& b)
	{
		return !(a == b);
	}

	friend constexpr bool operator<(NearestNode const& a, NearestNode const& b)
	{
		return a.squared_distance < b.squared_distance;
	}

	friend constexpr bool operator<=(NearestNode const& a, NearestNode const& b)
	{
		return a.squared_distance <= b.squared_distance;
	}

	friend constexpr bool operator>(NearestNode const& a, NearestNode const& b)
	{
		return a.squared_distance > b.squared_distance;
	}

	friend constexpr bool operator>=(NearestNode const& a, NearestNode const& b)
	{
		return a.squared_distance >= b.squared_distance;
	}
};

template <class Tree, class Geometry = geometry::Point,
          class Predicates = predicate::TRUE, class SquaredDistance = void,
          typename =
              std::enable_if_t<std::is_void_v<SquaredDistance> ||
                               std::is_invocable_r_v<double, SquaredDistance, NodeBV>>>
class NearestIterator : public IteratorBase<Tree, NearestNode>
{
 private:
	static constexpr bool OnlyLeaves =
	    predicate::contains_always_predicate_v<predicate::Leaf, Predicates>;

	using Base = IteratorBase<Tree, NearestNode>;

 public:
	// Tags
	using typename Base::const_pointer;
	using typename Base::const_reference;
	using typename Base::difference_type;
	using typename Base::iterator_category;
	using typename Base::pointer;
	using typename Base::reference;
	using typename Base::value_type;

	NearestIterator() {}

	NearestIterator(NodeBV const& root, Geometry const& geometry,
	                Predicates const& predicates)
	    : predicates_(predicates), geometry_(geometry)
	{
	}

	NearestIterator(Tree const* tree, NodeBV const& root, Geometry const& geometry,
	                Predicates const& predicates, float epsilon = 0.0f)
	    : Base(tree), predicates_(predicates), geometry_(geometry), epsilon_(epsilon)
	{
		init(root);
	}

	// NearestIterator(Tree const* tree, NodeBV const& root, SquaredDistance const& sq_dist,
	//                 Predicates const& predicates, float epsilon)
	//     : Base(tree), predicates_(predicates), sq_dist_(sq_dist), epsilon_(epsilon)
	// {
	// 	init(root);
	// }

	NearestIterator& next() override
	{
		increment();
		return *this;
	}

	NearestIterator* copy() override { return new NearestIterator(*this); }

	// reference data() override { return return_nodes_.top(); }

	const_reference data() const override { return return_nodes_.top(); }

	bool equal(Base const& other) const override
	{
		return other.tree() == this->tree() && other.data() == data();
	}

 private:
	double squaredDistance(NodeBV const& node) const
	{
		if constexpr (std::is_void_v<SquaredDistance>) {
			return geometry::squaredDistance(node.getBoundingVolume(), geometry_);
		} else {
			return SquaredDistance(node);
		}
	}

	void init(NodeBV const& node)
	{
		if constexpr (OnlyLeaves) {
			if (this->validReturnNode(node, predicates_)) {
				return_nodes_.emplace(node, squaredDistance(node));
			} else if (this->validInnerNodeOnlyLeaves(node, predicates_)) {
				std::vector<value_type> container;
				container.reserve(256);
				return_nodes_ = std::priority_queue<value_type, std::vector<value_type>,
				                                    std::greater<value_type>>(
				    std::greater<value_type>(), std::move(container));
				inner_nodes_ = return_nodes_;

				inner_nodes_.emplace(node, squaredDistance(node) + epsilon_);
				increment();
			} else {
				this->tree_ = nullptr;
			}
		} else {
			bool valid_inner = this->validInnerNode(node, predicates_);
			bool valid_return = this->validReturnNode(node, predicates_);

			if (valid_inner || valid_return) {
				std::vector<value_type> container;
				container.reserve(256);
				return_nodes_ = std::priority_queue<value_type, std::vector<value_type>,
				                                    std::greater<value_type>>(
				    std::greater<value_type>(), std::move(container));
				inner_nodes_ = return_nodes_;

				float dist_sq = squaredDistance(node);

				if (valid_inner) {
					inner_nodes_.emplace(node, dist_sq + epsilon_);
				}
				if (valid_return) {
					return_nodes_.emplace(node, dist_sq);
				} else {
					increment();
				}
			} else {
				this->tree_ = nullptr;
			}
		}
	}

	void increment()
	{
		if (!return_nodes_.empty()) {
			return_nodes_.pop();
		}

		// Skip forward to next valid return node
		while (!inner_nodes_.empty()) {
			if (!return_nodes_.empty() && return_nodes_.top() <= inner_nodes_.top()) {
				return;
			}

			NodeBV current = this->child(inner_nodes_.top().node, 0);
			inner_nodes_.pop();

			for (unsigned int idx = 0; 8 != idx; ++idx) {
				current = this->sibling(current, idx);

				if constexpr (OnlyLeaves) {
					if (this->validInnerNodeOnlyLeaves(current, predicates_)) {
						inner_nodes_.emplace(current, squaredDistance(current) + epsilon_);
					} else if (this->validReturnNode(current, predicates_)) {
						return_nodes_.emplace(current, squaredDistance(current));
					}
				} else {
					// No need to add inner nodes that does not have children
					bool const valid_inner = this->validInnerNode(current, predicates_);
					bool const valid_return = this->validReturnNode(current, predicates_);

					if (valid_inner || valid_return) {
						float dist_sq = squaredDistance(current);
						if (valid_inner) {
							inner_nodes_.emplace(current, dist_sq + epsilon_);
						}
						if (valid_return) {
							return_nodes_.emplace(current, dist_sq);
						}
					}
				}
			}
		}

		// No valid return node left
		this->tree_ = nullptr;
	}

 private:
	// Predicates
	Predicates predicates_;

	// Geometry
	std::conditional_t<std::is_void_v<SquaredDistance>, Geometry, bool> geometry_;

	// Squared distance function
	std::conditional_t<std::is_void_v<SquaredDistance>, bool, SquaredDistance> sq_dist_;

	// Epsilon for approximate search
	float epsilon_;

	std::priority_queue<value_type, std::vector<value_type>, std::greater<value_type>>
	    inner_nodes_;
	std::priority_queue<value_type, std::vector<value_type>, std::greater<value_type>>
	    return_nodes_;
};
}  // namespace ufo::map

#endif  // UFO_MAP_ITERATOR_OCTREE_H
