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

#ifndef UFO_MAP_OCTREE_ITERATOR_H
#define UFO_MAP_OCTREE_ITERATOR_H

// UFO
#include <ufo/geometry/bounding_volume.h>
#include <ufo/geometry/minimum_distance.h>
#include <ufo/geometry/point.h>
#include <ufo/map/code.h>
#include <ufo/map/node.h>
#include <ufo/map/octree/octree_predicate.h>
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

	constexpr IteratorBase(Tree const* tree) : tree_(tree) {}

	virtual ~IteratorBase() {}

	virtual void next() = 0;

	virtual IteratorBase* copy() = 0;

	// virtual reference data() = 0;
	virtual const_reference data() const = 0;

	constexpr Tree const* tree() const { return tree_; }

	virtual bool equal(IteratorBase const& other) const = 0;

 protected:
	virtual std::size_t status() const = 0;

	template <class NodeType>
	[[nodiscard]] constexpr depth_t depth(NodeType const& node) const
	{
		return node.depth();
	}

	template <class NodeType>
	[[nodiscard]] constexpr Code code(NodeType const& node) const
	{
		return node.code();
	}

	template <class NodeType>
	[[nodiscard]] constexpr std::size_t index(NodeType const& node) const
	{
		return node.index();
	}

	template <class NodeType>
	[[nodiscard]] constexpr bool isReal(NodeType const& node) const
	{
		return node.isReal();
	}

	template <class NodeType>
	[[nodiscard]] constexpr bool isPureLeaf(NodeType const& node) const
	{
		return tree_->isPureLeaf(node);
	}

	template <class NodeType>
	[[nodiscard]] constexpr bool isParent(NodeType const& node) const
	{
		return tree_->isParent(node);
	}

	template <class NodeType>
	[[nodiscard]] constexpr NodeType child(NodeType const& node, index_t child_index) const
	{
		return tree_->nodeChild(node, child_index);
	}

	template <class NodeType>
	[[nodiscard]] constexpr NodeType sibling(NodeType const& node,
	                                         index_t sibling_index) const
	{
		return tree_->nodeSibling(node, sibling_index);
	}

	template <class NodeType, class Predicates>
	[[nodiscard]] constexpr bool validInnerNode(NodeType const& node,
	                                            Predicates const& predicates) const
	{
		return 0 != node.depth() &&
		       predicate::PredicateInnerCheck<Predicates>::apply(predicates, *tree_, node);
	}

	template <class NodeType, class Predicates>
	[[nodiscard]] constexpr bool validReturnNode(NodeType const& node,
	                                             Predicates const& predicates) const
	{
		return predicate::PredicateValueCheck<Predicates>::apply(predicates, *tree_, node);
	}

 protected:
	// The UFOMap
	Tree const* tree_;
};

template <class Tree, typename T>
class IteratorWrapper
{
 private:
	using Base = IteratorBase<Tree, T>;

 public:
	// Tags
	using typename Base::const_pointer;
	using typename Base::const_reference;
	using typename Base::difference_type;
	using typename Base::iterator_category;
	using typename Base::pointer;
	using typename Base::reference;
	using typename Base::value_type;

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

template <class BaseNodeType, bool EarlyStopping, class Tree, class NodeType = Node,
          class Predicates = predicate::TRUE>
class Iterator : public IteratorBase<Tree, BaseNodeType>
{
 private:
	static constexpr bool OnlyLeavesOrFixedDepth =
	    predicate::contains_always_predicate_v<predicate::PureLeaf, Predicates> ||
	    predicate::contains_always_predicate_v<predicate::DepthE, Predicates> ||
	    EarlyStopping;

	using Base = IteratorBase<Tree, BaseNodeType>;

	using Stack = std::stack<NodeType, std::vector<NodeType>>;
	using Queue = std::queue<NodeType, std::vector<NodeType>>;

 public:
	// Tags
	using typename Base::const_pointer;
	using typename Base::const_reference;
	using typename Base::difference_type;
	using typename Base::iterator_category;
	using typename Base::pointer;
	using typename Base::reference;
	using typename Base::value_type;

	Iterator(Tree const* tree) : Base(tree) {}

	Iterator(Tree const* tree, NodeType const& root, Predicates const& predicates)
	    : Base(tree), predicates_(predicates)
	{
		init(root);
	}

	void next() override
	{
		if (!return_nodes_.empty()) {
			return_nodes_.pop();
		}

		// Skip forward to next valid return node
		while (return_nodes_.empty() && !inner_nodes_.empty()) {
			auto current = this->child(inner_nodes_.top(), 0);
			inner_nodes_.pop();

			// Go down the tree
			for (std::size_t i = 0; 8 != i; ++i) {
				current = this->sibling(current, i);

				if constexpr (OnlyLeavesOrFixedDepth) {
					if (this->validReturnNode(current, predicates_)) {
						return_nodes_.push(current);
					} else if (this->validInnerNode(current, predicates_)) {
						inner_nodes_.push(current);
					}
				} else {
					if (this->validReturnNode(current, predicates_)) {
						return_nodes_.push(current);
					}
					if (this->validInnerNode(current, predicates_)) {
						inner_nodes_.push(current);
					}
				}
			}
		}
	}

	Iterator* copy() override { return new Iterator(*this); }

	// reference data() override { return node_; }

	const_reference data() const override { return return_nodes_.front(); }

	bool equal(Base const& other) const override
	{
		return other.tree() == this->tree() && other.status() == status() &&
		       (!return_nodes_.empty() && other.data() == data());
	}

	std::size_t status() const override
	{
		return inner_nodes_.size() + return_nodes_.size();
	}

 private:
	void init(NodeType const& node)
	{
		if constexpr (OnlyLeavesOrFixedDepth) {
			if (this->validReturnNode(node, predicates_)) {
				return_nodes_.push(node);
			} else if (this->validInnerNode(node, predicates_)) {
				inner_nodes_.push(node);
				next();
			}
		} else {
			if (this->validReturnNode(node, predicates_)) {
				return_nodes_.push(node);
				if (this->validInnerNode(node, predicates_)) {
					inner_nodes_.push(node);
				}
			} else if (this->validInnerNode(node, predicates_)) {
				inner_nodes_.push(node);
				next();
			}
		}
	}

 private:
	Predicates const predicates_;  // Predicates that nodes has to fulfill
	Stack inner_nodes_;            // To be processed inner nodes
	Queue return_nodes_;           // To be processed return nodes
};

struct NearestNode {
	NodeBV const node;
	float const squared_distance;

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

template <bool EarlyStopping, class Tree, class Geometry = geometry::Point,
          class Predicates = predicate::TRUE>
class NearestIterator : public IteratorBase<Tree, NearestNode>
{
 private:
	static constexpr bool OnlyLeavesOrFixedDepth =
	    predicate::contains_always_predicate_v<predicate::PureLeaf, Predicates> ||
	    predicate::contains_always_predicate_v<predicate::DepthE, Predicates> ||
	    EarlyStopping;

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

 private:
	using Queue =
	    std::priority_queue<value_type, std::vector<value_type>, std::greater<value_type>>;

 public:
	NearestIterator(Tree const* tree) : Base(tree) {}

	NearestIterator(Tree const* tree, NodeBV const& root, Geometry const& geometry,
	                Predicates const& predicates, float epsilon = 0.0f)
	    : Base(tree), predicates_(predicates), geometry_(geometry), epsilon_(epsilon)
	{
		init(root);
	}

	void next() override
	{
		if (!return_nodes_.empty()) {
			return_nodes_.pop();
		}

		// Skip forward to next valid return node
		while (!inner_nodes_.empty()) {
			if (!return_nodes_.empty() && return_nodes_.top() <= inner_nodes_.top()) {
				return;
			}

			auto current = this->child(inner_nodes_.top().node, 0);
			inner_nodes_.pop();

			for (unsigned int idx = 0; 8 != idx; ++idx) {
				current = this->sibling(current, idx);

				if constexpr (OnlyLeavesOrFixedDepth) {
					if (this->validReturnNode(current, predicates_)) {
						return_nodes_.emplace(current, squaredDistance(current));
					} else if (this->validInnerNode(current, predicates_)) {
						inner_nodes_.emplace(current, squaredDistance(current) + epsilon_);
					}
				} else {
					if (this->validReturnNode(current, predicates_)) {
						auto dist_sq = squaredDistance(current);
						return_nodes_.emplace(current, dist_sq);
						if (this->validInnerNode(current, predicates_)) {
							inner_nodes_.emplace(current, dist_sq + epsilon_);
						}
					} else if (this->validInnerNode(current, predicates_)) {
						inner_nodes_.emplace(current, squaredDistance(current) + epsilon_);
					}
				}
			}
		}
	}

	NearestIterator* copy() override { return new NearestIterator(*this); }

	// reference data() override { return return_nodes_.top(); }

	const_reference data() const override { return return_nodes_.top(); }

	bool equal(Base const& other) const override
	{
		return other.tree() == this->tree() && other.status() == status() &&
		       (!return_nodes_.empty() && other.data() == data());
	}

	std::size_t status() const override
	{
		return inner_nodes_.size() + return_nodes_.size();
	}

 private:
	float squaredDistance(NodeBV const& node) const
	{
		return geometry::squaredDistance(node.boundingVolume(), geometry_);
	}

	void init(NodeBV const& node)
	{
		if constexpr (OnlyLeavesOrFixedDepth) {
			if (this->validReturnNode(node, predicates_)) {
				return_nodes_.emplace(node, squaredDistance(node));
			} else if (this->validInnerNode(node, predicates_)) {
				std::vector<value_type> container;
				container.reserve(256);
				return_nodes_ = std::priority_queue<value_type, std::vector<value_type>,
				                                    std::greater<value_type>>(
				    std::greater<value_type>(), std::move(container));
				inner_nodes_ = return_nodes_;

				inner_nodes_.emplace(node, squaredDistance(node) + epsilon_);
				next();
			}
		} else {
			if (this->validReturnNode(node, predicates_)) {
				std::vector<value_type> container;
				container.reserve(256);
				return_nodes_ = std::priority_queue<value_type, std::vector<value_type>,
				                                    std::greater<value_type>>(
				    std::greater<value_type>(), std::move(container));
				inner_nodes_ = return_nodes_;

				auto const dist_sq = squaredDistance(node);

				return_nodes_.emplace(node, dist_sq);
				if (this->validInnerNode(node, predicates_)) {
					inner_nodes_.emplace(node, dist_sq + epsilon_);
				}
			} else if (this->validInnerNode(node, predicates_)) {
				std::vector<value_type> container;
				container.reserve(256);
				return_nodes_ = std::priority_queue<value_type, std::vector<value_type>,
				                                    std::greater<value_type>>(
				    std::greater<value_type>(), std::move(container));
				inner_nodes_ = return_nodes_;

				inner_nodes_.emplace(node, squaredDistance(node) + epsilon_);
				next();
			}
		}
	}

 private:
	Predicates const predicates_;  // Predicates that nodes has to fulfill
	Geometry const geometry_;      // Geometry to find nearest to
	float const epsilon_;          // Epsilon for approximate search
	Queue inner_nodes_;            // To be processed inner nodes
	Queue return_nodes_;           // To be processed return nodes
};
}  // namespace ufo::map

#endif  // UFO_MAP_OCTREE_ITERATOR_H
