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

#ifndef UFO_MAP_ITERATOR_OCTREE_H
#define UFO_MAP_ITERATOR_OCTREE_H

// UFO
#include <ufo/geometry/bounding_volume.h>
#include <ufo/geometry/minimum_distance.h>
#include <ufo/map/code.h>
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
	using pointer = value_type*;                // or also value_type*
	using reference = value_type&;              // or also value_type&
	using const_pointer = value_type const*;    // or also value_type*
	using const_reference = value_type const&;  // or also value_type&

	IteratorBase() : tree_(nullptr) {}

	IteratorBase(Tree const* tree) : tree_(tree) {}

	virtual IteratorBase& next() = 0;

	virtual IteratorBase* getCopy() = 0;

	// virtual reference getData() = 0;
	virtual const_reference getData() const = 0;

	Tree const* getTree() const { return tree_; }

	virtual bool equal(IteratorBase const& other) const = 0;

 protected:
	DepthType getDepth(Node const& node) const { return node.getDepth(); }

	Code getCode(Node const& node) const { return node.getCode(); }

	size_t getIdx(Node const& node) const
	{
		return getCode(node).getChildIdx(getDepth(node));
	}

	bool hasChildren(Node const& node) const { return tree_->hasChildren(node); }

	bool hasParent(Node const& node) const { return tree_->hasParent(node); }

	Node getChild(Node const& node, unsigned int idx) const
	{
		return tree_->getChild(node, idx);
	}

	Node getSibling(Node const& node, unsigned int idx) const
	{
		return tree_->getSibling(node, idx);
	}

	template <class Predicates>
	bool validInnerNode(Node const& node, Predicates const& predicates) const
	{
		return hasChildren(node) && validInnerNodeOnlyLeaves(node, predicates);
	}

	template <class Predicates>
	bool validInnerNodeOnlyLeaves(Node const& node, Predicates const& predicates) const
	{
		return predicate::PredicateInnerCheck<Predicates>::apply(predicates, *tree_, node);
	}

	template <class Predicates>
	bool validReturnNode(Node const& node, Predicates const& predicates) const
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
	using iterator_category = typename Base::iterator_category;
	using difference_type = typename Base::difference_type;
	using value_type = typename Base::value_type;
	using pointer = typename Base::pointer;
	using reference = typename Base::reference;
	using const_pointer = typename Base::const_pointer;
	using const_reference = typename Base::const_reference;

	IteratorWrapper(Base* it_base) : it_base_(it_base) {}

	IteratorWrapper(IteratorWrapper const& other) : it_base_(other.it_base_->getCopy()) {}

	IteratorWrapper& operator++()
	{
		it_base_->next();
		return *this;
	}

	IteratorWrapper operator++(int)
	{
		IteratorWrapper result(it_base_->getCopy());
		++(*this);
		return result;
	}

	// pointer operator->() { return &(it_base_->getData()); }

	// reference operator*() { return it_base_->getData(); }

	const_pointer operator->() const { return &(it_base_->getData()); }

	const_reference operator*() const { return it_base_->getData(); }

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

template <class Tree, class Predicates = predicate::TRUE>
class Iterator : public IteratorBase<Tree, Node>
{
 private:
	static constexpr bool OnlyLeaves =
	    predicate::contains_predicate<predicate::Leaf, Predicates>();

	using Base = IteratorBase<Tree, Node>;

 public:
	// Tags
	using iterator_category = typename Base::iterator_category;
	using difference_type = typename Base::difference_type;
	using value_type = typename Base::value_type;
	using pointer = typename Base::pointer;
	using reference = typename Base::reference;
	using const_pointer = typename Base::const_pointer;
	using const_reference = typename Base::const_reference;

	Iterator(Node const& root)
	    : node_(root), predicates_(predicate::TRUE()), valid_inner_(false)
	{
	}

	Iterator(Node const& root, Predicates const& predicates)
	    : node_(root), predicates_(predicates), valid_inner_(false)
	{
	}

	Iterator(Tree const* tree, Node const& root, Predicates const& predicates)
	    : Base(tree), node_(root), predicates_(predicates), valid_inner_(false)
	{
		init();
	}

	Iterator& next() override
	{
		increment();
		return *this;
	}

	Iterator* getCopy() override { return new Iterator(*this); }

	// reference getData() override { return node_; }

	const_reference getData() const override { return node_; }

	bool equal(Base const& other) const
	{
		return other.getTree() == this->getTree() && other.getData() == getData();
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
				Node current = this->getChild(node_, 0);
				for (unsigned int idx = 0; idx != 8; ++idx) {
					current = this->getSibling(current, idx);

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
				Node current = this->getChild(node_, 0);
				
				for (unsigned int idx = 0; idx != 8; ++idx) {
					current = this->getSibling(current, idx);

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
		while (this->getDepth(node_) != this->tree_->getTreeDepthLevels()) {
			for (auto idx = this->getIdx(node_) + 1; 8 != idx; ++idx) {
				node_ = this->getSibling(node_, idx);

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
	Node node_;

	// Parents
	std::stack<Node, std::vector<Node>> parents_;
};

struct NearestNode {
	NearestNode(Node const& node, double squared_distance)
	    : node(node), squared_distance(squared_distance)
	{
	}

	friend bool operator<(NearestNode const& v1, NearestNode const v2)
	{
		return v1.squared_distance < v2.squared_distance;
	}

	friend bool operator<=(NearestNode const& v1, NearestNode const v2)
	{
		return v1.squared_distance <= v2.squared_distance;
	}

	friend bool operator>(NearestNode const& v1, NearestNode const v2)
	{
		return v1.squared_distance > v2.squared_distance;
	}

	friend bool operator>=(NearestNode const& v1, NearestNode const v2)
	{
		return v1.squared_distance >= v2.squared_distance;
	}

	Node node;
	double squared_distance;
};

template <class Tree, class Geometry = math::Vector3, class Predicates = predicate::TRUE>
class NearestIterator : public IteratorBase<Tree, NearestNode>
{
 private:
	static constexpr bool OnlyLeaves =
	    predicate::contains_predicate<predicate::Leaf, Predicates>();

	using Base = IteratorBase<Tree, NearestNode>;

 public:
	// Tags
	using iterator_category = typename Base::iterator_category;
	using difference_type = typename Base::difference_type;
	using value_type = typename Base::value_type;
	using pointer = typename Base::pointer;
	using reference = typename Base::reference;
	using const_pointer = typename Base::const_pointer;
	using const_reference = typename Base::const_reference;

	NearestIterator() {}

	NearestIterator(Node const& root, Geometry const& geometry,
	                Predicates const& predicates)
	    : predicates_(predicates), geometry_(geometry)
	{
	}

	NearestIterator(Tree const* tree, Node const& root, Geometry const& geometry,
	                Predicates const& predicates, float epsilon = 0.0f)
	    : Base(tree), predicates_(predicates), geometry_(geometry), epsilon_(epsilon)
	{
		init(root);
	}

	NearestIterator& next() override
	{
		increment();
		return *this;
	}

	NearestIterator* getCopy() override { return new NearestIterator(*this); }

	// reference getData() override { return return_nodes_.top(); }

	const_reference getData() const override { return return_nodes_.top(); }

	bool equal(Base const& other) const
	{
		return other.getTree() == this->getTree() && other.getData() == getData();
	}

 private:
	double distanceSquared(Node const& node) const
	{
		return geometry::minDistanceSquared(node.getBoundingVolume(), geometry_);
	}

	void init(Node const& node)
	{
		if constexpr (OnlyLeaves) {
			if (this->validReturnNode(node, predicates_)) {
				return_nodes_.emplace(node, distanceSquared(node));
			} else if (this->validInnerNodeOnlyLeaves(node, predicates_)) {
				std::vector<value_type> container;
				container.reserve(256);
				return_nodes_ = std::priority_queue<value_type, std::vector<value_type>,
				                                    std::greater<value_type>>(
				    std::greater<value_type>(), std::move(container));
				inner_nodes_ = return_nodes_;

				inner_nodes_.emplace(node, distanceSquared(node) + epsilon_);
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

				float dist_sq = distanceSquared(node);

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

			Node current = this->getChild(inner_nodes_.top().node, 0);
			inner_nodes_.pop();

			for (unsigned int idx = 0; 8 != idx; ++idx) {
				current = this->getSibling(current, idx);

				if constexpr (OnlyLeaves) {
					if (this->validInnerNodeOnlyLeaves(current, predicates_)) {
						inner_nodes_.emplace(current, distanceSquared(current) + epsilon_);
					} else if (this->validReturnNode(current, predicates_)) {
						return_nodes_.emplace(current, distanceSquared(current));
					}
				} else {
					// No need to add inner nodes that does not have children
					bool const valid_inner = this->validInnerNode(current, predicates_);
					bool const valid_return = this->validReturnNode(current, predicates_);

					if (valid_inner || valid_return) {
						float dist_sq = distanceSquared(current);
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
	Geometry geometry_;

	// Epsilon for approximate search
	float epsilon_;

	std::priority_queue<value_type, std::vector<value_type>, std::greater<value_type>>
	    inner_nodes_;
	std::priority_queue<value_type, std::vector<value_type>, std::greater<value_type>>
	    return_nodes_;
};
}  // namespace ufo::map

#endif  // UFO_MAP_ITERATOR_OCTREE_H
