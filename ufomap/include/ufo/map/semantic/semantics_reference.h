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

#ifndef UFO_MAP_SEMANTICS_REFERENCE_H
#define UFO_MAP_SEMANTICS_REFERENCE_H

// UFO
#include <ufo/map/semantic/semantics.h>
#include <ufo/map/types.h>

// STL

namespace ufo::map
{
class SemanticsReference
{
 public:
	//
	// Tags
	//

	using size_type = typename SemanticNode<8>::size_type;
	using difference_type = typename SemanticNode<8>::difference_type;
	using reference = typename SemanticNode<8>::reference;
	using const_reference = typename SemanticNode<8>::const_reference;
	using pointer = typename SemanticNode<8>::pointer;
	using const_pointer = typename SemanticNode<8>::const_pointer;
	using iterator = typename SemanticNode<8>::iterator;
	using const_iterator = typename SemanticNode<8>::const_iterator;
	using reverse_iterator = typename SemanticNode<8>::reverse_iterator;
	using const_reverse_iterator = typename SemanticNode<8>::const_reverse_iterator;

	//
	// Constructor
	//

	constexpr SemanticsReference() = default;

	constexpr SemanticsReference(SemanticsReference const&) = default;

	constexpr SemanticsReference(SemanticsReference&&) = default;

	//
	// Assignment operator
	//

	SemanticsReference& operator=(SemanticsReference const&) = default;

	SemanticsReference& operator=(SemanticsReference&&) = default;

	//
	// Conversion
	//

	operator Semantics() const
	{
		// TODO: Implement
	}

	//
	// Data
	//

	[[nodiscard]] Semantic const* data() const { return semantics_->data(index_); }

	//
	// Iterators
	//

	[[nodiscard]] const_iterator begin() const { return semantics_->begin(index_); }

	[[nodiscard]] const_iterator cbegin() const { return semantics_->cbegin(index_); }

	[[nodiscard]] const_reverse_iterator rbegin() const
	{
		return semantics_->rbegin(index_);
	}

	[[nodiscard]] const_reverse_iterator crbegin() const
	{
		return semantics_->crbegin(index_);
	}

	[[nodiscard]] const_iterator end() const { return semantics_->end(index_); }

	[[nodiscard]] const_iterator cend() const { return semantics_->cend(index_); }

	[[nodiscard]] const_reverse_iterator rend() const { return semantics_->rend(index_); }

	[[nodiscard]] const_reverse_iterator crend() const { return semantics_->crend(index_); }

	//
	// Empty
	//

	[[nodiscard]] bool empty() const { return semantics_->empty(index_); }

	//
	// Size
	//

	[[nodiscard]] size_type size() const { return semantics_->size(index_); }

	[[nodiscard]] size_type maxSize() const { return semantics_->maxSize(index_); }

	//
	// At
	//

	[[nodiscard]] std::optional<Semantic> at(label_t label) const
	{
		return semantics_->at(index_, label);
	}

	//
	// Value
	//

	[[nodiscard]] std::optional<value_t> value(label_t label) const
	{
		return semantics_->value(index_, label);
	}

	//
	// Count
	//

	[[nodiscard]] size_type count(label_t label) const
	{
		return semantics_->count(index_, label);
	}

	//
	// Find
	//

	[[nodiscard]] const_iterator find(label_t label) const
	{
		return semantics_->find(index_, label);
	}

	//
	// Contains
	//

	[[nodiscard]] bool contains(label_t label) const
	{
		return semantics_->contains(index_, label);
	}

	//
	// Equal range
	//

	[[nodiscard]] std::pair<const_iterator, const_iterator> equal_range(label_t label) const
	{
		return semantics_->equal_range(index_, label);
	}

	//
	// Lower bound
	//

	[[nodiscard]] const_iterator lower_bound(label_t label) const
	{
		return semantics_->lower_bound(index_, label);
	}

	//
	// Upper bound
	//

	[[nodiscard]] const_iterator upper_bound(label_t label) const
	{
		return semantics_->upper_bound(index_, label);
	}

	//
	// All
	//

	[[nodiscard]] bool all(SemanticRange range) const
	{
		return semantics_->all(index_, range);
	}

	[[nodiscard]] bool all(SemanticRangeSet const& ranges) const
	{
		return semantics_->all(index_, ranges);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool all(UnaryPredicate p) const
	{
		return semantics_->all(index_, p);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool all(SemanticRange range, UnaryPredicate p) const
	{
		return semantics_->all(index_, range, p);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool all(SemanticRangeSet const& ranges, UnaryPredicate p) const
	{
		return semantics_->all(index_, ranges, p);
	}

	//
	// Any
	//

	[[nodiscard]] bool any(SemanticRange range) const
	{
		return semantics_->any(index_, range);
	}

	[[nodiscard]] bool any(SemanticRangeSet const& ranges) const
	{
		return semantics_->any(index_, ranges);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool any(UnaryPredicate p) const
	{
		return semantics_->any(index_, p);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool any(SemanticRange range, UnaryPredicate p) const
	{
		return semantics_->any(index_, range, p);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool any(SemanticRangeSet const& ranges, UnaryPredicate p) const
	{
		return semantics_->any(index_, ranges, p);
	}

	//
	// None
	//

	[[nodiscard]] bool none(SemanticRange range) const
	{
		return semantics_->none(index_, range);
	}

	[[nodiscard]] bool none(SemanticRangeSet const& ranges) const
	{
		return semantics_->none(index_, ranges);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool none(UnaryPredicate p) const
	{
		return semantics_->none(index_, p);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool none(SemanticRange range, UnaryPredicate p) const
	{
		return semantics_->none(index_, range, p);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool none(SemanticRangeSet const& ranges, UnaryPredicate p) const
	{
		return semantics_->none(index_, ranges, p);
	}

 private:
	//
	// Constructor
	//

	SemanticsReference(SemanticMapping const* mapping, SemanticNode<8> const* semantics,
	                   index_t const index)
	    : mapping_(mapping), semantics_(semantics), index_(index)
	{
	}

	SemanticsReference(SemanticMapping const& mapping, SemanticNode<8> const& semantics,
	                   index_t const index)
	    : mapping_(&mapping), semantics_(&semantics), index_(index)
	{
	}

 private:
	SemanticMapping const* mapping_ = nullptr;
	SemanticNode<8> const* semantics_ = nullptr;
	index_t index_ = 0;
};
}  // namespace ufo::map

#endif  // UFO_MAP_SEMANTICS_REFERENCE_H