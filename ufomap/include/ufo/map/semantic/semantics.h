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

#ifndef UFO_MAP_SEMANTICS_H
#define UFO_MAP_SEMANTICS_H

// UFO
#include <ufo/container/range.h>
#include <ufo/container/range_map.h>
#include <ufo/container/ranges.h>
#include <ufo/util/iterator_wrapper.h>
#include <ufo/util/type_traits.h>

// STL
#include <algorithm>
#include <cstddef>  // For std::ptrdiff_t
#include <functional>
#include <initializer_list>
#include <istream>
#include <iterator>  // For std::random_access_iterator_tag / std::contiguous_iterator_tag
#include <memory>
#include <ostream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace ufo::map
{
// Forward declare
template <std::size_t N>
class SemanticNode;

class Semantics
{
 public:
	//  Tags
	using size_type = std::size_t;
	using difference_type = std::ptrdiff_t;
	using reference = Semantic &;  // TODO: Make label const
	using const_reference = Semantic const &;
	using pointer = Semantic *;
	using const_pointer = Semantic const *;
	using iterator = Semantic *;  // TODO: Make label const
	using const_iterator = Semantic const *;
	using reverse_iterator = std::reverse_iterator<iterator>;
	using const_reverse_iterator = std::reverse_iterator<const_iterator>;

	//
	// Constructors
	//

	constexpr Semantics() = default;

	Semantics(Semantics const &other) { *this = other; }

	Semantics(Semantics &&other) noexcept = default;

	Semantics(SemanticsReference other)
	{
		resize(other.size());
		std::copy(std::begin(other), std::end(other), begin());
	}

	template <class InputIt>
	Semantics(InputIt first, InputIt last)
	{
		// TODO: Implement
	}

	Semantics(std::initializer_list<Semantic> init)
	    : Semantics(std::begin(init), std::end(init))
	{
	}

	//
	// Assignment operator
	//

	Semantics &operator=(Semantics const &rhs)
	{
		if (rhs.empty()) {
			clear();
		} else {
			resize(rhs.size());
			std::copy(std::begin(rhs), std::end(rhs), begin());
		}
		return *this;
	}

	Semantics &operator=(Semantics &&rhs) noexcept = default;

	Semantics &operator=(SemanticsReference rhs)
	{
		resize(rhs.size());
		std::copy(std::begin(rhs), std::end(rhs), begin());
	}

	//
	// Data
	//

	[[nodiscard]] const_pointer data() const { return empty() ? nullptr : data_.get() + 1; }

	//
	// Iterators
	//

	iterator begin() noexcept { return empty() ? nullptr : data_.get() + 1; }

	const_iterator begin() const noexcept { return empty() ? nullptr : data_.get() + 1; }

	const_iterator cbegin() const noexcept { return begin(); }

	iterator end() noexcept { return empty() ? nullptr : data_.get() + allocSize(); }

	const_iterator end() const noexcept
	{
		return empty() ? nullptr : data_.get() + allocSize();
	}

	const_iterator cend() const noexcept { return end(); }

	//
	// Reverse iterators
	//

	reverse_iterator rbegin() noexcept { return std::make_reverse_iterator(end()); }

	const_reverse_iterator rbegin() const noexcept
	{
		return std::make_reverse_iterator(end());
	}

	const_reverse_iterator crbegin() const noexcept { return rbegin(); }

	reverse_iterator rend() noexcept { return std::make_reverse_iterator(begin()); }

	const_reverse_iterator rend() const noexcept
	{
		return std::make_reverse_iterator(begin());
	}

	const_reverse_iterator crend() const noexcept { return rend(); }

	//
	// Empty
	//

	[[nodiscard]] bool empty() const noexcept { return nullptr == data_; }

	//
	// Size
	//

	[[nodiscard]] size_type size() const { return empty() ? 0 : data_[0].label; }

	[[nodiscard]] static constexpr size_type maxSize() noexcept
	{
		return std::numeric_limits<label_t>::max();
	}

	//
	// At
	//

	[[nodiscard]] std::optional<Semantic> at(label_t label) const
	{
		auto it = find(label);
		return end() != it ? std::optional<Semantic>(*it) : std::nullopt;
	}

	//
	// Value
	//

	[[nodiscard]] std::optional<value_t> value(label_t label) const
	{
		auto it = find(label);
		return end() != it ? std::optional<value_t>(it->value) : std::nullopt;
	}

	//
	// Count
	//

	[[nodiscard]] size_type count(label_t label) const { return contains(label) ? 1 : 0; }

	//
	// Find
	//

	[[nodiscard]] iterator find(label_t label)
	{
		auto it = lower_bound(label);
		return end() != it && it->label == label ? it : end();
	}

	[[nodiscard]] const_iterator find(label_t label) const
	{
		auto it = lower_bound(label);
		return end() != it && it->label == label ? it : end();
	}

	//
	// Contains
	//

	[[nodiscard]] bool contains(label_t label) const { return end() != find(label); }

	//
	// Equal range
	//

	[[nodiscard]] std::pair<iterator, iterator> equal_range(label_t label)
	{
		return equal_range(begin(), end(), label);
	}

	[[nodiscard]] std::pair<const_iterator, const_iterator> equal_range(label_t label) const
	{
		return equal_range(begin(), end(), label);
	}

	//
	// Lower bound
	//

	[[nodiscard]] iterator lower_bound(label_t label)
	{
		return lower_bound(begin(), end(), label);
	}

	[[nodiscard]] const_iterator lower_bound(label_t label) const
	{
		return lower_bound(begin(), end(), label);
	}

	//
	// Upper bound
	//

	[[nodiscard]] iterator upper_bound(label_t label)
	{
		return upper_bound(begin(), end(), label);
	}

	[[nodiscard]] const_iterator upper_bound(label_t label) const
	{
		return upper_bound(begin(), end(), label);
	}

	//
	// All
	//

	[[nodiscard]] bool all(SemanticRange range) const
	{
		return all(SemanticRangeSet(range));
	}

	[[nodiscard]] bool all(SemanticRangeSet const &ranges) const
	{
		if (ranges.empty()) {
			return true;
		} else if (size() < ranges.numValues()) {
			return false;
		}

		auto first = cbegin();
		auto last = cend();
		for (auto range : ranges) {
			auto lower = lower_bound(first, last, range.lower());
			first = upper_bound(lower, last, range.upper());
			auto range_dist = range.upper() - range.lower() + 1;
			auto sem_dist = std::distance(lower, first);
			if (first == last || range_dist != sem_dist) {
				return false;
			}
		}
		return true;
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool all(UnaryPredicate p) const
	{
		return std::all_of(cbegin(), cend(), p);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool all(SemanticRange range, UnaryPredicate p) const
	{
		return all(SemanticRangeSet(range), p);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool all(SemanticRangeSet const &ranges, UnaryPredicate p) const
	{
		if (ranges.empty()) {
			return true;
		} else if (size() < ranges.numValues()) {
			return false;
		}

		auto first = cbegin();
		auto last = cend();
		for (auto range : ranges) {
			auto lower = lower_bound(first, last, range.lower());
			first = upper_bound(lower, last, range.upper());
			auto range_dist = range.upper() - range.lower() + 1;
			auto sem_dist = std::distance(lower, first);
			if (first == last || range_dist != sem_dist) {
				return false;
			}
			for (; lower != first; ++lower) {
				if (!p(*lower)) {
					return false;
				}
			}
		}
		return true;
	}

	//
	// Any
	//

	[[nodiscard]] bool any(SemanticRange range) const
	{
		return any(SemanticRangeSet(range));
	}

	[[nodiscard]] bool any(SemanticRangeSet const &ranges) const
	{
		if (ranges.empty()) {
			return true;
		}

		if (size() < ranges.size()) {
			for (auto it = cbegin(), last = cend(); it != last; ++it) {
				if (ranges.contains(it->label)) {
					return true;
				}
			}
		} else {
			auto first = cbegin();
			auto last = cend();
			for (auto range : ranges) {
				first = lower_bound(first, last, range.lower());
				if (first == last) {
					return false;
				} else if (first->label <= range.upper()) {
					return true;
				}
			}
		}
		return false;
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool any(UnaryPredicate p) const
	{
		return std::any_of(cbegin(), cend(), p);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool any(SemanticRange range, UnaryPredicate p) const
	{
		return any(SemanticRangeSet(range), p);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool any(SemanticRangeSet const &ranges, UnaryPredicate p) const
	{
		if (ranges.empty()) {
			return true;
		}

		if (size() < ranges.size()) {
			for (auto it = cbegin(), last = cend(); it != last; ++it) {
				if (ranges.contains(it->label) && f(*it)) {
					return true;
				}
			}
		} else {
			auto first = cbegin();
			auto last = cend();
			for (auto range : ranges) {
				first = lower_bound(first, last, range.lower());
				for (; first != last && first->label <= range.upper(); ++first) {
					if (p(*first)) {
						return true;
					}
				}
				if (first == last) {
					return false;
				}
			}
		}
	}

	//
	// None
	//

	[[nodiscard]] bool none(SemanticRange range) const
	{
		return return none(SemanticRangeSet(range));
	}

	[[nodiscard]] bool none(SemanticRangeSet const &ranges) const { return !any(ranges); }

	template <class UnaryPredicate>
	[[nodiscard]] bool none(UnaryPredicate p) const
	{
		return std::none_of(cbegin(), cend(), p);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool none(SemanticRange range, UnaryPredicate p) const
	{
		return none(SemanticRangeSet(range), p);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool none(SemanticRangeSet const &ranges, UnaryPredicate p) const
	{
		return !any(ranges, p);
	}

	//
	// Clear
	//

	void clear() noexcept { data_.reset(); }

	//
	// Insert
	//

	std::pair<iterator, bool> insert(Semantic semantic)
	{
		return insert(semantic.label, semantic.value);
	}

	std::pair<iterator, bool> insert(label_t label, value_t value)
	{
		return insert<false>(label, value);
	}

	iterator insert(const_iterator hint, Semantic semantic)
	{
		return insert(hint, semantic.label, semantic.value);
	}

	iterator insert(const_iterator hint, label_t label, value_t value)
	{
		return insert<false>(hint, label, value);
	}

	template <class InputIt>
	void insert(InputIt first, InputIt last)
	{
		insert<false>(first, last);
	}

	void insert(std::initializer_list<Semantic> ilist)
	{
		insert(std::cbegin(ilist), std::cend(ilist));
	}

	//
	// Insert or assign
	//

	std::pair<iterator, bool> insertOrAssign(Semantic semantic)
	{
		return insertOrAssign(semantic.label, semantic.value);
	}

	std::pair<iterator, bool> insertOrAssign(label_t label, value_t value)
	{
		return insert<true>(label, value);
	}

	iterator insertOrAssign(const_iterator hint, Semantic semantic)
	{
		return insertOrAssign(hint, semantic.label, semantic.value);
	}

	iterator insertOrAssign(const_iterator hint, label_t label, value_t value)
	{
		return insert<true>(hint, label, value);
	}

	template <class InputIt>
	void insertOrAssign(InputIt first, InputIt last)
	{
		insert<true>(first, last);
	}

	void insertOrAssign(std::initializer_list<Semantic> ilist)
	{
		insertOrAssign(std::cbegin(ilist), std::cend(ilist));
	}

	//
	// Insert or assign custom function
	//

	template <class UnaryFunction>
	void insertOrAssign(label_t label, UnaryFunction f)
	{
		insert<true>(label, f);
	}

	template <class InputIt, class UnaryFunction>
	void insertOrAssign(InputIt first, InputIt last, UnaryFunction f)
	{
		insert<true>(first, last, f);
	}

	template <class UnaryFunction>
	void insertOrAssign(std::initializer_list<label_t> ilist, UnaryFunction f)
	{
		insertOrAssign(std::cbegin(ilist), std::cend(ilist), f);
	}

	//
	// Assign
	//

	void assign(SemanticRange range, value_t value)
	{
		assign(SemanticRangeSet(range), value);
	}

	void assign(SemanticRangeSet const &ranges, value_t value)
	{
		assign(ranges, [value](auto) { return value; });
	}

	template <class UnaryPredicate>
	void assign(UnaryPredicate p, value_t value)
	{
		assign(p, [value](auto) { return value; });
	}

	template <class UnaryFunction>
	void assign(SemanticRange range, UnaryFunction f)
	{
		assign(SemanticRangeSet(range), f);
	}

	template <class UnaryFunction>
	void assign(SemanticRangeSet const &ranges, UnaryFunction f)
	{
		auto first = begin();
		auto last = end();
		for (auto range : ranges) {
			if (first == last) {
				break;
			}

			first = lower_bound(first, last, range.lower());
			auto upper = upper_bound(first, last, range.upper());
			for (; first != upper; ++first) {
				first->value = f(*first);
			}
		}
	}

	template <class UnaryPredicate, UnaryFunction>
	void assign(UnaryPredicate p, UnaryFunction f)
	{
		for (auto first = begin(), last = end(); first != last; ++first) {
			if (p(*first)) {
				first->value = f(*first);
			}
		}
	}

	//
	// Erase
	//

	iterator erase(const_iterator pos) { return erase(pos, std::next(pos)); }

	iterator erase(iterator pos) { return erase(pos, std::next(pos)); }

	iterator erase(const_iterator first, const_iterator last)
	{
		if (first == last || cend() == first) {
			return end();
		} else if (cbegin() == first && cend() == last) {
			clear();
			return end();
		} else if (cend() == last) {
			resize(size() - std::distance(first, last));
			return end();
		}

		// TODO: Implement

		resize(...);
	}

	size_type erase(label_t label)
	{
		auto p = find(label);
		return p == end() ? 0 : erase(p), 1;
	}

	size_type erase(SemanticRange range) { return erase(SemanticRangeSet{range}); }

	size_type erase(SemanticRangeSet const &ranges)
	{
		if (ranges.empty() || empty()) {
			return 0;
		}

		auto first = begin();
		auto last = end();
		size_type sum = 0;
		for (auto range : ranges) {
			first = lower_bound(first, last, range.lower());
			auto upper = upper_bound(first, last, range.upper());
			sum += std::distance(first, upper);
			erase(first, upper);
			first = upper;
		}

		return sum;
	}

	//
	// Erase if
	//

	template <class UnaryPredicate>
	size_type eraseIf(UnaryPredicate p)
	{
		if (empty()) {
			return 0;
		}

		auto s = sizes();
		auto sum = 0;
		for (index_t i = 0; N != i; ++i) {
			auto t = eraseIfImpl(i, p);
			s[i] -= t;
			sum += t;
		}

		resize(s);

		return sum;
	}

	template <class UnaryPredicate>
	size_type eraseIf(SemanticRange range, UnaryPredicate p)
	{
		return eraseIf(SemanticRangeSet{range}, p);
	}

	template <class UnaryPredicate>
	size_type eraseIf(SemanticRangeSet const &ranges, UnaryPredicate p)
	{
		if (ranges.empty() || empty()) {
			return 0;
		}

		auto s = sizes();
		auto sum = 0;
		for (index_t i = 0; N != i; ++i) {
			auto t = eraseIfImpl(i, ranges, p);
			s[i] -= t;
			sum += t;
		}

		resize(s);

		return sum;
	}

	//
	// Swap
	//

	void swap(Semantics &other) noexcept { std::swap(data_, other.data_); }

 protected:
	//
	// Data
	//

	[[nodiscard]] pointer data() { return empty() ? nullptr : data_.get() + 1; }

	//
	// Number of labels already exists
	//

	template <class InputIt>
	size_type numAlreadyExists(InputIt first, InputIt last) const
	{
		size_type num = 0;

		for (auto l = begin(), u = end(); first != last && l != u; ++first) {
			label_t label;
			if constexpr (std::is_same_v<Semantic, std::decay_t<typename std::iterator_traits<
			                                           InputIt>::value_type>>) {
				label = first->label;
			} else {
				label = *first;
			}
			l = lower_bound(l, u, label);
			if (l != u && l->label == label) {
				++num;
			}
		}

		return num;
	}

	//
	// Insert or assign
	//

	template <bool Assign>
	std::pair<iterator, bool> insert(label_t label, value_t value)
	{
		auto it = lower_bound(label);
		if (end() != it && it->label == label) {
			// Label already exists
			if constexpr (Assign) {
				it->value = value;
			}
			return {it, false};
		} else {
			// Label does not already exist
			auto dist = std::distance(begin(), it);
			resize(size() + 1);
			it = begin() + dist;
			std::move_backward(it, end() - 1, end());
			it->label = label;
			it->value = value;
			return {it, true};
		}
	}

	template <bool Assign>
	iterator insert(const_iterator hint, label_t label, value_t value)
	{
		auto lower = begin();
		auto upper = end();
		auto it = lower + std::distance(const_iterator(lower), hint);

		if (lower != it && (it - 1)->label < label) {
			lower = it;
		}
		if (upper != it && it->label >= label) {
			upper = it;
		}

		it = lower_bound(lower, upper, label);
		if (end() != it && it->label == label) {
			// Label already exists
			if constexpr (Assign) {
				it->value = value;
			}
		} else {
			// Label does not already exist
			auto dist = std::distance(begin(), it);
			resize(size() + 1);
			it = begin() + dist;
			auto last = end();
			std::move_backward(it, last - 1, last);
			it->label = label;
			it->value = value;
		}
		return it;
	}

	template <bool Assign, class InputIt>
	void insertOrAssign(InputIt first, InputIt last)
	{
		std::vector vec(first, last);
		auto f = std::begin(vec);
		auto l = std::end(vec);

		std::sort(f, l, [](auto a, auto b) { return a.label < b.label; });
		l = std::unique(f, l, [](auto a, auto b) { return a.label == b.label; });
		auto s = std::distance(f, l);

		if (empty()) {
			// Optimized insert
			resize(s);
			std::copy(f, l, begin());
		} else {
			auto num_new_elements = s - numAlreadyExists(f, l);
			resize(size() + num_new_elements);
			// Do insert where memory already has been allocated
			insertOrAssign<Assign>(num_new_elements, f, l);
		}
	}

	template <bool Assign, class InputIt>
	void insertOrAssign(size_type num_new_elements, InputIt first, InputIt last)
	{
		if (0 == num_new_elements) {
			if constexpr (Assign) {
				for (auto l = begin(), u = end(); first != last; ++first) {
					l = lower_bound(l, u, first->label);
					l->value = first->value;
				}
			}
			return;
		}

		auto last = end();
		auto cur_last = last - num_new_elements;
		auto lower = begin();
		auto upper = cur_last;

		for (auto r_first = std::make_reverse_iterator(last),
		          r_last = std::make_reverse_iterator(first);
		     r_first != r_last; ++r_first) {
			auto it = lower_bound(lower, upper, r_first->label);
			if (upper != it && it->label == r_first->label) {
				// Label already exists
				if constexpr (Assign) {
					it->value = r_first->value;
				}
			} else {
				// Label does not already exist
				last = std::move_backward(it, cur_last, last);
				*it = *r_first;
			}
			upper = it;
		}
	}

	template <bool Assign, class InputIt, UnaryFunction>
	void insertOrAssign(InputIt first, InputIt last, UnaryFunction f)
	{
		// TODO: Implement
	}

	//
	// Lower bound
	//

	static iterator lower_bound(iterator first, iterator last, label_t label)
	{
		return std::lower_bound(first, last,
		                        Semantic(label, std::numeric_limits<value_t>::lowest()));
	}

	static const_iterator lower_bound(const_iterator first, const_iterator last,
	                                  label_t label)
	{
		return std::lower_bound(first, last,
		                        Semantic(label, std::numeric_limits<value_t>::lowest()));
	}

	//
	// Upper bound
	//

	static iterator upper_bound(iterator first, iterator last, label_t label)
	{
		return std::upper_bound(first, last,
		                        Semantic(label, std::numeric_limits<value_t>::max()));
	}

	static const_iterator upper_bound(const_iterator first, const_iterator last,
	                                  label_t label)
	{
		return std::upper_bound(first, last,
		                        Semantic(label, std::numeric_limits<value_t>::max()));
	}

	//
	// Equal range
	//

	static std::pair<iterator, iterator> equal_range(iterator first, iterator last,
	                                                 label_t label)
	{
		auto it = lower_bound(first, last, label);
		return {it, last == it || it->label != label ? it : std::next(it)};
	}

	static std::pair<const_iterator, const_iterator> equal_range(const_iterator first,
	                                                             const_iterator last,
	                                                             label_t label)
	{
		auto it = lower_bound(first, last, label);
		return {it, last == it || it->label != label ? it : std::next(it)};
	}

	//
	// Erase
	//

	size_type eraseImpl(index_t index, label_t label)
	{
		auto first = begin(index);
		auto last = end(index);

		first = lower_bound(first, last, label);

		if (first == last || first->label != label) {
			return 0;
		}

		std::move(first + 1, last, first);

		return 1;
	}

	size_type eraseImpl(index_t index, SemanticRangeSet ranges)
	{
		auto first = begin(index);
		auto last = end(index);

		size_type num = 0;
		for (auto range : ranges) {
			if (first == last) {
				break;
			}

			first = lower_bound(first, last, range.lower());
			auto upper = upper_bound(first, last, range.upper());

			if (first != upper) {
				num += std::distance(first, upper);
				last = std::move(upper, last, first);
			}
		}

		return num;
	}

	//
	// Erase if
	//

	template <class UnaryPredicate>
	size_type eraseIfImpl(index_t index, UnaryPredicate p)
	{
		auto first = begin(index);
		auto last = end(index);

		first = std::find_if(first, last, p);

		if (first == last) {
			return 0;
		}

		size_type num = 0;
		for (auto it = first; ++it != last;) {
			if (p(*it)) {
				++num;
			} else {
				*first++ = std::move(*it);
			}
		}

		return num;
	}

	template <class UnaryPredicate>
	size_type eraseIfImpl(index_t index, SemanticRangeSet ranges, UnaryPredicate p)
	{
		auto first = begin(index);
		auto last = end(index);

		size_type num = 0;
		for (auto range : ranges) {
			if (first == last) {
				break;
			}

			first = lower_bound(first, last, range.lower());
			auto upper = upper_bound(first, last, range.upper());

			first = std::find_if(first, upper, p);

			if (first != upper) {
				for (auto it = first; ++it != upper;) {
					if (p(*it)) {
						++num;
					} else {
						*first++ = std::move(*it);
					}
				}

				if (first != upper) {
					last = std::move(upper, last, first);
				}
			}
		}

		return num;
	}

	//
	// Resize
	//

	void resize(size_type size)
	{
		if (0 == size) {
			clear();
			return;
		} else if (size() == size) {
			return;
		}

		pointer p_cur = data_.release();
		pointer p_new = static_cast<pointer>(realloc(p_cur, (size + 1) * sizeof(Semantic)));

		if (!p_new) {
			data_.reset(p_cur);
			throw std::bad_alloc();
		}

		data_.reset(p_new);
		data_[0].label = size;
	}

 private:
	std::unique_ptr<Semantic[]> data_;
};
}  // namespace ufo::map

namespace std
{
void swap(ufo::map::Semantics &lhs,
          ufo::map::Semantics &rhs) noexcept(noexcept(lhs.swap(rhs)))
{
	lhs.swap(rhs);
}
}  // namespace std

#endif  // UFO_MAP_SEMANTICS_H