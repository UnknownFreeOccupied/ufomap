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
#include <ufo/container/range_set.h>
#include <ufo/map/index_field.h>
#include <ufo/map/semantic/semantic_propagation.h>
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

template <std::size_t N = 1>
class Semantics
{
 private:
	// FIXME: Make so it works if this is false
	static_assert(sizeof(label_t) == sizeof(value_t));

	// Friend
	friend class SemanticNode<N>;

	// -1 half rounded up (i.e., the number of elements needed to store the sizes of the N
	// semantic containers)
	static constexpr std::size_t N_H = 1 + (N - 1) / 2;

 public:
	// Tags continue
	using size_type = std::size_t;
	using difference_type = std::ptrdiff_t;
	using reference = Semantic &;  // TODO: Make label const
	using const_reference = Semantic const &;
	// using pointer = typename std::allocator_traits<Allocator>::pointer;
	// using const_pointer = typename
	// std::allocator_traits<Allocator>::const_pointer;
	using iterator = Semantic *;  // TODO: Make label const
	using const_iterator = Semantic const *;
	using reverse_iterator = std::reverse_iterator<iterator>;
	using const_reverse_iterator = std::reverse_iterator<const_iterator>;

 public:
	//
	// Constructors
	//

	constexpr Semantics() = default;

	Semantics(Semantics const &other) { *this = other; }

	Semantics(Semantics &&other) noexcept = default;

	//
	// Assignment operator
	//

	Semantics &operator=(Semantics const &rhs)
	{
		if (rhs.empty()) {
			clear();
		} else {
			auto s = size();
			label_t l = 0;
			value_t v = reinterpret_cast<value_t const &>(l);
			for (std::size_t i = 0; N_H != i; ++i) {
				data_[i].label = l;
				data_[i].value = v;
			}
			data_[0].label = s;
			resize(0, rhs.size());
			std::copy(rhs.data(), rhs.data() + rhs.allocSize(), data());
		}
		return *this;
	}

	Semantics &operator=(Semantics &&rhs) noexcept = default;

	//
	// Iterators
	//

	iterator begin() noexcept { return empty() ? nullptr : data_.get() + N_H; }

	const_iterator begin() const noexcept { return empty() ? nullptr : data_.get() + N_H; }

	const_iterator cbegin() const noexcept { return begin(); }

	iterator end() noexcept
	{
		auto const s = allocSize();
		return 0 == s ? nullptr : data_.get() + s;
	}

	const_iterator end() const noexcept
	{
		auto const s = allocSize();
		return 0 == s ? nullptr : data_.get() + s;
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
	// Index iterators
	//

	iterator begin(index_t const index) noexcept
	{
		return empty() ? nullptr : data_.get() + N_H + offset(index);
	}

	const_iterator begin(index_t const index) const noexcept
	{
		return empty() ? nullptr : data_.get() + N_H + offset(index);
	}

	const_iterator cbegin(index_t const index) const noexcept { return begin(index); }

	iterator end(index_t const index) noexcept
	{
		return empty() ? nullptr : data_.get() + N_H + offset(index) + size(index);
	}

	const_iterator end(index_t const index) const noexcept
	{
		return empty() ? nullptr : data_.get() + N_H + offset(index) + size(index);
	}

	const_iterator cend(index_t const index) const noexcept { return end(index); }

	//
	// Reverse index iterators
	//

	reverse_iterator rbegin(index_t const index) noexcept
	{
		return std::make_reverse_iterator(end(index));
	}

	const_reverse_iterator rbegin(index_t const index) const noexcept
	{
		return std::make_reverse_iterator(end(index));
	}

	const_reverse_iterator crbegin(index_t const index) const noexcept
	{
		return rbegin(index);
	}

	reverse_iterator rend(index_t const index) noexcept
	{
		return std::make_reverse_iterator(begin(index));
	}

	const_reverse_iterator rend(index_t const index) const noexcept
	{
		return std::make_reverse_iterator(begin(index));
	}

	const_reverse_iterator crend(index_t const index) const noexcept { return rend(index); }

	//
	// Query
	// TODO: Come up with better name

	[[nodiscard]] util::IteratorWrapper<iterator> iter(index_t const index)
	{
		return util::IteratorWrapper<iterator>(begin(index), end(index));
	}

	[[nodiscard]] util::IteratorWrapper<const_iterator> iter(index_t const index) const
	{
		return util::IteratorWrapper<const_iterator>(begin(index), end(index));
	}

	[[nodiscard]] util::IteratorWrapper<const_iterator> citer(index_t const index) const
	{
		return util::IteratorWrapper<const_iterator>(cbegin(index), cend(index));
	}

	[[nodiscard]] util::IteratorWrapper<iterator> riter(index_t const index)
	{
		return util::IteratorWrapper<iterator>(rbegin(index), rend(index));
	}

	[[nodiscard]] util::IteratorWrapper<const_iterator> riter(index_t const index) const
	{
		return util::IteratorWrapper<const_iterator>(rbegin(index), rend(index));
	}

	[[nodiscard]] util::IteratorWrapper<const_iterator> criter(index_t const index) const
	{
		return util::IteratorWrapper<const_iterator>(crbegin(index), crend(index));
	}

	//
	// Empty
	//

	[[nodiscard]] bool empty() const noexcept { return nullptr == data_; }

	[[nodiscard]] bool empty(index_t const index) const { return 0 == size(index); }

	//
	// Size
	//

	[[nodiscard]] size_type size() const
	{
		if (empty()) {
			return 0;
		}

		size_type total_size = 0;
		for (std::size_t i = 0; N_H != i; ++i) {
			total_size += data_[i].label;
			total_size += reinterpret_cast<label_t const &>(data_[i].value);
		}
		return total_size;
	}

	[[nodiscard]] size_type size(index_t const index) const
	{
		return empty()
		           ? 0
		           : (index % 2 ? data_[index / 2].label
		                        : reinterpret_cast<label_t const &>(data_[index / 2].value));
	}

	[[nodiscard]] size_type allocSize() const { return empty() ? 0 : size() + N_H; }

	[[nodiscard]] size_type allocSize(index_t const index) const
	{
		return empty() ? 0 : size(index) + 1;
	}

	[[nodiscard]] static constexpr size_type maxSize() noexcept
	{
		return size_type(N) * maxSize(0);
	}

	[[nodiscard]] static constexpr size_type maxSize(index_t const index) noexcept
	{
		return std::numeric_limits<label_t>::max() - 1;
	}

	//
	// Clear
	//

	void clear() noexcept { data_.reset(); }

	void clear(index_t const index) { resize(index, 0); }

	//
	// Insert
	//

	void insert(Semantic semantic) { return insert(semantic.label, semantic.value); }

	void insert(label_t label, value_t value)
	{
		return insertOrAssign<false>(label, [value](auto) { return value; });
	}

	template <class InputIt>
	void insert(InputIt first, InputIt last)
	{
		insertOrAssign<false>(first, last);
	}

	void insert(std::initializer_list<Semantic> ilist)
	{
		insert(std::cbegin(ilist), std::cend(ilist));
	}

	std::pair<iterator, bool> insert(index_t const index, Semantic semantic)
	{
		return insert(index, semantic.label, semantic.value);
	}

	std::pair<iterator, bool> insert(index_t const index, label_t label, value_t value)
	{
		return insertOrAssign<false>(index, label, [value](auto) { return value; });
	}

	iterator insert(index_t const index, const_iterator hint, Semantic semantic)
	{
		return insert(index, hint, semantic.label, semantic.value);
	}

	iterator insert(index_t const index, const_iterator hint, label_t label, value_t value)
	{
		return insertOrAssign<false>(index, hint, label, [value](auto) { return value; });
	}

	template <class InputIt>
	void insert(index_t const index, InputIt first, InputIt last)
	{
		insertOrAssign<false>(index, first, last);
	}

	void insert(index_t const index, std::initializer_list<Semantic> ilist)
	{
		insert(index, std::cbegin(ilist), std::cend(ilist));
	}

	//
	// Insert or assign
	//

	void insertOrAssign(Semantic semantic)
	{
		return insertOrAssign(semantic.label, semantic.value);
	}

	void insertOrAssign(label_t label, value_t value)
	{
		return insertOrAssign<true>(label, [value](auto) { return value; });
	}

	template <class InputIt>
	void insertOrAssign(InputIt first, InputIt last)
	{
		insertOrAssign<true>(first, last);
	}

	void insertOrAssign(std::initializer_list<Semantic> ilist)
	{
		insertOrAssign(std::cbegin(ilist), std::cend(ilist));
	}

	std::pair<iterator, bool> insertOrAssign(index_t const index, Semantic semantic)
	{
		return insertOrAssign(index, semantic.label, semantic.value);
	}

	std::pair<iterator, bool> insertOrAssign(index_t const index, label_t label,
	                                         value_t value)
	{
		return insertOrAssign<true>(index, label, [value](auto) { return value; });
	}

	iterator insertOrAssign(index_t const index, const_iterator hint, Semantic semantic)
	{
		return insertOrAssign(index, hint, semantic.label, semantic.value);
	}

	iterator insertOrAssign(index_t const index, const_iterator hint, label_t label,
	                        value_t value)
	{
		return insertOrAssign<true>(index, hint, label, [value](auto) { return value; });
	}

	template <class InputIt>
	void insertOrAssign(index_t const index, InputIt first, InputIt last)
	{
		insertOrAssign<true>(index, first, last);
	}

	void insertOrAssign(index_t const index, std::initializer_list<Semantic> ilist)
	{
		insertOrAssign(index, std::cbegin(ilist), std::cend(ilist));
	}

	//
	// Insert or assign custom function
	//

	template <class UnaryFunction>
	void insertOrAssign(label_t label, UnaryFunction f)
	{
		insertOrAssign<true>(label, f);
	}

	template <class InputIt, class UnaryFunction>
	void insertOrAssign(InputIt first, InputIt last, UnaryFunction f)
	{
		insertOrAssign<true>(first, last, f);
	}

	template <class UnaryFunction>
	void insertOrAssign(std::initializer_list<label_t> ilist, UnaryFunction f)
	{
		insertOrAssign(std::cbegin(ilist), std::cend(ilist), f);
	}

	template <class UnaryFunction>
	std::pair<iterator, bool> insertOrAssign(index_t const index, label_t label,
	                                         UnaryFunction f)
	{
		return insertOrAssign<true>(index, label, f);
	}

	template <class UnaryFunction>
	iterator insertOrAssign(index_t const index, const_iterator hint, label_t label,
	                        UnaryFunction f)
	{
		return insertOrAssign<true>(index, hint, label, f);
	}

	template <class InputIt, class UnaryFunction>
	void insertOrAssign(index_t const index, InputIt first, InputIt last, UnaryFunction f)
	{
		insertOrAssign<true>(index, first, last, f);
	}

	template <class UnaryFunction>
	void insertOrAssign(index_t const index, std::initializer_list<label_t> ilist,
	                    UnaryFunction f)
	{
		insertOrAssign(index, std::cbegin(ilist), std::cend(ilist), f);
	}

	//
	// Assign
	//

	void assign(SemanticRange range, value_t value)
	{
		assign(SemanticRangeSet{range}, value);
	}

	void assign(SemanticRangeSet const &ranges, value_t value)
	{
		for (index_t i = 0; N != i; ++i) {
			assign(i, ranges, value);
		}
	}

	template <class UnaryFunction>
	void assign(SemanticRange range, UnaryFunction f)
	{
		assign(SemanticRangeSet{range}, f);
	}

	template <class UnaryFunction>
	void assign(SemanticRangeSet const &ranges, UnaryFunction f)
	{
		for (index_t i = 0; N != i; ++i) {
			assign(i, ranges, f);
		}
	}

	void assign(index_t const index, SemanticRange range, value_t value)
	{
		assign(index, SemanticRangeSet{range}, value);
	}

	void assign(index_t const index, SemanticRangeSet const &ranges, value_t value)
	{
		assign(index, ranges, [value](auto) { return value; });
	}

	template <class UnaryFunction>
	void assign(index_t const index, SemanticRange range, UnaryFunction f)
	{
		assign(index, SemanticRangeSet{range}, f);
	}

	template <class UnaryFunction>
	void assign(index_t const index, SemanticRangeSet const &ranges, UnaryFunction f)
	{
		auto first = begin(index);
		auto last = end(index);
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
		}

		auto s = sizes();

		auto first_index = index(first);
		auto last_index = index(last);

		size_type r_offset = offset(first_index);

		if (first_index == last_index) {
			s[first_index] -= std::distance(first, last);
			if (0 != s[first_index] && cend(first_index) != last) {
				auto beg = begin() + std::distance(cbegin(), last);
				auto dst = begin() + std::distance(cbegin(), first);
				auto l = std::move(beg, end(first_index), dst);
				r_offset = std::distance(begin(), l);
			} else {
				r_offset = std::distance(begin(), begin(first_index));
			}
		} else {
			// Handle first
			s[first_index] -= std::distance(first, cend(first_index));

			r_offset += s[first_index];

			// Handle middle
			for (; ++first_index != last_index;) {
				s[first_index] = 0;
			}

			// Handle last
			auto dist = std::distance(cbegin(last_index), last);
			s[last_index] -= dist;
			if (0 != dist && 0 != s[last_index]) {
				auto beg = begin() + std::distance(cbegin(), last);
				auto l = std::move(beg, end(last_index), begin(last_index));
				r_offset += std::distance(begin(last_index), l);
			}
		}

		resize(s);

		return begin() + r_offset;
	}

	size_type erase(label_t label)
	{
		if (empty()) {
			return 0;
		}

		auto s = sizes();
		auto sum = 0;
		for (index_t i = 0; N != i; ++i) {
			auto t = eraseImpl(i, label);
			s[i] -= t;
			sum += t;
		}

		resize(s);

		return sum;
	}

	size_type erase(SemanticRange range) { return erase(SemanticRangeSet{range}); }

	size_type erase(SemanticRangeSet const &range_set)
	{
		if (range_set.empty() || empty()) {
			return 0;
		}

		auto s = sizes();
		auto sum = 0;
		for (index_t i = 0; N != i; ++i) {
			auto t = eraseImpl(i, range_set);
			s[i] -= t;
			sum += t;
		}

		resize(s);

		return sum;
	}

	size_type erase(index_t const index, label_t label)
	{
		auto s = eraseImpl(index, label);
		resize(index, size(index) - s);
		return s;
	}

	size_type erase(index_t const index, SemanticRange range)
	{
		return erase(index, SemanticRangeSet{range});
	}

	size_type erase(index_t const index, SemanticRangeSet const &range_set)
	{
		if (range_set.empty() || empty(index)) {
			return 0;
		}

		auto s = eraseImpl(index, range_set);
		resize(index, size(index) - s);
		return s;
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
	size_type eraseIf(SemanticRangeSet const &range_set, UnaryPredicate p)
	{
		if (range_set.empty() || empty()) {
			return 0;
		}

		auto s = sizes();
		auto sum = 0;
		for (index_t i = 0; N != i; ++i) {
			auto t = eraseIfImpl(i, range_set, p);
			s[i] -= t;
			sum += t;
		}

		resize(s);

		return sum;
	}

	template <class UnaryPredicate>
	size_type eraseIf(index_t const index, UnaryPredicate p)
	{
		auto s = eraseIfImpl(index, p);
		resize(index, size(index) - s);
		return s;
	}

	template <class UnaryPredicate>
	size_type eraseIf(index_t const index, SemanticRange range, UnaryPredicate p)
	{
		return eraseIf(index, SemanticRangeSet{range}, p);
	}

	template <class UnaryPredicate>
	size_type eraseIf(index_t const index, SemanticRangeSet const &range_set,
	                  UnaryPredicate p)
	{
		if (range_set.empty() || empty(index)) {
			return 0;
		}

		auto s = eraseIfImpl(index, range_set, p);
		resize(index, size(index) - s);
		return s;
	}

	//
	// Swap
	//

	void swap(Semantics &other) noexcept { std::swap(data_, other.data_); }

	//
	// At
	//

	[[nodiscard]] Semantic at(index_t index, label_t label) const
	{
		if (auto it = find(index, label); end(index) != it) {
			return it->value;
		}
		throw std::out_of_range("Cannot find label " + std::to_string(label));
	}

	//
	// Value
	//

	[[nodiscard]] std::optional<value_t> value(index_t index, label_t label) const
	{
		auto it = find(index, label);
		return end(index) != it ? std::optional<value_t>(it->value) : std::nullopt;
	}

	//
	// Count
	//

	[[nodiscard]] size_type count(index_t index, label_t label) const
	{
		return contains(index, label) ? 1 : 0;
	}

	//
	// Find
	//

	[[nodiscard]] iterator find(index_t index, label_t label)
	{
		auto it = lower_bound(index, label);
		return end(index) != it && it->label == label ? it : end(index);
	}

	[[nodiscard]] const_iterator find(index_t index, label_t label) const
	{
		auto it = lower_bound(index, label);
		return end(index) != it && it->label == label ? it : end(index);
	}

	//
	// Contains
	//

	[[nodiscard]] bool contains(index_t index, label_t label) const
	{
		return end(index) != find(index, label);
	}

	//
	// Equal range
	//

	[[nodiscard]] std::pair<iterator, iterator> equal_range(index_t index, label_t label)
	{
		return equal_range(begin(index), end(index), label);
	}

	[[nodiscard]] std::pair<const_iterator, const_iterator> equal_range(index_t index,
	                                                                    label_t label) const
	{
		return equal_range(begin(index), end(index), label);
	}

	//
	// Lower bound
	//

	[[nodiscard]] iterator lower_bound(index_t index, label_t label)
	{
		return lower_bound(begin(index), end(index), label);
	}

	[[nodiscard]] const_iterator lower_bound(index_t index, label_t label) const
	{
		return lower_bound(begin(index), end(index), label);
	}

	//
	// Upper bound
	//

	[[nodiscard]] iterator upper_bound(index_t index, label_t label)
	{
		return upper_bound(begin(index), end(index), label);
	}

	[[nodiscard]] const_iterator upper_bound(index_t index, label_t label) const
	{
		return upper_bound(begin(index), end(index), label);
	}

	//
	// Data
	//

	[[nodiscard]] Semantic const *data() const { return data_.get() + N_H; }

	[[nodiscard]] Semantic const *data(index_t const index) const
	{
		return data() + offset(index);
	}

	//
	// All
	//

	[[nodiscard]] bool all(index_t index, SemanticRange const &range) const
	{
		return all(index, SemanticRangeSet{range});
	}

	[[nodiscard]] bool all(index_t index, SemanticRangeSet const &range_set) const
	{
		if (range_set.empty()) {
			return true;
		} else if (size(index) < range_set.numValues()) {
			return false;
		}

		auto first = cbegin(index);
		auto last = cend(index);
		for (auto range : range_set) {
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
	[[nodiscard]] bool all(index_t index, SemanticRange const &range,
	                       UnaryPredicate p) const
	{
		return all(index, SemanticRangeSet{range}, p);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool all(index_t index, SemanticRangeSet const &range_set,
	                       UnaryPredicate p) const
	{
		if (range_set.empty()) {
			return true;
		} else if (size(index) < range_set.numValues()) {
			return false;
		}

		auto first = cbegin(index);
		auto last = cend(index);
		for (auto range : range_set) {
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

	[[nodiscard]] bool any(index_t index, SemanticRange const &range) const
	{
		return any(index, SemanticRangeSet{range});
	}

	[[nodiscard]] bool any(index_t index, SemanticRangeSet const &range_set) const
	{
		if (range_set.empty()) {
			return true;
		}

		if (size(index) < range_set.size()) {
			for (auto it = cbegin(index), last = cend(index); it != last; ++it) {
				if (range_set.contains(it->label)) {
					return true;
				}
			}
		} else {
			auto first = cbegin(index);
			auto last = cend(index);
			for (auto range : range_set) {
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
	[[nodiscard]] bool any(index_t index, SemanticRange const &range,
	                       UnaryPredicate p) const
	{
		return any(index, SemanticRangeSet{range}, p);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool any(index_t index, SemanticRangeSet const &range_set,
	                       UnaryPredicate p) const
	{
		if (range_set.empty()) {
			return true;
		}

		if (size(index) < range_set.size()) {
			for (auto it = cbegin(index), last = cend(index); it != last; ++it) {
				if (range_set.contains(it->label) && f(*it)) {
					return true;
				}
			}
		} else {
			auto first = cbegin(index);
			auto last = cend(index);
			for (auto range : range_set) {
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

	[[nodiscard]] bool none(index_t index, SemanticRange const &range) const
	{
		return !any(index, range);
	}

	[[nodiscard]] bool none(index_t index, SemanticRangeSet const &range_set) const
	{
		return !any(index, range_set);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool none(index_t index, SemanticRange const &range,
	                        UnaryPredicate p) const
	{
		return !any(index, range, p);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool none(index_t index, SemanticRangeSet const &range_set,
	                        UnaryPredicate p) const
	{
		return !any(index, range_set, p);
	}

 protected:
	//
	// Data
	//

	[[nodiscard]] Semantic *data() { return data_.get() + N_H; }

	[[nodiscard]] Semantic *data(index_t const index) { return data() + offset(index); }

	//
	// Sizes
	//

	[[nodiscard]] std::array<size_type, N> sizes() const
	{
		if (empty()) {
			return std::array<size_type, N>{};
		}

		std::array<size_type, N> s;
		if constexpr (0 == N % 2) {
			std::copy(data_.get(), data_.get() + N_H, reinterpret_cast<Semantic *>(s.data()));
		} else {
			for (index_t i = 0; N != i; ++i) {
				s[i] = i % 2 ? data_[i / 2].label
				             : reinterpret_cast<label_t const &>(data_[i / 2].value);
			}
		}
		return s;
	}

	//
	// Index offset
	//

	[[nodiscard]] size_type offset(index_t const index) const
	{
		if (empty()) {
			return 0;
		}

		size_type offset = 0;
		for (index_t i = 0; index != i; ++i) {
			offset += i % 2 ? data_[i / 2].label
			                : reinterpret_cast<label_t const &>(data_[i / 2].value);
		}
		return offset;
	}

	//
	// Index
	//

	[[nodiscard]] index_t index(const_iterator it) const
	{
		auto s = sizes();
		auto dist = std::distance(begin(), it);
		index_t i = 0;
		for (auto offset = sizes[0]; N != i && offset < dist; ++i) {
			offset += sizes[i];
		}
		return i;
	}

	//
	// Insert or assign
	//

	template <bool Assign, class UnaryFunction>
	void insertOrAssign(label_t label, UnaryFunction f)
	{
		if (empty()) {
			std::array<size_type, N> s;
			s.fill(1);
			resize(s);
			std::fill(begin(), end(), Semantic(label, f(Semantic(label))));
			return;
		}

		std::array<size_type, N> new_sizes = sizes();
		std::array<difference_type, N> dist;
		for (index_t index = 0; N != index; ++index) {
			auto it = lower_bound(index, label);
			if (end(index) != it && it->label == label) {
				// Label already exists
				if constexpr (Assign) {
					it->value = f(*it);
				}
				dist[index] = 0;
			} else {
				++new_sizes[index];
				dist[index] = std::distance(index, it);
			}
		}

		if (0 == std::accumulate(std::begin(dist), std::end(dist))) {
			return;
		}

		resize(new_sizes);

		for (index_t index = 0; N != index; ++index) {
			if (0 == dist[index]) {
				continue;
			}
			auto it = begin(index) + dist[index];
			auto last_index = end(index);
			std::move_backward(it, last_index - 1, last_index);
			it->label = label;
			it->value = f(Semantic(label));
		}
	}

	template <bool Assign, class UnaryFunction>
	std::pair<iterator, bool> insertOrAssign(index_t const index, label_t label,
	                                         UnaryFunction f)
	{
		if (empty(index)) {
			resize(index, 1);
			auto it = begin(index);
			it->label = label;
			it->value = f(Semantic(label));
			return {it, true};
		}

		auto it = lower_bound(index, label);
		if (end(index) != it && it->label == label) {
			// Label already exists
			if constexpr (Assign) {
				it->value = f(*it);
			}
			return {it, false};
		}

		auto i = std::distance(begin(index), it);
		resize(index, size(index) + 1);
		it = begin(index) + i;
		auto last_index = end(index);
		std::move_backward(it, last_index - 1, last_index);
		it->label = label;
		it->value = f(Semantic(label));
		return {it, true};
	}

	template <bool Assign, class UnaryFunction>
	iterator insertOrAssign(index_t const index, const_iterator hint, label_t label,
	                        UnaryFunction f)
	{
		if (empty(index)) {
			resize(index, 1);
			auto it = begin(index);
			it->label = label;
			it->value = f(Semantic(label));
			return it;
		}

		auto first_index = begin(index);
		auto last_index = end(index);

		auto first =
		    first_index != hint && std::prev(hint, 1)->label < label ? hint : first_index;
		auto last = last_index != hint && hint->label >= label ? hint : last_index;
		hint = lower_bound(first, last, label);

		auto i = std::distance(first_index, hint);

		if (last_index != hint && hint->label == label) {
			// Label already exists
			auto it = first_index + i;
			if constexpr (Assign) {
				it->value = f(*it);
			}
			return it;
		}

		resize(index, size(index) + 1);
		auto it = begin(index) + i;
		last_index = end(index);
		std::move_backward(it, last_index - 1, last_index);
		it->label = label;
		it->value = f(Semantic(label));
		return it;
	}

	template <class InputIt>
	size_type numAlreadyExists(index_t const index, InputIt first, InputIt last) const
	{
		size_type num = 0;

		auto first_index = cbegin(index);
		auto last_index = cend(index);
		for (; first != last && first_index != last_index; ++first) {
			label_t label;
			if constexpr (std::is_same_v<Semantic,
			                             typename std::iterator_traits<InputIt>::value_type>) {
				label = first->label;
			} else {
				label = *first;
			}
			first_index = lower_bound(first_index, last_index, label);
			if (first_index != last_index && first_index->label == label) {
				++num;
			}
		}

		return num;
	}

	/*!
	 * @brief
	 *
	 * @note Memory assumed already allocated
	 *
	 * @param index
	 * @param cur_size
	 * @param new_size
	 * @param first
	 * @param last
	 */
	template <bool Assign, class InputIt>
	void insertOrAssign(index_t index, size_type cur_size, size_type new_size,
	                    InputIt first, InputIt last)
	{
		if (0 == new_size) {
			return;
		} else if (0 == cur_size) {
			std::copy(first, last, begin(index));
			return;
		}

		if constexpr (!Assign) {
			if (cur_size == new_size) {
				return;
			}
		}

		auto cur = end(index);
		auto first_index = begin(index);
		auto last_index = first_index + cur_size;
		while (first != last && first_index != last_index) {
			if constexpr (Assign) {
				if ((last_index - 1)->label == (last - 1)->label) {
					(--cur)->label = (--last_index)->label;
					cur->value = (--last)->value;
					continue;
				}
			}
			if ((last_index - 1)->label < (last - 1)->label) {
				*(--cur) = *(--last);
				if constexpr (!Assign) {
					++cur_size;
					// FIXME: Does this actually improve performance?
					if (cur_size == new_size) {
						return;
					}
				}
			} else {
				// FIXME: Can this be a move? What happens if it is the same?
				*(--cur) = *(--last_index);
			}
		}

		// Copy the remaining to the beginning
		std::copy(first, last, first_index);
	}

	/*!
	 * @brief
	 *
	 * @note Memory assumed already allocated
	 *
	 * @param index
	 * @param cur_size
	 * @param new_size
	 * @param first
	 * @param last
	 * @param f
	 */
	template <bool Assign, class InputIt, class UnaryFunction>
	void insertOrAssign(index_t index, size_type cur_size, size_type new_size,
	                    InputIt first, InputIt last, UnaryFunction f)
	{
		if (0 == new_size) {
			return;
		} else if (0 == cur_size) {
			for (auto it = begin(index); first != last; ++it, ++first) {
				it->label = *first;
				it->value = f(Semantic(*first));
			}
			return;
		}

		if constexpr (!Assign) {
			if (cur_size == new_size) {
				return;
			}
		}

		auto cur = end(index);
		auto first_index = begin(index);
		auto last_index = first_index + cur_size;
		while (first != last && first_index != last_index) {
			if constexpr (Assign) {
				if ((last_index - 1)->label == *(last - 1)) {
					(--cur)->label = (--last_index)->label;
					cur->value = f(*cur);
					continue;
				}
			}
			if ((last_index - 1)->label < *(last - 1)) {
				--cur;
				cur->label = *(--last);
				cur->value = f(Semantic(cur->label));
				if constexpr (!Assign) {
					++cur_size;
					// FIXME: Does this actually improve performance?
					if (cur_size == new_size) {
						return;
					}
				}
			} else {
				// FIXME: Can this be a move? What happens if it is the same?
				*(--cur) = *(--last_index);
			}
		}

		// Copy the remaining to the beginning
		for (auto it = begin(index); first != last; ++it, ++first) {
			it->label = *first;
			it->value = f(Semantic(*first));
		}
	}

	template <bool Assign, class InputIt>
	void insertOrAssign(InputIt first, InputIt last)
	{
		std::vector vec(first, last);

		std::sort(std::begin(vec), std::end(vec));

		// Erase duplicate labels, saving highest value for each label
		auto r_last = std::unique(std::rbegin(vec), std::rend(vec),
		                          [](auto a, auto b) { return a.label == b.label; });

		auto f = r_last.base();
		auto l = std::end(vec);
		auto s = std::distance(f, l);

		std::array<size_type, N> new_sizes;
		new_sizes.fill(s);

		if (empty()) {
			resize(new_sizes);
			for (index_t index = 0; N != index; ++index) {
				std::copy(f, l, begin(index));
			}
		} else {
			for (index_t index = 0; N != index; ++index) {
				new_sizes -= numAlreadyExists(index, f, l);
			}

			std::array<size_type, N> cur_sizes = sizes();
			resize(new_sizes);

			// Do insert where memory already has been allocated
			for (index_t index = 0; N != index; ++index) {
				insertOrAssign<Assign>(index, cur_sizes[index], new_sizes[index], f, l);
			}
		}
	}

	template <bool Assign, class InputIt, class UnaryFunction>
	void insertOrAssign(InputIt first, InputIt last, UnaryFunction fun)
	{
		std::vector vec(first, last);

		std::sort(std::begin(vec), std::end(vec));

		// Erase duplicate labels
		auto r_last = std::unique(std::rbegin(vec), std::rend(vec),
		                          [](auto a, auto b) { return a == b; });

		auto f = r_last.base();
		auto l = std::end(vec);
		auto s = std::distance(f, l);

		std::array<size_type, N> new_sizes;
		new_sizes.fill(s);

		if (empty()) {
			// Optimized insert
			std::vector<Semantic> sem;
			sem.reserve(s);
			for (; f != l; ++first) {
				sem.emplace_back(*f, fun(Semantic(*f)));
			}
			resize(new_sizes);
			for (index_t index = 0; N != index; ++index) {
				std::copy(std::cbegin(sem), std::cend(sem), begin(index));
			}
		} else {
			// Calculate how many elements already exists per index
			for (index_t index = 0; N != index; ++index) {
				new_sizes -= numAlreadyExists(index, f, l);
			}

			std::array<size_type, N> cur_sizes = sizes();
			resize(new_sizes);

			// Do insert where memory already has been allocated
			for (index_t index = 0; N != index; ++index) {
				insertOrAssign<Assign>(index, cur_sizes[index], new_sizes[index], f, l, fun);
			}
		}
	}

	template <bool Assign, class InputIt>
	void insertOrAssign(index_t const index, InputIt first, InputIt last)
	{
		std::vector vec(first, last);

		std::sort(std::begin(vec), std::end(vec));

		// Erase duplicate labels, saving highest value for each label
		auto r_last = std::unique(std::rbegin(vec), std::rend(vec),
		                          [](auto a, auto b) { return a.label == b.label; });

		auto f = r_last.base();
		auto l = std::end(vec);
		auto s = std::distance(f, l);

		if (empty(index)) {
			// Optimized insert
			resize(index, s);
			std::copy(f, l, begin(index));
		} else {
			auto new_size = s - numAlreadyExists(index, f, l);
			auto cur_size = size(index);

			resize(index, new_size);

			// Do insert where memory already has been allocated
			insertOrAssign<Assign>(index, cur_size, new_size, f, l);
		}
	}

	template <bool Assign, class InputIt, class UnaryFunction>
	void insertOrAssign(index_t const index, InputIt first, InputIt last, UnaryFunction fun)
	{
		std::vector vec(first, last);

		std::sort(std::begin(vec), std::end(vec));

		// Erase duplicate labels, saving highest value for each label
		auto r_last = std::unique(std::rbegin(vec), std::rend(vec),
		                          [](auto a, auto b) { return a == b; });

		auto f = r_last.base();
		auto l = std::end(vec);
		auto s = std::distance(f, l);

		if (empty(index)) {
			// Optimized insert
			resize(index, s);
			for (auto it = begin(index); f != l; ++it, ++f) {
				it->label = *f;
				it->value = fun(Semantic(*f));
			}
		} else {
			auto new_size = s - numAlreadyExists(index, f, l);
			auto cur_size = size(index);

			resize(index, new_size);

			// Do insert where memory already has been allocated
			insertOrAssign<Assign>(index, cur_size, new_size, f, l, fun);
		}
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

	void resize(std::array<size_type, N> new_sizes)
	{
		auto cur_sizes = sizes();
		if (cur_sizes == new_sizes) {
			return;
		}

		auto cur_size = std::accumulate(std::cbegin(cur_sizes), std::cend(cur_sizes), N_H);
		auto new_size = std::accumulate(std::cbegin(new_sizes), std::cend(new_sizes), N_H);

		for (index_t i = 0; N - 1 != i; ++i) {
			// Reduce the indices that should be reduced
			if (cur_sizes[i] <= new_sizes[i]) {
				continue;
			}

			std::move(begin(i + 1), end(i + 1), begin(i) + new_sizes[i]);

			// Set size
			if (i % 2) {
				data_[i / 2].label = new_sizes[i];
			} else {
				data_[i / 2].value =
				    reinterpret_cast<value_t const &>(static_cast<label_t const &>(new_sizes[i]));
			}
		}

		if (cur_size != new_size) {
			Semantic *ptr_cur = data_.release();

			Semantic *ptr_new =
			    static_cast<Semantic *>(realloc(ptr_cur, new_size * sizeof(Semantic)));

			if (!ptr_new) {
				data_.reset(ptr_cur);
				throw std::bad_alloc();
			}

			data_.reset(ptr_new);
		}

		if (0 == cur_size) {
			// Set sizes
			for (index_t i = 0; N != i; ++i) {
				if (i % 2) {
					data_[i / 2].label = new_sizes[i];
				} else {
					data_[i / 2].value = reinterpret_cast<value_t const &>(
					    static_cast<label_t const &>(new_sizes[i]));
				}
			}
		} else {
			// Increase the indices that should be increased
			auto last = data_.get() + new_size;
			for (index_t i = N - 1; 0 != i; --i) {
				// Set size
				if (i % 2) {
					data_[i / 2].label = new_sizes[i];
				} else {
					data_[i / 2].value = reinterpret_cast<value_t const &>(
					    static_cast<label_t const &>(new_sizes[i]));
				}

				if (0 == new_sizes[i] || 0 == cur_sizes[i]) {
					continue;
				}

				auto first_index = begin(i);
				auto last_index = end(i);
				if (last != last_index) {
					last = std::move_backward(first_index, last_index, last);
				} else {
					last = first_index;
				}
			}
		}
	}

	void resize(index_t const index, size_type new_size)
	{
		auto const cur_size = size(index);
		if (cur_size == new_size) {
			return;
		}

		if (cur_size > new_size) {
			// Move to last if size decreases
			auto first_index = begin(index) + new_size;
			auto last_index = end(index);
			auto last = end();
			if (last_index != last) {
				std::move(last_index, last, first_index);
			}
		}

		Semantic *ptr_cur = data_.release();

		auto new_total_size = new_size + size() - size(index) + N_H;

		Semantic *ptr_new =
		    static_cast<Semantic *>(realloc(ptr_cur, new_total_size * sizeof(Semantic)));

		if (!ptr_new) {
			data_.reset(ptr_cur);
			throw std::bad_alloc();
		}

		data_.reset(ptr_new);

		if (!ptr_cur) {
			// Initialize
			label_t l = 0;
			value_t v = reinterpret_cast<value_t const &>(l);
			for (std::size_t i = 0; N_H != i; ++i) {
				data_[i].label = l;
				data_[i].value = v;
			}
		} else if (cur_size < new_size) {
			// Move indices after
			auto last_index = end(index);
			auto last = end();
			if (last_index != last) {
				std::move_backward(last_index, last, data_.get() + new_total_size);
			}
		}

		// Set new size of index
		if (index % 2) {
			data_[index / 2].label = new_size;
		} else {
			data_[index / 2].value =
			    reinterpret_cast<value_t const &>(static_cast<label_t const &>(new_size));
		}
	}

	//
	// Get value
	//

	template <class InputIt>
	value_t getValue(InputIt first, InputIt last, PropagationCriteria prop_criteria)
	{
		switch (prop_criteria) {
			case PropagationCriteria::MIN:
			case PropagationCriteria::MAX:
				return first->value;
			case PropagationCriteria::MEAN:
				return std::accumulate(first, last, 0.0,
				                       [](double r, auto e) { return r + e.value; }) /
				       std::distance(first, last);
		}
	}

	//
	// Update
	//

	template <class InputIt>
	void update(IndexField indices, InputIt first, InputIt last,
	            SemanticPropagation const &prop)
	{
		if (1 == N) {
			std::vector<size_type> sizes;
			for (auto it = first; it != last; ++it) {
				sizes.push_back(it->size());
			}

			resize(0, std::reduce(std::cbegin(sizes), std::cend(sizes)));

			for (auto it = begin(); first != last; ++first) {
				it = std::copy(std::cbegin(*first), std::cend(*first), it);
			}

			// TODO: Implement
		} else {
			std::vector<Semantic> sem;
			for (index_t i = 0; N != i; ++i, ++first) {
				if (!indices[i]) {
					continue;
				}

				sem.resize(first->size());
				std::copy(std::cbegin(*first), std::cend(*first), std::begin(sem));

				auto sizes = first->sizes();

				// TODO: Implement

				// resize(i, std::distance(f, l));
				// std::copy(f, l, begin(i));
			}
		}
	}

	//
	// Input/Output
	//

	void read(std::istream &in, IndexField indices, SemanticPropagation const &prop)
	{
		uint8_t n;
		in.read(reinterpret_cast<char *>(&n), sizeof(n));

		if (N == n) {
			std::array<size_type, N> sizes;
			std::array<Semantic, N_H> s;
			in.read(reinterpret_cast<char *>(s.data()), N_H * sizeof(Semantic));
			for (index_t i = 0, j = 0; N_H != i; ++i) {
				sizes[j++] = s[i].label;
				sizes[j++] = s[i].value;
			}

			if (indices.all()) {
				resize(sizes);
				in.read(reinterpret_cast<char *>(begin()),
				        std::accumulate(std::cbegin(sizes), std::cend(sizes), std::size_t(0)) *
				            sizeof(Semantic));
			} else {
				auto cur_sizes = sizes();
				for (index_t i = 0; N != i; ++i) {
					if (!indices[i]) {
						sizes[i] = cur_sizes[i];
					}
				}

				resize(sizes);

				for (index_t i = 0; N != i; ++i) {
					if (!indices[i]) {
						// Skip forward
						in.seekg(sizes[i] * sizeof(Semantic), std::istream::cur);
					} else {
						in.read(reinterpret_cast<char *>(begin(i)), sizes[i] * sizeof(Semantic));
					}
				}
			}
		} else if (1 == n) {
			Semantic s;
			in.read(reinterpret_cast<char *>(&s), sizeof(s));

			std::array<size_type, N> sizes;
			sizes.fill(s.label);

			resize(sizes);

			in.read(reinterpret_cast<char *>(begin()), s.label * sizeof(Semantic));

			auto first = begin(0);
			auto last = end(0);

			for (index_t i = 1; N != i; ++i) {
				std::copy(first, last, begin(i));
			}
		} else if (1 == N) {
			std::vector<size_type> sizes;
			sizes.reserve(n + 1);
			auto n_h = 1 + (n - 1) / 2;
			std::vector<Semantic> s(n_h);
			in.read(reinterpret_cast<char *>(s.data()), n_h * sizeof(Semantic));
			for (index_t i = 0, j = 0; n_h != i; ++i) {
				sizes[j++] = s[i].label;
				sizes[j++] = s[i].value;
			}

			auto total_size = std::reduce(std::cbegin(sizes), std::cend(sizes));
			resize(0, total_size);
			in.read(reinterpret_cast<char *>(begin()), total_size + sizeof(Semantic));

			// Sort
			auto def = prop.defaultPropCriteria();
			auto first = begin();
			auto mid = first + sizes[0];
			switch (def) {
				case PropagationCriteria::MIN:
					// Lowest value first
					for (index_t i = 1; n != i; ++i) {
						auto last = mid + sizes[i];
						std::inplace_merge(first, mid, last);
						mid = last;
					}
					break;
				case PropagationCriteria::MAX:
					// Highest value first
					for (index_t i = 1; n != i; ++i) {
						auto last = mid + sizes[i];
						std::inplace_merge(first, mid, last, [](auto a, auto b) {
							return a.label < b.label || (a.label == b.label && a.value > b.value);
						});
						mid = last;
					}
					break;
				default:
					// Order of value does not matter
					for (index_t i = 1; n != i; ++i) {
						auto last = mid + sizes[i];
						std::inplace_merge(first, mid, last,
						                   [](auto a, auto b) { return a.label < b.label; });
						mid = last;
					}
			}

			// Remove duplicates
			if (!prop.empty() ||
			    (PropagationCriteria::MIN != def && PropagationCriteria::MAX != def)) {
				for (auto first = begin(), last = end(); first != last;) {
					auto it = std::find_if_not(std::next(first), last,
					                           [l = first->label](auto e) { return l == e.label; });
					it->value = getValue(first, it, prop.propCriteria(first->label));
					first = it;
				}
			}

			auto new_end =
			    std::unique(begin(), end(), [](auto a, auto b) { return a.label == b.label; });

			resize(0, std::distance(begin(), new_end));
		} else {
			throw std::invalid_argument("Wrong number of semantic indices, expected 1 or " +
			                            std::to_string(N) + " got " + std::to_string(n) + ".");
		}
	}

	void write(std::ostream &out) const
	{
		constexpr uint8_t n = N;
		out.write(reinterpret_cast<char const *>(&n), sizeof(n));

		if (empty()) {
			std::array<Semantic, N_H> s{};
			out.write(reinterpret_cast<char const *>(s.data()), N_H * sizeof(Semantic));
			return;
		}

		out.write(reinterpret_cast<char const *>(data()), (size() + N_H) * sizeof(Semantic));
	}

 private:
	std::unique_ptr<Semantic[]> data_;
};

template <std::size_t N>
std::ostream &operator<<(std::ostream &os, Semantics<N> const &map)
{
	// TODO: Implement correctly
	if (!map.empty()) {
		std::copy(std::begin(map), std::prev(std::end(map)),
		          std::ostream_iterator<Semantic>(os, "; "));
		os << *std::prev(std::end(map));
	}
	return os;
}

template <std::size_t N>
bool operator==(Semantics<N> const &lhs, Semantics<N> const &rhs)
{
	return std::equal(std::begin(lhs), std::end(lhs), std::begin(rhs), std::end(rhs));
}

template <std::size_t N>
bool operator!=(Semantics<N> const &lhs, Semantics<N> const &rhs)
{
	return !(lhs == rhs);
}

template <std::size_t N, class Pred>
typename Semantics<N>::size_type erase_if(Semantics<N> &c, Pred pred)
{
	auto old_size = c.size();
	for (auto it = std::begin(c), last = std::end(c); it != last;) {
		if (pred(*it)) {
			it = c.erase(it);
		} else {
			++it;
		}
	}
	return old_size - c.size();
}
}  // namespace ufo::map

namespace std
{
template <std::size_t N>
void swap(ufo::map::Semantics<N> &lhs,
          ufo::map::Semantics<N> &rhs) noexcept(noexcept(lhs.swap(rhs)))
{
	lhs.swap(rhs);
}
}  // namespace std

#endif  // UFO_MAP_SEMANTICS_H