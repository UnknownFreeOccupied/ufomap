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
#include <ufo/map/semantic/semantic_label_propagation.h>
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
template <std::size_t N = 1>
class Semantics
{
	static constexpr N_H =
	    1 + (N - 1) / 2;  // -1 half rounded up (i.e., the number of elements needed to
	                      // store the size of the N semantic containers)

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
			resize(rhs.size());
			std::copy(std::cbegin(rhs), std::cend(rhs), begin());
		}
		return *this;
	}

	Semantics &operator=(Semantics &&rhs) noexcept = default;

	//
	// Iterators
	//

	iterator begin() noexcept { return empty() ? nullptr : std::next(data_.get(), N_H); }

	const_iterator begin() const noexcept
	{
		return empty() ? nullptr : std::next(data_.get(), N_H);
	}

	const_iterator cbegin() const noexcept { return begin(); }

	iterator end() noexcept
	{
		auto const s = allocSize();
		return 0 == s ? nullptr : std::next(data_.get(), s);
	}

	const_iterator end() const noexcept
	{
		auto const s = allocSize();
		return 0 == s ? nullptr : std::next(data_.get(), s);
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
		return empty(index) ? nullptr : std::next(data_.get(), N_H + offset(index));
	}

	const_iterator begin(index_t const index) const noexcept
	{
		return empty(index) ? nullptr : std::next(data_.get(), N_H + offset(index));
	}

	const_iterator cbegin(index_t const index) const noexcept { return begin(index); }

	iterator end(index_t const index) noexcept
	{
		auto const s = size(index);
		return 0 == s ? nullptr : std::next(data_.get(), N_H + offset(index) + s);
	}

	const_iterator end(index_t const index) const noexcept
	{
		auto const s = size(index);
		return 0 == s ? nullptr : std::next(data_.get(), N_H + offset(index) + s);
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
	// Empty
	//

	[[nodiscard]] bool empty() const noexcept { return 0 == size(); }

	[[nodiscard]] bool empty(index_t const index) const noexcept
	{
		return 0 == size(index);
	}

	//
	// Size
	//

	[[nodiscard]] size_type size() const noexcept
	{
		if (!data_) {
			return 0;
		}

		size_type total_size = 0;
		for (std::size_t i = 0; N_H != i; ++i) {
			total_size += data_[i].label;
			total_size += reinterpret_cast<label_t>(data_[i].value);
		}
		return total_size;
	}

	[[nodiscard]] size_type size(index_t const index) const noexcept
	{
		return data_ ? (index % 2 ? data_[index / 2].label
		                          : reinterpret_cast<label_t>(data_[index / 2].value))
		             : 0;
	}

	[[nodiscard]] size_type allocSize() const noexcept { return data_ ? size() + N_H : 0; }

	[[nodiscard]] size_type allocSize(index_t const index) const noexcept
	{
		return data_ ? size(index) + 1;
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

	void clear(index_t const index)
	{
		size_type cur_size = size(index);
		if (0 == cur_size) {
			return;
		}

		if (size() == cur_size) {
			clear();
			return;
		}

		auto index_last = end(index);
		auto last = end();
		if (index_last != last) {
			std::move(index_last, last, begin(index));
		}
		resize(index, 0);
	}

	//
	// Set combine
	//

	template <class InputIt>
	void setCombine(InputIt first, InputIt last, SemanticLabelPropagation const &prop)
	{
		switch (prop.defaultPropCriteria()) {
			case PropagationCriteria::MAX:
				setCombine<InputIt, true>(first, last, prop);
				return;
			default:
				setCombine<InputIt, false>(first, last, prop);
		}
	}

	//
	// Insert
	//

	std::pair<iterator, bool> insert(Semantic semantic)
	{
		return insert<InsertType::NORMAL>(semantic);
	}

	std::pair<iterator, bool> insert(label_t label, value_t value)
	{
		return insert(Semantic(label, value));
	}

	iterator insert(const_iterator hint, Semantic semantic)
	{
		return insert<InsertType::NORMAL>(hint, semantic).first;
	}

	iterator insert(const_iterator hint, label_t label, value_t value)
	{
		return insert(hint, label, value);
	}

	template <class InputIt>
	void insert(InputIt first, InputIt last)
	{
		std::vector<Semantic> temp(first, last);
		insert(temp);
	}

	void insert(std::initializer_list<Semantic> ilist)
	{
		insert(std::cbegin(ilist), std::cend(ilist));
	}

	std::pair<iterator, bool> insert(index_t const index, Semantic semantic)
	{
		return insert<InsertType::NORMAL>(index, semantic);
	}

	std::pair<iterator, bool> insert(index_t const index, label_t label, value_t value)
	{
		return insert(index, Semantic(label, value));
	}

	iterator insert(index_t const index, const_iterator hint, Semantic semantic)
	{
		return insert<InsertType::NORMAL>(index, hint, semantic).first;
	}

	iterator insert(index_t const index, const_iterator hint, label_t label, value_t value)
	{
		return insert(index, hint, label, value);
	}

	template <class InputIt>
	void insert(index_t const index, InputIt first, InputIt last)
	{
		std::vector<Semantic> temp(first, last);
		insert(index, temp);
	}

	void insert(index_t const index, std::initializer_list<Semantic> ilist)
	{
		insert(index, std::cbegin(ilist), std::cend(ilist));
	}

	//
	// Insert or assign
	//

	std::pair<iterator, bool> insertOrAssign(Semantic semantic)
	{
		return insert<InsertType::ASSIGN>(semantic);
	}

	std::pair<iterator, bool> insertOrAssign(label_t label, value_t value)
	{
		return insertOrAssign(Semantic(label, value));
	}

	iterator insertOrAssign(const_iterator hint, Semantic semantic)
	{
		return insert<InsertType::ASSIGN>(hint, semantic).first;
	}

	iterator insertOrAssign(const_iterator hint, label_t label, value_t value)
	{
		return insertOrAssign(Semantic(hint, label, value));
	}

	template <class InputIt>
	void insertOrAssign(InputIt first, InputIt last)
	{
		std::vector<Semantic> temp(first, last);
		insertOrAssign(temp);
	}

	void insertOrAssign(std::initializer_list<Semantic> ilist)
	{
		insertOrAssign(std::cbegin(ilist), std::cend(ilist));
	}

	std::pair<iterator, bool> insertOrAssign(index_t const index, Semantic semantic)
	{
		return insert<InsertType::ASSIGN>(index, semantic);
	}

	std::pair<iterator, bool> insertOrAssign(index_t const index, label_t label,
	                                         value_t value)
	{
		return insertOrAssign(index, Semantic(label, value));
	}

	iterator insertOrAssign(index_t const index, const_iterator hint, Semantic semantic)
	{
		return insert<InsertType::ASSIGN>(index, hint, semantic).first;
	}

	iterator insertOrAssign(index_t const index, const_iterator hint, label_t label,
	                        value_t value)
	{
		return insertOrAssign(index, Semantic(hint, label, value));
	}

	template <class InputIt>
	void insertOrAssign(index_t const index, InputIt first, InputIt last)
	{
		std::vector<Semantic> temp(first, last);
		insertOrAssign(index, temp);
	}

	void insertOrAssign(index_t const index, std::initializer_list<Semantic> ilist)
	{
		insertOrAssign(index, std::cbegin(ilist), std::cend(ilist));
	}

	//
	// Insert or assign custom function
	//

	template <class UnaryFunction>
	std::pair<iterator, bool> insertOrAssign(label_t label, UnaryFunction f)
	{
		return insert(label, f);
	}

	template <class UnaryFunction>
	iterator insertOrAssign(const_iterator hint, label_t label, UnaryFunction f)
	{
		return insert(hint, label, f).first;
	}

	template <class InputIt, class UnaryFunction>
	void insertOrAssign(InputIt first, InputIt last, UnaryFunction f)
	{
		std::vector<Semantic> temp(first, last);
		insertOrAssign(temp, f);
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
		return insert(index, label, f);
	}

	template <class UnaryFunction>
	iterator insertOrAssign(index_t const index, const_iterator hint, label_t label,
	                        UnaryFunction f)
	{
		return insert(index, hint, label, f).first;
	}

	template <class InputIt, class UnaryFunction>
	void insertOrAssign(index_t const index, InputIt first, InputIt last, UnaryFunction f)
	{
		std::vector<Semantic> temp(first, last);
		insertOrAssign(index, temp, f);
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

	void assign(container::Range<label_t> range, value_t value)
	{
		assign(container::RangeSet<label_t>(range), value);
	}

	void assign(container::RangeSet<label_t> const &range, value_t value)
	{
		// TODO: Implement
	}

	template <class UnaryFunction>
	void assign(container::Range<label_t> range, UnaryFunction f)
	{
		assign(container::RangeSet<label_t>(range), f);
	}

	template <class UnaryFunction>
	void assign(container::RangeSet<label_t> const &range, UnaryFunction f)
	{
		// TODO: Implement
	}

	void assign(index_t const index, container::Range<label_t> range, value_t value)
	{
		assign(index, container::RangeSet<label_t>(range), value);
	}

	void assign(index_t const index, container::RangeSet<label_t> const &range,
	            value_t value)
	{
		// TODO: Implement
	}

	template <class UnaryFunction>
	void assign(index_t const index, container::Range<label_t> range, UnaryFunction f)
	{
		assign(index, container::RangeSet<label_t>(range), f);
	}

	template <class UnaryFunction>
	void assign(index_t const index, container::RangeSet<label_t> const &range,
	            UnaryFunction f)
	{
		// TODO: Implement
	}

	/*
	 * Assign value to Keys present in the given Range.
	 */
	template <class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	void assign_values(container::Range<T> const &range, value_t value)
	{
		assign_values(container::RangeSet<T>(range), value);
	}

	/*
	 * Assign value to Keys present in each Range of the RangeSet.
	 */
	template <class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	void assign_values(container::RangeSet<T> const &rangeSet, value_t value)
	{
		for (auto const &range : rangeSet) {
			auto lower = lower_bound(range.lower());
			auto upper = upper_bound(range.upper());
			for (; lower != upper; ++lower) {
				lower->setValue(value);
			}
		}
	}

	iterator erase(const_iterator pos) { return erase(pos, std::next(pos, 1)); }

	iterator erase(iterator pos) { return erase(pos, std::next(pos, 1)); }

	iterator erase(const_iterator first, const_iterator last)
	{
		if (first == last || cend() == first) {
			return end();
		}

		if (cbegin() == first && cend() == last) {
			clear();
			return end();
		}

		// FIXME: Is this safe with unsigned?
		auto new_size = size() - std::distance(first, last);

		auto it = begin();
		std::advance(it, std::distance<const_iterator>(it, first));
		std::move(last, cend(), it);

		resize(new_size);

		return it;
	}

	size_type erase(label_t label)
	{
		if (iterator it = find(label); end() != it) {
			erase(it);
			return 1;
		}
		return 0;
	}

	/*
	 * Erases all Semantics of range which fulfills the pairwise comparison
	 * comp(Semantic.value, value)
	 */
	template <class Compare, class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	size_type erase(container::Range<T> const &range, value_t value, Compare comp)
	{
		return erase(container::RangeSet<T>(range), value, comp);
	}

	template <class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	size_type erase(container::Range<T> const &range)
	{
		return erase(container::RangeSet<T>(range));
	}

	/*
	 * Erases all Semantics of rangeSet which fulfills the pairwise comparison
	 * comp(Semantic.value, value)
	 */
	template <class Compare, class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	size_type erase(container::RangeSet<T> const &rangeSet, value_t value, Compare comp)
	{
		size_type old_size = size();
		for (auto const &range : rangeSet) {
			auto lower = lower_bound(range.lower());
			auto upper = upper_bound(range.upper());
			uint32_t c = 0;

			while (lower != upper) {
				if (!comp(lower->value, value)) {
					++lower;
				} else {
					lower = erase(lower);
					--upper;
				}
			}
		}
		return old_size - size();
	}

	template <class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	size_type erase(container::RangeSet<T> const &rangeSet)
	{
		size_type old_size = size();
		for (auto const &range : rangeSet) {
			auto lower = lower_bound(range.lower());
			auto upper = upper_bound(range.upper());
			erase(lower, upper);
		}
		return old_size - size();
	}

	void swap(Semantics &other) noexcept { std::swap(data_, other.data_); }

	// Lookup
	[[nodiscard]] Semantic at(label_t label) const
	{
		if (auto it = find(label); end() != it) {
			return it->value;
		}
		throw std::out_of_range("Cannot find label " + std::to_string(label));
	}

	[[nodiscard]] Semantic getValue(label_t label) const
	{
		auto it = find(label);
		return end() != it ? it->value : 0;
	}

	[[nodiscard]] size_type count(label_t label) const { return contains(label) ? 1 : 0; }

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

	[[nodiscard]] bool contains(label_t label) const { return end() != find(label); }

	[[nodiscard]] std::pair<iterator, iterator> equal_range(label_t label)
	{
		return equal_range(begin(), end(), label);
	}

	[[nodiscard]] std::pair<const_iterator, const_iterator> equal_range(label_t label) const
	{
		return equal_range(begin(), end(), label);
	}

	[[nodiscard]] iterator lower_bound(label_t label)
	{
		return lower_bound(begin(), end(), label);
	}

	[[nodiscard]] const_iterator lower_bound(label_t label) const
	{
		return lower_bound(begin(), end(), label);
	}

	[[nodiscard]] iterator upper_bound(label_t label)
	{
		return upper_bound(begin(), end(), label);
	}

	[[nodiscard]] const_iterator upper_bound(label_t label) const
	{
		return upper_bound(begin(), end(), label);
	}

	// FIXME: Not safe Semantic *data() { return std::next(data_.get(), 1); }

	Semantic const *data() const { return std::next(data_.get(), 1); }

	/*
	 * True if container contains any Key present in Range.
	 * I.e., the container does not have to contain the full range.
	 */
	template <class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	[[nodiscard]] bool containsAny(container::Range<T> const &range) const
	{
		return containsAny({range});
	}

	/*
	 * True if container contains any Key present in Range which value fulfills the
	 * binary comparison comp.  I.e., the container does not have to contain the full range.
	 */
	template <class Compare, class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	[[nodiscard]] bool containsAny(container::Range<T> const &range, value_t value,
	                               Compare comp) const
	{
		return containsAny({range}, value, comp);
	}

	/*
	 * True if container contains any Key present in any of the Ranges of the set.
	 * I.e., the container does not have to contain the full range.
	 */
	template <class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	[[nodiscard]] bool containsAny(container::RangeSet<T> const &rangeSet) const
	{
		// map contains empty set of ranges
		if (rangeSet.size() == size_type(0)) {
			return true;
		}

		if (size() < rangeSet.size()) {
			for (auto const &element : *this) {
				if (rangeSet.contains(element.label)) {
					return true;
				}
			}
		} else {
			for (auto const &range : rangeSet) {
				const_iterator lower = lower_bound(range.lower());
				if (lower != end() && lower->label <= range.upper()) {
					return true;
				}
			}
		}
		return false;
	}

	/*
	 * True if container contains any Key present in any Range of the set which value
	 * fulfills the binary comparison comp.  I.e., the container does not have to contain
	 * the full range.
	 */
	template <class Compare, class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	[[nodiscard]] bool containsAny(container::RangeSet<T> const &rangeSet, value_t value,
	                               Compare comp) const
	{
		// map contains empty set of ranges
		if (rangeSet.size() == size_type(0)) {
			return true;
		}

		if (size() < rangeSet.size()) {
			for (auto const &element : *this) {
				if (rangeSet.contains(element.label) && comp(element.value, value)) {
					return true;
				}
			}
		} else {
			for (auto const &range : rangeSet) {
				auto lower = lower_bound(range.lower());
				auto upper = upper_bound(range.upper());
				for (; lower != upper; ++lower) {
					if (comp(lower->value, value)) {
						return true;
					}
				}
			}
		}
		return false;
	}

	/*
	 * Negated containsAny
	 */
	template <class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	[[nodiscard]] bool containsNone(container::Range<T> const &range) const
	{
		return containsNone({range});
	}

	/*
	 * Negated containsAny
	 */
	template <class Compare, class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	[[nodiscard]] bool containsNone(container::Range<T> const &range, value_t value,
	                                Compare comp) const
	{
		return !containsAny({range}, value, comp);
	}

	/*
	 * Negated containsAny
	 */
	template <class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	[[nodiscard]] bool containsNone(container::RangeSet<T> const &rangeSet) const
	{
		return !containsAny(rangeSet);
	}

	/*
	 * Negated containsAny
	 */
	template <class Compare, class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	[[nodiscard]] bool containsNone(container::RangeSet<T> const &rangeSet, value_t value,
	                                Compare comp) const
	{
		return !containsAny(rangeSet, value, comp);
	}

	/*
	 * True if container contains all Keys of the range and that comp(Semantic.value,
	 * value) is true for all such elements.
	 */
	template <class Compare, class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	[[nodiscard]] bool containsAll(container::Range<T> const &range, value_t value,
	                               Compare comp) const
	{
		return containsAll({range}, value, comp);
	}

	/*
	 * True if container contains all Keys of the range
	 */
	template <class Compare, class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	[[nodiscard]] bool containsAll(container::Range<T> const &range) const
	{
		return containsAll({range});
	}

	/*
	 * True if container contains all Keys of each range in the set and that
	 * comp(Semantic.value, value) is true for all such elements.
	 */
	template <class Compare, class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	[[nodiscard]] bool containsAll(container::RangeSet<T> const &rangeSet, value_t value,
	                               Compare comp) const
	{
		// map contains empty set of ranges
		if (rangeSet.size() == size_type(0)) {
			return true;
		}
		if (size() < rangeSet.size()) {
			return false;
		}
		typename container::RangeSet<uint32_t>::size_type checked_elems = 0;
		for (auto const &range : rangeSet) {
			// Is it possible that all range elements are in the map?
			checked_elems += (range.upper() - range.lower()) + 1;
			if (checked_elems > size()) {
				return false;
			}

			auto range_dist = range.upper() - range.lower() + 1;
			auto lower = lower_bound(range.lower());
			auto upper = upper_bound(range.upper());
			auto dist = std::distance(lower, upper);
			if (dist != range_dist) {
				return false;
			}

			for (; lower != upper; ++lower) {
				if (!comp(lower->value, value)) {
					return false;
				}
			}
		}
		return true;
	}

	/*
	 * True if container contains all Keys of each Range in the set
	 */
	template <class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	[[nodiscard]] bool containsAll(container::RangeSet<T> const &rangeSet) const
	{
		// map contains empty set of ranges
		if (rangeSet.size() == size_type(0)) {
			return true;
		}
		if (size() < rangeSet.size()) {
			return false;
		}
		typename container::RangeSet<uint32_t>::size_type checked_elems = 0;
		for (auto const &range : rangeSet) {
			// Is it possible that all range elemnts are in the map?
			checked_elems += (range.upper() - range.lower()) + 1;
			if (checked_elems > size()) {
				return false;
			}

			// Check if the individual range elems are here
			auto range_dist = range.upper() - range.lower() + 1;
			auto lower = lower_bound(range.lower());
			auto upper = upper_bound(range.upper());
			auto dist = std::distance(lower, upper);
			if (dist != range_dist) {
				return false;
			}
		}
		return true;
	}

	/*
	 * Return pair of iterators to limits (min max if e.g. less) according to comp
	 * Return (end(), end()) if none present.
	 */
	template <class Compare, class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	[[nodiscard]] std::pair<iterator, iterator> limits(container::Range<T> const &range,
	                                                   Compare comp)
	{
		return limits({range}, comp);
	}

	/*
	 * Return pair of iterators to limits (min max if e.g. less) according to comp
	 * Return (end(), end()) if none present.
	 */
	template <class Compare, class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	[[nodiscard]] std::pair<iterator, iterator> limits(
	    container::RangeSet<T> const &rangeSet, Compare comp)
	{
		if (rangeSet.size() == 0 || size() == 0) {
			return {end(), end()};
		}
		auto ret_low = end();
		auto ret_high = end();

		for (auto const &range : rangeSet) {
			auto lower = lower_bound(range.lower());
			auto upper = upper_bound(range.upper());
			for (; lower != upper; ++lower) {
				if (ret_high == end() || !comp(lower->value, ret_high->value)) {
					ret_high = lower;
				}
				if (ret_low == end() || comp(lower->value, ret_low->value)) {
					ret_low = lower;
				}
			}
		}
		return {ret_low, ret_high};
	}

	std::ostream &write(std::ostream &out_stream) const
	{
		size_type num = size();
		out_stream.write(reinterpret_cast<char const *>(&num), sizeof(size_type));
		return out_stream.write(reinterpret_cast<char const *>(getData()),
		                        sizeof(Semantic) * num);
	}

	std::istream &read(std::istream &in_stream)
	{
		size_type num;
		in_stream.read(reinterpret_cast<char *>(&num), sizeof(size_type));
		if (0 == num) {
			clear();
			return in_stream;
		}
		resize(num);
		return in_stream.read(reinterpret_cast<char *>(getData()), sizeof(Semantic) * num);
	}

	friend std::ostream &operator<<(std::ostream &os, Semantics const &map)
	{
		if (!map.empty()) {
			std::copy(std::begin(map), std::prev(std::end(map)),
			          std::ostream_iterator<Semantic>(os, "; "));
			os << *std::prev(std::end(map));
		}
		return os;
	}

 protected:
	//
	// Index offset
	//

	[[nodiscard]] std::size_t offset(index_t const index) const
	{
		if (!data_) {
			return 0;
		}

		std::size_t offset = 0;
		for (index_t i = 0; index != i; ++i) {
			offset +=
			    i % 2 ? data_[i / 2].label : reinterpret_cast<label_t>(data_[i / 2].value);
		}
		return offset;
	}

	template <class InputIt, bool Max>
	void setCombine(InputIt first, InputIt last, SemanticLabelPropagation const &prop)
	{
		std::vector<std::size_t> sizes;
		sizes.reserve(std::distance(first, last));
		std::transform(first, last, std::back_inserter(sizes),
		               [](auto const &s) { return s.size(); });

		std::size_t total_size =
		    std::accumulate(std::cbegin(sizes), std::cend(sizes), std::size_t(0));

		if (0 == total_size) {
			clear();
			return;
		}

		resize(total_size);

		for (auto it_beg = begin(); first != last; ++first) {
			it_beg = std::copy(std::cbegin(*first), std::cend(*first), it_beg);
		}

		auto it_beg = begin();
		for (std::size_t i = 0; sizes.size() != i; ++i) {
			if (0 == sizes[i]) {
				continue;
			}

			auto it_end = std::next(it_beg, sizes[i]);
			std::inplace_merge(begin(), it_beg, it_end, [](auto const &a, auto const &b) {
				if constexpr (IsPair) {
					return a.label < b.label;
				} else {
					return a < b;
				}
			});
			it_beg = it_end;
		}

		if (!prop.empty() || PropagationCriteria::MEAN == prop.getDefaultPropCriteria()) {
			for (auto it_first = begin(), it_end = end(); it_first != it_end;) {
				auto it_last = std::find_if_not(
				    std::next(it_first), it_end,
				    [l = it_first->label](auto const v) { return l == v.label; });

				auto value = getValue(it_first, it_last, prop.getPropCriteria(it_first->label));

				if constexpr (Max) {
					// FIXME: Improve
					std::prev(it_last)->setValue(value);
				} else {
					it_first->setValue(value);
				}

				it_first = it_last;
			}
		}

		// Remove duplicates (if duplicate, save correct value)
		if constexpr (Max) {
			auto new_beg = std::unique(rbegin(), rend(), [](auto const a, auto const b) {
				               return a.label == b.label;
			               }).base();
			if (begin() != new_beg) {
				std::copy(new_beg, end(), begin());
				resize(std::distance(new_beg, end()));
			}
		} else {
			auto new_end = std::unique(
			    begin(), end(), [](auto const a, auto const b) { return a.label == b.label; });
			if (end() != new_end) {
				resize(std::distance(begin(), new_end));
			}
		}
	}

	template <class InputIt>
	Semantic getValue(InputIt first, InputIt last, PropagationCriteria prop_criteria)
	{
		switch (prop_criteria) {
			case PropagationCriteria::MAX:
				Semantic max = std::numeric_limits<Semantic>::lowest();
				for (; first != last; std::advance(first, 1)) {
					max = std::max(max, first->value);
				}
				return max;
			case PropagationCriteria::MIN:
				Semantic min = std::numeric_limits<Semantic>::max();
				for (; first != last; std::advance(first, 1)) {
					min = std::min(min, first->value);
				}
				return min;
			case PropagationCriteria::MEAN:
				double total = 0;
				double num_elem = std::distance(first, last);
				for (; first != last; std::advance(first, 1)) {
					total += first->value;
				}
				return total / num_elem;
		}
	}

	enum class InsertType { NORMAL, ASSIGN, CUSTOM };

	template <InsertType T>
	std::pair<iterator, bool> insert_impl(label_t label, value_t value)
	{
		if (empty()) {
			resize(1);
			data_[1].label = label;
			data_[1].value = value;
			return {begin(), true};
		}

		auto it = lower_bound(label);
		if (end() != it && it->label == label) {
			// Label already exists
			if constexpr (InsertType::NORMAL == T) {
				// Do nothing
			} else if constexpr (InsertType::ASSIGN == T) {
				// Update value
				it->setValue(value);
			} else if constexpr (InsertType::CUSTOM == T) {
				it->value = f(*it);
			}
			return {it, false};
		} else {
			auto index = std::distance(begin(), it);

			resize(size() + 1);

			it = std::next(begin(), index);

			std::move_backward(it, std::prev(end(), 1), end());

			data_[index + 1].setLabelValue(label, value);

			return {it, true};
		}
	}

	template <InsertType T = InsertType::NORMAL>
	std::pair<iterator, bool> insert_impl(const_iterator hint, label_t label, value_t value)
	{
		if (empty()) {
			resize(1);
			data_[1].label = label;
			data_[1].value = value;
			return {begin(), true};
		}

		auto first = cbegin() != hint && std::prev(hint, 1)->label < label ? hint : cbegin();
		auto last = cend() != hint && hint->label >= label ? hint : cend();
		hint = lower_bound(first, last, label);

		auto index = std::distance(cbegin(), hint);

		if (cend() != hint && hint->label == label) {
			auto it = std::next(begin(), index);
			// Label already exists
			if constexpr (InsertType::NORMAL == T) {
				// Do nothing
			} else if constexpr (InsertType::ASSIGN == T) {
				// Update value
				it->setValue(value);
			} else if constexpr (InsertType::MAX == T) {
				// Set value to max
				it->setValue(std::max(it->value, value));
			}
			return {it, false};
		} else {
			resize(size() + 1);

			auto it = std::next(begin(), index);

			std::move_backward(it, std::prev(end(), 1), end());

			data_[index + 1].setLabelValue(label, value);

			return {it, true};
		}
	}

	template <InsertType InsertT = InsertType::NORMAL>
	void insert_impl(std::vector<Semantic> &vec)
	{
		std::sort(std::begin(vec), std::end(vec));

		// Erase duplicate labels, saving the highest value for each label
		auto r_last = std::unique(std::rbegin(vec), std::rend(vec),
		                          [](auto v1, auto v2) { return v1.label == v2.label; });

		auto first = r_last.base();
		auto last = std::end(vec);

		if (empty()) {
			// Optimized insert
			resize(std::distance(first, last));
			std::copy(first, last, begin());
		} else {
			// Normal insert
			auto hint = begin();
			for (; first != last; ++first) {
				hint = insert<InsertT>(hint, first->label, first->value).first;
			}
		}
	}

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

	static std::pair<iterator, iterator> equal_range(iterator first, iterator last,
	                                                 label_t label)
	{
		auto it = lower_bound(first, last, label);
		if (last == it || it->label != label) {
			return {it, it};
		} else {
			return {it, std::next(it)};
		}
	}

	static std::pair<const_iterator, const_iterator> equal_range(const_iterator first,
	                                                             const_iterator last,
	                                                             label_t label)
	{
		auto it = lower_bound(first, last, label);
		if (last == it || it->label != label) {
			return {it, it};
		} else {
			return {it, std::next(it)};
		}
	}

	void resize(index_t const index, size_type new_size)
	{
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
			for (std::size_t i = 0; N_H != i; ++i) {
				data_[i].label = 0;
				data_[i].value = reinterpret_cast<value_t>(label_t(0));
			}
		}

		// Special
		if (index % 2) {
			data_[index / 2].label = new_size;
		} else {
			data_[index / 2].value = reinterpret_cast<value_t>(static_cast<label_t>(new_size));
		}
	}

 private:
	std::unique_ptr<Semantic[]> data_;

	template <class Derived>
	friend class SemanticMap;
};

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

template <std::size_t N>
void swap(Semantics<N> &lhs, Semantics<N> &rhs) noexcept(noexcept(lhs.swap(rhs)))
{
	lhs.swap(rhs);
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

#endif  // UFO_MAP_SEMANTICS_H