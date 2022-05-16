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

#ifndef UFO_CONTAINER_FLAT_MAP_H
#define UFO_CONTAINER_FLAT_MAP_H

// UFO
#include <ufo/container/range.h>

// STL
#include <algorithm>
#include <cstddef>  // For std::ptrdiff_t
#include <execution>
#include <functional>
#include <initializer_list>
#include <istream>
#include <iterator>  // For std::random_access_iterator_tag / std::contiguous_iterator_tag
#include <memory>
#include <ostream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace ufo::container
{

/*!
 * Associative ordered container using a data-coherent implementation.
 * Elements of the container are stored using a single unsigned type
 * where the left-most bits represent the key and the right-most the value.
 * The number of bits allocated for the key and value is custom for each
 * container instance, e.g., if DataType = uint16_t and ValueWidth = 1
 * the key will be 15 bits and the value 1 bit.
 */
template <typename DataType, size_t ValueWidth, size_t FixedSize = 0>
class FlatMap
{
 private:
	static_assert(std::is_unsigned_v<DataType>, "FlatMap require unsigned data type.");

	static constexpr size_t LabelWidth = std::numeric_limits<DataType>::digits - ValueWidth;
	static constexpr DataType LabelMask =
	    static_cast<DataType>(std::numeric_limits<DataType>::max() >> ValueWidth)
	    << ValueWidth;
	static constexpr DataType ValueMask =
	    static_cast<DataType>(std::numeric_limits<DataType>::max() << LabelWidth) >>
	    LabelWidth;

 public:
	// Tags
	using key_type = DataType;
	using mapped_type = DataType;

	struct Element {
	 public:
		constexpr Element() = default;

		constexpr Element(key_type key, mapped_type value)
		    : data_((key << ValueWidth) | (value & ValueMask))
		{
		}

		[[nodiscard]] constexpr key_type key() const noexcept { return data_ >> ValueWidth; }

		[[nodiscard]] constexpr mapped_type value() const noexcept
		{
			return data_ & ValueMask;
		}

		constexpr bool operator==(Element const &rhs) const noexcept
		{
			return rhs.data_ == data_;
		}

		constexpr bool operator!=(Element const &rhs) const noexcept
		{
			return rhs.data_ != data_;
		}

		constexpr bool operator<(Element const &rhs) const noexcept
		{
			return data_ < rhs.data_;
		}

		constexpr bool operator<=(Element const &rhs) const noexcept
		{
			return data_ <= rhs.data_;
		}

		constexpr bool operator>(Element const &rhs) const noexcept { return rhs < *this; }

		constexpr bool operator>=(Element const &rhs) const noexcept { return rhs <= *this; }

		friend std::ostream &operator<<(std::ostream &os, Element element)
		{
			// Note: '+' to apply arithmetic promotion.
			os << +element.key() << ": " << +element.value();
			return os;
		}

	 private:
		constexpr void setKey(key_type key) noexcept
		{
			data_ = (key << ValueWidth) | (data_ & ValueMask);
		}

		constexpr void setValue(mapped_type value) noexcept
		{
			data_ = (data_ & LabelMask) | (value & ValueMask);
		}

		constexpr void setKeyValue(key_type key, mapped_type value) noexcept
		{
			data_ = (key << ValueWidth) | (value & ValueMask);
		}

		constexpr void setData(DataType data) noexcept { data_ = data; }

		constexpr void increaseValue(mapped_type inc) noexcept
		{
			// FIXME: Look at
			data_ =
			    (data_ & LabelMask) |
			    std::min(ValueMask, static_cast<mapped_type>(getValue() + (inc & ValueMask)));
		}

		constexpr void decreaseValue(mapped_type dec) noexcept
		{
			// FIXME: Look at
			auto cur = getValue();
			dec = std::min(cur, dec & ValueMask);
			data_ = (data_ & LabelMask) | (cur - dec);
		}

	 private:
		DataType data_ = 0;
		friend class FlatMap;
	};

	// Tags
	using value_type = Element;
	using size_type = DataType;
	using difference_type = std::ptrdiff_t;
	using reference = value_type &;
	using const_reference = value_type const &;
	// using pointer = typename std::allocator_traits<Allocator>::pointer;
	// using const_pointer = typename
	// std::allocator_traits<Allocator>::const_pointer;
	using iterator = Element *;
	using const_iterator = Element const *;
	using reverse_iterator = std::reverse_iterator<iterator>;
	using const_reverse_iterator = std::reverse_iterator<const_iterator>;

	// Constructors
	constexpr FlatMap() = default;

	template <class InputIt>
	FlatMap(InputIt first, InputIt last)
	{
		insert(first, last);
	}

	template <class ExecutionPolicy, class InputIt,
	          typename = std::enable_if_t<
	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
	FlatMap(ExecutionPolicy policy, InputIt first, InputIt last)
	{
		insert(policy, first, last);
	}

	FlatMap(FlatMap const &other) { *this = other; }

	FlatMap(FlatMap &&other) noexcept = default;

	FlatMap(std::initializer_list<value_type> init) { insert(init); }

	// Destructor
	~FlatMap() { clear(); }

	// operator=
	FlatMap &operator=(FlatMap const &rhs)
	{
		if (rhs.empty()) {
			clear();
		} else {
			resize(rhs.size());
			std::copy(std::begin(rhs), std::end(rhs), begin());
		}
		return *this;
	}

	FlatMap &operator=(FlatMap &&rhs) noexcept = default;

	FlatMap &operator=(std::initializer_list<value_type> ilist)
	{
		insert(ilist);
		return *this;
	}

	// Iterators
	iterator begin() noexcept
	{
		if constexpr (FixedSize) {
			return std::next(std::begin(data_));
		} else {
			return empty() ? nullptr : std::next(data_);
		}
	}

	const_iterator begin() const noexcept
	{
		if constexpr (FixedSize) {
			return std::next(std::begin(data_));
		} else {
			return empty() ? nullptr : std::next(data_);
		}
	}

	const_iterator cbegin() const noexcept { return begin(); }

	iterator end() noexcept
	{
		if constexpr (FixedSize) {
			return std::next(std::begin(data_), size() + 1);
		} else {
			auto const s = allocSize();
			return 0 == s ? nullptr : std::next(data_, s);
		}
	}

	const_iterator end() const noexcept
	{
		if constexpr (FixedSize) {
			return std::next(std::begin(data_), size() + 1);
		} else {
			auto const s = allocSize();
			return 0 == s ? nullptr : std::next(data_, s);
		}
	}

	const_iterator cend() const noexcept { return end(); }

	// Capacity
	// [[nodiscard]]
	[[nodiscard]] bool empty() const noexcept { return 0 == size(); }

	[[nodiscard]] size_type size() const noexcept
	{
		if constexpr (FixedSize) {
			return data_[0].key();
		} else {
			return data_ ? data_[0].getKey() : 0;
		}
	}

	[[nodiscard]] size_type allocSize() const noexcept
	{
		if constexpr (FixedSize) {
			return FixedSize + 1;
		} else {
			return empty() ? 0 : data_[0].getKey() + 1;
		}
	}

	[[nodiscard]] constexpr size_type max_size() const noexcept
	{
		if constexpr (FixedSize) {
			return FixedSize;
		} else {
			return LabelMask >> ValueWidth;
		}
	}

	// Modifiers
	void clear() noexcept
	{
		if constexpr (FixedSize) {
			data_[0].setData(0);
		} else {
			if (nullptr != data_) {
				free(data_);
				data_ = nullptr;
			}
		}
	}

	template <class InputIt>
	void setOrdered(InputIt first, InputIt last)
	{
		resize(std::distance(first, last));
		std::copy(first, last, begin());
	}

	template <class ExecutionPolicy, class InputIt,
	          typename = std::enable_if_t<
	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
	void setOrdered(ExecutionPolicy policy, InputIt first, InputIt last)
	{
		resize(std::distance(first, last));
		std::copy(policy, first, last, begin());
	}

	std::pair<iterator, bool> insert(value_type const &value)
	{
		return insert_impl<InsertType::NORMAL>(value.getKey(), value.getValue());
	}

	iterator insert(const_iterator hint, value_type const &value)
	{
		return insert_impl<InsertType::NORMAL>(hint, value.first, value.second).first;
	}

	template <class InputIt>
	void insert(InputIt first, InputIt last)
	{
		std::vector<typename std::iterator_traits<InputIt>::value_type> temp(first, last);
		insert_impl(temp);
	}

	template <class ExecutionPolicy, class InputIt,
	          typename = std::enable_if_t<
	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
	void insert(ExecutionPolicy policy, InputIt first, InputIt last)
	{
		std::vector<typename std::iterator_traits<InputIt>::value_type> temp(first, last);
		insert_impl(policy, temp);
	}

	void insert(std::initializer_list<value_type> ilist)
	{
		std::vector<value_type> temp(ilist);
		insert_impl(temp);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void insert(ExecutionPolicy policy, std::initializer_list<value_type> ilist)
	{
		std::vector<value_type> temp(ilist);
		insert_impl(policy, temp);
	}

	template <class... Args>
	std::pair<iterator, bool> emplace(Args &&...args)
	{
		return insert_impl(std::forward<Args>(args)...);
	}

	template <class... Args>
	std::pair<iterator, bool> try_emplace(key_type const &k, Args &&...args)
	{
		return insert(value_type(k, std::forward<Args>(args)...));
	}

	template <class... Args>
	std::pair<iterator, bool> try_emplace(key_type &&k, Args &&...args)
	{
		return insert(value_type(std::move(k), std::forward<Args>(args)...));
	}

	template <class... Args>
	std::pair<iterator, bool> try_emplace(const_iterator hint, key_type const &k,
	                                      Args &&...args)
	{
		return insert(hint, value_type(k, std::forward<Args>(args)...));
	}

	template <class... Args>
	std::pair<iterator, bool> try_emplace(const_iterator hint, key_type &&k, Args &&...args)
	{
		return insert(hint, value_type(std::move(k), std::forward<Args>(args)...));
	}

	template <class M>
	std::pair<iterator, bool> insert_or_assign(key_type const &k, M &&obj)
	{
		return insert_impl<InsertType::ASSIGN>(k, std::forward<M>(obj));
	}

	template <class M>
	std::pair<iterator, bool> insert_or_assign(key_type &&k, M &&obj)
	{
		return insert_impl<InsertType::ASSIGN>(std::move(k), std::forward<M>(obj));
	}

	template <class M>
	iterator insert_or_assign(const_iterator hint, key_type const &k, M &&obj)
	{
		return insert_impl<InsertType::ASSIGN>(hint, k, std::forward<M>(obj)).first;
	}

	template <class M>
	iterator insert_or_assign(const_iterator hint, key_type &&k, M &&obj)
	{
		return insert_impl<InsertType::ASSIGN>(hint, std::move(k), std::forward<M>(obj))
		    .first;
	}

	/*
	 * insert_or_replace_max
	 * If key_type k is not present in the map it is inserted with value obj.
	 * If key_type k is present then its value is update acording to
	 * 	max(current_value, obj).
	 */
	template <class M>
	std::pair<iterator, bool> insert_or_replace_max(key_type const &k, M &&obj)
	{
		return insert_impl<InsertType::MAX>(k, std::forward<M>(obj));
	}

	template <class M>
	std::pair<iterator, bool> insert_or_replace_max(key_type &&k, M &&obj)
	{
		return insert_impl<InsertType::MAX>(std::move(k), std::forward<M>(obj));
	}

	template <class M>
	iterator insert_or_replace_max(const_iterator hint, key_type const &k, M &&obj)
	{
		return insert_impl<InsertType::MAX>(hint, k, std::forward<M>(obj)).first;
	}

	template <class M>
	iterator insert_or_replace_max(const_iterator hint, key_type &&k, M &&obj)
	{
		return insert_impl<InsertType::MAX>(hint, std::move(k), std::forward<M>(obj)).first;
	}

	/*
	 * Assign value to Keys present in the given Range.
	 */
	template <class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	void assign_values(Range<T> const &range, mapped_type value)
	{
		assign_values(RangeSet<T>(range), value);
	}

	/*
	 * Assign value to Keys present in each Range of the RangeSet.
	 */
	template <class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	void assign_values(RangeSet<T> const &rangeSet, mapped_type value)
	{
		for (auto const &range : rangeSet) {
			auto lower = lower_bound(range.lower());
			auto upper = upper_bound(range.upper());
			for (; lower != upper; ++lower) {
				lower->setValue(value);
			}
		}
	}

	/*
	 * Increase the value by inc for all Keys present in Range
	 */
	template <class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	void increase_values(Range<T> const &range, mapped_type inc)
	{
		increase_values(RangeSet<T>(range), inc);
	}

	/*
	 * Increase the value by inc for all Keys in each Range of the RangeSet
	 */
	template <class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	void increase_values(RangeSet<T> const &rangeSet, mapped_type inc)
	{
		for (auto const &range : rangeSet) {
			auto lower = lower_bound(range.lower());
			auto upper = upper_bound(range.upper());
			for (; lower != upper; ++lower) {
				lower->increaseValue(inc);
			}
		}
	}

	/*
	 * Decrease the value by dec for all Keys present in Range
	 */
	template <class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	void decrease_values(Range<T> const &range, mapped_type dec)
	{
		decrease_values(RangeSet<T>(range), dec);
	}

	/*
	 * Decrease the value by dec for all Keys in each Range of the RangeSet
	 */
	template <class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	void decrease_values(RangeSet<T> const &rangeSet, mapped_type dec)
	{
		for (auto const &range : rangeSet) {
			auto lower = lower_bound(range.lower());
			auto upper = upper_bound(range.upper());
			for (; lower != upper; ++lower) {
				lower->decreaseValue(dec);
			}
		}
	}

	/*
	 * Increase or set the value of key
	 */
	std::pair<iterator, bool> increase_value(key_type key, mapped_type inc,
	                                         mapped_type init_value)
	{
		auto [it, inserted] = try_emplace(key, init_value);
		if (!inserted) {
			it->increaseValue(inc);
		}
		return {it, inserted};
	}

	/*
	 * Increase or set the value of key
	 */
	iterator increase_value(const_iterator hint, key_type key, mapped_type inc,
	                        mapped_type init_value)
	{
		auto [it, inserted] = try_emplace(hint, key, init_value);
		if (!inserted) {
			it->increaseValue(inc);
		}
		return it;
	}

	/*
	 * Decrease or set the value of key
	 */
	std::pair<iterator, bool> decrease_value(key_type key, mapped_type dec,
	                                         mapped_type init_value)
	{
		auto [it, inserted] = try_emplace(key, init_value);
		if (!inserted) {
			it->decreaseValue(dec);
		}
		return {it, inserted};
	}

	/*
	 * Decrease or set the value of key
	 */
	iterator decrease_value(const_iterator hint, key_type key, mapped_type dec,
	                        mapped_type init_value)
	{
		auto [it, inserted] = try_emplace(hint, key, init_value).first;
		if (!inserted) {
			it->decreaseValue(dec);
		}
		return it;
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

	size_type erase(key_type key)
	{
		if (iterator it = find(key); end() != it) {
			erase(it);
			return 1;
		}
		return 0;
	}

	/*
	 * Erases all Elements of range which fulfills the pairwise comparison
	 * comp(Element.value, value)
	 */
	template <class Compare, class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	size_type erase(Range<T> const &range, mapped_type value, Compare comp)
	{
		return erase(RangeSet<T>(range), value, comp);
	}

	template <class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	size_type erase(Range<T> const &range)
	{
		return erase(RangeSet<T>(range));
	}

	/*
	 * Erases all Elements of rangeSet which fulfills the pairwise comparison
	 * comp(Element.value, value)
	 */
	template <class Compare, class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	size_type erase(RangeSet<T> const &rangeSet, mapped_type value, Compare comp)
	{
		size_type old_size = size();
		for (auto const &range : rangeSet) {
			auto lower = lower_bound(range.lower());
			auto upper = upper_bound(range.upper());
			uint32_t c = 0;

			while (lower != upper) {
				if (!comp(lower->getValue(), value)) {
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
	size_type erase(RangeSet<T> const &rangeSet)
	{
		size_type old_size = size();
		for (auto const &range : rangeSet) {
			auto lower = lower_bound(range.lower());
			auto upper = upper_bound(range.upper());
			erase(lower, upper);
		}
		return old_size - size();
	}

	void swap(FlatMap &other) noexcept { std::swap(data_, other.data_); }

	// Lookup
	[[nodiscard]] mapped_type at(key_type key) const
	{
		if (auto it = find(key); end() != it) {
			return it->getValue();
		}
		throw std::out_of_range("Cannot find key " + std::to_string(key));
	}

	[[nodiscard]] mapped_type getValue(key_type key) const
	{
		auto it = find(key);
		return end() != it ? it->getValue() : 0;
	}

	[[nodiscard]] size_type count(key_type key) const { return contains(key) ? 1 : 0; }

	[[nodiscard]] iterator find(key_type key)
	{
		auto it = lower_bound(key);
		return end() != it && it->getKey() == key ? it : end();
	}

	[[nodiscard]] const_iterator find(key_type key) const
	{
		auto it = lower_bound(key);
		return end() != it && it->getKey() == key ? it : end();
	}

	[[nodiscard]] bool contains(key_type key) const { return end() != find(key); }

	[[nodiscard]] std::pair<iterator, iterator> equal_range(key_type key)
	{
		return equal_range(begin(), end(), key);
	}

	[[nodiscard]] std::pair<const_iterator, const_iterator> equal_range(key_type key) const
	{
		return equal_range(begin(), end(), key);
	}

	[[nodiscard]] iterator lower_bound(key_type key)
	{
		return lower_bound(begin(), end(), key);
	}

	[[nodiscard]] const_iterator lower_bound(key_type key) const
	{
		return lower_bound(begin(), end(), key);
	}

	[[nodiscard]] iterator upper_bound(key_type key)
	{
		return upper_bound(begin(), end(), key);
	}

	[[nodiscard]] const_iterator upper_bound(key_type key) const
	{
		return upper_bound(begin(), end(), key);
	}

	Element *getData()
	{
		if constexpr (FixedSize) {
			std::next(data_.data(), 1);
		} else {
			return std::next(data_, 1);
		}
	}

	Element const *getData() const
	{
		if constexpr (FixedSize) {
			std::next(data_.data(), 1);
		} else {
			return std::next(data_, 1);
		}
	}

	/*
	 * True if container contains any Key present in Range.
	 * I.e., the container does not have to contain the full range.
	 */
	template <class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	[[nodiscard]] bool containsAny(Range<T> const &range) const
	{
		return containsAny({range});
	}

	/*
	 * True if container contains any Key present in Range which value fulfills the
	 * binary comparison comp.  I.e., the container does not have to contain the full range.
	 */
	template <class Compare, class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	[[nodiscard]] bool containsAny(Range<T> const &range, mapped_type value,
	                               Compare comp) const
	{
		return containsAny({range}, value, comp);
	}

	/*
	 * True if container contains any Key present in any of the Ranges of the set.
	 * I.e., the container does not have to contain the full range.
	 */
	template <class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	[[nodiscard]] bool containsAny(RangeSet<T> const &rangeSet) const
	{
		// map contains empty set of ranges
		if (rangeSet.size() == size_type(0)) {
			return true;
		}

		if (size() < rangeSet.size()) {
			for (auto const &element : *this) {
				if (rangeSet.contains(element.getKey())) {
					return true;
				}
			}
		} else {
			for (auto const &range : rangeSet) {
				const_iterator lower = lower_bound(range.lower());
				if (lower != end() && lower->getKey() <= range.upper()) {
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
	[[nodiscard]] bool containsAny(RangeSet<T> const &rangeSet, mapped_type value,
	                               Compare comp) const
	{
		// map contains empty set of ranges
		if (rangeSet.size() == size_type(0)) {
			return true;
		}

		if (size() < rangeSet.size()) {
			for (auto const &element : *this) {
				if (rangeSet.contains(element.getKey()) && comp(element.getValue(), value)) {
					return true;
				}
			}
		} else {
			for (auto const &range : rangeSet) {
				auto lower = lower_bound(range.lower());
				auto upper = upper_bound(range.upper());
				for (; lower != upper; ++lower) {
					if (comp(lower->getValue(), value)) {
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
	[[nodiscard]] bool containsNone(Range<T> const &range) const
	{
		return containsNone({range});
	}

	/*
	 * Negated containsAny
	 */
	template <class Compare, class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	[[nodiscard]] bool containsNone(Range<T> const &range, mapped_type value,
	                                Compare comp) const
	{
		return !containsAny({range}, value, comp);
	}

	/*
	 * Negated containsAny
	 */
	template <class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	[[nodiscard]] bool containsNone(RangeSet<T> const &rangeSet) const
	{
		return !containsAny(rangeSet);
	}

	/*
	 * Negated containsAny
	 */
	template <class Compare, class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	[[nodiscard]] bool containsNone(RangeSet<T> const &rangeSet, mapped_type value,
	                                Compare comp) const
	{
		return !containsAny(rangeSet, value, comp);
	}

	/*
	 * True if container contains all Keys of the range and that comp(Element.value, value)
	 * is true for all such elements.
	 */
	template <class Compare, class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	[[nodiscard]] bool containsAll(Range<T> const &range, mapped_type value,
	                               Compare comp) const
	{
		return containsAll({range}, value, comp);
	}

	/*
	 * True if container contains all Keys of the range
	 */
	template <class Compare, class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	[[nodiscard]] bool containsAll(Range<T> const &range) const
	{
		return containsAll({range});
	}

	/*
	 * True if container contains all Keys of each range in the set and that
	 * comp(Element.value, value) is true for all such elements.
	 */
	template <class Compare, class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	[[nodiscard]] bool containsAll(RangeSet<T> const &rangeSet, mapped_type value,
	                               Compare comp) const
	{
		// map contains empty set of ranges
		if (rangeSet.size() == size_type(0)) {
			return true;
		}
		if (size() < rangeSet.size()) {
			return false;
		}
		typename RangeSet<uint32_t>::size_type checked_elems = 0;
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
				if (!comp(lower->getValue(), value)) {
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
	[[nodiscard]] bool containsAll(RangeSet<T> const &rangeSet) const
	{
		// map contains empty set of ranges
		if (rangeSet.size() == size_type(0)) {
			return true;
		}
		if (size() < rangeSet.size()) {
			return false;
		}
		typename RangeSet<uint32_t>::size_type checked_elems = 0;
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
	[[nodiscard]] std::pair<iterator, iterator> limits(Range<T> const &range, Compare comp)
	{
		return limits({range}, comp);
	}

	/*
	 * Return pair of iterators to limits (min max if e.g. less) according to comp
	 * Return (end(), end()) if none present.
	 */
	template <class Compare, class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	[[nodiscard]] std::pair<iterator, iterator> limits(RangeSet<T> const &rangeSet,
	                                                   Compare comp)
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
				if (ret_high == end() || !comp(lower->getValue(), ret_high->getValue())) {
					ret_high = lower;
				}
				if (ret_low == end() || comp(lower->getValue(), ret_low->getValue())) {
					ret_low = lower;
				}
			}
		}
		return {ret_low, ret_high};
	}

	std::ostream &writeData(std::ostream &out_stream) const
	{
		size_type num = size();
		out_stream.write(reinterpret_cast<char const *>(&num), sizeof(size_type));
		return out_stream.write(reinterpret_cast<char const *>(getData()),
		                        sizeof(Element) * num);
	}

	std::istream &readData(std::istream &in_stream)
	{
		size_type num;
		in_stream.read(reinterpret_cast<char *>(&num), sizeof(size_type));
		if (0 == num) {
			clear();
			return in_stream;
		}
		resize(num);
		return in_stream.read(reinterpret_cast<char *>(getData()), sizeof(Element) * num);
	}

	friend std::ostream &operator<<(std::ostream &os, FlatMap const &map)
	{
		if (!map.empty()) {
			std::copy(std::begin(map), std::prev(std::end(map)),
			          std::ostream_iterator<Element>(os, "; "));
			os << *std::prev(std::end(map));
		}
		return os;
	}

 protected:
	enum class InsertType { NORMAL, ASSIGN, MAX };

	template <InsertType T = InsertType::NORMAL>
	std::pair<iterator, bool> insert_impl(key_type key, mapped_type value)
	{
		if (empty()) {
			resize(1);
			data_[1].setKeyValue(key, value);
			return {begin(), true};
		}

		auto it = lower_bound(key);
		if (end() != it && it->getKey() == key) {
			// Label already exists
			if constexpr (InsertType::NORMAL == T) {
				// Do nothing
			} else if constexpr (InsertType::ASSIGN == T) {
				// Update value
				it->setValue(value);
			} else if constexpr (InsertType::MAX == T) {
				// Set value to max
				it->setValue(std::max(it->getValue(), value));
			}
			return {it, false};
		} else {
			auto index = std::distance(begin(), it);

			resize(size() + 1);

			it = std::next(begin(), index);

			std::move_backward(it, std::prev(end(), 1), end());

			data_[index + 1].setKeyValue(key, value);

			return {it, true};
		}
	}

	template <InsertType T = InsertType::NORMAL>
	std::pair<iterator, bool> insert_impl(const_iterator hint, key_type key,
	                                      mapped_type value)
	{
		if (empty()) {
			resize(1);
			data_[1].setKeyValue(key, value);
			return {begin(), true};
		}

		auto first = cbegin() != hint && std::prev(hint, 1)->getKey() < key ? hint : cbegin();
		auto last = cend() != hint && hint->getKey() >= key ? hint : cend();
		hint = lower_bound(first, last, key);

		auto index = std::distance(cbegin(), hint);

		if (cend() != hint && hint->getKey() == key) {
			auto it = std::next(begin(), index);
			// Label already exists
			if constexpr (InsertType::NORMAL == T) {
				// Do nothing
			} else if constexpr (InsertType::ASSIGN == T) {
				// Update value
				it->setValue(value);
			} else if constexpr (InsertType::MAX == T) {
				// Set value to max
				it->setValue(std::max(it->getValue(), value));
			}
			return {it, false};
		} else {
			resize(size() + 1);

			auto it = std::next(begin(), index);

			std::move_backward(it, std::prev(end(), 1), end());

			data_[index + 1].setKeyValue(key, value);

			return {it, true};
		}
	}

	template <InsertType InsertT = InsertType::NORMAL, class T>
	void insert_impl(std::vector<T> &vec)
	{
		// Sort based on key and such highest value first if same key
		std::sort(std::begin(vec), std::end(vec));

		// Erase duplicate keys, saving the highest value for each key
		auto r_last = std::unique(
		    std::rbegin(vec), std::rend(vec),
		    [](auto const &v1, auto const &v2) { return v1.getKey() == v2.getKey(); });
		vec.erase(std::begin(vec), r_last.base());

		if (empty()) {
			// Optimized insert
			setOrdered(std::begin(vec), std::end(vec));
		} else {
			// Normal insert
			auto hint = begin();
			for (auto const &elem : vec) {
				hint = insert<InsertT>(hint, elem.getKey(), elem.getValue()).first;
			}
		}
	}

	template <InsertType InsertT = InsertType::NORMAL, class ExecutionPolicy, class T,
	          typename = std::enable_if_t<
	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
	void insert_impl(ExecutionPolicy policy, std::vector<T> &vec)
	{
		// Sort based on key and such highest value first if same key
		std::sort(policy, std::begin(vec), std::end(vec));

		// Erase duplicate keys, saving the highest value for each key
		auto r_last = std::unique(
		    policy, std::rbegin(vec), std::rend(vec),
		    [](auto const &v1, auto const &v2) { return v1.getKey() == v2.getKey(); });
		vec.erase(std::begin(vec), r_last.base());

		if (empty()) {
			// Optimized insert
			setOrdered(policy, std::begin(vec), std::end(vec));
		} else {
			// Normal insert
			auto hint = begin();
			for (auto const &elem : vec) {
				hint = insert<InsertT>(hint, elem.getKey(), elem.getValue()).first;
			}
		}
	}

	static iterator lower_bound(iterator first, iterator last, key_type key)
	{
		return std::lower_bound(first, last, Element(key, 0));
	}

	static const_iterator lower_bound(const_iterator first, const_iterator last,
	                                  key_type key)
	{
		return std::lower_bound(first, last, Element(key, 0));
	}

	static iterator upper_bound(iterator first, iterator last, key_type key)
	{
		return std::upper_bound(first, last, Element(key, ValueMask));
	}

	static const_iterator upper_bound(const_iterator first, const_iterator last,
	                                  key_type key)
	{
		return std::upper_bound(first, last, Element(key, ValueMask));
	}

	static std::pair<iterator, iterator> equal_range(iterator first, iterator last,
	                                                 key_type key)
	{
		auto it = lower_bound(first, last, key);
		if (last == it || it->getKey() != key) {
			return {it, it};
		} else {
			return {it, std::next(it)};
		}
	}

	static std::pair<const_iterator, const_iterator> equal_range(const_iterator first,
	                                                             const_iterator last,
	                                                             key_type key)
	{
		auto it = lower_bound(first, last, key);
		if (last == it || it->getKey() != key) {
			return {it, it};
		} else {
			return {it, std::next(it)};
		}
	}

	void resize(size_type const new_size)
	{
		if constexpr (FixedSize) {
			if (data_.size() <= new_size) {
				throw std::length_error();
			}
		} else {
			Element *ptr_new =
			    static_cast<Element *>(realloc(data_, (new_size + 1) * sizeof(Element)));

			if (!ptr_new) {
				throw std::bad_alloc();
			}

			data_ = ptr_new;
		}

		data_[0].setKey(new_size);  // Special
	}

 private:
	std::conditional_t<FixedSize, std::array<DataType, FixedSize + 1>, Element *> data_{};
};

template <typename DataType, size_t ValueWidth>
bool operator==(FlatMap<DataType, ValueWidth> const &lhs,
                FlatMap<DataType, ValueWidth> const &rhs)
{
	return std::equal(std::begin(lhs), std::end(lhs), std::begin(rhs), std::end(rhs));
}

template <typename DataType, size_t ValueWidth>
bool operator!=(FlatMap<DataType, ValueWidth> const &lhs,
                FlatMap<DataType, ValueWidth> const &rhs)
{
	return !(lhs == rhs);
}

template <typename DataType, size_t ValueWidth>
void swap(FlatMap<DataType, ValueWidth> &lhs,
          FlatMap<DataType, ValueWidth> &rhs) noexcept(noexcept(lhs.swap(rhs)))
{
	lhs.swap(rhs);
}

template <typename DataType, size_t ValueWidth, class Pred>
typename FlatMap<DataType, ValueWidth>::size_type erase_if(
    FlatMap<DataType, ValueWidth> &c, Pred pred)
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

}  // namespace ufo::container

#endif  // UFO_CONTAINER_FLAT_MAP_H