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
/*!
 * Associative ordered container using a data-coherent implementation.
 * LabelValuePairs of the container are stored using a single unsigned type
 * where the most significant bits represent the key and the least significant the value.
 * The number of bits allocated for the key and value is custom for each
 * container instance, e.g., if DataType = uint16_t and ValueWidth = 1
 * the key will be 15 bits and the value 1 bit.
 */
template <std::size_t NumLabelBits, std::size_t NumValueBits>
class Semantics
{
 private:
	static_assert(0 != NumLabelBits && 64 >= NumLabelBits && 0 != NumValueBits &&
	                  64 >= NumValueBits,
	              "'NumLabelBits' and 'NumValueBits' have to be in the range [1..64]");

	static constexpr std::size_t TotalBits = NumLabelBits + NumValueBits;

	using data_type = std::conditional_t<
	    8 >= TotalBits, uint8_t,
	    std::conditional_t<
	        16 >= TotalBits,
	        std::conditional_t<8 < NumLabelBits || 8 < NumValueBits, uint16_t,
	                           std::pair<uint8_t, uint8_t>>,
	        std::conditional_t<
	            24 >= TotalBits,
	            std::conditional_t<
	                8 >= NumLabelBits && 16 >= NumValueBits, std::pair<uint8_t, uint16_t>,
	                std::conditional_t<
	                    16 >= NumLabelBits && 8 >= NumValueBits,
	                    std::pair<uint16_t, uint8_t>,
	                    std::conditional_t<16 >= NumLabelBits && 16 >= NumValueBits,
	                                       std::pair<uint16_t, uint16_t>, uint32_t>>>,
	            std::conditional_t<
	                32 >= TotalBits,
	                std::conditional_t<16 >= NumLabelBits && 16 >= NumValueBits,
	                                   std::pair<uint16_t, uint16_t>, uint32_t>,
	                std::conditional_t<
	                    40 >= TotalBits,
	                    std::conditional_t<
	                        8 >= NumLabelBits && 32 >= NumValueBits,
	                        std::pair<uint8_t, uint32_t>,
	                        std::conditional_t<32 >= NumLabelBits && 8 >= NumValueBits,
	                                           std::pair<uint32_t, uint8_t>, uint64_t>>,
	                    std::conditional_t<
	                        48 >= TotalBits,
	                        std::conditional_t<
	                            16 >= NumLabelBits && 32 >= NumValueBits,
	                            std::pair<uint16_t, uint32_t>,
	                            std::conditional_t<32 >= NumLabelBits && 16 >= NumValueBits,
	                                               std::pair<uint32_t, uint16_t>,
	                                               uint64_t>>,
	                        std::conditional_t<
	                            64 >= TotalBits,
	                            std::conditional_t<32 >= NumLabelBits && 32 >= NumValueBits,
	                                               std::pair<uint32_t, uint32_t>, uint64_t>,
	                            //  Everything below is above 64 bits
	                            std::conditional_t<
	                                8 >= NumLabelBits, std::pair<uint8_t, uint64_t>,
	                                std::conditional_t<
	                                    8 >= NumValueBits, std::pair<uint64_t, uint8_t>,
	                                    std::conditional_t<
	                                        16 >= NumLabelBits,
	                                        std::pair<uint16_t, uint64_t>,
	                                        std::conditional_t<
	                                            16 >= NumValueBits,
	                                            std::pair<uint64_t, uint16_t>,
	                                            std::conditional_t<
	                                                32 >= NumLabelBits,
	                                                std::pair<uint32_t, uint64_t>,
	                                                std::conditional_t<
	                                                    32 >= NumValueBits,
	                                                    std::pair<uint64_t, uint32_t>,
	                                                    std::pair<uint64_t,
	                                                              uint64_t>>>>>>>>>>>>>>;

	static constexpr bool IsPair = util::is_pair_v<data_type>;

 public:
	//  Tags
	using label_type =
	    std::conditional_t<IsPair, typename data_type::first_type, data_type>;
	using value_type =
	    std::conditional_t<IsPair, typename data_type::second_type, data_type>;

 private:
	static constexpr label_type LabelMask = []() {
		if constexpr (IsPair) {
			return ~label_type(0);
		} else {
			return ~label_type(0) << NumValueBits;
		};
	}();
	static constexpr value_type ValueMask = []() {
		if constexpr (IsPair) {
			return ~value_type(0);
		} else {
			return ~LabelMask;
		}
	}();

	static constexpr label_type LabelMax = []() {
		if constexpr (IsPair) {
			return LabelMask;
		} else {
			return LabelMask >> NumValueBits;
		}
	}();
	static constexpr value_type ValueMax = ValueMask;

 public:
	//
	// Label value pair
	//

	struct LabelValuePair {
	 public:
		constexpr LabelValuePair() = default;

		template <std::size_t NLB = NumLabelBits, std::size_t NVB = NumValueBits,
		          std::enable_if_t<IsPair, bool> = true>
		constexpr LabelValuePair(label_type label, value_type value) : data_(label, value)
		{
		}

		template <std::size_t NLB = NumLabelBits, std::size_t NVB = NumValueBits,
		          std::enable_if_t<!IsPair, bool> = true>
		constexpr LabelValuePair(label_type label, value_type value)
		    : data_((label << NumValueBits) | (value & ValueMask))
		{
		}

		[[nodiscard]] constexpr label_type getLabel() const noexcept
		{
			if constexpr (IsPair) {
				return data_.first;
			} else {
				return data_ >> NumValueBits;
			}
		}

		[[nodiscard]] constexpr value_type getValue() const noexcept
		{
			if constexpr (IsPair) {
				return data_.second;
			} else {
				return data_ & ValueMask;
			}
		}

		constexpr void setValue(value_type value) noexcept
		{
			if constexpr (IsPair) {
				data_.second = value;
			} else {
				data_ = (data_ & LabelMask) | (value & ValueMask);
			}
		}

		constexpr void increaseValue(value_type inc) noexcept
		{
			if constexpr (IsPair) {
				data_.second = ValueMax - data_.second > inc ? data_.second + inc : ValueMax;
			} else {
				value_type cur = getValue();
				data_ = (data_ & LabelMask) | (ValueMax - cur > inc ? cur + inc : ValueMax);
			}
		}

		constexpr void decreaseValue(value_type dec) noexcept
		{
			if constexpr (IsPair) {
				data_.second -= std::min(data_.second, dec);
			} else {
				value_type cur = getValue();
				data_ = (data_ & LabelMask) | (cur - std::min(cur, dec));
			}
		}

		constexpr bool operator==(LabelValuePair rhs) const noexcept
		{
			return rhs.data_ == data_;
		}

		constexpr bool operator!=(LabelValuePair rhs) const noexcept
		{
			return rhs.data_ != data_;
		}

		constexpr bool operator<(LabelValuePair rhs) const noexcept
		{
			return data_ < rhs.data_;
		}

		constexpr bool operator<=(LabelValuePair rhs) const noexcept
		{
			return data_ <= rhs.data_;
		}

		constexpr bool operator>(LabelValuePair rhs) const noexcept { return rhs < *this; }

		constexpr bool operator>=(LabelValuePair rhs) const noexcept { return rhs <= *this; }

		friend std::ostream &operator<<(std::ostream &os, LabelValuePair element)
		{
			// Note: '+' to apply arithmetic promotion.
			os << +element.getLabel() << ": " << +element.getValue();
			return os;
		}

	 private:
		constexpr void setLabel(label_type label) noexcept
		{
			if constexpr (IsPair) {
				data_.first = label;
			} else {
				data_ = (label << NumValueBits) | (data_ & ValueMask);
			}
		}

		constexpr void setLabelValue(label_type label, value_type value) noexcept
		{
			if constexpr (IsPair) {
				data_.first = label;
				data_.second = value;
			} else {
				data_ = (label << NumValueBits) | (value & ValueMask);
			}
		}

		constexpr void setData(data_type data) noexcept { data_ = data; }

		constexpr data_type getData() const noexcept { return data_; }

	 private:
		data_type data_{};
		friend class Semantics;
	};

	// Tags continue
	using size_type = label_type;
	using difference_type = std::ptrdiff_t;
	using reference = LabelValuePair &;
	using const_reference = LabelValuePair const &;
	// using pointer = typename std::allocator_traits<Allocator>::pointer;
	// using const_pointer = typename
	// std::allocator_traits<Allocator>::const_pointer;
	using iterator = LabelValuePair *;
	using const_iterator = LabelValuePair const *;
	using reverse_iterator = std::reverse_iterator<iterator>;
	using const_reverse_iterator = std::reverse_iterator<const_iterator>;

 public:
	[[nodiscard]] static constexpr std::size_t getNumLabelBits() noexcept
	{
		return NumLabelBits;
	}

	[[nodiscard]] static constexpr std::size_t getNumValueBits() noexcept
	{
		return NumValueBits;
	}

	//
	// Constructors
	//

	constexpr Semantics() = default;

	template <class InputIt>
	Semantics(InputIt first, InputIt last)
	{
		insert(first, last);
	}

	Semantics(Semantics const &other) { *this = other; }

	Semantics(Semantics &&other) noexcept = default;

	Semantics(std::initializer_list<value_type> init) { insert(init); }

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

	Semantics &operator=(std::initializer_list<value_type> ilist)
	{
		insert(ilist);
		return *this;
	}

	//
	// Iterators
	//

	iterator begin() noexcept { return empty() ? nullptr : std::next(data_.get()); }

	const_iterator begin() const noexcept
	{
		return empty() ? nullptr : std::next(data_.get());
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

	// TODO: Implement reverse iterators

	//
	// Capacity
	//

	[[nodiscard]] bool empty() const noexcept { return 0 == size(); }

	[[nodiscard]] size_type size() const noexcept
	{
		return data_ ? data_[0].getLabel() : 0;
	}

	[[nodiscard]] size_type allocSize() const noexcept
	{
		return empty() ? 0 : data_[0].getLabel() + 1;
	}

	[[nodiscard]] constexpr size_type max_size() const noexcept { return LabelMax; }

	//
	// Modifiers
	//

	void clear() noexcept { data_.reset(); }

	template <class InputIt>
	void setCombine(InputIt first, InputIt last, SemanticLabelPropagation const &prop)
	{
		switch (prop.getDefaultPropCriteria()) {
			case PropagationCriteria::MAX:
				setCombine<InputIt, true>(first, last, prop);
				return;
			default:
				setCombine<InputIt, false>(first, last, prop);
		}
	}

	std::pair<iterator, bool> insert(value_type const &value)
	{
		return insert_impl<InsertType::NORMAL>(value.getLabel(), value.getValue());
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

	void insert(std::initializer_list<value_type> ilist)
	{
		std::vector<value_type> temp(ilist);
		insert_impl(temp);
	}

	template <class... Args>
	std::pair<iterator, bool> emplace(Args &&...args)
	{
		return insert_impl(std::forward<Args>(args)...);
	}

	template <class... Args>
	std::pair<iterator, bool> try_emplace(label_type const &k, Args &&...args)
	{
		return insert(value_type(k, std::forward<Args>(args)...));
	}

	template <class... Args>
	std::pair<iterator, bool> try_emplace(label_type &&k, Args &&...args)
	{
		return insert(value_type(std::move(k), std::forward<Args>(args)...));
	}

	template <class... Args>
	std::pair<iterator, bool> try_emplace(const_iterator hint, label_type const &k,
	                                      Args &&...args)
	{
		return insert(hint, value_type(k, std::forward<Args>(args)...));
	}

	template <class... Args>
	std::pair<iterator, bool> try_emplace(const_iterator hint, label_type &&k,
	                                      Args &&...args)
	{
		return insert(hint, value_type(std::move(k), std::forward<Args>(args)...));
	}

	template <class M>
	std::pair<iterator, bool> insert_or_assign(label_type const &k, M &&obj)
	{
		return insert_impl<InsertType::ASSIGN>(k, std::forward<M>(obj));
	}

	template <class M>
	std::pair<iterator, bool> insert_or_assign(label_type &&k, M &&obj)
	{
		return insert_impl<InsertType::ASSIGN>(std::move(k), std::forward<M>(obj));
	}

	template <class M>
	iterator insert_or_assign(const_iterator hint, label_type const &k, M &&obj)
	{
		return insert_impl<InsertType::ASSIGN>(hint, k, std::forward<M>(obj)).first;
	}

	template <class M>
	iterator insert_or_assign(const_iterator hint, label_type &&k, M &&obj)
	{
		return insert_impl<InsertType::ASSIGN>(hint, std::move(k), std::forward<M>(obj))
		    .first;
	}

	/*
	 * insert_or_replace_max
	 * If label_type k is not present in the map it is inserted with value obj.
	 * If label_type k is present then its value is update acording to
	 * 	max(current_value, obj).
	 */
	template <class M>
	std::pair<iterator, bool> insert_or_replace_max(label_type const &k, M &&obj)
	{
		return insert_impl<InsertType::MAX>(k, std::forward<M>(obj));
	}

	template <class M>
	std::pair<iterator, bool> insert_or_replace_max(label_type &&k, M &&obj)
	{
		return insert_impl<InsertType::MAX>(std::move(k), std::forward<M>(obj));
	}

	template <class M>
	iterator insert_or_replace_max(const_iterator hint, label_type const &k, M &&obj)
	{
		return insert_impl<InsertType::MAX>(hint, k, std::forward<M>(obj)).first;
	}

	template <class M>
	iterator insert_or_replace_max(const_iterator hint, label_type &&k, M &&obj)
	{
		return insert_impl<InsertType::MAX>(hint, std::move(k), std::forward<M>(obj)).first;
	}

	/*
	 * Assign value to Keys present in the given Range.
	 */
	template <class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	void assign_values(container::Range<T> const &range, value_type value)
	{
		assign_values(container::RangeSet<T>(range), value);
	}

	/*
	 * Assign value to Keys present in each Range of the RangeSet.
	 */
	template <class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	void assign_values(container::RangeSet<T> const &rangeSet, value_type value)
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
	void increase_values(container::Range<T> const &range, value_type inc)
	{
		increase_values(container::RangeSet<T>(range), inc);
	}

	/*
	 * Increase the value by inc for all Keys in each Range of the RangeSet
	 */
	template <class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	void increase_values(container::RangeSet<T> const &rangeSet, value_type inc)
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
	void decrease_values(container::Range<T> const &range, value_type dec)
	{
		decrease_values(container::RangeSet<T>(range), dec);
	}

	/*
	 * Decrease the value by dec for all Keys in each Range of the RangeSet
	 */
	template <class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	void decrease_values(container::RangeSet<T> const &rangeSet, value_type dec)
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
	std::pair<iterator, bool> increase_value(label_type key, value_type inc,
	                                         value_type init_value)
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
	iterator increase_value(const_iterator hint, label_type key, value_type inc,
	                        value_type init_value)
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
	std::pair<iterator, bool> decrease_value(label_type key, value_type dec,
	                                         value_type init_value)
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
	iterator decrease_value(const_iterator hint, label_type key, value_type dec,
	                        value_type init_value)
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

	size_type erase(label_type key)
	{
		if (iterator it = find(key); end() != it) {
			erase(it);
			return 1;
		}
		return 0;
	}

	/*
	 * Erases all LabelValuePairs of range which fulfills the pairwise comparison
	 * comp(LabelValuePair.value, value)
	 */
	template <class Compare, class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	size_type erase(container::Range<T> const &range, value_type value, Compare comp)
	{
		return erase(container::RangeSet<T>(range), value, comp);
	}

	template <class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	size_type erase(container::Range<T> const &range)
	{
		return erase(container::RangeSet<T>(range));
	}

	/*
	 * Erases all LabelValuePairs of rangeSet which fulfills the pairwise comparison
	 * comp(LabelValuePair.value, value)
	 */
	template <class Compare, class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	size_type erase(container::RangeSet<T> const &rangeSet, value_type value, Compare comp)
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
	[[nodiscard]] value_type at(label_type key) const
	{
		if (auto it = find(key); end() != it) {
			return it->getValue();
		}
		throw std::out_of_range("Cannot find key " + std::to_string(key));
	}

	[[nodiscard]] value_type getValue(label_type key) const
	{
		auto it = find(key);
		return end() != it ? it->getValue() : 0;
	}

	[[nodiscard]] size_type count(label_type key) const { return contains(key) ? 1 : 0; }

	[[nodiscard]] iterator find(label_type key)
	{
		auto it = lower_bound(key);
		return end() != it && it->getLabel() == key ? it : end();
	}

	[[nodiscard]] const_iterator find(label_type key) const
	{
		auto it = lower_bound(key);
		return end() != it && it->getLabel() == key ? it : end();
	}

	[[nodiscard]] bool contains(label_type key) const { return end() != find(key); }

	[[nodiscard]] std::pair<iterator, iterator> equal_range(label_type key)
	{
		return equal_range(begin(), end(), key);
	}

	[[nodiscard]] std::pair<const_iterator, const_iterator> equal_range(
	    label_type key) const
	{
		return equal_range(begin(), end(), key);
	}

	[[nodiscard]] iterator lower_bound(label_type key)
	{
		return lower_bound(begin(), end(), key);
	}

	[[nodiscard]] const_iterator lower_bound(label_type key) const
	{
		return lower_bound(begin(), end(), key);
	}

	[[nodiscard]] iterator upper_bound(label_type key)
	{
		return upper_bound(begin(), end(), key);
	}

	[[nodiscard]] const_iterator upper_bound(label_type key) const
	{
		return upper_bound(begin(), end(), key);
	}

	LabelValuePair *getData() { return std::next(data_.get(), 1); }

	LabelValuePair const *getData() const { return std::next(data_.get(), 1); }

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
	[[nodiscard]] bool containsAny(container::Range<T> const &range, value_type value,
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
				if (rangeSet.contains(element.getLabel())) {
					return true;
				}
			}
		} else {
			for (auto const &range : rangeSet) {
				const_iterator lower = lower_bound(range.lower());
				if (lower != end() && lower->getLabel() <= range.upper()) {
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
	[[nodiscard]] bool containsAny(container::RangeSet<T> const &rangeSet, value_type value,
	                               Compare comp) const
	{
		// map contains empty set of ranges
		if (rangeSet.size() == size_type(0)) {
			return true;
		}

		if (size() < rangeSet.size()) {
			for (auto const &element : *this) {
				if (rangeSet.contains(element.getLabel()) && comp(element.getValue(), value)) {
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
	[[nodiscard]] bool containsNone(container::Range<T> const &range) const
	{
		return containsNone({range});
	}

	/*
	 * Negated containsAny
	 */
	template <class Compare, class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	[[nodiscard]] bool containsNone(container::Range<T> const &range, value_type value,
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
	[[nodiscard]] bool containsNone(container::RangeSet<T> const &rangeSet,
	                                value_type value, Compare comp) const
	{
		return !containsAny(rangeSet, value, comp);
	}

	/*
	 * True if container contains all Keys of the range and that comp(LabelValuePair.value,
	 * value) is true for all such elements.
	 */
	template <class Compare, class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	[[nodiscard]] bool containsAll(container::Range<T> const &range, value_type value,
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
	 * comp(LabelValuePair.value, value) is true for all such elements.
	 */
	template <class Compare, class T, class = std::enable_if_t<std::is_unsigned_v<T>>>
	[[nodiscard]] bool containsAll(container::RangeSet<T> const &rangeSet, value_type value,
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

	std::ostream &write(std::ostream &out_stream) const
	{
		size_type num = size();
		out_stream.write(reinterpret_cast<char const *>(&num), sizeof(size_type));
		return out_stream.write(reinterpret_cast<char const *>(getData()),
		                        sizeof(LabelValuePair) * num);
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
		return in_stream.read(reinterpret_cast<char *>(getData()),
		                      sizeof(LabelValuePair) * num);
	}

	friend std::ostream &operator<<(std::ostream &os, Semantics const &map)
	{
		if (!map.empty()) {
			std::copy(std::begin(map), std::prev(std::end(map)),
			          std::ostream_iterator<LabelValuePair>(os, "; "));
			os << *std::prev(std::end(map));
		}
		return os;
	}

 protected:
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
			std::inplace_merge(begin(), it_beg, it_end);
			it_beg = it_end;
		}

		if (!prop.empty() || PropagationCriteria::MEAN == prop.getDefaultPropCriteria()) {
			for (auto it = begin(), it_end = end(); it != it_end;) {
				auto it_2 = std::find_if_not(
				    std::next(it), it_end,
				    [l = it->getLabel()](auto const v) { return l == v.getLabel(); });

				if constexpr (Max) {
					// FIXME: Improve
					std::prev(it_2)->setValue(prop.getValue(it, it_2));
				} else {
					it->setValue(prop.getValue(it, it_2));
				}

				it = it_2;
			}
		}

		// Remove duplicates (if duplicate, save correct value)
		if constexpr (Max) {
			auto new_beg = std::unique(rbegin(), rend(), [](auto const a, auto const b) {
				               return a.getLabel() == b.getLabel();
			               }).base();
			if (begin() != new_beg) {
				std::copy(new_beg, end(), begin());
				resize(std::distance(new_beg, end()));
			}
		} else {
			auto new_end = std::unique(begin(), end(), [](auto const a, auto const b) {
				return a.getLabel() == b.getLabel();
			});
			if (end() != new_end) {
				resize(std::distance(begin(), new_end));
			}
		}
	}

	enum class InsertType { NORMAL, ASSIGN, MAX };

	template <InsertType T = InsertType::NORMAL>
	std::pair<iterator, bool> insert_impl(label_type key, value_type value)
	{
		if (empty()) {
			resize(1);
			data_[1].setLabelValue(key, value);
			return {begin(), true};
		}

		auto it = lower_bound(key);
		if (end() != it && it->getLabel() == key) {
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

			data_[index + 1].setLabelValue(key, value);

			return {it, true};
		}
	}

	template <InsertType T = InsertType::NORMAL>
	std::pair<iterator, bool> insert_impl(const_iterator hint, label_type key,
	                                      value_type value)
	{
		if (empty()) {
			resize(1);
			data_[1].setLabelValue(key, value);
			return {begin(), true};
		}

		auto first =
		    cbegin() != hint && std::prev(hint, 1)->getLabel() < key ? hint : cbegin();
		auto last = cend() != hint && hint->getLabel() >= key ? hint : cend();
		hint = lower_bound(first, last, key);

		auto index = std::distance(cbegin(), hint);

		if (cend() != hint && hint->getLabel() == key) {
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

			data_[index + 1].setLabelValue(key, value);

			return {it, true};
		}
	}

	template <InsertType InsertT = InsertType::NORMAL, class T>
	void insert_impl(std::vector<T> &vec)
	{
		// Sort based on key and such highest value first if same key
		std::sort(std::begin(vec), std::end(vec));

		// Erase duplicate keys, saving the highest value for each key
		auto r_last = std::unique(std::rbegin(vec), std::rend(vec), [](auto v1, auto v2) {
			return v1.getLabel() == v2.getLabel();
		});
		// vec.erase(std::begin(vec), r_last.base());

		if (empty()) {
			// Optimized insert
			resize(std::distance(r_last.base(), std::end(vec)));
			std::copy(r_last.base(), std::end(vec), begin());
		} else {
			// Normal insert
			auto hint = begin();
			for (auto it = r_last.base(); it != std::end(vec); ++it) {
				hint = insert<InsertT>(hint, it->getLabel(), it->getValue()).first;
			}
			// for (auto const &elem : vec) {
			// 	hint = insert<InsertT>(hint, elem.getLabel(), elem.getValue()).first;
			// }
		}
	}

	static iterator lower_bound(iterator first, iterator last, label_type key)
	{
		return std::lower_bound(first, last, LabelValuePair(key, 0));
	}

	static const_iterator lower_bound(const_iterator first, const_iterator last,
	                                  label_type key)
	{
		return std::lower_bound(first, last, LabelValuePair(key, 0));
	}

	static iterator upper_bound(iterator first, iterator last, label_type key)
	{
		return std::upper_bound(first, last, LabelValuePair(key, ValueMask));
	}

	static const_iterator upper_bound(const_iterator first, const_iterator last,
	                                  label_type key)
	{
		return std::upper_bound(first, last, LabelValuePair(key, ValueMask));
	}

	static std::pair<iterator, iterator> equal_range(iterator first, iterator last,
	                                                 label_type key)
	{
		auto it = lower_bound(first, last, key);
		if (last == it || it->getLabel() != key) {
			return {it, it};
		} else {
			return {it, std::next(it)};
		}
	}

	static std::pair<const_iterator, const_iterator> equal_range(const_iterator first,
	                                                             const_iterator last,
	                                                             label_type key)
	{
		auto it = lower_bound(first, last, key);
		if (last == it || it->getLabel() != key) {
			return {it, it};
		} else {
			return {it, std::next(it)};
		}
	}

	void resize(size_type const new_size)
	{
		LabelValuePair *ptr_cur = data_.release();

		LabelValuePair *ptr_new = static_cast<LabelValuePair *>(
		    realloc(ptr_cur, (new_size + 1) * sizeof(LabelValuePair)));

		if (!ptr_new) {
			data_.reset(ptr_cur);
			throw std::bad_alloc();
		}

		data_.reset(ptr_new);

		data_[0].setLabel(new_size);  // Special
	}

 private:
	std::unique_ptr<value_type[]> data_;

	template <class Derived, class LeafNode, class InnerNode>
	friend class SemanticMapBase;
};

template <typename DataType, std::size_t ValueWidth>
bool operator==(Semantics<DataType, ValueWidth> const &lhs,
                Semantics<DataType, ValueWidth> const &rhs)
{
	return std::equal(std::begin(lhs), std::end(lhs), std::begin(rhs), std::end(rhs));
}

template <typename DataType, std::size_t ValueWidth>
bool operator!=(Semantics<DataType, ValueWidth> const &lhs,
                Semantics<DataType, ValueWidth> const &rhs)
{
	return !(lhs == rhs);
}

template <typename DataType, std::size_t ValueWidth>
void swap(Semantics<DataType, ValueWidth> &lhs,
          Semantics<DataType, ValueWidth> &rhs) noexcept(noexcept(lhs.swap(rhs)))
{
	lhs.swap(rhs);
}

template <typename DataType, std::size_t ValueWidth, class Pred>
typename Semantics<DataType, ValueWidth>::size_type erase_if(
    Semantics<DataType, ValueWidth> &c, Pred pred)
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