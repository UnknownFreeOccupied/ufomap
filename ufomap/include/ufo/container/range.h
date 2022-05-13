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

#ifndef UFO_CONTAINER_RANGE_H
#define UFO_CONTAINER_RANGE_H

// STL
#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <iostream>
#include <iterator>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <vector>

/*
 * Range, RangeSet, and RangeMap implementation
 *
 * Functions present in the respective STL implementation adhere to their
 * interface (std::set, std::map).
 *
 * A Range takes an arithmetic type T and defines an inclusive interval [T1, T2]
 *
 * A RangeSet is a container of unique ranges. If a Range is inserted into the
 * container and is adjacent or overlapping with a Range already present the two
 * are joined: e.g., [4,10] and [11, 40] becomes [4, 40].
 *
 * A RangeMap is a container where each Range (key) has an associated value.
 * Adjacent or overlapping ranges with the same value are joined.
 * Ranges within the container are not allowed to overlap, e.g., if
 * the range and value [0,10]:4 is present and [5,15]:6 is inserted
 * the resulting container will contain: [0,10]:4, [11,15]:6
 * There is support for overwriting, see insert_or_assign
 */

namespace ufo::container
{

///////////////////////////////////////////////////////////////////////////
////////////////////////////////// Range //////////////////////////////////
///////////////////////////////////////////////////////////////////////////

template <typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
class Range
{
 public:
	struct Comparator {
		using range_t = std::pair<T, T>;
		using is_transparent = std::true_type;
		[[nodiscard]] constexpr bool operator()(Range lhs, range_t rhs) const noexcept
		{
			return lhs.upper() < rhs.first;
		}
		[[nodiscard]] constexpr bool operator()(range_t lhs, Range rhs) const noexcept
		{
			return lhs.second < rhs.upper();
		}
	};

 public:
	// Tags
	using value_type = T;

	Range() = default;

	Range(T value) : lower_(value), upper_(value) {}

	Range(T lower, T upper) : lower_(std::min(lower, upper)), upper_(std::max(lower, upper))
	{
	}

	template <typename V, typename = std::enable_if_t<std::is_arithmetic_v<V>>>
	Range(V value) : Range(value, value)
	{
	}

	template <typename V, typename = std::enable_if_t<std::is_arithmetic_v<V>>>
	Range(V lower, V upper)
	{
		setRange(lower, upper);
	}

	Range(Range const &other) = default;

	template <typename T2>
	Range(Range<T2> other) : Range(other.lower(), other.upper())
	{
	}

	Range(Range &&other) = default;

	Range &operator=(Range const &rhs) = default;

	template <typename T2>
	Range &operator=(Range<T2> rhs)
	{
		setRange(rhs.lower(), rhs.upper());
		return *this;
	}

	Range &operator=(Range &&rhs) = default;

	constexpr void setValue(T value)
	{
		lower_ = value;
		upper_ = value;
	}

	constexpr void setRange(T lower, T upper)
	{
		std::tie(lower_, upper_) = std::minmax(lower, upper);
	}

	template <typename V, typename = std::enable_if_t<std::is_arithmetic_v<V>>>
	constexpr void setValue(V value)
	{
		setRange(value, value);
	}

	template <typename V, typename = std::enable_if_t<std::is_arithmetic_v<V>>>
	constexpr void setRange(V lower, V upper)
	{
		// FIXME: Check that this is correct
		auto [min, max] = std::minmax(lower, upper);
		auto t_min = std::numeric_limits<T>::lowest();
		auto t_max = std::numeric_limits<T>::max();

		// Check for error
		if constexpr (std::is_floating_point_v<T> || std::is_floating_point_v<V> ||
		              std::is_signed_v<T> == std::is_signed_v<V>) {
			if (max < t_min || t_max < min) {
				// FIXME: Fix better message
				throw std::range_error("Outside representable range.");
			}
		} else if constexpr (std::is_unsigned_v<T>) {
			if (min < 0 || t_max < min) {
				// FIXME: Fix better message
				throw std::range_error("Outside representable range.");
			}
		} else if (t_max < min) {
			// FIXME: Fix better message
			throw std::range_error("Outside representable range.");
		}

		if constexpr (std::is_floating_point_v<T>) {
			lower_ = t_min <= min ? min : t_min;
			upper_ = t_max >= max ? max : t_max;
		} else if constexpr (std::is_floating_point_v<V>) {
			auto v_min = std::floor(min);
			auto v_max = std::ceil(max);
			lower_ = t_min <= v_min ? v_min : t_min;
			upper_ = t_max >= v_max ? v_max : t_max;
		} else if constexpr (std::is_signed_v<T> == std::is_signed_v<V>) {
			lower_ = t_min <= min ? min : t_min;
			upper_ = t_max >= max ? max : t_max;
		} else if constexpr (std::is_unsigned_v<T>) {
			lower_ = 0 > min ? 0 : (t_max >= min ? min : t_max);
			upper_ = 0 > max ? 0 : (t_max >= max ? max : t_max);
		} else {
			lower_ = t_max >= min ? min : t_max;
			upper_ = t_max >= max ? max : t_max;
		}
	}

	[[nodiscard]] constexpr T lower() const noexcept { return lower_; }

	[[nodiscard]] constexpr T upper() const noexcept { return upper_; }

	[[nodiscard]] constexpr bool contains(Range range) const noexcept
	{
		return lower_ <= range.lower_ && range.upper_ <= upper_;
	}

	template <typename T2>
	[[nodiscard]] constexpr bool contains(Range<T2> range) const noexcept
	{
		return contains(range.lower(), range.upper());
	}

	template <typename V, typename = std::enable_if_t<std::is_arithmetic_v<V>>>
	[[nodiscard]] constexpr bool contains(V value) const noexcept
	{
		return contains(value, value);
	}

	template <typename V, typename = std::enable_if_t<std::is_arithmetic_v<V>>>
	[[nodiscard]] constexpr bool contains(V lower, V upper) const noexcept
	{
		// FIXME: Is correct?
		auto [min, max] = std::minmax(lower, upper);
		if constexpr (std::is_floating_point_v<T> || std::is_floating_point_v<V> ||
		              (std::is_signed_v<T> == std::is_signed_v<V>)) {
			return lower_ <= min && max <= upper_;
		} else {
			if (min < 0 || upper_ < 0 || max > upper_) {
				return false;
			}
			return lower_ < 0 || lower_ <= min;
		}
	}

	void swap(Range other)
	{
		std::swap(lower_, other.lower_);
		std::swap(upper_, other.upper_);
	}

	std::ostream &writeData(std::ostream &out_stream) const
	{
		out_stream.write(reinterpret_cast<char *>(&lower_), sizeof(T));
		return out_stream.write(reinterpret_cast<char *>(&upper_), sizeof(T));
	}

	std::istream &readData(std::istream &in_stream)
	{
		in_stream.read(reinterpret_cast<char *>(&lower_), sizeof(T));
		return in_stream.read(reinterpret_cast<char *>(&upper_), sizeof(T));
	}

	//
	// Friends
	//

	template <typename T2>
	friend bool operator==(Range<T2> lhs, Range<T2> rhs);
	template <typename T2>
	friend bool operator!=(Range<T2> lhs, Range<T2> rhs);
	template <typename T2>
	friend bool operator<(Range<T2> lhs, Range<T2> rhs);
	template <typename T2>
	friend bool operator<=(Range<T2> lhs, Range<T2> rhs);
	template <typename T2>
	friend bool operator>(Range<T2> lhs, Range<T2> rhs);
	template <typename T2>
	friend bool operator>=(Range<T2> lhs, Range<T2> rhs);

	template <typename T2>
	friend void swap(Range<T2> &lhs, Range<T2> &rhs) noexcept(noexcept(lhs.swap(rhs)));

	template <typename T2>
	friend std::ostream &operator<<(std::ostream &os, Range<T2> range);

 private:
	T lower_;
	T upper_;
};

template <typename T>
bool operator==(Range<T> lhs, Range<T> rhs)
{
	return lhs.lower_ == rhs.lower_ && lhs.upper_ == rhs.upper_;
}

template <typename T>
bool operator!=(Range<T> lhs, Range<T> rhs)
{
	return lhs.lower_ != rhs.lower_ || lhs.upper_ != rhs.upper_;
}

template <typename T>
bool operator<(Range<T> lhs, Range<T> rhs)
{
	return lhs.upper_ < rhs.lower_;
}

template <typename T>
bool operator<=(Range<T> lhs, Range<T> rhs)
{
	return lhs.upper_ <= rhs.lower_;
}

template <typename T>
bool operator>(Range<T> lhs, Range<T> rhs)
{
	return lhs.lower_ > rhs.upper_;
}

template <typename T>
bool operator>=(Range<T> lhs, Range<T> rhs)
{
	return lhs.lower_ >= rhs.upper_;
}

template <typename T>
void swap(Range<T> &lhs, Range<T> &rhs) noexcept(noexcept(lhs.swap(rhs)))
{
	lhs.swap(rhs);
}

template <typename T>
std::ostream &operator<<(std::ostream &os, Range<T> range)
{
	if (range.lower() == range.upper()) {
		os << '[' << +range.lower() << ']';
	} else if constexpr (std::is_floating_point_v<T>) {
		os << '[' << +range.lower() << ',' << +range.upper() << ']';
	} else {
		os << '[' << +range.lower() << ".." << +range.upper() << ']';
	}
	return os;
}

///////////////////////////////////////////////////////////////////////////
//////////////////////////////// RangeSet /////////////////////////////////
///////////////////////////////////////////////////////////////////////////

template <typename Key>
class RangeSet
{
 public:
	//
	// Tags
	//

	using key_type = Range<Key>;
	using value_type = key_type;
	using size_type = typename std::vector<value_type>::size_type;  // Should this be Key?
	using difference_type = typename std::vector<value_type>::difference_type;
	using key_compare = typename key_type::Comparator;
	using value_compare = typename value_type::Comparator;
	using allocator_type = typename std::vector<value_type>::allocator_type;
	using reference = typename std::vector<value_type>::reference;
	using const_reference = typename std::vector<value_type>::const_reference;
	using pointer = typename std::vector<value_type>::pointer;
	using const_pointer = typename std::vector<value_type>::const_pointer;
	using iterator = typename std::vector<value_type>::const_iterator;
	using const_iterator = typename std::vector<value_type>::const_iterator;
	using reverse_iterator = typename std::vector<value_type>::const_reverse_iterator;
	using const_reverse_iterator = typename std::vector<value_type>::const_reverse_iterator;
	// FIXME: using node_type =
	// FIXME: using insert_return_type =

	//
	// Constructors
	//

	RangeSet() {}

	template <class InputIt,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	RangeSet(InputIt first, InputIt last)
	{
		insert(first, last);
	}

	RangeSet(Range<Key> const &range) { insert(range); }

	RangeSet(Range<Key> &&range) { insert(std::move(range)); }

	RangeSet(RangeSet const &other) : ranges_(other.ranges_) {}

	template <typename Key2>
	RangeSet(RangeSet<Key2> const &other)
	{
		// FIXME: Make sure it does not throw
		insert(std::cbegin(other), std::cend(other));
	}

	RangeSet(RangeSet &&other) : ranges_(std::move(other.ranges_)) {}

	RangeSet(std::initializer_list<value_type> init) { insert(init); }

	//
	// Destructor
	//

	~RangeSet() {}

	//
	// Assignment operator
	//

	RangeSet &operator=(RangeSet const &other)
	{
		ranges_ = other.ranges_;
		return *this;
	}

	template <typename Key2>
	RangeSet &operator=(RangeSet<Key2> const &other)
	{
		// FIXME: Make sure it does not throw
		clear();
		insert(std::cbegin(other), std::cend(other));
		return *this;
	}

	RangeSet &operator=(RangeSet &&other) noexcept(
	    std::allocator_traits<allocator_type>::is_always_equal::value
	        &&std::is_nothrow_move_assignable<value_compare>::value)  // FIXME: Look
	                                                                  // at noexcept
	{
		ranges_ = std::move(other.ranges_);
		return *this;
	}

	RangeSet &operator=(std::initializer_list<value_type> ilist)
	{
		clear();
		insert(ilist);
		return *this;
	}

	//
	// Get allocator
	//

	allocator_type get_allocator() const noexcept { return ranges_.get_allocator(); }

	//
	// Iterators
	//

	iterator begin() noexcept { return std::begin(ranges_); }

	const_iterator begin() const noexcept { return std::begin(ranges_); }

	const_iterator cbegin() const noexcept { return std::cbegin(ranges_); }

	iterator end() noexcept { return std::end(ranges_); }

	const_iterator end() const noexcept { return std::end(ranges_); }

	const_iterator cend() const noexcept { return std::cend(ranges_); }

	reverse_iterator rbegin() noexcept { return std::rbegin(ranges_); }

	const_reverse_iterator rbegin() const noexcept { return std::rbegin(ranges_); }

	const_reverse_iterator crbegin() const noexcept { return std::crbegin(ranges_); }

	reverse_iterator rend() noexcept { return std::rend(ranges_); }

	const_reverse_iterator rend() const noexcept { return std::rend(ranges_); }

	const_reverse_iterator crend() const noexcept { return std::crend(ranges_); }

	//
	// Capacity
	//

	[[nodiscard]] bool empty() const noexcept { return ranges_.empty(); }

	[[nodiscard]] size_type size() const noexcept { return ranges_.size(); }

	[[nodiscard]] size_type max_size() const noexcept { return ranges_.max_size(); }

	[[nodiscard]] size_type numRanges() const noexcept { return size(); }

	template <typename Key2 = Key>
	[[nodiscard]] std::enable_if_t<std::is_integral_v<Key2>, size_type> numValues() const
	{
		return std::accumulate(begin(), end(), numRanges(), [](auto cur, auto range) {
			return cur + (range.upper() - range.lower());
		});
	}

	//
	// Modifiers
	//

	void clear() noexcept { ranges_.clear(); }

	std::pair<iterator, bool> insert(value_type value)
	{
		// Subtract one to easily determine if [lower, upper] should be combined
		// with a range [X, lower - 1]. E.g., [8, 15] should be combined with [1, 7]
		// to become [1, 15]. Also note the protection against underflow
		Key const lower_up = decrement(value.lower());

		// Add one to easily determine if [lower, upper] should be combined with a
		// range [upper + 1, X]. E.g., [4, 15] should be combined with [16, 21] to
		// become [4, 21]. Also note the protection against overflow
		Key const upper_op = increment(value.upper());

		auto [lower, upper] = equal_range(lower_up, value.upper());

		auto dist = std::distance(begin(), lower);
		bool inserted = false;

		if (end() == lower) {
			inserted = true;

			// Add new range to the end
			// E.g., [21, 35] should be added at the end of [1, 4], [7, 15] to become
			// [1, 4], [7, 15], [21, 35].
			ranges_.push_back(value);
		} else if (lower == upper && upper_op < upper->lower()) {
			inserted = true;

			// Add new range to the front or middle
			// E.g., [4, 7] should be added to the middle of [0, 2], [9, 24] to become
			// [0, 2], [4, 7], [9, 24].
			ranges_.insert(lower, value);
		} else {
			// Combine range with other range(s)

			// Note: lower and upper are const_iterator here
			auto nc_lower = std::next(std::begin(ranges_), dist);

			inserted = value.lower() < lower->lower();

			nc_lower->setRange(std::min(lower->lower(), value.lower()), lower->upper());
			if (end() == upper || upper_op < upper->lower()) {
				inserted = inserted || lower->upper() < value.upper();

				// New range stretches [lower, upper)
				// E.g., [21, 25] should be combinded with [4, 22], [24, 24], [100, 150]
				// to become [4, 25], [100, 150].
				nc_lower->setRange(lower->lower(), std::max(lower->upper(), value.upper()));
				erase(std::next(lower), upper);
			} else {
				inserted = inserted || lower != upper;

				// New range stretches [lower, upper]
				// E.g., [0, 50] should be combinded with [5, 15], [45, 55], [88, 99] to
				// become [0, 55], [88, 99].
				nc_lower->setRange(lower->lower(), upper->upper());
				erase(std::next(lower), std::next(upper));
			}
		}

		return {std::next(begin(), dist), inserted};
	}

	iterator insert(const_iterator hint, value_type value)
	{
		// FIXME: Use hint
		return insert(value).first;
	}

	template <class InputIt,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	void insert(InputIt first, InputIt last)
	{
		// FIXME: Correct?
		std::for_each(first, last, [this](auto &&...args) { this->insert(args...); });
	}

	void insert(std::initializer_list<value_type> ilist)
	{
		insert(std::cbegin(ilist), std::cend(ilist));
	}

	// FIXME: insert_return_type insert(node_type&& nh);

	// FIXME: iterator insert(const_iterator hint, node_type&& nh);

	template <class... Args>
	std::pair<iterator, bool> emplace(Args &&...args)
	{
		return insert(value_type(std::forward<Args>(args)...));
	}

	template <class... Args>
	iterator emplace_hint(const_iterator hint, Args &&...args)
	{
		return insert(hint, value_type(std::forward<Args>(args)...));
	}

	iterator erase(const_iterator pos) { return ranges_.erase(pos); }

	iterator erase(const_iterator first, const_iterator last)
	{
		return ranges_.erase(first, last);
	}

	size_type erase(key_type key)
	{
		// FIXME: What should num_removed correspond to in here?

		auto [lower, upper] = equal_range(key);

		auto nc_lower = std::next(std::begin(ranges_), std::distance(begin(), lower));
		auto nc_upper = std::next(std::begin(ranges_), std::distance(begin(), upper));

		Key const lower_up = decrement(key.lower());
		Key const upper_op = increment(key.upper());

		size_type num_removed = 0;

		if (end() == lower || (lower == upper && key.upper() < lower->lower())) {
			// Nothing to erase
		} else if (lower == upper) {
			num_removed = 1;

			if (key.lower() <= lower->lower()) {
				// Erase lower part of a single range
				nc_lower->setRange(upper_op, lower->upper());
			} else {
				// Erase middle part of a single range (split it into two ranges)
				auto cur_upper = lower->upper();
				nc_lower->setRange(lower->lower(), lower_up);
				ranges_.emplace(std::next(lower), upper_op, cur_upper);
			}
		} else {
			if (end() != upper) {
				num_removed = upper->lower() < upper_op ? 1 : 0;
				nc_upper->setRange(std::max(upper->lower(), upper_op), upper->upper());
			}

			num_removed += std::distance(lower, upper);

			if (key.lower() <= lower->lower()) {
				// Erase [lower, upper) ranges
				ranges_.erase(lower, upper);
			} else {
				// Erase (lower, upper) ranges
				nc_lower->setRange(lower->lower(), lower_up);
				ranges_.erase(std::next(lower), upper);
			}
		}

		return num_removed;
	}

	template <typename Key2>
	size_type erase(Range<Key2> key)
	{
		return erase(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	size_type erase(K key)
	{
		return erase(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	size_type erase(K lower, K upper)
	{
		return erase(key_type(lower, upper));
	}

	void swap(RangeSet &other) noexcept(
	    std::allocator_traits<allocator_type>::is_always_equal::value
	        &&std::is_nothrow_move_assignable<value_compare>::value)  // FIXME: Look
	                                                                  // at noexcept
	{
		ranges_.swap(other.ranges_);
	}

	// FIXME: node_type extract(const_iterator position);

	// FIXME: node_type extract(key_type const& k);

	// void merge(RangeSet& source)
	// {
	// 	// FIXME: Implement
	// }

	// void merge(RangeSet&& source)
	// {
	// 	// FIXME: Implement
	// }

	//
	// Lookup
	//
	[[nodiscard]] size_type count(key_type key) const { return contains(key) ? 1 : 0; }

	template <typename Key2>
	[[nodiscard]] size_type count(Range<Key2> key)
	{
		return count(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] size_type count(K key)
	{
		return count(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] size_type count(K lower, K upper)
	{
		return count(key_type(lower, upper));
	}

	iterator find(key_type key)
	{
		auto lower = lower_bound(key);

		return end() != lower && lower->lower() <= key.lower() &&
		               lower->upper() >= key.upper()
		           ? lower
		           : end();
	}

	template <typename Key2>
	iterator find(Range<Key2> key)
	{
		return find(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	iterator find(K key)
	{
		return find(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	iterator find(K lower, K upper)
	{
		return find(key_type(lower, upper));
	}

	const_iterator find(key_type key) const
	{
		auto lower = lower_bound(key);

		return end() != lower && lower->lower() <= key.lower() &&
		               lower->upper() >= key.upper()
		           ? lower
		           : end();
	}

	template <typename Key2>
	const_iterator find(Range<Key2> key) const
	{
		return find(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	const_iterator find(K key) const
	{
		return find(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	const_iterator find(K lower, K upper) const
	{
		return find(key_type(lower, upper));
	}

	[[nodiscard]] bool contains(key_type key) const
	{
		auto lower = lower_bound(key);
		return end() != lower && lower->lower() <= key.lower() &&
		       lower->upper() >= key.upper();
	}

	template <typename Key2>
	[[nodiscard]] bool contains(Range<Key2> key) const
	{
		return contains(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] bool contains(K key) const
	{
		return contains(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] bool contains(K lower, K upper) const
	{
		return contains(key_type(lower, upper));
	}

	[[nodiscard]] std::pair<iterator, iterator> equal_range(key_type key)
	{
		typename value_compare::range_t const s_range{key.lower(), key.upper()};
		return std::equal_range(begin(), end(), s_range, comp_);
	}

	template <typename Key2>
	[[nodiscard]] std::pair<iterator, iterator> equal_range(Range<Key2> key)
	{
		return equal_range(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] std::pair<iterator, iterator> equal_range(K key)
	{
		return equal_range(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] std::pair<iterator, iterator> equal_range(K lower, K upper)
	{
		return equal_range(key_type(lower, upper));
	}

	[[nodiscard]] std::pair<const_iterator, const_iterator> equal_range(key_type key) const
	{
		typename value_compare::range_t const s_range{key.lower(), key.upper()};
		return std::equal_range(begin(), end(), s_range, comp_);
	}

	template <typename Key2>
	[[nodiscard]] std::pair<const_iterator, const_iterator> equal_range(
	    Range<Key2> key) const
	{
		return equal_range(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] std::pair<const_iterator, const_iterator> equal_range(K key) const
	{
		return equal_range(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] std::pair<const_iterator, const_iterator> equal_range(K lower,
	                                                                    K upper) const
	{
		return equal_range(key_type(lower, upper));
	}

	[[nodiscard]] iterator lower_bound(key_type key)
	{
		typename key_compare::range_t const s_range{key.lower(), key.upper()};
		return std::lower_bound(begin(), end(), s_range, comp_);
	}

	template <typename Key2>
	[[nodiscard]] iterator lower_bound(Range<Key2> key)
	{
		return lower_bound(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] iterator lower_bound(K key)
	{
		return lower_bound(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] iterator lower_bound(K lower, K upper)
	{
		return lower_bound(key_type(lower, upper));
	}

	[[nodiscard]] const_iterator lower_bound(key_type key) const
	{
		typename key_compare::range_t const s_range{key.lower(), key.upper()};
		return std::lower_bound(begin(), end(), s_range, comp_);
	}

	template <typename Key2>
	[[nodiscard]] const_iterator lower_bound(Range<Key2> key) const
	{
		return lower_bound(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] const_iterator lower_bound(K key) const
	{
		return lower_bound(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] const_iterator lower_bound(K lower, K upper) const
	{
		return lower_bound(key_type(lower, upper));
	}

	[[nodiscard]] iterator upper_bound(key_type key)
	{
		typename key_compare::range_t const s_range{key.lower(), key.upper()};
		return std::upper_bound(begin(), end(), s_range, comp_);
	}

	template <typename Key2>
	[[nodiscard]] iterator upper_bound(Range<Key2> key)
	{
		return upper_bound(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] iterator upper_bound(K key)
	{
		return upper_bound(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] iterator upper_bound(K lower, K upper)
	{
		return upper_bound(key_type(lower, upper));
	}

	[[nodiscard]] const_iterator upper_bound(key_type key) const
	{
		typename key_compare::range_t const s_range{key.lower(), key.upper()};
		return std::upper_bound(begin(), end(), s_range, comp_);
	}

	template <typename Key2>
	[[nodiscard]] const_iterator upper_bound(Range<Key2> key) const
	{
		return upper_bound(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] const_iterator upper_bound(K key) const
	{
		return upper_bound(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] const_iterator upper_bound(K lower, K upper) const
	{
		return upper_bound(key_type(lower, upper));
	}

	//
	// (De)serialize
	//

	std::ostream &writeData(std::ostream &out_stream) const
	{
		size_type num = numRanges();
		out_stream.write(reinterpret_cast<char *>(&num), sizeof(size_type));
		// FIXME: Add comp_
		// FIXME: Does this work?
		return out_stream.write(reinterpret_cast<char *>(ranges_.data()),
		                        sizeof(value_type) * num);
	}

	std::istream &readData(std::istream &in_stream)
	{
		size_type num;
		in_stream.read(reinterpret_cast<char *>(&num), sizeof(size_type));
		if (0 == num) {
			clear();
			return in_stream;
		}
		// FIXME: Add comp_
		ranges_.resize(num);
		// FIXME: Does this work?
		return in_stream.read(reinterpret_cast<char *>(ranges_.data()),
		                      sizeof(value_type) * num);
	}

	//
	// Observers
	//

	key_compare key_comp() const { return comp_; }

	value_compare value_comp() const { return comp_; }

	//
	// Friends
	//

	template <typename K>
	friend bool operator==(RangeSet<K> const &lhs, RangeSet<K> const &rhs);
	template <typename K>
	friend bool operator!=(RangeSet<K> const &lhs, RangeSet<K> const &rhs);
	template <typename K>
	friend bool operator<(RangeSet<K> const &lhs, RangeSet<K> const &rhs);
	template <typename K>
	friend bool operator<=(RangeSet<K> const &lhs, RangeSet<K> const &rhs);
	template <typename K>
	friend bool operator>(RangeSet<K> const &lhs, RangeSet<K> const &rhs);
	template <typename K>
	friend bool operator>=(RangeSet<K> const &lhs, RangeSet<K> const &rhs);

	template <typename K>
	friend void swap(RangeSet<K> &lhs, RangeSet<K> &rhs) noexcept(noexcept(lhs.swap(rhs)));

	template <typename K, class Pred>
	friend size_type erase_if(RangeSet<K> &range_set, Pred pred);

	template <typename K>
	friend std::ostream &operator<<(std::ostream &os, RangeSet<K> const &range_set);

 protected:
	static constexpr Key increment(Key value)
	{
		if constexpr (std::is_floating_point_v<Key>) {
			return std::nextafter(value, std::numeric_limits<Key>::max());
		} else {
			return std::max(value, value + 1);
		}
	}

	static constexpr Key decrement(Key value)
	{
		if constexpr (std::is_floating_point_v<Key>) {
			return std::nextafter(value, std::numeric_limits<Key>::lowest());
		} else {
			return std::min(value, value - 1);
		}
	}

 protected:
	std::vector<value_type> ranges_;
	value_compare comp_;
};

template <typename Key>
bool operator==(RangeSet<Key> const &lhs, RangeSet<Key> const &rhs)
{
	// FIXME: Implement correctly
	return lhs.ranges_ == rhs.ranges_;
}

template <typename Key>
bool operator!=(RangeSet<Key> const &lhs, RangeSet<Key> const &rhs)
{
	// FIXME: Implement correctly
	return lhs.ranges_ != rhs.ranges_;
}

template <typename Key>
bool operator<(RangeSet<Key> const &lhs, RangeSet<Key> const &rhs)
{
	// FIXME: Implement correctly
	return lhs.ranges_ < rhs.ranges_;
}

template <typename Key>
bool operator<=(RangeSet<Key> const &lhs, RangeSet<Key> const &rhs)
{
	// FIXME: Implement correctly
	return lhs.ranges_ <= rhs.ranges_;
}

template <typename Key>
bool operator>(RangeSet<Key> const &lhs, RangeSet<Key> const &rhs)
{
	// FIXME: Implement correctly
	return lhs.ranges_ > rhs.ranges_;
}

template <typename Key>
bool operator>=(RangeSet<Key> const &lhs, RangeSet<Key> const &rhs)
{
	// FIXME: Implement correctly
	return lhs.ranges_ >= rhs.ranges_;
}

template <typename Key>
void swap(RangeSet<Key> &lhs, RangeSet<Key> &rhs) noexcept(noexcept(lhs.swap(rhs)))
{
	lhs.swap(rhs);
}

template <typename Key, class Pred>
typename RangeSet<Key>::size_type erase_if(RangeSet<Key> &range_set, Pred pred)
{
	auto old_size = range_set.size();
	for (auto it = std::begin(range_set), last = std::end(range_set); it != last;) {
		if (pred(*it)) {
			it = range_set.erase(it);
		} else {
			++it;
		}
	}
	return old_size - range_set.size();
}

template <typename Key>
std::ostream &operator<<(std::ostream &os, RangeSet<Key> const &range_set)
{
	if (!range_set.empty()) {
		std::copy(std::cbegin(range_set), std::prev(std::cend(range_set)),
		          std::ostream_iterator<typename RangeSet<Key>::value_type>(os, ", "));
		os << *std::prev(std::end(range_set));
	}
	return os;
}

/*!
 * @brief Returns true if b is a subsequence of a.
 *
 * @tparam Key1
 * @tparam Key2
 * @param a
 * @param b
 * @return true
 * @return false
 */
template <typename Key1, typename Key2>
[[nodiscard]] bool rangesIncludes(RangeSet<Key1> const &a, RangeSet<Key2> const &b)
{
	auto a_first = std::begin(a);
	auto a_last = std::end(a);
	auto b_first = std::begin(b);
	auto b_last = std::end(b);
	while (b_first != b_last) {
		if (a_first == a_last || a_first->lower() > b_first->upper()) {
			return false;
		}
		if (a_first->lower() <= b_first->lower() && a_first->upper() >= b_first->upper()) {
			++b_first;
		} else {
			++a_first;
		}
	}
	return true;
}

/*!
 * @brief Creates a new RangeSet containing the ranges in a which are not find
 * in b.
 *
 * @tparam Key1
 * @tparam Key2
 * @param a
 * @param b
 * @return RangeSet<Key1>
 */
template <typename Key1, typename Key2>
[[nodiscard]] RangeSet<Key1> rangesDifference(RangeSet<Key1> const &a,
                                              RangeSet<Key2> const &b)
{
	// FIXME: Implement efficiently
	RangeSet<Key1> output = a;
	for (auto range : b) {
		output.erase(range);
	}
	return output;
}

template <typename Key1, typename Key2>
[[nodiscard]] RangeSet<Key1> rangesIntersection(RangeSet<Key1> const &a,
                                                RangeSet<Key2> const &b)
{
	// FIXME: Implement efficiently
	RangeSet<Key1> output;
	for (auto range : a) {
		if (b.contains(range)) {
			output.insert(range);
		}
	}
	for (auto range : b) {
		if (a.contains(range)) {
			output.insert(range);
		}
	}
	return output;
}

template <typename Key1, typename Key2>
[[nodiscard]] RangeSet<Key1> rangesSymmetricDifference(RangeSet<Key1> const &a,
                                                       RangeSet<Key2> const &b)
{
	// FIXME: Implement efficiently
	RangeSet<Key1> output = a;
	for (auto range : b) {
		output.erase(range);
	}
	RangeSet<Key1> temp = b;
	for (auto range : a) {
		temp.erase(range);
	}
	output.insert(std::begin(temp), std::end(temp));
	return output;
}

template <typename Key1, typename Key2>
[[nodiscard]] RangeSet<Key1> rangesUnion(RangeSet<Key1> const &a, RangeSet<Key2> const &b)
{
	// FIXME: Implement efficiently
	RangeSet<Key1> output = a;
	for (auto range : b) {
		output.insert(range);
	}
	return output;
}

///////////////////////////////////////////////////////////////////////////
//////////////////////////////// RangeMap /////////////////////////////////
///////////////////////////////////////////////////////////////////////////

template <typename Key, typename T>
class RangeMap
{
	// FIXME: Use std::piecewise_construct more

 public:
	//
	// Tags
	//

	using key_type = Range<Key>;
	using mapped_type = T;
	// FIXME: Key should be const: using value_type = std::pair<key_type const,
	// mapped_type>;
	using value_type = std::pair<key_type, mapped_type>;
	using size_type = typename std::vector<value_type>::size_type;
	using difference_type = typename std::vector<value_type>::difference_type;
	using key_compare = typename key_type::Comparator;
	using allocator_type = typename std::vector<value_type>::allocator_type;
	using reference = typename std::vector<value_type>::reference;
	using const_reference = typename std::vector<value_type>::const_reference;
	using pointer = typename std::vector<value_type>::pointer;
	using const_pointer = typename std::vector<value_type>::const_pointer;
	using iterator = typename std::vector<value_type>::iterator;
	using const_iterator = typename std::vector<value_type>::const_iterator;
	using reverse_iterator = typename std::vector<value_type>::reverse_iterator;
	using const_reverse_iterator = typename std::vector<value_type>::const_reverse_iterator;
	// FIXME: using node_type =
	// FIXME: using insert_return_type =

	//
	// Value compare
	//

	struct value_compare {
		using is_transparent = std::true_type;

		// [[nodiscard]] constexpr bool operator()(value_type const& lhs,
		//                                         value_type const& rhs) const
		// {
		// 	return comp_(lhs.first, rhs.first);
		// }

		[[nodiscard]] constexpr bool operator()(typename key_compare::range_t lhs,
		                                        value_type const &rhs) const
		{
			return comp_(lhs, rhs.first);
		}

		[[nodiscard]] constexpr bool operator()(value_type const &lhs,
		                                        typename key_compare::range_t rhs)
		{
			return comp_(lhs.first, rhs);
		}

	 protected:
		key_compare comp_;
	};

	//
	// Constructors
	//

	RangeMap() {}

	template <class InputIt,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	RangeMap(InputIt first, InputIt last)
	{
		insert(first, last);
	}

	RangeMap(RangeMap const &other) : ranges_(other.ranges_) {}

	template <typename Key2>
	RangeMap(RangeMap<Key2, T> const &other)
	{
		// FIXME: Make sure it does not throw
		insert(std::cbegin(other), std::cend(other));
	}

	RangeMap(RangeMap &&other) : ranges_(std::move(other.ranges_)) {}

	RangeMap(std::initializer_list<value_type> init) { insert(init); }

	//
	// Destructor
	//

	~RangeMap() {}

	//
	// Assignment operator
	//

	RangeMap &operator=(RangeMap const &other)
	{
		ranges_ = other.ranges_;
		return *this;
	}

	template <typename Key2>
	RangeMap &operator=(RangeMap<Key2, T> const &other)
	{
		// FIXME: Make sure it does not throw
		clear();
		insert(std::cbegin(other), std::cend(other));
		return *this;
	}

	RangeMap &operator=(RangeMap &&other)
	{
		ranges_ = std::move(other.ranges_);
		return *this;
	}

	RangeMap &operator=(std::initializer_list<value_type> ilist)
	{
		clear();
		insert(ilist);
		return *this;
	}

	//
	// Get allocator
	//

	allocator_type get_allocator() const noexcept { return ranges_.get_allocator(); }

	//
	// Element access
	//

	// TODO: T& at()

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] T const &get(K key)
	{
		auto it = find(key);
		if (end() == it) {
			throw std::out_of_range("Key is not stored in the container.");
		}
		return it->second;
	}

	//
	// Iterators
	//

	iterator begin() noexcept { return std::begin(ranges_); }

	const_iterator begin() const noexcept { return std::begin(ranges_); }

	const_iterator cbegin() const noexcept { return std::cbegin(ranges_); }

	iterator end() noexcept { return std::end(ranges_); }

	const_iterator end() const noexcept { return std::end(ranges_); }

	const_iterator cend() const noexcept { return std::cend(ranges_); }

	reverse_iterator rbegin() noexcept { return std::rbegin(ranges_); }

	const_reverse_iterator rbegin() const noexcept { return std::rbegin(ranges_); }

	const_reverse_iterator crbegin() const noexcept { return std::crbegin(ranges_); }

	reverse_iterator rend() noexcept { return std::rend(ranges_); }

	const_reverse_iterator rend() const noexcept { return std::rend(ranges_); }

	const_reverse_iterator crend() const noexcept { return std::crend(ranges_); }

	//
	// Capacity
	//

	[[nodiscard]] bool empty() const noexcept { return ranges_.empty(); }

	[[nodiscard]] size_type size() const noexcept { return ranges_.size(); }

	[[nodiscard]] size_type max_size() const noexcept { return ranges_.max_size(); }

	[[nodiscard]] size_type numRanges() const noexcept { return size(); }

	template <typename Key2 = Key>
	[[nodiscard]] std::enable_if_t<std::is_integral_v<Key2>, size_type> numValues() const
	{
		return std::accumulate(begin(), end(), numRanges(), [](auto cur, auto elem) {
			return cur + (elem.first.upper() - elem.first.lower());
		});
	}

	//
	// Modifiers
	//

	void clear() noexcept { ranges_.clear(); }

	std::pair<iterator, bool> insert(value_type const &value)
	{
		std::cout << value.first.lower() << ' ' << value.first.upper() << '\n';
		key_type key = value.first;

		// Subtract one to easily determine if [lower, upper] should be combined
		// with a range [X, lower - 1]. E.g., [8, 15] should be combined with [1, 7]
		// to become [1, 15]. Also note the protection against underflow
		Key const lower_up = decrement(key.lower());

		// Add one to easily determine if [lower, upper] should be combined with a
		// range [upper + 1, X]. E.g., [4, 15] should be combined with [16, 21] to
		// become [4, 21]. Also note the protection against overflow
		Key const upper_op = increment(key.upper());

		auto [lower, upper] = equal_range(lower_up, key.upper());

		if (end() == lower) {
			std::cout << "1\n";
			// Add new range to the end
			// E.g., [21, 35] should be added at the end of [1, 4], [7, 15] to become
			// [1, 4], [7, 15], [21, 35].
			ranges_.push_back(value);
			return {std::prev(end()), true};
		} else if (lower == upper && upper_op < upper->first.lower()) {
			std::cout << "2\n";
			// Add new range to the front or middle
			// E.g., [4, 7] should be added to the middle of [0, 2], [9, 24] to become
			// [0, 2], [4, 7], [9, 24].
			auto it = ranges_.insert(lower, value);
			return {it, true};
		}
		std::cout << "3\n";

		auto lower_dist = std::distance(begin(), lower);
		auto upper_dist = std::distance(begin(), upper);

		bool inserted = false;

		// FIXME: Fix inserted

		//
		// Fix beginning
		//

		if (lower_up == lower->first.upper() && value.second != lower->second) {
			auto lower_next = std::next(lower);
			if (value.second == lower_next->second) {
				// Extend
				lower_next->first.setRange(key.lower(), lower_next->first.upper());
			} else {
				// Add
				ranges_.emplace(lower_next, key_type(key.lower()), value.second);
				++upper_dist;
			}
			++lower_dist;
		} else if (key.lower() < lower->first.lower()) {
			if (increment(key.lower()) < lower->first.lower() ||
			    value.second != lower->second) {
				// Add
				ranges_.emplace(lower, key_type(key.lower(), decrement(lower->first.lower())),
				                value.second);
				++upper_dist;
			} else {
				// Extend
				lower->first.setRange(key.lower(), lower->first.upper());
			}
		}

		lower = std::next(begin(), lower_dist);
		upper = std::next(begin(), upper_dist);

		//
		// Fix end
		//

		auto upper_prev = std::prev(upper);

		if (end() == upper) {
			if (value.second == upper_prev->second) {
				// Extend
				upper_prev->first.setRange(upper_prev->first.lower(), key.upper());
				--upper_dist;
			} else {
				// Add
				ranges_.emplace_back(key_type(increment(upper_prev->first.upper()), key.upper()),
				                     value.second);
			}
		} else if (key.upper() < upper->first.lower()) {
			if (upper_op < upper->first.lower() || value.second != upper->second) {
				// Add
				ranges_.emplace(upper,
				                key_type(increment(upper_prev->first.upper()), key.upper()),
				                value.second);
			} else {
				// Extend
				upper->first.setRange(key.upper(), upper->first.upper());
			}
		}

		lower = std::next(begin(), lower_dist);
		upper = std::next(begin(), upper_dist);

		//
		// Fix middle
		//

		for (auto it = lower; it != upper; ++it) {
			if (value.second == it->second) {
				continue;
			}
			auto it_next = std::next(it);
			if (value.second == it_next->second) {
				// Extend
				it_next->first.setRange(increment(it->first.upper()), it_next->first.upper());
			} else if (increment(it->first.upper()) <= decrement(it_next->first.lower())) {
				// Add
				ranges_.emplace(
				    it_next,
				    key_type(increment(it->first.upper()), decrement(it_next->first.lower())),
				    value.second);
				++upper_dist;
			}
		}

		lower = std::next(begin(), lower_dist);
		upper = std::next(begin(), upper_dist);

		for (auto it = lower; it != std::prev(upper);) {
			if (value.second != it->second) {
				continue;
			}
			auto it_next = std::next(it);
			if (value.second == it_next->second) {
				// Combine ranges
				it->first.setRange(it->first.lower(), it_next->first.upper());
				it = erase(it_next);
				std::prev(it);
				--upper_dist;
				upper = std::next(begin(), upper_dist);
			} else {
				// Extend current range
				it->first.setRange(it->first.lower(), decrement(it_next->first.lower()));
				++it;
			}
		}

		lower = std::next(begin(), lower_dist);
		upper = std::next(begin(), upper_dist);

		return {lower, inserted};
	}

	template <class P,
	          typename = std::enable_if_t<std::is_constructible_v<value_type, P &&>>>
	std::pair<iterator, bool> insert(P &&value)
	{
		return emplace(std::forward<P>(value));
	}

	std::pair<iterator, bool> insert(value_type &&value)
	{
		key_type key = value.first;

		// Subtract one to easily determine if [lower, upper] should be combined
		// with a range [X, lower - 1]. E.g., [8, 15] should be combined with [1, 7]
		// to become [1, 15]. Also note the protection against underflow
		Key const lower_up = decrement(key.lower());

		// Add one to easily determine if [lower, upper] should be combined with a
		// range [upper + 1, X]. E.g., [4, 15] should be combined with [16, 21] to
		// become [4, 21]. Also note the protection against overflow
		Key const upper_op = increment(key.upper());

		auto [lower, upper] = equal_range(lower_up, key.upper());

		if (end() != lower && lower->first.upper() < key.lower() &&
		    value.second != lower->second) {
			// To make sure that lower always points to the correct when returned.
			// It functions the same without this, however the returned iterator will
			// be one before the actual sometimes.
			++lower;
		}

		// Easy cases
		if (end() == lower) {
			// Add new range to the end
			// E.g., [21, 35] should be added at the end of [1, 4], [7, 15] to become
			// [1, 4], [7, 15], [21, 35].
			ranges_.push_back(value);
			return {std::prev(end()), true};
		} else if (lower == upper) {
			if (upper_op < lower->first.lower() ||
			    (key.upper() < lower->first.lower() && value.second != lower->second)) {
				// Add new range to the front or middle
				// E.g., [4, 7] should be added to the middle of [0, 2], [9, 24] to become
				// [0, 2], [4, 7], [9, 24].
				auto it = ranges_.insert(lower, value);
				return {it, true};
			} else if (value.second == lower->second) {
				bool inserted = key.lower() < lower->first.lower();
				lower->first.setRange(std::min(key.lower(), lower->first.lower()),
				                      lower->first.upper());
				return {lower, inserted};
			} else if (key.lower() < lower->first.lower()) {
				lower = ranges_.emplace(
				    lower, key_type(key.lower(), decrement(lower->first.lower())), value.second);
				return {lower, true};
			} else {
				return {lower, false};
			}
		}

		// Difficult cases
		upper = end() != upper ? upper : std::prev(upper);

		auto lower_dist = std::distance(begin(), lower);
		auto upper_dist = std::distance(begin(), upper);

		bool inserted = false;

		// Fix lower
		if (key.lower() < lower->first.lower()) {
			inserted = true;
			if (value.second == lower->second) {
				// Extend
				lower->first.setRange(key.lower(), lower->first.upper());
			} else {
				// Add
				lower = ranges_.emplace(
				    lower, key_type(key.lower(), decrement(lower->first.lower())), value.second);
				++upper_dist;
				upper = std::next(begin(), upper_dist);
			}
		}

		// Fix upper
		if (key.upper() > upper->first.upper()) {
			inserted = true;
			if (value.second == upper->second) {
				// Extend
				upper->first.setRange(upper->first.lower(), key.upper());
			} else {
				// Add
				upper = ranges_.emplace(std::next(upper),
				                        key_type(increment(upper->first.upper()), key.upper()),
				                        value.second);
				lower = std::next(begin(), lower_dist);
				++upper_dist;
			}
		} else if (upper_op < upper->first.lower()) {
			inserted = true;
			auto upper_prev = std::prev(upper);  // lower cannot be the same as upper
			                                     // here, hence there is a prev
			if (value.second == upper_prev->second) {
				// Extend
				upper_prev->first.setRange(upper_prev->first.lower(), key.upper());
				--upper_dist;
				upper = upper_prev;
			} else if (increment(upper_prev->first.upper()) < key.upper()) {
				// Add
				upper = ranges_.emplace(
				    upper, key_type(increment(upper_prev->first.upper()), key.upper()),
				    value.second);
				lower = std::next(begin(), lower_dist);
			}
		}

		// Everything between (lower, upper) should be filled in
		for (auto it = std::next(lower); std::next(upper) != it; ++it) {
			auto it_prev = std::prev(it);
			if (value.second == it_prev->second) {
				if (value.second == it->second) {
					inserted = true;
					// Combine
					it_prev->first.setRange(it_prev->first.lower(), it->first.upper());
					it = erase(it);
					std::prev(it);
					--upper_dist;
				} else {
					// Extend uppwards
					auto old = it_prev->first;
					it_prev->first.setRange(it_prev->first.lower(), decrement(it->first.lower()));
					inserted = inserted || old != it_prev->first;
				}
			} else if (value.second == it->second) {
				// Extend downards
				auto old = it->first;
				it->first.setRange(increment(it_prev->first.upper()), it->first.upper());
				inserted = inserted || old != it->first;
			} else if (auto temp = increment(it_prev->first.upper());
			           temp < it->first.lower()) {
				inserted = true;
				// Add
				it = ranges_.emplace(it, key_type(temp, decrement(it->first.lower())),
				                     value.second);
				++upper_dist;
			}
			upper = std::next(begin(), upper_dist);
		}

		lower = std::next(begin(), lower_dist);
		if (end() != upper && upper->first.upper() <= key.upper()) {
			++upper;
		}
		return {lower, inserted};
	}

	iterator insert(const_iterator hint, value_type const &value)
	{
		// FIXME: Use hint
		return insert(value).first;
	}

	template <class P>
	iterator insert(const_iterator hint, P &&value)
	{
		// FIXME: Use hint
		return insert(std::forward<P>(value)).first;
	}

	iterator insert(const_iterator hint, value_type &&value)
	{
		//  FIXME: Use hint
		return insert(std::forward<value_type>(value)).first;
	}

	template <class InputIt,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	void insert(InputIt first, InputIt last)
	{
		// FIXME: Correct?
		std::for_each(first, last, [this](auto &&...args) { this->insert(args...); });
	}

	void insert(std::initializer_list<value_type> ilist)
	{
		insert(std::cbegin(ilist), std::cend(ilist));
	}

	// FIXME: insert_return_type insert(node_type&& nh);

	// FIXME: iterator insert(const_iterator hint, node_type&& nh);

	/*
	 * Insertion that additionally assigns if parts of key are already present in map
	 * E.g., if the map contains: [1,3]:2, [6,9]:1 and [3, 8]:1 is inserted the resulting
	 * map will be: [1,2]:2, [3, 9]:1.
	 */
	template <class M>
	std::pair<iterator, bool> insert_or_assign(key_type key, M &&obj)
	{
		// Subtract one to easily determine if [lower, upper] should be combined
		// with a range [X, lower - 1]. E.g., [8, 15] should be combined with [1, 7]
		// to become [1, 15]. Also note the protection against underflow
		Key const lower_up = decrement(key.lower());

		// Add one to easily determine if [lower, upper] should be combined with a
		// range [upper + 1, X]. E.g., [4, 15] should be combined with [16, 21] to
		// become [4, 21]. Also note the protection against overflow
		Key const upper_op = increment(key.upper());

		auto [lower, upper] = equal_range(lower_up, key.upper());

		// Fix lower
		if (end() != lower && lower->first.upper() < key.lower() && obj != lower->second) {
			++lower;
		}

		if (end() == lower) {
			// Add new range to the end
			// E.g., [21, 35] should be added at the end of [1, 4], [7, 15] to become
			// [1, 4], [7, 15], [21, 35].
			ranges_.emplace_back(key, std::forward<M>(obj));
			return {std::prev(end()), true};
		} else if (lower == upper) {
			if (upper_op < lower->first.lower() ||
			    (key.upper() < lower->first.lower() && obj != lower->second)) {
				// Add new range to the front or middle
				// E.g., [4, 7] should be added to the middle of [0, 2], [9, 24] to become
				// [0, 2], [4, 7], [9, 24].
				lower = ranges_.emplace(lower, key, std::forward<M>(obj));
				return {lower, true};
			} else if (obj == lower->second) {
				// Extend
				bool inserted = key.lower() < lower->first.lower();
				lower->first.setRange(std::min(key.lower(), lower->first.lower()),
				                      lower->first.upper());
				return {lower, inserted};
			} else if (key.lower() <= lower->first.lower()) {
				// Split into two
				lower->first.setRange(upper_op, lower->first.upper());
				lower = ranges_.emplace(lower, key, std::forward<M>(obj));
				return {lower, true};
			} else {
				// Split into three
				auto max = lower->first.upper();
				lower->first.setRange(lower->first.lower(), decrement(key.lower()));
				lower = ranges_.emplace(std::next(lower), key_type(increment(key.upper()), max),
				                        lower->second);
				lower = ranges_.emplace(lower, key, std::forward<M>(obj));
				return {lower, true};
			}
		}

		bool inserted = false;  // FIXME: Update

		// Fix lower
		if (lower->first.lower() < key.lower()) {
			if (lower->second == obj) {
				key.setRange(lower->first.lower(), key.upper());
			} else {
				lower->first.setRange(lower->first.lower(), decrement(key.lower()));
				++lower;

				if (end() == lower) {
					ranges_.emplace_back(key, std::forward<M>(obj));
					return {std::prev(end()), true};
				}
			}
		}

		// Fix upper
		if (end() != upper) {
			if (upper->second != obj) {
				upper->first.setRange(std::max(upper_op, upper->first.lower()),
				                      upper->first.upper());
			} else if (upper_op >= upper->first.lower()) {
				key.setRange(key.lower(), upper->first.upper());
				++upper;
			}
		}

		if (lower == upper) {
			lower = ranges_.emplace(lower, key, std::forward<M>(obj));
			return {lower, true};
		}

		// Erase
		auto lower_dist = std::distance(begin(), lower);
		erase(std::next(lower), upper);
		lower = std::next(begin(), lower_dist);

		// Assign
		lower->first = key;
		lower->second = std::forward<M>(obj);

		return {lower, inserted};
	}

	template <class M>
	iterator insert_or_assign(const_iterator hint, key_type key, M &&obj)
	{
		// FIXME: Use hint
		return insert_or_assign(key, std::forward<M>(obj)).first;
	}

	template <class... Args>
	std::pair<iterator, bool> emplace(Args &&...args)
	{
		// FIXME: Construct value_type only if needed
		return insert(value_type(std::forward<Args>(args)...));
	}

	template <class... Args>
	iterator emplace_hint(const_iterator hint, Args &&...args)
	{
		// FIXME: Use hint
		return emplace(std::forward<Args>(args)...).first;
	}

	template <class... Args>
	std::pair<iterator, bool> try_emplace(key_type k, Args &&...args)
	{
		// FIXME: Construct value_type only if needed
		return insert(value_type(std::piecewise_construct, std::forward_as_tuple(k),
		                         std::forward_as_tuple(std::forward<Args>(args)...)));
	}

	template <class... Args>
	iterator try_emplace(const_iterator hint, key_type k, Args &&...args)
	{
		// FIXME: Use hint
		return try_emplace(k, std::forward<Args>(args)...).first;
	}

	iterator erase(const_iterator pos) { return ranges_.erase(pos); }

	iterator erase(const_iterator first, const_iterator last)
	{
		return ranges_.erase(first, last);
	}

	size_type erase(key_type key)
	{
		// FIXME: What should num_removed correspond to in here?

		auto [lower, upper] = equal_range(key);

		Key const lower_up = decrement(key.lower());
		Key const upper_op = increment(key.upper());

		size_type num_removed = 0;

		if (end() == lower || (lower == upper && key.upper() < lower->first.lower())) {
			// Nothing to erase
		} else if (lower == upper) {
			num_removed = 1;

			if (key.lower() <= lower->first.lower()) {
				// Erase lower part of a single range
				lower->first.setRange(upper_op, lower->first.upper());
			} else {
				// Erase middle part of a single range (split it into two ranges)
				auto cur_upper = lower->first.upper();
				lower->first.setRange(lower->first.lower(), lower_up);
				ranges_.emplace(std::next(lower), upper_op, cur_upper);
			}
		} else {
			if (end() != upper) {
				num_removed = upper->first.lower() < upper_op ? 1 : 0;
				upper->first.setRange(std::max(upper->first.lower(), upper_op),
				                      upper->first.upper());
			}

			num_removed += std::distance(lower, upper);

			if (key.lower() <= lower->first.lower()) {
				// Erase [lower, upper) ranges
				ranges_.erase(lower, upper);
			} else {
				// Erase (lower, upper) ranges
				lower->first.setRange(lower->first.lower(), lower_up);
				ranges_.erase(std::next(lower), upper);
			}
		}

		return num_removed;
	}

	template <typename Key2>
	size_type erase(Range<Key2> key)
	{
		return erase(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	size_type erase(K key)
	{
		return erase(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	size_type erase(K lower, K upper)
	{
		return erase(key_type(lower, upper));
	}

	void swap(RangeMap &other) noexcept(
	    std::allocator_traits<allocator_type>::is_always_equal::value
	        &&std::is_nothrow_move_assignable<value_compare>::value)  // FIXME: Look
	                                                                  // at noexcept
	{
		ranges_.swap(other.ranges_);
	}

	// FIXME: node_type extract(const_iterator position);

	// FIXME: node_type extract(key_type const& k);

	// void merge(RangeMap& source)
	// {
	// 	// FIXME: Implement
	// }

	// void merge(RangeMap&& source)
	// {
	// 	// FIXME: Implement
	// }

	//
	// Lookup
	//

	[[nodiscard]] size_type count(key_type key) const { return contains(key) ? 1 : 0; }

	template <typename Key2>
	[[nodiscard]] size_type count(Range<Key2> key)
	{
		return count(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] size_type count(K key)
	{
		return count(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] size_type count(K lower, K upper)
	{
		return count(key_type(lower, upper));
	}

	iterator find(key_type key)
	{
		// FIXME: A single range can occupy mulitple indices now since the mapped
		// value can be different. How should this be handled?
		auto lower = lower_bound(key);

		return end() != lower && lower->first.lower() <= key.lower() &&
		               lower->first.upper() >= key.upper()
		           ? lower
		           : end();
	}

	template <typename Key2>
	iterator find(Range<Key2> key)
	{
		return find(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	iterator find(K key)
	{
		return find(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	iterator find(K lower, K upper)
	{
		return find(key_type(lower, upper));
	}

	const_iterator find(key_type key) const
	{
		// FIXME: A single range can occupy mulitple indices now since the mapped
		// value can be different. How should this be handled?
		auto lower = lower_bound(key);

		return end() != lower && lower->first.lower() <= key.lower() &&
		               lower->first.upper() >= key.upper()
		           ? lower
		           : end();
	}

	template <typename Key2>
	const_iterator find(Range<Key2> key) const
	{
		return find(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	const_iterator find(K key) const
	{
		return find(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	const_iterator find(K lower, K upper) const
	{
		return find(key_type(lower, upper));
	}

	[[nodiscard]] bool contains(key_type key) const
	{
		// FIXME: A single range can occupy mulitple indices now since the mapped
		// value can be different. How should this be handled?
		auto lower = lower_bound(key);
		return end() != lower && lower->first.lower() <= key.lower() &&
		       lower->first.upper() >= key.upper();
	}

	template <typename Key2>
	[[nodiscard]] bool contains(Range<Key2> key) const
	{
		return contains(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] bool contains(K key) const
	{
		return contains(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] bool contains(K lower, K upper) const
	{
		return contains(key_type(lower, upper));
	}

	[[nodiscard]] std::pair<iterator, iterator> equal_range(key_type key)
	{
		typename key_compare::range_t const s_range{key.lower(), key.upper()};
		return std::equal_range(begin(), end(), s_range, comp_);
	}

	template <typename Key2>
	[[nodiscard]] std::pair<iterator, iterator> equal_range(Range<Key2> key)
	{
		return equal_range(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] std::pair<iterator, iterator> equal_range(K key)
	{
		return equal_range(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] std::pair<iterator, iterator> equal_range(K lower, K upper)
	{
		return equal_range(key_type(lower, upper));
	}

	[[nodiscard]] std::pair<const_iterator, const_iterator> equal_range(key_type key) const
	{
		typename key_compare::range_t const s_range{key.lower(), key.upper()};
		return std::equal_range(begin(), end(), s_range, comp_);
	}

	template <typename Key2>
	[[nodiscard]] std::pair<const_iterator, const_iterator> equal_range(
	    Range<Key2> key) const
	{
		return equal_range(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] std::pair<const_iterator, const_iterator> equal_range(K key) const
	{
		return equal_range(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] std::pair<const_iterator, const_iterator> equal_range(K lower,
	                                                                    K upper) const
	{
		return equal_range(key_type(lower, upper));
	}

	[[nodiscard]] iterator lower_bound(key_type key)
	{
		typename key_compare::range_t const s_range{key.lower(), key.upper()};
		return std::lower_bound(begin(), end(), s_range, comp_);
	}

	template <typename Key2>
	[[nodiscard]] iterator lower_bound(Range<Key2> key)
	{
		return lower_bound(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] iterator lower_bound(K key)
	{
		return lower_bound(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] iterator lower_bound(K lower, K upper)
	{
		return lower_bound(key_type(lower, upper));
	}

	[[nodiscard]] const_iterator lower_bound(key_type key) const
	{
		typename key_compare::range_t const s_range{key.lower(), key.upper()};
		return std::lower_bound(begin(), end(), s_range, comp_);
	}

	template <typename Key2>
	[[nodiscard]] const_iterator lower_bound(Range<Key2> key) const
	{
		return lower_bound(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] const_iterator lower_bound(K key) const
	{
		return lower_bound(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] const_iterator lower_bound(K lower, K upper) const
	{
		return lower_bound(key_type(lower, upper));
	}

	[[nodiscard]] iterator upper_bound(key_type key)
	{
		typename key_compare::range_t const s_range{key.lower(), key.upper()};
		return std::upper_bound(begin(), end(), s_range, comp_);
	}

	template <typename Key2>
	[[nodiscard]] iterator upper_bound(Range<Key2> key)
	{
		return upper_bound(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] iterator upper_bound(K key)
	{
		return upper_bound(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] iterator upper_bound(K lower, K upper)
	{
		return upper_bound(key_type(lower, upper));
	}

	[[nodiscard]] const_iterator upper_bound(key_type key) const
	{
		typename key_compare::range_t const s_range{key.lower(), key.upper()};
		return std::upper_bound(begin(), end(), s_range, comp_);
	}

	template <typename Key2>
	[[nodiscard]] const_iterator upper_bound(Range<Key2> key) const
	{
		return upper_bound(key_type(key));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] const_iterator upper_bound(K value) const
	{
		return upper_bound(key_type(value));
	}

	template <typename K, typename = std::enable_if_t<std::is_arithmetic_v<K>>>
	[[nodiscard]] const_iterator upper_bound(K lower, K upper) const
	{
		return upper_bound(key_type(lower, upper));
	}

	//
	// (De)serialize
	//

	std::ostream &writeData(std::ostream &out_stream) const
	{
		size_type num = numRanges();
		out_stream.write(reinterpret_cast<char *>(&num), sizeof(size_type));
		// FIXME: Add comp_
		// FIXME: Does this work?
		return out_stream.write(reinterpret_cast<char *>(ranges_.data()),
		                        sizeof(value_type) * num);
	}

	std::istream &readData(std::istream &in_stream)
	{
		size_type num;
		in_stream.read(reinterpret_cast<char *>(&num), sizeof(size_type));
		if (0 == num) {
			clear();
			return in_stream;
		}
		// FIXME: Add comp_
		ranges_.resize(num);
		// FIXME: Does this work?
		return in_stream.read(reinterpret_cast<char *>(ranges_.data()),
		                      sizeof(value_type) * num);
	}

	//
	// Observers
	//

	key_compare key_comp() const
	{
		// FIXME: Implement correct
		return key_compare();
	}

	value_compare value_comp() const { return comp_; }

	//
	// Friends
	//

	template <typename K, typename M>
	friend bool operator==(RangeMap<K, M> const &lhs, RangeMap<K, M> const &rhs);
	template <typename K, typename M>
	friend bool operator!=(RangeMap<K, M> const &lhs, RangeMap<K, M> const &rhs);
	template <typename K, typename M>
	friend bool operator<(RangeMap<K, M> const &lhs, RangeMap<K, M> const &rhs);
	template <typename K, typename M>
	friend bool operator<=(RangeMap<K, M> const &lhs, RangeMap<K, M> const &rhs);
	template <typename K, typename M>
	friend bool operator>(RangeMap<K, M> const &lhs, RangeMap<K, M> const &rhs);
	template <typename K, typename M>
	friend bool operator>=(RangeMap<K, M> const &lhs, RangeMap<K, M> const &rhs);

	template <typename K, typename M>
	friend void swap(RangeMap<K, M> &lhs,
	                 RangeMap<K, M> &rhs) noexcept(noexcept(lhs.swap(rhs)));

	template <typename K, typename M, class Pred>
	friend size_type erase_if(RangeMap<K, M> &range_set, Pred pred);

	template <typename K, typename M>
	friend std::ostream &operator<<(std::ostream &os, RangeMap<K, M> const &range_map);

 protected:
	static constexpr Key increment(Key value)
	{
		if constexpr (std::is_floating_point_v<Key>) {
			return std::nextafter(value, std::numeric_limits<Key>::max());
		} else {
			return std::numeric_limits<Key>::max() == value ? std::numeric_limits<Key>::max()
			                                                : value + 1;
		}
	}

	static constexpr Key decrement(Key value)
	{
		if constexpr (std::is_floating_point_v<Key>) {
			return std::nextafter(value, std::numeric_limits<Key>::lowest());
		} else {
			return std::numeric_limits<Key>::min() == value ? std::numeric_limits<Key>::min()
			                                                : value - 1;
		}
	}

 protected:
	std::vector<value_type> ranges_;
	value_compare comp_;
};

template <typename Key, typename T>
bool operator==(RangeMap<Key, T> const &lhs, RangeMap<Key, T> const &rhs)
{
	// FIXME: Implement correctly
	return lhs.ranges_ == rhs.ranges_;
}

template <typename Key, typename T>
bool operator!=(RangeMap<Key, T> const &lhs, RangeMap<Key, T> const &rhs)
{
	// FIXME: Implement correctly
	return lhs.ranges_ != rhs.ranges_;
}

template <typename Key, typename T>
bool operator<(RangeMap<Key, T> const &lhs, RangeMap<Key, T> const &rhs)
{
	// FIXME: Implement correctly
	return lhs.ranges_ < rhs.ranges_;
}

template <typename Key, typename T>
bool operator<=(RangeMap<Key, T> const &lhs, RangeMap<Key, T> const &rhs)
{
	// FIXME: Implement correctly
	return lhs.ranges_ <= rhs.ranges_;
}

template <typename Key, typename T>
bool operator>(RangeMap<Key, T> const &lhs, RangeMap<Key, T> const &rhs)
{
	// FIXME: Implement correctly
	return lhs.ranges_ > rhs.ranges_;
}

template <typename Key, typename T>
bool operator>=(RangeMap<Key, T> const &lhs, RangeMap<Key, T> const &rhs)
{
	// FIXME: Implement correctly
	return lhs.ranges_ >= rhs.ranges_;
}

template <typename Key, typename T>
void swap(RangeMap<Key, T> &lhs, RangeMap<Key, T> &rhs) noexcept(noexcept(lhs.swap(rhs)))
{
	lhs.swap(rhs);
}

template <typename Key, typename T, class Pred>
typename RangeMap<Key, T>::size_type erase_if(RangeMap<Key, T> &range_map, Pred pred)
{
	auto old_size = range_map.size();
	for (auto it = std::begin(range_map), last = std::end(range_map); it != last;) {
		if (pred(*it)) {
			it = range_map.erase(it);
		} else {
			++it;
		}
	}
	return old_size - range_map.size();
}

template <typename Key, typename T>
std::ostream &operator<<(std::ostream &os, RangeMap<Key, T> const &range_map)
{
	if (!range_map.empty()) {
		// FIXME: Make nicer?
		auto it = std::cbegin(range_map);
		for (; it != std::prev(std::cend(range_map)); ++it) {
			os << it->first << " = " << it->second << ", ";
		}
		os << it->first << " = " << it->second;
	}
	return os;
}

}  // namespace ufo::container

#endif  // UFO_CONTAINER_RANGE_H