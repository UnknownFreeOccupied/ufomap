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

#ifndef UFO_CONTAINER_RANGE_SET_H
#define UFO_CONTAINER_RANGE_SET_H

// UFO
#include <ufo/container/range.h>

namespace ufo::container
{
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
		std::for_each(first, last, [this](auto &&... args) { this->insert(args...); });
	}

	void insert(std::initializer_list<value_type> ilist)
	{
		insert(std::cbegin(ilist), std::cend(ilist));
	}

	// FIXME: insert_return_type insert(node_type&& nh);

	// FIXME: iterator insert(const_iterator hint, node_type&& nh);

	template <class... Args>
	std::pair<iterator, bool> emplace(Args &&... args)
	{
		return insert(value_type(std::forward<Args>(args)...));
	}

	template <class... Args>
	iterator emplace_hint(const_iterator hint, Args &&... args)
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
}  // namespace ufo::container

#endif  // UFO_CONTAINER_RANGE_SET_H