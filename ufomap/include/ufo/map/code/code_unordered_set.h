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

#ifndef UFO_MAP_CODE_UNORDERED_SET_H
#define UFO_MAP_CODE_UNORDERED_SET_H

// UFO
#include <ufo/map/code/code.h>
#include <ufo/map/code/node_handle.h>

// STL
#include <algorithm>
#include <forward_list>
#include <unordered_set>
#include <vector>

namespace ufo::map
{
using CodeUnorderedSet = std::unordered_set<Code, Code::Hash>;

// template <class Allocator = std::allocator<Code>>
// class CodeUnorderedSet
// {
//  public:
// 	using key_type = Code;
// 	using value_type = Code;
// 	using size_type = std::size_t;
// 	using difference_type = std::ptrdiff_t;
// 	using hasher = Code::Hash;
// 	using key_equal = std::equal_to<Code>;
// 	using allocator_type = Allocator;
// 	using reference = value_type&;
// 	using const_reference = value_type const&;
// 	using pointer = std::forward_list<Code>::pointer;
// 	using const_pointer = std::forward_list<Code>::const_pointer;
// 	using iterator = typename std::forward_list<Code>::iterator;
// 	using const_iterator = typename std::forward_list<Code>::const_iterator;
// 	using local_iterator = iterator;
// 	using const_local_iterator = const_iterator;
// 	using node_type = _Node_handle<_Key, _Value, __node_alloc_type>;
// 	using insert_return_type = _Node_insert_return<iterator, node_type>;

// 	struct node_type {
// 	 public:
// 		using value_type = Code;
// 		using allocator_type = Allocator;

// 		//
// 		// Constructors
// 		//

// 		constexpr node_type() noexcept
// 		{
// 			// TODO: Implement
// 		}

// 		node_type(node_type&& nt) noexcept
// 		{
// 			// TODO: Implement
// 		}

// 		//
// 		// operator=
// 		//

// 		node_type& operator=(node_type&& nt)
// 		{
// 			// TODO: Implement
// 		}

// 		//
// 		// Destructor
// 		//

// 		~node_type()
// 		{
// 			if (empty()) {
// 				return;
// 			}
// 			std::allocator_traits<allocator_type>::destroy;
// 			std::allocator_traits<allocator_type>::rebind_traits<>::deallocate;
// 		}

// 		//
// 		// Empty
// 		//

// 		[[nodiscard]] bool empty() const noexcept
// 		{
// 			// TODO: Implement
// 		}

// 		explicit operator bool() const noexcept { return !empty(); }

// 		allocator_type get_allocator() const { return alloc_; }

// 		value_type& value() const
// 		{
// 			// TODO: Implement
// 		}

// 		void swap(node_type& nt) noexcept(
// 		    std::allocator_traits<allocator_type>::propagate_on_container_swap::value ||
// 		    std::allocator_traits<allocator_type>::is_always_equal::value)
// 		{
// 			// TODO: Implement
// 		}

// 		friend void swap(node_type& x, node_type& y) noexcept(noexcept(x.swap(y)))
// 		{
// 			x.swap(y);

// 			std::unordered_set<int> test;
// 			test.extract(5);
// 		}

// 	 private:
// 		allocator_type alloc_;

// 		value_type value_;

// 		bool is_empty_;
// 	};

// 	template <class Iter, class NodeType>
// 	struct insert_return_type {
// 		Iter position;
// 		bool inserted;
// 		NodeType node;
// 	};

//  public:
// 	CodeUnorderedSet() : CodeUnorderedSet(1U << 18) {}

// 	explicit CodeUnorderedSet(size_type bucket_count) { reserve(bucket_count); }

// 	CodeUnorderedSet(size_type bucket_count, Allocator const& alloc) : data_(alloc)
// 	{
// 		reserve(bucket_count);
// 	}

// 	explicit CodeUnorderedSet(Allocator const& alloc) : CodeUnorderedSet(1U << 18, alloc)
// {}

// 	template <class InputIt>
// 	CodeUnorderedSet(InputIt first, InputIt last, size_type bucket_count = 1U << 18,
// 	                 Allocator const& alloc = Allocator())
// 	    : CodeUnorderedSet(std::max(std::distance(first, last), bucket_count), alloc)
// 	{
// 		insert(first, last);
// 	}

// 	CodeUnorderedSet(CodeUnorderedSet const& other) = default;
// 	{
// 		// TODO: Implement
// 	}

// 	CodeUnorderedSet(CodeUnorderedSet const& other, Allocator const& alloc)
// 	{
// 		// TODO: Implement
// 	}

// 	CodeUnorderedSet(CodeUnorderedSet&& other) = default;

// 	CodeUnorderedSet(CodeUnorderedSet&& other, Allocator const& alloc)
// 	{
// 		// TODO: Implement
// 	}

// 	CodeUnorderedSet(std::initializer_list<value_type> init,
// 	                 size_type bucket_count = 1U << 18,
// 	                 Allocator const& alloc = Allocator())
// 	    : CodeUnorderedSet(std::max(init.size(), bucket_count), alloc)
// 	{
// 		insert(init);
// 	}

// 	CodeUnorderedSet& operator=(CodeUnorderedSet const& other) = default;

// 	CodeUnorderedSet& operator=(CodeUnorderedSet&& other) = default;

// 	CodeUnorderedSet& operator=(std::initializer_list<value_type> ilist)
// 	{
// 		clear();
// 		insert(ilist);
// 		return *this;
// 	}

// 	allocator_type get_allocator() const noexcept { return data_.get_allocator(); }

// 	//
// 	// Iterators
// 	//

// 	iterator begin() noexcept { return std::begin(data_); }

// 	const_iterator begin() const noexcept { return std::begin(data_); }

// 	const_iterator cbegin() const noexcept { return begin(); }

// 	iterator end() noexcept { return std::end(data_); }

// 	const_iterator end() const noexcept { return std::end(data_); }

// 	const_iterator cend() const noexcept { return end(); }

// 	//
// 	// Capacity
// 	//

// 	[[nodiscard]] bool empty() const noexcept { return 0 == size_; }

// 	[[nodiscard]] size_type size() const noexcept { return size_; }

// 	[[nodiscard]] size_type max_size() const noexcept { return data_.max_size(); }

// 	//
// 	// Modifiers
// 	//

// 	void clear() noexcept
// 	{
// 		data_.clear();
// 		std::for_each(std::begin(buckets_), std::end(buckets_),
// 		              [](auto& bucket) { bucket = data_.before_begin(); });
// 		size_ = 0;
// 	}

// 	std::pair<iterator, bool> insert(value_type const& value)
// 	{
// 		auto b = bucket(value);
// 		auto it = std::find(begin(b), end(b), value);
// 		if (end() != it) {
// 			return {it, false};
// 		}

// 		++size_;

// 		if (load_factor() > max_load_factor()) {
// 			rehash(bucket_count() << 1U);
// 			b = bucket(value);
// 		}

// 		auto s = bucket_size(b);

// 		auto it = data_.insert_after(begin(b), value);

// 		if (0 == s) {
// 			for (auto i = b + 1; i != bucket_count(); ++i) {
// 				if (begin(b) != begin(i)) {
// 					break;
// 				}
// 				buckets_[i] = it;
// 			}
// 		}

// 		return {it, true};
// 	}

// 	std::pair<iterator, bool> insert(value_type&& value)
// 	{
// 		auto b = bucket(value);
// 		auto it = std::find(begin(b), end(b), value);
// 		if (end() != it) {
// 			return {it, false};
// 		}

// 		++size_;

// 		if (load_factor() > max_load_factor()) {
// 			rehash(bucket_count() << 1U);
// 			b = bucket(value);
// 		}

// 		auto s = bucket_size(b);

// 		auto it = data_.insert_after(begin(b), std::move(value));

// 		if (0 == s) {
// 			for (auto i = b + 1; i != bucket_count(); ++i) {
// 				if (begin(b) != begin(i)) {
// 					break;
// 				}
// 				buckets_[i] = it;
// 			}
// 		}

// 		return {it, true};
// 	}

// 	iterator insert(const_iterator hint, value_type const& value)
// 	{
// 		return insert(value).first;
// 	}

// 	iterator insert(const_iterator hint, value_type&& value)
// 	{
// 		return insert(std::move(value)).first;
// 	}

// 	template <class InputIt>
// 	void insert(InputIt first, InputIt last)
// 	{
// 		// FIXME: Optimize
// 		std::for_each(first, last, [this](auto&& elem) { insert(elem); });
// 	}

// 	void insert(std::initializer_list<value_type> ilist)
// 	{
// 		// FIXME: Optimize
// 		insert(std::begin(ilist), std::end(ilist));
// 	}

// 	insert_return_type insert(node_type&& nh)
// 	{
// 		// TODO: Implement
// 	}

// 	iterator insert(const_iterator hint, node_type&& nh)
// 	{
// 		// FIXME: Use hint
// 		return insert(std::move(nh));
// 	}

// 	template <class... Args>
// 	std::pair<iterator, bool> emplace(Args&&... args)
// 	{
// 		// FIXME: Optimize
// 		return insert(Code(args...));
// 	}

// 	template <class... Args>
// 	iterator emplace_hint(const_iterator hint, Args&&... args)
// 	{
// 		return emplace(std::forward<Args>(args...));
// 	}

// 	iterator erase(const_iterator pos)
// 	{
// 		auto b = bucket(*pos);

// 		iterator one_before = begin(b);
// 		if (one_before == pos) {
// 			one_before = buckets_[b];
// 		} else {
// 			for (auto n = std::next(one_before); n != pos; ++n) {
// 				one_before = n;
// 			}
// 		}

// 		if (end(b) == std::next(pos)) {
// 			for (auto i = b + 1; i != bucket_count(); ++i) {
// 				if (begin(i) != end(b)) {
// 					break;
// 				}
// 				buckets_[i] = one_before;
// 			}
// 		}

// 		return data_.erase_after(one_before);
// 	}

// 	iterator erase(const_iterator first, const_iterator last)
// 	{
// 		while (first != last) {
// 			first = erase(first);
// 		}
// 	}

// 	size_type erase(Key const& key)
// 	{
// 		auto it = find(key);
// 		return end() == it ? 0 : erase(it), 1;
// 	}

// 	void swap(CodeUnorderedSet& other) noexcept(
// 	    std::allocator_traits<Allocator>::is_always_equal::value)
// 	{
// 		std::swap(buckets_, other.buckets_);
// 		std::swap(data_, other.data_);
// 		std::swap(size_, other.size_);
// 		std::swap(max_load_factor_, other.max_load_factor_);
// 	}

// 	node_type extract(const_iterator position)
// 	{
// 		// TODO: Implement
// 	}

// 	node_type extract(Key const& k)
// 	{
// 		// TODO: Implement
// 	}

// 	void merge(CodeUnorderedSet& source)
// 	{
// 		// TODO: Implement
// 	}

// 	void merge(CodeUnorderedSet&& source)
// 	{
// 		// TODO: Implement
// 	}

// 	//
// 	// Lookup
// 	//

// 	[[nodiscard]] size_type count(Key const& key) const
// 	{
// 		return end() == find(key) ? 0 : 1;
// 	}

// 	[[nodiscard]] iterator find(Key const& key)
// 	{
// 		auto b = bucket(key);
// 		auto it = std::find(begin(b), end(b), key);
// 		return end(b) == it ? end() : it;
// 	}

// 	[[nodiscard]] const_iterator find(Key const& key) const
// 	{
// 		auto b = bucket(key);
// 		auto it = std::find(begin(b), end(b), key);
// 		return end(b) == it ? end() : it;
// 	}

// 	[[nodiscard]] bool contains(Key const& key) const { return 0 != count(key); }

// 	std::pair<iterator, iterator> equal_range(Key const& key)
// 	{
// 		auto it = find(key);
// 		return end() == it ? {it, it} : {it, std::next(it)};
// 	}

// 	std::pair<const_iterator, const_iterator> equal_range(Key const& key) const
// 	{
// 		auto it = find(key);
// 		return end() == it ? {it, it} : {it, std::next(it)};
// 	}

// 	//
// 	// Bucket interface
// 	//

// 	local_iterator begin(size_type n) { return std::next(buckets_[n]); }

// 	const_local_iterator begin(size_type n) const { return std::next(buckets_[n]); }

// 	const_local_iterator cbegin(size_type n) const { return begin(n); }

// 	local_iterator end(size_type n)
// 	{
// 		return bucket_count() == n + 1 ? end() : std::next(begin(n + 1));
// 	}

// 	const_local_iterator end(size_type n) const
// 	{
// 		return bucket_count() == n + 1 ? end() : std::next(begin(n + 1));
// 	}

// 	const_local_iterator cend(size_type n) const { return end(n); }

// 	[[nodiscard]] size_type bucket_count() const { return buckets_.size(); }

// 	[[nodiscard]] size_type max_bucket_count() const { return buckets_.max_size(); }

// 	[[nodiscard]] size_type bucket_size(size_type n) const
// 	{
// 		return std::distance(begin(n), end(n));
// 	}

// 	[[nodiscard]] size_type bucket(Key const& key) const
// 	{
// 		unsigned int offset = 3U * key.depth();
// 		auto hash = key.code();
// 		decltype(hash) modder = bucket_count() - 1;
// 		return (hash & (modder << offset)) >> offset;
// 	}

// 	//
// 	// Hash policy
// 	//

// 	float load_factor() const { return size_ / static_cast<float>(bucket_count()); }

// 	float max_load_factor() const { return max_load_factor_; }

// 	void max_load_factor(float ml)
// 	{
// 		max_load_factor_ = max_load_factor;

// 		if (load_factor() > max_load_factor_ && power_ < MAX_POWER) {
// 			rehash(num_buckets_ * 2);
// 		}
// 	}

// 	void rehash(size_type count)
// 	{
// 		size_type min_count = std::max(static_cast<float>(count), size() /
// max_load_factor()); 		unsigned int power = 		    std::min(static_cast<unsigned
// int>(std::log2(min_count) + 1), MAX_POWER);

// 		size_type num_buckets = size_type(1) << power;

// 		if (bucket_count() == num_buckets) {
// 			return;
// 		}

// 		std::fill(std::begin(buckets_), std::end(buckets_), data_.before_begin());
// 		buckets_.resize(num_buckets, data_.before_begin());

// 		for (auto one_before = data_.before_begin();;) {
// 			auto it = std::next(one_before);
// 			if (end() == it) {
// 				break;
// 			}

// 			// TODO: Insert

// 			one_before = data_.erase_after(one_before);
// 		}
// 	}

// 	void reserve(size_type count) { rehash(std::ceil(count / max_load_factor())); }

// 	//
// 	// Observer
// 	//

// 	hasher hash_function() const
// 	{
// 		// TODO: Implement
// 	}

// 	key_equal key_eq() const
// 	{
// 		// TODO: Implement
// 	}

// 	//
// 	// Non-member functions
// 	//

// 	template <class Alloc>
// 	friend bool operator==(CodeUnorderedSet<Alloc> const& lhs,
// 	                       CodeUnorderedSet<Alloc> const& rhs)
// 	{
// 		// FIXME: Optimize

// 		if (lhs.size() != rhs.size()) {
// 			return false;
// 		}

// 		for (auto elem : lhs.data_) {
// 			if (!rhs.contains(elem)) {
// 				return false;
// 			}
// 		}

// 		return true;
// 	}

// 	template <class Alloc>
// 	friend bool operator!=(CodeUnorderedSet<Alloc> const& lhs,
// 	                       CodeUnorderedSet<Alloc> const& rhs)
// 	{
// 		return !(lhs == rhs);
// 	}

// 	template <class Alloc>
// 	friend void swap(CodeUnorderedSet<Alloc>& lhs,
// 	                 CodeUnorderedSet<Alloc>& rhs) noexcept(noexcept(lhs.swap(rhs)))
// 	{
// 		lhs.swap(rhs);
// 	}

// 	template <class Alloc, class Pred>
// 	friend typename CodeUnorderedSet<Alloc>::size_type erase_if(CodeUnorderedSet<Alloc>&
// c, 	                                                            Pred pred)
// 	{
// 		auto old_size = c.size();
// 		for (auto it = std::begin(c), last = std::end(c); i != last;) {
// 			if (pred(*it)) {
// 				it = c.erase(it);
// 			} else {
// 				++it;
// 			}
// 		}
// 		return old_size - c.size();
// 	}

//  private:
// 	std::vector<iterator> buckets_;
// 	std::forward_list<Code, Allocator> data_;

// 	size_type size_ = 0;
// 	float max_load_factor_ = 1.0;

// 	static constexpr unsigned int MAX_POWER = 28;

// 	friend struct Iterator;
// };
}  // namespace ufo::map

#endif  // UFO_MAP_CODE_UNORDERED_SET_H