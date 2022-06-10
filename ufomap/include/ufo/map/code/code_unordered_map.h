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

#ifndef UFO_MAP_CODE_UNORDERED_MAP_H
#define UFO_MAP_CODE_UNORDERED_MAP_H

// UFO
#include <ufo/map/code/code.h>

// STL
#include <unordered_map>

namespace ufo::map
{
template <typename T>
using CodeUnorderedMap = std::unordered_map<Code, T>;

// template <class T>
// class CodeMap
// {
//  public:
// 	CodeMap(unsigned int power = 18) : power_(power)
// 	{
// 		num_buckets_ = std::size_t(1) << power_;  // std::pow(2, power_);
// 		data_.resize(num_buckets_);
// 	}

// 	struct CodeMapIterator {
// 		// Tags
// 		// FIXME: Fix these
// 		using iterator_category = std::forward_iterator_tag;
// 		using difference_type = std::ptrdiff_t;
// 		using value_type = std::pair<Code, T>;
// 		using pointer = value_type*;    // or also value_type*
// 		using reference = value_type&;  // or also value_type&
// 		using const_pointer = value_type const*;
// 		using const_reference = value_type const&;

// 		CodeMapIterator(const CodeMap* map = nullptr) : map_(map)
// 		{
// 			if (nullptr == map_) {
// 				return;
// 			}

// 			if (map_->data_.empty()) {
// 				map_ = nullptr;
// 			} else {
// 				outer_iter_end_ = map_->data_.cend();
// 				outer_iter_ = std::find_if(map_->data_.cbegin(), outer_iter_end_,
// 				                           [](auto const& e) { return !e.empty(); });
// 				if (outer_iter_ == outer_iter_end_) {
// 					map_ = nullptr;
// 				} else {
// 					inner_iter_ = outer_iter_->cbegin();
// 					inner_iter_end_ = outer_iter_->cend();
// 				}
// 			}
// 		}

// 		std::pair<Code, T> const& operator*() const
// 		{
// 			return *inner_iter_;  // map_->data_[outer_index_][inner_index_];
// 		}

// 		std::pair<Code, T> operator*()
// 		{
// 			return *inner_iter_;  // map_->data_[outer_index_][inner_index_];
// 		}

// 		// Postfix increment
// 		CodeMapIterator operator++(int)
// 		{
// 			CodeMapIterator result = *this;
// 			++(*this);
// 			return result;
// 		}

// 		// Prefix increment
// 		CodeMapIterator& operator++()
// 		{
// 			++inner_iter_;
// 			if (inner_iter_ == inner_iter_end_) {
// 				++outer_iter_;
// 				outer_iter_ = std::find_if(outer_iter_, outer_iter_end_,
// 				                           [](auto const& e) { return !e.empty(); });
// 				if (outer_iter_ == outer_iter_end_) {
// 					map_ = nullptr;
// 				} else {
// 					inner_iter_ = outer_iter_->cbegin();
// 					inner_iter_end_ = outer_iter_->cend();
// 				}
// 			}
// 			return *this;
// 		}

// 		bool operator==(CodeMapIterator const& rhs) const { return (rhs.map_ == map_); }

// 		bool operator!=(CodeMapIterator const& rhs) const { return (rhs.map_ != map_); }

// 	 private:
// 		const CodeMap* map_;
// 		typename std::vector<std::vector<std::pair<Code, T>>>::const_iterator outer_iter_;
// 		typename std::vector<std::vector<std::pair<Code, T>>>::const_iterator
// outer_iter_end_; 		typename std::vector<std::pair<Code, T>>::const_iterator
// inner_iter_; 		typename std::vector<std::pair<Code, T>>::const_iterator inner_iter_end_;
// 		// typename decltype(CodeMap<T>::data_)::const_iterator outer_iter_;
// 		// typename decltype(CodeMap<T>::data_)::const_iterator outer_iter_end_;
// 		// typename decltype(CodeMap<T>::data_)::value_type::const_iterator inner_iter_;
// 		// typename decltype(CodeMap<T>::data_)::value_type::const_iterator inner_iter_end_;
// 	};

// 	struct CodeMapBucketIterator {
// 		// Tags
// 		// FIXME: Fix these
// 		using iterator_category = std::forward_iterator_tag;
// 		using difference_type = std::ptrdiff_t;
// 		using value_type = std::vector<std::pair<Code, T>>;
// 		using pointer = value_type*;    // or also value_type*
// 		using reference = value_type&;  // or also value_type&
// 		using const_pointer = value_type const*;
// 		using const_reference = value_type const&;

// 		CodeMapBucketIterator(const CodeMap* map = nullptr) : map_(map)
// 		{
// 			if (nullptr == map_) {
// 				return;
// 			}

// 			if (map_->data_.empty()) {
// 				map_ = nullptr;
// 			} else {
// 				outer_iter_end_ = map_->data_.cend();
// 				outer_iter_ = std::find_if(map_->data_.cbegin(), outer_iter_end_,
// 				                           [](auto const& e) { return !e.empty(); });
// 				if (outer_iter_ == outer_iter_end_) {
// 					map_ = nullptr;
// 				}
// 			}
// 		}

// 		const_reference operator*() const
// 		{
// 			return *outer_iter_;  // map_->data_[outer_index_][inner_index_];
// 		}

// 		value_type operator*()
// 		{
// 			return *outer_iter_;  // map_->data_[outer_index_][inner_index_];
// 		}

// 		// Postfix increment
// 		CodeMapBucketIterator operator++(int)
// 		{
// 			CodeMapBucketIterator result = *this;
// 			++(*this);
// 			return result;
// 		}

// 		// Prefix increment
// 		CodeMapBucketIterator& operator++()
// 		{
// 			++outer_iter_;
// 			outer_iter_ = std::find_if(outer_iter_, outer_iter_end_,
// 			                           [](auto const& e) { return !e.empty(); });
// 			if (outer_iter_ == outer_iter_end_) {
// 				map_ = nullptr;
// 			}
// 			return *this;
// 		}

// 		bool operator==(CodeMapBucketIterator const& rhs) const { return (rhs.map_ == map_);
// }

// 		bool operator!=(CodeMapBucketIterator const& rhs) const { return (rhs.map_ != map_);
// }

// 	 private:
// 		const CodeMap* map_;
// 		typename std::vector<std::vector<std::pair<Code, T>>>::const_iterator outer_iter_;
// 		typename std::vector<std::vector<std::pair<Code, T>>>::const_iterator
// outer_iter_end_;
// 	};

// 	T& operator[](Code const& key)
// 	{
// 		std::size_t hash = getBucket(key);
// 		auto it = std::find_if(std::begin(data_[hash]), std::end(data_[hash]),
// 		                       [&key](auto const& elem) { return key == elem.first; });
// 		if (it != std::end(data_[hash])) {
// 			return it->second;
// 		}

// 		++size_;

// 		if (load_factor() > max_load_factor() && power_ < MAX_POWER) {
// 			rehash(num_buckets_ * 2);
// 			hash = getBucket(key);
// 		}

// 		return std::get<1>(data_[hash].emplace_back(key, T()));  // FIXME: How to
// 		                                                         // call default?
// 	}

// 	std::pair<int, bool> try_emplace(Code const& key,
// 	                                 T const& value)  // TODO: Fix
// 	{
// 		std::size_t hash = getBucket(key);
// 		if (std::any_of(std::cbegin(data_[hash]), std::cend(data_[hash]),
// 		                [&key](auto const& elem) { return key == elem.first; })) {
// 			return {0, false};  // TODO: Fix
// 		}

// 		++size_;

// 		if (load_factor() > max_load_factor() && power_ < MAX_POWER) {
// 			rehash(num_buckets_ * 2);
// 			hash = getBucket(key);
// 		}

// 		data_[hash].emplace_back(key, value);

// 		return {0, true};  // TODO: Fix
// 	}

// 	void clear()
// 	{
// 		std::for_each(std::begin(data_), std::end(data_),
// 		              [](auto& bucket) { bucket.clear(); });
// 		size_ = 0;
// 	}

// 	bool empty() const { return 0 == size_; }

// 	constexpr std::size_t size() const { return size_; }

// 	std::size_t bucket_count() const { return num_buckets_; }

// 	unsigned int bucket_count_power() const { return power_; }

// 	double load_factor() const { return size_ / static_cast<double>(num_buckets_); }

// 	double max_load_factor() const { return max_load_factor_; }

// 	void max_load_factor(double max_load_factor)
// 	{
// 		max_load_factor_ = max_load_factor;

// 		if (load_factor() > max_load_factor_ && power_ < MAX_POWER) {
// 			rehash(num_buckets_ * 2);
// 		}
// 	}

// 	void rehash(std::size_t count)
// 	{
// 		std::size_t min_count =
// 		    std::max(static_cast<double>(count), size() / max_load_factor());
// 		unsigned int power = std::max(
// 		    power_, std::min(static_cast<unsigned int>(std::log2(min_count) + 1),
// MAX_POWER));

// 		if (power_ == power) {
// 			return;
// 		}

// 		power_ = power;
// 		num_buckets_ = std::size_t(1) << power_;

// 		decltype(data_) new_data;
// 		new_data.resize(num_buckets_);

// 		for (const auto& [key, value] : *this) {
// 			new_data[getBucket(key)].emplace_back(key, value);
// 		}

// 		data_.swap(new_data);
// 		// fprintf(stderr, "\n\nRehash, new power: %u\n\n", power_);
// 	}

// 	void reserve(std::size_t count)
// 	{
// 		power_ = std::max(power_, std::min((unsigned int)std::log2(count) + 1, MAX_POWER));
// 		data_.reserve(std::size_t(1) << power_);
// 	}

// 	CodeMapIterator begin() const { return CodeMapIterator(this); }

// 	CodeMapIterator end() const { return CodeMapIterator(); }

// 	CodeMapBucketIterator beginBuckets() const { return CodeMapBucketIterator(this); }

// 	CodeMapBucketIterator endBuckets() const { return CodeMapBucketIterator(); }

// 	void swap(CodeMap<T>& other)
// 	{
// 		data_.swap(other.data_);
// 		std::swap(power_, other.power_);
// 		std::swap(num_buckets_, other.num_buckets_);
// 		std::swap(size_, other.size_);
// 		std::swap(max_load_factor_, other.max_load_factor_);
// 	}

//  private:
// 	std::size_t getBucket(Code const& key) const
// 	{
// 		unsigned int offset = 3 * key.depth();
// 		unsigned int modder = (bucket_count() - 1) << offset;
// 		return (Code::Hash()(key) & modder) >> offset;
// 	}

//  private:
// 	std::vector<std::vector<std::pair<Code, T>>> data_;
// 	unsigned int power_;
// 	std::size_t num_buckets_;
// 	std::size_t size_ = 0;
// 	double max_load_factor_ = 1.0;

// 	static constexpr unsigned int MAX_POWER = 28;

// 	friend struct CodeMapIterator;
// };
}  // namespace ufo::map

#endif  // UFO_MAP_CODE_UNORDERED_MAP_H