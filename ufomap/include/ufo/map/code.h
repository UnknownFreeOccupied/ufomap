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

#ifndef UFO_MAP_CODE_H
#define UFO_MAP_CODE_H

// UFO
#include <ufo/map/key.h>
#include <ufo/map/types.h>

// STL
#include <immintrin.h>

#include <algorithm>
#include <execution>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace ufo::map
{
/*!
 * @brief A code is a single value for indexing a specific node in an octree at
 * a specific depth
 *
 * @details Morton codes are used in UFOMap to increase performance when
 * accessing the octree
 *
 */
class Code
{
 public:
	using CodeType = uint64_t;

	constexpr Code() = default;

	constexpr Code(CodeType code, Depth depth = 0)
	    : code_((code >> depth) << depth), depth_(depth)
	{
	}

	constexpr Code(Key const& key) : code_(toCode(key)), depth_(key.depth()) {}

	/*!
	 * @brief Get the corresponding key to code
	 *
	 * @return The corresponding key to code
	 */
	constexpr operator Key() const noexcept
	{
#if defined(__BMI2__)
		return Key(_pext_u64(code_, 0x9249249249249249), _pext_u64(code_, 0x2492492492492492),
		           _pext_u64(code_, 0x4924924924924924), depth_);
#else
		return Key(toKey(code_, 0), toKey(code_, 1), toKey(code_, 2), depth_);
#endif
	}

	constexpr bool operator==(Code const& rhs) const
	{
		return code_ == rhs.code_ && depth_ == rhs.depth_;
	}

	constexpr bool operator!=(Code const& rhs) const { return !(*this == rhs); }

	constexpr bool operator<(Code const& rhs) const
	{
		return code_ < rhs.code_ || (code_ == rhs.code_ && depth_ > rhs.depth_);
	}

	constexpr bool operator<=(Code const& rhs) const
	{
		return code_ < rhs.code_ || (code_ == rhs.code_ && depth_ >= rhs.depth_);
	}

	constexpr bool operator>(Code const& rhs) const
	{
		return code_ > rhs.code_ || (code_ == rhs.code_ && depth_ < rhs.depth_);
	}

	constexpr bool operator>=(Code const& rhs) const
	{
		return code_ > rhs.code_ || (code_ == rhs.code_ && depth_ <= rhs.depth_);
	}

	/*!
	 * @brief Return the code at a specified depth
	 *
	 * @param depth The depth of the code
	 * @return Code The code at the specified depth
	 */
	constexpr Code toDepth(Depth depth) const
	{
		CodeType temp = 3 * depth;
		return Code((code_ >> temp) << temp, depth);
	}

	/*!
	 * @brief Converts a key to a code
	 *
	 * @param key The key to convert
	 * @return uint64_t The code corresponding to the key
	 */
	static constexpr CodeType toCode(Key const& key)
	{
#if defined(__BMI2__)
		return _pdep_u64(static_cast<CodeType>(key[0]), 0x9249249249249249) |
		       _pdep_u64(static_cast<CodeType>(key[1]), 0x2492492492492492) |
		       _pdep_u64(static_cast<CodeType>(key[2]), 0x4924924924924924);
#else
		return splitBy3(key[0]) | (splitBy3(key[1]) << 1) | (splitBy3(key[2]) << 2);
#endif
	}

	/*!
	 * @brief Get the key component from a code
	 *
	 * @param code The code to generate the key component from
	 * @param index The index of the key component
	 * @return The key component value
	 */
	static Key::KeyType toKey(Code const& code, std::size_t index)
	{
		return get3Bits(code.code_ >> index);
	}

	/*!
	 * @brief Get the key component from this code
	 *
	 * @param index The index of the key component
	 * @return The key component value
	 */
	Key::KeyType toKey(std::size_t index) const { return toKey(*this, index); }

	/*!
	 * @brief Get the index at a specific depth for this code.
	 *
	 * @param depth The depth the index is requested for.
	 * @return The index at the specified depth.
	 */
	constexpr std::size_t indexAtDepth(Depth depth) const
	{
		return (code_ >> static_cast<CodeType>(3 * depth)) & ((CodeType)0x7);
	}

	/*!
	 * @brief Get the code of a specific child to this code
	 *
	 * @param index The index of the child
	 * @return Code The child code
	 */
	constexpr Code child(std::size_t index) const
	{
		if (0 == depth_) {
			// FIXME: Throw error?
			return *this;
		}

		Depth child_depth = depth_ - 1;
		return Code(
		    code_ + (static_cast<CodeType>(index) << static_cast<CodeType>(3 * child_depth)),
		    child_depth);
	}

	/*!
	 * @brief Get the code of a specific sibling to this code
	 *
	 * @param index The index of the sibling
	 * @return Code The sibling code
	 */
	inline Code sibling(std::size_t index) const
	{
		CodeType sibling_code = (code_ >> static_cast<CodeType>(3 * (depth_ + 1)))
		                        << static_cast<CodeType>(3 * (depth_ + 1));
		return Code(sibling_code +
		                (static_cast<CodeType>(index) << static_cast<CodeType>(3 * depth_)),
		            depth_);
	}

	/*!
	 * @brief Get the code
	 *
	 * @return CodeType The code
	 */
	constexpr CodeType code() const noexcept { return code_; }

	/*!
	 * @brief Get the depth that this code is specified at
	 *
	 * @return Depth The depth this code is specified at
	 */
	constexpr Depth depth() const noexcept { return depth_; }

	/*!
	 * @brief
	 *
	 */
	struct Hash {
		static constexpr CodeType hash(Code const& code) { return code.code(); }

		constexpr CodeType operator()(Code const& code) const { return hash(code); }

		static constexpr bool equal(Code const& lhs, Code const& rhs) { return lhs == rhs; }
	};

 private:
	static CodeType splitBy3(Key::KeyType a)
	{
#if defined(__BMI2__)
		return _pdep_u64(static_cast<CodeType>(a), 0x9249249249249249);
#else
		CodeType code = static_cast<CodeType>(a) & 0x1fffff;
		code = (code | code << 32) & 0x1f00000000ffff;
		code = (code | code << 16) & 0x1f0000ff0000ff;
		code = (code | code << 8) & 0x100f00f00f00f00f;
		code = (code | code << 4) & 0x10c30c30c30c30c3;
		code = (code | code << 2) & 0x1249249249249249;
		return code;
#endif
	}

	static Key::KeyType get3Bits(CodeType code)
	{
#if defined(__BMI2__)
		return static_cast<Key::KeyType>(_pext_u64(code, 0x9249249249249249));
#else
		CodeType a = code & 0x1249249249249249;
		a = (a ^ (a >> 2)) & 0x10c30c30c30c30c3;
		a = (a ^ (a >> 4)) & 0x100f00f00f00f00f;
		a = (a ^ (a >> 8)) & 0x1f0000ff0000ff;
		a = (a ^ (a >> 16)) & 0x1f00000000ffff;
		a = (a ^ a >> 32) & 0x1fffff;
		return static_cast<Key::KeyType>(a);
#endif
	}

 private:
	// The Morton code
	CodeType code_ = 0;
	// The depth of the Morton code
	Depth depth_ = 0;
};

// using CodeSet = std::unordered_set<Code, Code::Hash>;
// template <typename T>
// using CodeMap = std::unordered_map<Code, T, Code::Hash>;
using CodeRay = std::vector<Code>;

class CodeSet
{
 public:
	CodeSet(unsigned int power = 18) : power_(power)
	{
		num_buckets_ = size_t(1) << power_;
		data_.resize(num_buckets_);
	}

	struct CodeSetIterator {
		CodeSetIterator(CodeSet const* set = nullptr) : set_(set)
		{
			if (!set_) {
				return;
			}

			if (set_->data_.empty()) {
				set_ = nullptr;
			} else {
				outer_iter_ = std::begin(set_->data_);
				outer_iter_end_ = std::end(set_->data_);
				while (outer_iter_ != outer_iter_end_ && outer_iter_->empty()) {
					++outer_iter_;
				}
				if (outer_iter_ == outer_iter_end_) {
					set_ = nullptr;
				} else {
					inner_iter_ = std::begin(*outer_iter_);
					inner_iter_end_ = std::end(*outer_iter_);
				}
			}
		}

		Code const& operator*() const { return *inner_iter_; }

		// Postfix increment
		CodeSetIterator operator++(int)
		{
			CodeSetIterator result = *this;
			++(*this);
			return result;
		}

		// Prefix increment
		CodeSetIterator& operator++()
		{
			++inner_iter_;
			if (inner_iter_ == inner_iter_end_) {
				++outer_iter_;
				while (outer_iter_ != outer_iter_end_ && outer_iter_->empty()) {
					++outer_iter_;
				}
				if (outer_iter_ == outer_iter_end_) {
					set_ = nullptr;
				} else {
					inner_iter_ = outer_iter_->begin();
					inner_iter_end_ = outer_iter_->end();
				}
			}
			return *this;
		}

		bool operator==(CodeSetIterator const& rhs) const { return (rhs.set_ == set_); }

		bool operator!=(CodeSetIterator const& rhs) const { return (rhs.set_ != set_); }

	 private:
		CodeSet const* set_;
		std::vector<std::vector<Code>>::const_iterator outer_iter_;
		std::vector<std::vector<Code>>::const_iterator outer_iter_end_;
		std::vector<Code>::const_iterator inner_iter_;
		std::vector<Code>::const_iterator inner_iter_end_;
		// typename decltype(CodeSet::data_)::const_iterator outer_iter_;
		// typename decltype(CodeSet::data_)::const_iterator outer_iter_end_;
		// typename decltype(CodeSet::data_)::value_type::const_iterator inner_iter_;
		// typename decltype(CodeSet::data_)::value_type::const_iterator inner_iter_end_;
	};

	std::pair<int, bool> insert(Code const& value)
	{
		size_t hash = getBucket(value);
		if (std::any_of(std::cbegin(data_[hash]), std::cend(data_[hash]),
		                [&value](auto const& elem) { return value == elem; })) {
			return {0, false};  // TODO: Fix
		}

		++size_;

		if (load_factor() > max_load_factor() && power_ < MAX_POWER) {
			rehash(num_buckets_ * 2);
			hash = getBucket(value);
		}

		data_[hash].push_back(value);

		return {0, true};  // TOOD: Fix
	}

	void clear()
	{
		std::for_each(std::begin(data_), std::end(data_),
		              [](auto& bucket) { bucket.clear(); });
		size_ = 0;
	}

	bool empty() const noexcept { return 0 == size_; }

	size_t size() const noexcept { return size_; }

	size_t bucket_count() const noexcept { return num_buckets_; }

	unsigned int bucket_count_power() const noexcept { return power_; }

	double load_factor() const { return size_ / ((double)num_buckets_); }

	double max_load_factor() const { return max_load_factor_; }

	void max_load_factor(double max_load_factor)
	{
		max_load_factor_ = max_load_factor;

		if (load_factor() > max_load_factor_ && power_ < MAX_POWER) {
			rehash(num_buckets_ * 2);
		}
	}

	void rehash(std::size_t count)
	{
		std::size_t min_count = std::max((double)count, size() / max_load_factor());
		unsigned int power =
		    std::max(power_, std::min((unsigned int)std::log2(min_count) + 1, MAX_POWER));

		if (power_ == power) {
			return;
		}

		power_ = power;
		num_buckets_ = std::size_t(1) << power_;

		decltype(data_) new_data;
		new_data.resize(num_buckets_);

		for (Code const& value : *this) {
			new_data[getBucket(value)].push_back(value);
		}

		data_.swap(new_data);
	}

	void reserve(std::size_t count)
	{
		power_ = std::max(power_, std::min((unsigned int)std::log2(count) + 1, MAX_POWER));
		data_.reserve(std::size_t(1) << power_);
	}

	CodeSetIterator begin() const { return CodeSetIterator(this); }

	CodeSetIterator end() const { return CodeSetIterator(); }

	void swap(CodeSet& other)
	{
		data_.swap(other.data_);
		std::swap(power_, other.power_);
		std::swap(num_buckets_, other.num_buckets_);
		std::swap(size_, other.size_);
		std::swap(max_load_factor_, other.max_load_factor_);
	}

	using const_iterator = CodeSetIterator;

 private:
	std::size_t getBucket(Code const& key) const
	{
		unsigned int offset = 3 * key.depth();
		unsigned int modder = (num_buckets_ - 1) << offset;
		return (Code::Hash()(key) & modder) >> offset;
	}

 private:
	std::vector<std::vector<Code>> data_;
	unsigned int power_;
	std::size_t num_buckets_;
	std::size_t size_ = 0;
	double max_load_factor_ = 1.0;

	inline static const unsigned int MAX_POWER = 28;

	friend struct CodeSetIterator;
};

template <class T>
class CodeMap
{
 public:
	CodeMap(unsigned int power = 18) : power_(power)
	{
		num_buckets_ = std::size_t(1) << power_;  // std::pow(2, power_);
		data_.resize(num_buckets_);
	}

	struct CodeMapIterator {
		// Tags
		// FIXME: Fix these
		using iterator_category = std::forward_iterator_tag;
		using difference_type = std::ptrdiff_t;
		using value_type = std::pair<Code, T>;
		using pointer = value_type*;    // or also value_type*
		using reference = value_type&;  // or also value_type&
		using const_pointer = value_type const*;
		using const_reference = value_type const&;

		CodeMapIterator(const CodeMap* map = nullptr) : map_(map)
		{
			if (nullptr == map_) {
				return;
			}

			if (map_->data_.empty()) {
				map_ = nullptr;
			} else {
				outer_iter_end_ = map_->data_.cend();
				outer_iter_ = std::find_if(map_->data_.cbegin(), outer_iter_end_,
				                           [](auto const& e) { return !e.empty(); });
				if (outer_iter_ == outer_iter_end_) {
					map_ = nullptr;
				} else {
					inner_iter_ = outer_iter_->cbegin();
					inner_iter_end_ = outer_iter_->cend();
				}
			}
		}

		std::pair<Code, T> const& operator*() const
		{
			return *inner_iter_;  // map_->data_[outer_index_][inner_index_];
		}

		std::pair<Code, T> operator*()
		{
			return *inner_iter_;  // map_->data_[outer_index_][inner_index_];
		}

		// Postfix increment
		CodeMapIterator operator++(int)
		{
			CodeMapIterator result = *this;
			++(*this);
			return result;
		}

		// Prefix increment
		CodeMapIterator& operator++()
		{
			++inner_iter_;
			if (inner_iter_ == inner_iter_end_) {
				++outer_iter_;
				outer_iter_ = std::find_if(outer_iter_, outer_iter_end_,
				                           [](auto const& e) { return !e.empty(); });
				if (outer_iter_ == outer_iter_end_) {
					map_ = nullptr;
				} else {
					inner_iter_ = outer_iter_->cbegin();
					inner_iter_end_ = outer_iter_->cend();
				}
			}
			return *this;
		}

		bool operator==(CodeMapIterator const& rhs) const { return (rhs.map_ == map_); }

		bool operator!=(CodeMapIterator const& rhs) const { return (rhs.map_ != map_); }

	 private:
		const CodeMap* map_;
		typename std::vector<std::vector<std::pair<Code, T>>>::const_iterator outer_iter_;
		typename std::vector<std::vector<std::pair<Code, T>>>::const_iterator outer_iter_end_;
		typename std::vector<std::pair<Code, T>>::const_iterator inner_iter_;
		typename std::vector<std::pair<Code, T>>::const_iterator inner_iter_end_;
		// typename decltype(CodeMap<T>::data_)::const_iterator outer_iter_;
		// typename decltype(CodeMap<T>::data_)::const_iterator outer_iter_end_;
		// typename decltype(CodeMap<T>::data_)::value_type::const_iterator inner_iter_;
		// typename decltype(CodeMap<T>::data_)::value_type::const_iterator inner_iter_end_;
	};

	struct CodeMapBucketIterator {
		// Tags
		// FIXME: Fix these
		using iterator_category = std::forward_iterator_tag;
		using difference_type = std::ptrdiff_t;
		using value_type = std::vector<std::pair<Code, T>>;
		using pointer = value_type*;    // or also value_type*
		using reference = value_type&;  // or also value_type&
		using const_pointer = value_type const*;
		using const_reference = value_type const&;

		CodeMapBucketIterator(const CodeMap* map = nullptr) : map_(map)
		{
			if (nullptr == map_) {
				return;
			}

			if (map_->data_.empty()) {
				map_ = nullptr;
			} else {
				outer_iter_end_ = map_->data_.cend();
				outer_iter_ = std::find_if(map_->data_.cbegin(), outer_iter_end_,
				                           [](auto const& e) { return !e.empty(); });
				if (outer_iter_ == outer_iter_end_) {
					map_ = nullptr;
				}
			}
		}

		const_reference operator*() const
		{
			return *outer_iter_;  // map_->data_[outer_index_][inner_index_];
		}

		value_type operator*()
		{
			return *outer_iter_;  // map_->data_[outer_index_][inner_index_];
		}

		// Postfix increment
		CodeMapBucketIterator operator++(int)
		{
			CodeMapBucketIterator result = *this;
			++(*this);
			return result;
		}

		// Prefix increment
		CodeMapBucketIterator& operator++()
		{
			++outer_iter_;
			outer_iter_ = std::find_if(outer_iter_, outer_iter_end_,
			                           [](auto const& e) { return !e.empty(); });
			if (outer_iter_ == outer_iter_end_) {
				map_ = nullptr;
			}
			return *this;
		}

		bool operator==(CodeMapBucketIterator const& rhs) const { return (rhs.map_ == map_); }

		bool operator!=(CodeMapBucketIterator const& rhs) const { return (rhs.map_ != map_); }

	 private:
		const CodeMap* map_;
		typename std::vector<std::vector<std::pair<Code, T>>>::const_iterator outer_iter_;
		typename std::vector<std::vector<std::pair<Code, T>>>::const_iterator outer_iter_end_;
	};

	T& operator[](Code const& key)
	{
		std::size_t hash = getBucket(key);
		auto it = std::find_if(std::begin(data_[hash]), std::end(data_[hash]),
		                       [&key](auto const& elem) { return key == elem.first; });
		if (it != std::end(data_[hash])) {
			return it->second;
		}

		++size_;

		if (load_factor() > max_load_factor() && power_ < MAX_POWER) {
			rehash(num_buckets_ * 2);
			hash = getBucket(key);
		}

		return std::get<1>(data_[hash].emplace_back(key, T()));  // FIXME: How to
		                                                         // call default?
	}

	std::pair<int, bool> try_emplace(Code const& key,
	                                 T const& value)  // TODO: Fix
	{
		std::size_t hash = getBucket(key);
		if (std::any_of(std::cbegin(data_[hash]), std::cend(data_[hash]),
		                [&key](auto const& elem) { return key == elem.first; })) {
			return {0, false};  // TODO: Fix
		}

		++size_;

		if (load_factor() > max_load_factor() && power_ < MAX_POWER) {
			rehash(num_buckets_ * 2);
			hash = getBucket(key);
		}

		data_[hash].emplace_back(key, value);

		return {0, true};  // TODO: Fix
	}

	void clear()
	{
		std::for_each(std::begin(data_), std::end(data_),
		              [](auto& bucket) { bucket.clear(); });
		size_ = 0;
	}

	bool empty() const { return 0 == size_; }

	constexpr std::size_t size() const { return size_; }

	std::size_t bucket_count() const { return num_buckets_; }

	unsigned int bucket_count_power() const { return power_; }

	double load_factor() const { return size_ / static_cast<double>(num_buckets_); }

	double max_load_factor() const { return max_load_factor_; }

	void max_load_factor(double max_load_factor)
	{
		max_load_factor_ = max_load_factor;

		if (load_factor() > max_load_factor_ && power_ < MAX_POWER) {
			rehash(num_buckets_ * 2);
		}
	}

	void rehash(std::size_t count)
	{
		std::size_t min_count =
		    std::max(static_cast<double>(count), size() / max_load_factor());
		unsigned int power = std::max(
		    power_, std::min(static_cast<unsigned int>(std::log2(min_count) + 1), MAX_POWER));

		if (power_ == power) {
			return;
		}

		power_ = power;
		num_buckets_ = std::size_t(1) << power_;

		decltype(data_) new_data;
		new_data.resize(num_buckets_);

		for (const auto& [key, value] : *this) {
			new_data[getBucket(key)].emplace_back(key, value);
		}

		data_.swap(new_data);
		// fprintf(stderr, "\n\nRehash, new power: %u\n\n", power_);
	}

	void reserve(std::size_t count)
	{
		power_ = std::max(power_, std::min((unsigned int)std::log2(count) + 1, MAX_POWER));
		data_.reserve(std::size_t(1) << power_);
	}

	CodeMapIterator begin() const { return CodeMapIterator(this); }

	CodeMapIterator end() const { return CodeMapIterator(); }

	CodeMapBucketIterator beginBuckets() const { return CodeMapBucketIterator(this); }

	CodeMapBucketIterator endBuckets() const { return CodeMapBucketIterator(); }

	void swap(CodeMap<T>& other)
	{
		data_.swap(other.data_);
		std::swap(power_, other.power_);
		std::swap(num_buckets_, other.num_buckets_);
		std::swap(size_, other.size_);
		std::swap(max_load_factor_, other.max_load_factor_);
	}

 private:
	std::size_t getBucket(Code const& key) const
	{
		unsigned int offset = 3 * key.depth();
		unsigned int modder = (bucket_count() - 1) << offset;
		return (Code::Hash()(key) & modder) >> offset;
	}

 private:
	std::vector<std::vector<std::pair<Code, T>>> data_;
	unsigned int power_;
	std::size_t num_buckets_;
	std::size_t size_ = 0;
	double max_load_factor_ = 1.0;

	static constexpr unsigned int MAX_POWER = 28;

	friend struct CodeMapIterator;
};
}  // namespace ufo::map

#endif  // UFO_MAP_CODE_H