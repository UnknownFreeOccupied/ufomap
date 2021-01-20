/**
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * License: BSD 3
 *
 */

/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, D. Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
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

// STD
#include <immintrin.h>

#include <algorithm>
#include <execution>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace ufo::map
{
/**
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
	Code() : code_(0), depth_(0) {}

	Code(CodeType code, DepthType depth = 0) : code_(code), depth_(depth) {}

	Code(Key const& key) : code_(toCode(key)), depth_(key.getDepth()) {}

	Code(Code const& other) : code_(other.code_), depth_(other.depth_) {}

	Code& operator=(Code const& rhs)
	{
		code_ = rhs.code_;
		depth_ = rhs.depth_;
		return *this;
	}

	bool operator==(Code const& rhs) const
	{
		return code_ == rhs.code_ && depth_ == rhs.depth_;
	}

	bool operator!=(Code const& rhs) const
	{
		return code_ != rhs.code_ || depth_ != rhs.depth_;
	}

	bool operator<(Code const& rhs) const
	{
		// TODO: Check
		return get3Bits(code_) < get3Bits(rhs.code_) &&
		       get3Bits(code_ >> 1) < get3Bits(rhs.code_ >> 1) &&
		       get3Bits(code_ >> 2) < get3Bits(rhs.code_ >> 2);
	}

	bool operator<=(Code const& rhs) const
	{
		return get3Bits(code_) <= get3Bits(rhs.code_) &&
		       get3Bits(code_ >> 1) <= get3Bits(rhs.code_ >> 1) &&
		       get3Bits(code_ >> 2) <= get3Bits(rhs.code_ >> 2);
	}

	bool operator>(Code const& rhs) const
	{
		return get3Bits(code_) > get3Bits(rhs.code_) &&
		       get3Bits(code_ >> 1) > get3Bits(rhs.code_ >> 1) &&
		       get3Bits(code_ >> 2) > get3Bits(rhs.code_ >> 2);
	}

	bool operator>=(Code const& rhs) const
	{
		return get3Bits(code_) >= get3Bits(rhs.code_) &&
		       get3Bits(code_ >> 1) >= get3Bits(rhs.code_ >> 1) &&
		       get3Bits(code_ >> 2) >= get3Bits(rhs.code_ >> 2);
	}

	/**
	 * @brief Return the code at a specified depth
	 *
	 * @param depth The depth of the code
	 * @return Code The code at the specified depth
	 */
	Code toDepth(DepthType depth) const
	{
		CodeType temp = 3 * depth;
		return Code((code_ >> temp) << temp, depth);
	}

	void moveX(int offset)
	{
#if defined(__BMI2__)  // TODO: Is correct?
		KeyType x = static_cast<KeyType>(_pext_u64(code_, 0x9249249249249249)) + offset;
		code_ &= 0x6DB6DB6DB6DB6DB6;
		code_ |= _pdep_u64(static_cast<CodeType>(x), 0x9249249249249249);
#else
		KeyType x = toKey(0) + offset;
		code_ &= 0x6DB6DB6DB6DB6DB6;
		code_ |= splitBy3(x);
#endif
	}

	void moveY(int offset)
	{
#if defined(__BMI2__)  // TODO: Is correct?
		KeyType y = static_cast<KeyType>(_pext_u64(code_, 0x2492492492492492)) + offset;
		code_ &= 0x5B6DB6DB6DB6DB6D;
		code_ |= _pdep_u64(static_cast<CodeType>(y), 0x2492492492492492);
#else
		KeyType y = toKey(1) + offset;
		code_ &= 0x5B6DB6DB6DB6DB6D;
		code_ |= (splitBy3(y) << 1);
#endif
	}

	void moveZ(int offset)
	{
#if defined(__BMI2__)  // TODO: Is correct?
		KeyType z = static_cast<KeyType>(_pext_u64(code_, 0x4924924924924924)) + offset;
		code_ &= 0xB6DB6DB6DB6DB6DB;
		code_ |= _pdep_u64(static_cast<CodeType>(z), 0x4924924924924924);
#else
		KeyType z = toKey(2) + offset;
		code_ &= 0xB6DB6DB6DB6DB6DB;
		code_ |= (splitBy3(z) << 2);
#endif
	}

	/**
	 * @brief Converts a key to a code
	 *
	 * @param key The key to convert
	 * @return uint64_t The code corresponding to the key
	 */
	static CodeType toCode(Key const& key)
	{
#if defined(__BMI2__)  // TODO: Is correct?
		return _pdep_u64(static_cast<CodeType>(key[0]), 0x9249249249249249) |
		       _pdep_u64(static_cast<CodeType>(key[1]), 0x2492492492492492) |
		       _pdep_u64(static_cast<CodeType>(key[2]), 0x4924924924924924);
#else
		return splitBy3(key[0]) | (splitBy3(key[1]) << 1) | (splitBy3(key[2]) << 2);
#endif
	}

	/**
	 * @brief Get the key component from a code
	 *
	 * @param code The code to generate the key component from
	 * @param index The index of the key component
	 * @return KeyType The key component value
	 */
	static KeyType toKey(Code const& code, std::size_t index)
	{
		return get3Bits(code.code_ >> index);
	}

	/**
	 * @brief Get the key component from this code
	 *
	 * @param index The index of the key component
	 * @return KeyType The key component value
	 */
	KeyType toKey(std::size_t index) const { return toKey(*this, index); }

	/**
	 * @brief Get the corresponding key to code
	 *
	 * @param code The code the corresponding key should be returned
	 * @return Key The corresponding key to code
	 */
	static Key toKey(Code const& code)
	{
#if defined(__BMI2__)  // TODO: Is correct?
		return Key(static_cast<KeyType>(_pext_u64(code.code_, 0x9249249249249249)),
		           static_cast<KeyType>(_pext_u64(code.code_, 0x2492492492492492)),
		           static_cast<KeyType>(_pext_u64(code.code_, 0x4924924924924924)),
		           code.getDepth());
#else
		return Key(toKey(code, 0), toKey(code, 1), toKey(code, 2), code.getDepth());
#endif
	}

	/**
	 * @brief Get the corresponding key to this code
	 *
	 * @return Key The corresponding key to this code
	 */
	Key toKey() const { return toKey(*this); }

	/**
	 * @brief Get the child index at a specific depth for this code
	 *
	 * @param depth The depth the child index is requested for
	 * @return std::size_t The child index at the specified depth
	 */
	std::size_t getChildIdx(DepthType depth) const
	{
		return (code_ >> static_cast<CodeType>(3 * depth)) & ((CodeType)0x7);
	}

	/**
	 * @brief Get the code of a specific child to this code
	 *
	 * @param index The index of the child
	 * @return Code The child code
	 */
	Code getChild(std::size_t index) const
	{
		if (0 == depth_) {
			// TODO: Throw error?
			return *this;
		}

		DepthType child_depth = depth_ - 1;
		return Code(
		    code_ + (static_cast<CodeType>(index) << static_cast<CodeType>(3 * child_depth)),
		    child_depth);
	}

	/**
	 * @brief Get the eight child codes that comes from this code
	 *
	 * @return std::vector<Code> The eight child codes
	 */
	std::vector<Code> getChildren() const
	{
		std::vector<Code> children;
		if (0 == depth_) {
			return children;
		}

		DepthType child_depth = depth_ - 1;
		CodeType offset = 3 * child_depth;
		for (CodeType i = 0; i < 8; ++i) {
			children.emplace_back(code_ + (i << offset), child_depth);
		}
		return children;
	}

	/**
	 * @brief Get all children that this code can have from this code's depth to
	 * depth 0
	 *
	 * @return std::vector<Code> Collection of all possible child codes of this
	 * code
	 */
	std::vector<Code> getAllChildren() const
	{
		std::vector<Code> children;
		CodeType max = 8 << (3 * depth_);
		for (CodeType i = 0; i < max; ++i) {
			children.emplace_back(code_ + i, 0);
		}
		return children;
	}

	/**
	 * @brief Get the code
	 *
	 * @return CodeType The code
	 */
	CodeType getCode() const { return code_; }

	/**
	 * @brief Get the depth that this code is specified at
	 *
	 * @return DepthType The depth this code is specified at
	 */
	DepthType getDepth() const { return depth_; }

	/**
	 * @brief
	 *
	 */
	struct Hash {
		std::size_t operator()(Code const& code) const
		{
			return static_cast<std::size_t>(code.code_);
		}

		static size_t hash(Code const& code) { return code.code_; }

		static bool equal(Code const& a, Code const& b) { return a == b; }
	};

 private:
	static CodeType splitBy3(KeyType a)
	{
#if defined(__BMI2__)  // TODO: Is correct?
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

	static KeyType get3Bits(CodeType code)
	{
#if defined(__BMI2__)  // TODO: Is correct?
		return static_cast<KeyType>(_pext_u64(code, 0x9249249249249249));
#else
		CodeType a = code & 0x1249249249249249;
		a = (a ^ (a >> 2)) & 0x10c30c30c30c30c3;
		a = (a ^ (a >> 4)) & 0x100f00f00f00f00f;
		a = (a ^ (a >> 8)) & 0x1f0000ff0000ff;
		a = (a ^ (a >> 16)) & 0x1f00000000ffff;
		a = (a ^ a >> 32) & 0x1fffff;
		return static_cast<KeyType>(a);
#endif
	}

 private:
	// The Morton code
	CodeType code_;
	// The depth of the Morton code
	DepthType depth_;
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
				outer_iter_ = set_->data_.begin();
				outer_iter_end_ = set_->data_.end();
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
		if (std::any_of(std::execution::seq, data_[hash].begin(), data_[hash].end(),
		                [&value](auto const& elem) { return value == elem; })) {
			return std::make_pair(0, false);  // TODO: Fix
		}

		++size_;

		if (load_factor() > max_load_factor() && power_ < MAX_POWER) {
			rehash(num_buckets_ * 2);
			hash = getBucket(value);
		}

		data_[hash].push_back(value);

		return std::make_pair(0, true);  // TOOD: Fix
	}

	void clear()
	{
		std::for_each(std::execution::seq, data_.begin(), data_.end(),
		              [](auto& bucket) { bucket.clear(); });
		size_ = 0;
	}

	bool empty() const noexcept { return 0 == size_; }

	size_t size() const noexcept { return size_; }

	size_t bucket_count() const noexcept { return num_buckets_; }

	unsigned int bucket_count_power() const noexcept { return power_; }

	float load_factor() const { return size_ / ((float)num_buckets_); }

	float max_load_factor() const { return max_load_factor_; }

	void max_load_factor(float max_load_factor)
	{
		max_load_factor_ = max_load_factor;

		if (load_factor() > max_load_factor_ && power_ < MAX_POWER) {
			rehash(num_buckets_ * 2);
		}
	}

	void rehash(std::size_t count)
	{
		std::size_t min_count = std::max((float)count, size() / max_load_factor());
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
		unsigned int offset = 3 * key.getDepth();
		unsigned int modder = (num_buckets_ - 1) << offset;
		return (Code::Hash()(key) & modder) >> offset;
	}

 private:
	std::vector<std::vector<Code>> data_;
	unsigned int power_;
	std::size_t num_buckets_;
	std::size_t size_ = 0;
	float max_load_factor_ = 1.0;

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
		CodeMapIterator(const CodeMap* map = nullptr) : map_(map)
		{
			if (nullptr == map_) {
				return;
			}

			if (map_->data_.empty()) {
				map_ = nullptr;
			} else {
				outer_iter_ = map_->data_.begin();
				outer_iter_end_ = map_->data_.end();
				while (outer_iter_ != outer_iter_end_ && outer_iter_->empty()) {
					++outer_iter_;
				}
				if (outer_iter_ == outer_iter_end_) {
					map_ = nullptr;
				} else {
					inner_iter_ = outer_iter_->begin();
					inner_iter_end_ = outer_iter_->end();
				}
			}
		}

		const std::pair<Code, T>& operator*() const
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
				while (outer_iter_ != outer_iter_end_ && outer_iter_->empty()) {
					++outer_iter_;
				}
				if (outer_iter_ == outer_iter_end_) {
					map_ = nullptr;
				} else {
					inner_iter_ = outer_iter_->begin();
					inner_iter_end_ = outer_iter_->end();
				}
			}
			return *this;
		}

		bool operator==(const CodeMapIterator& rhs) const { return (rhs.map_ == map_); }

		bool operator!=(const CodeMapIterator& rhs) const { return (rhs.map_ != map_); }

	 private:
		const CodeMap* map_;
		typename std::vector<std::list<std::pair<Code, T>>>::const_iterator outer_iter_;
		typename std::vector<std::list<std::pair<Code, T>>>::const_iterator outer_iter_end_;
		typename std::list<std::pair<Code, T>>::const_iterator inner_iter_;
		typename std::list<std::pair<Code, T>>::const_iterator inner_iter_end_;
		// typename decltype(CodeMap<T>::data_)::const_iterator outer_iter_;
		// typename decltype(CodeMap<T>::data_)::const_iterator outer_iter_end_;
		// typename decltype(CodeMap<T>::data_)::value_type::const_iterator inner_iter_;
		// typename decltype(CodeMap<T>::data_)::value_type::const_iterator inner_iter_end_;
	};

	T& operator[](Code const& key)
	{
		std::size_t hash = getBucket(key);
		auto it = std::find_if(std::execution::seq, data_[hash].begin(), data_[hash].end(),
		                       [&key](const auto& elem) { return key == elem.first; });
		if (it != data_[hash].end()) {
			return it->second;
		}

		++size_;

		if (load_factor() > max_load_factor() && power_ < MAX_POWER) {
			rehash(num_buckets_ * 2);
			hash = getBucket(key);
		}

		return std::get<1>(data_[hash].emplace_front(key, T()));  // TODO: How to
		                                                          // call default?
	}

	std::pair<int, bool> try_emplace(Code const& key,
	                                 const T& value)  // TODO: Fix
	{
		std::size_t hash = getBucket(key);
		if (std::any_of(std::execution::seq, data_[hash].begin(), data_[hash].end(),
		                [&key](const auto& elem) { return key == elem.first; })) {
			return std::make_pair(0, false);  // TODO: Fix
		}

		++size_;

		if (load_factor() > max_load_factor() && power_ < MAX_POWER) {
			rehash(num_buckets_ * 2);
			hash = getBucket(key);
		}

		data_[hash].emplace_front(key, value);

		return std::make_pair(0, true);  // TODO: Fix
	}

	void clear()
	{
		std::for_each(std::execution::seq, data_.begin(), data_.end(),
		              [](auto& bucket) { bucket.clear(); });
		size_ = 0;
	}

	bool empty() const { return 0 == size_; }

	std::size_t size() { return size_; }

	std::size_t bucket_count() const { return num_buckets_; }

	unsigned int bucket_count_power() const { return power_; }

	float load_factor() const { return size_ / ((float)num_buckets_); }

	float max_load_factor() const { return max_load_factor_; }

	void max_load_factor(float max_load_factor)
	{
		max_load_factor_ = max_load_factor;

		if (load_factor() > max_load_factor_ && power_ < MAX_POWER) {
			rehash(num_buckets_ * 2);
		}
	}

	void rehash(std::size_t count)
	{
		std::size_t min_count = std::max((float)count, size() / max_load_factor());
		unsigned int power =
		    std::max(power_, std::min((unsigned int)std::log2(min_count) + 1, MAX_POWER));

		if (power_ == power) {
			return;
		}

		power_ = power;
		num_buckets_ = std::size_t(1) << power_;

		decltype(data_) new_data;
		new_data.resize(num_buckets_);

		for (const auto& [key, value] : *this) {
			new_data[getBucket(key)].emplace_front(key, value);
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
		unsigned int offset = 3 * key.getDepth();
		unsigned int modder = (num_buckets_ - 1) << offset;
		return (Code::Hash()(key) & modder) >> offset;
	}

 private:
	std::vector<std::list<std::pair<Code, T>>> data_;
	unsigned int power_;
	std::size_t num_buckets_;
	std::size_t size_ = 0;
	float max_load_factor_ = 1.0;

	inline static const unsigned int MAX_POWER = 28;

	friend struct CodeMapIterator;
};
}  // namespace ufo::map

#endif  // UFO_MAP_CODE_H