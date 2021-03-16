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

#ifndef UFO_MAP_KEY_H
#define UFO_MAP_KEY_H

// UFO
#include <ufo/map/types.h>

// STD
#include <immintrin.h>

#include <array>
#include <cstddef>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace ufo::map
{
/**
 * @brief A key represent an octree index at a specified depth
 *
 */
class Key
{
 public:
	Key() {}

	Key(KeyType x, KeyType y, KeyType z, DepthType depth) : key_{x, y, z}, depth_(depth) {}

	Key(Key const& other) : key_{other.key_}, depth_(other.depth_) {}

	Key& operator=(Key const& rhs)
	{
		key_ = rhs.key_;
		depth_ = rhs.depth_;
		return *this;
	}

	/**
	 * @brief Get the depth that this key is specified at
	 *
	 * @return DepthType The depth this key is specified at
	 */
	DepthType getDepth() const { return depth_; }

	/**
	 * @brief Returns if this key is equal to another key at a certain depth
	 *
	 * @param other The other key to compare to
	 * @param depth The depth to do the comparison at
	 * @return true If the keys are equal at the specified depth
	 * @return false If the keys are not equal at the specified depth
	 */
	bool equals(Key const& other, DepthType depth = 0) const
	{
		return (key_[0] >> depth) == (other.key_[0] >> depth) &&
		       (key_[1] >> depth) == (other.key_[1] >> depth) &&
		       (key_[2] >> depth) == (other.key_[2] >> depth);
	}

	bool operator==(Key const& rhs) const
	{
		return (rhs.depth_ == depth_) && (rhs.key_ == key_);
	}

	bool operator!=(Key const& rhs) const
	{
		return (rhs.depth_ != depth_) || (rhs.key_ != key_);
	}

	KeyType const& operator[](std::size_t index) const { return key_[index]; }

	KeyType& operator[](std::size_t index) { return key_[index]; }

	/**
	 * @brief Returns the x component of the key
	 *
	 * @return const KeyType& The x component of the key
	 */
	KeyType const& x() const { return key_[0]; }

	/**
	 * @brief Returns the y component of the key
	 *
	 * @return const KeyType& The y component of the key
	 */
	KeyType const& y() const { return key_[1]; }

	/**
	 * @brief Returns the z component of the key
	 *
	 * @return const KeyType& The z component of the key
	 */
	KeyType const& z() const { return key_[2]; }

	/**
	 * @brief Returns the x component of the key
	 *
	 * @return KeyType& The x component of the key
	 */
	KeyType& x() { return key_[0]; }

	/**
	 * @brief Returns the y component of the key
	 *
	 * @return KeyType& The y component of the key
	 */
	KeyType& y() { return key_[1]; }

	/**
	 * @brief Returns the z component of the key
	 *
	 * @return KeyType& The z component of the key
	 */
	KeyType& z() { return key_[2]; }

	/**
	 * @brief
	 *
	 */
	struct Hash {
		std::size_t operator()(Key const& key) const
		{
#if defined(__BMI2__)  // TODO: Is correct?
			return _pdep_u64(static_cast<CodeType>(key[0]), 0x9249249249249249) |
			       _pdep_u64(static_cast<CodeType>(key[1]), 0x2492492492492492) |
			       _pdep_u64(static_cast<CodeType>(key[2]), 0x4924924924924924);
#else
			return splitBy3(key[0]) | (splitBy3(key[1]) << 1) | (splitBy3(key[2]) << 2);
#endif
		}
	};

 private:
	static uint64_t splitBy3(KeyType a)
	{
#if defined(__BMI2__)  // TODO: Is correct?
		return _pdep_u64(static_cast<uint64_t>(a), 0x9249249249249249);
#else
		uint64_t code = static_cast<uint64_t>(a) & 0x1fffff;
		code = (code | code << 32) & 0x1f00000000ffff;
		code = (code | code << 16) & 0x1f0000ff0000ff;
		code = (code | code << 8) & 0x100f00f00f00f00f;
		code = (code | code << 4) & 0x10c30c30c30c30c3;
		code = (code | code << 2) & 0x1249249249249249;
		return code;
#endif
	}

 private:
	// The key
	std::array<KeyType, 3> key_;
	// The depth of the key
	DepthType depth_;
};

using KeySet = std::unordered_set<Key, Key::Hash>;
template <typename T>
using KeyMap = std::unordered_map<Key, T, Key::Hash>;
using KeyRay = std::vector<Key>;
}  // namespace ufo::map

#endif  // UFO_MAP_KEY_H