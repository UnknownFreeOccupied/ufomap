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

#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>

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
	// TODO: Save depth in the most/least (depending on how to order) significant 5 bits
	// TODO: Save depth by just checking first set bit position
 public:
	constexpr Code() = default;

	constexpr Code(code_t code, depth_t depth = 0)
	    : code_((code >> (3 * depth)) << (3 * depth)), depth_(depth)
	{
	}

	Code(Key const& key) : code_(toCode(key)), depth_(key.depth()) {}

	/*!
	 * @brief Get the corresponding key to code
	 *
	 * @return The corresponding key to code
	 */
	operator Key() const noexcept
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
	constexpr Code toDepth(depth_t depth) const { return Code(code_, depth); }

	/*!
	 * @brief Converts a key to a code
	 *
	 * @param key The key to convert
	 * @return uint64_t The code corresponding to the key
	 */
	static code_t toCode(Key const& key)
	{
#if defined(__BMI2__)
		return _pdep_u64(static_cast<code_t>(key[0]), 0x9249249249249249) |
		       _pdep_u64(static_cast<code_t>(key[1]), 0x2492492492492492) |
		       _pdep_u64(static_cast<code_t>(key[2]), 0x4924924924924924);
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
	static Key::key_t toKey(Code const& code, std::size_t index)
	{
		return get3Bits(code.code_ >> index);
	}

	/*!
	 * @brief Get the key component from this code
	 *
	 * @param index The index of the key component
	 * @return The key component value
	 */
	Key::key_t toKey(std::size_t index) const { return toKey(*this, index); }

	/*!
	 * @brief Get the index at a specific depth for this code.
	 *
	 * @param depth The depth the index is requested for.
	 * @return The index at the specified depth.
	 */
	constexpr std::size_t index(depth_t depth) const
	{
		return (code_ >> (code_t(3) * depth)) & code_t(0x7);
	}

	// TODO: Rename?
	constexpr std::size_t index() const { return index(depth()); }

	constexpr Code parent(depth_t parent_depth) const
	{
		assert(parent_depth >= depth_);  // TODO: Fix
		code_t temp = 3 * parent_depth;
		return Code((code_ >> temp) << temp, parent_depth);
	}

	// TODO: Rename?
	constexpr Code parent() const { return parent(depth() + 1); }

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

		depth_t child_depth = depth_ - 1;
		return Code(
		    code_ + (static_cast<code_t>(index) << static_cast<code_t>(3 * child_depth)),
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
		code_t sibling_code = (code_ >> static_cast<code_t>(3 * (depth_ + 1)))
		                      << static_cast<code_t>(3 * (depth_ + 1));
		return Code(
		    sibling_code + (static_cast<code_t>(index) << static_cast<code_t>(3 * depth_)),
		    depth_);
	}

	/*!
	 * @brief Get the code
	 *
	 * @return code_t The code
	 */
	constexpr code_t code() const noexcept { return code_; }

	/*!
	 * @brief Get the depth that this code is specified at
	 *
	 * @return depth_t The depth this code is specified at
	 */
	constexpr depth_t depth() const noexcept { return depth_; }

	/*!
	 * @brief
	 *
	 */
	struct Hash {
		static constexpr code_t hash(Code const& code) { return code.code(); }

		constexpr code_t operator()(Code const& code) const { return hash(code); }

		static constexpr bool equal(Code const& lhs, Code const& rhs) { return lhs == rhs; }
	};

 private:
	static code_t splitBy3(Key::key_t a)
	{
#if defined(__BMI2__)
		return _pdep_u64(static_cast<code_t>(a), 0x9249249249249249);
#else
		code_t code = static_cast<code_t>(a) & 0x1fffff;
		code = (code | code << 32) & 0x1f00000000ffff;
		code = (code | code << 16) & 0x1f0000ff0000ff;
		code = (code | code << 8) & 0x100f00f00f00f00f;
		code = (code | code << 4) & 0x10c30c30c30c30c3;
		code = (code | code << 2) & 0x1249249249249249;
		return code;
#endif
	}

	static Key::key_t get3Bits(code_t code)
	{
#if defined(__BMI2__)
		return static_cast<Key::key_t>(_pext_u64(code, 0x9249249249249249));
#else
		code_t a = code & 0x1249249249249249;
		a = (a ^ (a >> 2)) & 0x10c30c30c30c30c3;
		a = (a ^ (a >> 4)) & 0x100f00f00f00f00f;
		a = (a ^ (a >> 8)) & 0x1f0000ff0000ff;
		a = (a ^ (a >> 16)) & 0x1f00000000ffff;
		a = (a ^ a >> 32) & 0x1fffff;
		return static_cast<Key::key_t>(a);
#endif
	}

 private:
	// The Morton code
	code_t code_ = 0;
	// The depth of the Morton code
	depth_t depth_ = 0;
};

using CodeSet = std::set<Code>;
using CodeUnorderedSet = std::unordered_set<Code, Code::Hash>;
template <typename T>
using CodeMap = std::map<Code, T>;
template <typename T>
using CodeUnorderedMap = std::unordered_map<Code, T, Code::Hash>;
}  // namespace ufo::map

#endif  // UFO_MAP_CODE_H