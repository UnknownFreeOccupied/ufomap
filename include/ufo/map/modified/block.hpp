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

#ifndef UFO_MAP_MODIFIED_BLOCK_HPP
#define UFO_MAP_MODIFIED_BLOCK_HPP

// UFO
#include <ufo/utility/bit_set.hpp>

// STL
#include <atomic>
#include <cassert>
#include <cstddef>
#include <cstdint>

namespace ufo
{
template <std::size_t BF>
struct ModifiedBlock {
	using value_type = std::conditional_t<8 >= BF, std::uint8_t, std::uint16_t>;

	static constexpr value_type const ALL_SET = ~(static_cast<value_type>(-1) << BF);

	std::atomic<value_type> modified;

	constexpr ModifiedBlock() = default;

	constexpr ModifiedBlock(bool value) : modified(-static_cast<value_type>(value)) {}

	constexpr ModifiedBlock& operator=(ModifiedBlock const& rhs)
	{
		modified = rhs.modified.load();
		return *this;
	}

	constexpr ModifiedBlock& operator=(bool value)
	{
		modified = -static_cast<value_type>(value);
		return *this;
	}

	constexpr ModifiedBlock& operator=(BitSet<BF> value)
	{
		modified = value.data();
		return *this;
	}

	[[nodiscard]] constexpr bool any() const { return value_type(0) != modified; }

	[[nodiscard]] constexpr bool all() const { return ALL_SET == modified; }

	[[nodiscard]] constexpr bool none() const { return value_type(0) == modified; }

	[[nodiscard]] constexpr bool operator[](std::size_t pos) const
	{
		assert(BF > pos);
		return (modified >> pos) & value_type(1);
	}

	constexpr void set(std::size_t pos)
	{
		assert(BF > pos);
		modified |= value_type(1) << pos;
	}

	constexpr void set(std::size_t pos, bool value)
	{
		assert(BF > pos);
		modified ^=
		    (-static_cast<value_type>(value) ^ modified.load()) & (value_type(1) << pos);
	}

	constexpr void reset(std::size_t pos)
	{
		assert(BF > pos);
		modified &= ~(value_type(1) << pos);
	}

	friend constexpr bool operator==(ModifiedBlock const& lhs, ModifiedBlock const& rhs)
	{
		return lhs.modified == rhs.modified;
	}

	friend constexpr bool operator!=(ModifiedBlock const& lhs, ModifiedBlock const& rhs)
	{
		return !(lhs == rhs);
	};
};
}  // namespace ufo

#endif  // UFO_MAP_MODIFIED_BLOCK_HPP