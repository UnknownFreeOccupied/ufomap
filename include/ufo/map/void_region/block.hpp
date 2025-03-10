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
#ifndef UFO_MAP_VOID_REGION_BLOCK_HPP
#define UFO_MAP_VOID_REGION_BLOCK_HPP

// UFO
#include <ufo/utility/bit_set.hpp>

// STL
#include <cassert>
#include <cstddef>

namespace ufo
{
template <std::size_t BF>
struct VoidRegionBlock {
	using value_type = typename BitSet<BF>::value_type;

	BitSet<BF> void_region;
	BitSet<BF> contains_void_region;

	constexpr VoidRegionBlock() = default;

	constexpr VoidRegionBlock(bool value)
	    : void_region(-static_cast<value_type>(value))
	    , contains_void_region(-static_cast<value_type>(value))
	{
	}

	constexpr VoidRegionBlock& operator=(bool value)
	{
		void_region          = -static_cast<value_type>(value);
		contains_void_region = -static_cast<value_type>(value);
		return *this;
	}

	[[nodiscard]] constexpr bool operator[](std::size_t pos) const
	{
		assert(BF > pos);
		return void_region[pos];
	}

	[[nodiscard]] constexpr typename BitSet<BF>::Reference operator[](std::size_t pos)
	{
		assert(BF > pos);
		return void_region[pos];
	}

	[[nodiscard]] constexpr bool contains(std::size_t pos) const
	{
		assert(BF > pos);
		return contains_void_region[pos];
	}

	[[nodiscard]] constexpr typename BitSet<BF>::Reference contains(std::size_t pos)
	{
		assert(BF > pos);
		return contains_void_region[pos];
	}

	friend constexpr bool operator==(VoidRegionBlock const& lhs, VoidRegionBlock const& rhs)
	{
		return lhs.void_region == rhs.void_region &&
		       lhs.contains_void_region == rhs.contains_void_region;
	}

	friend constexpr bool operator!=(VoidRegionBlock const& lhs, VoidRegionBlock const& rhs)
	{
		return !(lhs == rhs);
	};
};
}  // namespace ufo
#endif  // UFO_MAP_VOID_REGION_BLOCK_HPP