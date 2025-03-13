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

	BitSet<BF> primary_void_region;
	BitSet<BF> primary_contains_void_region;
	BitSet<BF> secondary_void_region;
	BitSet<BF> secondary_contains_void_region;

	constexpr VoidRegionBlock() = default;

	constexpr VoidRegionBlock(VoidRegionBlock const&) = default;

	constexpr VoidRegionBlock(VoidRegionBlock const& other, std::size_t pos)
	{
		primaryInit(other.primary(pos));
		secondaryInit(other.secondary(pos));
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Primary                                       |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] constexpr bool primary(std::size_t pos) const
	{
		assert(BF > pos);
		return primary_void_region[pos];
	}

	constexpr void primaryInit(bool value)
	{
		primary_void_region          = -static_cast<value_type>(value);
		primary_contains_void_region = -static_cast<value_type>(value);
	}

	constexpr void primarySet(std::size_t pos, bool value)
	{
		assert(BF > pos);
		primary_void_region.set(pos, value);
		primary_contains_void_region.set(pos, value);
	}

	constexpr void primarySet(std::size_t pos)
	{
		assert(BF > pos);
		primary_void_region.set(pos);
		primary_contains_void_region.set(pos);
	}

	constexpr void primaryReset(std::size_t pos)
	{
		assert(BF > pos);
		primary_void_region.reset(pos);
		primary_contains_void_region.reset(pos);
	}

	[[nodiscard]] constexpr bool primaryAny() const { return primary_void_region.any(); }

	[[nodiscard]] constexpr bool primaryAll() const { return primary_void_region.all(); }

	[[nodiscard]] constexpr bool primaryNone() const { return primary_void_region.none(); }

	[[nodiscard]] constexpr bool primaryContains(std::size_t pos) const
	{
		assert(BF > pos);
		return primary_contains_void_region[pos];
	}

	[[nodiscard]] constexpr typename BitSet<BF>::Reference primaryContains(std::size_t pos)
	{
		assert(BF > pos);
		return primary_contains_void_region[pos];
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Secondary                                      |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] constexpr bool secondary(std::size_t pos) const
	{
		assert(BF > pos);
		return secondary_void_region[pos];
	}

	constexpr void secondaryInit(bool value)
	{
		secondary_void_region          = -static_cast<value_type>(value);
		secondary_contains_void_region = -static_cast<value_type>(value);
	}

	constexpr void secondarySet(std::size_t pos, bool value)
	{
		assert(BF > pos);
		secondary_void_region.set(pos, value);
		secondary_contains_void_region.set(pos, value);
	}

	constexpr void secondarySet(std::size_t pos)
	{
		assert(BF > pos);
		secondary_void_region.set(pos);
		secondary_contains_void_region.set(pos);
	}

	constexpr void secondaryReset(std::size_t pos)
	{
		assert(BF > pos);
		secondary_void_region.reset(pos);
		secondary_contains_void_region.reset(pos);
	}

	[[nodiscard]] constexpr bool secondaryAny() const
	{
		return secondary_void_region.any();
	}

	[[nodiscard]] constexpr bool secondaryAll() const
	{
		return secondary_void_region.all();
	}

	[[nodiscard]] constexpr bool secondaryNone() const
	{
		return secondary_void_region.none();
	}

	[[nodiscard]] constexpr bool secondaryContains(std::size_t pos) const
	{
		assert(BF > pos);
		return secondary_contains_void_region[pos];
	}

	[[nodiscard]] constexpr typename BitSet<BF>::Reference secondaryContains(
	    std::size_t pos)
	{
		assert(BF > pos);
		return secondary_contains_void_region[pos];
	}

	friend constexpr bool operator==(VoidRegionBlock const& lhs, VoidRegionBlock const& rhs)
	{
		return lhs.primary_void_region == rhs.primary_void_region &&
		       lhs.primary_contains_void_region == rhs.primary_contains_void_region &&
		       lhs.secondary_void_region == rhs.secondary_void_region &&
		       lhs.secondary_contains_void_region == rhs.secondary_contains_void_region;
	}

	friend constexpr bool operator!=(VoidRegionBlock const& lhs, VoidRegionBlock const& rhs)
	{
		return !(lhs == rhs);
	};
};
}  // namespace ufo
#endif  // UFO_MAP_VOID_REGION_BLOCK_HPP