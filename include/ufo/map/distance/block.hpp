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
#ifndef UFO_MAP_DISTANCE_BLOCK_HPP
#define UFO_MAP_DISTANCE_BLOCK_HPP

// UFO
#include <ufo/geometry/aabb.hpp>
#include <ufo/map/distance/info.hpp>
#include <ufo/math/vec.hpp>
#include <ufo/utility/create_array.hpp>

// STL
#include <array>
#include <cassert>
#include <cstddef>
#include <limits>

namespace ufo
{
template <std::size_t Dim, std::size_t BF, bool WithBounds>
struct DistanceBlock {
	using Point  = typename DistanceInfo<Dim>::Point;
	using Bounds = AABB<Dim, float>;

	constexpr DistanceBlock() = default;

	constexpr DistanceBlock(DistanceBlock const& parent, std::size_t offset)
	    : info_(createArray<BF>(parent.info_[offset]))
	    , bounds_(createArray<BF>(parent.bounds_[offset]))
	{
	}

	constexpr void fill(DistanceBlock const& parent, std::size_t offset)
	{
		info_   = createArray<BF>(parent.info_[offset]);
		bounds_ = createArray<BF>(parent.bounds_[offset]);
	}

	[[nodiscard]] constexpr DistanceInfo<Dim>& operator[](std::size_t pos)
	{
		assert(BF > pos);
		return info_[pos];
	}

	[[nodiscard]] constexpr DistanceInfo<Dim> const& operator[](std::size_t pos) const
	{
		assert(BF > pos);
		return info_[pos];
	}

	[[nodiscard]] constexpr auto& info() { return info_; }

	[[nodiscard]] constexpr auto const& info() const { return info_; }

	[[nodiscard]] constexpr auto& bounds() { return bounds_; }

	[[nodiscard]] constexpr auto const& bounds() const { return bounds_; }

	[[nodiscard]] constexpr Bounds& bounds(std::size_t pos)
	{
		assert(BF > pos);
		return bounds_[pos];
	}

	[[nodiscard]] constexpr Bounds const& bounds(std::size_t pos) const
	{
		assert(BF > pos);
		return bounds_[pos];
	}

	[[nodiscard]] static constexpr DistanceInfo<Dim> resetInfo()
	{
		return DistanceInfo<Dim>{Point(std::numeric_limits<float>::max()), 0.0f};
	}

	[[nodiscard]] static constexpr Bounds resetBounds()
	{
		return Bounds(Point(std::numeric_limits<float>::max()),
		              Point(std::numeric_limits<float>::lowest()));
	}
	void reset() { fill(resetInfo(), resetBounds()); }

	void reset(std::size_t pos)
	{
		assert(BF > pos);
		info_[pos]   = resetInfo();
		bounds_[pos] = resetBounds();
	}

 private:
	// We have them in different arrays to simplify (de)serialization
	std::array<DistanceInfo<Dim>, BF> info_ = createArray<BF>(resetInfo());
	// TODO: Store single per block
	std::array<Bounds, BF> bounds_ = createArray<BF>(resetBounds());
};
}  // namespace ufo

#endif  // UFO_MAP_DISTANCE_BLOCK_HPP