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

#ifndef UFO_MAP_INTEGRATOR_DETAIL_INVERSE_INFO_HPP
#define UFO_MAP_INTEGRATOR_DETAIL_INVERSE_INFO_HPP

// UFO
#include <ufo/math/vec.hpp>

// STL
#include <cstddef>
#include <cstdint>
#include <limits>

namespace ufo::detail
{
template <std::size_t Dim>
class InverseInfo
{
 public:
	constexpr InverseInfo() = default;

	constexpr InverseInfo(Vec<Dim, float> const& point, std::uint32_t first_child)
	    : point_(point), first_child_(first_child)
	{
	}

	constexpr InverseInfo(Vec<Dim, float> const& point, float min_distance,
	                      float max_distance, std::uint32_t first_child,
	                      std::uint32_t first_lut, std::uint32_t count)
	    : point_(point)
	    , min_distance_(min_distance)
	    , max_distance_(max_distance)
	    , first_child_(first_child)
	    , first_lut_(first_lut)
	    , count_(count)
	{
	}

	[[nodiscard]] constexpr Vec<Dim, float> const& point() const noexcept { return point_; }

	[[nodiscard]] constexpr float& minDistance() noexcept { return min_distance_; }

	[[nodiscard]] constexpr float minDistance() const noexcept { return min_distance_; }

	[[nodiscard]] constexpr float& maxDistance() noexcept { return max_distance_; }

	[[nodiscard]] constexpr float maxDistance() const noexcept { return max_distance_; }

	[[nodiscard]] constexpr std::uint32_t firstChild() const noexcept
	{
		return first_child_;
	}

	[[nodiscard]] constexpr std::uint32_t& firstLut() noexcept { return first_lut_; }

	[[nodiscard]] constexpr std::uint32_t firstLut() const noexcept { return first_lut_; }

	[[nodiscard]] constexpr std::uint32_t& count() noexcept { return count_; }

	[[nodiscard]] constexpr std::uint32_t count() const noexcept { return count_; }

 private:
	Vec<Dim, float> point_{};
	float           min_distance_{};
	float           max_distance_{};
	std::uint32_t   first_child_{};
	std::uint32_t   first_lut_{};
	std::uint32_t   count_{};
};

template <std::size_t Dim>
class InverseInfoLeaf
{
 public:
	constexpr InverseInfoLeaf() = default;

	constexpr InverseInfoLeaf(Vec<Dim, float> const& point, float distance,
	                          std::uint32_t first_lut)
	    : point_(point), distance_(distance), first_lut_(first_lut)
	{
	}

	[[nodiscard]] constexpr Vec<Dim, float> const& point() const noexcept { return point_; }

	[[nodiscard]] constexpr float distance() const noexcept { return distance_; }

	[[nodiscard]] constexpr std::uint32_t& firstLut() noexcept { return first_lut_; }

	[[nodiscard]] constexpr std::uint32_t firstLut() const noexcept { return first_lut_; }

 private:
	Vec<Dim, float> point_{};
	float           distance_{};
	std::uint32_t   first_lut_{};
};
}  // namespace ufo::detail

#endif  // UFO_MAP_INTEGRATOR_DETAIL_INVERSE_INFO_HPP