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

#ifndef UFO_MAP_SURFEL_H
#define UFO_MAP_SURFEL_H

// UFO
#include <ufo/map/surfel/surfel_util.h>
#include <ufo/map/types.h>
#include <ufo/math/vector3.h>

// STL
#include <cstdint>
#include <type_traits>

namespace ufo::map
{
struct Surfel {
	constexpr Surfel() = default;

	constexpr Surfel(math::Vector3<surfel_scalar_t> sum,
	                 std::array<surfel_scalar_t, 6> sum_squares, count_t num_points)
	    : sum_(sum), sum_squares_(sum_squares), num_points_(num_points)
	{
	}

	constexpr Surfel(math::Vector3<surfel_scalar_t> point) : num_points_(1), sum_(point) {}

	template <class InputIt>
	constexpr Surfel(InputIt first, InputIt last)
	{
		addPoint(first, last);
	}

	constexpr Surfel(std::initializer_list<math::Vector3<surfel_scalar_t>> points)
	    : Surfel(std::begin(points), std::end(points))
	{
	}

	constexpr Surfel(Surfel const& other) = default;

	constexpr Surfel(Surfel&& other) = default;

	constexpr Surfel& operator=(Surfel const& rhs) = default;

	constexpr Surfel& operator=(Surfel&& rhs) = default;

	constexpr bool operator==(Surfel const& rhs) const
	{
		return num_points_ == rhs.num_points_ && sum_ == rhs.sum_ &&
		       sum_squares_ == rhs.sum_squares_;
	}

	constexpr bool operator!=(Surfel const& rhs) const { return !(*this == rhs); }

	//
	// Empty
	//

	[[nodiscard]] constexpr bool empty() const { return 0 == num_points_; }

	//
	// Add surfel
	//

	Surfel& operator+=(Surfel const& rhs)
	{
		addSurfel(rhs);
		return *this;
	}

	friend Surfel operator+(Surfel lhs, Surfel const& rhs);

	constexpr void addSurfel(Surfel const& other)
	{
		surfel::add(sum_, sum_squares_, num_points_, other.sum_, other.sum_squares_,
		            other.num_points_);
	}

	//
	// Remove surfel
	//

	Surfel& operator-=(Surfel const& rhs)
	{
		removeSurfel(rhs);
		return *this;
	}

	friend Surfel operator-(Surfel lhs, Surfel const& rhs);

	constexpr void removeSurfel(Surfel const& other)
	{
		surfel::remove(sum_, sum_squares_, num_points_, other.sum_, other.sum_squares_,
		               other.num_points_);
	}

	//
	// Add point
	//

	constexpr void addPoint(math::Vector3d point)
	{
		surfel::addPoint(sum_, sum_squares_, num_points_, point);
	}

	template <class InputIt>
	constexpr void addPoint(InputIt first, InputIt last)
	{
		surfel::addPoint(sum_, sum_squares_, num_points_, first, last);
	}

	constexpr void addPoint(std::initializer_list<math::Vector3<surfel_scalar_t>> points)
	{
		surfel::addPoint(sum_, sum_squares_, num_points_, points);
	}

	//
	// Remove point
	//

	constexpr void removePoint(math::Vector3d point)
	{
		surfel::removePoint(sum_, sum_squares_, num_points_, point);
	}

	template <class InputIt>
	constexpr void removePoint(InputIt first, InputIt last)
	{
		surfel::removePoint(sum_, sum_squares_, num_points_, first, last);
	}

	constexpr void removePoint(std::initializer_list<math::Vector3<surfel_scalar_t>> points)
	{
		surfel::removePoint(sum_, sum_squares_, num_points_, points);
	}

	//
	// Clear
	//

	constexpr void clear() { surfel::clear(sum_, sum_squares_, num_points_); }

	//
	// Get mean
	//

	constexpr math::Vector3<surfel_scalar_t> mean() const
	{
		return surfel::mean(sum_, num_points_);
	}

	//
	// Get normal
	//

	constexpr math::Vector3d normal() const
	{
		return surfel::normal(sum_squares_, num_points_);
	}

	//
	// Get planarity
	//

	constexpr double planarity() const
	{
		return surfel::planarity(sum_squares_, num_points_);
	}

	//
	// Get covariance
	//

	constexpr std::array<std::array<double, 3>, 3> covariance() const
	{
		return surfel::covariance(sum_squares_, num_points_);
	}

	//
	// Get eigenvalues
	//

	constexpr math::Vector3d eigenValues() const
	{
		return surfel::eigenValues(sum_squares_, num_points_);
	}

	//
	// Get eigen vectors
	//

	constexpr std::array<math::Vector3d, 3> eigenVectors() const
	{
		return surfel::eigenVectors(sum_squares_, num_points_);
	}

	//
	// Get num points
	//

	constexpr count_t numPoints() const { return num_points_; }

	//
	// Get sum
	//

	constexpr math::Vector3<surfel_scalar_t> sum() const { return sum_; }

	//
	// Get sum squares
	//

	constexpr std::array<surfel_scalar_t, 6> sumSquares() const { return sum_squares_; }

 private:
	std::array<surfel_scalar_t, 6> sum_squares_ = {0, 0, 0, 0, 0, 0};
	math::Vector3<surfel_scalar_t> sum_;
	count_t num_points_ = 0;

	// TODO: Cache eigen values and vectors
};
}  // namespace ufo::map

ufo::map::Surfel operator+(ufo::map::Surfel lhs, ufo::map::Surfel const& rhs)
{
	lhs += rhs;
	return lhs;
}

ufo::map::Surfel operator-(ufo::map::Surfel lhs, ufo::map::Surfel const& rhs)
{
	lhs -= rhs;
	return lhs;
}

#endif  // UFO_MAP_SURFEL_H