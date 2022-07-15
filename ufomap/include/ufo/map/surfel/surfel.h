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
#include <ufo/math/vector3.h>

// STL
#include <cstdint>

namespace ufo::map
{
template <typename T = float>
struct Surfel {
	using scalar_t = T;

	constexpr Surfel() = default;

	constexpr Surfel(math::Vector3<scalar_t> point) : num_points_(1), sum_(point) {}

	template <class InputIt>
	constexpr Surfel(InputIt first, InputIt last) : num_points_(std::distance(first, last))
	{
		if (first == last) {
			return;
		}

		for (; first != last; ++first) {
			sum_ += *first;

			sum_squares_[0] = (*first)[0] * (*first)[0];
			sum_squares_[1] = (*first)[0] * (*first)[1];
			sum_squares_[2] = (*first)[0] * (*first)[2];
			sum_squares_[3] = (*first)[1] * (*first)[1];
			sum_squares_[4] = (*first)[1] * (*first)[2];
			sum_squares_[5] = (*first)[2] * (*first)[2];
		}

		scalar_t n = scalar_t(1) / scalar_t(num_points_);

		sum_squares_[0] -= sum_[0] * sum_[0] * n;
		sum_squares_[1] -= sum_[0] * sum_[1] * n;
		sum_squares_[2] -= sum_[0] * sum_[2] * n;
		sum_squares_[3] -= sum_[1] * sum_[1] * n;
		sum_squares_[4] -= sum_[1] * sum_[2] * n;
		sum_squares_[5] -= sum_[2] * sum_[2] * n;
	}

	constexpr Surfel(std::initializer_list<math::Vector3<scalar_t>> points)
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

	constexpr void addSurfel(Surfel const& other)
	{
		if (0 == num_points_) {
			num_points_ = other.num_points_;
			sum_ = other.sum_;
			sum_squares_ = other.sum_squares_;
		} else {
			scalar_t const n = num_points_;
			scalar_t const n_o = other.num_points_;

			auto const alpha = scalar_t(1) / (n * n_o * (n + n_o));
			auto const beta = (sum_ * n_o) - (other.sum_ * n);

			num_points_ += other.num_points_;
			sum_ += other.sum_;

			sum_squares_[0] += other.sum_squares_[0] + alpha * beta[0] * beta[0];
			sum_squares_[1] += other.sum_squares_[1] + alpha * beta[0] * beta[1];
			sum_squares_[2] += other.sum_squares_[2] + alpha * beta[0] * beta[2];
			sum_squares_[3] += other.sum_squares_[3] + alpha * beta[1] * beta[1];
			sum_squares_[4] += other.sum_squares_[4] + alpha * beta[1] * beta[2];
			sum_squares_[5] += other.sum_squares_[5] + alpha * beta[2] * beta[2];
		}
	}

	//
	// Remove surfel
	//

	Surfel& operator-=(Surfel const& rhs)
	{
		removeSurfel(rhs);
		return *this;
	}

	constexpr void removeSurfel(Surfel const& other)
	{
		if (other.num_points_ >= num_points_) {
			clear();
			return;
		}

		num_points_ -= other.num_points_;
		sum_ -= other.sum_;

		scalar_t const n = num_points_;
		scalar_t const n_o = other.num_points_;

		auto const& alpha = scalar_t(1) / (n * n_o * (n + n_o));
		auto const& beta = (sum_ * n_o) - (other.sum_ * n);

		sum_squares_[0] -= other.sum_squares_[0] - alpha * beta[0] * beta[0];
		sum_squares_[1] -= other.sum_squares_[1] - alpha * beta[0] * beta[1];
		sum_squares_[2] -= other.sum_squares_[2] - alpha * beta[0] * beta[2];
		sum_squares_[3] -= other.sum_squares_[3] - alpha * beta[1] * beta[1];
		sum_squares_[4] -= other.sum_squares_[4] - alpha * beta[1] * beta[2];
		sum_squares_[5] -= other.sum_squares_[5] - alpha * beta[2] * beta[2];
	}

	//
	// Add point
	//

	constexpr void addPoint(math::Vector3<scalar_t> point)
	{
		if (0 == num_points_) {
			num_points_ = 1;
			sum_ = point;
		} else {
			scalar_t const n = num_points_;

			auto const alpha = scalar_t(1) / (n * (n + scalar_t(1)));
			auto const beta = (sum_ - (point * n));

			num_points_ += 1;
			sum_ += point;

			sum_squares_[0] += alpha * beta[0] * beta[0];
			sum_squares_[1] += alpha * beta[0] * beta[1];
			sum_squares_[2] += alpha * beta[0] * beta[2];
			sum_squares_[3] += alpha * beta[1] * beta[1];
			sum_squares_[4] += alpha * beta[1] * beta[2];
			sum_squares_[5] += alpha * beta[2] * beta[2];
		}
	}

	template <class InputIt>
	constexpr void addPoint(InputIt first, InputIt last)
	{
		// FIXME: Improve
		std::for_each(first, last, [this](auto&& p) { addPoint(p); });
	}

	constexpr void addPoint(std::initializer_list<math::Vector3<scalar_t>> points)
	{
		addPoint(std::begin(points), std::end(points));
	}

	//
	// Remove point
	//

	constexpr void removePoint(math::Vector3<scalar_t> point)
	{
		if (0 == num_points_) {
			return;
		} else if (1 == num_points_) {
			clear();
		} else {
			num_points_ -= 1;
			sum_ -= point;

			scalar_t const n = num_points_;

			auto const alpha = scalar_t(1) / (n * (n + scalar_t(1)));
			auto const beta = (sum_ - (point * n));

			sum_squares_[0] -= alpha * beta[0] * beta[0];
			sum_squares_[1] -= alpha * beta[0] * beta[1];
			sum_squares_[2] -= alpha * beta[0] * beta[2];
			sum_squares_[3] -= alpha * beta[1] * beta[1];
			sum_squares_[4] -= alpha * beta[1] * beta[2];
			sum_squares_[5] -= alpha * beta[2] * beta[2];
		}
	}

	template <class InputIt>
	constexpr void removePoint(InputIt first, InputIt last)
	{
		// FIXME: Improve
		std::for_each(first, last, [this](auto&& p) { removePoint(p); });
	}

	constexpr void removePoint(std::initializer_list<math::Vector3<scalar_t>> points)
	{
		removePoint(std::begin(points), std::end(points));
	}

	//
	// Clear
	//

	constexpr void clear()
	{
		num_points_ = 0;
		sum_ = math::Vector3<scalar_t>();
		sum_squares_ = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
	}

	//
	// Get mean
	//

	constexpr math::Vector3<scalar_t> getMean() const
	{
		return sum_ / scalar_t(num_points_);
	}

	//
	// Get covariance
	//

	constexpr std::array<std::array<scalar_t, 3>, 3> getCovariance() const
	{
		scalar_t f = scalar_t(1) / (scalar_t(num_points_) - scalar_t(1));

		std::array<std::array<scalar_t, 3>, 3> covariance;

		covariance[0][0] = f * sum_squares_[0];
		covariance[0][1] = covariance[1][0] = f * sum_squares_[1];
		covariance[0][2] = covariance[2][0] = f * sum_squares_[2];
		covariance[1][1] = f * sum_squares_[3];
		covariance[1][2] = covariance[2][1] = f * sum_squares_[4];
		covariance[2][2] = f * sum_squares_[5];

		return covariance;
	}

	//
	// Get normal
	//

	constexpr math::Vector3<scalar_t> getNormal() const
	{
		// TODO: Implement

		return math::Vector3<scalar_t>();
	}

	//
	// Get num points
	//

	constexpr uint32_t numPoints() const { return num_points_; }

	//
	// Get sum
	//

	constexpr math::Vector3<scalar_t> getSum() const { return sum_; }

	//
	// Get sum squares
	//

	constexpr std::array<scalar_t, 6> getSumSquares() const { return sum_squares_; }

 private:
	uint32_t num_points_ = 0;
	math::Vector3<scalar_t> sum_;
	std::array<scalar_t, 6> sum_squares_ = {0, 0, 0, 0, 0, 0};
};
}  // namespace ufo::map

#endif  // UFO_MAP_SURFEL_H