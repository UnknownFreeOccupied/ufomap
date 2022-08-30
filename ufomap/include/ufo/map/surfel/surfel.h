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
#include <type_traits>

namespace ufo::map
{
template <typename T = float,
          typename std::enable_if_t<std::is_floating_point_v<T>>* = nullptr>
struct Surfel {
	using scalar_t = T;

	constexpr Surfel() = default;

	constexpr Surfel(math::Vector3<scalar_t> point) : num_points_(1), sum_(point) {}

	template <class InputIt>
	constexpr Surfel(InputIt first, InputIt last)
	{
		addPoint(first, last);
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

	friend Surfel operator+(Surfel lhs, Surfel const& rhs)
	{
		lhs += rhs;
		return lhs;
	}

	constexpr void addSurfel(Surfel const& other)
	{
		auto const n = num_points_;
		if (0 == n) {
			num_points_ = other.num_points_;
			sum_ = other.sum_;
			sum_squares_ = other.sum_squares_;
		} else {
			auto const n_o = other.num_points_;

			auto const alpha = n * n_o * (n + n_o);
			auto const beta = (sum_ * n_o) - (other.sum_ * n);

			sum_squares_[0] += other.sum_squares_[0] + beta[0] * beta[0] / alpha;
			sum_squares_[1] += other.sum_squares_[1] + beta[0] * beta[1] / alpha;
			sum_squares_[2] += other.sum_squares_[2] + beta[0] * beta[2] / alpha;
			sum_squares_[3] += other.sum_squares_[3] + beta[1] * beta[1] / alpha;
			sum_squares_[4] += other.sum_squares_[4] + beta[1] * beta[2] / alpha;
			sum_squares_[5] += other.sum_squares_[5] + beta[2] * beta[2] / alpha;

			sum_ += other.sum_;
			num_points_ += n_o;
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

	friend Surfel operator-(Surfel lhs, Surfel const& rhs)
	{
		lhs -= rhs;
		return lhs;
	}

	constexpr void removeSurfel(Surfel const& other)
	{
		if (other.num_points_ >= num_points_) {
			clear();
			return;
		}

		sum_ -= other.sum_;
		num_points_ -= other.num_points_;

		auto const n = num_points_;
		auto const n_o = other.num_points_;

		auto const alpha = n * n_o * (n + n_o);
		auto const beta = (sum_ * n_o) - (other.sum_ * n);

		sum_squares_[0] -= other.sum_squares_[0] - beta[0] * beta[0] / alpha;
		sum_squares_[1] -= other.sum_squares_[1] - beta[0] * beta[1] / alpha;
		sum_squares_[2] -= other.sum_squares_[2] - beta[0] * beta[2] / alpha;
		sum_squares_[3] -= other.sum_squares_[3] - beta[1] * beta[1] / alpha;
		sum_squares_[4] -= other.sum_squares_[4] - beta[1] * beta[2] / alpha;
		sum_squares_[5] -= other.sum_squares_[5] - beta[2] * beta[2] / alpha;
	}

	//
	// Add point
	//

	constexpr void addPoint(math::Vector3<scalar_t> point)
	{
		auto const n = num_points_;

		if (0 == n) {
			num_points_ = 1;
			sum_ = point;
			return;
		}

		auto const alpha = n * (n + 1);
		auto const beta = (sum_ - (point * n));

		sum_squares_[0] += beta[0] * beta[0] / alpha;
		sum_squares_[1] += beta[0] * beta[1] / alpha;
		sum_squares_[2] += beta[0] * beta[2] / alpha;
		sum_squares_[3] += beta[1] * beta[1] / alpha;
		sum_squares_[4] += beta[1] * beta[2] / alpha;
		sum_squares_[5] += beta[2] * beta[2] / alpha;

		sum_ += point;
		++num_points_;
	}

	template <class InputIt>
	constexpr void addPoint(InputIt first, InputIt last)
	{
		if (empty()) {
			// FIXME: Make into a function
			if (first == last) {
				return;
			}

			for (; first != last; ++first) {
				sum_squares_[0] += (*first)[0] * (*first)[0];
				sum_squares_[1] += (*first)[0] * (*first)[1];
				sum_squares_[2] += (*first)[0] * (*first)[2];
				sum_squares_[3] += (*first)[1] * (*first)[1];
				sum_squares_[4] += (*first)[1] * (*first)[2];
				sum_squares_[5] += (*first)[2] * (*first)[2];

				sum_ += *first;
				++num_points_;
			}

			auto const n = num_points_;

			sum_squares_[0] -= sum_[0] * sum_[0] / n;
			sum_squares_[1] -= sum_[0] * sum_[1] / n;
			sum_squares_[2] -= sum_[0] * sum_[2] / n;
			sum_squares_[3] -= sum_[1] * sum_[1] / n;
			sum_squares_[4] -= sum_[1] * sum_[2] / n;
			sum_squares_[5] -= sum_[2] * sum_[2] / n;
		} else {
			// FIXME: Improve
			std::for_each(first, last, [this](auto const& p) { addPoint(p); });
		}
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
		auto const n = num_points_;

		switch (n) {
			case 0:
				return;
			case 1:
				clear();
				return;
			default:
				auto const alpha = n * (n + 1);
				auto const beta = (sum_ - (point * n));

				sum_squares_[0] -= beta[0] * beta[0] / alpha;
				sum_squares_[1] -= beta[0] * beta[1] / alpha;
				sum_squares_[2] -= beta[0] * beta[2] / alpha;
				sum_squares_[3] -= beta[1] * beta[1] / alpha;
				sum_squares_[4] -= beta[1] * beta[2] / alpha;
				sum_squares_[5] -= beta[2] * beta[2] / alpha;

				sum_ -= point;
				--num_points_;
		}
	}

	template <class InputIt>
	constexpr void removePoint(InputIt first, InputIt last)
	{
		// FIXME: Improve
		std::for_each(first, last, [this](auto const& p) { removePoint(p); });
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
		sum_squares_ = {0, 0, 0, 0, 0, 0};
	}

	//
	// Get mean
	//

	constexpr math::Vector3<scalar_t> getMean() const { return sum_ / num_points_; }

	//
	// Get normal
	//

	constexpr math::Vector3<double> getNormal() const
	{
		return getEigenVectors(getSymmetricCovariance())[0].normalize();
	}

	//
	// Get planarity
	//

	constexpr double getPlanarity() const
	{
		auto const e = getEigenValues();
		return 2 * (e[1] - e[0]) / (e[0] + e[1] + e[2]);
	}

	//
	// Get covariance
	//

	constexpr std::array<std::array<double, 3>, 3> getCovariance() const
	{
		using as = std::array<scalar_t, 3>;
		using cov = std::array<as, 3>;

		double const n = num_points_ - 1;
		return cov{as{sum_squares_[0] / n, sum_squares_[1] / n, sum_squares_[2] / n},
		           as{sum_squares_[1] / n, sum_squares_[3] / n, sum_squares_[4] / n},
		           as{sum_squares_[2] / n, sum_squares_[4] / n, sum_squares_[5] / n}};
	}

	//
	// Get eigenvalues
	//

	constexpr math::Vector3<double> getEigenValues() const
	{
		return getEigenValues(getSymmetricCovariance());
	}

	//
	// Get eigen vectors
	//

	constexpr std::array<math::Vector3<double>, 3> getEigenVectors() const
	{
		auto eigen_vectors = getEigenVectors(getSymmetricCovariance());
		for (auto& v : eigen_vectors) {
			v.normalize();  // FIXME: Needed?
		}
		return eigen_vectors;
	}

	//
	// Get num points
	//

	constexpr uint32_t getNumPoints() const { return num_points_; }

	//
	// Get sum
	//

	constexpr math::Vector3<scalar_t> getSum() const { return sum_; }

	//
	// Get sum squares
	//

	constexpr std::array<scalar_t, 6> getSumSquares() const { return sum_squares_; }

 private:
	//
	// Get symmetric convariance
	//

	constexpr std::array<double, 6> getSymmetricCovariance() const
	{
		double const n = num_points_ - 1;
		return {sum_squares_[0] / n, sum_squares_[1] / n, sum_squares_[2] / n,
		        sum_squares_[3] / n, sum_squares_[4] / n, sum_squares_[5] / n};
	}

	//
	// Get eigen values
	//

	constexpr math::Vector3<double> getEigenValues(std::array<double, 6> const& sym_m) const
	{
		double const a = sym_m[0];
		double const b = sym_m[3];
		double const c = sym_m[5];
		double const d = sym_m[1];
		double const e = sym_m[4];
		double const f = sym_m[2];

		double const x_1 =
		    a * a + b * b + c * c - a * b - a * c - b * c + 3 * (d * d + f * f + e * e);

		double const x_2 = -(2 * a - b - c) * (2 * b - a - c) * (2 * c - a - b) +
		                   9 * ((2 * c - a - b) * (d * d) + (2 * b - a - c) * (f * f) +
		                        (2 * a - b - c) * (e * e)) -
		                   54 * (d * e * f);

		double const phi =
		    0 < x_2
		        ? std::atan(std::sqrt(4 * x_1 * x_1 * x_1 - x_2 * x_2) / x_2)
		        : (0 > x_2
		               ? std::atan(std::sqrt(4 * x_1 * x_1 * x_1 - x_2 * x_2) / x_2) + M_PI
		               : M_PI_2);

		return math::Vector3<double>(
		    (a + b + c - 2 * std::sqrt(x_1) * std::cos(phi / 3)) / 3,
		    (a + b + c + 2 * std::sqrt(x_1) * std::cos((phi + M_PI) / 3)) / 3,
		    (a + b + c + 2 * std::sqrt(x_1) * std::cos((phi - M_PI) / 3)) / 3);
	}

	//
	// Get eigen vectors
	//

	constexpr std::array<math::Vector3<scalar_t>, 3> getEigenVectors(
	    std::array<double, 6> const& sym_m) const
	{
		return getEigenVectors(sym_m, getEigenValues(sym_m));
	}

	constexpr std::array<math::Vector3<scalar_t>, 3> getEigenVectors(
	    std::array<double, 6> const& sym_m, math::Vector3<double> const& eigen_values) const
	{
		// FIXME: Make sure denominator is not zero

		double const a = sym_m[0];
		double const b = sym_m[3];
		double const c = sym_m[5];
		double const d = sym_m[1];
		double const e = sym_m[4];
		double const f = 0 == sym_m[2] ? std::numeric_limits<float>::epsilon() : sym_m[2];

		double const l_1 = eigen_values[0];
		double const l_2 = eigen_values[1];
		double const l_3 = eigen_values[2];

		double const m_1 = (d * (c - l_1) - e * f) / (f * (b - l_1) - d * e);
		double const m_2 = (d * (c - l_2) - e * f) / (f * (b - l_2) - d * e);
		double const m_3 = (d * (c - l_3) - e * f) / (f * (b - l_3) - d * e);

		return {math::Vector3<scalar_t>((l_1 - c - e * m_1) / f, m_1, 1),
		        math::Vector3<scalar_t>((l_2 - c - e * m_2) / f, m_2, 1),
		        math::Vector3<scalar_t>((l_3 - c - e * m_3) / f, m_3, 1)};
	}

 private:
	std::array<scalar_t, 6> sum_squares_ = {0, 0, 0, 0, 0, 0};
	math::Vector3<scalar_t> sum_;
	uint32_t num_points_ = 0;
};
}  // namespace ufo::map

#endif  // UFO_MAP_SURFEL_H