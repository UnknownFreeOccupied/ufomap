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

#ifndef UFO_MAP_SURFEL_HPP
#define UFO_MAP_SURFEL_HPP

// UFO
#include <ufo/map/types.hpp>
#include <ufo/math/vec3.hpp>

// STL
#include <cstdint>
#include <type_traits>

namespace ufo
{
class Surfel
{
 public:
	constexpr Surfel() = default;

	constexpr Surfel(Vec3<surfel_scalar_t> sum, std::array<surfel_scalar_t, 6> sum_squares,
	                 count_t num_points)
	    : sum_(sum), sum_squares_(sum_squares), num_points_(num_points)
	{
	}

	constexpr Surfel(Vec3<surfel_scalar_t> point) : num_points_(1), sum_(point) {}

	template <class InputIt>
	constexpr Surfel(InputIt first, InputIt last)
	{
		add(first, last);
	}

	template <class PointRange>
	constexpr Surfel(PointRange const& points)
	    : Surfel(std::begin(points), std::end(points))
	{
	}

	constexpr Surfel(std::initializer_list<Vec3<surfel_scalar_t>> points)
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
	// Add
	//

	Surfel& operator+=(Surfel const& rhs)
	{
		add(rhs);
		return *this;
	}

	Surfel& operator+=(Vec3<surfel_scalar_t> rhs)
	{
		add(rhs);
		return *this;
	}

	friend Surfel operator+(Surfel lhs, Surfel const& rhs);

	friend Surfel operator+(Surfel lhs, Vec3<surfel_scalar_t> rhs);

	constexpr void add(Surfel const& surfel)
	{
		auto const n = num_points_;
		if (0 == n) {
			num_points_  = surfel.num_points_;
			sum_         = surfel.sum_;
			sum_squares_ = surfel.sum_squares_;
		} else {
			Vec3d      s   = sum_;
			Vec3d      s_o = surfel.sum_;
			auto const n_o = surfel.num_points_;

			auto const alpha = n * n_o * (n + n_o);
			auto const beta  = (s * n_o) - (s_o * n);

			sum_squares_[0] += surfel.sum_squares_[0] + beta[0] * beta[0] / alpha;
			sum_squares_[1] += surfel.sum_squares_[1] + beta[0] * beta[1] / alpha;
			sum_squares_[2] += surfel.sum_squares_[2] + beta[0] * beta[2] / alpha;
			sum_squares_[3] += surfel.sum_squares_[3] + beta[1] * beta[1] / alpha;
			sum_squares_[4] += surfel.sum_squares_[4] + beta[1] * beta[2] / alpha;
			sum_squares_[5] += surfel.sum_squares_[5] + beta[2] * beta[2] / alpha;

			sum_ = s + s_o;
			num_points_ += n_o;
		}
	}

	constexpr void add(Vec3d point)
	{
		auto const n = num_points_;

		if (0 == n) {
			num_points_ = 1;
			sum_        = point;
			return;
		}

		Vec3d      s     = sum_;
		auto const alpha = n * (n + 1);
		auto const beta  = (s - (point * n));

		sum_squares_[0] += beta[0] * beta[0] / alpha;
		sum_squares_[1] += beta[0] * beta[1] / alpha;
		sum_squares_[2] += beta[0] * beta[2] / alpha;
		sum_squares_[3] += beta[1] * beta[1] / alpha;
		sum_squares_[4] += beta[1] * beta[2] / alpha;
		sum_squares_[5] += beta[2] * beta[2] / alpha;

		sum_ = s + point;
		++num_points_;
	}

	template <class InputIt>
	constexpr void add(InputIt first, InputIt last)
	{
		if (first == last) {
			return;
		}

		Vec3d                 s;
		std::array<double, 6> ss{0, 0, 0, 0, 0, 0};
		count_t               n = 0;

		for (; first != last; ++first) {
			Vec3d const p = *first;
			ss[0] += p[0] * p[0];
			ss[1] += p[0] * p[1];
			ss[2] += p[0] * p[2];
			ss[3] += p[1] * p[1];
			ss[4] += p[1] * p[2];
			ss[5] += p[2] * p[2];

			s += p;
			++n;
		}

		if (1 == n) {
			add(s);
			return;
		}

		ss[0] -= s[0] * s[0] / n;
		ss[1] -= s[0] * s[1] / n;
		ss[2] -= s[0] * s[2] / n;
		ss[3] -= s[1] * s[1] / n;
		ss[4] -= s[1] * s[2] / n;
		ss[5] -= s[2] * s[2] / n;

		if (0 == num_points_) {
			if (1 != n) {
				sum_squares_[0] = ss[0];
				sum_squares_[1] = ss[1];
				sum_squares_[2] = ss[2];
				sum_squares_[3] = ss[3];
				sum_squares_[4] = ss[4];
				sum_squares_[5] = ss[5];
			}
			sum_        = s;
			num_points_ = n;
		} else {
			Vec3d const s_c = sum_;
			auto const  n_c = num_points_;

			auto const alpha = n_c * n * (n_c + n);
			auto const beta  = (s_c * n) - (s * n_c);

			sum_squares_[0] += ss[0] + beta[0] * beta[0] / alpha;
			sum_squares_[1] += ss[1] + beta[0] * beta[1] / alpha;
			sum_squares_[2] += ss[2] + beta[0] * beta[2] / alpha;
			sum_squares_[3] += ss[3] + beta[1] * beta[1] / alpha;
			sum_squares_[4] += ss[4] + beta[1] * beta[2] / alpha;
			sum_squares_[5] += ss[5] + beta[2] * beta[2] / alpha;

			sum_ = s_c + s;
			num_points_ += n;
		}
	}

	template <class PointRange>
	constexpr void add(PointRange const& points)
	{
		add(std::cbegin(points), std::cend(points));
	}

	constexpr void add(std::initializer_list<Vec3<surfel_scalar_t>> points)
	{
		add(std::cbegin(points), std::cend(points));
	}

	//
	// Remove
	//

	Surfel& operator-=(Surfel const& rhs)
	{
		remove(rhs);
		return *this;
	}

	friend Surfel operator-(Surfel lhs, Surfel const& rhs);

	constexpr void remove(Surfel const& surfel)
	{
		// FIXME: Update with double precision
		if (surfel.num_points_ >= num_points_) {
			clear();
			return;
		}

		sum_ -= surfel.sum_;
		num_points_ -= surfel.num_points_;

		auto const n   = num_points_;
		auto const n_o = surfel.num_points_;

		auto const alpha = n * n_o * (n + n_o);
		auto const beta  = (sum_ * n_o) - (surfel.sum_ * n);

		sum_squares_[0] -= surfel.sum_squares_[0] - beta[0] * beta[0] / alpha;
		sum_squares_[1] -= surfel.sum_squares_[1] - beta[0] * beta[1] / alpha;
		sum_squares_[2] -= surfel.sum_squares_[2] - beta[0] * beta[2] / alpha;
		sum_squares_[3] -= surfel.sum_squares_[3] - beta[1] * beta[1] / alpha;
		sum_squares_[4] -= surfel.sum_squares_[4] - beta[1] * beta[2] / alpha;
		sum_squares_[5] -= surfel.sum_squares_[5] - beta[2] * beta[2] / alpha;
	}

	constexpr void remove(Vec3d point)
	{
		auto const n = num_points_;

		switch (n) {
			case 0: return;
			case 1: clear(); return;
			default:
				// FIXME: Update with double precision
				auto const alpha = n * (n + 1);
				auto const beta  = (sum_ - (point * n));

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
	constexpr void remove(InputIt first, InputIt last)
	{
		// FIXME: Optimize
		std::for_each(first, last, [this](Vec3d p) { remove(p); });
	}

	template <class PointRange>
	constexpr void remove(PointRange const& points)
	{
		remove(std::cbegin(points), std::cend(points));
	}

	constexpr void remove(std::initializer_list<Vec3<surfel_scalar_t>> points)
	{
		remove(std::cbegin(points), std::cend(points));
	}

	//
	// Clear
	//

	constexpr void clear()
	{
		num_points_  = 0;
		sum_         = {0, 0, 0};
		sum_squares_ = {0, 0, 0, 0, 0, 0};
	}

	//
	// Get mean
	//

	constexpr Vec3<surfel_scalar_t> mean() const { return sum_ / num_points_; }

	//
	// Get normal
	//

	constexpr Vec3d normal() const { return eigenVectors()[0]; }

	//
	// Get planarity
	//

	constexpr double planarity() const
	{
		auto const e = eigenValues();
		return 2 * (e[1] - e[0]) / (e[0] + e[1] + e[2]);
	}

	//
	// Get covariance
	//

	constexpr std::array<std::array<double, 3>, 3> covariance() const
	{
		using as  = std::array<double, 3>;
		using cov = std::array<as, 3>;

		double const n = num_points_ - 1;
		return cov{as{sum_squares_[0] / n, sum_squares_[1] / n, sum_squares_[2] / n},
		           as{sum_squares_[1] / n, sum_squares_[3] / n, sum_squares_[4] / n},
		           as{sum_squares_[2] / n, sum_squares_[4] / n, sum_squares_[5] / n}};
	}

	constexpr std::array<double, 6> symmetricCovariance() const
	{
		double const n = num_points_ - 1;
		return {sum_squares_[0] / n, sum_squares_[1] / n, sum_squares_[2] / n,
		        sum_squares_[3] / n, sum_squares_[4] / n, sum_squares_[5] / n};
	}

	//
	// Get eigenvalues
	//

	constexpr Vec3d eigenValues() const { return eigenValues(symmetricCovariance()); }

	//
	// Get eigen vectors
	//

	constexpr std::array<Vec3d, 3> eigenVectors() const
	{
		auto sym_m = symmetricCovariance();
		return eigenVectors(sym_m, eigenValues(sym_m));
	}

	//
	// Get num points
	//

	constexpr count_t numPoints() const { return num_points_; }

	//
	// Get sum
	//

	constexpr Vec3<surfel_scalar_t> sum() const { return sum_; }

	//
	// Get sum squares
	//

	constexpr std::array<surfel_scalar_t, 6> sumSquares() const { return sum_squares_; }

 protected:
	//
	// Eigen values
	//

	constexpr Vec3d eigenValues(std::array<double, 6> const& sym_m) const
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

		return Vec3d((a + b + c - 2 * std::sqrt(x_1) * std::cos(phi / 3)) / 3,
		             (a + b + c + 2 * std::sqrt(x_1) * std::cos((phi + M_PI) / 3)) / 3,
		             (a + b + c + 2 * std::sqrt(x_1) * std::cos((phi - M_PI) / 3)) / 3);
	}

	//
	// Eigen vectors
	//

	constexpr std::array<Vec3d, 3> eigenVectors(std::array<double, 6> const& sym_m,
	                                            Vec3d const& eigen_values) const
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

		return {Vec3d((l_1 - c - e * m_1) / f, m_1, 1).normalized(),
		        Vec3d((l_2 - c - e * m_2) / f, m_2, 1).normalized(),
		        Vec3d((l_3 - c - e * m_3) / f, m_3, 1).normalized()};
	}

 protected:
	std::array<surfel_scalar_t, 6> sum_squares_{0, 0, 0, 0, 0, 0};
	Vec3<surfel_scalar_t>          sum_;
	count_t                        num_points_{};

	// TODO: Cache eigen values and vectors
};

class SurfelCached : public Surfel
{
 public:
	constexpr SurfelCached() = default;

	constexpr SurfelCached(Vec3<surfel_scalar_t>          sum,
	                       std::array<surfel_scalar_t, 6> sum_squares, count_t num_points)
	    : Surfel(sum, sum_squares, num_points)
	{
	}

	constexpr SurfelCached(Vec3<surfel_scalar_t> point) : Surfel(point) {}

	template <class InputIt>
	constexpr SurfelCached(InputIt first, InputIt last) : Surfel(first, last)
	{
	}

	template <class PointRange>
	constexpr SurfelCached(PointRange const& points) : Surfel(points)
	{
	}

	constexpr SurfelCached(std::initializer_list<Vec3<surfel_scalar_t>> points)
	    : Surfel(points)
	{
	}

	constexpr SurfelCached(Surfel const& other) : Surfel(other) {}

	constexpr SurfelCached(Surfel&& other) : Surfel(std::move(other)) {}

	constexpr SurfelCached(SurfelCached const& other) = default;

	constexpr SurfelCached(SurfelCached&& other) = default;

	constexpr SurfelCached& operator=(SurfelCached const& rhs) = default;

	constexpr SurfelCached& operator=(SurfelCached&& rhs) = default;

	constexpr bool operator==(SurfelCached const& rhs) const
	{
		return static_cast<Surfel const&>(*this) == rhs;
	}

	constexpr bool operator!=(SurfelCached const& rhs) const { return !(*this == rhs); }

	//
	// Add
	//

	SurfelCached& operator+=(Surfel const& rhs)
	{
		add(rhs);
		return *this;
	}

	SurfelCached& operator+=(Vec3<surfel_scalar_t> rhs)
	{
		add(rhs);
		return *this;
	}

	friend SurfelCached operator+(SurfelCached lhs, Surfel const& rhs);

	friend SurfelCached operator+(SurfelCached lhs, Vec3<surfel_scalar_t> rhs);

	constexpr void add(Surfel const& surfel)
	{
		Surfel::add(surfel);
		invalidateCache();
	}

	constexpr void add(Vec3d point)
	{
		Surfel::add(point);
		invalidateCache();
	}

	template <class InputIt>
	constexpr void add(InputIt first, InputIt last)
	{
		Surfel::add(first, last);
		invalidateCache();
	}

	template <class PointRange>
	constexpr void add(PointRange const& points)
	{
		Surfel::add(points);
		invalidateCache();
	}

	constexpr void add(std::initializer_list<Vec3<surfel_scalar_t>> points)
	{
		Surfel::add(points);
		invalidateCache();
	}

	//
	// Remove
	//

	SurfelCached& operator-=(Surfel const& rhs)
	{
		remove(rhs);
		return *this;
	}

	SurfelCached& operator-=(Vec3<surfel_scalar_t> rhs)
	{
		remove(rhs);
		return *this;
	}

	friend SurfelCached operator-(SurfelCached lhs, Surfel const& rhs);

	friend SurfelCached operator-(SurfelCached lhs, Vec3<surfel_scalar_t> rhs);

	constexpr void remove(Surfel const& surfel)
	{
		Surfel::remove(surfel);
		invalidateCache();
	}

	constexpr void remove(Vec3d point)
	{
		Surfel::remove(point);
		invalidateCache();
	}

	template <class InputIt>
	constexpr void remove(InputIt first, InputIt last)
	{
		Surfel::remove(first, last);
		invalidateCache();
	}

	template <class PointRange>
	constexpr void remove(PointRange const& points)
	{
		Surfel::remove(points);
		invalidateCache();
	}

	constexpr void remove(std::initializer_list<Vec3<surfel_scalar_t>> points)
	{
		Surfel::remove(points);
		invalidateCache();
	}

	//
	// Clear
	//

	constexpr void clear() noexcept
	{
		Surfel::clear();
		invalidateCache();
	}

	//
	// Get normal
	//

	constexpr Vec3d normal() const { return eigenVectors()[0]; }

	//
	// Get planarity
	//

	constexpr double planarity() const
	{
		auto const e = eigenValues();
		return 2 * (e[1] - e[0]) / (e[0] + e[1] + e[2]);
	}

	//
	// Get eigenvalues
	//

	constexpr Vec3d eigenValues() const
	{
		if (!cached()) {
			eigen_values_ = Surfel::eigenValues();
		}
		return eigen_values_;
	}

	//
	// Get eigen vectors
	//

	constexpr std::array<Vec3d, 3> const& eigenVectors() const
	{
		if (!cached()) {
			eigen_vectors_ = Surfel::eigenVectors();
		}
		return eigen_vectors_;
	}

 private:
	[[nodiscard]] bool cached() const
	{
		return 0 != eigen_values_[0] || 0 != eigen_values_[1] || 0 != eigen_values_[2];
	}

	void invalidateCache() { eigen_values_ = {0, 0, 0}; }

 private:
	mutable Vec3d                eigen_values_;
	mutable std::array<Vec3d, 3> eigen_vectors_;
};
}  // namespace ufo

inline ufo::Surfel operator+(ufo::Surfel lhs, ufo::Surfel const& rhs)
{
	lhs += rhs;
	return lhs;
}

inline ufo::Surfel operator-(ufo::Surfel lhs, ufo::Surfel const& rhs)
{
	lhs -= rhs;
	return lhs;
}

#endif  // UFO_MAP_SURFEL_HPP