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

#ifndef UFO_MAP_SURFEL_UTIL_H
#define UFO_MAP_SURFEL_UTIL_H

// UFO
#include <ufo/map/types.h>
#include <ufo/math/vector3.h>

// STL
#include <array>
#include <cmath>
#include <initializer_list>
#include <limits>

namespace ufo::map::surfel
{
//
// Clear
//

constexpr void clear(math::Vector3<surfel_scalar_t>& sum,
                     std::array<surfel_scalar_t, 6>& sum_squares, count_t& num_points)
{
	num_points = 0;
	sum = math::Vector3<surfel_scalar_t>();
	sum_squares = {0, 0, 0, 0, 0, 0};
}

//
// Add
//

constexpr void add(math::Vector3<surfel_scalar_t>& sum,
                   std::array<surfel_scalar_t, 6>& sum_squares, count_t& num_points,
                   math::Vector3<surfel_scalar_t> other_sum,
                   std::array<surfel_scalar_t, 6> const& other_sum_squares,
                   count_t other_num_points)
{
	auto const n = num_points;
	if (0 == n) {
		num_points = other_num_points;
		sum = other_sum;
		sum_squares = other_sum_squares;
	} else {
		math::Vector3d s = sum;
		math::Vector3d s_o = other_sum;
		auto const n_o = other_num_points;

		auto const alpha = n * n_o * (n + n_o);
		auto const beta = (s * n_o) - (s_o * n);

		sum_squares[0] += other_sum_squares[0] + beta[0] * beta[0] / alpha;
		sum_squares[1] += other_sum_squares[1] + beta[0] * beta[1] / alpha;
		sum_squares[2] += other_sum_squares[2] + beta[0] * beta[2] / alpha;
		sum_squares[3] += other_sum_squares[3] + beta[1] * beta[1] / alpha;
		sum_squares[4] += other_sum_squares[4] + beta[1] * beta[2] / alpha;
		sum_squares[5] += other_sum_squares[5] + beta[2] * beta[2] / alpha;

		sum = s + s_o;
		num_points += n_o;
	}
}

constexpr void addPoint(math::Vector3<surfel_scalar_t>& sum,
                        std::array<surfel_scalar_t, 6>& sum_squares, count_t& num_points,
                        math::Vector3d point)
{
	auto const n = num_points;

	if (0 == n) {
		num_points = 1;
		sum = point;
		return;
	}

	math::Vector3d s = sum;
	auto const alpha = n * (n + 1);
	auto const beta = (s - (point * n));

	sum_squares[0] += beta[0] * beta[0] / alpha;
	sum_squares[1] += beta[0] * beta[1] / alpha;
	sum_squares[2] += beta[0] * beta[2] / alpha;
	sum_squares[3] += beta[1] * beta[1] / alpha;
	sum_squares[4] += beta[1] * beta[2] / alpha;
	sum_squares[5] += beta[2] * beta[2] / alpha;

	sum = s + point;
	++num_points;
}

template <class InputIt>
constexpr void addPoint(math::Vector3<surfel_scalar_t>& sum,
                        std::array<surfel_scalar_t, 6>& sum_squares, count_t& num_points,
                        InputIt first, InputIt last)
{
	if (first == last) {
		return;
	}

	math::Vector3d s;
	std::array<double, 6> ss{0, 0, 0, 0, 0, 0};
	count_t n = 0;

	for (; first != last; ++first) {
		math::Vector3d const p = *first;
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
		addPoint(sum, sum_squares, num_points, s);
		return;
	}

	ss[0] -= s[0] * s[0] / n;
	ss[1] -= s[0] * s[1] / n;
	ss[2] -= s[0] * s[2] / n;
	ss[3] -= s[1] * s[1] / n;
	ss[4] -= s[1] * s[2] / n;
	ss[5] -= s[2] * s[2] / n;

	if (0 == num_points) {
		if (1 != n) {
			sum_squares[0] = ss[0];
			sum_squares[1] = ss[1];
			sum_squares[2] = ss[2];
			sum_squares[3] = ss[3];
			sum_squares[4] = ss[4];
			sum_squares[5] = ss[5];
		}
		sum = s;
		num_points = n;
	} else {
		math::Vector3d const s_c = sum;
		auto const n_c = num_points;

		auto const alpha = n_c * n * (n_c + n);
		auto const beta = (s_c * n) - (s * n_c);

		sum_squares[0] += ss[0] + beta[0] * beta[0] / alpha;
		sum_squares[1] += ss[1] + beta[0] * beta[1] / alpha;
		sum_squares[2] += ss[2] + beta[0] * beta[2] / alpha;
		sum_squares[3] += ss[3] + beta[1] * beta[1] / alpha;
		sum_squares[4] += ss[4] + beta[1] * beta[2] / alpha;
		sum_squares[5] += ss[5] + beta[2] * beta[2] / alpha;

		sum = s_c + s;
		num_points += n;
	}
}

template <class T>
constexpr void addPoint(math::Vector3<surfel_scalar_t>& sum,
                        std::array<surfel_scalar_t, 6>& sum_squares, count_t& num_points,
                        std::initializer_list<math::Vector3<T>> points)
{
	addPoint(sum, sum_squares, num_points, std::cbegin(points), std::cend(points));
}

//
// Remove
//

constexpr void remove(math::Vector3<surfel_scalar_t>& sum,
                      std::array<surfel_scalar_t, 6>& sum_squares, count_t& num_points,
                      math::Vector3<surfel_scalar_t> other_sum,
                      std::array<surfel_scalar_t, 6> const& other_sum_squares,
                      count_t other_num_points)
{
	// FIXME: Update with double precision
	if (other_num_points >= num_points) {
		clear(sum, sum_squares, num_points);
		return;
	}

	sum -= other_sum;
	num_points -= other_num_points;

	auto const n = num_points;
	auto const n_o = other_num_points;

	auto const alpha = n * n_o * (n + n_o);
	auto const beta = (sum * n_o) - (other_sum * n);

	sum_squares[0] -= other_sum_squares[0] - beta[0] * beta[0] / alpha;
	sum_squares[1] -= other_sum_squares[1] - beta[0] * beta[1] / alpha;
	sum_squares[2] -= other_sum_squares[2] - beta[0] * beta[2] / alpha;
	sum_squares[3] -= other_sum_squares[3] - beta[1] * beta[1] / alpha;
	sum_squares[4] -= other_sum_squares[4] - beta[1] * beta[2] / alpha;
	sum_squares[5] -= other_sum_squares[5] - beta[2] * beta[2] / alpha;
}

constexpr void removePoint(math::Vector3<surfel_scalar_t>& sum,
                           std::array<surfel_scalar_t, 6>& sum_squares,
                           count_t& num_points, math::Vector3d point)
{
	auto const n = num_points;

	switch (n) {
		case 0:
			return;
		case 1:
			clear(sum, sum_squares, num_points);
			return;
		default:
			// FIXME: Update with double precision
			auto const alpha = n * (n + 1);
			auto const beta = (sum - (point * n));

			sum_squares[0] -= beta[0] * beta[0] / alpha;
			sum_squares[1] -= beta[0] * beta[1] / alpha;
			sum_squares[2] -= beta[0] * beta[2] / alpha;
			sum_squares[3] -= beta[1] * beta[1] / alpha;
			sum_squares[4] -= beta[1] * beta[2] / alpha;
			sum_squares[5] -= beta[2] * beta[2] / alpha;

			sum -= point;
			--num_points;
	}
}

template <class InputIt>
constexpr void removePoint(math::Vector3<surfel_scalar_t>& sum,
                           std::array<surfel_scalar_t, 6>& sum_squares,
                           count_t& num_points, InputIt first, InputIt last)
{
	// FIXME: Optimize
	std::for_each(first, last, [&sum, &sum_squares, &num_points](math::Vector3d p) {
		removePoint(sum, sum_squares, num_points, p);
	});
}

template <class T>
constexpr void removePoint(math::Vector3<surfel_scalar_t>& sum,
                           std::array<surfel_scalar_t, 6>& sum_squares,
                           count_t& num_points,
                           std::initializer_list<math::Vector3<T>> points)
{
	removePoint(sum, sum_squares, num_points, std::cbegin(points), std::cend(points));
}

//
// Mean
//

constexpr math::Vector3<surfel_scalar_t> mean(math::Vector3<surfel_scalar_t> sum,
                                              count_t num_points)
{
	return sum / num_points;
}

//
// Covariance
//

constexpr std::array<std::array<double, 3>, 3> covariance(
    std::array<surfel_scalar_t, 6> const& sum_squares, count_t num_points)
{
	using as = std::array<double, 3>;
	using cov = std::array<as, 3>;

	double const n = num_points - 1;
	return cov{as{sum_squares[0] / n, sum_squares[1] / n, sum_squares[2] / n},
	           as{sum_squares[1] / n, sum_squares[3] / n, sum_squares[4] / n},
	           as{sum_squares[2] / n, sum_squares[4] / n, sum_squares[5] / n}};
}

//
// Symmetric covariance
//

constexpr std::array<double, 6> symmetricCovariance(
    std::array<surfel_scalar_t, 6> const& sum_squares, count_t num_points)
{
	double const n = num_points - 1;
	return {sum_squares[0] / n, sum_squares[1] / n, sum_squares[2] / n,
	        sum_squares[3] / n, sum_squares[4] / n, sum_squares[5] / n};
}

//
// Eigen values
//

constexpr math::Vector3d eigenValues(std::array<double, 6> const& sym_m)
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
	        : (0 > x_2 ? std::atan(std::sqrt(4 * x_1 * x_1 * x_1 - x_2 * x_2) / x_2) + M_PI
	                   : M_PI_2);

	return math::Vector3d(
	    (a + b + c - 2 * std::sqrt(x_1) * std::cos(phi / 3)) / 3,
	    (a + b + c + 2 * std::sqrt(x_1) * std::cos((phi + M_PI) / 3)) / 3,
	    (a + b + c + 2 * std::sqrt(x_1) * std::cos((phi - M_PI) / 3)) / 3);
}

constexpr math::Vector3d eigenValues(std::array<surfel_scalar_t, 6> const& sum_squares,
                                     count_t num_points)
{
	return eigenValues(symmetricCovariance(sum_squares, num_points));
}

//
// Eigen vectors
//

constexpr std::array<math::Vector3d, 3> eigenVectors(std::array<double, 6> const& sym_m,
                                                     math::Vector3d const& eigen_values)
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

	return {math::Vector3d((l_1 - c - e * m_1) / f, m_1, 1).normalized(),
	        math::Vector3d((l_2 - c - e * m_2) / f, m_2, 1).normalized(),
	        math::Vector3d((l_3 - c - e * m_3) / f, m_3, 1).normalized()};
}

constexpr std::array<math::Vector3d, 3> eigenVectors(
    std::array<surfel_scalar_t, 6> const& sum_squares, count_t num_points)
{
	auto sym_m = symmetricCovariance(sum_squares, num_points);
	return eigenVectors(sym_m, eigenValues(sym_m));
}

constexpr std::array<math::Vector3d, 3> eigenVectors(std::array<double, 6> const& sym_m)
{
	return eigenVectors(sym_m, eigenValues(sym_m));
}

//
// Normal
//

constexpr math::Vector3d normal(std::array<surfel_scalar_t, 6> const& sum_squares,
                                count_t num_points)
{
	return eigenVectors(sum_squares, num_points)[0];
}

//
// Planarity
//

constexpr double planarity(std::array<surfel_scalar_t, 6> const& sum_squares,
                           count_t num_points)
{
	auto const e = eigenValues(symmetricCovariance(sum_squares, num_points));
	return 2 * (e[1] - e[0]) / (e[0] + e[1] + e[2]);
}
}  // namespace ufo::map::surfel

#endif  // UFO_MAP_SURFEL_UTIL_H