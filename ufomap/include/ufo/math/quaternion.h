/*
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * License: BSD 3
 */

/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, D. Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
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

#ifndef UFO_MATH_QUATERNION_H
#define UFO_MATH_QUATERNION_H

// UFO
#include <ufo/math/vector3.h>

// STL
#include <array>
#include <cmath>
#include <type_traits>
#include <vector>

namespace ufo::math
{
template <typename T>
struct Quaternion {
	T w = 1;
	T x = 0;
	T y = 0;
	T z = 0;

	constexpr Quaternion() noexcept = default;

	constexpr Quaternion(T w, T x, T y, T z) noexcept : w(w), x(x), y(y), z(z) {}

	template <typename T2>
	constexpr Quaternion(Vector3<T2> const& other) noexcept
	    : Quaternion(other.roll(), other.pitch(), other.yaw())
	{
	}

	constexpr Quaternion(T roll, T pitch, T yaw) noexcept
	{
		T sroll = std::sin(roll);
		T spitch = std::sin(pitch);
		T syaw = std::sin(yaw);
		T croll = std::cos(roll);
		T cpitch = std::cos(pitch);
		T cyaw = std::cos(yaw);

		T m[3][3] = {// create rotational Matrix
		             {cyaw * cpitch, cyaw * spitch * sroll - syaw * croll,
		              cyaw * spitch * croll + syaw * sroll},
		             {syaw * cpitch, syaw * spitch * sroll + cyaw * croll,
		              syaw * spitch * croll - cyaw * sroll},
		             {-spitch, cpitch * sroll, cpitch * croll}};

		T _w = std::sqrt(std::max(T(0), T(1) + m[0][0] + m[1][1] + m[2][2])) / T(2);
		T _x = std::sqrt(std::max(T(0), T(1) + m[0][0] - m[1][1] - m[2][2])) / T(2);
		T _y = std::sqrt(std::max(T(0), T(1) - m[0][0] + m[1][1] - m[2][2])) / T(2);
		T _z = std::sqrt(std::max(T(0), T(1) - m[0][0] - m[1][1] + m[2][2])) / T(2);

		w = _w;
		x = (m[2][1] - m[1][2]) >= T(0) ? std::abs(_x) : -std::abs(_x);
		y = (m[0][2] - m[2][0]) >= T(0) ? std::abs(_y) : -std::abs(_y);
		z = (m[1][0] - m[0][1]) >= T(0) ? std::abs(_z) : -std::abs(_z);
	}

	template <typename T2>
	constexpr Quaternion(Vector3<T2> const& axis, T angle) noexcept
	{
		T sa = std::sin(angle / T(2));
		x = axis.x * sa;
		y = axis.y * sa;
		z = axis.z * sa;
		w = std::cos(angle / T(2));
	}

	constexpr Quaternion(Quaternion const&) noexcept = default;  // Redundant

	template <typename T2, class = std::enable_if_t<not std::is_same_v<T, T2>>>
	constexpr Quaternion(Quaternion<T2> const other) noexcept
	    : w(other.w), x(other.x), y(other.y), z(other.z)
	{
	}

	constexpr Quaternion& operator=(Quaternion const&) noexcept = default;  // Redundant

	template <typename T2, class = std::enable_if_t<not std::is_same_v<T, T2>>>
	constexpr Quaternion& operator=(Quaternion<T2> const rhs) noexcept
	{
		w = rhs.w;
		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
		return *this;
	}

	constexpr Vector3<T> toEuler() const noexcept
	{  // create rotational matrix
		T n = norm();
		T s = n > T(0) ? T(2) / (n * n) : T(0);

		T xs = x * s;
		T ys = y * s;
		T zs = z * s;

		T wx = w * xs;
		T wy = w * ys;
		T wz = w * zs;

		T xx = x * xs;
		T xy = x * ys;
		T xz = x * zs;

		T yy = y * ys;
		T yz = y * zs;
		T zz = z * zs;

		T m[3][3];

		m[0][0] = T(1) - (yy + zz);
		m[1][1] = T(1) - (xx + zz);
		m[2][2] = T(1) - (xx + yy);

		m[1][0] = xy + wz;
		m[0][1] = xy - wz;

		m[2][0] = xz - wy;
		m[0][2] = xz + wy;
		m[2][1] = yz + wx;
		m[1][2] = yz - wx;

		T roll = std::atan2(m[2][1], m[2][2]);
		T pitch = std::atan2(-m[2][0], std::sqrt(m[2][1] * m[2][1] + m[2][2] * m[2][2]));
		T yaw = std::atan2(m[1][0], m[0][0]);

		return Vector3<T>(roll, pitch, yaw);
	}

	constexpr void toRotMatrix(std::array<T, 9>& rot_matrix_3_3) const noexcept
	{
		// create rotational matrix
		T n = norm();
		T s = n > T(0) ? T(2) / (n * n) : T(0);

		T xs = x * s;
		T ys = y * s;
		T zs = z * s;

		T wx = w * xs;
		T wy = w * ys;
		T wz = w * zs;

		T xx = x * xs;
		T xy = x * ys;
		T xz = x * zs;

		T yy = y * ys;
		T yz = y * zs;
		T zz = z * zs;

		T m[3][3];
		m[0][0] = T(1) - (yy + zz);
		m[1][1] = T(1) - (xx + zz);
		m[2][2] = T(1) - (xx + yy);

		m[1][0] = xy + wz;
		m[0][1] = xy - wz;

		m[2][0] = xz - wy;
		m[0][2] = xz + wy;
		m[2][1] = yz + wx;
		m[1][2] = yz - wx;

		for (unsigned int i = 0; i < 3; i++) {
			rot_matrix_3_3[i * 3] = m[i][0];
			rot_matrix_3_3[i * 3 + 1] = m[i][1];
			rot_matrix_3_3[i * 3 + 2] = m[i][2];
		}
	}

	constexpr std::array<T, 9> getRotMatrix() const noexcept
	{
		std::array<T, 9> rot_matrix_3_3;
		toRotMatrix(rot_matrix_3_3);
		return rot_matrix_3_3;
	}

	constexpr T const& operator[](size_t index) const noexcept { return *(&w + index); }

	constexpr T& operator[](size_t index) noexcept { return *(&w + index); }

	constexpr T norm() const noexcept
	{
		T n = 0;
		for (unsigned int i = 0; i < 4; i++) {
			n += operator[](i) * operator[](i);
		}
		return std::sqrt(n);
	}

	constexpr Quaternion normalized() const noexcept
	{
		Quaternion result(*this);
		result.normalize();
		return result;
	}

	constexpr Quaternion& normalize() noexcept
	{
		T len = norm();
		if (len > T(0)) *this /= len;
		return *this;
	}

	constexpr void operator/=(T x) noexcept
	{
		for (unsigned int i = 0; i < 4; ++i) {
			operator[](i) /= x;
		}
	}

	constexpr bool operator==(Quaternion const& rhs) const noexcept
	{
		for (unsigned int i = 0; i < 4; i++) {
			if (operator[](i) != rhs[i]) {
				return false;
			}
		}
		return true;
	}

	constexpr bool operator!=(Quaternion const& rhs) const noexcept
	{
		return !(*this == rhs);
	}

	constexpr Quaternion operator*(Quaternion const& rhs) const noexcept
	{
		return Quaternion(w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
		                  w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
		                  w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
		                  w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w);
	}

	template <typename T2>
	constexpr Quaternion operator*(Vector3<T2> const& v) const noexcept
	{
		return *this * Quaternion(0, v(0), v(1), v(2));
	}

	constexpr Quaternion inversed() const noexcept { return Quaternion(w, -x, -y, -z); }

	constexpr Quaternion& inverse() noexcept
	{
		x = -x;
		y = -y;
		z = -z;
		return *this;
	}

	template <class P>
	constexpr P rotate(P point) const noexcept
	{
		Quaternion q = *this * point * this->inversed();
		point.x = q.x;
		point.y = q.y;
		point.z = q.z;
		return point;
	}

	template <class P>
	constexpr void rotateInPlace(P& point) const noexcept
	{
		Quaternion q = *this * point * this->inversed();
		point.x = q.x;
		point.y = q.y;
		point.z = q.z;
	}

	constexpr T roll() const noexcept { return toEuler()[0]; }

	constexpr T pitch() const noexcept { return toEuler()[1]; }

	constexpr T yaw() const noexcept { return toEuler()[2]; }
};

using Quaternionf = Quaternion<float>;
using Quaterniond = Quaternion<double>;
}  // namespace ufo::math

#endif  // UFO_MATH_QUATERNION_H