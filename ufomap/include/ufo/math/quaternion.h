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
class Quaternion
{
 public:
	inline Quaternion() : data_{1, 0, 0, 0} {}

	inline Quaternion(float w, float x, float y, float z) : data_{w, x, y, z} {}

	inline Quaternion(Vector3 const& other)
	    : Quaternion(other.roll(), other.pitch(), other.yaw())
	{
	}

	inline Quaternion(float roll, float pitch, float yaw)
	{
		float sroll = sin(roll);
		float spitch = sin(pitch);
		float syaw = sin(yaw);
		float croll = cos(roll);
		float cpitch = cos(pitch);
		float cyaw = cos(yaw);

		float m[3][3] = {// create rotational Matrix
		                 {cyaw * cpitch, cyaw * spitch * sroll - syaw * croll,
		                  cyaw * spitch * croll + syaw * sroll},
		                 {syaw * cpitch, syaw * spitch * sroll + cyaw * croll,
		                  syaw * spitch * croll - cyaw * sroll},
		                 {-spitch, cpitch * sroll, cpitch * croll}};

		float _w = (float)(std::sqrt(std::max(0.0f, 1 + m[0][0] + m[1][1] + m[2][2])) / 2.0);
		float _x = (float)(std::sqrt(std::max(0.0f, 1 + m[0][0] - m[1][1] - m[2][2])) / 2.0);
		float _y = (float)(std::sqrt(std::max(0.0f, 1 - m[0][0] + m[1][1] - m[2][2])) / 2.0);
		float _z = (float)(std::sqrt(std::max(0.0f, 1 - m[0][0] - m[1][1] + m[2][2])) / 2.0);
		w() = _w;
		x() = (m[2][1] - m[1][2]) >= 0 ? fabs(_x) : -fabs(_x);
		y() = (m[0][2] - m[2][0]) >= 0 ? fabs(_y) : -fabs(_y);
		z() = (m[1][0] - m[0][1]) >= 0 ? fabs(_z) : -fabs(_z);
	}

	inline Quaternion(Vector3 const& axis, float angle)
	{
		float sa = std::sin(angle / 2);
		float ca = std::cos(angle / 2);
		x() = (float)(axis.x() * sa);
		y() = (float)(axis.y() * sa);
		z() = (float)(axis.z() * sa);
		w() = (float)ca;
	}

	Vector3 toEuler() const
	{  // create rotational matrix
		float n = norm();
		float s = n > 0 ? 2.0 / (n * n) : 0.0;

		float xs = x() * s;
		float ys = y() * s;
		float zs = z() * s;

		float wx = w() * xs;
		float wy = w() * ys;
		float wz = w() * zs;

		float xx = x() * xs;
		float xy = x() * ys;
		float xz = x() * zs;

		float yy = y() * ys;
		float yz = y() * zs;
		float zz = z() * zs;

		float m[3][3];

		m[0][0] = 1.0 - (yy + zz);
		m[1][1] = 1.0 - (xx + zz);
		m[2][2] = 1.0 - (xx + yy);

		m[1][0] = xy + wz;
		m[0][1] = xy - wz;

		m[2][0] = xz - wy;
		m[0][2] = xz + wy;
		m[2][1] = yz + wx;
		m[1][2] = yz - wx;

		float roll = (float)atan2(m[2][1], m[2][2]);
		float pitch = (float)atan2(-m[2][0], sqrt(m[2][1] * m[2][1] + m[2][2] * m[2][2]));
		float yaw = (float)atan2(m[1][0], m[0][0]);

		return Vector3(roll, pitch, yaw);
	}

	void toRotMatrix(std::array<float, 9>& rot_matrix_3_3) const
	{
		// create rotational matrix
		float n = norm();
		float s = n > 0 ? 2.0 / (n * n) : 0.0;

		float xs = x() * s;
		float ys = y() * s;
		float zs = z() * s;

		float wx = w() * xs;
		float wy = w() * ys;
		float wz = w() * zs;

		float xx = x() * xs;
		float xy = x() * ys;
		float xz = x() * zs;

		float yy = y() * ys;
		float yz = y() * zs;
		float zz = z() * zs;

		float m[3][3];
		m[0][0] = 1.0 - (yy + zz);
		m[1][1] = 1.0 - (xx + zz);
		m[2][2] = 1.0 - (xx + yy);

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

	std::array<float, 9> getRotMatrix() const
	{
		std::array<float, 9> rot_matrix_3_3;

		// create rotational matrix
		float n = norm();
		float s = n > 0 ? 2.0 / (n * n) : 0.0;

		float xs = x() * s;
		float ys = y() * s;
		float zs = z() * s;

		float wx = w() * xs;
		float wy = w() * ys;
		float wz = w() * zs;

		float xx = x() * xs;
		float xy = x() * ys;
		float xz = x() * zs;

		float yy = y() * ys;
		float yz = y() * zs;
		float zz = z() * zs;

		float m[3][3];
		m[0][0] = 1.0 - (yy + zz);
		m[1][1] = 1.0 - (xx + zz);
		m[2][2] = 1.0 - (xx + yy);

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

		return rot_matrix_3_3;
	}

	float const& operator[](size_t index) const { return data_[index]; }

	float& operator[](size_t index) { return data_[index]; }

	float norm() const
	{
		float n = 0;
		for (unsigned int i = 0; i < 4; i++) {
			n += operator[](i) * operator[](i);
		}
		return (float)sqrt(n);
	}

	Quaternion normalized() const
	{
		Quaternion result(*this);
		result.normalize();
		return result;
	}

	Quaternion& normalize()
	{
		float len = norm();
		if (len > 0) *this /= (float)len;
		return *this;
	}

	void operator/=(float x)
	{
		for (unsigned int i = 0; i < 4; ++i) {
			operator[](i) /= x;
		}
	}

	bool operator==(Quaternion const& rhs) const
	{
		for (unsigned int i = 0; i < 4; i++) {
			if (operator[](i) != rhs[i]) {
				return false;
			}
		}
		return true;
	}

	bool operator!=(Quaternion const& rhs) const
	{
		for (unsigned int i = 0; i < 4; i++) {
			if (operator[](i) != rhs[i]) {
				return true;
			}
		}
		return false;
	}

	Quaternion operator*(Quaternion const& rhs) const
	{
		return Quaternion(w() * rhs.w() - x() * rhs.x() - y() * rhs.y() - z() * rhs.z(),
		                  w() * rhs.x() + x() * rhs.w() + y() * rhs.z() - z() * rhs.y(),
		                  w() * rhs.y() - x() * rhs.z() + y() * rhs.w() + z() * rhs.x(),
		                  w() * rhs.z() + x() * rhs.y() - y() * rhs.x() + z() * rhs.w());
	}

	Quaternion operator*(Vector3 const& v) const
	{
		return *this * Quaternion(0, v(0), v(1), v(2));
	}

	Quaternion inversed() const { return Quaternion(w(), -x(), -y(), -z()); }

	Quaternion& inverse()
	{
		x() = -x();
		y() = -y();
		z() = -z();
		return *this;
	}

	template <typename T, typename = std::enable_if_t<std::is_base_of_v<Vector3, T>>>
	T rotate(T point) const
	{
		// TODO: Improve
		Quaternion q = *this * point * this->inversed();
		point.x() = q.x();
		point.y() = q.y();
		point.z() = q.z();
		return point;
		// T new_v = v;
		// Quaternion q = *this * v * this->inversed();
		// new_v.x() = q.x();
		// new_v.y() = q.y();
		// new_v.z() = q.z();
		// return new_v;
	}

	template <typename T, typename = std::enable_if_t<std::is_base_of_v<Vector3, T>>>
	void rotateInPlace(T& point) const
	{
		Quaternion q = *this * point * this->inversed();
		point.x() = q.x();
		point.y() = q.y();
		point.z() = q.z();
	}

	constexpr float const& w() const noexcept { return data_[0]; }
	constexpr float& w() noexcept { return data_[0]; }
	constexpr float const& x() const noexcept { return data_[1]; }
	constexpr float& x() noexcept { return data_[1]; }
	constexpr float const& y() const noexcept { return data_[2]; }
	constexpr float& y() noexcept { return data_[2]; }
	constexpr float const& z() const noexcept { return data_[3]; }
	constexpr float& z() noexcept { return data_[3]; }

 private:
	std::array<float, 4> data_;
};
}  // namespace ufo::math

#endif  // UFO_MATH_QUATERNION_H