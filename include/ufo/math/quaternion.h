/**
 * UFOMath - the math library used in UFO
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomath
 * License: BSD 3
 *
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

// STD
#include <array>
#include <cmath>
#include <type_traits>
#include <vector>

namespace ufo::math
{
class Quaternion
{
 public:
	Quaternion() : data_{1, 0, 0, 0} {}

	Quaternion(Quaternion const& other) : data_(other.data_) {}

	Quaternion(double w, double x, double y, double z) : data_{w, x, y, z} {}

	Quaternion(Vector3 const& other) : Quaternion(other.roll(), other.pitch(), other.yaw())
	{
	}

	Quaternion(double roll, double pitch, double yaw)
	{
		double sroll = sin(roll);
		double spitch = sin(pitch);
		double syaw = sin(yaw);
		double croll = cos(roll);
		double cpitch = cos(pitch);
		double cyaw = cos(yaw);

		double m[3][3] = {// create rotational Matrix
		                  {cyaw * cpitch, cyaw * spitch * sroll - syaw * croll,
		                   cyaw * spitch * croll + syaw * sroll},
		                  {syaw * cpitch, syaw * spitch * sroll + cyaw * croll,
		                   syaw * spitch * croll - cyaw * sroll},
		                  {-spitch, cpitch * sroll, cpitch * croll}};

		double _w = (double)(sqrt(std::max(0.0, 1 + m[0][0] + m[1][1] + m[2][2])) / 2.0);
		double _x = (double)(sqrt(std::max(0.0, 1 + m[0][0] - m[1][1] - m[2][2])) / 2.0);
		double _y = (double)(sqrt(std::max(0.0, 1 - m[0][0] + m[1][1] - m[2][2])) / 2.0);
		double _z = (double)(sqrt(std::max(0.0, 1 - m[0][0] - m[1][1] + m[2][2])) / 2.0);
		w() = _w;
		x() = (m[2][1] - m[1][2]) >= 0 ? fabs(_x) : -fabs(_x);
		y() = (m[0][2] - m[2][0]) >= 0 ? fabs(_y) : -fabs(_y);
		z() = (m[1][0] - m[0][1]) >= 0 ? fabs(_z) : -fabs(_z);
	}

	Quaternion(Vector3 const& axis, double angle)
	{
		double sa = sin(angle / 2);
		double ca = cos(angle / 2);
		x() = (double)(axis.x() * sa);
		y() = (double)(axis.y() * sa);
		z() = (double)(axis.z() * sa);
		w() = (double)ca;
	}

	Vector3 toEuler() const
	{  // create rotational matrix
		double n = norm();
		double s = n > 0 ? 2.0 / (n * n) : 0.0;

		double xs = x() * s;
		double ys = y() * s;
		double zs = z() * s;

		double wx = w() * xs;
		double wy = w() * ys;
		double wz = w() * zs;

		double xx = x() * xs;
		double xy = x() * ys;
		double xz = x() * zs;

		double yy = y() * ys;
		double yz = y() * zs;
		double zz = z() * zs;

		double m[3][3];

		m[0][0] = 1.0 - (yy + zz);
		m[1][1] = 1.0 - (xx + zz);
		m[2][2] = 1.0 - (xx + yy);

		m[1][0] = xy + wz;
		m[0][1] = xy - wz;

		m[2][0] = xz - wy;
		m[0][2] = xz + wy;
		m[2][1] = yz + wx;
		m[1][2] = yz - wx;

		double roll = (double)atan2(m[2][1], m[2][2]);
		double pitch = (double)atan2(-m[2][0], sqrt(m[2][1] * m[2][1] + m[2][2] * m[2][2]));
		double yaw = (double)atan2(m[1][0], m[0][0]);

		return Vector3(roll, pitch, yaw);
	}

	void toRotMatrix(std::vector<double>& rot_matrix_3_3) const
	{  // create rotational matrix
		double n = norm();
		double s = n > 0 ? 2.0 / (n * n) : 0.0;

		double xs = x() * s;
		double ys = y() * s;
		double zs = z() * s;

		double wx = w() * xs;
		double wy = w() * ys;
		double wz = w() * zs;

		double xx = x() * xs;
		double xy = x() * ys;
		double xz = x() * zs;

		double yy = y() * ys;
		double yz = y() * zs;
		double zz = z() * zs;

		double m[3][3];
		m[0][0] = 1.0 - (yy + zz);
		m[1][1] = 1.0 - (xx + zz);
		m[2][2] = 1.0 - (xx + yy);

		m[1][0] = xy + wz;
		m[0][1] = xy - wz;

		m[2][0] = xz - wy;
		m[0][2] = xz + wy;
		m[2][1] = yz + wx;
		m[1][2] = yz - wx;

		rot_matrix_3_3.clear();
		rot_matrix_3_3.resize(9, 0.0);
		for (unsigned int i = 0; i < 3; i++) {
			rot_matrix_3_3[i * 3] = m[i][0];
			rot_matrix_3_3[i * 3 + 1] = m[i][1];
			rot_matrix_3_3[i * 3 + 2] = m[i][2];
		}
	}

	double const& operator[](size_t index) const { return data_[index]; }

	double& operator[](size_t index) { return data_[index]; }

	double norm() const
	{
		double n = 0;
		for (unsigned int i = 0; i < 4; i++) {
			n += operator[](i) * operator[](i);
		}
		return (double)sqrt(n);
	}

	Quaternion normalized() const
	{
		Quaternion result(*this);
		result.normalize();
		return result;
	}

	Quaternion& normalize()
	{
		double len = norm();
		if (len > 0) *this /= (double)len;
		return *this;
	}

	void operator/=(double x)
	{
		for (unsigned int i = 0; i < 4; ++i) {
			operator[](i) /= x;
		}
	}

	Quaternion& operator=(Quaternion const& rhs)
	{
		w() = rhs.w();
		x() = rhs.x();
		y() = rhs.y();
		z() = rhs.z();
		return *this;
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
		                  y() * rhs.z() - rhs.y() * z() + w() * rhs.x() + rhs.w() * x(),
		                  z() * rhs.x() - rhs.z() * x() + w() * rhs.y() + rhs.w() * y(),
		                  x() * rhs.y() - rhs.x() * y() + w() * rhs.z() + rhs.w() * z());
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
	T rotate(T const& v) const
	{
		// TODO: Improve
		T new_v = v;
		Quaternion q = *this * v * this->inversed();
		new_v.x() = q.x();
		new_v.y() = q.y();
		new_v.z() = q.z();
		return new_v;
	}

	double const& w() const { return data_[0]; }
	double& w() { return data_[0]; }
	double const& x() const { return data_[1]; }
	double& x() { return data_[1]; }
	double const& y() const { return data_[2]; }
	double& y() { return data_[2]; }
	double const& z() const { return data_[3]; }
	double& z() { return data_[3]; }

 private:
	std::array<double, 4> data_;
};
}  // namespace ufo::math

#endif  // UFO_MATH_QUATERNION_H