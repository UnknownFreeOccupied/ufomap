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

#ifndef UFO_MATH_POSE6_H
#define UFO_MATH_POSE6_H

// UFO
#include <ufo/math/quaternion.h>
#include <ufo/math/vector3.h>

// STL
#include <cmath>
#include <type_traits>

namespace ufo::math
{
template <typename T>
class Pose6
{
	Vector3<T> translation;
	Quaternion<T> rotation;

	constexpr Pose6() = default;

	template <typename T1, typename T2>
	constexpr Pose6(Vector3<T1> const& translation, Quaternion<T2> const& rotation)
	    : translation(translation), rotation(rotation)
	{
	}

	constexpr Pose6(T x, T y, T z, T roll, T pitch, T yaw)
	    : translation(x, y, z), rotation(roll, pitch, yaw)
	{
	}

	constexpr Pose6(T t_x, T t_y, T t_z, T r_w, T r_x, T r_y, T r_z)
	    : translation(t_x, t_y, t_z), rotation(r_w, r_x, r_y, r_z)
	{
	}

	constexpr bool operator==(Pose6 const& rhs)
	{
		return translation == rhs.translation && rotation == rhs.rotation;
	}

	constexpr bool operator!=(Pose6 const& rhs) { return !(*this == rhs); }

	constexpr T& x() noexcept { return translation[0]; }

	constexpr T x() const noexcept { return translation[0]; }

	constexpr T& y() noexcept { return translation[1]; }

	constexpr T y() const noexcept { return translation[1]; }

	constexpr T& z() noexcept { return translation[2]; }

	constexpr T z() const noexcept { return translation[2]; }

	constexpr T roll() const noexcept { return rotation.roll(); }

	constexpr T pitch() const noexcept { return rotation.pitch(); }

	constexpr T yaw() const noexcept { return rotation.yaw(); }

	template <class P>
	constexpr P transform(P point) const
	{
		point = rotation.rotate(point);
		point += translation;
		return point;
	}

	template <class P>
	constexpr void transformInPlace(P& point) const
	{
		rotation.rotateInPlace(point);
		point += translation;
	}

	constexpr Pose6 inversed() const
	{
		Pose6 result(*this);
		result.rotation.inverse().normalize();
		result.rotation.rotateInPlace(-result.translation);
		return result;
	}

	constexpr Pose6& inverse()
	{
		rotation.inverse().normalize();
		rotation.rotateInPlace(-translation);
		return *this;
	}

	constexpr Pose6 operator*(Pose6 const& other) const
	{
		Quaternion<T> rotationnew = rotation * other.rotation;
		Vector3<T> transation_new = rotation.rotate(other.translation) + translation;
		return Pose6(transation_new, rotationnew.normalize());
	}

	constexpr Pose6& operator*=(Pose6 const& other)
	{
		translation += rotation.rotate(other.translation);
		rotation = rotation * other.rotation;
		return *this;
	}

	constexpr T distance(Pose6 const& other) const
	{
		return translation.distance(other.translation);
	}

	constexpr T translationLength() const { return translation.norm(); }
};

using Pose6f = Pose6<float>;
using Pose6d = Pose6<double>;
}  // namespace ufo::math

#endif  // UFO_MATH_POSE6_H