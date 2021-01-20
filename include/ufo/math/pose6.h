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

#ifndef UFO_MATH_POSE6_H
#define UFO_MATH_POSE6_H

// UFO
#include <ufo/math/quaternion.h>
#include <ufo/math/vector3.h>

// STD
#include <cmath>
#include <type_traits>

namespace ufo::math
{
class Pose6
{
 public:
	Pose6() {}
	~Pose6() {}

	Pose6(Pose6 const& other) : translation_(other.translation_), rotation_(other.rotation_)
	{
	}

	Pose6(Vector3 const& translation, Quaternion const& rotation)
	    : translation_(translation), rotation_(rotation)
	{
	}

	Pose6(double x, double y, double z, double roll, double pitch, double yaw)
	    : translation_(x, y, z), rotation_(roll, pitch, yaw)
	{
	}

	Pose6(double t_x, double t_y, double t_z, double r_w, double r_x, double r_y,
	      double r_z)
	    : translation_(t_x, t_y, t_z), rotation_(r_w, r_x, r_y, r_z)
	{
	}

	Pose6& operator=(Pose6 const& rhs)
	{
		translation_ = rhs.translation_;
		rotation_ = rhs.rotation_;
		return *this;
	}

	bool operator==(Pose6 const& rhs)
	{
		return translation_ == rhs.translation_ && rotation_ == rhs.rotation_;
	}

	bool operator!=(Pose6 const& rhs)
	{
		return translation_ != rhs.translation_ || rotation_ != rhs.rotation_;
	}

	Vector3& translation() { return translation_; }
	Vector3 translation() const { return translation_; }
	Quaternion& rotation() { return rotation_; }
	Quaternion rotation() const { return rotation_; }

	double& x() { return translation_[0]; }
	double x() const { return translation_[0]; }
	double& y() { return translation_[1]; }
	double y() const { return translation_[1]; }
	double& z() { return translation_[2]; }
	double z() const { return translation_[2]; }

	double roll() const { return rotation_.toEuler()[0]; }
	double pitch() const { return rotation_.toEuler()[1]; }
	double yaw() const { return rotation_.toEuler()[2]; }

	template <typename T, typename = std::enable_if_t<std::is_base_of_v<Vector3, T>>>
	T transform(T const& v) const
	{
		// TODO: Improve
		T new_v = v;
		Vector3 result = rotation_.rotate(v);
		new_v.x() = result.x();
		new_v.y() = result.y();
		new_v.z() = result.z();
		new_v += translation_;
		return new_v;
	}

	Pose6 inversed() const
	{
		Pose6 result(*this);
		result.rotation_ = result.rotation_.inversed().normalized();
		result.translation_ = result.rotation_.rotate(-translation_);
		return result;
	}

	Pose6& inverse()
	{
		rotation_ = rotation_.inversed().normalized();
		translation_ = rotation_.rotate(-translation_);
		return *this;
	}

	Pose6 operator*(Pose6 const& other) const
	{
		Quaternion rotation_new = rotation_ * other.rotation_;
		Vector3 transation_new = rotation_.rotate(other.translation_) + translation_;
		return Pose6(transation_new, rotation_new.normalized());
	}

	Pose6& operator*=(Pose6 const& other)
	{
		translation_ += rotation_.rotate(other.translation_);
		rotation_ = rotation_ * other.rotation_;
		return *this;
	}

	double distance(Pose6 const& other) const
	{
		double dist_x = x() - other.x();
		double dist_y = y() - other.y();
		double dist_z = z() - other.z();
		return sqrt((dist_x * dist_x) + (dist_y * dist_y) + (dist_z * dist_z));
	}

	double translationLength() const
	{
		return sqrt((x() * x()) + (y() * y()) + (z() * z()));
	}

 private:
	Vector3 translation_;
	Quaternion rotation_;
};
}  // namespace ufo::math

#endif  // UFO_MATH_POSE6_H