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

#ifndef UFO_MAP_INTEGRATOR_SENSOR_ERROR_HPP
#define UFO_MAP_INTEGRATOR_SENSOR_ERROR_HPP

// STL
#include <cmath>

namespace ufo
{
struct FixedSensorError {
	float error_distance{};

	[[nodiscard]] float operator()(float /* distance */) const { return error_distance; }
};

struct LinearSensorError {
	float factor{};

	[[nodiscard]] float operator()(float distance) const { return factor * distance; }
};

struct PolynomialSensorError {
	float a = 0.002797f;
	float b = -0.004249f;
	float c = 0.007311f;

	[[nodiscard]] float operator()(float distance) const
	{
		return a + b * distance + c * distance * distance;
	}
};

struct ExponentialSensorError {
	float a = 0.0005877f;
	float b = 0.9925f;

	[[nodiscard]] float operator()(float distance) const
	{
		return a * std::exp(b * distance);
	}
};

static constexpr PolynomialSensorError const  KinectV1ErrorFunction{0.002797f, -0.004249f,
                                                                   0.007311};
static constexpr ExponentialSensorError const KinectV2ErrorFunction{0.0005877f, 0.9925f};
static constexpr ExponentialSensorError const ZEDErrorFunction{0.007437f, 0.3855f};
}  // namespace ufo

#endif  // UFO_MAP_INTEGRATOR_SENSOR_ERROR_HPP