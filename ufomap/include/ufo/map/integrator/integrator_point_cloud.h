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

#ifndef UFO_MAP_INTEGRATOR_POINT_CLOUD_H
#define UFO_MAP_INTEGRATOR_POINT_CLOUD_H

// UFO
#include <ufo/map/code.h>
#include <ufo/map/types.h>

// STL
#include <vector>

namespace ufo::map
{
template <class P>
struct DiscretePoint {
	std::vector<P> points;
	Code code;
};

template <class P>
using DiscretePointCloud = PointCloudT<P>;

template <class P>
class DiscretePointCloud
{
 public:
	template <class Map>
	DiscretePointCloud(Map const& map, PointCloudT<P> const& cloud)
	{
	}

 private:
	PointCloudT<DiscretePoint> cloud_;
};

template <class T>
struct IntegratorPoint {
	IntegratorPoint() = default;

	IntegratorPoint(Point3 from, T const& to, Code code, float distance, bool has_moved)
	    : from(from), to(to), code(code), distance(distance), has_moved(has_moved)
	{
	}

	// Sensor origin
	Point3 from;
	// The end point
	T to;
	// The Morton code for the end point
	Code code;
	// Distance from sensor to point
	float distance;
	// If the point's position has moved compared to the initial point cloud
	bool has_moved;
};

template <class T>
using IntegratorPointCloud = std::vector<IntegratorPoint<T>>;

template <class T>
struct IntegratorPoint2 {
	IntegratorPoint2() = default;

	IntegratorPoint2(T const& to, Code code, bool has_moved, float distance)
	    : to(to), code(code), has_moved(has_moved), distance(distance)
	{
	}

	// The end point
	T to;
	// The Morton code for the end point
	Code code;
	// If the point's position has moved compared to the initial point cloud
	bool has_moved;
	// Distance from sensor to point
	float distance;
};

template <class T>
using IntegratorPointCloud2 = std::vector<IntegratorPoint2<T>>;
}  // namespace ufo::map

#endif  // UFO_MAP_INTEGRATOR_POINT_CLOUD_H