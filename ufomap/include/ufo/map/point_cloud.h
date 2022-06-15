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

#ifndef UFO_MAP_POINT_CLOUD_H
#define UFO_MAP_POINT_CLOUD_H

// UFO
#include <ufo/map/point.h>
#include <ufo/math/pose6.h>

// STL
#include <algorithm>
#include <execution>
#include <type_traits>
#include <utility>
#include <vector>

namespace ufo::map
{
template <class P>
using PointCloudT = std::vector<P>;
using PointCloud = PointCloudT<Point3>;
using PointCloudColor = PointCloudT<Point3Color>;
using PointCloudSemantic = PointCloudT<Point3Semantic>;
using PointCloudColorSemantic = PointCloudT<Point3ColorSemantic>;

/*!
 * @brief Transform each point in the point cloud
 *
 * @param transform The transformation to be applied to each point
 */
template <class InputIt, typename T>
void applyTransform(InputIt first, InputIt last, math::Pose6<T> const& transform)
{
	std::for_each(
	    first, last,
	    [t = transform.translation, r = transform.rotation.getRotMatrix()](auto& p) {
		    auto const x = p.x;
		    auto const y = p.y;
		    auto const z = p.z;
		    p.x = r[0] * x + r[1] * y + r[2] * z + t.x;
		    p.y = r[3] * x + r[4] * y + r[5] * z + t.y;
		    p.z = r[6] * x + r[7] * y + r[8] * z + t.z;
	    });
}

template <class ExecutionPolicy, class ForwardIt, typename T,
          typename =
              std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
void applyTransform(ExecutionPolicy&& policy, ForwardIt first, ForwardIt last,
                    math::Pose6<T> const& transform)
{
	std::for_each(
	    std::forward<ExecutionPolicy>(policy), first, last,
	    [t = transform.translation, r = transform.rotation.getRotMatrix()](auto& p) {
		    auto const x = p.x;
		    auto const y = p.y;
		    auto const z = p.z;
		    p.x = r[0] * x + r[1] * y + r[2] * z + t.x;
		    p.y = r[3] * x + r[4] * y + r[5] * z + t.y;
		    p.z = r[6] * x + r[7] * y + r[8] * z + t.z;
	    });
}

template <class P, typename T>
void applyTransform(PointCloudT<P>& cloud, math::Pose6<T> const& transform)
{
	applyTransform(std::begin(cloud), std::end(cloud), transform);
}

template <class ExecutionPolicy, class P, typename T,
          typename =
              std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
void applyTransform(ExecutionPolicy&& policy, PointCloudT<P>& cloud,
                    math::Pose6<T> const& transform)
{
	applyTransform(std::forward<ExecutionPolicy>(policy), std::begin(cloud),
	               std::end(cloud), transform);
}

template <class ForwardIt>
void removeNaN(ForwardIt first, ForwardIt last)
{
	auto it = std::remove_if(first, last, [](auto const& point) {
		return std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z);
	});
	erase(it, last);
}

template <class ExecutionPolicy, class ForwardIt,
          typename =
              std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
void removeNaN(ExecutionPolicy&& policy, ForwardIt first, ForwardIt last)
{
	auto it = std::remove_if(
	    std::forward<ExecutionPolicy>(policy), first, last, [](auto const& point) {
		    return std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z);
	    });
	erase(it, last);
}

template <class P>
void removeNaN(PointCloudT<P>& cloud)
{
	removeNaN(std::begin(cloud), std::end(cloud));
}

template <class ExecutionPolicy, class P,
          typename =
              std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
void removeNaN(ExecutionPolicy&& policy, PointCloudT<P>& cloud)
{
	removeNaN(std::forward<ExecutionPolicy>(policy), std::begin(cloud), std::end(cloud));
}
}  // namespace ufo::map

#endif  // UFO_MAP_POINT_CLOUD_H