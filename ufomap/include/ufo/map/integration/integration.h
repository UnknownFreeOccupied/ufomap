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

#ifndef UFO_MAP_INTEGRATION_H
#define UFO_MAP_INTEGRATION_H

// UFO
#include <ufo/map/code/code.h>
#include <ufo/map/code/code_unordered_set.h>
#include <ufo/map/integration/integration_point.h>
#include <ufo/map/integration/integration_point_cloud.h>
#include <ufo/map/key.h>
#include <ufo/map/point.h>
#include <ufo/map/point_cloud.h>
#include <ufo/map/ray_caster/ray_caster.h>
#include <ufo/map/types.h>

// STL
#include <algorithm>

namespace ufo::map
{

using Misses = std::vector<Code>;

/*!
 * @brief
 *
 * @param map
 * @param cloud
 * @param transform_function
 * @param valid_function
 * @param depth_function
 * @return IntegrationCloud<P>
 */
template <class Map, class P, class TransformFunction, class ValidFunction,
          class DepthFunction>
IntegrationCloud<P> toIntegrationCloud(Map const& map, PointCloudT<P> const& cloud,
                                       TransformFunction trans_f, ValidFunction valid_f,
                                       DepthFunction depth_f)
{
	IntegrationCloud<P> i_cloud;

	if (cloud.empty()) {
		return i_cloud;
	}

	// Get the corresponding code for each points
	i_cloud.reserve(cloud.size());
	std::transform(std::cbegin(cloud), std::cend(cloud), std::back_inserter(i_cloud),
	               [&map, trans_f, valid_f, depth_f](auto const& p) {
		               auto tp = trans_f(p);
		               auto code = map.toCode(tp, depth_f(p));
		               return IntegrationPoint<P>(code, tp, valid_f(p));
	               });

	// Sort
	std::sort(std::begin(i_cloud), std::end(i_cloud));

	// Boundle together points with same code and remove duplicates
	auto cur = std::begin(i_cloud);
	for (auto it = std::next(std::begin(i_cloud)); it != std::end(i_cloud);) {
		if (cur->code == it->code) {
			cur->points.push_back(it->points.front());
			it = i_cloud.erase(it);
		} else {
			cur = it;
			++it;
		}
	}

	return i_cloud;
}

template <class Map, class P, class TransformFunction, class ValidFunction>
IntegrationCloud<P> toIntegrationCloud(Map const& map, PointCloudT<P> const& cloud,
                                       TransformFunction trans_f, ValidFunction valid_f,
                                       depth_t const depth = 0)
{
	return IntegrationCloud(map, cloud, trans_f, valid_f,
	                        [depth](auto const&) { return depth; });
}

template <class Map, class P, class DepthFunction>
IntegrationCloud<P> toIntegrationCloud(Map const& map, PointCloudT<P> const& cloud,
                                       DepthFunction depth_f)
{
	return IntegrationCloud(
	    map, cloud, [](auto const& p) { return p; }, [](auto const&) { return true; },
	    depth_f);
}

template <class Map, class P>
IntegrationCloud<P> toIntegrationCloud(Map const& map, PointCloudT<P> const& cloud,
                                       depth_t const depth = 0)
{
	return IntegrationCloud(map, cloud, [depth](auto const&) { return depth; });
}

template <class Map, class P, class DepthFunction>
IntegrationCloud<P> toIntegrationCloud(Map const& map, PointCloudT<P> const& cloud,
                                       Point3 const sensor_origin, double const max_range,
                                       DepthFunction depth_function)
{
	return IntegrationCloud(
	    map, cloud,
	    [sensor_origin, max_range](auto p) {
		    p -= sensor_origin;
		    p.normalize();
		    p *= max_range;
		    return p;
	    },
	    [sensor_origin, max_sqrt = max_range * max_range](Point3 const p) {
		    return sensor_origin.squaredDistance(p) <= max_sqrt;
	    },
	    depth_function);
}

template <class Map, class P>
IntegrationCloud<P> toIntegrationCloud(Map const& map, PointCloudT<P> const& cloud,
                                       Point3 const sensor_origin, double const max_range,
                                       depth_t const depth = 0)
{
	return IntegrationCloud(map, cloud, sensor_origin, max_range,
	                        [depth](auto const&) { return depth; });
}

template <class Map, class P>
Misses getMisses(Map const& map, IntegrationCloud<P> const& cloud,
                 Point3 const sensor_origin, bool const only_valid = false,
                 depth_t const depth = 0)
{
	CodeUnorderedSet indices;

	for (auto const& p : cloud) {
		for (auto const& [p, v] : p.points) {
			if (only_valid && !v) {
				continue;
			}
			for (auto e : computeRay(sensor_origin, p)) {
				indices.insert(e);
			}
		}
	}

	Misses misses(std::cbegin(indices), std::cend(indices));
	std::sort(std::begin(misses), std::end(misses));
	return misses;
}

template <class Map, class P>
Misses getMissesDiscrete(Map const& map, IntegrationCloud<P> const& cloud,
                         Point3 const sensor_origin, bool const only_valid = false,
                         depth_t const depth = 0)
{
	Key const origin = map.toKey(sensor_origin, depth);

	CodeUnorderedSet indices;
	Code prev;
	for (auto const& p : cloud) {
		if (only_valid && !p.valid()) {
			continue;
		}

		Code cur = p.code.toDepth(std::max(p.code.depth(), depth));
		if (cur == prev) {
			continue;
		}
		prev = cur;

		for (auto e : computeRay(Key(origin, cur.depth()), cur)) {
			indices.insert(e);
		}
	}

	Misses misses(std::cbegin(indices), std::cend(indices));
	std::sort(std::begin(misses), std::end(misses));
	return misses;
}

template <class Map, class P>
Misses getMissesDiscreteFast(Map const& map, IntegrationCloud<P> const& cloud,
                             Point3 const sensor_origin, bool const only_valid = false,
                             depth_t const& depth = 0)
{
	// TODO: Implement

	Misses misses;
	std::sort(std::begin(misses), std::end(misses));
	return misses;
}
}  // namespace ufo::map

#endif  // UFO_MAP_INTEGRATION_H