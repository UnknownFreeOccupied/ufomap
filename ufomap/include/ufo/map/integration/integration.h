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
#include <ufo/algorithm/algorithm.h>
#include <ufo/geometry/minimum_distance.h>
#include <ufo/map/code.h>
#include <ufo/map/integration/grid.h>
#include <ufo/map/integration/integration_point.h>
#include <ufo/map/integration/integration_point_cloud.h>
#include <ufo/map/key.h>
#include <ufo/map/point.h>
#include <ufo/map/point_cloud.h>
#include <ufo/map/ray_caster/ray_caster.h>
#include <ufo/map/types.h>

// STL
#include <algorithm>
#include <queue>

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
IntegrationCloud<P> toIntegrationCloud(Map const& map, PointCloudT<P> cloud,
                                       TransformFunction trans_f, ValidFunction valid_f,
                                       DepthFunction depth_f)
{
	constexpr static std::size_t mode = 3;

	IntegrationCloud<P> i_cloud;
	auto const cloud_size = cloud.size();

	if constexpr (0 == mode) {
		IntegrationCloudSmall<P> temp;
		temp.reserve(cloud_size);

		// Get the corresponding code for each point
		std::transform(std::cbegin(cloud), std::cend(cloud), std::back_inserter(temp),
		               [&map, trans_f, valid_f, depth_f](P point) {
			               auto depth = depth_f(point);
			               auto valid = valid_f(point);
			               point = trans_f(point);
			               return IntegrationPointSmall<P>(map.toCode(point, depth), point,
			                                               valid);
		               });

		// Sort
		std::sort(std::begin(temp), std::end(temp));

		// Boundle together points with same code and remove duplicates
		i_cloud.reserve(cloud_size);
		Code prev_code;
		for (auto const& p : temp) {
			if (prev_code == p.code) {
				i_cloud.back().points.push_back(p.point);
			} else {
				prev_code = p.code;
				i_cloud.emplace_back(p.code, p.point, p.valid());
			}
		}
	} else if constexpr (1 == mode) {
		i_cloud.reserve(cloud_size);

		// Get the corresponding code for each point
		for (std::size_t i = 0; cloud_size != i; ++i) {
			auto const depth = depth_f(cloud[i]);
			auto const valid = valid_f(cloud[i]);
			auto const point = trans_f(cloud[i]);
			auto const code = map.toCode(point, depth);
			if (0 != i && i_cloud.back().code == code) {
				i_cloud.back().points.emplace_back(point, valid);
			} else {
				i_cloud.emplace_back(code, point, valid);
			}
		}

		// Sort
		std::sort(std::begin(i_cloud), std::end(i_cloud));

		// Boundle together points with same code and remove duplicates
		auto first = std::begin(i_cloud);
		auto last = std::end(i_cloud);
		if (first == last) {
			return i_cloud;
		}

		auto new_last = first;
		do {
			auto new_first = first + 1;
			auto total_size = first->points.size();
			while (new_first == first) {
				total_size += new_first->points.size();
				++new_first;
			}
			first->points.reserve(total_size);
			for (auto it = first + 1; it != new_first; ++it) {
				for (auto e : it->points) {
					first->points.push_back(e);
				}
			}

			if (first != new_last) {
				*new_last = std::move(*first);
			}

			first = new_first;
			++new_last;
		} while (first != last);

		i_cloud.resize(std::distance(std::begin(i_cloud), new_last));
	} else if constexpr (2 == mode) {
		CodeUnorderedMap<std::vector<ValidPoint<P>>> temp;

		// Get the corresponding code for each point
		for (P point : cloud) {
			auto depth = depth_f(point);
			auto valid = valid_f(point);
			point = trans_f(point);
			temp[map.toCode(point, depth)].emplace_back(point, valid);
		}

		// Boundle together points with same code and remove duplicates
		i_cloud.reserve(temp.size());
		for (auto& [code, p] : temp) {
			i_cloud.emplace_back(code).points.swap(p);
		}

		// Sort
		std::sort(std::begin(i_cloud), std::end(i_cloud));
	} else if constexpr (3 == mode) {
		// CodeUnorderedMap<IntegrationCloudSmall<P>> temp;
		std::map<Code, IntegrationCloudSmall<P>> temp;
		// std::unordered_map<Code, IntegrationCloudSmall<P>> temp;

		// Get the corresponding code for each point
		Code prev_code;
		IntegrationCloudSmall<P>* prev;
		for (P point : cloud) {
			auto depth = depth_f(point);
			auto valid = valid_f(point);
			point = trans_f(point);
			auto code = map.toCode(point, depth);
			if (code.toDepth(depth + 5) != prev_code) {
				prev_code = code.toDepth(depth + 5);
				prev = &temp[prev_code];
			}
			prev->emplace_back(code, point, valid);
		}

		// Boundle together points with same code and remove duplicates
		i_cloud.reserve(temp.size());
		for (auto& [_, temp_3] : temp) {
			std::sort(std::begin(temp_3), std::end(temp_3));
			Code prev_code;
			for (auto const& p : temp_3) {
				if (prev_code == p.code) {
					i_cloud.back().points.push_back(p.point);
				} else {
					prev_code = p.code;
					i_cloud.emplace_back(p.code, p.point, p.valid());
				}
			}
		}
	}

	// i_cloud.clear();
	return i_cloud;
}

template <class Map, class P, class TransformFunction, class ValidFunction>
IntegrationCloud<P> toIntegrationCloud(Map const& map, PointCloudT<P> const& cloud,
                                       TransformFunction trans_f, ValidFunction valid_f,
                                       depth_t const depth = 0)
{
	return toIntegrationCloud(map, cloud, trans_f, valid_f,
	                          [depth](auto const&) { return depth; });
}

template <class Map, class P, class DepthFunction>
IntegrationCloud<P> toIntegrationCloud(Map const& map, PointCloudT<P> const& cloud,
                                       DepthFunction depth_f)
{
	return toIntegrationCloud(
	    map, cloud, [](auto const& p) { return p; }, [](auto const&) { return true; },
	    depth_f);
}

template <class Map, class P>
IntegrationCloud<P> toIntegrationCloud(Map const& map, PointCloudT<P> const& cloud,
                                       depth_t const depth = 0)
{
	return toIntegrationCloud(map, cloud, [depth](auto const&) { return depth; });
}

template <class Map, class P, class DepthFunction>
IntegrationCloud<P> toIntegrationCloud(Map const& map, PointCloudT<P> const& cloud,
                                       Point const sensor_origin, double const max_range,
                                       DepthFunction depth_function)
{
	return toIntegrationCloud(
	    map, cloud,
	    [sensor_origin, max_range](auto p) {
		    p -= sensor_origin;
		    p.normalize();
		    p *= max_range;
		    return p;
	    },
	    [sensor_origin, max_sqrt = max_range * max_range](Point const p) {
		    return sensor_origin.squaredDistance(p) <= max_sqrt;
	    },
	    depth_function);
}

template <class Map, class P>
IntegrationCloud<P> toIntegrationCloud(Map const& map, PointCloudT<P> const& cloud,
                                       Point const sensor_origin, double const max_range,
                                       depth_t const depth = 0)
{
	return toIntegrationCloud(map, cloud, sensor_origin, max_range,
	                          [depth](auto const&) { return depth; });
}

template <class Map, class P>
Misses getMisses(Map const& map, IntegrationCloud<P> const& cloud,
                 Point const sensor_origin, bool const only_valid = false,
                 depth_t const depth = 0)
{
	CodeUnorderedSet indices;

	double const step_size = map.size(depth);

	for (auto const& p : cloud) {
		for (auto const& point : p.points) {
			if (only_valid && !point.valid) {
				continue;
			}
			for (auto const& e : computeRaySimple(sensor_origin, point, step_size)) {
				indices.insert(map.toCode(e, depth));
			}
		}
	}

	return Misses(std::cbegin(indices), std::cend(indices));
}

template <class Map, class P>
Misses getMissesDiscrete(Map const& map, IntegrationCloud<P> const& cloud,
                         Point const sensor_origin, bool const only_valid = false,
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

	return Misses(std::cbegin(indices), std::cend(indices));
}

template <class Map, class P>
Misses getMissesDiscreteFast(Map const& map, IntegrationCloud<P> const& cloud,
                             Point const sensor_origin, bool const only_valid = false,
                             depth_t depth = 0)
{
	static constexpr depth_t GRID_DEPTH = 6;  // FIXME: What should this be?

	// NOTE: Assuming all points have same depth
	if (!cloud.empty()) {
		depth = std::max(cloud.front().code.depth(), depth);
	}

	std::vector<Key> keys;
	keys.reserve(cloud.size());
	Code prev;
	for (auto const& p : cloud) {
		Code const cur = p.code.toDepth(depth);
		if (cur == prev || (only_valid && !p.valid())) {
			continue;
		}
		keys.push_back(map.toKey(cur));
		prev = cur;
	}

	CodeUnorderedMap<Grid<GRID_DEPTH>> grids;

	// NOTE: For parallel create all grids here first
	// Key const parallel_origin = map.toKey(sensor_origin, GRID_DEPTH + depth);
	// for (auto const& p : codes) {
	// 	Code const cur = p.code.toDepth(GRID_DEPTH + depth);
	// 	if (cur == prev || (only_valid && !p.valid())) {
	// 		continue;
	// 	}

	// 	prev = cur;

	// 	for (auto const key : computeRay(parallel_origin, cur)) {
	// 		grids.try_emplace(map.toCode(key));
	// 	}
	// }

	Key const origin = map.toKey(sensor_origin, depth);

	KeyRay ray;
	Code prev_at_depth;
	Grid<GRID_DEPTH>* grid = nullptr;
	// std::size_t g = 0;
	// std::size_t b = 0;
	// std::array<std::pair<Code, Grid<GRID_DEPTH>*>, 20> previous;
	for (Key const key : keys) {
		computeRay(ray, origin, key);
		for (Code const code : ray) {
			Code const cur_at_depth = code.toDepth(GRID_DEPTH + depth);

			// bool done = false;
			// for (auto [c, p] : previous) {
			// 	if (c == cur_at_depth) {
			// 		p->set(code);
			// 		done = true;
			// 		// ++g;
			// 		break;
			// 	}
			// }

			// if (!done) {
			// 	// ++b;
			// 	std::rotate(std::rbegin(previous), std::rbegin(previous) + 1,
			// 	            std::rend(previous));
			// 	previous.front().first = cur_at_depth;
			// 	previous.front().second = &grids[cur_at_depth];
			// 	previous.front().second->set(code);
			// }

			if (cur_at_depth != prev_at_depth) {
				prev_at_depth = cur_at_depth;
				grid = &grids[cur_at_depth];
			}

			grid->set(code);
		}
		ray.clear();
	}

	// std::cout << g << " vs " << b << '\n';

	Misses misses;

	for (auto const& [code, grid] : grids) {
		auto const c = code.code();

		std::size_t i = 0;
		for (auto it = std::cbegin(grid); it != std::cend(grid); ++it, i += 64) {
			if (0 == *it) {
				continue;
			}

			if (std::numeric_limits<std::uint64_t>::max() == *it) {
				if (0 == i % 512 && std::all_of(it + 1, it + 8, [](std::uint64_t a) {
					    return std::numeric_limits<std::uint64_t>::max() == a;
				    })) {
					misses.emplace_back(c | (i << 3 * depth), depth + 3);
					it += 7;
					i += (512 - 64);
				} else {
					misses.emplace_back(c | (i << 3 * depth), depth + 2);
				}
			} else {
				// FIXME: Can be improved. Step 8 steps at a time
				for (std::size_t j = 0; 64 != j; ++j) {
					if (0 == j % 8 && (*it >> j) & 0xFF == 0xFF) {
						misses.emplace_back(c | ((i + j) << 3 * depth), depth + 1);
						j += 7;
					} else if (*it & (std::uint64_t(1) << j)) {
						misses.emplace_back(c | ((i + j) << 3 * depth), depth);
					}
				}
			}
		}
	}

	return misses;
}
}  // namespace ufo::map

#endif  // UFO_MAP_INTEGRATION_H