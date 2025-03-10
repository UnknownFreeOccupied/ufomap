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

#ifndef UFO_MAP_INTEGRATOR_SIMPLE_INTEGRATOR_2D_HPP
#define UFO_MAP_INTEGRATOR_SIMPLE_INTEGRATOR_2D_HPP

// UFO
#include <ufo/cloud/point_cloud.hpp>
#include <ufo/execution/execution.hpp>
#include <ufo/map/integrator/detail/simple_integrator.hpp>
#include <ufo/map/occupancy/block.hpp>
#include <ufo/utility/index_iterator.hpp>
#include <ufo/utility/spinlock.hpp>

// STL
#include <algorithm>
#include <cstddef>
#include <iterator>
#include <type_traits>

namespace ufo
{
template <>
class SimpleIntegrator<2>
{
 public:
	using occupancy_t = float;
	using logit_t     = OccupancyElement::logit_t;
	using depth_t     = unsigned;

	depth_t hit_depth  = 0;
	depth_t miss_depth = 0;

	// Min range to integrate
	float min_distance = 0.0f;
	// Max range to integrate, negative value is infinity range
	float max_distance = std::numeric_limits<float>::infinity();
	// To extend or shorten the rays
	float distance_offset = 0.0f;

	// Occupancy hit [0, 1]
	occupancy_t occupancy_hit = 0.9f;
	// Occupancy miss [0, 1]
	occupancy_t occupancy_miss = 0.45f;

	bool insert_rays_async = true;

	bool insert_rays_hits   = true;
	bool insert_rays_misses = true;

 private:
	struct Config {
		depth_t depth;

		logit_t occupancy_logit;
	};

	// I want one MEGA bite of chickens
	mutable std::array<Spinlock, 1'000'000> chickens;

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                    Insert Points                                    |
	|                                                                                     |
	**************************************************************************************/

	template <class Map, class T, class... Rest>
	void insertPoints(Map& map, PointCloud<2, T, Rest...> const& cloud,
	                  bool propagate = true) const
	{
		insertHits(hitConfig(map), map, cloud, propagate);

		// TODO: Implement
	}

	template <
	    class ExecutionPolicy, class Map, class T, class... Rest,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void insertPoints(ExecutionPolicy&& policy, Map& map,
	                  PointCloud<2, T, Rest...> const& cloud, bool propagate = true) const
	{
		insertHits(std::forward<ExecutionPolicy>(policy), hitConfig(map), map, cloud,
		           propagate);

		// TODO: Implement
	}

	template <class Map, class T, class... Rest>
	void insertPoints(Map& map, PointCloud<2, T, Rest...> cloud,
	                  Transform<2, T> const& frame_origin, bool propagate = true) const
	{
		transformInPlace(frame_origin, get<0>(cloud));
		insertPoints(map, cloud, propagate);
	}

	template <
	    class ExecutionPolicy, class Map, class T, class... Rest,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void insertPoints(ExecutionPolicy&& policy, Map& map, PointCloud<2, T, Rest...> cloud,
	                  Transform<2, T> const& frame_origin, bool propagate = true) const
	{
		transformInPlace(policy, frame_origin, get<0>(cloud));
		insertPoints(std::forward<ExecutionPolicy>(policy), map, cloud, propagate);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Insert Rays                                     |
	|                                                                                     |
	**************************************************************************************/

	template <class Map, class T, class... Rest>
	void insertRays(Map& map, PointCloud<2, T, Rest...> const& cloud,
	                Vec<2, T> const& sensor_origin, bool propagate = true) const
	{
		Config hit_config  = hitConfig(map);
		Config miss_config = missConfig(map);

		auto hits_insert_f = std::async(std::launch::async, [&]() {
			if (!insert_rays_hits) {
				return;
			}

			if (0.0f >= min_distance && std::numeric_limits<float>::max() <= max_distance) {
				insertHits(hit_config, map, cloud, propagate);
			} else {
				insertHits(hit_config, map,
				           filterDistance(cloud, sensor_origin, min_distance, max_distance),
				           propagate);
			}
		});

		if (!insert_rays_misses) {
			hits_insert_f.wait();
			return;
		}

		if (!insert_rays_async) {
			hits_insert_f.wait();
		}

		auto [misses, count] = rayCast(map, get<0>(cloud), sensor_origin);

		if (insert_rays_async) {
			hits_insert_f.wait();
		}

		insertMisses(miss_config, map, misses, count, propagate);

		// TODO: Implement
	}

	template <
	    class ExecutionPolicy, class Map, class T, class... Rest,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void insertRays(ExecutionPolicy&& policy, Map& map,
	                PointCloud<2, T, Rest...> const& cloud, Vec<2, T> const& sensor_origin,
	                bool propagate = true) const
	{
		auto const hit_config  = hitConfig(map);
		auto const miss_config = missConfig(map);

		// TODO: Implement
	}

	template <class Map, class T, class... Rest>
	void insertRays(Map& map, PointCloud<2, T, Rest...> cloud,
	                Vec<2, T> const& sensor_origin, Transform<2, T> const& frame_origin,
	                bool propagate = true) const
	{
		transformInPlace(frame_origin, get<0>(cloud));
		insertRays(map, cloud, sensor_origin, propagate);
	}

	template <
	    class ExecutionPolicy, class Map, class T, class... Rest,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void insertRays(ExecutionPolicy&& policy, Map& map, PointCloud<2, T, Rest...> cloud,
	                Vec<2, T> const& sensor_origin, Transform<2, T> const& frame_origin,
	                bool propagate = true) const
	{
		transformInPlace(policy, frame_origin, get<0>(cloud));
		insertRays(std::forward<ExecutionPolicy>(policy), map, cloud, sensor_origin,
		           propagate);
	}

 private:
	template <class Map, class T, class... Rest>
	void insertHits(Config const& config, Map& map, PointCloud<2, T, Rest...> const& cloud,
	                bool propagate) const
	{
		auto const nodes = create(config, map, cloud, propagate);

		assert(nodes.size() == cloud.size());

		for (std::size_t i{}; nodes.size() > i; ++i) {
			update(config, map, nodes[i], cloud, i, propagate);
		}
	}

	template <
	    class ExecutionPolicy, class Map, class T, class... Rest,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void insertHits(ExecutionPolicy&& policy, Config const& config, Map& map,
	                PointCloud<2, T, Rest...> const& cloud, bool propagate) const
	{
		if constexpr (execution::is_seq_v<ExecutionPolicy> ||
		              execution::is_unseq_v<ExecutionPolicy>) {
			return insertHits(config, map, cloud, propagate);
		}

		auto const nodes = create(policy, config, map, cloud, propagate);

		assert(nodes.size() == cloud.size());

		auto fun = [&](std::size_t i) {
			auto node = nodes[i];

			// This chick wants to rule the block (node.pos being the block)
			std::lock_guard lock(chickens[node.pos % chickens.size()]);

			update(config, map, node, cloud, i, propagate);
		};

		if constexpr (execution::is_stl_v<ExecutionPolicy>) {
			IndexIterator<std::size_t> it(0, nodes.size());
			std::for_each(execution::toSTL(policy), it.begin(), it.end(), fun);
		}
#if defined(UFO_PAR_GCD)
		else if constexpr (execution::is_gcd_v<ExecutionPolicy>) {
			dispatch_apply(nodes.size(), dispatch_get_global_queue(0, 0), ^(std::size_t i) {
				fun(i);
			});
		}
#endif
#if defined(UFO_PAR_TBB)
		else if constexpr (execution::is_tbb_v<ExecutionPolicy>) {
			oneapi::tbb::parallel_for(std::size_t(0), nodes.size(), fun);
		}
#endif
		else if constexpr (execution::is_omp_v<ExecutionPolicy>) {
#pragma omp parallel for
			for (std::size_t i = 0; cloud.size() > i; ++i) {
				fun(i);
			}
		} else {
			static_assert(dependent_false_v<ExecutionPolicy>,
			              "Not implemented for the execution policy 'Unknown'");
		}

		for (std::size_t i{}; nodes.size() > i; ++i) {
			update(config, map, nodes[i], cloud, i, propagate);
		}
	}

	template <class Map, class T, class... Rest>
	void insertMisses(Config const& config, Map& map,
	                  std::vector<TreeCode<2>> const& misses,
	                  std::vector<unsigned> const& count, bool propagate) const
	{
		auto const nodes = create(config, map, misses, propagate);

		assert(nodes.size() == misses.size());

		for (std::size_t i{}; nodes.size() > i; ++i) {
			update(config, map, nodes[i], misses, i, propagate);
		}
	}

	template <class Map>
	[[nodiscard]] Config hitConfig(Map const& map) const
	{
		Config config;

		config.depth = hit_depth;

		if constexpr (Map::mapType(MapType::OCCUPANCY)) {
			// TODO: What function should be used here?
			config.occupancy_logit = map.occupancyLogit(occupancy_hit);
		}

		// TODO: Implement

		return config;
	}

	template <class Map>
	[[nodiscard]] Config missConfig(Map const& map) const
	{
		Config config;

		config.depth = miss_depth;

		if constexpr (Map::mapType(MapType::OCCUPANCY)) {
			// TODO: What function should be used here?
			config.occupancy_logit = map.occupancyLogit(occupancy_miss);
		}

		// TODO: Implement

		return config;
	}

	template <class Map, class T, class... Rest>
	[[nodiscard]] std::vector<TreeIndex> create(Config const& config, Map& map,
	                                            PointCloud<2, T, Rest...> const& cloud,
	                                            bool propagate) const
	{
		auto const depth = config.depth;

		if (0 == depth) {
			return propagate ? map.create(get<0>(cloud)) : map.modifiedSet(get<0>(cloud));
		}

		std::vector<TreeCode<2>> codes;
		codes.reserve(cloud.size());

		std::transform(
		    get<0>(cloud).begin(), get<0>(cloud).end(), std::back_inserter(codes),
		    [this, &map, depth](auto const& p) { return map.code(TreeCoord(p, depth)); });

		return propagate ? map.create(codes) : map.modifiedSet(codes);
	}

	template <
	    class ExecutionPolicy, class Map, class T, class... Rest,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	[[nodiscard]] std::vector<TreeIndex> create(ExecutionPolicy&& policy,
	                                            Config const& config, Map& map,
	                                            PointCloud<2, T, Rest...> const& cloud,
	                                            bool propagate) const
	{
		if constexpr (execution::is_seq_v<ExecutionPolicy> ||
		              execution::is_unseq_v<ExecutionPolicy>) {
			return create(config, map, cloud, propagate);
		}

		auto const depth = config.depth;

		if (0 == depth) {
			return propagate
			           ? map.create(std::forward<ExecutionPolicy>(policy), get<0>(cloud))
			           : map.modifiedSet(std::forward<ExecutionPolicy>(policy), get<0>(cloud));
		}
		if constexpr (execution::is_stl_v<ExecutionPolicy>) {
			std::vector<TreeCode<2>> codes(cloud.size());

			std::transform(execution::toSTL(policy), get<0>(cloud).begin(), get<0>(cloud).end(),
			               codes.begin(), [&map, depth](auto const& p) {
				               return map.code(TreeCoord(p, depth));
			               });
		}
#if defined(UFO_PAR_GCD)
		else if constexpr (execution::is_gcd_v<ExecutionPolicy>) {
			__block std::vector<TreeCode<2>> codes(cloud.size());

			dispatch_apply(cloud.size(), dispatch_get_global_queue(0, 0), ^(std::size_t i) {
				codes[i] = map.code(TreeCoord(get<0>(cloud)[i], depth));
			});

			return propagate ? map.create(std::forward<ExecutionPolicy>(policy), codes)
			                 : map.modifiedSet(std::forward<ExecutionPolicy>(policy), codes);
		}
#endif
#if defined(UFO_PAR_TBB)
		else if constexpr (execution::is_tbb_v<ExecutionPolicy>) {
			std::vector<TreeCode<2>> codes(cloud.size());

			oneapi::tbb::parallel_for(std::size_t(0), cloud.size(),
			                          [&map, &codes, &cloud, depth](std::size_t i) {
				                          codes[i] = map.code(TreeCoord(get<0>(cloud)[i], depth));
			                          });

			return propagate ? map.create(std::forward<ExecutionPolicy>(policy), codes)
			                 : map.modifiedSet(std::forward<ExecutionPolicy>(policy), codes);
		}
#endif
		else if constexpr (execution::is_omp_v<ExecutionPolicy>) {
			std::vector<TreeCode<2>> codes(cloud.size());

#pragma omp parallel for
			for (std::size_t i = 0; cloud.size() > i; ++i) {
				codes[i] = map.code(TreeCoord(get<0>(cloud)[i], depth));
			}

			return propagate ? map.create(std::forward<ExecutionPolicy>(policy), codes)
			                 : map.modifiedSet(std::forward<ExecutionPolicy>(policy), codes);
		} else {
			static_assert(dependent_false_v<ExecutionPolicy>,
			              "Not implemented for the execution policy 'Unknown'");
		}
	}

	template <class Map, class T, class... Rest>
	void update(Config const& config, Map& map, TreeIndex node,
	            PointCloud<2, T, Rest...> const& cloud, std::size_t cloud_index,
	            bool propagate) const
	{
		if constexpr (Map::mapType(MapType::OCCUPANCY)) {
			map.occupancyUpdateLogit(node, config.occupancy_logit, propagate);
		}

		// TODO: Check if cloud contains color
		if constexpr (Map::mapType(MapType::COLOR)) {
			map.colorSet(node, get<Color>(cloud)[cloud_index], propagate);
		}

		// TODO: Add more map types
	}
};
}  // namespace ufo

#endif  // UFO_MAP_INTEGRATOR_SIMPLE_INTEGRATOR_2D_HPP