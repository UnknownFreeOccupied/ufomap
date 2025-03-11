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
#include <ufo/execution/algorithm.hpp>
#include <ufo/execution/execution.hpp>
#include <ufo/map/integrator/detail/bool_grid.hpp>
#include <ufo/map/integrator/detail/count_grid.hpp>
#include <ufo/map/integrator/detail/grid_map.hpp>
#include <ufo/map/integrator/detail/simple_integrator.hpp>
#include <ufo/map/integrator/integrator.hpp>
#include <ufo/map/occupancy/block.hpp>
#include <ufo/utility/index_iterator.hpp>

// STL
#include <algorithm>
#include <cstddef>
#include <iterator>
#include <type_traits>

namespace ufo
{
template <>
class SimpleIntegrator<2> : public Integrator<2>
{
 public:
	//
	// Tags
	//
	using occupancy_t = float;
	using logit_t     = OccupancyElement::logit_t;
	using depth_t     = unsigned;

 public:
	template <class Map, class T, class... Rest>
	void operator()(Map& map, PointCloud<2, T, Rest...> cloud,
	                Vec<2, T> const&       sensor_origin,
	                Transform<2, T> const& frame_origin = {}, bool propagate = true) const
	{
		if (Transform<2, T>{} != frame_origin) {
			transformInPlace(frame_origin, get<0>(cloud));
			// TODO: Should sensor_origin be transformed as well?
		}

		auto misses_f = std::async(std::launch::async,
		                           [this, &map, points = get<0>(cloud), &sensor_origin]() {
			                           return rayCast(map, points, sensor_origin);
		                           });

		filterDistanceInPlace(cloud, sensor_origin, min_distance, max_distance, true);
		auto const hit_nodes = create(map, cloud, hit_depth, propagate);

		auto const [misses, misses_count] = misses_f.get();

		insertMisses(map, misses, misses_count, propagate);
		insertHits(map, hit_nodes, cloud, propagate);

		// TODO: Implement
	}

	template <
	    class ExecutionPolicy, class Map, class T, class... Rest,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void operator()(ExecutionPolicy&& policy, Map& map, PointCloud<2, T, Rest...> cloud,
	                Vec<2, T> const&       sensor_origin,
	                Transform<2, T> const& frame_origin = {}, bool propagate = true) const
	{
		if constexpr (execution::is_seq_v<ExecutionPolicy> ||
		              execution::is_unseq_v<ExecutionPolicy>) {
			operator()(map, cloud, sensor_origin, frame_origin, propagate);
		}

		if (Transform<2, T>{} != frame_origin) {
			transformInPlace(policy, frame_origin, get<0>(cloud));
			// TODO: Should sensor_origin be transformed as well?
		}

		auto misses_f =
		    std::async(std::launch::async,
		               [this, policy, &map, points = get<0>(cloud), &sensor_origin]() {
			               return rayCast(policy, map, points, sensor_origin);
		               });

		filterDistanceInPlace(policy, cloud, sensor_origin, min_distance, max_distance, true);
		auto hit_nodes = create(policy, map, cloud, hit_depth, propagate);

		auto [misses, misses_count] = misses_f.get();
		insertMisses(policy, map, misses, misses_count, propagate);

		for_each(std::forward<ExecutionPolicy>(policy), std::size_t(0), hit_nodes.size(),
		         [&](std::size_t i) {
			         auto node = hit_nodes[i];

			         // This chick wants to rule the block (node.pos being the block)
			         std::lock_guard lock(map.chicken(node.pos));

			         update(map, node, cloud, i, propagate);
		         });

		// TODO: Implement
	}

 private:
	template <class Map, class T>
	[[nodiscard]] std::pair<std::vector<TreeCode<2>>, std::vector<unsigned>> rayCast(
	    Map const& map, std::vector<Vec<2, T>> const& points,
	    Vec<2, T> const& sensor_origin) const
	{
		count_grids_.clear();

		auto const grid_res     = map.length(miss_depth);
		auto const origin       = map.key(TreeCoord<2, T>(sensor_origin, miss_depth));
		auto const voxel_border = map.center(origin) - sensor_origin;

		auto  origin_grid_key = count_grids_.key(map.code(origin));
		auto* origin_grid     = &count_grids_[origin_grid_key];

		auto  grid_key = origin_grid_key;
		auto* grid     = origin_grid;

		for (auto point : points) {
			auto code = map.code(TreeCoord<2, T>(point, miss_depth));
			if (auto cur_grid_key = count_grids_.key(code); grid_key != cur_grid_key) {
				grid_key = cur_grid_key;
				grid     = &count_grids_[grid_key];
			}

			if (bool first_hit = !grid->markHit(code);
			    DownSamplingMethod::NONE != sample_method && !first_hit) {
				continue;
			}

			if (DownSamplingMethod::CENTER == sample_method) {
				point = map.center(code);
			}

			auto dir      = point - sensor_origin;
			auto distance = norm(dir);
			dir /= distance;

			distance = std::clamp(distance + distance_offset, 0.0f, max_distance);

			auto step = sign(dir);

			auto t_max   = (voxel_border + cast<float>(step) * grid_res / 2.0f) / dir;
			auto t_delta = grid_res / abs(dir);
			// FIXME: Is this correct? Should it be zero if all zero?
			auto steps = max(Vec<2, float>(0.0f), ceil((distance - t_max) / t_delta));

			for (std::size_t i{}; 2 > i; ++i) {
				t_max[i]   = 0 != step[i] ? t_max[i] : std::numeric_limits<float>::max();
				t_delta[i] = 0 != step[i] ? t_delta[i] : std::numeric_limits<float>::max();
				steps[i]   = 0 != step[i] ? steps[i] : 0;
			}

			auto cur = origin;
			grid_key = origin_grid_key;
			grid     = origin_grid;

			std::size_t total_steps = sum(steps);
			total_steps -= (free_hits || 0 == total_steps) ? 0 : 1;
			while (total_steps--) {
				// Add
				auto c = map.code(cur);
				if (auto cur_grid_key = count_grids_.key(c); grid_key != cur_grid_key) {
					grid_key = cur_grid_key;
					grid     = &count_grids_[grid_key];
				}
				grid->addMiss(c);

				// Move to next
				auto const advance_dim = minIndex(t_max);
				cur[advance_dim] += step[advance_dim];
				t_max[advance_dim] += t_delta[advance_dim];
			}
		}

		std::pair<std::vector<TreeCode<2>>, std::vector<unsigned>> res;
		std::vector<TreeCode<2>>&                                  misses = res.first;
		std::vector<unsigned>&                                     count  = res.second;

		for (auto const& [key, grid] : count_grids_) {
			for (std::size_t i{}; grid.size() > i; ++i) {
				unsigned num_misses = grid.misses(i);
				if (0u == num_misses) {
					continue;
				}

				misses.emplace_back(grid.code(key, i));
				count.emplace_back(num_misses);
			}
		}

		return res;
	}

 private:
	mutable detail::GridMap<CountGrid<2, 4, false>> count_grids_{4096};
	mutable detail::GridMap<CountGrid<2, 4, true>>  count_grids_atomic_{4096};
};
}  // namespace ufo

#endif  // UFO_MAP_INTEGRATOR_SIMPLE_INTEGRATOR_2D_HPP