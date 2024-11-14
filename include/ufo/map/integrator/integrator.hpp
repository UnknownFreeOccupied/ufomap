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

#ifndef UFO_MAP_INTEGRATOR_INTEGRATOR_HPP
#define UFO_MAP_INTEGRATOR_INTEGRATOR_HPP

// UFO
#include <ufo/cloud/point_cloud.hpp>
#include <ufo/container/tree/code.hpp>
#include <ufo/container/tree/index.hpp>
#include <ufo/map/color/map.hpp>
#include <ufo/map/distance/map.hpp>
#include <ufo/map/integrator/detail/bool_grid.hpp>
#include <ufo/map/integrator/detail/count_grid.hpp>
#include <ufo/map/integrator/detail/grid_map.hpp>
#include <ufo/map/occupancy/map.hpp>
#include <ufo/math/math.hpp>
#include <ufo/utility/execution.hpp>
#include <ufo/utility/index_iterator.hpp>
#include <ufo/utility/spinlock.hpp>

// STL
#include <cmath>
#include <future>
#include <mutex>

namespace ufo
{
enum class DownSamplingMethod { NONE, FIRST, CENTER };

struct Integrator {
	using occupancy_t = float;
	using logit_t     = std::int8_t;
	using depth_t     = unsigned;
	using time_t      = float;

	// Time
	mutable time_t time = 1;
	// How much time should automatically increase after function call
	time_t time_auto_inc = 1;

	// Occupancy hit [0, 1]
	occupancy_t occupancy_hit = 0.9f;
	// Occupancy miss [0, 1]
	occupancy_t occupancy_miss = 0.45f;

	// Min range to integrate
	float min_distance = 0.0f;
	// Max range to integrate, negative value is infinity range
	float max_distance = std::numeric_limits<float>::infinity();

	// To extend or shorten the rays
	float distance_offset = 0.0f;

	depth_t hit_depth  = 0;
	depth_t miss_depth = 0;

	DownSamplingMethod sample_method = DownSamplingMethod::NONE;

	bool free_hits = false;

	bool counted = false;

 private:
	mutable detail::GridMap<BoolGrid<3, 5, true>, BoolGrid<3, 5, true>>  grids{4096};
	mutable detail::GridMap<CountGrid<3, 4, true>, BoolGrid<3, 4, true>> count_grids{4096};

	// I want one MEGA bite of chickens
	mutable std::array<Spinlock, 1'000'000> chickens;

 public:
	template <
	    class ExecutionPolicy, class Map, std::size_t Dim, class T, class... Rest,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void insertPoints(ExecutionPolicy&& policy, Map& map,
	                  PointCloud<Dim, T, Rest...> const& cloud, bool propagate = true) const
	{
		auto nodes = propagate ? map.create(policy, get<0>(cloud))
		                       : map.modifiedSet(policy, get<0>(cloud));

		logit_t occupancy_hit_logit;
		if constexpr (Map::mapType(MapType::OCCUPANCY)) {
			// TODO: What function should be used here?
			occupancy_hit_logit = map.occupancyLogit(occupancy_hit);
		}

		auto insert_f = [&](TreeIndex const& node, std::size_t cloud_index) {
			if constexpr (Map::mapType(MapType::OCCUPANCY)) {
				map.occupancyUpdateLogit(node, occupancy_hit_logit, propagate);
			}

			if constexpr (Map::mapType(MapType::COLOR) /* TODO: && is_color_v<PointCloud> */) {
				// map.colorUpdate(node, get<Color>(cloud)[cloud_index], 0.5f, propagate);
				map.colorSet(node, get<Color>(cloud)[cloud_index], propagate);
			}

			if constexpr (Map::mapType(MapType::DISTANCE)) {
				// TODO: Add check if occupancy over a threshold
				map.distanceUpdate(node, get<0>(cloud)[cloud_index], 1.0f, propagate);
			}
		};

		if constexpr (execution::is_seq_v<ExecutionPolicy>) {
			for (std::size_t i{}; nodes.size() > i; ++i) {
				insert_f(nodes[i], i);
			}
		} else if constexpr (execution::is_tbb_v<ExecutionPolicy>) {
			auto fun = [&](std::size_t cloud_index) {
				auto node = nodes[cloud_index];

				// This chick wants to rule the block (node.pos being the block)
				std::lock_guard lock(chickens[node.pos % chickens.size()]);

				insert_f(node, cloud_index);
			};

			IndexIterator<std::size_t> it(0, nodes.size());
			std::for_each(UFO_TBB_PAR it.begin(), it.end(), fun);
		} else if constexpr (execution::is_omp_v<ExecutionPolicy>) {
#pragma omp parallel for
			for (std::size_t i = 0; nodes.size() > i; ++i) {
				auto node = nodes[i];

				// This chick wants to rule the block (node.pos being the block)
				std::lock_guard lock(chickens[node.pos % chickens.size()]);

				insert_f(nodes[i], i);
			}
		} else {
			// TODO: Error
		}

		// TODO: Implement

		time += time_auto_inc;
	}

	template <
	    class ExecutionPolicy, class Map, std::size_t Dim, class T, class... Rest,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void insertPoints(ExecutionPolicy&& policy, Map& map, PointCloud<Dim, T, Rest...> cloud,
	                  Transform<Dim, T> const& frame_origin, bool propagate = true) const
	{
		transformInPlace(policy, frame_origin, get<0>(cloud));
		insertPoints(map, cloud, propagate);
	}

	template <
	    class ExecutionPolicy, class Map, std::size_t Dim, class T, class... Rest,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void insertRays(ExecutionPolicy&& policy, Map& map,
	                PointCloud<Dim, T, Rest...> const& cloud,
	                Vec<Dim, T> const& sensor_origin, bool propagate = true) const
	{
		auto hits_insert_f = std::async(std::launch::async, [&]() {
			if (T(0) >= min_distance && std::numeric_limits<T>::max() <= max_distance) {
				insertPoints(policy, map, cloud, propagate);
			} else {
				auto f_cloud =
				    filterDistance(policy, cloud, sensor_origin, min_distance, max_distance);
				insertPoints(policy, map, f_cloud, propagate);
			}
		});

		if (!counted) {
			auto misses = rayCast(policy, map, get<0>(cloud), sensor_origin);

			hits_insert_f.wait();

			auto nodes =
			    propagate ? map.create(policy, misses) : map.modifiedSet(policy, misses);

			// TODO: Implement different execution policies

			// TODO: What function should be used here?
			logit_t occupancy_miss_logit = map.occupancyLogit(occupancy_miss);

			// TODO: Need to add the chickens to the below
			std::for_each(UFO_TBB_PAR nodes.begin(), nodes.end(),
			              [&map, occupancy_miss_logit, propagate](auto const& node) {
				              map.occupancyUpdateLogit(node, occupancy_miss_logit, propagate);

				              // TODO: Add call to resetIf for distance
			              });
		} else {
			auto [misses, count] = rayCastCounted(policy, map, get<0>(cloud), sensor_origin);

			hits_insert_f.wait();

			auto nodes =
			    propagate ? map.create(policy, misses) : map.modifiedSet(policy, misses);

			// TODO: Implement different execution policies

			// TODO: What function should be used here?
			logit_t occupancy_miss_logit = map.occupancyLogit(occupancy_miss);

			// TODO: Need to add the chickens to the below
			IndexIterator<std::size_t> it(0, nodes.size());
			std::for_each(UFO_TBB_PAR it.begin(), it.end(), [&](std::size_t pos) {
				auto node = nodes[pos];
				auto c    = count[pos];
				map.occupancyUpdateLogit(node, c * occupancy_miss_logit, propagate);

				// TODO: Add call to resetIf for distance
			});
		}

		// TODO: Implement
	}

	template <
	    class ExecutionPolicy, class Map, std::size_t Dim, class T, class... Rest,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void insertRays(ExecutionPolicy&& policy, Map& map, PointCloud<Dim, T, Rest...> cloud,
	                Vec<Dim, T> const& sensor_origin, Transform<Dim, T> const& frame_origin,
	                bool propagate = true) const
	{
		transformInPlace(policy, frame_origin, get<0>(cloud));
		insertRays(std::forward<ExecutionPolicy>(policy), map, cloud, sensor_origin,
		           propagate);
	}

 private:
	template <
	    class ExecutionPolicy, class Map, std::size_t Dim, class T, class... Rest,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	[[nodiscard]] std::vector<TreeCode<Dim>> rayCast(ExecutionPolicy&&               policy,
	                                                 Map const&                      map,
	                                                 std::vector<Vec<Dim, T>> const& hits,
	                                                 Vec<Dim, T> const& sensor_origin) const
	{
		std::vector<TreeCode<Dim>> misses;

		float const grid_size    = map.length(miss_depth);
		auto const  k_origin     = map.key(TreeCoord<Dim, T>(sensor_origin, miss_depth));
		auto const  voxel_border = map.center(k_origin) - sensor_origin;

		if constexpr (2 == Dim) {
			// TODO: Implemment
		} else if constexpr (3 == Dim) {
			// Add origin
			auto       origin_code                    = map.code(k_origin);
			auto const origin_grid_key                = grids.key(origin_code);
			auto& [origin_miss_grid, origin_hit_grid] = grids[origin_grid_key];
			origin_miss_grid.set(origin_code);

			if constexpr (execution::is_seq_v<ExecutionPolicy>) {
				// TODO: Implement
			} else if constexpr (execution::is_tbb_v<ExecutionPolicy>) {
				std::for_each(UFO_TBB_PAR hits.begin(), hits.end(), [&](auto hit) {
					thread_local auto  grid_key  = origin_grid_key;
					thread_local auto* miss_grid = &origin_miss_grid;
					thread_local auto* hit_grid  = &origin_hit_grid;

					auto code = map.code(TreeCoord<Dim, float>(hit, miss_depth));
					if (auto cur_grid_key = grids.key(code); grid_key != cur_grid_key) {
						grid_key       = cur_grid_key;
						auto& [mg, hg] = grids[grid_key];
						miss_grid      = &mg;
						hit_grid       = &hg;
					}

					auto first_hit = !hit_grid->set(code);

					if (DownSamplingMethod::NONE != sample_method && !first_hit) {
						return;
					}

					if (DownSamplingMethod::CENTER == sample_method) {
						hit = map.center(code);
					}

					// TODO: Start from min_distance

					auto dir      = hit - sensor_origin;
					auto distance = norm(dir);
					dir /= distance;

					// TODO: Filter on distance

					distance = std::clamp(distance + distance_offset, 0.0f, max_distance);

					auto step = sign(dir);

					auto t_max   = (voxel_border + cast<float>(step) * grid_size / 2.0f) / dir;
					auto t_delta = grid_size / abs(dir);
					// FIXME: Is this correct? Should it be zero if all zero?
					auto steps = max(Vec<Dim, float>(0.0f), ceil((distance - t_max) / t_delta));

					for (std::size_t i{}; Dim > i; ++i) {
						t_max[i]   = 0 != step[i] ? t_max[i] : std::numeric_limits<float>::max();
						t_delta[i] = 0 != step[i] ? t_delta[i] : std::numeric_limits<float>::max();
						steps[i]   = 0 != step[i] ? steps[i] : 0;
					}

					auto k_cur = k_origin;
					grid_key   = origin_grid_key;
					miss_grid  = &origin_miss_grid;
					hit_grid   = &origin_hit_grid;

					std::size_t total_steps = sum(steps);
					total_steps -= (free_hits || 0 == total_steps) ? 0 : 1;
					while (total_steps--) {
						auto const advance_dim = minIndex(t_max);
						k_cur[advance_dim] += step[advance_dim];
						t_max[advance_dim] += t_delta[advance_dim];

						auto cur = map.code(k_cur);

						if (auto cur_grid_key = grids.key(cur); grid_key != cur_grid_key) {
							grid_key       = cur_grid_key;
							auto& [mg, hg] = grids[grid_key];
							miss_grid      = &mg;
							hit_grid       = &hg;
						}

						miss_grid->set(cur);
					}
				});

				Spinlock mutex;

				std::for_each(UFO_TBB_PAR grids.begin(), grids.end(), [&](auto& x) {
					auto c                      = x.first;
					auto& [miss_grid, hit_grid] = x.second;

					auto m_it = miss_grid.begin();
					// TODO: Implement this better
					auto h_it = free_hits ? grids.end()->second.second.begin() : hit_grid.begin();

					thread_local decltype(misses) local_misses;
					local_misses.reserve(1'000'000);

					for (std::uint64_t i{}; miss_grid.end() > m_it;
					     m_it += 8, h_it += 8, i += 512) {
						std::array<std::uint64_t, 8> v{
						    (m_it[0]) & (~(h_it[0])), (m_it[1]) & (~(h_it[1])),
						    (m_it[2]) & (~(h_it[2])), (m_it[3]) & (~(h_it[3])),
						    (m_it[4]) & (~(h_it[4])), (m_it[5]) & (~(h_it[5])),
						    (m_it[6]) & (~(h_it[6])), (m_it[7]) & (~(h_it[7]))};

						if (std::all_of(v.begin(), v.end(), [](auto x) {
							    return std::numeric_limits<std::uint64_t>::max() == x;
						    })) {
							local_misses.emplace_back(miss_grid.code(c, i, miss_depth, 3));
							continue;
						}

						for (std::uint64_t j{}; 8 > j; ++j) {
							if (std::numeric_limits<std::uint64_t>::max() == v[j]) {
								local_misses.emplace_back(miss_grid.code(c, i + j * 64, miss_depth, 2));
								continue;
							} else if (std::uint64_t(0) == v[j]) {
								continue;
							}

							for (std::uint64_t k{}; 64 > k; k += 8) {
								std::uint64_t mask = 0xFF << k;
								if (mask == (mask & v[j])) {
									local_misses.emplace_back(
									    miss_grid.code(c, i + j * 64 + k, miss_depth, 1));
									continue;
								}

								for (std::uint64_t m{}; 8 > m; ++m) {
									std::uint64_t mask = 0x1 << (k + m);
									if (mask == (mask & v[j])) {
										local_misses.emplace_back(
										    miss_grid.code(c, i + j * 64 + k + m, miss_depth, 0));
									}
								}
							}
						}
					}

					{
						std::lock_guard lock(mutex);
						misses.insert(misses.end(), local_misses.begin(), local_misses.end());
					}

					local_misses.clear();

					miss_grid.clear();
					hit_grid.clear();
				});

				grids.clear();

				// std::cout << misses.size() << std::endl;  // 80813
			} else if constexpr (execution::is_omp_v<ExecutionPolicy>) {
				// TODO: Implement
			} else {
				// TODO: Error
			}
		}

		return misses;
	}

	template <
	    class ExecutionPolicy, class Map, std::size_t Dim, class T, class... Rest,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	[[nodiscard]] std::pair<std::vector<TreeCode<Dim>>, std::vector<int>> rayCastCounted(
	    ExecutionPolicy&& policy, Map const& map, std::vector<Vec<Dim, T>> const& hits,
	    Vec<Dim, T> const& sensor_origin) const
	{
		std::vector<TreeCode<Dim>> misses;
		std::vector<int>           count;

		float const grid_size    = map.length(miss_depth);
		auto const  k_origin     = map.key(TreeCoord<Dim, T>(sensor_origin, miss_depth));
		auto const  voxel_border = map.center(k_origin) - sensor_origin;

		if constexpr (2 == Dim) {
			// TODO: Implemment
		} else if constexpr (3 == Dim) {
			// Add origin
			auto       origin_code                    = map.code(k_origin);
			auto const origin_grid_key                = count_grids.key(origin_code);
			auto& [origin_miss_grid, origin_hit_grid] = count_grids[origin_grid_key];
			origin_miss_grid.inc(origin_code);

			if constexpr (execution::is_seq_v<ExecutionPolicy>) {
				// TODO: Implement
			} else if constexpr (execution::is_tbb_v<ExecutionPolicy>) {
				std::for_each(UFO_TBB_PAR hits.begin(), hits.end(), [&](auto hit) {
					thread_local auto  grid_key  = origin_grid_key;
					thread_local auto* miss_grid = &origin_miss_grid;
					thread_local auto* hit_grid  = &origin_hit_grid;

					auto code = map.code(TreeCoord<Dim, float>(hit, miss_depth));
					if (auto cur_grid_key = count_grids.key(code); grid_key != cur_grid_key) {
						grid_key       = cur_grid_key;
						auto& [mg, hg] = count_grids[grid_key];
						miss_grid      = &mg;
						hit_grid       = &hg;
					}

					auto first_hit = !hit_grid->set(code);

					if (DownSamplingMethod::NONE != sample_method && !first_hit) {
						return;
					}

					if (DownSamplingMethod::CENTER == sample_method) {
						hit = map.center(code);
					}

					auto dir      = hit - sensor_origin;
					auto distance = norm(dir);
					dir /= distance;

					distance = std::clamp(distance + distance_offset, 0.0f, max_distance);

					auto step = sign(dir);

					auto t_max   = (voxel_border + cast<float>(step) * grid_size / 2.0f) / dir;
					auto t_delta = grid_size / abs(dir);
					// FIXME: Is this correct? Should it be zero if all zero?
					auto steps = max(Vec<Dim, float>(0.0f), ceil((distance - t_max) / t_delta));

					for (std::size_t i{}; Dim > i; ++i) {
						t_max[i]   = 0 != step[i] ? t_max[i] : std::numeric_limits<float>::max();
						t_delta[i] = 0 != step[i] ? t_delta[i] : std::numeric_limits<float>::max();
						steps[i]   = 0 != step[i] ? steps[i] : 0;
					}

					auto k_cur = k_origin;
					grid_key   = origin_grid_key;
					miss_grid  = &origin_miss_grid;
					hit_grid   = &origin_hit_grid;

					std::size_t total_steps = sum(steps);
					total_steps -= (free_hits || 0 == total_steps) ? 0 : 1;
					while (total_steps--) {
						auto const advance_dim = minIndex(t_max);
						k_cur[advance_dim] += step[advance_dim];
						t_max[advance_dim] += t_delta[advance_dim];

						auto cur = map.code(k_cur);

						if (auto cur_grid_key = count_grids.key(cur); grid_key != cur_grid_key) {
							grid_key       = cur_grid_key;
							auto& [mg, hg] = count_grids[grid_key];
							miss_grid      = &mg;
							hit_grid       = &hg;
						}

						miss_grid->inc(cur);
					}
				});

				Spinlock mutex;

				std::for_each(UFO_TBB_PAR count_grids.begin(), count_grids.end(), [&](auto& x) {
					auto c                      = x.first;
					auto& [miss_grid, hit_grid] = x.second;

					auto m_it = miss_grid.begin();
					// TODO: Implement this better
					auto h_it =
					    free_hits ? count_grids.end()->second.second.begin() : hit_grid.begin();

					thread_local decltype(misses) local_misses;
					thread_local decltype(count)  local_count;
					local_misses.reserve(1'000'000);
					local_count.reserve(1'000'000);

					for (std::uint64_t i{}; miss_grid.end() > m_it; m_it += 64, ++h_it, i += 64) {
						std::uint64_t hit = *h_it;
						for (std::uint64_t j{}; 64 > j; ++j) {
							int v = m_it[j];
							if (0 == v || (hit & (std::uint64_t(1) << j))) {
								continue;
							}

							local_misses.emplace_back(miss_grid.code(c, i + j, miss_depth, 0));
							local_count.emplace_back(v);
						}
					}

					{
						std::lock_guard lock(mutex);
						misses.insert(misses.end(), local_misses.begin(), local_misses.end());
						count.insert(count.end(), local_count.begin(), local_count.end());
					}

					local_misses.clear();
					local_count.clear();

					miss_grid.clear();
					hit_grid.clear();
				});

				// std::cout << count_grids.size() << std::endl;

				count_grids.clear();

				// std::cout << misses.size() << std::endl;  // 80813
			} else if constexpr (execution::is_omp_v<ExecutionPolicy>) {
				// TODO: Implement
			} else {
				// TODO: Error
			}
		}

		return std::pair(std::move(misses), std::move(count));
	}
};
}  // namespace ufo

#endif  // UFO_MAP_INTEGRATOR_INTEGRATOR_HPP
