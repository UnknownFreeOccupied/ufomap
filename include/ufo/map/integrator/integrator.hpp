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
#include <ufo/container/tree/coord.hpp>
#include <ufo/container/tree/index.hpp>
#include <ufo/core/label.hpp>
#include <ufo/execution/algorithm.hpp>
#include <ufo/execution/execution.hpp>
#include <ufo/map/color/map.hpp>
#include <ufo/map/occupancy/map.hpp>
// #include <ufo/map/time/map.hpp>
#include <ufo/map/integrator/count_sampling_method.hpp>
#include <ufo/map/integrator/detail/bool_grid.hpp>
#include <ufo/map/integrator/detail/count_grid.hpp>
#include <ufo/map/integrator/detail/grid_map.hpp>
#include <ufo/map/type.hpp>
#include <ufo/map/void_region/map.hpp>
#include <ufo/math/vec.hpp>
#include <ufo/utility/spinlock.hpp>
#include <ufo/utility/type_traits.hpp>

// STL
#include <algorithm>
#include <cstddef>
#include <future>
#include <type_traits>
#include <vector>

namespace ufo
{
enum class DownSamplingMethod { NONE, FIRST, CENTER };

template <std::size_t Dim, std::size_t GridLevels = 5>
class Integrator
{
 public:
	//
	// Tags
	//
	using occupancy_t = float;
	using logit_t     = OccupancyElement::logit_t;
	using depth_t     = unsigned;

	depth_t hit_depth  = 0;
	depth_t miss_depth = 0;

	// Min range to integrate
	double min_distance = 0.0;
	// Max range to integrate, negative value is infinity range
	double max_distance = std::numeric_limits<double>::infinity();
	// To extend or shorten the rays
	double distance_offset = 0.0;

	// Occupancy hit [0, 1]
	occupancy_t occupancy_hit = 0.75;
	// Occupancy miss [0, 1]
	// occupancy_t occupancy_miss = 0.45;
	occupancy_t occupancy_miss = 0.499;

	unsigned void_region_distance = 2;

	DownSamplingMethod sample_method = DownSamplingMethod::NONE;

	// TODO: Should this be here?
	bool free_hits = false;

	MapType integrate_types = MapType::ALL;

	bool verbose = false;

	CountSamplingMethod count_sample_method = CountSamplingMethod::NONE;

	// A single hit in a void region will set the occupancy to max
	bool void_region_instant_max_occupancy = true;
	// A single miss in a void region will set the occupancy to min
	bool void_region_instant_min_occupancy = true;

 public:
	template <class Map, class T, class... Rest>
	void operator()(Map& map, PointCloud<Dim, T, Rest...> cloud,
	                Transform<Dim, T> const& transform = {}, bool propagate = true) const
	{
		// TODO: Implement
	}

	template <
	    class ExecutionPolicy, class Map, class T, class... Rest,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void operator()(ExecutionPolicy&& policy, Map& map, PointCloud<Dim, T, Rest...> cloud,
	                Transform<Dim, T> const& transform = {}, bool propagate = true) const
	{
		// TODO: Implement
	}

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                                       Create                                        |
	|                                                                                     |
	**************************************************************************************/

	template <class Map>
	void create(Map& map, std::vector<TreeCode<Dim>> const& codes,
	            std::vector<TreeIndex>& nodes) const
	{
		nodes.resize(codes.size());
		map.create(codes, nodes.begin());
	}

	template <
	    class ExecutionPolicy, class Map,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void create(ExecutionPolicy&& policy, Map& map, std::vector<TreeCode<Dim>> const& codes,
	            std::vector<TreeIndex>& nodes) const
	{
		nodes.resize(codes.size());
		map.create(std::forward<ExecutionPolicy>(policy), codes, nodes.begin());
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Clear                                        |
	|                                                                                     |
	**************************************************************************************/

	void clear() const
	{
		hits_grid_.clear();
		void_region_hits_grid_.clear();
		misses_grid_.clear();
		count_grid_.clear();
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Hits                                         |
	|                                                                                     |
	**************************************************************************************/

	template <class Map, class T, class... Rest>
	void hits(Map& map, PointCloud<Dim, T, Rest...> cloud,
	          Transform<Dim, T> const& transform, bool async = false) const
	{
		filterDistanceInPlace(cloud, Vec<Dim, T>{}, static_cast<T>(min_distance),
		                      static_cast<T>(max_distance), true, true);
		transformInPlace(transform, get<0>(cloud));

		hit_codes_.resize(cloud.size());
		std::transform(
		    get<0>(cloud).begin(), get<0>(cloud).end(), hit_codes_.begin(),
		    [&map, depth = hit_depth](auto p) { return map.code(TreeCoord<Dim>(p, depth)); });

		// auto grid_f = std::async(
		//     async ? std::launch::async : std::launch::deferred,
		//     // TODO: Should origin be - or +?
		//     [this, &map, &points = get<0>(cloud), origin = transform(Vec<Dim, float>{})]()
		//     {
		// 	    auto const md = miss_depth;
		// 	    for (auto c : hit_codes_) {
		// 		    c.setDepth(md);
		// 		    hits_grid_[c].set(c);
		// 	    }

		// 	    auto grid_size = cast<float>(map.length(md));
		// 	    for (auto const& p : points) {
		// 		    auto  dir      = p - origin;
		// 		    float distance = norm(dir);
		// 		    dir /= distance;

		// 		    float error = 1.0f;  // TODO: Have as parameter (should depend on distance?)

		// 		    auto start = origin + (std::max(0.0f, distance - error) * dir);
		// 		    auto step  = sign(dir);
		// 		    distance   = error + std::min(error, std::abs(distance - error));

		// 		    auto       k_cur        = map.key(TreeCoord<Dim, T>(start, md));
		// 		    auto const voxel_border = map.center(k_cur) - start;

		// 		    auto t_max   = (voxel_border + cast<float>(step) * grid_size / 2.0f) / dir;
		// 		    auto t_delta = grid_size / abs(dir);
		// 		    // FIXME: Is this correct? Should it be zero if all zero?
		// 		    auto steps = max(Vec<Dim, float>(0.0f), ceil((distance - t_max) / t_delta));

		// 		    // std::cout << origin << " -> " << p << '\n';
		// 		    // std::cout << dir << '\n';
		// 		    // std::cout << start << '\n';
		// 		    // std::cout << step << '\n';
		// 		    // std::cout << distance << '\n';
		// 		    // std::cout << k_cur << '\n';
		// 		    // std::cout << voxel_border << '\n';
		// 		    // std::cout << t_max << '\n';
		// 		    // std::cout << t_delta << '\n';
		// 		    // std::cout << steps << '\n';

		// 		    for (std::size_t i{}; Dim > i; ++i) {
		// 			    t_max[i]   = 0 != step[i] ? t_max[i] : std::numeric_limits<float>::max();
		// 			    t_delta[i] = 0 != step[i] ? t_delta[i] :
		// std::numeric_limits<float>::max(); 			    steps[i]   = 0 != step[i] ? steps[i] :
		// 0;
		// 		    }

		// 		    auto cur = map.code(k_cur);
		// 		    void_region_hits_grid_[cur].set(cur);

		// 		    std::size_t total_steps = sum(steps);
		// 		    // std::cout << total_steps << '\n';
		// 		    while (total_steps--) {
		// 			    auto const advance_dim = minIndex(t_max);
		// 			    k_cur[advance_dim] += step[advance_dim];
		// 			    t_max[advance_dim] += t_delta[advance_dim];

		// 			    cur = map.code(k_cur);
		// 			    void_region_hits_grid_[cur].set(cur);
		// 		    }
		// 	    }
		//     });

		auto grid_f = std::async(
		    async ? std::launch::async : std::launch::deferred,
		    // TODO: Should origin be - or +?
		    [this, &map, &points = get<0>(cloud), origin = transform(Vec<Dim, float>{})]() {
			    // auto const md = miss_depth;
			    // for (auto c : hit_codes_) {
			    //   c.setDepth(md);
			    //   hits_grid_[c].set(c);
			    // }

			    // for (auto const& p : points) {
			    //   // TODO: Have as parameter (should depend on distance?)
			    //   auto error = cast<T>(map.length(md));
			    //   auto min   = map.key(TreeCoord(p - error, md));
			    //   auto max   = map.key(TreeCoord(p + error, md));

			    //   auto k = min;
			    //   if constexpr (2 == Dim) {
			    //     for (k.y = min.y; max.y >= k.y; ++k.y) {
			    // 	    for (k.x = min.x; max.x >= k.x; ++k.x) {
			    // 		    auto c = map.code(k);
			    // 		    void_region_hits_grid_[c].set(c);
			    // 	    }
			    //     }
			    //   } else {
			    //     for (k.z = min.z; max.z >= k.z; ++k.z) {
			    // 	    for (k.y = min.y; max.y >= k.y; ++k.y) {
			    // 		    for (k.x = min.x; max.x >= k.x; ++k.x) {
			    // 			    auto c = map.code(k);
			    // 			    void_region_hits_grid_[c].set(c);
			    // 		    }
			    // 	    }
			    //     }
			    //   }
			    // }
		    });

		create(map, hit_codes_, hit_nodes_);
		insertHits(map, hit_nodes_, cloud);

		grid_f.wait();
	}

	template <
	    class ExecutionPolicy, class Map, class T, class... Rest,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void hits(ExecutionPolicy&& policy, Map& map, PointCloud<Dim, T, Rest...> cloud,
	          Transform<Dim, T> const& transform) const
	{
		// TODO: Change `T(0.0001)` to a very small value
		filterDistanceInPlace(cloud, Vec<Dim, T>{},
		                      std::max(T(0.0001), static_cast<T>(min_distance)),
		                      static_cast<T>(max_distance), true);
		transformInPlace(policy, transform, get<0>(cloud));

		hit_codes_.resize(cloud.size());
		ufo::transform(
		    policy, get<0>(cloud).begin(), get<0>(cloud).end(), hit_codes_.begin(),
		    [&map, d = hit_depth](auto const& p) { return map.code(TreeCoord(p, d)); });

		create(policy, map, hit_codes_, hit_nodes_);
		insertHits(policy, map, hit_nodes_, cloud);

		{
			auto& points = get<0>(cloud);

			std::atomic_size_t size = 0;
			std::mutex         mutex;

			auto const md = miss_depth;
			for_each(policy, points.begin(), points.end(), [&](auto const& p) {
				thread_local std::size_t grid_idx;
				auto const               id = std::this_thread::get_id();
				if (size.load(std::memory_order_relaxed) <= grid_idx ||
				    id != std::get<0>(local_hits_grid_[grid_idx])) {
					std::scoped_lock lock(mutex);
					grid_idx = size;
					if (local_hits_grid_.size() <= grid_idx) {
						local_hits_grid_.emplace_back();
						assert(local_hits_grid_.size() >= grid_idx);
					}
					std::get<0>(local_hits_grid_[grid_idx]) = id;
					++size;
				}

				auto& hits_grid             = std::get<1>(local_hits_grid_[grid_idx]);
				auto& void_region_hits_grid = std::get<2>(local_hits_grid_[grid_idx]);

				auto c = map.code(TreeCoord(p, md));
				// if (hits_grid[c].test(c)) {
				// 	return;
				// }
				hits_grid[c].set(c);

				// TODO: Have as parameter (should depend on distance?)
				auto error = T(1) * cast<T>(map.length(md));
				auto min   = map.key(TreeCoord(p - error, md));
				auto max   = map.key(TreeCoord(p + error, md));

				auto k = min;
				if constexpr (2 == Dim) {
					for (k.y = min.y; max.y >= k.y; ++k.y) {
						for (k.x = min.x; max.x >= k.x; ++k.x) {
							auto c = map.code(k);
							void_region_hits_grid[c].set(c);
						}
					}
				} else {
					for (k.z = min.z; max.z >= k.z; ++k.z) {
						for (k.y = min.y; max.y >= k.y; ++k.y) {
							for (k.x = min.x; max.x >= k.x; ++k.x) {
								auto c = map.code(k);
								void_region_hits_grid[c].set(c);
							}
						}
					}
				}
			});

			std::unordered_map<TreeCode<Dim>, unsigned> key_to_index_1;
			std::unordered_map<TreeCode<Dim>, unsigned> key_to_index_2;
			unsigned                                    index_1{};
			unsigned                                    index_2{};
			for (std::size_t i{}, s = size; s > i; ++i) {
				ufo::for_each(std::get<1>(local_hits_grid_[i]).begin(),
				              std::get<1>(local_hits_grid_[i]).end(),
				              [this, &key_to_index_1, &index_1](auto const& e) {
					              if (auto it = key_to_index_1.find(e.first);
					                  key_to_index_1.end() == it) {
						              key_to_index_1[e.first] = index_1++;
						              hits_grid_.addKey(e.first);
					              }
				              });

				ufo::for_each(std::get<2>(local_hits_grid_[i]).begin(),
				              std::get<2>(local_hits_grid_[i]).end(),
				              [this, &key_to_index_2, &index_2](auto const& e) {
					              if (auto it = key_to_index_2.find(e.first);
					                  key_to_index_2.end() == it) {
						              key_to_index_2[e.first] = index_2++;
						              void_region_hits_grid_.addKey(e.first);
					              }
				              });

				ufo::for_each(policy, std::get<1>(local_hits_grid_[i]).begin(),
				              std::get<1>(local_hits_grid_[i]).end(),
				              [this, &key_to_index_1](auto const& e) {
					              auto& v0 = hits_grid_[key_to_index_1[e.first]];
					              ufo::transform(v0.begin(), v0.end(), e.second.begin(), v0.begin(),
					                             [](auto a, auto b) { return a | b; });
				              });

				ufo::for_each(policy, std::get<2>(local_hits_grid_[i]).begin(),
				              std::get<2>(local_hits_grid_[i]).end(),
				              [this, &key_to_index_2](auto const& e) {
					              auto& v0 = void_region_hits_grid_[key_to_index_2[e.first]];
					              ufo::transform(v0.begin(), v0.end(), e.second.begin(), v0.begin(),
					                             [](auto a, auto b) { return a | b; });
				              });

				std::get<0>(local_hits_grid_[i]) = {};
				std::get<1>(local_hits_grid_[i]).clear();
				std::get<2>(local_hits_grid_[i]).clear();
			}
		}
	}

	template <class Map, class Point>
	void insertHit(Map& map, TreeIndex const& node, Point const& data,
	               logit_t occupancy_logit) const
	{
		if constexpr (Map::hasMapTypes(MapType::OCCUPANCY)) {
			if constexpr (Map::hasMapTypes(MapType::VOID_REGION)) {
				if (void_region_instant_max_occupancy && map.voidRegion(node)) {
					map.occupancySetLogit(node, map.occupancyMaxLogit(), false);
				} else {
					map.occupancyUpdateLogit(node, occupancy_logit, false);
				}
			} else {
				map.occupancyUpdateLogit(node, occupancy_logit, false);
			}
		}

		if constexpr (Map::hasMapTypes(MapType::OCCUPANCY)) {
			map.occupancyUpdateLogit(node, occupancy_logit, false);
		}

		if constexpr (Map::hasMapTypes(MapType::COLOR) && std::is_base_of_v<Color, Point>) {
			// TODO: Make correct
			map.colorSet(node, static_cast<Color>(data), false);
		}

		// TODO: Add more map types
	}

	template <class Map, class T, class... Rest>
	void insertHits(Map& map, std::vector<TreeIndex> const& nodes,
	                PointCloud<Dim, T, Rest...> const& cloud) const
	{
		logit_t occupancy_logit;
		if constexpr (Map::hasMapTypes(MapType::OCCUPANCY)) {
			// TODO: What function should be used here?
			occupancy_logit = map.occupancyLogit(occupancy_hit);
		}

		for (std::size_t i{}; nodes.size() > i; ++i) {
			insertHit(map, nodes[i], cloud[i], occupancy_logit);
		}
	}

	template <
	    class ExecutionPolicy, class Map, class T, class... Rest,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void insertHits(ExecutionPolicy&& policy, Map& map, std::vector<TreeIndex> const& nodes,
	                PointCloud<Dim, T, Rest...> const& cloud) const
	{
		logit_t occupancy_logit;
		if constexpr (Map::hasMapTypes(MapType::OCCUPANCY)) {
			// TODO: What function should be used here?
			occupancy_logit = map.occupancyLogit(occupancy_hit);
		}

		for_each(std::forward<ExecutionPolicy>(policy), std::size_t(0), nodes.size(),
		         [&](std::size_t i) {
			         auto node = nodes[i];

			         // This chick wants to rule the block (node.pos being the block)
			         std::lock_guard lock(map.chicken(node.pos));

			         insertHit(map, node, cloud[i], occupancy_logit);
		         });
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Misses                                        |
	|                                                                                     |
	**************************************************************************************/

	template <class Map>
	void misses(Map& map) const
	{
		// TODO: Get `miss_codes_` and `void_region_codes_`

		clear();

		create(map, miss_codes_, miss_nodes_);
		create(map, void_region_codes_, void_region_nodes_);

		insertMisses(map, miss_nodes_, miss_counts_);
		insertVoidRegions(map, void_region_nodes_);
		// TODO: What should this be?
		// insertVoidRegionsSecondary( map, miss_nodes_);
	}

	template <
	    class ExecutionPolicy, class Map,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void misses(ExecutionPolicy&& policy, Map& map) const
	{
		miss_codes_.clear();
		miss_counts_.clear();
		void_region_.clear();
		assert(misses_grid_.size() == count_grid_.size());

		BoolGrid<Dim, GridLevels> empty;
		for (auto const& [k, v] : misses_grid_) {
			auto last_it  = v.end();
			auto miss_it  = v.begin();
			auto count_it = count_grid_[k].begin();
			auto hit_it   = hits_grid_.contains(k) ? hits_grid_[k].begin() : empty.begin();
			auto vr_hit_it =
			    void_region_hits_grid_.contains(k) ? void_region_hits_grid_[k].begin() : hit_it;
			for (std::size_t i{}; last_it > miss_it;
			     i += 64u, ++miss_it, ++count_it, ++hit_it, ++vr_hit_it) {
				std::uint64_t miss    = *miss_it & ~(*hit_it);
				std::uint64_t vr_miss = *miss_it & ~(*vr_hit_it);
				auto          count   = *count_it;

				if (0u == miss) {
					continue;
				}

				if constexpr (2 == Dim) {
					// TODO: Implement
				} else if constexpr (3 == Dim) {
					if (std::numeric_limits<std::uint64_t>::max() == miss &&
					    std::numeric_limits<std::uint64_t>::max() == vr_miss) {
						miss_codes_.push_back(
						    v.code(k, i, miss_depth, 2u));  // TODO: What should `depth` be?
						miss_counts_.push_back(count);
						void_region_.push_back(true);
						continue;
					} else if (std::numeric_limits<std::uint64_t>::max() == miss && 0u == vr_miss) {
						miss_codes_.push_back(
						    v.code(k, i, miss_depth, 2u));  // TODO: What should `depth` be?
						miss_counts_.push_back(count);
						void_region_.push_back(false);
						continue;
					}

					for (std::size_t j{}; 64u > j; j += 8) {
						std::uint64_t mask = std::uint64_t(0xFFu) << j;
						if (mask == (mask & miss) && mask == (mask & vr_miss)) {
							miss_codes_.push_back(
							    v.code(k, i | j, miss_depth, 1u));  // TODO: What should `depth` be?
							miss_counts_.push_back(count);
							void_region_.push_back(true);
							continue;
						} else if (mask == (mask & miss) && 0u == (mask & vr_miss)) {
							miss_codes_.push_back(
							    v.code(k, i | j, miss_depth, 1u));  // TODO: What should `depth` be?
							miss_counts_.push_back(count);
							void_region_.push_back(false);
							continue;
						}

						for (std::size_t m{}; 8u > m; ++m) {
							mask = std::uint64_t(0x1u) << (j + m);
							if (mask == (mask & miss)) {
								miss_codes_.push_back(v.code(k, i | j | m, miss_depth,
								                             0u));  // TODO: What should `depth` be?
								miss_counts_.push_back(count);
								void_region_.push_back(mask == (mask & vr_miss));
							}
						}
					}
				} else {
					// TODO: Error
				}
			}
		}

		std::size_t num_misses = void_region_.size();
		std::size_t num_void_regions{};
		for (bool v : void_region_) {
			num_void_regions += v ? 1u : 0u;
		}

		std::cout << num_misses << " vs " << num_void_regions << '\n';

		// TODO: Get `miss_codes_` and `void_region_codes_`

		auto clear_f = std::async(std::launch::async, [this]() { clear(); });

		create(policy, map, miss_codes_, miss_nodes_);

		auto misses_f = std::async(std::launch::async, [this, policy, &map]() {
			insertMisses(policy, map, miss_nodes_, miss_counts_, void_region_);
		});

		misses_f.wait();
		// TODO: What should this be?
		// insertVoidRegionsSecondary(policy, map, miss_nodes_);

		clear_f.wait();
	}

	template <class Map>
	void insertMiss(Map& map, TreeIndex node, unsigned count, logit_t occupancy_logit,
	                bool void_region) const
	{
		if constexpr (Map::hasMapTypes(MapType::VOID_REGION)) {
			if (void_region) {
				map.voidRegionSet(node, true, false);
			}
		}

		if constexpr (Map::hasMapTypes(MapType::OCCUPANCY)) {
			// TODO: Is this good?
			if constexpr (Map::hasMapTypes(MapType::VOID_REGION)) {
				if (void_region_instant_min_occupancy && map.voidRegion(node)) {
					map.occupancySetLogit(node, map.occupancyMinLogit(), false);
				} else {
					// TODO: Make sure `info.count() * occupancy_logit` does not overflow
					map.occupancyUpdateLogit(node, count * occupancy_logit, false);
				}
			} else {
				// TODO: Make sure `info.count() * occupancy_logit` does not overflow
				map.occupancyUpdateLogit(node, count * occupancy_logit, false);
			}
		}

		// TODO: Add more map types
	}

	template <class Map>
	void insertMisses(Map& map, std::vector<TreeIndex> const& nodes,
	                  std::vector<unsigned> const& counts,
	                  std::vector<bool> const&     void_region) const
	{
		logit_t occupancy_logit;
		if constexpr (Map::hasMapTypes(MapType::OCCUPANCY)) {
			// TODO: What function should be used here?
			occupancy_logit = map.occupancyLogit(occupancy_miss);
		}

		for (std::size_t i{}; nodes.size() > i; ++i) {
			insertMiss(map, nodes[i], counts[i], occupancy_logit, void_region[i]);
		}
	}

	template <
	    class ExecutionPolicy, class Map,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void insertMisses(ExecutionPolicy&& policy, Map& map,
	                  std::vector<TreeIndex> const& nodes,
	                  std::vector<unsigned> const&  counts,
	                  std::vector<bool> const&      void_region) const
	{
		logit_t occupancy_logit;
		if constexpr (Map::hasMapTypes(MapType::OCCUPANCY)) {
			// TODO: What function should be used here?
			occupancy_logit = map.occupancyLogit(occupancy_miss);
		}

		for_each(std::forward<ExecutionPolicy>(policy), std::size_t(0), nodes.size(),
		         [&](std::size_t i) {
			         auto node = nodes[i];

			         // This chick wants to rule the block (node.pos being the block)
			         std::lock_guard lock(map.chicken(node.pos));

			         insertMiss(map, node, counts[i], occupancy_logit, void_region[i]);
		         });
	}

	/**************************************************************************************
	|                                                                                     |
	|                                Secondary void region                                |
	|                                                                                     |
	**************************************************************************************/

	template <class Map>
	void insertVoidRegionSecondary(Map& map, TreeIndex node) const
	{
		if constexpr (Map::hasMapTypes(MapType::VOID_REGION & MapType::OCCUPANCY)) {
			if (map.voidRegion(node) || map.voidRegionSecondary(node) ||
			    map.occupancyMinLogit() != map.occupancyLogit(node)) {
				return;
			}

			// TODO: Implement correctly
			if (map.voidRegionAny(node.pos)) {
				map.voidRegionSecondarySet(node, true, false);
			} else {
				// TODO: Optimize
				auto c  = map.center(node);
				auto hl = cast<float>(map.halfLength(node) + map.halfLength(0));
				for (auto n : map.query(pred::Leaf() && pred::VoidRegion() &&
				                        pred::Intersects(AABB3f(c - hl, c + hl)))) {
					if (n.index != node) {
						map.voidRegionSecondarySet(node, true, false);
						break;
					}
				}
			}
		}
	}

	template <class Map>
	void insertVoidRegionsSecondary(Map& map, std::vector<TreeIndex> const& nodes) const
	{
		if constexpr (!Map::hasMapTypes(MapType::VOID_REGION & MapType::OCCUPANCY)) {
			return;
		} else if (MapType::VOID_REGION != (MapType::VOID_REGION & integrate_types)) {
			return;
		}

		for (std::size_t i{}; nodes.size() > i; ++i) {
			insertVoidRegionSecondary(map, nodes[i]);
		}
	}

	template <
	    class ExecutionPolicy, class Map,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void insertVoidRegionsSecondary(ExecutionPolicy&& policy, Map& map,
	                                std::vector<TreeIndex> const& nodes) const
	{
		if constexpr (!Map::hasMapTypes(MapType::VOID_REGION & MapType::OCCUPANCY)) {
			return;
		} else if (MapType::VOID_REGION != (MapType::VOID_REGION & integrate_types)) {
			return;
		}

		for_each(std::forward<ExecutionPolicy>(policy), std::size_t(0), nodes.size(),
		         [&](std::size_t i) {
			         auto node = nodes[i];

			         // This chick wants to rule the block (node.pos being the block)
			         std::lock_guard lock(map.chicken(node.pos));

			         insertVoidRegionSecondary(map, node);
		         });
	}

 protected:
	mutable __block std::vector<TreeIndex> hit_nodes_;
	mutable __block std::vector<TreeIndex> miss_nodes_;
	mutable std::vector<unsigned>          miss_counts_;
	mutable std::vector<bool>              void_region_;
	mutable __block std::vector<TreeIndex> void_region_nodes_;

	mutable __block std::vector<TreeCode<Dim>> hit_codes_;
	mutable std::vector<TreeCode<Dim>>         miss_codes_;
	mutable std::vector<TreeCode<Dim>>         void_region_codes_;

	mutable __block detail::GridMap<BoolGrid, Dim, GridLevels> hits_grid_;
	mutable __block detail::GridMap<BoolGrid, Dim, GridLevels> void_region_hits_grid_;
	mutable __block detail::GridMap<BoolGrid, Dim, GridLevels> misses_grid_;
	mutable __block detail::GridMap<CountGrid, Dim, GridLevels> count_grid_;

	mutable __block
	    std::deque<std::tuple<std::thread::id, detail::GridMap<BoolGrid, Dim, GridLevels>,
	                          detail::GridMap<BoolGrid, Dim, GridLevels>>>
	        local_hits_grid_;
};
}  // namespace ufo

#endif  // UFO_MAP_INTEGRATOR_INTEGRATOR_HPP