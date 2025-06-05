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
#include <ufo/map/integrator/detail/hit.hpp>
#include <ufo/map/integrator/detail/hit_grid.hpp>
#include <ufo/map/integrator/detail/miss.hpp>
#include <ufo/map/integrator/detail/miss_grid.hpp>
#include <ufo/map/type.hpp>
#include <ufo/map/void_region/map.hpp>
#include <ufo/math/vec.hpp>
#include <ufo/utility/spinlock.hpp>
#include <ufo/utility/type_traits.hpp>

// STL
#include <cstddef>
#include <type_traits>
#include <vector>

namespace ufo
{
enum class DownSamplingMethod { NONE, FIRST, CENTER };

template <std::size_t Dim>
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

	occupancy_t occupancy_hit             = 0.75f;    // [0, 1]
	occupancy_t occupancy_miss            = 0.45f;    // [0, 1]
	logit_t     occupancy_max_clamp_thres = 0.971f;   // [0, 1]
	logit_t     occupancy_min_clamp_thres = 0.1192f;  // [0, 1]

	unsigned void_region_distance = 2;

	DownSamplingMethod sample_method = DownSamplingMethod::NONE;

	// TODO: Should this be here?
	bool free_hits = false;

	bool verbose = false;

	CountSamplingMethod count_sample_method = CountSamplingMethod::NONE;

	// A single hit in a void region will set the occupancy to max
	bool void_region_instant_max_occupancy = true;
	// A single miss in a void region will set the occupancy to min
	bool void_region_instant_min_occupancy = true;

	bool propagate      = false;
	bool prune          = true;
	bool reset_modified = true;

 public:
	template <class Map, class T, class... Rest>
	void operator()(Map& map, PointCloud<Dim, T, Rest...> cloud,
	                Transform<Dim, T> const& transform = {}) const
	{
		// TODO: Implement
	}

	template <
	    class ExecutionPolicy, class Map, class T, class... Rest,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void operator()(ExecutionPolicy&& policy, Map& map, PointCloud<Dim, T, Rest...> cloud,
	                Transform<Dim, T> const& transform = {}) const
	{
		// TODO: Implement
	}

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                                        Hits                                         |
	|                                                                                     |
	**************************************************************************************/

	template <class Map, class Data>
	void insertHit(Map& map, TreeIndex const& node, Data const& data, logit_t occupancy,
	               logit_t occupancy_min, logit_t occupancy_max) const
	{
		if constexpr (Map::hasMapTypes(MapType::OCCUPANCY)) {
			if constexpr (Map::hasMapTypes(MapType::VOID_REGION)) {
				if (void_region_instant_max_occupancy && map.voidRegion(node)) {
					map.occupancySetLogit(node, occupancy_max, false);
				} else {
					map.occupancyUpdateLogit(node, occupancy, occupancy_min, occupancy_max, false);
				}
			} else {
				map.occupancyUpdateLogit(node, occupancy, occupancy_min, occupancy_max, false);
			}
		}

		if constexpr (Map::hasMapTypes(MapType::COLOR) && contains_type_v<Color, Data>) {
			// TODO: Make correct
			map.colorSet(node, data.template get<Color>(), false);
		}

		// TODO: Add more map types
	}

	template <class Map, class T, class... Rest>
	void insertHits(Map& map, PointCloud<Dim, T, Rest...> const& cloud) const
	{
		auto const occ     = probabilityToLogit(occupancy_hit);
		auto const occ_min = probabilityToLogit(occupancy_min_clamp_thres);
		auto const occ_max = probabilityToLogit(occupancy_max_clamp_thres);

		cached_hits_.resize(cloud.size());
		auto points = cloud.template view<0>();
		ufo::transform(
		    points.begin(), points.end(), cached_hits_.begin(),
		    [&map, d = hit_depth](auto const& p) { return map.code(TreeCoord(p, d)); });

		map.create(cached_hits_, cached_hits_.begin());

		ufo::for_each(std::size_t(0), static_cast<std::size_t>(cached_hits_.size()),
		              [this, &map, &cloud, occ, occ_min, occ_max](std::size_t i) {
			              auto node = cached_hits_[i].node;

			              // This chick wants to rule the block (node.pos being the block)
			              //  std::lock_guard lock(map.chicken(node.pos));

			              insertHit(map, node, cloud[i], occ, occ_min, occ_max);
		              });
	}

	template <
	    class ExecutionPolicy, class Map, class T, class... Rest,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void insertHits(ExecutionPolicy&& policy, Map& map,
	                PointCloud<Dim, T, Rest...> const& cloud) const
	{
		auto const occ     = probabilityToLogit(occupancy_hit);
		auto const occ_min = probabilityToLogit(occupancy_min_clamp_thres);
		auto const occ_max = probabilityToLogit(occupancy_max_clamp_thres);

		cached_hits_.resize(cloud.size());
		auto points = cloud.template view<0>();
		ufo::transform(
		    policy, points.begin(), points.end(), cached_hits_.begin(),
		    [&map, d = hit_depth](auto const& p) { return map.code(TreeCoord(p, d)); });

		map.create(policy, cached_hits_, cached_hits_.begin());

		ufo::for_each(std::forward<ExecutionPolicy>(policy), std::size_t(0),
		              static_cast<std::size_t>(cached_hits_.size()),
		              [this, &map, &cloud, occ, occ_min, occ_max](std::size_t i) {
			              auto node = cached_hits_[i].node;

			              // This chick wants to rule the block (node.pos being the block)
			              //  std::lock_guard lock(map.chicken(node.pos));

			              insertHit(map, node, cloud[i], occ, occ_min, occ_max);
		              });
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Misses                                        |
	|                                                                                     |
	**************************************************************************************/

	template <class Map>
	void insertMiss(Map& map, detail::Miss<Dim> const& miss, logit_t occupancy,
	                logit_t occupancy_min, logit_t occupancy_max) const
	{
		if constexpr (Map::hasMapTypes(MapType::VOID_REGION)) {
			if (miss.void_region) {
				map.voidRegionSet(miss.node, true, false);
			}
		}

		if constexpr (Map::hasMapTypes(MapType::OCCUPANCY)) {
			// TODO: Is this good?
			if constexpr (Map::hasMapTypes(MapType::VOID_REGION)) {
				if (void_region_instant_min_occupancy && map.voidRegion(miss.node)) {
					map.occupancySetLogit(miss.node, occupancy_min, false);
				} else {
					map.occupancyUpdateLogit(miss.node, miss.count * occupancy, occupancy_min,
					                         occupancy_max, false);
				}
			} else {
				map.occupancyUpdateLogit(miss.node, miss.count * occupancy, occupancy_min,
				                         occupancy_max, false);
			}
		}

		// TODO: Add more map types
	}

	template <class Map>
	void insertMisses(Map& map, std::vector<detail::Miss<Dim>>& misses) const
	{
		auto const occ     = probabilityToLogit(occupancy_miss);
		auto const occ_min = probabilityToLogit(occupancy_min_clamp_thres);
		auto const occ_max = probabilityToLogit(occupancy_max_clamp_thres);

		map.create(misses, misses.begin());

		ufo::for_each(misses.begin(), misses.end(),
		              [this, &map, occ, occ_min, occ_max](auto const& miss) {
			              // This chick wants to rule the block (node.pos being the block)
			              //  std::lock_guard lock(map.chicken(miss.index));

			              insertMiss(map, miss, occ, occ_min, occ_max);
		              });
	}

	template <
	    class ExecutionPolicy, class Map,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void insertMisses(ExecutionPolicy&& policy, Map& map,
	                  std::vector<detail::Miss<Dim>>& misses) const
	{
		auto const occ     = probabilityToLogit(occupancy_miss);
		auto const occ_min = probabilityToLogit(occupancy_min_clamp_thres);
		auto const occ_max = probabilityToLogit(occupancy_max_clamp_thres);

		map.create(std::forward<ExecutionPolicy>(policy), misses, misses.begin());

		ufo::for_each(std::forward<ExecutionPolicy>(policy), misses.begin(), misses.end(),
		              [this, &map, occ, occ_min, occ_max](auto const& miss) {
			              // This chick wants to rule the block (node.pos being the block)
			              //  std::lock_guard lock(map.chicken(miss.index));

			              insertMiss(map, miss, occ, occ_min, occ_max);
		              });
	}

 protected:
	mutable __block std::vector<detail::Hit<Dim>> cached_hits_;
};
}  // namespace ufo

#endif  // UFO_MAP_INTEGRATOR_INTEGRATOR_HPP