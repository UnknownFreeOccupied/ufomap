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
#include <ufo/execution/execution.hpp>
#include <ufo/map/color/map.hpp>
#include <ufo/map/occupancy/map.hpp>
// #include <ufo/map/time/map.hpp>
#include <ufo/map/type.hpp>
#include <ufo/map/void_region/map.hpp>
#include <ufo/math/vec.hpp>
#include <ufo/utility/spinlock.hpp>
#include <ufo/utility/type_traits.hpp>

// STL
#include <algorithm>
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

	// Min range to integrate
	float min_distance = 0.0f;
	// Max range to integrate, negative value is infinity range
	float max_distance = std::numeric_limits<float>::infinity();
	// To extend or shorten the rays
	float distance_offset = 0.0f;

	// Occupancy hit [0, 1]
	occupancy_t occupancy_hit = 0.75f;
	// Occupancy miss [0, 1]
	occupancy_t occupancy_miss = 0.45f;

	DownSamplingMethod sample_method = DownSamplingMethod::NONE;

	// TODO: Should this be here?
	bool free_hits = false;

	MapType integrate_types = MapType::ALL;

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

	template <class Map, class T>
	void create(Map& map, std::vector<TreeIndex>& nodes,
	            std::vector<TreeCoord<Dim, T>> const& points) const
	{
		nodes.resize(points.size());
		map.create(points, nodes.begin());
	}

	template <class Map, class T>
	void create(Map& map, std::vector<TreeIndex>& nodes,
	            std::vector<Vec<Dim, T>> const& cloud, depth_t depth) const
	{
		if (0 == depth) {
			nodes.resize(cloud.size());
			map.create(cloud, nodes.begin());
		} else {
			std::vector<typename Map::Coord> points;
			points.reserve(cloud.size());

			std::transform(cloud.begin(), cloud.end(), std::back_inserter(points),
			               [depth](auto const& p) { return TreeCoord(p, depth); });

			create(map, nodes, points);
		}
	}

	template <class Map, class T>
	[[nodiscard]] std::vector<TreeIndex> create(
	    Map& map, std::vector<TreeCoord<Dim, T>> const& points) const
	{
		std::vector<TreeIndex> nodes(points.size());
		create(map, nodes, points);
		return nodes;
	}

	template <class Map, class T>
	[[nodiscard]] std::vector<TreeIndex> create(Map&                            map,
	                                            std::vector<Vec<Dim, T>> const& cloud,
	                                            depth_t                         depth) const
	{
		std::vector<TreeIndex> nodes(cloud.size());
		create(map, nodes, cloud, depth);
		return nodes;
	}

	template <
	    class ExecutionPolicy, class Map, class T,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void create(ExecutionPolicy&& policy, Map& map, std::vector<TreeIndex>& nodes,
	            std::vector<TreeCoord<Dim, T>> const& points) const
	{
		if constexpr (execution::is_seq_v<ExecutionPolicy> ||
		              execution::is_unseq_v<ExecutionPolicy>) {
			return create(map, nodes, points);
		}

		nodes.resize(points.size());
		map.create(policy, points, nodes.begin());
	}

	template <
	    class ExecutionPolicy, class Map, class T,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void create(ExecutionPolicy&& policy, Map& map, std::vector<TreeIndex>& nodes,
	            std::vector<Vec<Dim, T>> const& cloud, depth_t depth) const
	{
		if constexpr (execution::is_seq_v<ExecutionPolicy> ||
		              execution::is_unseq_v<ExecutionPolicy>) {
			return create(map, nodes, cloud, depth);
		}

		if (0 == depth) {
			nodes.resize(cloud.size());
			map.create(policy, cloud, nodes.begin());
		} else {
			__block std::vector<typename Map::Coord> points(cloud.size());

			transform(policy, cloud.begin(), cloud.end(), points.begin(),
			          [depth](auto const& p) { return TreeCoord(p, depth); });

			create(policy, map, nodes, points);
		}
	}

	template <
	    class ExecutionPolicy, class Map, class T,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	[[nodiscard]] std::vector<TreeIndex> create(
	    ExecutionPolicy&& policy, Map& map, std::vector<TreeCoord<Dim, T>> const& points,
	    depth_t depth) const
	{
		std::vector<TreeIndex> nodes(points.size());
		create(std::forward<ExecutionPolicy>(policy), map, nodes, points, depth);
		return nodes;
	}

	template <
	    class ExecutionPolicy, class Map, class T,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	[[nodiscard]] std::vector<TreeIndex> create(ExecutionPolicy&& policy, Map& map,
	                                            std::vector<Vec<Dim, T>> const& cloud,
	                                            depth_t                         depth) const
	{
		std::vector<TreeIndex> nodes(cloud.size());
		create(std::forward<ExecutionPolicy>(policy), map, nodes, cloud, depth);
		return nodes;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Hits                                         |
	|                                                                                     |
	**************************************************************************************/

	template <bool SetModified, class Map, class Point>
	void insertHit(Map& map, TreeIndex const& node, Point const& data,
	               logit_t occupancy_logit) const
	{
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

		if (0 == hit_depth) {
			for (std::size_t i{}; nodes.size() > i; ++i) {
				insertHit<false>(map, nodes[i], cloud[i], occupancy_logit);
			}
		} else {
			for (std::size_t i{}; nodes.size() > i; ++i) {
				insertHit<true>(map, nodes[i], cloud[i], occupancy_logit);
			}
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

		if (0 == hit_depth) {
			for_each(std::forward<ExecutionPolicy>(policy), std::size_t(0), nodes.size(),
			         [&](std::size_t i) {
				         auto node = nodes[i];

				         // This chick wants to rule the block (node.pos being the block)
				         std::lock_guard lock(chickens[node.pos % chickens.size()]);

				         insertHit<false>(map, node, cloud[i], occupancy_logit);
			         });
		} else {
			for_each(std::forward<ExecutionPolicy>(policy), std::size_t(0), nodes.size(),
			         [&](std::size_t i) {
				         auto node = nodes[i];

				         // This chick wants to rule the block (node.pos being the block)
				         std::lock_guard lock(chickens[node.pos % chickens.size()]);

				         insertHit<true>(map, node, cloud[i], occupancy_logit);
			         });
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Misses                                        |
	|                                                                                     |
	**************************************************************************************/

	template <bool SetModified, class Map, class Info>
	void insertMiss(Map& map, TreeIndex const& node, Info const& info,
	                logit_t occupancy_logit) const
	{
		if constexpr (Map::hasMapTypes(MapType::OCCUPANCY)) {
			// TODO: Make sure `info.count() * occupancy_logit` does not overflow
			map.occupancyUpdateLogit(node, info.count() * occupancy_logit, false);
		}

		if constexpr (Map::hasMapTypes(MapType::VOID_REGION)) {
			if (info.voidRegion()) {
				map.voidRegionSet(node, true, false);
			}
		}

		// TODO: Add more map types
	}

	template <class Map, class Info>
	void insertMisses(Map& map, std::vector<TreeIndex> const& nodes,
	                  std::vector<Info> const& info) const
	{
		logit_t occupancy_logit;
		if constexpr (Map::hasMapTypes(MapType::OCCUPANCY)) {
			// TODO: What function should be used here?
			occupancy_logit = map.occupancyLogit(occupancy_miss);
		}

		if (0 == miss_depth) {
			for (std::size_t i{}; nodes.size() > i; ++i) {
				insertMiss<false>(map, nodes[i], info[i], occupancy_logit);
			}
		} else {
			for (std::size_t i{}; nodes.size() > i; ++i) {
				insertMiss<true>(map, nodes[i], info[i], occupancy_logit);
			}
		}
	}

	template <
	    class ExecutionPolicy, class Map, class Info,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void insertMisses(ExecutionPolicy&& policy, Map& map,
	                  std::vector<TreeIndex> const& nodes,
	                  std::vector<Info> const&      info) const
	{
		logit_t occupancy_logit;
		if constexpr (Map::hasMapTypes(MapType::OCCUPANCY)) {
			// TODO: What function should be used here?
			occupancy_logit = map.occupancyLogit(occupancy_miss);
		}

		if (0 == miss_depth) {
			for_each(std::forward<ExecutionPolicy>(policy), std::size_t(0), nodes.size(),
			         [&](std::size_t i) {
				         auto node = nodes[i];

				         // This chick wants to rule the block (node.pos being the block)
				         std::lock_guard lock(chickens[node.pos % chickens.size()]);

				         insertMiss<false>(map, node, info[i], occupancy_logit);
			         });
		} else {
			for_each(std::forward<ExecutionPolicy>(policy), std::size_t(0), nodes.size(),
			         [&](std::size_t i) {
				         auto node = nodes[i];

				         // This chick wants to rule the block (node.pos being the block)
				         std::lock_guard lock(chickens[node.pos % chickens.size()]);

				         insertMiss<true>(map, node, info[i], occupancy_logit);
			         });
		}
	}

 protected:
	// I want one MEGA bite of chickens
	mutable std::array<Spinlock, 1'000'000> chickens;
};
}  // namespace ufo

#endif  // UFO_MAP_INTEGRATOR_INTEGRATOR_HPP