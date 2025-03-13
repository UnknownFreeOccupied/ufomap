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

enum class CountSamplingMethod { NONE, BOOLEAN, MIN, MAX, MEAN };

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

	DownSamplingMethod sample_method = DownSamplingMethod::NONE;

	// TODO: Should this be here?
	bool free_hits = false;

	MapType integrate_types = MapType::ALL;

	bool verbose = false;

	CountSamplingMethod count_sample_method           = CountSamplingMethod::NONE;
	unsigned            count_max_diff                = 0;
	bool                misses_require_all_before_hit = false;

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
	void create(Map& map, std::vector<TreeCoord<Dim, T>> const& points,
	            std::vector<TreeIndex>& nodes) const
	{
		nodes.resize(points.size());
		map.create(points, nodes.begin());
	}

	template <class Map, class T>
	void create(Map& map, std::vector<Vec<Dim, T>> const& cloud, depth_t depth,
	            std::vector<TreeIndex>& nodes) const
	{
		if (0 == depth) {
			nodes.resize(cloud.size());
			map.create(cloud, nodes.begin());
		} else {
			std::vector<typename Map::Coord> points;
			points.reserve(cloud.size());

			std::transform(cloud.begin(), cloud.end(), std::back_inserter(points),
			               [depth](auto const& p) { return TreeCoord(p, depth); });

			create(map, points, nodes);
		}
	}

	template <class Map, class T>
	[[nodiscard]] std::vector<TreeIndex> create(
	    Map& map, std::vector<TreeCoord<Dim, T>> const& points) const
	{
		std::vector<TreeIndex> nodes(points.size());
		create(map, points, nodes);
		return nodes;
	}

	template <class Map, class T>
	[[nodiscard]] std::vector<TreeIndex> create(Map&                            map,
	                                            std::vector<Vec<Dim, T>> const& cloud,
	                                            depth_t                         depth) const
	{
		std::vector<TreeIndex> nodes(cloud.size());
		create(map, cloud, depth, nodes);
		return nodes;
	}

	template <
	    class ExecutionPolicy, class Map, class T,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void create(ExecutionPolicy&& policy, Map& map,
	            std::vector<TreeCoord<Dim, T>> const& points,
	            std::vector<TreeIndex>&               nodes) const
	{
		if constexpr (execution::is_seq_v<ExecutionPolicy> ||
		              execution::is_unseq_v<ExecutionPolicy>) {
			return create(map, points, nodes);
		}

		nodes.resize(points.size());
		map.create(policy, points, nodes.begin());
	}

	template <
	    class ExecutionPolicy, class Map, class T,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void create(ExecutionPolicy&& policy, Map& map, std::vector<Vec<Dim, T>> const& cloud,
	            depth_t depth, std::vector<TreeIndex>& nodes) const
	{
		if constexpr (execution::is_seq_v<ExecutionPolicy> ||
		              execution::is_unseq_v<ExecutionPolicy>) {
			return create(map, cloud, depth, nodes);
		}

		if (0 == depth) {
			nodes.resize(cloud.size());
			map.create(policy, cloud, nodes.begin());
		} else {
			__block std::vector<typename Map::Coord> points(cloud.size());

			transform(policy, cloud.begin(), cloud.end(), points.begin(),
			          [depth](auto const& p) { return TreeCoord(p, depth); });

			create(policy, map, points, nodes);
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
		create(std::forward<ExecutionPolicy>(policy), map, points, depth, nodes);
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
		create(std::forward<ExecutionPolicy>(policy), map, cloud, depth, nodes);
		return nodes;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Hits                                         |
	|                                                                                     |
	**************************************************************************************/

	template <class Map, class Point>
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
	void insertMiss(Map& map, TreeIndex node, unsigned count, logit_t occupancy_logit) const
	{
		if constexpr (Map::hasMapTypes(MapType::OCCUPANCY)) {
			// TODO: Make sure `info.count() * occupancy_logit` does not overflow
			map.occupancyUpdateLogit(node, count * occupancy_logit, false);
		}

		// TODO: Add more map types
	}

	template <class Map>
	void insertMisses(Map& map, std::vector<TreeIndex> const& nodes,
	                  std::vector<unsigned> const& counts) const
	{
		logit_t occupancy_logit;
		if constexpr (Map::hasMapTypes(MapType::OCCUPANCY)) {
			// TODO: What function should be used here?
			occupancy_logit = map.occupancyLogit(occupancy_miss);
		}

		for (std::size_t i{}; nodes.size() > i; ++i) {
			insertMiss(map, nodes[i], counts[i], occupancy_logit);
		}
	}

	template <
	    class ExecutionPolicy, class Map,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void insertMisses(ExecutionPolicy&& policy, Map& map,
	                  std::vector<TreeIndex> const& nodes,
	                  std::vector<unsigned> const&  counts) const
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

			         insertMiss(map, node, counts[i], occupancy_logit);
		         });
	}

	/**************************************************************************************
	|                                                                                     |
	|                                 Primary void region                                 |
	|                                                                                     |
	**************************************************************************************/

	template <class Map>
	void insertVoidRegion(Map& map, TreeIndex node) const
	{
		if constexpr (Map::hasMapTypes(MapType::VOID_REGION)) {
			map.voidRegionSet(node, true, false);
		}
	}

	template <class Map>
	void insertVoidRegions(Map& map, std::vector<TreeIndex> const& nodes) const
	{
		if constexpr (!Map::hasMapTypes(MapType::VOID_REGION)) {
			return;
		} else if (MapType::VOID_REGION != (MapType::VOID_REGION & integrate_types)) {
			return;
		}

		for (std::size_t i{}; nodes.size() > i; ++i) {
			insertVoidRegion(map, nodes[i]);
		}
	}

	template <
	    class ExecutionPolicy, class Map,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void insertVoidRegions(ExecutionPolicy&& policy, Map& map,
	                       std::vector<TreeIndex> const& nodes) const
	{
		if constexpr (!Map::hasMapTypes(MapType::VOID_REGION)) {
			return;
		} else if (MapType::VOID_REGION != (MapType::VOID_REGION & integrate_types)) {
			return;
		}

		for_each(std::forward<ExecutionPolicy>(policy), std::size_t(0), nodes.size(),
		         [&](std::size_t i) {
			         auto node = nodes[i];

			         // This chick wants to rule the block (node.pos being the block)
			         std::lock_guard lock(map.chicken(node.pos));

			         insertVoidRegion(map, node);
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
};
}  // namespace ufo

#endif  // UFO_MAP_INTEGRATOR_INTEGRATOR_HPP