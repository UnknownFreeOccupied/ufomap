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

#ifndef UFO_MAP_INTEGRATOR_H
#define UFO_MAP_INTEGRATOR_H

// UFO
#include <ufo/algorithm/algorithm.h>
#include <ufo/map/code.h>
#include <ufo/map/color/color_map.h>
#include <ufo/map/integration/integration.h>
#include <ufo/map/integration/integration_point.h>
#include <ufo/map/integration/integration_point_cloud.h>
#include <ufo/map/occupancy/occupancy_map.h>
#include <ufo/map/point_cloud.h>
#include <ufo/map/ray_caster/ray_caster.h>
// #include <ufo/map/semantic/semantic_map.h>
#include <ufo/map/time/time_map.h>
#include <ufo/map/types.h>
#include <ufo/math/pose6.h>
#include <ufo/util/timing.h>
#include <ufo/util/type_traits.h>

// STL
#include <algorithm>
#include <future>
#include <thread>
#include <type_traits>
#include <vector>

namespace ufo::map
{
/*!
 * Integrator facilitate integration of point cloud data into maps.
 *
 */
class Integrator
{
 public:
	//  If there should be discretization
	bool discretize{true};

	// Max range to integrate, negative value is infinity range
	double max_range{-1};

	// Hit depth
	depth_t hit_depth{};
	// Miss depth
	depth_t miss_depth{};

	// Occupancy hit [0, 1]
	occupancy_t occupancy_hit{0.7};
	// Occupancy miss [0, 1]
	occupancy_t occupancy_miss{0.4};

	// Time
	time_t time{1};
	// How much time should automatically increase after function call
	time_t time_auto_inc{1};

	// Semantic specific
	value_t value_hit{2};
	value_t value_miss{1};

 public:
	//
	// Insert point cloud
	//

	/*!
	 * Integrate a point cloud into a map.
	 *
	 * @param map Map to integrate into.
	 * @param cloud Point cloud to integrate.
	 * @param propagate Whether to update the inner nodes of the map.
	 */
	template <class Map, class P>
	void insertPointCloud(Map& map, PointCloudT<P> const& cloud,
	                      bool propagate = true) const
	{
		// Get current state
		auto hit_depth = getHitDepth();

		// Create integration cloud
		auto ic = toIntegrationCloud(map, cloud, hit_depth);

		// Integrate into the map
		integrateHits(map, ic, time);

		// Propagate information in the map
		if (propagate) {
			map.propagateModified();
		}

		// Increase time
		time += time_auto_inc;
	}

	/*!
	 * Integrate a point cloud into a map. Ray casting is used to clear free space between
	 * the points in the point cloud and the sensor origin.
	 *
	 * @param map Map to integrate into.
	 * @param cloud Point cloud in global reference frame to integrate.
	 * @param sensor_origin Origin of the sensor in global reference frame.
	 * @param propagate Whether to update the inner nodes of the map.
	 */
	template <class Map, class P>
	void insertPointCloud(Map& map, PointCloudT<P> const& cloud, Point sensor_origin,
	                      bool propagate = true) const
	{
		// Get current state
		auto hit_depth = getHitDepth();
		auto miss_depth = getMissDepth();
		auto max_range = getMaxRange();
		auto is_discrete = isDiscretize();

		// Create integration cloud
		IntegrationCloud<P> ic =
		    0 > max_range_
		        ? toIntegrationCloud(map, cloud, hit_depth)
		        : toIntegrationCloud(map, cloud, sensor_origin, max_range, hit_depth);

		// Ray cast to get free space
		Misses misses = is_discrete
		                    ? getMissesDiscreteFast(map, ic, sensor_origin, false, miss_depth)
		                    : getMisses(map, ic, sensor_origin, false, miss_depth);

		// Integrate into the map
		integrateMisses(map, misses, time);
		integrateHits(map, ic, time);

		if (propagate) {
			// Propagate information in the map
			map.propagateModified();
		}

		// Increase time
		time += time_auto_inc;
	}

	/*!
	 * Integrate a point cloud into a map. Ray casting is used to clear free space between
	 * the points in the point cloud and the sensor origin.
	 *
	 * @param map Map to integrate into.
	 * @param sensor_origin Origin of the sensor relative to frame_origin.
	 * @param cloud Point cloud relative to frame_origin to integrate.
	 * @param frame_origin Origin of reference frame, determines transform to be applied to
	 * cloud and sensor_origin.
	 * @param propagate Whether to update the inner nodes of the map.
	 */
	template <class Map, class P>
	void insertPointCloud(Map& map, PointCloudT<P> cloud, Point sensor_origin,
	                      math::Pose6f frame_origin, bool propagate = true) const
	{
		applyTransform(cloud, frame_origin);
		insertPointCloud(map, cloud, frame_origin.transform(sensor_origin), propagate);
	}

 private:
	//
	// Integrate hits
	//

	template <class Map, class P>
	void integrateHits(Map& map, IntegrationCloud<P> const& cloud, time_t time) const
	{
		std::vector<std::pair<Index, std::uint32_t>> data;
		data.reserve(cloud.size());
		for (std::uint32_t i{}; cloud.size() != i; ++i) {
			if (!cloud[i].points.empty()) {
				data.emplace_back(map.create(points[i].code), i);
			}
		}

		for (auto [idx, _] : data) {
			map.setModifiedUnsafe(idx);
		}

		if constexpr (util::is_base_of_template_v<OccupancyMap, std::decay_t<Map>>) {
			auto prob = map.toOccupancyChangeLogit(
			    occupancy_prob_hit);  // + map.toOccupancyChangeLogit(occupancy_prob_miss)
			for (auto [idx, _] : data) {
				map.increaseOccupancyLogitUnsafe(idx, prob);
			}
		}

		if constexpr (util::is_base_of_template_v<TimeMap, std::decay_t<Map>>) {
			for (auto [idx, _] : data) {
				map.setTimeUnsafe(idx, time);
			}
		}

		if constexpr (util::is_base_of_template_v<ColorMap, std::decay_t<Map>> &&
		              std::is_base_of_v<Color, P>) {
			for (auto [idx, i] : data) {
				std::uint32_t r{}, g{}, b{}, n{};
				for (Color c : cloud[i].points) {
					r += c.red;
					g += c.green;
					b += c.blue;
					n += c.isSet();
				}

				if (0 == n) {
					continue;
				}

				map.updateColorUnsafe(idx, [r = r / n, g = g / n, b = b / n](Color c) {
					int n = c.isSet();
					return Color((c.red + r) >> n, (c.green + g) >> n, (c.blue + b) >> n);
				});
			}
		}

		// TODO: Something with labels
		if constexpr (util::is_base_of_template_v<LabelMap, std::decay_t<Map>> &&
		              std::is_base_of_v<Label, P>) {
			for (auto [idx, i] : data) {
			}
		}

		// TODO: Something with semantics
		if constexpr (util::is_base_of_template_v<SemanticMap, std::decay_t<Map>> &&
		              std::is_base_of_v<SemanticPair, P>) {
			for (auto [idx, i] : data) {
				// 	// FIXME: Implement correctly

				// 	std::vector<SemanticPair> semantics(first_point_it, last_point_it);

				// 	// Remove label 0
				// 	semantics.erase(std::remove(std::begin(semantics), std::end(semantics),
				// 	                            [](auto sem) { return 0 == sem.label; }),
				// 	                std::end(semantics));

				// 	// Decrease all
				// 	map.decreaseSemantic(node, semantic_value_miss_, false);

				// 	// Incrase hits
				// 	map.increaseSemantics(node, std::cbegin(semantics), std::cend(semantics),
				// 	                      semantic_value_hit_ + semantic_value_miss_, false);
			}
		}
	}

	//
	// Integrate misses
	//

	template <class Map>
	void integrateMisses(Map& map, Misses const& misses, time_t time) const
	{
		if constexpr (!util::is_base_of_template_v<OccupancyMap, std::decay_t<Map>> &&
		              !util::is_base_of_template_v<TimeMap, std::decay_t<Map>>) {
			return;
		}

		std::vector<IndexFam> data;
		data.reserve(misses.size());
		for (auto [code, children] : misses) {
			data.emplace_back(map.create(code).index, children);
		}

		for (auto e : data) {
			map.setModifiedUnsafe(e);
		}

		if constexpr (util::is_base_of_template_v<OccupancyMap, std::decay_t<Map>>) {
			auto prob = map.toOccupancyChangeLogit(occupancy_prob_miss);
			for (auto e : data) {
				map.decreaseOccupancyLogitUnsafe(e, prob);
			}
		}

		if constexpr (util::is_base_of_template_v<TimeMap, std::decay_t<Map>>) {
			for (auto e : data) {
				map.setTimeUnsafe(e, time);
			}
		}
	}
};
}  // namespace ufo::map

#endif  // UFO_MAP_INTEGRATOR_H