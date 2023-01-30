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

#ifndef UFO_MAP_INTEGRATOR_BASE_H
#define UFO_MAP_INTEGRATOR_BASE_H

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
	//
	// Integrate hits
	//

	template <class Map, class P>
	void integrateHits(Map& map, IntegrationCloud<P> const& cloud) const
	{
		// Get current state
		auto const time = getTime();

		logit_t prob;
		if constexpr (util::is_base_of_template_v<OccupancyMapBase, std::decay_t<Map>>) {
			prob = map.toOccupancyChangeLogit(
			    getOccupancyProbHit());  // + map.toOccupancyChangeLogit(getOccupancyProbMiss())
		}

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

		if constexpr (util::is_base_of_template_v<OccupancyMapBase, std::decay_t<Map>>) {
			for (auto [idx, _] : data) {
				map.increaseOccupancyLogitUnsafe(idx, prob);
			}
		}

		if constexpr (util::is_base_of_template_v<TimeMapBase, std::decay_t<Map>>) {
			for (auto [idx, _] : data) {
				map.setTimeUnsafe(idx, time);
			}
		}

		if constexpr (util::is_base_of_template_v<ColorMapBase, std::decay_t<Map>> &&
		              std::is_base_of_v<Color, P>) {
			for (auto [idx, i] : data) {
				std::uint32_t r{};
				std::uint32_t g{};
				std::uint32_t b{};
				std::uint32_t n{};
				for (Color c : cloud[i].points) {
					r += c.red;
					g += c.green;
					b += c.blue;
					n += c.isSet();
				}

				if (0 != n) {
					map.updateColorUnsafe(idx, [r = r / n, g = g / n, b = b / n](Color c) {
						int n = c.isSet();
						return Color((r + c.red) >> n, (g + c.green) >> n, (b + c.blue) >> n);
					});
				}
			}
		}

		// TODO: Something with labels
		if constexpr (util::is_base_of_template_v<LabelMapBase, std::decay_t<Map>> &&
		              std::is_base_of_v<Label, P>) {
			for (auto [idx, i] : data) {
			}
		}

		// TODO: Something with semantics
		if constexpr (util::is_base_of_template_v<SemanticMapBase, std::decay_t<Map>> &&
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

		// FIXME: Increment time step
		time_ += time_auto_inc_;
	}

	//
	// Integrate misses
	//

	template <class Map>
	void integrateMisses(Map& map, Misses const& misses) const
	{
		if constexpr (!util::is_base_of_template_v<OccupancyMapBase, std::decay_t<Map>> &&
		              !util::is_base_of_template_v<TimeMapBase, std::decay_t<Map>>) {
			return;
		}

		auto const time = getTime();

		std::vector<IndexFam> data;
		data.reserve(misses.size());
		for (auto [code, children] : misses) {
			data.emplace_back(map.create(code).index, children);
		}

		for (auto e : data) {
			map.setModifiedUnsafe(e);
		}

		if constexpr (util::is_base_of_template_v<OccupancyMapBase, std::decay_t<Map>>) {
			auto const prob = map.toOccupancyChangeLogit(getOccupancyProbMiss());
			for (auto e : data) {
				map.decreaseOccupancyLogitUnsafe(e, prob);

				if constexpr (util::is_base_of_template_v<TimeMapBase, std::decay_t<Map>>) {
					map.setTimeUnsafe(e, time);
				}
			}
		} else if constexpr (util::is_base_of_template_v<TimeMapBase, std::decay_t<Map>>) {
			for (auto e : data) {
				map.setTimeUnsafe(e, time);
			}
		}
	}

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
	                      bool const propagate = true) const
	{
		// Get current state
		auto const hit_depth = getHitDepth();

		// Create integration cloud
		auto ic = toIntegrationCloud(map, cloud, hit_depth);

		// Integrate into the map
		integrateHits(map, ic);

		// Propagate information in the map
		if (propagate) {
			map.propagateModified();
		}
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
	void insertPointCloud(Map& map, PointCloudT<P> const& cloud, Point const sensor_origin,
	                      bool const propagate = true) const
	{
		// Get current state
		auto const hit_depth = getHitDepth();
		auto const miss_depth = getMissDepth();
		auto const max_range = getMaxRange();
		auto const is_discrete = isDiscretize();

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
		integrateMisses(map, misses);
		integrateHits(map, ic);

		if (propagate) {
			// Propagate information in the map
			map.propagateModified();
		}
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
	void insertPointCloud(Map& map, PointCloudT<P> cloud, Point const sensor_origin,
	                      math::Pose6f frame_origin, bool const propagate = true) const
	{
		applyTransform(cloud, frame_origin);
		insertPointCloud(map, cloud, frame_origin.transform(sensor_origin), propagate);
	}

	//
	// Getters
	//

	/*!
	 * If discretize integration is enabled.
	 *
	 * @return true if discretize integration is enabled.
	 * @return false if discretize integration is disabled.
	 */
	[[nodiscard]] constexpr bool isDiscretize() const noexcept { return discretize_; }

	/*!
	 * The max range to integrate.
	 *
	 * @return The max range to integrate.
	 */
	[[nodiscard]] constexpr double getMaxRange() const noexcept { return max_range_; }

	/*!
	 * The depth used for misses from ray casting.
	 *
	 * @return The depth for misses.
	 */
	[[nodiscard]] constexpr depth_t getMissDepth() const noexcept { return miss_depth_; }

	/*!
	 * The depth used for hits.
	 *
	 * @return The depth for hits.
	 */
	[[nodiscard]] constexpr depth_t getHitDepth() const noexcept { return hit_depth_; }

	/*!
	 * @return Whether weighted integration is enable.
	 */
	[[nodiscard]] constexpr bool isWeighted() const noexcept { return weighted_; }

	/*!
	 * @return The probability for a hit in the sensor model.
	 */
	[[nodiscard]] constexpr float getOccupancyProbHit() const noexcept
	{
		return occupancy_prob_hit_;
	}

	/*!
	 * @return The probability for a miss in the sensor model.
	 */
	[[nodiscard]] constexpr float getOccupancyProbMiss() const noexcept
	{
		return occupancy_prob_miss_;
	}

	/*!
	 * @return The time step.
	 */
	[[nodiscard]] constexpr time_t getTime() const noexcept { return time_; }

	/*!
	 * @return Whether the time step is automatically incremented.
	 */
	[[nodiscard]] constexpr bool isTimeAutoInc() const noexcept
	{
		return 0 != time_auto_inc_;
	}

	/*!
	 * @return The automatic increment step.
	 */
	[[nodiscard]] constexpr int getTimeAutoInc() const noexcept { return time_auto_inc_; }

	/*!
	 * @return The amount the semantic value increases for a hit.
	 */
	[[nodiscard]] constexpr value_t getSemanticValueHit() const noexcept
	{
		return semantic_value_hit_;
	}

	/*!
	 * @return The amount the semantic value decreases for a miss.
	 */
	[[nodiscard]] constexpr value_t getSemanticValueMiss() const noexcept
	{
		return semantic_value_miss_;
	}

	//
	// Setters
	//

	constexpr void setDiscretize(bool discretize) noexcept { discretize_ = discretize; }

	constexpr void setMaxRange(double max_range) noexcept { max_range_ = max_range; }

	constexpr void setMissDepth(depth_t depth) noexcept { miss_depth_ = depth; }

	constexpr void setHitDepth(depth_t depth) noexcept { hit_depth_ = depth; }

	constexpr void setWeighted(bool weighted) noexcept { weighted_ = weighted; }

	constexpr void setOccupancyProbHit(float prob) noexcept { occupancy_prob_hit_ = prob; }

	constexpr void setOccupancyProbMiss(float prob) noexcept
	{
		occupancy_prob_miss_ = prob;
	}

	constexpr void setTime(time_t time) noexcept { time_ = time; }

	constexpr void setTimeAutoInc(int inc) noexcept { time_auto_inc_ = inc; }

	constexpr void setSemanticValueHit(value_t value) noexcept
	{
		semantic_value_hit_ = value;
	}

	constexpr void setSemanticValueMiss(value_t value) noexcept
	{
		semantic_value_miss_ = value;
	}

 private:
	// If there should be discretization
	bool discretize_ = true;

	// Max range to integrate
	double max_range_ = -1;

	// Miss depth
	depth_t miss_depth_ = 0;

	// Hit depth
	depth_t hit_depth_ = 0;

	// Weighted
	bool weighted_ = false;

	// Occupancy specific
	float occupancy_prob_hit_ = 0.7;
	float occupancy_prob_miss_ = 0.4;

	// Time step specific
	mutable time_t time_ = 1;
	int time_auto_inc_ = 1;

	// Semantic specific
	value_t semantic_value_hit_ = 2;
	value_t semantic_value_miss_ = 1;
};
}  // namespace ufo::map

#endif  // UFO_MAP_INTEGRATOR_BASE_H