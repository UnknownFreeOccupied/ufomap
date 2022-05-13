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
#include <ufo/map/code.h>
#include <ufo/map/color/color_map_base.h>
// #include <ufo/map/integrator/grid.h>
// #include <ufo/map/integrator/integrator_point_cloud.h>
#include <ufo/algorithm/algorithm.h>
#include <ufo/map/occupancy/occupancy_map_base.h>
#include <ufo/map/occupancy/occupancy_map_time_base.h>
#include <ufo/map/point_cloud.h>
#include <ufo/map/ray_caster/ray_caster.h>
#include <ufo/map/semantic/semantic_map_base.h>
#include <ufo/map/types.h>
#include <ufo/math/pose6.h>
#include <ufo/util/timing.h>

// STL
#include <algorithm>
#include <execution>
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
	void insertPointCloud(Map& map, PointCloudT<P> cloud, bool const propagate = true)
	{
		// Get the corresponding code for each point
		std::vector<Code> hits =
		    toCodes(map, std::cbegin(cloud), std::cend(cloud), hit_depth_);

		// Sort the codes and points based on the codes
		sort(std::begin(hits), std::end(hits), std::begin(cloud), std::end(cloud));

		// Get Equal indices
		std::vector<std::pair<std::size_t, std::size_t>> indices =
		    getEqualIndices(std::cbegin(hits), std::cend(hits));

		// Integrate into the map
		integrateHits(map, cloud, hits, indices);

		if (propagate) {
			// Propagate information in the map
			map.updateModifiedNodes();
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
	void insertPointCloud(Map& map, PointCloudT<P> cloud, Point3 const sensor_origin,
	                      bool const propagate = true)
	{
		// Get the corresponding code for each point
		std::vector<Code> hits =
		    toCodes(map, std::cbegin(cloud), std::cend(cloud), hit_depth_);

		// Sort the codes and points based on the codes
		sort(std::begin(hits), std::end(hits), std::begin(cloud), std::end(cloud));

		// Get Equal indices
		std::vector<std::pair<std::size_t, std::size_t>> indices =
		    getEqualIndices(std::cbegin(hits), std::cend(hits));

		// Ray cast to get free space
		std::vector<Code> misses = getMisses(map.toCode(sensor_origin, miss_depth_),
		                                     std::cbegin(hits), std::cend(hits), max_length_);

		// Integrate into the map
		integrateMisses(map, misses);
		integrateHits(map, cloud, hits, indices);

		if (propagate) {
			// Propagate information in the map
			map.updateModifiedNodes();
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
	void insertPointCloud(Map& map, PointCloudT<P> cloud, Point3 const sensor_origin,
	                      math::Pose6f frame_origin, bool const propagate = true)
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
	[[nodiscard]] constexpr double maxRange() const noexcept { return max_range_; }

	/*!
	 * The depth used for misses from ray casting.
	 *
	 * @return The depth for misses.
	 */
	[[nodiscard]] constexpr Depth missDepth() const noexcept { return miss_depth_; }

	/*!
	 * The depth used for hits.
	 *
	 * @return The depth for hits.
	 */
	[[nodiscard]] constexpr Depth hitDepth() const noexcept { return hit_depth_; }

	/*!
	 * @return Whether weighted integration is enable.
	 */
	[[nodiscard]] constexpr bool isWeighted() const noexcept { return weighted_; }

	/*!
	 * @return The probability for a hit in the sensor model.
	 */
	[[nodiscard]] constexpr float occupancyProbHit() const noexcept
	{
		return occupancy_prob_hit_;
	}

	/*!
	 * @return The probability for a miss in the sensor model.
	 */
	[[nodiscard]] constexpr float occupancyProbMiss() const noexcept
	{
		return occupancy_prob_miss_;
	}

	/*!
	 * @return The time step.
	 */
	[[nodiscard]] constexpr TimeStepType timeStep() const noexcept { return time_step_; }

	/*!
	 * @return Whether the time step is automatically incremented.
	 */
	[[nodiscard]] constexpr bool isTimeStepAutoInc() const noexcept
	{
		return 0 != time_step_auto_inc_;
	}

	/*!
	 * @return The automatic increment step.
	 */
	[[nodiscard]] constexpr int timeStepAutoInc() const noexcept
	{
		return time_step_auto_inc_;
	}

	/*!
	 * @return The amount the semantic value increases for a hit.
	 */
	[[nodiscard]] constexpr SemanticValue semanticValueHit() const noexcept
	{
		return semantic_value_hit_;
	}

	/*!
	 * @return The amount the semantic value decreases for a miss.
	 */
	[[nodiscard]] constexpr SemanticValue semanticValueMiss() const noexcept
	{
		return semantic_value_miss_;
	}

	//
	// Setters
	//

	constexpr void setDiscretize(bool discretize) noexcept { discretize_ = discretize; }

	constexpr void setMaxRange(double max_range) noexcept { max_range_ = max_range; }

	constexpr void setMissDepth(Depth depth) noexcept { miss_depth_ = depth; }

	constexpr void setHitDepth(Depth depth) noexcept { hit_depth_ = depth; }

	constexpr void setWeighted(bool weighted) noexcept { weighted_ = weighted; }

	constexpr void setOccupancyProbHit(float prob) noexcept { occupancy_prob_hit_ = prob; }

	constexpr void setOccupancyProbMiss(float prob) noexcept
	{
		occupancy_prob_miss_ = prob;
	}

	constexpr void setTimeStep(TimeStepType time_step) noexcept { time_step_ = time_step; }

	constexpr void setTimeStepAutoInc(int inc) noexcept { time_step_auto_inc_ = inc; }

	constexpr void setSemanticValueHit(SemanticValue value) noexcept
	{
		semantic_value_hit_ = value;
	}

	constexpr void setSemanticValueMiss(SemanticValue value) noexcept
	{
		semantic_value_miss_ = value;
	}

 private:
	//
	// Get logit
	//

	/*!
	 * Convert a probability to logit form for a given map.
	 *
	 * @param map The map.
	 * @param prob The probability to convert.
	 * @return The logit corresponding to the probability given the map.
	 */
	template <class Map>
	static auto getLogit(Map const& map, float const prob)
	{
		if constexpr (std::is_same_v<LogitType, uint8_t>) {
			return math::logitChangeValue<uint8_t>(prob,
			                                       map.getOccupancyClampingThresMinLogit(),
			                                       map.getOccupancyClampingThresMaxLogit());
		} else {
			return math::logit(prob);
		}
	}

	//
	// Integrate hits
	//

	template <class Map, class P>
	void integrateHits(
	    Map& map, PointCloudT<P> const& cloud, std::vector<Code> const& codes,
	    std::vector<std::pair<std::size_t, std::size_t>> const& indices) const
	{
		auto prob = getLogit(map, occupancy_prob_hit_);

		// TODO: Something with semantics

		std::for_each(std::cbegin(indices), std::cend(indices), [&](auto const& index) {
			auto node = map.createNode(codes[index.first]);
			auto logit = map.getOccupancyLogit(node);

			map.increateOccupancyLogit(node, prob, false);

			if constexpr (is_base_of_template_v<OccupancyMapTimeBase, std::decay_t<Map>>) {
				map.setTimeStep(node, current_time_step_, false);
			}

			if constexpr (is_base_of_template_v<ColorMapBase, std::decay_t<Map>> &&
			              std::is_base_of_v<RGBColor, T>) {
				RGBColor avg_color =
				    RGBColor::average(std::next(std::cbegin(cloud), index.first),
				                      std::next(std::cbegin(cloud), index.second));

				if (avg_color.set()) {
					double total_logit = logit + prob;
					double weight = prob / total_logit;
					map.updateColor(node, avg_color, weight, false);
				}
			}

			if constexpr (is_base_of_template_v<SemanticMapBase, std::decay_t<Map>> &&
			              std::is_base_of_v<SemanticPair, T>) {
				// FIXME: Implement correctly

				std::vector<SemanticPair> semantics(std::next(std::cbegin(cloud), index.first),
				                                    std::next(std::cbegin(cloud), index.second));

				// Remove label 0
				semantics.erase(std::remove(std::begin(semantics), std::end(semantics),
				                            [](auto sem) { return 0 == sem.label; }),
				                std::end(semantics));

				// Decrease all
				map.decreaseSemantic(node, semantic_value_miss_, false);

				// Incrase hits
				map.increaseSemantics(node, std::cbegin(semantics), std::cend(semantics),
				                      semantic_value_hit_ + semantic_value_miss_, false);
			}
		});
	}

	//
	// Integrate misses
	//

	template <class Map>
	void integrateMisses(Map& map, std::vector<Code> const& codes) const
	{
		auto prob = getLogit(map, occupancy_prob_miss_);

		std::for_each(std::cbegin(codes), std::cend(codes), [&map, prob](auto code) {
			auto node = map.createNode(code);

			map.decreaseOccupancyLogit(node, prob, false);

			if constexpr (is_base_of_template_v<OccupancyMapTimeBase, std::decay_t<Map>>) {
				map.setTimeStep(node, current_time_step_, false);
			}
		});
	}

	//
	// To codes
	//

	template <class Map, class InputIt>
	static std::vector<Code> toCodes(Map const& map, InputIt first, InputIt last,
	                                 Depth const depth = 0)
	{
		std::vector<Code> codes;
		codes.reserve(std::distance(first, last));
		std::transform(first, last, std::back_inserter(codes),
		               [&map, depth](auto const& p) { return map.toCode(p, depth); });
		return codes;
	}

	//
	// Sort
	//

	template <class RandomIt, class RandomIt2>
	static void sort(RandomIt first, RandomIt last, RandomIt2 first_2, RandomIt2 last_2)
	{
		auto perm = sortPermutation(first, last);
		applyPermutation(first, last, perm);
		applyPermutation(first_2, last_2, perm);
	}

	//
	// Get equal indices
	//

	template <class InputIt>
	static std::vector<std::pair<std::size_t, std::size_t>> getEqualIndices(InputIt first,
	                                                                        InputIt last)
	{
		std::vector<std::pair<std::size_t, std::size_t>> indices;
		indices.reserve(std::distance(first, last));
		for (auto it = first; it != last;) {
			auto it_last = std::find_if_not(
			    first, last, [value = *first](auto const& elem) { return value == elem; });
			indices.emplace_back(std::distance(first, it), std::distance(first, it_last));
			it = it_last;
		}
		return indices;
	}

	//
	// Get misses
	//

	template <class InputIt>
	static std::vector<Code> getMisses(Code sensor_origin, InputIt first, InputIt last,
	                                   double const max_length)
	{
		Key const start = sensor_origin;

		CodeSet indices;
		Code prev;
		while (first != last) {
			Code cur = first->toDepth(start.depth());
			if (cur == prev) {
				continue;
			}

			prev = cur;
			for (auto e : computeRay(start, cur)) {
				indices.insert(e);
			}
		}

		std::vector<Code> misses(std::cbegin(indices), std::cend(indices));
		std::sort(std::begin(misses), std::end(misses));
		return misses;
	}

 private:
	// If there should be discretization
	bool discretize_ = true;

	// Max range to integrate
	double max_range_ = -1;

	// Miss depth
	Depth miss_depth_ = 0;

	// Hit depth
	Depth hit_depth_ = 0;

	// Weighted
	bool weighted_ = false;

	// Occupancy specific
	float occupancy_prob_hit_ = 0.7;
	float occupancy_prob_miss_ = 0.4;

	// Time step specific
	TimeStepType time_step_ = 1;
	int time_step_auto_inc_ = 1;

	// Semantic specific
	SemanticValue semantic_value_hit_ = 2;   // Logodds probability of hit
	SemanticValue semantic_value_miss_ = 1;  // Logodds probability of miss
};
}  // namespace ufo::map

#endif  // UFO_MAP_INTEGRATOR_BASE_H