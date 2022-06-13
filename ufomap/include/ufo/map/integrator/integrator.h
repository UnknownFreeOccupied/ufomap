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
 * @brief
 *
 */
template <class P>
struct IntegrationPoint {
	struct Point {
		P point;
		bool valid;
	};

	Code code;
	std::vector<Point> points;

	IntegrationPoint(Code const code) : code(code) {}

	IntegrationPoint(Code const code, P const& point, bool const valid = true)
	    : code(code), points(1, Point{point, valid})
	{
	}

	IntegrationPoint(Code code, std::initializer_list<std::pair<P, bool>> init)
	    : code(code), points(init)
	{
	}

	[[nodiscard]] bool valid() const
	{
		return std::any_of(std::cbegin(points), std::cend(points),
		                   [](auto const& p) { return p.valid; });
	}

	bool operator==(IntegrationPoint const& rhs) const { return code == rhs.code; }

	bool operator!=(IntegrationPoint const& rhs) const { return !(*this == rhs); }

	bool operator<(IntegrationPoint const& rhs) const { return code < rhs.code; }

	bool operator<=(IntegrationPoint const& rhs) const { return code <= rhs.code; }

	bool operator>(IntegrationPoint const& rhs) const { return code > rhs.code; }

	bool operator>=(IntegrationPoint const& rhs) const { return code >= rhs.code; }
};

template <class P>
using IntegrationCloud = std::vector<IntegrationPoint<P>>;
using Misses = std::vector<Code>;

/*!
 * @brief
 *
 * @param map
 * @param cloud
 * @param transform_function
 * @param valid_function
 * @param depth_function
 * @return IntegrationCloud<P>
 */
template <class Map, class P, class TransformFunction, class ValidFunction,
          class DepthFunction>
IntegrationCloud<P> toIntegrationCloud(Map const& map, PointCloudT<P> const& cloud,
                                       TransformFunction trans_f, ValidFunction valid_f,
                                       DepthFunction depth_f)
{
	IntegrationCloud<P> i_cloud;

	if (cloud.empty()) {
		return i_cloud;
	}

	// Get the corresponding code for each points
	i_cloud.reserve(cloud.size());
	std::transform(std::cbegin(cloud), std::cend(cloud), std::back_inserter(i_cloud),
	               [&map, trans_f, valid_f, depth_f](auto const& p) {
		               auto tp = trans_f(p);
		               auto code = map.toCode(tp, depth_f(p));
		               return IntegrationPoint<P>(code, tp, valid_f(p));
	               });

	// Sort
	std::sort(std::begin(i_cloud), std::end(i_cloud));

	// Boundle together points with same code and remove duplicates
	auto cur = std::begin(i_cloud);
	for (auto it = std::next(std::begin(i_cloud)); it != std::end(i_cloud);) {
		if (cur->code == it->code) {
			cur->points.push_back(it->points.front());
			it = i_cloud.erase(it);
		} else {
			cur = it;
			++it;
		}
	}

	return i_cloud;
}

template <class Map, class P, class TransformFunction, class ValidFunction>
IntegrationCloud<P> toIntegrationCloud(Map const& map, PointCloudT<P> const& cloud,
                                       TransformFunction trans_f, ValidFunction valid_f,
                                       depth_t const depth = 0)
{
	return IntegrationCloud(map, cloud, trans_f, valid_f,
	                        [depth](auto const&) { return depth; });
}

template <class Map, class P, class DepthFunction>
IntegrationCloud<P> toIntegrationCloud(Map const& map, PointCloudT<P> const& cloud,
                                       DepthFunction depth_f)
{
	return IntegrationCloud(
	    map, cloud, [](auto const& p) { return p; }, [](auto const&) { return true; },
	    depth_f);
}

template <class Map, class P>
IntegrationCloud<P> toIntegrationCloud(Map const& map, PointCloudT<P> const& cloud,
                                       depth_t const depth = 0)
{
	return IntegrationCloud(map, cloud, [depth](auto const&) { return depth; });
}

template <class Map, class P, class DepthFunction>
IntegrationCloud<P> toIntegrationCloud(Map const& map, PointCloudT<P> const& cloud,
                                       Point3 const sensor_origin, double const max_range,
                                       DepthFunction depth_function)
{
	return IntegrationCloud(
	    map, cloud,
	    [sensor_origin, max_range](auto p) {
		    p -= sensor_origin;
		    p.normalize();
		    p *= max_range;
		    return p;
	    },
	    [sensor_origin, max_sqrt = max_range * max_range](Point3 const p) {
		    return sensor_origin.squaredDistance(p) <= max_sqrt;
	    },
	    depth_function);
}

template <class Map, class P>
IntegrationCloud<P> toIntegrationCloud(Map const& map, PointCloudT<P> const& cloud,
                                       Point3 const sensor_origin, double const max_range,
                                       depth_t const depth = 0)
{
	return IntegrationCloud(map, cloud, sensor_origin, max_range,
	                        [depth](auto const&) { return depth; });
}

template <class Map, class P>
Misses getMisses(Map const& map, IntegrationCloud<P> const& cloud,
                 Point3 const sensor_origin, bool const only_valid = false,
                 depth_t const depth = 0)
{
	CodeUnorderedSet indices;

	for (auto const& p : cloud) {
		for (auto const& [p, v] : p.points) {
			if (only_valid && !v) {
				continue;
			}
			for (auto e : computeRay(sensor_origin, p)) {
				indices.insert(e);
			}
		}
	}

	Misses misses(std::cbegin(indices), std::cend(indices));
	std::sort(std::begin(misses), std::end(misses));
	return misses;
}

template <class Map, class P>
Misses getMissesDiscrete(Map const& map, IntegrationCloud<P> const& cloud,
                         Point3 const sensor_origin, bool const only_valid = false,
                         depth_t const depth = 0)
{
	Key const origin = map.toKey(sensor_origin, depth);

	CodeUnorderedSet indices;
	Code prev;
	for (auto const& p : cloud) {
		if (only_valid && !p.valid()) {
			continue;
		}

		Code cur = p.code.toDepth(std::max(p.code.depth(), depth));
		if (cur == prev) {
			continue;
		}
		prev = cur;

		for (auto e : computeRay(origin.toDepth(cur.depth()), cur)) {
			indices.insert(e);
		}
	}

	Misses misses(std::cbegin(indices), std::cend(indices));
	std::sort(std::begin(misses), std::end(misses));
	return misses;
}

template <class Map, class P>
Misses getMissesDiscreteFast(Map const& map, IntegrationCloud<P> const& cloud,
                             Point3 const sensor_origin, bool const only_valid = false,
                             depth_t const& depth = 0)
{
	// TODO: Implement

	Misses misses;
	std::sort(std::begin(misses), std::end(misses));
	return misses;
}

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
		auto prob = map.toOccupancyLogit(occupancy_prob_hit_);

		// TODO: Something with semantics

		// For each node
		std::for_each(std::cbegin(cloud), std::cend(cloud), [&](auto const& p) {
			if (!p.valid()) {
				// Not a single valid point fell into this node
				return;
			}

			// Get the points [first, last) falling into the node
			auto first_point = std::cbegin(p.points);
			auto last_point = std::cend(p.points);

			// Create and retrieve the node
			auto node = map.createNode(cloud.codes()[p.code]);

			// Get current occupancy
			auto logit = map.getOccupancyLogit(node);

			// Update occupancy
			map.increateOccupancyLogit(node, prob, false);

			// Update time step
			if constexpr (is_base_of_template_v<OccupancyMapTimeBase, std::decay_t<Map>>) {
				map.setTimeStep(node, current_time_step_, false);
			}

			// Update color
			if constexpr (is_base_of_template_v<ColorMapBase, std::decay_t<Map>> &&
			              std::is_base_of_v<RGBColor, T>) {
				RGBColor avg_color = RGBColor::average(first_point, last_point);

				if (avg_color.set()) {
					double total_logit = logit + prob;
					double weight = prob / total_logit;
					map.updateColor(node, avg_color, weight, false);
				}
			}

			// Update semantics
			if constexpr (is_base_of_template_v<SemanticMapBase, std::decay_t<Map>> &&
			              std::is_base_of_v<SemanticPair, T>) {
				// FIXME: Implement correctly

				std::vector<SemanticPair> semantics(first_point, last_point);

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
	void integrateMisses(Map& map, Misses const& codes) const
	{
		auto prob = map.toOccupancyLogit(occupancy_prob_miss_);

		std::for_each(std::cbegin(codes), std::cend(codes), [&map, prob](auto code) {
			auto node = map.createNode(code);

			map.decreaseOccupancyLogit(node, prob, false);

			if constexpr (is_base_of_template_v<OccupancyMapTimeBase, std::decay_t<Map>>) {
				map.setTimeStep(node, current_time_step_, false);
			}
		});
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
	void insertPointCloud(Map& map, PointCloudT<P> cloud, bool const propagate = true) const
	{
		// Create integration cloud
		auto ic = toIntegrationCloud(map, cloud, hit_depth_);

		// Integrate into the map
		integrateHits(map, ic);

		// Propagate information in the map
		if (propagate) {
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
	                      bool const propagate = true) const
	{
		// Create integration cloud
		IntegrationCloud<P> ic =
		    0 > max_range_
		        ? toIntegrationCloud(map, cloud, hit_depth_)
		        : toIntegrationCloud(map, cloud, sensor_origin, max_range_, hit_depth_);

		// Ray cast to get free space
		Misses misses = getMisses(map.toCode(sensor_origin, miss_depth_), std::cbegin(hits),
		                          std::cend(hits), max_length_);

		// Integrate into the map
		integrateMisses(map, misses);
		integrateHits(map, ic);

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
	[[nodiscard]] constexpr double maxRange() const noexcept { return max_range_; }

	/*!
	 * The depth used for misses from ray casting.
	 *
	 * @return The depth for misses.
	 */
	[[nodiscard]] constexpr depth_t missDepth() const noexcept { return miss_depth_; }

	/*!
	 * The depth used for hits.
	 *
	 * @return The depth for hits.
	 */
	[[nodiscard]] constexpr depth_t hitDepth() const noexcept { return hit_depth_; }

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
	[[nodiscard]] constexpr time_step_t  timeStep() const noexcept { return time_step_; }

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
	[[nodiscard]] constexpr semantic_value_t semanticValueHit() const noexcept
	{
		return semantic_value_hit_;
	}

	/*!
	 * @return The amount the semantic value decreases for a miss.
	 */
	[[nodiscard]] constexpr semantic_value_t semanticValueMiss() const noexcept
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

	constexpr void setTimeStep(time_step_t  time_step) noexcept { time_step_ = time_step; }

	constexpr void setTimeStepAutoInc(int inc) noexcept { time_step_auto_inc_ = inc; }

	constexpr void setSemanticValueHit(semantic_value_t value) noexcept
	{
		semantic_value_hit_ = value;
	}

	constexpr void setSemanticValueMiss(semantic_value_t value) noexcept
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
	time_step_t  time_step_ = 1;
	int time_step_auto_inc_ = 1;

	// Semantic specific
	semantic_value_t semantic_value_hit_ = 2;
	semantic_value_t semantic_value_miss_ = 1;
};
}  // namespace ufo::map

#endif  // UFO_MAP_INTEGRATOR_BASE_H