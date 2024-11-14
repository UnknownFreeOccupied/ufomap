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

#ifndef UFO_MAP_INTEGRATION_INTEGRATOR_HPP
#define UFO_MAP_INTEGRATION_INTEGRATOR_HPP

// UFO
#include <ufo/map/integration/misses.hpp>
#include <ufo/math/pose3.hpp>
#include <ufo/math/pose6.hpp>
#include <ufo/math/vec2.hpp>
#include <ufo/math/vec3.hpp>
#include <ufo/math/vec4.hpp>
#include <ufo/pcl/cloud.hpp>

// STL
#include <algorithm>
#include <future>
#include <type_traits>
#include <utility>

namespace ufo
{
enum class RayCastingMethod { PROPER, SIMPLE };

enum class DownsamplingMethod { NONE, CENTER, CENTROID, FIRST, UNIFORM };

enum class SemanticMethod { SIMPLE, PANOPTIC_FUSION, KIMERA };

template <class Coord, class Pose>
class Integrator
{
 public:
	DownsamplingMethod down_sampling_method{DownsamplingMethod::CENTER};

	depth_t hit_depth  = 0;
	depth_t miss_depth = 0;

	// Min range to integrate
	float min_range = 0.0f;
	// Max range to integrate, negative value is infinity range
	float max_range = -1.0f;

	bool only_valid = false;

	float early_stop_distance = 0.0f;

	RayCastingMethod ray_casting_method        = RayCastingMethod::PROPER;
	float            simple_ray_casting_factor = 1.0f;

	bool ray_passthrough_hits = true;

	// Occupancy hit [0, 1]
	occupancy_t occupancy_hit = 0.7f;
	// Occupancy miss [0, 1]
	occupancy_t occupancy_miss = 0.4f;

	// Time
	mutable time_t time = 1;
	// How much time should automatically increase after function call
	time_t time_auto_inc = 1;

	// Semantic specific
	value_t value_hit  = 2;
	value_t value_miss = 1;

	// Inflate unknown
	std::size_t inflate_unknown              = 2;
	bool        inflate_unknown_compensation = false;

	// Inflate hits
	float inflate_hits_dist = 0.0f;

	bool        parallel    = true;
	std::size_t num_threads = 8 * std::thread::hardware_concurrency();

	// Device
	Device device = Device::CPU;

 public:
	template <class Map, class PointCloud,
	          std::enable_if_t<contains_type_v<Coord, PointCloud>, bool> = true>
	void insertPoints(Map& map, PointCloud cloud, bool propagate = true)
	{
		integrateHits(map, cloud);

		if (propagate) {
			map.propagateModified();
		}

		time += time_auto_inc;
	}

	template <class Map, class PointCloud,
	          std::enable_if_t<contains_type_v<Coord, PointCloud>, bool> = true>
	void insertPoints(Map& map, PointCloud cloud, Pose frame_origin, bool propagate = true)
	{
#if defined(UFO_TBB) || defined(UFO_OMP)
		if (parallel) {
#if defined(UFO_TBB)
			applyTransform(std::execution::par_unseq, cloud, frame_origin);
#elif defined(UFO_OMP)
			applyTransformParallel(cloud, frame_origin);
#endif
		} else
#endif
		{
			applyTransform(cloud, frame_origin);
		}

		insertPoints(map, cloud, propagate);
	}

	template <class Map, class PointCloud,
	          std::enable_if_t<contains_type_v<Coord, PointCloud>, bool> = true>
	void insertRays(Map& map, PointCloud cloud, Coord sensor_origin, bool propagate = true)
	{
		// Copy of the points from the point cloud
		auto points = ufo::get<Coord>(cloud);

		// Hits
		auto hits =
		    std::async(std::launch::async, [this, &map, &cloud, sensor_origin]() mutable {
			    filterDistance(cloud, sensor_origin);
			    integrateHits(map, cloud);
		    });

		// Misses
		if (only_valid) {
			filterDistance(points, sensor_origin);
		} else {
			// TODO: Move points closer
		}

		auto misses = rayCast(map, points, sensor_origin);

		hits.wait();

		integrateMisses(map, misses);

		if (propagate) {
			map.propagateModified();
		}

		time += time_auto_inc;
	}

	template <class Map, class PointCloud,
	          std::enable_if_t<contains_type_v<Coord, PointCloud>, bool> = true>
	void insertRays(Map& map, PointCloud cloud, Coord sensor_origin, Pose frame_origin,
	                bool propagate = true)
	{
		if (parallel) {
#if defined(UFO_TBB)
			applyTransform(std::execution::par_unseq, cloud, frame_origin);
#elif defined(UFO_OMP)
			applyTransformParallel(cloud, frame_origin);
#else
			// Fallback
			applyTransform(cloud, frame_origin);
#endif
		} else {
			applyTransform(cloud, frame_origin);
		}

		insertRays(map, cloud, frame_origin.transform(sensor_origin), propagate);
	}

 private:
	template <class PointCloud, class Compare>
	void sort(PointCloud& cloud, Compare comp) const
	{
#if defined(UFO_TBB)
		if (parallel) {
			std::sort(std::execution::par_unseq, std::begin(cloud), std::end(cloud), comp);
		} else
#endif
		{
			std::sort(std::begin(cloud), std::end(cloud), comp);
		}
	}

	template <class Map, class... Ts>
	[[nodiscard]] Cloud<typename Map::Code, Ts...> addCodes(Map const&     map,
	                                                        Cloud<Ts...>&& cloud,
	                                                        depth_t        depth)
	{
		// TODO: Make nicer

		using Code = typename Map::Code;

		Cloud<Code, Ts...> cloud_with_codes(std::move(cloud));

		auto& codes  = ufo::get<Code>(cloud_with_codes);
		auto& points = ufo::get<Coord>(cloud_with_codes);

#if defined(UFO_TBB) || defined(UFO_OMP)
		if (parallel) {
#if defined(UFO_TBB)
			std::transform(std::execution::par_unseq, std::cbegin(points), std::cend(points),
			               std::begin(codes),
			               [&map, depth](Coord p) { return map.toCode(p, depth); });
#elif defined(UFO_OMP)
			int n = points.size();
#pragma omp parallel for
			for (int i = 0; n > i; ++i) {
				codes[i] = map.toCode(points[i], depth);
			}
#endif
		} else
#endif
		{
			std::transform(std::cbegin(points), std::cend(points), std::begin(codes),
			               [&map, depth](Coord p) { return map.toCode(p, depth); });
		}

		return cloud_with_codes;
	}

	template <class Map, class... Ts>
	[[nodiscard]] Cloud<typename Map::Index, Ts...> addIndices(Map const&     map,
	                                                           Cloud<Ts...>&& cloud,
	                                                           depth_t        depth)
	{
		using Code  = typename Map::Code;
		using Index = typename Map::Index;

		Cloud<Index, Ts...> cloud_with_indices(std::move(cloud));

		auto& indices = ufo::get<Index>(cloud_with_indices);
		auto& points  = ufo::get<Coord>(cloud_with_indices);

		return cloud_with_indices;
	}

	template <class PointCloud>
	void filterDistance(PointCloud& cloud, Coord origin) const
	{
		if (0 >= min_range && 0 > max_range) {
			return;
		}

		if (0 > max_range) {
			max_range = std::numeric_limits<decltype(max_range)>::max();
		}

#if defined(UFO_TBB) || defined(UFO_OMP)
		if (parallel) {
#if defined(UFO_TBB)
			filterDistance(std::execution::par_unseq, cloud, origin, min_range, max_range);
#elif defined(UFO_OMP)
			filterDistanceParallel(cloud, origin, min_range, max_range);
#endif
		} else
#endif
		{
			filterDistance(cloud, origin, min_range, max_range);
		}
	}

	template <class Map>
	void downsampleCenter(Map const& map, Cloud<typename Map::Code, Coord>& cloud) const
	{
		// TODO: Make parallel?

		using Code = typename Map::Code;

		auto& codes = ufo::get<Code>(cloud);

		// Remove duplicate codes
		auto last = std::unique(std::begin(codes), std::end(codes));
		codes.erase(last, std::end(codes));

		// Set points to center
		auto& points = ufo::get<Coord>(cloud);
		points.resize(codes.size());

		std::transform(std::cbegin(codes), std::cend(codes), std::begin(points),
		               [&map](Code c) { return map.toCoord(c); });
	}

	template <class Map>
	void downsampleCentroid(Map const& map, Cloud<typename Map::Code, Coord>& cloud) const
	{
		using Code = typename Map::Code;

		auto& codes  = ufo::get<Code>(cloud);
		auto& points = ufo::get<Coord>(cloud);

		std::size_t start{};
		for (std::size_t i{1}; codes.size() > i; ++i) {
			if (codes[start] == codes[i]) {
				continue;
			}

			for (std::size_t j = start + 1; i > j; ++j) {
				points[start] += points[j];
			}

			points[start] /= i - start;

			start = i;
		}

		downsampleFirst(map, cloud);
	}

	template <class Map>
	void downsampleFirst(Map const& map, Cloud<typename Map::Code, Coord>& cloud) const
	{
		// TODO: Make parallel?

		using Code = typename Map::Code;

		// Remove duplicate codes
		auto last = std::unique(std::begin(cloud), std::end(cloud),
		                        [](Code a, Code b) { return a == b; });
		cloud.erase(last, std::end(cloud));
	}

	template <class Map>
	void downsampleUniform(Map const& map, Cloud<typename Map::Code, Coord>& cloud) const
	{
		using Code = typename Map::Code;

		auto& codes  = ufo::get<Code>(cloud);
		auto& points = ufo::get<Coord>(cloud);

		std::size_t start{};
		for (std::size_t i{1}; codes.size() > i; ++i) {
			if (codes[start] == codes[i]) {
				continue;
			}

			Coord       center          = map.toCoord(codes[start]);
			std::size_t closest_idx     = start;
			float       closest_dist_sq = center.distanceSquared(points[start]);
			for (std::size_t j = start + 1; i > j; ++j) {
				float dist_sq = center.distanceSquared(points[j]);
				if (dist_sq < closest_dist_sq) {
					closest_dist_sq = dist_sq;
					closest_idx     = j;
				}
			}

			points[start] = points[closest_idx];

			start = i;
		}

		downsampleFirst(map, cloud);
	}

	template <class Map>
	void downsample(Map const& map, Cloud<typename Map::Code, Coord>& cloud) const
	{
		switch (down_sampling_method) {
			case DownsamplingMethod::NONE: return;
			case DownsamplingMethod::CENTER: downsampleCenter(map, cloud); return;
			case DownsamplingMethod::CENTROID: downsampleCentroid(map, cloud); return;
			case DownsamplingMethod::FIRST: downsampleFirst(map, cloud); return;
			case DownsamplingMethod::UNIFORM: downsampleUniform(map, cloud); return;
		}
	}

	template <class Map>
	[[nodiscard]] Misses rayCast(Map const& map, Cloud<Coord>& points,
	                             Coord sensor_origin) const
	{
		using Code = typename Map::Code;

		auto points_with_codes = addCodes(map, points, miss_depth);

		sort(points_with_codes, [](Code a, Code b) { return a < b; });

		downsample(map, points_with_codes);

		Misses misses;

		// TODO: Implement

		return misses;
	}

	template <class Map>
	void integrateOccupancyHits(Map& map, std::vector<typename Map::Index> const& nodes)
	{
		if (!map.isMapTypeEnabled(MapType::OCCUPANCY)) {
			return;
		}

		logit_t value = map.toOccupancyChangeLogit(occupancy_hit);
		for (auto node : nodes) {
			map.updateOccupancyLogit(node, value);
		}
	}

	template <class Map, class PointCloud>
	void integrateColorHits(Map& map, std::vector<typename Map::Index> const& nodes,
	                        PointCloud const& cloud)
	{
		using Code = typename Map::Code;

		if (!map.isMapTypeEnabled(MapType::COLOR)) {
			return;
		}

		auto const& codes  = ufo::get<Code>(cloud);
		auto const& colors = ufo::get<Color>(cloud);

		std::size_t first{};
		for (auto node : nodes) {
			unsigned red   = colors[first].red;
			unsigned green = colors[first].green;
			unsigned blue  = colors[first].blue;
			unsigned num   = colors[first].empty() ? 0 : 1;
			++first;
			for (auto c = codes[first - 1]; codes.size() > first && codes[first] == c;
			     ++first) {
				red += colors[first].red;
				green += colors[first].green;
				blue += colors[first].blue;
				num += colors[first].empty() ? 0 : 1;
			}
			num = num ? num : 1;
			red /= num;
			green /= num;
			blue /= num;

			map.updateColor(node, [red, green, blue](Color c) {
				unsigned r   = red + c.red;
				unsigned g   = green + c.green;
				unsigned b   = blue + c.blue;
				unsigned div = c.empty() ? 0 : 1;
				return Color(r >> div, g >> div, b >> div);
			});
		}
	}

	template <class Map>
	void integrateTimeHits(Map& map, std::vector<typename Map::Index> const& nodes)
	{
		if (!map.isMapTypeEnabled(MapType::TIME)) {
			return;
		}

		for (auto node : nodes) {
			map.setTime(node, time);
		}
	}

	template <class Map>
	void integrateCountHits(Map& map, std::vector<typename Map::Index> const& nodes)
	{
		if (!map.isMapTypeEnabled(MapType::COUNT)) {
			return;
		}

		for (auto node : nodes) {
			map.updateCount(node, 1);
		}
	}

	template <class Map, class PointCloud>
	void integrateReflectionHits(Map& map, std::vector<typename Map::Index> const& nodes,
	                             PointCloud const& cloud)
	{
		using Code = typename Map::Code;

		if (!map.isMapTypeEnabled(MapType::REFLECTION)) {
			return;
		}

		auto const& codes = ufo::get<Code>(cloud);

		std::size_t first{};
		for (auto node : nodes) {
			std::size_t last = first + 1;
			for (auto c = codes[first]; codes.size() > last && codes[last] == c; ++last) {
			}

			map.updateReflection(node, last - first, 0);

			first = last;
		}
	}

	template <class Map, class PointCloud>
	void integrateHits(Map& map, PointCloud& cloud)
	{
		using Code  = typename Map::Code;
		using Index = typename Map::Index;

		auto cloud_with_codes = addCodes(map, cloud, hit_depth);

		sort(cloud_with_codes, [](Code a, Code b) { return a < b; });

		std::vector<Index> nodes = map.createIndex(ufo::get<Code>(cloud_with_codes), true);

		// Remove duplicate
		auto last = std::unique(std::begin(nodes), std::end(nodes));
		nodes.erase(last, std::end(nodes));

#if defined(UFO_OMP)
		if (parallel) {
#pragma omp parallel
#pragma omp single nowait
			{
				if constexpr (is_occupancy_map_v<Map>) {
#pragma omp task
					integrateOccupancyHits(map, nodes);
				}

				if constexpr (is_color_map_v<Map> && is_color_v<PointCloud>) {
#pragma omp task
					integrateColorHits(map, nodes, cloud);
				}

				if constexpr (is_time_map_v<Map>) {
#pragma omp task
					integrateTimeHits(map, nodes);
				}
			}
		} else
#endif
		{
			if constexpr (is_occupancy_map_v<Map>) {
				integrateOccupancyHits(map, nodes);
			}

			if constexpr (is_color_map_v<Map> && is_color_v<PointCloud>) {
				integrateColorHits(map, nodes, cloud);
			}

			if constexpr (is_time_map_v<Map>) {
				integrateTimeHits(map, nodes);
			}
		}
	}

	template <class Map>
	void integrateMisses(Map& map, Misses const& misses)
	{
		using Code  = typename Map::Code;
		using Index = typename Map::Index;

		// TODO: Implement
	}
};

using Integrator2D = Integrator<Vec2f, Pose3f>;
using Integrator3D = Integrator<Vec3f, Pose6f>;
}  // namespace ufo

#endif  // UFO_MAP_INTEGRATION_INTEGRATOR_HPP