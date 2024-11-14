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

#ifndef UFO_MAP_INTEGRATION_HPP
#define UFO_MAP_INTEGRATION_HPP

// UFO
#include <ufo/map/integration/integration_grid.hpp>
// #include <ufo/map/integration/misses.hpp>
#include <ufo/map/types.hpp>
#include <ufo/pcl/cloud.hpp>
#include <ufo/utility/tuple_iterator.hpp>

// STL
#include <future>

#ifdef UFO_OMP
// OMP
#include <omp.h>
#endif

#ifdef UFO_TBB
// STL
#include <execution>
#endif

namespace ufo
{
enum class RayCastingMethod { PROPER, SIMPLE };

enum class DownSamplingMethod { NONE, CENTER, CENTROID, UNIFORM };

enum class SemanticMethod { SIMPLE, PANOPTIC_FUSION, KIMERA };

class Integrator
{
 public:
	DownSamplingMethod down_sampling_method{DownSamplingMethod::CENTER};

	depth_t hit_depth{};
	depth_t miss_depth{};

	// Min range to integrate
	float min_range{0};
	// Max range to integrate, negative value is infinity range
	float max_range{-1};

	bool only_valid{false};

	float early_stop_distance{0.0f};

	RayCastingMethod ray_casting_method{RayCastingMethod::PROPER};
	float            simple_ray_casting_factor{1.0f};

	bool ray_passthrough_hits{true};

	// int   shrink_free_steps{2};
	// bool  extend_free{false};
	// bool  extended_free_marked_hit{false};
	// float extend_hits_distance{};
	// bool  extend_hits_weighted{false};

	// Occupancy hit [0, 1]
	occupancy_t occupancy_hit{0.7f};
	// Occupancy miss [0, 1]
	occupancy_t occupancy_miss{0.4f};

	// Time
	mutable time_t time{1};
	// How much time should automatically increase after function call
	time_t time_auto_inc{1};

	// Semantic specific
	value_t value_hit{2};
	value_t value_miss{1};

	int sliding_window_size{};

	bool        parallel{true};
	std::size_t num_threads = 8 * std::thread::hardware_concurrency();

	bool propagate{false};

	// Device
	Device device{Device::CPU};

	// Inflate unknown
	std::size_t inflate_unknown{2};
	bool        inflate_unknown_compensation{false};

	// Inflate hits
	float inflate_hits_dist{0.0f};

 public:
	template <class Map, class PointCloud>
	void insertPoints(Map& map, PointCloud cloud, bool propagate = true)
	{
		// Create codes
		auto codes = toCodes(map, cloud, hit_depth, parallel);

		// Integrate hits into the map
		integrateHits(map, codes, cloud);

		if (propagate) {
			map.propagateModified();
		}

		time += time_auto_inc;
	}

	//
	// 3D
	//

	template <class Map, class PointCloud>
	void insertPoints(Map& map, PointCloud cloud, Pose6f frame_origin,
	                  bool propagate = true)
	{
#ifdef UFO_TBB
		if (parallel) {
			applyTransform(std::execution::par_unseq, cloud, frame_origin);
		} else
#endif
		{
			applyTransform(cloud, frame_origin);
		}

		insertPoints(map, cloud, propagate);
	}

	//
	// 2D
	//

	template <class Map, class PointCloud>
	void insertPoints(Map& map, PointCloud cloud, Pose3f frame_origin,
	                  bool propagate = true)
	{
#ifdef UFO_TBB
		if (parallel) {
			applyTransform(std::execution::par_unseq, cloud, frame_origin);
		} else
#endif
		{
			applyTransform(cloud, frame_origin);
		}

		insertPoints(map, cloud, propagate);
	}

	//
	// 3D
	//

	template <class Map, class... P>
	void insertRays(Map& map, Cloud<P...> cloud, Vec3f sensor_origin, bool propagate = true)
	{
		if (only_valid) {
			filterDistance(cloud, sensor_origin);
		}

		auto codes = toCodes(map, cloud, std::min(hit_depth, miss_depth));

		// #ifdef UFO_OMP
		// 		if (parallel) {
		// #pragma omp        parallel
		// #pragma omp single nowait
		// 			{
		// 				Misses                           misses;
		// 				Cloud<P...>*                     hit_cloud = &cloud;
		// 				std::vector<typename Map::Index> hit_nodes;

		// #pragma omp task
		// 				{
		// 					if (miss_depth <= hit_depth) {
		// 						misses = rayCast(map, cloud, codes, sensor_origin);
		// 					} else {
		// 						auto tmp = codes;
		// 						for (auto& c : tmp) {
		// 							c = c.toDepth(miss_depth);
		// 						}
		// 						misses = rayCast(map, cloud, tmp, sensor_origin);
		// 					}
		// 				}

		// #pragma omp task
		// 				{
		// 					if (!only_valid) {
		// 						cloud2;
		// 						codes2;
		// 						// TODO: Implement

		// 						if (hit_depth <= miss_depth) {
		// 							hit_nodes = map.createAndSetModifiedIndex(codes2);
		// 						} else {
		// 							auto tmp = codes2;
		// 							for (auto& c : tmp) {
		// 								c = c.toDepth(miss_depth);
		// 							}
		// 							hit_nodes = map.createAndSetModifiedIndex(tmp);
		// 						}
		// 					} else {
		// 						if (hit_depth <= miss_depth) {
		// 							hit_nodes = map.createAndSetModifiedIndex(codes);
		// 						} else {
		// 							auto tmp = codes;
		// 							for (auto& c : tmp) {
		// 								c = c.toDepth(miss_depth);
		// 							}
		// 							hit_nodes = map.createAndSetModifiedIndex(tmp);
		// 						}
		// 					}
		// 				}

		// #pragma omp task
		// 				{
		// 					integrateHits(map, ..., hit_nodes);
		// 				}

		// #pragma omp task
		// 				{
		// 					integrateMisses(map, misses);
		// 				}
		// 			}
		// 		} else
		// #endif
		// 		{
		// 		}

		if (!cloud.empty()) {
			auto hits = std::async(std::launch::async,
			                       [this, &map, cloud, codes, sensor_origin]() mutable {
				                       if (!only_valid) {
					                       // TODO: Filter both codes and cloud
					                       // filterDistance(cloud, sensor_origin);
				                       }

				                       if (!codes.empty() && hit_depth < codes.front().depth()) {
					                       for (auto& c : codes) {
						                       c = c.toDepth(hit_depth);
					                       }
				                       }

				                       return map.createAndSetModifiedIndex(codes);
			                       });

			auto misses = rayCast(map, cloud, codes, sensor_origin);

			hits.wait();

			auto ih = std::async(std::launch::async, &Integrator::integrateHits<Map, P...>,
			                     *this, map, cloud, hits.get());

			integrateMisses(map, misses);

			ih.wait();
		}

		if (propagate) {
			map.propagateModified();
		}

		time += time_auto_inc;
	}

	template <class Map, class PointCloud>
	void insertRays(Map& map, PointCloud cloud, Vec3f sensor_origin, Pose6f frame_origin,
	                bool propagate = true)
	{
#ifdef UFO_TBB
		if (parallel) {
			applyTransform(std::execution::par_unseq, cloud, frame_origin);
		} else
#endif
		{
			applyTransform(cloud, frame_origin);
		}

		insertRays(map, cloud, frame_origin.transform(sensor_origin), propagate);
	}

	//
	// 2D
	//

	template <class Map, class PointCloud>
	void insertRays(Map& map, PointCloud cloud, Vec2f sensor_origin, bool propagate = true)
	{
		// TODO: Implement
	}

	template <class Map, class PointCloud>
	void insertRays(Map& map, PointCloud cloud, Vec2f sensor_origin, Pose3f frame_origin,
	                bool propagate = true)
	{
#ifdef UFO_TBB
		if (parallel) {
			applyTransform(std::execution::par_unseq, cloud, frame_origin);
		} else
#endif
		{
			applyTransform(cloud, frame_origin);
		}

		insertRays(map, cloud, frame_origin.transform(sensor_origin), propagate);
	}

 private:
	//
	// To codes
	//

	template <class Map, class PointCloud>
	[[nodiscard]] std::vector<typename Map::Code> toCodes(Map const& map, PointCloud& cloud,
	                                                      depth_t depth) const
	{
		std::vector<typename Map::Code> codes(cloud.size());

#ifdef UFO_TBB
		if (parallel) {
			std::transform(std::execution::par_unseq, std::cbegin(cloud), std::cend(cloud),
			               std::begin(codes),
			               [&map, depth](auto const& p) { return map.toCode(p, depth); });

			TupleIterator ti(codes, cloud);
			std::sort(std::execution::par, std::begin(ti), std::end(ti));

			return codes;
		} else
#elif defined(UFO_OMP)
		if (parallel) {
#pragma omp parallel for
			for (std::size_t i = 0; i < cloud.size(); ++i) {
				codes[i] = map.toCode(cloud[i], depth);
			}
		} else
#endif
		{
			std::transform(std::cbegin(cloud), std::cend(cloud), std::begin(codes),
			               [&map, depth](auto const& p) { return map.toCode(p, depth); });
		}

		TupleIterator ti(codes, cloud);
		std::sort(std::begin(ti), std::end(ti));

		return codes;
	}

	//
	// Filter distance
	//

	template <class PointCloud, class Coord>
	void filterDistance(PointCloud& cloud, Coord origin) const
	{
		if (0 <= max_range) {
#ifdef UFO_TBB
			if (parallel) {
				filterDistance(std::execution::par_unseq, cloud, origin, max_range);
			} else
#endif
			{
				filterDistance(cloud, origin, max_range);
			}
		}
	}

	//
	// Down sample
	//

	template <class Map, class... P>
	[[nodiscard]] std::vector<std::pair<typename Map::Code, Point3>> downSample(
	    Map const& map, std::vector<typename Map::Code> const& codes,
	    Cloud<P...> const& cloud, DownSamplingMethod method, depth_t depth)
	{
		assert(codes.size() == cloud.size());

		std::vector<std::pair<typename Map::Code, Point3>> ds;
		ds.reserve(codes.size());

		switch (method) {
			case DownSamplingMethod::NONE:
				for (std::size_t i{}; i != codes.size(); ++i) {
					ds.emplace_back(codes[i].toDepth(depth), static_cast<Point3>(cloud[i]));
				}
			case DownSamplingMethod::CENTER:
				for (std::size_t i{}; i != codes.size();) {
					auto c = codes[i].toDepth(depth);
					ds.emplace_back(c, map.toCoord(c));
					for (++i; i != codes.size() && c.equalAtDepth(codes[i], depth); ++i) {
					}
				}
			case DownSamplingMethod::CENTROID:
				for (std::size_t i{}; i != codes.size();) {
					auto        c     = codes[i].toDepth(depth);
					Point3      coord = cloud[i];
					std::size_t num{1};
					for (++i; i != codes.size() && c.equalAtDepth(codes[i], depth); ++i, ++num) {
						coord += cloud[i];
					}
					ds.emplace_back(c, coord / num);
				}
			case DownSamplingMethod::UNIFORM:
				for (std::size_t i{}; i != codes.size();) {
					auto        c           = codes[i].toDepth(depth);
					Point3      center      = map.toCoord(c);
					std::size_t closest_idx = i;
					float dist_sq = center.distanceSquared(static_cast<Point3>(cloud[closest_idx]));
					for (++i; i != codes.size() && c.equalAtDepth(codes[i], depth); ++i) {
						float dsq = center.distanceSquared(static_cast<Point3>(cloud[i]));
						if (dsq < dist_sq) {
							dist_sq     = dsq;
							closest_idx = i;
						}
					}
					ds.emplace_back(c, static_cast<Point3>(cloud[closest_idx]));
				}
		}
		return ds;
	}

	template <class Map, class... P, class Coord>
	[[nodiscard]] Misses rayCast(Map const& map, Cloud<P...>& cloud,
	                             Coord sensor_origin) const
	{
		using Code = typename Map::Code;

		std::vector<std::pair<Code, Coord>> points;

		downSample(map, points, down_sampling_method, miss_depth);

		std::unordered_map<Code, IntegrationGrid<Code>> misses;
		std::unordered_map<Code, IntegrationGrid<Code>> hits;

		// TODO: Add hits

		Code  origin_code         = map.toCode(sensor_origin, miss_depth);
		Coord origin_coord        = map.toCoord(origin_code);
		float step_size_factor    = simple_ray_casting_factor;
		bool  simple              = RayCastingMethod::SIMPLE == ray_casting_method;
		float early_stop_distance = early_stop_distance;
		float min_distance        = min_range;
		float max_distance    = 0 > max_range ? std::numeric_limits<float>::max() : max_range;
		float max_distance_sq = max_distance * max_distance;
		bool  only_valid      = only_valid;
		bool  inflate_unknown_compensation = inflate_unknown_compensation;
		std::size_t inflate_unknown        = inflate_unknown;
		std::size_t compensate_inflate_unknown =
		    inflate_unknown_compensation ? inflate_unknown : 0;
		float grid_size = map.size(miss_depth);
		// float  iuc =
		// inflate_unknown_compensation ? inflate_unknown * grid_size : 0;  // *
		// std::sqrt(3)
		bool        ray_passthrough_hits = ray_passthrough_hits;
		float       inflate_hits_dist    = inflate_hits_dist;
		std::size_t num_threads{};

#ifdef UFO_OMP
		if (parallel) {
			num_threads = num_threads ? num_threads : 8 * std::thread::hardware_concurrency();
#pragma omp parallel num_threads(num_threads)
			{
				std::unordered_map<Code, IntegrationGrid<Code>> thread_misses;
#pragma omp for schedule(static)
				for (auto [_, goal] : points) {
					computeRay(thread_misses, hits, sensor_origin, goal, origin_code,
					           origin_coord - sensor_origin, grid_size, max_distance,
					           compensate_inflate_unknown, ray_passthrough_hits);
				}

#pragma omp critical
				{
					for (auto&& [code, grid] : thread_misses) {
						if (auto [it, b] = misses.try_emplace(code, std::move(grid)); !b) {
							std::transform(std::cbegin(grid), std::cend(grid), std::cbegin(it->second),
							               std::begin(it->second),
							               [](auto a, auto b) { return a | b; });
						}
					}
				}
			}
		} else
#endif
		{
			for (auto goal : goals) {
				auto       direction = goal - sensor_origin;
				auto const distance  = direction.norm();
				direction /= distance;
				auto origin = goal - (direction * inflate_hits_dist);

				auto origin_key = map.toKey(origin, depth);
				computeRay(hits, origin, goal, origin_key, map.toCoord(origin_key) - origin,
				           direction, grid_size, compensate_inflate_unknown);
			}

			for (auto goal : goals) {
				if (!only_valid && 0 <= max_distance) {
					auto       direction   = goal - sensor_origin;
					auto const distance_sq = direction.squaredNorm();
					if (max_distance_sq < distance_sq) {
						direction /= std::sqrt(distance_sq);
						goal = sensor_origin + (direction * max_distance);
					}
				}

				computeRay(misses, hits, sensor_origin, goal, origin_key,
				           origin_coord - sensor_origin, grid_size, max_distance,
				           compensate_inflate_unknown, ray_passthrough_hits);
			}
		}
		params.timing[5][2].stop();

		Misses x;
		if (1 < params.sliding_window_size) {
			params.timing[5][3].start();
			params.misses.resize(params.sliding_window_size);
			params.hits.resize(params.sliding_window_size);
			params.misses_idx %= params.sliding_window_size;
			params.hits_idx %= params.sliding_window_size;
			params.misses[params.misses_idx++] = std::move(misses);
			params.hits[params.hits_idx++]     = std::move(hits);

			std::unordered_map<Code, Grid> acc_misses;
			for (auto const& g : params.misses) {
				for (auto&& [code, grid] : g) {
					if (auto [it, b] = acc_misses.try_emplace(code, std::move(grid)); !b) {
						std::transform(std::cbegin(grid), std::cend(grid), std::cbegin(it->second),
						               std::begin(it->second), [](auto a, auto b) { return a | b; });
					}
				}
			}
			std::unordered_map<Code, Grid> acc_hits;
			for (auto const& g : params.hits) {
				for (auto&& [code, grid] : g) {
					if (auto [it, b] = acc_hits.try_emplace(code, std::move(grid)); !b) {
						std::transform(std::cbegin(grid), std::cend(grid), std::cbegin(it->second),
						               std::begin(it->second), [](auto a, auto b) { return a | b; });
					}
				}
			}
			params.timing[5][3].stop();

			params.timing[5][4].start();
			x = getMisses(inflate_unknown, std::move(acc_misses), std::move(acc_hits), depth,
			              num_threads);
			params.timing[5][4].stop();
		} else {
			params.timing[5][4].start();
			x = getMisses(inflate_unknown, std::move(misses), std::move(hits), depth,
			              num_threads);
			params.timing[5][4].stop();
		}

		return x;
	}

	template <class Map, class... P>
	[[nodiscard]] Misses getMisses(Map const& map, IntegrationCloud<P...> const& points,
	                               Point const              sensor_origin,
	                               IntegrationParams const& params)
	{
		if (points.empty()) {
			return Misses();
		}

		// TODO: Use different miss_depth and ray_casting_depth

		// INFO: Assuming all points have same depth
		depth_t depth = std::max(points.front().code.depth(), params.miss_depth);

		params.timing[5][1].start();
		std::vector<Point> goals =
		    downSample(map, points, params.down_sampling_method, depth);
		params.timing[5][1].stop();

		std::unordered_map<typename Map::Code, Grid> misses;
		std::unordered_map<typename Map::Code, Grid> hits;

		Key const   origin_key       = map.toKey(sensor_origin, depth);
		Point const origin_coord     = map.toCoord(origin_key);
		float const step_size_factor = params.simple_ray_casting_factor;
		bool const  simple           = RayCastingMethod::SIMPLE == params.ray_casting_method;
		float const early_stop_distance = params.early_stop_distance;
		float const min_distance        = params.min_range;
		float const max_distance =
		    0 > params.max_range ? std::numeric_limits<float>::max() : params.max_range;
		float const max_distance_sq              = max_distance * max_distance;
		bool const  only_valid                   = params.only_valid;
		bool const  inflate_unknown_compensation = params.inflate_unknown_compensation;
		std::size_t inflate_unknown              = params.inflate_unknown;
		std::size_t compensate_inflate_unknown =
		    inflate_unknown_compensation ? inflate_unknown : 0;
		float const grid_size = map.size(depth);
		// float const iuc =
		// inflate_unknown_compensation ? inflate_unknown * grid_size : 0;  // *
		// std::sqrt(3)
		bool const  ray_passthrough_hits = params.ray_passthrough_hits;
		float const inflate_hits_dist    = params.inflate_hits_dist;
		std::size_t num_threads{};

		params.timing[5][2].start();
#ifdef UFO_OMP
		if (params.parallel) {
			num_threads = 0 == params.num_threads ? 8 * std::thread::hardware_concurrency()
			                                      : params.num_threads;
#pragma omp parallel num_threads(num_threads)
			{
				std::unordered_map<Code, Grid> thread_hits;
#pragma omp for schedule(static)
				for (auto goal : goals) {
					auto       direction = goal - sensor_origin;
					auto const distance  = direction.norm();
					if (min_distance >
					    distance) {  // IMPROVE: Probably have to do something more than this
						continue;
					}
					direction /= distance;
					auto origin = goal - (direction * inflate_hits_dist);

					auto origin_key = map.toKey(origin, depth);

					// FIXME: If inflate_hits_dist is zero then no direction for inflate_unknown
					computeRay(thread_hits, origin, goal, origin_key,
					           map.toCoord(origin_key) - origin, direction, grid_size,
					           inflate_unknown_compensation);
				}

#pragma omp critical
				{
					for (auto&& [code, grid] : thread_hits) {
						if (auto [it, b] = hits.try_emplace(code, std::move(grid)); !b) {
							std::transform(std::cbegin(grid), std::cend(grid), std::cbegin(it->second),
							               std::begin(it->second),
							               [](auto a, auto b) { return a | b; });
						}
					}
				}

				std::unordered_map<Code, Grid> thread_misses;

#pragma omp barrier
#pragma omp for schedule(static)
				for (auto goal : goals) {
					computeRay(thread_misses, hits, sensor_origin, goal, origin_key,
					           origin_coord - sensor_origin, grid_size, max_distance,
					           compensate_inflate_unknown, ray_passthrough_hits);
				}

#pragma omp critical
				{
					for (auto&& [code, grid] : thread_misses) {
						if (auto [it, b] = misses.try_emplace(code, std::move(grid)); !b) {
							std::transform(std::cbegin(grid), std::cend(grid), std::cbegin(it->second),
							               std::begin(it->second),
							               [](auto a, auto b) { return a | b; });
						}
					}
				}
			}
		} else
#endif
		{
			for (auto goal : goals) {
				auto       direction = goal - sensor_origin;
				auto const distance  = direction.norm();
				direction /= distance;
				auto origin = goal - (direction * inflate_hits_dist);

				auto origin_key = map.toKey(origin, depth);
				computeRay(hits, origin, goal, origin_key, map.toCoord(origin_key) - origin,
				           direction, grid_size, compensate_inflate_unknown);
			}

			for (auto goal : goals) {
				if (!only_valid && 0 <= max_distance) {
					auto       direction   = goal - sensor_origin;
					auto const distance_sq = direction.squaredNorm();
					if (max_distance_sq < distance_sq) {
						direction /= std::sqrt(distance_sq);
						goal = sensor_origin + (direction * max_distance);
					}
				}

				computeRay(misses, hits, sensor_origin, goal, origin_key,
				           origin_coord - sensor_origin, grid_size, max_distance,
				           compensate_inflate_unknown, ray_passthrough_hits);
			}
		}
		params.timing[5][2].stop();

		Misses x;
		if (1 < params.sliding_window_size) {
			params.timing[5][3].start();
			params.misses.resize(params.sliding_window_size);
			params.hits.resize(params.sliding_window_size);
			params.misses_idx %= params.sliding_window_size;
			params.hits_idx %= params.sliding_window_size;
			params.misses[params.misses_idx++] = std::move(misses);
			params.hits[params.hits_idx++]     = std::move(hits);

			std::unordered_map<Code, Grid> acc_misses;
			for (auto const& g : params.misses) {
				for (auto&& [code, grid] : g) {
					if (auto [it, b] = acc_misses.try_emplace(code, std::move(grid)); !b) {
						std::transform(std::cbegin(grid), std::cend(grid), std::cbegin(it->second),
						               std::begin(it->second), [](auto a, auto b) { return a | b; });
					}
				}
			}
			std::unordered_map<Code, Grid> acc_hits;
			for (auto const& g : params.hits) {
				for (auto&& [code, grid] : g) {
					if (auto [it, b] = acc_hits.try_emplace(code, std::move(grid)); !b) {
						std::transform(std::cbegin(grid), std::cend(grid), std::cbegin(it->second),
						               std::begin(it->second), [](auto a, auto b) { return a | b; });
					}
				}
			}
			params.timing[5][3].stop();

			params.timing[5][4].start();
			x = getMisses(inflate_unknown, std::move(acc_misses), std::move(acc_hits), depth,
			              num_threads);
			params.timing[5][4].stop();
		} else {
			params.timing[5][4].start();
			x = getMisses(inflate_unknown, std::move(misses), std::move(hits), depth,
			              num_threads);
			params.timing[5][4].stop();
		}

		return x;
	}

	//
	// Integrate hits
	//

	template <class Map, class... P>
	void integrateHits(Map& map, Cloud<P...> const& cloud,
	                   std::vector<typename Map::Index> const& nodes) const
	{
		decltype(nodes) unique_nodes;

		auto make_unique = [&nodes, &unique_nodes, parallel]() {
			if (unique_nodes.empty()) {
				unique_nodes.reserve(nodes.size());
#ifdef UFO_TBB
				if (parallel) {
					std::unique_copy(std::execution::par_unseq, std::cbegin(nodes),
					                 std::cend(nodes), std::back_inserter(unique_nodes));
				} else
#endif
				{
					std::unique_copy(std::cbegin(nodes), std::cend(nodes),
					                 std::back_inserter(unique_nodes));
				}
			}
		};

#ifdef UFO_OMP
		if (parallel) {
			make_unique();

#pragma omp        parallel
#pragma omp single nowait
			{
				if constexpr (is_occupancy_map_v<Map>) {
#pragma omp task
					{
						logit_t lc = map.toOccupancyChangeLogit(occupancy_hit);
						for (auto node : unique_nodes) {
							map.updateOccupancyLogit(node, lc);
						}
					}
				}

				if constexpr (is_time_map_v<Map>) {
					if (map.isMapTypesEnabled(MapType::TIME)) {
#pragma omp task
						{
							for (auto node : unique_nodes) {
								map.setTime(node, time);
							}
						}
					}
				}
			}
		} else
#endif
		{
			if constexpr (is_occupancy_map_v<Map>) {
				make_unique();
				logit_t lc = map.toOccupancyChangeLogit(occupancy_hit);
				for (auto node : unique_nodes) {
					map.updateOccupancyLogit(node, lc);
				}
			}

			if constexpr (is_time_map_v<Map>) {
				make_unique();
				for (auto node : unique_nodes) {
					map.setTime(node, time);
				}
			}

			if constexpr (is_count_map_v<Map>) {
				make_unique();
				for (auto node : unique_nodes) {
					map.updateCount(node, 1);
				}
			}

			if constexpr (is_color_map_v<Map> && is_color_v<P...>) {
				for (std::size_t i{}; nodes.size() != i;) {
					auto     node = nodes[i];
					unsigned red{}, green{}, blue{}, num{};
					for (++i; nodes.size() != i && nodes[idx] == nodes[i]; ++i) {
						red += c->red;
						green += c->green;
						blue += c->blue;
						num += !static_cast<Color>(*c).empty();
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

			if constexpr (is_reflection_map_v<Map>) {
				auto node = nodes[i];
				for (std::size_t i{}; nodes.size() != i;) {
					auto idx = i;
					for (++i; nodes.size() != i && nodes[idx] == nodes[i]; ++i) {
					}
					map.updateReflection(nodes[idx], i - idx, 0);
				}
			}

			if constexpr (is_points_map_v<Map>) {
				map.insertPoints(node, cur, it);
			}
		}
	}

	//
	// Integrate misses
	//

	template <class Map>
	void integrateMisses(Map& map, Misses misses, IntegrationParams const& params)
	{
		params.timing[6][1].start();
		map.createIndicesFromCodes(misses);
		params.timing[6][1].stop();

		logit_t prob{};
		if constexpr (is_occupancy_map_v<Map>) {
			prob = map.toOccupancyChangeLogit(params.occupancy_miss);
		}

		params.timing[6][2].start();
#ifdef UFO_TBB
		if (params.parallel) {
			std::for_each(std::execution::par_unseq, std::cbegin(misses), std::cend(misses),
			              [&map, prob, time = params.time](auto miss) {
				              for (offset_t i{}; 8 != i; ++i) {
					              if (miss.sibling[i]) {
						              auto node = map.sibling(miss.index, i);

						              map.setModified(node);

						              if constexpr (is_occupancy_map_v<Map>) {
							              map.updateOccupancyLogit(node, prob);
						              }

						              if constexpr (is_time_map_v<Map>) {
							              map.setTime(node, time);
						              }

						              if constexpr (is_reflection_map_v<Map>) {
							              map.updateReflection(node, 0, 1);
						              }

						              if constexpr (is_seen_empty_map_v<Map>) {
							              map.setSeenEmpty(node);
						              }
					              }
				              }
			              });
		} else
#elif defined(UFO_OMP)
		if (params.parallel) {
#pragma omp parallel for
			for (std::size_t j = 0; j != misses.size(); ++j) {
				for (offset_t i{}; 8 != i; ++i) {
					if (misses[j].sibling[i]) {
						auto node = map.sibling(misses[j].index, i);

						map.setModified(node);

						if constexpr (is_occupancy_map_v<Map>) {
							map.updateOccupancyLogit(node, prob);
						}

						if constexpr (is_time_map_v<Map>) {
							map.setTime(node, time);
						}

						if constexpr (is_reflection_map_v<Map>) {
							map.updateReflection(node, 0, 1);
						}

						if constexpr (is_seen_empty_map_v<Map>) {
							map.setSeenEmpty(node);
						}
					}
				}
			}
		} else
#endif
		{
			for (auto miss : misses) {
				for (offset_t i{}; 8 != i; ++i) {
					if (miss.sibling[i]) {
						auto node = map.sibling(miss.index, i);

						map.setModified(node);

						if constexpr (is_occupancy_map_v<Map>) {
							map.updateOccupancyLogit(node, prob);
						}

						if constexpr (is_time_map_v<Map>) {
							map.setTime(node, params.time);
						}

						if constexpr (is_reflection_map_v<Map>) {
							map.updateReflection(node, 0, 1);
						}

						if constexpr (is_seen_empty_map_v<Map>) {
							map.setSeenEmpty(node);
						}
					}
				}
			}
		}
		params.timing[6][2].stop();
	}

 private:
	// IMPROVE: Move somewhere else
	mutable std::vector<std::unordered_map<Code, Grid>> misses;
	mutable std::vector<std::unordered_map<Code, Grid>> hits;
	mutable std::size_t                                 misses_idx{};
	mutable std::size_t                                 hits_idx{};

	// Timing
	mutable Timing timing{
	    "Insert Pointcloud",
	    {{1, {"Filter Distance 1"}},
	     {2, {"Create Int. Cloud"}},
	     {3, {"Filter Distance 2"}},
	     {4, {"Integrate Hits", {{1, {"Create Nodes"}}, {2, {"Update Nodes"}}}}},
	     {5,
	      {"Get Misses",
	       {{1, {"Generate Ray Cast Goals"}},
	        {2, {"Ray Cast"}},
	        {3, {"Accum. Grids"}},
	        {4, {"Extract Misses"}}}}},
	     {6, {"Integrate Misses", {{1, {"Create Nodes"}}, {2, {"Update Nodes"}}}}},
	     {7, {"Propagate"}}}};
};

/*!
 * Integrate a point points into a map.
 *
 * @param map Map to integrate into.
 * @param points Point points to integrate.
 * @param propagate Whether to update the inner nodes of the map.
 */
template <class Map, class... P>
void insertPointCloud(Map& map, Cloud<P...> const& points,
                      IntegrationParams const& params, bool propagate = true)
{
	// Create integration points
	auto ic = impl::toIntegrationCloud(map, points, params);

	// Integrate hits into the map
	impl::integrateHits(map, std::move(ic), params);

	if (propagate) {
		// Propagate information in the map
		map.propagateModified();
	}

	// Increase time
	params.time += params.time_auto_inc;
}

/*!
 * Integrate a point points into a map. Ray casting is used to clear free space between
 * the points in the point points and the sensor origin.
 *
 * @param map Map to integrate into.
 * @param points Point points in global reference frame to integrate.
 * @param sensor_origin Origin of the sensor in global reference frame.
 * @param propagate Whether to update the inner nodes of the map.
 */
template <class Map, class... P>
void insertPointCloud(Map& map, Cloud<P...> points, Point sensor_origin,
                      IntegrationParams const& params, bool propagate = true)
{
	params.timing.start();

	params.timing[1].start();
	if (params.only_valid && 0 <= params.max_range) {
		// Remove points that are further than max range
#ifdef UFO_TBB
		if (params.parallel) {
			filterDistance(std::execution::par_unseq, points, sensor_origin, params.max_range);
		} else
#endif
		{
			filterDistance(points, sensor_origin, params.max_range);
		}
	}
	params.timing[1].stop();

	// Create integration points
	params.timing[2].start();
	auto ic = impl::toIntegrationCloud(map, std::move(points), params);
	params.timing[2].stop();

	auto f = std::async(std::launch::async, [&map, ic, sensor_origin, &params]() mutable {
		params.timing[3].start();
		if (!params.only_valid && 0 <= params.max_range) {
			// Remove points that are further than max range
#ifdef UFO_TBB
			if (params.parallel) {
				filterDistance(std::execution::par_unseq, ic, sensor_origin, params.max_range);
			} else
#endif
			{
				filterDistance(ic, sensor_origin, params.max_range);
			}
		}
		params.timing[3].stop();

		// Integrate hits into the map
		params.timing[4].start();
		impl::integrateHits(map, std::move(ic), params);
		params.timing[4].stop();
	});

	// Ray cast to get misses (free space)
	params.timing[5].start();
	auto misses = impl::getMisses(map, std::move(ic), sensor_origin, params);
	params.timing[5].stop();

	// Wait until all hits has been inserted
	f.wait();

	// Integrate misses into the map
	params.timing[6].start();
	impl::integrateMisses(map, std::move(misses), params);
	params.timing[6].stop();

	params.timing[7].start();
	if (propagate) {
		// Propagate information in the map
		map.propagateModified();
	}
	params.timing[7].stop();

	// Increase time
	params.time += params.time_auto_inc;
	params.timing.stop();
}

/*!
 * Integrate a point points into a map. Ray casting is used to clear free space between
 * the points in the point points and the sensor origin.
 *
 * @param map Map to integrate into.
 * @param sensor_origin Origin of the sensor relative to frame_origin.
 * @param points Point points relative to frame_origin to integrate.
 * @param frame_origin Origin of reference frame, determines transform to be applied to
 * points and sensor_origin.
 * @param propagate Whether to update the inner nodes of the map.
 */
template <class Map, class... P>
void insertPointCloud(Map& map, Cloud<P...> points, Point sensor_origin,
                      Pose6f frame_origin, IntegrationParams const& params,
                      bool propagate = true)
{
#ifdef UFO_TBB
	if (params.parallel) {
		applyTransform(std::execution::par_unseq, points, frame_origin);
	} else
#endif
	{
		applyTransform(points, frame_origin);
	}

	// FIXME: What is correct?
	// insertPointCloud(map, std::move(points),
	// frame_origin.transform(sensor_origin), params,
	//                  propagate);
	insertPointCloud(map, std::move(points), sensor_origin, params, propagate);
}
}  // namespace ufo

#endif  // UFO_MAP_INTEGRATION_HPP