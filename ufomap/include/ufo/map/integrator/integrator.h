/*
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * License: BSD 3
 */

/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, D. Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
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
enum class RayCastingMethod { PROPER, SIMPLE };

struct DefaultIntegrator {
	template <class Map, class P>
	void operator()(Map& map, Node node, PointCloudT<P> const& points)
	{
	}

	template <class Map>
	void operator()(Map& map, Node node)
	{
	}
};

namespace details
{
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

template <class RandomIt, class RandomIt2>
static void sort(RandomIt first, RandomIt last, RandomIt2 first_2, RandomIt2 last_2)
{
	auto perm = sortPermutation(first, last);
	applyPermutation(first, last, perm);
	applyPermutation(first_2, last_2, perm);
}

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

template <class Map, class P, class TernaryFunction>
static void integrateHits(Map& map, PointCloudT<P> const& cloud,
                          std::vector<Code> const& codes,
                          std::vector<std::pair<std::size_t, std::size_t>> const& indices,
                          TernaryFunction f)
{
	std::for_each(
	    std::cbegin(indices), std::cend(indices),
	    [&, first = std::cbegin(cloud)](auto const& index) {
		    f(map, map.getNode(codes[index.first]),
		      PointCloudT<P>(std::next(first, index.first), std::next(first, index.second)));
	    });
}

template <class Map, class P, class BinaryFunction>
static void integrateMisses(Map& map, std::vector<Code> const& codes, BinaryFunction f)
{
	std::for_each(std::cbegin(codes), std::cend(codes),
	              [&map, f](auto code) { f(map, map.getNode(code)); });
}
}  // namespace details

//
// Integrate points
//

template <class Map, class P, class TernaryFunction>
void integratePoints(Map& map, PointCloudT<P> cloud, TernaryFunction f,
                     Depth const depth = 0, bool const propagate = true)
{
	using namespace details;

	// Get the corresponding code for each point
	std::vector<Code> codes = toCodes(map, std::cbegin(cloud), std::cend(cloud), depth);

	// Sort the codes and points based on the codes
	sort(std::begin(codes), std::end(codes), std::begin(cloud), std::end(cloud));

	// Get Equal indices
	std::vector<std::pair<std::size_t, std::size_t>> indices =
	    getEqualIndices(std::cbegin(codes), std::cend(codes));

	// Integrate into the map
	integrateHits(map, cloud, codes, indices, f);

	if (propagate) {
		// Propagate information in the map
		map.updateModifiedNodes();
	}
}

template <class Map, class P>
void integratePoints(Map& map, PointCloudT<P> const& cloud, Depth const depth = 0,
                     bool const propagate = true)
{
	integratePoints(map, cloud, DefaultIntegrator(), depth, propagate);
}

//
// Integrate point cloud
//

template <class Map, class P, class TernaryFunction, class BinaryFunction>
void integratePointCloud(Map& map, PointCloudT<P> cloud, Point3 const sensor_origin,
                         TernaryFunction f_hit, BinaryFunction f_miss,
                         double const max_length = -1, Depth const depth_miss = 0,
                         Depth const depth_hit = 0, bool const propagate = true)
{
	using namespace details;

	// Get the corresponding code for each point
	std::vector<Code> hits = toCodes(map, std::cbegin(cloud), std::cend(cloud), depth_hit);

	// Sort the codes and points based on the codes
	sort(std::begin(hits), std::end(hits), std::begin(cloud), std::end(cloud));

	// Get Equal indices
	std::vector<std::pair<std::size_t, std::size_t>> indices =
	    getEqualIndices(std::cbegin(hits), std::cend(hits));

	// Ray cast to get free space
	std::vector<Code> misses = getMisses(map.toCode(sensor_origin, depth_miss),
	                                     std::cbegin(hits), std::cend(hits), max_length);

	// Integrate into the map
	integrateMisses(map, misses, f_miss);
	integrateHits(map, cloud, hits, indices, f_hit);

	if (propagate) {
		// Propagate information in the map
		map.updateModifiedNodes();
	}
}

template <class Map, class P, class TernaryBinaryFunction>
void integratePointCloud(Map& map, PointCloudT<P> const& cloud,
                         Point3 const sensor_origin, TernaryBinaryFunction f,
                         double const max_length = -1, Depth const depth_miss = 0,
                         Depth const depth_hit = 0, bool const propagate = true)
{
	integratePointCloud(map, cloud, sensor_origin, f, f, max_length, depth_miss, depth_hit,
	                    propagate);
}

template <class Map, class P>
void integratePointCloud(Map& map, PointCloudT<P> const& cloud,
                         Point3 const sensor_origin, double const max_length = -1,
                         Depth const depth_miss = 0, Depth const depth_hit = 0,
                         bool const propagate = true)
{
	integratePointCloud(map, cloud, sensor_origin, DefaultIntegrator(), max_length,
	                    depth_miss, depth_hit, propagate);
}

// template <class ExecutionPolicy, class Map, class P, class BinaryFunction,
//           typename =
//               std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
// void integratePoints(ExecutionPolicy&& policy, Map& map, PointCloudT<P> const& cloud,
//                      BinaryFunction f, bool propagate = true)
// {
// 	IntegratorPointCloud<P> i_cloud(map, cloud);

// 	std::for_each(std::forward<ExecutionPolicy>(policy), std::cbegin(i_cloud),
// 	              std::cend(i_cloud),
// 	              [&map, &f](auto const& p) { f(map, p.points, map.getNode(p.code)); });

// 	if (propagate) {
// 		map.updateModifiedNodes();
// 	}
// }

// template <class Map, class P>
// void integratePointCloud(Map& map, PointCloudT<P> const& cloud, Point3 sensor_origin,
//                          bool propagate = true)
// {
// 	// TODO: Implement
// }

// template <class Map, class P>
// void insertPointCloudDiscrete(Map& map, PointCloudT<P> const& cloud, Point3
// sensor_origin,
//                               bool propagate = true)
// {
// 	// TODO: Implement
// }

// using MissedSpace = std::vector<std::pair<Code, float>>;

// class Integrator
// {
//  public:
// 	//
// 	// Constructors
// 	//

// 	Integrator(bool discretize = true, float max_range = -1, Depth miss_depth = 0,
// 	           bool weighted = false, Depth hit_depth = 0,
// 	           RayCastingMethod ray_casting_method = RayCastingMethod::PROPER,
// 	           size_t early_stopping = 0)
// 	    : discretize_(discretize),
// 	      max_range_(max_range),
// 	      hit_depth_(hit_depth),
// 	      miss_depth_(miss_depth),
// 	      ray_casting_method_(ray_casting_method),
// 	      early_stopping_(early_stopping),
// 	      weighted_(weighted)
// 	{
// 	}

// 	//
// 	// Getters
// 	//

// 	constexpr bool isDiscretize() const noexcept { return discretize_; }

// 	constexpr float maxRange() const noexcept { return max_range_; }

// 	constexpr Depth hitDepth() const noexcept { return hit_depth_; }

// 	constexpr Depth missDepth() const noexcept { return miss_depth_; }

// 	constexpr RayCastingMethod rayCastingMethod() const noexcept
// 	{
// 		return ray_casting_method_;
// 	}

// 	constexpr size_t earlyStopping() const noexcept { return early_stopping_; }

// 	constexpr bool isWeightedEnabled() const noexcept { return weighted_; }

// 	constexpr float occupancyProbHit() const noexcept { return occupancy_prob_hit_; }

// 	constexpr float ocupancyProbMiss() const noexcept { return occupancy_prob_miss_; }

// 	constexpr SemanticValue semanticValueHit() const noexcept
// 	{
// 		return semantic_value_hit_;
// 	}

// 	constexpr SemanticValue semanticValueMiss() const noexcept
// 	{
// 		return semantic_value_miss_;
// 	}

// 	//
// 	// Setters
// 	//

// 	constexpr void setDiscretize(bool discretize) noexcept { discretize_ = discretize; }

// 	constexpr void setMaxRange(float max_range) noexcept { max_range_ = max_range; }

// 	constexpr void setHitDepth(Depth depth) noexcept { hit_depth_ = depth; }

// 	constexpr void setMissDepth(Depth depth) noexcept { miss_depth_ = depth; }

// 	constexpr void setRayCastingMethod(RayCastingMethod method) noexcept
// 	{
// 		ray_casting_method_ = method;
// 	}

// 	constexpr void setEarlyStopping(size_t early_stopping) noexcept
// 	{
// 		early_stopping_ = early_stopping;
// 	}

// 	constexpr void setWeighted(bool weighted) noexcept { weighted_ = weighted; }

// 	constexpr void setOccupancyProbHit(float prob_hit) noexcept
// 	{
// 		occupancy_prob_hit_ = prob_hit;
// 	}

// 	constexpr void setOccupancyProbMiss(float prob_miss) noexcept
// 	{
// 		occupancy_prob_miss_ = prob_miss;
// 	}

// 	constexpr void getSemanticValueHit(SemanticValue value_hit) noexcept
// 	{
// 		semantic_value_hit_ = value_hit;
// 	}

// 	constexpr void getSemanticValueMiss(SemanticValue value_miss) noexcept
// 	{
// 		semantic_value_miss_ = value_miss;
// 	}

// 	//
// 	// Insert point cloud
// 	//

// 	template <class Map, class P>
// 	void insertPointCloud(Map& map, Point3 sensor_origin, PointCloudT<P> const& cloud,
// 	                      bool insert_misses = true, bool propagate = true)
// 	{
// 		insertPointCloud(std::execution::seq, map, sensor_origin, cloud, insert_misses,
// 		                 propagate);
// 	}

// 	template <class ExecutionPolicy, class Map, class P,
// 	          typename = std::enable_if_t<
// 	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
// 	void insertPointCloud(ExecutionPolicy policy, Map& map, Point3 sensor_origin,
// 	                      PointCloudT<P> const& cloud, bool insert_misses = true,
// 	                      bool propagate = true)
// 	{
// 		std::thread insert_thread([this]() {
// 			if (async_handler_.valid()) {
// 				async_handler_.get();
// 			}
// 		});

// 		IntegratorPointCloud<P> integrator_cloud =
// 		    createIntegratorPointCloud(policy, map, sensor_origin, cloud);

// 		if (insert_misses) {
// 			MissedSpace free_space = getMisses(policy, map, integrator_cloud);

// 			insert_thread.join();

// 			async_handler_ = std::async(std::launch::deferred, [this, policy, &map,
// 			                                                    integrator_cloud =
// 			                                                        std::move(integrator_cloud),
// 			                                                    free_space =
// 			                                                        std::move(free_space),
// 			                                                    propagate]() {
// 				// Insert free space
// 				integrateMisses(policy, map, free_space, miss_depth_);

// 				// Insert occupied space
// 				integrateHits(policy, map, integrator_cloud, hit_depth_);

// 				// Update inner nodes
// 				if (propagate) {
// 					map.updateModifiedNodes(policy);
// 				}

// 				// Increase time step
// 				if constexpr (is_base_of_template_v<OccupancyMapTimeBase, std::decay_t<Map>>) {
// 					if (auto_inc_time_step_) {
// 						current_time_step_ += time_step_inc_;
// 					}
// 				}
// 			});

// 			// Insert free space
// 			// integrateMisses(policy, map, free_space, miss_depth_);
// 		}

// 		// Insert occupied space
// 		// integrateHits(policy, map, integrator_cloud, hit_depth_);

// 		// // Update inner nodes
// 		// if (propagate) {
// 		// 	map.updateModifiedNodes(policy);
// 		// }

// 		// // Increase time step
// 		// if constexpr (is_base_of_template_v<OccupancyMapTimeBaseBase,
// 		//                                               std::decay_t<Map>>) {
// 		// 	if (map.isAutomaticTimeStepEnabled()) {
// 		// 		++map.getCurrentTimeStep();
// 		// 	}
// 		// }
// 	}

// 	template <class ExecutionPolicy, class PostFunction, class Map, class P,
// 	          typename = std::enable_if_t<
// 	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
// 	void insertPointCloud(ExecutionPolicy policy, PostFunction f, Map& map,
// 	                      Point3 sensor_origin, PointCloudT<P> const& cloud,
// 	                      bool insert_misses = true, bool propagate = true)
// 	{
// 		IntegratorPointCloud<P> integrator_cloud =
// 		    createIntegratorPointCloud(policy, map, sensor_origin, cloud);

// 		if (insert_misses) {
// 			MissedSpace free_space = getMisses(policy, map, integrator_cloud);

// 			if (async_handler_.valid()) {
// 				async_handler_.get();
// 			}

// 			async_handler_ = std::async(std::launch::async, [this, policy, f, &map,
// 			                                                 integrator_cloud =
// 			                                                     std::move(integrator_cloud),
// 			                                                 free_space =
// std::move(free_space), propagate]()
// {
// 				// Insert free space
// 				integrateMisses(policy, map, free_space, miss_depth_);

// 				// Insert occupied space
// 				integrateHits(policy, map, integrator_cloud, hit_depth_);

// 				// Update inner nodes
// 				if (propagate) {
// 					map.updateModifiedNodes(policy);
// 				}

// 				// Increase time step
// 				if constexpr (is_base_of_template_v<OccupancyMapTimeBase, std::decay_t<Map>>) {
// 					if (auto_inc_time_step_) {
// 						current_time_step_ += time_step_inc_;
// 					}
// 				}

// 				// Call post function
// 				f();
// 			});
// 		} else {
// 			if (async_handler_.valid()) {
// 				async_handler_.get();
// 			}

// 			async_handler_ = std::async(std::launch::async, [this, policy, f, &map,
// 			                                                 integrator_cloud =
// 			                                                     std::move(integrator_cloud),
// 			                                                 propagate]() {
// 				// Insert occupied space
// 				integrateHits(policy, map, integrator_cloud, hit_depth_);

// 				// Update inner nodes
// 				if (propagate) {
// 					map.updateModifiedNodes(policy);
// 				}

// 				// Increase time step
// 				if constexpr (is_base_of_template_v<OccupancyMapTimeBase, std::decay_t<Map>>) {
// 					if (auto_inc_time_step_) {
// 						current_time_step_ += time_step_inc_;
// 					}
// 				}

// 				// Call post function
// 				f();
// 			});
// 		}
// 	}

// 	template <class PostFunction, class Map, class P>
// 	std::pair<IntegratorPointCloud<P>, MissedSpace> insertPointCloud2(
// 	    PostFunction f, Map& map, Point3 sensor_origin, PointCloudT<P> const& cloud,
// 	    bool insert_misses = true, bool propagate = true)
// 	{
// 		IntegratorPointCloud<P> integrator_cloud =
// 		    createIntegratorPointCloud(map, sensor_origin, cloud);

// 		if (insert_misses) {
// 			MissedSpace free_space = getMisses(map, integrator_cloud);

// 			if (async_handler_.valid()) {
// 				async_handler_.get();
// 			}

// 			async_handler_ = std::async(std::launch::async, [this, f, &map, integrator_cloud,
// 			                                                 free_space, propagate]() {
// 				// Insert free space
// 				integrateMisses(map, free_space, miss_depth_);

// 				// Insert occupied space
// 				integrateHits(map, integrator_cloud, hit_depth_);

// 				// Update inner nodes
// 				if (propagate) {
// 					map.updateModifiedNodes();
// 				}

// 				// Increase time step
// 				if constexpr (is_base_of_template_v<OccupancyMapTimeBase, std::decay_t<Map>>) {
// 					if (auto_inc_time_step_) {
// 						current_time_step_ += time_step_inc_;
// 					}
// 				}

// 				// Call post function
// 				f();
// 			});

// 			return {integrator_cloud, free_space};
// 		} else {
// 			if (async_handler_.valid()) {
// 				async_handler_.get();
// 			}

// 			async_handler_ = std::async(std::launch::async, [this, f, &map, integrator_cloud,
// 			                                                 propagate]() {
// 				// Insert occupied space
// 				integrateHits(map, integrator_cloud, hit_depth_);

// 				// Update inner nodes
// 				if (propagate) {
// 					map.updateModifiedNodes();
// 				}

// 				// Increase time step
// 				if constexpr (is_base_of_template_v<OccupancyMapTimeBase, std::decay_t<Map>>) {
// 					if (auto_inc_time_step_) {
// 						current_time_step_ += time_step_inc_;
// 					}
// 				}

// 				// Call post function
// 				f();
// 			});

// 			return {integrator_cloud, {}};
// 		}
// 	}

// 	template <class ExecutionPolicy, class PostFunction, class Map, class P,
// 	          typename = std::enable_if_t<
// 	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
// 	std::pair<IntegratorPointCloud<P>, MissedSpace> insertPointCloud2(
// 	    ExecutionPolicy policy, PostFunction f, Map& map, Point3 sensor_origin,
// 	    PointCloudT<P> const& cloud, bool insert_misses = true, bool propagate = true)
// 	{
// 		IntegratorPointCloud<P> integrator_cloud =
// 		    createIntegratorPointCloud(policy, map, sensor_origin, cloud);

// 		if (insert_misses) {
// 			MissedSpace free_space = getMisses(policy, map, integrator_cloud);

// 			if (async_handler_.valid()) {
// 				async_handler_.get();
// 			}

// 			async_handler_ = std::async(std::launch::async, [this, policy, f, &map,
// 			                                                 integrator_cloud, free_space,
// 			                                                 propagate]() {
// 				// Insert free space
// 				integrateMisses(policy, map, free_space, miss_depth_);

// 				// Insert occupied space
// 				integrateHits(policy, map, integrator_cloud, hit_depth_);

// 				// Update inner nodes
// 				if (propagate) {
// 					map.updateModifiedNodes(policy);
// 				}

// 				// Increase time step
// 				if constexpr (is_base_of_template_v<OccupancyMapTimeBase, std::decay_t<Map>>) {
// 					if (auto_inc_time_step_) {
// 						current_time_step_ += time_step_inc_;
// 					}
// 				}

// 				// Call post function
// 				f();
// 			});

// 			return {integrator_cloud, free_space};
// 		} else {
// 			if (async_handler_.valid()) {
// 				async_handler_.get();
// 			}

// 			async_handler_ = std::async(std::launch::async, [this, policy, f, &map,
// 			                                                 integrator_cloud, propagate]() {
// 				// Insert occupied space
// 				integrateHits(policy, map, integrator_cloud, hit_depth_);

// 				// Update inner nodes
// 				if (propagate) {
// 					map.updateModifiedNodes(policy);
// 				}

// 				// Increase time step
// 				if constexpr (is_base_of_template_v<OccupancyMapTimeBase, std::decay_t<Map>>) {
// 					if (auto_inc_time_step_) {
// 						current_time_step_ += time_step_inc_;
// 					}
// 				}

// 				// Call post function
// 				f();
// 			});

// 			return {integrator_cloud, {}};
// 		}
// 	}

// 	template <class PostFunction, class Map, class P>
// 	void insertPointCloud3(util::Timing& timing, PostFunction f, Map& map,
// 	                       Point3 sensor_origin, PointCloudT<P> const& cloud,
// 	                       bool insert_misses = true, bool propagate = true)
// 	{
// 		// timing.start("1.1          ");
// 		IntegratorPointCloud2<P> integrator_cloud =
// 		    createIntegratorPointCloud2(map, sensor_origin, cloud);
// 		// timing.stop("1.1          ");

// 		// std::sort(std::begin(integrator_cloud), std::end(integrator_cloud),
// 		//           [](auto const& a, auto const& b) { return a.code < b.code; });

// 		if (insert_misses) {
// 			// timing.start("1.2          ");
// 			// constexpr Depth depth = 4;

// 			// CodeMap<std::vector<Code>> something;
// 			// Code prev_code;
// 			// for (auto const& point : integrator_cloud) {
// 			// 	Code code_at_depth = point.code.toDepth(miss_depth_);
// 			// 	if (code_at_depth == prev_code) {
// 			// 		continue;
// 			// 	}
// 			// 	prev_code = code_at_depth;

// 			// 	something[point.code.toDepth(miss_depth_ + depth)].push_back(code_at_depth);
// 			// }

// 			// auto comp = [](std::pair<Code, float> const& a, std::pair<Code, float> const&
// b)
// 			// { 	return a.second > b.second;
// 			// };
// 			// std::set<std::pair<Code, float>, decltype(comp)> code_distance(comp);

// 			// for (auto const& [code, _] : something) {
// 			// 	code_distance.emplace(code, (map.toCoord(code) - sensor_origin).norm());
// 			// }

// 			// Grid<depth> grid;
// 			// free_space_.clear();

// 			// Key from_key = map.toKey(sensor_origin, miss_depth_);
// 			// Point3 voxel_border = map.toCoord(from_key) - sensor_origin;
// 			// for (auto it = std::begin(code_distance); std::end(code_distance) != it; ++it)
// {
// 			// 	auto code = it->first;

// 			// 	std::sort(std::begin(something[code]), std::end(something[code]));

// 			// 	Code prev_code;
// 			// 	for (auto cur_code : something[code]) {
// 			// 		if (cur_code == prev_code) {
// 			// 			continue;
// 			// 		}
// 			// 		prev_code = cur_code;

// 			// 		Key to_key = cur_code.toKey();
// 			// 		Point3 point = map.toCoord(to_key);

// 			// 		RayCaster ray_caster(to_key, from_key, point, sensor_origin, voxel_border,
// 			// 		                     map.nodeSize(miss_depth_));

// 			// 		// if (!ray_caster.hasLeft()) {
// 			// 		// 	// FIXME: Add current
// 			// 		// 	continue;
// 			// 		// }

// 			// 		while (ray_caster.hasLeft()) {
// 			// 			Code asd = ray_caster.getCurrent();
// 			// 			Code current_code = asd.toDepth(code.depth());
// 			// 			if (code != current_code) {
// 			// 				// Outside
// 			// 				auto& data = something[current_code];
// 			// 				if (data.empty()) {
// 			// 					float distance = (map.toCoord(current_code) - sensor_origin).norm();
// 			// 					code_distance.emplace(current_code, distance);
// 			// 				}
// 			// 				data.push_back(asd);
// 			// 				break;
// 			// 			}
// 			// 			auto index = ray_caster.getCurrentKey();
// 			// 			if (grid[index]) {
// 			// 				break;
// 			// 			}
// 			// 			grid.set(index);
// 			// 			ray_caster.takeStep();
// 			// 		}

// 			// 		// FIXME: Make sure it does not overwrite occupied
// 			// 	}

// 			// 	for (std::size_t i = 0; grid.size() != i; ++i) {
// 			// 		if (grid[i]) {
// 			// 			Key key = grid.getKey(i, miss_depth_);
// 			// 			free_space_.emplace_back(
// 			// 			    Code(code.getCode() | Code::toCode(key), miss_depth_), 1.0f);
// 			// 		}
// 			// 	}

// 			// 	grid.clear();
// 			// }
// 			// timing.stop("1.2          ");

// 			// timing.start("1.3          ");
// 			// integrateMisses(map, free_space_, miss_depth_);
// 			// timing.stop("1.3          ");

// 			// timing.start("1.2          ");
// 			// MissedSpace free_space = getMisses(map, sensor_origin, integrator_cloud);
// 			getMisses(map, sensor_origin, integrator_cloud);
// 			// timing.stop("1.2          ");

// 			// timing.start("1.3          ");
// 			integrateMisses(map, free_space_, miss_depth_);
// 			// timing.stop("1.3          ");
// 		}

// 		// timing.start("1.4          ");
// 		// Insert occupied space
// 		integrateHits(map, integrator_cloud, hit_depth_);
// 		// timing.stop("1.4          ");

// 		// Update inner nodes
// 		if (propagate) {
// 			// timing.start("1.5          ");
// 			map.updateModifiedNodes();
// 			// timing.stop("1.5          ");
// 		}

// 		// Increase time step
// 		if constexpr (is_base_of_template_v<OccupancyMapTimeBase, std::decay_t<Map>>) {
// 			// timing.start("1.6          ");
// 			if (auto_inc_time_step_) {
// 				current_time_step_ += time_step_inc_;
// 			}
// 			// timing.stop("1.6          ");
// 		}

// 		// timing.start("1.7          ");
// 		// Call post function
// 		f();
// 		// timing.stop("1.7          ");
// 	}

// 	// TODO: Remove?
// 	void wait() const
// 	{
// 		if (async_handler_.valid()) {
// 			async_handler_.wait();
// 		}
// 	}

// 	template <class Map, class P>
// 	void insertPointCloud(Map& map, Point3 sensor_origin, PointCloudT<P> const& cloud,
// 	                      math::Pose6f const& frame_origin, bool insert_misses = true,
// 	                      bool propagate = true)
// 	{
// 		insertPointCloud(std::execution::seq, map, sensor_origin, cloud, frame_origin,
// 		                 insert_misses, propagate);
// 	}

// 	template <class ExecutionPolicy, class Map, class P,
// 	          typename = std::enable_if_t<
// 	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
// 	void insertPointCloud(ExecutionPolicy policy, Map& map, Point3 sensor_origin,
// 	                      PointCloudT<P> cloud, math::Pose6f const& frame_origin,
// 	                      bool insert_misses = true, bool propagate = true)
// 	{
// 		cloud.transform(policy, frame_origin);
// 		insertPointCloud(policy, map, sensor_origin, cloud, insert_misses, propagate);
// 	}

//  protected:
// 	//
// 	// Integrate misses
// 	//

// 	template <class Map>
// 	void integrateMisses(Map& map, MissedSpace free_space, Depth depth) const
// 	{
// 		using OccupancyLogitType = typename Map::LogitType;

// 		OccupancyLogitType occupancy_prob_miss_log;
// 		if constexpr (std::is_same_v<OccupancyLogitType, uint8_t>) {
// 			occupancy_prob_miss_log = math::logitChangeValue<uint8_t>(
// 			    occupancy_prob_miss_, map.getOccupancyClampingThresMinLogit(),
// 			    map.getOccupancyClampingThresMaxLogit());
// 		} else {
// 			occupancy_prob_miss_log = math::logit(occupancy_prob_miss_);
// 		}

// 		float temp = occupancy_prob_miss_log / ((2.0 * depth) + 1.0);
// 		OccupancyLogitType value = temp;
// 		if constexpr (std::is_same_v<OccupancyLogitType, uint8_t>) {
// 			if (0 == value && 0 != occupancy_prob_miss_log) {
// 				value = 1;
// 			}
// 		}

// 		// Code prev_code;

// 		if (true) {
// 			if (weighted_) {
// 				for (auto const& [code, distance] : free_space) {
// 					Node node = map.createNode(code);

// 					if (weighted_) {
// 						OccupancyLogitType v = value / (distance * distance);

// 						if constexpr (std::is_same_v<OccupancyLogitType, uint8_t>) {
// 							if (0 == v && 0 != value) {
// 								v = 1;
// 							}
// 						}
// 						map.decreaseOccupancyLogit(node, v, false);
// 					} else {
// 						map.decreaseOccupancyLogit(node, value, false);
// 					}
// 					if constexpr (is_base_of_template_v<OccupancyMapTimeBase, std::decay_t<Map>>)
// { 						map.setTimeStep(node, current_time_step_, false);
// 					}
// 				}
// 			} else {
// 				for (auto const& [code, _] : free_space) {
// 					Node node = map.createNode(code);
// 					map.decreaseOccupancyLogit(node, value, false);
// 					if constexpr (is_base_of_template_v<OccupancyMapTimeBase, std::decay_t<Map>>)
// { 						map.setTimeStep(node, current_time_step_, false);
// 					}
// 				}
// 			}
// 		} else {
// 			// FIXME: Check so implemented correctly
// 			for (auto const& [code, distance] : free_space) {
// 				// if (code == prev_code) {
// 				// 	continue;
// 				// }
// 				// prev_code = code;

// 				Node node = map.createNode(code);

// 				// FIXME: What execution policy?
// 				if (weighted_) {
// 					float temp = value / std::pow(distance, 2.0);
// 					OccupancyLogitType v = temp;

// 					if constexpr (std::is_same_v<OccupancyLogitType, uint8_t>) {
// 						// FIXME: Make sure it does not get to 0 if it is not 0.5
// 						if (0 == v && 0 != value) {
// 							v = 1;
// 						}
// 					}
// 					map.decreaseOccupancyLogit(node, v, false);
// 				} else {
// 					map.decreaseOccupancyLogit(node, value, false);
// 				}
// 				if constexpr (is_base_of_template_v<OccupancyMapTimeBase, std::decay_t<Map>>) {
// 					map.setTimeStep(node, current_time_step_, false);
// 				}
// 			}
// 		}
// 	}

// 	template <class ExecutionPolicy, class Map,
// 	          typename = std::enable_if_t<
// 	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
// 	void integrateMisses(ExecutionPolicy policy, Map& map, MissedSpace free_space,
// 	                     Depth depth) const
// 	{
// 		using OccupancyLogitType = typename Map::LogitType;

// 		OccupancyLogitType occupancy_prob_miss_log;
// 		if constexpr (std::is_same_v<OccupancyLogitType, uint8_t>) {
// 			occupancy_prob_miss_log = math::logitChangeValue<uint8_t>(
// 			    occupancy_prob_miss_, map.getOccupancyClampingThresMinLogit(),
// 			    map.getOccupancyClampingThresMaxLogit());
// 		} else {
// 			occupancy_prob_miss_log = math::logit(occupancy_prob_miss_);
// 		}

// 		float temp = occupancy_prob_miss_log / float((2.0 * depth) + 1);
// 		OccupancyLogitType value = temp;
// 		if constexpr (std::is_same_v<OccupancyLogitType, uint8_t>) {
// 			// FIXME: Make sure it does not get to 0 if it is not 0.5
// 			if (0 == value && 0 != occupancy_prob_miss_log) {
// 				value = 1;
// 			}
// 		}

// 		// FIXME: Check so implemented correctly
// 		std::for_each(policy, free_space.cbegin(), free_space.cend(), [&](auto const& elem)
// { 			Node node = map.createNode(elem.first);

// 			// FIXME: What execution policy?
// 			if (weighted_) {
// 				float temp = value / std::pow(elem.second, 2.0);
// 				OccupancyLogitType v = temp;

// 				if constexpr (std::is_same_v<OccupancyLogitType, uint8_t>) {
// 					// FIXME: Make sure it does not get to 0 if it is not 0.5
// 					if (0 == v && 0 != value) {
// 						v = 1;
// 					}
// 				}
// 				map.decreaseOccupancyLogit(node, v, false);
// 			} else {
// 				map.decreaseOccupancyLogit(node, value, false);
// 			}
// 			if constexpr (is_base_of_template_v<OccupancyMapTimeBase, std::decay_t<Map>>) {
// 				map.setTimeStep(node, current_time_step_, false);
// 			}
// 		});
// 	}

// 	//
// 	// Integrate hits
// 	//

// 	template <class Map, class T>
// 	void integrateHits(Map& map, IntegratorPointCloud2<T> const& cloud, Depth depth) const
// 	{
// 		using OccupancyLogitType = typename Map::LogitType;

// 		OccupancyLogitType occupancy_prob_hit_log;
// 		if constexpr (std::is_same_v<OccupancyLogitType, uint8_t>) {
// 			occupancy_prob_hit_log = math::logitChangeValue<uint8_t>(
// 			    occupancy_prob_hit_, map.getOccupancyClampingThresMinLogit(),
// 			    map.getOccupancyClampingThresMaxLogit());
// 		} else {
// 			occupancy_prob_hit_log = math::logit(occupancy_prob_hit_);
// 		}

// 		std::vector<std::tuple<Code, std::vector<T>, float>> discrete_cloud;
// 		auto it = std::begin(cloud);
// 		auto prev_it = it;
// 		while (std::end(cloud) != it) {
// 			Code code = it->code.toDepth(depth);
// 			it = std::find_if_not(it, cloud.end(), [&code, depth](auto const& elem) {
// 				return elem.code.toDepth(depth) == code;
// 			});
// 			std::vector<T> points;
// 			points.reserve(std::distance(prev_it, it));
// 			float distance = std::numeric_limits<float>::max();
// 			while (prev_it != it) {
// 				if (!prev_it->has_moved && 0 <= prev_it->distance) {
// 					points.push_back(prev_it->to);
// 					distance = std::min(distance, prev_it->distance);
// 				}
// 				++prev_it;
// 			}
// 			if (!points.empty()) {
// 				discrete_cloud.emplace_back(code, std::move(points), distance);
// 			}
// 		}

// 		for (auto const& [code, points, distance] : discrete_cloud) {
// 			auto value = occupancy_prob_hit_log;
// 			if (weighted_) {
// 				value /= distance * distance;
// 				if constexpr (std::is_same_v<OccupancyLogitType, uint8_t>) {
// 					if (0 == value && 0 != occupancy_prob_hit_log) {
// 						value = 1;
// 					}
// 				}
// 			}

// 			auto node = map.createNode(code);

// 			OccupancyLogitType logit = map.getOccupancyLogit(node);

// 			map.increaseOccupancyLogit(node, value, false);

// 			if constexpr (is_base_of_template_v<OccupancyMapTimeBase, std::decay_t<Map>>) {
// 				map.setTimeStep(node, current_time_step_, false);
// 			}
// 			if constexpr (std::is_base_of_v<RGBColor, T> &&
// 			              is_base_of_template_v<ColorMapBase, std::decay_t<Map>>) {
// 				if constexpr (std::is_base_of_v<SemanticPair, T>) {
// 					static std::unordered_map<uint32_t, RGBColor> kitti_colorMap = {
// 					    {0, {0, 0, 0}},        {1, {0, 0, 255}},      {10, {245, 150, 100}},
// 					    {11, {245, 230, 100}}, {13, {250, 80, 100}},  {15, {150, 60, 30}},
// 					    {16, {255, 0, 0}},     {18, {180, 30, 80}},   {20, {255, 0, 0}},
// 					    {30, {30, 30, 255}},   {31, {200, 40, 255}},  {32, {90, 30, 150}},
// 					    {40, {255, 0, 255}},   {44, {255, 150, 255}}, {48, {75, 0, 75}},
// 					    {49, {75, 0, 175}},    {50, {0, 200, 255}},   {51, {50, 120, 255}},
// 					    {52, {0, 150, 255}},   {60, {170, 255, 150}}, {70, {0, 175, 0}},
// 					    {71, {0, 60, 135}},    {72, {80, 240, 150}},  {80, {150, 240, 255}},
// 					    {81, {0, 0, 255}},     {99, {255, 255, 50}},  {252, {245, 150, 100}},
// 					    {256, {255, 0, 0}},    {253, {200, 40, 255}}, {254, {30, 30, 255}},
// 					    {255, {90, 30, 150}},  {257, {250, 80, 100}}, {258, {180, 30, 80}},
// 					    {259, {255, 0, 0}}};

// 					map.updateColor(node, kitti_colorMap[points[0].getLabel()], 1.0, false);
// 				} else {
// 					double total_logit = logit + value;
// 					double weight = value / total_logit;
// 					std::vector<RGBColor> colors(std::cbegin(points), std::cend(points));
// 					RGBColor avg_color = RGBColor::average(std::cbegin(colors),
// std::cend(colors)); 					if (avg_color.set()) { 						map.updateColor(node,
// avg_color, weight, false);
// 					}
// 				}
// 			}
// 			if constexpr (is_base_of_template_v<SemanticMapBase, std::decay_t<Map>>) {
// 				if constexpr (std::is_base_of_v<SemanticPair, T>) {
// 					SemanticValue inc_and_dec = semantic_value_hit_ + semantic_value_miss_;
// 					std::vector<SemanticPair> semantics(points.size());
// 					for (size_t i = 0; semantics.size() != i; ++i) {
// 						if (0 != points[i].getLabel()) {
// 							semantics[i] = points[i];
// 						}
// 					}

// 					// Decrease all
// 					map.decreaseSemantic(node, semantic_value_miss_, false);

// 					// Incrase hits
// 					map.increaseSemantics(node, semantics.cbegin(), semantics.cend(),
// 					                      semantic_value_hit_, false);

// 					// Filter 0
// 					// map.filterSematicsLogit(code, 0, false);
// 				}
// 			}
// 		}
// 	}

// 	template <class Map, class P>
// 	void integrateHits(Map& map, IntegratorPointCloud<P> const& cloud, Depth depth) const
// 	{
// 		using OccupancyLogitType = typename Map::LogitType;

// 		OccupancyLogitType occupancy_prob_hit_log;
// 		if constexpr (std::is_same_v<OccupancyLogitType, uint8_t>) {
// 			occupancy_prob_hit_log = math::logitChangeValue<uint8_t>(
// 			    occupancy_prob_hit_, map.getOccupancyClampingThresMinLogit(),
// 			    map.getOccupancyClampingThresMaxLogit());
// 		} else {
// 			occupancy_prob_hit_log = math::logit(occupancy_prob_hit_);
// 		}

// 		std::vector<std::tuple<Code, std::vector<P>, float>> discrete_cloud;
// 		auto it = cloud.begin();
// 		auto prev_it = it;
// 		while (cloud.end() != it) {
// 			Code code = it->code.toDepth(depth);
// 			it = std::find_if_not(it, cloud.end(), [&code, depth](auto const& elem) {
// 				return elem.code.toDepth(depth) == code;
// 			});
// 			std::vector<P> points;
// 			points.reserve(std::distance(prev_it, it));
// 			float distance = std::numeric_limits<float>::max();
// 			while (prev_it != it) {
// 				if (!prev_it->has_moved && 0 <= prev_it->distance) {
// 					points.push_back(prev_it->to);
// 					distance = std::min(distance, prev_it->distance);
// 				}
// 				++prev_it;
// 			}
// 			if (!points.empty()) {
// 				discrete_cloud.emplace_back(code, std::move(points), distance);
// 			}
// 		}

// 		for (auto const& [code, points, distance] : discrete_cloud) {
// 			integrate(map, points, code, distance, depth, occupancy_prob_hit_log);
// 		}
// 	}

// 	template <class ExecutionPolicy, class Map, class P,
// 	          typename = std::enable_if_t<
// 	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
// 	void integrateHits(ExecutionPolicy policy, Map& map,
// 	                   IntegratorPointCloud<P> const& cloud, Depth depth) const
// 	{
// 		using OccupancyLogitType = typename Map::LogitType;

// 		OccupancyLogitType occupancy_prob_hit_log;
// 		if constexpr (std::is_same_v<OccupancyLogitType, uint8_t>) {
// 			occupancy_prob_hit_log = math::logitChangeValue<uint8_t>(
// 			    occupancy_prob_hit_, map.getOccupancyClampingThresMinLogit(),
// 			    map.getOccupancyClampingThresMaxLogit());
// 		} else {
// 			occupancy_prob_hit_log = math::logit(occupancy_prob_hit_);
// 		}

// 		std::vector<std::tuple<Code, std::vector<P>, float>> discrete_cloud;
// 		auto it = cloud.begin();
// 		auto prev_it = it;
// 		while (cloud.end() != it) {
// 			Code code = it->code.toDepth(depth);
// 			it = std::find_if_not(it, cloud.end(), [&code, depth](auto const& elem) {
// 				return elem.code.toDepth(depth) == code;
// 			});
// 			std::vector<P> points;
// 			points.reserve(std::distance(prev_it, it));
// 			float distance = std::numeric_limits<float>::max();
// 			while (prev_it != it) {
// 				if (!prev_it->has_moved && 0 <= prev_it->distance) {
// 					points.push_back(prev_it->to);
// 					distance = std::min(distance, prev_it->distance);
// 				}
// 				++prev_it;
// 			}
// 			if (!points.empty()) {
// 				discrete_cloud.emplace_back(code, std::move(points), distance);
// 			}
// 		}

// 		std::for_each(policy, discrete_cloud.cbegin(), discrete_cloud.cend(),
// 		              [&](auto const& elem) {
// 			              integrate(map, std::get<1>(elem), std::get<0>(elem),
// 			                        std::get<2>(elem), depth, occupancy_prob_hit_log);
// 		              });
// 	}

// 	template <class Map, class T, class OccupancyLogitType>
// 	void integrate(Map& map, std::vector<T> const& points, Code code, float distance,
// 	               Depth depth, OccupancyLogitType occupancy_prob_hit_log) const
// 	{
// 		float temp = occupancy_prob_hit_log;  // TODO: / float((2.0 * depth) + 1);
// 		OccupancyLogitType value = temp;
// 		if constexpr (std::is_same_v<OccupancyLogitType, uint8_t>) {
// 			// FIXME: Make sure it does not get to 0 if it is not 0.5
// 			if (0 == value && 0 != occupancy_prob_hit_log) {
// 				value = 1;
// 			}
// 		}

// 		if (weighted_) {
// 			temp /= std::pow(distance, 2.0);
// 			value = temp;
// 			if constexpr (std::is_same_v<OccupancyLogitType, uint8_t>) {
// 				// FIXME: Make sure it does not get to 0 if it is not 0.5
// 				if (0 == value && 0 != occupancy_prob_hit_log) {
// 					value = 1;
// 				}
// 			}
// 		}

// 		Node node = map.createNode(code);
// 		OccupancyLogitType logit = map.getOccupancyLogit(node);

// 		map.increaseOccupancyLogit(node, value, false);

// 		if constexpr (is_base_of_template_v<OccupancyMapTimeBase, std::decay_t<Map>>) {
// 			map.setTimeStep(node, current_time_step_, false);
// 		}
// 		if constexpr (std::is_base_of_v<RGBColor, T> &&
// 		              is_base_of_template_v<ColorMapBase, std::decay_t<Map>>) {
// 			double total_logit = logit + value;
// 			double weight = value / total_logit;
// 			std::vector<RGBColor> colors(std::begin(points), std::end(points));
// 			RGBColor avg_color = RGBColor::average(std::cbegin(colors), std::cend(colors));
// 			if (avg_color.set()) {
// 				map.updateColor(node, avg_color, weight, false);
// 			}
// 		}
// 		if constexpr (is_base_of_template_v<SemanticMapBase, std::decay_t<Map>>) {
// 			if constexpr (std::is_base_of_v<SemanticPair, T>) {
// 				SemanticValue inc_and_dec = semantic_value_hit_ + semantic_value_miss_;
// 				std::vector<SemanticPair> semantics(points.size());
// 				for (size_t i = 0; semantics.size() != i; ++i) {
// 					if (0 != points[i].getLabel()) {
// 						semantics[i] = points[i];
// 					}
// 				}

// 				// Decrease all
// 				map.decreaseSemantic(node, semantic_value_miss_, false);

// 				// Incrase hits
// 				map.increaseSemantics(node, semantics.cbegin(), semantics.cend(),
// 				                      semantic_value_hit_, false);

// 				// Filter 0
// 				// map.filterSematicsLogit(code, 0, false);
// 			}
// 		}
// 	}

// 	//
// 	// Create integrator point cloud
// 	//

// 	template <class Map, class P>
// 	IntegratorPointCloud2<P> createIntegratorPointCloud2(Map const& map,
// 	                                                     Point3 sensor_origin,
// 	                                                     PointCloudT<P> const& cloud)
// const
// 	{
// 		IntegratorPointCloud2<P> integrator_cloud;
// 		integrator_cloud.reserve(cloud.size());

// 		if (0 > max_range_) {
// 			for (auto const& point : cloud) {
// 				integrator_cloud.emplace_back(point, map.toCode(point), false,
// 				                              (point - sensor_origin).norm());
// 			}
// 		} else {
// 			// TODO: Implement
// 		}

// 		std::sort(std::begin(integrator_cloud), std::end(integrator_cloud),
// 		          [](auto const& a, auto const& b) { return a.code < b.code; });

// 		return integrator_cloud;
// 	}

// 	template <class Map, class P>
// 	IntegratorPointCloud<P> createIntegratorPointCloud(Map const& map, Point3
// sensor_origin, 	                                                   PointCloudT<P>
// const& cloud) const
// 	{
// 		std::vector<Code> codes;
// 		codes.reserve(cloud.size());

// 		for (auto const& point : cloud) {
// 			codes.push_back(map.toCode(point));
// 		}

// 		IntegratorPointCloud<P> integrator_cloud;
// 		integrator_cloud.reserve(cloud.size());

// 		for (size_t i = 0; cloud.size() != i; ++i) {
// 			auto const& point = cloud[i];

// 			Point3 direction = point - sensor_origin;
// 			auto distance = direction.norm();

// 			if (0 > max_range_ || distance < max_range_) {
// 				if (map.isInside(point)) {
// 					// Move origin map
// 					Point3 current = sensor_origin;
// 					Point3 temp = point;
// 					map.moveLineInside(current, temp);  // TODO: Fix
// 					integrator_cloud.emplace_back(current, point, codes[i], distance, false);
// 					continue;
// 				}
// 			}

// 			direction /= distance;
// 			Point3 point_max_range = sensor_origin + (direction * max_range_);

// 			Point3 current = sensor_origin;
// 			// Move origin and end inside map
// 			if (map.moveLineInside(current, point_max_range)) {
// 				// Line inside of map
// 				integrator_cloud.emplace_back(current, point_max_range, codes[i],
// 				                              (point_max_range - sensor_origin).norm(), true);
// 			} else {
// 				// Line outside of map
// 				integrator_cloud.emplace_back(current, point_max_range, codes[i], -1, true);
// 			}
// 		}

// 		std::sort(std::begin(integrator_cloud), std::end(integrator_cloud),
// 		          [](auto const& a, auto const& b) { return a.code < b.code; });

// 		return integrator_cloud;
// 	}

// 	template <class ExecutionPolicy, class Map, class P,
// 	          typename = std::enable_if_t<
// 	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
// 	IntegratorPointCloud<P> createIntegratorPointCloud(ExecutionPolicy policy,
// 	                                                   Map const& map, Point3
// sensor_origin, 	                                                   PointCloudT<P>
// const& cloud)
// 	{
// 		std::vector<size_t> indices(cloud.size());
// 		std::iota(indices.begin(), indices.end(), 0);

// 		IntegratorPointCloud<P> integrator_cloud(cloud.size());

// 		std::for_each(policy, indices.begin(), indices.end(), [&](size_t index) {
// 			Point3 direction = cloud[index] - sensor_origin;
// 			float distance = direction.norm();

// 			if (0 > max_range_ || distance < max_range_) {
// 				if (map.isInside(cloud[index])) {
// 					// Move origin map
// 					Point3 current = sensor_origin;
// 					Point3 temp = cloud[index];
// 					map.moveLineInside(current, temp);
// 					integrator_cloud[index].from = current;
// 					integrator_cloud[index].to = cloud[index];
// 					integrator_cloud[index].code = map.toCode(cloud[index]);
// 					integrator_cloud[index].distance = distance;
// 					integrator_cloud[index].has_moved = false;
// 					return;
// 				}
// 			}

// 			integrator_cloud[index].has_moved = true;

// 			direction /= distance;
// 			Point3 point_max_range = sensor_origin + (direction * max_range_);

// 			Point3 current = sensor_origin;
// 			// Move origin and end inside map
// 			if (!map.moveLineInside(current, point_max_range)) {
// 				// Line outside of map
// 				integrator_cloud[index].distance = -1;
// 				return;
// 			}

// 			integrator_cloud[index].from = current;
// 			integrator_cloud[index].to = point_max_range;
// 			integrator_cloud[index].code = map.toCode(point_max_range);
// 			integrator_cloud[index].distance = (point_max_range - sensor_origin).norm();
// 		});

// 		std::sort(policy, integrator_cloud.begin(), integrator_cloud.end(),
// 		          [](auto const& a, auto const& b) { return a.code < b.code; });

// 		return integrator_cloud;
// 	}

// 	//
// 	// Free space
// 	//

// 	template <class Map, class T>
// 	void getMisses(Map const& map, Point3 sensor_origin,
// 	               IntegratorPointCloud2<T> const& cloud)
// 	{
// 		free_space_.clear();

// 		// The space that should be freed
// 		getMissesGetIndices(map, sensor_origin, std::begin(cloud), std::end(cloud));

// 		// std::cerr << free_space_.size() << '\n';

// 		// free_space.reserve(indices.size());
// 		// for (auto const& elem : indices) {
// 		// 	free_space.push_back(elem);
// 		// }

// 		// Prune free space
// 		// std::sort(std::begin(free_space_), std::end(free_space_),
// 		//           [](auto const& a, auto const& b) { return a.first < b.first; });
// 		// free_space.erase(
// 		//     std::unique(std::begin(free_space), std::end(free_space),
// 		//                 [](auto const& a, auto const& b) { return a.first == b.first; }),
// 		//     end(free_space));

// 		// pruneMissedSpace(free_space);

// 		// return free_space;
// 	}

// 	template <class Map, class P>
// 	MissedSpace getMisses(Map const& map, IntegratorPointCloud<P> const& cloud)
// 	{
// 		// The space that should be freed
// 		MissedSpace free_space;

// 		CodeMap<float> indices = getMissesGetIndices(map, std::begin(cloud),
// std::end(cloud));

// 		free_space.reserve(indices.size());
// 		for (auto const& elem : indices) {
// 			free_space.push_back(elem);
// 		}

// 		// Prune free space
// 		// std::sort(std::begin(free_space), std::end(free_space),
// 		//           [](auto const& a, auto const& b) { return a.first < b.first; });
// 		// free_space.erase(
// 		//     std::unique(std::begin(free_space), std::end(free_space),
// 		//                 [](auto const& a, auto const& b) { return a.first == b.first; }),
// 		//     end(free_space));

// 		// pruneMissedSpace(free_space);

// 		return free_space;
// 	}

// 	template <class ExecutionPolicy, class Map, class P,
// 	          typename = std::enable_if_t<
// 	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
// 	MissedSpace getMisses(ExecutionPolicy policy, Map const& map,
// 	                      IntegratorPointCloud<P> const& cloud)
// 	{
// 		// The space that should be freed
// 		MissedSpace free_space;

// 		if constexpr (std::is_same_v<std::execution::parallel_policy, ExecutionPolicy> ||
// 		              std::is_same_v<std::execution::parallel_unsequenced_policy,
// 		                             ExecutionPolicy>) {
// 			//  Parallel

// 			std::vector<size_t> par(
// 			    std::min(8u, std::max(1u, std::thread::hardware_concurrency())));
// 			std::iota(std::begin(par), std::end(par), static_cast<size_t>(0));

// 			std::vector<std::vector<std::pair<Code, float>>> codes_maps(par.size());

// 			size_t length = cloud.size() / par.size();
// 			size_t remain = cloud.size() % par.size();

// 			std::for_each(policy, std::begin(par), std::end(par), [&](size_t index) {
// 				size_t start = length * index;
// 				size_t stop = length * (index + 1);
// 				if (par.back() == index) {
// 					stop += remain;
// 				}

// 				CodeMap<float> indices = getMissesGetIndices(
// 				    map, std::next(std::begin(cloud), start), std::next(std::begin(cloud),
// stop));

// 				codes_maps[index].reserve(indices.size());
// 				for (auto b_it = indices.beginBuckets(), b_end_it = indices.endBuckets();
// 				     b_it != b_end_it; ++b_it) {
// 					codes_maps[index].insert(std::end(codes_maps[index]),
// 					                         std::begin(*std::as_const(b_it)),
// 					                         std::end(*std::as_const(b_it)));
// 				}

// 				// codes_maps[index].insert(begin(codes_maps[index]), indices.begin(),
// 				//                          indices.end());
// 			});

// 			size_t max_size =
// 			    std::transform_reduce(std::begin(codes_maps), std::end(codes_maps), 0,
// 			                          std::plus<>(), [](auto const& c) { return c.size(); });

// 			free_space.resize(max_size);

// 			auto it = std::begin(free_space);
// 			for (auto const& c : codes_maps) {
// 				it = std::copy(policy, std::begin(c), std::end(c), it);
// 			}
// 		} else {
// 			// Sequential

// 			CodeMap<float> indices =
// 			    getMissesGetIndices(map, std::begin(cloud), std::end(cloud));

// 			free_space.reserve(indices.size());
// 			for (auto const& elem : indices) {
// 				free_space.push_back(elem);
// 			}
// 		}

// 		// Prune free space
// 		std::sort(policy, std::begin(free_space), std::end(free_space),
// 		          [](auto const& a, auto const& b) { return a.first < b.first; });
// 		free_space.erase(
// 		    std::unique(policy, std::begin(free_space), std::end(free_space),
// 		                [](auto const& a, auto const& b) { return a.first == b.first; }),
// 		    end(free_space));

// 		// pruneMissedSpace(policy, free_space);

// 		return free_space;
// 	}

// 	template <class Map, class InputIt>
// 	CodeMap<float> getMissesGetIndices(Map const& map, InputIt first, InputIt last) const
// 	{
// 		// Set up parameters that will be used
// 		Depth depth = miss_depth_;
// 		size_t early_stopping = early_stopping_;
// 		RayCastingMethod ray_casting_method = ray_casting_method_;
// 		bool discretize = discretize_ || 0 != depth;

// 		// The indices
// 		CodeMap<float> indices;

// 		Code prev_code;
// 		for (; first != last; ++first) {
// 			Code code_at_depth = first->code.toDepth(depth);
// 			if (0 > first->distance || (discretize && code_at_depth == prev_code)) {
// 				continue;
// 			}

// 			prev_code = code_at_depth;

// 			Point3 end =
// 			    discretize ? map.toCoord(code_at_depth) : static_cast<Point3
// const&>(first->to);

// 			switch (ray_casting_method) {
// 				case RayCastingMethod::PROPER:
// 					getMissesProper(map, first->from, end, indices, depth, early_stopping);
// 					break;
// 				case RayCastingMethod::SIMPLE:
// 					getMissesSimple(map, first->from, end, indices, depth, early_stopping);
// 					break;
// 			}
// 		}

// 		return indices;
// 	}

// 	template <class Map, class InputIt>
// 	void getMissesGetIndices(Map const& map, Point3 sensor_origin, InputIt first,
// 	                         InputIt last)
// 	{
// 		// Set up parameters that will be used
// 		Depth depth = miss_depth_;
// 		size_t early_stopping = early_stopping_;
// 		RayCastingMethod ray_casting_method = ray_casting_method_;
// 		bool discretize = discretize_ || 0 != depth;

// 		// The indices
// 		MissedSpace temp;

// 		Key sensor_origin_key = map.toKey(sensor_origin, depth);

// 		Code prev_code;
// 		for (; first != last; ++first) {
// 			Code code_at_depth = first->code.toDepth(depth);
// 			if (code_at_depth == prev_code) {
// 				continue;
// 			}

// 			prev_code = code_at_depth;

// 			getMissesProper(map, sensor_origin_key, code_at_depth, sensor_origin, first->to,
// 			                temp, depth);

// 			for (auto const& [code, distance] : temp) {
// 				if (indices_.insert(code).second) {
// 					free_space_.emplace_back(code, distance);
// 				}
// 			}

// 			temp.clear();
// 		}

// 		indices_.clear();
// 	}

// 	template <class Map>
// 	void getMissesProper(Map const& map, Key from_key, Key to_key, Point3 from, Point3 to,
// 	                     MissedSpace& indices, Depth depth)
// 	{
// 		Point3 voxel_border = map.toCoord(from_key) - from;

// 		// Do it backwards
// 		RayCaster ray_caster(to_key, from_key, to, from, voxel_border, map.nodeSize(depth));

// 		if (!ray_caster.hasLeft()) {
// 			if (0 != depth) {
// 				indices.emplace_back(ray_caster.getCurrent(), ray_caster.distanceLeft());
// 			}
// 			return;
// 		}

// 		if (0 == depth) {
// 			// So we do not insert free space on top of occupied
// 			ray_caster.takeStep();
// 		}

// 		do {
// 			indices.emplace_back(ray_caster.getCurrent(), ray_caster.distanceLeft());
// 			ray_caster.takeStep();
// 		} while (ray_caster.hasLeft());
// 	}

// 	template <class Map>
// 	void getMissesProper(Map const& map, Point3 from, Point3 to, CodeMap<float>& indices,
// 	                     Depth depth, size_t early_stopping) const
// 	{
// 		Key from_key = map.toKey(from, depth);
// 		Key to_key = map.toKey(to, depth);
// 		Point3 voxel_border = map.toCoord(from_key) - from;

// 		// Do it backwards
// 		RayCaster ray_caster(to_key, from_key, to, from, voxel_border, map.nodeSize(depth));

// 		if (!ray_caster.hasLeft()) {
// 			if (0 != depth) {
// 				indices.try_emplace(ray_caster.getCurrent(), ray_caster.distanceLeft());
// 			}
// 			return;
// 		}

// 		if (0 == depth) {
// 			// So we do not insert free space on top of occupied
// 			ray_caster.takeStep();
// 		}

// 		size_t already_update_in_row = 0;
// 		do {
// 			if (indices.try_emplace(ray_caster.getCurrent(), ray_caster.distanceLeft())
// 			        .second) {
// 				already_update_in_row = 0;
// 			} else {
// 				++already_update_in_row;
// 				if (0 < early_stopping && already_update_in_row >= early_stopping) {
// 					break;
// 				}
// 			}
// 			ray_caster.takeStep();
// 		} while (ray_caster.hasLeft());
// 	}

// 	template <class Map>
// 	void getMissesSimple(Map const& map, Point3 from, Point3 to, CodeMap<float>& indices,
// 	                     Depth depth, size_t early_stopping) const
// 	{
// 		// Do it backwards
// 		SimpleRayCaster ray_caster(to, from, map.nodeSize(depth));

// 		if (!ray_caster.hasLeft()) {
// 			if (0 != depth) {
// 				indices.try_emplace(map.toCode(ray_caster.getCurrent(), depth),
// 				                    ray_caster.distanceLeft());
// 			}
// 		}

// 		if (0 == depth) {
// 			// So we do not insert free space on top of occupied
// 			ray_caster.takeStep();
// 		}

// 		size_t already_update_in_row = 0;
// 		do {
// 			if (indices
// 			        .try_emplace(map.toCode(ray_caster.getCurrent(), depth),
// 			                     ray_caster.distanceLeft())
// 			        .second) {
// 				already_update_in_row = 0;
// 			} else {
// 				++already_update_in_row;
// 				if (0 < early_stopping && already_update_in_row >= early_stopping) {
// 					break;
// 				}
// 			}
// 			ray_caster.takeStep();
// 		} while (ray_caster.hasLeft());
// 	}

// 	template <class ExecutionPolicy, typename =
// std::enable_if_t<std::is_execution_policy_v< std::decay_t<ExecutionPolicy>>>> 	static
// void pruneMissedSpace(ExecutionPolicy policy, MissedSpace& free_space)
// 	{
// 		MissedSpace temp;
// 		temp.reserve(free_space.size());

// 		size_t size_before;
// 		do {
// 			size_before = free_space.size();

// 			if (8 > size_before) {
// 				return;
// 			}

// 			temp.swap(free_space);

// 			size_t i = 0;
// 			for (; i < temp.size() - 7;) {
// 				Depth depth = temp[i].first.depth() + 1;
// 				Code code = temp[i].first.toDepth(depth);
// 				bool same = true;
// 				size_t j = 1;
// 				for (; j < 8; ++j) {
// 					if (temp[i + j].first.toDepth(depth) != code) {
// 						same = false;
// 						break;
// 					}
// 				}
// 				if (same) {
// 					free_space.emplace_back(code, temp[i].second);
// 				} else {
// 					for (size_t k = 0; k < j; ++k) {
// 						free_space.push_back(temp[i + k]);
// 					}
// 				}
// 				i += j;
// 			}
// 			for (; i < temp.size(); ++i) {
// 				free_space.push_back(temp[i]);
// 			}

// 			temp.clear();
// 			// TODO: What should this be?
// 		} while (size_before != free_space.size());
// 	}

//  protected:
// 	// If there should be discretization
// 	bool discretize_;

// 	// Max range to integrate
// 	float max_range_;

// 	// Hit depth
// 	Depth hit_depth_;

// 	// Miss depth
// 	Depth miss_depth_;

// 	// Ray casting method
// 	RayCastingMethod ray_casting_method_;

// 	// Early stopping
// 	size_t early_stopping_;

// 	// Weighted
// 	bool weighted_;

// 	// Occupancy specific
// 	float occupancy_prob_hit_ = 0.7;
// 	float occupancy_prob_miss_ = 0.4;

// 	// Time step specific
// 	// TODO: Make it so it is possible to change these
// 	TimeStepType current_time_step_ = 1;
// 	bool auto_inc_time_step_ = true;
// 	TimeStepType time_step_inc_ = 1;

// 	// Semantic specific
// 	// TODO: Limit the amount of bits for these
// 	SemanticValue semantic_value_hit_ = 2;   // Logodds probability of hit
// 	SemanticValue semantic_value_miss_ = 1;  // Logodds probability of miss

// 	// For async
// 	std::future<void> async_handler_;

// 	// TODO: Remove
// 	CodeSet indices_;
// 	MissedSpace free_space_;
// };
}  // namespace ufo::map

#endif  // UFO_MAP_INTEGRATOR_BASE_H