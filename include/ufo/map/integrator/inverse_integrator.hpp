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

#ifndef UFO_MAP_INTEGRATOR_INVERSE_INTEGRATOR_HPP
#define UFO_MAP_INTEGRATOR_INVERSE_INTEGRATOR_HPP

// UFO
#include <ufo/cloud/point_cloud.hpp>
#include <ufo/execution/algorithm.hpp>
#include <ufo/execution/execution.hpp>
#include <ufo/map/integrator/detail/inverse/cloud_element.hpp>
#include <ufo/map/integrator/detail/inverse/info.hpp>
#include <ufo/map/integrator/detail/inverse/info_and_lut.hpp>
#include <ufo/map/integrator/detail/inverse/map.hpp>
#include <ufo/map/integrator/detail/inverse/misses_info.hpp>
#include <ufo/map/integrator/integrator.hpp>
#include <ufo/map/map_full.hpp>
#include <ufo/map/occupancy/block.hpp>
#include <ufo/utility/index_iterator.hpp>
#include <ufo/utility/spinlock.hpp>

// STL
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <future>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <tuple>
#include <variant>
#include <vector>

namespace ufo
{
enum class SensorType {
	RAY,
	/* CONE, */
	FRUSTUM
};

enum class ScanOrder {
	HORIZONTAL_MAJOR,
	VERTICAL_MAJOR,
};

template <std::size_t Dim>
class InverseIntegrator : public Integrator<Dim>
{
 public:
	SensorType sensor_type = SensorType::RAY;

	bool treat_nan_as_infinity = false;

 public:
	InverseIntegrator() = default;

	InverseIntegrator(std::filesystem::path const& config_file) { loadConfig(config_file); }

	template <class Map, class T, class... Rest>
	void operator()(Map& map, PointCloud<Dim, T, Rest...> cloud,
	                Transform<Dim, T> const& transform, bool propagate = true) const
	{
		auto misses_and_void_regions_f = std::async(
		    std::launch::deferred, [this, &map, points = get<0>(cloud), &transform]() {
			    if (directions_.empty()) {
				    std::cerr << "No config present, cannot find misses\n";
				    return;
			    }

			    if (directions_.size() != points.size()) {
				    std::cerr << "Present config differs from `cloud`, cannot find misses\n";
				    return;
			    }

			    misses(map, points, transform);
			    voidRegions(map, points, transform);
		    });

		filterDistanceInPlace(cloud, Vec<Dim, T>{}, static_cast<T>(this->min_distance),
		                      static_cast<T>(this->max_distance), true);
		transformInPlace(transform, get<0>(cloud));
		this->create(map, get<0>(cloud), this->hit_depth, hit_nodes_);

		misses_and_void_regions_f.wait();

		std::cout << "Misses:       " << miss_coords_.size() << '\n';
		std::cout << "Void Regions: " << void_region_coords_.size() << '\n';

		this->create(map, miss_coords_, miss_nodes_);
		this->create(map, void_region_coords_, void_region_nodes_);

		this->insertMisses(map, miss_nodes_, miss_counts_);
		this->insertHits(map, hit_nodes_, cloud);
		this->insertVoidRegions(map, void_region_nodes_);
		// TODO: What should this be?
		// this->insertVoidRegionsSecondary(map, miss_nodes_);

		// TODO: Implement

		if (propagate) {
			// TODO: Implement
			map.modifiedPropagate();
		}

		miss_coords_.clear();
		miss_counts_.clear();
		void_region_coords_.clear();
	}

	template <
	    class ExecutionPolicy, class Map, class T, class... Rest,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void operator()(ExecutionPolicy&& policy, Map& map, PointCloud<Dim, T, Rest...> cloud,
	                Transform<Dim, T> const& transform, bool propagate = true) const
	{
		if constexpr (execution::is_seq_v<ExecutionPolicy> ||
		              execution::is_unseq_v<ExecutionPolicy>) {
			operator()(map, cloud, transform);
		}

		auto misses_and_void_regions_f = std::async(
		    std::launch::async, [this, policy, &map, points = get<0>(cloud), &transform]() {
			    if (directions_.empty()) {
				    std::cerr << "No config present, cannot find misses\n";
				    return;
			    }

			    if (directions_.size() != points.size()) {
				    std::cerr << "Present config differs from `cloud`, cannot find misses\n";
				    return;
			    }

			    auto const t1 = std::chrono::high_resolution_clock::now();
			    misses(policy, map, points, transform);
			    auto const t2 = std::chrono::high_resolution_clock::now();
			    voidRegions(policy, map, points, transform);
			    auto const t3 = std::chrono::high_resolution_clock::now();
			    std::chrono::duration<double, std::milli> const misses_ms       = t2 - t1;
			    std::chrono::duration<double, std::milli> const void_regions_ms = t3 - t2;
			    std::cout << "Misses:       " << misses_ms.count() << " ms\n";
			    std::cout << "Void Regions: " << void_regions_ms.count() << " ms\n";
		    });

		filterDistanceInPlace(cloud, Vec<Dim, T>{}, static_cast<T>(this->min_distance),
		                      static_cast<T>(this->max_distance), true);
		transformInPlace(transform, get<0>(cloud));
		this->create(map, get<0>(cloud), this->hit_depth, hit_nodes_);

		misses_and_void_regions_f.wait();

		std::cout << "Misses:       " << miss_coords_.size() << '\n';
		std::cout << "Void Regions: " << void_region_coords_.size() << '\n';

		this->create(policy, map, miss_coords_, miss_nodes_);
		this->create(policy, map, void_region_coords_, void_region_nodes_);

		this->insertMisses(policy, map, miss_nodes_, miss_counts_);
		this->insertHits(policy, map, hit_nodes_, cloud);
		this->insertVoidRegions(policy, map, void_region_nodes_);
		// TODO: What should this be?
		// this->insertVoidRegionsSecondary(policy, map, miss_nodes_);

		// TODO: Implement

		if (propagate) {
			// TODO: Implement
			map.modifiedPropagate(policy);
		}

		miss_coords_.clear();
		miss_counts_.clear();
		void_region_coords_.clear();
	}

	template <class Map, class T>
	void generateConfig(Map const& map, std::vector<Vec<Dim, T>> const& complete_cloud)
	{
		generateDirections(complete_cloud);
		generateConfig(map);
	}

	template <class Map, class T, class... Rest>
	void generateConfig(Map const& map, PointCloud<Dim, T, Rest...> const& complete_cloud)
	{
		generateConfig(map, get<0>(complete_cloud));
	}

	template <class Map>
	void generateConfig(Map const& map, double horizontal_angle_min,
	                    double horizontal_angle_max, double horizontal_angle_increment,
	                    double vertical_angle_min = 0.0, double vertical_angle_max = 0.0,
	                    double    vertical_angle_increment = 0.0,
	                    ScanOrder scan_order               = ScanOrder::HORIZONTAL_MAJOR)
	{
		generateDirections(horizontal_angle_min, horizontal_angle_max,
		                   horizontal_angle_increment, vertical_angle_min, vertical_angle_max,
		                   vertical_angle_increment, scan_order);
		generateConfig(map);
	}

	template <class Map, class T>
	bool generateConfigEstimateStructure(Map const&                      map,
	                                     std::vector<Vec<Dim, T>> const& cloud)
	{
		if (!generateDirectionsEstimateStructure(cloud)) {
			return false;
		}
		generateConfig(map);
	}

	template <class Map, class T, class... Rest>
	bool generateConfigEstimateStructure(Map const&                         map,
	                                     PointCloud<Dim, T, Rest...> const& cloud)
	{
		return generateConfigEstimateStructure(map, get<0>(cloud));
	}

	void loadConfig(std::filesystem::path const& file)
	{
		// TODO: Implement
	}

	void saveConfig(std::filesystem::path const& file)
	{
		// TODO: Implement
	}

	// void loadDirections(std::filesystem::path const& filename)
	// {
	// 	std::ifstream file;
	// 	file.exceptions(std::ifstream::badbit);
	// 	file.imbue(std::locale());
	// 	file.open(filename, std::ios::in | std::ios::binary);

	// 	// TODO: Implement
	// 	// Add dimensions and stuff

	// 	std::uint64_t compl_dirs_size, miss_dirs_size;
	// 	file.read(reinterpret_cast<char*>(&compl_dirs_size), sizeof(compl_dirs_size));
	// 	file.read(reinterpret_cast<char*>(&miss_dirs_size), sizeof(miss_dirs_size));

	// 	complete_directions_.resize(compl_dirs_size);
	// 	missing_directions_.resize(miss_dirs_size);

	// 	file.read(reinterpret_cast<char*>(complete_directions_.data()),
	// 	          compl_dirs_size * sizeof(Vec<Dim, float>));
	// 	file.read(reinterpret_cast<char*>(missing_directions_.data()),
	// 	          miss_dirs_size * sizeof(std::uint64_t));
	// }

	// void saveDirections(std::filesystem::path const& filename) const
	// {
	// 	std::ofstream file;
	// 	file.exceptions(std::ifstream::badbit);
	// 	file.imbue(std::locale());
	// 	file.open(filename, std::ios::out | std::ios::binary);

	// 	// TODO: Implement better
	// 	// Add dimensions and stuff

	// 	std::uint64_t compl_dirs_size = complete_directions_.size();
	// 	std::uint64_t miss_dirs_size  = missing_directions_.size();
	// 	file.write(reinterpret_cast<char*>(&compl_dirs_size), sizeof(compl_dirs_size));
	// 	file.write(reinterpret_cast<char*>(&miss_dirs_size), sizeof(miss_dirs_size));

	// 	file.write(reinterpret_cast<char const*>(complete_directions_.data()),
	// 	           compl_dirs_size * sizeof(Vec<Dim, float>));
	// 	file.write(reinterpret_cast<char const*>(missing_directions_.data()),
	// 	           miss_dirs_size * sizeof(std::uint64_t));
	// }

 private:
	template <class T>
	void generateDirections(std::vector<Vec<Dim, T>> const& points)
	{
		directions_.clear();
		directions_.reserve(points.size());

		std::size_t num_NaN{};
		std::size_t num_origin{};
		for (auto const& p : points) {
			if (isnan(p)) {
				++num_NaN;
				directions_.push_back(Vec<Dim, double>(std::numeric_limits<double>::quiet_NaN()));
			} else if (Vec<Dim, T>{} == p) {
				++num_origin;
				directions_.push_back(Vec<Dim, double>(std::numeric_limits<double>::quiet_NaN()));
			} else {
				directions_.push_back(normalize(cast<double>(p)));
			}
		}

		if (0 < num_NaN) {
			std::cerr << "`complete_cloud` contains " << num_NaN
			          << " NaN points, they will be ignored\n";
		}
		if (0 < num_origin) {
			std::cerr << "`complete_cloud` contains " << num_origin
			          << " points which are at the origin, they will be ignored\n";
		}
	}

	void generateDirections(double hor_angle_min, double hor_angle_max,
	                        double hor_angle_inc, double ver_angle_min,
	                        double ver_angle_max, double ver_angle_inc,
	                        ScanOrder scan_order)
	{
		directions_.clear();

		// NOTE: Should it be horizontal or azimuth?

		if constexpr (2 == Dim) {
			if (0.0 != ver_angle_min || 0.0 != ver_angle_max || 0.0 != ver_angle_inc ||
			    ScanOrder::HORIZONTAL_MAJOR != scan_order) {
				std::cerr << "2 dimensional integrator only supports horizontal, vertical "
				             "parameters will be ignored\n";
			}

			for (auto yaw = hor_angle_min; hor_angle_max >= yaw; yaw += hor_angle_inc) {
				directions_.emplace_back(std::cos(yaw), std::sin(yaw));
			}
		} else if constexpr (3 == Dim) {
			if (ScanOrder::HORIZONTAL_MAJOR == scan_order) {
				for (auto yaw = hor_angle_min; hor_angle_max >= yaw; yaw += hor_angle_inc) {
					for (auto pitch = ver_angle_min; ver_angle_max >= pitch;
					     pitch += ver_angle_inc) {
						directions_.emplace_back(std::cos(yaw) * std::cos(pitch),
						                         std::sin(yaw) * std::cos(pitch), std::sin(pitch));
					}
				}
			} else {
				for (auto pitch = ver_angle_min; ver_angle_max >= pitch; pitch += ver_angle_inc) {
					for (auto yaw = hor_angle_min; hor_angle_max >= yaw; yaw += hor_angle_inc) {
						directions_.emplace_back(std::cos(yaw) * std::cos(pitch),
						                         std::sin(yaw) * std::cos(pitch), std::sin(pitch));
					}
				}
			}

			std::cout << directions_.size() << '\n';
		} else {
			// TODO: What to do?
		}
	}

	template <class T>
	bool generateDirectionsEstimateStructure(std::vector<Vec<Dim, T>> const& points)
	{
		if constexpr (2 == Dim) {
			std::vector<std::pair<double, std::size_t>> yaw_index;
			yaw_index.reserve(points.size());
			for (std::size_t i{}; points.size() > i; ++i) {
				auto point = points[i];
				if (isnan(point) || Vec<Dim, T>{} == point) {
					continue;
				}
				Vec<Dim, double> p = cast<double>(point);
				yaw_index.emplace_back(std::atan2(p.y, p.x), i);
			}

			// TODO: Implement
			return false;
		} else if (3 == Dim) {
			std::vector<std::tuple<double, double, std::size_t>> yaw_pitch_index;
			yaw_pitch_index.reserve(points.size());
			for (std::size_t i{}; points.size() > i; ++i) {
				auto point = points[i];
				if (isnan(point) || Vec<Dim, T>{} == point) {
					continue;
				}
				// TODO: Implement
				// Vec<Dim, double> p     = cast<double>(point);
				// double           pitch = std::asin(-d.Y);
				// double           yaw   = std::atan2(d.X, d.Z);
				// yaw_index.emplace_back(yaw, pitch, i);
			}

			// TODO: Implement
			return false;
		} else {
			// TODO: What to do?
			return false;
		}
	}

	template <class Map>
	void generateConfig(Map const& map)
	{
		auto print_f = [start = std::chrono::high_resolution_clock::now(),
		                last  = std::chrono::high_resolution_clock::now()](
		                   std::string const& task, unsigned task_num, std::size_t task_step,
		                   std::size_t task_total_steps, auto task_start) mutable {
			auto const now = std::chrono::high_resolution_clock::now();
			if (0.1 > std::chrono::duration<double>(now - last).count()) {
				return;
			}
			last = now;

			std::chrono::duration<double> const total_time = now - start;
			std::chrono::duration<double> const task_time  = now - task_start;

			std::printf("[%.1fs] [%d/4 complete] [%s %u%% - %.1fs]\r", total_time.count(),
			            task_num - 1, task.c_str(), (100 * task_step) / task_total_steps,
			            task_time.count());
			std::fflush(stdout);
		};

		std::printf("Generating Inverse Config\n");

		auto inv_map = inverseMap(map.length(this->miss_depth),
		                          map.numDepthLevels() - this->miss_depth, print_f);
		generateInverseStructure(inv_map, print_f);
	}

	template <class PrintFun>
	[[nodiscard]] Map<Dim, detail::InverseMap> inverseMap(
	    Vec<Dim, double> const& leaf_node_length, unsigned num_depth_levels,
	    PrintFun print_f) const
	{
		std::string task = "InverseMap:build";
		std::printf("Starting >>> %s\n", task.c_str());
		auto start = std::chrono::high_resolution_clock::now();

		Map<Dim, detail::InverseMap> map(leaf_node_length, num_depth_levels);

		std::vector<TreeCode<Dim>> codes;
		for (std::size_t i{}; directions_.size() > i; ++i) {
			print_f(task, 1, i, directions_.size(), start);

			auto dir = directions_[i];

			if (isnan(dir)) {
				continue;
			}

			codes.clear();

			Vec<Dim, float> start(this->min_distance * dir);
			// TODO: Add `this->distance_offset`?
			Vec<Dim, float> end(this->max_distance * dir);
			std::variant<LineSegment<Dim, float>, Frustum<Dim, float>> geometry;

			switch (sensor_type) {
				case SensorType::RAY: {
					geometry = LineSegment<Dim, float>(start, end);
					break;
				}
				case SensorType::FRUSTUM: {
					// TODO: Implement
					geometry = Frustum<Dim, float>();
					break;
				}
			}

			std::visit(
			    [&map, &codes](auto&& geometry) {
				    for (auto const& n :
				         map.query(pred::PureLeaf() && pred::Intersects(geometry), false)) {
					    codes.push_back(n.code);
				    }
			    },
			    geometry);

			auto const nodes = map.create(codes);
			for (auto n : nodes) {
				map.inverseIndices(n).push_back(i);
				// TODO: What should this be?
				double dist            = norm(map.center(n)) + norm(map.halfLength(0));
				map.inverseDistance(n) = dist * dist;
			}
		}

		auto                          now       = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> task_time = now - start;
		std::printf("Finished <<< %s [%.1fs]                 \n", task.c_str(),
		            task_time.count());

		task = "InverseMap:propagate";
		std::printf("Starting >>> %s\n", task.c_str());
		start = std::chrono::high_resolution_clock::now();

		map.propagate(false);

		now       = std::chrono::high_resolution_clock::now();
		task_time = now - start;
		std::printf("Finished <<< %s [%.1fs]                 \n", task.c_str(),
		            task_time.count());

		task = "InverseMap:indexing";
		std::printf("Starting >>> %s\n", task.c_str());
		start = std::chrono::high_resolution_clock::now();

		std::uint32_t index{};
		std::size_t   total = map.size();
		for (auto const& n : map.query(pred::PureLeaf())) {
			print_f(task, 3, index, total, start);

			if (map.inverseIndices(n.index).empty()) {
				continue;
			}
			map.inverseIndex(n.index) = index++;
		}

		now       = std::chrono::high_resolution_clock::now();
		task_time = now - start;
		std::printf("Finished <<< %s [%.1fs]                 \n", task.c_str(),
		            task_time.count());

		return map;
	}

	template <class PrintFun>
	void generateInverseStructure(Map<Dim, detail::InverseMap> const& inv_map,
	                              PrintFun                            print_f) const
	{
		std::string task = "InverseStructure";
		std::printf("Starting >>> %s\n", task.c_str());
		auto start = std::chrono::high_resolution_clock::now();

		info_and_lut_middle_.resize(inverse_levels_ - 2);

		info_and_lut_top_.info.clear();
		info_and_lut_top_.lut.clear();
		info_and_lut_bottom_.info.clear();
		info_and_lut_bottom_.lut.clear();
		info_and_lut_bottom_.lut_void_region.clear();
		for (auto& e : info_and_lut_middle_) {
			e.info.clear();
			e.lut.clear();
		}

		std::size_t print_index{};
		std::size_t print_total = inv_map.size();
		for (auto n : inv_map.query(pred::Depth() < inverse_levels_, true, false)) {
			print_f(task, 4, print_index++, print_total, start);

			auto const& indices = inv_map.inverseIndices(n.index);

			if (indices.empty()) {
				continue;
			}

			auto  center   = inv_map.center(n.code);
			float distance = inv_map.inverseDistance(n.index);

			std::vector<std::uint32_t>* lut;

			auto depth = inv_map.depth(n);
			if (0 == depth) {
				// Bottom layer
				auto& info            = info_and_lut_bottom_.info;
				lut                   = &info_and_lut_bottom_.lut;
				auto& lut_void_region = info_and_lut_bottom_.lut_void_region;

				info.emplace_back(center, distance, lut->size(), lut_void_region.size());

				Vec<Dim, float> half_length =
				    cast<float>(inv_map.length(0) * (void_region_distance_ / 2.0));
				AABB<Dim, float> aabb(center - half_length, center + half_length);
				for (auto m : inv_map.query(pred::PureLeaf() && pred::Intersects(aabb), false)) {
					if (n.index == m.index) {
						continue;
					}
					lut_void_region.push_back(inv_map.inverseIndex(m.index));
				}
			} else if (inverse_levels_ - 1 == depth) {
				// Top layer
				auto& info = info_and_lut_top_.info;
				lut        = &info_and_lut_top_.lut;

				std::uint32_t first_child = info_and_lut_middle_.empty()
				                                ? info_and_lut_bottom_.info.size()
				                                : info_and_lut_middle_.back().info.size();
				std::uint32_t num_children{};
				if (inv_map.isParent(n.index)) {
					auto block = inv_map.children(n.index);
					for (std::size_t i{}; inv_map.branchingFactor() > i; ++i) {
						num_children += inv_map.inverseIndices(TreeIndex(block, i)).empty() ? 0 : 1;
					}
				}

				info.emplace_back(center, distance, lut->size(), lut->size() + indices.size(),
				                  first_child, first_child + num_children);
			} else {
				// Middle layer
				auto& info = info_and_lut_middle_[depth - 1].info;
				lut        = &info_and_lut_middle_[depth - 1].lut;

				std::uint32_t first_child = 1 == depth
				                                ? info_and_lut_bottom_.info.size()
				                                : info_and_lut_middle_[depth - 2].info.size();

				info.emplace_back(center, distance, lut->size(), first_child);
			}

			lut->insert(lut->end(), indices.begin(), indices.end());
		}

		info_and_lut_top_.info.shrink_to_fit();
		info_and_lut_top_.lut.shrink_to_fit();
		info_and_lut_bottom_.info.shrink_to_fit();
		info_and_lut_bottom_.lut.shrink_to_fit();
		info_and_lut_bottom_.lut_void_region.shrink_to_fit();
		for (auto& e : info_and_lut_middle_) {
			e.info.shrink_to_fit();
			e.lut.shrink_to_fit();
		}

		auto                          now       = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> task_time = now - start;
		std::printf("Finished <<< %s [%.1fs]                 \n", task.c_str(),
		            task_time.count());
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Misses                                        |
	|                                                                                     |
	**************************************************************************************/

	template <class Map, class T>
	void misses(Map const& map, std::vector<Vec<Dim, T>> const& points,
	            Transform<Dim, T> const& transform) const
	{
		sq_distances_.resize(points.size());
		double length_max = std::min(this->max_distance, max(map.length(this->miss_depth)));
		float  inf = (this->max_distance - length_max) * (this->max_distance - length_max);
		ufo::transform(points.begin(), points.end(), sq_distances_.begin(),
		               [inf, nan_as_inf = treat_nan_as_infinity](auto const& point) {
			               return isnan(point) && nan_as_inf ? inf : normSquared(point);
		               });

		auto const  depth = inverse_levels_ - 1;
		auto const& lut   = info_and_lut_top_.lut;
		for (auto& info : info_and_lut_top_.info) {
			Vec<Dim, float> point       = info.point;
			float           distance    = info.distance;
			std::uint32_t   first_lut   = info.first_lut;
			std::uint32_t   last_lut    = info.last_lut;
			std::uint32_t   first_child = info.first_child;
			std::uint32_t   last_child  = info.last_child;

			for (std::size_t i = first_lut; last_lut > i; ++i) {
				if (distance <= sq_distances_[lut[i]]) {
					auto [count, seen] = missesRecurs(miss_coords_, miss_counts_, transform,
					                                  depth - 1, first_child, last_child);
					info.seen          = seen;
					if (0u != count) {
						// TODO: Implement
					}
					break;
				}
			}
		}
	}

	template <
	    class ExecutionPolicy, class Map, class T,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void misses(ExecutionPolicy&& policy, Map const& map,
	            std::vector<Vec<Dim, T>> const& points,
	            Transform<Dim, T> const&        transform) const
	{
		sq_distances_.resize(points.size());
		double length_max = std::min(this->max_distance, max(map.length(this->miss_depth)));
		float  inf = (this->max_distance - length_max) * (this->max_distance - length_max);
		ufo::transform(policy, points.begin(), points.end(), sq_distances_.begin(),
		               [inf, nan_as_inf = treat_nan_as_infinity](auto const& point) {
			               return isnan(point) && nan_as_inf ? inf : normSquared(point);
		               });

		std::atomic_size_t local_miss_size = 0;
		std::mutex         mutex;

		for_each(policy, info_and_lut_top_.info.begin(), info_and_lut_top_.info.end(),
		         [&](auto& info) {
			         remove_cvref_t<decltype(miss_coords_)>* miss_coords = nullptr;
			         remove_cvref_t<decltype(miss_counts_)>* miss_counts = nullptr;

			         {
				         std::scoped_lock lock(mutex);
				         auto const       id = std::this_thread::get_id();
				         for (std::size_t i = 0, l = local_miss_size; l > i; ++i) {
					         if (id == std::get<0>(local_miss_[i])) {
						         miss_coords = &std::get<1>(local_miss_[i]);
						         miss_counts = &std::get<2>(local_miss_[i]);
						         break;
					         }
				         }

				         if (nullptr == miss_coords /* || nullptr == miss_counts */) {
					         if (local_miss_.size() > local_miss_size) {
						         std::get<0>(local_miss_[local_miss_size]) = id;
						         miss_coords = &std::get<1>(local_miss_[local_miss_size]);
						         miss_counts = &std::get<2>(local_miss_[local_miss_size]);
						         miss_coords->clear();
						         miss_counts->clear();
					         } else {
						         auto& tmp = local_miss_.emplace_back(
						             id, remove_cvref_t<decltype(miss_coords_)>{},
						             remove_cvref_t<decltype(miss_counts_)>{});
						         miss_coords = &std::get<1>(tmp);
						         miss_counts = &std::get<2>(tmp);
					         }
					         ++local_miss_size;
				         }
			         }

			         Vec<Dim, float> point       = info.point;
			         float           distance    = info.distance;
			         std::uint32_t   first_lut   = info.first_lut;
			         std::uint32_t   last_lut    = info.last_lut;
			         std::uint32_t   first_child = info.first_child;
			         std::uint32_t   last_child  = info.last_child;

			         for (std::size_t i = first_lut; last_lut > i; ++i) {
				         if (distance <= sq_distances_[info_and_lut_top_.lut[i]]) {
					         auto [count, seen] =
					             missesRecurs(*miss_coords, *miss_counts, transform,
					                          inverse_levels_ - 2, first_child, last_child);
					         info.seen = seen;
					         if (0u != count) {
						         // TODO: Implement
					         }
					         return;
				         }
			         }
		         });

		std::size_t s{};
		std::vector<std::tuple<std::size_t, remove_cvref_t<decltype(miss_coords_)>*,
		                       remove_cvref_t<decltype(miss_counts_)>*>>
		    tmp;
		tmp.resize(local_miss_size);
		for (std::size_t i{}; tmp.size() > i; ++i) {
			tmp[i] = {s, &std::get<1>(local_miss_[i]), &std::get<2>(local_miss_[i])};
			s += std::get<1>(local_miss_[i]).size();
		}

		miss_coords_.resize(s);
		miss_counts_.resize(s);

		for_each(policy, tmp.begin(), tmp.end(), [this](auto const& e) {
			std::size_t s = std::get<0>(e);
			std::copy(std::get<1>(e)->begin(), std::get<1>(e)->end(), miss_coords_.begin() + s);
			std::copy(std::get<2>(e)->begin(), std::get<2>(e)->end(), miss_counts_.begin() + s);
		});
	}

	template <class T>
	[[nodiscard]] std::pair<unsigned, bool> missesRecurs(
	    std::vector<TreeCoord<Dim, float>>& miss_coords, std::vector<unsigned>& miss_counts,
	    Transform<Dim, T> const& transform, unsigned depth, std::uint32_t first_node,
	    std::uint32_t last_node) const
	{
		std::array<Vec<Dim, float>, 1u << Dim> miss_coord{};
		std::array<unsigned, 1u << Dim>        miss_count{};
		bool                                   seen = false;

		if (0 == depth) {
			auto&       info = info_and_lut_bottom_.info;
			auto const& lut  = info_and_lut_bottom_.lut;

			for (std::size_t idx{}, i = first_node; last_node > i; ++idx, ++i) {
				float distance = info[i].distance;

				std::uint32_t first_lut = info[i].first_lut;
				std::uint32_t last_lut = info.size() > i + 1 ? info[i + 1].first_lut : lut.size();

				unsigned count{};
				bool     after_hit = false;

				for (std::size_t j = first_lut; last_lut > j; ++j) {
					auto d   = sq_distances_[lut[j]];
					bool nan = std::isnan(d);
					count += !nan && distance < d;
					after_hit = after_hit || (!nan && distance >= d);
				}

				seen = seen || (!after_hit && 0u != count);
				info[i].seen(!after_hit && 0u != count);

				if (!after_hit || !this->misses_require_all_before_hit) {
					miss_coord[idx] = info[i].point;
					miss_count[idx] = count;
				}
			}
		} else {
			auto&       info = info_and_lut_middle_[depth - 1].info;
			auto const& lut  = info_and_lut_middle_[depth - 1].lut;

			for (std::size_t idx{}, i = first_node; last_node > i; ++idx, ++i) {
				std::uint32_t first_lut = info[i].first_lut;
				std::uint32_t last_lut = info.size() > i + 1 ? info[i + 1].first_lut : lut.size();

				std::uint32_t first_child = info[i].firstChild();
				std::uint32_t last_child =
				    info.size() > i + 1
				        ? info[i + 1].firstChild()
				        : (1 == depth ? info_and_lut_bottom_.info.size()
				                      : info_and_lut_middle_[depth - 2].info.size());

				float distance = info[i].distance;

				for (std::size_t j = first_lut; last_lut > j; ++j) {
					if (distance <= sq_distances_[lut[j]]) {
						auto [c, s]     = missesRecurs(miss_coords, miss_counts, transform, depth - 1,
						                               first_child, last_child);
						miss_coord[idx] = info[i].point;
						miss_count[idx] = c;
						seen            = seen || s;
						info[i].seen(s);
						break;
					}
				}
			}
		}

		// // TODO: Implement
		// // OBB obb;
		// // obb = transform(obb);
		// // for (auto n : map.query(pred::Inside(obb) || (pred::PureLeaf() &&
		// // pred::Intersects(obb)))) {

		// // }

		unsigned count{};
		switch (this->count_sample_method) {
			case CountSamplingMethod::NONE: {
				for (std::size_t i{}; miss_count.size() > i; ++i) {
					if (0u == miss_count[i]) {
						continue;
					}
					miss_coords.emplace_back(transform(miss_coord[i]), this->miss_depth + depth);
					miss_counts.push_back(miss_count[i]);
				}
				break;
			}
			case CountSamplingMethod::BOOLEAN: {
				// TODO: Implement
				break;
			}
			case CountSamplingMethod::MIN: {
				// TODO: Implement
				break;
			}
			case CountSamplingMethod::MAX: {
				// TODO: Implement
				break;
			}
			case CountSamplingMethod::MEAN: {
				// TODO: Implement
				break;
			}
		}

		return std::make_pair(count, seen);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                    Void Regions                                     |
	|                                                                                     |
	**************************************************************************************/

	template <class Map, class T>
	void voidRegions(Map const& map, std::vector<Vec<Dim, T>> const& points,
	                 Transform<Dim, T> const& transform) const
	{
		if constexpr (!Map::hasMapTypes(MapType::VOID_REGION)) {
			return;
		} else if (MapType::VOID_REGION != (MapType::VOID_REGION & this->integrate_types)) {
			return;
		}

		auto const depth = inverse_levels_ - 1;
		for (auto const& info : info_and_lut_top_.info) {
			if (!info.seen) {
				continue;
			}

			Vec<Dim, float> point       = info.point;
			std::uint32_t   first_child = info.first_child;
			std::uint32_t   last_child  = info.last_child;

			bool void_region = voidRegionsRecurs(void_region_coords_, transform, depth - 1,
			                                     first_child, last_child);

			if (!void_region) {
				continue;
			}

			// TODO: Implement
		}

		// Reset seen
		resetSeen();
	}

	template <
	    class ExecutionPolicy, class Map, class T,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void voidRegions(ExecutionPolicy&& policy, Map const& map,
	                 std::vector<Vec<Dim, T>> const& points,
	                 Transform<Dim, T> const&        transform) const
	{
		if constexpr (!Map::hasMapTypes(MapType::VOID_REGION)) {
			return;
		} else if (MapType::VOID_REGION != (MapType::VOID_REGION & this->integrate_types)) {
			return;
		}

		std::atomic_size_t local_miss_size = 0;
		std::mutex         mutex;

		for_each(policy, info_and_lut_top_.info.begin(), info_and_lut_top_.info.end(),
		         [&](auto const& info) {
			         if (!info.seen) {
				         return;
			         }

			         remove_cvref_t<decltype(void_region_coords_)>* void_region_coords =
			             nullptr;

			         {
				         std::scoped_lock lock(mutex);
				         auto const       id = std::this_thread::get_id();
				         for (std::size_t i = 0, l = local_miss_size; l > i; ++i) {
					         if (id == std::get<0>(local_miss_[i])) {
						         void_region_coords = &std::get<1>(local_miss_[i]);
						         break;
					         }
				         }

				         if (nullptr == void_region_coords) {
					         if (local_miss_.size() > local_miss_size) {
						         std::get<0>(local_miss_[local_miss_size]) = id;
						         void_region_coords = &std::get<1>(local_miss_[local_miss_size]);
						         void_region_coords->clear();
					         } else {
						         auto& tmp = local_miss_.emplace_back(
						             id, remove_cvref_t<decltype(miss_coords_)>{},
						             remove_cvref_t<decltype(miss_counts_)>{});
						         void_region_coords = &std::get<1>(tmp);
					         }
					         ++local_miss_size;
				         }
			         }

			         Vec<Dim, float> point       = info.point;
			         std::uint32_t   first_child = info.first_child;
			         std::uint32_t   last_child  = info.last_child;

			         bool void_region =
			             voidRegionsRecurs(*void_region_coords, transform, inverse_levels_ - 2,
			                               first_child, last_child);

			         if (!void_region) {
				         return;
			         }

			         // TODO: Implement
		         });

		// Reset seen
		resetSeen(policy);

		std::size_t                                                                  s{};
		std::vector<std::pair<std::size_t, remove_cvref_t<decltype(miss_coords_)>*>> tmp;
		tmp.resize(local_miss_size);
		for (std::size_t i{}; tmp.size() > i; ++i) {
			tmp[i] = {s, &std::get<1>(local_miss_[i])};
			s += std::get<1>(local_miss_[i]).size();
		}

		void_region_coords_.resize(s);

		for_each(policy, tmp.begin(), tmp.end(), [this](auto const& e) {
			std::size_t s = e.first;
			std::copy(e.second->begin(), e.second->end(), void_region_coords_.begin() + s);
		});
	}

	template <class T>
	[[nodiscard]] bool voidRegionsRecurs(
	    std::vector<TreeCoord<Dim, float>>& void_region_coords,
	    Transform<Dim, T> const& transform, unsigned depth, std::uint32_t first_node,
	    std::uint32_t last_node) const
	{
		std::array<Vec<Dim, float>, 1u << Dim> coords{};
		std::array<bool, 1u << Dim>            void_regions{};

		if (0 == depth) {
			auto&       info = info_and_lut_bottom_.info;
			auto const& lut  = info_and_lut_bottom_.lut_void_region;

			for (std::size_t idx{}, i = first_node; last_node > i; ++idx, ++i) {
				if (!info[i].seen()) {
					continue;
				}

				std::uint32_t first_lut = info[i].firstLutVoidRegion();
				std::uint32_t last_lut =
				    info.size() > i + 1 ? info[i + 1].firstLutVoidRegion() : lut.size();

				bool void_region = true;
				for (std::size_t i = first_lut; last_lut > i; ++i) {
					if (!info[lut[i]].seen()) {
						void_region = false;
						break;
					}
				}

				coords[idx]       = info[i].point;
				void_regions[idx] = void_region;
			}
		} else {
			auto const& info = info_and_lut_middle_[depth - 1].info;

			for (std::size_t idx{}, i = first_node; last_node > i; ++idx, ++i) {
				if (!info[i].seen()) {
					continue;
				}

				std::uint32_t first_child = info[i].firstChild();
				std::uint32_t last_child =
				    info.size() > i + 1
				        ? info[i + 1].firstChild()
				        : (1 == depth ? info_and_lut_bottom_.info.size()
				                      : info_and_lut_middle_[depth - 2].info.size());

				coords[idx]       = info[i].point;
				void_regions[idx] = voidRegionsRecurs(void_region_coords, transform, depth - 1,
				                                      first_child, last_child);
			}
		}

		// // TODO: Implement
		// // OBB obb;
		// // obb = transform(obb);
		// // for (auto n : map.query(pred::Inside(obb) || (pred::PureLeaf() &&
		// // pred::Intersects(obb)))) {

		// // }

		bool void_region = false;
		switch (this->count_sample_method) {
			case CountSamplingMethod::NONE: {
				for (std::size_t i{}; void_regions.size() > i; ++i) {
					if (!void_regions[i]) {
						continue;
					}
					void_region_coords.emplace_back(transform(coords[i]), this->miss_depth + depth);
				}
				break;
			}
			case CountSamplingMethod::BOOLEAN: {
				// TODO: Implement
				break;
			}
			case CountSamplingMethod::MIN: {
				// TODO: Implement
				break;
			}
			case CountSamplingMethod::MAX: {
				// TODO: Implement
				break;
			}
			case CountSamplingMethod::MEAN: {
				// TODO: Implement
				break;
			}
		}

		return void_region;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Seen                                         |
	|                                                                                     |
	**************************************************************************************/

	void setSeen() const
	{
		// TODO: Implement

		// Fill in seen_indices_
	}

	template <
	    class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void setSeen(ExecutionPolicy&& policy) const
	{
		// TODO: Implement

		// Fill in seen_indices_
	}

	void setSeenRecurs(unsigned depth, std::uint32_t first_node,
	                   std::uint32_t last_node) const
	{
		// TODO: Implement

		// Fill in seen_indices_
	}

	void resetSeen() const
	{
		auto const depth = inverse_levels_ - 1;
		for (auto& info : info_and_lut_top_.info) {
			if (!info.seen) {
				continue;
			}

			info.seen = false;

			std::uint32_t first_child = info.first_child;
			std::uint32_t last_child  = info.last_child;

			resetSeenRecurs(depth - 1, first_child, last_child);
		}
	}

	template <
	    class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void resetSeen(ExecutionPolicy&& policy) const
	{
		for_each(std::forward<ExecutionPolicy>(policy), info_and_lut_top_.info.begin(),
		         info_and_lut_top_.info.end(), [this](auto& info) {
			         if (!info.seen) {
				         return;
			         }

			         info.seen = false;

			         std::uint32_t first_child = info.first_child;
			         std::uint32_t last_child  = info.last_child;

			         resetSeenRecurs(inverse_levels_ - 2, first_child, last_child);
		         });
	}

	void resetSeenRecurs(unsigned depth, std::uint32_t first_node,
	                     std::uint32_t last_node) const
	{
		if (0 == depth) {
			auto& info = info_and_lut_bottom_.info;
			for (std::size_t i = first_node; last_node > i; ++i) {
				info[i].seen(false);
			}
		} else {
			auto& info = info_and_lut_middle_[depth - 1].info;

			for (std::size_t i = first_node; last_node > i; ++i) {
				if (!info[i].seen()) {
					continue;
				}

				info[i].seen(false);

				std::uint32_t first_child = info[i].firstChild();
				std::uint32_t last_child =
				    info.size() > i + 1
				        ? info[i + 1].firstChild()
				        : (1 == depth ? info_and_lut_bottom_.info.size()
				                      : info_and_lut_middle_[depth - 2].info.size());

				resetSeenRecurs(depth - 1, first_child, last_child);
			}
		}
	}

 public:
	// Needs to be at least 2
	std::size_t inverse_levels_       = 5;
	double      void_region_distance_ = 2.0;

	std::vector<Vec<Dim, double>> directions_;

	mutable detail::InverseInfoAndLutTop<Dim>                 info_and_lut_top_;
	mutable std::vector<detail::InverseInfoAndLutMiddle<Dim>> info_and_lut_middle_;
	mutable detail::InverseInfoAndLutBottom<Dim>              info_and_lut_bottom_;

	mutable std::vector<float> sq_distances_;

	mutable __block std::vector<TreeIndex> hit_nodes_;
	mutable __block std::vector<TreeIndex> miss_nodes_;
	mutable __block std::vector<TreeIndex> void_region_nodes_;

	mutable std::vector<TreeCoord<Dim, float>> miss_coords_;
	mutable std::vector<unsigned>              miss_counts_;
	mutable std::vector<TreeCoord<Dim, float>> void_region_coords_;
	mutable std::vector<unsigned>              seen_indices_;

	mutable std::deque<std::tuple<std::thread::id, std::vector<TreeCoord<Dim, float>>,
	                              std::vector<unsigned>>>
	    local_miss_;
};
}  // namespace ufo

#endif  // UFO_MAP_INTEGRATOR_INVERSE_INTEGRATOR_HPP