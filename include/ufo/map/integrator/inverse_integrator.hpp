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
#include <ufo/container/tree_map.hpp>
#include <ufo/execution/algorithm.hpp>
#include <ufo/execution/execution.hpp>
#include <ufo/geometry/obb.hpp>
#include <ufo/map/integrator/detail/grid_map.hpp>
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
#include <tuple>
#include <unordered_map>
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

enum class InverseNaNBehavior { IGNORE, ZERO, INF };

template <std::size_t Dim, unsigned InverseLevels = 5>
class InverseIntegrator : public Integrator<Dim, InverseLevels>
{
	static_assert(2 <= InverseLevels, "'InverseLevels' needs to be at least 2");

 public:
	SensorType sensor_type = SensorType::RAY;

	InverseNaNBehavior nan_behavior = InverseNaNBehavior::IGNORE;

	bool conservative_freeing = true;

	bool unordered_mode = true;
	// double unordered_max_dist = 0.1;
	double unordered_dir_factor = 10000.0;

	unsigned min_num_for_miss        = 1;
	unsigned min_num_for_void_region = 1;

 public:
	InverseIntegrator() = default;

	InverseIntegrator(std::filesystem::path const& config_file) { loadConfig(config_file); }

	template <class Map, class T, class... Rest>
	void operator()(Map& map, PointCloud<Dim, T, Rest...> const& cloud,
	                Transform<Dim, T> const& transform, bool propagate = true) const
	{
		auto const t0 = std::chrono::high_resolution_clock::now();

		if (!valid(cloud, true)) {
			return;
		}

		auto const t1 = std::chrono::high_resolution_clock::now();

		fillDistances(map, get<0>(cloud));

		auto const t2 = std::chrono::high_resolution_clock::now();

		this->hits(map, cloud, transform);

		auto const t3 = std::chrono::high_resolution_clock::now();

		bool with_count = true;  // TODO: Fill in some way
		findMisses(map, transform, with_count);
		auto const t4 = std::chrono::high_resolution_clock::now();

		this->misses(map);
		auto const t5 = std::chrono::high_resolution_clock::now();

		// TODO: Implement

		if (propagate) {
			// TODO: Implement
			map.modifiedPropagate();
		}
		auto const t6 = std::chrono::high_resolution_clock::now();

		std::chrono::duration<double, std::milli> const valid_ms          = t1 - t0;
		std::chrono::duration<double, std::milli> const fill_distances_ms = t2 - t1;
		std::chrono::duration<double, std::milli> const hits_ms           = t3 - t2;
		std::chrono::duration<double, std::milli> const find_misses_ms    = t4 - t3;
		std::chrono::duration<double, std::milli> const misses_ms         = t5 - t4;
		std::chrono::duration<double, std::milli> const propagate_ms      = t6 - t5;
		std::chrono::duration<double, std::milli> const total_ms          = t6 - t0;

		std::cout << "Valid:          " << valid_ms.count() << " ms\n";
		std::cout << "Fill distances: " << fill_distances_ms.count() << " ms\n";
		std::cout << "Hits:           " << hits_ms.count() << " ms\n";
		std::cout << "Find misses:    " << find_misses_ms.count() << " ms\n";
		std::cout << "Misses:         " << misses_ms.count() << " ms\n";
		std::cout << "Propagate:      " << propagate_ms.count() << " ms\n";
		std::cout << "Total:          " << total_ms.count() << " ms\n\n";
	}

	template <
	    class ExecutionPolicy, class Map, class T, class... Rest,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void operator()(ExecutionPolicy&& policy, Map& map,
	                PointCloud<Dim, T, Rest...> const& cloud,
	                Transform<Dim, T> const& transform, bool propagate = true) const
	{
		if constexpr (execution::is_seq_v<ExecutionPolicy> ||
		              execution::is_unseq_v<ExecutionPolicy>) {
			return operator()(map, cloud, transform);
		}

		auto const t0 = std::chrono::high_resolution_clock::now();

		if (!valid(cloud, true)) {
			return;
		}

		auto const t1 = std::chrono::high_resolution_clock::now();

		fillDistances(policy, map, get<0>(cloud));

		auto const t2 = std::chrono::high_resolution_clock::now();

		this->hits(policy, map, cloud, transform);

		auto const t3 = std::chrono::high_resolution_clock::now();

		bool with_count = true;  // TODO: Fill in some way
		findMisses(policy, map, transform, with_count);

		auto const t4 = std::chrono::high_resolution_clock::now();

		this->misses(policy, map);

		auto const t5 = std::chrono::high_resolution_clock::now();

		// TODO: Implement

		if (propagate) {
			// TODO: Implement
			map.modifiedPropagate(policy);
		}

		auto const t6 = std::chrono::high_resolution_clock::now();

		std::chrono::duration<double, std::milli> const valid_ms          = t1 - t0;
		std::chrono::duration<double, std::milli> const fill_distances_ms = t2 - t1;
		std::chrono::duration<double, std::milli> const hits_ms           = t3 - t2;
		std::chrono::duration<double, std::milli> const find_misses_ms    = t4 - t3;
		std::chrono::duration<double, std::milli> const misses_ms         = t5 - t4;
		std::chrono::duration<double, std::milli> const propagate_ms      = t6 - t5;
		std::chrono::duration<double, std::milli> const total_ms          = t6 - t0;

		std::cout << "Valid:          " << valid_ms.count() << " ms\n";
		std::cout << "Fill distances: " << fill_distances_ms.count() << " ms\n";
		std::cout << "Hits:           " << hits_ms.count() << " ms\n";
		std::cout << "Find misses:    " << find_misses_ms.count() << " ms\n";
		std::cout << "Misses:         " << misses_ms.count() << " ms\n";
		std::cout << "Propagate:      " << propagate_ms.count() << " ms\n";
		std::cout << "Total:          " << total_ms.count() << " ms\n\n";
	}

	template <class Map, class T>
	void generateConfig(Map const& map, std::vector<Vec<Dim, T>> const& complete_cloud)
	{
		generateDirections(complete_cloud);
		if (unordered_mode) {
			generateDirToIndex();
		}
		generateConfig(map);
	}

	template <class Map, class T, class... Rest>
	void generateConfig(Map const& map, PointCloud<Dim, T, Rest...> const& complete_cloud)
	{
		generateConfig(map, get<0>(complete_cloud));
	}

	// template <class Map>
	// void generateConfig(Map const& map, double horizontal_angle_min,
	//                     double horizontal_angle_inc, unsigned horizontal_num,
	//                     double vertical_angle_min = 0.0, double vertical_angle_inc = 0.0,
	//                     unsigned  vertical_num = 1u,
	//                     ScanOrder scan_order   = ScanOrder::HORIZONTAL_MAJOR)
	// {
	// 	generateDirections(horizontal_angle_min, horizontal_angle_inc, horizontal_num,
	// 	                   vertical_angle_min, vertical_angle_inc, vertical_num,
	// scan_order); 	generateConfig(map);
	// }

	// template <class Map, class T>
	// bool generateConfigEstimateStructure(Map const&                      map,
	//                                      std::vector<Vec<Dim, T>> const& cloud)
	// {
	// 	if (!generateDirectionsEstimateStructure(cloud)) {
	// 		return false;
	// 	}
	// 	generateConfig(map);
	// }

	// template <class Map, class T, class... Rest>
	// bool generateConfigEstimateStructure(Map const&                         map,
	//                                      PointCloud<Dim, T, Rest...> const& cloud)
	// {
	// 	return generateConfigEstimateStructure(map, get<0>(cloud));
	// }

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
	template <class T, class... Rest>
	[[nodiscard]] bool valid(PointCloud<Dim, T, Rest...> const& cloud,
	                         bool                               print_if_not_valid) const
	{
		if (directions_.empty()) {
			if (print_if_not_valid) {
				std::cerr << "No config present. A config can be generated by using one of the "
				             "`generateConfig` or `generateConfigEstimateStructure` functions.\n";
			}
			return false;
		}

		if (!unordered_mode && directions_.size() != cloud.size()) {
			// TODO: Add that it is required for ordered_mode
			if (print_if_not_valid) {
				std::cerr << "The generated config does not match the structure of `cloud`. Keep "
				             "the structure the same for every cloud passed in; otherwise, "
				             "generate a new config using one of the `generateConfig` or "
				             "`generateConfigEstimateStructure` functions.\n";
			}
			return false;
		}

		return true;
	}

	void generateDirToIndex()
	{
		for (unsigned i{}; directions_.size() > i; ++i) {
			auto dir = directions_[i];
			if (isnan(dir)) {
				continue;
			}

			// dir_to_index_.emplace(cast<float>(dir), i);
			// dir *= round(dir * unordered_dir_factor);
			std::uint64_t dir_i{};
			for (unsigned j{}; Dim > j; ++j) {
				dir_i += static_cast<std::uint64_t>(
				             std::round((1.0 + dir[j]) * (unordered_dir_factor / 2.0)))
				         << (j * 20u);
			}
			dir_to_index_[dir_i] = i;
		}
	}

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

	void generateDirections(double horizontal_angle_min, double horizontal_angle_inc,
	                        unsigned horizontal_num, double vertical_angle_min,
	                        double vertical_angle_inc, unsigned vertical_num,
	                        ScanOrder scan_order)
	{
		directions_.clear();

		// NOTE: Should it be horizontal or azimuth?

		// TODO: Implement

		// if constexpr (2 == Dim) {
		// 	if (0.0 != ver_angle_min || 0.0 != ver_angle_max || 0.0 != ver_angle_inc ||
		// 	    ScanOrder::HORIZONTAL_MAJOR != scan_order) {
		// 		std::cerr << "2 dimensional integrator only supports horizontal, vertical "
		// 		             "parameters will be ignored\n";
		// 	}

		// 	for (auto yaw = hor_angle_min; hor_angle_max >= yaw; yaw += hor_angle_inc) {
		// 		directions_.emplace_back(std::cos(yaw), std::sin(yaw));
		// 	}
		// } else if constexpr (3 == Dim) {
		// 	if (ScanOrder::HORIZONTAL_MAJOR == scan_order) {
		// 		for (auto yaw = hor_angle_min; hor_angle_max >= yaw; yaw += hor_angle_inc) {
		// 			for (auto pitch = ver_angle_min; ver_angle_max >= pitch;
		// 			     pitch += ver_angle_inc) {
		// 				directions_.emplace_back(std::cos(yaw) * std::cos(pitch),
		// 				                         std::sin(yaw) * std::cos(pitch), std::sin(pitch));
		// 			}
		// 		}
		// 	} else {
		// 		for (auto pitch = ver_angle_min; ver_angle_max >= pitch; pitch += ver_angle_inc)
		// { 			for (auto yaw = hor_angle_min; hor_angle_max >= yaw; yaw += hor_angle_inc) {
		// 				directions_.emplace_back(std::cos(yaw) * std::cos(pitch),
		// 				                         std::sin(yaw) * std::cos(pitch), std::sin(pitch));
		// 			}
		// 		}
		// 	}

		// 	std::cout << directions_.size() << '\n';
		// } else {
		// 	// TODO: What to do?
		// }
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
		auto print_f = [start           = std::chrono::high_resolution_clock::now(),
		                last_printed    = std::chrono::high_resolution_clock::now(),
		                last_task       = std::string(),
		                last_task_start = std::chrono::high_resolution_clock::now(),
		                task_num        = 0u](std::string const& task, std::size_t task_step,
		                               std::size_t task_total_steps) mutable {
			auto const                          now = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double> const total_time = now - start;

			if (last_task != task) {
				if ("" != last_task) {
					std::chrono::duration<double> const task_time = now - last_task_start;
					std::printf("Finished <<< %s [%.1fs]                 \n", last_task.c_str(),
					            task_time.count());
				}
				if ("" != task) {
					++task_num;
					std::printf("Starting >>> %s\n", task.c_str());
				}
				last_task       = task;
				last_task_start = now;
			}

			std::chrono::duration<double> const task_time = now - last_task_start;

			if (0.1 > std::chrono::duration<double>(now - last_printed).count()) {
				return;
			}
			last_printed = now;

			std::printf("[%.1fs] [%d/7 complete] [%s %u%% - %.1fs]\r", total_time.count(),
			            task_num - 1, task.c_str(), (100 * task_step) / task_total_steps,
			            task_time.count());
			std::fflush(stdout);
		};

		std::printf("Generating Inverse Config\n");

		auto const t0      = std::chrono::high_resolution_clock::now();
		auto       inv_map = inverseMap(map.length(this->miss_depth),
		                                map.numDepthLevels() - this->miss_depth, print_f);
		generateInverseStructure(inv_map, print_f);
		auto const                          t1   = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> const time = t1 - t0;

		print_f("", 1, 1);

		std::printf("Generated Inverse Config [%.1fs]        \n", time.count());

		info_and_lut_.printMemoryUsage();
	}

	template <class PrintFun>
	[[nodiscard]] Map<Dim, detail::InverseMap> inverseMap(
	    Vec<Dim, double> const& leaf_node_length, unsigned num_depth_levels,
	    PrintFun& print_f) const
	{
		Map<Dim, detail::InverseMap> map(leaf_node_length, num_depth_levels);
		// This is needed in order to be able to query while adding to the map
		Map<Dim, detail::InverseMap> ghost_map(leaf_node_length, num_depth_levels);

		std::vector<TreeCode<Dim>> codes;
		std::vector<TreeIndex>     nodes;
		std::vector<TreeIndex>     seen_nodes;
		codes.reserve(1'000'000);
		nodes.reserve(1'000'000);
		std::mutex         mutex;
		auto const         main_thread = std::this_thread::get_id();
		std::atomic_size_t print_index{};
		// FIXME: Take policy instead
		for_each(execution::par, directions_.begin(), directions_.end(),
		         [&](auto const& dir) {
			         if (std::this_thread::get_id() == main_thread) {
				         print_f("InverseMap:build", print_index, directions_.size());
			         }
			         ++print_index;

			         if (isnan(dir)) {
				         return;
			         }

			         Vec<Dim, float> start(this->min_distance * dir);
			         Vec<Dim, float> end((this->max_distance + this->distance_offset) * dir);
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

			         thread_local std::vector<TreeCode<Dim>> local_codes;
			         std::visit(
			             [&ghost_map](auto&& g) {
				             auto pred = pred::PureLeaf() && pred::Intersects(g);
				             for (auto const& n : ghost_map.query(pred, false)) {
					             local_codes.push_back(n.code);
				             }
			             },
			             geometry);

			         {
				         std::scoped_lock lock(mutex);
				         codes.insert(codes.end(), local_codes.begin(), local_codes.end());
			         }
			         local_codes.clear();

			         if (main_thread == std::this_thread::get_id()) {
				         {
					         std::scoped_lock lock(mutex);
					         std::swap(codes, local_codes);
				         }
				         nodes.resize(local_codes.size());
				         map.create(local_codes, nodes.begin());
				         std::for_each(nodes.begin(), nodes.end(), [&map, &seen_nodes](auto n) {
					         if (0u == map.inverseCount(n)++) {
						         seen_nodes.push_back(n);
					         }
				         });
				         local_codes.clear();
			         }
		         });

		nodes.resize(codes.size());
		map.create(execution::par, codes, nodes.begin());
		std::for_each(nodes.begin(), nodes.end(), [&map, &seen_nodes](auto n) {
			if (0u == map.inverseCount(n)++) {
				seen_nodes.push_back(n);
			}
		});

		if (conservative_freeing) {
			std::atomic_size_t print_index{};
			std::size_t        print_total = seen_nodes.size();
			auto const         main_thread = std::this_thread::get_id();
			for_each(execution::par, seen_nodes.begin(), seen_nodes.end(), [&](auto n) {
				if (std::this_thread::get_id() == main_thread) {
					print_f("InverseMap:postprocess", print_index, print_total);
				}
				++print_index;

				auto key = map.key(n);
				auto k   = key;
				if constexpr (2 == Dim) {
					for (k.y = key.y - 1; key.y + 1 >= k.y; ++k.y) {
						for (k.x = key.x - 1; key.x + 1 >= k.x; ++k.x) {
							if (0u == map.inverseCount(k)) {
								// NOTE: Index means that it should be prune in the next step
								map.inverseIndex(n) = std::numeric_limits<std::uint_fast32_t>::max();
								return;
							}
						}
					}
				} else if constexpr (3 == Dim) {
					for (k.z = key.z - 1; key.z + 1 >= k.z; ++k.z) {
						for (k.y = key.y - 1; key.y + 1 >= k.y; ++k.y) {
							for (k.x = key.x - 1; key.x + 1 >= k.x; ++k.x) {
								if (0u == map.inverseCount(k)) {
									// NOTE: Index means that it should be prune in the next step
									map.inverseIndex(n) = std::numeric_limits<std::uint_fast32_t>::max();
									return;
								}
							}
						}
					}
				} else {
					// TODO: Error
				}
			});

			print_index = {};
			for_each(execution::par, seen_nodes.begin(), seen_nodes.end(), [&](auto n) {
				if (std::this_thread::get_id() == main_thread) {
					print_f("InverseMap:prune", print_index, print_total);
				}
				++print_index;

				if (std::numeric_limits<std::uint_fast32_t>::max() == map.inverseIndex(n)) {
					map.inverseCount(n) = 0u;
				}
			});
		} else {
			print_f("InverseMap:postprocess", 0, 1);
			print_f("InverseMap:prune", 0, 1);
		}

		// TODO: Propagate instead of below
		print_f("InverseMap:propagate", 0, 1);
		map.propagate(false);
		print_f("InverseMap:propagate", 1, 1);

		return map;
	}

	template <class PrintFun>
	void generateInverseStructure(Map<Dim, detail::InverseMap>& map, PrintFun& print_f)
	{
		info_and_lut_.clear();

		std::array<std::size_t, InverseLevels> info_size{};
		std::size_t                            leaf_lut_size{};
		for (auto n : map.query(pred::Depth() < InverseLevels)) {
			if (0u == map.inverseCount(n.index)) {
				continue;
			}
			++info_size[map.depth(n)];
			if (0u == map.depth(n)) {
				leaf_lut_size += map.inverseCount(n.index);
			}
		}

		for (unsigned d{}; info_size.size() > d; ++d) {
			info_and_lut_.reserveInfo(info_size[d], d);
		}
		info_and_lut_.resizeLut(leaf_lut_size, 0u);

		std::uint_fast32_t index{};
		std::uint32_t      first_leaf_lut{};
		std::size_t        print_index{};
		std::size_t        print_total = map.size();
		for (auto n : map.query(pred::Depth() < InverseLevels)) {
			print_f("InverseStructure:build", print_index++, print_total);
			if (0u == map.inverseCount(n.index)) {
				continue;
			}

			auto center = map.center(n);
			auto depth  = map.depth(n);

			if (0u == depth) {  // Leaf layer
				// NOTE: Minimum distance, because we remove if it overlaps with hits anyways
				double dist    = norm(center) + norm(map.length(0u));
				float  dist_sq = static_cast<float>(dist * dist);
				info_and_lut_.leafEmplaceBack(center, dist_sq, first_leaf_lut);
				first_leaf_lut += map.inverseCount(n.index);
				map.inverseIndex(n.index) = index++;
				map.inverseCount(n.index) = 0u;
			} else {  // All other layers
				info_and_lut_.emplaceBack(depth, center, info_and_lut_.infoSize(depth - 1u));
			}
		}

		{
			auto const         main_thread = std::this_thread::get_id();
			std::atomic_size_t print_index{};
			// FIXME: Take policy instead
			for_each(execution::par, std::size_t(0u), directions_.size(), [&](std::size_t i) {
				if (std::this_thread::get_id() == main_thread) {
					print_f("InverseStructure:fill", print_index, directions_.size());
				}
				++print_index;

				auto dir = directions_[i];

				if (isnan(dir)) {
					return;
				}

				Vec<Dim, float> start(this->min_distance * dir);
				Vec<Dim, float> end((this->max_distance + this->distance_offset) * dir);
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

				thread_local std::vector<TreeCode<Dim>> codes;
				thread_local std::vector<TreeIndex>     nodes;
				std::visit(
				    [&map](auto&& g) {
					    auto pred = pred::PureLeaf() && pred::Intersects(g);
					    for (auto const& n : map.query(pred, false)) {
						    codes.push_back(n.code);
					    }
				    },
				    geometry);

				nodes.resize(codes.size());
				// NOTE: This will not "create" any nodes here since they have already been
				// created, so no "ghost" map needed.
				map.create(codes, nodes.begin());
				for (auto n : nodes) {
					if (std::numeric_limits<std::uint_fast32_t>::max() == map.inverseIndex(n)) {
						continue;
					}
					auto it =
					    info_and_lut_.beginLut(map.inverseIndex(n), 0u) + map.inverseCount(n)++;
					*it = i;
				}
				codes.clear();
			});
		}

		// NOTE: Add one extra to make it possible to find last
		info_and_lut_.leafEmplaceBack(Vec<Dim, float>(), 0.0f, info_and_lut_.lutSize(0u));

		print_index = {};
		print_total = {};
		for (unsigned d = 1u; InverseLevels > d; ++d) {
			print_total += info_and_lut_.infoSize(d);
		}

		std::vector<std::uint32_t> tmp_lut;
		for (unsigned depth = 1u; InverseLevels > depth; ++depth) {
			std::uint32_t size = info_and_lut_.infoSize(depth);
			for (std::uint32_t index{}; size > index; ++index) {
				print_f("InverseStructure:propagate", print_index++, print_total);
				auto first = info_and_lut_.firstChild(index, depth);
				auto last  = size - 1 > index ? info_and_lut_.lastChild(index, depth)
				                              : (info_and_lut_.infoSize(depth - 1u) - 1);

				bool  all_children = map.branchingFactor() == (last - first);
				float min_dist     = std::numeric_limits<float>::max();
				float max_dist     = std::numeric_limits<float>::lowest();
				tmp_lut.clear();
				for (; last > first; ++first) {
					auto first_lut = info_and_lut_.beginLut(first, depth - 1u);
					auto last_lut  = info_and_lut_.endLut(first, depth - 1u);
					tmp_lut.insert(tmp_lut.end(), first_lut, last_lut);
					if (1u == depth) {
						min_dist = std::min(min_dist, info_and_lut_.distance(first));
						max_dist = std::max(max_dist, info_and_lut_.distance(first));
					} else {
						min_dist = std::min(min_dist, info_and_lut_.minDistance(first, depth - 1u));
						max_dist = std::max(max_dist, info_and_lut_.maxDistance(first, depth - 1u));
					}
				}
				std::sort(tmp_lut.begin(), tmp_lut.end());
				tmp_lut.erase(std::unique(tmp_lut.begin(), tmp_lut.end()), tmp_lut.end());

				info_and_lut_.minDistance(index, depth) = min_dist;
				info_and_lut_.maxDistance(index, depth) = max_dist;
				info_and_lut_.firstLut(index, depth)    = info_and_lut_.lutSize(depth);
				info_and_lut_.count(index, depth)       = all_children ? tmp_lut.size() : 0u;
				info_and_lut_.lutInsert(depth, tmp_lut);
			}

			// NOTE: Add one extra to make it possible to find last
			info_and_lut_.emplaceBack(depth, Vec<Dim, float>(), 0.0f, 0.0f,
			                          info_and_lut_.infoSize(depth - 1u) - 1,
			                          info_and_lut_.lutSize(depth), 0u);
		}

		info_and_lut_.shrinkToFit();
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Distances                                      |
	|                                                                                     |
	**************************************************************************************/

	template <class Map, class T>
	void fillDistances(Map const& map, std::vector<Vec<Dim, T>> const& points) const
	{
		if (unordered_mode) {
			fillDistancesUnordered(map, points);
		} else {
			fillDistancesOrdered(map, points);
		}
	}

	template <class Map, class T>
	void fillDistancesOrdered(Map const& map, std::vector<Vec<Dim, T>> const& points) const
	{
		// TODO: Add distance offset here

		distances_.resize(points.size());
		float nan = fillDistanceNaNValue(map);
		std::transform(
		    points.begin(), points.end(), distances_.begin(),
		    [nan](auto const& point) { return isnan(point) ? nan : normSquared(point); });
	}

	template <class Map, class T>
	void fillDistancesUnordered(Map const&                      map,
	                            std::vector<Vec<Dim, T>> const& points) const
	{
		// TODO: Add distance offset here

		// float nan = fillDistanceNaNValue(map);
		// distances_.assign(directions_.size(), nan);

		// for (auto p : points) {
		// 	if (isnan(p)) {
		// 		continue;
		// 	}

		// 	auto ds = normSquared(p);
		// 	p /= std::sqrt(ds);

		// 	auto [v, d] = dir_to_index_.nearestPoint(p);
		// 	if (nullptr != v && unordered_max_dist >= d) {
		// 		// TODO: Should it take the minimum?
		// 		distances_[v->second] = ds;
		// 	}
		// }
	}

	template <
	    class ExecutionPolicy, class Map, class T,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void fillDistances(ExecutionPolicy&& policy, Map const& map,
	                   std::vector<Vec<Dim, T>> const& points) const
	{
		if (unordered_mode) {
			fillDistancesUnordered(std::forward<ExecutionPolicy>(policy), map, points);
		} else {
			fillDistancesOrdered(std::forward<ExecutionPolicy>(policy), map, points);
		}
	}

	template <
	    class ExecutionPolicy, class Map, class T,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void fillDistancesOrdered(ExecutionPolicy&& policy, Map const& map,
	                          std::vector<Vec<Dim, T>> const& points) const
	{
		// TODO: Add distance offset here

		assert(directions_.size() == points.size());

		distances_.resize(points.size());
		float nan = fillDistanceNaNValue(map);
		transform(std::forward<ExecutionPolicy>(policy), points.begin(), points.end(),
		          distances_.begin(),
		          [nan](auto const& p) { return isnan(p) ? nan : normSquared(p); });
	}

	template <
	    class ExecutionPolicy, class Map, class T,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void fillDistancesUnordered(ExecutionPolicy&& policy, Map const& map,
	                            std::vector<Vec<Dim, T>> const& points) const
	{
		// TODO: Add distance offset here

		float nan = fillDistanceNaNValue(map);
		distances_.assign(directions_.size(), nan);

		std::vector<std::pair<unsigned, float>> index_and_dist(points.size());
		transform(
		    std::forward<ExecutionPolicy>(policy), points.begin(), points.end(),
		    index_and_dist.begin(), [this](auto const& p) {
			    std::pair<unsigned, float> ret(-1, std::numeric_limits<float>::quiet_NaN());

			    if (!isnan(p) && Vec<Dim, T>{} != p) {
				    auto   ds = normSquared(p);
				    double d  = std::sqrt(static_cast<double>(ds));

				    std::uint64_t dir_i{};
				    for (unsigned j{}; Dim > j; ++j) {
					    dir_i += static_cast<std::uint64_t>(
					                 std::round((1.0 + p[j] / d) * (unordered_dir_factor / 2.0)))
					             << (j * 20u);
				    }

				    if (auto it = dir_to_index_.find(dir_i); dir_to_index_.end() != it) {
					    ret.first  = it->second;
					    ret.second = ds;
				    }
			    }

			    return ret;
		    });

		// std::vector<std::pair<unsigned, float>> index_and_dist(points.size());
		// transform(std::forward<ExecutionPolicy>(policy), points.begin(), points.end(),
		//           index_and_dist.begin(), [this](auto p) {
		// 	          std::pair<unsigned, float> ret(-1,
		// 	                                         std::numeric_limits<float>::quiet_NaN());

		// 	          if (!isnan(p)) {
		// 		          auto ds = normSquared(p);
		// 		          p /= std::sqrt(ds);

		// 		          auto [v, d] = dir_to_index_.nearestPoint(p);
		// 		          if (nullptr != v && unordered_max_dist >= d) {
		// 			          ret.first  = v->second;
		// 			          ret.second = ds;
		// 		          }
		// 	          }

		// 	          return ret;
		//           });

		for (auto [i, ds] : index_and_dist) {
			if (std::isnan(ds)) {
				continue;
			}

			distances_[i] = std::isnan(distances_[i]) ? ds : std::min(distances_[i], ds);
		}
	}

	template <class Map>
	[[nodiscard]] float fillDistanceNaNValue(Map const& map) const
	{
		// TODO: Add distance offset here

		switch (nan_behavior) {
			case InverseNaNBehavior::IGNORE: return std::numeric_limits<float>::quiet_NaN();
			case ufo::InverseNaNBehavior::INF: {
				double length_max =
				    std::min(this->max_distance, max(map.length(this->miss_depth)));
				return (this->max_distance - length_max) * (this->max_distance - length_max);
			}
			case ufo::InverseNaNBehavior::ZERO: return 0.0f;
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Misses                                        |
	|                                                                                     |
	**************************************************************************************/

	template <class Map, class T>
	void findMisses(Map const& map, Transform<Dim, T> const& transform,
	                bool with_count) const
	{
		auto& miss_grid  = this->misses_grid_;
		auto& count_grid = this->count_grid_;

		auto const depth = info_and_lut_.numLevels() - 1;
		for (auto const& info : info_and_lut_) {
			std::uint32_t index = static_cast<std::uint32_t>(&info - info_and_lut_.data());

			auto [seen, possibly_seen] =
			    seenOrPossiblySeen(info.minDistance(), info.maxDistance(), index, depth);

			if (0u < info.count() && seen) {
				if (with_count) {
					addMiss(map, transform, miss_grid, count_grid, index, info.count(), depth);
				} else {
					addMiss(map, transform, miss_grid, index, depth);
				}
			} else if (possibly_seen) {
				if (with_count) {
					findMissesRecurs<true>(map, transform, miss_grid, count_grid, index, depth);
				} else {
					findMissesRecurs<false>(map, transform, miss_grid, count_grid, index, depth);
				}
			}
		}
	}

	template <
	    class ExecutionPolicy, class Map, class T,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void findMisses(ExecutionPolicy&& policy, Map const& map,
	                Transform<Dim, T> const& transform, bool with_count) const
	{
		std::atomic_size_t size = 0;
		std::mutex         mutex;

		auto const depth = info_and_lut_.numLevels() - 1;
		for_each(policy, info_and_lut_.begin(), info_and_lut_.end(),
		         [&, first = true](auto const& info) mutable {
			         thread_local std::size_t grid_idx;
			         auto const               id = std::this_thread::get_id();
			         if (size.load(std::memory_order_relaxed) <= grid_idx ||
			             id != std::get<0>(local_misses_grid_[grid_idx])) {
				         std::scoped_lock lock(mutex);
				         grid_idx = size;
				         if (local_misses_grid_.size() <= grid_idx) {
					         local_misses_grid_.emplace_back();
					         assert(local_misses_grid_.size() >= grid_idx);
				         }
				         std::get<0>(local_misses_grid_[grid_idx]) = id;
				         ++size;
			         }

			         auto& misses_grid = std::get<1>(local_misses_grid_[grid_idx]);
			         auto& count_grid  = std::get<2>(local_misses_grid_[grid_idx]);

			         std::uint32_t index =
			             static_cast<std::uint32_t>(&info - info_and_lut_.data());

			         auto [seen, possibly_seen] = seenOrPossiblySeen(
			             info.minDistance(), info.maxDistance(), index, depth);

			         if (0u < info.count() && seen) {
				         if (with_count) {
					         addMiss(map, transform, misses_grid, count_grid, index, info.count(),
					                 depth);
				         } else {
					         addMiss(map, transform, misses_grid, index, depth);
				         }
			         } else if (possibly_seen) {
				         if (with_count) {
					         findMissesRecurs<true>(map, transform, misses_grid, count_grid, index,
					                                depth);
				         } else {
					         findMissesRecurs<false>(map, transform, misses_grid, count_grid, index,
					                                 depth);
				         }
			         }
		         });

		std::unordered_map<TreeCode<Dim>, unsigned> key_to_index;
		unsigned                                    index{};
		for (std::size_t i{}, s = size; s > i; ++i) {
			ufo::for_each(std::get<1>(local_misses_grid_[i]).begin(),
			              std::get<1>(local_misses_grid_[i]).end(),
			              [this, &key_to_index, &index](auto const& e) {
				              if (auto it = key_to_index.find(e.first);
				                  key_to_index.end() == it) {
					              key_to_index[e.first] = index++;
					              this->misses_grid_.addKey(e.first);
					              this->count_grid_.addKey(e.first);
				              }
			              });

			ufo::for_each(policy, std::get<1>(local_misses_grid_[i]).begin(),
			              std::get<1>(local_misses_grid_[i]).end(),
			              [this, &key_to_index](auto const& e) {
				              auto& v0 = this->misses_grid_[key_to_index[e.first]];
				              ufo::transform(v0.begin(), v0.end(), e.second.begin(), v0.begin(),
				                             [](auto a, auto b) { return a | b; });
			              });

			ufo::for_each(policy, std::get<2>(local_misses_grid_[i]).begin(),
			              std::get<2>(local_misses_grid_[i]).end(),
			              [this, &key_to_index](auto const& e) {
				              auto& v0 = this->count_grid_[key_to_index[e.first]];
				              ufo::transform(v0.begin(), v0.end(), e.second.begin(), v0.begin(),
				                             [](auto a, auto b) {
					                             // TODO: What should this be?
					                             return std::max(a, b);
				                             });
			              });

			std::get<0>(local_misses_grid_[i]) = {};
			std::get<1>(local_misses_grid_[i]).clear();
			std::get<2>(local_misses_grid_[i]).clear();
		}
	}

	template <bool WithCount, class Map, class T>
	void findMissesRecurs(Map const& map, Transform<Dim, T> const& transform,
	                      detail::GridMap<BoolGrid, Dim, InverseLevels>&  miss_grid,
	                      detail::GridMap<CountGrid, Dim, InverseLevels>& count_grid,
	                      std::uint32_t index, unsigned depth) const
	{
		auto first_node = info_and_lut_.firstChild(index, depth);
		auto last_node  = info_and_lut_.lastChild(index, depth);
		--depth;

		if (0u == depth) {  // Leaf layer
			for (std::size_t i = first_node; last_node > i; ++i) {
				auto  coord    = info_and_lut_.point(i);
				float distance = info_and_lut_.distance(i);

				auto first_lut = info_and_lut_.beginLut(i, 0u);
				auto last_lut  = info_and_lut_.endLut(i, 0u);

				unsigned count{};
				for (; last_lut > first_lut; ++first_lut) {
					auto d = distances_[*first_lut];
					if (!(distance <= d) && !std::isnan(d)) {
						count = 0u;
						break;
					}
					++count;
				}

				if (0u < count) {
					if constexpr (WithCount) {
						addMiss(map, transform, miss_grid, count_grid, i, count, depth);
					} else {
						addMiss(map, transform, miss_grid, i, depth);
					}
				}
			}
		} else {  // All other layers
			for (std::size_t i = first_node; last_node > i; ++i) {
				auto  coord        = info_and_lut_.point(i, depth);
				float min_distance = info_and_lut_.minDistance(i, depth);
				float max_distance = info_and_lut_.maxDistance(i, depth);

				auto [seen, possibly_seen] =
				    seenOrPossiblySeen(min_distance, max_distance, i, depth);

				auto count = info_and_lut_.count(i, depth);
				if (0u < count && seen) {
					if constexpr (WithCount) {
						addMiss(map, transform, miss_grid, count_grid, i, count, depth);
					} else {
						addMiss(map, transform, miss_grid, i, depth);
					}
				} else if (possibly_seen) {
					findMissesRecurs<WithCount>(map, transform, miss_grid, count_grid, i, depth);
				}
			}
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Something                                      |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] std::pair<bool, bool> seenOrPossiblySeen(
	    float min_distance, float max_distance,
	    std::vector<std::uint32_t>::const_iterator first,
	    std::vector<std::uint32_t>::const_iterator last) const
	{
		bool seen_or_nan       = true;
		bool at_least_one_seen = false;
		bool possibly_seen     = false;
		for (; last > first; ++first) {
			auto d            = distances_[*first];
			auto seen         = max_distance <= d;
			seen_or_nan       = seen_or_nan && (seen || std::isnan(d));
			at_least_one_seen = at_least_one_seen || seen;
			possibly_seen     = possibly_seen || min_distance <= d;
		}
		return std::pair(seen_or_nan && at_least_one_seen, possibly_seen);
	}

	[[nodiscard]] std::pair<bool, bool> seenOrPossiblySeen(float         min_distance,
	                                                       float         max_distance,
	                                                       std::uint32_t index,
	                                                       unsigned      depth) const
	{
		return seenOrPossiblySeen(min_distance, max_distance,
		                          info_and_lut_.beginLut(index, depth),
		                          info_and_lut_.endLut(index, depth));
	}

	template <class Map, class T>
	void addMiss(Map const& map, Transform<Dim, T> const& transform,
	             detail::GridMap<BoolGrid, Dim, InverseLevels>& miss_grid,
	             std::uint32_t index, unsigned depth) const
	{
		auto md = this->miss_depth;

		if (0 == depth) {
			auto coord = TreeCoord<Dim>(transform(info_and_lut_.point(index)), md);
			auto code  = map.code(coord);
			miss_grid[code].set(code);
			return;
		}

		auto point = info_and_lut_.point(index, depth);

		TreeKey<Dim> key;
		auto         min = map.key(TreeCoord<Dim>(point, md + depth));
		auto         max = min + 1u;
		min.setDepth(md);
		max.setDepth(md);
		key.setDepth(md);
		if constexpr (2 == Dim) {
			for (key.y = min.y; max.y > key.y; ++key.y) {
				for (key.x = min.x; max.x > key.x; ++key.x) {
					auto coord = TreeCoord<Dim>(transform(map.center(key)), md);
					auto code  = map.code(coord);
					miss_grid[code].set(code);
				}
			}
		} else if constexpr (3 == Dim) {
			for (key.z = min.z; max.z > key.z; ++key.z) {
				for (key.y = min.y; max.y > key.y; ++key.y) {
					for (key.x = min.x; max.x > key.x; ++key.x) {
						auto coord = TreeCoord<Dim>(transform(map.center(key)), md);
						auto code  = map.code(coord);
						miss_grid[code].set(code);
					}
				}
			}
		}
	}

	template <class Map, class T>
	void addMiss(Map const& map, Transform<Dim, T> const& transform,
	             detail::GridMap<BoolGrid, Dim, InverseLevels>&  miss_grid,
	             detail::GridMap<CountGrid, Dim, InverseLevels>& count_grid,
	             std::uint32_t index, unsigned count, unsigned depth) const
	{
		auto md = this->miss_depth;

		// TODO: What should this be?
		count =
		    std::min(static_cast<unsigned>(std::numeric_limits<std::uint8_t>::max()), count);

		if (0 == depth) {
			auto coord = TreeCoord<Dim>(transform(info_and_lut_.point(index)), md);
			auto code  = map.code(coord);
			miss_grid[code].set(code);
			count_grid[code][code] = count;  // TODO: What should this be?
			return;
		}

		auto point = info_and_lut_.point(index, depth);

		TreeKey<Dim> key;
		auto         min = map.key(TreeCoord<Dim>(point, md + depth));
		auto         max = min + 1u;
		min.setDepth(md);
		max.setDepth(md);
		key.setDepth(md);
		if constexpr (2 == Dim) {
			for (key.y = min.y; max.y > key.y; ++key.y) {
				for (key.x = min.x; max.x > key.x; ++key.x) {
					auto coord = TreeCoord<Dim>(transform(map.center(key)), md);
					auto code  = map.code(coord);
					miss_grid[code].set(code);
					count_grid[code][code] = count;  // TODO: What should this be?
				}
			}
		} else if constexpr (3 == Dim) {
			for (key.z = min.z; max.z > key.z; ++key.z) {
				for (key.y = min.y; max.y > key.y; ++key.y) {
					for (key.x = min.x; max.x > key.x; ++key.x) {
						auto coord = TreeCoord<Dim>(transform(map.center(key)), md);
						auto code  = map.code(coord);
						miss_grid[code].set(code);
						count_grid[code][code] = count;  // TODO: What should this be?
					}
				}
			}
		}
	}

	// TODO: Move this somewhere else
	template <class Map, class T>
	void addCoords(std::vector<TreeCoord<Dim, float>>& coords, Map const& map,
	               Transform<Dim, T> const& transform, std::uint32_t index,
	               unsigned depth) const
	{
		// TODO: Fix depth here

		auto center = info_and_lut_.point(index, depth);
		depth += this->miss_depth;
		if (0u == depth) {
			coords.emplace_back(transform(center), depth);
		} else {
			std::vector<std::pair<TreeCode<Dim>, std::vector<std::uint64_t>>> grid;

			std::size_t num = std::max(1u, (1u << (Dim * depth)) / 64u);

			// using code_t = typename TreeCode<Dim>::code_t;
			// code_t mask  = (~code_t(0) >> (std::numeric_limits<code_t>::digits - Dim *
			// depth));

			using Key = typename TreeKey<Dim>::Key;

			auto min = map.key(TreeCoord<Dim>(center, depth));
			auto max = TreeKey<Dim>(min + Key(1), depth);  // TODO: Add operator+ to Key
			min.setDepth(0u);
			max.setDepth(0u);
			if constexpr (2 == Dim) {
				// TODO: Implement
			} else if constexpr (3 == Dim) {
				for (auto x = min.x; max.x > x; ++x) {
					for (auto y = min.y; max.y > y; ++y) {
						for (auto z = min.z; max.z > z; ++z) {
							auto c  = map.code(transform(map.center(TreeKey<Dim>(Key(x, y, z), 0u))));
							auto lo = c.lowestOffsets();
							c.setDepth(depth);
							lo ^= c.lowestOffsets();  // Only the first `depth` levels bits will be set

							std::vector<std::uint64_t>* v;
							if (auto it = std::find_if(grid.begin(), grid.end(),
							                           [&c](auto const& v) { return v.first == c; });
							    grid.end() != it) {
								v = &it->second;
							} else {
								v = &grid.emplace_back(c, std::vector<std::uint64_t>(num)).second;
							}

							(*v)[lo / 64u] |= std::uint64_t(1) << (lo % 64u);
						}
					}
				}

				for (auto const& [c, v] : grid) {
					if (std::all_of(v.begin(), v.end(), [](auto e) {
						    return std::numeric_limits<std::uint64_t>::max() == e;
					    })) {
						coords.push_back(map.center(c));
						continue;
					}

					auto         lo = c.lowestOffsets();
					decltype(lo) j{};
					for (auto e : v) {
						if (std::numeric_limits<std::uint64_t>::max() == e) {
							auto t = c;
							t.lowestOffsets(lo | j, 2u);  // TODO: Is correct?
							coords.push_back(map.center(t));
						} else if (std::uint64_t(0) != e) {
							for (std::uint64_t k{}; 64u > k; k += 8u) {
								std::uint64_t mask = 0xFFu << k;
								if (mask == (mask & e)) {
									auto t = c;
									t.lowestOffsets(lo | j | k, 1u);  // TODO: Is correct?
									coords.push_back(map.center(t));
									continue;
								}

								for (std::uint64_t m{}; 8u > m; ++m) {
									std::uint64_t mask = 0x1u << (k + m);
									if (mask == (mask & e)) {
										auto t = c;
										t.lowestOffsets(lo | j | k | m, 0u);  // TODO: Is correct?
										coords.push_back(map.center(t));
									}
								}
							}
						}

						j += 64u;
					}
				}

				// for (auto const& [k, _] : tmp[0]) {
				// 	coords.emplace_back(map.center(k), 0u);
				// }
			}
			// center           = transform(center);
			// auto half_length = cast<float>(map.halfLength(depth) + map.halfLength(0));
			// // OBB<Dim, float> obb(center, half_length);
			// AABB<Dim, float> obb(center - half_length, center + half_length);
			// // TODO: Apply transform transform(obb)
			// for (auto n : map.query(pred::Inside(obb), false, true)) {
			// 	coords.emplace_back(map.center(n.code), map.depth(n));
			// }
		}
	}

 public:
	detail::InverseInfoAndLut<Dim, InverseLevels> info_and_lut_;
	std::vector<Vec<Dim, double>>                 directions_;

	// TreeMap<Dim, unsigned> dir_to_index_;
	std::unordered_map<std::uint64_t, unsigned> dir_to_index_;

	mutable __block std::vector<float> distances_;

	mutable __block std::deque<
	    std::tuple<std::thread::id, detail::GridMap<BoolGrid, Dim, InverseLevels>,
	               detail::GridMap<CountGrid, Dim, InverseLevels>>>
	    local_misses_grid_;
};
}  // namespace ufo

#endif  // UFO_MAP_INTEGRATOR_INVERSE_INTEGRATOR_HPP