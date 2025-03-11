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
#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <future>
#include <limits>
#include <mutex>
#include <tuple>
#include <vector>

namespace ufo
{
enum class SensorType {
	RAY,
	/* CONE, */
	FRUSTUM
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
		auto misses_f = std::async(std::launch::deferred, [this, &map, points = get<0>(cloud),
		                                                   &transform]() {
			if (maximum_points_.size() != points.size()) {
				if (!maximum_points_.empty()) {
					std::cerr << "Point cloud has not the same size as the previous, re-generating "
					             "config\n";
				}
				const_cast<InverseIntegrator&>(*this).generateConfig(map, points);
			}

			misses(map, points, transform);
			if constexpr (Map::hasMapTypes(MapType::VOID_REGION)) {
				voidRegions();
			}
		});

		filterDistanceInPlace(cloud, Vec<Dim, T>{}, this->min_distance, this->max_distance,
		                      true);
		transformInPlace(transform, get<0>(cloud));
		this->create(map, hit_nodes_, get<0>(cloud), this->hit_depth);

		misses_f.wait();

		std::cout << misses_nodes_.size() << '\n';

		this->create(map, miss_nodes_, misses_nodes_);

		this->insertMisses(map, miss_nodes_, misses_info_);
		this->insertHits(map, hit_nodes_, cloud);

		// TODO: Implement

		if (propagate) {
			// TODO: Implement
			map.modifiedPropagate();
		}
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

		auto misses_f = std::async(std::launch::async, [this, policy, &map,
		                                                points = get<0>(cloud),
		                                                &transform]() {
			if (maximum_points_.size() != points.size()) {
				if (!maximum_points_.empty()) {
					std::cerr << "Point cloud has not the same size as the previous, re-generating "
					             "config\n";
				}
				const_cast<InverseIntegrator&>(*this).generateConfig(map, points);
			}

			misses(policy, map, points, transform);
			if constexpr (Map::hasMapTypes(MapType::VOID_REGION)) {
				voidRegions(policy);
			}
		});

		filterDistanceInPlace(cloud, Vec<Dim, T>{}, this->min_distance, this->max_distance,
		                      true);
		transformInPlace(transform, get<0>(cloud));
		this->create(map, hit_nodes_, get<0>(cloud), this->hit_depth);

		misses_f.wait();

		std::cout << misses_nodes_.size() << '\n';

		// misses_nodes_.resize(misses_nodes_.size() / 10);
		// misses_info_.resize(misses_info_.size() / 10);
		this->create(policy, map, miss_nodes_, misses_nodes_);

		this->insertMisses(policy, map, miss_nodes_, misses_info_);
		this->insertHits(policy, map, hit_nodes_, cloud);

		// TODO: Implement

		if (propagate) {
			// TODO: Implement
			map.modifiedPropagate(policy);
		}
	}

	void loadConfig(std::filesystem::path const& file)
	{
		// TODO: Implement
	}

	void saveConfig(std::filesystem::path const& file)
	{
		// TODO: Implement
	}

	template <class Map, class T, class... Rest>
	void generateConfig(Map const& map, PointCloud<Dim, T, Rest...> const& cloud)
	{
		generateConfig(map, get<0>(cloud));
	}

 private:
	template <class Map, class T>
	void generateConfig(Map const& map, std::vector<Vec<Dim, T>> const& points)
	{
		std::cout << "Creating inverse map\n";
		auto inv_map = inverseMap(map.length(this->miss_depth),
		                          map.numDepthLevels() - this->miss_depth, points);
		std::cout << "Updating inverse structures\n";
		updateInverseStructures(inv_map);
		std::cout << "Generated config\n";
	}

	template <class T>
	[[nodiscard]] Map<Dim, detail::InverseMap> inverseMap(
	    Vec<Dim, double> const& leaf_node_length, unsigned num_depth_levels,
	    std::vector<Vec<Dim, T>> const& points) const
	{
		Map<Dim, detail::InverseMap> map(leaf_node_length, num_depth_levels);

		maximum_points_.resize(points.size());

		std::vector<TreeCode<Dim>> codes;
		for (std::size_t i{}; points.size() > i; ++i) {
			if (isnan(points[i])) {
				maximum_points_nan_indices_.push_back(i);
				continue;
			}

			maximum_points_[i] = this->max_distance * normalize(points[i]);

			codes.clear();

			switch (sensor_type) {
				case SensorType::RAY: {
					LineSegment<Dim, T> line(Vec<Dim, T>{},
					                         this->max_distance * normalize(points[i]));
					for (auto n : map.query(pred::PureLeaf() && pred::Intersects(line), false)) {
						codes.push_back(n.code);
					}
					break;
				}
				case SensorType::FRUSTUM: {
					// TODO: Implement
					Frustum<Dim, T> frustum;
					for (auto n : map.query(pred::PureLeaf() && pred::Intersects(frustum), false)) {
						codes.push_back(n.code);
					}
					break;
				}
			}

			auto const nodes = map.create(codes);
			for (auto n : nodes) {
				map.inverseIndices(n).push_back(i);
				map.inverseDistance(n) = normSquared(map.center(n));
			}
		}

		map.propagate(MapType::ALL, false);

		std::uint32_t index{};
		for (auto n : map.query(pred::PureLeaf(), true, false)) {
			if (map.inverseIndices(n.index).empty()) {
				continue;
			}
			map.inverseIndex(n.index) = index++;
		}

		return map;
	}

	void updateInverseStructures(Map<Dim, detail::InverseMap> const& inv_map) const
	{
		info_and_lut_middle_.resize(inverse_levels_ - 2);

		info_and_lut_top_.info.clear();
		info_and_lut_top_.lut.clear();
		info_and_lut_bottom_.info.clear();
		info_and_lut_bottom_.lut.clear();
		info_and_lut_bottom_.lut_void.clear();
		for (auto& e : info_and_lut_middle_) {
			e.info.clear();
			e.lut.clear();
		}

		for (auto n : inv_map.query(pred::Depth() < inverse_levels_, true, false)) {
			auto const& indices = inv_map.inverseIndices(n.index);

			if (indices.empty()) {
				continue;
			}

			float distance = inv_map.inverseDistance(n.index);

			std::vector<std::uint32_t>* lut;

			auto depth = inv_map.depth(n);
			if (0 == depth) {
				// Bottom layer
				auto& info     = info_and_lut_bottom_.info;
				lut            = &info_and_lut_bottom_.lut;
				auto& lut_void = info_and_lut_bottom_.lut_void;

				info.emplace_back(inv_map.center(n.code), distance, lut->size(), lut_void.size());

				Vec<Dim, float> center = inv_map.center(n.code);
				Vec<Dim, float> half_length =
				    cast<float>(inv_map.length(0) * (void_region_distance_ / 2.0));
				AABB<Dim, float> aabb(center - half_length, center + half_length);
				for (auto m : inv_map.query(pred::PureLeaf() && pred::Intersects(aabb), false)) {
					if (n.index == m.index) {
						continue;
					}
					lut_void.push_back(inv_map.inverseIndex(m.index));
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

				info.emplace_back(distance, lut->size(), lut->size() + indices.size(),
				                  first_child, first_child + num_children);
			} else {
				// Middle layer
				auto& info = info_and_lut_middle_[depth - 1].info;
				lut        = &info_and_lut_middle_[depth - 1].lut;

				std::uint32_t first_child = 1 == depth
				                                ? info_and_lut_bottom_.info.size()
				                                : info_and_lut_middle_[depth - 2].info.size();

				info.emplace_back(distance, lut->size(), first_child);
			}

			lut->insert(lut->end(), indices.begin(), indices.end());
		}

		info_and_lut_top_.info.shrink_to_fit();
		info_and_lut_top_.lut.shrink_to_fit();
		info_and_lut_bottom_.info.shrink_to_fit();
		info_and_lut_bottom_.lut.shrink_to_fit();
		info_and_lut_bottom_.lut_void.shrink_to_fit();
		for (auto& e : info_and_lut_middle_) {
			e.info.shrink_to_fit();
			e.lut.shrink_to_fit();
		}
	}

	template <class Map, class T>
	void misses(Map const& map, std::vector<Vec<Dim, T>> const& points,
	            Transform<Dim, T> const& transform) const
	{
		misses_nodes_.clear();
		misses_info_.clear();

		sq_distances_.resize(points.size());
		std::transform(points.begin(), points.end(), sq_distances_.begin(),
		               [](auto const& point) { return normSquared(point); });

		auto const  depth = inverse_levels_ - 1;
		auto const& lut   = info_and_lut_top_.lut;
		for (auto const& info : info_and_lut_top_.info) {
			float         distance    = info.distance;
			std::uint32_t first_lut   = info.first_lut;
			std::uint32_t last_lut    = info.last_lut;
			std::uint32_t first_child = info.first_child;
			std::uint32_t last_child  = info.last_child;

			for (std::size_t i = first_lut; last_lut > i; ++i) {
				if (distance <= sq_distances_[lut[i]]) {
					missesRecurs(misses_nodes_, misses_info_, transform, depth - 1, first_child,
					             last_child);
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
		misses_nodes_.clear();
		misses_info_.clear();

		sq_distances_.resize(points.size());
		ufo::transform(policy, points.begin(), points.end(), sq_distances_.begin(),
		               [](auto const& point) { return normSquared(point); });

		std::atomic_size_t local_misses_size = 0;
		std::mutex         mutex;

		for_each(policy, info_and_lut_top_.info.begin(), info_and_lut_top_.info.end(),
		         [&](auto const& info) {
			         remove_cvref_t<decltype(misses_nodes_)>* misses_nodes = nullptr;
			         remove_cvref_t<decltype(misses_info_)>*  misses_info  = nullptr;

			         {
				         std::scoped_lock lock(mutex);
				         auto const       id = std::this_thread::get_id();
				         for (std::size_t i = 0, l = local_misses_size; l > i; ++i) {
					         if (id == std::get<0>(local_misses_[i])) {
						         misses_nodes = &std::get<1>(local_misses_[i]);
						         misses_info  = &std::get<2>(local_misses_[i]);
						         break;
					         }
				         }

				         if (nullptr == misses_nodes /* || nullptr == misses_info */) {
					         if (local_misses_.size() > local_misses_size) {
						         std::get<0>(local_misses_[local_misses_size]) = id;
						         misses_nodes = &std::get<1>(local_misses_[local_misses_size]);
						         misses_info  = &std::get<2>(local_misses_[local_misses_size]);
						         misses_nodes->clear();
						         misses_info->clear();
					         } else {
						         auto& tmp = local_misses_.emplace_back(
						             id, remove_cvref_t<decltype(misses_nodes_)>{},
						             remove_cvref_t<decltype(misses_info_)>{});
						         misses_nodes = &std::get<1>(tmp);
						         misses_info  = &std::get<2>(tmp);
					         }
					         ++local_misses_size;
				         }
			         }

			         float         distance    = info.distance;
			         std::uint32_t first_lut   = info.first_lut;
			         std::uint32_t last_lut    = info.last_lut;
			         std::uint32_t first_child = info.first_child;
			         std::uint32_t last_child  = info.last_child;

			         for (std::size_t i = first_lut; last_lut > i; ++i) {
				         if (distance <= sq_distances_[info_and_lut_top_.lut[i]]) {
					         missesRecurs(*misses_nodes, *misses_info, transform,
					                      inverse_levels_ - 2, first_child, last_child);
					         return;
				         }
			         }
		         });

		std::size_t s{};
		std::vector<std::tuple<std::size_t, remove_cvref_t<decltype(misses_nodes_)>*,
		                       remove_cvref_t<decltype(misses_info_)>*>>
		    tmp;
		tmp.resize(local_misses_size);
		for (std::size_t i{}; tmp.size() > i; ++i) {
			tmp[i] = {s, &std::get<1>(local_misses_[i]), &std::get<2>(local_misses_[i])};
			s += std::get<1>(local_misses_[i]).size();
		}

		misses_nodes_.resize(s);
		misses_info_.resize(s);

		for_each(policy, tmp.begin(), tmp.end(), [this](auto const& e) {
			std::size_t s = std::get<0>(e);
			std::copy(std::get<1>(e)->begin(), std::get<1>(e)->end(),
			          misses_nodes_.begin() + s);
			std::copy(std::get<2>(e)->begin(), std::get<2>(e)->end(), misses_info_.begin() + s);
		});
	}

	template <class T>
	void missesRecurs(std::vector<TreeCoord<Dim, float>>&     misses_nodes,
	                  std::vector<detail::InverseMissesInfo>& misses_info,
	                  Transform<Dim, T> const& transform, unsigned depth,
	                  std::uint32_t first_node, std::uint32_t last_node) const
	{
		if (0 == depth) {
			auto&       info = info_and_lut_bottom_.info;
			auto const& lut  = info_and_lut_bottom_.lut;

			for (std::size_t i = first_node; last_node > i; ++i) {
				float distance = info[i].distance;

				std::uint32_t first_lut = info[i].first_lut;
				std::uint32_t last_lut = info.size() > i + 1 ? info[i + 1].first_lut : lut.size();

				unsigned num{};

				for (std::size_t j = first_lut; last_lut > j; ++j) {
					if (distance <= sq_distances_[lut[j]]) {
						++num;
					}
				}

				info[i].seen(0 != num);

				if (0 == num) {
					continue;
				}

				// TODO: Implement
				// OBB obb;
				// obb = transform(obb);
				// for (auto n : map.query(pred::Inside(obb) || (pred::PureLeaf() &&
				// pred::Intersects(obb)))) {

				// }

				misses_nodes.emplace_back(transform(info[i].point), this->miss_depth);
				misses_info.emplace_back(i, num);
			}
		} else {
			auto const& info = info_and_lut_middle_[depth - 1].info;
			auto const& lut  = info_and_lut_middle_[depth - 1].lut;

			for (std::size_t i = first_node; last_node > i; ++i) {
				std::uint32_t first_lut = info[i].first_lut;
				std::uint32_t last_lut = info.size() > i + 1 ? info[i + 1].first_lut : lut.size();

				std::uint32_t first_child = info[i].first_child;
				std::uint32_t last_child =
				    info.size() > i + 1
				        ? info[i + 1].first_child
				        : (1 == depth ? info_and_lut_bottom_.info.size()
				                      : info_and_lut_middle_[depth - 2].info.size());

				float distance = info[i].distance;

				for (std::size_t j = first_lut; last_lut > j; ++j) {
					if (distance <= sq_distances_[lut[j]]) {
						missesRecurs(misses_nodes, misses_info, transform, depth - 1, first_child,
						             last_child);
						break;
					}
				}
			}
		}
	}

	void voidRegions() const
	{
		auto const& lut_void = info_and_lut_bottom_.lut_void;
		for (auto& mi : misses_info_) {
			auto const&   info           = info_and_lut_bottom_.info[mi.index];
			std::uint32_t first_lut_void = info.firstLutVoid();
			std::uint32_t last_lut_void =
			    info_and_lut_bottom_.info.size() > mi.index + 1
			        ? info_and_lut_bottom_.info[mi.index + 1].firstLutVoid()
			        : lut_void.size();

			if (first_lut_void == last_lut_void) {
				continue;
			}

			bool void_region = true;
			for (std::size_t i = first_lut_void; last_lut_void > i; ++i) {
				if (!info_and_lut_bottom_.info[lut_void[i]].seen()) {
					void_region = false;
					break;
				}
			}

			mi.voidRegion(void_region);
		}

		// Clear all seen flags
		for (auto const& mi : misses_info_) {
			info_and_lut_bottom_.info[mi.index].seen(false);
		}
	}

	template <
	    class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void voidRegions(ExecutionPolicy&& policy) const
	{
		for_each(policy, misses_info_.begin(), misses_info_.end(), [this](auto& mi) {
			auto const& lut_void = info_and_lut_bottom_.lut_void;

			auto const&   info           = info_and_lut_bottom_.info[mi.index];
			std::uint32_t first_lut_void = info.firstLutVoid();
			std::uint32_t last_lut_void =
			    info_and_lut_bottom_.info.size() > mi.index + 1
			        ? info_and_lut_bottom_.info[mi.index + 1].firstLutVoid()
			        : lut_void.size();

			if (first_lut_void == last_lut_void) {
				return;
			}

			for (std::size_t i = first_lut_void; last_lut_void > i; ++i) {
				if (!info_and_lut_bottom_.info[lut_void[i]].seen()) {
					return;
				}
			}

			mi.voidRegion(true);
		});

		// Clear all seen flags
		for_each(policy, misses_info_.begin(), misses_info_.end(),
		         [this](auto& mi) { info_and_lut_bottom_.info[mi.index].seen(false); });
	}

 private:
	// Needs to be at least 2
	std::size_t inverse_levels_       = 5;
	double      void_region_distance_ = 1.0;

	mutable std::vector<Vec<Dim, float>> maximum_points_;
	mutable std::vector<std::size_t>     maximum_points_nan_indices_;

	mutable detail::InverseInfoAndLutTop                 info_and_lut_top_;
	mutable std::vector<detail::InverseInfoAndLutMiddle> info_and_lut_middle_;
	mutable detail::InverseInfoAndLutBottom<Dim>         info_and_lut_bottom_;

	mutable std::vector<float> sq_distances_;

	mutable __block std::vector<TreeIndex> hit_nodes_;
	mutable __block std::vector<TreeIndex> miss_nodes_;

	mutable std::vector<TreeCoord<Dim, float>>     misses_nodes_;
	mutable std::vector<detail::InverseMissesInfo> misses_info_;
	mutable std::vector<TreeCoord<Dim, float>>     void_region_points_;

	mutable std::deque<std::tuple<std::thread::id, std::vector<TreeCoord<Dim, float>>,
	                              std::vector<detail::InverseMissesInfo>>>
	    local_misses_;
};
}  // namespace ufo

#endif  // UFO_MAP_INTEGRATOR_INVERSE_INTEGRATOR_HPP