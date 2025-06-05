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

#ifndef UFO_MAP_INTEGRATOR_ANGULAR_INTEGRATOR_HPP
#define UFO_MAP_INTEGRATOR_ANGULAR_INTEGRATOR_HPP

// UFO
#include <ufo/cloud/point_cloud.hpp>
#include <ufo/container/tree/code.hpp>
#include <ufo/container/tree/index.hpp>
#include <ufo/container/tree/key.hpp>
#include <ufo/execution/algorithm.hpp>
#include <ufo/execution/execution.hpp>
#include <ufo/geometry/aabb.hpp>
#include <ufo/geometry/geometry.hpp>
#include <ufo/map/integrator/integrator.hpp>
#include <ufo/map/integrator/sensor_error.hpp>
#include <ufo/math/math.hpp>
#include <ufo/math/numbers.hpp>
#include <ufo/math/transform.hpp>
#include <ufo/math/vec.hpp>
#include <ufo/utility/spinlock.hpp>

// STL
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <future>
#include <numeric>
#include <thread>
#include <vector>

namespace ufo
{
template <std::size_t Dim, class SensorErrorFun = FixedSensorError>
class AngularIntegrator final : public Integrator<Dim>
{
 public:
	// Min range to integrate
	float min_distance = 0.0f;
	// Max range to integrate, negative value is infinity range
	float max_distance = std::numeric_limits<float>::infinity();

	float sensor_angular_resolution = radians(0.0f);

	float translation_error = 0.0f;
	float orientation_error = radians(0.0f);

 public:
	AngularIntegrator(SensorErrorFun sensor_error_f = SensorErrorFun())
	    : sensor_error_f_(sensor_error_f)
	{
		angularResolution(radians(1.0f));
	}

	template <class Map, class T, class... Rest>
	void operator()(Map& map, PointCloud<Dim, T, Rest...> cloud,
	                Transform<Dim, T> const& frame_origin  = {},
	                Vec<Dim, T>              sensor_origin = {}) const
	{
		// TODO: Fill in
	}

	template <
	    class ExecutionPolicy, class Map, class T, class... Rest,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void operator()(ExecutionPolicy&& policy, Map& map, PointCloud<Dim, T, Rest...> cloud,
	                Transform<Dim, T> const& frame_origin        = {},
	                Vec<Dim, T>              sensor_origin       = {},
	                std::future<void> const& finish_before_touch = {}) const
	{
		// TODO: Filter distances

		if constexpr (execution::is_seq_v<ExecutionPolicy> ||
		              execution::is_unseq_v<ExecutionPolicy>) {
			return operator()(map, cloud, transform);
		}

		auto const t0 = std::chrono::high_resolution_clock::now();

		fillDistances(policy, cloud.template view<0>(), sensor_origin);

		removeNanAndMinDistance(cloud);

		auto const t1 = std::chrono::high_resolution_clock::now();

		if (Transform<Dim, T>{} != frame_origin) {
			transformInPlace(policy, frame_origin, cloud);
			sensor_origin = frame_origin(sensor_origin);
		}

		auto const t2 = std::chrono::high_resolution_clock::now();

		fillData(policy, cloud.template view<0>(), sensor_origin);

		auto const t3 = std::chrono::high_resolution_clock::now();

		fillMisses(policy, map, cloud.template view<0>(), sensor_origin);

		auto const t4 = std::chrono::high_resolution_clock::now();

		if (finish_before_touch.valid()) {
			finish_before_touch.wait();
		}

		auto const t5 = std::chrono::high_resolution_clock::now();

		this->insertMisses(policy, map, cached_misses_);

		auto const t6 = std::chrono::high_resolution_clock::now();

		resetData(policy);

		auto const t7 = std::chrono::high_resolution_clock::now();

		filterHits(policy, cloud, sensor_origin);

		auto const t8 = std::chrono::high_resolution_clock::now();

		this->insertHits(policy, map, cloud);

		auto const t9 = std::chrono::high_resolution_clock::now();

		// TODO: Implement

		if (this->propagate) {
			map.propagate(policy);
			// TODO: Reset modified?
		}

		auto const t10 = std::chrono::high_resolution_clock::now();

		std::chrono::duration<double, std::milli> const remove_nan_ms    = t1 - t0;
		std::chrono::duration<double, std::milli> const transform_ms     = t2 - t1;
		std::chrono::duration<double, std::milli> const fill_data_ms     = t3 - t2;
		std::chrono::duration<double, std::milli> const fill_misses_ms   = t4 - t3;
		std::chrono::duration<double, std::milli> const wait_to_touch_ms = t5 - t4;
		std::chrono::duration<double, std::milli> const insert_misses_ms = t6 - t5;
		std::chrono::duration<double, std::milli> const reset_data_ms    = t7 - t6;
		std::chrono::duration<double, std::milli> const filter_hits_ms   = t8 - t7;
		std::chrono::duration<double, std::milli> const insert_hits_ms   = t9 - t8;
		std::chrono::duration<double, std::milli> const propagate_ms     = t10 - t9;
		std::chrono::duration<double, std::milli> const total_ms         = t10 - t0;

		std::cout << "Remove NaN:    " << remove_nan_ms.count() << " ms\n";
		std::cout << "Transform:     " << transform_ms.count() << " ms\n";
		std::cout << "Fill data:     " << fill_data_ms.count() << " ms\n";
		std::cout << "Fill misses:   " << fill_misses_ms.count() << " ms\n";
		std::cout << "Wait to touch: " << wait_to_touch_ms.count() << " ms\n";
		std::cout << "Insert misses: " << insert_misses_ms.count() << " ms\n";
		std::cout << "Reset data:    " << reset_data_ms.count() << " ms\n";
		std::cout << "Filter hits:   " << filter_hits_ms.count() << " ms\n";
		std::cout << "Insert hits:   " << insert_hits_ms.count() << " ms\n";
		std::cout << "Propagate:     " << propagate_ms.count() << " ms\n";
		std::cout << "Total:         " << total_ms.count() << " ms\n\n";
	}

	[[nodiscard]] constexpr float angularResolution() const noexcept
	{
		return 1.0f / angular_resolution_factor_;
	}

	constexpr void angularResolution(float resolution) noexcept
	{
		angular_resolution_factor_ = 1.0f / resolution;
		auto const arf             = angular_resolution_factor_;

		assert(0.0f < resolution && 0.0f < angular_resolution_factor_);

		columns_ = static_cast<std::uint_fast32_t>(2.0f * numbers::pi_v<float> * arf) + 1;

		if constexpr (2 == Dim) {
			rows_ = 1u;
		} else if constexpr (3 == Dim) {
			rows_ = static_cast<std::uint_fast32_t>(numbers::pi_v<float> * arf) + 1;
		}

		data_.resize(rows_ * columns_);
		data_2_.resize(rows_ * columns_ / (ds_ * ds_));
	}

	[[nodiscard]] SensorErrorFun& sensorErrorFunction() { return sensor_error_f_; }

	[[nodiscard]] SensorErrorFun const& sensorErrorFunction() const
	{
		return sensor_error_f_;
	}

 private:
	/**************************************************************************************
	|                                                                                     |
	|                                       Utility                                       |
	|                                                                                     |
	**************************************************************************************/

	template <class T>
	void fillDistances(SoAView<Vec<Dim, T>> const& points, Vec<Dim, T> const& origin) const
	{
		cached_distances_.resize(points.size());
		ufo::transform(points.begin(), points.end(), cached_distances_.begin(),
		               [&origin](auto const& p) { return distance(origin, p); });
	}

	template <
	    class ExecutionPolicy, class T,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void fillDistances(ExecutionPolicy&& policy, SoAView<Vec<Dim, T>> const& points,
	                   Vec<Dim, T> const& origin) const
	{
		cached_distances_.resize(points.size());
		ufo::transform(std::forward<ExecutionPolicy>(policy), points.begin(), points.end(),
		               cached_distances_.begin(),
		               [&origin](auto const& p) { return distance(origin, p); });
	}

	template <class T, class... Rest>
	void removeNanAndMinDistance(PointCloud<Dim, T, Rest...>& cloud) const
	{
		// TODO: Implement

		// auto       first_d = cached_distances_.begin();
		// auto const last_d  = cached_distances_.end();
		// auto       first_c = cloud.begin();

		// for (; last_d != first_d; ++first_d, ++first_c) {
		// 	if (std::isnan(*first_d) || this->min_distance > *first_d) {
		// 		break;
		// 	}
		// }

		// if (last_d == first_d) {
		// 	return;
		// }

		// auto i_d = std::next(first_d);
		// auto i_c = std::next(first_c);
		// for (; last_d != i_d; ++i_d, i_c) {
		// 	if (!std::isnan(*i_d) && this->min_distance <= *i_d) {
		// 		std::iter_swap(i_d, first_d);
		// 		std::iter_swap(i_c, first_c);
		// 		++first_d;
		// 		++first_c;
		// 	}
		// }

		// cached_distances_.erase(first_d, cached_distances_.end());
		// cloud.erase(first_c, cloud.end());
	}

	template <class T>
	[[nodiscard]] AABB<Dim, T> bounds(SoAView<Vec<Dim, T>> points,
	                                  Vec<Dim, T> const&   origin) const
	{
		// TODO: Optimize

		Vec<Dim, T> min = origin;
		Vec<Dim, T> max = origin;
		for (std::size_t i{}; points.size() > i; ++i) {
			auto p    = points[i];
			auto dir  = p - origin;
			T    dist = static_cast<T>(cached_distances_[i]);
			dir /= dist;
			dist = std::min(static_cast<T>(this->max_distance), dist);
			// FIXME: Only need to call this two times
			dist += sensor_error_f_(dist);
			p   = origin + dir * dist;
			min = ufo::min(min, p);
			max = ufo::max(max, p);
		}

		return AABB<Dim, T>{min, max};
	}

	template <class T>
	[[nodiscard]] std::int_fast32_t polarIndex(T polar_angle) const
	{
		if constexpr (2u == Dim) {
			return 0;
		} else if constexpr (3u == Dim) {
			return static_cast<std::int_fast32_t>(
			    std::floor(polar_angle * angular_resolution_factor_));
		}
	}

	template <class T>
	[[nodiscard]] std::int_fast32_t azimuthalIndex(T azimuthal_angle) const
	{
		azimuthal_angle += numbers::pi_v<T>;
		return static_cast<std::int_fast32_t>(
		    std::floor(azimuthal_angle * angular_resolution_factor_));
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Data                                         |
	|                                                                                     |
	**************************************************************************************/

	template <class T>
	void fillData(SoAView<Vec<Dim, T>> points, Vec<Dim, T> const& origin) const
	{
		// TODO: Implement
	}

	template <
	    class ExecutionPolicy, class T,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void fillData(ExecutionPolicy&& policy, SoAView<Vec<Dim, T>> points,
	              Vec<Dim, T> const& origin) const
	{
		// Spherical coordinate system

		assert(0.0f <= sensor_angular_resolution);
		auto const hsar = sensor_angular_resolution / 2.0f;

		ufo::for_each(std::forward<ExecutionPolicy>(policy), std::size_t(0), points.size(),
		              [this, points, &origin, hsar](std::size_t i) {
			              auto p = points[i];
			              p -= origin;
			              float const radial_distance = cached_distances_[i];
			              float const azimuthal_angle = azimuthalAngle(Vec<2, T>(p));
			              float       polar_angle;
			              if constexpr (2 == Dim) {
				              polar_angle = 0.0f;
			              } else if constexpr (3 == Dim) {
				              polar_angle = polarAngle(p.z, radial_distance);
			              }

			              // TODO: Need to check if `polar_angle +- hsar` is between [0, pi]

			              auto const min_ai = azimuthalIndex(azimuthal_angle - hsar) + columns_;
			              auto const max_ai = azimuthalIndex(azimuthal_angle + hsar) + columns_;
			              auto const min_pi = polarIndex(polar_angle - hsar) + rows_;
			              auto const max_pi = polarIndex(polar_angle + hsar) + rows_;

			              for (auto ai = min_ai; max_ai >= ai; ++ai) {
				              auto const first_ai = (ai % columns_) * rows_;
				              for (auto pi = min_pi; max_pi >= pi; ++pi) {
					              auto index = first_ai + (pi % rows_);
					              assert(data_.size() > index);
					              Data&            data = data_[index];
					              std::scoped_lock lock(data.lock);
					              if (0u == data.count || data.distance > radial_distance) {
						              data.distance       = radial_distance;
						              data.distance_error = sensor_error_f_(radial_distance);
					              }
					              ++data.count;
				              }
			              }
		              });

		for (std::uint_fast32_t ai{}; columns_ > ai; ++ai) {
			auto       ai_2       = ai / ds_;
			auto const first_ai   = ai * rows_;
			auto const first_ai_2 = ai_2 * (rows_ / ds_);
			for (std::uint_fast32_t pi{}; rows_ > pi; ++pi) {
				auto const pi_2    = pi / ds_;
				auto const index   = first_ai + pi;
				auto const index_2 = first_ai_2 + pi_2;
				assert(data_.size() > index);
				assert(data_2_.size() > index_2);
				auto& data   = data_[index];
				auto& data_2 = data_2_[index_2];

				if (data.distance < data_2.min_distance) {
					data_2.min_distance       = data.distance;
					data_2.min_distance_error = data.distance_error;
				}
				if (data.distance > data_2.max_distance) {
					data_2.max_distance       = data.distance;
					data_2.max_distance_error = data.distance_error;
				}
				data_2.count += data.count;
			}
		}
	}

	void resetData() const
	{
		ufo::transform(data_.begin(), data_.end(), data_.begin(),
		               [](auto const&) { return Data{}; });
		ufo::transform(data_2_.begin(), data_2_.end(), data_2_.begin(),
		               [](auto const&) { return Data2{}; });
	}

	template <
	    class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void resetData(ExecutionPolicy&& policy) const
	{
		ufo::transform(policy, data_.begin(), data_.end(), data_.begin(),
		               [](auto const&) { return Data{}; });
		ufo::transform(std::forward<ExecutionPolicy>(policy), data_2_.begin(), data_2_.end(),
		               data_2_.begin(), [](auto const&) { return Data2{}; });
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Misses                                        |
	|                                                                                     |
	**************************************************************************************/

	template <class Map, class T>
	std::vector<TreeKey<Dim>> missesKeys(Map const& map, SoAView<Vec<Dim, T>> points,
	                                     Vec<Dim, T> const& origin) const
	{
		auto const bb = bounds(points, origin);

		// TODO: Make parameter
		auto const depth = this->miss_depth + 5;

		auto const min = map.key(TreeCoord<Dim>(ufo::min(bb), depth));
		auto const max = map.key(TreeCoord<Dim>(ufo::max(bb), depth));

		using key_t     = typename TreeKey<Dim>::Key;
		auto const diff = (static_cast<key_t>(max) - static_cast<key_t>(min)) + 1u;

		std::vector<TreeKey<Dim>> keys;
		keys.reserve(std::accumulate(begin(diff), end(diff), 0u, std::multiplies<>()));

		if constexpr (2 == Dim) {
			for (auto k = min; max.x >= k.x; ++k.x) {
				for (k.y = min.y; max.y >= k.y; ++k.y) {
					keys.push_back(k);
				}
			}
		} else if constexpr (3 == Dim) {
			for (auto k = min; max.x >= k.x; ++k.x) {
				for (k.y = min.y; max.y >= k.y; ++k.y) {
					for (k.z = min.z; max.z >= k.z; ++k.z) {
						keys.push_back(k);
					}
				}
			}
		} else {
			// TODO: Error
		}

		return keys;
	}

	template <class Map, class T>
	void fillMisses(Map const& map, Vec<Dim, T> const& origin) const
	{
		// TODO: Implement
	}

	template <
	    class ExecutionPolicy, class Map, class T,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void fillMisses(ExecutionPolicy&& policy, Map const& map, SoAView<Vec<Dim, T>> points,
	                Vec<Dim, T> const& origin) const
	{
		auto const keys = missesKeys(map, points, origin);

		std::mutex  mutex;
		std::size_t size{};

		ufo::for_each(policy, keys.begin(), keys.end(),
		              [this, &mutex, &size, &map, &origin](auto const& key) {
			              thread_local std::size_t idx;
			              auto const               id = std::this_thread::get_id();
			              if (size <= idx || id != thread_misses_[idx].id) {
				              std::scoped_lock lock(mutex);
				              idx = size++;
				              if (thread_misses_.size() < size) {
					              thread_misses_.emplace_back();
					              assert(thread_misses_.size() == size);
				              }
				              thread_misses_[idx].id = id;
			              }

			              auto& misses = thread_misses_[idx].misses;

			              auto const code   = map.code(key);
			              auto const center = cast<T>(map.center(key)) - origin;

			              fillMissesRecurs(misses, map, code, center);
		              });

		std::vector<std::size_t> offset(size);
		std::size_t              total_size{};
		for (std::size_t i{}; size > i; ++i) {
			offset[i] = total_size;
			total_size += thread_misses_[i].misses.size();
		}

		std::cout << total_size << '\n';

		cached_misses_.resize(total_size);
		ufo::for_each(policy, std::size_t(0), size, [this, &offset](std::size_t i) {
			auto&       data = thread_misses_[i];
			auto const& m    = data.misses;
			auto const  o    = offset[i];

			std::copy(m.begin(), m.end(), cached_misses_.begin() + o);

			data.clear();
		});
	}

	template <class Map, class T>
	std::pair<bool, bool> fillMissesRecurs(std::vector<detail::Miss<Dim>>& misses,
	                                       Map const& map, TreeCode<Dim> const& code,
	                                       Vec<Dim, T> const& center) const
	{
		auto const ca = abs(center);
		auto const hl = cast<T>(map.halfLength(code));

		auto const return_distance = norm(ca + hl);
		auto const inner_distance  = norm(ca - hl);

		auto const [min_polar, max_polar]         = polarAngles(center, hl);
		auto const [min_azimuthal, max_azimuthal] = azimuthalAngles(center, hl);

		// TODO: Add angular error

		// + columns_ so they are positive
		auto const min_ai = azimuthalIndex(min_azimuthal) + columns_;
		auto const max_ai = azimuthalIndex(max_azimuthal) + columns_;
		// + rows_ so they are positive
		auto const min_pi = polarIndex(min_polar) + rows_;
		auto const max_pi = polarIndex(max_polar) + rows_;

		bool               valid_return      = false;
		bool               valid_parent      = false;
		bool               valid_void_region = false;
		std::uint_fast32_t count{};

		if (this->miss_depth == code.depth()) {
			valid_return      = validReturn(return_distance, min_ai, max_ai, min_pi, max_pi);
			valid_void_region = valid_return && validVoidRegion(return_distance, min_ai, max_ai,
			                                                    min_pi, max_pi);

			if (valid_return) {
				misses.emplace_back(code, center, count, valid_void_region);
			}
		} else {
			valid_parent = validParent(inner_distance, min_ai, max_ai, min_pi, max_pi);
			valid_return =
			    valid_parent && validReturn(return_distance, min_ai, max_ai, min_pi, max_pi);
			valid_void_region = valid_return && validVoidRegion(return_distance, min_ai, max_ai,
			                                                    min_pi, max_pi);

			if (valid_return && valid_void_region) {
				misses.emplace_back(code, center, count, valid_void_region);
			} else if (valid_parent) {
				auto size_before = misses.size();

				auto child_code      = code.firstborn();
				valid_return         = true;
				valid_void_region    = true;
				bool any_void_region = false;
				for (std::uint_fast32_t i{}; map.branchingFactor() > i; ++i) {
					auto [vr, vvr] =
					    fillMissesRecurs(misses, map, child_code.firstbornSibling(i),
					                     map.child(TreeCoord<Dim, T>(center, code.depth()), i));
					valid_return      = valid_return && vr;
					valid_void_region = valid_void_region && vvr;
					any_void_region   = any_void_region || vvr;
				}

				if (valid_return && (valid_void_region || !any_void_region)) {
					misses.resize(size_before);
					misses.emplace_back(code, center, count, valid_void_region);
				}
			}
		}

		return std::pair{valid_return, valid_void_region};
	}

	[[nodiscard]] bool validReturn(float const distance, std::int_fast32_t min_ai,
	                               std::int_fast32_t max_ai, std::int_fast32_t min_pi,
	                               std::int_fast32_t max_pi) const
	{
		// TODO: Optimize

		for (auto ai = min_ai; max_ai >= ai; ++ai) {
			auto const first_ai = (ai % columns_) * rows_;
			for (auto pi = min_pi; max_pi >= pi; ++pi) {
				auto const& data = data_[first_ai + (pi % rows_)];
				auto const  d    = data.distance + data.distance_error;
				if (d <= distance) {
					return false;
				}
			}
		}
		return true;
	}

	[[nodiscard]] bool validVoidRegion(float const distance, std::int_fast32_t min_ai,
	                                   std::int_fast32_t max_ai, std::int_fast32_t min_pi,
	                                   std::int_fast32_t max_pi) const
	{
		// TODO: Optimize

		for (auto ai = min_ai; max_ai >= ai; ++ai) {
			auto const first_ai = (ai % columns_) * rows_;
			for (auto pi = min_pi; max_pi >= pi; ++pi) {
				auto const& data = data_[first_ai + (pi % rows_)];
				auto const  d    = data.distance - data.distance_error;
				if (d <= distance) {
					return false;
				}
			}
		}
		return true;
	}

	[[nodiscard]] bool validParent(float const distance, std::int_fast32_t min_ai,
	                               std::int_fast32_t max_ai, std::int_fast32_t min_pi,
	                               std::int_fast32_t max_pi) const
	{
		// TODO: Optimize

		for (auto ai = min_ai; max_ai >= ai; ++ai) {
			auto const first_ai = (ai % columns_) * rows_;
			for (auto pi = min_pi; max_pi >= pi; ++pi) {
				auto const& data = data_[first_ai + (pi % rows_)];
				auto const  d    = data.distance + data.distance_error;
				if (d > distance) {
					return true;
				}
			}
		}
		return false;

		// min_ai /= ds_;
		// max_ai /= ds_;
		// min_pi /= ds_;
		// max_pi /= ds_;

		// auto const columns = columns_ / ds_;
		// auto const rows    = rows_ / ds_;

		// for (auto ai = min_ai; max_ai >= ai; ++ai) {
		// 	auto const first_ai = (ai % columns) * rows;
		// 	for (auto pi = min_pi; max_pi >= pi; ++pi) {
		// 		auto const  index = first_ai + (pi % rows);
		// 		auto const& data  = data_2_[index];
		// 		auto const  d     = data.max_distance + data.max_distance_error;
		// 		if (d > distance) {
		// 			auto const min_ai_hr = ai * ds_;
		// 			auto const max_ai_hr = (ai + 1u) * ds_;
		// 			auto const min_pi_hr = pi * ds_;
		// 			auto const max_pi_hr = (pi + 1u) * ds_;
		// 			for (auto ai_hr = min_ai_hr; max_ai_hr > ai_hr; ++ai_hr) {
		// 				auto const first_ai_hr = (ai_hr % columns_) * rows_;
		// 				for (auto pi_hr = min_pi_hr; max_pi_hr >= pi_hr; ++pi_hr) {
		// 					auto const  index_hr = first_ai_hr + (pi_hr % rows_);
		// 					auto const& data     = data_[index_hr];
		// 					auto const  d_hr     = data.distance + data.distance_error;

		// 					if (d_hr > distance) {
		// 						return true;
		// 					}
		// 				}
		// 			}
		// 		}
		// 	}
		// }
		// return false;

		// for (std::uint_fast32_t ai{}; columns_ > ai; ++ai) {
		// 	auto       ai_2       = ai / ds_;
		// 	auto const first_ai   = ai * rows_;
		// 	auto const first_ai_2 = ai_2 * (rows_ / ds_);
		// 	for (std::uint_fast32_t pi{}; rows_ > pi; ++pi) {
		// 		auto const pi_2    = pi / ds_;
		// 		auto const index   = first_ai + pi;
		// 		auto const index_2 = first_ai_2 + pi_2;
		// 		assert(data_.size() > index);
		// 		assert(data_2_.size() > index_2);
		// 		auto& data   = data_[index];
		// 		auto& data_2 = data_2_[index_2];

		// 		if (data.distance < data_2.min_distance) {
		// 			data_2.min_distance       = data.distance;
		// 			data_2.min_distance_error = data.distance_error;
		// 		}
		// 		if (data.distance > data_2.max_distance) {
		// 			data_2.max_distance       = data.distance;
		// 			data_2.max_distance_error = data.distance_error;
		// 		}
		// 		data_2.count += data.count;
		// 	}
		// }
	}

	template <class T>
	[[nodiscard]] T polarAngle(T const& z, T radial_distance) const
	{
		return std::acos(z / radial_distance);
	}

	template <class T>
	[[nodiscard]] T polarAngle(Vec<Dim, T> const& point) const
	{
		if constexpr (2u == Dim) {
			return T(0);
		} else if constexpr (3u == Dim) {
			return polarAngle(point.z, norm(point));
		} else {
			// TODO: Error
		}
	}

	template <class T>
	[[nodiscard]] T azimuthalAngle(Vec<2, T> const& point) const
	{
		return std::atan2(point.y, point.x);
	}

	// template <class T>
	// [[nodiscard]] std::pair<T, T> polarAngles(Vec<Dim, T> const& center,
	//                                           Vec<Dim, T> const& half_length) const
	// {
	// 	if constexpr (2 == Dim) {
	// 		return std::pair{T(0), T(0)};
	// 	} else {
	// 		auto const ca    = abs(Vec<2, T>(center));
	// 		auto const min_z = center.z - half_length.z;
	// 		auto const max_z = center.z + half_length.z;

	// 		auto max = T(0) < min_z ? ca + Vec2<T>(half_length) : ca - Vec2<T>(half_length);
	// 		auto min = T(0) > max_z ? ca + Vec2<T>(half_length) : ca - Vec2<T>(half_length);

	// 		return std::pair{polarAngle(Vec<3, T>(min, max_z)),
	// 		                 polarAngle(Vec<3, T>(max, min_z))};
	// 	}
	// }

	// /*!
	//  * @brief Minimum and maximum azimuthal angles between [-pi, 2 * pi] such that first
	//  is
	//  * always greater than second.
	//  *
	//  * @param offset
	//  * @param point
	//  * @param half_length
	//  * @return std::pair<T, T>
	//  */
	// template <class T>
	// [[nodiscard]] std::pair<T, T> azimuthalAngles(Vec<Dim, T> const& center,
	//                                               Vec<Dim, T> const& half_length) const
	// {
	// 	Vec<2, T> min_p;
	// 	Vec<2, T> max_p;

	// 	if (T(0) > center.x) {
	// 		min_p.y = center.y + half_length.y;
	// 		max_p.y = center.y - half_length.y;
	// 	} else {
	// 		min_p.y = center.y - half_length.y;
	// 		max_p.y = center.y + half_length.y;
	// 	}

	// 	if (T(0) > center.y) {
	// 		min_p.x = center.x - half_length.x;
	// 		max_p.x = center.x + half_length.x;
	// 	} else {
	// 		min_p.x = center.x + half_length.x;
	// 		max_p.x = center.x - half_length.x;
	// 	}

	// 	return std::pair{azimuthalAngle(min_p), azimuthalAngle(max_p)};
	// }

	template <class T>
	[[nodiscard]] std::pair<T, T> polarAngles(Vec<Dim, T> const& center,
	                                          Vec<Dim, T> const& half_length) const
	{
		if constexpr (2 == Dim) {
			return std::pair{T(0), T(0)};
		} else {
			auto const ca    = abs(Vec<2, T>(center));
			auto const min_z = center.z - half_length.z;
			auto const max_z = center.z + half_length.z;

			T max;
			T min;
			if (T(0) <= min_z) {
				max = polarAngle(Vec<3, T>(ca + Vec2<T>(half_length), min_z));
			} else if (ca.x >= half_length.x || ca.y >= half_length.y) {
				max = polarAngle(Vec<3, T>(std::max(T(0), ca.x - half_length.x),
				                           std::max(T(0), ca.y - half_length.y), min_z));
			} else {
				max = numbers::pi_v<float>;
			}
			if (T(0) >= max_z) {
				min = polarAngle(Vec<3, T>(ca + Vec2<T>(half_length), max_z));
			} else if (ca.x >= half_length.x || ca.y >= half_length.y) {
				min = polarAngle(Vec<3, T>(std::max(T(0), ca.x - half_length.x),
				                           std::max(T(0), ca.y - half_length.y), max_z));
			} else {
				min = T(0);
			}

			return std::pair{min, max};
		}
	}

	/*!
	 * @brief Minimum and maximum azimuthal angles between [-pi, 2 * pi] such that first is
	 * always greater than second.
	 *
	 * @param offset
	 * @param point
	 * @param half_length
	 * @return std::pair<T, T>
	 */
	template <class T>
	[[nodiscard]] std::pair<T, T> azimuthalAngles(Vec<Dim, T> const& center,
	                                              Vec<Dim, T> const& half_length) const
	{
		auto const ca = abs(Vec<2, T>(center));

		Vec<2, int> const offset{ca.x <= half_length.x ? 0 : (T(0) < center.x ? 1 : -1),
		                         ca.y <= half_length.y ? 0 : (T(0) < center.y ? 1 : -1)};

		if (0 == offset.x && 0 == offset.y) {
			return std::pair(-numbers::pi_v<T>, numbers::pi_v<T>);
		}

		Vec<2, T> min_p;
		Vec<2, T> max_p;
		T         a{};

		if (-1 == offset.x) {
			min_p.y = center.y + half_length.y;
			max_p.y = center.y - half_length.y;
		} else if (1 == offset.x) {
			min_p.y = center.y - half_length.y;
			max_p.y = center.y + half_length.y;
		} else if (-1 == offset.y) {
			min_p.y = center.y + half_length.y;
			max_p.y = center.y + half_length.y;
		} else {
			min_p.y = center.y - half_length.y;
			max_p.y = center.y - half_length.y;
		}

		if (-1 == offset.y) {
			min_p.x = center.x - half_length.x;
			max_p.x = center.x + half_length.x;
		} else if (1 == offset.y) {
			min_p.x = center.x + half_length.x;
			max_p.x = center.x - half_length.x;
		} else if (-1 == offset.x) {
			min_p.x = center.x + half_length.x;
			max_p.x = center.x + half_length.x;
			a       = T(2) * numbers::pi_v<T>;
		} else {
			min_p.x = center.x - half_length.x;
			max_p.x = center.x - half_length.x;
		}

		return std::pair{azimuthalAngle(min_p), azimuthalAngle(max_p) + a};
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Hits                                         |
	|                                                                                     |
	**************************************************************************************/

	template <class T, class... Rest>
	void filterHits(PointCloud<Dim, T, Rest...>& cloud, Vec<Dim, T> const& origin) const
	{
		cloud.erase_if(
		    [&origin, max_sq = max_distance * max_distance](Vec<Dim, T> const& p) mutable {
			    return max_sq < distanceSquared(origin, p);
		    });
	}

	template <
	    class ExecutionPolicy, class T, class... Rest,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void filterHits(ExecutionPolicy&& policy, PointCloud<Dim, T, Rest...>& cloud,
	                Vec<Dim, T> const& origin) const
	{
		filterHits(cloud, origin);
	}

 public:
	struct Data {
		float              distance{};
		float              distance_error{};
		std::uint_fast32_t count{};
		Spinlock           lock{};

		Data() = default;

		Data(Data const& other)
		    : distance(other.distance)
		    , distance_error(other.distance_error)
		    , count(other.count)
		{
		}

		Data& operator=(Data const& rhs)
		{
			distance       = rhs.distance;
			distance_error = rhs.distance_error;
			count          = rhs.count;
			return *this;
		}
	};

	struct Data2 {
		float min_distance = std::numeric_limits<float>::max();
		float min_distance_error{};

		float max_distance = std::numeric_limits<float>::lowest();
		float max_distance_error{};

		std::uint_fast32_t count{};
	};

	struct ThreadMisses {
		std::thread::id                id;
		std::vector<detail::Miss<Dim>> misses;

		void clear()
		{
			id = {};
			misses.clear();
		}
	};

	SensorErrorFun sensor_error_f_;

	float angular_resolution_factor_{};

	std::uint_fast32_t rows_{};
	std::uint_fast32_t columns_{};
	mutable __block std::vector<Data> data_;
	std::uint_fast32_t                ds_ = 10u;
	mutable __block std::vector<Data2> data_2_;  // TODO: Use this

	mutable __block std::vector<float> cached_distances_;

	mutable __block std::vector<detail::Miss<Dim>> cached_misses_;

	mutable __block std::deque<ThreadMisses> thread_misses_;
};
}  // namespace ufo

#endif  // UFO_MAP_INTEGRATOR_ANGULAR_INTEGRATOR_HPP