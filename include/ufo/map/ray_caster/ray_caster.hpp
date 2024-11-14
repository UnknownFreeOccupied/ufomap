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

#ifndef UFO_MAP_RAY_CASTER_HPP
#define UFO_MAP_RAY_CASTER_HPP

// UFO
#include <ufo/map/integration/integration_grid.hpp>
#include <ufo/map/octree/octree_code.hpp>
#include <ufo/map/octree/octree_key.hpp>
#include <ufo/map/quadtree/quadtree_code.hpp>
#include <ufo/map/quadtree/quadtree_key.hpp>
#include <ufo/map/types.hpp>
#include <ufo/math/util.hpp>
#include <ufo/math/vec2.hpp>
#include <ufo/math/vec3.hpp>

// STL
#include <cassert>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace ufo
{
inline void computeRay(std::unordered_map<OctCode, IntegrationGrid<OctCode>>& misses,
                       Vec3f origin, Vec3f goal, OctCode c_origin, Vec3f voxel_border,
                       float grid_size, float max_distance, unsigned inflate_unknown)
{
	static constexpr auto max   = std::numeric_limits<float>::max();
	depth_t               depth = c_origin.depth();

	auto dir      = goal - origin;
	auto distance = dir.norm();
	dir /= distance;

	distance = std::min(distance, max_distance);

	std::array<int, 3> sign{sgn(dir.x), sgn(dir.y), sgn(dir.z)};

	if (0 == sign[0] && 0 == sign[1] && 0 == sign[2]) {
		return;
	}

	std::array<void (OctCode ::*)(code_t), 3> step_f{
	    0 <= sign[0] ? (void(OctCode ::*)(code_t))(&OctCode::incX)
	                 : (void(OctCode ::*)(code_t))(&OctCode::decX),
	    0 <= sign[1] ? (void(OctCode ::*)(code_t))(&OctCode::incY)
	                 : (void(OctCode ::*)(code_t))(&OctCode::decY),
	    0 <= sign[2] ? (void(OctCode ::*)(code_t))(&OctCode::incZ)
	                 : (void(OctCode ::*)(code_t))(&OctCode::decZ)};
	std::array<code_t, 3> step_v{c_origin.xStep(), c_origin.yStep(), c_origin.zStep()};

	Vec3f t_max(sign[0] ? (voxel_border.x + sign[0] * grid_size / 2.0f) / dir.x : max,
	            sign[1] ? (voxel_border.y + sign[1] * grid_size / 2.0f) / dir.y : max,
	            sign[2] ? (voxel_border.z + sign[2] * grid_size / 2.0f) / dir.z : max);

	Vec3f t_delta(sign[0] ? grid_size / std::abs(dir.x) : max,
	              sign[1] ? grid_size / std::abs(dir.y) : max,
	              sign[2] ? grid_size / std::abs(dir.z) : max);

	// FIXME: Is this correct? Should it be zero if all zero?
	unsigned steps = (sign[0] ? std::ceil((distance - t_max[0]) / t_delta[0]) : 0) +
	                 (sign[1] ? std::ceil((distance - t_max[1]) / t_delta[1]) : 0) +
	                 (sign[2] ? std::ceil((distance - t_max[2]) / t_delta[2]) : 0);

	OctCode prev_at_depth = c_origin.toDepth(IntegrationGrid<OctCode>::depth() + depth);
	IntegrationGrid<OctCode>* misses_grid = &misses[prev_at_depth];
	misses_grid->set(c_origin);

	while (steps--) {
		auto const advance_dim = t_max.minElementIndex();
		(c_origin.*step_f[advance_dim])(step_v[advance_dim]);
		t_max[advance_dim] += t_delta[advance_dim];

		if (!OctCode::equalAtDepth(prev_at_depth, c_origin,
		                           IntegrationGrid<OctCode>::depth() + depth)) {
			prev_at_depth = c_origin.toDepth(IntegrationGrid<OctCode>::depth() + depth);
			misses_grid   = &misses[prev_at_depth];
		}
		misses_grid->set(c_origin);
	}

	// TODO: Can optimize this by calculating how many extra steps are needed above

	unsigned                advance_dim{};
	std::array<unsigned, 3> s{};
	while (s[advance_dim] != inflate_unknown) {
		advance_dim = t_max.minElementIndex();
		(c_origin.*step_f[advance_dim])(step_v[advance_dim]);
		t_max[advance_dim] += t_delta[advance_dim];
		++s[advance_dim];

		if (!OctCode::equalAtDepth(prev_at_depth, c_origin,
		                           IntegrationGrid<OctCode>::depth() + depth)) {
			prev_at_depth = c_origin.toDepth(IntegrationGrid<OctCode>::depth() + depth);
			misses_grid   = &misses[prev_at_depth];
		}
		misses_grid->set(c_origin);
	}
}

inline void computeRay(std::unordered_map<OctCode, IntegrationGrid<OctCode>>& misses,
                       std::unordered_map<OctCode, IntegrationGrid<OctCode>> const& hits,
                       Vec3f origin, Vec3f goal, OctCode c_origin, Vec3f voxel_border,
                       float grid_size, float max_distance, unsigned inflate_unknown,
                       bool ray_passthrough_hits)
{
	static constexpr auto max   = std::numeric_limits<float>::max();
	depth_t               depth = c_origin.depth();

	auto dir      = goal - origin;
	auto distance = dir.norm();
	dir /= distance;

	distance = std::min(distance, max_distance);

	std::array<int, 3> sign{sgn(dir.x), sgn(dir.y), sgn(dir.z)};

	if (0 == sign[0] && 0 == sign[1] && 0 == sign[2]) {
		return;
	}

	std::array<void (OctCode ::*)(code_t), 3> step_f{
	    0 <= sign[0] ? (void(OctCode ::*)(code_t))(&OctCode::incX)
	                 : (void(OctCode ::*)(code_t))(&OctCode::decX),
	    0 <= sign[1] ? (void(OctCode ::*)(code_t))(&OctCode::incY)
	                 : (void(OctCode ::*)(code_t))(&OctCode::decY),
	    0 <= sign[2] ? (void(OctCode ::*)(code_t))(&OctCode::incZ)
	                 : (void(OctCode ::*)(code_t))(&OctCode::decZ)};
	std::array<code_t, 3> step_v{c_origin.xStep(), c_origin.yStep(), c_origin.zStep()};

	Vec3f t_max(sign[0] ? (voxel_border.x + sign[0] * grid_size / 2.0f) / dir.x : max,
	            sign[1] ? (voxel_border.y + sign[1] * grid_size / 2.0f) / dir.y : max,
	            sign[2] ? (voxel_border.z + sign[2] * grid_size / 2.0f) / dir.z : max);

	Vec3f t_delta(sign[0] ? grid_size / std::abs(dir.x) : max,
	              sign[1] ? grid_size / std::abs(dir.y) : max,
	              sign[2] ? grid_size / std::abs(dir.z) : max);

	// FIXME: Is this correct? Should it be zero if all zero?
	unsigned steps = (sign[0] ? std::ceil((distance - t_max[0]) / t_delta[0]) : 0) +
	                 (sign[1] ? std::ceil((distance - t_max[1]) / t_delta[1]) : 0) +
	                 (sign[2] ? std::ceil((distance - t_max[2]) / t_delta[2]) : 0);

	OctCode prev_at_depth = c_origin.toDepth(IntegrationGrid<OctCode>::depth() + depth);
	IntegrationGrid<OctCode>* misses_grid = &misses[prev_at_depth];
	misses_grid->set(c_origin);

	if (ray_passthrough_hits) {
		while (steps--) {
			auto const advance_dim = t_max.minElementIndex();
			(c_origin.*step_f[advance_dim])(step_v[advance_dim]);
			t_max[advance_dim] += t_delta[advance_dim];

			if (!OctCode::equalAtDepth(prev_at_depth, c_origin,
			                           IntegrationGrid<OctCode>::depth() + depth)) {
				prev_at_depth = c_origin.toDepth(IntegrationGrid<OctCode>::depth() + depth);
				misses_grid   = &misses[prev_at_depth];
			}
			misses_grid->set(c_origin);
		}
	} else {
		auto       hit_grid     = hits.find(prev_at_depth);
		auto const hit_grid_end = std::cend(hits);

		while (steps-- && (hit_grid_end == hit_grid || !hit_grid->second.test(c_origin))) {
			auto const advance_dim = t_max.minElementIndex();
			(c_origin.*step_f[advance_dim])(step_v[advance_dim]);
			t_max[advance_dim] += t_delta[advance_dim];

			if (!OctCode::equalAtDepth(prev_at_depth, c_origin,
			                           IntegrationGrid<OctCode>::depth() + depth)) {
				prev_at_depth = c_origin.toDepth(IntegrationGrid<OctCode>::depth() + depth);
				misses_grid   = &misses[prev_at_depth];
				hit_grid      = hits.find(prev_at_depth);
			}
			misses_grid->set(c_origin);
		}
	}

	// TODO: Can optimize this by calculating how many extra steps are needed above

	unsigned                advance_dim{};
	std::array<unsigned, 3> s{};
	while (s[advance_dim] != inflate_unknown) {
		advance_dim = t_max.minElementIndex();
		(c_origin.*step_f[advance_dim])(step_v[advance_dim]);
		t_max[advance_dim] += t_delta[advance_dim];
		++s[advance_dim];

		if (!OctCode::equalAtDepth(prev_at_depth, c_origin,
		                           IntegrationGrid<OctCode>::depth() + depth)) {
			prev_at_depth = c_origin.toDepth(IntegrationGrid<OctCode>::depth() + depth);
			misses_grid   = &misses[prev_at_depth];
		}
		misses_grid->set(c_origin);
	}
}

template <class Map>
void computeRaySimple(Map const& map, std::unordered_map<OctCode, Grid>& grids,
                      Vec3f origin, Vec3f goal, depth_t depth, float step_size,
                      float max_distance        = std::numeric_limits<float>::max(),
                      float early_stop_distance = 0.0f)
{
	Vec3f dir      = goal - origin;
	float distance = dir.norm();
	dir /= distance;

	distance = std::min(distance - early_stop_distance, max_distance);

	std::size_t num_steps = static_cast<std::size_t>(distance / step_size);
	Vec3f       step      = dir * step_size;

	OctCode prev_at_depth = map.toCode(origin, Grid::depth() + depth);
	Grid*   grid          = &grids[prev_at_depth];
	grid->set(map.toCode(origin, depth));
	for (std::size_t i{}; i != num_steps; ++i, origin += step) {
		OctCode cur = map.toCode(origin, depth);

		if (OctCode::equalAtDepth(prev_at_depth, cur, Grid::depth() + depth)) {
			grid          = &grids[cur.toDepth(Grid::depth() + depth)];
			prev_at_depth = cur.toDepth(Grid::depth() + depth);
		}
		grid->set(cur);
	}
}

[[nodiscard]] inline std::vector<OctCode> computeRay(
    OctKey origin, OctKey goal, float max_distance = std::numeric_limits<float>::max(),
    std::size_t early_stop = 0, float early_stop_distance = 0.0f)
{
	assert(origin.depth() == goal.depth());

	int const size = static_cast<int>(origin.step());

	Vec3f o(static_cast<float>(origin.x), static_cast<float>(origin.y),
	        static_cast<float>(origin.z));
	Vec3f g(static_cast<float>(goal.x), static_cast<float>(goal.y),
	        static_cast<float>(goal.z));

	Vec3f dir      = g - o;
	float distance = dir.norm();
	dir /= distance;

	distance = std::min(distance, max_distance);

	Vec3i step(sgn(dir.x) * size, sgn(dir.y) * size, sgn(dir.z) * size);

	dir.abs();

	constexpr auto max    = std::numeric_limits<float>::max();
	float          f_size = static_cast<float>(size);

	Vec3f t_delta(step.x ? f_size / dir.x : max, step.y ? f_size / dir.y : max,
	              step.z ? f_size / dir.z : max);

	Vec3f t_max = t_delta / 2.0f;

	// ray.reserve(static_cast<KeyRay::size_type>(1.5 * Vec3f::abs(g - o).norm() /
	// f_size));
	std::vector<OctCode> ray;
	ray.emplace_back(origin);
	distance -= early_stop_distance;
	while (origin != goal && t_max.min() <= distance) {
		auto advance_dim = t_max.minElementIndex();
		// TODO: How to fix this case?
		origin[advance_dim] += static_cast<key_t>(step[advance_dim]);
		t_max[advance_dim] += t_delta[advance_dim];
		ray.emplace_back(origin);
	}
	for (; 0 != early_stop && !ray.empty(); --early_stop) {
		ray.pop_back();
	}
	return ray;
}

[[nodiscard]] inline std::vector<OctCode> computeRaySimple(
    OctKey origin, OctKey goal, float step_size_factor = 1.0f,
    float max_distance = std::numeric_limits<float>::max(), std::size_t early_stop = 0,
    float early_stop_distance = 0.0f)
{
	auto const depth = origin.depth;

	assert(goal.depth() == depth);

	Vec3f current(static_cast<float>(origin.x), static_cast<float>(origin.y),
	              static_cast<float>(origin.z));
	Vec3f last(static_cast<float>(goal.x), static_cast<float>(goal.y),
	           static_cast<float>(goal.z));

	Vec3f dir      = last - current;
	float distance = dir.norm();
	dir /= distance;

	distance = std::min(distance, max_distance);

	auto step_size = static_cast<float>(1U << depth) * step_size_factor;

	std::size_t num_steps = static_cast<std::size_t>(
	    (distance - std::min(distance, early_stop_distance)) / step_size);
	Vec3f step = dir * step_size;

	num_steps = num_steps - std::min(num_steps, early_stop);

	if (0 == num_steps) {
		return {};
	}

	// FIXME: Should it take one more step?
	std::vector<OctCode> ray;
	ray.reserve(num_steps);
	for (std::size_t i{}; i != num_steps; ++i, current += step) {
		ray.emplace_back(OctKey(static_cast<key_t>(current.x), static_cast<key_t>(current.y),
		                        static_cast<key_t>(current.z), depth));
	}
	return ray;
}

[[nodiscard]] inline std::vector<QuadCode> computeRaySimple(
    QuadKey origin, QuadKey goal, float step_size_factor = 1.0f,
    float max_distance = std::numeric_limits<float>::max(), std::size_t early_step = 0,
    float early_stop_distance = 0.0f)
{
	// TODO: Implement
}

[[nodiscard]] inline std::vector<Vec3f> computeRaySimple(
    Vec3f origin, Vec3f goal, float step_size,
    float max_distance = std::numeric_limits<float>::max(), std::size_t early_stop = 0,
    float early_stop_distance = 0.0f)
{
	Vec3f dir      = goal - origin;
	float distance = dir.norm();
	dir /= distance;

	distance = std::min(distance, max_distance);

	distance -= early_stop_distance;

	std::size_t num_steps = static_cast<std::size_t>(distance / step_size);
	num_steps -= std::min(num_steps, early_stop);
	Vec3f step = dir * step_size;

	// FIXME: Should it take one more step?
	std::vector<Vec3f> ray;
	ray.reserve(num_steps);
	for (std::size_t i{}; i != num_steps; ++i, origin += step) {
		ray.push_back(origin);
	}
	return ray;
}

[[nodiscard]] inline std::vector<Vec2f> computeRaySimple(
    Vec2f origin, Vec2f goal, float step_size,
    float max_distance = std::numeric_limits<float>::max(), std::size_t early_stop = 0,
    float early_stop_distance = 0.0f)
{
	Vec2f dir      = goal - origin;
	float distance = dir.norm();
	dir /= distance;

	distance = std::min(distance, max_distance);

	distance -= early_stop_distance;

	std::size_t num_steps = static_cast<std::size_t>(distance / step_size);
	num_steps -= std::min(num_steps, early_stop);
	Vec2f step = dir * step_size;

	// FIXME: Should it take one more step?
	std::vector<Vec2f> ray;
	ray.reserve(num_steps);
	for (std::size_t i{}; i != num_steps; ++i, origin += step) {
		ray.push_back(origin);
	}
	return ray;
}
}  // namespace ufo

#endif  // UFO_MAP_RAY_CASTER_HPP