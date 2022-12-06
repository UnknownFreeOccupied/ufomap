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

#ifndef UFOMAP_RVIZ_PLUGINS_FILTER_H
#define UFOMAP_RVIZ_PLUGINS_FILTER_H

// UFO
#include <ufo/geometry/aabb.h>
#include <ufo/map/semantic/semantic.h>
#include <ufo/map/types.h>

// STD
#include <cstdint>

namespace ufomap_ros::rviz_plugins
{
struct Filter {
	// Depth
	ufo::map::depth_t min_depth = 0;

	// Occupancy
	bool occupancy;
	bool unknown;
	bool free;
	bool occupied;
	ufo::map::logit_t min_occupancy;
	ufo::map::logit_t max_occupancy;

	// Color
	bool color;

	// Time step
	bool time;
	ufo::map::time_t min_time;
	ufo::map::time_t max_time;

	// Intensity
	bool intensity;
	ufo::map::intensity_t min_intensity;
	ufo::map::intensity_t max_intensity;

	// Count
	bool count;
	ufo::map::count_t min_count;
	ufo::map::count_t max_count;

	// Reflection
	bool reflection;
	ufo::map::reflection_t min_reflection;
	ufo::map::reflection_t max_reflection;
	ufo::map::count_t min_hits;
	ufo::map::count_t max_hits;
	ufo::map::count_t min_misses;
	ufo::map::count_t max_misses;

	// Surfel
	bool surfel;
	ufo::map::count_t min_surfel_points;
	ufo::map::count_t max_surfel_points;
	double min_planarity;
	double max_planarity;

	// Semantics
	bool semantics;
	ufo::map::value_t min_semantic_value;
	ufo::map::value_t max_semantic_value;

	// Bounding volume
	bool bounding_volume;
	ufo::geometry::AABB bv;

	bool operator==(Filter const& rhs) const
	{
		return true;
		// TODO: Implement
		// bool occ = rhs.filter_occupancy == filter_occupancy &&
		//            (!filter_occupancy || (rhs.min_occupancy == min_occupancy &&
		//                                   rhs.max_occupancy == max_occupancy));

		// bool ts = rhs.filter_time == filter_time &&
		//           (!filter_time || (rhs.min_time == min_time && rhs.max_time == max_time));

		// bool sem = rhs.filter_semantics == filter_semantics &&
		//            (!filter_semantics || (rhs.min_semantic_value == min_semantic_value &&
		//                                   rhs.max_semantic_value == max_semantic_value));

		// bool bv = rhs.filter_bounding_volume == filter_bounding_volume &&
		//           (!filter_bounding_volume || (rhs.bounding_volume == bounding_volume));

		// return occ && ts && sem && bv;
	}

	bool operator!=(Filter const& rhs) const { return !(*this == rhs); }
};
}  // namespace ufomap_ros::rviz_plugins

#endif  // UFOMAP_RVIZ_PLUGINS_FILTER_H