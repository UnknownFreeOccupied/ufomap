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
	// Occupancy
	bool filter_occupancy;
	uint8_t min_occupancy;
	uint8_t max_occupancy;

	// Color
	bool filter_color;

	// Time step
	bool filter_time_step;
	ufo::map::time_step_t min_time_step;
	ufo::map::time_step_t max_time_step;

	bool unlabeled;
	bool outlier;
	bool car;
	bool bicycle;
	bool bus;
	bool motorcycle;
	bool on_rails;
	bool truck;
	bool other_vehicle;
	bool person;
	bool bicyclist;
	bool motorcyclist;
	bool road;
	bool parking;
	bool sidewalk;
	bool other_ground;
	bool building;
	bool fence;
	bool other_structure;
	bool lane_marking;
	bool vegetation;
	bool trunk;
	bool terrain;
	bool pole;
	bool traffic_sign;
	bool other_object;
	bool moving_car;
	bool moving_bicyclist;
	bool moving_person;
	bool moving_motorcyclist;
	bool moving_on_rails;
	bool moving_bus;
	bool moving_truck;
	bool moving_other_vehicle;

	// bool noise;
	// bool animal;
	// bool human_pedestrian_adult;
	// bool human_pedestrian_child;
	// bool human_pedestrian_construction_worker;
	// bool human_pedestrian_personal_mobility;
	// bool human_pedestrian_police_officer;
	// bool human_pedestrian_stroller;
	// bool human_pedestrian_wheelchair;
	// bool movable_object_barrier;
	// bool movable_object_debris;
	// bool movable_object_pushable_pullable;
	// bool movable_object_trafficcone;
	// bool static_object_bicycle_rack;
	// bool vehicle_bicycle;
	// bool vehicle_bus_bendy;
	// bool vehicle_bus_rigid;
	// bool vehicle_car;
	// bool vehicle_construction;
	// bool vehicle_emergencyambulance;
	// bool vehicle_emergencypolice;
	// bool vehicle_motorcycle;
	// bool vehicle_trailer;
	// bool vehicle_truck;
	// bool flat_driveable_surface;
	// bool flat_other;
	// bool flat_sidewalk;
	// bool flat_terrain;
	// bool static_manmade;
	// bool static_other;
	// bool static_vegetation;
	// bool vehicle_ego;

	// Semantics
	bool filter_semantics;
	// TODO: Add labels
	ufo::map::semantic_value_t min_semantic_value;
	ufo::map::semantic_value_t max_semantic_value;

	// Bounding volume
	bool filter_bounding_volume;
	ufo::geometry::AABB bounding_volume;

	bool operator==(Filter const& rhs) const
	{
		bool occ = rhs.filter_occupancy == filter_occupancy &&
		           (!filter_occupancy || (rhs.min_occupancy == min_occupancy &&
		                                  rhs.max_occupancy == max_occupancy));

		bool ts = rhs.filter_time_step == filter_time_step &&
		          (!filter_time_step || (rhs.min_time_step == min_time_step &&
		                                 rhs.max_time_step == max_time_step));

		bool sem = rhs.filter_semantics == filter_semantics &&
		           (!filter_semantics || (rhs.min_semantic_value == min_semantic_value &&
		                                  rhs.max_semantic_value == max_semantic_value));

		bool bv = rhs.filter_bounding_volume == filter_bounding_volume &&
		          (!filter_bounding_volume || (rhs.bounding_volume == bounding_volume));

		return occ && ts && sem && bv;
	}

	bool operator!=(Filter const& rhs) const { return !(*this == rhs); }
};
}  // namespace ufomap_ros::rviz_plugins

#endif  // UFOMAP_RVIZ_PLUGINS_FILTER_H