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

// UFO
#include <ufomap_rviz_plugins/data.h>

namespace ufomap_ros::rviz_plugins
{
Data::Data()
    : min_position_(std::numeric_limits<Ogre::Real>::max(),
                    std::numeric_limits<Ogre::Real>::max(),
                    std::numeric_limits<Ogre::Real>::max()),
      max_position_(std::numeric_limits<Ogre::Real>::lowest(),
                    std::numeric_limits<Ogre::Real>::lowest(),
                    std::numeric_limits<Ogre::Real>::lowest()),
      min_time_step_(std::numeric_limits<ufo::map::time_step_t>::max()),
      max_time_step_(std::numeric_limits<ufo::map::time_step_t>::lowest())
{
}

void Data::swap(Data& other)
{
	occupancy_.swap(other.occupancy_);
	time_step_.swap(other.time_step_);
	color_.swap(other.color_);
	semantics_.swap(other.semantics_);
}

std::vector<Voxels::Voxel> Data::generateVoxels(RenderMode const& render,
                                                Filter const& filter, double size,
                                                Heatmap const& heatmap) const
{
	std::vector<Voxels::Voxel> voxels;
	voxels.reserve(position_.size());

	for (size_t i = 0; i < position_.size(); ++i) {
		if (includeVoxel(filter, size, i)) {
			voxels.emplace_back(position_[i], getColor(render, heatmap, i));
		}
	}

	return voxels;
}

void Data::clear()
{
	occupancy_.clear();
	time_step_.clear();
	color_.clear();
	semantics_.clear();
}

bool Data::includeVoxel(Filter const& filter, double size, size_t index) const
{
	if (filter.filter_occupancy && (occupancy_.size() == position_.size()) &&
	    (filter.min_occupancy > occupancy_[index] ||
	     occupancy_[index] > filter.max_occupancy)) {
		return false;
	}

	if (filter.filter_color && Ogre::ColourValue(0, 0, 0) == color_[index]) {
		return false;
	}

	if (filter.filter_time_step && time_step_.size() == position_.size()) {
		switch (time_step_[index]) {
			case 0:
				if (!filter.unlabeled) {
					return false;
				}
				break;
			case 1:
				if (!filter.outlier) {
					return false;
				}
				break;
			case 10:
				if (!filter.car) {
					return false;
				}
				break;
			case 11:
				if (!filter.bicycle) {
					return false;
				}
				break;
			case 13:
				if (!filter.bus) {
					return false;
				}
				break;
			case 15:
				if (!filter.motorcycle) {
					return false;
				}
				break;
			case 16:
				if (!filter.on_rails) {
					return false;
				}
				break;
			case 18:
				if (!filter.truck) {
					return false;
				}
				break;
			case 20:
				if (!filter.other_vehicle) {
					return false;
				}
				break;
			case 30:
				if (!filter.person) {
					return false;
				}
				break;
			case 31:
				if (!filter.bicyclist) {
					return false;
				}
				break;
			case 32:
				if (!filter.motorcyclist) {
					return false;
				}
				break;
			case 40:
				if (!filter.road) {
					return false;
				}
				break;
			case 44:
				if (!filter.parking) {
					return false;
				}
				break;
			case 48:
				if (!filter.sidewalk) {
					return false;
				}
				break;
			case 49:
				if (!filter.other_ground) {
					return false;
				}
				break;
			case 50:
				if (!filter.building) {
					return false;
				}
				break;
			case 51:
				if (!filter.fence) {
					return false;
				}
				break;
			case 52:
				if (!filter.other_structure) {
					return false;
				}
				break;
			case 60:
				if (!filter.lane_marking) {
					return false;
				}
				break;
			case 70:
				if (!filter.vegetation) {
					return false;
				}
				break;
			case 71:
				if (!filter.trunk) {
					return false;
				}
				break;
			case 72:
				if (!filter.terrain) {
					return false;
				}
				break;
			case 80:
				if (!filter.pole) {
					return false;
				}
				break;
			case 81:
				if (!filter.traffic_sign) {
					return false;
				}
				break;
			case 99:
				if (!filter.other_object) {
					return false;
				}
				break;
			case 252:
				if (!filter.moving_car) {
					return false;
				}
				break;
			case 253:
				if (!filter.moving_bicyclist) {
					return false;
				}
				break;
			case 254:
				if (!filter.moving_person) {
					return false;
				}
				break;
			case 255:
				if (!filter.moving_motorcyclist) {
					return false;
				}
				break;
			case 256:
				if (!filter.moving_on_rails) {
					return false;
				}
				break;
			case 257:
				if (!filter.moving_bus) {
					return false;
				}
				break;
			case 258:
				if (!filter.moving_truck) {
					return false;
				}
				break;
			case 259:
				if (!filter.moving_other_vehicle) {
					return false;
				}
				break;

				// case 0:
				// 	if (!filter.noise) {
				// 		return false;
				// 	}
				// 	break;
				// case 1:
				// 	if (!filter.animal) {
				// 		return false;
				// 	}
				// 	break;
				// case 2:
				// 	if (!filter.human_pedestrian_adult) {
				// 		return false;
				// 	}
				// 	break;
				// case 3:
				// 	if (!filter.human_pedestrian_child) {
				// 		return false;
				// 	}
				// 	break;
				// case 4:
				// 	if (!filter.human_pedestrian_construction_worker) {
				// 		return false;
				// 	}
				// 	break;
				// case 5:
				// 	if (!filter.human_pedestrian_personal_mobility) {
				// 		return false;
				// 	}
				// 	break;
				// case 6:
				// 	if (!filter.human_pedestrian_police_officer) {
				// 		return false;
				// 	}
				// 	break;
				// case 7:
				// 	if (!filter.human_pedestrian_stroller) {
				// 		return false;
				// 	}
				// 	break;
				// case 8:
				// 	if (!filter.human_pedestrian_wheelchair) {
				// 		return false;
				// 	}
				// 	break;
				// case 9:
				// 	if (!filter.movable_object_barrier) {
				// 		return false;
				// 	}
				// 	break;
				// case 10:
				// 	if (!filter.movable_object_debris) {
				// 		return false;
				// 	}
				// 	break;
				// case 11:
				// 	if (!filter.movable_object_pushable_pullable) {
				// 		return false;
				// 	}
				// 	break;
				// case 12:
				// 	if (!filter.movable_object_trafficcone) {
				// 		return false;
				// 	}
				// 	break;
				// case 13:
				// 	if (!filter.static_object_bicycle_rack) {
				// 		return false;
				// 	}
				// 	break;
				// case 14:
				// 	if (!filter.vehicle_bicycle) {
				// 		return false;
				// 	}
				// 	break;
				// case 15:
				// 	if (!filter.vehicle_bus_bendy) {
				// 		return false;
				// 	}
				// 	break;
				// case 16:
				// 	if (!filter.vehicle_bus_rigid) {
				// 		return false;
				// 	}
				// 	break;
				// case 17:
				// 	if (!filter.vehicle_car) {
				// 		return false;
				// 	}
				// 	break;
				// case 18:
				// 	if (!filter.vehicle_construction) {
				// 		return false;
				// 	}
				// 	break;
				// case 19:
				// 	if (!filter.vehicle_emergencyambulance) {
				// 		return false;
				// 	}
				// 	break;
				// case 20:
				// 	if (!filter.vehicle_emergencypolice) {
				// 		return false;
				// 	}
				// 	break;
				// case 21:
				// 	if (!filter.vehicle_motorcycle) {
				// 		return false;
				// 	}
				// 	break;
				// case 22:
				// 	if (!filter.vehicle_trailer) {
				// 		return false;
				// 	}
				// 	break;
				// case 23:
				// 	if (!filter.vehicle_truck) {
				// 		return false;
				// 	}
				// 	break;
				// case 24:
				// 	if (!filter.flat_driveable_surface) {
				// 		return false;
				// 	}
				// 	break;
				// case 25:
				// 	if (!filter.flat_other) {
				// 		return false;
				// 	}
				// 	break;
				// case 26:
				// 	if (!filter.flat_sidewalk) {
				// 		return false;
				// 	}
				// 	break;
				// case 27:
				// 	if (!filter.flat_terrain) {
				// 		return false;
				// 	}
				// 	break;
				// case 28:
				// 	if (!filter.static_manmade) {
				// 		return false;
				// 	}
				// 	break;
				// case 29:
				// 	if (!filter.static_other) {
				// 		return false;
				// 	}
				// 	break;
				// case 30:
				// 	if (!filter.static_vegetation) {
				// 		return false;
				// 	}
				// 	break;
				// case 31:
				// 	if (!filter.vehicle_ego) {
				// 		return false;
				// 	}
				// 	break;
		}
	}

	if (filter.filter_bounding_volume) {
		ufo::geometry::AAEBB aaebb(position_[index][0], position_[index][1],
		                           position_[index][2], size / 2.0);

		if (!ufo::geometry::intersects(aaebb, filter.bounding_volume)) {
			return false;
		}
	}

	return true;
}

Ogre::ColourValue Data::getColor(RenderMode const& render, Heatmap const& heatmap,
                                 size_t index) const
{
	// FIXME: Use alpha?

	switch (render.coloring_mode) {
		case ColoringMode::FIXED_COLOR:
			return render.color;
		case ColoringMode::X_AXIS_COLOR:
			return render.normalized_value
			           ? Heatmap::getColor(position_[index].x, heatmap.min_position.x,
			                               heatmap.max_position.x, render.color_factor)
			           : Heatmap::getColor(position_[index].x, render.min_normalized_value,
			                               render.max_normalized_value, render.color_factor);
		case ColoringMode::Y_AXIS_COLOR:
			return render.normalized_value
			           ? Heatmap::getColor(position_[index].y, heatmap.min_position.y,
			                               heatmap.max_position.y, render.color_factor)
			           : Heatmap::getColor(position_[index].y, render.min_normalized_value,
			                               render.max_normalized_value, render.color_factor);
		case ColoringMode::Z_AXIS_COLOR:
			return render.normalized_value
			           ? Heatmap::getColor(position_[index].z, heatmap.min_position.z,
			                               heatmap.max_position.z, render.color_factor)
			           : Heatmap::getColor(position_[index].z, render.min_normalized_value,
			                               render.max_normalized_value, render.color_factor);
		case ColoringMode::TIME_STEP_COLOR:
			assert(position_.size() == time_step_.size());
			switch (time_step_[index]) {
				case 0:
					return Ogre::ColourValue(0 / 255.0, 0 / 255.0, 0 / 255.0);
				case 1:
				case 100:
					return Ogre::ColourValue(0 / 255.0, 0 / 255.0, 255 / 255.0);
				case 10:
				case 1000:
					return Ogre::ColourValue(245 / 255.0, 150 / 255.0, 100 / 255.0);
				case 11:
				case 1100:
					return Ogre::ColourValue(245 / 255.0, 230 / 255.0, 100 / 255.0);
				case 13:
				case 1300:
					return Ogre::ColourValue(250 / 255.0, 80 / 255.0, 100 / 255.0);
				case 15:
				case 1500:
					return Ogre::ColourValue(150 / 255.0, 60 / 255.0, 30 / 255.0);
				case 16:
				case 1600:
					return Ogre::ColourValue(255 / 255.0, 0 / 255.0, 0 / 255.0);
				case 18:
				case 1800:
					return Ogre::ColourValue(180 / 255.0, 30 / 255.0, 80 / 255.0);
				case 20:
				case 2000:
					return Ogre::ColourValue(255 / 255.0, 0 / 255.0, 0 / 255.0);
				case 30:
				case 3000:
					return Ogre::ColourValue(30 / 255.0, 30 / 255.0, 255 / 255.0);
				case 31:
				case 3100:
					return Ogre::ColourValue(200 / 255.0, 40 / 255.0, 255 / 255.0);
				case 32:
				case 3200:
					return Ogre::ColourValue(90 / 255.0, 30 / 255.0, 150 / 255.0);
				case 40:
				case 4000:
					return Ogre::ColourValue(255 / 255.0, 0 / 255.0, 255 / 255.0);
				case 44:
				case 4400:
					return Ogre::ColourValue(255 / 255.0, 150 / 255.0, 255 / 255.0);
				case 48:
				case 4800:
					return Ogre::ColourValue(75 / 255.0, 0 / 255.0, 75 / 255.0);
				case 49:
				case 4900:
					return Ogre::ColourValue(75 / 255.0, 0 / 255.0, 175 / 255.0);
				case 50:
				case 5000:
					return Ogre::ColourValue(0 / 255.0, 200 / 255.0, 255 / 255.0);
				case 51:
				case 5100:
					return Ogre::ColourValue(50 / 255.0, 120 / 255.0, 255 / 255.0);
				case 52:
				case 5200:
					return Ogre::ColourValue(0 / 255.0, 150 / 255.0, 255 / 255.0);
				case 60:
				case 6000:
					return Ogre::ColourValue(170 / 255.0, 255 / 255.0, 150 / 255.0);
				case 70:
				case 7000:
					return Ogre::ColourValue(0 / 255.0, 175 / 255.0, 0 / 255.0);
				case 71:
				case 7100:
					return Ogre::ColourValue(0 / 255.0, 60 / 255.0, 135 / 255.0);
				case 72:
				case 7200:
					return Ogre::ColourValue(80 / 255.0, 240 / 255.0, 150 / 255.0);
				case 80:
				case 8000:
					return Ogre::ColourValue(150 / 255.0, 240 / 255.0, 255 / 255.0);
				case 81:
				case 8100:
					return Ogre::ColourValue(0 / 255.0, 0 / 255.0, 255 / 255.0);
				case 99:
				case 9900:
					return Ogre::ColourValue(255 / 255.0, 255 / 255.0, 50 / 255.0);
				case 252:
				case 25200:
					return Ogre::ColourValue(245 / 255.0, 150 / 255.0, 100 / 255.0);
				case 256:
				case 25600:
					return Ogre::ColourValue(255 / 255.0, 0 / 255.0, 0 / 255.0);
				case 253:
				case 25300:
					return Ogre::ColourValue(200 / 255.0, 40 / 255.0, 255 / 255.0);
				case 254:
				case 25400:
					return Ogre::ColourValue(30 / 255.0, 30 / 255.0, 255 / 255.0);
				case 255:
				case 25500:
					return Ogre::ColourValue(90 / 255.0, 30 / 255.0, 150 / 255.0);
				case 257:
				case 25700:
					return Ogre::ColourValue(250 / 255.0, 80 / 255.0, 100 / 255.0);
				case 258:
				case 25800:
					return Ogre::ColourValue(180 / 255.0, 30 / 255.0, 80 / 255.0);
				case 259:
				case 25900:
					return Ogre::ColourValue(255 / 255.0, 0 / 255.0, 0 / 255.0);

					// case 0:  return Ogre::ColourValue(0/ 255.0, 0 / 255.0, 0 / 255.0);
					// case 1:  return Ogre::ColourValue(70/ 255.0, 130 / 255.0, 180 / 255.0);
					// case 2:  return Ogre::ColourValue(0/ 255.0, 0 / 255.0, 230 / 255.0);
					// case 3:  return Ogre::ColourValue(135/ 255.0, 206 / 255.0, 235 / 255.0);
					// case 4:  return Ogre::ColourValue(100/ 255.0, 149 / 255.0, 237 / 255.0);
					// case 5:  return Ogre::ColourValue(219/ 255.0, 112 / 255.0, 147 / 255.0);
					// case 6:  return Ogre::ColourValue(0/ 255.0, 0 / 255.0, 128 / 255.0);
					// case 7:  return Ogre::ColourValue(240/ 255.0, 128 / 255.0, 128 / 255.0);
					// case 8:  return Ogre::ColourValue(138/ 255.0, 43 / 255.0, 226 / 255.0);
					// case 9:  return Ogre::ColourValue(112/ 255.0, 128 / 255.0, 144 / 255.0);
					// case 10: return Ogre::ColourValue(210/ 255.0, 105 / 255.0, 30 / 255.0);
					// case 11: return Ogre::ColourValue(105/ 255.0, 105 / 255.0, 105 / 255.0);
					// case 12: return Ogre::ColourValue(47/ 255.0, 79 / 255.0, 79 / 255.0);
					// case 13: return Ogre::ColourValue(188/ 255.0, 143 / 255.0, 143 / 255.0);
					// case 14: return Ogre::ColourValue(220/ 255.0, 20 / 255.0, 60 / 255.0);
					// case 15: return Ogre::ColourValue(255/ 255.0, 127 / 255.0, 80 / 255.0);
					// case 16: return Ogre::ColourValue(255/ 255.0, 69 / 255.0, 0 / 255.0);
					// case 17: return Ogre::ColourValue(255/ 255.0, 158 / 255.0, 0 / 255.0);
					// case 18: return Ogre::ColourValue(233/ 255.0, 150 / 255.0, 70 / 255.0);
					// case 19: return Ogre::ColourValue(255/ 255.0, 83 / 255.0, 0 / 255.0);
					// case 20: return Ogre::ColourValue(255/ 255.0, 215 / 255.0, 0 / 255.0);
					// case 21: return Ogre::ColourValue(255/ 255.0, 61 / 255.0, 99 / 255.0);
					// case 22: return Ogre::ColourValue(255/ 255.0, 140 / 255.0, 0 / 255.0);
					// case 23: return Ogre::ColourValue(255/ 255.0, 99 / 255.0, 71 / 255.0);
					// case 24: return Ogre::ColourValue(0/ 255.0, 207 / 255.0, 191 / 255.0);
					// case 25: return Ogre::ColourValue(175/ 255.0, 0 / 255.0, 75 / 255.0);
					// case 26: return Ogre::ColourValue(75/ 255.0, 0 / 255.0, 75 / 255.0);
					// case 27: return Ogre::ColourValue(112/ 255.0, 180 / 255.0, 60 / 255.0);
					// case 28: return Ogre::ColourValue(222/ 255.0, 184 / 255.0, 135 / 255.0);
					// case 29: return Ogre::ColourValue(255/ 255.0, 228 / 255.0, 196 / 255.0);
					// case 30: return Ogre::ColourValue(0/ 255.0, 175 / 255.0, 0 / 255.0);
					// case 31: return Ogre::ColourValue(255/ 255.0, 240 / 255.0, 245 / 255.0);
			}
			// return render.normalized_value
			//            ? Heatmap::getColor(time_step_[index], heatmap.min_time_step,
			//                                heatmap.max_time_step, render.color_factor)
			//            : Heatmap::getColor(time_step_[index], render.min_normalized_value,
			//                                render.max_normalized_value, render.color_factor);
		case ColoringMode::OCCUPANCY_COLOR:
			assert(position_.size() == occupancy_.size());
			return Heatmap::getColor(occupancy_[index], 0, 100, render.color_factor);
		case ColoringMode::VOXEL_COLOR:
			assert(position_.size() == color_.size());
			return color_[index];
		case ColoringMode::SEMANTIC_COLOR:
		default:
			return Ogre::ColourValue(0, 0, 0, 1);
	}
}
}  // namespace ufomap_ros::rviz_plugins