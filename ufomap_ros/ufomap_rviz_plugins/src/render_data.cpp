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
#include <ufomap_rviz_plugins/render_data.h>

namespace ufomap_ros::rviz_plugins
{
RenderData::RenderData() : manager_(nullptr), scene_node_(nullptr) {}

RenderData::~RenderData() { clear(); }

void RenderData::clear()
{
	if (scene_node_) {
		manager_->destroySceneNode(scene_node_);
		scene_node_ = nullptr;
	}
}

void RenderData::generateVoxels(RenderMode const& render_mode, Filter const& filter,
                                Heatmap const& heatmap, double leaf_size)
{
	bool regenerate = transformed_voxels_.size() != voxels_.size() ||
	                  render_mode.coloring_mode != voxels_render_mode_.coloring_mode;

	// Filter
	if (!regenerate) {
		bool occ = filter.filter_occupancy != voxels_filter_.filter_occupancy ||
		           (filter.filter_occupancy &&
		            (filter.min_occupancy != voxels_filter_.min_occupancy ||
		             filter.max_occupancy != voxels_filter_.max_occupancy));

		bool c = filter.filter_color != voxels_filter_.filter_color;

		bool ts = filter.filter_time_step != voxels_filter_.filter_time_step ||
		          (filter.filter_time_step &&
		           (filter.min_time_step != voxels_filter_.min_time_step ||
		            filter.max_time_step != voxels_filter_.max_time_step));

		ts = filter.unlabeled != voxels_filter_.unlabeled ||
		     filter.outlier != voxels_filter_.outlier || filter.car != voxels_filter_.car ||
		     filter.bicycle != voxels_filter_.bicycle || filter.bus != voxels_filter_.bus ||
		     filter.motorcycle != voxels_filter_.motorcycle ||
		     filter.on_rails != voxels_filter_.on_rails ||
		     filter.truck != voxels_filter_.truck ||
		     filter.other_vehicle != voxels_filter_.other_vehicle ||
		     filter.person != voxels_filter_.person ||
		     filter.bicyclist != voxels_filter_.bicyclist ||
		     filter.motorcyclist != voxels_filter_.motorcyclist ||
		     filter.road != voxels_filter_.road || filter.parking != voxels_filter_.parking ||
		     filter.sidewalk != voxels_filter_.sidewalk ||
		     filter.other_ground != voxels_filter_.other_ground ||
		     filter.building != voxels_filter_.building ||
		     filter.fence != voxels_filter_.fence ||
		     filter.other_structure != voxels_filter_.other_structure ||
		     filter.lane_marking != voxels_filter_.lane_marking ||
		     filter.vegetation != voxels_filter_.vegetation ||
		     filter.trunk != voxels_filter_.trunk ||
		     filter.terrain != voxels_filter_.terrain || filter.pole != voxels_filter_.pole ||
		     filter.traffic_sign != voxels_filter_.traffic_sign ||
		     filter.other_object != voxels_filter_.other_object ||
		     filter.moving_car != voxels_filter_.moving_car ||
		     filter.moving_bicyclist != voxels_filter_.moving_bicyclist ||
		     filter.moving_person != voxels_filter_.moving_person ||
		     filter.moving_motorcyclist != voxels_filter_.moving_motorcyclist ||
		     filter.moving_on_rails != voxels_filter_.moving_on_rails ||
		     filter.moving_bus != voxels_filter_.moving_bus ||
		     filter.moving_truck != voxels_filter_.moving_truck ||
		     filter.moving_other_vehicle != voxels_filter_.moving_other_vehicle;

		// ts = filter.noise != voxels_filter_.noise || filter.animal != voxels_filter_.animal
		// ||
		//      filter.human_pedestrian_adult != voxels_filter_.human_pedestrian_adult ||
		//      filter.human_pedestrian_child != voxels_filter_.human_pedestrian_child ||
		//      filter.human_pedestrian_construction_worker !=
		//          voxels_filter_.human_pedestrian_construction_worker ||
		//      filter.human_pedestrian_personal_mobility !=
		//          voxels_filter_.human_pedestrian_personal_mobility ||
		//      filter.human_pedestrian_police_officer !=
		//          voxels_filter_.human_pedestrian_police_officer ||
		//      filter.human_pedestrian_stroller != voxels_filter_.human_pedestrian_stroller
		//      || filter.human_pedestrian_wheelchair !=
		//          voxels_filter_.human_pedestrian_wheelchair ||
		//      filter.movable_object_barrier != voxels_filter_.movable_object_barrier ||
		//      filter.movable_object_debris != voxels_filter_.movable_object_debris ||
		//      filter.movable_object_pushable_pullable !=
		//          voxels_filter_.movable_object_pushable_pullable ||
		//      filter.movable_object_trafficcone != voxels_filter_.movable_object_trafficcone
		//      || filter.static_object_bicycle_rack !=
		//      voxels_filter_.static_object_bicycle_rack || filter.vehicle_bicycle !=
		//      voxels_filter_.vehicle_bicycle || filter.vehicle_bus_bendy !=
		//      voxels_filter_.vehicle_bus_bendy || filter.vehicle_bus_rigid !=
		//      voxels_filter_.vehicle_bus_rigid || filter.vehicle_car !=
		//      voxels_filter_.vehicle_car || filter.vehicle_construction !=
		//      voxels_filter_.vehicle_construction || filter.vehicle_emergencyambulance !=
		//      voxels_filter_.vehicle_emergencyambulance || filter.vehicle_emergencypolice !=
		//      voxels_filter_.vehicle_emergencypolice || filter.vehicle_motorcycle !=
		//      voxels_filter_.vehicle_motorcycle || filter.vehicle_trailer !=
		//      voxels_filter_.vehicle_trailer || filter.vehicle_truck !=
		//      voxels_filter_.vehicle_truck || filter.flat_driveable_surface !=
		//      voxels_filter_.flat_driveable_surface || filter.flat_other !=
		//      voxels_filter_.flat_other || filter.flat_sidewalk !=
		//      voxels_filter_.flat_sidewalk || filter.flat_terrain !=
		//      voxels_filter_.flat_terrain || filter.static_manmade !=
		//      voxels_filter_.static_manmade || filter.static_other !=
		//      voxels_filter_.static_other || filter.static_vegetation !=
		//      voxels_filter_.static_vegetation || filter.vehicle_ego !=
		//      voxels_filter_.vehicle_ego;

		bool sem = filter.filter_semantics != voxels_filter_.filter_semantics ||
		           (filter.filter_semantics &&
		            (filter.min_semantic_value != voxels_filter_.min_semantic_value ||
		             filter.max_semantic_value != voxels_filter_.max_semantic_value));

		regenerate = occ || c || ts || sem ||
		             filter.filter_bounding_volume != voxels_filter_.filter_bounding_volume;

		if (!regenerate && filter.filter_bounding_volume) {
			auto new_min = filter.bounding_volume.min();
			auto new_max = filter.bounding_volume.max();
			auto old_min = voxels_filter_.bounding_volume.min();
			auto old_max = voxels_filter_.bounding_volume.max();

			for (size_t i = 0; 3 != i; ++i) {
				double new_diff = new_max[i] - new_min[i];
				double min_diff = std::abs(old_min[i] - new_min[i]);
				double max_diff = std::abs(old_max[i] - new_max[i]);

				regenerate = regenerate ||
				             (render_mode.normalized_min_change <= (min_diff / new_diff)) ||
				             (render_mode.normalized_min_change <= (max_diff / new_diff));
			}
		}
	}

	if (!regenerate) {
		switch (render_mode.coloring_mode) {
			case ColoringMode::TIME_STEP_COLOR:
			case ColoringMode::X_AXIS_COLOR:
			case ColoringMode::Y_AXIS_COLOR:
			case ColoringMode::Z_AXIS_COLOR:
				regenerate = render_mode.color_factor != voxels_render_mode_.color_factor ||
				             render_mode.normalized_value != voxels_render_mode_.normalized_value;
				if (regenerate) {
					break;
				}

				if (!render_mode.normalized_value) {
					regenerate = render_mode.min_normalized_value !=
					                 voxels_render_mode_.min_normalized_value ||
					             render_mode.max_normalized_value !=
					                 voxels_render_mode_.max_normalized_value;
				} else {
					double new_diff;
					double min_diff;
					double max_diff;

					switch (render_mode.coloring_mode) {
						case ColoringMode::TIME_STEP_COLOR:
							new_diff = heatmap.max_time_step - heatmap.min_time_step;
							min_diff = std::abs(static_cast<double>(voxels_heatmap_.min_time_step) -
							                    static_cast<double>(heatmap.min_time_step));
							max_diff = std::abs(static_cast<double>(voxels_heatmap_.max_time_step) -
							                    static_cast<double>(heatmap.max_time_step));
							break;
						case ColoringMode::X_AXIS_COLOR:
							new_diff = heatmap.max_position.x - heatmap.min_position.x;
							min_diff =
							    std::abs(voxels_heatmap_.min_position.x - heatmap.min_position.x);
							max_diff =
							    std::abs(voxels_heatmap_.max_position.x - heatmap.max_position.x);
							break;
						case ColoringMode::Y_AXIS_COLOR:
							new_diff = heatmap.max_position.y - heatmap.min_position.y;
							min_diff =
							    std::abs(voxels_heatmap_.min_position.y - heatmap.min_position.y);
							max_diff =
							    std::abs(voxels_heatmap_.max_position.y - heatmap.max_position.y);
							break;
						case ColoringMode::Z_AXIS_COLOR:
							new_diff = heatmap.max_position.z - heatmap.min_position.z;
							min_diff =
							    std::abs(voxels_heatmap_.min_position.z - heatmap.min_position.z);
							max_diff =
							    std::abs(voxels_heatmap_.max_position.z - heatmap.max_position.z);
							break;
						default:
							break;
					}

					regenerate = (render_mode.normalized_min_change < (min_diff / new_diff)) ||
					             (render_mode.normalized_min_change < (max_diff / new_diff));
				}

				break;
			case ColoringMode::FIXED_COLOR:
				regenerate = render_mode.color != voxels_render_mode_.color;
				break;
			case ColoringMode::OCCUPANCY_COLOR:
			// TODO: Implement
			default:
				break;
		}
	}

	if (regenerate) {
		static size_t name = 0;
		for (auto& [depth, data] : transformed_voxels_) {
			double size = leaf_size * (1U << depth);

			Voxels* voxels = new Voxels();
			voxels->setName("voxel" + std::to_string(name++) + "depth" + std::to_string(depth));
			voxels->setRenderStyle(render_mode.style);
			auto v = data.generateVoxels(render_mode, filter, size, heatmap);
			voxels->addVoxels(v.data(), v.size());
			voxels->setAlpha(render_mode.alpha, false);
			voxels->setDimensions(size * render_mode.scale);

			scene_node_->attachObject(voxels);

			voxels_[depth].reset(voxels);
		}

		voxels_render_mode_ = render_mode;
		voxels_filter_ = filter;
		voxels_heatmap_ = heatmap;
	} else {
		for (auto& [depth, v] : voxels_) {
			double size = leaf_size * (1U << depth);

			if (render_mode.style != voxels_render_mode_.style) {
				v->setRenderStyle(render_mode.style);
				voxels_render_mode_.style = render_mode.style;
			}
			if (render_mode.alpha != voxels_render_mode_.alpha) {
				v->setAlpha(render_mode.alpha, false);
				voxels_render_mode_.alpha = render_mode.alpha;
			}
			if (render_mode.scale != voxels_render_mode_.scale) {
				v->setDimensions(size * render_mode.scale);
				voxels_render_mode_.scale = render_mode.scale;
			}
		}
	}
}
}  // namespace ufomap_ros::rviz_plugins