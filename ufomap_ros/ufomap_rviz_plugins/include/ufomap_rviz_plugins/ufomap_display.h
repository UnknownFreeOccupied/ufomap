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

#ifndef UFOMAP_RVIZ_PLUGINS_UFOMAP_DISPLAY_H
#define UFOMAP_RVIZ_PLUGINS_UFOMAP_DISPLAY_H

// UFO
#include <ufo/geometry/frustum.h>
#include <ufo/map/code.h>
#include <ufo/map/ufomap.h>
#include <ufomap_msgs/UFOMap.h>
// #include <ufomap_rviz_plugins/data.h>
#include <ufomap_rviz_plugins/filter_display.h>
#include <ufomap_rviz_plugins/performance_display.h>
// #include <ufomap_rviz_plugins/render_data.h>
// #include <ufomap_rviz_plugins/render_display.h>
// #include <ufomap_rviz_plugins/voxels.h>
#include <ufomap_rviz_plugins/render_mode.h>
#include <ufomap_rviz_plugins/state.h>
#include <ufomap_rviz_plugins/worker_base.h>

// ROS
#include <rviz/message_filter_display.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/properties/vector_property.h>

// STL
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

namespace ufomap_ros::rviz_plugins
{
class UFOMapDisplay : public rviz::MessageFilterDisplay<ufomap_msgs::UFOMap>
{
	Q_OBJECT
 public:
	UFOMapDisplay();

	~UFOMapDisplay() override;

	void reset() override;

	void update(float wall_dt, float ros_dt) override;

 protected:
	void setupResources();

	void onInitialize() override;

	void updateGUI();

	void createWorker();

	void processMessage(ufomap_msgs::UFOMap::ConstPtr const& msg) override;

 private:
	ufo::geometry::Frustum viewFrustum(Ogre::Real far_clip) const;

	// std::array<RenderMode, 3> getRenderMode() const;

	// std::array<Heatmap, 3> getHeatmap(Filter const& filter) const;

	void updateStatus();

 private:
	//  Worker
	std::unique_ptr<WorkerBase> worker_;
	ufo::map::mt_t map_type_ = 0;

	// State
	State state_;

	// Filter
	std::unique_ptr<FilterDisplay> filter_display_;
	Filter filter_;

	// Performance
	std::unique_ptr<PerformanceDisplay> performance_display_;

	//
	// GUI properties
	//

	rviz::Property* map_type_property_;
	rviz::BoolProperty* occupancy_property_;
	rviz::BoolProperty* color_property_;
	rviz::BoolProperty* time_property_;
	rviz::BoolProperty* intensity_property_;
	rviz::BoolProperty* count_property_;
	rviz::BoolProperty* reflection_property_;
	rviz::BoolProperty* semantic_property_;
	rviz::BoolProperty* surfel_property_;

	rviz::EnumProperty* style_property_;
	rviz::EnumProperty* coloring_property_;

	QString default_style_ = getStr(RenderStyle::BOXES);
	QString default_coloring_ = getStr(ColoringMode::VOXEL);

	rviz::FloatProperty* alpha_property_;
	rviz::FloatProperty* scale_property_;
	rviz::IntProperty* depth_property_;

	rviz::Property* occupancy_thres_property_;
	rviz::IntProperty* occupied_thres_property_;
	rviz::IntProperty* free_thres_property_;
};

}  // namespace ufomap_ros::rviz_plugins

#endif  // UFOMAP_RVIZ_PLUGINS_UFOMAP_DISPLAY_H
