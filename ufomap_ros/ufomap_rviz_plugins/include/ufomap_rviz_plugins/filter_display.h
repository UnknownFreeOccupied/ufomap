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

#ifndef UFOMAP_RVIZ_PLUGINS_FILTER_DISPLAY_H
#define UFOMAP_RVIZ_PLUGINS_FILTER_DISPLAY_H

// UFO
#include <ufomap_rviz_plugins/filter.h>

// ROS
#include <rviz/frame_manager.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/properties/vector_property.h>

// STL
#include <unordered_map>

namespace ufomap_ros::rviz_plugins
{
class FilterDisplay
{
 public:
	FilterDisplay(rviz::Property* parent, rviz::FrameManager* frame_manager);

	Filter getFilter() const;

 private:
	// Filter
	rviz::Property* filter_;

	// Occupancy
	rviz::BoolProperty* occupancy_;
	rviz::BoolProperty* unknown_;
	rviz::BoolProperty* free_;
	rviz::BoolProperty* occupied_;
	rviz::IntProperty* min_occupancy_;
	rviz::IntProperty* max_occupancy_;

	// Color
	rviz::BoolProperty* color_;

	// Time
	rviz::BoolProperty* time_;
	rviz::FloatProperty* min_time_;
	rviz::FloatProperty* max_time_;

	// Intensity
	rviz::BoolProperty* intensity_;
	rviz::FloatProperty* min_intensity_;
	rviz::FloatProperty* max_intensity_;

	// Count
	rviz::BoolProperty* count_;
	rviz::IntProperty* min_count_;
	rviz::IntProperty* max_count_;

	// Reflection
	rviz::BoolProperty* reflection_;
	rviz::BoolProperty* reflectiveness_;
	rviz::FloatProperty* min_reflectivness_;
	rviz::FloatProperty* max_reflectivness_;
	rviz::BoolProperty* hits_;
	rviz::IntProperty* min_hits_;
	rviz::IntProperty* max_hits_;
	rviz::BoolProperty* misses_;
	rviz::IntProperty* min_misses_;
	rviz::IntProperty* max_misses_;

	// Surfel
	rviz::BoolProperty* surfel_;
	rviz::BoolProperty* planarity_;
	rviz::FloatProperty* min_planarity_;
	rviz::FloatProperty* max_planarity_;

	// Semantic
	rviz::BoolProperty* semantics_;
	rviz::BoolProperty* semantic_value_;
	rviz::FloatProperty* min_semantic_value_;
	rviz::FloatProperty* max_semantic_value_;
	rviz::BoolProperty* semantic_tag_;
	std::vector<rviz::BoolProperty*> semantic_tags_;
	std::unordered_map<rviz::BoolProperty*, rviz::BoolProperty*> semantic_tag_set_color_;
	std::unordered_map<rviz::BoolProperty*, rviz::ColorProperty*> semantic_tag_colors_;
	std::unordered_map<rviz::BoolProperty*, rviz::BoolProperty*> semantic_tag_set_label_;
	std::unordered_map<rviz::BoolProperty*, std::vector<rviz::BoolProperty*>>
	    semantic_tag_labels_;
	rviz::BoolProperty* semantic_label_;
	std::vector<rviz::BoolProperty*> semantic_labels_;

	// Frame manager
	mutable rviz::FrameManager* frame_manager_;

	// Bounding box
	rviz::BoolProperty* bbx_;
	rviz::TfFrameProperty* tf_bbx_;
	rviz::VectorProperty* min_bbx_;
	rviz::VectorProperty* max_bbx_;
};
}  // namespace ufomap_ros::rviz_plugins

#endif  // UFOMAP_RVIZ_PLUGINS_FILTER_DISPLAY_H