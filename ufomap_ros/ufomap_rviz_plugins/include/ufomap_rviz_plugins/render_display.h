/*
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * License: BSD 3
 */

/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, D. Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
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

#ifndef UFOMAP_RVIZ_PLUGINS_RENDER_DISPLAY_H
#define UFOMAP_RVIZ_PLUGINS_RENDER_DISPLAY_H

// UFO
#include <ufomap_rviz_plugins/render_mode.h>

// ROS
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/string_property.h>

// STL
#include <algorithm>
#include <map>
#include <unordered_map>

namespace ufomap_ros::rviz_plugins
{
class RenderDisplay
{
 public:
	RenderDisplay(rviz::Property* parent, std::string name, RenderStyle default_style,
	              ColoringMode default_coloring_mode, QColor default_color,
	              float default_alpha, bool enabled, bool has_color, bool has_semantics,
	              bool has_time_step);

	bool isEnabled() const;

	void enableVoxelColor(bool enable);

	void enableTimeStep(bool enable);

	void enableSemantics(bool enable);

	RenderMode getRenderMode(double min_occ, double max_occ) const;

 private:
	void updateColoring(ColoringMode wanted);

 private:
	bool has_color_;
	bool has_time_step_;
	bool has_semantics_;

	rviz::BoolProperty* should_render_;
	rviz::EnumProperty* style_;
	rviz::EnumProperty* coloring_;
	std::unordered_map<ColoringMode, rviz::ColorProperty*> color_;
	std::unordered_map<ColoringMode, rviz::BoolProperty*> color_normalize_range_;
	std::unordered_map<ColoringMode, rviz::FloatProperty*> color_normalize_min_value_;
	std::unordered_map<ColoringMode, rviz::FloatProperty*> color_normalize_max_value_;
	std::unordered_map<ColoringMode, rviz::FloatProperty*> color_factor_;
	rviz::IntProperty* time_step_normalize_min_value_;
	rviz::IntProperty* time_step_normalize_max_value_;

	rviz::FloatProperty* alpha_;
	rviz::FloatProperty* scale_;
	rviz::IntProperty* depth_;

	rviz::BoolProperty* use_semantic_;
	rviz::BoolProperty* semantic_labels_container_;
	// FIXME: std::map<std::string,
	//          std::pair<rviz::BoolProperty*,
	//                    std::map<ufo::map::SemanticLabel, rviz::BoolProperty*>>>
	//     semantic_labels_;
	rviz::FloatProperty* semantic_min_probability_;
};
}  // namespace ufomap_ros::rviz_plugins

#endif  // UFOMAP_RVIZ_PLUGINS_RENDER_DISPLAY_H
