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

#ifndef UFOMAP_RVIZ_PLUGINS_FILTER_DISPLAY_H
#define UFOMAP_RVIZ_PLUGINS_FILTER_DISPLAY_H

// UFO
#include <ufomap_rviz_plugins/filter.h>

// ROS
#include <rviz/frame_manager.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/properties/vector_property.h>

namespace ufomap_ros::rviz_plugins
{
class FilterDisplay
{
 public:
	FilterDisplay(rviz::Property* parent, rviz::FrameManager* frame_manager);

	Filter getFilter() const;

 private:
	// Occupancy thresholds
	rviz::BoolProperty* filter_occupancy_;
	rviz::IntProperty* min_occupancy_;
	rviz::IntProperty* max_occupancy_;

	// Time step
	rviz::BoolProperty* filter_time_step_;
	rviz::IntProperty* min_time_step_;
	rviz::IntProperty* max_time_step_;

	// Semantics
	rviz::BoolProperty* filter_semantics_;
	// TODO: Add labels
	rviz::IntProperty* min_semantic_value_;
	rviz::IntProperty* max_semantic_value_;

	// Frame manager
	mutable rviz::FrameManager* frame_manager_;

	// Bounding box
	rviz::BoolProperty* filter_bbx_;
	rviz::TfFrameProperty* tf_bbx_;
	rviz::VectorProperty* min_bbx_;
	rviz::VectorProperty* max_bbx_;
};
}  // namespace ufomap_ros::rviz_plugins

#endif  // UFOMAP_RVIZ_PLUGINS_FILTER_DISPLAY_H