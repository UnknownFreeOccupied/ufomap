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
#include <ufomap_rviz_plugins/performance_display.h>

namespace ufomap_ros::rviz_plugins
{
PerformanceDisplay::PerformanceDisplay(rviz::Property* parent)
{
	far_clip_ = new rviz::FloatProperty("Far clip distance (m)", -1.0,
	                                    "Infinity if negative", parent);
	far_clip_->setMin(-1.0);

	// lod_levels_ =
	//     new rviz::IntProperty("LOD levels", 5, "Number of level of detail levels",
	//     parent);
	// lod_levels_->setMin(0);
	// lod_levels_->setMax(21);
	// lod_distance_ = new rviz::FloatProperty("LOD distance drop-off (m)", 30.0,
	//                                         "Distance for selecting other LOD", parent);
	// lod_distance_->setMin(0.0);
	// lod_distance_->setMax(100000.0);

	grid_size_ = new rviz::FloatProperty("Grid size (m)", 10.0, "", parent);
	grid_size_->setMin(1.0);
	grid_size_->setMax(100.0);

	// target_fps_ = new rviz::FloatProperty("Target FPS", 60.0, "Target frame rate",
	// parent); target_fps_->setMin(0.0); target_fps_->setMax(10000.0);

	normalized_min_change_ =
	    new rviz::IntProperty("Normalized min change (%)", 10, "", parent);
	normalized_min_change_->setMin(0);
	normalized_min_change_->setMax(100);

	multithreaded_ =
	    new rviz::BoolProperty("Multithreading", true, "Use multithreading", parent);

	render_states_ = new rviz::Property("Render states", QVariant(),
	                                    "Should which states to render", parent);
	render_unknown_ =
	    new rviz::BoolProperty("Unknown", true, "Render unknown space", render_states_);
	render_free_ =
	    new rviz::BoolProperty("Free", true, "Render free space", render_states_);
	render_occupied_ =
	    new rviz::BoolProperty("Occupied", true, "Render occupied space", render_states_);

	render_depths_ = new rviz::Property("Render depths", QVariant(),
	                                    "Min depth to render for state", parent);
	min_depth_unknown_ =
	    new rviz::IntProperty("Unknown", 0, "Min depth for unknown space", render_depths_);
	min_depth_free_ =
	    new rviz::IntProperty("Free", 0, "Min depth for free space", render_depths_);
	min_depth_occupied_ = new rviz::IntProperty(
	    "Occupied", 0, "Min depth for occupied space", render_depths_);
	min_depth_unknown_->setMin(0);
	min_depth_unknown_->setMax(21);
	min_depth_free_->setMin(0);
	min_depth_free_->setMax(21);
	min_depth_occupied_->setMin(0);
	min_depth_occupied_->setMax(21);
}

Performance PerformanceDisplay::getPerformance() const
{
	Performance performance;

	performance.far_clip = far_clip_->getFloat();

	// performance.lod_levels = lod_levels_->getInt();

	// performance.lod_distance = lod_distance_->getFloat();

	performance.grid_size = grid_size_->getFloat();

	// performance.target_fps = target_fps_->getFloat();

	performance.normalized_min_change = normalized_min_change_->getInt() / 100.0;

	performance.multithreaded = multithreaded_->getBool();

	performance.render_unknown = render_unknown_->getBool();
	performance.render_free = render_free_->getBool();
	performance.render_occupied = render_occupied_->getBool();

	performance.min_depth_unknown = min_depth_unknown_->getInt();
	performance.min_depth_free = min_depth_free_->getInt();
	performance.min_depth_occupied = min_depth_occupied_->getInt();

	return performance;
}
}  // namespace ufomap_ros::rviz_plugins