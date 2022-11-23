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
	performance_ =
	    new rviz::Property("Performance", QVariant(), "Performance settings", parent);

	far_clip_ = new rviz::FloatProperty("Far clip distance (m)", -1.0,
	                                    "Infinity if negative", performance_);
	far_clip_->setMin(-1.0);

	grid_size_ = new rviz::FloatProperty("Grid size (m)", 10.0, "", performance_);
	grid_size_->setMin(1.0);
	grid_size_->setMax(100.0);

	min_change_ = new rviz::IntProperty("Min change (%)", 10, "", performance_);
	min_change_->setMin(0);
	min_change_->setMax(100);

	multithreaded_ =
	    new rviz::BoolProperty("Multithreading", true, "Use multithreading", performance_);
}

Performance PerformanceDisplay::performance() const
{
	Performance performance;

	performance.far_clip = far_clip_->getFloat();

	performance.grid_size = grid_size_->getFloat();

	performance.normalized_min_change = min_change_->getInt() / 100.0;

	performance.multithreaded = multithreaded_->getBool();

	return performance;
}
}  // namespace ufomap_ros::rviz_plugins