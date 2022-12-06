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
#include <ufomap_rviz_plugins/color.h>
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
      min_time_(std::numeric_limits<ufo::map::time_t>::max()),
      max_time_(std::numeric_limits<ufo::map::time_t>::lowest())
{
}

void Data::swap(Data& other)
{
	occupancy_.swap(other.occupancy_);
	time_.swap(other.time_);
	color_.swap(other.color_);
	semantics_.swap(other.semantics_);
}

std::vector<Voxels::Voxel> Data::generateVoxels(RenderMode const& render,
                                                Filter const& filter, double size,
                                                Heatmap const& heatmap) const
{
	std::vector<Voxels::Voxel> voxels;
	voxels.reserve(position_.size());

	for (std::size_t i = 0; i < position_.size(); ++i) {
		if (includeVoxel(filter, size, i)) {
			voxels.emplace_back(position_[i], color(render, heatmap, i));
		}
	}

	return voxels;
}

void Data::clear()
{
	occupancy_.clear();
	time_.clear();
	color_.clear();
	semantics_.clear();
}

bool Data::includeVoxel(Filter const& filter, double size, std::size_t index) const
{
	if (filter.occupancy && occupancy_.size() == position_.size() &&
	    (occupancy_[index] < filter.min_occupancy ||
	     occupancy_[index] > filter.max_occupancy)) {
		return false;
	}

	if (filter.color && color_.size() == position_.size() && !color_[index].isSet()) {
		return false;
	}

	if (filter.time && time_.size() == position_.size() &&
	    (time_[index] < filter.min_time || time_[index] > filter.max_time)) {
		return false;
	}

	if (filter.intensity && intensity_.size() == position_.size() &&
	    (intensity_[index] < filter.min_intensity ||
	     intensity_[index] > filter.max_intensity)) {
		return false;
	}

	if (filter.count && count_.size() == position_.size() &&
	    (count_[index] < filter.min_count || count_[index] > filter.max_count)) {
		return false;
	}

	// TODO: Add reflection, surfel, and semantics

	if (filter.bounding_volume) {
		ufo::geometry::AAEBB aaebb(position_[index][0], position_[index][1],
		                           position_[index][2], size / 2.0);

		if (!ufo::geometry::intersects(aaebb, filter.bv)) {
			return false;
		}
	}

	return true;
}

Ogre::ColourValue Data::color(RenderMode const& render, Heatmap const& heatmap,
                              std::size_t index) const
{
	// FIXME: Use alpha?

	switch (render.coloring_mode) {
		case ColoringMode::FIXED:
			return render.color;
		case ColoringMode::X_AXIS:
			return render.normalized_value
			           ? Heatmap::color(position_[index].x, heatmap.min_position.x,
			                            heatmap.max_position.x, render.color_factor)
			           : Heatmap::color(position_[index].x, render.min_normalized_value,
			                            render.max_normalized_value, render.color_factor);
		case ColoringMode::Y_AXIS:
			return render.normalized_value
			           ? Heatmap::color(position_[index].y, heatmap.min_position.y,
			                            heatmap.max_position.y, render.color_factor)
			           : Heatmap::color(position_[index].y, render.min_normalized_value,
			                            render.max_normalized_value, render.color_factor);
		case ColoringMode::Z_AXIS:
			return render.normalized_value
			           ? Heatmap::color(position_[index].z, heatmap.min_position.z,
			                            heatmap.max_position.z, render.color_factor)
			           : Heatmap::color(position_[index].z, render.min_normalized_value,
			                            render.max_normalized_value, render.color_factor);
		case ColoringMode::TIME:
			assert(position_.size() == time_.size());
			return render.normalized_value
			           ? Heatmap::color(time_[index], heatmap.min_time, heatmap.max_time,
			                            render.color_factor)
			           : Heatmap::color(time_[index], render.min_normalized_value,
			                            render.max_normalized_value, render.color_factor);
		case ColoringMode::OCCUPANCY:
			assert(position_.size() == occupancy_.size());
			return Heatmap::color(occupancy_[index], 0, 100, render.color_factor);
		case ColoringMode::VOXEL:
			assert(position_.size() == color_.size());
			return Ogre::ColourValue(color_lut[color_[index].red],
			                         color_lut[color_[index].green],
			                         color_lut[color_[index].blue]);
		case ColoringMode::INTENSITY:
			assert(position_.size() == intensity_.size());
			return render.normalized_value
			           ? Heatmap::color(intensity_[index], heatmap.min_intensity,
			                            heatmap.max_intensity, render.color_factor)
			           : Heatmap::color(intensity_[index], render.min_normalized_value,
			                            render.max_normalized_value, render.color_factor);
		case ColoringMode::COUNT:
			assert(position_.size() == count_.size());
			return render.normalized_value
			           ? Heatmap::color(count_[index], heatmap.min_count, heatmap.max_count,
			                            render.color_factor)
			           : Heatmap::color(count_[index], render.min_normalized_value,
			                            render.max_normalized_value, render.color_factor);
		case ColoringMode::HITS:
			assert(position_.size() == hits_.size());
			return render.normalized_value
			           ? Heatmap::color(hits_[index], heatmap.min_hits, heatmap.max_hits,
			                            render.color_factor)
			           : Heatmap::color(hits_[index], render.min_normalized_value,
			                            render.max_normalized_value, render.color_factor);
		case ColoringMode::MISSES:
			assert(position_.size() == misses_.size());
		case ColoringMode::REFLECTIVENESS:
			assert(position_.size() == hits_.size());
		case ColoringMode::SEMANTIC:
			assert(position_.size() == semantics_.size());
		case ColoringMode::SURFEL_NORMAL:
			assert(position_.size() == surfel_.size());
			return Ogre::ColourValue(0, 0, 0, 1);
	}
}
}  // namespace ufomap_ros::rviz_plugins