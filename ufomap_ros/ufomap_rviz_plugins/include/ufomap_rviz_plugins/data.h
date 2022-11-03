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

#ifndef UFOMAP_RVIZ_PLUGINS_UFOMAP_DATA_H
#define UFOMAP_RVIZ_PLUGINS_UFOMAP_DATA_H

// UFO
#include <ufo/geometry/aaebb.h>
#include <ufo/geometry/intersects.h>
#include <ufo/map/point.h>
#include <ufo/map/semantic/semantic.h>
#include <ufo/map/types.h>
#include <ufomap_rviz_plugins/color.h>
#include <ufomap_rviz_plugins/filter.h>
#include <ufomap_rviz_plugins/heatmap.h>
#include <ufomap_rviz_plugins/render_mode.h>
#include <ufomap_rviz_plugins/voxels.h>

// STL
#include <cstdint>
#include <map>
#include <unordered_map>
#include <vector>

namespace ufomap_ros::rviz_plugins
{
class Data
{
 public:
	Data();

	void addPosition(ufo::map::Point3 position)
	{
		position_.emplace_back(position.x, position.y, position.z);

		for (size_t i = 0; 3 != i; ++i) {
			min_position_[i] = std::min(min_position_[i], position[i]);
			max_position_[i] = std::max(max_position_[i], position[i]);
		}
	}

	void addOccupancy(float occupancy) { occupancy_.emplace_back(occupancy); }

	void addTimeStep(ufo::map::time_step_t time_step)
	{
		time_step_.emplace_back(time_step);

		min_time_step_ = std::min(min_time_step_, time_step);
		max_time_step_ = std::max(max_time_step_, time_step);
	}

	void addColor(ufo::map::RGBColor color)
	{
		color_.emplace_back(color_lut[color.blue], color_lut[color.green],
		                    color_lut[color.red], 1.0f);
	}

	void addSemantics(std::vector<ufo::map::SemanticPair> const& semantics)
	{
		semantics_.push_back(semantics);
	}

	void swap(Data& other);

	std::vector<Voxels::Voxel> generateVoxels(RenderMode const& render,
	                                          Filter const& filter, double size,
	                                          Heatmap const& heatmap) const;

	ufo::map::Point3 minPosition() const { return min_position_; }

	ufo::map::Point3 maxPosition() const { return max_position_; }

	ufo::map::time_step_t minTimeStep() const { return min_time_step_; }

	ufo::map::time_step_t maxTimeStep() const { return max_time_step_; }

	ufo::map::SemanticPair maxSemantic(std::size_t index) const
	{
		if (semantics_[index].empty()) {
			return ufo::map::SemanticPair();
		}
		ufo::map::SemanticPair max = semantics_[index].front();
		for (auto a : semantics_[index]) {
			if (max.value < a.value) {
				max = a;
			}
		}
		return max;
	}

	void clear();

 protected:
	bool includeVoxel(Filter const& filter, double size, size_t index) const;

	Ogre::ColourValue getColor(RenderMode const& mode, Heatmap const& heatmap,
	                           size_t index) const;

 private:
	std::vector<Ogre::Vector3> position_;
	std::vector<float> occupancy_;
	std::vector<ufo::map::time_step_t> time_step_;
	std::vector<Ogre::ColourValue> color_;
	std::vector<std::vector<ufo::map::SemanticPair>> semantics_;

	ufo::map::Point3 min_position_;
	ufo::map::Point3 max_position_;

	ufo::map::time_step_t min_time_step_;
	ufo::map::time_step_t max_time_step_;
};
}  // namespace ufomap_ros::rviz_plugins

#endif  // UFOMAP_RVIZ_PLUGINS_UFOMAP_DATA_H