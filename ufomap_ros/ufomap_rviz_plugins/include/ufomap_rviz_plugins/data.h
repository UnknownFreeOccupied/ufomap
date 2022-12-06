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
#include <ufo/map/surfel/surfel.h>
#include <ufo/map/types.h>
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

	void addPosition(ufo::map::Point position)
	{
		position_.emplace_back(position.x, position.y, position.z);

		for (size_t i = 0; 3 != i; ++i) {
			min_position_[i] = std::min(min_position_[i], position[i]);
			max_position_[i] = std::max(max_position_[i], position[i]);
		}
	}

	void addOccupancy(float occupancy) { occupancy_.push_back(occupancy); }

	void addColor(ufo::map::Color color) { color_.push_back(color); }

	void addTime(ufo::map::time_t time)
	{
		time_.push_back(time);

		min_time_ = std::min(min_time_, time);
		max_time_ = std::max(max_time_, time);
	}

	void addIntensity(ufo::map::intensity_t intensity) { intensity_.push_back(intensity); }

	void addCount(ufo::map::count_t count) { count_.push_back(count); }

	void addReflection(ufo::map::Reflection reflection)
	{
		reflection_.push_back(reflection);
	}

	void addSurfel(ufo::map::Surfel const& surfel) { surfel_.push_back(surfel); }

	template <class SemanticContainer>
	void addSemantics(SemanticContainer const& semantics)
	{
		auto s = semantics_.emplace_back();
		s.reserve(semantics.size());
		for (auto [label, value] : semantics) {
			s.emplace_back(label, value);
		}
	}

	void swap(Data& other);

	std::vector<Voxels::Voxel> generateVoxels(RenderMode const& render,
	                                          Filter const& filter, double size,
	                                          Heatmap const& heatmap) const;

	ufo::map::Point minPosition() const { return min_position_; }

	ufo::map::Point maxPosition() const { return max_position_; }

	ufo::map::time_t minTime() const { return min_time_; }

	ufo::map::time_t maxTime() const { return max_time_; }

	void clear();

 protected:
	bool includeVoxel(Filter const& filter, double size, size_t index) const;

	Ogre::ColourValue color(RenderMode const& mode, Heatmap const& heatmap,
	                        size_t index) const;

 private:
	std::vector<Ogre::Vector3> position_;
	ufo::map::Point min_position_;
	ufo::map::Point max_position_;

	std::vector<ufo::map::occupancy_t> occupancy_;

	std::vector<ufo::map::Color> color_;

	std::vector<ufo::map::time_t> time_;
	ufo::map::time_t min_time_;
	ufo::map::time_t max_time_;

	std::vector<ufo::map::intensity_t> intensity_;
	ufo::map::intensity_t min_intensity_;
	ufo::map::intensity_t max_intensity_;

	std::vector<ufo::map::count_t> count_;
	ufo::map::count_t min_count_;
	ufo::map::count_t max_count_;

	std::vector<ufo::map::Reflection> reflection_;
	ufo::map::count_t min_hits_;
	ufo::map::count_t max_hits_;
	ufo::map::count_t min_misses_;
	ufo::map::count_t max_misses_;
	ufo::map::count_t min_reflectivness_;
	ufo::map::count_t max_reflectivness_;

	std::vector<ufo::map::Surfel> surfel_;

	std::vector<std::vector<ufo::map::Semantic>> semantics_;
};
}  // namespace ufomap_ros::rviz_plugins

#endif  // UFOMAP_RVIZ_PLUGINS_UFOMAP_DATA_H