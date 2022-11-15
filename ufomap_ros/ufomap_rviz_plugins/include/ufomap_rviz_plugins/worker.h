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

#ifndef UFOMAP_RVIZ_PLUGINS_WORKER_H
#define UFOMAP_RVIZ_PLUGINS_WORKER_H

// UFO
#include <ufo/map/ufomap.h>
#include <ufomap_rviz_plugins/state.h>
#include <ufomap_rviz_plugins/worker_base.h>

namespace ufomap_ros::rviz_plugins
{
template <ufo::map::MapType MapType>
class Worker final : public WorkerBase
{
	using Map = ufo::map::UFOMap<MapType, false, ufo::map::UFOLock::NONE, false, true>;

 public:
	Worker(State& state) : state_(state) {
		// TODO: Start thread
	}

	Worker(ReadBuffer& in, State& state) : map_(in), Worker(state) {}

	double resolution() const { return map_.size(); }

	std::size_t numLeafNodes() const { return map_.numLeafNodes(); }

	std::size_t numInnerLeafNodes() const { return map_.numInnerLeafNodes(); }

	std::size_t numInnerNodes() const { return map_.numInnerNodes(); }

	std::size_t memoryUsage() const { return map_.memoryUsage(); }

	std::size_t memoryAllocated() const { return map_.memoryAllocated(); }

	std::vector<ufo::map::Code> codesInFOV(ufo::geometry::Frustum const& view,
	                                       ufo::map::depth_t depth) const
	{
		std::vector<ufo::map::Code> codes;
		// TODO: Make correct
		using Pred = ufo::map::predicate;
		for (auto node : map_.query(Pred::Depth(depth) && Pred::Intersects(view))) {
			codes.push_back(node.code());
		}
		return codes;
	}

	Buffer write() const { return map_.write(); }

 private:
	void processMessages()
	{
		while (!state_.done) {
			std::unique_lock<std::mutex> message_lock(state_.message_mutex);
			state_.message_cv.wait(
			    message_lock, [this] { return !state_.message_queue.empty() || state_.done; });

			if (done_) {
				// TODO: Implement
			}

			std::vector<ufomap_msgs::UFOMap::ConstPtr> queue;
			queue.swap(message_queue_);
			message_lock.unlock();

			updateMap(queue);

			if (regenerate_) {
				state_.regenerate = false;
				map_.setModified(depth);
			}

			generateObjects(depth);

			map_.resetModified();
		}
	}

	void updateMap(std::vector<ufomap_msgs::UFOMap::ConstPtr> const& queue)
	{
		auto const size = map_.size();
		auto const depth_levels = map_.depthLevels();

		for (auto const& msg : msgs) {
			msgToUfo(msg, map_, false);
		}

		if (map_.size() != size || map_.depthLevels() != depth_levels) {
			clearObjects();
			updateGridSizeDepth();
			state_.regenerate = true;
		}
	}

	void updateGridSizeDepth()
	{
		auto const prev = grid_size_depth_;

		Performance perf = performance_display_->getPerformance();

		for (grid_size_depth_ = 0; grid_size_depth_ < map_.depthLevels() &&
		                           map_.size(grid_size_depth_) < perf.grid_size;
		     ++grid_size_depth_) {
		}

		return prev != grid_size_depth_;
	}

	void generateObjects()
	{
		using Pred = ufo::map::predicate;
		for (auto node :
		     map_.query(Pred::Depth(depth) && Pred::Exists() && Pred::Modified())) {
			RenderData data;
			data.manager_ = ...;
			data.position_ = ...;
			data.orientation_ = ...;

			auto pred =
			    Pred::Leaf() && Pred::DepthMin(min_depth) && Pred::Exists() &&
			    Pred::THEN(Pred::OccupancyMap(), ...) && Pred::THEN(Pred::ColorMap(), ...) &&
			    Pred::THEN(Pred::TimeMap(), ...) && Pred::THEN(Pred::IntensityMap(), ...) &&
			    Pred::THEN(Pred::CountMap(), ...) && Pred::THEN(Pred::ReflectionMap(), ...) &&
			    Pred::THEN(Pred::SurfelMap(), ...) && Pred::THEN(Pred::SemanticMap(), ...);

			for (auto node : map_.queryBV(node, pred)) {
				fillData(data.transformed_voxels_[node.depth()], map_, node);
			}

			std::scoped_lock object_lock(state_.object_mutex);
			state_.queued_objects.insert_or_assign(node.code(), std::move(render_data));
		}
	}

	void fillData(Data& data, ufo::map::NodeBV node) const
	{
		data.addPosition(node.center());

		if constexpr (ufo::util::is_base_of_template_v<OccupancyMapBase, Map>) {
			data.addOccupancy(map.occupancy(node) * 100);
		}

		if constexpr (ufo::util::is_base_of_template_v<TimeMapBase, Map>) {
			data.addTime(map.time(node));
		}

		if constexpr (ufo::util::is_base_of_template_v<IntensityMapBase, Map>) {
			data.addIntensity(map.intensity(node));
		}

		if constexpr (ufo::util::is_base_of_template_v<CountMapBase, Map>) {
			data.addCount(map.count(node));
		}

		if constexpr (ufo::util::is_base_of_template_v<ReflectionMapBase, Map>) {
			data.addHits(map.hits(node));
			data.addMisses(map.misses(node));
		}

		if constexpr (ufo::util::is_base_of_template_v<ColorMapBase, Map>) {
			data.addColor(map.color(node));
		}

		if constexpr (ufo::util::is_base_of_template_v<SemanticMapBase, Map>) {
			data.addSemantics(map.semantics(node));
		}

		if constexpr (ufo::util::is_base_of_template_v<SurfelMapBase, Map>) {
			data.addSurfel(map.surfel(node));
		}
	}

 private:
	//  Map
	Map map_;

	// State
	State& state_;
};

}  // namespace ufomap_ros::rviz_plugins

#endif  // UFOMAP_RVIZ_PLUGINS_WORKER_H