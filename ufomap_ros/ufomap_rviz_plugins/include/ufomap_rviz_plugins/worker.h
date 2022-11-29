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
#include <ufo/map/types.h>
#include <ufo/map/ufomap.h>
#include <ufomap_msgs/UFOMap.h>
#include <ufomap_msgs/conversions.h>
#include <ufomap_rviz_plugins/data.h>
#include <ufomap_rviz_plugins/filter.h>
#include <ufomap_rviz_plugins/state.h>
#include <ufomap_rviz_plugins/worker_base.h>

// STL
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

namespace ufomap_ros::rviz_plugins
{
template <ufo::map::mt_t MapType>
class Worker final : public WorkerBase
{
	using Map = ufo::map::UFOMap<MapType, false, ufo::map::UFOLock::NONE, false, true>;

 public:
	Worker(State& state, Filter& filter)
	    : state_(state), filter_(filter), thread_(&Worker::processMessages, this)
	{
	}

	Worker(ufo::map::ReadBuffer& in, State& state, Filter& filter)
	    : map_(in), state_(state), filter_(filter), thread_(&Worker::processMessages, this)
	{
	}

	virtual ~Worker() { stop(); }

	void stop() override
	{
		done_ = true;
		notify();
		if (thread_.joinable()) {
			thread_.join();
		}
	}

	void notify() override { message_cv_.notify_all(); }

	void setGridSize(double grid_size) override { grid_size_ = grid_size; }

	double resolution() const override { return map_.size(); }

	std::size_t numLeafNodes() const override { return map_.numLeafNodes(); }

	std::size_t numInnerLeafNodes() const override { return map_.numInnerLeafNodes(); }

	std::size_t numInnerNodes() const override { return map_.numInnerNodes(); }

	std::size_t memoryUsage() const override { return map_.memoryUsage(); }

	std::size_t memoryAllocated() const override { return map_.memoryUsageAllocated(); }

	std::vector<ufo::map::Code> codesInFOV(ufo::geometry::Frustum const& view,
	                                       ufo::map::depth_t depth) const override
	{
		namespace Pred = ufo::map::predicate;
		std::vector<ufo::map::Code> codes;
		for (auto node : map_.query(
		         Pred::Leaf(depth) && Pred::Exists() && Pred::Intersects(view), true)) {
			codes.push_back(node.code());
		}
		return codes;
	}

	ufo::map::Buffer write() const override { return map_.write(); }

 private:
	void processMessages()
	{
		while (!done_) {
			std::unique_lock<std::mutex> message_lock(state_.message_mutex);
			message_cv_.wait(message_lock,
			                 [this] { return !state_.message_queue.empty() || done_; });

			if (done_) {
				return;
			}

			std::vector<ufomap_msgs::UFOMap::ConstPtr> queue;
			queue.reserve(state_.message_queue.size());
			queue.swap(state_.message_queue);
			message_lock.unlock();

			// updateMap(queue);

			// if (updateGridSizeDepth()) {
			// 	state_.regenerate = true;
			// }

			// generateObjects();

			// map_.resetModified();
		}
	}

	void updateMap(std::vector<ufomap_msgs::UFOMap::ConstPtr> const& queue)
	{
		for (auto const& msg : queue) {
			ufomap_msgs::msgToUfo(*msg, map_, false);
		}
	}

	bool updateGridSizeDepth()
	{
		ufo::map::depth_t depth = std::floor(std::log2(grid_size_ / map_.size()));
		depth = std::min(depth, map_.rootDepth());
		std::swap(depth, grid_size_depth_);
		return depth != grid_size_depth_;
	}

	void generateObjects()
	{
		namespace Pred = ufo::map::predicate;

		bool only_modified = true;
		if (state_.regenerate) {
			state_.clearObjects();
			state_.regenerate = false;
			only_modified = false;
		}

		// TODO: Implement
		auto pred = Pred::Leaf(filter_.min_depth) && Pred::Exists() &&
		            Pred::THEN(Pred::OccupancyMap(),
		                       !filter_.occupancy ||
		                           Pred::OccupancyStates(filter_.unknown, filter_.free,
		                                                 filter_.occupied));
		// 				 &&
		// Pred::THEN(Pred::ColorMap(), !filter_.color || Pred::HasColor()) &&
		// Pred::THEN(
		//     Pred::TimeMap(),
		//     !filter_.time || Pred::TimeInterval(filter_.min_time, filter_.max_time)) &&
		// Pred::THEN(
		//     Pred::IntensityMap(),
		//     !filter_.intensity ||
		//         Pred::IntensityInterval(filter_.min_intensity, filter_.max_intensity)) &&
		// Pred::THEN(
		//     Pred::CountMap(),
		//     !filter_.count || Pred::CountInterval(filter_.min_count, filter_.max_count));
		// 							  &&
		// Pred::THEN(Pred::ReflectionMap(), !filter_.reflection || ...) &&
		// Pred::THEN(Pred::SurfelMap(), !filter_.surfel || ...) &&
		// Pred::THEN(Pred::SemanticMap(), !filter_.semantic || ...);

		for (auto node : map_.query(Pred::Depth(grid_size_depth_) && Pred::Exists() &&
		                            (!only_modified || Pred::Modified()))) {
			std::unordered_map<ufo::map::depth_t, Data> data;

			for (auto node : map_.queryBV(node, pred)) {
				fillData(data[node.depth()], node);
			}

			std::scoped_lock object_lock(state_.object_mutex);
			state_.queued_objects.insert_or_assign(node.code(), std::move(data));
		}
	}

	void fillData(Data& data, ufo::map::NodeBV node) const
	{
		data.addPosition(map_.center(node));

		if constexpr (ufo::util::is_base_of_template_v<ufo::map::OccupancyMapBase, Map>) {
			data.addOccupancy(map_.occupancy(node) * 100);
		}

		if constexpr (ufo::util::is_base_of_template_v<ufo::map::ColorMapBase, Map>) {
			data.addColor(map_.color(node));
		}

		if constexpr (ufo::util::is_base_of_template_v<ufo::map::TimeMapBase, Map>) {
			data.addTime(map_.time(node));
		}

		if constexpr (ufo::util::is_base_of_template_v<ufo::map::IntensityMapBase, Map>) {
			data.addIntensity(map_.intensity(node));
		}

		if constexpr (ufo::util::is_base_of_template_v<ufo::map::CountMapBase, Map>) {
			data.addCount(map_.count(node));
		}

		if constexpr (ufo::util::is_base_of_template_v<ufo::map::ReflectionMapBase, Map>) {
			data.addReflection(map_.hits(node), map_.misses(node));
		}

		if constexpr (ufo::util::is_base_of_template_v<ufo::map::SurfelMapBase, Map>) {
			data.addSurfel(map_.surfel(node));
		}

		// if constexpr (ufo::util::is_base_of_template_v<ufo::map::SemanticMapBase, Map>) {
		// 	data.addSemantics(map_.semantics(node));
		// }
	}

 private:
	//  Map
	Map map_;

	// State
	State& state_;

	// Filter
	Filter& filter_;

	// Grid size
	double grid_size_;
	ufo::map::depth_t grid_size_depth_;

	// Done
	bool done_ = false;

	// Condition variable
	std::condition_variable message_cv_;

	// Thread
	std::thread thread_;
};

}  // namespace ufomap_ros::rviz_plugins

#endif  // UFOMAP_RVIZ_PLUGINS_WORKER_H