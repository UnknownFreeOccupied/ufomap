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

#ifndef UFOMAP_RVIZ_PLUGINS_UFOMAP_DISPLAY_H
#define UFOMAP_RVIZ_PLUGINS_UFOMAP_DISPLAY_H

// UFO
#include <ufo/geometry/frustum.h>
#include <ufo/map/code/code.h>
#include <ufo/map/occupancy_map_time_color.h>
#include <ufo/map/occupancy_map_time_color_semantic.h>
#include <ufomap_msgs/UFOMapStamped.h>
#include <ufomap_rviz_plugins/data.h>
#include <ufomap_rviz_plugins/filter_display.h>
#include <ufomap_rviz_plugins/performance_display.h>
#include <ufomap_rviz_plugins/render_data.h>
#include <ufomap_rviz_plugins/render_display.h>
#include <ufomap_rviz_plugins/voxels.h>

// ROS
#include <rviz/message_filter_display.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/properties/vector_property.h>

// STL
#include <atomic>
#include <condition_variable>
#include <memory>
#include <thread>

namespace ufomap_ros::rviz_plugins
{
class UFOMapDisplay : public rviz::MessageFilterDisplay<ufomap_msgs::UFOMapStamped>
{
	Q_OBJECT
 public:
	UFOMapDisplay();
	~UFOMapDisplay() override;

	void reset() override;

	void update(float wall_dt, float ros_dt) override;

 protected:
	void onInitialize() override;

	void setupResources();

	void processMessage(ufomap_msgs::UFOMapStamped::ConstPtr const& msg) override;

 private:
	void processMessages();

	void generateObjects(ufo::map::depth_t depth);

	template <ufo::map::OccupancyState State>
	void generateObjectsImpl(std::vector<ufo::map::Code> const& codes,
	                         ufo::map::depth_t min_depth)
	{
		std::for_each(
		    std::execution::par, std::begin(codes), std::end(codes), [&](auto code) {
			    auto pred = ufo::map::predicate::Leaf(min_depth) &&
			                ufo::map::predicate::ContainOccupancyState<State>() &&
			                ufo::map::predicate::CodePrefix(code);

			    RenderData render_data;
			    render_data.manager_ = context_->getSceneManager();
			    render_data.position_ = Ogre::Vector3(0, 0, 0);  // FIXME: What should this be?
			    render_data.orientation_ =
			        Ogre::Quaternion(1, 0, 0, 0);  // FIXME: What should this be?

			    for (auto const& node : map_.query(pred)) {
				    fillData(render_data.transformed_voxels_[node.depth()], map_, node);
			    }

			    std::scoped_lock object_lock(object_mutex_);
			    queued_objects_[stateToIndex(State)].insert_or_assign(code,
			                                                          std::move(render_data));
		    });
	}

	template <class Map>
	void fillData(Data& data, Map const& map, ufo::map::NodeBV const& node)
	{
		data.addPosition(node.center());

		data.addOccupancy(map.getOccupancy(node) * 100);

		if constexpr (ufo::map::is_base_of_template_v<ufo::map::OccupancyMapTimeBase, Map>) {
			data.addTimeStep(map.getTimeStep(node));
		}

		if constexpr (ufo::map::is_base_of_template_v<ufo::map::ColorMapBase, Map>) {
			data.addColor(map.getColor(node));
		}

		// if constexpr (ufo::map::is_base_of_template_v<ufo::map::SemanticMapBase, Map>) {
		// 	// TODO: Add semantics
		// }
	}

	// void filterMsgs(std::vector<ufomap_msgs::UFOMapStamped::ConstPtr>& msgs);

	void updateMap(std::vector<ufomap_msgs::UFOMapStamped::ConstPtr> const& msgs);

	std::vector<ufo::map::Code> getCodesInFOV(ufo::geometry::Frustum const& view,
	                                          ufo::map::depth_t depth) const;

	ufo::geometry::Frustum getViewFrustum(Ogre::Real far_clip) const;

	std::array<RenderMode, 3> getRenderMode() const;

	std::array<Heatmap, 3> getHeatmap(Filter const& filter) const;

	bool updateGridSizeDepth();

	void clearObjects();

	void updateStatus();

 private:
	static constexpr size_t stateToIndex(ufo::map::OccupancyState state)
	{
		switch (state) {
			case ufo::map::OccupancyState::UNKNOWN:
				return 0;
			case ufo::map::OccupancyState::FREE:
				return 1;
			case ufo::map::OccupancyState::OCCUPIED:
				return 2;
		}
	}

 private:
	//  Map
	ufo::map::OccupancyMapTimeColor map_;

	// Flag to tell the other threads to stop
	std::atomic_bool done_ = false;

	// Flag to tell if we have to regenerate
	std::atomic_bool regenerate_ = false;

	// Message worker
	std::thread message_worker_;
	std::mutex message_mutex_;
	std::condition_variable message_cv_;
	std::vector<ufomap_msgs::UFOMapStamped::ConstPtr> message_queue_;

	// Visible
	std::vector<RenderData*> prev_visible_;

	//
	// Objects
	//

	using ObjectMap = std::unordered_map<ufo::map::Code, RenderData, ufo::map::Code::Hash>;

	std::mutex object_mutex_;

	std::array<ObjectMap, 3> objects_;
	std::array<ObjectMap, 3> queued_objects_;

	//
	// GUI properties
	//

	// Render
	rviz::Property* render_category_property_;
	std::unique_ptr<RenderDisplay> occupied_display_;
	std::unique_ptr<RenderDisplay> free_display_;
	std::unique_ptr<RenderDisplay> unknown_display_;

	// Filter
	rviz::Property* filter_category_property_;
	std::unique_ptr<FilterDisplay> filter_display_;

	// Performance
	rviz::Property* performance_property_;
	std::unique_ptr<PerformanceDisplay> performance_display_;
	ufo::map::depth_t grid_size_depth_ = 12;
	Performance prev_performance_;
};

}  // namespace ufomap_ros::rviz_plugins

#endif  // UFOMAP_RVIZ_PLUGINS_UFOMAP_DISPLAY_H
