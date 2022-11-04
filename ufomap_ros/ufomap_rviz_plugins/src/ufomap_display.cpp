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
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>
#include <ufomap_rviz_plugins/ufomap_display.h>

// ROS
#include <ros/package.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <rviz/visualization_manager.h>
#include <tf/transform_listener.h>

// OGRE
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

// QT
// #include <QLocale>

// STL
#include <exception>
#include <future>

namespace ufomap_ros::rviz_plugins
{
UFOMapDisplay::UFOMapDisplay()
    : message_worker_(&UFOMapDisplay::processMessages, this), map_(0.1, 16, false)
{
	setupResources();
}

UFOMapDisplay::~UFOMapDisplay()
{
	done_ = true;
	message_cv_.notify_one();
	message_worker_.join();
}

void UFOMapDisplay::setupResources()
{
	if (Ogre::ResourceGroupManager::getSingleton().resourceGroupExists("ufomap")) {
		return;
	}

	std::string pkg_path = ros::package::getPath("ufomap_rviz_plugins");

	Ogre::ResourceGroupManager::getSingleton().createResourceGroup("ufomap");

	Ogre::ResourceGroupManager::getSingleton().addResourceLocation(pkg_path + "/ogre_media",
	                                                               "FileSystem", "ufomap");
	Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
	    pkg_path + "/ogre_media/materials", "FileSystem", "ufomap");
	Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
	    pkg_path + "/ogre_media/materials/scripts", "FileSystem", "ufomap");
	Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
	    pkg_path + "/ogre_media/materials/glsl", "FileSystem", "ufomap");

	Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup("ufomap");
}

void UFOMapDisplay::onInitialize()
{
	// Use the threaded queue for processing of incoming messages
	// INFO: Has to be called before 'MFDClass::onInitialize();'
	// FIXME: This does not seem to work
	update_nh_.setCallbackQueue(context_->getThreadedQueue());

	MFDClass::onInitialize();

	//
	// GUI properties
	//

	// Render
	render_category_property_ =
	    new rviz::Property("Render", QVariant(), "Render options", this);

	occupied_display_ = std::make_unique<RenderDisplay>(
	    render_category_property_, "Occupied", RenderStyle::BOXES,
	    ColoringMode::VOXEL_COLOR, Qt::blue, 1.0, true, true, true, true);
	free_display_ = std::make_unique<RenderDisplay>(
	    render_category_property_, "Free", RenderStyle::BOXES, ColoringMode::FIXED_COLOR,
	    Qt::green, 0.03, false, true, true, true);
	unknown_display_ = std::make_unique<RenderDisplay>(
	    render_category_property_, "Unknown", RenderStyle::BOXES, ColoringMode::FIXED_COLOR,
	    Qt::white, 0.03, false, true, true, true);

	// Filter
	filter_category_property_ =
	    new rviz::Property("Filter", QVariant(), "Filtering options", this);
	filter_display_ = std::make_unique<FilterDisplay>(filter_category_property_,
	                                                  context_->getFrameManager());

	// Performance
	performance_property_ =
	    new rviz::Property("Performance", QVariant(), "Performance options", this);
	performance_display_ = std::make_unique<PerformanceDisplay>(performance_property_);
}

void UFOMapDisplay::reset()
{
	MFDClass::reset();
	done_ = true;
	message_cv_.notify_one();
	message_worker_.join();
	message_queue_.clear();
	map_.clear();
	clearObjects();
	done_ = false;
	message_worker_ = std::thread(&UFOMapDisplay::processMessages, this);
}

void UFOMapDisplay::processMessage(ufomap_msgs::UFOMap::ConstPtr const& msg)
{
	std::scoped_lock<std::mutex> message_lock(message_mutex_);
	message_queue_.push_back(msg);
	message_cv_.notify_one();
}

void UFOMapDisplay::processMessages()
{
	while (!done_) {
		std::unique_lock<std::mutex> message_lock(message_mutex_);
		message_cv_.wait(message_lock, [this] { return !message_queue_.empty() || done_; });

		if (done_) {
			return;
		}

		auto start = std::chrono::high_resolution_clock::now();

		// Copy and clear global queue
		std::vector<ufomap_msgs::UFOMap::ConstPtr> local_queue;
		local_queue.swap(message_queue_);

		message_lock.unlock();

		// TODO: Get parameters
		ufo::map::depth_t depth = grid_size_depth_;

		updateMap(local_queue);

		map_.updateModifiedNodes(std::max(static_cast<ufo::map::depth_t>(1U), depth) -
		                         1);  // Reset modified up to depth-1

		float duration = std::chrono::duration<float, std::chrono::seconds::period>(
		                     std::chrono::high_resolution_clock::now() - start)
		                     .count();
		std::cout << "Process message time taken " << duration << " s\n";

		if (regenerate_) {
			regenerate_ = false;
			map_.setModified(depth);
		}

		generateObjects(depth);

		map_.clearModified();

		updateStatus();
	}
}

void UFOMapDisplay::generateObjects(ufo::map::depth_t depth)
{
	auto start = std::chrono::high_resolution_clock::now();

	std::vector<ufo::map::Code> codes;

	auto pred = ufo::map::predicate::Leaf(depth) && ufo::map::predicate::Depth(depth) &&
	            ufo::map::predicate::Modified();

	for (auto const& node : map_.query(pred)) {
		codes.push_back(node.code());
	}

	Performance perf = performance_display_->getPerformance();

	std::future<void> unknown_future;
	std::future<void> free_future;

	if (perf.render_unknown) {
		unknown_future =
		    std::async(std::launch::async,
		               &UFOMapDisplay::generateObjectsImpl<ufo::map::OccupancyState::UNKNOWN>,
		               this, codes, perf.min_depth_unknown);
	}
	if (perf.render_free) {
		free_future =
		    std::async(std::launch::async,
		               &UFOMapDisplay::generateObjectsImpl<ufo::map::OccupancyState::FREE>,
		               this, codes, perf.min_depth_free);
	}

	if (perf.render_occupied) {
		generateObjectsImpl<ufo::map::OccupancyState::OCCUPIED>(codes,
		                                                        perf.min_depth_occupied);
	}

	if (perf.render_unknown) {
		unknown_future.wait();
	}
	if (perf.render_free) {
		free_future.wait();
	}

	float duration = std::chrono::duration<float, std::chrono::seconds::period>(
	                     std::chrono::high_resolution_clock::now() - start)
	                     .count();
	std::cout << "Generate objects time taken " << duration << " s\n";
}

void UFOMapDisplay::update(float /* wall_dt */, float /* ros_dt */)
{
	auto start = std::chrono::high_resolution_clock::now();

	Performance performance = performance_display_->getPerformance();

	if (updateGridSizeDepth() ||
	    prev_performance_.min_depth_unknown != performance.min_depth_unknown ||
	    prev_performance_.min_depth_free != performance.min_depth_free ||
	    prev_performance_.min_depth_occupied != performance.min_depth_occupied) {
		clearObjects();
		regenerate_ = true;
		prev_performance_ = performance;
		return;
	}

	ufo::map::depth_t depth = grid_size_depth_;

	decltype(queued_objects_) queued_objects;
	{
		std::scoped_lock object_lock(object_mutex_);
		queued_objects.swap(queued_objects_);
	}

	// Set previous invisible
	for (auto const& v : prev_visible_) {
		scene_node_->removeChild(v->scene_node_);
	}
	prev_visible_.clear();

	for (size_t s = 0; s < queued_objects.size(); ++s) {
		for (auto const& [code, data] : queued_objects[s]) {
			if (data.transformed_voxels_.empty()) {
				objects_[s].erase(code);
			} else {
				objects_[s].insert_or_assign(code, std::move(data));
			}
		}
	}

	// Read all values
	auto render_mode = getRenderMode();
	for (auto& rm : render_mode) {
		rm.normalized_min_change = performance.normalized_min_change;
	}
	Filter filter = filter_display_->getFilter();
	auto heatmap = getHeatmap(filter);

	// Set things inside FOV visible
	for (auto code : getCodesInFOV(getViewFrustum(performance.far_clip), depth)) {
		for (size_t s = 0; s < objects_.size(); ++s) {
			if (!render_mode[s].should_render) {
				continue;
			}

			auto obj_it = objects_[s].find(code);
			if (std::end(objects_[s]) == obj_it) {
				continue;
			}

			if (nullptr == obj_it->second.scene_node_) {
				obj_it->second.scene_node_ = scene_node_->createChildSceneNode(
				    obj_it->second.position_, obj_it->second.orientation_);

				obj_it->second.generateVoxels(render_mode[s], filter, heatmap[s],
				                              map_.resolution());
			} else {
				obj_it->second.generateVoxels(render_mode[s], filter, heatmap[s],
				                              map_.resolution());

				scene_node_->addChild(obj_it->second.scene_node_);
			}

			prev_visible_.push_back(&(obj_it->second));
		}
	}

	float duration = std::chrono::duration<float, std::chrono::seconds::period>(
	                     std::chrono::high_resolution_clock::now() - start)
	                     .count();
	// std::cout << "Updating time taken " << duration << " s\n";
}

std::vector<ufo::map::Code> UFOMapDisplay::getCodesInFOV(
    ufo::geometry::Frustum const& view, ufo::map::depth_t depth) const
{
	std::vector<ufo::map::Code> codes;
	auto pred = ufo::map::predicate::Leaf(depth) && ufo::map::predicate::Intersects(view);
	for (auto const& node : map_.query(pred)) {
		codes.push_back(node.code());
	}
	return codes;
}

// void UFOMapDisplay::filterMsgs(std::vector<ufomap_msgs::UFOMap::ConstPtr>& msgs)
// {
// 	if (msgs.empty()) {
// 		return;
// 	}

// 	// Check if all messages has same type. If not, take the type of last message and
// 	// remove all message from first until last message that is not of the same type as
// 	// the last message
// 	for (auto it = std::next(msgs.crbegin()); it != msgs.crend(); ++it) {
// 		if (map_.canMerge((*it)->map.data)) {
// 			// Remove from first to and including currently pointed to
// 			msgs.erase(msgs.cbegin(), std::prev(it).base());
// 			break;
// 		}
// 	}
// }

void UFOMapDisplay::updateMap(
    std::vector<ufomap_msgs::UFOMap::ConstPtr> const& msgs)
{
	auto res = map_.resolution();
	auto depth = map_.depthLevels();

	for (auto const& msg : msgs) {
		msgToUfo(msg->map, map_, false);
	}

	if (map_.resolution() != res || map_.depthLevels() != depth) {
		clearObjects();
		updateGridSizeDepth();
		regenerate_ = true;
	}
}

ufo::geometry::Frustum UFOMapDisplay::getViewFrustum(Ogre::Real far_clip) const
{
	Ogre::Viewport* viewport = scene_manager_->getCurrentViewport();
	if (!viewport) {
		return ufo::geometry::Frustum();
		// throw std::runtime_error("Could not retrieve current viewport");
	}

	Ogre::Camera* camera = viewport->getCamera();
	if (!camera) {
		return ufo::geometry::Frustum();
		// throw std::runtime_error("Could not retrieve camera");
	}

	Ogre::Vector3 const& position = camera->getPositionForViewUpdate();
	Ogre::Quaternion const& orientation = camera->getOrientationForViewUpdate();
	Ogre::Vector3 target = position - orientation.zAxis();
	Ogre::Vector3 up = orientation.yAxis();
	Ogre::Real vertical_angle = camera->getFOVy().valueRadians();
	Ogre::Real horizontal_angle = camera->getAspectRatio() * vertical_angle;
	Ogre::Real near_distance = camera->getNearClipDistance();

	Ogre::Real cam_far_dist = camera->getFarClipDistance();
	Ogre::Real far_distance =
	    (far_clip < 0.0) ? cam_far_dist : std::min(far_clip, cam_far_dist);

	return ufo::geometry::Frustum(ufo::geometry::Point(position.x, position.y, position.z),
	                              ufo::geometry::Point(target.x, target.y, target.z),
	                              ufo::geometry::Point(up.x, up.y, up.z), vertical_angle,
	                              horizontal_angle, near_distance, far_distance);
}

std::array<RenderMode, 3> UFOMapDisplay::getRenderMode() const
{
	std::array<RenderMode, 3> render_mode;
	render_mode[stateToIndex(ufo::map::OccupancyState::UNKNOWN)] =
	    unknown_display_->getRenderMode(100 * map_.getFreeThres(),
	                                    100 * map_.getOccupiedThres());
	render_mode[stateToIndex(ufo::map::OccupancyState::FREE)] =
	    free_display_->getRenderMode(100 * map_.getOccupancyClampingThresMin(),
	                                 100 * map_.getFreeThres());
	render_mode[stateToIndex(ufo::map::OccupancyState::OCCUPIED)] =
	    occupied_display_->getRenderMode(100 * map_.getOccupiedThres(),
	                                     100 * map_.getOccupancyClampingThresMax());
	return render_mode;
}

std::array<Heatmap, 3> UFOMapDisplay::getHeatmap(Filter const& filter) const
{
	ufo::geometry::Point min_allowed_pos;
	ufo::geometry::Point max_allowed_pos;

	if (filter.filter_bounding_volume) {
		min_allowed_pos = filter.bounding_volume.min();
		max_allowed_pos = filter.bounding_volume.max();
	} else {
		for (size_t i = 0; 3 != i; ++i) {
			min_allowed_pos[i] =
			    std::numeric_limits<std::decay_t<decltype(min_allowed_pos[i])>>::lowest();
			max_allowed_pos[i] =
			    std::numeric_limits<std::decay_t<decltype(max_allowed_pos[i])>>::max();
		}
	}

	ufo::map::time_t min_allowed_ts;
	ufo::map::time_t max_allowed_ts;
	if (filter.filter_time) {
		min_allowed_ts = filter.min_time;
		max_allowed_ts = filter.max_time;
	} else {
		min_allowed_ts = std::numeric_limits<ufo::map::time_t>::lowest();
		max_allowed_ts = std::numeric_limits<ufo::map::time_t>::max();
	}

	std::array<Heatmap, 3> heatmap;
	for (size_t s = 0; heatmap.size() != s; ++s) {
		for (size_t i = 0; 3 != i; ++i) {
			using PosType = std::decay_t<decltype(heatmap[s].min_position[i])>;
			heatmap[s].min_position[i] = std::numeric_limits<PosType>::max();
			heatmap[s].max_position[i] = std::numeric_limits<PosType>::lowest();
		}
		heatmap[s].min_time = std::numeric_limits<ufo::map::time_t>::max();
		heatmap[s].max_time = std::numeric_limits<ufo::map::time_t>::lowest();

		for (auto const& [_, obj] : objects_[s]) {
			ufo::geometry::Point min_cur_pos;
			ufo::geometry::Point max_cur_pos;
			for (size_t i = 0; 3 != i; ++i) {
				using PosType = std::decay_t<decltype(min_cur_pos[i])>;
				min_cur_pos[i] = std::numeric_limits<PosType>::max();
				max_cur_pos[i] = std::numeric_limits<PosType>::lowest();
			}
			ufo::map::time_t min_cur_time =
			    std::numeric_limits<ufo::map::time_t>::max();
			ufo::map::time_t max_cur_time =
			    std::numeric_limits<ufo::map::time_t>::lowest();

			for (auto const& [_, data] : obj.transformed_voxels_) {
				auto min_pos = data.minPosition();
				auto max_pos = data.maxPosition();
				auto min_ts = data.minTime();
				auto max_ts = data.maxTime();
				for (size_t i = 0; 3 != i; ++i) {
					min_cur_pos[i] = std::min(min_cur_pos[i], min_pos[i]);
					max_cur_pos[i] = std::max(max_cur_pos[i], max_pos[i]);
				}
				min_cur_time = std::min(min_cur_time, min_ts);
				max_cur_time = std::max(max_cur_time, max_ts);
			}

			if (min_cur_time > max_allowed_ts || max_cur_time < min_allowed_ts) {
				continue;
			}

			if (min_cur_pos.x > max_allowed_pos.x || min_cur_pos.y > max_allowed_pos.y ||
			    min_cur_pos.z > max_allowed_pos.z || max_cur_pos.x < min_allowed_pos.x ||
			    max_cur_pos.y < min_allowed_pos.y || max_cur_pos.z < min_allowed_pos.z) {
				continue;
			}

			for (size_t i = 0; 3 != i; ++i) {
				heatmap[s].min_position[i] = std::min(heatmap[s].min_position[i], min_cur_pos[i]);
				heatmap[s].max_position[i] = std::max(heatmap[s].max_position[i], max_cur_pos[i]);
			}
			heatmap[s].min_time = std::min(heatmap[s].min_time, min_cur_time);
			heatmap[s].max_time = std::max(heatmap[s].max_time, max_cur_time);
		}

		for (size_t i = 0; 3 != i; ++i) {
			heatmap[s].min_position[i] =
			    std::max(heatmap[s].min_position[i], min_allowed_pos[i]);
			heatmap[s].max_position[i] =
			    std::min(heatmap[s].max_position[i], max_allowed_pos[i]);
		}
		heatmap[s].min_time = std::max(heatmap[s].min_time, min_allowed_ts);
		heatmap[s].max_time = std::min(heatmap[s].max_time, max_allowed_ts);
	}

	return heatmap;
}

bool UFOMapDisplay::updateGridSizeDepth()
{
	auto prev = grid_size_depth_;

	Performance perf = performance_display_->getPerformance();
	grid_size_depth_ = 0;
	for (; grid_size_depth_ < map_.depthLevels() &&
	       map_.getNodeSize(grid_size_depth_) < perf.grid_size;
	     ++grid_size_depth_) {
	}

	return prev != grid_size_depth_;
}

void UFOMapDisplay::clearObjects()
{
	std::scoped_lock object_lock(object_mutex_);
	for (auto& obj : objects_) {
		obj.clear();
	}
	for (auto& obj : queued_objects_) {
		obj.clear();
	}
	prev_visible_.clear();
}

void UFOMapDisplay::updateStatus()
{
	double res = map_.resolution();

	QString res_str;
	if (0.01 > res) {
		res_str.setNum(res * 1000.0, 'g', 3);
		res_str += " mm";
	} else if (1 > res) {
		res_str.setNum(res * 100.0, 'g', 3);
		res_str += " cm";
	} else {
		res_str.setNum(res, 'g', 3);
		res_str += " m";
	}
	// QLocale locale;
	setStatus(rviz::StatusProperty::Ok, "Resolution", res_str);
	setStatus(rviz::StatusProperty::Ok, "Num. leaf nodes",
	          QString("%L1").arg(map_.numLeafNodes()));
	setStatus(rviz::StatusProperty::Ok, "Num. inner leaf nodes",
	          QString("%L1").arg(map_.numInnerLeafNodes()));
	setStatus(rviz::StatusProperty::Ok, "Num. inner nodes",
	          QString("%L1").arg(map_.numInnerNodes()));
	// setStatus(rviz::StatusProperty::Ok, "Memory usage",
	//           locale.formattedDataSize(map_.memoryUsage()));
	// setStatus(rviz::StatusProperty::Ok, "Memory allocated",
	//           locale.formattedDataSize(map_.memoryUsageAllocated()));
}

}  // namespace ufomap_ros::rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ufomap_ros::rviz_plugins::UFOMapDisplay, rviz::Display)
