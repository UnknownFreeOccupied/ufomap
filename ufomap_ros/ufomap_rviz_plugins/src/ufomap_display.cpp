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
#include <ufo/util/enum.h>
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>
#include <ufomap_rviz_plugins/ufomap_display.h>
#include <ufomap_rviz_plugins/worker.h>

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
UFOMapDisplay::UFOMapDisplay() { setupResources(); }

UFOMapDisplay::~UFOMapDisplay()
{
	if (worker_) {
		worker_->stop();
	}
}

void UFOMapDisplay::setupResources()
{
	auto& rgm = Ogre::ResourceGroupManager::getSingleton();
	if (rgm.resourceGroupExists("ufomap")) {
		return;
	}

	std::string pkg = ros::package::getPath("ufomap_rviz_plugins");

	rgm.createResourceGroup("ufomap");
	rgm.addResourceLocation(pkg + "/ogre_media", "FileSystem", "ufomap");
	rgm.addResourceLocation(pkg + "/ogre_media/materials", "FileSystem", "ufomap");
	rgm.addResourceLocation(pkg + "/ogre_media/materials/scripts", "FileSystem", "ufomap");
	rgm.addResourceLocation(pkg + "/ogre_media/materials/glsl", "FileSystem", "ufomap");
	rgm.initialiseResourceGroup("ufomap");
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

	map_type_property_ = new rviz::Property("Map Type", QVariant(), "Map types", this);
	occupancy_property_ =
	    new rviz::BoolProperty("Occupancy", false, "", map_type_property_);
	color_property_ = new rviz::BoolProperty("Color", false, "", map_type_property_);
	time_property_ = new rviz::BoolProperty("Time", false, "", map_type_property_);
	intensity_property_ =
	    new rviz::BoolProperty("Intensity", false, "", map_type_property_);
	count_property_ = new rviz::BoolProperty("Count", false, "", map_type_property_);
	reflection_property_ =
	    new rviz::BoolProperty("Reflection", false, "", map_type_property_);
	semantic_property_ = new rviz::BoolProperty("Semantic", false, "", map_type_property_);
	surfel_property_ = new rviz::BoolProperty("Surfel", false, "", map_type_property_);

	style_property_ =
	    new rviz::EnumProperty("Render", default_style_, "Render style", this);
	for (RenderStyle style :
	     {RenderStyle::POINTS, RenderStyle::SQUARES, RenderStyle::FLAT_SQUARES,
	      RenderStyle::SPHERES, RenderStyle::TILES, RenderStyle::BOXES,
	      RenderStyle::SURFEL}) {
		style_property_->addOption(getStr(style), ufo::util::enumToValue(style));
	}

	coloring_property_ =
	    new rviz::EnumProperty("Coloring", default_coloring_, "Coloring mode", this);

	alpha_property_ = new rviz::FloatProperty("Alpha", 1.0, "Alpha (transparency)", this);
	alpha_property_->setMin(0.0);
	alpha_property_->setMax(1.0);

	scale_property_ = new rviz::FloatProperty("Scale", 1.0, "Scale", this);
	scale_property_->setMin(0.0);

	depth_property_ = new rviz::IntProperty("Min. Depth", 0, "Minimum render depth", this);
	depth_property_->setMin(0);
	depth_property_->setMax(22);

	occupancy_thres_property_ =
	    new rviz::Property("Occupancy", QVariant(), "Occupancy threshold", this);
	occupied_thres_property_ = new rviz::IntProperty(
	    "Occupied thres.", 50, "Occupied threshold", occupancy_thres_property_);
	free_thres_property_ = new rviz::IntProperty("Free thres.", 50, "Free threshold",
	                                             occupancy_thres_property_);
	occupied_thres_property_->setMin(0);
	occupied_thres_property_->setMax(100);
	free_thres_property_->setMin(0);
	free_thres_property_->setMax(100);

	filter_display_ = std::make_unique<FilterDisplay>(this, context_->getFrameManager());

	performance_display_ = std::make_unique<PerformanceDisplay>(this);

	updateGUI();
}

void UFOMapDisplay::updateGUI()
{
	auto option = coloring_property_->getOptionInt();
	coloring_property_->clearOptions();

	std::array<bool, 12> available_options;
	available_options.fill(false);

	if (color_property_->getBool()) {
		auto num = ufo::util::enumToValue(ColoringMode::VOXEL);
		available_options[num] = true;
		coloring_property_->addOption(getStr(ColoringMode::VOXEL), num);
	}
	if (time_property_->getBool()) {
		auto num = ufo::util::enumToValue(ColoringMode::TIME);
		available_options[num] = true;
		coloring_property_->addOption(getStr(ColoringMode::TIME), num);
	}
	if (semantic_property_->getBool()) {
		auto num = ufo::util::enumToValue(ColoringMode::SEMANTIC);
		available_options[num] = true;
		coloring_property_->addOption(getStr(ColoringMode::SEMANTIC),
		                              ufo::util::enumToValue(ColoringMode::SEMANTIC));
	}
	if (surfel_property_->getBool()) {
		auto num = ufo::util::enumToValue(ColoringMode::SURFEL_NORMAL);
		available_options[num] = true;
		coloring_property_->addOption(getStr(ColoringMode::SURFEL_NORMAL),
		                              ufo::util::enumToValue(ColoringMode::SURFEL_NORMAL));
	}
	if (occupancy_property_->getBool()) {
		auto num = ufo::util::enumToValue(ColoringMode::OCCUPANCY);
		available_options[num] = true;
		coloring_property_->addOption(getStr(ColoringMode::OCCUPANCY),
		                              ufo::util::enumToValue(ColoringMode::OCCUPANCY));
	}
	if (intensity_property_->getBool()) {
		auto num = ufo::util::enumToValue(ColoringMode::INTENSITY);
		available_options[num] = true;
		coloring_property_->addOption(getStr(ColoringMode::INTENSITY),
		                              ufo::util::enumToValue(ColoringMode::INTENSITY));
	}
	if (count_property_->getBool()) {
		auto num = ufo::util::enumToValue(ColoringMode::COUNT);
		available_options[num] = true;
		coloring_property_->addOption(getStr(ColoringMode::COUNT),
		                              ufo::util::enumToValue(ColoringMode::COUNT));
	}
	if (reflection_property_->getBool()) {
		for (auto opt :
		     {ColoringMode::REFLECTIVENESS, ColoringMode::HITS, ColoringMode::MISSES}) {
			auto num = ufo::util::enumToValue(opt);
			available_options[num] = true;
			coloring_property_->addOption(getStr(opt), num);
		}
	}

	for (auto opt : {ColoringMode::X_AXIS, ColoringMode::Y_AXIS, ColoringMode::Z_AXIS,
	                 ColoringMode::FIXED}) {
		auto num = ufo::util::enumToValue(opt);
		available_options[num] = true;
		coloring_property_->addOption(getStr(opt), num);
	}

	coloring_property_->setString(getStr(available_options[option]
	                                         ? static_cast<ColoringMode>(option)
	                                         : ColoringMode::Z_AXIS));
}

void UFOMapDisplay::reset()
{
	MFDClass::reset();
	if (worker_) {
		worker_->stop();
	}
	state_.message_queue.clear();
	state_.clearObjects();
	worker_.reset();
	createWorker();
}

void UFOMapDisplay::createWorker()
{
	ufo::map::mt_t map_type = 0;
	if (occupancy_property_->getBool()) {
		map_type |= ufo::map::MapType::OCCUPANCY;
	}
	if (color_property_->getBool()) {
		map_type |= ufo::map::MapType::COLOR;
	}
	if (time_property_->getBool()) {
		map_type |= ufo::map::MapType::TIME;
	}
	if (intensity_property_->getBool()) {
		map_type |= ufo::map::MapType::INTENSITY;
	}
	if (count_property_->getBool()) {
		map_type |= ufo::map::MapType::COUNT;
	}
	if (reflection_property_->getBool()) {
		map_type |= ufo::map::MapType::REFLECTION;
	}
	if (semantic_property_->getBool()) {
		map_type |= ufo::map::MapType::SEMANTIC;
	}
	if (surfel_property_->getBool()) {
		map_type |= ufo::map::MapType::SURFEL;
	}

	if (map_type == map_type_) {
		return;
	}

	bool share_data = map_type & map_type_;
	updateGUI();
	map_type_ = map_type;

// clang-format off
#define MAP_CASES_REPEAT_2(M, N) M(N) M(N + 1)
#define MAP_CASES_REPEAT_4(M, N) MAP_CASES_REPEAT_2(M, N) MAP_CASES_REPEAT_2(M, N + 2)
#define MAP_CASES_REPEAT_8(M, N) MAP_CASES_REPEAT_4(M, N) MAP_CASES_REPEAT_4(M, N + 4)
#define MAP_CASES_REPEAT_16(M, N) MAP_CASES_REPEAT_8(M, N) MAP_CASES_REPEAT_8(M, N + 8)
#define MAP_CASES_REPEAT_32(M, N) MAP_CASES_REPEAT_16(M, N) MAP_CASES_REPEAT_16(M, N + 16)
#define MAP_CASES_REPEAT_64(M, N) MAP_CASES_REPEAT_32(M, N) MAP_CASES_REPEAT_32(M, N + 32)
#define MAP_CASES_REPEAT_128(M, N) MAP_CASES_REPEAT_64(M, N) MAP_CASES_REPEAT_64(M, N + 64)
#define MAP_CASES_REPEAT_256(M, N) MAP_CASES_REPEAT_128(M, N) MAP_CASES_REPEAT_128(M, N + 128)
// #define MAP_CASES_REPEAT_512(M, N) MAP_CASES_REPEAT_256(M, N) MAP_CASES_REPEAT_256(M, N + 256)
// #define MAP_CASES_REPEAT_1024(M, N) MAP_CASES_REPEAT_512(M, N) MAP_CASES_REPEAT_512(M, N + 512)
// #define MAP_CASES_REPEAT_2048(M, N) MAP_CASES_REPEAT_1024(M, N) MAP_CASES_REPEAT_1024(M, N + 1024)
// #define MAP_CASES_REPEAT_4096(M, N) MAP_CASES_REPEAT_2048(M, N) MAP_CASES_REPEAT_2048(M, N + 2048)
// #define MAP_CASES_REPEAT_8192(M, N) MAP_CASES_REPEAT_4096(M, N) MAP_CASES_REPEAT_4096(M, N + 4096)
// #define MAP_CASES_REPEAT_16384(M, N) MAP_CASES_REPEAT_8192(M, N) MAP_CASES_REPEAT_8192(M, N + 8192)
// #define MAP_CASES_REPEAT_32768(M, N) MAP_CASES_REPEAT_16384(M, N) MAP_CASES_REPEAT_16384(M, N + 16384)
// #define MAP_CASES_REPEAT_65536(M, N) MAP_CASES_REPEAT_32768(M, N) MAP_CASES_REPEAT_32768(M, N + 32768)
// #define MAP_CASES_REPEAT_131072(M, N) MAP_CASES_REPEAT_65536(M, N) MAP_CASES_REPEAT_65536(M, N + 65536)
// #define MAP_CASES_REPEAT_262144(M, N) MAP_CASES_REPEAT_131072(M, N) MAP_CASES_REPEAT_131072(M, N + 131072)
// #define MAP_CASES_REPEAT_524288(M, N) MAP_CASES_REPEAT_262144(M, N) MAP_CASES_REPEAT_262144(M, N + 262144)
// #define MAP_CASES_REPEAT_1048576(M, N) MAP_CASES_REPEAT_524288(M, N) MAP_CASES_REPEAT_524288(M, N + 524288)
	// clang-format on

#define MAP_CASES(N)                                               \
	case N: {                                                        \
		if (worker_ && share_data) {                                   \
			auto buf = worker_->write();                                 \
			worker_ = std::make_unique<Worker<N>>(buf, state_, filter_); \
		} else {                                                       \
			worker_ = std::make_unique<Worker<N>>(state_, filter_);      \
		}                                                              \
		break;                                                         \
	}

	switch (map_type) {
		MAP_CASES_REPEAT_2(MAP_CASES, ufo::map::mt_t(1));
		default:
			worker_.reset();
			break;
	}
}

void UFOMapDisplay::processMessage(ufomap_msgs::UFOMap::ConstPtr const& msg)
{
	std::unique_lock<std::mutex> message_lock(state_.message_mutex);
	if (!state_.message_queue.empty()) {
		auto h1 = ufomap_msgs::msgToHeader(*msg);
		auto h2 = ufomap_msgs::msgToHeader(*state_.message_queue.front());
		if (h1.leaf_size != h2.leaf_size || h1.depth_levels != h2.depth_levels) {
			state_.message_queue.clear();
		}
	}
	state_.message_queue.push_back(msg);
	message_lock.unlock();
	createWorker();
	if (worker_) {
		worker_->notify();
	}
}

void UFOMapDisplay::update(float /* wall_dt */, float /* ros_dt */)
{
	state_.timing.start("Update");
	updateStatus();

	decltype(state_.queued_objects) queued_objects;
	{
		std::scoped_lock object_lock(state_.object_mutex);
		queued_objects.swap(state_.queued_objects);
	}

	// Set previous invisible
	for (auto const& v : state_.prev_visible_objects) {
		scene_node_->removeChild(v->scene_node_);
	}
	state_.prev_visible_objects.clear();

	// Process queued objects
	for (auto& [code, data] : queued_objects) {
		if (data.empty()) {
			state_.objects.erase(code);
		} else {
			auto& obj = state_.objects[code];
			obj.transformed_voxels_ = std::move(data);
		}
	}

	Performance performance = performance_display_->performance();

	// // // Read all values
	// // auto render_mode = getRenderMode();
	// // for (auto& rm : render_mode) {
	// // 	rm.normalized_min_change = performance.normalized_min_change;
	// // }
	// // Filter filter = filter_display_->getFilter();
	// // auto heatmap = getHeatmap(filter);

	if (worker_) {
		// Set things inside FOV visible
		for (auto code : worker_->codesInFOV(viewFrustum(performance.far_clip),
		                                     worker_->gridSizeDepth())) {
			auto obj_it = state_.objects.find(code);
			if (std::end(state_.objects) == obj_it) {
				continue;
			}

			if (obj_it->second.scene_node_) {
				// obj_it->second.generateVoxels(...);
				scene_node_->addChild(obj_it->second.scene_node_);
			} else {
				obj_it->second.position_ = Ogre::Vector3(0, 0, 0);
				obj_it->second.scene_node_ = scene_node_->createChildSceneNode(
				    obj_it->second.position_, obj_it->second.orientation_);
				// obj_it->second.generateVoxels(...);
			}

			state_.prev_visible_objects.push_back(&(obj_it->second));
		}
	}

	state_.timing.stop("Update");

	// auto start = std::chrono::high_resolution_clock::now();

	// Performance performance = performance_display_->getPerformance();

	// if (updateGridSizeDepth() ||
	//     prev_performance_.min_depth_unknown != performance.min_depth_unknown ||
	//     prev_performance_.min_depth_free != performance.min_depth_free ||
	//     prev_performance_.min_depth_occupied != performance.min_depth_occupied) {
	// 	clearObjects();
	// 	regenerate_ = true;
	// 	prev_performance_ = performance;
	// 	return;
	// }

	// ufo::map::depth_t depth = grid_size_depth_;

	// decltype(queued_objects_) queued_objects;
	// {
	// 	std::scoped_lock object_lock(object_mutex_);
	// 	queued_objects.swap(queued_objects_);
	// }

	// // Set previous invisible
	// for (auto const& v : prev_visible_) {
	// 	scene_node_->removeChild(v->scene_node_);
	// }
	// prev_visible_.clear();

	// for (size_t s = 0; s < queued_objects.size(); ++s) {
	// 	for (auto const& [code, data] : queued_objects[s]) {
	// 		if (data.transformed_voxels_.empty()) {
	// 			objects_[s].erase(code);
	// 		} else {
	// 			objects_[s].insert_or_assign(code, std::move(data));
	// 		}
	// 	}
	// }

	// // Read all values
	// auto render_mode = getRenderMode();
	// for (auto& rm : render_mode) {
	// 	rm.normalized_min_change = performance.normalized_min_change;
	// }
	// Filter filter = filter_display_->getFilter();
	// auto heatmap = getHeatmap(filter);

	// // Set things inside FOV visible
	// for (auto code : codesInFOV(viewFrustum(performance.far_clip), depth)) {
	// 	for (size_t s = 0; s < objects_.size(); ++s) {
	// 		if (!render_mode[s].should_render) {
	// 			continue;
	// 		}

	// 		auto obj_it = objects_[s].find(code);
	// 		if (std::end(objects_[s]) == obj_it) {
	// 			continue;
	// 		}

	// 		if (nullptr == obj_it->second.scene_node_) {
	// 			obj_it->second.scene_node_ = scene_node_->createChildSceneNode(
	// 			    obj_it->second.position_, obj_it->second.orientation_);

	// 			obj_it->second.generateVoxels(render_mode[s], filter, heatmap[s],
	// 			                              map_.resolution());
	// 		} else {
	// 			obj_it->second.generateVoxels(render_mode[s], filter, heatmap[s],
	// 			                              map_.resolution());

	// 			scene_node_->addChild(obj_it->second.scene_node_);
	// 		}

	// 		prev_visible_.push_back(&(obj_it->second));
	// 	}
	// }

	// float duration = std::chrono::duration<float, std::chrono::seconds::period>(
	//                      std::chrono::high_resolution_clock::now() - start)
	//                      .count();
	// // std::cout << "Updating time taken " << duration << " s\n";

	if (!state_.timing.empty()) {
		std::string comp = "Component";
		int s = std::max(comp.size(), state_.timing.maxSize());
		int c_s = s - ((s - comp.size()) / 2) - 1;
		printf("Timings\n");
		printf("\t %*s\tTotal\tLast\tMean\tStDev\t Min\t Max\t Steps\n", c_s, comp.c_str());
		std::lock_guard lock(state_.timing_mutex);
		for (auto const& tag : state_.timing.tags()) {
			printf("\t%s%-*s\t%5.2f\t%5.4f\t%5.4f\t%5.4f\t%5.4f\t%5.4f\t%6lu%s\n",
			       state_.timing.color(tag).c_str(), s, tag.c_str(),
			       state_.timing.totalSeconds(tag), state_.timing.lastSeconds(tag),
			       state_.timing.meanSeconds(tag), state_.timing.stdSeconds(tag),
			       state_.timing.minSeconds(tag), state_.timing.maxSeconds(tag),
			       state_.timing.numSamples(tag), ufo::util::Timing::resetColor());
		}
	}
}

ufo::geometry::Frustum UFOMapDisplay::viewFrustum(Ogre::Real far_clip) const
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

// std::array<RenderMode, 3> UFOMapDisplay::getRenderMode() const
// {
// std::array<RenderMode, 3> render_mode;
// render_mode[stateToIndex(ufo::map::OccupancyState::UNKNOWN)] =
//     unknown_display_->getRenderMode(100 * map_.getFreeThres(),
//                                     100 * map_.getOccupiedThres());
// render_mode[stateToIndex(ufo::map::OccupancyState::FREE)] =
//     free_display_->getRenderMode(100 * map_.getOccupancyClampingThresMin(),
//                                  100 * map_.getFreeThres());
// render_mode[stateToIndex(ufo::map::OccupancyState::OCCUPIED)] =
//     occupied_display_->getRenderMode(100 * map_.getOccupiedThres(),
//                                      100 * map_.getOccupancyClampingThresMax());
// return render_mode;
// }

// std::array<Heatmap, 3> UFOMapDisplay::getHeatmap(Filter const& filter) const
// {
// ufo::geometry::Point min_allowed_pos;
// ufo::geometry::Point max_allowed_pos;

// if (filter.filter_bounding_volume) {
// 	min_allowed_pos = filter.bounding_volume.min();
// 	max_allowed_pos = filter.bounding_volume.max();
// } else {
// 	for (size_t i = 0; 3 != i; ++i) {
// 		min_allowed_pos[i] =
// 		    std::numeric_limits<std::decay_t<decltype(min_allowed_pos[i])>>::lowest();
// 		max_allowed_pos[i] =
// 		    std::numeric_limits<std::decay_t<decltype(max_allowed_pos[i])>>::max();
// 	}
// }

// ufo::map::time_t min_allowed_ts;
// ufo::map::time_t max_allowed_ts;
// if (filter.filter_time) {
// 	min_allowed_ts = filter.min_time;
// 	max_allowed_ts = filter.max_time;
// } else {
// 	min_allowed_ts = std::numeric_limits<ufo::map::time_t>::lowest();
// 	max_allowed_ts = std::numeric_limits<ufo::map::time_t>::max();
// }

// std::array<Heatmap, 3> heatmap;
// for (size_t s = 0; heatmap.size() != s; ++s) {
// 	for (size_t i = 0; 3 != i; ++i) {
// 		using PosType = std::decay_t<decltype(heatmap[s].min_position[i])>;
// 		heatmap[s].min_position[i] = std::numeric_limits<PosType>::max();
// 		heatmap[s].max_position[i] = std::numeric_limits<PosType>::lowest();
// 	}
// 	heatmap[s].min_time = std::numeric_limits<ufo::map::time_t>::max();
// 	heatmap[s].max_time = std::numeric_limits<ufo::map::time_t>::lowest();

// 	for (auto const& [_, obj] : objects_[s]) {
// 		ufo::geometry::Point min_cur_pos;
// 		ufo::geometry::Point max_cur_pos;
// 		for (size_t i = 0; 3 != i; ++i) {
// 			using PosType = std::decay_t<decltype(min_cur_pos[i])>;
// 			min_cur_pos[i] = std::numeric_limits<PosType>::max();
// 			max_cur_pos[i] = std::numeric_limits<PosType>::lowest();
// 		}
// 		ufo::map::time_t min_cur_time = std::numeric_limits<ufo::map::time_t>::max();
// 		ufo::map::time_t max_cur_time = std::numeric_limits<ufo::map::time_t>::lowest();

// 		for (auto const& [_, data] : obj.transformed_voxels_) {
// 			auto min_pos = data.minPosition();
// 			auto max_pos = data.maxPosition();
// 			auto min_ts = data.minTime();
// 			auto max_ts = data.maxTime();
// 			for (size_t i = 0; 3 != i; ++i) {
// 				min_cur_pos[i] = std::min(min_cur_pos[i], min_pos[i]);
// 				max_cur_pos[i] = std::max(max_cur_pos[i], max_pos[i]);
// 			}
// 			min_cur_time = std::min(min_cur_time, min_ts);
// 			max_cur_time = std::max(max_cur_time, max_ts);
// 		}

// 		if (min_cur_time > max_allowed_ts || max_cur_time < min_allowed_ts) {
// 			continue;
// 		}

// 		if (min_cur_pos.x > max_allowed_pos.x || min_cur_pos.y > max_allowed_pos.y ||
// 		    min_cur_pos.z > max_allowed_pos.z || max_cur_pos.x < min_allowed_pos.x ||
// 		    max_cur_pos.y < min_allowed_pos.y || max_cur_pos.z < min_allowed_pos.z) {
// 			continue;
// 		}

// 		for (size_t i = 0; 3 != i; ++i) {
// 			heatmap[s].min_position[i] = std::min(heatmap[s].min_position[i],
// min_cur_pos[i]); 			heatmap[s].max_position[i] =
// std::max(heatmap[s].max_position[i], max_cur_pos[i]);
// 		}
// 		heatmap[s].min_time = std::min(heatmap[s].min_time, min_cur_time);
// 		heatmap[s].max_time = std::max(heatmap[s].max_time, max_cur_time);
// 	}

// 	for (size_t i = 0; 3 != i; ++i) {
// 		heatmap[s].min_position[i] =
// 		    std::max(heatmap[s].min_position[i], min_allowed_pos[i]);
// 		heatmap[s].max_position[i] =
// 		    std::min(heatmap[s].max_position[i], max_allowed_pos[i]);
// 	}
// 	heatmap[s].min_time = std::max(heatmap[s].min_time, min_allowed_ts);
// 	heatmap[s].max_time = std::min(heatmap[s].max_time, max_allowed_ts);
// }

// return heatmap;
// }

// bool UFOMapDisplay::updateGridSizeDepth()
// {
// auto prev = grid_size_depth_;

// Performance perf = performance_display_->getPerformance();
// grid_size_depth_ = 0;
// for (; grid_size_depth_ < map_.depthLevels() &&
//        map_.getNodeSize(grid_size_depth_) < perf.grid_size;
//      ++grid_size_depth_) {
// }

// return prev != grid_size_depth_;
// }

void UFOMapDisplay::updateStatus()
{
	if (!worker_) {
		setStatus(rviz::StatusProperty::Ok, "Resolution", "");
		setStatus(rviz::StatusProperty::Ok, "Num. leaf nodes", QString("%L1").arg(0));
		setStatus(rviz::StatusProperty::Ok, "Num. inner leaf nodes", QString("%L1").arg(0));
		setStatus(rviz::StatusProperty::Ok, "Num. inner nodes", QString("%L1").arg(0));
		setStatus(rviz::StatusProperty::Ok, "Memory usage", "0");
		setStatus(rviz::StatusProperty::Ok, "Memory allocated", "0");
		return;
	}

	double res = worker_->resolution();
	double mem_usage = worker_->memoryUsage();
	double mem_alloc = worker_->memoryAllocated();

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

	QString mem_usage_str;
	if (1024 > mem_usage) {
		mem_usage_str.setNum(mem_usage, 'g', 3);
		mem_usage_str += " B";
	} else if (1024 * 1024 > mem_usage) {
		mem_usage_str.setNum(mem_usage / 1024, 'g', 3);
		mem_usage_str += " KiB";
	} else if (1024 * 1024 * 1024 > mem_usage) {
		mem_usage_str.setNum(mem_usage / (1024 * 1024), 'g', 3);
		mem_usage_str += " MiB";
	} else {
		mem_usage_str.setNum(mem_usage / (1024 * 1024 * 1024), 'g', 3);
		mem_usage_str += " GiB";
	}

	QString mem_alloc_str;
	if (1024 > mem_alloc) {
		mem_alloc_str.setNum(mem_alloc, 'g', 3);
		mem_alloc_str += " B";
	} else if (1024 * 1024 > mem_alloc) {
		mem_alloc_str.setNum(mem_alloc / 1024, 'g', 3);
		mem_alloc_str += " KiB";
	} else if (1024 * 1024 * 1024 > mem_alloc) {
		mem_alloc_str.setNum(mem_alloc / (1024 * 1024), 'g', 3);
		mem_alloc_str += " MiB";
	} else {
		mem_alloc_str.setNum(mem_alloc / (1024 * 1024 * 1024), 'g', 3);
		mem_alloc_str += " GiB";
	}

	// QLocale locale;
	setStatus(rviz::StatusProperty::Ok, "Resolution", res_str);
	setStatus(rviz::StatusProperty::Ok, "Num. leaf nodes",
	          QString("%L1").arg(worker_->numLeafNodes()));
	setStatus(rviz::StatusProperty::Ok, "Num. inner leaf nodes",
	          QString("%L1").arg(worker_->numInnerLeafNodes()));
	setStatus(rviz::StatusProperty::Ok, "Num. inner nodes",
	          QString("%L1").arg(worker_->numInnerNodes()));
	setStatus(rviz::StatusProperty::Ok, "Memory usage", mem_usage_str);
	setStatus(rviz::StatusProperty::Ok, "Memory allocated", mem_alloc_str);
}

}  // namespace ufomap_ros::rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ufomap_ros::rviz_plugins::UFOMapDisplay, rviz::Display)
