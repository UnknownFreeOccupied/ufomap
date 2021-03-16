/**
 * UFO RViz integration
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomap_ros
 * License: BSD 3
 *
 */

/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, D. Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
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
#include <ufo/map/occupancy_map.h>
#include <ufo/map/occupancy_map_color.h>

// UFO ROS
#include <ufomap_msgs/UFOMapStamped.h>
#include <ufomap_msgs/UFOMapMetaData.h>

// ROS
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/display.h>
#include <rviz/ogre_helpers/point_cloud.h>
#endif  // Q_MOC_RUN

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <message_filters/subscriber.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/visualization_manager.h>

// STD
#include <memory>
#include <mutex>
#include <unordered_map>
#include <variant>

namespace ufomap_ros::rviz_plugins
{
class UFOMapDisplay : public rviz::Display
{
	Q_OBJECT
 public:
	enum VoxelType {
		OCCUPIED,
		FREE,
		UNKNOWN,
	};

	enum ColoringMode {
		VOXEL_COLOR,
		X_AXIS_COLOR,
		Y_AXIS_COLOR,
		Z_AXIS_COLOR,
		PROBABLILTY_COLOR,
		FIXED_COLOR,
	};

	UFOMapDisplay();

	virtual ~UFOMapDisplay();

	// Overrides from rviz::Display
	virtual void onInitialize() override;

	virtual void update(float wall_dt, float ros_dt) override;

	virtual void reset() override;

 protected Q_SLOTS:
	void updateQueueSize();

	void updateTopic();

	void updateDepth();

	void updateOccupiedFreeThres();

	void updateRenderMode();

	void updateRenderStyle();

	void updateColorMode();

	void updateAlpha();

	void updateScale();

	void updateBBX();

	void updateReset();

 protected:
	virtual void onEnable() override;

	virtual void onDisable() override;

	void subscribe();

	void unsubscribe();

	void mapCallback(ufomap_msgs::UFOMapStamped::ConstPtr const& msg);

	void addPoint(
	    QHash<VoxelType, std::vector<std::vector<rviz::PointCloud::Point>>>& points,
	    QHash<VoxelType, std::vector<std::vector<float>>>& probabilities,
	    ufo::geometry::AABB const& bbx, VoxelType type, int min_depth,
	    rviz::PointCloud::Point point, double occupancy, unsigned int depth,
	    ufo::geometry::AABB const& aabb) const;

	void updateInfo(double res, size_t num_leaf_nodes, size_t num_inner_nodes, size_t size);

	void colorPoint(rviz::PointCloud::Point& point, ufo::map::Point3 const& min_value,
	                ufo::map::Point3 max_value, double probability, VoxelType type) const;

	void setColor(double value, double min_value, double max_value, double color_factor,
	              rviz::PointCloud::Point& point) const;

	void clear();

	bool updateFromTF();

	bool createMap(ufomap_msgs::UFOMapMetaData const& type);

	bool checkMap(std::string const& type, double resolution,
	              ufo::map::DepthType depth_levels) const;

	virtual void setTopic(QString const& topic, QString const& datatype) override;

	std::string getStrVoxelType(VoxelType const& type) const;

 protected:
	std::variant<std::monostate, ufo::map::OccupancyMap, ufo::map::OccupancyMapColor> map_;

	unsigned int num_messages_received_ = 0;
	bool should_update_ = false;

	std::mutex mutex_;

	std::shared_ptr<message_filters::Subscriber<ufomap_msgs::UFOMapStamped>> sub_;

	// Plugin properties
	rviz::IntProperty* queue_size_property_;
	rviz::RosTopicProperty* topic_property_;
	QHash<VoxelType, rviz::BoolProperty*> render_type_;
	rviz::Property* render_category_property_;
	QHash<VoxelType, rviz::EnumProperty*> render_mode_;
	QHash<VoxelType, rviz::EnumProperty*> coloring_property_;
	QHash<VoxelType, rviz::ColorProperty*> color_property_;
	QHash<VoxelType, rviz::FloatProperty*> color_factor_property_;
	QHash<VoxelType, rviz::FloatProperty*> alpha_property_;
	QHash<VoxelType, rviz::FloatProperty*> scale_property_;
	rviz::IntProperty* depth_property_;
	rviz::Property* occupancy_thres_category_property_;
	rviz::IntProperty* occupied_thres_property_;
	rviz::IntProperty* free_thres_property_;
	rviz::BoolProperty* use_bbx_property_;
	rviz::TfFrameProperty* tf_bbx_property_;
	rviz::VectorProperty* min_bbx_property_;
	rviz::VectorProperty* max_bbx_property_;
	rviz::Property* info_property_;
	rviz::StringProperty* resolution_property_;
	rviz::StringProperty* num_leaf_nodes_property_;
	rviz::StringProperty* num_inner_nodes_property_;
	rviz::StringProperty* size_property_;

	// Point clouds
	QHash<VoxelType, std::vector<rviz::PointCloud>> clouds_;
	std_msgs::Header header_;
};
}  // namespace ufomap_ros::rviz_plugins

#endif  // UFOMAP_RVIZ_PLUGINS_UFOMAP_DISPLAY_H