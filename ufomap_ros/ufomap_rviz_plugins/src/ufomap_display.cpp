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

// UFO ROS
#include <ufomap_msgs/conversions.h>
#include <ufomap_rviz_plugins/ufomap_display.h>

#include <QLocale>

namespace ufomap_ros::rviz_plugins
{
UFOMapDisplay::UFOMapDisplay() : rviz::Display() {}

UFOMapDisplay::~UFOMapDisplay()
{
	unsubscribe();

	clouds_.clear();

	if (scene_node_) {
		scene_node_->detachAllObjects();
	}
}

void UFOMapDisplay::onInitialize()
{
	// *************
	// Initialize all the plugin properties for rviz
	// *************
	topic_property_ = new rviz::RosTopicProperty(
	    "UFOMap Topic", "",
	    QString::fromStdString(ros::message_traits::datatype<ufomap_msgs::UFOMapStamped>()),
	    "ufomap_msgs::UFOMapStamped topic to subscribe to", this, SLOT(updateTopic()));

	queue_size_property_ = new rviz::IntProperty(
	    "Queue size", 10, "Set the size of the incoming message queue", this,
	    SLOT(updateQueueSize()));
	queue_size_property_->setMin(1);

	info_property_ = new rviz::Property("Information", QVariant(), "", this);
	resolution_property_ = new rviz::StringProperty(
	    "Resolution", "", "Resolution of the occupancy map", info_property_, nullptr, this);
	num_leaf_nodes_property_ =
	    new rviz::StringProperty("# Leaf Nodes", "", "Number of leaf nodes in the octree",
	                             info_property_, nullptr, this);
	num_inner_nodes_property_ =
	    new rviz::StringProperty("# Inner Nodes", "", "Number of inner nodes in the octree",
	                             info_property_, nullptr, this);
	size_property_ = new rviz::StringProperty("Size", "", "Size of the octree",
	                                          info_property_, nullptr, this);

	render_category_property_ = new rviz::Property("Voxel Rendering", QVariant(), "", this);
	for (VoxelType const& type : {OCCUPIED, FREE, UNKNOWN}) {
		QString default_coloring;
		QColor default_color;
		double default_alpha;
		switch (type) {
			case OCCUPIED:
				default_coloring = "Voxel Color";  // FIXME: Should be Z_AXIS if not color UFOMap
				default_color = Qt::blue;
				default_alpha = 1.0;
				break;
			case FREE:
				default_coloring = "Fixed";
				default_color = Qt::green;
				default_alpha = 0.03;
				break;
			case UNKNOWN:
				default_coloring = "Fixed";
				default_color = Qt::white;
				default_alpha = 0.03;
		}

		QString type_str(getStrVoxelType(type).c_str());

		auto it =
		    render_type_.insert(type, new rviz::BoolProperty(type_str, type == OCCUPIED, "",
		                                                     render_category_property_,
		                                                     SLOT(updateRenderMode()), this));
		it.value()->setDisableChildrenIfFalse(true);

		auto render_mode_it = render_mode_.insert(
		    type,
		    new rviz::EnumProperty("Style", "Flat Squares", "Select voxel rendering style",
		                           it.value(), SLOT(updateRenderStyle()), this));
		render_mode_it.value()->addOption("Points", rviz::PointCloud::RM_POINTS);
		render_mode_it.value()->addOption("Squares", rviz::PointCloud::RM_SQUARES);
		render_mode_it.value()->addOption("Flat Squares", rviz::PointCloud::RM_FLAT_SQUARES);
		render_mode_it.value()->addOption("Spheres", rviz::PointCloud::RM_SPHERES);
		render_mode_it.value()->addOption("Tiles", rviz::PointCloud::RM_TILES);
		render_mode_it.value()->addOption("Boxes", rviz::PointCloud::RM_BOXES);

		auto coloring_it = coloring_property_.insert(
		    type,
		    new rviz::EnumProperty("Coloring", default_coloring, "Select voxel coloring mode",
		                           it.value(), SLOT(updateColorMode()), this));

		if (type == OCCUPIED) {  // FIXME: Only if color UFOMap
			coloring_it.value()->addOption("Voxel Color", OCCUPIED);
		}
		coloring_it.value()->addOption("X-Axis", X_AXIS_COLOR);
		coloring_it.value()->addOption("Y-Axis", Y_AXIS_COLOR);
		coloring_it.value()->addOption("Z-Axis", Z_AXIS_COLOR);
		coloring_it.value()->addOption("Cell Probability", PROBABLILTY_COLOR);
		coloring_it.value()->addOption("Fixed", FIXED_COLOR);

		auto color_factor_it = color_factor_property_.insert(
		    type, new rviz::FloatProperty("Factor", 0.8, "", coloring_it.value(),
		                                  SLOT(updateColorMode()), this));
		color_factor_it.value()->setMin(0.0);
		color_factor_it.value()->setMax(1.0);
		if ("Voxel Color" == default_coloring || "Cell Probability" == default_coloring ||
		    "Fixed" == default_coloring) {
			color_factor_it.value()->hide();
		}

		auto color_it = color_property_.insert(
		    type, new rviz::ColorProperty("Color", default_color, "", coloring_it.value(),
		                                  SLOT(updateColorMode()), this));
		if ("Voxel Color" == default_coloring || "X-Axis" == default_coloring ||
		    "Y-Axis" == default_coloring || "Z-Axis" == default_coloring) {
			color_it.value()->hide();
		}

		auto alpha_it = alpha_property_.insert(
		    type,
		    new rviz::FloatProperty("Alpha", default_alpha, "Set voxel transparency alpha",
		                            it.value(), SLOT(updateAlpha()), this));
		alpha_it.value()->setMin(0.0);
		alpha_it.value()->setMax(1.0);

		auto scale_it = scale_property_.insert(
		    type, new rviz::FloatProperty("Scale", 1.0, "Set the voxel scale", it.value(),
		                                  SLOT(updateScale()), this));
	}

	depth_property_ = new rviz::IntProperty("Min. Depth", 0, "", this, SLOT(updateDepth()));
	depth_property_->setMin(0);
	depth_property_->setMax(21);  // FIXME: Should not be hardcoded

	occupancy_thres_category_property_ =
	    new rviz::Property("Occupancy Thresholds", QVariant(), "", this);
	occupied_thres_property_ = new rviz::IntProperty(
	    "Occupied (%)", 50, "Set occupied threshold", occupancy_thres_category_property_,
	    SLOT(updateOccupiedFreeThres()), this);
	free_thres_property_ = new rviz::IntProperty("Free (%)", 50, "Set free threshold",
	                                             occupancy_thres_category_property_,
	                                             SLOT(updateOccupiedFreeThres()), this);
	occupied_thres_property_->setMin(0);
	occupied_thres_property_->setMax(100);
	free_thres_property_->setMin(0);
	free_thres_property_->setMax(100);

	use_bbx_property_ =
	    new rviz::BoolProperty("Use BBX", false, "", this, SLOT(updateBBX()), this);
	use_bbx_property_->setDisableChildrenIfFalse(true);
	tf_bbx_property_ = new rviz::TfFrameProperty(
	    "BBX Frame", rviz::TfFrameProperty::FIXED_FRAME_STRING,
	    "The frame to use for the BBX", use_bbx_property_, context_->getFrameManager(),
	    true, SLOT(updateBBX()), this);
	min_bbx_property_ = new rviz::VectorProperty(
	    "Minimum",
	    Ogre::Vector3(-1000),  // FIXME: Should not be hardcoded
	    "Defines the minimum BBX to display", use_bbx_property_, SLOT(updateBBX()), this);

	max_bbx_property_ = new rviz::VectorProperty(
	    "Maximum",
	    Ogre::Vector3(1000),  // FIXME: Should not be hardcoded
	    "Defines the maximum BBX to display", use_bbx_property_, SLOT(updateBBX()), this);

	std::lock_guard<std::mutex> lock(mutex_);
	for (VoxelType const& type : {OCCUPIED, FREE, UNKNOWN}) {
		clouds_[type].resize(21);  // FIXME: Should not be hardcoded

		for (unsigned int d = 0; d < clouds_[type].size(); ++d) {
			std::stringstream sname;
			sname << getStrVoxelType(type) << " point cloud depth " << d;
			clouds_[type][d].setName(sname.str());
			clouds_[type][d].setRenderMode(rviz::PointCloud::RM_FLAT_SQUARES);
			clouds_[type][d].setCastShadows(false);
			scene_node_->attachObject(&clouds_[type][d]);
		}
	}
}

void UFOMapDisplay::update(float wall_dt, float ros_dt)
{
	if (should_update_) {
		std::lock_guard<std::mutex> lock(mutex_);
		should_update_ = false;

		if (!std::holds_alternative<std::monostate>(map_) &&
		    (render_type_[OCCUPIED]->getBool() || render_type_[FREE]->getBool() ||
		     render_type_[UNKNOWN]->getBool())) {
			std::visit(
			    [this](auto& map) {
				    using T = std::decay_t<decltype(map)>;
				    if constexpr (!std::is_same_v<T, std::monostate>) {
					    QHash<VoxelType, std::vector<std::vector<rviz::PointCloud::Point>>> points;
					    QHash<VoxelType, std::vector<std::vector<float>>> probabilities;
					    for (VoxelType const& type : {OCCUPIED, FREE, UNKNOWN}) {
						    points[type].resize(clouds_[type].size());
						    probabilities[type].resize(clouds_[type].size());
					    }

					    ufo::map::Point3 min_value = map.getMin();
					    ufo::map::Point3 max_value = map.getMax();

					    if (use_bbx_property_->getBool()) {
						    Ogre::Vector3 position;
						    Ogre::Quaternion orientation;
						    context_->getFrameManager()->getTransform(
						        tf_bbx_property_->getFrameStd(), ros::Time(0), position, orientation);

						    Ogre::Vector3 min_bbx = min_bbx_property_->getVector() + position;
						    Ogre::Vector3 max_bbx = max_bbx_property_->getVector() + position;

						    for (int i = 0; i < 3; ++i) {
							    min_value[i] = std::max(min_value[i], static_cast<double>(min_bbx[i]));
							    max_value[i] = std::min(max_value[i], static_cast<double>(max_bbx[i]));
						    }
					    }

					    QHash<VoxelType, ufo::map::Point3> min_coord;
					    QHash<VoxelType, ufo::map::Point3> max_coord;

					    ufo::geometry::AABB aabb_bbx(min_value, max_value);
					    int min_depth = depth_property_->getInt();
					    for (auto it = map.beginLeaves(aabb_bbx, render_type_[OCCUPIED]->getBool(),
					                                   render_type_[FREE]->getBool(),
					                                   render_type_[UNKNOWN]->getBool(), false,
					                                   min_depth),
					              end = map.endLeaves();
					         it != end; ++it) {
						    VoxelType type;
						    if (it.isOccupied()) {
							    type = OCCUPIED;
						    } else if (it.isFree()) {
							    type = FREE;
						    } else {
							    type = UNKNOWN;
						    }

						    ufo::geometry::AABB it_aabb = it.getBoundingVolume();
						    ufo::map::Point3 it_min = it_aabb.getMin();
						    ufo::map::Point3 it_max = it_aabb.getMax();

						    if (min_coord.contains(type)) {
							    for (int i : {0, 1, 2}) {
								    min_coord[type][i] = std::min(min_coord[type][i], it_min[i]);
								    max_coord[type][i] = std::max(max_coord[type][i], it_max[i]);
							    }
						    } else {
							    min_coord[type] = it_min;
							    max_coord[type] = it_max;
						    }

						    rviz::PointCloud::Point point;
						    if constexpr (std::is_same_v<T, ufo::map::OccupancyMapColor>) {
							    point.setColor(it->color.r / 255.0, it->color.g / 255.0,
							                   it->color.b / 255.0, it.getOccupancy());
						    }

						    addPoint(points, probabilities, aabb_bbx, type, min_depth, point,
						             it.getOccupancy(), it.getDepth(), it.getBoundingVolume());
					    }

					    for (VoxelType const& type : points.keys()) {
						    if (OCCUPIED != type ||
						        VOXEL_COLOR != coloring_property_[type]->getOptionInt()) {
							    for (int i : {0, 1, 2}) {
								    // Make sure it is not outside BBX
								    min_coord[type][i] = std::max(min_coord[type][i], min_value[i]);
								    max_coord[type][i] = std::min(max_coord[type][i], max_value[i]);
							    }
							    // Color points
							    for (size_t i = 0; i < points[type].size(); ++i) {
								    for (size_t j = 0; j < points[type][i].size(); ++j) {
									    colorPoint(points[type][i][j], min_coord[type], max_coord[type],
									               probabilities[type][i][j], type);
								    }
							    }
						    }
					    }

					    for (VoxelType const& type : clouds_.keys()) {
						    float scale = scale_property_[type]->getFloat();
						    for (size_t i = 0; i < clouds_[type].size(); ++i) {
							    clouds_[type][i].clear();
							    clouds_[type][i].setAlpha(alpha_property_[type]->getFloat());
							    if (i < map.getTreeDepthLevels()) {
								    float size = scale * map.getNodeSize(i);
								    clouds_[type][i].setDimensions(size, size, size);
							    }
							    if (!points[type][i].empty()) {
								    clouds_[type][i].addPoints(&points[type][i].front(),
								                               points[type][i].size());
							    }
						    }
					    }
				    }
			    },
			    map_);
		} else {
			for (VoxelType const& type : clouds_.keys()) {
				for (size_t i = 0; i < clouds_[type].size(); ++i) {
					clouds_[type][i].clear();
				}
			}
		}

		updateRenderStyle();  // TODO: Should not have to call this here
	}

	updateFromTF();
}

void UFOMapDisplay::reset()
{
	clear();
	num_messages_received_ = 0;
	setStatus(rviz::StatusProperty::Ok, "Messages", QString("0 UFOMap messages received"));
}

void UFOMapDisplay::updateQueueSize() { subscribe(); }

void UFOMapDisplay::updateTopic()
{
	unsubscribe();
	reset();
	subscribe();
	context_->queueRender();
}

void UFOMapDisplay::updateDepth() { should_update_ = true; }

void UFOMapDisplay::updateOccupiedFreeThres()
{
	std::lock_guard<std::mutex> lock(mutex_);

	// Fix floating point accuarcy problems
	double occupied_thres = occupied_thres_property_->getInt() / 100.0;
	double free_thres = free_thres_property_->getInt() / 100.0;

	std::visit(
	    [this, occupied_thres, free_thres](auto& map) {
		    if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>) {
			    map.setOccupiedFreeThres(occupied_thres, free_thres);
		    }
	    },
	    map_);
	should_update_ = true;
}

void UFOMapDisplay::updateRenderMode() { should_update_ = true; }

void UFOMapDisplay::updateRenderStyle()
{
	for (VoxelType const& type : render_mode_.keys()) {
		for (auto& cloud : clouds_[type]) {
			cloud.setRenderMode(
			    static_cast<rviz::PointCloud::RenderMode>(render_mode_[type]->getOptionInt()));
		}
	}
	should_update_ = true;
}

void UFOMapDisplay::updateColorMode()
{
	for (auto const& type : coloring_property_.keys()) {
		switch (coloring_property_[type]->getOptionInt()) {
			case VOXEL_COLOR:
				color_factor_property_[type]->hide();
				color_property_[type]->hide();
				break;
			case PROBABLILTY_COLOR:
			case FIXED_COLOR:
				color_factor_property_[type]->hide();
				color_property_[type]->show();
				break;
			default:
				color_factor_property_[type]->show();
				color_property_[type]->hide();
				break;
		}
	}

	should_update_ = true;
}

void UFOMapDisplay::updateAlpha() { should_update_ = true; }

void UFOMapDisplay::updateScale() { should_update_ = true; }

void UFOMapDisplay::updateBBX() { should_update_ = true; }

void UFOMapDisplay::onEnable()
{
	scene_node_->setVisible(true);
	subscribe();
}

void UFOMapDisplay::onDisable()
{
	scene_node_->setVisible(false);
	unsubscribe();
	clear();
}

void UFOMapDisplay::subscribe()
{
	if (!isEnabled()) {
		return;
	}

	try {
		unsubscribe();

		std::string const& topic(topic_property_->getStdString());

		if (!topic.empty()) {
			sub_.reset(new message_filters::Subscriber<ufomap_msgs::UFOMapStamped>());
			sub_->subscribe(threaded_nh_, topic, queue_size_property_->getInt());
			sub_->registerCallback(boost::bind(&UFOMapDisplay::mapCallback, this, _1));
		}
	} catch (ros::Exception& e) {
		setStatus(rviz::StatusProperty::Error, "Topic",
		          (std::string("Error subscribing: ") + e.what()).c_str());
	}
}

void UFOMapDisplay::unsubscribe()
{
	clear();

	try {
		sub_.reset();
	} catch (ros::Exception& e) {
		setStatus(rviz::StatusProperty::Error, "Topic",
		          (std::string("Error unsubscribing: ") + e.what()).c_str());
	}
}

void UFOMapDisplay::mapCallback(ufomap_msgs::UFOMapStamped::ConstPtr const& msg)
{
	++num_messages_received_;
	setStatus(rviz::StatusProperty::Ok, "Messages",
	          QString::number(num_messages_received_) + " UFOMap messages received");
	setStatusStd(rviz::StatusProperty::Ok, "Type", msg->map.info.id.c_str());

	header_ = msg->header;
	if (!updateFromTF()) {
		std::stringstream ss;
		ss << "Failed to transform from frame [" << header_.frame_id << "] to frame ["
		   << context_->getFrameManager()->getFixedFrame() << "]";
		setStatusStd(rviz::StatusProperty::Error, "Message", ss.str());
		return;
	}

	std::lock_guard<std::mutex> lock(mutex_);

	if (!checkMap(msg->map.info.id, msg->map.info.resolution, msg->map.info.depth_levels)) {
		if (!createMap(msg->map.info)) {
			setStatusStd(rviz::StatusProperty::Error, "Message",
			             (std::string("Unknown UFOMap type '") + msg->map.info.id + "'").c_str());
			return;
		}
	}

	if (!std::visit(
	        [this, &msg](auto& map) -> bool {
		        if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>) {
			        if (ufomap_msgs::msgToUfo(msg->map, map)) {
				        updateInfo(map.getResolution(), map.getNumLeafNodes(),
				                   map.getNumInnerNodes(), map.memoryUsage());
				        return true;
			        }
			        return false;
		        }
		        return false;
	        },
	        map_)) {
		setStatusStd(rviz::StatusProperty::Error, "Message", "Could not create UFOMap");
	}

	should_update_ = true;
}

void UFOMapDisplay::addPoint(
    QHash<VoxelType, std::vector<std::vector<rviz::PointCloud::Point>>>& points,
    QHash<VoxelType, std::vector<std::vector<float>>>& probabilities,
    ufo::geometry::AABB const& bbx, VoxelType type, int min_depth,
    rviz::PointCloud::Point point, double occupancy, unsigned int depth,
    ufo::geometry::AABB const& aabb) const
{
	// ufo::map::Point3 min = aabb.getMin();
	// ufo::map::Point3 max = aabb.getMax();

	// bool valid = depth <= min_depth || (ufo::geometry::intersects(bbx, min) &&
	//                                     ufo::geometry::intersects(bbx, max));

	// if (valid) {
	probabilities[type][depth].push_back(occupancy);
	point.position.x = aabb.center[0];
	point.position.y = aabb.center[1];
	point.position.z = aabb.center[2];
	points[type][depth].push_back(point);
	return;
	// }

	// // Recurse down
	// ufo::geometry::AABB child_aabb;
	// child_aabb.half_size = aabb.half_size / 2.0;
	// for (size_t i = 0; i < 8; ++i) {
	// 	child_aabb.center[0] +=
	// 	    ((i & 1) ? child_aabb.half_size[0] : -child_aabb.half_size[0]);
	// 	child_aabb.center[1] +=
	// 	    ((i & 2) ? child_aabb.half_size[1] : -child_aabb.half_size[1]);
	// 	child_aabb.center[2] +=
	// 	    ((i & 4) ? child_aabb.half_size[2] : -child_aabb.half_size[2]);
	// 	if (ufo::geometry::intersects(bbx, child_aabb)) {
	// 		addPoint(points, probabilities, bbx, type, min_depth, point, occupancy, depth - 1,
	// 		         child_aabb);
	// 	}
	// }
}

void UFOMapDisplay::updateInfo(double res, size_t num_leaf_nodes, size_t num_inner_nodes,
                               size_t size)
{
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
	resolution_property_->setString(res_str);

	num_leaf_nodes_property_->setString(QString("%L1").arg(num_leaf_nodes));
	num_inner_nodes_property_->setString(QString("%L1").arg(num_inner_nodes));

	QLocale locale;
	size_property_->setString(locale.formattedDataSize(size));
}

void UFOMapDisplay::colorPoint(rviz::PointCloud::Point& point,
                               ufo::map::Point3 const& min_value,
                               ufo::map::Point3 max_value, double probability,
                               VoxelType type) const
{
	switch (coloring_property_[type]->getOptionInt()) {
		case X_AXIS_COLOR:
			setColor(point.position.x, min_value.x(), max_value.x(),
			         color_factor_property_[type]->getFloat(), point);
			break;
		case Y_AXIS_COLOR:
			setColor(point.position.y, min_value.y(), max_value.y(),
			         color_factor_property_[type]->getFloat(), point);
			break;
		case Z_AXIS_COLOR:
			setColor(point.position.z, min_value.z(), max_value.z(),
			         color_factor_property_[type]->getFloat(), point);
			break;
		case PROBABLILTY_COLOR: {
			QColor color = color_property_[type]->getColor();
			point.setColor(probability * (color.red() / 255.0),
			               probability * (color.green() / 255.0),
			               probability * (color.blue() / 255.0));
			break;
		}
		case FIXED_COLOR: {
			QColor color = color_property_[type]->getColor();
			point.setColor(color.red() / 255.0, color.green() / 255.0, color.blue() / 255.0);
			break;
		}
	}
}

void UFOMapDisplay::setColor(double value, double min_value, double max_value,
                             double color_factor, rviz::PointCloud::Point& point) const
{
	int i;
	double m, n, f;

	double s = 1.0;
	double v = 1.0;

	double h = (1.0 - std::min(std::max((value - min_value) / (max_value - min_value), 0.0),
	                           1.0)) *
	           color_factor;

	h -= floor(h);
	h *= 6;
	i = floor(h);
	f = h - i;
	if (!(i & 1)) f = 1 - f;  // if i is even
	m = v * (1 - s);
	n = v * (1 - s * f);

	switch (i) {
		case 6:
		case 0:
			point.setColor(v, n, m);
			break;
		case 1:
			point.setColor(n, v, m);
			break;
		case 2:
			point.setColor(m, v, n);
			break;
		case 3:
			point.setColor(m, n, v);
			break;
		case 4:
			point.setColor(n, m, v);
			break;
		case 5:
			point.setColor(v, m, n);
			break;
		default:
			point.setColor(1, 0.5, 0.5);
			break;
	}
}

void UFOMapDisplay::clear()
{
	std::lock_guard<std::mutex> lock(mutex_);

	map_.emplace<std::monostate>();

	for (VoxelType const& type : clouds_.keys()) {
		for (size_t i = 0; i < clouds_[type].size(); ++i) {
			clouds_[type][i].clear();
		}
	}

	// TODO: Implement
}

bool UFOMapDisplay::updateFromTF()
{
	Ogre::Vector3 position;
	Ogre::Quaternion orientation;
	if (context_->getFrameManager()->getTransform(header_, position, orientation)) {
		scene_node_->setOrientation(orientation);
		scene_node_->setPosition(position);
		return true;
	}
	return false;
}

bool UFOMapDisplay::createMap(ufomap_msgs::UFOMapMetaData const& info)
{
	// Fix floating point accuarcy problems
	double occupied_thres = occupied_thres_property_->getInt() / 100.0;
	double free_thres = free_thres_property_->getInt() / 100.0;

	// FIXME: Remove hardcoded
	if ("occupancy_map" == info.id) {
		map_.emplace<ufo::map::OccupancyMap>(info.resolution, info.depth_levels, true,
		                                     occupied_thres, free_thres);
		return true;
	} else if ("occupancy_map_color" == info.id) {
		map_.emplace<ufo::map::OccupancyMapColor>(info.resolution, info.depth_levels, true,
		                                          occupied_thres, free_thres);
		return true;
	}

	// TODO: Should remove VOXEL_COLOR option if none color UFOMap
	return false;
}

bool UFOMapDisplay::checkMap(std::string const& type, double resolution,
                             ufo::map::DepthType depth_levels) const
{
	return std::visit(
	    [this, &type, resolution, depth_levels](auto& map) -> bool {
		    if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>) {
			    return map.getTreeType() == type && map.getResolution() == resolution &&
			           map.getTreeDepthLevels() == depth_levels;
		    }
		    return false;
	    },
	    map_);
}

void UFOMapDisplay::setTopic(QString const& topic, QString const& datatype)
{
	topic_property_->setString(topic);
}

std::string UFOMapDisplay::getStrVoxelType(VoxelType const& type) const
{
	switch (type) {
		case OCCUPIED:
			return "Occupied";
		case FREE:
			return "Free";
		case UNKNOWN:
			return "Unknown";
		default:
			return "";
	}
}
}  // namespace ufomap_ros::rviz_plugins

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ufomap_ros::rviz_plugins::UFOMapDisplay, rviz::Display)