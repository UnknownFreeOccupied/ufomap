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
#include <ufomap_rviz_plugins/filter_display.h>

namespace ufomap_ros::rviz_plugins
{
FilterDisplay::FilterDisplay(rviz::Property* parent, rviz::FrameManager* frame_manager)
    : frame_manager_(frame_manager)
{
	// Filter
	filter_ = new rviz::Property("Filter", QVariant(), "Filters", parent);

	// Occupancy
	occupancy_ = new rviz::BoolProperty("Occupancy", true, "Filter on occupancy", filter_);
	occupancy_->setDisableChildrenIfFalse(true);
	unknown_ =
	    new rviz::BoolProperty("Unknown", false, "Visualize unknown nodes", occupancy_);
	free_ = new rviz::BoolProperty("Free", false, "Visualize free nodes", occupancy_);
	occupied_ =
	    new rviz::BoolProperty("Occupied", true, "Visualize occupied nodes", occupancy_);
	min_occupancy_ = new rviz::IntProperty("Min (%)", 0, "Min occupancy (%)", occupancy_);
	max_occupancy_ = new rviz::IntProperty("Max (%)", 100, "Max occupancy (%)", occupancy_);
	min_occupancy_->setMin(0);
	min_occupancy_->setMax(100);
	max_occupancy_->setMin(0);
	max_occupancy_->setMax(100);

	// Color
	color_ = new rviz::BoolProperty("Color", false, "Filter on color", filter_);
	color_->setDisableChildrenIfFalse(true);

	// Time
	time_ = new rviz::BoolProperty("Time", false, "Filter on time", filter_);
	time_->setDisableChildrenIfFalse(true);
	min_time_ = new rviz::FloatProperty("Min", std::numeric_limits<float>::lowest(),
	                                    "Min time", time_);
	max_time_ = new rviz::FloatProperty("Max", std::numeric_limits<float>::max(),
	                                    "Max time", time_);
	// min_time_->setMin(std::numeric_limits<float>::lowest());
	// min_time_->setMax(std::numeric_limits<float>::max());
	// max_time_->setMin(std::numeric_limits<float>::lowest());
	// max_time_->setMax(std::numeric_limits<float>::max());

	// Intensity
	intensity_ = new rviz::BoolProperty("Intensity", false, "Filter on intensity", filter_);
	intensity_->setDisableChildrenIfFalse(true);
	min_intensity_ = new rviz::FloatProperty("Min", std::numeric_limits<float>::lowest(),
	                                         "Min intensity", intensity_);
	max_intensity_ = new rviz::FloatProperty("Max", std::numeric_limits<float>::max(),
	                                         "Max intensity", intensity_);

	// Count
	count_ = new rviz::BoolProperty("Count", false, "Filter on count", filter_);
	count_->setDisableChildrenIfFalse(true);
	min_count_ = new rviz::IntProperty("Min", 0, "Min count", count_);
	max_count_ =
	    new rviz::IntProperty("Max", std::numeric_limits<int>::max(), "Max count", count_);
	min_count_->setMin(0);
	max_count_->setMin(0);

	// Reflection
	reflection_ =
	    new rviz::BoolProperty("Reflection", false, "Filter on reflection", filter_);
	reflection_->setDisableChildrenIfFalse(true);
	reflectiveness_ = new rviz::BoolProperty("Reflectiveness", false,
	                                         "Filter on reflectiveness", reflection_);
	reflectiveness_->setDisableChildrenIfFalse(true);
	min_reflectivness_ = new rviz::FloatProperty(
	    "Min", std::numeric_limits<float>::lowest(), "Min reflectiveness", reflectiveness_);
	max_reflectivness_ = new rviz::FloatProperty("Max", std::numeric_limits<float>::max(),
	                                             "Max reflectiveness", reflectiveness_);
	hits_ = new rviz::BoolProperty("Hits", false, "Filter on hits", reflection_);
	hits_->setDisableChildrenIfFalse(true);
	min_hits_ = new rviz::IntProperty("Min", 0, "Min hits", hits_);
	max_hits_ =
	    new rviz::IntProperty("Max", std::numeric_limits<int>::max(), "Max hits", hits_);
	min_hits_->setMin(0);
	max_hits_->setMin(0);
	misses_ = new rviz::BoolProperty("Misses", false, "Filter on misses", reflection_);
	misses_->setDisableChildrenIfFalse(true);
	min_misses_ = new rviz::IntProperty("Min", 0, "Min misses", misses_);
	max_misses_ = new rviz::IntProperty("Max", std::numeric_limits<int>::max(),
	                                    "Max misses", misses_);
	min_misses_->setMin(0);
	max_misses_->setMin(0);

	// Surfel
	surfel_ = new rviz::BoolProperty("Surfel", false, "Filter on surfel", filter_);
	surfel_->setDisableChildrenIfFalse(true);
	planarity_ = new rviz::BoolProperty("Planarity", false, "Filter on planarity", surfel_);
	planarity_->setDisableChildrenIfFalse(true);
	min_planarity_ = new rviz::FloatProperty("Min", std::numeric_limits<float>::lowest(),
	                                         "Min planarity", planarity_);
	max_planarity_ = new rviz::FloatProperty("Max", std::numeric_limits<float>::max(),
	                                         "Max planarity", planarity_);

	// Semantics
	semantics_ = new rviz::BoolProperty("Semantics", false, "Filter on semantics", filter_);
	semantics_->setDisableChildrenIfFalse(true);
	semantic_value_ =
	    new rviz::BoolProperty("Value", false, "Filter on semantic value", semantics_);
	semantic_value_->setDisableChildrenIfFalse(true);
	min_semantic_value_ =
	    new rviz::FloatProperty("Min value", std::numeric_limits<float>::lowest(),
	                            "Min semantic value", semantic_value_);
	max_semantic_value_ =
	    new rviz::FloatProperty("Max value", std::numeric_limits<float>::max(),
	                            "Max semantic value", semantic_value_);
	semantic_tag_ =
	    new rviz::BoolProperty("Tag", false, "Filter on semantic tag", semantics_);
	semantic_tag_->setDisableChildrenIfFalse(true);
	for (QString tag : {"car", "cup", "lamp"}) {
		semantic_tags_.push_back(
		    new rviz::BoolProperty(tag, true, "Filter on " + tag, semantic_tag_));
		semantic_tags_.back()->setDisableChildrenIfFalse(true);
		semantic_tag_set_color_[semantic_tags_.back()] =
		    new rviz::BoolProperty("Color", false, "Set color of tag", semantic_tags_.back());
		semantic_tag_set_color_[semantic_tags_.back()]->setDisableChildrenIfFalse(true);
		semantic_tag_colors_[semantic_tags_.back()] =
		    new rviz::ColorProperty("Color", Qt::black, "Color for " + tag,
		                            semantic_tag_set_color_[semantic_tags_.back()]);
	}
	semantic_label_ =
	    new rviz::BoolProperty("Label", false, "Filter on semantic label", semantics_);
	for (ufo::map::label_t label : {0, 1, 3, 4, 19, 200231}) {
		semantic_labels_.push_back(new rviz::BoolProperty(
		    QString::number(label), true, "Filter on label", semantic_label_));
	}

	// Bounding box
	bbx_ = new rviz::BoolProperty("BBX", false, "", filter_);
	bbx_->setDisableChildrenIfFalse(true);
	tf_bbx_ = new rviz::TfFrameProperty("Frame", rviz::TfFrameProperty::FIXED_FRAME_STRING,
	                                    "The frame to use for the BBX", bbx_, frame_manager,
	                                    true);
	min_bbx_ = new rviz::VectorProperty(
	    "Min", Ogre::Vector3(std::numeric_limits<Ogre::Real>::lowest()),
	    "Defines the min BBX coordinate", bbx_);

	max_bbx_ = new rviz::VectorProperty(
	    "Max", Ogre::Vector3(std::numeric_limits<Ogre::Real>::max()),
	    "Defines the max BBX coordinate", bbx_);
}

Filter FilterDisplay::getFilter() const
{
	Filter filter;

	// filter.filter_occupancy = filter_occupancy_->getBool();
	// filter.min_occupancy = min_occupancy_->getInt();
	// filter.max_occupancy = max_occupancy_->getInt();

	// filter.filter_time = filter_time_->getBool();
	// filter.min_time = min_time_->getInt();
	// filter.max_time = max_time_->getInt();

	// filter.filter_semantics = filter_semantics_->getBool();
	// // TODO: Add labels
	// filter.min_semantic_value = min_semantic_value_->getInt();
	// filter.max_semantic_value = max_semantic_value_->getInt();

	// // BBX

	// // Get transform
	// Ogre::Vector3 position;
	// Ogre::Quaternion orientation;
	// frame_manager_->getTransform(tf_bbx_->getFrameStd(), ros::Time(0), position,
	//                              orientation);

	// filter.filter_bounding_volume = filter_bbx_->getBool();
	// auto min = min_bbx_->getVector() + position;
	// auto max = max_bbx_->getVector() + position;
	// filter.bounding_volume = ufo::geometry::AABB(ufo::geometry::Point(min.x, min.y,
	// min.z),
	//                                              ufo::geometry::Point(max.x, max.y,
	//                                              max.z));

	return filter;
}
}  // namespace ufomap_ros::rviz_plugins