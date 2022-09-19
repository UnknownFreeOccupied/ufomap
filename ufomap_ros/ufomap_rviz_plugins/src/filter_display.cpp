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
	// Occupancy
	filter_occupancy_ =
	    new rviz::BoolProperty("Occupancy", false, "Filter on occupancy value", parent);
	filter_occupancy_->setDisableChildrenIfFalse(true);
	min_occupancy_ =
	    new rviz::IntProperty("Min (%)", 0, "Min occupancy (%)", filter_occupancy_);
	max_occupancy_ =
	    new rviz::IntProperty("Max (%)", 100, "Max occupancy (%)", filter_occupancy_);
	min_occupancy_->setMin(0);
	min_occupancy_->setMax(100);
	max_occupancy_->setMin(0);
	max_occupancy_->setMax(100);

	// Time step
	filter_time_ =
	    new rviz::BoolProperty("Time step", false, "Filter on time step", parent);
	filter_time_->setDisableChildrenIfFalse(true);
	min_time_ = new rviz::IntProperty("Min", 0, "Min time step", filter_time_);
	max_time_ =
	    new rviz::IntProperty("Max", std::pow(2, 24), "Max time step", filter_time_);
	min_time_->setMin(0);
	min_time_->setMax(std::pow(2, 24));
	max_time_->setMin(0);
	max_time_->setMax(std::pow(2, 24));

	// Semantics
	filter_semantics_ =
	    new rviz::BoolProperty("Semantics", false, "Filter on semantics", parent);
	filter_semantics_->setDisableChildrenIfFalse(true);
	min_semantic_value_ =
	    new rviz::IntProperty("Min value", 0, "Min semantic value", filter_semantics_);
	max_semantic_value_ =
	    new rviz::IntProperty("Max value", std::numeric_limits<int>::max(),
	                          "Max semantic value", filter_semantics_);
	min_semantic_value_->setMin(0);
	min_semantic_value_->setMax(std::numeric_limits<int>::max());
	max_semantic_value_->setMin(0);
	max_semantic_value_->setMax(std::numeric_limits<int>::max());
	// TODO: Add labels

	// Bounding box
	filter_bbx_ = new rviz::BoolProperty("BBX", false, "", parent);
	filter_bbx_->setDisableChildrenIfFalse(true);
	tf_bbx_ = new rviz::TfFrameProperty("Frame", rviz::TfFrameProperty::FIXED_FRAME_STRING,
	                                    "The frame to use for the BBX", filter_bbx_,
	                                    frame_manager, true);
	min_bbx_ = new rviz::VectorProperty(
	    "Min", Ogre::Vector3(std::numeric_limits<Ogre::Real>::lowest()),
	    "Defines the min BBX coordinate", filter_bbx_);

	max_bbx_ = new rviz::VectorProperty(
	    "Max", Ogre::Vector3(std::numeric_limits<Ogre::Real>::max()),
	    "Defines the max BBX coordinate", filter_bbx_);
}

Filter FilterDisplay::getFilter() const
{
	Filter filter;

	filter.filter_occupancy = filter_occupancy_->getBool();
	filter.min_occupancy = min_occupancy_->getInt();
	filter.max_occupancy = max_occupancy_->getInt();

	filter.filter_time = filter_time_->getBool();
	filter.min_time = min_time_->getInt();
	filter.max_time = max_time_->getInt();

	filter.filter_semantics = filter_semantics_->getBool();
	// TODO: Add labels
	filter.min_semantic_value = min_semantic_value_->getInt();
	filter.max_semantic_value = max_semantic_value_->getInt();

	// BBX

	// Get transform
	Ogre::Vector3 position;
	Ogre::Quaternion orientation;
	frame_manager_->getTransform(tf_bbx_->getFrameStd(), ros::Time(0), position,
	                             orientation);

	filter.filter_bounding_volume = filter_bbx_->getBool();
	auto min = min_bbx_->getVector() + position;
	auto max = max_bbx_->getVector() + position;
	filter.bounding_volume = ufo::geometry::AABB(ufo::geometry::Point(min.x, min.y, min.z),
	                                             ufo::geometry::Point(max.x, max.y, max.z));

	return filter;
}
}  // namespace ufomap_ros::rviz_plugins