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

	// Color
	filter_color_ = new rviz::BoolProperty("Color", false, "Filter on color", parent);

	// Time step
	filter_time_step_ =
	    new rviz::BoolProperty("Semantic", false, "Filter on time step", parent);
	filter_time_step_->setDisableChildrenIfFalse(true);
	// min_time_step_ = new rviz::IntProperty("Min", 0, "Min time step", filter_time_step_);
	// max_time_step_ =
	//     new rviz::IntProperty("Max", std::pow(2, 24), "Max time step",
	//     filter_time_step_);
	// min_time_step_->setMin(0);
	// min_time_step_->setMax(std::pow(2, 24));
	// max_time_step_->setMin(0);
	// max_time_step_->setMax(std::pow(2, 24));

	unlabeled_ = new rviz::BoolProperty("Unlabeled", true, "", filter_time_step_);
	outlier_ = new rviz::BoolProperty("Outlier", true, "", filter_time_step_);
	car_ = new rviz::BoolProperty("Car", true, "", filter_time_step_);
	bicycle_ = new rviz::BoolProperty("Bicycle", true, "", filter_time_step_);
	bus_ = new rviz::BoolProperty("Bus", true, "", filter_time_step_);
	motorcycle_ = new rviz::BoolProperty("Motorcycle", true, "", filter_time_step_);
	on_rails_ = new rviz::BoolProperty("On rails", true, "", filter_time_step_);
	truck_ = new rviz::BoolProperty("Truck", true, "", filter_time_step_);
	other_vehicle_ = new rviz::BoolProperty("Other vehicle", true, "", filter_time_step_);
	person_ = new rviz::BoolProperty("Person", true, "", filter_time_step_);
	bicyclist_ = new rviz::BoolProperty("Bicyclist", true, "", filter_time_step_);
	motorcyclist_ = new rviz::BoolProperty("Motorcyclist", true, "", filter_time_step_);
	road_ = new rviz::BoolProperty("Road", true, "", filter_time_step_);
	parking_ = new rviz::BoolProperty("Parking", true, "", filter_time_step_);
	sidewalk_ = new rviz::BoolProperty("Sidewalk", true, "", filter_time_step_);
	other_ground_ = new rviz::BoolProperty("Other ground", true, "", filter_time_step_);
	building_ = new rviz::BoolProperty("Building", true, "", filter_time_step_);
	fence_ = new rviz::BoolProperty("Fence", true, "", filter_time_step_);
	other_structure_ =
	    new rviz::BoolProperty("Other structure", true, "", filter_time_step_);
	lane_marking_ = new rviz::BoolProperty("Lane marking", true, "", filter_time_step_);
	vegetation_ = new rviz::BoolProperty("Vegetation", true, "", filter_time_step_);
	trunk_ = new rviz::BoolProperty("Trunk", true, "", filter_time_step_);
	terrain_ = new rviz::BoolProperty("Terrain", true, "", filter_time_step_);
	pole_ = new rviz::BoolProperty("Pole", true, "", filter_time_step_);
	traffic_sign_ = new rviz::BoolProperty("Traffic sign", true, "", filter_time_step_);
	other_object_ = new rviz::BoolProperty("Other object", true, "", filter_time_step_);
	moving_car_ = new rviz::BoolProperty("Moving car", true, "", filter_time_step_);
	moving_bicyclist_ =
	    new rviz::BoolProperty("Moving bicyclist", true, "", filter_time_step_);
	moving_person_ = new rviz::BoolProperty("Moving person", true, "", filter_time_step_);
	moving_motorcyclist_ =
	    new rviz::BoolProperty("Moving motorcyclist", true, "", filter_time_step_);
	moving_on_rails_ =
	    new rviz::BoolProperty("Moving on rails", true, "", filter_time_step_);
	moving_bus_ = new rviz::BoolProperty("Moving bus", true, "", filter_time_step_);
	moving_truck_ = new rviz::BoolProperty("Moving truck", true, "", filter_time_step_);
	moving_other_vehicle_ =
	    new rviz::BoolProperty("Moving other vehicle", true, "", filter_time_step_);

	// noise_ = new rviz::BoolProperty("Noise", true, "", filter_time_step_);
	// animal_ = new rviz::BoolProperty("Animal", true, "", filter_time_step_);
	// human_pedestrian_adult_ =
	//     new rviz::BoolProperty("Pedestrian adult", true, "", filter_time_step_);
	// human_pedestrian_child_ =
	//     new rviz::BoolProperty("Pedestrian child", true, "", filter_time_step_);
	// human_pedestrian_construction_worker_ = new rviz::BoolProperty(
	//     "Pedestrian construction worker", true, "", filter_time_step_);
	// human_pedestrian_personal_mobility_ =
	//     new rviz::BoolProperty("Pedestrian personal mobility", true, "",
	//     filter_time_step_);
	// human_pedestrian_police_officer_ =
	//     new rviz::BoolProperty("Pedestrian police officer", true, "", filter_time_step_);
	// human_pedestrian_stroller_ =
	//     new rviz::BoolProperty("Pedestrian stroller", true, "", filter_time_step_);
	// human_pedestrian_wheelchair_ =
	//     new rviz::BoolProperty("Pedestrian wheelchair", true, "", filter_time_step_);
	// movable_object_barrier_ =
	//     new rviz::BoolProperty("Barrier", true, "", filter_time_step_);
	// movable_object_debris_ = new rviz::BoolProperty("Debris", true, "",
	// filter_time_step_); movable_object_pushable_pullable_ =
	//     new rviz::BoolProperty("Pushable pullable", true, "", filter_time_step_);
	// movable_object_trafficcone_ =
	//     new rviz::BoolProperty("Traffic cone", true, "", filter_time_step_);
	// static_object_bicycle_rack_ =
	//     new rviz::BoolProperty("Bicycle rack", true, "", filter_time_step_);
	// vehicle_bicycle_ = new rviz::BoolProperty("Bicycle", true, "", filter_time_step_);
	// vehicle_bus_bendy_ = new rviz::BoolProperty("Bus bendy", true, "",
	// filter_time_step_); vehicle_bus_rigid_ = new rviz::BoolProperty("Bus rigid", true,
	// "", filter_time_step_); vehicle_car_ = new rviz::BoolProperty("Car", true, "",
	// filter_time_step_); vehicle_construction_ =
	//     new rviz::BoolProperty("Construction", true, "", filter_time_step_);
	// vehicle_emergencyambulance_ =
	//     new rviz::BoolProperty("Ambulance", true, "", filter_time_step_);
	// vehicle_emergencypolice_ =
	//     new rviz::BoolProperty("Police car", true, "", filter_time_step_);
	// vehicle_motorcycle_ = new rviz::BoolProperty("Motorcycle", true, "",
	// filter_time_step_); vehicle_trailer_ = new rviz::BoolProperty("Trailer", true, "",
	// filter_time_step_); vehicle_truck_ = new rviz::BoolProperty("Truck", true, "",
	// filter_time_step_); flat_driveable_surface_ =
	//     new rviz::BoolProperty("Driveable surface", true, "", filter_time_step_);
	// flat_other_ = new rviz::BoolProperty("Flat other", true, "", filter_time_step_);
	// flat_sidewalk_ = new rviz::BoolProperty("Sidewalk", true, "", filter_time_step_);
	// flat_terrain_ = new rviz::BoolProperty("Terrain", true, "", filter_time_step_);
	// static_manmade_ = new rviz::BoolProperty("Manmade", true, "", filter_time_step_);
	// static_other_ = new rviz::BoolProperty("Static other", true, "", filter_time_step_);
	// static_vegetation_ = new rviz::BoolProperty("Vegetation", true, "",
	// filter_time_step_); vehicle_ego_ = new rviz::BoolProperty("Ego vechicle", true, "",
	// filter_time_step_);

	// Semantics
	filter_semantics_ =
	    new rviz::BoolProperty("Semantics", false, "Filter on semantics", parent);
	filter_semantics_->setDisableChildrenIfFalse(true);
	filter_semantics_value_ = new rviz::BoolProperty(
	    "Value", false, "Filter on semantic value", filter_semantics_);
	min_semantic_value_ =
	    new rviz::FloatProperty("Min", std::numeric_limits<float>::lowest(),
	                            "Min semantic value", filter_semantics_value_);
	max_semantic_value_ =
	    new rviz::FloatProperty("Max", std::numeric_limits<float>::max(),
	                            "Max semantic value", filter_semantics_value_);
	filter_semantics_label_ = new rviz::BoolProperty(
	    "Label", false, "Filter on semantic label", filter_semantics_);
	semantic_labels_.push_back(
	    new rviz::BoolProperty("Unlabeled", false, "", filter_semantics_label_));
	semantic_labels_.push_back(
	    new rviz::BoolProperty("0", false, "", semantic_labels_.back()));
	semantic_labels_.push_back(
	    new rviz::BoolProperty("Outlier", false, "", filter_semantics_label_));
	semantic_labels_.push_back(
	    new rviz::BoolProperty("1", false, "", semantic_labels_.back()));

	semantic_labels_.push_back(
	    new rviz::BoolProperty("GT", false, "Grount truth", filter_semantics_label_));
	auto gt = semantic_labels_.back();
	semantic_labels_.push_back(
	    new rviz::BoolProperty("Pred", false, "Grount truth", filter_semantics_label_));
	auto pred = semantic_labels_.back();

	semantic_labels_.push_back(new rviz::BoolProperty("Vehicle", false, "", gt));
	auto gt_vehicle = semantic_labels_.back();
	semantic_labels_.push_back(new rviz::BoolProperty("Vehicle", false, "", pred));
	auto pred_vehicle = semantic_labels_.back();

	for (auto [name, label] :
	     {std::pair("Car", 10), std::pair("Bicycle", 11), std::pair("Bus", 13),
	      std::pair("Motorcycle", 15), std::pair("On rails", 16), std::pair("Truck", 18),
	      std::pair("Other vehicle", 20)}) {
		semantic_labels_.push_back(new rviz::BoolProperty(name, false, "", gt_vehicle));
		semantic_labels_.push_back(
		    new rviz::BoolProperty(QString(label), false, "", semantic_labels_.back()));
		semantic_labels_.push_back(new rviz::BoolProperty(name, false, "", pred_vehicle));
		semantic_labels_.push_back(
		    new rviz::BoolProperty(QString(label * 100), false, "", semantic_labels_.back()));
	}

	semantic_labels_.push_back(new rviz::BoolProperty("Car", false, "", gt_vehicle));
	semantic_labels_.push_back(
	    new rviz::BoolProperty("10", false, "", semantic_labels_.back()));

	for (auto [name, label] : {std::pair("Unlabeled", 0),
	                           std::pair("Outlier", 1),
	                           std::pair("Car", 10),
	                           std::pair("Bicycle", 11),
	                           std::pair("Bus", 13),
	                           std::pair("Motorcycle", 15),
	                           std::pair("On rails", 16),
	                           std::pair("Truck", 18),
	                           std::pair("Other vehicle", 20),
	                           std::pair("Person", 30),
	                           std::pair("Bicyclist", 31),
	                           std::pair("Motorcyclist", 32),
	                           std::pair("Road", 40),
	                           std::pair("Parking", 44),
	                           std::pair("Sidewalk", 48),
	                           std::pair("Other ground", 49),
	                           std::pair("Building", 50),
	                           std::pair("Fence", 51),
	                           std::pair("Other structure", 52),
	                           std::pair("Lane marking", 60),
	                           std::pair("Vegetation", 70),
	                           std::pair("Trunk", 71),
	                           std::pair("Terrain", 72),
	                           std::pair("Pole", 80),
	                           std::pair("Traffic sign", 81),
	                           std::pair("Other object", 99),
	                           std::pair("Moving car", 252),
	                           std::pair("Moving bicyclist", 253),
	                           std::pair("", 254),
	                           std::pair("", 255),
	                           std::pair("", 256),
	                           std::pair("", 257),
	                           std::pair("", 258),
	                           std::pair("", 259)}) {
	}
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

	filter.filter_color = filter_color_->getBool();

	filter.filter_time_step = filter_time_step_->getBool();
	filter.min_time_step = 0;       // min_time_step_->getInt();
	filter.max_time_step = 100000;  // max_time_step_->getInt();

	filter.unlabeled = unlabeled_->getBool();
	filter.outlier = outlier_->getBool();
	filter.car = car_->getBool();
	filter.bicycle = bicycle_->getBool();
	filter.bus = bus_->getBool();
	filter.motorcycle = motorcycle_->getBool();
	filter.on_rails = on_rails_->getBool();
	filter.truck = truck_->getBool();
	filter.other_vehicle = other_vehicle_->getBool();
	filter.person = person_->getBool();
	filter.bicyclist = bicyclist_->getBool();
	filter.motorcyclist = motorcyclist_->getBool();
	filter.road = road_->getBool();
	filter.parking = parking_->getBool();
	filter.sidewalk = sidewalk_->getBool();
	filter.other_ground = other_ground_->getBool();
	filter.building = building_->getBool();
	filter.fence = fence_->getBool();
	filter.other_structure = other_structure_->getBool();
	filter.lane_marking = lane_marking_->getBool();
	filter.vegetation = vegetation_->getBool();
	filter.trunk = trunk_->getBool();
	filter.terrain = terrain_->getBool();
	filter.pole = pole_->getBool();
	filter.traffic_sign = traffic_sign_->getBool();
	filter.other_object = other_object_->getBool();
	filter.moving_car = moving_car_->getBool();
	filter.moving_bicyclist = moving_bicyclist_->getBool();
	filter.moving_person = moving_person_->getBool();
	filter.moving_motorcyclist = moving_motorcyclist_->getBool();
	filter.moving_on_rails = moving_on_rails_->getBool();
	filter.moving_bus = moving_bus_->getBool();
	filter.moving_truck = moving_truck_->getBool();
	filter.moving_other_vehicle = moving_other_vehicle_->getBool();

	// filter.noise = noise_->getBool();
	// filter.animal = animal_->getBool();
	// filter.human_pedestrian_adult = human_pedestrian_adult_->getBool();
	// filter.human_pedestrian_child = human_pedestrian_child_->getBool();
	// filter.human_pedestrian_construction_worker =
	//     human_pedestrian_construction_worker_->getBool();
	// filter.human_pedestrian_personal_mobility =
	//     human_pedestrian_personal_mobility_->getBool();
	// filter.human_pedestrian_police_officer = human_pedestrian_police_officer_->getBool();
	// filter.human_pedestrian_stroller = human_pedestrian_stroller_->getBool();
	// filter.human_pedestrian_wheelchair = human_pedestrian_wheelchair_->getBool();
	// filter.movable_object_barrier = movable_object_barrier_->getBool();
	// filter.movable_object_debris = movable_object_debris_->getBool();
	// filter.movable_object_pushable_pullable =
	// movable_object_pushable_pullable_->getBool(); filter.movable_object_trafficcone =
	// movable_object_trafficcone_->getBool(); filter.static_object_bicycle_rack =
	// static_object_bicycle_rack_->getBool(); filter.vehicle_bicycle =
	// vehicle_bicycle_->getBool(); filter.vehicle_bus_bendy =
	// vehicle_bus_bendy_->getBool(); filter.vehicle_bus_rigid =
	// vehicle_bus_rigid_->getBool(); filter.vehicle_car = vehicle_car_->getBool();
	// filter.vehicle_construction = vehicle_construction_->getBool();
	// filter.vehicle_emergencyambulance = vehicle_emergencyambulance_->getBool();
	// filter.vehicle_emergencypolice = vehicle_emergencypolice_->getBool();
	// filter.vehicle_motorcycle = vehicle_motorcycle_->getBool();
	// filter.vehicle_trailer = vehicle_trailer_->getBool();
	// filter.vehicle_truck = vehicle_truck_->getBool();
	// filter.flat_driveable_surface = flat_driveable_surface_->getBool();
	// filter.flat_other = flat_other_->getBool();
	// filter.flat_sidewalk = flat_sidewalk_->getBool();
	// filter.flat_terrain = flat_terrain_->getBool();
	// filter.static_manmade = static_manmade_->getBool();
	// filter.static_other = static_other_->getBool();
	// filter.static_vegetation = static_vegetation_->getBool();
	// filter.vehicle_ego = vehicle_ego_->getBool();

	filter.filter_semantics = filter_semantics_->getBool();
	// TODO: Add labels
	filter.min_semantic_value = min_semantic_value_->getFloat();
	filter.max_semantic_value = max_semantic_value_->getFloat();

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