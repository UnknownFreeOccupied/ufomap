/*
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * License: BSD 3
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

#ifndef UFOMAP_GAZEBO_PLUGIN_H
#define UFOMAP_GAZEBO_PLUGIN_H

// UFO
#include <ufo/geometry/aabb.h>
#include <ufo/map/occupancy_map.h>
#include <ufo/map/occupancy_map_color.h>

// UFO ROS
#include <ufomap_gazebo_plugins/GetMap.h>

// ROS
#include <ros/ros.h>
#include <std_srvs/Empty.h>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <sdf/sdf.hh>

// STL
#include <math.h>

#include <iostream>

namespace ufomap_gazebo_plugins
{
class UFOMapFromGazebo : public gazebo::WorldPlugin
{
 public:
	UFOMapFromGazebo();

	virtual ~UFOMapFromGazebo();

 protected:
	void Load(gazebo::physics::WorldPtr _parent, sdf::ElementPtr _sdf);

	bool getMapCallback(GetMap::Request& req, GetMap::Response& res);

	void createMap(ufo::map::OccupancyMapColor& map, ufo::geometry::AABB const& bv,
	               bool as_unknown, unsigned int depth);

	void createMap2(ufo::map::OccupancyMapColor& map, ufo::geometry::AABB const& bv,
	                bool as_unknown, unsigned int depth);

	void getHits(std::vector<ignition::math::Vector3d>& hits,
	             gazebo::physics::RayShapePtr ray, ignition::math::Vector3d from,
	             ignition::math::Vector3d const& to, double resolution);

	bool checkIfInterest(ignition::math::Vector3d const& central_point,
	                     gazebo::physics::RayShapePtr ray, double resolution);

 private:
	// Gazebo
	gazebo::physics::WorldPtr world_;

	// Node handles
	ros::NodeHandle nh_;
	ros::NodeHandle nh_priv_;

	// Service
	ros::ServiceServer get_map_srv_;

	// Publisher
	ros::Publisher pub_;
};

}  // namespace ufomap_gazebo_plugins

#endif  // UFOMAP_GAZEBO_PLUGIN_H
