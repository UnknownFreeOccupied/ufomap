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

#ifndef UFO_MAP_MAPPING_SERVER_H
#define UFO_MAP_MAPPING_SERVER_H

// UFO
#include <ufo/map/ufomap.h>
#include <ufo/util/timing.h>
#include <ufomap_mapping/ServerConfig.h>
#include <ufomap_srvs/ClearVolume.h>
#include <ufomap_srvs/GetMap.h>
#include <ufomap_srvs/Reset.h>
#include <ufomap_srvs/SaveMap.h>

// ROS
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

// STL
#include <future>
#include <variant>
#include <vector>

namespace ufomap_mapping
{
class Server
{
 public:
	Server(ros::NodeHandle &nh, ros::NodeHandle &nh_priv);

 private:
	void cloudCallback(sensor_msgs::PointCloud2::ConstPtr const &msg);

	void printInfo() const;

	void writeComputationTimeToFile();

	void mapConnectCallback(ros::SingleSubscriberPublisher const &pub, int depth);

	bool getMapCallback(ufomap_srvs::GetMap::Request &request,
	                    ufomap_srvs::GetMap::Response &response);

	bool clearVolumeCallback(ufomap_srvs::ClearVolume::Request &request,
	                         ufomap_srvs::ClearVolume::Response &response);

	bool resetCallback(ufomap_srvs::Reset::Request &request,
	                   ufomap_srvs::Reset::Response &response);

	bool saveMapCallback(ufomap_srvs::SaveMap::Request &request,
	                     ufomap_srvs::SaveMap::Response &response);

	void timerCallback(ros::TimerEvent const &event);

	void configCallback(ufomap_mapping::ServerConfig &config, uint32_t level);

 private:
	//
	// ROS parameters
	//

	// Node handles
	ros::NodeHandle &nh_;
	ros::NodeHandle &nh_priv_;

	// Subscribers
	ros::Subscriber cloud_sub_;
	unsigned int cloud_in_queue_size_;

	// Publishers
	std::vector<ros::Publisher> map_pub_;
	unsigned int map_queue_size_;
	ros::Timer pub_timer_;
	double pub_rate_;
	ros::Duration update_rate_;
	ros::Time last_update_time_;

	// Services
	ros::ServiceServer get_map_server_;
	ros::ServiceServer clear_volume_server_;
	ros::ServiceServer reset_server_;
	ros::ServiceServer save_map_server_;

	// TF2
	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_;
	ros::Duration transform_timeout_;

	// Dynamic reconfigure
	dynamic_reconfigure::Server<ufomap_mapping::ServerConfig> cs_;

	//
	// UFO Parameters
	//

	// Map
	ufo::map::UFOMap map_;
	std::string frame_id_;

	// Integration
	ufo::map::Integrator integrator_;
	bool simple_ray_casting_;
	bool async_;

	// Clear robot
	bool clear_robot_;
	std::string robot_frame_id_;
	double robot_height_;
	double robot_radius_;
	int clearing_depth_;

	// Publishing
	bool compress_;
	bool update_part_of_map_;
	ufo::map::depth_t publish_depth_;
	std::future<void> update_async_handler_;

	// Timing information
	ufo::util::Timing timing_;
	std::ofstream output_file_;

	// Verbose
	bool verbose_;
};
}  // namespace ufomap_mapping

#endif  // UFO_MAP_MAPPING_SERVER_H