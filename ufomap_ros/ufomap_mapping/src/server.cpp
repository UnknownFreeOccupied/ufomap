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
#include <ufomap_mapping/server.h>
#include <ufomap_msgs/UFOMapStamped.h>
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>

// STL
#include <chrono>
#include <future>
#include <numeric>

namespace ufomap_mapping
{
// Colors
#define RESET "\033[0m"
#define BLACK "\033[30m"              /* Black */
#define RED "\033[31m"                /* Red */
#define GREEN "\033[32m"              /* Green */
#define YELLOW "\033[33m"             /* Yellow */
#define BLUE "\033[34m"               /* Blue */
#define MAGENTA "\033[35m"            /* Magenta */
#define CYAN "\033[36m"               /* Cyan */
#define WHITE "\033[37m"              /* White */
#define BOLDBLACK "\033[1m\033[30m"   /* Bold Black */
#define BOLDRED "\033[1m\033[31m"     /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m"   /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m"    /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m"    /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m"   /* Bold White */

struct separate_thousands : std::numpunct<char> {
	char_type do_thousands_sep() const override { return ' '; }  // separate with commas
	string_type do_grouping() const override { return "\3"; }    // groups of 3 digit
};

Server::Server(ros::NodeHandle &nh, ros::NodeHandle &nh_priv)
    : nh_(nh), nh_priv_(nh_priv), tf_listener_(tf_buffer_), cs_(nh_priv)
{
	// Set up map
	double resolution = nh_priv_.param("resolution", 0.05);
	ufo::map::depth_t depth_levels = nh_priv_.param("depth_levels", 16);
	uint32_t map_type_index = 0;
	map_type_index |= (nh_priv_.param("time_map", false) ? 1U : 0U) << 0;
	map_type_index |= (nh_priv_.param("color_map", false) ? 1U : 0U) << 1;
	map_type_index |= (nh_priv_.param("semantic_map", false) ? 1U : 0U) << 2;
	map_type_index |= (nh_priv_.param("small_map", false) ? 1U : 0U) << 3;

	// Automatic pruning is disabled so we can work in multiple threads for subscribers,
	// services and publishers
	map_ = ufo::map::createMap(map_type_index + 1, resolution, depth_levels, false);

	// Set up dynamic reconfigure server
	cs_.setCallback(boost::bind(&Server::configCallback, this, _1, _2));

	// Enable services
	get_map_server_ = nh_priv_.advertiseService("get_map", &Server::getMapCallback, this);
	clear_volume_server_ =
	    nh_priv_.advertiseService("clear_volume", &Server::clearVolumeCallback, this);
	reset_server_ = nh_priv_.advertiseService("reset", &Server::resetCallback, this);
	save_map_server_ =
	    nh_priv_.advertiseService("save_map", &Server::saveMapCallback, this);
}

void Server::cloudCallback(sensor_msgs::PointCloud2::ConstPtr const &msg)
{
	ufo::math::Pose6 transform;
	try {
		transform =
		    ufomap_ros::rosToUfo(tf_buffer_
		                             .lookupTransform(frame_id_, msg->header.frame_id,
		                                              msg->header.stamp, transform_timeout_)
		                             .transform);
	} catch (tf2::TransformException &ex) {
		ROS_WARN_THROTTLE(1, "%s", ex.what());
		return;
	}

	std::visit(
	    [this, &msg, &transform](auto &map) {
		    using T = std::decay_t<decltype(map)>;
		    if constexpr (!std::is_same_v<std::monostate, T>) {
			    // Update map
			    timing_.start("Integrate");
			    ufo::map::PointCloudColor cloud;
			    ufomap_ros::rosToUfo(*msg, cloud);

			    cloud.transform(transform);
			    integrator_.insertPointCloud(map, transform.translation(), cloud, true, false);
			    timing_.stop("Integrate");

			    // TODO: Clear robot
			    // if (clear_robot_) {
			    //   start = std::chrono::steady_clock::now();

			    //   try {
			    //     transform = ufomap_ros::rosToUfo(
			    //         tf_buffer_
			    //             .lookupTransform(frame_id_, robot_frame_id_, msg->header.stamp,
			    //                              transform_timeout_)
			    //             .transform);
			    //   } catch (tf2::TransformException &ex) {
			    //     ROS_WARN_THROTTLE(1, "%s", ex.what());
			    //     return;
			    //   }

			    //   ufo::map::Point3 r(robot_radius_, robot_radius_, robot_height_ / 2.0);
			    //   ufo::geometry::AABB aabb(transform.translation() - r,
			    //                            transform.translation() + r);
			    //   map.setValueVolume(aabb, map.getClampingThresMin(), clearing_depth_);

			    //   double clear_time =
			    //       std::chrono::duration<float, std::chrono::seconds::period>(
			    //           std::chrono::steady_clock::now() - start)
			    //           .count();
			    //   if (0 == num_clears_ || clear_time < min_clear_time_) {
			    //     min_clear_time_ = clear_time;
			    //   }
			    //   if (clear_time > max_clear_time_) {
			    //     max_clear_time_ = clear_time;
			    //   }
			    //   accumulated_clear_time_ += clear_time;
			    //   ++num_clears_;
			    // }

			    // Publish update
			    if (!map_pub_.empty() && update_part_of_map_ &&
			        (!last_update_time_.isValid() ||
			         (msg->header.stamp - last_update_time_) >= update_rate_)) {
				    last_update_time_ = msg->header.stamp;

				    timing_.start("Publish");
				    for (size_t i = 0; i != map_pub_.size(); ++i) {
					    if (map_pub_[i] &&
					        (0 < map_pub_[i].getNumSubscribers() || map_pub_[i].isLatched())) {
						    if (0 != i) {
							    map.updateModifiedNodes(i);
						    }

						    ufomap_msgs::UFOMapStamped::Ptr update_msg(
						        new ufomap_msgs::UFOMapStamped);
						    auto pred = ufo::map::predicate::UpdatedNode();
						    if (ufomap_msgs::ufoToMsg(map, update_msg->map, pred, i, compress_)) {
							    update_msg->header.stamp = msg->header.stamp;
							    update_msg->header.frame_id = frame_id_;
							    map_pub_[i].publish(update_msg);
						    }
					    }
				    }
				    map.updateModifiedNodes();
				    timing_.stop("Publish");
			    }

			    if (verbose_) {
				    printInfo();
			    }

			    writeComputationTimeToFile();
		    }
	    },
	    map_);
}

void Server::printInfo() const
{
	printf("Timings\n");
	printf(
	    "\t    Component               %sCurrent    %sMean     %sStD      %sMin      "
	    "%sMax%s\n",
	    BLUE, MAGENTA, CYAN, GREEN, RED, RESET);
	for (std::string const &component_name : {"Integrate", "Clear Robot", "Publish"}) {
		if (timing_.contains(component_name)) {
			printf("\t%-20s (ms)   %s%6.2f   %s%6.2f   %s%6.2f   %s%6.2f   %s%6.2f%s\n",
			       component_name.c_str(), BLUE, timing_.getLastMilliseconds(component_name),
			       MAGENTA, timing_.getMeanMilliseconds(component_name), CYAN,
			       timing_.getStdMilliseconds(component_name), GREEN,
			       timing_.minMilliseconds(component_name), RED,
			       timing_.maxMilliseconds(component_name), RESET);
		}
	}
	std::visit(
	    [](auto &map) {
		    using T = std::decay_t<decltype(map)>;
		    if constexpr (!std::is_same_v<std::monostate, T>) {
			    double memory = map.memoryUsage();
			    std::string memory_unit = "B";
			    if (std::pow(1024, 3) < memory) {
				    memory /= std::pow(1024, 3);
				    memory_unit = "GiB";
			    } else if (std::pow(1024, 2) < memory) {
				    memory /= std::pow(1024, 2);
				    memory_unit = "MiB";
			    } else if (1024 < memory) {
				    memory /= 1024;
				    memory_unit = "KiB";
			    }
			    printf("Memory\n");
			    printf("\tSize:        %6.2f %s\n", memory, memory_unit.c_str());
			    auto leaf_thousands =
			        std::unique_ptr<separate_thousands>(new separate_thousands());
			    std::stringstream leaf_ss;
			    leaf_ss.imbue(std::locale(std::locale(), leaf_thousands.release()));
			    leaf_ss << map.numLeafNodes();
			    printf("\tNum. leaf:   %s\n", leaf_ss.str().c_str());
			    auto inner_thousands =
			        std::unique_ptr<separate_thousands>(new separate_thousands());
			    std::stringstream inner_ss;
			    inner_ss.imbue(std::locale(std::locale(), inner_thousands.release()));
			    inner_ss << map.numInnerNodes();
			    printf("\tNum. inner:  %s\n", inner_ss.str().c_str());
			    printf("%s", RESET);
		    }
	    },
	    map_);
}

void Server::writeComputationTimeToFile()
{
	// TODO: Implement
	// if (output_file_.is_open()) {
	// 	output_file_ << timing_.getTotalWithCurrentSeconds("Total");
	// 	output_file_ << "\t" << explored_volume_;
	// 	output_file_ << "\t" << 100.0 * (explored_volume_ / explorable_volume_);
	// 	output_file_ << "\t" << 100.0 * (1.0 - (explored_volume_ / explorable_volume_));
	// 	for (std::string const &component_name : {"Integrate", "Clear Robot", "Publish"}) {
	// 		if (timing_.contains(component_name)) {
	// 			output_file_ << "\t" << timing_.getLastMilliseconds(component_name);
	// 		} else {
	// 			output_file_ << "\t" << NAN;
	// 		}
	// 	}
	// 	// output_file_ << "\t" << velocity_.norm() << "\t" << yaw_rate_;
	// 	output_file_ << "\n";
	// }
}

void Server::mapConnectCallback(ros::SingleSubscriberPublisher const &pub, int depth)
{
	// When a new node subscribes we will publish the whole map to that node.

	// TODO: Make this async

	std::visit(
	    [this, &pub, depth](auto &map) {
		    using T = std::decay_t<decltype(map)>;
		    if constexpr (!std::is_same_v<std::monostate, T>) {
			    timing_.start("Full Publish");
			    ufomap_msgs::UFOMapStamped::Ptr msg(new ufomap_msgs::UFOMapStamped);
			    if (ufomap_msgs::ufoToMsg(map, msg->map, compress_, depth)) {
				    msg->header.stamp = ros::Time::now();
				    msg->header.frame_id = frame_id_;
				    pub.publish(msg);
			    }
			    timing_.stop("Full Publish");
		    }
	    },
	    map_);
}

bool Server::getMapCallback(ufomap_srvs::GetMap::Request &request,
                            ufomap_srvs::GetMap::Response &response)
{
	std::visit(
	    [this, &request, &response](auto &map) {
		    using T = std::decay_t<decltype(map)>;
		    if constexpr (!std::is_same_v<std::monostate, T>) {
			    ufo::geometry::BoundingVolume bv =
			        ufomap_msgs::msgToUfo(request.bounding_volume);
			    auto pred = ufo::map::predicate::Intersects(bv);
			    response.success = ufomap_msgs::ufoToMsg(map, response.map, pred, request.depth,
			                                             request.compress);
		    } else {
			    response.success = false;
		    }
	    },
	    map_);
	return true;
}

bool Server::clearVolumeCallback(ufomap_srvs::ClearVolume::Request &request,
                                 ufomap_srvs::ClearVolume::Response &response)
{
	// TODO: Implement
	// std::visit(
	//     [this, &request, &response](auto &map) {
	// 	    if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>) {
	// 		    ufo::geometry::BoundingVolume bv =
	// 		        ufomap_msgs::msgToUfo(request.bounding_volume);
	// 		    for (auto &b : bv) {
	// 			    map.setValueVolume(b, map.getClampingThresMin(), request.depth);
	// 		    }
	// 		    for (int i = 0; i < map_pub_.size(); ++i) {
	// 			    if (map_pub_[i] &&
	// 			        (0 < map_pub_[i].getNumSubscribers() || map_pub_[i].isLatched())) {
	// 				    ufomap_msgs::UFOMapStamped::Ptr msg(new ufomap_msgs::UFOMapStamped);
	// 				    if (ufomap_msgs::ufoToMsg(map, msg->map, bv, compress_, i)) {
	// 					    msg->header.stamp = ros::Time::now();
	// 					    msg->header.frame_id = frame_id_;
	// 					    map_pub_[i].publish(msg);
	// 				    }
	// 			    }
	// 		    }
	// 		    response.success = true;
	// 	    } else {
	// 		    response.success = false;
	// 	    }
	//     },
	//     map_);
	return true;
}

bool Server::resetCallback(ufomap_srvs::Reset::Request &request,
                           ufomap_srvs::Reset::Response &response)
{
	std::visit(
	    [this, &request, &response](auto &map) {
		    using T = std::decay_t<decltype(map)>;
		    if constexpr (!std::is_same_v<std::monostate, T>) {
			    map.clear(request.new_resolution, request.new_depth_levels);
			    response.success = true;
		    } else {
			    response.success = false;
		    }
	    },
	    map_);
	return true;
}

bool Server::saveMapCallback(ufomap_srvs::SaveMap::Request &request,
                             ufomap_srvs::SaveMap::Response &response)
{
	std::visit(
	    [this, &request, &response](auto &map) {
		    using T = std::decay_t<decltype(map)>;
		    if constexpr (!std::is_same_v<std::monostate, T>) {
			    ufo::geometry::BoundingVolume bv =
			        ufomap_msgs::msgToUfo(request.bounding_volume);
			    auto pred = ufo::map::predicate::Intersects(bv);
			    map.write(request.filename, pred, request.depth, request.compress, 1,
			              request.compression_level);
			    response.success = true;
		    } else {
			    response.success = false;
		    }
	    },
	    map_);
	return true;
}

void Server::timerCallback(ros::TimerEvent const &event)
{
	std_msgs::Header header;
	header.stamp = ros::Time::now();
	header.frame_id = frame_id_;

	if (!map_pub_.empty()) {
		for (int i = 0; i < map_pub_.size(); ++i) {
			if (map_pub_[i] &&
			    (0 < map_pub_[i].getNumSubscribers() || map_pub_[i].isLatched())) {
				std::visit(
				    [this, &header, i](auto &map) {
					    using T = std::decay_t<decltype(map)>;
					    if constexpr (!std::is_same_v<std::monostate, T>) {
						    timing_.start("Full Publish");
						    ufomap_msgs::UFOMapStamped::Ptr msg(new ufomap_msgs::UFOMapStamped);
						    if (ufomap_msgs::ufoToMsg(map, msg->map, i, compress_)) {
							    msg->header = header;
							    map_pub_[i].publish(msg);
						    }
						    timing_.stop("Full Publish");
					    }
				    },
				    map_);
			}
		}
	}
	if (verbose_) {
		printInfo();
	}
}

void Server::configCallback(ufomap_mapping::ServerConfig &config, uint32_t level)
{
	// Read parameters
	frame_id_ = config.frame_id;

	verbose_ = config.verbose;

	integrator_.setMaxRange(config.max_range);
	integrator_.setMissDepth(config.insert_depth);
	simple_ray_casting_ = config.simple_ray_casting;
	integrator_.setEarlyStopping(config.early_stopping);
	async_ = config.async;

	clear_robot_ = config.clear_robot;
	robot_frame_id_ = config.robot_frame_id;
	robot_height_ = config.robot_height;
	robot_radius_ = config.robot_radius;
	clearing_depth_ = config.clearing_depth;

	compress_ = config.compress;
	update_part_of_map_ = config.update_part_of_map;
	publish_depth_ = config.publish_depth;

	integrator_.setOccupancyProbHit(config.prob_hit);
	integrator_.setOccupancyProbMiss(config.prob_miss);

	std::visit(
	    [this, &config](auto &map) {
		    if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>) {
			    map.setOccupancyClampingThres(config.clamping_thres_min,
			                                  config.clamping_thres_max);
		    }
	    },
	    map_);

	transform_timeout_.fromSec(config.transform_timeout);

	// Set up publisher
	if (map_pub_.empty() || map_pub_[0].isLatched() != config.map_latch ||
	    map_queue_size_ != config.map_queue_size) {
		map_pub_.resize(publish_depth_ + 1);
		for (int i = 0; i < map_pub_.size(); ++i) {
			map_queue_size_ = config.map_queue_size;
			std::string final_topic = i == 0 ? "map" : "map_depth_" + std::to_string(i);
			map_pub_[i] = nh_priv_.advertise<ufomap_msgs::UFOMapStamped>(
			    final_topic, map_queue_size_,
			    boost::bind(&Server::mapConnectCallback, this, _1, i),
			    ros::SubscriberStatusCallback(), ros::VoidConstPtr(), config.map_latch);
		}
	}

	// Set up subscriber
	if (!cloud_sub_ || cloud_in_queue_size_ != config.cloud_in_queue_size) {
		cloud_in_queue_size_ = config.cloud_in_queue_size;
		cloud_sub_ =
		    nh_.subscribe("cloud_in", cloud_in_queue_size_, &Server::cloudCallback, this);
	}

	// Set up timer
	if (!pub_timer_ || pub_rate_ != config.pub_rate) {
		pub_rate_ = config.pub_rate;
		if (0 < pub_rate_) {
			pub_timer_ =
			    nh_priv_.createTimer(ros::Rate(pub_rate_), &Server::timerCallback, this);
		} else {
			pub_timer_.stop();
		}
	}

	update_rate_ = ros::Duration(1.0 / config.update_rate);
}

}  // namespace ufomap_mapping
