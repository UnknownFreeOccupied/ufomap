/**
 * UFOMap Mapping
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomap_mapping
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

// UFO
#include <ufomap_mapping/server.h>
#include <ufomap_msgs/UFOMapStamped.h>
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>

// STD
#include <chrono>
#include <future>
#include <numeric>

namespace ufomap_mapping
{
Server::Server(ros::NodeHandle &nh, ros::NodeHandle &nh_priv)
    : nh_(nh), nh_priv_(nh_priv), tf_listener_(tf_buffer_), cs_(nh_priv)
{
	// Set up map
	double resolution = nh_priv_.param("resolution", 0.05);
	ufo::map::DepthType depth_levels = nh_priv_.param("depth_levels", 16);

	// Automatic pruning is disabled so we can work in multiple threads for subscribers,
	// services and publishers
	if (nh_priv_.param("color_map", false)) {
		map_.emplace<ufo::map::OccupancyMapColor>(resolution, depth_levels, false);
	} else {
		map_.emplace<ufo::map::OccupancyMap>(resolution, depth_levels, false);
	}

	// Enable min/max change detection
	std::visit(
	    [this](auto &map) {
		    if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>) {
			    map.enableMinMaxChangeDetection(true);
		    }
	    },
	    map_);

	// Set up dynamic reconfigure server
	cs_.setCallback(boost::bind(&Server::configCallback, this, _1, _2));

	// Set up publisher
	info_pub_ = nh_priv_.advertise<diagnostic_msgs::DiagnosticStatus>("info", 10, false);

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
		    if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>) {
			    auto start = std::chrono::steady_clock::now();

			    // Update map
			    ufo::map::PointCloudColor cloud;
			    ufomap_ros::rosToUfo(*msg, cloud);
			    cloud.transform(transform, true);

			    map.insertPointCloudDiscrete(transform.translation(), cloud, max_range_,
			                                 insert_depth_, simple_ray_casting_,
			                                 early_stopping_, async_);

			    double integration_time =
			        std::chrono::duration<float, std::chrono::seconds::period>(
			            std::chrono::steady_clock::now() - start)
			            .count();

			    if (0 == num_integrations_ || integration_time < min_integration_time_) {
				    min_integration_time_ = integration_time;
			    }
			    if (integration_time > max_integration_time_) {
				    max_integration_time_ = integration_time;
			    }
			    accumulated_integration_time_ += integration_time;
			    ++num_integrations_;

			    // Clear robot
			    if (clear_robot_) {
				    start = std::chrono::steady_clock::now();

				    try {
					    transform = ufomap_ros::rosToUfo(
					        tf_buffer_
					            .lookupTransform(frame_id_, robot_frame_id_, msg->header.stamp,
					                             transform_timeout_)
					            .transform);
				    } catch (tf2::TransformException &ex) {
					    ROS_WARN_THROTTLE(1, "%s", ex.what());
					    return;
				    }

				    ufo::map::Point3 r(robot_radius_, robot_radius_, robot_height_ / 2.0);
				    ufo::geometry::AABB aabb(transform.translation() - r,
				                             transform.translation() + r);
				    map.setValueVolume(aabb, map.getClampingThresMin(), clearing_depth_);

				    double clear_time =
				        std::chrono::duration<float, std::chrono::seconds::period>(
				            std::chrono::steady_clock::now() - start)
				            .count();
				    if (0 == num_clears_ || clear_time < min_clear_time_) {
					    min_clear_time_ = clear_time;
				    }
				    if (clear_time > max_clear_time_) {
					    max_clear_time_ = clear_time;
				    }
				    accumulated_clear_time_ += clear_time;
				    ++num_clears_;
			    }

			    // Publish update
			    if (!map_pub_.empty() && update_part_of_map_ && map.validMinMaxChange() &&
			        (!last_update_time_.isValid() ||
			         (msg->header.stamp - last_update_time_) >= update_rate_)) {
				    bool can_update = true;
				    if (update_async_handler_.valid()) {
					    can_update = std::future_status::ready ==
					                 update_async_handler_.wait_for(std::chrono::seconds(0));
				    }

				    if (can_update) {
					    last_update_time_ = msg->header.stamp;
					    start = std::chrono::steady_clock::now();

					    ufo::geometry::AABB aabb(map.minChange(), map.maxChange());
					    // TODO: should this be here?
					    map.resetMinMaxChangeDetection();

					    update_async_handler_ = std::async(
					        std::launch::async, [this, aabb, stamp = msg->header.stamp]() {
						        std::visit(
						            [this, &aabb, stamp](auto &map) {
							            if constexpr (!std::is_same_v<std::decay_t<decltype(map)>,
							                                          std::monostate>) {
								            for (int i = 0; i < map_pub_.size(); ++i) {
									            if (map_pub_[i] && (0 < map_pub_[i].getNumSubscribers() ||
									                                map_pub_[i].isLatched())) {
										            ufomap_msgs::UFOMapStamped::Ptr msg(
										                new ufomap_msgs::UFOMapStamped);
										            if (ufomap_msgs::ufoToMsg(map, msg->map, aabb, compress_,
										                                      i)) {
											            msg->header.stamp = stamp;
											            msg->header.frame_id = frame_id_;
											            map_pub_[i].publish(msg);
										            }
									            }
								            }
							            }
						            },
						            map_);
					        });

					    double update_time =
					        std::chrono::duration<float, std::chrono::seconds::period>(
					            std::chrono::steady_clock::now() - start)
					            .count();
					    if (0 == num_updates_ || update_time < min_update_time_) {
						    min_update_time_ = update_time;
					    }
					    if (update_time > max_update_time_) {
						    max_update_time_ = update_time;
					    }
					    accumulated_update_time_ += update_time;
					    ++num_updates_;
				    }
			    }

			    publishInfo();
		    }
	    },
	    map_);
}

void Server::publishInfo()
{
	if (verbose_) {
		printf("\nTimings:\n");
		if (0 != num_integrations_) {
			printf("\tIntegration time (s): %5d %09.6f\t(%09.6f +- %09.6f)\n",
			       num_integrations_, accumulated_integration_time_,
			       accumulated_integration_time_ / num_integrations_, max_integration_time_);
		}
		if (0 != num_clears_) {
			printf("\tClear time (s):       %5d %09.6f\t(%09.6f +- %09.6f)\n", num_clears_,
			       accumulated_clear_time_, accumulated_clear_time_ / num_clears_,
			       max_clear_time_);
		}
		if (0 != num_updates_) {
			printf("\tUpdate time (s):      %5d %09.6f\t(%09.6f +- %09.6f)\n", num_updates_,
			       accumulated_update_time_, accumulated_update_time_ / num_updates_,
			       max_update_time_);
		}
		if (0 != num_wholes_) {
			printf("\tWhole time (s):       %5d %09.6f\t(%09.6f +- %09.6f)\n", num_wholes_,
			       accumulated_whole_time_, accumulated_whole_time_ / num_wholes_,
			       max_whole_time_);
		}
	}

	if (info_pub_ && 0 < info_pub_.getNumSubscribers()) {
		diagnostic_msgs::DiagnosticStatus msg;
		msg.level = diagnostic_msgs::DiagnosticStatus::OK;
		msg.name = "UFOMap mapping timings";
		msg.values.resize(12);
		msg.values[0].key = "Min integration time (ms)";
		msg.values[0].value = std::to_string(min_integration_time_);
		msg.values[1].key = "Max integration time (ms)";
		msg.values[1].value = std::to_string(max_integration_time_);
		msg.values[2].key = "Average integration time (ms)";
		msg.values[2].value =
		    std::to_string(accumulated_integration_time_ / num_integrations_);
		msg.values[3].key = "Min clear time (ms)";
		msg.values[3].value = std::to_string(min_clear_time_);
		msg.values[4].key = "Max clear time (ms)";
		msg.values[4].value = std::to_string(max_clear_time_);
		msg.values[5].key = "Average clear time (ms)";
		msg.values[5].value = std::to_string(accumulated_clear_time_ / num_clears_);
		msg.values[6].key = "Min update time (ms)";
		msg.values[6].value = std::to_string(min_update_time_);
		msg.values[7].key = "Max update time (ms)";
		msg.values[7].value = std::to_string(max_update_time_);
		msg.values[8].key = "Average update time (ms)";
		msg.values[8].value = std::to_string(accumulated_update_time_ / num_updates_);
		msg.values[9].key = "Min whole time (ms)";
		msg.values[9].value = std::to_string(min_whole_time_);
		msg.values[10].key = "Max whole time (ms)";
		msg.values[10].value = std::to_string(max_whole_time_);
		msg.values[11].key = "Average whole time (ms)";
		msg.values[11].value = std::to_string(accumulated_whole_time_ / num_wholes_);
		info_pub_.publish(msg);
	}
}

void Server::mapConnectCallback(ros::SingleSubscriberPublisher const &pub, int depth)
{
	// When a new node subscribes we will publish the whole map to that node.

	// TODO: Make this async

	std::visit(
	    [this, &pub, depth](auto &map) {
		    if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>) {
			    auto start = std::chrono::steady_clock::now();

			    ufomap_msgs::UFOMapStamped::Ptr msg(new ufomap_msgs::UFOMapStamped);
			    if (ufomap_msgs::ufoToMsg(map, msg->map, compress_, depth)) {
				    msg->header.stamp = ros::Time::now();
				    msg->header.frame_id = frame_id_;
				    pub.publish(msg);
			    }

			    double whole_time = std::chrono::duration<float, std::chrono::seconds::period>(
			                            std::chrono::steady_clock::now() - start)
			                            .count();
			    if (0 == num_wholes_ || whole_time < min_whole_time_) {
				    min_whole_time_ = whole_time;
			    }
			    if (whole_time > max_whole_time_) {
				    max_whole_time_ = whole_time;
			    }
			    accumulated_whole_time_ += whole_time;
			    ++num_wholes_;
		    }
	    },
	    map_);
}

bool Server::getMapCallback(ufomap_srvs::GetMap::Request &request,
                            ufomap_srvs::GetMap::Response &response)
{
	std::visit(
	    [this, &request, &response](auto &map) {
		    if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>) {
			    ufo::geometry::BoundingVolume bv =
			        ufomap_msgs::msgToUfo(request.bounding_volume);
			    response.success = ufomap_msgs::ufoToMsg(map, response.map, bv,
			                                             request.compress, request.depth);
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
	std::visit(
	    [this, &request, &response](auto &map) {
		    if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>) {
			    ufo::geometry::BoundingVolume bv =
			        ufomap_msgs::msgToUfo(request.bounding_volume);
			    for (auto &b : bv) {
				    map.setValueVolume(b, map.getClampingThresMin(), request.depth);
			    }
			    response.success = true;
		    } else {
			    response.success = false;
		    }
	    },
	    map_);
	return true;
}

bool Server::resetCallback(ufomap_srvs::Reset::Request &request,
                           ufomap_srvs::Reset::Response &response)
{
	std::visit(
	    [this, &request, &response](auto &map) {
		    if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>) {
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
		    if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>) {
			    ufo::geometry::BoundingVolume bv =
			        ufomap_msgs::msgToUfo(request.bounding_volume);
			    response.success = map.write(request.filename, bv, request.compress,
			                                 request.depth, 1, request.compression_level);
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
					    if constexpr (!std::is_same_v<std::decay_t<decltype(map)>,
					                                  std::monostate>) {
						    auto start = std::chrono::steady_clock::now();

						    ufomap_msgs::UFOMapStamped::Ptr msg(new ufomap_msgs::UFOMapStamped);
						    if (ufomap_msgs::ufoToMsg(map, msg->map, compress_, i)) {
							    msg->header = header;
							    map_pub_[i].publish(msg);
						    }

						    double whole_time =
						        std::chrono::duration<float, std::chrono::seconds::period>(
						            std::chrono::steady_clock::now() - start)
						            .count();
						    if (0 == num_wholes_ || whole_time < min_whole_time_) {
							    min_whole_time_ = whole_time;
						    }
						    if (whole_time > max_whole_time_) {
							    max_whole_time_ = whole_time;
						    }
						    accumulated_whole_time_ += whole_time;
						    ++num_wholes_;
					    }
				    },
				    map_);
			}
		}
	}
	publishInfo();
}

void Server::configCallback(ufomap_mapping::ServerConfig &config, uint32_t level)
{
	// Read parameters
	frame_id_ = config.frame_id;

	verbose_ = config.verbose;

	max_range_ = config.max_range;
	insert_depth_ = config.insert_depth;
	simple_ray_casting_ = config.simple_ray_casting;
	early_stopping_ = config.early_stopping;
	async_ = config.async;

	clear_robot_ = config.clear_robot;
	robot_frame_id_ = config.robot_frame_id;
	robot_height_ = config.robot_height;
	robot_radius_ = config.robot_radius;
	clearing_depth_ = config.clearing_depth;

	compress_ = config.compress;
	update_part_of_map_ = config.update_part_of_map;
	publish_depth_ = config.publish_depth;

	std::visit(
	    [this, &config](auto &map) {
		    if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>) {
			    map.setProbHit(config.prob_hit);
			    map.setProbMiss(config.prob_miss);
			    map.setClampingThresMin(config.clamping_thres_min);
			    map.setClampingThresMax(config.clamping_thres_max);
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
