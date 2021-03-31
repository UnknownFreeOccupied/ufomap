/**
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomap
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

#ifndef UFO_MAP_OCCUPANCY_MAP_BASE_H
#define UFO_MAP_OCCUPANCY_MAP_BASE_H

#include <ufo/map/iterator/occupancy_map.h>
#include <ufo/map/iterator/occupancy_map_nearest.h>
#include <ufo/map/occupancy_map_node.h>
#include <ufo/map/octree.h>
#include <ufo/map/point_cloud.h>
#include <ufo/map/types.h>

namespace ufo::map
{
template <typename DATA_TYPE>
class OccupancyMapBase : public Octree<DATA_TYPE, OccupancyMapInnerNode<DATA_TYPE>,
                                       OccupancyMapLeafNode<DATA_TYPE>>
{
 protected:
	using Base = Octree<DATA_TYPE, OccupancyMapInnerNode<DATA_TYPE>,
	                    OccupancyMapLeafNode<DATA_TYPE>>;
	using INNER_NODE = OccupancyMapInnerNode<DATA_TYPE>;
	using LEAF_NODE = OccupancyMapLeafNode<DATA_TYPE>;

	using OccupancyMapBasereeIterator =
	    OccupancyMapIterator<OccupancyMapBase, DATA_TYPE, INNER_NODE, LEAF_NODE, false>;
	using OccupancyMapLeafIterator =
	    OccupancyMapIterator<OccupancyMapBase, DATA_TYPE, INNER_NODE, LEAF_NODE, true>;
	using OccupancyMapBasereeNNIterator =
	    OccupancyMapNearestIterator<OccupancyMapBase, DATA_TYPE, INNER_NODE, LEAF_NODE,
	                                false>;
	using OccupancyMapLeafNNIterator =
	    OccupancyMapNearestIterator<OccupancyMapBase, DATA_TYPE, INNER_NODE, LEAF_NODE,
	                                true>;

	using LogitType = decltype(DATA_TYPE::occupancy);

	// TODO: Why do I need this here instead of using it from Base?
	using Path = std::array<LEAF_NODE*, Base::MAX_DEPTH_LEVELS>;

 public:
	//
	// Tree type
	//

	// virtual std::string getTreeType() const noexcept override { return "occupancy_map"; }

	//
	// "Normal" iterators
	//

	OccupancyMapBasereeIterator beginTree(bool occupied_space = true,
	                                      bool free_space = true,
	                                      bool unknown_space = false, bool contains = false,
	                                      DepthType min_depth = 0) const noexcept
	{
		return OccupancyMapBasereeIterator(this, Base::getRoot(),
		                                   ufo::geometry::BoundingVolume(), occupied_space,
		                                   free_space, unknown_space, contains, min_depth);
	}

	OccupancyMapBasereeIterator endTree() const noexcept
	{
		return OccupancyMapBasereeIterator();
	}

	OccupancyMapBasereeIterator beginTree(ufo::geometry::BoundingVar const& bounding_volume,
	                                      bool occupied_space = true,
	                                      bool free_space = true,
	                                      bool unknown_space = false, bool contains = false,
	                                      DepthType min_depth = 0) const noexcept
	{
		ufo::geometry::BoundingVolume bv;
		bv.add(bounding_volume);
		return OccupancyMapBasereeIterator(this, Base::getRoot(), bv, occupied_space,
		                                   free_space, unknown_space, contains, min_depth);
	}

	OccupancyMapBasereeIterator beginTree(
	    ufo::geometry::BoundingVolume const& bounding_volume, bool occupied_space = true,
	    bool free_space = true, bool unknown_space = false, bool contains = false,
	    DepthType min_depth = 0) const noexcept
	{
		return OccupancyMapBasereeIterator(this, Base::getRoot(), bounding_volume,
		                                   occupied_space, free_space, unknown_space,
		                                   contains, min_depth);
	}

	OccupancyMapLeafIterator beginLeaves(bool occupied_space = true, bool free_space = true,
	                                     bool unknown_space = false, bool contains = false,
	                                     DepthType min_depth = 0) const noexcept
	{
		return OccupancyMapLeafIterator(this, Base::getRoot(),
		                                ufo::geometry::BoundingVolume(), occupied_space,
		                                free_space, unknown_space, contains, min_depth);
	}

	OccupancyMapLeafIterator endLeaves() const noexcept
	{
		return OccupancyMapLeafIterator();
	}

	OccupancyMapLeafIterator beginLeaves(ufo::geometry::BoundingVar const& bounding_volume,
	                                     bool occupied_space = true, bool free_space = true,
	                                     bool unknown_space = false, bool contains = false,
	                                     DepthType min_depth = 0) const noexcept
	{
		ufo::geometry::BoundingVolume bv;
		bv.add(bounding_volume);
		return OccupancyMapLeafIterator(this, Base::getRoot(), bv, occupied_space, free_space,
		                                unknown_space, contains, min_depth);
	}

	OccupancyMapLeafIterator beginLeaves(
	    ufo::geometry::BoundingVolume const& bounding_volume, bool occupied_space = true,
	    bool free_space = true, bool unknown_space = false, bool contains = false,
	    DepthType min_depth = 0) const noexcept
	{
		return OccupancyMapLeafIterator(this, Base::getRoot(), bounding_volume,
		                                occupied_space, free_space, unknown_space, contains,
		                                min_depth);
	}

	//
	// Nearest neighbor iterators
	//

	OccupancyMapBasereeNNIterator beginNNTree(Point3 const& coordinate,
	                                          bool occupied_space = true,
	                                          bool free_space = true,
	                                          bool unknown_space = false,
	                                          bool contains = false,
	                                          DepthType min_depth = 0) const noexcept
	{
		return OccupancyMapBasereeNNIterator(
		    this, Base::getRoot(), ufo::geometry::BoundingVolume(), coordinate,
		    occupied_space, free_space, unknown_space, contains, min_depth);
	}

	OccupancyMapBasereeNNIterator endNNTree() const noexcept
	{
		return OccupancyMapBasereeNNIterator();
	}

	OccupancyMapBasereeNNIterator beginNNTree(
	    Point3 const& coordinate, ufo::geometry::BoundingVar const& bounding_volume,
	    bool occupied_space = true, bool free_space = true, bool unknown_space = false,
	    bool contains = false, DepthType min_depth = 0) const noexcept
	{
		ufo::geometry::BoundingVolume bv;
		bv.add(bounding_volume);
		return OccupancyMapBasereeNNIterator(this, Base::getRoot(), bv, coordinate,
		                                     occupied_space, free_space, unknown_space,
		                                     contains, min_depth);
	}

	OccupancyMapBasereeNNIterator beginNNTree(
	    Point3 const& coordinate, ufo::geometry::BoundingVolume const& bounding_volume,
	    bool occupied_space = true, bool free_space = true, bool unknown_space = false,
	    bool contains = false, DepthType min_depth = 0) const noexcept
	{
		return OccupancyMapBasereeNNIterator(this, Base::getRoot(), bounding_volume,
		                                     coordinate, occupied_space, free_space,
		                                     unknown_space, contains, min_depth);
	}

	OccupancyMapLeafNNIterator beginNNLeaves(Point3 const& coordinate,
	                                         bool occupied_space = true,
	                                         bool free_space = true,
	                                         bool unknown_space = false,
	                                         bool contains = false,
	                                         DepthType min_depth = 0) const noexcept
	{
		return OccupancyMapLeafNNIterator(
		    this, Base::getRoot(), ufo::geometry::BoundingVolume(), coordinate,
		    occupied_space, free_space, unknown_space, contains, min_depth);
	}

	OccupancyMapLeafNNIterator endNNLeaves() const noexcept
	{
		return OccupancyMapLeafNNIterator();
	}

	OccupancyMapLeafNNIterator beginNNLeaves(
	    Point3 const& coordinate, ufo::geometry::BoundingVar const& bounding_volume,
	    bool occupied_space = true, bool free_space = true, bool unknown_space = false,
	    bool contains = false, DepthType min_depth = 0) const noexcept
	{
		ufo::geometry::BoundingVolume bv;
		bv.add(bounding_volume);
		return OccupancyMapLeafNNIterator(this, Base::getRoot(), bv, coordinate,
		                                  occupied_space, free_space, unknown_space, contains,
		                                  min_depth);
	}

	OccupancyMapLeafNNIterator beginNNLeaves(
	    Point3 const& coordinate, ufo::geometry::BoundingVolume const& bounding_volume,
	    bool occupied_space = true, bool free_space = true, bool unknown_space = false,
	    bool contains = false, DepthType min_depth = 0) const noexcept
	{
		return OccupancyMapLeafNNIterator(this, Base::getRoot(), bounding_volume, coordinate,
		                                  occupied_space, free_space, unknown_space, contains,
		                                  min_depth);
	}

	//
	// Integration
	//

	void insertMissOnRay(Point3 const& origin, Point3 const& end, double max_range = -1,
	                     DepthType depth = 0)
	{
		for (Code const& code : Base::computeRay(origin, end, max_range, depth)) {
			// Free space
			integrateMiss(code);
		}
	}

	template <typename T, typename = std::enable_if_t<std::is_base_of_v<Point3, T>>>
	void insertRay(Point3 const& origin, T const& end, double max_range = -1,
	               DepthType depth = 0)
	{
		// Free space
		Base::insertMissOnRay(origin, end, max_range, depth);
		// Occupied space
		Base::integrateHit(end);
	}

	template <typename T>
	void insertPointCloud(Point3 const& sensor_origin, T cloud, double max_range = -1,
	                      DepthType depth = 0, bool simple_ray_casting = false,
	                      unsigned int early_stopping = 0, bool async = false)
	{
		std::vector<std::pair<Code, float>> occupied_hits;
		occupied_hits.reserve(cloud.size());
		PointCloud discretized;
		discretized.reserve(cloud.size());
		Point3 min_change = Base::getMax();
		Point3 max_change = Base::getMin();
		for (Point3& end : cloud) {
			Point3 origin = sensor_origin;
			Point3 direction = (end - origin);
			double distance = direction.norm();

			// Move origin and end inside BBX
			if (!Base::moveLineInside(origin, end)) {
				// Line outside of BBX
				continue;
			}

			if (0 > max_range || distance <= max_range) {
				// Occupied space
				Code end_code = Base::toCode(end);
				if (indices_.insert(end_code).second) {
					occupied_hits.push_back(std::make_pair(end_code, prob_hit_log_));
				}
			} else {
				direction /= distance;
				end = origin + (direction * max_range);
			}

			discretized.push_back(end);

			for (int i : {0, 1, 2}) {
				min_change[i] = std::min(min_change[i], std::min(end[i], origin[i]));
				max_change[i] = std::max(max_change[i], std::max(end[i], origin[i]));
			}
		}

		LogitType prob_miss_log = prob_miss_log_ / double((2.0 * depth) + 1);

		indices_.clear();

		if (integrate_.valid()) {
			integrate_.wait();
		}

		if (async) {
			integrate_ = std::async(
			    std::launch::async, &OccupancyMapBase<DATA_TYPE>::insertPointCloudHelper, this,
			    sensor_origin, std::move(discretized), std::move(occupied_hits), prob_miss_log,
			    depth, simple_ray_casting, early_stopping, min_change, max_change);
		} else {
			insertPointCloudHelper(sensor_origin, std::move(discretized),
			                       std::move(occupied_hits), prob_miss_log, depth,
			                       simple_ray_casting, early_stopping, min_change, max_change);
		}
	}

	template <typename T>
	void insertPointCloud(Point3 const& sensor_origin, T cloud,
	                      math::Pose6 const& frame_origin, double max_range = -1,
	                      DepthType depth = 0, bool simple_ray_casting = false,
	                      unsigned int early_stopping = 0, bool async = false)
	{
		cloud.transform(frame_origin, async);
		insertPointCloud(sensor_origin, cloud, max_range, depth, simple_ray_casting,
		                 early_stopping, async);
	}

	template <typename T>
	void insertPointCloudDiscrete(Point3 const& sensor_origin, T const& cloud,
	                              double max_range = -1, DepthType depth = 0,
	                              bool simple_ray_casting = false,
	                              unsigned int early_stopping = 0, bool async = false)
	{
		double squared_max_range = max_range * max_range;

		std::vector<std::pair<Code, float>> occupied_hits;
		occupied_hits.reserve(cloud.size());
		PointCloud discretized;
		discretized.reserve(cloud.size());
		Point3 min_change = Base::getMax();
		Point3 max_change = Base::getMin();
		for (Point3 end : cloud) {
			if (0 > max_range || (end - sensor_origin).squaredNorm() < squared_max_range) {
				if (Base::isInside(end)) {
					Code end_code = Base::toCode(end);
					if (!indices_.insert(end_code).second) {
						continue;
					}
					occupied_hits.push_back(std::make_pair(end_code, prob_hit_log_));
				}
			} else {
				Point3 direction = Base::toCoord(Base::toKey(end, depth)) - sensor_origin;
				double distance = direction.norm();
				direction /= distance;
				if (0 <= max_range && distance > max_range) {
					end = sensor_origin + (direction * max_range);
				}
			}
			Point3 current = sensor_origin;
			// Move origin and end inside map
			if (!Base::moveLineInside(current, end)) {
				// Line outside of map
				continue;
			}

			Key end_key = Base::toKey(end, depth);

			if (!indices_.insert(Base::toCode(end_key)).second) {
				continue;
			}

			Point3 end_coord = Base::toCoord(end_key);

			discretized.push_back(end_coord);

			// Min/max change detection
			Point3 current_center = Base::toCoord(Base::toKey(current, depth));
			Point3 end_center = end_coord;

			double temp = Base::getNodeHalfSize(depth);
			for (int i : {0, 1, 2}) {
				min_change[i] = std::min(
				    min_change[i], std::min(end_center[i] - temp, current_center[i] - temp));
				max_change[i] = std::max(
				    max_change[i], std::max(end_center[i] + temp, current_center[i] + temp));
			}
		}

		LogitType prob_miss_log = prob_miss_log_ / double((2.0 * depth) + 1);

		indices_.clear();

		if (integrate_.valid()) {
			integrate_.wait();
		}

		if (async) {
			integrate_ = std::async(
			    std::launch::async, &OccupancyMapBase<DATA_TYPE>::insertPointCloudHelper, this,
			    sensor_origin, std::move(discretized), std::move(occupied_hits), prob_miss_log,
			    depth, simple_ray_casting, early_stopping, min_change, max_change);
		} else {
			insertPointCloudHelper(sensor_origin, std::move(discretized),
			                       std::move(occupied_hits), prob_miss_log, depth,
			                       simple_ray_casting, early_stopping, min_change, max_change);
		}
	}

	template <typename T>
	void InsertPointCloudDiscrete(Point3 const& sensor_origin, T cloud,
	                              math::Pose6 const& frame_origin, double max_range = -1,
	                              DepthType depth = 0, bool simple_ray_casting = false,
	                              unsigned int early_stopping = 0, bool async = false)
	{
		cloud.transform(frame_origin, async);
		insertPointCloudDiscrete(sensor_origin, cloud, max_range, depth, simple_ray_casting,
		                         early_stopping, async);
	}

	bool insertPointCloudDone() const
	{
		if (integrate_.valid()) {
			return std::future_status::ready == integrate_.wait_for(std::chrono::seconds(0));
		}
		return true;
	}

	void insertPointCloudWait() const
	{
		if (integrate_.valid()) {
			integrate_.wait();
		}
	}

	//
	// Cast ray
	//

	std::optional<Code> castRay(Point3 origin, Point3 direction,
	                            bool ignore_unknown = false, double max_range = -1,
	                            DepthType depth = 0) const
	{
		if (0 > max_range) {
			max_range = Base::getMin().distance(Base::getMin());
		}

		direction.normalize();
		Point3 end = origin + (direction * max_range);

		if (!Base::moveLineIntoBBX(origin, end)) {
			// Line fully outside of octree bounds
			return std::nullopt;
		}

		Key current;
		Key ending;
		std::array<int, 3> step;
		Point3 t_delta;
		Point3 t_max;
		Base::computeRayInit(origin, end, direction, current, ending, step, t_delta, t_max,
		                     depth);
		// Increment
		Code current_code = Base::toCode(current);
		while (current != ending && t_max.min() <= max_range) {
			if (isOccupied(current_code)) {
				return current_code;
			}
			if (!ignore_unknown && isUnknown(current_code)) {
				return std::nullopt;
			}
			Base::computeRayTakeStep(current, step, t_delta, t_max);
			current_code = Base::toCode(current);
		}

		return isOccupied(current_code) ? current_code : std::nullopt;
	}

	//
	// Set value volume
	//

	void setValueVolume(ufo::geometry::BoundingVar const& bounding_volume,
	                    double occupancy_value, DepthType min_depth = 0)
	{
		if (Base::getTreeDepthLevels() < min_depth) {
			return;
		}

		Point3 const center(0, 0, 0);
		double half_size = Base::getNodeHalfSize(Base::getTreeDepthLevels());
		ufo::geometry::AABB aabb(center, half_size);
		if (!std::visit(
		        [&aabb](auto&& arg) -> bool { return geometry::intersects(arg, aabb); },
		        bounding_volume)) {
			return;  // No node intersects
		} else if (Base::getTreeDepthLevels() == min_depth) {
			Base::deleteChildren(Base::getRoot(), Base::getTreeDepthLevels());
			setOccupancy(Base::getRoot().value.occupancy, toLogit(occupancy_value));
			updateNode(Base::getRoot(), Base::getTreeDepthLevels());
			return;
		}

		if (setValueVolumeRecurs(bounding_volume, toLogit(occupancy_value), Base::getRoot(),
		                         center, Base::getTreeDepthLevels(), min_depth)) {
			// TODO: Is this needed?
			updateNode(Base::getRoot(), Base::getTreeDepthLevels());
		}
	}

	//
	// Set value
	//

	void setOccupancy(Code const& code, double occupancy_value)
	{
		// TODO: Implement
		// setValue(code, toLogit(occupancy_value));
	}

	//
	// Update value
	//

	void updateOccupancy(Code const& code, double occupancy_value_update)
	{
		updateValue(code, toLogit(occupancy_value_update));
	}

	//
	// Integrate hit/miss
	//

	void integrateHit(Code const& code)
	{
		updateValue(code, static_cast<float>(prob_hit_log_));
	}

	void integrateHit(Point3 const& coord) { integrateHit(Base::toCode(coord)); }

	void integrateHit(double x, double y, double z) { integrateHit(Base::toCode(x, y, z)); }

	void integrateMiss(Code const& code)
	{
		updateValue(code, static_cast<float>(prob_miss_log_));
	}

	//
	// Get occupancy
	//

	double getOccupancy(Code const& code) const
	{
		return toProb(Base::getNode(code).first->value.occupancy);
	}

	//
	// Checking state
	//

	bool isOccupied(Code const& code) const
	{
		auto [node, depth] = Base::getNode(code);
		return isOccupied(*node);
	}

	bool isUnknown(Code const& code) const
	{
		auto [node, depth] = Base::getNode(code);
		return isUnknown(*node);
	}

	bool isFree(Code const& code) const
	{
		auto [node, depth] = Base::getNode(code);
		return isFree(*node);
	}

	//
	// Checking if contains
	//

	bool containsOccupied(Code const& code) const { return isOccupied(code); }

	bool containsUnknown(Code const& code) const
	{
		auto [node, depth] = Base::getNode(code);
		return containsUnknown(*node, depth);
	}

	bool containsFree(Code const& code) const
	{
		auto [node, depth] = Base::getNode(code);
		return containsFree(*node, depth);
	}

	//
	// Sensor model
	//

	double getOccupiedThres() const { return toProb(occupied_thres_log_); }

	double getFreeThres() const { return toProb(free_thres_log_); }

	double getProbHit() const { return toProb(prob_hit_log_); }

	double getProbMiss() const { return toProb(prob_miss_log_); }

	double getClampingThresMin() const { return toProb(clamping_thres_min_log_); }

	double getClampingThresMax() const { return toProb(clamping_thres_max_log_); }

	void setOccupiedFreeThres(double new_occupied_thres, double new_free_thres)
	{
		// TODO: Should add a warning that these are very computational expensive to
		// call since the whole tree has to be updated

		// FIXME: Implement better
		std::stringstream s(std::ios_base::in | std::ios_base::out | std::ios_base::binary);
		Base::write(s);

		occupied_thres_log_ = toLogit(new_occupied_thres);
		free_thres_log_ = toLogit(new_free_thres);

		Base::read(s);
	}

	void setProbHit(double probability) { prob_hit_log_ = toLogit(probability); }

	void setProbMiss(double probability) { prob_miss_log_ = toLogit(probability); }

	void setClampingThresMin(double probability)
	{
		clamping_thres_min_log_ = toLogit(probability);
	}

	void setClampingThresMax(double probability)
	{
		clamping_thres_max_log_ = toLogit(probability);
	}

	//
	// Change detection
	//

	CodeSet::const_iterator changesBegin() const noexcept { return changes_.begin(); }

	CodeSet::const_iterator changesEnd() const noexcept { return changes_.end(); }

	void enableChangeDetection(bool enable) noexcept { change_detection_enabled_ = enable; }

	bool isChangeDetectionEnabled() const noexcept { return change_detection_enabled_; }

	void resetChangeDetection() noexcept { changes_.clear(); }

	std::size_t numChangedDetected() const noexcept { return changes_.size(); }

	void enableMinMaxChangeDetection(bool enable) noexcept
	{
		if (!min_max_change_detection_enabled_ && enable) {
			resetMinMaxChangeDetection();
		}
		min_max_change_detection_enabled_ = enable;
	}

	bool isMinMaxChangeDetectionEnabled() const noexcept
	{
		return min_max_change_detection_enabled_;
	}

	Point3 const& minChange() const noexcept { return min_change_; }

	Point3 const& maxChange() const noexcept { return max_change_; }

	void resetMinMaxChangeDetection() noexcept
	{
		min_change_ = Base::getMax();
		max_change_ = Base::getMin();
	}

	bool validMinMaxChange() const noexcept
	{
		for (int i : {0, 1, 2}) {
			if (min_change_[i] > max_change_[i]) {
				return false;
			}
		}
		return true;
	}

	//
	// Bounding box contain all known
	//

	ufo::geometry::AABB getKnownBBX() const
	{
		if (!containsFree(Base::getRootCode()) && !containsOccupied(Base::getRootCode())) {
			// Only unknown
			return ufo::geometry::AABB(Point3(0, 0, 0), 0);
		}

		Point3 min(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
		           std::numeric_limits<double>::max());
		Point3 max(std::numeric_limits<double>::lowest(),
		           std::numeric_limits<double>::lowest(),
		           std::numeric_limits<double>::lowest());

		for (auto it = beginLeaves(true, true, false), it_end = endLeaves(); it != it_end;
		     ++it) {
			double hf = it.getHalfSize();
			Point3 center = it.getCenter();
			for (int i : {0, 1, 2}) {
				min[i] = std::min(min[i], center[i] - hf);
				max[i] = std::max(max[i], center[i] + hf);
			}
		}

		return ufo::geometry::AABB(min, max);
	}

 protected:
	//
	// Constructor
	//

	OccupancyMapBase(double resolution, DepthType depth_levels = 16,
	                 bool automatic_pruning = true, double occupied_thres = 0.5,
	                 double free_thres = 0.5, double prob_hit = 0.7, double prob_miss = 0.4,
	                 double clamping_thres_min = 0.1192, double clamping_thres_max = 0.971)
	    : Base(resolution, depth_levels, automatic_pruning),
	      occupied_thres_log_(toLogit(occupied_thres)),
	      free_thres_log_(toLogit(free_thres)),
	      prob_hit_log_(toLogit(prob_hit)),
	      prob_miss_log_(toLogit(prob_miss)),
	      clamping_thres_min_log_(toLogit(clamping_thres_min)),
	      clamping_thres_max_log_(toLogit(clamping_thres_max))
	{
		updateNode(Base::getRoot(), Base::getTreeDepthLevels());

		// Reserve for better performance
		indices_.max_load_factor(0.8);
		indices_.reserve(100003);
	}

	OccupancyMapBase(std::string const& filename, bool automatic_pruning = true,
	                 double occupied_thres = 0.5, double free_thres = 0.5,
	                 double prob_hit = 0.7, double prob_miss = 0.4,
	                 double clamping_thres_min = 0.1192, double clamping_thres_max = 0.971)
	    : OccupancyMapBase(0.1, 16, automatic_pruning, occupied_thres, free_thres, prob_hit,
	                       prob_miss, clamping_thres_min, clamping_thres_max)
	{
		Base::read(filename);
	}

	OccupancyMapBase(OccupancyMapBase const& other)
	    : OccupancyMapBase(other.resolution_, other.depth_levels_,
	                       other.automatic_pruning_enabled_, other.getOccupiedThres(),
	                       other.getFreeThres(), other.getProbHit(), other.getProbMiss(),
	                       other.getClampingThresMin(), other.getClampingThresMax())
	{
		std::stringstream s(std::ios_base::in | std::ios_base::out | std::ios_base::binary);
		other.write(s);
		Base::read(s);
	}

	//
	// Destructor
	//

	virtual ~OccupancyMapBase() {}

	//
	// Probability <-> logit
	//

	static double toLogit(double prob) { return std::log(prob / (1.0 - prob)); }

	static double toProb(LogitType logit) { return 1.0 / (1.0 + std::exp(-logit)); }

	//
	// Get occupancy
	//

	static double getOccupancy(LEAF_NODE const& node)
	{
		return toProb(node.value.occupancy);
	}

	//
	// Checking state
	//

	bool isOccupied(LEAF_NODE const& node) const
	{
		return occupied_thres_log_ < node.value.occupancy;
	}

	bool isUnknown(LEAF_NODE const& node) const
	{
		return free_thres_log_ <= node.value.occupancy &&
		       occupied_thres_log_ >= node.value.occupancy;
	}

	bool isFree(LEAF_NODE const& node) const
	{
		return free_thres_log_ > node.value.occupancy;
	}

	//
	// Checking if contains
	//

	// TODO: Should this exist?
	bool containsOccupied(LEAF_NODE const& node, DepthType depth) const
	{
		return isOccupied(node);
	}

	// TODO: Should this exist?
	bool containsUnknown(LEAF_NODE const& node, DepthType depth) const
	{
		if (0 == depth) {
			return isUnknown(node);
		}
		return containsUnknown(static_cast<INNER_NODE const&>(node));
	}

	// TODO: Should this exist?
	bool containsFree(LEAF_NODE const& node, DepthType depth) const
	{
		if (0 == depth) {
			return isFree(node);
		}
		return containsFree(static_cast<INNER_NODE const&>(node));
	}

	bool containsOccupied(INNER_NODE const& node) const { return isOccupied(node); }

	bool containsUnknown(INNER_NODE const& node) const
	{
		return static_cast<INNER_NODE const&>(node).contains_unknown;
	}

	bool containsFree(INNER_NODE const& node) const
	{
		return static_cast<INNER_NODE const&>(node).contains_free;
	}

	//
	// Set value volume
	//

	bool setValueVolumeRecurs(ufo::geometry::BoundingVar const& bounding_volume,
	                          double occupancy_value, INNER_NODE& node,
	                          Point3 const& center, DepthType current_depth,
	                          DepthType min_depth = 0)
	{
		DepthType const child_depth = current_depth - 1;
		double const child_half_size = Base::getNodeHalfSize(child_depth);

		Base::createChildren(node, current_depth);

		ufo::geometry::AABB aabb;
		aabb.half_size =
		    ufo::geometry::Point(child_half_size, child_half_size, child_half_size);
		bool changed = false;
		for (size_t i = 0; i < 8; ++i) {
			aabb.center = Base::getChildCenter(center, child_half_size, i);
			if (std::visit(
			        [&aabb](auto&& arg) -> bool { return geometry::intersects(arg, aabb); },
			        bounding_volume)) {
				if (0 == child_depth) {
					if (setOccupancy(Base::getLeafChild(node, i).value.occupancy,
					                 occupancy_value)) {
						changed = true;
					}
				} else {
					INNER_NODE& child = Base::getInnerChild(node, i);
					if (min_depth < child_depth) {
						if (setValueVolumeRecurs(bounding_volume, occupancy_value, child, aabb.center,
						                         child_depth, min_depth)) {
							changed = true;
						}
					} else {
						Base::deleteChildren(child, child_depth);
						if (setOccupancy(child.value.occupancy, occupancy_value)) {
							changed = true;
						}
						if (updateNode(child, child_depth)) {
							changed = true;
						}
					}
				}
			}
		}

		return !changed || updateNode(node, current_depth);
	}

	//
	// Set value
	//

	void setNodeValue(Code const& code, float occupancy)
	{
		auto [path, depth] = Base::getNodePath(code);

		occupancy = clampOccupancy(occupancy);
		if (path[depth]->value.occupancy == occupancy) {
			return;
		}

		if (code.getDepth() != depth) {
			createNode(path, code, depth);
			depth = code.getDepth();
		}

		path[depth]->value.occupancy = occupancy;
		if (Base::hasChildren(path[depth], depth)) {
			Base::deleteChildren(static_cast<INNER_NODE&>(*path[depth]), depth);
		}

		updateParents(path, depth);
	}

	//
	// Update value
	//

	void updateValue(Code const& code, LogitType const& update)
	{
		auto path = Base::createNode(code);
		DepthType depth = code.getDepth();

		if (Base::isLeaf(path[depth], depth)) {
			if (updateOccupancy(path[depth]->value.occupancy, update)) {
				if (change_detection_enabled_) {
					changes_.insert(code);
				}
			}
		} else {
			if (!updateAllChildren(code, static_cast<INNER_NODE&>(*path[depth]), depth,
			                       update)) {
				return;
			}
			++depth;
		}

		updateParents(path, depth);
	}

	bool updateAllChildren(Code const& code, INNER_NODE& node, DepthType depth,
	                       LogitType const& update)
	{
		bool changed = false;
		if (1 == depth) {
			for (int i = 0; i < 8; ++i) {
				LEAF_NODE& child = Base::getLeafChild(node, i);
				if (updateOccupancy(child.value.occupancy, update)) {
					changed = true;
					if (change_detection_enabled_) {
						changes_.insert(code);
					}
				}
			}
		} else {
			for (int i = 0; i < 8; ++i) {
				INNER_NODE& child = Base::getInnerChild(node, i);
				if (Base::isLeaf(child)) {
					if (updateOccupancy(child.value.occupancy, update)) {
						changed = true;
						updateNode(child, depth - 1);
						if (change_detection_enabled_) {
							changes_.insert(code);
						}
					}
				} else {
					// TODO: Careful here
					if (updateAllChildren(code.getChild(i), child, depth - 1, update)) {
						changed = true;
					}
				}
			}
		}

		return changed && updateNode(node, depth);
	}

	//
	// Update parents
	//

	void updateParents(Path const& path, DepthType depth)
	{
		for (unsigned int d = std::max(1u, depth); d <= Base::getTreeDepthLevels(); ++d) {
			if (!updateNode(static_cast<INNER_NODE&>(*path[d]), d)) {
				return;
			}
		}
	}

	//
	// Update occupancy
	//

	bool updateOccupancy(LogitType& current, LogitType const& update)
	{
		LogitType old_occupancy = current;
		current = std::clamp<LogitType>(current + update, clamping_thres_min_log_,
		                                clamping_thres_max_log_);
		return old_occupancy != current;
	}

	//
	// Set occupancy
	//

	bool setOccupancy(LogitType& current_value, LogitType const& new_value)
	{
		LogitType old_occupancy = current_value;
		current_value = std::clamp<LogitType>(new_value, clamping_thres_min_log_,
		                                      clamping_thres_max_log_);
		return old_occupancy != current_value;
	}

	//
	// Will value change
	//

	virtual bool willValueChange(LogitType current, LogitType update) const
	{
		return (0 > update && clamping_thres_min_log_ < current) ||
		       (0 < update && clamping_thres_max_log_ > current);
	}

	virtual LogitType clampOccupancy(LogitType occupancy) const
	{
		return std::clamp<LogitType>(occupancy, clamping_thres_min_log_,
		                             clamping_thres_max_log_);
	}

	//
	// Update node
	//

	virtual bool updateNode(INNER_NODE& node, DepthType depth)
	{
		if (Base::isLeaf(node)) {
			bool new_contains_free = isFree(node);
			bool new_contains_unknown = isUnknown(node);
			bool updated = (node.contains_free != new_contains_free) ||
			               (node.contains_unknown != new_contains_unknown);
			node.contains_free = new_contains_free;
			node.contains_unknown = new_contains_unknown;
			return updated;
		}

		LogitType new_occupancy_value = std::numeric_limits<LogitType>::lowest();
		bool new_contains_free = false;
		bool new_contains_unknown = false;

		if (1 == depth) {
			for (int i = 0; i < 8; ++i) {
				LEAF_NODE const& child = Base::getLeafChild(node, i);
				new_occupancy_value = std::max(new_occupancy_value, child.value.occupancy);
				new_contains_free = new_contains_free || isFree(child);
				new_contains_unknown = new_contains_unknown || isUnknown(child);
			}
		} else {
			for (int i = 0; i < 8; ++i) {
				INNER_NODE const& child = Base::getInnerChild(node, i);
				new_occupancy_value = std::max(new_occupancy_value, child.value.occupancy);
				new_contains_free = new_contains_free || containsFree(child);
				new_contains_unknown = new_contains_unknown || containsUnknown(child);
			}
		}

		if (Base::isNodeCollapsible(node, depth)) {
			Base::deleteChildren(node, depth);
		}

		if (node.value.occupancy != new_occupancy_value ||
		    node.contains_free != new_contains_free ||
		    node.contains_unknown != new_contains_unknown) {
			node.value.occupancy = new_occupancy_value;
			node.contains_free = new_contains_free;
			node.contains_unknown = new_contains_unknown;
			return true;
		}
		return false;
	}

	//
	// Calculate free space
	//
	template <typename T, typename C>
	void freeSpace(Point3 const& sensor_origin, C const& cloud, CodeMap<T>& indices,
	               T const& value, DepthType depth = 0, bool simple_ray_casting = false,
	               unsigned int early_stopping = 0) const
	{
		for (auto const& point : cloud) {
			Point3 current = sensor_origin;
			Point3 end;

			using point_type = std::decay_t<decltype(point)>;
			if constexpr (std::is_same_v<point_type, Code>) {
				end = Base::toCoord(point);
			} else if constexpr (std::is_base_of_v<point_type, Point3>) {
				end = point;
			} else {
				// TODO: Error
			}

			// Move origin and end inside map
			if (!Base::moveLineInside(current, end)) {
				// Line outside of map
				continue;
			}

			if (simple_ray_casting) {
				freeSpaceSimple(current, end, indices, value, depth, early_stopping);
			} else {
				freeSpaceNormal(current, end, indices, value, depth, early_stopping);
			}
		}
	}

	template <typename T>
	void freeSpaceNormal(Point3 const& from, Point3 const& to, CodeMap<T>& indices,
	                     T const& value, DepthType depth = 0,
	                     unsigned int early_stopping = 0) const
	{
		// Do it backwards
		Point3 current = to;
		Point3 end = from;

		Point3 direction = end - current;
		double distance = direction.norm();
		direction /= distance;
		Key current_key;
		Key end_key;
		std::array<int, 3> step;
		Point3 t_delta;
		Point3 t_max;
		Base::computeRayInit(current, end, direction, current_key, end_key, step, t_delta,
		                     t_max, depth);

		if (current_key == end_key) {
			indices.try_emplace(Base::toCode(current_key), value);
			return;
		}

		// if (0 == depth) {
		// 	Base::computeRayTakeStep(current_key, step, t_delta, t_max);
		// }
		unsigned int already_update_in_row = 0;
		do {
			if (indices.try_emplace(Base::toCode(current_key), value).second) {
				already_update_in_row = 0;
			} else {
				++already_update_in_row;
				if (0 < early_stopping && already_update_in_row >= early_stopping) {
					break;
				}
			}
			Base::computeRayTakeStep(current_key, step, t_delta, t_max);
		} while (current_key != end_key && t_max.min() <= distance);
	}

	template <typename T>
	void freeSpaceSimple(Point3 const& from, Point3 const& to, CodeMap<T>& indices,
	                     T const& value, DepthType depth = 0,
	                     unsigned int early_stopping = 0) const
	{
		// Do it backwards
		Point3 current = to;
		Point3 end = from;

		Point3 direction = end - current;
		double distance = direction.norm();
		direction /= distance;
		int num_steps = distance / Base::getNodeSize(depth);
		Point3 step = direction * Base::getNodeSize(depth);
		int current_step = 0;
		double current_distance = distance;
		double dist_per_step = distance / num_steps;
		// if (0 == depth) {
		// 	current += step;
		// 	current_step = 1;
		// }
		unsigned int already_update_in_row = 0;
		for (; current_step <= num_steps; ++current_step) {
			// if (indices.try_emplace(Base::toCode(current, depth), value / (current_distance *
			// current_distance)).second) {
			if (indices.try_emplace(Base::toCode(current, depth), value).second) {
				already_update_in_row = 0;
			} else {
				++already_update_in_row;
				if (0 < early_stopping && already_update_in_row >= early_stopping) {
					break;
				}
			}
			current += step;
			current_distance -= dist_per_step;
		}
	}

	//
	// Integrator helper
	//

	void insertPointCloudHelper(Point3 sensor_origin, PointCloud&& discretized,
	                            std::vector<std::pair<Code, float>>&& occupied_hits,
	                            LogitType prob_miss_log, DepthType depth,
	                            bool simple_ray_casting, unsigned int early_stopping,
	                            Point3 min_change, Point3 max_change)
	{
		std::future<void> f = std::async(std::launch::async, [this, &occupied_hits]() {
			std::for_each(begin(occupied_hits), end(occupied_hits),
			              [this](auto&& hit) { updateValue(hit.first, hit.second); });
		});

		CodeMap<LogitType> free_hits;

		freeSpace(sensor_origin, discretized, free_hits, prob_miss_log, depth,
		          simple_ray_casting, early_stopping);

		f.wait();

		for (auto const& [code, value] : free_hits) {
			updateValue(code, value);
		}

		if (min_max_change_detection_enabled_) {
			for (int i : {0, 1, 2}) {
				min_change_[i] = std::min(min_change_[i], min_change[i]);
				max_change_[i] = std::max(max_change_[i], max_change[i]);
			}
		}
	}

	//
	// Input/output (read/write)
	//

	virtual bool readNodes(std::istream& s,
	                       ufo::geometry::BoundingVolume const& bounding_volume) override
	{
		// Check if inside bounding_volume
		Point3 const center(0, 0, 0);
		double half_size = Base::getNodeHalfSize(Base::getTreeDepthLevels());
		if (!bounding_volume.empty() &&
		    !bounding_volume.intersects(ufo::geometry::AABB(center, half_size))) {
			return true;  // No node intersects
		}

		uint8_t children;
		s.read(reinterpret_cast<char*>(&children), sizeof(children));

		if (0 == children) {
			Base::deleteChildren(Base::getRoot(), Base::getTreeDepthLevels());
			Base::getRoot().readData(s);
			updateNode(Base::getRoot(), Base::getTreeDepthLevels());
			return true;
		}
		return readNodesRecurs(s, bounding_volume, Base::getRoot(), center,
		                       Base::getTreeDepthLevels());
	}

	bool readNodesRecurs(std::istream& s,
	                     ufo::geometry::BoundingVolume const& bounding_volume,
	                     INNER_NODE& node, Point3 const& center, unsigned int current_depth)
	{
		DepthType const child_depth = current_depth - 1;
		double const child_half_size = Base::getNodeHalfSize(child_depth);

		// 1 bit for each child; 0: leaf child, 1: child has children
		uint8_t children;
		s.read(reinterpret_cast<char*>(&children), sizeof(children));

		std::array<Point3, 8> child_centers;
		std::bitset<8> child_intersects;
		for (size_t i = 0; i < 8; ++i) {
			child_centers[i] = Base::getChildCenter(center, child_half_size, i);
			child_intersects[i] =
			    bounding_volume.empty() || bounding_volume.intersects(ufo::geometry::AABB(
			                                   child_centers[i], child_half_size));
		}

		Base::createChildren(node, current_depth);

		for (size_t i = 0; i < 8; ++i) {
			if (child_intersects[i]) {
				INNER_NODE& child = Base::getInnerChild(node, i);
				if ((children >> i) & 1U) {
					if (1 == child_depth) {
						double const grandchild_half_size = Base::getNodeHalfSize(0);
						Base::createChildren(child, child_depth);
						for (size_t j = 0; j < 8; ++j) {
							if (bounding_volume.empty() ||
							    bounding_volume.intersects(ufo::geometry::AABB(
							        Base::getChildCenter(child_centers[i], grandchild_half_size, j),
							        grandchild_half_size))) {
								Base::getLeafChild(child, j).readData(s);
							}
						}
						updateNode(child, child_depth);
					} else {
						readNodesRecurs(s, bounding_volume, child, child_centers[i], child_depth);
					}
				} else {
					Base::deleteChildren(child, child_depth);
					child.readData(s);
					updateNode(child, child_depth);
				}
			}
		}

		updateNode(node, current_depth);  // To set indicators

		return true;
	}

	virtual bool writeNodes(std::ostream& s,
	                        ufo::geometry::BoundingVolume const& bounding_volume,
	                        DepthType min_depth) const override
	{
		// Check if inside bounding_volume
		Point3 const center(0, 0, 0);
		double half_size = Base::getNodeHalfSize(Base::getTreeDepthLevels());
		if (!bounding_volume.empty() &&
		    !bounding_volume.intersects(ufo::geometry::AABB(center, half_size))) {
			return true;  // No node intersects
		}

		uint8_t children = 0;
		if (Base::hasChildren(Base::getRoot()) && Base::getTreeDepthLevels() > min_depth) {
			children = UINT8_MAX;
		}
		s.write(reinterpret_cast<char*>(&children), sizeof(children));

		if (0 == children) {
			Base::getRoot().writeData(s);
			return true;
		}
		return writeNodesRecurs(s, bounding_volume, Base::getRoot(), center,
		                        Base::getTreeDepthLevels(), min_depth);
	}

	bool writeNodesRecurs(std::ostream& s,
	                      ufo::geometry::BoundingVolume const& bounding_volume,
	                      INNER_NODE const& node, Point3 const& center,
	                      DepthType current_depth, DepthType min_depth = 0) const
	{
		DepthType const child_depth = current_depth - 1;
		double const child_half_size = Base::getNodeHalfSize(child_depth);

		// 1 bit for each child; 0: leaf child, 1: child has children
		uint8_t children = 0;
		std::array<Point3, 8> child_centers;
		std::bitset<8> child_intersects;
		for (size_t i = 0; i < 8; ++i) {
			if (child_depth > min_depth && Base::hasChildren(Base::getInnerChild(node, i))) {
				children |= 1U << i;
			}

			child_centers[i] = Base::getChildCenter(center, child_half_size, i);
			child_intersects[i] =
			    bounding_volume.empty() || bounding_volume.intersects(ufo::geometry::AABB(
			                                   child_centers[i], child_half_size));
		}

		s.write(reinterpret_cast<char*>(&children), sizeof(children));

		for (size_t i = 0; i < 8; ++i) {
			if (child_intersects[i]) {
				INNER_NODE const& child = Base::getInnerChild(node, i);
				if ((children >> i) & 1U) {
					if (1 == child_depth) {
						double const grandchild_half_size = Base::getNodeHalfSize(0);
						for (size_t j = 0; j < 8; ++j) {
							if (bounding_volume.empty() ||
							    bounding_volume.intersects(ufo::geometry::AABB(
							        Base::getChildCenter(child_centers[i], grandchild_half_size, j),
							        grandchild_half_size))) {
								Base::getLeafChild(child, j).writeData(s);
							}
						}
					} else {
						writeNodesRecurs(s, bounding_volume, child, child_centers[i], child_depth,
						                 min_depth);
					}
				} else {
					child.writeData(s);
				}
			}
		}

		return true;
	}

 protected:
	// Sensor model
	double occupied_thres_log_;      // Threshold for occupied
	double free_thres_log_;          // Threshold for free
	double prob_hit_log_;            // Logodds probability of hit
	double prob_miss_log_;           // Logodds probability of miss
	double clamping_thres_min_log_;  // Min logodds value
	double clamping_thres_max_log_;  // Max logodds value

	// Change detection
	bool change_detection_enabled_ = false;
	CodeSet changes_;
	bool min_max_change_detection_enabled_ = false;
	Point3 min_change_;
	Point3 max_change_;

	// Defined here for speedup
	CodeSet indices_;
	std::future<void> integrate_;

	template <typename T, typename D, typename I, typename L, bool O>
	friend class OccupancyMapIterator;
	template <typename T, typename D, typename I, typename L, bool O>
	friend class OccupancyMapNearestIterator;
};
}  // namespace ufo::map

#endif  // UFO_MAP_OCCUPANCY_MAP_BASE_H