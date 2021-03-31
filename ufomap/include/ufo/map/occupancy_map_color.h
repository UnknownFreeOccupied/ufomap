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

#ifndef UFO_MAP_OCCUPANCY_MAP_COLOR_H
#define UFO_MAP_OCCUPANCY_MAP_COLOR_H

#include <ufo/map/occupancy_map_base.h>

namespace ufo::map
{
class OccupancyMapColor : public OccupancyMapBase<ColorOccupancyNode<float>>
{
 private:
	using DATA_TYPE = ColorOccupancyNode<float>;
	using Base = OccupancyMapBase<DATA_TYPE>;

 public:
	//
	// Constructors
	//

	OccupancyMapColor(double resolution, DepthType depth_levels = 16,
	                  bool automatic_pruning = true, double occupied_thres = 0.5,
	                  double free_thres = 0.5, double prob_hit = 0.7,
	                  double prob_miss = 0.4, double clamping_thres_min = 0.1192,
	                  double clamping_thres_max = 0.971);

	OccupancyMapColor(std::string const& filename, bool automatic_pruning = true,
	                  double occupied_thres = 0.5, double free_thres = 0.5,
	                  double prob_hit = 0.7, double prob_miss = 0.4,
	                  double clamping_thres_min = 0.1192,
	                  double clamping_thres_max = 0.971);

	OccupancyMapColor(OccupancyMapColor const& other);

	//
	// Destructor
	//

	virtual ~OccupancyMapColor() {}

	//
	// Tree Type
	//

	virtual std::string getTreeType() const noexcept override
	{
		return "occupancy_map_color";
	}

	//
	// Integration
	//

	template <typename T>
	void insertPointCloud(Point3 const& sensor_origin, T const& cloud,
	                      double max_range = -1, DepthType depth = 0,
	                      bool simple_ray_casting = false, unsigned int early_stopping = 0,
	                      bool async = false)
	{
		if constexpr (std::is_same_v<T, PointCloud>) {
			Base::insertPointCloud(sensor_origin, cloud, max_range, depth, simple_ray_casting,
			                       early_stopping, async);

			if constexpr (std::is_same_v<T, PointCloudColor>) {
				integrateColors(sensor_origin, cloud, max_range);
			}
		} else if constexpr (std::is_same_v<T, PointCloudColor>) {
			std::vector<std::tuple<Code, float, Color>> occupied_hits;
			occupied_hits.reserve(cloud.size());
			PointCloud discretized;
			discretized.reserve(cloud.size());
			Point3 min_change = Base::getMax();
			Point3 max_change = Base::getMin();
			for (Point3Color& end_color : cloud) {
				Point3 end = end_color;
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
						occupied_hits.push_back(
						    std::make_tuple(end_code, prob_hit_log_, end_color.getColor()));
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
				integrate_ =
				    std::async(std::launch::async, &OccupancyMapColor::insertPointCloudHelper,
				               this, sensor_origin, std::move(discretized),
				               std::move(occupied_hits), prob_miss_log, depth, simple_ray_casting,
				               early_stopping, min_change, max_change);
			} else {
				insertPointCloudHelper(sensor_origin, std::move(discretized),
				                       std::move(occupied_hits), prob_miss_log, depth,
				                       simple_ray_casting, early_stopping, min_change,
				                       max_change);
			}
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
		if constexpr (std::is_same_v<T, PointCloud>) {
			Base::insertPointCloudDiscrete(sensor_origin, cloud, max_range, depth,
			                               simple_ray_casting, early_stopping);
		} else if constexpr (std::is_same_v<T, PointCloudColor>) {
			double squared_max_range = max_range * max_range;

			std::vector<std::tuple<Code, float, Color>> occupied_hits;
			occupied_hits.reserve(cloud.size());
			PointCloud discretized;
			discretized.reserve(cloud.size());
			Point3 min_change = Base::getMax();
			Point3 max_change = Base::getMin();
			for (Point3Color const& end_color : cloud) {
				Point3 end = end_color;
				double dist_sqrt = (end - sensor_origin).squaredNorm();
				if (0 > max_range || dist_sqrt < squared_max_range) {
					if (Base::isInside(end)) {
						Code end_code = Base::toCode(end);
						if (!indices_.insert(end_code).second) {
							continue;
						}
						// double dist = std::sqrt(dist_sqrt);
						// testis.push_back(std::make_tuple(end_code, prob_hit_log_ / (dist * dist),
						//                                  end_color.getColor()));
						occupied_hits.push_back(
						    std::make_tuple(end_code, prob_hit_log_, end_color.getColor()));
					}
				} else {
					Point3 direction = Base::toCoord(Base::toKey(end, depth)) - sensor_origin;
					dist_sqrt = direction.squaredNorm();
					if (0 <= max_range && dist_sqrt > squared_max_range) {
						direction /= std::sqrt(dist_sqrt);
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
				integrate_ =
				    std::async(std::launch::async, &OccupancyMapColor::insertPointCloudHelper,
				               this, sensor_origin, std::move(discretized),
				               std::move(occupied_hits), prob_miss_log, depth, simple_ray_casting,
				               early_stopping, min_change, max_change);
			} else {
				insertPointCloudHelper(sensor_origin, std::move(discretized),
				                       std::move(occupied_hits), prob_miss_log, depth,
				                       simple_ray_casting, early_stopping, min_change,
				                       max_change);
			}
		}
	}

	void updateValue(Code const& code, LogitType const& update, Color color)
	{
		auto path = Base::createNode(code);
		DepthType depth = code.getDepth();

		if (Base::isLeaf(path[depth], depth)) {
			updateNodeColor(*path[depth], color, toProb(update));

			if (updateOccupancy(path[depth]->value.occupancy, update)) {
				if (change_detection_enabled_) {
					changes_.insert(code);
				}
			}
		} else {
			// TODO: Error
		}

		Base::updateParents(path, depth);
	}

	template <typename T>
	void InsertPointCloudDiscrete(Point3 const& sensor_origin, PointCloudColor cloud,
	                              math::Pose6 const& frame_origin, double max_range = -1,
	                              DepthType depth = 0, bool simple_ray_casting = false,
	                              unsigned int early_stopping = 0, bool async = false)
	{
		cloud.transform(frame_origin, async);
		insertPointCloudDiscrete(sensor_origin, cloud, max_range, depth, simple_ray_casting,
		                         early_stopping, async);
	}

	//
	// Set color
	//

	void setColor(Code const& code, Color color);

	//
	// Get color
	//

	Color getColor(Code const& code) const;

 protected:
	//
	// Integrate colors
	//

	void integrateColors(Point3 const& sensor_origin, PointCloudColor const& cloud,
	                     double max_range = -1);

	//
	// Integrator helper
	//

	void insertPointCloudHelper(Point3 sensor_origin, PointCloud&& discretized,
	                            std::vector<std::tuple<Code, float, Color>>&& occupied_hits,
	                            LogitType prob_miss_log, DepthType depth,
	                            bool simple_ray_casting, unsigned int early_stopping,
	                            Point3 min_change, Point3 max_change)
	{
		std::future<void> f = std::async(std::launch::async, [this, &occupied_hits]() {
			std::for_each(begin(occupied_hits), end(occupied_hits), [this](auto&& hit) {
				updateValue(std::get<0>(hit), std::get<1>(hit), std::get<2>(hit));
			});
		});

		CodeMap<LogitType> free_hits;

		freeSpace(sensor_origin, discretized, free_hits, prob_miss_log, depth,
		          simple_ray_casting, early_stopping);

		f.wait();

		for (auto const& [code, value] : free_hits) {
			Base::updateValue(code, value);
		}

		if (min_max_change_detection_enabled_) {
			for (int i : {0, 1, 2}) {
				min_change_[i] = std::min(min_change_[i], min_change[i]);
				max_change_[i] = std::max(max_change_[i], max_change[i]);
			}
		}
	}

	//
	// Update node
	//

	virtual bool updateNode(INNER_NODE& node, DepthType depth) override;

	//
	// Update node color
	//

	void updateNodeColor(Code code, Color update);

	void updateNodeColor(LEAF_NODE& node, Color update, double prob);

	//
	// Average child color
	//

	Color getAverageChildColor(INNER_NODE const& node, DepthType depth) const;

	//
	// Average color
	//

	Color getAverageColor(std::vector<Color> const& colors) const;
};
}  // namespace ufo::map

#endif  // UFO_MAP_OCCUPANCY_MAP_COLOR_H