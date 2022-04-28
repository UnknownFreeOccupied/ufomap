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

#ifndef UFOMAP_ROS_MSGS_CONVERSIONS_H
#define UFOMAP_ROS_MSGS_CONVERSIONS_H

// UFO
#include <ufo/geometry/aabb.h>
#include <ufo/geometry/bounding_volume.h>
#include <ufo/geometry/frustum.h>
#include <ufo/geometry/line_segment.h>
#include <ufo/geometry/obb.h>
#include <ufo/geometry/plane.h>
#include <ufo/geometry/point.h>
#include <ufo/geometry/ray.h>
#include <ufo/geometry/sphere.h>
#include <ufo/geometry/types.h>
#include <ufo/map/io.h>
#include <ufo/map/predicate/predicates.h>
#include <ufo/map/types.h>

// UFO msg
#include <ufomap_msgs/AABB.h>
#include <ufomap_msgs/BoundingVolume.h>
#include <ufomap_msgs/Frustum.h>
#include <ufomap_msgs/LineSegment.h>
#include <ufomap_msgs/OBB.h>
#include <ufomap_msgs/Plane.h>
#include <ufomap_msgs/Point.h>
#include <ufomap_msgs/Ray.h>
#include <ufomap_msgs/Sphere.h>
#include <ufomap_msgs/UFOMap.h>

// STL
#include <optional>
#include <type_traits>

namespace ufomap_msgs
{
//
// ROS message type to UFO type
//

ufo::geometry::Point msgToUfo(ufomap_msgs::Point const& point);

ufo::geometry::AABB msgToUfo(ufomap_msgs::AABB const& aabb);

ufo::geometry::Plane msgToUfo(ufomap_msgs::Plane const& plane);

ufo::geometry::Frustum msgToUfo(ufomap_msgs::Frustum const& frustum);

ufo::geometry::LineSegment msgToUfo(ufomap_msgs::LineSegment const& line_segment);

ufo::geometry::OBB msgToUfo(ufomap_msgs::OBB const& obb);

ufo::geometry::Ray msgToUfo(ufomap_msgs::Ray const& ray);

ufo::geometry::Sphere msgToUfo(ufomap_msgs::Sphere const& sphere);

ufo::geometry::BoundingVolume msgToUfo(ufomap_msgs::BoundingVolume const& msg);

//
// UFO type to ROS message type
//

ufomap_msgs::Point ufoToMsg(ufo::geometry::Point const& point);

ufomap_msgs::AABB ufoToMsg(ufo::geometry::AABB const& aabb);

ufomap_msgs::Plane ufoToMsg(ufo::geometry::Plane const& plane);

ufomap_msgs::Frustum ufoToMsg(ufo::geometry::Frustum const& frustum);

ufomap_msgs::LineSegment ufoToMsg(ufo::geometry::LineSegment const& line_segment);

ufomap_msgs::OBB ufoToMsg(ufo::geometry::OBB const& obb);

ufomap_msgs::Ray ufoToMsg(ufo::geometry::Ray const& ray);

ufomap_msgs::Sphere ufoToMsg(ufo::geometry::Sphere const& sphere);

ufomap_msgs::BoundingVolume ufoToMsg(
    ufo::geometry::BoundingVolume const& bounding_volume);

//
// ROS message type to UFO type
//

template <class Map>
void msgToUfo(ufomap_msgs::UFOMap const& msg, Map& map, bool propagate = true)
{
	if (msg.data.empty()) {
		return;
	}

	std::stringstream data(std::ios_base::in | std::ios_base::out | std::ios_base::binary);
	data.exceptions(std::stringstream::failbit | std::stringstream::badbit);

	data.write(reinterpret_cast<char const*>(msg.data.data()),
	           sizeof(typename decltype(msg.data)::value_type) * msg.data.size());

	map.read(data, propagate);
}

//
// UFO type to ROS message type
//

template <class Map, class Predicates,
          typename = std::enable_if_t<!std::is_scalar_v<Predicates>>>
ufomap_msgs::UFOMap ufoToMsg(Map const& map, Predicates const& predicates,
                             ufo::map::DepthType depth = 0, bool compress = false,
                             int compression_acceleration_level = 1,
                             int compression_level = 0)
{
	std::stringstream data(std::ios_base::in | std::ios_base::out | std::ios_base::binary);
	data.exceptions(std::stringstream::failbit | std::stringstream::badbit);

	map.write(data, predicates, depth, compress, compression_acceleration_level,
	          compression_level);

	ufomap_msgs::UFOMap msg;
	msg.data.resize(data.tellp());
	data.seekg(0, std::ios_base::beg);
	data.read(reinterpret_cast<char*>(msg.data.data()), msg.data.size());
	return msg;
}

template <class Map>
ufomap_msgs::UFOMap ufoToMsg(Map const& map, ufo::map::DepthType depth = 0,
                             bool compress = false,
                             int compression_acceleration_level = 1,
                             int compression_level = 0)
{
	return ufoToMsg(map, ufo::map::predicate::TRUE(), depth, compress,
	                compression_acceleration_level, compression_level);
}

}  // namespace ufomap_msgs

#endif  // UFOMAP_ROS_MSGS_CONVERSIONS_H