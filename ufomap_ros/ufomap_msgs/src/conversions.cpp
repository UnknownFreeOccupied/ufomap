/**
 * UFOMap ROS message conversions
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufo_ros
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

#include <ufomap_msgs/conversions.h>

namespace ufomap_msgs
{
ufo::geometry::Point msgToUfo(ufomap_msgs::Point const& point)
{
	return ufo::geometry::Point(point.x, point.y, point.z);
}

ufo::geometry::AABB msgToUfo(ufomap_msgs::AABB const& aabb)
{
	ufo::geometry::AABB a;
	a.center = msgToUfo(aabb.center);
	a.half_size = msgToUfo(aabb.half_size);
	return a;
}

ufo::geometry::Plane msgToUfo(ufomap_msgs::Plane const& plane)
{
	return ufo::geometry::Plane(msgToUfo(plane.normal), plane.distance);
}

ufo::geometry::Frustum msgToUfo(ufomap_msgs::Frustum const& frustum)
{
	ufo::geometry::Frustum f;
	for (size_t i = 0; i < frustum.planes.size(); ++i) {
		f.planes[i] = msgToUfo(frustum.planes[i]);
	}
	return f;
}

ufo::geometry::LineSegment msgToUfo(ufomap_msgs::LineSegment const& line_segment)
{
	return ufo::geometry::LineSegment(msgToUfo(line_segment.start),
	                                  msgToUfo(line_segment.end));
}

ufo::geometry::OBB msgToUfo(ufomap_msgs::OBB const& obb)
{
	return ufo::geometry::OBB(msgToUfo(obb.center), msgToUfo(obb.half_size),
	                          msgToUfo(obb.rotation));
}

ufo::geometry::Ray msgToUfo(ufomap_msgs::Ray const& ray)
{
	return ufo::geometry::Ray(msgToUfo(ray.origin), msgToUfo(ray.direction));
}

ufo::geometry::Sphere msgToUfo(ufomap_msgs::Sphere const& sphere)
{
	return ufo::geometry::Sphere(msgToUfo(sphere.center), sphere.radius);
}

ufo::geometry::BoundingVolume msgToUfo(ufomap_msgs::BoundingVolume const& msg)
{
	ufo::geometry::BoundingVolume bv;
	for (ufomap_msgs::AABB const& aabb : msg.aabbs) {
		bv.add(msgToUfo(aabb));
	}
	for (ufomap_msgs::Frustum const& frustum : msg.frustums) {
		bv.add(msgToUfo(frustum));
	}
	for (ufomap_msgs::LineSegment const& line_segment : msg.line_segments) {
		bv.add(msgToUfo(line_segment));
	}
	for (ufomap_msgs::OBB const& obb : msg.obbs) {
		bv.add(msgToUfo(obb));
	}
	for (ufomap_msgs::Plane const& plane : msg.planes) {
		bv.add(msgToUfo(plane));
	}
	for (ufomap_msgs::Point const& point : msg.points) {
		bv.add(msgToUfo(point));
	}
	for (ufomap_msgs::Ray const& ray : msg.rays) {
		bv.add(msgToUfo(ray));
	}
	for (ufomap_msgs::Sphere const& sphere : msg.spheres) {
		bv.add(msgToUfo(sphere));
	}
	return bv;
}

//
// UFOMap type to ROS message type
//

ufomap_msgs::Point ufoToMsg(ufo::geometry::Point const& point)
{
	ufomap_msgs::Point msg;
	msg.x = point.x();
	msg.y = point.y();
	msg.z = point.z();
	return msg;
}

ufomap_msgs::AABB ufoToMsg(ufo::geometry::AABB const& aabb)
{
	ufomap_msgs::AABB msg;
	msg.center = ufoToMsg(aabb.center);
	msg.half_size = ufoToMsg(aabb.half_size);
	return msg;
}

ufomap_msgs::Plane ufoToMsg(ufo::geometry::Plane const& plane)
{
	ufomap_msgs::Plane msg;
	msg.normal = ufoToMsg(plane.normal);
	msg.distance = plane.distance;
	return msg;
}

ufomap_msgs::Frustum ufoToMsg(ufo::geometry::Frustum const& frustum)
{
	ufomap_msgs::Frustum msg;
	for (size_t i = 0; i < msg.planes.size(); ++i) {
		msg.planes[i] = ufoToMsg(frustum.planes[i]);
	}
	return msg;
}

ufomap_msgs::LineSegment ufoToMsg(ufo::geometry::LineSegment const& line_segment)
{
	ufomap_msgs::LineSegment msg;
	msg.start = ufoToMsg(line_segment.start);
	msg.end = ufoToMsg(line_segment.end);
	return msg;
}

ufomap_msgs::OBB ufoToMsg(ufo::geometry::OBB const& obb)
{
	ufomap_msgs::OBB msg;
	msg.center = ufoToMsg(obb.center);
	msg.half_size = ufoToMsg(obb.half_size);
	// TODO: Fix
	// msg.rotation = ufoToMsg(obb.rotation);
	return msg;
}

ufomap_msgs::Ray ufoToMsg(ufo::geometry::Ray const& ray)
{
	ufomap_msgs::Ray msg;
	msg.origin = ufoToMsg(ray.origin);
	msg.direction = ufoToMsg(ray.direction);
	return msg;
}

ufomap_msgs::Sphere ufoToMsg(ufo::geometry::Sphere const& sphere)
{
	ufomap_msgs::Sphere msg;
	msg.center = ufoToMsg(sphere.center);
	msg.radius = sphere.radius;
	return msg;
}

ufomap_msgs::BoundingVolume ufoToMsg(ufo::geometry::BoundingVolume const& bounding_volume)
{
	ufomap_msgs::BoundingVolume msg;
	for (ufo::geometry::BoundingVar const& bv : bounding_volume) {
		std::visit(
		    [&msg](auto&& arg) -> void {
			    using T = std::decay_t<decltype(arg)>;
			    if constexpr (std::is_same_v<T, ufo::geometry::AABB>) {
				    msg.aabbs.push_back(ufoToMsg(arg));
			    } else if constexpr (std::is_same_v<T, ufo::geometry::Frustum>) {
				    msg.frustums.push_back(ufoToMsg(arg));
			    } else if constexpr (std::is_same_v<T, ufo::geometry::LineSegment>) {
				    msg.line_segments.push_back(ufoToMsg(arg));
			    } else if constexpr (std::is_same_v<T, ufo::geometry::OBB>) {
				    msg.obbs.push_back(ufoToMsg(arg));
			    } else if constexpr (std::is_same_v<T, ufo::geometry::Plane>) {
				    msg.planes.push_back(ufoToMsg(arg));
			    } else if constexpr (std::is_same_v<T, ufo::math::Vector3>) {
				    msg.points.push_back(ufoToMsg(arg));
			    } else if constexpr (std::is_same_v<T, ufo::geometry::Ray>) {
				    msg.rays.push_back(ufoToMsg(arg));
			    } else if constexpr (std::is_same_v<T, ufo::geometry::Sphere>) {
				    msg.spheres.push_back(ufoToMsg(arg));
			    }
		    },
		    bv);
	}
	return msg;
}
}  // namespace ufomap_msgs