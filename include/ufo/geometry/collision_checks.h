/**
 * UFOGeometry - the geometry library used in UFO
 * 
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufogeometry
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

#ifndef UFO_GEOMETRY_COLLISION_CHECKS_H
#define UFO_GEOMETRY_COLLISION_CHECKS_H

#include <ufo/geometry/aabb.h>
#include <ufo/geometry/bounding_volume.h>
// #include <ufo/geometry/capsule.h>
#include <ufo/geometry/frustum.h>
#include <ufo/geometry/line_segment.h>
#include <ufo/geometry/obb.h>
#include <ufo/geometry/plane.h>
#include <ufo/geometry/ray.h>
#include <ufo/geometry/sphere.h>

namespace ufo::geometry {
/////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Intersection tests
//////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

// AABB
bool intersects(const AABB& aabb_1, const AABB& aabb_2);
// bool intersects(const AABB& aabb, const Capsule& capsule);
bool intersects(const AABB& aabb, const Frustum& frustum);
bool intersects(const AABB& aabb, const LineSegment& line_segment);
bool intersects(const AABB& aabb, const OBB& obb);
bool intersects(const AABB& aabb, const Plane& plane);
bool intersects(const AABB& aabb, const Point& point);
bool intersects(const AABB& aabb, const Ray& ray);
bool intersects(const AABB& aabb, const Sphere& sphere);

// TODO: Capsule
// bool intersects(const Capsule& capsule, const AABB& aabb);
// bool intersects(const Capsule& capsule_1, const Capsule& capsule_2);
// bool intersects(const Capsule& capsule, const Frustum& frustum);
// bool intersects(const Capsule& capsule, const LineSegment& line_segment);
// bool intersects(const Capsule& capsule, const OBB& obb);
// bool intersects(const Capsule& capsule, const Plane& plane);
// bool intersects(const Capsule& capsule, const Point& point);
// bool intersects(const Capsule& capsule, const Ray& ray);
// bool intersects(const Capsule& capsule, const Sphere& sphere);

// TODO: Cone

// TODO: Cylinder

// TODO: Ellipsoid

// Frustum
bool intersects(const Frustum& frustum, const AABB& aabb);
// bool intersects(const Frustum& frustum, const Capsule& capsule);
bool intersects(const Frustum& frustum_1, const Frustum& frustum_2);
bool intersects(const Frustum& frustum, const LineSegment& line_segment);
bool intersects(const Frustum& frustum, const OBB& obb);
bool intersects(const Frustum& frustum, const Plane& plane);
bool intersects(const Frustum& frustum, const Point& point);
bool intersects(const Frustum& frustum, const Ray& ray);
bool intersects(const Frustum& frustum, const Sphere& sphere);

// Line segment
bool intersects(const LineSegment& line_segment, const AABB& aabb);
// bool intersects(const LineSegment& line_segment, const Capsule& capsule);
bool intersects(const LineSegment& line_segment, const Frustum& frustum);
bool intersects(const LineSegment& line_segment_1,
                const LineSegment& line_segment_2);
bool intersects(const LineSegment& line_segment, const OBB& obb);
bool intersects(const LineSegment& line_segment, const Plane& plane);
bool intersects(const LineSegment& line_segment, const Point& point);
bool intersects(const LineSegment& line_segment, const Ray& ray);
bool intersects(const LineSegment& line_segment, const Sphere& sphere);

// OBB
bool intersects(const OBB& obb, const AABB& aabb);
// bool intersects(const OBB& obb, const Capsule& capsule);
bool intersects(const OBB& obb, const Frustum& frustum);
bool intersects(const OBB& obb, const LineSegment& line_segment);
bool intersects(const OBB& obb_1, const OBB& obb_2);
bool intersects(const OBB& obb, const Plane& plane);
bool intersects(const OBB& obb, const Point& point);
bool intersects(const OBB& obb, const Ray& ray);
bool intersects(const OBB& obb, const Sphere& sphere);

// Plane
bool intersects(const Plane& plane, const AABB& aabb);
// bool intersects(const Plane& plane, const Capsule& capsule);
bool intersects(const Plane& plane, const Frustum& frustum);
bool intersects(const Plane& plane, const LineSegment& line_segment);
bool intersects(const Plane& plane, const OBB& obb);
bool intersects(const Plane& plane_1, const Plane& plane_2);
bool intersects(const Plane& plane, const Point& point);
bool intersects(const Plane& plane, const Ray& ray);
bool intersects(const Plane& plane, const Sphere& sphere);

// Point
bool intersects(const Point& point, const AABB& aabb);
// bool intersects(const Point& point, const Capsule& capsule);
bool intersects(const Point& point, const Frustum& frustum);
bool intersects(const Point& point, const LineSegment& line_segment);
bool intersects(const Point& point, const OBB& obb);
bool intersects(const Point& point, const Plane& plane);
bool intersects(const Point& point_1, const Point& point_2);
bool intersects(const Point& point, const Ray& ray);
bool intersects(const Point& point, const Sphere& sphere);

// Ray
bool intersects(const Ray& ray, const AABB& aabb);
// bool intersects(const Ray& ray, const Capsule& capsule);
bool intersects(const Ray& ray, const Frustum& frustum);
bool intersects(const Ray& ray, const LineSegment& line_segment);
bool intersects(const Ray& ray, const OBB& obb);
bool intersects(const Ray& ray, const Plane& plane);
bool intersects(const Ray& ray, const Point& point);
bool intersects(const Ray& ray_1, const Ray& ray_2);
bool intersects(const Ray& ray, const Sphere& sphere);

// Sphere
bool intersects(const Sphere& sphere, const AABB& aabb);
// bool intersects(const Sphere& sphere, const Capsule& capsule);
bool intersects(const Sphere& sphere, const Frustum& frustum);
bool intersects(const Sphere& sphere, const LineSegment& line_segment);
bool intersects(const Sphere& sphere, const OBB& obb);
bool intersects(const Sphere& sphere, const Plane& plane);
bool intersects(const Sphere& sphere, const Point& point);
bool intersects(const Sphere& sphere, const Ray& ray);
bool intersects(const Sphere& sphere_1, const Sphere& sphere_2);

// TODO: Triangle

/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// Inside tests
/////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

// AABB
bool inside(const AABB& aabb_1, const AABB& aabb_2);
bool inside(const AABB& aabb, const Frustum& frustum);
bool inside(const AABB& aabb, const LineSegment& line_segment);
bool inside(const AABB& aabb, const OBB& obb);
bool inside(const AABB& aabb, const Plane& plane);
bool inside(const AABB& aabb, const Point& point);
bool inside(const AABB& aabb, const Ray& ray);
bool inside(const AABB& aabb, const Sphere& sphere);

// TODO: Capsule

// TODO: Cone

// TODO: Cylinder

// TODO: Ellipsoid

// Frustum
bool inside(const Frustum& frustum, const AABB& aabb);
bool inside(const Frustum& frustum_1, const Frustum& frustum_2);
bool inside(const Frustum& frustum, const LineSegment& line_segment);
bool inside(const Frustum& frustum, const OBB& obb);
bool inside(const Frustum& frustum, const Plane& plane);
bool inside(const Frustum& frustum, const Point& point);
bool inside(const Frustum& frustum, const Ray& ray);
bool inside(const Frustum& frustum, const Sphere& sphere);

// Line segment
bool inside(const LineSegment& line_segment, const AABB& aabb);
bool inside(const LineSegment& line_segment, const Frustum& frustum);
bool inside(const LineSegment& line_segment_1,
            const LineSegment& line_segment_2);
bool inside(const LineSegment& line_segment, const OBB& obb);
bool inside(const LineSegment& line_segment, const Plane& plane);
bool inside(const LineSegment& line_segment, const Point& point);
bool inside(const LineSegment& line_segment, const Ray& ray);
bool inside(const LineSegment& line_segment, const Sphere& sphere);

// OBB
bool inside(const OBB& obb, const AABB& aabb);
bool inside(const OBB& obb, const Frustum& frustum);
bool inside(const OBB& obb, const LineSegment& line_segment);
bool inside(const OBB& obb_1, const OBB& obb_2);
bool inside(const OBB& obb, const Plane& plane);
bool inside(const OBB& obb, const Point& point);
bool inside(const OBB& obb, const Ray& ray);
bool inside(const OBB& obb, const Sphere& sphere);

// Plane
bool inside(const Plane& plane, const AABB& aabb);
bool inside(const Plane& plane, const Frustum& frustum);
bool inside(const Plane& plane, const LineSegment& line_segment);
bool inside(const Plane& plane, const OBB& obb);
bool inside(const Plane& plane_1, const Plane& plane_2);
bool inside(const Plane& plane, const Point& point);
bool inside(const Plane& plane, const Ray& ray);
bool inside(const Plane& plane, const Sphere& sphere);

// Point
bool inside(const Point& point, const AABB& aabb);
bool inside(const Point& point, const Frustum& frustum);
bool inside(const Point& point, const LineSegment& line_segment);
bool inside(const Point& point, const OBB& obb);
bool inside(const Point& point, const Plane& plane);
bool inside(const Point& point_1, const Point& point_2);
bool inside(const Point& point, const Ray& ray);
bool inside(const Point& point, const Sphere& sphere);

// Ray
bool inside(const Ray& ray, const AABB& aabb);
bool inside(const Ray& ray, const Frustum& frustum);
bool inside(const Ray& ray, const LineSegment& line_segment);
bool inside(const Ray& ray, const OBB& obb);
bool inside(const Ray& ray, const Plane& plane);
bool inside(const Ray& ray, const Point& point);
bool inside(const Ray& ray_1, const Ray& ray_2);
bool inside(const Ray& ray, const Sphere& sphere);

// Sphere
bool inside(const Sphere& sphere, const AABB& aabb);
bool inside(const Sphere& sphere, const Frustum& frustum);
bool inside(const Sphere& sphere, const LineSegment& line_segment);
bool inside(const Sphere& sphere, const OBB& obb);
bool inside(const Sphere& sphere, const Plane& plane);
bool inside(const Sphere& sphere, const Point& point);
bool inside(const Sphere& sphere, const Ray& ray);
bool inside(const Sphere& sphere_1, const Sphere& sphere_2);

// TODO: Triangle

}  // namespace ufo::geometry

#endif  // UFO_GEOMETRY_COLLISION_CHECKS_H