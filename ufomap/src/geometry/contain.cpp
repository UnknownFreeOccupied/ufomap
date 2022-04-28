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

// UFO
#include <ufo/geometry/contain.h>

namespace ufo::geometry
{
//
// AABB
//

bool contain(AABB const& aabb_1, AABB const& aabb_2)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(AABB const& aabb, Frustum const& frustum)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(AABB const& aabb, LineSegment const& line_segment)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(AABB const& aabb, OBB const& obb)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(AABB const& aabb, Plane const& plane)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(AABB const& aabb, Point const& point)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(AABB const& aabb, Ray const& ray)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(AABB const& aabb, Sphere const& sphere)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

//
// Frustum
//

bool contain(Frustum const& frustum, AABB const& aabb)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Frustum const& frustum_1, Frustum const& frustum_2)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Frustum const& frustum, LineSegment const& line_segment)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Frustum const& frustum, OBB const& obb)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Frustum const& frustum, Plane const& plane)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Frustum const& frustum, Point const& point)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Frustum const& frustum, Ray const& ray)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Frustum const& frustum, Sphere const& sphere)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

//
// Line segment
//

bool contain(LineSegment const& line_segment, AABB const& aabb)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(LineSegment const& line_segment, Frustum const& frustum)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(LineSegment const& line_segment_1, LineSegment const& line_segment_2)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(LineSegment const& line_segment, OBB const& obb)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(LineSegment const& line_segment, Plane const& plane)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(LineSegment const& line_segment, Point const& point)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(LineSegment const& line_segment, Ray const& ray)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(LineSegment const& line_segment, Sphere const& sphere)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

//
// OBB
//

bool contain(OBB const& obb, AABB const& aabb)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(OBB const& obb, Frustum const& frustum)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(OBB const& obb, LineSegment const& line_segment)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(OBB const& obb_1, OBB const& obb_2)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(OBB const& obb, Plane const& plane)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(OBB const& obb, Point const& point)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(OBB const& obb, Ray const& ray)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(OBB const& obb, Sphere const& sphere)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

//
// Plane
//

bool contain(Plane const& plane, AABB const& aabb)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Plane const& plane, Frustum const& frustum)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Plane const& plane, LineSegment const& line_segment)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Plane const& plane, OBB const& obb)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Plane const& plane_1, Plane const& plane_2)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Plane const& plane, Point const& point)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Plane const& plane, Ray const& ray)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Plane const& plane, Sphere const& sphere)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

//
// Point
//

bool contain(Point const& point, AABB const& aabb)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Point const& point, Frustum const& frustum)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Point const& point, LineSegment const& line_segment)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Point const& point, OBB const& obb)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Point const& point, Plane const& plane)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Point const& point_1, Point const& point_2)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Point const& point, Ray const& ray)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Point const& point, Sphere const& sphere)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

//
// Ray
//

bool contain(Ray const& ray, AABB const& aabb)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Ray const& ray, Frustum const& frustum)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Ray const& ray, LineSegment const& line_segment)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Ray const& ray, OBB const& obb)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Ray const& ray, Plane const& plane)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Ray const& ray, Point const& point)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Ray const& ray_1, Ray const& ray_2)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Ray const& ray, Sphere const& sphere)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

//
// Sphere
//

bool contain(Sphere const& sphere, AABB const& aabb)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Sphere const& sphere, Frustum const& frustum)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Sphere const& sphere, LineSegment const& line_segment)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Sphere const& sphere, OBB const& obb)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Sphere const& sphere, Plane const& plane)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Sphere const& sphere, Point const& point)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Sphere const& sphere, Ray const& ray)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

bool contain(Sphere const& sphere_1, Sphere const& sphere_2)
{
	throw std::logic_error("Function not yet implemented.");
	// TODO: Implement
}

}  // namespace ufo::geometry