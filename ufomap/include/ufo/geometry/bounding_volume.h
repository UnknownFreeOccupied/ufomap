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

#ifndef UFO_GEOMETRY_BOUDING_VOLUME_H
#define UFO_GEOMETRY_BOUDING_VOLUME_H

#include <ufo/geometry/collision_checks.h>
#include <ufo/geometry/types.h>

namespace ufo::geometry
{
class BoundingVolume
{
 public:
	void add(BoundingVar const& bv) { bounding_volume_.push_back(bv); }

	size_t size() const { return bounding_volume_.size(); }

	bool empty() const { return bounding_volume_.empty(); }

	bool intersects(BoundingVar const& bv) const;

	bool intersects(BoundingVolume const& other) const;

	std::vector<BoundingVar>::iterator begin() { return bounding_volume_.begin(); }

	std::vector<BoundingVar>::const_iterator begin() const
	{
		return bounding_volume_.begin();
	}

	std::vector<BoundingVar>::const_iterator cbegin() const
	{
		return bounding_volume_.cbegin();
	}

	std::vector<BoundingVar>::iterator end() { return bounding_volume_.end(); }

	std::vector<BoundingVar>::const_iterator end() const { return bounding_volume_.end(); }

	std::vector<BoundingVar>::const_iterator cend() const
	{
		return bounding_volume_.cend();
	}

	std::vector<BoundingVar>::reverse_iterator rbegin()
	{
		return bounding_volume_.rbegin();
	}

	std::vector<BoundingVar>::const_reverse_iterator rbegin() const
	{
		return bounding_volume_.rbegin();
	}

	std::vector<BoundingVar>::const_reverse_iterator crbegin() const
	{
		return bounding_volume_.crbegin();
	}

	std::vector<BoundingVar>::reverse_iterator rend() { return bounding_volume_.rend(); }

	std::vector<BoundingVar>::const_reverse_iterator rend() const
	{
		return bounding_volume_.rend();
	}

	std::vector<BoundingVar>::const_reverse_iterator crend() const
	{
		return bounding_volume_.crend();
	}

 private:
	std::vector<BoundingVar> bounding_volume_;
};
}  // namespace ufo::geometry

#endif  // UFO_GEOMETRY_BOUDING_VOLUME_H