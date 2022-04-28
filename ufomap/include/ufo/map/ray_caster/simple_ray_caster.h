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

#ifndef UFO_MAP_SIMPLE_RAY_CASTER_H
#define UFO_MAP_SIMPLE_RAY_CASTER_H

// UFO
#include <ufo/map/code.h>
#include <ufo/map/key.h>
#include <ufo/map/types.h>

// STL
#include <exception>
#include <numeric>

namespace ufo::map
{
class SimpleRayCaster
{
 public:
	SimpleRayCaster(Point3 origin, Point3 to, float node_size)
	    : current_(origin), end_(to), current_step_(0), current_distance_(0.0)
	{
		Point3 direction = end_ - current_;
		total_distance_ = direction.norm();
		direction /= total_distance_;

		total_num_steps_ = total_distance_ / node_size;
		step_ = direction * node_size;
		distance_per_step_ = total_distance_ / static_cast<float>(total_num_steps_);
	}

	SimpleRayCaster(SimpleRayCaster const& rhs) = default;

	SimpleRayCaster(SimpleRayCaster&& rhs) = default;

	SimpleRayCaster& operator=(SimpleRayCaster const& rhs) = default;

	SimpleRayCaster& operator=(SimpleRayCaster&& rhs) = default;

	void takeStep()
	{
		current_ += step_;
		current_distance_ += distance_per_step_;
		current_step_ += 1;
	}

	Point3 getCurrent() const { return current_; }

	Point3 getEnd() const { return end_; }

	float distanceMoved() const { return current_distance_; }

	float distanceLeft() const { return total_distance_ - distanceMoved(); }

	float totalDistance() const { return total_distance_; }

	bool hasLeft() const
	{
		return current_step_ != total_num_steps_ && distanceMoved() <= totalDistance();
	}

 private:
	Point3 current_;
	Point3 end_;
	Point3 step_;
	size_t current_step_;
	size_t total_num_steps_;
	float distance_per_step_;
	float current_distance_;
	float total_distance_;
};
}  // namespace ufo::map

#endif  // UFO_MAP_SIMPLE_RAY_CASTER_H