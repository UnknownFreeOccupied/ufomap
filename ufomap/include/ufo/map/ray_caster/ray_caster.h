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

#ifndef UFO_MAP_RAY_CASTER_H
#define UFO_MAP_RAY_CASTER_H

// UFO
#include <ufo/map/code.h>
#include <ufo/map/key.h>
#include <ufo/map/types.h>

// STL
#include <exception>
#include <numeric>

namespace ufo::map
{
class RayCaster
{
 public:
	RayCaster(Key current, Key end, Point3 origin, Point3 to, Point3 voxel_border,
	          float node_size)
	    : current_(current), end_(end)
	{
		if (current.getDepth() != end.getDepth()) {
			throw std::invalid_argument("Current and end need to be on same depth.");
		}

		if (current == end) {
			total_distance_ = 0.0;
			for (size_t i = 0; i != 3; ++i) {
				step_[i] = 0;
				t_delta_[i] = std::numeric_limits<float>::max();
				t_max_[i] = std::numeric_limits<float>::max();
			}
			return;
		}

		DepthType depth = current.getDepth();

		Point3 direction = to - origin;
		total_distance_ = direction.norm();
		direction /= total_distance_;
		// // Add some extra
		total_distance_ += 1 * node_size;

		for (size_t i = 0; i != 3; ++i) {
			if (0 == direction[i]) {
				step_[i] = 0;
				t_delta_[i] = std::numeric_limits<float>::max();
				t_max_[i] = std::numeric_limits<float>::max();
				continue;
			}

			if (0 < direction[i]) {
				step_[i] = static_cast<int>(1U << depth);
				voxel_border[i] += (node_size / 2.0);
			} else {
				step_[i] = -static_cast<int>(1U << depth);
				voxel_border[i] -= (node_size / 2.0);
			}

			t_delta_[i] = node_size / std::abs(direction[i]);
			t_max_[i] = voxel_border[i] / direction[i];
		}
	}

	RayCaster(RayCaster const& rhs) = default;

	RayCaster(RayCaster&& rhs) = default;

	RayCaster& operator=(RayCaster const& rhs) = default;

	RayCaster& operator=(RayCaster&& rhs) = default;

	void takeStep()
	{
		size_t advance_dim = t_max_.minElementIndex();
		current_[advance_dim] += step_[advance_dim];
		t_max_[advance_dim] += t_delta_[advance_dim];
	}

	Code getCurrent() const { return Code(current_); }

	Key getCurrentKey() const { return current_; }

	Code getEnd() const { return Code(end_); }

	Key getEndKey() const { return end_; }

	float distanceMoved() const { return t_max_.min(); }

	float distanceLeft() const { return total_distance_ - distanceMoved(); }

	float totalDistance() const { return total_distance_; }

	bool hasLeft() const { return current_ != end_ && distanceMoved() <= totalDistance(); }

 private:
	Key current_;
	Key end_;
	std::array<int, 3> step_;
	Point3 t_delta_;
	Point3 t_max_;
	float total_distance_;
};
}  // namespace ufo::map

#endif  // UFO_MAP_RAY_CASTER_H