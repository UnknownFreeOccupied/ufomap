/*!
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author Daniel Duberg (dduberg@kth.se)
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * @version 1.0
 * @date 2022-05-13
 *
 * @copyright Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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

#ifndef UFO_MAP_TIME_NODE_H
#define UFO_MAP_TIME_NODE_H

// UFO
#include <ufo/algorithm/algorithm.h>
#include <ufo/map/types.h>

// STL
#include <array>

namespace ufo::map
{
template <std::size_t N>
struct TimeNode {
	// Data
	std::array<time_t, N> time;

	//
	// Size
	//

	[[nodiscard]] static constexpr std::size_t timeSize() { return N; }

	//
	// Fill
	//

	void fill(TimeNode const parent, index_t const index)
	{
		if constexpr (1 == N) {
			time = parent.time;
		} else {
			time.fill(parent.time[index]);
		}
	}

	//
	// Is collapsible
	//

	[[nodiscard]] constexpr bool isCollapsible(TimeNode const parent,
	                                           index_t const index) const
	{
		if constexpr (1 == N) {
			return time == parent.time;
		} else {
			return all_of(time, [t = parent.time[index]](auto const e) { return e == t; });
		}
	}

	//
	// Get time
	//

	[[nodiscard]] constexpr time_t timeIndex(index_t const index) const
	{
		if constexpr (1 == N) {
			return time[0];
		} else {
			return time[index];
		}
	}

	//
	// Set time
	//

	void setTime(time_t const value) { time.fill(value); }

	void setTimeIndex(index_t const index, time_t const value)
	{
		if constexpr (1 == N) {
			setTime(value);
		} else {
			time[index] = value;
		}
	}
};
}  // namespace ufo::map

#endif  // UFO_MAP_TIME_NODE_H