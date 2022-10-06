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
	//
	// Size
	//

	[[nodiscard]] static constexpr std::size_t timeSize() { return N; }

	//
	// Data
	//

	[[nodiscard]] constexpr time_t const* timeData() const noexcept { return time_.data(); }

	[[nodiscard]] constexpr time_t* timeData() noexcept { return time_.data(); }

	//
	// Iterators
	//

	[[nodiscard]] constexpr auto beginTime() noexcept { return time_.begin(); }

	[[nodiscard]] constexpr auto beginTime() const noexcept { return time_.begin(); }

	[[nodiscard]] constexpr auto cbeginTime() const noexcept { return time_.cbegin(); }

	[[nodiscard]] constexpr auto endTime() noexcept { return time_.end(); }

	[[nodiscard]] constexpr auto endTime() const noexcept { return time_.end(); }

	[[nodiscard]] constexpr auto cendTime() const noexcept { return time_.cend(); }

	[[nodiscard]] constexpr auto rbeginTime() noexcept { return time_.rbegin(); }

	[[nodiscard]] constexpr auto rbeginTime() const noexcept { return time_.rbegin(); }

	[[nodiscard]] constexpr auto crbeginTime() const noexcept { return time_.crbegin(); }

	[[nodiscard]] constexpr auto rendTime() noexcept { return time_.rend(); }

	[[nodiscard]] constexpr auto rendTime() const noexcept { return time_.rend(); }

	[[nodiscard]] constexpr auto crendTime() const noexcept { return time_.crend(); }

	//
	// Fill
	//

	void fill(TimeNode const parent, index_t const index) { setTime(parent.time(index)); }

	//
	// Is collapsible
	//

	[[nodiscard]] constexpr bool isCollapsible(TimeNode const parent,
	                                           index_t const index) const
	{
		return all_of(time_, [t = parent.time(index)](auto const e) { return e == t; });
	}

	//
	// Get time
	//

	[[nodiscard]] constexpr time_t time(index_t const index) const
	{
		if constexpr (1 == N) {
			return time_[0];
		} else {
			return time_[index];
		}
	}

	//
	// Set time
	//

	void setTime(time_t const value) { time_.fill(value); }

	void setTime(index_t const index, time_t const value)
	{
		if constexpr (1 == N) {
			setTime(value);
		} else {
			time_[index] = value;
		}
	}

 private:
	// Data
	std::array<time_t, N> time_;
};
}  // namespace ufo::map

#endif  // UFO_MAP_TIME_NODE_H