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

#ifndef UFO_MAP_COUNT_NODE_H
#define UFO_MAP_COUNT_NODE_H

// UFO
#include <ufo/map/types.h>

// STL
#include <array>

namespace ufo::map
{
template <std::size_t N = 8>
struct CountNode {
	//
	// Size
	//

	[[nodiscard]] static constexpr std::size_t countSize() { return N; }

	//
	// Data
	//

	[[nodiscard]] constexpr count_t const* countData() const noexcept
	{
		return count_.data();
	}

	[[nodiscard]] constexpr count_t* countData() noexcept { return count_.data(); }

	//
	// Iterators
	//

	[[nodiscard]] constexpr auto beginCount() noexcept { return count_.begin(); }

	[[nodiscard]] constexpr auto beginCount() const noexcept { return count_.begin(); }

	[[nodiscard]] constexpr auto cbeginCount() const noexcept { return count_.cbegin(); }

	[[nodiscard]] constexpr auto endCount() noexcept { return count_.end(); }

	[[nodiscard]] constexpr auto endCount() const noexcept { return count_.end(); }

	[[nodiscard]] constexpr auto cendCount() const noexcept { return count_.cend(); }

	[[nodiscard]] constexpr auto rbeginCount() noexcept { return count_.rbegin(); }

	[[nodiscard]] constexpr auto rbeginCount() const noexcept { return count_.rbegin(); }

	[[nodiscard]] constexpr auto crbeginCount() const noexcept { return count_.crbegin(); }

	[[nodiscard]] constexpr auto rendCount() noexcept { return count_.rend(); }

	[[nodiscard]] constexpr auto rendCount() const noexcept { return count_.rend(); }

	[[nodiscard]] constexpr auto crendCount() const noexcept { return count_.crend(); }

	//
	// Fill
	//

	void fill(CountNode const parent, index_t const index)
	{
		setCount(parent.count(index));
	}

	//
	// Is collapsible
	//

	[[nodiscard]] constexpr bool isCollapsible(CountNode const parent,
	                                           index_t const index) const
	{
		return all_of(count_, [a = parent.count(index)](auto const b) { return a == b; });
	}

	//
	// Get count
	//

	[[nodiscard]] constexpr count_t count(index_t const index) const
	{
		if constexpr (1 == N) {
			return count_[0];
		} else {
			return count_[index];
		}
	}

	//
	// Set count
	//

	void setCount(count_t const value) { count_.fill(value); }

	void setCount(index_t const index, count_t const value)
	{
		if constexpr (1 == N) {
			setCount(value);
		} else {
			count_[index] = value;
		}
	}

 private:
	// Data
	std::array<count_t, N> count_;
};
}  // namespace ufo::map

#endif  // UFO_MAP_COUNT_NODE_H