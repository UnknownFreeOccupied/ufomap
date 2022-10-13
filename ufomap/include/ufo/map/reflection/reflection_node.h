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
 *     this software without specific prior written permissesion.
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

#ifndef UFO_MAP_REFLECTION_NODE_H
#define UFO_MAP_REFLECTION_NODE_H

// UFO
#include <ufo/map/types.h>

// STL
#include <array>

namespace ufo::map
{
template <std::size_t N = 8>
struct ReflectionNode {
	//
	// Size
	//

	[[nodiscard]] static constexpr std::size_t reflectionSize() { return N; }

	//
	// Hits iterators
	//

	[[nodiscard]] constexpr auto beginHits() noexcept { return hits_.begin(); }

	[[nodiscard]] constexpr auto beginHits() const noexcept { return hits_.begin(); }

	[[nodiscard]] constexpr auto cbeginHits() const noexcept { return hits_.cbegin(); }

	[[nodiscard]] constexpr auto endHits() noexcept { return hits_.end(); }

	[[nodiscard]] constexpr auto endHits() const noexcept { return hits_.end(); }

	[[nodiscard]] constexpr auto cendHits() const noexcept { return hits_.cend(); }

	[[nodiscard]] constexpr auto rbeginHits() noexcept { return hits_.rbegin(); }

	[[nodiscard]] constexpr auto rbeginHits() const noexcept { return hits_.rbegin(); }

	[[nodiscard]] constexpr auto crbeginHits() const noexcept { return hits_.crbegin(); }

	[[nodiscard]] constexpr auto rendHits() noexcept { return hits_.rend(); }

	[[nodiscard]] constexpr auto rendHits() const noexcept { return hits_.rend(); }

	[[nodiscard]] constexpr auto crendHits() const noexcept { return hits_.crend(); }

	//
	// Misses iterators
	//

	[[nodiscard]] constexpr auto beginMisses() noexcept { return misses_.begin(); }

	[[nodiscard]] constexpr auto beginMisses() const noexcept { return misses_.begin(); }

	[[nodiscard]] constexpr auto cbeginMisses() const noexcept { return misses_.cbegin(); }

	[[nodiscard]] constexpr auto endMisses() noexcept { return misses_.end(); }

	[[nodiscard]] constexpr auto endMisses() const noexcept { return misses_.end(); }

	[[nodiscard]] constexpr auto cendMisses() const noexcept { return misses_.cend(); }

	[[nodiscard]] constexpr auto rbeginMisses() noexcept { return misses_.rbegin(); }

	[[nodiscard]] constexpr auto rbeginMisses() const noexcept { return misses_.rbegin(); }

	[[nodiscard]] constexpr auto crbeginMisses() const noexcept
	{
		return misses_.crbegin();
	}

	[[nodiscard]] constexpr auto rendMisses() noexcept { return misses_.rend(); }

	[[nodiscard]] constexpr auto rendMisses() const noexcept { return misses_.rend(); }

	[[nodiscard]] constexpr auto crendMisses() const noexcept { return misses_.crend(); }

	//
	// Fill
	//

	void fill(ReflectionNode const parent, index_t const index)
	{
		setHits(parent.hits(index));
		setMisses(parent.misses(index));
	}

	//
	// Is collapsible
	//

	[[nodiscard]] constexpr bool isCollapsible(ReflectionNode const parent,
	                                           index_t const index) const
	{
		// TODO: Use floor(log2(X))?
		return all_of(hits_, [p = parent.hits(index)](auto const e) { return e == p; }) &&
		       all_of(misses_, [p = parent.misses(index)](auto const e) { return e == p; });
	}

	//
	// Get hits
	//

	[[nodiscard]] constexpr count_t hits(index_t const index) const
	{
		if constexpr (1 == N) {
			return hits_[0];
		} else {
			return hits_[index];
		}
	}

	//
	// Set hits
	//

	void setHits(count_t const value) { hits_.fill(value); }

	void setHits(index_t const index, count_t const value)
	{
		if constexpr (1 == N) {
			setHits(value);
		} else {
			hits_[index] = value;
		}
	}

	//
	// Get misses
	//

	[[nodiscard]] constexpr count_t misses(index_t const index) const
	{
		if constexpr (1 == N) {
			return misses_[0];
		} else {
			return misses_[index];
		}
	}

	//
	// Set misses
	//

	void setMisses(count_t const value) { misses_.fill(value); }

	void setMisses(index_t const index, count_t const value)
	{
		if constexpr (1 == N) {
			setMisses(value);
		} else {
			misses_[index] = value;
		}
	}

 private:
	// Data
	std::array<count_t, N> hits_;
	std::array<count_t, N> misses_;
};
}  // namespace ufo::map

#endif  // UFO_MAP_REFLECTION_NODE_H