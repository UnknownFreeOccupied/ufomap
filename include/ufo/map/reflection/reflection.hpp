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

#ifndef UFO_MAP_REFLECTION_HPP
#define UFO_MAP_REFLECTION_HPP

// UFO
#include <ufo/map/types.hpp>

// STL
#include <algorithm>
#include <ostream>

namespace ufo
{
struct Reflection {
	count_t hits{};
	count_t misses{};

	constexpr Reflection() = default;

	constexpr Reflection(count_t hits, count_t misses) : hits(hits), misses(misses) {}

	friend void swap(Reflection& lhs, Reflection& rhs) noexcept
	{
		std::swap(lhs.hits, rhs.hits);
		std::swap(lhs.misses, rhs.misses);
	}

	friend bool operator==(Reflection lhs, Reflection rhs)
	{
		return lhs.hits == rhs.hits && lhs.misses == rhs.misses;
	}

	friend bool operator!=(Reflection lhs, Reflection rhs) { return !(lhs == rhs); }

	[[nodiscard]] constexpr reflection_t reflectiveness() const
	{
		return hits / static_cast<double>(hits + misses);
	}

	constexpr void reset()
	{
		hits   = 0;
		misses = 0;
	}
};
}  // namespace ufo

namespace std
{
inline ostream& operator<<(ostream& out, ufo::Reflection reflection)
{
	return out << "Hits: " << +reflection.hits << " Misses: " << +reflection.misses;
}
}  // namespace std

#endif  // UFO_MAP_REFLECTION_HPP