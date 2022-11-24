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
template <std::size_t N>
struct ReflectionNode {
	// Data
	std::array<count_t, N> hits;
	std::array<count_t, N> misses;

	//
	// Size
	//

	[[nodiscard]] static constexpr std::size_t reflectionSize() { return N; }

	//
	// Fill
	//

	void fill(ReflectionNode const parent, index_t const index)
	{
		hits.fill(parent.hits[index]);
		misses.fill(parent.misses[index]);
	}

	//
	// Is collapsible
	//

	[[nodiscard]] bool isCollapsible() const
	{
		// TODO: Use floor(log2(X))?
		return std::all_of(std::begin(hits) + 1, std::end(hits),
		                   [p = hits.front()](auto e) { return e == p; }) &&
		       std::all_of(std::begin(misses) + 1, std::end(misses),
		                   [p = misses.front()](auto e) { return e == p; });
	}
};
}  // namespace ufo::map

#endif  // UFO_MAP_REFLECTION_NODE_H