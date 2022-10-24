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

#ifndef UFO_MAP_SEMANTIC_NODE_H
#define UFO_MAP_SEMANTIC_NODE_H

// UFO
#include <ufo/map/semantic/semantics.h>

// STL
#include <algorithm>
#include <array>

namespace ufo::map
{
template <std::size_t N>
struct SemanticNode {
	// Data
	std::unique_ptr<Semantic[]> semantics;

	//
	// Size
	//

	[[nodiscard]] static constexpr std::size_t semanticSize() { return N; }

	//
	// Fill
	//

	void fill(SemanticNode const& parent, index_t const index)
	{
		resize(parent.size(index));
		auto first = parent.begin(index);
		auto last = parent.end(index);
		for (index_t i = 0; N != i; ++i) {
			std::copy(first, last, begin(i));
		}
	}

	//
	// Is collapsible
	//

	[[nodiscard]] bool isCollapsible(SemanticNode const& parent, index_t const index) const
	{
		auto first = parent.begin(index);
		auto last = parent.end(index);
		for (index_t i = 0; N != i; ++i) {
			if (!std::equal(first, last, cbegin(i), cend(i))) {
				return false;
			}
		}
		return true;
	}

	//
	// Iterators
	//

	//
	// Resize
	//

	void resize(std::array<std::size_t, N> const& sizes)
	{
		// TODO: Implement
	}

	void resize(std::size_t const size)
	{
		std::array<std::size_t, N> sizes;
		sizes.fill(size);
		resize(sizes);
	}

	void resize(index_t const index, std::size_t const size)
	{
		// TODO: Implement
	}
};
}  // namespace ufo::map

#endif  // UFO_MAP_SEMANTIC_NODE_H