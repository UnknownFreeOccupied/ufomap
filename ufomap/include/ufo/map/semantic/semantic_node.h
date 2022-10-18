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
template <std::size_t N = 8>
struct SemanticNode {
	//
	// Size
	//

	[[nodiscard]] static constexpr std::size_t semanticSize() { return N; }

	//
	// Fill
	//

	void fill(SemanticNode const& parent, index_t const index)
	{
		// TODO: Implement
	}

	//
	// Is collapsible
	//

	[[nodiscard]] bool isCollapsible(SemanticNode const& parent, index_t const index) const
	{
		if constexpr (1 == N) {
			return std::equal(parent.semantics_.cbegin(0), parent.semantics_.cend(0),
			                  semantics_.cbegin(0), semantics_.cend(0));
		} else {
			auto s = parent.semantics_.size(index);
			for (std::size_t i = 0; N != i; ++i) {
				if (semantics_.size(i) != s) {
					return false;
				}
			}

			auto first = parent.semantics_.cbegin(index);
			auto last = parent.semantics_.cend(index);
			for (std::size_t i = 0; N != i; ++i) {
				if (!std::equal(first, last, semantics_.cbegin(i), semantics_.cend(i))) {
					return false;
				}
			}
			return true;
		}
	}

	//
	// Get semantics
	//

	[[nodiscard]] Semantics semantics(index_t const index) const
	{
		if constexpr (1 == N) {
			return semantics_[0];
		} else {
			return semantics_[index];
		}
	}

	//
	// Set semantics
	//

	void setSemantics(Semantics const& value)
	{
		// TODO: Implement
	}

	void setSemantics(index_t const index, Semantics const& value)
	{
		if constexpr (1 == N) {
			setSemantics(value);
		} else {
			// TODO: Implement
		}
	}

 private:
	// Data
	std::unique_ptr<Semantic[]> semantics_;
};
}  // namespace ufo::map

#endif  // UFO_MAP_SEMANTIC_NODE_H