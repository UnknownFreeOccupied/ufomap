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

#ifndef UFO_MAP_SURFEL_NODE_H
#define UFO_MAP_SURFEL_NODE_H

// UFO
#include <ufo/map/surfel/surfel.h>

// STL
#include <cstdint>
#include <memory>

namespace ufo::map
{
template <bool Single = false>
struct SurfelNode {
	using surfel_t = Surfel<float>;
	using surfel_type = std::conditional_t<Single, surfel_t, std::array<surfel_t, 8>>;

	surfel_type surfel;

	bool operator==(SurfelNodeBlock const& rhs) const { return surfel == rhs.surfel; }

	bool operator!=(SurfelNodeBlock const& rhs) const { return !(*this == rhs); }

	constexpr surfel_type& getSurfel(std::size_t index)
	{
		if constexpr (Single) {
			return surfel;
		} else {
			return surfel[index];
		}
	}

	constexpr surfel_type const& getSurfel(std::size_t index) const
	{
		if constexpr (Single) {
			return surfel;
		} else {
			return surfel[index];
		}
	}

	void setSurfel(surfel_t const& s)
	{
		if constexpr (Single) {
			surfel = s;
		} else {
			surfel.fill(s);
		}
	}

	void setSurfel(std::size_t index, surfel_t const& s)
	{
		if constexpr (Single) {
			surfel = s;
		} else {
			surfel[index] = s;
		}
	}
};
}  // namespace ufo::map

#endif  // UFO_MAP_SURFEL_NODE_H