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

#ifndef UFO_MAP_INTEGRATION_DETAIL_HIT_GRID_HPP
#define UFO_MAP_INTEGRATION_DETAIL_HIT_GRID_HPP

// UFO
#include <ufo/map/integrator/detail/bool_grid.hpp>
#include <ufo/map/integrator/detail/count_grid.hpp>

// STL
#include <cstddef>

namespace ufo::detail
{
template <std::size_t Dim, unsigned Depth>
class HitGrid
{
 public:
	using Code    = TreeCode<Dim>;
	using code_t  = typename Code::code_t;
	using depth_t = typename Code::depth_t;

 private:
	static constexpr std::size_t const Size = ipow(std::size_t(2), Dim* Depth);

	static constexpr code_t const Mask = ~(((~code_t(0)) >> Dim * Depth) << Dim * Depth);

 public:
	BoolGrid<Dim, Depth> hit;

	void clear() { hit.clear(); }

	[[nodiscard]] static constexpr std::size_t size() noexcept { return Size; }

	[[nodiscard]] static constexpr depth_t depth() noexcept { return Depth; }

	[[nodiscard]] static constexpr code_t pos(Code const& code) noexcept
	{
		return (code.lowestOffsets() >> Dim * code.depth()) & Mask;
	}

	[[nodiscard]] static constexpr Code code(Code prefix, code_t pos, depth_t depth_offset,
	                                         depth_t depth)
	{
		prefix.lowestOffsets(prefix.lowestOffsets() | (pos << Dim * depth_offset),
		                     depth_offset + depth);
		return prefix;
	}
};
}  // namespace ufo::detail

#endif  // UFO_MAP_INTEGRATION_DETAIL_HIT_GRID_HPP