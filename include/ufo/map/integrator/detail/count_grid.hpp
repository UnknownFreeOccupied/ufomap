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

#ifndef UFO_MAP_INTEGRATION_COUNT_GRID_HPP
#define UFO_MAP_INTEGRATION_COUNT_GRID_HPP

// UFO
#include <ufo/container/tree/code.hpp>
#include <ufo/math/math.hpp>

// STL
#include <array>
#include <cstddef>
#include <cstdint>

namespace ufo
{
template <std::size_t Dim, unsigned Depth>
class CountGrid
{
 public:
	using Code    = TreeCode<Dim>;
	using code_t  = typename Code::code_t;
	using depth_t = typename Code::depth_t;

 private:
	static constexpr std::size_t const Size = ipow(std::size_t(2), Dim* Depth);

	static constexpr code_t const Mask = ~(((~code_t(0)) >> Dim * Depth) << Dim * Depth);

	static constexpr code_t const PosBits = 6;  // [0..63] needs 6 bits

	using Container = std::array<std::uint8_t, Size / 64>;

 public:
	using value_type      = typename Container::value_type;
	using reference       = typename Container::reference;
	using const_reference = typename Container::const_reference;
	using iterator        = typename Container::iterator;
	using const_iterator  = typename Container::const_iterator;

 public:
	iterator begin() { return grid_.begin(); }

	const_iterator begin() const { return grid_.begin(); }

	const_iterator cbegin() const { return grid_.cbegin(); }

	iterator end() { return grid_.end(); }

	const_iterator end() const { return grid_.end(); }

	const_iterator cend() const { return grid_.cend(); }

	reference operator[](code_t pos) { return grid_[index(pos)]; }

	reference operator[](Code const code) { return operator[](pos(code)); }

	const_reference operator[](code_t pos) const { return grid_[index(pos)]; }

	const_reference operator[](Code const code) const { return operator[](pos(code)); }

	void clear() { grid_.fill(0u); }

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

 private:
	[[nodiscard]] static constexpr code_t index(code_t pos) noexcept
	{
		return pos >> PosBits;
	}

 private:
	Container grid_{};
};
}  // namespace ufo

#endif  // UFO_MAP_INTEGRATION_COUNT_GRID_HPP