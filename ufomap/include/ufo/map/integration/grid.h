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

#ifndef UFO_MAP_INTEGRATOR_GRID_H
#define UFO_MAP_INTEGRATOR_GRID_H

// UFO
#include <ufo/map/code.h>
#include <ufo/math/util.h>

// STL
#include <array>
#include <limits>

namespace ufo::map
{
template <depth_t Depth>
class Grid
{
 private:
	static constexpr std::size_t NumIndices = math::ipow(8, Depth);

	static constexpr code_t Mask =
	    ~((std::numeric_limits<code_t>::max() >> 3 * Depth) << 3 * Depth);

	using DataType = std::array<uint64_t, NumIndices / 64>;
	using iterator = typename DataType::iterator;
	using const_iterator = typename DataType::const_iterator;
	using reverse_iterator = typename DataType::reverse_iterator;
	using const_reverse_iterator = typename DataType::const_reverse_iterator;

	// using DataType = std::bitset<NumIndices>;

 public:
	using value_type = typename DataType::value_type;
	using reference = typename DataType::reference;

 public:
	constexpr iterator begin() { return std::begin(indices_); }

	constexpr const_iterator begin() const { return std::begin(indices_); }

	constexpr const_iterator cbegin() const { return begin(); }

	constexpr iterator end() { return std::end(indices_); }

	constexpr const_iterator end() const { return std::end(indices_); }

	constexpr const_iterator cend() const { return end(); }

	constexpr reverse_iterator rbegin() { return std::rbegin(indices_); }

	constexpr const_reverse_iterator rbegin() const { return std::rbegin(indices_); }

	constexpr const_reverse_iterator crbegin() const { return rbegin(); }

	constexpr reverse_iterator rend() { return std::rend(indices_); }

	constexpr const_reverse_iterator rend() const { return std::rend(indices_); }

	constexpr const_reverse_iterator crend() const { return rend(); }

	constexpr value_type operator[](std::size_t const index) const
	{
		return indices_[index];
	}

	constexpr value_type operator[](Code const code) const { return indices_[index(code)]; }

	constexpr reference operator[](std::size_t const index) { return indices_[index]; }

	constexpr reference operator[](Code const code) { return indices_[index(code)]; }

	constexpr bool test(std::size_t const index) const
	{
		return indices_[index >> 6] & value_type(1) << (index & 0x3F);
	}

	constexpr bool test(Code const code) const { return test(index(code)); }

	constexpr void set(std::size_t const index)
	{
		indices_[index >> 6] |= value_type(1) << (index & 0x3F);
	}

	constexpr void set(Code const code) { set(index(code)); }

	constexpr void reset(std::size_t const index)
	{
		indices_[index >> 6] &= value_type(0) << (index & 0x3F);
	}

	constexpr void reset(Code const code) { reset(index(code)); }

	void clear() { indices_.fill(0); }

	constexpr std::size_t size() const noexcept { return indices_.size() * 64; }

	static constexpr std::size_t index(Code const code)
	{
		return (code.code() >> 3 * code.depth()) & Mask;
	}

	static constexpr Code code(std::size_t const index, depth_t const depth)
	{
		return Code(index << 3 * depth, depth);
	}

	static constexpr Code code(code_t prefix, std::size_t const index, depth_t const depth)
	{
		return Code(prefix | (index << 3 * depth), depth);
	}

 private:
	DataType indices_ = {};
};
}  // namespace ufo::map

#endif  // UFO_MAP_INTEGRATOR_GRID_H