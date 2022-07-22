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
#include <ufo/map/key.h>
#include <ufo/math/util.h>

// STL
#include <bitset>
#include <limits>

namespace ufo::map
{
template <depth_t Depth>
class Grid
{
 private:
	static constexpr std::size_t NumIndices = math::ipow(8, Depth);

	static constexpr Key::key_t Mask =
	    ~((std::numeric_limits<Key::key_t>::max() >> Depth) << Depth);

 public:
	using reference = typename std::bitset<NumIndices>::reference;

 public:
	constexpr bool operator[](std::size_t const index) const { return indices_[index]; }

	constexpr bool operator[](Key const key) const { return indices_[index(key)]; }

	reference operator[](std::size_t const index) { return indices_[index]; }

	reference operator[](Key const key) { return indices_[index(key)]; }

	bool test(std::size_t const index) const { return indices_.test(index); }

	bool test(Key const key) const { return indices_.test(index(key)); }

	void set(Key const key) { indices_.set(index(key)); }

	void reset(Key const key) { indices_.reset(index(key)); }

	void clear() { indices_.reset(); }

	constexpr std::size_t size() const noexcept { return NumIndices; }

	static constexpr std::size_t index(Key const key)
	{
		depth_t const depth = key.depth();
		return ((key.x() >> depth) & Mask) | (((key.y() >> depth) & Mask) << Depth) |
		       (((key.z() >> depth) & Mask) << (2 * Depth));
	}

	static constexpr Key key(std::size_t const index, depth_t const depth)
	{
		return Key((index & Mask) << depth, ((index >> Depth) & Mask) << depth,
		           ((index >> (2 * Depth)) & Mask) << depth, depth);
	}

 private:
	std::bitset<NumIndices> indices_;
};
}  // namespace ufo::map

#endif  // UFO_MAP_INTEGRATOR_GRID_H