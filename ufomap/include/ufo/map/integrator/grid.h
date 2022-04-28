/*
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * License: BSD 3
 */

/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, D. Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
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

// STL
#include <bitset>
#include <limits>

namespace ufo::map
{
template <DepthType Depth>
class Grid
{
 private:
	static constexpr std::size_t ipow(DepthType pow)
	{
		return 0 == pow ? 1 : 8 * ipow(pow - 1);
	}
	static constexpr std::size_t NumIndices = ipow(Depth);

	static constexpr KeyType Mask =
	    ~((std::numeric_limits<KeyType>::max() >> Depth) << Depth);

 public:
	using reference = typename std::bitset<NumIndices>::reference;

 public:
	constexpr bool operator[](std::size_t index) const { return indices_[index]; }

	constexpr bool operator[](Key const& key) const { return indices_[getIndex(key)]; }

	reference operator[](std::size_t index) { return indices_[index]; }

	reference operator[](Key const& key) { return indices_[getIndex(key)]; }

	bool test(std::size_t index) const { return indices_.test(index); }

	bool test(Key const& key) const { return indices_.test(getIndex(key)); }

	void set(Key const& key) { indices_.set(getIndex(key)); }

	void reset(Key const& key) { indices_.reset(getIndex(key)); }

	void clear() { indices_.reset(); }

	constexpr std::size_t size() const noexcept { return NumIndices; }

	static constexpr std::size_t getIndex(Key const& key)
	{
		DepthType const depth = key.getDepth();
		return ((key.x() >> depth) & Mask) | (((key.y() >> depth) & Mask) << Depth) |
		       (((key.z() >> depth) & Mask) << (2 * Depth));
	}

	static Key getKey(std::size_t index, DepthType depth)
	{
		return Key((index & Mask) << depth, ((index >> Depth) & Mask) << depth,
		           ((index >> (2 * Depth)) & Mask) << depth, depth);
	}

 private:
	std::bitset<NumIndices> indices_;
};
}  // namespace ufo::map

#endif  // UFO_MAP_INTEGRATOR_GRID_H