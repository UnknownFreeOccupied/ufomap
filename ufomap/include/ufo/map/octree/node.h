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

#ifndef UFO_MAP_OCTREE_NODE_H
#define UFO_MAP_OCTREE_NODE_H

// UFO
#include <ufo/geometry/aaebb.h>
#include <ufo/geometry/point.h>
#include <ufo/map/code.h>

namespace ufo::map
{

struct MinimalNode {
 public:
	bool operator==(MinimalNode const& rhs) const
	{
		return rhs.data_ == data_ && rhs.code_ == code_;
	}

	bool operator!=(MinimalNode const& rhs) const { return !operator==(rhs); }

	inline Code getCode() const noexcept { return code_; }

	constexpr DepthType getDepth() const noexcept { return code_.getDepth(); }

 protected:
	MinimalNode(void* data, Code code) : data_(data), code_(code) {}

	constexpr void* getData() noexcept { return data_; }

	constexpr void const* getData() const noexcept { return data_; }

 protected:
	// Pointer to the actual node
	void* data_;
	// The code for the node
	Code code_;

	template <class D1, class D2, class I>
	friend class OctreeBase;
};
struct Node : public MinimalNode {
 public:
	bool operator==(Node const& rhs) const
	{
		return static_cast<MinimalNode const&>(rhs) == static_cast<MinimalNode const&>(*this);
	}

	bool operator!=(Node const& rhs) const { return !operator==(rhs); }

	inline geometry::AAEBB getBoundingVolume() const noexcept { return aaebb_; }

	inline geometry::Point getCenter() const noexcept { return aaebb_.center(); }

	inline geometry::Point getMin() const { return aaebb_.getMin(); }

	inline geometry::Point getMax() const { return aaebb_.getMax(); }

	constexpr float getHalfSize() const noexcept { return aaebb_.halfSize(); }

	constexpr float getSize() const noexcept { return 2.0 * getHalfSize(); }

	constexpr float getX() const noexcept { return aaebb_.x(); }

	constexpr float getY() const noexcept { return aaebb_.y(); }

	constexpr float getZ() const noexcept { return aaebb_.z(); }

 protected:
	Node(geometry::AAEBB aaebb, void* data, Code code)
	    : MinimalNode(data, code), aaebb_(aaebb)
	{
	}

 protected:
	// The AAEBB for the node
	geometry::AAEBB aaebb_;

	template <class D1, class D2, class I>
	friend class OctreeBase;
};
}  // namespace ufo::map

#endif  // UFO_MAP_OCTREE_NODE_H