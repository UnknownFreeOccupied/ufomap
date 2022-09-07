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

#ifndef UFO_MAP_PREDICATE_OCTREE_H
#define UFO_MAP_PREDICATE_OCTREE_H

// UFO
#include <ufo/map/code/code.h>
#include <ufo/map/octree/node.h>
#include <ufo/map/predicate/predicates.h>
#include <ufo/map/types.h>

// STL
#include <cmath>
#include <functional>

namespace ufo::map::predicate
{
//
// Predicates
//

struct PureLeaf {
};

struct Leaf {
	constexpr Leaf(depth_t min_depth = 0) noexcept : min_depth(min_depth) {}

	depth_t min_depth;
};

template <PredicateRounding PR>
struct LeafVoxelSize {
	constexpr LeafVoxelSize(double voxel_size = 0.0) noexcept : voxel_size(voxel_size) {}

	constexpr void setVoxelSize(double new_voxel_size) noexcept
	{
		leaf_modified = leaf_modified || voxel_size != new_voxel_size;
		voxel_size = new_voxel_size;
	}

	constexpr double voxelSize() const noexcept { return voxel_size; }

 private:
	double voxel_size;
	mutable Leaf leaf;
	mutable bool leaf_modified = true;

	template <PredicateRounding P>
	friend struct PredicateValueCheck;
	template <PredicateRounding P>
	friend struct PredicateInnerCheck;
};

using LeafVoxelSizeNearest = LeafVoxelSize<PredicateRounding::NEAREST>;
using LeafVoxelSizeDownwards = LeafVoxelSize<PredicateRounding::FLOOR>;
using LeafVoxelSizeUpwards = LeafVoxelSize<PredicateRounding::CEIL>;

struct Parent {
};

template <PredicateCompare PC = PredicateCompare::EQUAL,
          PredicateType PT = PredicateType::RETURN_AND_INNER>
struct Depth {
	constexpr Depth(depth_t depth) noexcept : depth(depth) {}

	depth_t depth;
};

using DepthE = Depth<PredicateCompare::EQUAL>;
using DepthLE = Depth<PredicateCompare::LESS_EQUAL>;
using DepthGE = Depth<PredicateCompare::GREATER_EQUAL>;
using DepthL = Depth<PredicateCompare::LESS>;
using DepthG = Depth<PredicateCompare::GREATER>;

using DepthMin = DepthGE;
using DepthMax = DepthLE;

struct DepthInterval {
	constexpr DepthInterval(depth_t min, depth_t max) noexcept : min(min), max(max) {}

	DepthMin min;
	DepthMax max;
};

template <PredicateRounding PR, PredicateCompare PC = PredicateCompare::EQUAL,
          PredicateType PT = PredicateType::RETURN_AND_INNER>
struct VoxelSize {
	constexpr VoxelSize(double voxel_size) noexcept : voxel_size(voxel_size), depth(0) {}

	constexpr void setVoxelSize(double new_voxel_size) noexcept
	{
		depth_modified = depth_modified || voxel_size != new_voxel_size;
		voxel_size = new_voxel_size;
	}

	constexpr double voxelSize() const noexcept { return voxel_size; }

 private:
	double voxel_size;
	mutable Depth<PC, PT> depth;
	mutable bool depth_modified = true;

	friend struct PredicateValueCheck<VoxelSize<PR, PC, PT>>;
	friend struct PredicateInnerCheck<VoxelSize<PR, PC, PT>>;
};

template <PredicateCompare PC = PredicateCompare::EQUAL,
          PredicateType PT = PredicateType::RETURN_AND_INNER>
using VoxelSizeNearest = VoxelSize<PredicateRounding::NEAREST, PC, PT>;
using VoxelSizeENearest = VoxelSizeNearest<>;
using VoxelSizeLENearest = VoxelSizeNearest<PredicateCompare::LESS_EQUAL>;
using VoxelSizeGENearest = VoxelSizeNearest<PredicateCompare::GREATER_EQUAL>;
using VoxelSizeLNearest = VoxelSizeNearest<PredicateCompare::LESS>;
using VoxelSizeGNearest = VoxelSizeNearest<PredicateCompare::GREATER>;

template <PredicateCompare PC = PredicateCompare::EQUAL,
          PredicateType PT = PredicateType::RETURN_AND_INNER>
using VoxelSizeDownwards = VoxelSize<PredicateRounding::FLOOR, PC, PT>;
using VoxelSizeEDownwards = VoxelSizeDownwards<>;
using VoxelSizeLEDownwards = VoxelSizeDownwards<PredicateCompare::LESS_EQUAL>;
using VoxelSizeGEDownwards = VoxelSizeDownwards<PredicateCompare::GREATER_EQUAL>;
using VoxelSizeLDownwards = VoxelSizeDownwards<PredicateCompare::LESS>;
using VoxelSizeGDownwards = VoxelSizeDownwards<PredicateCompare::GREATER>;

template <PredicateCompare PC = PredicateCompare::EQUAL,
          PredicateType PT = PredicateType::RETURN_AND_INNER>
using VoxelSizeUpwards = VoxelSize<PredicateRounding::CEIL, PC, PT>;
using VoxelSizeEUpwards = VoxelSizeUpwards<>;
using VoxelSizeLEUpwards = VoxelSizeUpwards<PredicateCompare::LESS_EQUAL>;
using VoxelSizeGEUpwards = VoxelSizeUpwards<PredicateCompare::GREATER_EQUAL>;
using VoxelSizeLUpwards = VoxelSizeUpwards<PredicateCompare::LESS>;
using VoxelSizeGUpwards = VoxelSizeUpwards<PredicateCompare::GREATER>;

using VoxelSizeMin = VoxelSizeGEUpwards;
using VoxelSizeMax = VoxelSizeLEDownwards;

struct VoxelSizeInterval {
	constexpr VoxelSizeInterval(double min, double max) noexcept : min(min), max(max) {}

	VoxelSizeMin min;
	VoxelSizeMax max;
};

struct Modified {
};

struct ChildrenOf {
	constexpr ChildrenOf(Node const& node) noexcept : node(node) {}

	Node node;
};

struct CodePrefix {
	constexpr CodePrefix(Code code) noexcept : code(code) {}

	Code code;
};

//
// Predicate value/return check
//

template <>
struct PredicateValueCheck<PureLeaf> {
	using Pred = PureLeaf;

	template <class Map>
	static constexpr bool apply(Pred const&, Map const&, Node const& n) noexcept
	{
		return Map::isPureLeaf(n);
	}
};

template <>
struct PredicateValueCheck<Leaf> {
	using Pred = Leaf;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n) noexcept
	{
		return m.isLeaf(n) || n.depth() == p.min_depth;
	}
};

template <PredicateRounding PR>
struct PredicateValueCheck<LeafVoxelSize<PR>> {
	using Pred = LeafVoxelSize<PR>;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n) noexcept
	{
		if (p.depth_modified) {
			double temp = std::max(0.0, (m.resolution() / p.resolution) + 1.0);
			if constexpr (PredicateRounding::NEAREST == PR) {
				p.leaf.min_depth = std::round(temp);
			} else if constexpr (PredicateRounding::FLOOR == PR) {
				p.leaf.min_depth = std::floor(temp);
			} else {
				p.leaf.min_depth = std::ceil(temp);
			}
			p.depth_modified = false;
		}

		return PredicateValueCheck<std::decay_t<decltype(p.leaf)>>::apply(p.leaf, m, n);
	}
};

template <>
struct PredicateValueCheck<Parent> {
	using Pred = Parent;

	template <class Map>
	static constexpr bool apply(Pred const&, Map const& m, Node const& n) noexcept
	{
		return Map::isParent(n);
	}
};

template <PredicateCompare PC, PredicateType PT>
struct PredicateValueCheck<Depth<PC, PT>> {
	using Pred = Depth<PC, PT>;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n) noexcept
	{
		if constexpr (PredicateType::INNER == PT) {
			return false;
		} else if constexpr (PredicateCompare::EQUAL == PC) {
			return n.depth() == p.depth;
		} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
			return n.depth() <= p.depth;
		} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
			return n.depth() >= p.depth;
		} else if constexpr (PredicateCompare::LESS == PC) {
			return n.depth() < p.depth;
		} else if constexpr (PredicateCompare::GREATER == PC) {
			return n.depth() > p.depth;
		} else {
			static_assert("Non-supported depth predicate comparison.");
		}
	}
};

template <>
struct PredicateValueCheck<DepthInterval> {
	using Pred = DepthInterval;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n) noexcept
	{
		return PredicateValueCheck<std::decay_t<decltype(p.min)>>::apply(p.min, m, n) &&
		       PredicateValueCheck<std::decay_t<decltype(p.max)>>::apply(p.max, m, n);
	}
};

template <PredicateRounding PR, PredicateCompare PC, PredicateType PT>
struct PredicateValueCheck<VoxelSize<PR, PC, PT>> {
	using Pred = VoxelSize<PR, PC, PT>;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n) noexcept
	{
		if (p.depth_modified) {
			double temp = std::max(0.0, (m.resolution() / p.resolution) + 1.0);
			if constexpr (PredicateRounding::NEAREST == PR) {
				p.depth.depth = std::round(temp);
			} else if constexpr (PredicateRounding::FLOOR == PR) {
				p.depth.depth = std::floor(temp);
			} else {
				p.depth.depth = std::ceil(temp);
			}
			p.depth_modified = false;
		}

		return PredicateValueCheck<std::decay_t<decltype(p.depth)>>::apply(p.depth, m, n);
	}
};

template <>
struct PredicateValueCheck<VoxelSizeInterval> {
	using Pred = VoxelSizeInterval;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n) noexcept
	{
		return PredicateValueCheck<std::decay_t<decltype(p.min)>>::apply(p.min, m, n) &&
		       PredicateValueCheck<std::decay_t<decltype(p.max)>>::apply(p.max, m, n);
	}
};

template <>
struct PredicateValueCheck<Modified> {
	using Pred = Modified;

	template <class Map>
	static constexpr bool apply(Pred const&, Map const& m, Node const& n) noexcept
	{
		return m.isModified(n);
	}
};

template <>
struct PredicateValueCheck<ChildrenOf> {
	using Pred = ChildrenOf;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const&, Node const& n) noexcept
	{
		return n.depth() < p.node.depth() &&
		       n.code().toDepth(p.node.depth()) == p.node.depth();
	}
};

template <>
struct PredicateValueCheck<CodePrefix> {
	using Pred = CodePrefix;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const&, Node const& n) noexcept
	{
		return n.depth() <= p.code.depth() && n.code().toDepth(p.code.depth()) == p.code;
	}
};

//
// Predicate inner check
//

template <>
struct PredicateInnerCheck<PureLeaf> {
	using Pred = PureLeaf;

	template <class Map>
	static constexpr bool apply(Pred const&, Map const&, Node const& n) noexcept
	{
		return 0 != n.depth();
	}
};

template <>
struct PredicateInnerCheck<Leaf> {
	using Pred = Leaf;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n) noexcept
	{
		return m.isParent(n) && n.depth() > p.min_depth;
	}
};

template <PredicateRounding PR>
struct PredicateInnerCheck<LeafVoxelSize<PR>> {
	using Pred = LeafVoxelSize<PR>;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n) noexcept
	{
		if (p.leaf_modified) {
			double temp = std::max(0.0, (m.resolution() / p.resolution) + 1.0);
			if constexpr (PredicateRounding::NEAREST == PR) {
				p.leaf.depth = std::round(temp);
			} else if constexpr (PredicateRounding::FLOOR == PR) {
				p.leaf.depth = std::floor(temp);
			} else {
				p.leaf.depth = std::ceil(temp);
			}
			p.leaf_modified = false;
		}

		return PredicateInnerCheck<std::decay_t<decltype(p.leaf)>>::apply(p.leaf, m, n);
	}
};

template <>
struct PredicateInnerCheck<Parent> {
	using Pred = Parent;

	template <class Map>
	static constexpr bool apply(Pred const&, Map const& m, Node const& n) noexcept
	{
		return Map::isParent(n);
	}
};

template <PredicateCompare PC, PredicateType PT>
struct PredicateInnerCheck<Depth<PC, PT>> {
	using Pred = Depth<PC, PT>;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n) noexcept
	{
		if constexpr (PredicateType::RETURN == PT) {
			return PredicateValueCheck<Pred>::apply(p, m, n);
		} else if constexpr (PredicateCompare::EQUAL == PC) {
			return n.depth() > p.depth;
		} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
			return true;
		} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
			return n.depth() > p.depth;
		} else if constexpr (PredicateCompare::LESS == PC) {
			return true;
		} else if constexpr (PredicateCompare::GREATER == PC) {
			return n.depth() > (p.depth + 1U);
		} else {
			static_assert("Non-supported depth predicate comparison.");
		}
	}
};

template <>
struct PredicateInnerCheck<DepthInterval> {
	using Pred = DepthInterval;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n) noexcept
	{
		return PredicateInnerCheck<std::decay_t<decltype(p.min)>>::apply(p.min, m, n) &&
		       PredicateInnerCheck<std::decay_t<decltype(p.max)>>::apply(p.max, m, n);
	}
};

template <PredicateRounding PR, PredicateCompare PC, PredicateType PT>
struct PredicateInnerCheck<VoxelSize<PR, PC, PT>> {
	using Pred = VoxelSize<PR, PC, PT>;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n) noexcept
	{
		if constexpr (PredicateType::RETURN == PT) {
			return PredicateValueCheck<Pred>::apply(p, m, n);
		}

		if (p.depth_modified) {
			double temp = std::max(0.0, (m.resolution() / p.resolution) + 1.0);
			if constexpr (PredicateRounding::NEAREST == PR) {
				p.depth.depth = std::round(temp);
			} else if constexpr (PredicateRounding::FLOOR == PR) {
				p.depth.depth = std::floor(temp);
			} else {
				p.depth.depth = std::ceil(temp);
			}
			p.depth_modified = false;
		}

		return PredicateInnerCheck<std::decay_t<decltype(p.depth)>>::apply(p.depth, m, n);
	}
};

template <>
struct PredicateInnerCheck<VoxelSizeInterval> {
	using Pred = VoxelSizeInterval;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n) noexcept
	{
		return PredicateInnerCheck<std::decay_t<decltype(p.min)>>::apply(p.min, m, n) &&
		       PredicateInnerCheck<std::decay_t<decltype(p.max)>>::apply(p.max, m, n);
	}
};

template <>
struct PredicateInnerCheck<Modified> {
	using Pred = Modified;

	template <class Map>
	static constexpr bool apply(Pred const&, Map const& m, Node const& n) noexcept
	{
		return m.isModified(n);
	}
};

template <>
struct PredicateInnerCheck<ChildrenOf> {
	using Pred = ChildrenOf;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const&, Node const& n) noexcept
	{
		if (n.depth() <= p.node.depth()) {
			return n.code().toDepth(p.node.depth()) == p.node.code();
		} else {
			return n.code() == p.node.code().toDepth(n.depth());
		}
	}
};

template <>
struct PredicateInnerCheck<CodePrefix> {
	using Pred = CodePrefix;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const&, Node const& n) noexcept
	{
		if (n.depth() <= p.code.depth()) {
			return n.code().toDepth(p.code.depth()) == p.code;
		} else {
			return n.code() == p.code.toDepth(n.depth());
		}
	}
};
}  // namespace ufo::map::predicate

#endif  // UFO_MAP_PREDICATE_OCTREE_H