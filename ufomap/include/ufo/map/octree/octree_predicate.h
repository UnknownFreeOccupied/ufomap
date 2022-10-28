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
#include <ufo/map/code.h>
#include <ufo/map/node.h>
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

struct Leaf {
};

struct Inner {
};

struct Parent {
};

struct Exists {
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
struct Size {
	constexpr Size(double size) noexcept : size_(size), depth(0) {}

	constexpr void setSize(double size) noexcept
	{
		depth_modified = depth_modified || size != size_;
		size_ = size;
	}

	constexpr double size() const noexcept { return size_; }

 private:
	double size_;
	mutable Depth<PC, PT> depth_;
	mutable bool depth_modified_ = true;

	friend struct PredicateValueCheck<Size<PR, PC, PT>>;
	friend struct PredicateInnerCheck<Size<PR, PC, PT>>;
};

template <PredicateCompare PC = PredicateCompare::EQUAL,
          PredicateType PT = PredicateType::RETURN_AND_INNER>
using SizeNearest = Size<PredicateRounding::NEAREST, PC, PT>;
using SizeENearest = SizeNearest<>;
using SizeLENearest = SizeNearest<PredicateCompare::LESS_EQUAL>;
using SizeGENearest = SizeNearest<PredicateCompare::GREATER_EQUAL>;
using SizeLNearest = SizeNearest<PredicateCompare::LESS>;
using SizeGNearest = SizeNearest<PredicateCompare::GREATER>;

template <PredicateCompare PC = PredicateCompare::EQUAL,
          PredicateType PT = PredicateType::RETURN_AND_INNER>
using SizeDownwards = Size<PredicateRounding::FLOOR, PC, PT>;
using SizeEDownwards = SizeDownwards<>;
using SizeLEDownwards = SizeDownwards<PredicateCompare::LESS_EQUAL>;
using SizeGEDownwards = SizeDownwards<PredicateCompare::GREATER_EQUAL>;
using SizeLDownwards = SizeDownwards<PredicateCompare::LESS>;
using SizeGDownwards = SizeDownwards<PredicateCompare::GREATER>;

template <PredicateCompare PC = PredicateCompare::EQUAL,
          PredicateType PT = PredicateType::RETURN_AND_INNER>
using SizeUpwards = Size<PredicateRounding::CEIL, PC, PT>;
using SizeEUpwards = SizeUpwards<>;
using SizeLEUpwards = SizeUpwards<PredicateCompare::LESS_EQUAL>;
using SizeGEUpwards = SizeUpwards<PredicateCompare::GREATER_EQUAL>;
using SizeLUpwards = SizeUpwards<PredicateCompare::LESS>;
using SizeGUpwards = SizeUpwards<PredicateCompare::GREATER>;

using SizeMin = SizeGEUpwards;
using SizeMax = SizeLEDownwards;

struct SizeInterval {
	constexpr SizeInterval(double min, double max) noexcept : min(min), max(max) {}

	SizeMin min;
	SizeMax max;
};

struct Modified {
};

struct ChildOf {
	constexpr ChildOf(Node node) noexcept : code(node.code()) {}

	constexpr ChildOf(Code code) noexcept : code(code) {}

	constexpr ChildOf(Key key) noexcept : code(key) {}

	Code code;
};

struct Index {
	constexpr Index(index_t index) noexcept : index(index) {}

	index_t index;
};

//
// Predicate value/return check
//

template <>
struct PredicateValueCheck<Leaf> {
	using Pred = Leaf;

	template <class Map>
	static constexpr bool apply(Pred, Map const& m, Node n)
	{
		return m.isLeaf(n);
	}
};

template <>
struct PredicateValueCheck<Inner> {
	using Pred = Inner;

	template <class Map>
	static constexpr bool apply(Pred, Map const&, Node n) noexcept
	{
		return 0 != n.depth();
	}
};

template <>
struct PredicateValueCheck<Parent> {
	using Pred = Parent;

	template <class Map>
	static constexpr bool apply(Pred, Map const& m, Node n)
	{
		return m.isParent(n);
	}
};

template <>
struct PredicateValueCheck<Exists> {
	using Pred = Exists;

	template <class Map>
	static constexpr bool apply(Pred, Map const& m, Node n)
	{
		return m.exists(n);  // TODO: Look at
	}
}

template <PredicateCompare PC, PredicateType PT>
struct PredicateValueCheck<Depth<PC, PT>> {
	using Pred = Depth<PC, PT>;

	template <class Map>
	static constexpr bool apply(Pred p, Map const& m, Node n) noexcept
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
	static constexpr bool apply(Pred p, Map const& m, Node n) noexcept
	{
		return PredicateValueCheck<std::decay_t<decltype(p.min)>>::apply(p.min, m, n) &&
		       PredicateValueCheck<std::decay_t<decltype(p.max)>>::apply(p.max, m, n);
	}
};

template <PredicateRounding PR, PredicateCompare PC, PredicateType PT>
struct PredicateValueCheck<Size<PR, PC, PT>> {
	using Pred = Size<PR, PC, PT>;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const& m, Node n) noexcept
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
struct PredicateValueCheck<SizeInterval> {
	using Pred = SizeInterval;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const& m, Node n) noexcept
	{
		return PredicateValueCheck<std::decay_t<decltype(p.min)>>::apply(p.min, m, n) &&
		       PredicateValueCheck<std::decay_t<decltype(p.max)>>::apply(p.max, m, n);
	}
};

template <>
struct PredicateValueCheck<Modified> {
	using Pred = Modified;

	template <class Map>
	static constexpr bool apply(Pred, Map const& m, Node n) noexcept
	{
		return m.isModified(n);
	}
};

template <>
struct PredicateValueCheck<ChildOf> {
	using Pred = ChildOf;

	template <class Map>
	static constexpr bool apply(Pred p, Map const&, Node n) noexcept
	{
		return n.depth() < p.code.depth() && n.code().toDepth(p.code.depth()) == p.code;
	}
};

template <>
struct PredicateValueCheck<Index> {
	using Pred = Index;

	template <class Map>
	static constexpr bool apply(Pred p, Map const&, Node n) noexcept
	{
		return p.index == n.index();
	}
};

//
// Predicate inner check
//

template <>
struct PredicateInnerCheck<Leaf> {
	using Pred = Leaf;

	template <class Map>
	static constexpr bool apply(Pred, Map const&, Node n) noexcept
	{
		return 0 != n.depth();
	}
};

template <>
struct PredicateInnerCheck<Inner> {
	using Pred = Inner;

	template <class Map>
	static constexpr bool apply(Pred, Map const& m, Node n)
	{
		return 1 < n.depth() && m.isParent(n);
	}
};

template <>
struct PredicateInnerCheck<Parent> {
	using Pred = Parent;

	template <class Map>
	static constexpr bool apply(Pred, Map const& m, Node n)
	{
		return 1 < n.depth() && m.isParent(n);
	}
};

template <>
struct PredicateInnerCheck<Exists> {
	using Pred = Exists;

	template <class Map>
	static constexpr bool apply(Pred, Map const& m, Node n)
	{
		return m.isParent(n);
	}
};

template <PredicateCompare PC, PredicateType PT>
struct PredicateInnerCheck<Depth<PC, PT>> {
	using Pred = Depth<PC, PT>;

	template <class Map>
	static constexpr bool apply(Pred p, Map const& m, Node n) noexcept
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
	static constexpr bool apply(Pred p, Map const& m, Node n) noexcept
	{
		return PredicateInnerCheck<std::decay_t<decltype(p.min)>>::apply(p.min, m, n) &&
		       PredicateInnerCheck<std::decay_t<decltype(p.max)>>::apply(p.max, m, n);
	}
};

template <PredicateRounding PR, PredicateCompare PC, PredicateType PT>
struct PredicateInnerCheck<Size<PR, PC, PT>> {
	using Pred = Size<PR, PC, PT>;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const& m, Node n) noexcept
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
struct PredicateInnerCheck<SizeInterval> {
	using Pred = SizeInterval;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const& m, Node n) noexcept
	{
		return PredicateInnerCheck<std::decay_t<decltype(p.min)>>::apply(p.min, m, n) &&
		       PredicateInnerCheck<std::decay_t<decltype(p.max)>>::apply(p.max, m, n);
	}
};

template <>
struct PredicateInnerCheck<Modified> {
	using Pred = Modified;

	template <class Map>
	static constexpr bool apply(Pred, Map const& m, Node n) noexcept
	{
		return m.isModified(n);
	}
};

template <>
struct PredicateInnerCheck<ChildOf> {
	using Pred = ChildOf;

	template <class Map>
	static constexpr bool apply(Pred p, Map const&, Node n) noexcept
	{
		return n.depth() > p.code.depth() ? n.code() == p.code.toDepth(n.depth())
		                                  : n.code().toDepth(p.code.depth()) == p.code;
	}
};

template <>
struct PredicateInnerCheck<Index> {
	using Pred = Index;

	template <class Map>
	static constexpr bool apply(Pred, Map const&, Node) noexcept
	{
		return true;
	}
};
}  // namespace ufo::map::predicate

#endif  // UFO_MAP_PREDICATE_OCTREE_H