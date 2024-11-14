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

#ifndef UFO_MAP_VIEW_DIRECTION_MAP_HPP
#define UFO_MAP_VIEW_DIRECTION_MAP_HPP

// UFO
#include <ufo/map/view_direction/block.hpp>

// STL
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <limits>
#include <utility>

namespace ufo
{
template <class Derived>
class ViewDirectionMap
{
 public:
	/**************************************************************************************
	|                                                                                     |
	|                                        Tags                                         |
	|                                                                                     |
	**************************************************************************************/

	// UFO stuff
	using Index    = typename Derived::Index;
	using Node     = typename Derived::Node;
	using Code     = typename Derived::Code;
	using Key      = typename Derived::Key;
	using Point    = typename Derived::Point;
	using Coord    = typename Derived::Coord;
	using coord_t  = typename Derived::coord_t;
	using depth_t  = typename Derived::depth_t;
	using offset_t = typename Derived::offset_t;
	using length_t = typename Derived::length_t;
	using pos_t    = typename Derived::pos_t;

	using Direction = typename ViewDirectionBlock<Dim, BF>::Direction;

	/**************************************************************************************
	|                                                                                     |
	|                                       Lookup                                        |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] Direction viewDirectionNonNormalized(Index node) const
	{
		return view_direction_[node.pos][node.offset];
	}

	[[nodiscard]] Direction viewDirectionNonNormalized(Node node) const
	{
		return viewDirectionNonNormalized(derived().index(node));
	}

	[[nodiscard]] Direction viewDirectionNonNormalized(Code node) const
	{
		return viewDirectionNonNormalized(derived().index(node));
	}

	[[nodiscard]] Direction viewDirectionNonNormalized(Key node) const
	{
		return viewDirectionNonNormalized(derived().index(node));
	}

	[[nodiscard]] Direction viewDirectionNonNormalized(Coord node) const
	{
		return viewDirectionNonNormalized(derived().index(node));
	}

	[[nodiscard]] Direction viewDirection(Index node) const
	{
		return normalize(viewDirectionNonNormalized(node));
	}

	[[nodiscard]] Direction viewDirection(Node node) const
	{
		return viewDirection(derived().index(node));
	}

	[[nodiscard]] Direction viewDirection(Code node) const
	{
		return viewDirection(derived().index(node));
	}

	[[nodiscard]] Direction viewDirection(Key node) const
	{
		return viewDirection(derived().index(node));
	}

	[[nodiscard]] Direction viewDirection(Coord node) const
	{
		return viewDirection(derived().index(node));
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Modifiers                                      |
	|                                                                                     |
	**************************************************************************************/

	void viewDirectionSet(Index node, Direction const& direction)
	{
		view_direction_[node.pos][node.offset] = direction;
	}

	void viewDirectionSet(Node node, Direction const& direction, bool propagate = true)
	{
		// TODO: Implement
	}

	void viewDirectionSet(Code node, Direction const& direction, bool propagate = true)
	{
		// TODO: Implement
	}

	void viewDirectionSet(Key node, Direction const& direction, bool propagate = true)
	{
		viewDirectionSet(derived().code(node), direction, propagate);
	}

	void viewDirectionSet(Coord node, Direction const& direction, bool propagate = true)
	{
		viewDirectionSet(derived().code(node), direction, propagate);
	}

	void viewDirectionUpdate(Index node, Direction const& direction, float weight = 1.0f)
	{
		view_direction_[node.pos][node.offset] += weight * direction;
	}

	void viewDirectionUpdate(Node node, Direction const& direction, float weight = 1.0f,
	                         bool propagate = true)
	{
		// TODO: Implement
	}

	void viewDirectionUpdate(Code node, Direction const& direction, float weight = 1.0f,
	                         bool propagate = true)
	{
		// TODO: Implement
	}

	void viewDirectionUpdate(Key node, Direction const& direction, float weight = 1.0f,
	                         bool propagate = true)
	{
		viewDirectionUpdate(derived().code(node), direction, weight, propagate);
	}

	void viewDirectionUpdate(Coord node, Direction const& direction, float weight = 1.0f,
	                         bool propagate = true)
	{
		viewDirectionUpdate(derived().code(node), direction, weight, propagate);
	}

	void viewDirectionUpdateNonNormalized(Index node, Direction const& direction,
	                                      float weight = 1.0f)
	{
		viewDirectionUpdate(node, normalize(direction), weight);
	}

	void viewDirectionUpdateNonNormalized(Node node, Direction const& direction,
	                                      float weight = 1.0f, bool propagate = true)
	{
		viewDirectionUpdate(node, normalize(direction), weight, propagate);
	}

	void viewDirectionUpdateNonNormalized(Code node, Direction const& direction,
	                                      float weight = 1.0f, bool propagate = true)
	{
		viewDirectionUpdate(node, normalize(direction), weight, propagate);
	}

	void viewDirectionUpdateNonNormalized(Key node, Direction const& direction,
	                                      float weight = 1.0f, bool propagate = true)
	{
		viewDirectionUpdateNonNormalized(derived().code(node), direction, weight, propagate);
	}

	void viewDirectionUpdateNonNormalized(Coord node, Direction const& direction,
	                                      float weight = 1.0f, bool propagate = true)
	{
		viewDirectionUpdateNonNormalized(derived().code(node), direction, weight, propagate);
	}

	void viewDirectionClear(Index node) { view_direction_[node.pos][node.offset] = {}; }

	void viewDirectionClear(Node node, bool propagate = true)
	{
		// TODO: Implement
	}

	void viewDirectionClear(Code node, bool propagate = true)
	{
		// TODO: Implement
	}

	void viewDirectionClear(Key node, bool propagate = true)
	{
		viewDirectionClear(derived().code(node), propagate);
	}

	void viewDirectionClear(Coord node, bool propagate = true)
	{
		viewDirectionClear(derived().code(node), propagate);
	}

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Implement

	/**************************************************************************************
	|                                                                                     |
	|                                     Destructor                                      |
	|                                                                                     |
	**************************************************************************************/

	~ViewDirectionMap() = default;

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Implement

	/**************************************************************************************
	|                                                                                     |
	|                                        Swap                                         |
	|                                                                                     |
	**************************************************************************************/

	void swap(ViewDirectionMap& other) noexcept(
	    noexcept(view_direction_.swap(other.view_direction_)))
	{
		view_direction_.swap(other.view_direction_);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Derived                                       |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] constexpr Derived& derived() { return *static_cast<Derived*>(this); }

	[[nodiscard]] constexpr Derived const& derived() const
	{
		return *static_cast<Derived const*>(this);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Propagate                                      |
	|                                                                                     |
	**************************************************************************************/

	void propagate(Index node, std::array<Index, 8> children)
	{
		Direction dir{};
		for (Index child : children) {
			dir += view_direction_[child.pos][child.offset];
		}
		view_direction_[node.pos][node.offset] = dir;
	}

	// TODO: Implement

 protected:
	TreeContainer<ViewDirectionBlock<Dim, BF>> view_direction_;
};
}  // namespace ufo

#endif  // UFO_MAP_VIEW_DIRECTION_MAP_HPP