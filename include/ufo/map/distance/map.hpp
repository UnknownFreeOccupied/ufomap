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

#ifndef UFO_MAP_DISTANCE_MAP_HPP
#define UFO_MAP_DISTANCE_MAP_HPP

// UFO
#include <ufo/container/tree/container.hpp>
#include <ufo/container/tree/tree.hpp>
#include <ufo/geometry/distance.hpp>
#include <ufo/geometry/line_segment.hpp>
#include <ufo/map/distance/block.hpp>
#include <ufo/map/distance/info.hpp>
#include <ufo/map/distance/interpolate.hpp>
#include <ufo/map/type.hpp>

// STL
#include <algorithm>
#include <cassert>
#include <cmath>
#include <functional>
#include <limits>
#include <type_traits>
#include <utility>

namespace ufo
{
template <class Derived, class Tree, bool WithBounds = true>
class DistanceMap
{
	template <class Derived2, class Tree2, bool WithBounds2>
	friend class DistanceMap;

	static constexpr auto const BF  = Tree::branchingFactor();
	static constexpr auto const Dim = Tree::dimensions();

	using Block = DistanceBlock<Dim, BF, WithBounds>;

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                        Tags                                         |
	|                                                                                     |
	**************************************************************************************/

	static constexpr MapType const Type = MapType::DISTANCE;

	// Container
	using Index    = typename Tree::Index;
	using Node     = typename Tree::Node;
	using Code     = typename Tree::Code;
	using Key      = typename Tree::Key;
	using Point    = typename Tree::Point;
	using Coord    = typename Tree::Coord;
	using coord_t  = typename Tree::coord_t;
	using depth_t  = typename Tree::depth_t;
	using offset_t = typename Tree::offset_t;
	using length_t = typename Tree::length_t;
	using pos_t    = typename Tree::pos_t;

	using DistanceBounds = typename Block::Bounds;

	/**************************************************************************************
	|                                                                                     |
	|                                       Access                                        |
	|                                                                                     |
	**************************************************************************************/

	template <class Geometry>
	[[nodiscard]] bool intersects(Index node, Geometry const& query) const
	{
		// TODO: Implement
		return false;
	}

	template <class Geometry>
	[[nodiscard]] float distanceSquared(Index node, Geometry const& query) const
	{
		if (!distanceContainsSurface(node)) {
			return -1.0f;
		}

		if (intersects(node, query)) {
			return 0.0f;
		}

		if constexpr (2 == Dim) {
			constexpr std::array<std::array<std::uint8_t, 25>, ipow(2, 8)> const lut{
			    std::array<std::uint8_t, 25>{2, 4, 4},
			    std::array<std::uint8_t, 25>{4, 0, 9, 4, 9},
			    std::array<std::uint8_t, 25>{2, 1, 4},
			    std::array<std::uint8_t, 25>{8, 0, 1, 0, 9, 1, 4, 4, 9},
			    std::array<std::uint8_t, 25>{4, 2, 10, 4, 10},
			    std::array<std::uint8_t, 25>{8, 0, 9, 2, 10, 4, 9, 4, 10},
			    std::array<std::uint8_t, 25>{8, 1, 2, 1, 4, 2, 10, 4, 10},
			    std::array<std::uint8_t, 25>{12, 0, 1, 0, 9, 1, 2, 2, 10, 4, 9, 4, 10},
			    std::array<std::uint8_t, 25>{2, 3, 4},
			    std::array<std::uint8_t, 25>{8, 0, 3, 0, 9, 3, 4, 4, 9},
			    std::array<std::uint8_t, 25>{8, 1, 4, 1, 9, 3, 4, 3, 9},
			    std::array<std::uint8_t, 25>{8, 0, 1, 0, 3, 1, 4, 3, 4},
			    std::array<std::uint8_t, 25>{6, 2, 10, 3, 4, 4, 10},
			    std::array<std::uint8_t, 25>{12, 0, 3, 0, 9, 2, 10, 3, 4, 4, 9, 4, 10},
			    std::array<std::uint8_t, 25>{12, 1, 2, 1, 9, 2, 10, 3, 4, 3, 9, 4, 10},
			    std::array<std::uint8_t, 25>{12, 0, 1, 0, 3, 1, 2, 2, 10, 3, 4, 4, 10},
			    std::array<std::uint8_t, 25>{2, 4, 5},
			    std::array<std::uint8_t, 25>{6, 0, 9, 4, 5, 4, 9},
			    std::array<std::uint8_t, 25>{8, 1, 4, 1, 10, 4, 5, 5, 10},
			    std::array<std::uint8_t, 25>{12, 0, 1, 0, 9, 1, 10, 4, 5, 4, 9, 5, 10},
			    std::array<std::uint8_t, 25>{8, 2, 5, 2, 10, 4, 5, 4, 10},
			    std::array<std::uint8_t, 25>{12, 0, 9, 2, 5, 2, 10, 4, 5, 4, 9, 4, 10},
			    std::array<std::uint8_t, 25>{8, 1, 2, 1, 4, 2, 5, 4, 5},
			    std::array<std::uint8_t, 25>{12, 0, 1, 0, 9, 1, 2, 2, 5, 4, 5, 4, 9},
			    std::array<std::uint8_t, 25>{4, 3, 4, 4, 5},
			    std::array<std::uint8_t, 25>{10, 0, 3, 0, 9, 3, 4, 4, 5, 4, 9},
			    std::array<std::uint8_t, 25>{12, 1, 9, 1, 10, 3, 4, 3, 9, 4, 5, 5, 10},
			    std::array<std::uint8_t, 25>{12, 0, 1, 0, 3, 1, 10, 3, 4, 4, 5, 5, 10},
			    std::array<std::uint8_t, 25>{10, 2, 5, 2, 10, 3, 4, 4, 5, 4, 10},
			    std::array<std::uint8_t, 25>{16, 0, 3, 0, 9, 2, 5, 2, 10, 3, 4, 4, 5, 4, 9, 4,
			                                 10},
			    std::array<std::uint8_t, 25>{12, 1, 2, 1, 9, 2, 5, 3, 4, 3, 9, 4, 5},
			    std::array<std::uint8_t, 25>{12, 0, 1, 0, 3, 1, 2, 2, 5, 3, 4, 4, 5},
			    std::array<std::uint8_t, 25>{4, 4, 11, 6, 11},
			    std::array<std::uint8_t, 25>{8, 0, 9, 4, 9, 4, 11, 6, 11},
			    std::array<std::uint8_t, 25>{6, 1, 4, 4, 11, 6, 11},
			    std::array<std::uint8_t, 25>{12, 0, 1, 0, 9, 1, 4, 4, 9, 4, 11, 6, 11},
			    std::array<std::uint8_t, 25>{8, 2, 10, 4, 10, 4, 11, 6, 11},
			    std::array<std::uint8_t, 25>{12, 0, 9, 2, 10, 4, 9, 4, 10, 4, 11, 6, 11},
			    std::array<std::uint8_t, 25>{12, 1, 2, 1, 4, 2, 10, 4, 10, 4, 11, 6, 11},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 9, 1, 2, 2, 10, 4, 9, 4, 10, 4, 11, 6,
			                                 11},
			    std::array<std::uint8_t, 25>{8, 3, 4, 3, 6, 4, 11, 6, 11},
			    std::array<std::uint8_t, 25>{12, 0, 3, 0, 9, 3, 6, 4, 9, 4, 11, 6, 11},
			    std::array<std::uint8_t, 25>{12, 1, 4, 1, 9, 3, 6, 3, 9, 4, 11, 6, 11},
			    std::array<std::uint8_t, 25>{12, 0, 1, 0, 3, 1, 4, 3, 6, 4, 11, 6, 11},
			    std::array<std::uint8_t, 25>{12, 2, 10, 3, 4, 3, 6, 4, 10, 4, 11, 6, 11},
			    std::array<std::uint8_t, 25>{16, 0, 3, 0, 9, 2, 10, 3, 6, 4, 9, 4, 10, 4, 11, 6,
			                                 11},
			    std::array<std::uint8_t, 25>{16, 1, 2, 1, 9, 2, 10, 3, 6, 3, 9, 4, 10, 4, 11, 6,
			                                 11},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 3, 1, 2, 2, 10, 3, 6, 4, 10, 4, 11, 6,
			                                 11},
			    std::array<std::uint8_t, 25>{6, 4, 5, 4, 11, 6, 11},
			    std::array<std::uint8_t, 25>{10, 0, 9, 4, 5, 4, 9, 4, 11, 6, 11},
			    std::array<std::uint8_t, 25>{12, 1, 4, 1, 10, 4, 5, 4, 11, 5, 10, 6, 11},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 9, 1, 10, 4, 5, 4, 9, 4, 11, 5, 10, 6,
			                                 11},
			    std::array<std::uint8_t, 25>{12, 2, 5, 2, 10, 4, 5, 4, 10, 4, 11, 6, 11},
			    std::array<std::uint8_t, 25>{16, 0, 9, 2, 5, 2, 10, 4, 5, 4, 9, 4, 10, 4, 11, 6,
			                                 11},
			    std::array<std::uint8_t, 25>{12, 1, 2, 1, 4, 2, 5, 4, 5, 4, 11, 6, 11},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 9, 1, 2, 2, 5, 4, 5, 4, 9, 4, 11, 6,
			                                 11},
			    std::array<std::uint8_t, 25>{10, 3, 4, 3, 6, 4, 5, 4, 11, 6, 11},
			    std::array<std::uint8_t, 25>{14, 0, 3, 0, 9, 3, 6, 4, 5, 4, 9, 4, 11, 6, 11},
			    std::array<std::uint8_t, 25>{16, 1, 9, 1, 10, 3, 6, 3, 9, 4, 5, 4, 11, 5, 10, 6,
			                                 11},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 3, 1, 10, 3, 6, 4, 5, 4, 11, 5, 10, 6,
			                                 11},
			    std::array<std::uint8_t, 25>{16, 2, 5, 2, 10, 3, 4, 3, 6, 4, 5, 4, 10, 4, 11, 6,
			                                 11},
			    std::array<std::uint8_t, 25>{20, 0, 3, 0, 9, 2,  5, 2,  10, 3, 6,
			                                 4,  5, 4, 9, 4, 10, 4, 11, 6,  11},
			    std::array<std::uint8_t, 25>{16, 1, 2, 1, 9, 2, 5, 3, 6, 3, 9, 4, 5, 4, 11, 6,
			                                 11},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 3, 1, 2, 2, 5, 3, 6, 4, 5, 4, 11, 6,
			                                 11},
			    std::array<std::uint8_t, 25>{2, 4, 7},
			    std::array<std::uint8_t, 25>{6, 0, 9, 4, 7, 4, 9},
			    std::array<std::uint8_t, 25>{4, 1, 4, 4, 7},
			    std::array<std::uint8_t, 25>{10, 0, 1, 0, 9, 1, 4, 4, 7, 4, 9},
			    std::array<std::uint8_t, 25>{6, 2, 10, 4, 7, 4, 10},
			    std::array<std::uint8_t, 25>{10, 0, 9, 2, 10, 4, 7, 4, 9, 4, 10},
			    std::array<std::uint8_t, 25>{10, 1, 2, 1, 4, 2, 10, 4, 7, 4, 10},
			    std::array<std::uint8_t, 25>{14, 0, 1, 0, 9, 1, 2, 2, 10, 4, 7, 4, 9, 4, 10},
			    std::array<std::uint8_t, 25>{8, 3, 4, 3, 11, 4, 7, 7, 11},
			    std::array<std::uint8_t, 25>{12, 0, 3, 0, 9, 3, 11, 4, 7, 4, 9, 7, 11},
			    std::array<std::uint8_t, 25>{12, 1, 4, 1, 9, 3, 9, 3, 11, 4, 7, 7, 11},
			    std::array<std::uint8_t, 25>{12, 0, 1, 0, 3, 1, 4, 3, 11, 4, 7, 7, 11},
			    std::array<std::uint8_t, 25>{12, 2, 10, 3, 4, 3, 11, 4, 7, 4, 10, 7, 11},
			    std::array<std::uint8_t, 25>{16, 0, 3, 0, 9, 2, 10, 3, 11, 4, 7, 4, 9, 4, 10, 7,
			                                 11},
			    std::array<std::uint8_t, 25>{16, 1, 2, 1, 9, 2, 10, 3, 9, 3, 11, 4, 7, 4, 10, 7,
			                                 11},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 3, 1, 2, 2, 10, 3, 11, 4, 7, 4, 10, 7,
			                                 11},
			    std::array<std::uint8_t, 25>{8, 4, 5, 4, 7, 5, 12, 7, 12},
			    std::array<std::uint8_t, 25>{12, 0, 9, 4, 5, 4, 7, 4, 9, 5, 12, 7, 12},
			    std::array<std::uint8_t, 25>{12, 1, 4, 1, 10, 4, 7, 5, 10, 5, 12, 7, 12},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 9, 1, 10, 4, 7, 4, 9, 5, 10, 5, 12, 7,
			                                 12},
			    std::array<std::uint8_t, 25>{12, 2, 5, 2, 10, 4, 7, 4, 10, 5, 12, 7, 12},
			    std::array<std::uint8_t, 25>{16, 0, 9, 2, 5, 2, 10, 4, 7, 4, 9, 4, 10, 5, 12, 7,
			                                 12},
			    std::array<std::uint8_t, 25>{12, 1, 2, 1, 4, 2, 5, 4, 7, 5, 12, 7, 12},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 9, 1, 2, 2, 5, 4, 7, 4, 9, 5, 12, 7,
			                                 12},
			    std::array<std::uint8_t, 25>{12, 3, 4, 3, 11, 4, 5, 5, 12, 7, 11, 7, 12},
			    std::array<std::uint8_t, 25>{16, 0, 3, 0, 9, 3, 11, 4, 5, 4, 9, 5, 12, 7, 11, 7,
			                                 12},
			    std::array<std::uint8_t, 25>{16, 1, 9, 1, 10, 3, 9, 3, 11, 5, 10, 5, 12, 7, 11,
			                                 7, 12},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 3, 1, 10, 3, 11, 5, 10, 5, 12, 7, 11,
			                                 7, 12},
			    std::array<std::uint8_t, 25>{16, 2, 5, 2, 10, 3, 4, 3, 11, 4, 10, 5, 12, 7, 11,
			                                 7, 12},
			    std::array<std::uint8_t, 25>{20, 0, 3, 0,  9, 2,  5, 2,  10, 3, 11,
			                                 4,  9, 4, 10, 5, 12, 7, 11, 7,  12},
			    std::array<std::uint8_t, 25>{16, 1, 2, 1, 9, 2, 5, 3, 9, 3, 11, 5, 12, 7, 11, 7,
			                                 12},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 3, 1, 2, 2, 5, 3, 11, 5, 12, 7, 11, 7,
			                                 12},
			    std::array<std::uint8_t, 25>{8, 4, 7, 4, 11, 6, 7, 6, 11},
			    std::array<std::uint8_t, 25>{12, 0, 9, 4, 7, 4, 9, 4, 11, 6, 7, 6, 11},
			    std::array<std::uint8_t, 25>{10, 1, 4, 4, 7, 4, 11, 6, 7, 6, 11},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 9, 1, 4, 4, 7, 4, 9, 4, 11, 6, 7, 6,
			                                 11},
			    std::array<std::uint8_t, 25>{12, 2, 10, 4, 7, 4, 10, 4, 11, 6, 7, 6, 11},
			    std::array<std::uint8_t, 25>{16, 0, 9, 2, 10, 4, 7, 4, 9, 4, 10, 4, 11, 6, 7, 6,
			                                 11},
			    std::array<std::uint8_t, 25>{16, 1, 2, 1, 4, 2, 10, 4, 7, 4, 10, 4, 11, 6, 7, 6,
			                                 11},
			    std::array<std::uint8_t, 25>{20, 0, 1, 0,  9, 1,  2, 2, 10, 4, 7,
			                                 4,  9, 4, 10, 4, 11, 6, 7, 6,  11},
			    std::array<std::uint8_t, 25>{8, 3, 4, 3, 6, 4, 7, 6, 7},
			    std::array<std::uint8_t, 25>{12, 0, 3, 0, 9, 3, 6, 4, 7, 4, 9, 6, 7},
			    std::array<std::uint8_t, 25>{12, 1, 4, 1, 9, 3, 6, 3, 9, 4, 7, 6, 7},
			    std::array<std::uint8_t, 25>{12, 0, 1, 0, 3, 1, 4, 3, 6, 4, 7, 6, 7},
			    std::array<std::uint8_t, 25>{12, 2, 10, 3, 4, 3, 6, 4, 7, 4, 10, 6, 7},
			    std::array<std::uint8_t, 25>{16, 0, 3, 0, 9, 2, 10, 3, 6, 4, 7, 4, 9, 4, 10, 6,
			                                 7},
			    std::array<std::uint8_t, 25>{16, 1, 2, 1, 9, 2, 10, 3, 6, 3, 9, 4, 7, 4, 10, 6,
			                                 7},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 3, 1, 2, 2, 10, 3, 6, 4, 7, 4, 10, 6,
			                                 7},
			    std::array<std::uint8_t, 25>{12, 4, 5, 4, 11, 5, 12, 6, 7, 6, 11, 7, 12},
			    std::array<std::uint8_t, 25>{16, 0, 9, 4, 5, 4, 9, 4, 11, 5, 12, 6, 7, 6, 11, 7,
			                                 12},
			    std::array<std::uint8_t, 25>{16, 1, 4, 1, 10, 4, 11, 5, 10, 5, 12, 6, 7, 6, 11,
			                                 7, 12},
			    std::array<std::uint8_t, 25>{20, 0,  1, 0,  9, 1, 10, 4,  9, 4, 11,
			                                 5,  10, 5, 12, 6, 7, 6,  11, 7, 12},
			    std::array<std::uint8_t, 25>{16, 2, 5, 2, 10, 4, 10, 4, 11, 5, 12, 6, 7, 6, 11,
			                                 7, 12},
			    std::array<std::uint8_t, 25>{20, 0,  9, 2,  5, 2, 10, 4,  9, 4, 10,
			                                 4,  11, 5, 12, 6, 7, 6,  11, 7, 12},
			    std::array<std::uint8_t, 25>{16, 1, 2, 1, 4, 2, 5, 4, 11, 5, 12, 6, 7, 6, 11, 7,
			                                 12},
			    std::array<std::uint8_t, 25>{20, 0,  1, 0,  9, 1, 2, 2,  5, 4, 9,
			                                 4,  11, 5, 12, 6, 7, 6, 11, 7, 12},
			    std::array<std::uint8_t, 25>{12, 3, 4, 3, 6, 4, 5, 5, 12, 6, 7, 7, 12},
			    std::array<std::uint8_t, 25>{16, 0, 3, 0, 9, 3, 6, 4, 5, 4, 9, 5, 12, 6, 7, 7,
			                                 12},
			    std::array<std::uint8_t, 25>{16, 1, 9, 1, 10, 3, 6, 3, 9, 5, 10, 5, 12, 6, 7, 7,
			                                 12},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 3, 1, 10, 3, 6, 5, 10, 5, 12, 6, 7, 7,
			                                 12},
			    std::array<std::uint8_t, 25>{16, 2, 5, 2, 10, 3, 4, 3, 6, 4, 10, 5, 12, 6, 7, 7,
			                                 12},
			    std::array<std::uint8_t, 25>{20, 0, 3, 0,  9, 2,  5, 2, 10, 3, 6,
			                                 4,  9, 4, 10, 5, 12, 6, 7, 7,  12},
			    std::array<std::uint8_t, 25>{16, 1, 2, 1, 9, 2, 5, 3, 6, 3, 9, 5, 12, 6, 7, 7,
			                                 12},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 3, 1, 2, 2, 5, 3, 6, 5, 12, 6, 7, 7,
			                                 12},
			    std::array<std::uint8_t, 25>{4, 4, 12, 8, 12},
			    std::array<std::uint8_t, 25>{8, 0, 9, 4, 9, 4, 12, 8, 12},
			    std::array<std::uint8_t, 25>{6, 1, 4, 4, 12, 8, 12},
			    std::array<std::uint8_t, 25>{12, 0, 1, 0, 9, 1, 4, 4, 9, 4, 12, 8, 12},
			    std::array<std::uint8_t, 25>{8, 2, 10, 4, 10, 4, 12, 8, 12},
			    std::array<std::uint8_t, 25>{12, 0, 9, 2, 10, 4, 9, 4, 10, 4, 12, 8, 12},
			    std::array<std::uint8_t, 25>{12, 1, 2, 1, 4, 2, 10, 4, 10, 4, 12, 8, 12},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 9, 1, 2, 2, 10, 4, 9, 4, 10, 4, 12, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{6, 3, 4, 4, 12, 8, 12},
			    std::array<std::uint8_t, 25>{12, 0, 3, 0, 9, 3, 4, 4, 9, 4, 12, 8, 12},
			    std::array<std::uint8_t, 25>{12, 1, 4, 1, 9, 3, 4, 3, 9, 4, 12, 8, 12},
			    std::array<std::uint8_t, 25>{12, 0, 1, 0, 3, 1, 4, 3, 4, 4, 12, 8, 12},
			    std::array<std::uint8_t, 25>{10, 2, 10, 3, 4, 4, 10, 4, 12, 8, 12},
			    std::array<std::uint8_t, 25>{16, 0, 3, 0, 9, 2, 10, 3, 4, 4, 9, 4, 10, 4, 12, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{16, 1, 2, 1, 9, 2, 10, 3, 4, 3, 9, 4, 10, 4, 12, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 3, 1, 2, 2, 10, 3, 4, 4, 10, 4, 12, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{8, 4, 5, 4, 12, 5, 8, 8, 12},
			    std::array<std::uint8_t, 25>{12, 0, 9, 4, 5, 4, 9, 4, 12, 5, 8, 8, 12},
			    std::array<std::uint8_t, 25>{12, 1, 4, 1, 10, 4, 12, 5, 8, 5, 10, 8, 12},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 9, 1, 10, 4, 9, 4, 12, 5, 8, 5, 10, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{12, 2, 5, 2, 10, 4, 10, 4, 12, 5, 8, 8, 12},
			    std::array<std::uint8_t, 25>{16, 0, 9, 2, 5, 2, 10, 4, 9, 4, 10, 4, 12, 5, 8, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{12, 1, 2, 1, 4, 2, 5, 4, 12, 5, 8, 8, 12},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 9, 1, 2, 2, 5, 4, 9, 4, 12, 5, 8, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{10, 3, 4, 4, 5, 4, 12, 5, 8, 8, 12},
			    std::array<std::uint8_t, 25>{16, 0, 3, 0, 9, 3, 4, 4, 5, 4, 9, 4, 12, 5, 8, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{16, 1, 9, 1, 10, 3, 4, 3, 9, 4, 12, 5, 8, 5, 10, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 3, 1, 10, 3, 4, 4, 12, 5, 8, 5, 10, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{14, 2, 5, 2, 10, 3, 4, 4, 10, 4, 12, 5, 8, 8, 12},
			    std::array<std::uint8_t, 25>{20, 0, 3, 0,  9, 2,  5, 2, 10, 3, 4,
			                                 4,  9, 4, 10, 4, 12, 5, 8, 8,  12},
			    std::array<std::uint8_t, 25>{16, 1, 2, 1, 9, 2, 5, 3, 4, 3, 9, 4, 12, 5, 8, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 3, 1, 2, 2, 5, 3, 4, 4, 12, 5, 8, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{8, 4, 11, 4, 12, 6, 11, 8, 12},
			    std::array<std::uint8_t, 25>{12, 0, 9, 4, 9, 4, 11, 4, 12, 6, 11, 8, 12},
			    std::array<std::uint8_t, 25>{10, 1, 4, 4, 11, 4, 12, 6, 11, 8, 12},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 9, 1, 4, 4, 9, 4, 11, 4, 12, 6, 11, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{12, 2, 10, 4, 10, 4, 11, 4, 12, 6, 11, 8, 12},
			    std::array<std::uint8_t, 25>{16, 0, 9, 2, 10, 4, 9, 4, 10, 4, 11, 4, 12, 6, 11,
			                                 8, 12},
			    std::array<std::uint8_t, 25>{16, 1, 2, 1, 4, 2, 10, 4, 10, 4, 11, 4, 12, 6, 11,
			                                 8, 12},
			    std::array<std::uint8_t, 25>{20, 0,  1, 0,  9, 1,  2, 2,  10, 4, 9,
			                                 4,  10, 4, 11, 4, 12, 6, 11, 8,  12},
			    std::array<std::uint8_t, 25>{12, 3, 4, 3, 6, 4, 11, 4, 12, 6, 11, 8, 12},
			    std::array<std::uint8_t, 25>{16, 0, 3, 0, 9, 3, 6, 4, 9, 4, 11, 4, 12, 6, 11, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{16, 1, 4, 1, 9, 3, 6, 3, 9, 4, 11, 4, 12, 6, 11, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 3, 1, 4, 3, 6, 4, 11, 4, 12, 6, 11, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{16, 2, 10, 3, 4, 3, 6, 4, 10, 4, 11, 4, 12, 6, 11,
			                                 8, 12},
			    std::array<std::uint8_t, 25>{20, 0,  3, 0,  9, 2,  10, 3,  6, 4, 9,
			                                 4,  10, 4, 11, 4, 12, 6,  11, 8, 12},
			    std::array<std::uint8_t, 25>{20, 1,  2, 1,  9, 2,  10, 3,  6, 3, 9,
			                                 4,  10, 4, 11, 4, 12, 6,  11, 8, 12},
			    std::array<std::uint8_t, 25>{20, 0,  1, 0,  3, 1,  2, 2,  10, 3, 6,
			                                 4,  10, 4, 11, 4, 12, 6, 11, 8,  12},
			    std::array<std::uint8_t, 25>{12, 4, 5, 4, 11, 4, 12, 5, 8, 6, 11, 8, 12},
			    std::array<std::uint8_t, 25>{16, 0, 9, 4, 5, 4, 9, 4, 11, 4, 12, 5, 8, 6, 11, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{16, 1, 4, 1, 10, 4, 11, 4, 12, 5, 8, 5, 10, 6, 11,
			                                 8, 12},
			    std::array<std::uint8_t, 25>{20, 0,  1, 0, 9, 1,  10, 4,  9, 4, 11,
			                                 4,  12, 5, 8, 5, 10, 6,  11, 8, 12},
			    std::array<std::uint8_t, 25>{16, 2, 5, 2, 10, 4, 10, 4, 11, 4, 12, 5, 8, 6, 11,
			                                 8, 12},
			    std::array<std::uint8_t, 25>{20, 0,  9, 2,  5, 2, 10, 4,  9, 4, 10,
			                                 4,  11, 4, 12, 5, 8, 6,  11, 8, 12},
			    std::array<std::uint8_t, 25>{16, 1, 2, 1, 4, 2, 5, 4, 11, 4, 12, 5, 8, 6, 11, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{20, 0,  1, 0,  9, 1, 2, 2,  5, 4, 9,
			                                 4,  11, 4, 12, 5, 8, 6, 11, 8, 12},
			    std::array<std::uint8_t, 25>{16, 3, 4, 3, 6, 4, 5, 4, 11, 4, 12, 5, 8, 6, 11, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{20, 0,  3, 0,  9, 3, 6, 4,  5, 4, 9,
			                                 4,  11, 4, 12, 5, 8, 6, 11, 8, 12},
			    std::array<std::uint8_t, 25>{20, 1,  9, 1, 10, 3,  6, 3,  9, 4, 11,
			                                 4,  12, 5, 8, 5,  10, 6, 11, 8, 12},
			    std::array<std::uint8_t, 25>{20, 0,  1, 0, 3, 1,  10, 3,  6, 4, 11,
			                                 4,  12, 5, 8, 5, 10, 6,  11, 8, 12},
			    std::array<std::uint8_t, 25>{20, 2,  5, 2,  10, 3, 4, 3,  6, 4, 10,
			                                 4,  11, 4, 12, 5,  8, 6, 11, 8, 12},
			    std::array<std::uint8_t, 25>{24, 0,  3, 0,  9, 2,  5, 2, 10, 3,  6, 4, 9,
			                                 4,  10, 4, 11, 4, 12, 5, 8, 6,  11, 8, 12},
			    std::array<std::uint8_t, 25>{20, 1,  2, 1,  9, 2, 5, 3,  6, 3, 9,
			                                 4,  11, 4, 12, 5, 8, 6, 11, 8, 12},
			    std::array<std::uint8_t, 25>{20, 0,  1, 0,  3, 1, 2, 2,  5, 3, 6,
			                                 4,  11, 4, 12, 5, 8, 6, 11, 8, 12},
			    std::array<std::uint8_t, 25>{8, 4, 7, 4, 12, 7, 8, 8, 12},
			    std::array<std::uint8_t, 25>{12, 0, 9, 4, 7, 4, 9, 4, 12, 7, 8, 8, 12},
			    std::array<std::uint8_t, 25>{10, 1, 4, 4, 7, 4, 12, 7, 8, 8, 12},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 9, 1, 4, 4, 7, 4, 9, 4, 12, 7, 8, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{12, 2, 10, 4, 7, 4, 10, 4, 12, 7, 8, 8, 12},
			    std::array<std::uint8_t, 25>{16, 0, 9, 2, 10, 4, 7, 4, 9, 4, 10, 4, 12, 7, 8, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{16, 1, 2, 1, 4, 2, 10, 4, 7, 4, 10, 4, 12, 7, 8, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{20, 0, 1, 0,  9, 1,  2, 2, 10, 4, 7,
			                                 4,  9, 4, 10, 4, 12, 7, 8, 8,  12},
			    std::array<std::uint8_t, 25>{12, 3, 4, 3, 11, 4, 12, 7, 8, 7, 11, 8, 12},
			    std::array<std::uint8_t, 25>{16, 0, 3, 0, 9, 3, 11, 4, 9, 4, 12, 7, 8, 7, 11, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{16, 1, 4, 1, 9, 3, 9, 3, 11, 4, 12, 7, 8, 7, 11, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 3, 1, 4, 3, 11, 4, 12, 7, 8, 7, 11, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{16, 2, 10, 3, 4, 3, 11, 4, 10, 4, 12, 7, 8, 7, 11,
			                                 8, 12},
			    std::array<std::uint8_t, 25>{20, 0,  3, 0,  9, 2, 10, 3,  11, 4, 9,
			                                 4,  10, 4, 12, 7, 8, 7,  11, 8,  12},
			    std::array<std::uint8_t, 25>{20, 1,  2, 1,  9, 2, 10, 3,  9, 3, 11,
			                                 4,  10, 4, 12, 7, 8, 7,  11, 8, 12},
			    std::array<std::uint8_t, 25>{20, 0,  1, 0,  3, 1, 2, 2,  10, 3, 11,
			                                 4,  10, 4, 12, 7, 8, 7, 11, 8,  12},
			    std::array<std::uint8_t, 25>{8, 4, 5, 4, 7, 5, 8, 7, 8},
			    std::array<std::uint8_t, 25>{12, 0, 9, 4, 5, 4, 7, 4, 9, 5, 8, 7, 8},
			    std::array<std::uint8_t, 25>{12, 1, 4, 1, 10, 4, 7, 5, 8, 5, 10, 7, 8},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 9, 1, 10, 4, 7, 4, 9, 5, 8, 5, 10, 7,
			                                 8},
			    std::array<std::uint8_t, 25>{12, 2, 5, 2, 10, 4, 7, 4, 10, 5, 8, 7, 8},
			    std::array<std::uint8_t, 25>{16, 0, 9, 2, 5, 2, 10, 4, 7, 4, 9, 4, 10, 5, 8, 7,
			                                 8},
			    std::array<std::uint8_t, 25>{12, 1, 2, 1, 4, 2, 5, 4, 7, 5, 8, 7, 8},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 9, 1, 2, 2, 5, 4, 7, 4, 9, 5, 8, 7,
			                                 8},
			    std::array<std::uint8_t, 25>{12, 3, 4, 3, 11, 4, 5, 5, 8, 7, 8, 7, 11},
			    std::array<std::uint8_t, 25>{16, 0, 3, 0, 9, 3, 11, 4, 5, 4, 9, 5, 8, 7, 8, 7,
			                                 11},
			    std::array<std::uint8_t, 25>{16, 1, 9, 1, 10, 3, 9, 3, 11, 5, 8, 5, 10, 7, 8, 7,
			                                 11},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 3, 1, 10, 3, 11, 5, 8, 5, 10, 7, 8, 7,
			                                 11},
			    std::array<std::uint8_t, 25>{16, 2, 5, 2, 10, 3, 4, 3, 11, 4, 10, 5, 8, 7, 8, 7,
			                                 11},
			    std::array<std::uint8_t, 25>{20, 0, 3, 0,  9, 2, 5, 2, 10, 3, 11,
			                                 4,  9, 4, 10, 5, 8, 7, 8, 7,  11},
			    std::array<std::uint8_t, 25>{16, 1, 2, 1, 9, 2, 5, 3, 9, 3, 11, 5, 8, 7, 8, 7,
			                                 11},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 3, 1, 2, 2, 5, 3, 11, 5, 8, 7, 8, 7,
			                                 11},
			    std::array<std::uint8_t, 25>{12, 4, 11, 4, 12, 6, 7, 6, 11, 7, 8, 8, 12},
			    std::array<std::uint8_t, 25>{16, 0, 9, 4, 9, 4, 11, 4, 12, 6, 7, 6, 11, 7, 8, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{14, 1, 4, 4, 11, 4, 12, 6, 7, 6, 11, 7, 8, 8, 12},
			    std::array<std::uint8_t, 25>{20, 0,  1, 0, 9, 1,  4, 4, 9, 4, 11,
			                                 4,  12, 6, 7, 6, 11, 7, 8, 8, 12},
			    std::array<std::uint8_t, 25>{16, 2, 10, 4, 10, 4, 11, 4, 12, 6, 7, 6, 11, 7, 8,
			                                 8, 12},
			    std::array<std::uint8_t, 25>{20, 0,  9, 2, 10, 4,  9, 4, 10, 4, 11,
			                                 4,  12, 6, 7, 6,  11, 7, 8, 8,  12},
			    std::array<std::uint8_t, 25>{20, 1,  2, 1, 4, 2,  10, 4, 10, 4, 11,
			                                 4,  12, 6, 7, 6, 11, 7,  8, 8,  12},
			    std::array<std::uint8_t, 25>{24, 0,  1, 0,  9, 1, 2, 2,  10, 4, 9, 4, 10,
			                                 4,  11, 4, 12, 6, 7, 6, 11, 7,  8, 8, 12},
			    std::array<std::uint8_t, 25>{12, 3, 4, 3, 6, 4, 12, 6, 7, 7, 8, 8, 12},
			    std::array<std::uint8_t, 25>{16, 0, 3, 0, 9, 3, 6, 4, 9, 4, 12, 6, 7, 7, 8, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{16, 1, 4, 1, 9, 3, 6, 3, 9, 4, 12, 6, 7, 7, 8, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 3, 1, 4, 3, 6, 4, 12, 6, 7, 7, 8, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{16, 2, 10, 3, 4, 3, 6, 4, 10, 4, 12, 6, 7, 7, 8, 8,
			                                 12},
			    std::array<std::uint8_t, 25>{20, 0,  3, 0,  9, 2, 10, 3, 6, 4, 9,
			                                 4,  10, 4, 12, 6, 7, 7,  8, 8, 12},
			    std::array<std::uint8_t, 25>{20, 1,  2, 1,  9, 2, 10, 3, 6, 3, 9,
			                                 4,  10, 4, 12, 6, 7, 7,  8, 8, 12},
			    std::array<std::uint8_t, 25>{20, 0,  1, 0,  3, 1, 2, 2, 10, 3, 6,
			                                 4,  10, 4, 12, 6, 7, 7, 8, 8,  12},
			    std::array<std::uint8_t, 25>{12, 4, 5, 4, 11, 5, 8, 6, 7, 6, 11, 7, 8},
			    std::array<std::uint8_t, 25>{16, 0, 9, 4, 5, 4, 9, 4, 11, 5, 8, 6, 7, 6, 11, 7,
			                                 8},
			    std::array<std::uint8_t, 25>{16, 1, 4, 1, 10, 4, 11, 5, 8, 5, 10, 6, 7, 6, 11,
			                                 7, 8},
			    std::array<std::uint8_t, 25>{20, 0, 1, 0,  9, 1, 10, 4,  9, 4, 11,
			                                 5,  8, 5, 10, 6, 7, 6,  11, 7, 8},
			    std::array<std::uint8_t, 25>{16, 2, 5, 2, 10, 4, 10, 4, 11, 5, 8, 6, 7, 6, 11,
			                                 7, 8},
			    std::array<std::uint8_t, 25>{20, 0,  9, 2, 5, 2, 10, 4,  9, 4, 10,
			                                 4,  11, 5, 8, 6, 7, 6,  11, 7, 8},
			    std::array<std::uint8_t, 25>{16, 1, 2, 1, 4, 2, 5, 4, 11, 5, 8, 6, 7, 6, 11, 7,
			                                 8},
			    std::array<std::uint8_t, 25>{20, 0,  1, 0, 9, 1, 2, 2,  5, 4, 9,
			                                 4,  11, 5, 8, 6, 7, 6, 11, 7, 8},
			    std::array<std::uint8_t, 25>{12, 3, 4, 3, 6, 4, 5, 5, 8, 6, 7, 7, 8},
			    std::array<std::uint8_t, 25>{16, 0, 3, 0, 9, 3, 6, 4, 5, 4, 9, 5, 8, 6, 7, 7,
			                                 8},
			    std::array<std::uint8_t, 25>{16, 1, 9, 1, 10, 3, 6, 3, 9, 5, 8, 5, 10, 6, 7, 7,
			                                 8},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 3, 1, 10, 3, 6, 5, 8, 5, 10, 6, 7, 7,
			                                 8},
			    std::array<std::uint8_t, 25>{16, 2, 5, 2, 10, 3, 4, 3, 6, 4, 10, 5, 8, 6, 7, 7,
			                                 8},
			    std::array<std::uint8_t, 25>{20, 0, 3, 0,  9, 2, 5, 2, 10, 3, 6,
			                                 4,  9, 4, 10, 5, 8, 6, 7, 7,  8},
			    std::array<std::uint8_t, 25>{16, 1, 2, 1, 9, 2, 5, 3, 6, 3, 9, 5, 8, 6, 7, 7,
			                                 8},
			    std::array<std::uint8_t, 25>{16, 0, 1, 0, 3, 1, 2, 2, 5, 3, 6, 5, 8, 6, 7, 7,
			                                 8}};

			QuadKey key         = derived().key(node);
			auto    depth       = derived().depth(key);
			auto    center      = derived().center(key);
			auto    half_length = derived().halfLength(key);

			QuadKey::Key k = static_cast<QuadKey::Key>(key);
			--k.x;
			--k.y;

			std::array<Index, 8> nodes{derived().index(QuadKey(k + QuadKey::Key(0, 0), depth)),
			                           derived().index(QuadKey(k + QuadKey::Key(1, 0), depth)),
			                           derived().index(QuadKey(k + QuadKey::Key(2, 0), depth)),
			                           derived().index(QuadKey(k + QuadKey::Key(0, 1), depth)),
			                           derived().index(QuadKey(k + QuadKey::Key(2, 1), depth)),
			                           derived().index(QuadKey(k + QuadKey::Key(0, 2), depth)),
			                           derived().index(QuadKey(k + QuadKey::Key(1, 2), depth)),
			                           derived().index(QuadKey(k + QuadKey::Key(2, 2), depth))};

			unsigned idx{};
			for (unsigned i{}; 8 > i; ++i) {
				idx |= static_cast<unsigned>(distanceContainsSurface(nodes[i])) << i;
			}

			std::array<Point, 13> points{distance_[nodes[0].pos][nodes[0].offset].point,
			                             distance_[nodes[1].pos][nodes[1].offset].point,
			                             distance_[nodes[2].pos][nodes[2].offset].point,
			                             distance_[nodes[3].pos][nodes[3].offset].point,
			                             distance_[node.pos][node.offset].point,
			                             distance_[nodes[4].pos][nodes[4].offset].point,
			                             distance_[nodes[5].pos][nodes[5].offset].point,
			                             distance_[nodes[6].pos][nodes[6].offset].point,
			                             distance_[nodes[7].pos][nodes[7].offset].point,
			                             center + Point(-half_length, -half_length),
			                             center + Point(half_length, -half_length),
			                             center + Point(-half_length, half_length),
			                             center + Point(half_length, half_length)};

			float distance = std::numeric_limits<float>::max();
			for (unsigned i = 1u; lut[idx][0] > i; i += 2) {
				LineSegment<Dim, float> line(points[lut[idx][i]], points[lut[idx][i + 1u]]);
				distance = std::min(distance, ufo::distanceSquared(line, query));
			}

			return distance;
		} else if constexpr (3 == Dim) {
			// TODO: Implement
		}
	}

	template <class Geometry>
	[[nodiscard]] float distanceSomething(Index node, Geometry const& query) const
	{
		return std::sqrt(distanceSquared(node, query));
	}

	template <class Geometry, class Predicate = pred::Leaf,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	[[nodiscard]] float distance(
	    Geometry const& query, Predicate const& pred = pred::Leaf(),
	    DistanceInterpolate interpolate = DistanceInterpolate::ALL,
	    float max_distance = std::numeric_limits<float>::max(), float epsilon = 0.0f,
	    NearestSearchAlgorithm search_alg = NearestSearchAlgorithm::DEPTH_FIRST) const
	{
		return distance(derived().index(), pred, query, interpolate, max_distance, epsilon,
		                search_alg);
	}

	template <class NodeType, class Pred, class Geometry,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool>              = true>
	[[nodiscard]] float distance(
	    NodeType node, Predicate const& pred, Geometry const& query,
	    DistanceInterpolate interpolate = DistanceInterpolate::ALL,
	    float max_distance = std::numeric_limits<float>::max(), float epsilon = 0.0f,
	    NearestSearchAlgorithm search_alg = NearestSearchAlgorithm::DEPTH_FIRST) const
	{
		// TODO: Implement
		return 0.0f;
	}

	template <bool FastAsSonic, class Geometry>
	[[nodiscard]] float distance(
	    Geometry const& query, DistanceInterpolate interpolate = DistanceInterpolate::ALL,
	    float max_distance = std::numeric_limits<float>::max(), float epsilon = 0.0f,
	    NearestSearchAlgorithm search_alg = NearestSearchAlgorithm::DEPTH_FIRST) const
	{
		return distance<FastAsSonic>(derived().index(), query, interpolate, max_distance,
		                             epsilon, search_alg);
	}

	template <bool FastAsSonic, class NodeType, class Geometry,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] float distance(
	    NodeType node, Geometry const& query,
	    DistanceInterpolate interpolate = DistanceInterpolate::ALL,
	    float max_distance = std::numeric_limits<float>::max(), float epsilon = 0.0f,
	    NearestSearchAlgorithm search_alg = NearestSearchAlgorithm::DEPTH_FIRST) const
	{
		auto n = derived().index(node);

		// TODO: Implement

		auto value_f = [this, &query](Index node) {
			return derived().isLeaf(node)
			           ? std::numeric_limits<float>::infinity()
			           : [this, &query](Index node) {
				             if constexpr (std::is_same_v<Vec<Dim, float>,
				                                          std::decay_t<Geometry>>) {
					             auto p = distance_[node.pos][node.offset].point;
					             p.x -= query.x;
					             p.x *= p.x;
					             p.y -= query.y;
					             p.y *= p.y;
					             // p.z -= query.z;
					             // p.z *= p.z;
					             return p.x + p.y;  // + p.z;
				             } else {
					             // TODO: Implement
				             }
			             }(node);
		};

		auto inner_f = [this, &query](Index node) {
			if constexpr (std::is_same_v<Vec<Dim, float>, std::decay_t<Geometry>>) {
				float x = UFO_CLAMP(query.x, distance_[node.pos].bounds(node.offset).min.x,
				                    distance_[node.pos].bounds(node.offset).max.x);
				float y = UFO_CLAMP(query.y, distance_[node.pos].bounds(node.offset).min.y,
				                    distance_[node.pos].bounds(node.offset).max.y);
				// float z = UFO_CLAMP(query.z, distance_[node.pos].bounds(node.offset).min.z,
				//                     distance_[node.pos].bounds(node.offset).max.z);

				x -= query.x;
				x *= x;
				y -= query.y;
				y *= y;
				// z -= query.z;
				// z *= z;

				return x + y;  // + z;
			} else {
				// TODO: Implement
			}
		};

		return std::sqrt(derived().template nearest<true, FastAsSonic>(
		    n, search_alg, value_f, inner_f, max_distance, epsilon));
	}

	template <class Predicate, class Geometry,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	[[nodiscard]] std::pair<float, Point> distancePoint(
	    Predicate const& pred, Geometry const& query,
	    DistanceInterpolate interpolate = DistanceInterpolate::ALL,
	    float max_distance = std::numeric_limits<float>::max(), float epsilon = 0.0f,
	    NearestSearchAlgorithm search_alg = NearestSearchAlgorithm::DEPTH_FIRST) const
	{
		return distancePoint(derived().index(), pred, query, interpolate, max_distance,
		                     epsilon, search_alg);
	}

	template <class NodeType, class Predicate, class Geometry,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool>              = true>
	[[nodiscard]] std::pair<float, Point> distancePoint(
	    NodeType node, Predicate const& pred, Geometry const& query,
	    DistanceInterpolate interpolate = DistanceInterpolate::ALL,
	    float max_distance = std::numeric_limits<float>::max(), float epsilon = 0.0f,
	    NearestSearchAlgorithm search_alg = NearestSearchAlgorithm::DEPTH_FIRST) const
	{
		auto n = derived().index(node);

		// TODO: Implement
	}

	template <bool FastAsSonic, class Geometry>
	[[nodiscard]] std::pair<float, Index> distancePoint(
	    Geometry const& query, DistanceInterpolate interpolate = DistanceInterpolate::ALL,
	    float max_distance = std::numeric_limits<float>::max(), float epsilon = 0.0f,
	    NearestSearchAlgorithm search_alg = NearestSearchAlgorithm::DEPTH_FIRST) const
	{
		return distancePoint<FastAsSonic>(derived().index(), query, interpolate, max_distance,
		                                  epsilon, search_alg);
	}

	template <bool FastAsSonic, class NodeType, class Geometry,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] std::pair<float, Index> distancePoint(
	    NodeType node, Geometry const& query,
	    DistanceInterpolate interpolate = DistanceInterpolate::ALL,
	    float max_distance = std::numeric_limits<float>::max(), float epsilon = 0.0f,
	    NearestSearchAlgorithm search_alg = NearestSearchAlgorithm::DEPTH_FIRST) const
	{
		auto n = derived().index(node);

		// TODO: Implement

		auto value_f = [this, &query](Index node) {
			if constexpr (std::is_same_v<Vec<Dim, float>, std::decay_t<Geometry>>) {
				auto p = distance_[node.pos][node.offset].point;
				p.x -= query.x;
				p.x *= p.x;
				p.y -= query.y;
				p.y *= p.y;
				// p.z -= query.z;
				// p.z *= p.z;
				return p.x + p.y;  // + p.z;
			} else {
				// TODO: Implement
			}
		};

		auto inner_f = [this, &query](Index node) {
			if constexpr (std::is_same_v<Vec<Dim, float>, std::decay_t<Geometry>>) {
				float x = UFO_CLAMP(query.x, distance_[node.pos].bounds(node.offset).min.x,
				                    distance_[node.pos].bounds(node.offset).max.x);
				float y = UFO_CLAMP(query.y, distance_[node.pos].bounds(node.offset).min.y,
				                    distance_[node.pos].bounds(node.offset).max.y);
				// float z = UFO_CLAMP(query.z, distance_[node.pos].bounds(node.offset).min.z,
				//                     distance_[node.pos].bounds(node.offset).max.z);

				x -= query.x;
				x *= x;
				y -= query.y;
				y *= y;
				// z -= query.z;
				// z *= z;

				return x + y;  // + z;
			} else {
				// TODO: Implement
			}
		};

		auto [c_dist, c_node] = derived().template nearest<false, FastAsSonic>(
		    n, search_alg, value_f, inner_f, max_distance, epsilon);

		// return std::pair(std::sqrt(c_dist), distance_[c_node.pos][c_node.offset].point);
		return std::pair(std::sqrt(c_dist), c_node);
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool distanceContainsSurface(NodeType node) const
	{
		auto n = derived().index(node);
		return 0.0f < distance_[node.pos][node.offset].weight;
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] DistanceInfo<Dim> distanceInfo(NodeType node) const
	{
		auto n = derived().index(node);
		return distance_[n.pos][n.offset];
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] DistanceBounds distanceBounds(NodeType node) const
	{
		if constexpr (WithBounds) {
			auto n = derived().index(node);
			return distance_[n.pos].bounds(n.offset);
		} else {
			return derived().bounds(node);
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Modifiers                                      |
	|                                                                                     |
	**************************************************************************************/

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	void distanceSet(NodeType node, DistanceInfo<Dim> info, bool propagate = true)
	{
		assert(isfinite(info.point));
		assert(std::isfinite(info.weight));

		DistanceBounds bounds;

		if (0.0f >= info.weight) {
			info   = Block::resetInfo();
			bounds = Block::resetBounds();
		} else {
			bounds = DistanceBounds(info.point, info.point);
		}

		auto node_f = [this, &info, &bounds](Index node) {
			distance_[node.pos][node.offset]        = info;
			distance_[node.pos].bounds(node.offset) = bounds;
		};

		auto block_f = [this, &info, &bounds](pos_t pos) {
			distance_[pos].fill(info, bounds);
		};

		if constexpr (std::is_same_v<std::decay_t<NodeType>, Index>) {
			if (propagate) {
				derived().recursParentFirst(node, node_f, block_f);
			} else {
				derived().recursLeaves(node, node_f, block_f);
			}
		} else {
			if (propagate) {
				auto update_f = [this](Index node, pos_t children) {
					onUpdateNode(node, children);
				};

				derived().recursLeaves(derived().code(node), node_f, block_f, update_f,
				                       propagate);
			} else {
				derived().recursParentFirst(derived().code(node), node_f, block_f);
			}
		}
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	void distanceSet(NodeType node, Point const& point, float weight = 1.0f,
	                 bool propagate = true)
	{
		distanceSet(node, DistanceInfo<Dim>{point, weight}, propagate);
	}

	template <class NodeType, class UnaryOp,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true,
	          std::enable_if_t<std::is_invocable_r_v<DistanceInfo<Dim>, UnaryOp, Index>,
	                           bool>                                          = true>
	void distanceSet(NodeType node, UnaryOp unary_op, bool propagate = true)
	{
		auto node_f = [this, unary_op](Index node) {
			DistanceInfo<Dim> info = unary_op(node);

			assert(isfinite(info.point));
			assert(std::isfinite(info.weight));

			if (0.0f >= info.weight) {
				distance_[node.pos].reset(node.offset);
			} else {
				distance_[node.pos][node.offset]        = info;
				distance_[node.pos].bounds(node.offset) = DistanceBounds(info.point, info.point);
			}
		};

		auto block_f = [this, node_f](pos_t pos) {
			for (std::size_t i{}; BF > i; ++i) {
				node_f(Index(pos, i));
			}
		};

		auto update_f = [this](Index node, pos_t children) { onUpdateNode(node, children); };

		if constexpr (std::is_same_v<Index, std::decay_t<NodeType>>) {
			if (propagate) {
				derived().recursLeaves(node, node_f, block_f, update_f);
			} else {
				derived().recursLeaves(node, node_f, block_f);
			}
		} else {
			derived().recursLeaves(derived().code(node), node_f, block_f, update_f, propagate);
		}
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	void distanceUpdate(NodeType node, Point const& point, float weight = 1.0f,
	                    bool propagate = true)
	{
		assert(isfinite(point));
		assert(std::isfinite(weight));

		auto node_f = [this, &point, weight](Index node) {
			DistanceInfo<Dim>& cur = distance_[node.pos][node.offset];

			auto w = cur.weight + weight;

			if (0.0f >= w) {
				distance_[node.pos].reset(node.offset);
			} else {
				cur.point  = cur.point * (cur.weight / w) + point * (weight / w);
				cur.weight = w;
				if constexpr (WithBounds) {
					distance_[node.pos].bounds(node.offset) = DistanceBounds(cur.point, cur.point);
				}
			}

			assert(isfinite(cur.point));
			assert(std::isfinite(cur.weight));
		};

		auto block_f = [this, node_f](pos_t pos) {
			for (std::size_t i{}; BF > i; ++i) {
				node_f(Index(pos, i));
			}
		};

		auto update_f = [this](Index node, pos_t children) { onUpdateNode(node, children); };

		if constexpr (std::is_same_v<Index, std::decay_t<NodeType>>) {
			if (propagate) {
				derived().recursLeaves(node, node_f, block_f, update_f);
			} else {
				derived().recursLeaves(node, node_f, block_f);
			}
		} else {
			derived().recursLeaves(derived().code(node), node_f, block_f, update_f, propagate);
		}
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	void distanceReset(NodeType node, bool propagate = true)
	{
		auto info = Block::resetInfo();
		distanceSet(node, info.point, info.weight, propagate);
	}

	template <class NodeType, class UnaryPred,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool>       = true,
	          std::enable_if_t<std::is_invocable_r_v<bool, UnaryPred, Index>, bool> = true>
	void distanceResetIf(NodeType node, UnaryPred p, bool propagate = true)
	{
		auto node_f = [this, p](Index node) {
			if (p) {
				distance_[node.pos].reset(node.offset);
			}
		};

		auto block_f = [this, node_f](pos_t pos) {
			for (std::size_t i{}; BF > i; ++i) {
				node_f(Index(pos, i));
			}
		};

		auto update_f = [this](Index node, pos_t children) { onUpdateNode(node, children); };

		if constexpr (std::is_same_v<Index, std::decay_t<NodeType>>) {
			if (propagate) {
				derived().recursLeaves(node, node_f, block_f, update_f);
			} else {
				derived().recursLeaves(node, node_f, block_f);
			}
		} else {
			derived().recursLeaves(derived().code(node), node_f, block_f, update_f, propagate);
		}
	}

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	DistanceMap() { createRoot(); }

	DistanceMap(DistanceMap const& other) = default;

	DistanceMap(DistanceMap&& other) = default;

	template <class Derived2, class Tree2, bool WithBounds2>
	DistanceMap(DistanceMap<Derived2, Tree2, WithBounds2> const& other)
	    : distance_(other.distance_)
	{
	}

	template <class Derived2, class Tree2, bool WithBounds2>
	DistanceMap(DistanceMap<Derived2, Tree2, WithBounds2>&& other)
	    : distance_(std::move(other.distance_))
	{
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Destructor                                      |
	|                                                                                     |
	**************************************************************************************/

	~DistanceMap() = default;

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	DistanceMap& operator=(DistanceMap const& rhs) = default;

	DistanceMap& operator=(DistanceMap&& rhs) = default;

	template <class Derived2, class Tree2, bool WithBounds2>
	DistanceMap& operator=(DistanceMap<Derived2, Tree2, WithBounds2> const& rhs)
	{
		distance_ = rhs.distance_;
		return *this;
	}

	template <class Derived2, class Tree2, bool WithBounds2>
	DistanceMap& operator=(DistanceMap<Derived2, Tree2, WithBounds2>&& rhs)
	{
		distance_ = std::move(rhs.distance_);
		return *this;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Swap                                         |
	|                                                                                     |
	**************************************************************************************/

	void swap(DistanceMap& other) noexcept(noexcept(distance_.swap(other.distance_)))
	{
		distance_.swap(other.distance_);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Create root                                     |
	|                                                                                     |
	**************************************************************************************/

	void createRoot() { distance_.emplace_back(); }

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
	|                              Functions Derived expect                               |
	|                                                                                     |
	**************************************************************************************/

	void onClear()
	{
		distance_.clear();
		createRoot();
	}

	void onCreateChildren(Index node)
	{
		distance_.emplace_back(distance_[node.pos], node.offset);
	}

	void onFillChildren(Index node, pos_t children)
	{
		distance_[children].fill(distance_[node.pos], node.offset);
	}

	void onPruneChildren(Index node, pos_t /* children */)
	{
		// Update bounds to correspond to the node alone
		auto const& info = distance_[node.pos][node.offset];
		distance_[node.pos].bounds(node.offset) =
		    0.0f == info.weight ? Block::resetBounds()
		                        : DistanceBounds(info.point, info.point);
	}

	void onReserve(std::size_t cap) { distance_.reserve(cap); }

	void onSetSize(std::size_t size) { distance_.setSize(size); }

	void onEnable(std::size_t num_blocks)
	{
		createRoot();
		distance_.resize(num_blocks, distance_[0]);
	}

	void onDisable() { distance_.clear(); }

	void onUpdateNode(Index node, pos_t children)
	{
		DistanceInfo<Dim> info{Point{}, 0.0f};
		DistanceBounds    bounds = Block::resetBounds();

		auto& c = distance_[children];
		for (std::size_t i{}; BF > i; ++i) {
			info.point += c[i].point * c[i].weight;
			info.weight += c[i].weight;
			if constexpr (WithBounds) {
				bounds.min = min(bounds.min, c.bounds(i).min);
				bounds.max = max(bounds.max, c.bounds(i).max);
			}
		}

		info.point =
		    0.0f == info.weight ? Block::resetInfo().point : info.point / info.weight;

		distance_[node.pos][node.offset] = info;
		if constexpr (WithBounds) {
			distance_[node.pos].bounds(node.offset) = bounds;
		}
	}

	[[nodiscard]] bool isPrunable(pos_t block) const
	{
		using std::begin;
		using std::end;
		return std::all_of(begin(distance_[block].info()) + 1, end(distance_[block].info()),
		                   [v = distance_[block][0]](auto const& e) { return v == e; });
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr std::size_t serializedSizeNode() noexcept
	{
		return sizeof(DistanceInfo<Dim>);
	}

	[[nodiscard]] constexpr std::size_t serializedSize(
	    std::vector<std::pair<pos_t, BitSet<BF>>> const& /* nodes */,
	    std::size_t num_nodes) const
	{
		return num_nodes * serializedSizeNode();
	}

	void onRead(ReadBuffer& in, std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes)
	{
		for (auto [block, offset] : nodes) {
			if (offset.all()) {
				in.read(distance_[block].info());
				for (offset_t i{}; BF > i; ++i) {
					distance_[block].bounds(i) =
					    0.0f == distance_[block][i].weight
					        ? Block::resetBounds()
					        : DistanceBounds(distance_[block][i].point, distance_[block][i].point);
				}
			} else {
				for (offset_t i{}; BF > i; ++i) {
					if (offset[i]) {
						in.read(distance_[block][i]);
						distance_[block].bounds(i) = 0.0f == distance_[block][i].weight
						                                 ? Block::resetBounds()
						                                 : DistanceBounds(distance_[block][i].point,
						                                                  distance_[block][i].point);
					}
				}
			}
		}
	}

	void onWrite(WriteBuffer& out, std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes)
	{
		for (auto [block, offset] : nodes) {
			if (offset.all()) {
				out.write(distance_[block].info());
			} else {
				for (offset_t i{}; BF > i; ++i) {
					if (offset[i]) {
						out.write(distance_[block][i]);
					}
				}
			}
		}
	}

	//
	// Dot file info
	//

	void onDotFileInfo(std::ostream& out, Index node) const
	{
		out << "Distance: " << distanceInfo(node);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Distance                                       |
	|                                                                                     |
	**************************************************************************************/

 protected:
	TreeContainer<Block> distance_;
};

template <class Derived, class Tree>
using DistanceMapFast = DistanceMap<Derived, Tree, true>;

template <class Derived, class Tree>
using DistanceMapSmall = DistanceMap<Derived, Tree, false>;
}  // namespace ufo

#endif  // UFO_MAP_DISTANCE_MAP_HPP