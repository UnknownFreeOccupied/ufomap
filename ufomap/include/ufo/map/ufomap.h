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

#ifndef UFO_MAP_UFO_MAP_H
#define UFO_MAP_UFO_MAP_H

// UFO
#include <ufo/map/color/color_map_base.h>
#include <ufo/map/distance/distance_map_base.h>
#include <ufo/map/empty/empty_map.h>
#include <ufo/map/io.h>
#include <ufo/map/occupancy/occupancy_map_base.h>
#include <ufo/map/octree/octree_map_base.h>
#include <ufo/map/semantic/semantic_map_base.h>
#include <ufo/map/surfel/surfel_map_base.h>
#include <ufo/map/time/time_map_base.h>

// STL
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <type_traits>

namespace ufo::map
{
//
// Helpers
//

using mt_t = std::uint64_t;

template <mt_t MT, mt_t T, mt_t HRT, template <class> class Node>
using cond_node_t = std::conditional_t<MT & T, Node<MT ^ T & HRT>, EmptyNode<T>>;

template <bool C, mt_t T, template <typename...> typename MapBase>
struct cond_map_base {
	template <typename... ts>
	using type = MapBase<ts...>;
};

template <mt_t T, template <typename...> typename MapBase>
struct cond_map_base<false, T, MapBase> {
	template <typename... Ts>
	using type = EmptyMap<T, Ts...>;
};

//
// Map types
//

enum MapType : mt_t {
	// clang-format off
	OCCUPANCY = mt_t(1),
	TIME      = mt_t(1) << 1U,
	COLOR     = mt_t(1) << 2U,
	SEMANTIC  = mt_t(1) << 3U,
	SURFEL    = mt_t(1) << 4U,
	DISTANCE  = mt_t(1) << 5U,
	INTENSITY = mt_t(1) << 6U,
	// Half resolution (/single) below
	HR_OCCUPANCY = (mt_t(1) << 63U) | OCCUPANCY,
	HR_TIME      = (mt_t(1) << 62U) | TIME,
	HR_COLOR     = (mt_t(1) << 61U) | COLOR,
	HR_SEMANTIC  = (mt_t(1) << 60U) | SEMANTIC,
	HR_SURFEL    = (mt_t(1) << 59U) | SURFEL,
	HR_DISTANCE  = (mt_t(1) << 58U) | DISTANCE,
	HR_INTENSITY = (mt_t(1) << 57U) | INTENSITY,
	// clang-format on
};

//
// UFOMap
//

template <mt_t MapType, bool ReuseNodes = false, bool LockLess = false>
class UFOMap
    : public OctreeMapBase<
          // clang-format off
          OctreeNodeBase<cond_node_t<MapType, SEMANTIC,  HR_SEMANTIC,  SemanticNode>,
                         cond_node_t<MapType, SURFEL,    HR_SURFEL,    SurfelNode>,
                         cond_node_t<MapType, DISTANCE,  HR_DISTANCE,  DistanceNode>,
                         cond_node_t<MapType, TIME,      HR_TIME,      TimeNode>,
                         cond_node_t<MapType, COLOR,     HR_COLOR,     ColorNode>,
                         cond_node_t<MapType, OCCUPANCY, HR_OCCUPANCY, OccupancyNode>>,
          ReuseNodes, LockLess,
          cond_map_base<MapType & OCCUPANCY, OCCUPANCY, OccupancyMapBase>::template type,
          cond_map_base<MapType & TIME,      TIME,      TimeMapBase>::template type,
          cond_map_base<MapType & COLOR,     COLOR,     ColorMapBase>::template type,
          cond_map_base<MapType & SEMANTIC,  SEMANTIC,  SemanticMapBase>::template type,
          cond_map_base<MapType & SURFEL,    SURFEL,    SurfelMapBase>::template type,
          cond_map_base<MapType & DISTANCE,  DISTANCE,  DistanceMapBase>::template type
          // clang-format on
          >
{
 private:
	using Base = OctreeMapBase<
	    // clang-format off
	    OctreeNodeBase<cond_node_t<MapType, SEMANTIC,  HR_SEMANTIC,  SemanticNode>,
	                   cond_node_t<MapType, SURFEL,    HR_SURFEL,    SurfelNode>,
	                   cond_node_t<MapType, DISTANCE,  HR_DISTANCE,  DistanceNode>,
	                   cond_node_t<MapType, TIME,      HR_TIME,      TimeNode>,
	                   cond_node_t<MapType, COLOR,     HR_COLOR,     ColorNode>,
	                   cond_node_t<MapType, OCCUPANCY, HR_OCCUPANCY, OccupancyNode>>,
	    ReuseNodes, LockLess,
	    cond_map_base<MapType & OCCUPANCY, OCCUPANCY, OccupancyMapBase>::template type,
	    cond_map_base<MapType & TIME,      TIME,      TimeMapBase>::template type,
	    cond_map_base<MapType & COLOR,     COLOR,     ColorMapBase>::template type,
	    cond_map_base<MapType & SEMANTIC,  SEMANTIC,  SemanticMapBase>::template type,
	    cond_map_base<MapType & SURFEL,    SURFEL,    SurfelMapBase>::template type,
	    cond_map_base<MapType & DISTANCE,  DISTANCE,  DistanceMapBase>::template type
	    // clang-format on
	    >;

 public:
	//
	// Constructors
	//

	UFOMap(resolution_t res = 0.1, depth_t depth_levels = 16, bool auto_prune = true)
	    : Base(res, depth_levels, auto_prune)
	{
	}

	UFOMap(std::filesystem::path const& file, bool auto_prune = true)
	    : Base(file, auto_prune)
	{
	}

	UFOMap(std::istream& in, bool auto_prune = true) : Base(in, auto_prune) {}

	UFOMap(UFOMap const& other) = default;

	template <mt_t MapType2, bool ReuseNodes2, bool LockLess2>
	UFOMap(UFOMap<MapType2, ReuseNodes2, LockLess2> const& other) : Base(other)
	{
	}

	UFOMap(UFOMap&& other) = default;

	//
	// Operator assignment
	//

	UFOMap& operator=(UFOMap const& rhs) = default;

	template <mt_t MapType2, bool ReuseNodes2, bool LockLess2>
	UFOMap& operator=(UFOMap<MapType2, ReuseNodes2, LockLess2> const& rhs)
	{
		Base::operator=(rhs);
		return *this;
	}

	UFOMap& operator=(UFOMap&& rhs) = default;
};

//
// Explicitly define common map types
//

// clang-format off
using OccupancyMap = UFOMap<OCCUPANCY>;
using TimeMap      = UFOMap<TIME>;
using ColorMap     = UFOMap<COLOR>;
using SemanticMap  = UFOMap<SEMANTIC>;
using SurfelMap    = UFOMap<SURFEL>;
using DistanceMap  = UFOMap<DISTANCE>;
using IntensityMap = UFOMap<INTENSITY>;
// clang-format on
}  // namespace ufo::map

#endif  // UFO_MAP_UFO_MAP_H