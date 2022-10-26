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
#include <ufo/map/color/color_map.h>
#include <ufo/map/distance/distance_map.h>
#include <ufo/map/empty/empty_map.h>
#include <ufo/map/io.h>
#include <ufo/map/occupancy/occupancy_map.h>
#include <ufo/map/octree/octree_map.h>
// #include <ufo/map/semantic/semantic_map.h>
#include <ufo/map/surfel/surfel_map.h>
#include <ufo/map/time/time_map.h>

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

template <mt_t MT, mt_t T, template <class> class Node>
using cond_node_t = std::conditional_t<MT & T, Node<8>, EmptyNode<T>>;

template <bool C, mt_t T, template <typename...> typename Map>
struct cond_map_base {
	template <typename... ts>
	using type = Map<ts...>;
};

template <mt_t T, template <typename...> typename Map>
struct cond_map_base<false, T, Map> {
	template <typename... Ts>
	using type = EmptyMap<T, Ts...>;
};

//
// Map types
//

enum MapType : mt_t {
	// clang-format off
	OCCUPANCY  = mt_t(1),
	TIME       = mt_t(1) << 1,
	COLOR      = mt_t(1) << 2,
	SEMANTIC   = mt_t(1) << 3,
	SURFEL     = mt_t(1) << 4,
	DISTANCE   = mt_t(1) << 5,
	INTENSITY  = mt_t(1) << 6,
	COUNT      = mt_t(1) << 7,
	REFLECTION = mt_t(1) << 8,
	// clang-format on
};

//
// UFOMap
//

template <mt_t MapType, bool ReuseNodes = false, bool LockLess = false,
          bool CountNodes = true>
class UFOMap
    : public OctreeMap<
          // clang-format off
          OctreeNodeBase<
												 // These should be ordered based on size
												 cond_node_t<MapType, SEMANTIC,   SemanticNode>,
                         cond_node_t<MapType, SURFEL,     SurfelNode>,
                         cond_node_t<MapType, DISTANCE,   DistanceNode>,
												 cond_node_t<MapType, REFLECTION, ReflectionNode>,
												 cond_node_t<MapType, COUNT,      CountNode>,
                         cond_node_t<MapType, TIME,       TimeNode>,
												 cond_node_t<MapType, INTENSITY,  IntensityNode>,
                         cond_node_t<MapType, COLOR,      ColorNode>,
                         cond_node_t<MapType, OCCUPANCY,  OccupancyNode>>,
					std::conditional_t<MapType & OCCUPANCY, ContainsOccupancy<8>, void>,
          ReuseNodes, LockLess, CountNodes,
					// Order does not matter
          cond_map_base<MapType & SEMANTIC,   SEMANTIC,   SemanticMap>::template type,
          cond_map_base<MapType & SURFEL,     SURFEL,     SurfelMap>::template type,
          cond_map_base<MapType & DISTANCE,   DISTANCE,   DistanceMap>::template type,
          cond_map_base<MapType & REFLECTION, REFLECTION, ReflectionMap>::template type,
          cond_map_base<MapType & COUNT,      COUNT,      CountMap>::template type,
          cond_map_base<MapType & TIME,       TIME,       TimeMap>::template type,
          cond_map_base<MapType & INTENSITY,  INTENSITY,  IntensityMap>::template type,
          cond_map_base<MapType & COLOR,      COLOR,      ColorMap>::template type,
          cond_map_base<MapType & OCCUPANCY,  OCCUPANCY,  OccupancyMap>::template type
          // clang-format on
          >
{
 private:
	using Base = OctreeMap<
	    // clang-format off
			OctreeNodeBase<
										 cond_node_t<MapType, SEMANTIC,   SemanticNode>,
										 cond_node_t<MapType, SURFEL,     SurfelNode>,
										 cond_node_t<MapType, DISTANCE,   DistanceNode>,
										 cond_node_t<MapType, REFLECTION, ReflectionNode>,
										 cond_node_t<MapType, COUNT,      CountNode>,
										 cond_node_t<MapType, TIME,       TimeNode>,
										 cond_node_t<MapType, INTENSITY,  IntensityNode>,
										 cond_node_t<MapType, COLOR,      ColorNode>,
										 cond_node_t<MapType, OCCUPANCY,  OccupancyNode>>,
			std::conditional_t<MapType & OCCUPANCY, ContainsOccupancy<8>, void>,
			ReuseNodes, LockLess, CountNodes,
			cond_map_base<MapType & SEMANTIC,   SEMANTIC,   SemanticMap>::template type,
			cond_map_base<MapType & SURFEL,     SURFEL,     SurfelMap>::template type,
			cond_map_base<MapType & DISTANCE,   DISTANCE,   DistanceMap>::template type,
			cond_map_base<MapType & REFLECTION, REFLECTION, ReflectionMap>::template type,
			cond_map_base<MapType & COUNT,      COUNT,      CountMap>::template type,
			cond_map_base<MapType & TIME,       TIME,       TimeMap>::template type,
			cond_map_base<MapType & INTENSITY,  INTENSITY,  IntensityMap>::template type,
			cond_map_base<MapType & COLOR,      COLOR,      ColorMap>::template type,
			cond_map_base<MapType & OCCUPANCY,  OCCUPANCY,  OccupancyMap>::template type
	    // clang-format on
	    >;

 public:
	//
	// Constructors
	//

	UFOMap(node_size_t leaf_node_size = 0.1, depth_t depth_levels = 17,
	       bool auto_prune = true)
	    : Base(leaf_node_size, depth_levels, auto_prune)
	{
	}

	UFOMap(std::filesystem::path const& file, bool auto_prune = true)
	    : Base(file, auto_prune)
	{
	}

	UFOMap(std::istream& in, bool auto_prune = true) : Base(in, auto_prune) {}

	UFOMap(UFOMap const& other) = default;

	template <mt_t MapType2, bool ReuseNodes2, bool LockLess2, bool CountNodes2>
	UFOMap(UFOMap<MapType2, ReuseNodes2, LockLess2, CountNodes2> const& other) : Base(other)
	{
	}

	UFOMap(UFOMap&& other) = default;

	//
	// Operator assignment
	//

	UFOMap& operator=(UFOMap const& rhs) = default;

	template <mt_t MapType2, bool ReuseNodes2, bool LockLess2, bool CountNodes2>
	UFOMap& operator=(UFOMap<MapType2, ReuseNodes2, LockLess2, CountNodes2> const& rhs)
	{
		Base::operator=(rhs);
		return *this;
	}

	UFOMap& operator=(UFOMap&& rhs) = default;

	//
	// Swap
	//

	void swap(UFOMap& other) noexcept(noexcept(Base::swap(other))) { Base::swap(other); }
};

//
// Explicitly define common map types
//

// clang-format off
using OccupancyMap    = UFOMap<OCCUPANCY>;
using TimeMap         = UFOMap<TIME>;
using ColorMap        = UFOMap<COLOR>;
using SemanticMap     = UFOMap<SEMANTIC>;
using SurfelMap       = UFOMap<SURFEL>;
using DistanceMap     = UFOMap<DISTANCE>;
using IntensityMap    = UFOMap<INTENSITY>;
using CountMap        = UFOMap<COUNT>;
using ReflectionMap   = UFOMap<REFLECTION>;
// clang-format on
}  // namespace ufo::map

namespace std
{
template <ufo::map::mt_t MapType, bool ReuseNodes, bool LockLess, bool CountNodes>
void swap(UFOMap<MapType, ReuseNodes, LockLess, CountNodes>& lhs,
          UFOMap<MapType, ReuseNodes, LockLess, CountNodes>&
              rhs) noexcept(noexcept(lhs.swap(rhs)))
{
	lhs.swap(rhs);
}
}  // namespace std

#endif  // UFO_MAP_UFO_MAP_H