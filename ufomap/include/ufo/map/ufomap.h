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
#include <ufo/map/color/color_node.h>
#include <ufo/map/io.h>
#include <ufo/map/occupancy/occupancy_indicators.h>
#include <ufo/map/occupancy/occupancy_map_base.h>
#include <ufo/map/occupancy/occupancy_node.h>
#include <ufo/map/semantic/semantic_map_base.h>
#include <ufo/map/semantic/semantic_node.h>
#include <ufo/map/signed_distance/signed_distance_map_base.h>
#include <ufo/map/signed_distance/signed_distance_node.h>
#include <ufo/map/surfel/surfel_map_base.h>
#include <ufo/map/surfel/surfel_node.h>
#include <ufo/map/time/time_map_base.h>
#include <ufo/map/time/time_node.h>

//////////////////////////////////////////////
#include <ufo/map/occupancy_map.h>
#include <ufo/map/occupancy_map_color.h>
// #include <ufo/map/occupancy_map_color_semantic.h>
// #include <ufo/map/occupancy_map_semantic.h>
#include <ufo/map/occupancy_map_time.h>
#include <ufo/map/occupancy_map_time_color.h>
// #include <ufo/map/occupancy_map_time_color_semantic.h>
// #include <ufo/map/occupancy_map_time_semantic.h>
#include <ufo/map/integrator/integrator.h>
#include <ufo/map/point_cloud.h>
#include <ufo/map/types.h>

// STL
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <type_traits>

namespace ufo::map
{
enum MapType : std::uint64_t {
	OCCUPANCY = 1U,
	TIME = 1U << 1U,
	COLOR = 1U << 2U,
	SEMANTIC = 1U << 3U,
	SURFEL = 1U << 4U,
	SIGNED_DISTANCE = 1U << 5U,
	OCCUPANCY_SMALL = 1U << 6U,
	SEMANTIC_TINY = 1U << 7U,
	SEMANTIC_SMALL = 1U << 8U,
	SEMANTIC_BIG = 1U << 9U,
};

template <std::uint64_t MapType, bool ReuseNodes = false, bool LockLess = false>
class Map
    : public OctreeMapBase<
          NodeBase<
              std::conditional_t<MapType & SEMANTIC, SemanticNode, EmptyNode<0>>,
              std::conditional_t<MapType & SEMANTIC_TINY, SemanticNodeTiny, EmptyNode<1>>,
              std::conditional_t<MapType & SEMANTIC_SMALL, SemanticNodeSmall,
                                 EmptyNode<2>>,
              std::conditional_t<MapType & SEMANTIC_BIG, SemanticNodeBig, EmptyNode<3>>,
              std::conditional_t<MapType & OCCUPANCY, OccupancyNode, EmptyNode<4>>,
              std::conditional_t<
                  MapType & OCCUPANCY_SMALL,
                  std::conditional_t<MapType & TIME, OccupancyTimeNode,
                                     OccupancyNodeSmall>,
                  std::conditional_t<MapType & TIME, TimeNode, EmptyNode<5>>>,
              std::conditional_t<MapType & COLOR, ColorNode, EmptyNode<6>>,
              std::conditional_t<MapType & SURFEL, SurfelNode, EmptyNode<7>>,
              std::conditional_t<MapType & SIGNED_DISTANCE, SignedDistanceNode,
                                 EmptyNode<8>>>,
          std::conditional_t<MapType&(OCCUPANCY | OCCUPANCY_SMALL), OccupancyIndicators,
                             OctreeIndicators>,
          ReuseNodes, LockLess,
          std::conditional_t<MapType&(OCCUPANCY | OCCUPANCY_SMALL), OccupancyMapBase,
                             EmptyMap<0>>,
          std::conditional_t<MapType & TIME, TimeMapBase, EmptyMap<1>>,
          std::conditional_t<MapType & COLOR, ColorMapBase, EmptyMap<2>>,
          std::conditional_t<MapType&(SEMANTIC | SEMANTIC_TINY | SEMANTIC_SMALL |
                                      SEMANTIC_BIG),
                             SemanticMapBase, EmptyMap<3>>,
          std::conditional_t<MapType & SURFEL, SurfelMapBase, EmptyMap<4>>,
          std::conditional_t<MapType & SIGNED_DISTANCE, SignedDistanceMapBase,
                             EmptyMap<5>>>
{
 private:
	static_assert(!(MapType & OCCUPANCY && MapType & OCCUPANCY_SMALL));
	static_assert(!(MapType & SEMANTIC &&));

	using Base = OctreeMapBase<
	    NodeBase<
	        std::conditional_t<MapType & SEMANTIC, SemanticNode, EmptyNode<0>>,
	        std::conditional_t<MapType & SEMANTIC_TINY, SemanticNodeTiny, EmptyNode<1>>,
	        std::conditional_t<MapType & SEMANTIC_SMALL, SemanticNodeSmall, EmptyNode<2>>,
	        std::conditional_t<MapType & SEMANTIC_BIG, SemanticNodeBig, EmptyNode<3>>,
	        std::conditional_t<MapType & OCCUPANCY, OccupancyNode, EmptyNode<4>>,
	        std::conditional_t<
	            MapType & OCCUPANCY_SMALL,
	            std::conditional_t<MapType & TIME, OccupancyTimeNode, OccupancyNodeSmall>,
	            std::conditional_t<MapType & TIME, TimeNode, EmptyNode<5>>>,
	        std::conditional_t<MapType & COLOR, ColorNode, EmptyNode<6>>,
	        std::conditional_t<MapType & SURFEL, SurfelNode, EmptyNode<7>>,
	        std::conditional_t<MapType & SIGNED_DISTANCE, SignedDistanceNode,
	                           EmptyNode<8>>>,
	    std::conditional_t<MapType&(OCCUPANCY | OCCUPANCY_SMALL), OccupancyIndicators,
	                       OctreeIndicators>,
	    ReuseNodes, LockLess,
	    std::conditional_t<MapType&(OCCUPANCY | OCCUPANCY_SMALL), OccupancyMapBase,
	                       EmptyMap<0>>,
	    std::conditional_t<MapType & TIME, TimeMapBase, EmptyMap<1>>,
	    std::conditional_t<MapType & COLOR, ColorMapBase, EmptyMap<2>>,
	    std::conditional_t<MapType&(SEMANTIC | SEMANTIC_TINY | SEMANTIC_SMALL |
	                                SEMANTIC_BIG),
	                       SemanticMapBase, EmptyMap<3>>,
	    std::conditional_t<MapType & SURFEL, SurfelMapBase, EmptyMap<4>>,
	    std::conditional_t<MapType & SIGNED_DISTANCE, SignedDistanceMapBase, EmptyMap<5>>>;

 public:
	//
	// Constructors
	//

	Map(double resolution = 0.1, depth_t depth_levels = 16, bool automatic_pruning = true)
	    : Base(resolution, depth_levels, automatic_pruning)
	{
	}

	Map(std::filesystem::path const& filename, bool automatic_pruning = true)
	    : Base(filename, automatic_pruning)
	{
	}

	Map(std::istream& in_stream, bool automatic_pruning = true)
	    : Base(in_stream, automatic_pruning)
	{
	}

	Map(Base const& other) = default;

	template <std::uint64_t MapType2, bool ReuseNodes2, bool LockLess2>
	Map(Map<MapType2, ReuseNodes2, LockLess2> const& other) : Base(other)
	{
	}

	Map(Map&& other) = default;

	//
	// Operator assignment
	//

	Map& operator=(Map const& rhs) = default;

	template <std::uint64_t MapType2, bool ReuseNodes2, bool LockLess2>
	Map& operator=(Map<MapType2, ReuseNodes2, LockLess2> const& rhs) = default;

	Map& operator=(Map&& rhs) = default;
};

template <std::uint64_t MapType, class SemanticType = uint32_t,
          size_t SemanticValueWidth = 16, std::size_t SemanticFixedSize = 0,
          bool ReuseNodes = false, bool LockLess = false>
class SemanticMap
    : public OctreeMapBase<
          NodeBase<SemanticNode<SemanticType, SemanticValueWidth, SemanticFixedSize>,
                   std::conditional_t<MapType & OCCUPANCY, OccupancyNode, EmptyNode<4>>,
                   std::conditional_t<
                       MapType & OCCUPANCY_SMALL,
                       std::conditional_t<MapType & TIME, OccupancyTimeNode,
                                          OccupancyNodeSmall>,
                       std::conditional_t<MapType & TIME, TimeNode, EmptyNode<5>>>,
                   std::conditional_t<MapType & COLOR, ColorNode, EmptyNode<6>>,
                   std::conditional_t<MapType & SURFEL, SurfelNode, EmptyNode<7>>,
                   std::conditional_t<MapType & SIGNED_DISTANCE, SignedDistanceNode,
                                      EmptyNode<8>>>,
          std::conditional_t<MapType&(OCCUPANCY | OCCUPANCY_SMALL), OccupancyIndicators,
                             OctreeIndicators>,
          ReuseNodes, LockLess,
          std::conditional_t<MapType&(OCCUPANCY | OCCUPANCY_SMALL), OccupancyMapBase,
                             EmptyMap<0>>,
          std::conditional_t<MapType & TIME, TimeMapBase, EmptyMap<1>>,
          std::conditional_t<MapType & COLOR, ColorMapBase, EmptyMap<2>>, SemanticMapBase,
          std::conditional_t<MapType & SURFEL, SurfelMapBase, EmptyMap<3>>,
          std::conditional_t<MapType & SIGNED_DISTANCE, SignedDistanceMapBase,
                             EmptyMap<4>>>
{
 private:
	static_assert(0 != MapType);
	static_assert(!(MapType & OCCUPANCY && MapType & OCCUPANCY_SMALL));
	static_assert(!(MapType & SEMANTIC &&));

	using Base = OctreeMapBase<
	    NodeBase<
	        SemanticNode<SemanticType, SemanticValueWidth, SemanticFixedSize>,
	        std::conditional_t<MapType & OCCUPANCY, OccupancyNode, EmptyNode<4>>,
	        std::conditional_t<
	            MapType & OCCUPANCY_SMALL,
	            std::conditional_t<MapType & TIME, OccupancyTimeNode, OccupancyNodeSmall>,
	            std::conditional_t<MapType & TIME, TimeNode, EmptyNode<5>>>,
	        std::conditional_t<MapType & COLOR, ColorNode, EmptyNode<6>>,
	        std::conditional_t<MapType & SURFEL, SurfelNode, EmptyNode<7>>,
	        std::conditional_t<MapType & SIGNED_DISTANCE, SignedDistanceNode,
	                           EmptyNode<8>>>,
	    std::conditional_t<MapType&(OCCUPANCY | OCCUPANCY_SMALL), OccupancyIndicators,
	                       OctreeIndicators>,
	    ReuseNodes, LockLess,
	    std::conditional_t<MapType&(OCCUPANCY | OCCUPANCY_SMALL), OccupancyMapBase,
	                       EmptyMap<0>>,
	    std::conditional_t<MapType & TIME, TimeMapBase, EmptyMap<1>>,
	    std::conditional_t<MapType & COLOR, ColorMapBase, EmptyMap<2>>, SemanticMapBase,
	    std::conditional_t<MapType & SURFEL, SurfelMapBase, EmptyMap<3>>,
	    std::conditional_t<MapType & SIGNED_DISTANCE, SignedDistanceMapBase, EmptyMap<4>>>;

 public:
	//
	// Constructors
	//

	Map(double resolution = 0.1, depth_t depth_levels = 16, bool automatic_pruning = true)
	    : Base(resolution, depth_levels, automatic_pruning)
	{
	}

	Map(std::filesystem::path const& filename, bool automatic_pruning = true)
	    : Base(filename, automatic_pruning)
	{
	}

	Map(std::istream& in_stream, bool automatic_pruning = true)
	    : Base(in_stream, automatic_pruning)
	{
	}

	Map(Base const& other) = default;

	template <std::uint64_t MapType2, class SemanticType2, size_t SemanticValueWidth2,
	          std::size_t SemanticFixedSize2, bool ReuseNodes2, bool LockLess2>
	Map(Map<MapType2, SemanticType2, SemanticValueWidth2, SemanticFixedSize2, ReuseNodes2,
	        LockLess2> const& other)
	    : Base(other)
	{
	}

	Map(Map&& other) = default;

	//
	// Operator assignment
	//

	Map& operator=(Map const& rhs) = default;

	template <std::uint64_t MapType2, class SemanticType2, size_t SemanticValueWidth2,
	          std::size_t SemanticFixedSize2, bool ReuseNodes2, bool LockLess2>
	Map& operator=(Map<MapType2, SemanticType2, SemanticValueWidth2, SemanticFixedSize2,
	                   ReuseNodes2, LockLess2> const& rhs) = default;

	Map& operator=(Map&& rhs) = default;
};

using OccupancyMap = Map<OCCUPANCY>;
using OccupancyMapSmall = Map<OCCUPANCY_SMALL>;

using TimeMap = Map<TIME>;

using ColorMap = Map<COLOR>;

using SurfelMap = Map<SURFEL>;

using SignedDistanceMap = Map<SIGNED_DISTANCE>;
}  // namespace ufo::map

#endif  // UFO_MAP_UFO_MAP_H