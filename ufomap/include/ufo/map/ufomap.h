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
#include <ufo/map/empty/empty_map.h>
#include <ufo/map/empty/empty_node.h>
#include <ufo/map/io.h>
#include <ufo/map/node_base.h>
#include <ufo/map/occupancy/occupancy_indicators.h>
#include <ufo/map/occupancy/occupancy_map_base.h>
#include <ufo/map/occupancy/occupancy_node.h>
#include <ufo/map/octree_map_base.h>
#include <ufo/map/semantic/semantic_map_base.h>
#include <ufo/map/semantic/semantic_node.h>
#include <ufo/map/signed_distance/signed_distance_map_base.h>
#include <ufo/map/signed_distance/signed_distance_node.h>
#include <ufo/map/surfel/surfel_map_base.h>
#include <ufo/map/surfel/surfel_node.h>
#include <ufo/map/time/time_map_base.h>
#include <ufo/map/time/time_node.h>

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
	SURFEL = 1U << 3U,
	SIGNED_DISTANCE = 1U << 4U,
	OCCUPANCY_SMALL = 1U << 5U,
};

template <bool C, std::size_t Num, class Node>
using conditional_node_t = std::conditional_t<C, Node, EmptyNode<Num>>;

template <bool C, std::size_t Num, template <typename...> typename MapBase>
struct conditional_map_base {
	template <typename... ts>
	using type = MapBase<ts...>;
};

template <std::size_t Num, template <typename...> typename MapBase>
struct conditional_map_base<false, Num, MapBase> {
	template <typename... Ts>
	using type = EmptyMap<Num, Ts...>;
};

// Forward decleration
template <std::uint64_t MapType, std::size_t SemanticLabelBits = 16,
          std::size_t SemanticValueBits = 16, bool ReuseNodes = false,
          bool LockLess = false>
class SemanticUFOMap;

template <std::uint64_t MapType, bool ReuseNodes = false, bool LockLess = false>
class UFOMap
    : public OctreeMapBase<
          NodeBase<conditional_node_t<MapType & OCCUPANCY, 0, OccupancyNode<float>>,
                   conditional_node_t<MapType & OCCUPANCY_SMALL, 1, OccupancyNodeSmall>,
                   conditional_node_t<MapType & TIME, 2, TimeNode>,
                   conditional_node_t<MapType & COLOR, 3, ColorNode>,
                   conditional_node_t<MapType & SURFEL, 4, SurfelNode<float>>,
                   conditional_node_t<MapType & SIGNED_DISTANCE, 5,
                                      SignedDistanceNode<float>>>,
          std::conditional_t<MapType&(OCCUPANCY | OCCUPANCY_SMALL), OccupancyIndicators,
                             OctreeIndicators>,
          ReuseNodes, LockLess,
          conditional_map_base<MapType&(OCCUPANCY | OCCUPANCY_SMALL), 0,
                               OccupancyMapBase>::template type,
          conditional_map_base<MapType & TIME, 1, TimeMapBase>::template type,
          conditional_map_base<MapType & COLOR, 2, ColorMapBase>::template type,
          conditional_map_base<MapType & SURFEL, 3, SurfelMapBase>::template type,
          conditional_map_base<MapType & SIGNED_DISTANCE, 4,
                               SignedDistanceMapBase>::template type>
{
 private:
	// static_assert(!(MapType & OCCUPANCY && MapType & OCCUPANCY_SMALL));

	using Base = OctreeMapBase<
	    NodeBase<
	        conditional_node_t<MapType & OCCUPANCY, 0, OccupancyNode<float>>,
	        conditional_node_t<MapType & OCCUPANCY_SMALL, 1, OccupancyNodeSmall>,
	        conditional_node_t<MapType & TIME, 2, TimeNode>,
	        conditional_node_t<MapType & COLOR, 3, ColorNode>,
	        conditional_node_t<MapType & SURFEL, 4, SurfelNode<float>>,
	        conditional_node_t<MapType & SIGNED_DISTANCE, 5, SignedDistanceNode<float>>>,
	    std::conditional_t<MapType&(OCCUPANCY | OCCUPANCY_SMALL), OccupancyIndicators,
	                       OctreeIndicators>,
	    ReuseNodes, LockLess,
	    conditional_map_base<MapType&(OCCUPANCY | OCCUPANCY_SMALL), 0,
	                         OccupancyMapBase>::template type,
	    conditional_map_base<MapType & TIME, 1, TimeMapBase>::template type,
	    conditional_map_base<MapType & COLOR, 2, ColorMapBase>::template type,
	    conditional_map_base<MapType & SURFEL, 3, SurfelMapBase>::template type,
	    conditional_map_base<MapType & SIGNED_DISTANCE, 4,
	                         SignedDistanceMapBase>::template type>;

 public:
	//
	// Constructors
	//

	UFOMap(double resolution = 0.1, depth_t depth_levels = 16,
	       bool automatic_pruning = true)
	    : Base(resolution, depth_levels, automatic_pruning)
	{
	}

	UFOMap(std::filesystem::path const& filename, bool automatic_pruning = true)
	    : Base(filename, automatic_pruning)
	{
	}

	UFOMap(std::istream& in_stream, bool automatic_pruning = true)
	    : Base(in_stream, automatic_pruning)
	{
	}

	// FIXME: Why cannot this be default?
	UFOMap(UFOMap const& other) = default;

	template <std::uint64_t MapType2, bool ReuseNodes2, bool LockLess2>
	UFOMap(UFOMap<MapType2, ReuseNodes2, LockLess2> const& other) : Base(other)
	{
	}

	template <std::uint64_t MapType2, std::size_t SemanticLabelBits2,
	          std::size_t SemanticValueBits2, bool ReuseNodes2, bool LockLess2>
	UFOMap(SemanticUFOMap<MapType2, SemanticLabelBits2, SemanticValueBits2, ReuseNodes2,
	                      LockLess2> const& other)
	    : Base(other)
	{
	}

	UFOMap(UFOMap&& other) = default;

	//
	// Operator assignment
	//

	UFOMap& operator=(UFOMap const& rhs) = default;

	template <std::uint64_t MapType2, bool ReuseNodes2, bool LockLess2>
	UFOMap& operator=(UFOMap<MapType2, ReuseNodes2, LockLess2> const& rhs)
	{
		// FIXME: Correct?
		std::stringstream io_stream(std::ios_base::in | std::ios_base::out |
		                            std::ios_base::binary);
		io_stream.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		io_stream.imbue(std::locale());

		rhs.write(io_stream);
		Base::read(io_stream);

		return *this;
	}

	template <std::uint64_t MapType2, std::size_t SemanticLabelBits2,
	          std::size_t SemanticValueBits2, bool ReuseNodes2, bool LockLess2>
	UFOMap& operator=(SemanticUFOMap<MapType2, SemanticLabelBits2, SemanticValueBits2,
	                                 ReuseNodes2, LockLess2> const& rhs)
	{
		// FIXME: Correct?
		std::stringstream io_stream(std::ios_base::in | std::ios_base::out |
		                            std::ios_base::binary);
		io_stream.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		io_stream.imbue(std::locale());

		rhs.write(io_stream);
		Base::read(io_stream);

		return *this;
	}

	UFOMap& operator=(UFOMap&& rhs) = default;
};

template <std::uint64_t MapType, std::size_t SemanticLabelBits,
          std::size_t SemanticValueBits, bool ReuseNodes, bool LockLess>
class SemanticUFOMap
    : public OctreeMapBase<
          NodeBase<SemanticNode<SemanticLabelBits, SemanticValueBits>,
                   conditional_node_t<MapType & OCCUPANCY, 0, OccupancyNode<float>>,
                   conditional_node_t<MapType & OCCUPANCY_SMALL, 1, OccupancyNodeSmall>,
                   conditional_node_t<MapType & TIME, 2, TimeNode>,
                   conditional_node_t<MapType & COLOR, 3, ColorNode>,
                   conditional_node_t<MapType & SURFEL, 4, SurfelNode<float>>,
                   conditional_node_t<MapType & SIGNED_DISTANCE, 5,
                                      SignedDistanceNode<float>>>,
          std::conditional_t<MapType&(OCCUPANCY | OCCUPANCY_SMALL), OccupancyIndicators,
                             OctreeIndicators>,
          ReuseNodes, LockLess, SemanticMapBase,
          conditional_map_base<MapType&(OCCUPANCY | OCCUPANCY_SMALL), 0,
                               OccupancyMapBase>::template type,
          conditional_map_base<MapType & TIME, 1, TimeMapBase>::template type,
          conditional_map_base<MapType & COLOR, 2, ColorMapBase>::template type,
          conditional_map_base<MapType & SURFEL, 3, SurfelMapBase>::template type,
          conditional_map_base<MapType & SIGNED_DISTANCE, 4,
                               SignedDistanceMapBase>::template type>
{
 private:
	// static_assert(0 != MapType);
	// static_assert(!(MapType & OCCUPANCY && MapType & OCCUPANCY_SMALL));
	// static_assert(!(MapType & SEMANTIC &&));

	using Base = OctreeMapBase<
	    NodeBase<
	        SemanticNode<SemanticLabelBits, SemanticValueBits>,
	        conditional_node_t<MapType & OCCUPANCY, 0, OccupancyNode<float>>,
	        conditional_node_t<MapType & OCCUPANCY_SMALL, 1, OccupancyNodeSmall>,
	        conditional_node_t<MapType & TIME, 2, TimeNode>,
	        conditional_node_t<MapType & COLOR, 3, ColorNode>,
	        conditional_node_t<MapType & SURFEL, 4, SurfelNode<float>>,
	        conditional_node_t<MapType & SIGNED_DISTANCE, 5, SignedDistanceNode<float>>>,
	    std::conditional_t<MapType&(OCCUPANCY | OCCUPANCY_SMALL), OccupancyIndicators,
	                       OctreeIndicators>,
	    ReuseNodes, LockLess, SemanticMapBase,
	    conditional_map_base<MapType&(OCCUPANCY | OCCUPANCY_SMALL), 0,
	                         OccupancyMapBase>::template type,
	    conditional_map_base<MapType & TIME, 1, TimeMapBase>::template type,
	    conditional_map_base<MapType & COLOR, 2, ColorMapBase>::template type,
	    conditional_map_base<MapType & SURFEL, 3, SurfelMapBase>::template type,
	    conditional_map_base<MapType & SIGNED_DISTANCE, 4,
	                         SignedDistanceMapBase>::template type>;

 public:
	//
	// Constructors
	//

	SemanticUFOMap(double resolution = 0.1, depth_t depth_levels = 16,
	               bool automatic_pruning = true)
	    : Base(resolution, depth_levels, automatic_pruning)
	{
	}

	SemanticUFOMap(std::filesystem::path const& filename, bool automatic_pruning = true)
	    : Base(filename, automatic_pruning)
	{
	}

	SemanticUFOMap(std::istream& in_stream, bool automatic_pruning = true)
	    : Base(in_stream, automatic_pruning)
	{
	}

	SemanticUFOMap(SemanticUFOMap const& other) = default;

	template <std::uint64_t MapType2, std::size_t SemanticLabelBits2,
	          std::size_t SemanticValueBits2, bool ReuseNodes2, bool LockLess2>
	SemanticUFOMap(SemanticUFOMap<MapType2, SemanticLabelBits2, SemanticValueBits2,
	                              ReuseNodes2, LockLess2> const& other)
	    : Base(other)
	{
	}

	template <std::uint64_t MapType2, bool ReuseNodes2, bool LockLess2>
	SemanticUFOMap(UFOMap<MapType2, ReuseNodes2, LockLess2> const& other) : Base(other)
	{
	}

	SemanticUFOMap(SemanticUFOMap&& other) = default;

	//
	// Operator assignment
	//

	SemanticUFOMap& operator=(SemanticUFOMap const& rhs) = default;

	template <std::uint64_t MapType2, std::size_t SemanticLabelBits2,
	          std::size_t SemanticValueBits2, bool ReuseNodes2, bool LockLess2>
	SemanticUFOMap& operator=(
	    SemanticUFOMap<MapType2, SemanticLabelBits2, SemanticValueBits2, ReuseNodes2,
	                   LockLess2> const& rhs)
	{
		// FIXME: Correct?
		std::stringstream io_stream(std::ios_base::in | std::ios_base::out |
		                            std::ios_base::binary);
		io_stream.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		io_stream.imbue(std::locale());

		rhs.write(io_stream);
		Base::read(io_stream);

		return *this;
	}

	template <std::uint64_t MapType2, bool ReuseNodes2, bool LockLess2>
	SemanticUFOMap& operator=(UFOMap<MapType2, ReuseNodes2, LockLess2> const& rhs)
	{
		// FIXME: Correct?
		std::stringstream io_stream(std::ios_base::in | std::ios_base::out |
		                            std::ios_base::binary);
		io_stream.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		io_stream.imbue(std::locale());

		rhs.write(io_stream);
		Base::read(io_stream);

		return *this;
	}

	SemanticUFOMap& operator=(SemanticUFOMap&& rhs) = default;
};

using OccupancyMap = UFOMap<OCCUPANCY>;
using OccupancyMapSmall = UFOMap<OCCUPANCY_SMALL>;

using TimeMap = UFOMap<TIME>;

using ColorMap = UFOMap<COLOR>;

// using SurfelMap = Map<SURFEL>;

using SignedDistanceMap = UFOMap<SIGNED_DISTANCE>;
}  // namespace ufo::map

#endif  // UFO_MAP_UFO_MAP_H