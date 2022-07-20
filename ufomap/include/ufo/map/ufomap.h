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
#include <ufo/map/io.h>
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
#include <filesystem>
#include <fstream>
#include <optional>
#include <type_traits>

namespace ufo::map
{
enum class MapType {
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
	// SEMANTIC_FIXED_4 =
};

template <std::size_t MapType, bool ReuseNodes = false, bool LockLess = false>
    class Map
    : public OctreeMapBase <
      NodeBase<
          std::conditional_t<MapType & 0b1000, SemanticNode, EmptyMap<4>>,
          std::conditional_t<
              MapType & 0 std::conditional_t<MapType & 0b1, OccupancyNode, EmptyMap<1>>,
              std::conditional_t<MapType & 0b10, TimeNode, EmptyMap<2>>,
              std::conditional_t<MapType & 0b100, ColorNode, EmptyMap<3>>,
              std::conditional_t<MapType & 0b10000, SurfelNode, EmptyMap<5>>>,
          std::conditional_t<MapType & 0b1, OccupancyIndicators, OctreeIndicators>,
          ReuseNodes, LockLess,
          std::conditional_t<MapType & 0b1, OccupancyMapBase, EmptyMap<1>>,
          std::conditional_t<MapType & 0b10, TimeMapBase, EmptyMap<2>>,
          std::conditional_t<MapType & 0b100, ColorMapBase, EmptyMap<3>>,
          std::conditional_t<MapType & 0b1000, SemanticMapBase, EmptyMap<4>>,
          std::conditional_t<MapType & 0b10000, SurfelMapBase, EmptyMap<5>>>
{
};

template <std::size_t MapType, class SemanticType = uint32_t,
          size_t SemanticValueWidth = 16, bool ReuseNodes = false, bool LockLess = false>
class SemanticMap
    : public OctreeMapBase<
          NodeBase<SemanticNode,
                   std::conditional_t<MapType & 0b1, OccupancyNode, EmptyMap<1>>,
                   std::conditional_t<MapType & 0b10, TimeNode, EmptyMap<2>>,
                   std::conditional_t<MapType & 0b100, ColorNode, EmptyMap<3>>,
                   std::conditional_t<MapType & 0b10000, SurfelNode, EmptyMap<5>>>,
          std::conditional_t<MapType & 0b1, OccupancyIndicators, OctreeIndicators>,
          ReuseNodes, LockLess,
          std::conditional_t<MapType & 0b1, OccupancyMapBase, EmptyMap<1>>,
          std::conditional_t<MapType & 0b10, TimeMapBase, EmptyMap<2>>,
          std::conditional_t<MapType & 0b100, ColorMapBase, EmptyMap<3>>,
          std::conditional_t<MapType & 0b10000, SurfelMapBase, EmptyMap<4>>>
{
};

// template <std::size_t MapType, bool ReuseNodes = false, bool LockLess = false,
// > class Map
//     : public
//       // clang-format off
// 		std::conditional_t<0b1     == MapType, OccupancyMap<ReuseNodes, LockLess>,
// 		std::conditional_t<0b10    == MapType, TimeMap<ReuseNodes, LockLess>,
// 		std::conditional_t<0b11    == MapType, OccupancyTimeMap<ReuseNodes,
// LockLess>, 		std::conditional_t<0b100   == MapType, ColorMap<ReuseNodes,
// LockLess>, 		std::conditional_t<0b101   == MapType, OccupancyColorMap<ReuseNodes,
// LockLess>, 		std::conditional_t<0b110   == MapType, TimeColorMap<ReuseNodes,
// LockLess>, 		std::conditional_t<0b111   == MapType,
// OccupancyTimeColorMap<ReuseNodes, LockLess>, 		std::conditional_t<0b1000  ==
// MapType, SemanticMap<ReuseNodes, LockLess>, 		std::conditional_t<0b1001  ==
// MapType, OccupancySemanticMap<ReuseNodes, LockLess>, 		std::conditional_t<0b1010
// == MapType, TimeSemanticMap<ReuseNodes, LockLess>, 		std::conditional_t<0b1011 ==
// MapType, OccupancyTimeSemanticMap<ReuseNodes, LockLess>,
//     std::conditional_t<0b1100  == MapType, ColorSemanticMap<ReuseNodes,
//     LockLess>, std::conditional_t<0b1101  == MapType,
//     OccupancyColorSemanticMap<ReuseNodes, LockLess>, std::conditional_t<0b1110
//     == MapType, TimeColorSemanticMap<ReuseNodes, LockLess>,
//     std::conditional_t<0b1111  == MapType,
//     OccupancyTimeColorSemanticMap<ReuseNodes, LockLess>,
//     std::conditional_t<0b10000 == MapType, SurfelMap<ReuseNodes, LockLess>,
//     std::conditional_t<0b10001 == MapType, OccupancySurfelMap<ReuseNodes,
//     LockLess>, std::conditional_t<0b10010 == MapType, TimeSurfelMap<ReuseNodes,
//     LockLess>, std::conditional_t<0b10011 == MapType,
//     OccupancyTimeSurfelMap<ReuseNodes, LockLess>, std::conditional_t<0b10100 ==
//     MapType, ColorSurfelMap<ReuseNodes, LockLess>, std::conditional_t<0b10101
//     == MapType, OccupancyColorSurfelMap<ReuseNodes, LockLess>,
//     std::conditional_t<0b10110
//     == MapType, TimeColorSurfelMap<ReuseNodes, LockLess>,
//     std::conditional_t<0b10111 == MapType,
//     OccupancyTimeColorSurfelMap<ReuseNodes, LockLess>,
//     std::conditional_t<0b11000 == MapType, SemanticSurfelMap<ReuseNodes,
//     LockLess>, std::conditional_t<0b11001 == MapType,
//     OccupancySemanticSurfelMap<ReuseNodes, LockLess>,
//     std::conditional_t<0b11010 == MapType, TimeSemanticSurfelMap<ReuseNodes,
//     LockLess>, std::conditional_t<0b11011 == MapType,
//     OccupancyTimeSemanticSurfelMap<ReuseNodes, LockLess>,
//     std::conditional_t<0b11100 == MapType, ColorSemanticSurfelMap<ReuseNodes,
//     LockLess>, std::conditional_t<0b11101
//     == MapType, OccupancyColorSemanticSurfelMap<ReuseNodes, LockLess>,
//     std::conditional_t<0b11110 == MapType,
//     TimeColorSemanticSurfelMap<ReuseNodes, LockLess>,
//     std::conditional_t<0b11111 == MapType,
//     OccupancyTimeColorSemanticSurfelMap<ReuseNodes, LockLess>,
//     void>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// // clang-format on
// {
// };

template <bool Occupancy = true, bool Time = false, bool Color = false,
          bool Semantic = false, bool Surfel = false, bool ReuseNodes = false,
          bool LockLess = false>
class Map
{
};

// using UFOMap =
//     std::variant<std::monostate, OccupancyMap, OccupancyMapTime, OccupancyMapColor,
//                  OccupancyMapSemantic, OccupancyMapTimeColor, OccupancyMapTimeSemantic,
//                  OccupancyMapColorSemantic, OccupancyMapTimeColorSemantic,
//                  OccupancyMapSmall, OccupancyMapColorSmall, OccupancyMapSemanticSmall,
//                  OccupancyMapColorSemanticSmall>;
// using UFOMap =
//     std::variant<std::monostate, OccupancyMap, OccupancyMapTimeMin, OccupancyMapColor,
//                  OccupancyMapSemantic, OccupancyMapSmall, OccupancyMapColorSmall,
//                  OccupancyMapSemanticSmall, OccupancyMapTimeMax>;
using UFOMap =
    std::variant<std::monostate, OccupancyMap, OccupancyMapTime, OccupancyMapColor,
                 OccupancyMapTimeColor, OccupancyMapSmall, OccupancyMapColorSmall>;

// using UFOMap = std::variant<
//     std::monostate, OccupancyMap, OccupancyMapTime, OccupancyMapColor,
//     OccupancyMapSemantic64, OccupancyMapSemantic32, OccupancyMapSemantic16,
//     OccupancyMapSemantic8, OccupancyMapTimeColor, OccupancyMapTimeSemantic64,
//     OccupancyMapTimeSemantic32, OccupancyMapTimeSemantic16, OccupancyMapTimeSemantic8,
//     OccupancyMapColorSemantic64, OccupancyMapColorSemantic32,
//     OccupancyMapColorSemantic16, OccupancyMapColorSemantic8,
//     OccupancyMapTimeColorSemantic64, OccupancyMapTimeColorSemantic32,
//     OccupancyMapTimeColorSemantic16, OccupancyMapTimeColorSemantic8, OccupancyMap,
//     OccupancyMapTimeSmall, OccupancyMapColorSmall, OccupancyMapSemantic64Small,
//     OccupancyMapSemantic32Small, OccupancyMapSemantic16Small,
//     OccupancyMapSemantic8Small, OccupancyMapColorSemantic64Small,
//     OccupancyMapColorSemantic32Small, OccupancyMapColorSemantic16Small,
//     OccupancyMapColorSemantic8Small>;

template <template <typename...> class Base>
constexpr bool isMapBase(UFOMap const& map)
{
	return std::visit(
	    [&](auto const& arg) -> bool {
		    using T = std::decay_t<decltype(arg)>;
		    return ufo::map::is_base_of_template_v<Base, T>;
	    },
	    map);
}

inline bool canMergeMap(std::istream& in_stream_1, std::istream& in_stream_2)
{
	auto pos = in_stream_1.tellg();
	FileInfo header_1 = readHeader(in_stream_1);
	in_stream_1.seekg(pos);
	pos = in_stream_2.tellg();
	FileInfo header_2 = readHeader(in_stream_2);
	in_stream_2.seekg(pos);

	// TODO: Implement better
	return header_1.at("resolution").at(0) == header_2.at("resolution").at(0) &&
	       header_1.at("depth_levels").at(0) == header_2.at("depth_levels").at(0);
}

inline bool canMergeMap(std::filesystem::path const& filename_1,
                        std::filesystem::path const& filename_2)
{
	std::ifstream file_1;
	file_1.exceptions(std::ifstream::failbit | std::ifstream::badbit);

	file_1.open(filename_1, std::ios_base::in | std::ios_base::binary);

	std::ifstream file_2;
	file_2.exceptions(std::ifstream::failbit | std::ifstream::badbit);

	file_2.open(filename_2, std::ios_base::in | std::ios_base::binary);

	return canMergeMap(file_1, file_2);
}

inline bool canMergeMap(UFOMap const& map, std::istream& in_stream)
{
	return std::visit(
	    [&](auto const& arg) -> bool {
		    using T = std::decay_t<decltype(arg)>;
		    if constexpr (!std::is_same_v<std::monostate, T>) {
			    return arg.canMerge(in_stream);
		    }
		    return false;
	    },
	    map);
}

inline bool canMergeMap(UFOMap const& map, std::filesystem::path const& filename)
{
	std::ifstream file;
	file.exceptions(std::ifstream::failbit | std::ifstream::badbit);

	file.open(filename, std::ios_base::in | std::ios_base::binary);

	return canMergeMap(map, file);
}

constexpr std::string_view mapType(UFOMap const& map)
{
	return std::visit(
	    [](auto const& arg) -> std::string_view {
		    using T = std::decay_t<decltype(arg)>;
		    if constexpr (!std::is_same_v<std::monostate, T>) {
			    return T::mapType();
		    }
		    return "";
	    },
	    map);
}

inline bool isSameMapType(std::istream& in_stream_1, std::istream& in_stream_2)
{
	auto pos = in_stream_1.tellg();
	FileInfo header_1 = readHeader(in_stream_1);
	in_stream_1.seekg(pos);
	pos = in_stream_2.tellg();
	FileInfo header_2 = readHeader(in_stream_2);
	in_stream_2.seekg(pos);

	return header_1.at("map_type").at(0) == header_2.at("map_type").at(0);
}

inline bool isSameMapType(std::filesystem::path const& filename_1,
                          std::filesystem::path const& filename_2)
{
	std::ifstream file_1;
	file_1.exceptions(std::ifstream::failbit | std::ifstream::badbit);

	file_1.open(filename_1, std::ios_base::in | std::ios_base::binary);

	std::ifstream file_2;
	file_2.exceptions(std::ifstream::failbit | std::ifstream::badbit);

	file_2.open(filename_2, std::ios_base::in | std::ios_base::binary);

	return isSameMapType(file_1, file_2);
}

inline bool isSameMapType(UFOMap const& map, std::istream& in_stream)
{
	auto pos = in_stream.tellg();
	FileInfo header = readHeader(in_stream);
	in_stream.seekg(pos);

	return mapType(map) == header.at("map_type").at(0);
}

inline bool isSameMapType(UFOMap const& map, std::filesystem::path const& filename)
{
	std::ifstream file;
	file.exceptions(std::ifstream::failbit | std::ifstream::badbit);

	file.open(filename, std::ios_base::in | std::ios_base::binary);

	return isSameMapType(map, file);
}

template <size_t I, class... Args>
inline UFOMap createMap(size_t index, Args&&... args)
{
	if constexpr (I < std::variant_size_v<UFOMap>) {
		if (I == index) {
			using Map = std::variant_alternative_t<I, UFOMap>;
			if constexpr (!std::is_same_v<std::monostate, Map>) {
				return Map(std::forward<Args>(args)...);
			}
		} else {
			return createMap<I + 1>(index, std::forward<Args>(args)...);
		}
	}
	return std::monostate();
}

template <class... Args>
inline UFOMap createMap(size_t index, Args&&... args)
{
	return createMap<0>(index, std::forward<Args>(args)...);
}

template <size_t I, class... Args>
inline UFOMap createMap(std::istream& in_stream, std::string const& map_type,
                        Args&&... args)
{
	if constexpr (I < std::variant_size_v<UFOMap>) {
		using Map = std::variant_alternative_t<I, UFOMap>;
		if constexpr (!std::is_same_v<std::monostate, Map>) {
			return map_type == Map::mapType()
			           ? Map(in_stream, std::forward<Args>(args)...)
			           : createMap<I + 1>(in_stream, map_type, std::forward<Args>(args)...);
		} else {
			return createMap<I + 1>(in_stream, map_type, std::forward<Args>(args)...);
		}
	}
	return std::monostate();
}

template <class... Args>
inline UFOMap createMap(std::istream& in_stream, Args&&... args)
{
	auto pos = in_stream.tellg();
	FileInfo header = readHeader(in_stream);
	in_stream.seekg(pos);

	return createMap<0>(in_stream, header.at("map_type").at(0),
	                    std::forward<Args>(args)...);
}

template <class... Args>
inline UFOMap createMap(std::filesystem::path const& filename, Args&&... args)
{
	std::ifstream file;
	file.exceptions(std::ifstream::failbit | std::ifstream::badbit);

	file.open(filename, std::ios_base::in | std::ios_base::binary);

	return createMap(file, std::forward<Args>(args)...);
}

inline void mergeMap(UFOMap& map, std::istream& in_stream)
{
	if (!canMergeMap(map, in_stream)) {
		throw std::invalid_argument("Cannot merge map.");
	}

	return std::visit(
	    [&](auto& arg) {
		    using T = std::decay_t<decltype(arg)>;
		    if constexpr (!std::is_same_v<std::monostate, T>) {
			    arg.read(in_stream);
		    } else {
			    throw std::invalid_argument("Cannot merge map with monostate.");
		    }
	    },
	    map);
}

inline void mergeMap(UFOMap& map, std::filesystem::path const& filename)
{
	std::ifstream file;
	file.exceptions(std::ifstream::failbit | std::ifstream::badbit);

	file.open(filename, std::ios_base::in | std::ios_base::binary);

	mergeMap(map, file);
}

// If map is of same type as stream map, then insert data into it. Otherwise, create new
// map.
template <class... Args>
inline void createOrMergeMap(UFOMap& map, std::istream& in_stream,
                             bool force_same_map_type, Args&&... args)
{
	if (!canMergeMap(map, in_stream)) {
		map = createMap(in_stream, std::forward<Args>(args)...);
		return;
	}

	if (force_same_map_type && !isSameMapType(map, in_stream)) {
		map = createMap(in_stream, std::forward<Args>(args)...);
		return;
	}

	mergeMap(map, in_stream);
}

template <class... Args>
inline void createOrMergeMap(UFOMap& map, std::filesystem::path const& filename,
                             bool force_same_map_type, Args&&... args)
{
	std::ifstream file;
	file.exceptions(std::ifstream::failbit | std::ifstream::badbit);

	file.open(filename, std::ios_base::in | std::ios_base::binary);

	createOrMergeMap(map, file, force_same_map_type, std::forward<Args>(args)...);
}
}  // namespace ufo::map

#endif  // UFO_MAP_UFO_MAP_H