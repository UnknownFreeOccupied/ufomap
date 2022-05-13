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
#include <variant>

namespace ufo::map
{
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