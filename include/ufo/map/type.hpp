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

#ifndef UFO_MAP_TYPE
#define UFO_MAP_TYPE

// UFO
#include <ufo/utility/enum.hpp>

// STL
#include <cstdint>
#include <limits>
#include <ostream>
#include <string>
#include <string_view>
#include <vector>

namespace ufo
{
enum class MapType : std::uint64_t {
	NONE         = std::uint64_t(0),
	ALL          = ~std::uint64_t(0),
	OCCUPANCY    = std::uint64_t(1) << 0,
	COLOR        = std::uint64_t(1) << 1,
	TIME         = std::uint64_t(1) << 2,
	COUNT        = std::uint64_t(1) << 3,
	REFLECTION   = std::uint64_t(1) << 4,
	INTENSITY    = std::uint64_t(1) << 5,
	SURFEL       = std::uint64_t(1) << 6,
	VOID_REGION  = std::uint64_t(1) << 7,
	DISTANCE     = std::uint64_t(1) << 8,
	LABEL        = std::uint64_t(1) << 9,
	SEMANTIC     = std::uint64_t(1) << 10,
	LABEL_SET    = std::uint64_t(1) << 11,
	SEMANTIC_SET = std::uint64_t(1) << 12,
	COST         = std::uint64_t(1) << 13,
};

[[nodiscard]] constexpr MapType operator|(MapType a, MapType b) noexcept
{
	return MapType(to_underlying(a) | to_underlying(b));
}

[[nodiscard]] constexpr MapType operator&(MapType a, MapType b) noexcept
{
	return MapType(to_underlying(a) & to_underlying(b));
}

[[nodiscard]] constexpr std::string_view to_string(MapType mt)
{
	switch (mt) {
		case MapType::ALL: return std::string_view{"all"};
		case MapType::NONE: return std::string_view{"none"};
		case MapType::OCCUPANCY: return std::string_view{"occupancy"};
		case MapType::COLOR: return std::string_view{"color"};
		case MapType::TIME: return std::string_view{"time"};
		case MapType::COUNT: return std::string_view{"count"};
		case MapType::REFLECTION: return std::string_view{"reflection"};
		case MapType::INTENSITY: return std::string_view{"intensity"};
		case MapType::SURFEL: return std::string_view{"surfel"};
		case MapType::VOID_REGION: return std::string_view{"void_region"};
		case MapType::DISTANCE: return std::string_view{"distance"};
		case MapType::LABEL: return std::string_view{"label"};
		case MapType::SEMANTIC: return std::string_view{"semantic"};
		case MapType::LABEL_SET: return std::string_view{"label_set"};
		case MapType::SEMANTIC_SET: return std::string_view{"semantic_set"};
		case MapType::COST: return std::string_view{"cost"};
		default: return std::string_view{"unknown"};
	}
}

[[nodiscard]] inline std::vector<std::string> mapTypes(MapType mt)
{
	constexpr std::size_t const max_num_map_types =
	    std::numeric_limits<std::uint64_t>::digits;

	std::vector<std::string> res;
	for (std::size_t i{}; max_num_map_types > i; ++i) {
		auto name = to_string(mt & static_cast<MapType>(std::uint64_t(1) << i));
		if (std::string_view{"none"} != name && std::string_view{"unknown"} != name) {
			res.emplace_back(name);
		}
	}
	return res;
}

inline std::ostream& operator<<(std::ostream& os, MapType mt)
{
	constexpr std::size_t const max_num_map_types =
	    std::numeric_limits<std::uint64_t>::digits;

	std::string c;
	for (std::size_t i{}; max_num_map_types > i; ++i) {
		auto name = to_string(mt & static_cast<MapType>(std::uint64_t(1) << i));
		if (std::string_view{"none"} != name && std::string_view{"unknown"} != name) {
			os << c << name;
			c = ", ";
		}
	}
	return os;
}
}  // namespace ufo

#endif  // UFO_MAP_TYPE