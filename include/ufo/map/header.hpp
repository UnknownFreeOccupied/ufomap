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

#ifndef UFO_MAP_MAP_HEADER_HPP
#define UFO_MAP_MAP_HEADER_HPP

// UFO
#include <ufo/map/type.hpp>
#include <ufo/math/vec.hpp>
#include <ufo/utility/enum.hpp>
#include <ufo/utility/io/buffer.hpp>
#include <ufo/utility/string.hpp>

// STL
#if __cplusplus >= 202002L
#include <bit>
#endif
#include <algorithm>
#include <cctype>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace ufo
{
struct MapTypeInfo {
	MapType       type{};
	std::uint64_t size{};

	constexpr MapTypeInfo() = default;

	MapTypeInfo(MapType type, std::uint64_t size = 0) : type(type), size(size) {}
};

struct MapHeader {
	using length_t  = double;
	using depth_t   = std::uint32_t;
	using dim_t     = std::uint32_t;
	using version_t = std::uint32_t;

	static constexpr std::string_view const MAP_HEADER    = "# UFO Map";
	static constexpr version_t const        CURRENT_MAJOR = 1;
	static constexpr version_t const        CURRENT_MINOR = 0;
	static constexpr version_t const        CURRENT_PATCH = 0;
	// static constexpr std::uint8_t     IS_LITTLE_ENDIAN =
	//     std::endian::native == std::endian::little;

	version_t major{};
	version_t minor = std::numeric_limits<version_t>::max();
	version_t patch = std::numeric_limits<version_t>::max();
	// std::uint8_t is_little_endian;

	std::vector<length_t>    leaf_node_length;
	depth_t                  num_depth_levels{};
	std::uint64_t            num_blocks{};
	std::uint64_t            num_nodes{};
	std::vector<MapTypeInfo> map_info;

	MapHeader()                 = default;
	MapHeader(MapHeader const&) = default;

	template <std::size_t Dim>
	MapHeader(Vec<Dim, length_t> const& leaf_node_length, depth_t num_depth_levels,
	          MapType map_types)
	    : major(CURRENT_MAJOR)
	    , minor(CURRENT_MINOR)
	    , patch(CURRENT_PATCH)
	    , num_depth_levels(num_depth_levels)
	{
		this->leaf_node_length.reserve(Dim);
		for (std::size_t i{}; Dim > i; ++i) {
			this->leaf_node_length.push_back(leaf_node_length[i]);
		}

		// The tree
		map_info.emplace_back(MapType::NONE);

		// The map types
		for (std::size_t i{};
		     std::numeric_limits<std::underlying_type_t<MapType>>::digits > i; ++i) {
			MapType const type = static_cast<MapType>(std::underlying_type_t<MapType>(1) << i);
			if (type != (map_types & type)) {
				continue;
			}
			map_info.emplace_back(type);
		}
	}

	template <class Map>
	MapHeader(Map const& map)
	    : MapHeader(map.length(0), map.numDepthLevels(), map.mapTypes())
	{
	}

	MapHeader(std::filesystem::path const& filename) { read(filename); }

	MapHeader(std::istream& in) { read(in); }

	MapHeader(ReadBuffer& in) { read(in); }

	[[nodiscard]] static std::ifstream openRead(std::filesystem::path const& filename)
	{
		std::ifstream file;
		file.exceptions(std::ifstream::badbit);
		file.imbue(std::locale());
		file.open(filename, std::ios::in | std::ios::binary);
		return file;
	}

	[[nodiscard]] static std::ofstream openWrite(std::filesystem::path const& filename)
	{
		std::ofstream file;
		file.exceptions(std::ifstream::badbit);
		file.imbue(std::locale());
		file.open(filename, std::ios::out | std::ios::binary);
		return file;
	}

	[[nodiscard]] static bool isMap(std::filesystem::path const& filename)
	{
		auto file = openRead(filename);
		return isMap(file);
	}

	[[nodiscard]] static bool isMap(std::istream& in)
	{
		std::string line;
		return std::getline(in, line) && MAP_HEADER == line;
	}

	[[nodiscard]] static bool isMap(ReadBuffer& in)
	{
		std::string line;
		return in.readLine(line) && MAP_HEADER != line;
	}

	void read(std::filesystem::path const& filename)
	{
		auto file = openRead(filename);
		read(file);
	}

	std::istream& read(std::istream& in)
	{
		std::string line;

		if (std::getline(in, line) && MAP_HEADER != line) {
			throw std::runtime_error("Trying to read non-UFO Map");
		}

		// Reset
		major = {};
		minor = std::numeric_limits<version_t>::max();
		patch = std::numeric_limits<version_t>::max();
		leaf_node_length.clear();
		num_depth_levels = {};
		num_blocks       = {};
		num_nodes        = {};
		map_info.clear();

		bool reading_map_info = false;
		while (std::getline(in, line)) {
			if (read(line, reading_map_info)) {
				break;
			}
		}

		if (version_t(0) == major || std::numeric_limits<version_t>::max() == minor ||
		    std::numeric_limits<version_t>::max() == patch) {
			throw std::logic_error("Wrong or missing version '" + std::to_string(major) + "." +
			                       std::to_string(minor) + "." + std::to_string(patch));
		}

		// TODO: Add more checks that all information is there

		return in;
	}

	ReadBuffer& read(ReadBuffer& in)
	{
		std::string line;

		if (in.readLine(line) && MAP_HEADER != line) {
			throw std::runtime_error("Trying to read non-UFO Map");
		}
		// Reset
		major = {};
		minor = std::numeric_limits<version_t>::max();
		patch = std::numeric_limits<version_t>::max();
		leaf_node_length.clear();
		num_depth_levels = {};
		num_blocks       = {};
		num_nodes        = {};
		map_info.clear();

		bool reading_map_info = false;
		while (in.readLine(line)) {
			if (read(line, reading_map_info)) {
				break;
			}
		}

		if (version_t(0) == major || std::numeric_limits<version_t>::max() == minor ||
		    std::numeric_limits<version_t>::max() == patch) {
			throw std::logic_error("Wrong or missing version '" + std::to_string(major) + "." +
			                       std::to_string(minor) + "." + std::to_string(patch));
		}

		// TODO: Add more checks that all information is there

		return in;
	}

	void write(std::filesystem::path const& filename) const
	{
		auto file = openWrite(filename);
		write(file);
	}

	std::ostream& write(std::ostream& out) const
	{
		out << MAP_HEADER << '\n';
		out << "VERSION: " << major << '.' << minor << '.' << patch << '\n';
		out << "LEAF_NODE_LENGTH: ";
		// TODO: Make sure that enough precision is used for the leaf node length
		if (2 == leaf_node_length.size()) {
			out << Vec<2, length_t>(leaf_node_length[0], leaf_node_length[1]);
		} else if (3 == leaf_node_length.size()) {
			out << Vec<3, length_t>(leaf_node_length[0], leaf_node_length[1],
			                        leaf_node_length[2]);
		} else if (4 == leaf_node_length.size()) {
			out << Vec<4, length_t>(leaf_node_length[0], leaf_node_length[1],
			                        leaf_node_length[2], leaf_node_length[4]);
		}
		out << '\n';
		out << "NUMBER_OF_DEPTH_LEVELS: " << num_depth_levels << '\n';
		out << "NUM_BLOCKS: " << num_blocks << '\n';
		out << "NUM_NODES: " << num_nodes << '\n';

		out << "MAP_INFO:\n";

		std::size_t longest_name = 4;
		std::size_t longest_type = 4;
		std::size_t longest_size = 12;
		for (auto const& info : map_info) {
			longest_name = std::max(longest_name, to_string(info.type).size());
			std::stringstream ss;
			ss << std::hex << std::showbase << to_underlying(info.type);
			longest_type = std::max(longest_type, static_cast<std::size_t>(ss.tellp()));
			ss.clear();
			ss << std::hex << std::showbase << info.size;
			longest_size = std::max(longest_size, static_cast<std::size_t>(ss.tellp()));
		}

		out << "  # ";
		out << std::setw(longest_name) << "Name";
		out << ": " << std::setw(longest_type) << "Type";
		out << ", " << std::setw(longest_size) << "Size (bytes)";
		out << '\n';
		for (auto const& info : map_info) {
			out << "  - ";
			if (MapType::NONE == info.type) {
				out << std::setw(longest_name) << "tree";
			} else {
				out << std::setw(longest_name) << to_string(info.type);
			}
			out << std::hex << std::showbase;
			out << ": " << std::setw(longest_type) << to_underlying(info.type);
			out << ", " << std::setw(longest_size) << info.size;
			out << std::noshowbase << std::dec << '\n';
		}

		out << "DATA:\n";

		return out;
	}

	WriteBuffer& write(WriteBuffer& out) const
	{
		std::ostringstream ss;
		write(ss);
		std::string s = ss.str();
		out.write(s.data(), s.length());
		return out;
	}

 private:
	[[nodiscard]] bool read(std::string& line, bool& reading_map_info)
	{
		std::cout << line << '\n';
		trim(line);
		if (line.empty() || '#' == line[0]) {
			return false;
		}

		auto        it = std::find(line.begin(), line.end(), ':');
		std::string param(line.begin(), it);
		rtrim(param);

		if (line.end() == it) {
			throw std::logic_error("No value for parameter '" + param + "'");
		}

		std::string value(it + 1, line.end());
		ltrim(value);

		if (reading_map_info && '-' == param[0]) {
			MapTypeInfo info;
			if (2 !=
			    std::sscanf(value.c_str(), "%" SCNx64 ", %" SCNx64, &info.type, &info.size)) {
				throw std::logic_error("Wrong map type info");
			}
			map_info.push_back(info);
			return false;
		}

		reading_map_info = false;

		if ("VERSION" == param) {
			if (3 != std::sscanf(value.c_str(), "%" SCNu32 ".%" SCNu32 ".%" SCNu32, &major,
			                     &minor, &patch)) {
				throw std::logic_error("Wrong version format");
			}
		} else if ("LEAF_NODE_LENGTH" == param) {
			leaf_node_length.resize(4);
			auto dim =
			    std::sscanf(value.c_str(), "x: %lf y: %lf z: %lf w: %lf", &leaf_node_length[0],
			                &leaf_node_length[1], &leaf_node_length[2], &leaf_node_length[3]);
			if (0 == dim) {
				throw std::logic_error("Wrong leaf node length format");
			}
			leaf_node_length.resize(dim);
		} else if ("NUMBER_OF_DEPTH_LEVELS" == param) {
			num_depth_levels = std::stoul(value);
		} else if ("NUM_BLOCKS" == param) {
			num_blocks = std::stoull(value);
		} else if ("NUM_NODES" == param) {
			num_nodes = std::stoull(value);
		} else if ("MAP_INFO" == param) {
			reading_map_info = true;
		} else if ("DATA" == param) {
			return true;
		}

		return false;
	}
};

inline std::ostream& operator<<(std::ostream& os, MapHeader const& header)
{
	return header.write(os);
}

inline std::istream& operator>>(std::istream& is, MapHeader& header)
{
	return header.read(is);
}
}  // namespace ufo

#endif  // UFO_MAP_MAP_HEADER_HPP