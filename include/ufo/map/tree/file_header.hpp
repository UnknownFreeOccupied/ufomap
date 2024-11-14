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

#ifndef UFO_MAP_TREE_FILE_HEADER_HPP
#define UFO_MAP_TREE_FILE_HEADER_HPP

// UFO
#include <ufo/container/tree/type.hpp>
#include <ufo/utility/enum.hpp>
#include <ufo/utility/io/buffer.hpp>

// STL
#if __cplusplus >= 202002L
#include <bit>
#endif
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace ufo
{
struct FileHeader {
	using length_t  = double;
	using depth_t   = std::uint32_t;
	using tree_t    = std::uint32_t;
	using version_t = std::uint32_t;

	static constexpr std::string_view const FILE_HEADER   = "# UFOMap file";
	static constexpr version_t const        CURRENT_MAJOR = 1;
	static constexpr version_t const        CURRENT_MINOR = 0;
	static constexpr version_t const        CURRENT_PATCH = 0;
	// static constexpr std::uint8_t     IS_LITTLE_ENDIAN =
	//     std::endian::native == std::endian::little;

	version_t major;
	version_t minor;
	version_t patch;
	// std::uint8_t is_little_endian;

	TreeType tree_type;
	length_t leaf_node_length;
	depth_t  num_depth_levels;
	bool     compressed;

	FileHeader()                  = default;
	FileHeader(FileHeader const&) = default;

	FileHeader(TreeType tree_type, length_t leaf_node_length, depth_t num_depth_levels,
	           bool compressed)
	    : major(CURRENT_MAJOR)
	    , minor(CURRENT_MINOR)
	    , patch(CURRENT_PATCH)
	    , tree_type(tree_type)
	    , leaf_node_length(leaf_node_length)
	    , num_depth_levels(num_depth_levels)
	    , compressed(compressed)
	{
	}

	FileHeader(std::filesystem::path const& filename) { read(filename); }

	FileHeader(std::istream& in) { read(in); }

	FileHeader(ReadBuffer& in) { read(in); }

	[[nodiscard]] static std::ifstream openRead(std::filesystem::path const& filename)
	{
		std::ifstream file;
		file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		file.imbue(std::locale());
		file.open(filename, std::ios::in | std::ios::binary);
		return file;
	}

	[[nodiscard]] static std::ofstream openWrite(std::filesystem::path const& filename)
	{
		std::ofstream file;
		file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		file.imbue(std::locale());
		file.open(filename, std::ios::out | std::ios::binary);
		return file;
	}

	void read(std::filesystem::path const& filename)
	{
		auto file = openRead(filename);
		read(file);
	}

	std::istream& read(std::istream& in)
	{
		std::string line;
		std::getline(in, line);

		if (FILE_HEADER != line) {
			throw std::runtime_error("Trying to read non-UFOMap");
		}

		in.read(reinterpret_cast<char*>(&major), sizeof(major));
		in.read(reinterpret_cast<char*>(&minor), sizeof(minor));
		in.read(reinterpret_cast<char*>(&patch), sizeof(patch));
		// in.read(reinterpret_cast<char*>(&is_little_endian),
		// sizeof(is_little_endian));

		tree_t tt;
		in.read(reinterpret_cast<char*>(&tt), sizeof(tt));
		tree_type = static_cast<TreeType>(tt);

		in.read(reinterpret_cast<char*>(&leaf_node_length), sizeof(leaf_node_length));
		in.read(reinterpret_cast<char*>(&num_depth_levels), sizeof(num_depth_levels));

		std::uint8_t c;
		in.read(reinterpret_cast<char*>(&c), sizeof(c));
		compressed = static_cast<bool>(c);

		return in;
	}

	ReadBuffer& read(ReadBuffer& in)
	{
		auto length = FILE_HEADER.length();
		if (in.readLeft() < length) {
			throw std::runtime_error("Trying to read non-UFOMap");
		}

		std::string line(length, ' ');
		in.read(line.data(), length);
		if (FILE_HEADER != line) {
			throw std::runtime_error("Trying to read non-UFOMap");
		}

		// Skipping new line
		in.skipRead(1);

		in.read(&major);
		in.read(&minor);
		in.read(&patch);
		// in.read(&is_little_endian);

		tree_t tt;
		in.read(&tt);
		tree_type = static_cast<TreeType>(tt);

		in.read(&leaf_node_length);
		in.read(&num_depth_levels);

		std::uint8_t c;
		in.read(&c);
		compressed = static_cast<bool>(c);

		return in;
	}

	void write(std::filesystem::path const& filename) const
	{
		auto file = openWrite(filename);
		write(file);
	}

	std::ostream& write(std::ostream& out) const
	{
		out << FILE_HEADER << '\n';
		out.write(reinterpret_cast<char const*>(&major), sizeof(major));
		out.write(reinterpret_cast<char const*>(&minor), sizeof(minor));
		out.write(reinterpret_cast<char const*>(&patch), sizeof(patch));
		// out.write(reinterpret_cast<char const*>(&is_little_endian),
		//           sizeof(is_little_endian));

		tree_t tt = static_cast<tree_t>(to_underlying(tree_type));
		out.write(reinterpret_cast<char const*>(&tt), sizeof(tt));

		out.write(reinterpret_cast<char const*>(&leaf_node_length), sizeof(leaf_node_length));
		out.write(reinterpret_cast<char const*>(&num_depth_levels), sizeof(num_depth_levels));

		std::uint8_t c = static_cast<std::uint8_t>(compressed);
		out.write(reinterpret_cast<char const*>(&c), sizeof(c));

		return out;
	}

	WriteBuffer& write(WriteBuffer& out) const
	{
		out.write(FILE_HEADER.data(), FILE_HEADER.length());
		char nl = '\n';
		out.write(&nl);
		out.write(&major);
		out.write(&minor);
		out.write(&patch);
		// out.write(&is_little_endian);

		tree_t tt = static_cast<tree_t>(to_underlying(tree_type));
		out.write(&tt);

		out.write(&leaf_node_length);
		out.write(&num_depth_levels);

		std::uint8_t c = static_cast<std::uint8_t>(compressed);
		out.write(&c);

		return out;
	}
};
}  // namespace ufo

#endif  // UFO_MAP_TREE_FILE_HEADER_HPP