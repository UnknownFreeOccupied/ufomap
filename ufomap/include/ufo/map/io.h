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

#ifndef UFO_MAP_IO_H
#define UFO_MAP_IO_H

// UFO
#include <ufo/map/types.h>

// STL
#include <filesystem>
#include <iostream>
#include <map>
#include <string>
#include <string_view>

namespace ufo::map
{
// Data identifiers
enum struct DataIdentifier : std::uint8_t {
	OCCUPANCY,
	TIME,
	COLOR,
	SEMANTIC,
	SURFEL,
	SIGNED_DISTANCE,
};

// File options
struct FileOptions {
	bool compressed;
	double resolution;
	depth_t depth_levels;
};

// File header
struct FileHeader : FileOptions {
	static constexpr std::string_view FILE_HEADER = "# UFOMap file";
	static constexpr uint8_t CURRENT_MAJOR = 1;
	static constexpr uint8_t CURRENT_MINOR = 0;
	static constexpr uint8_t CURRENT_PATCH = 0;

	uint8_t major;
	uint8_t minor;
	uint8_t patch;
};

bool isUFOMapFile(std::filesystem::path const& filename);

bool isUFOMapFile(std::istream& in_stream);

template <class InputIt>
bool isUFOMapFile(InputIt& in_buffer)
{
	std::string line;
	// TODO: Implement std::getline(in_stream, line);
	return 0 == line.compare(0, FileHeader::FILE_HEADER.length(), FileHeader::FILE_HEADER);
}

FileHeader readHeader(std::filesystem::path const& filename);

FileHeader readHeader(std::istream& in_stream);

template <class InputIt>
FileHeader readHeader(InputIt& in_buffer)
{
	if (!isUFOMapFile(in_buffer)) {
		throw std::runtime_error("Trying to read non-UFOMap file");
	}

	FileHeader header;
	in_stream.read(reinterpret_cast<char*>(&header.major), sizeof(header.major));
	in_stream.read(reinterpret_cast<char*>(&header.minor), sizeof(header.minor));
	in_stream.read(reinterpret_cast<char*>(&header.patch), sizeof(header.patch));

	uint8_t compressed;
	in_stream.read(reinterpret_cast<char*>(&compressed), sizeof(compressed));
	header.compressed = compressed & 1U;

	in_stream.read(reinterpret_cast<char*>(&header.resolution), sizeof(header.resolution));
	in_stream.read(reinterpret_cast<char*>(&header.depth_levels),
	               sizeof(header.depth_levels));

	return header;
}

void writeHeader(std::ostream& out_stream, FileOptions const& options);

template <class OutputIt>
void writeHeader(OutputIt& out_buffer, FileOptions const& options)
{
	// TODO: Implement

	out_stream << FileHeader::FILE_HEADER;
	out_stream.write(reinterpret_cast<char const*>(&FileHeader::CURRENT_MAJOR),
	                 sizeof(FileHeader::CURRENT_MAJOR));
	out_stream.write(reinterpret_cast<char const*>(&FileHeader::CURRENT_MINOR),
	                 sizeof(FileHeader::CURRENT_MINOR));
	out_stream.write(reinterpret_cast<char const*>(&FileHeader::CURRENT_PATCH),
	                 sizeof(FileHeader::CURRENT_PATCH));

	uint8_t compressed = options.compressed ? 1U : 0U;
	out_stream.write(reinterpret_cast<char const*>(&compressed), sizeof(compressed));

	out_stream.write(reinterpret_cast<char const*>(&options.resolution),
	                 sizeof(options.resolution));
	out_stream.write(reinterpret_cast<char const*>(&options.depth_levels),
	                 sizeof(options.depth_levels));
}

bool compressData(std::istream& in_stream, std::ostream& out_stream,
                  uint64_t uncompressed_data_size, int acceleration_level = 1,
                  int compression_level = 0);

bool decompressData(std::istream& in_stream, std::ostream& out_stream,
                    uint64_t uncompressed_data_size);

bool decompressData(std::istream& in_stream, std::ostream& out_stream,
                    uint64_t uncompressed_data_size, uint64_t& compressed_data_size);
}  // namespace ufo::map
#endif  // UFO_MAP_IO_H