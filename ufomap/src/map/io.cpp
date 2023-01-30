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

// UFO
#include <ufo/map/io.h>

// STL
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>

// LZ4 compression
#include <lz4.h>
#include <lz4hc.h>

namespace ufo::map
{
bool isUFOMap(std::filesystem::path const& filename)
{
	std::ifstream file;
	file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	file.imbue(std::locale());
	file.open(filename, std::ios_base::in | std::ios_base::binary);

	return isUFOMap(file);
}

bool isUFOMap(std::istream& in)
{
	// FIXME: Read max FILE_HEADER length
	std::string line;
	std::getline(in, line);
	return 0 == line.compare(0, FileHeader::FILE_HEADER.length(), FileHeader::FILE_HEADER);
}

bool isUFOMap(ReadBuffer& in)
{
	constexpr auto length = FileHeader::FILE_HEADER.length();
	if (length > in.readLeft()) {
		return false;
	}

	std::string line(length, ' ');
	in.read(line.data(), length);
	in.setReadIndex(in.readIndex() + 1);  // Skip newline
	return 0 == line.compare(0, length, FileHeader::FILE_HEADER);
}

FileHeader readHeader(std::filesystem::path const& filename)
{
	std::ifstream file;
	file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	file.imbue(std::locale());
	file.open(filename, std::ios_base::in | std::ios_base::binary);

	return readHeader(file);
}

FileHeader readHeader(std::istream& in)
{
	if (!isUFOMap(in)) {
		throw std::runtime_error("Trying to read non-UFOMap");
	}

	FileHeader header;
	in.read(reinterpret_cast<char*>(&header.major), sizeof(header.major));
	in.read(reinterpret_cast<char*>(&header.minor), sizeof(header.minor));
	in.read(reinterpret_cast<char*>(&header.patch), sizeof(header.patch));
	in.read(reinterpret_cast<char*>(&header.little_endian), sizeof(header.little_endian));

	std::uint8_t compressed;
	in.read(reinterpret_cast<char*>(&compressed), sizeof(compressed));
	header.compressed = compressed & 1U;

	in.read(reinterpret_cast<char*>(&header.leaf_size), sizeof(header.leaf_size));
	in.read(reinterpret_cast<char*>(&header.depth_levels), sizeof(header.depth_levels));

	return header;
}

FileHeader readHeader(ReadBuffer& in)
{
	if (!isUFOMap(in)) {
		throw std::runtime_error("Trying to read non-UFOMap");
	}

	FileHeader header;
	in.read(&header.major, sizeof(header.major));
	in.read(&header.minor, sizeof(header.minor));
	in.read(&header.patch, sizeof(header.patch));
	in.read(&header.little_endian, sizeof(header.little_endian));

	std::uint8_t compressed;
	in.read(&compressed, sizeof(compressed));
	header.compressed = compressed & 1U;

	in.read(&header.leaf_size, sizeof(header.leaf_size));
	in.read(&header.depth_levels, sizeof(header.depth_levels));

	return header;
}

void writeHeader(std::ostream& out, FileOptions const& options)
{
	out << FileHeader::FILE_HEADER << '\n';
	out.write(reinterpret_cast<char const*>(&FileHeader::CURRENT_MAJOR),
	          sizeof(FileHeader::CURRENT_MAJOR));
	out.write(reinterpret_cast<char const*>(&FileHeader::CURRENT_MINOR),
	          sizeof(FileHeader::CURRENT_MINOR));
	out.write(reinterpret_cast<char const*>(&FileHeader::CURRENT_PATCH),
	          sizeof(FileHeader::CURRENT_PATCH));
	out.write(reinterpret_cast<char const*>(&FileHeader::LITTLE_ENDIAN),
	          sizeof(FileHeader::LITTLE_ENDIAN));

	std::uint8_t const compressed = options.compressed ? std::uint8_t(1) : std::uint8_t(0);
	double const leaf_size = options.leaf_size;
	std::uint8_t const depth_levels = options.depth_levels;

	out.write(reinterpret_cast<char const*>(&compressed), sizeof(compressed));
	out.write(reinterpret_cast<char const*>(&leaf_size), sizeof(leaf_size));
	out.write(reinterpret_cast<char const*>(&depth_levels), sizeof(depth_levels));
}

void writeHeader(WriteBuffer& out, FileOptions const& options)
{
	out.write(FileHeader::FILE_HEADER.data(), FileHeader::FILE_HEADER.length());
	char nl = '\n';
	out.write(&nl, sizeof(nl));
	out.write(&FileHeader::CURRENT_MAJOR, sizeof(FileHeader::CURRENT_MAJOR));
	out.write(&FileHeader::CURRENT_MINOR, sizeof(FileHeader::CURRENT_MINOR));
	out.write(&FileHeader::CURRENT_PATCH, sizeof(FileHeader::CURRENT_PATCH));
	out.write(&FileHeader::LITTLE_ENDIAN, sizeof(FileHeader::LITTLE_ENDIAN));

	std::uint8_t const compressed = options.compressed ? std::uint8_t(1) : std::uint8_t(0);
	double const leaf_size = options.leaf_size;
	std::uint8_t const depth_levels = options.depth_levels;

	out.write(&compressed, sizeof(compressed));
	out.write(&leaf_size, sizeof(leaf_size));
	out.write(&depth_levels, sizeof(depth_levels));
}

std::size_t maxSizeCompressed(std::size_t uncompressed_size)
{
	constexpr std::size_t max = std::numeric_limits<std::int32_t>::max();

	std::size_t compressed_size = 0;

	for (std::size_t size = std::min(max, uncompressed_size); 0 != size;
	     uncompressed_size -= size, size = std::min(max, uncompressed_size)) {
		compressed_size += size;
	}

	return compressed_size;
}

bool compressData(std::istream& in, std::ostream& out,
                  std::uint_fast64_t uncompressed_data_size, int acceleration_level,
                  int compression_level)
{
	constexpr std::uint_fast64_t max = std::numeric_limits<std::int32_t>::max();

	std::uint_fast64_t begin = 0;
	std::uint_fast64_t end = std::min(max, uncompressed_data_size);
	int const max_dst_size = LZ4_compressBound(static_cast<int>(end));
	auto data = std::make_unique<char[]>(end);
	auto compressed_data = std::make_unique<char[]>(max_dst_size);
	while (begin != end) {
		std::int32_t num_data = static_cast<int>(end - begin);

		// Compress data
		in.read(data.get(), num_data);

		std::int32_t compressed_data_size =
		    (0 >= compression_level)
		        ? LZ4_compress_fast(data.get(), compressed_data.get(), num_data, max_dst_size,
		                            acceleration_level)
		        : LZ4_compress_HC(data.get(), compressed_data.get(), num_data, max_dst_size,
		                          compression_level);

		// Check if compression successful
		if (0 <= compressed_data_size) {
			// Write amount of data to output stream
			out.write(reinterpret_cast<char*>(&compressed_data_size),
			          sizeof(compressed_data_size));
			// Write compressed data to output stream
			out.write(compressed_data.get(), compressed_data_size);
		} else {
			return false;
		}

		// Move to next
		begin = end;
		std::uint_fast64_t const diff = uncompressed_data_size - end;
		end += std::min(max, diff);
	}

	return true;
}

bool compressData(ReadBuffer& in, WriteBuffer& out, std::uint64_t uncompressed_data_size,
                  int acceleration_level, int compression_level)
{
	constexpr std::uint_fast64_t max = std::numeric_limits<std::int32_t>::max();

	std::uint_fast64_t begin = 0;
	std::uint_fast64_t end = std::min(max, uncompressed_data_size);
	int const max_dst_size = LZ4_compressBound(static_cast<int>(end));
	auto data = std::make_unique<char[]>(end);
	auto compressed_data = std::make_unique<char[]>(max_dst_size);
	while (begin != end) {
		std::int32_t num_data = static_cast<int>(end - begin);

		// Compress data
		in.read(data.get(), num_data);

		std::int32_t compressed_data_size =
		    (0 >= compression_level)
		        ? LZ4_compress_fast(data.get(), compressed_data.get(), num_data, max_dst_size,
		                            acceleration_level)
		        : LZ4_compress_HC(data.get(), compressed_data.get(), num_data, max_dst_size,
		                          compression_level);

		// Check if compression successful
		if (0 <= compressed_data_size) {
			// Write amount of data to output stream
			out.write(&compressed_data_size, sizeof(compressed_data_size));
			// Write compressed data to output stream
			out.write(compressed_data.get(), compressed_data_size);
		} else {
			return false;
		}

		// Move to next
		begin = end;
		std::uint_fast64_t const diff = uncompressed_data_size - end;
		end += std::min(max, diff);
	}

	return true;
}

bool decompressData(std::istream& in, std::ostream& out,
                    std::uint_fast64_t uncompressed_data_size)
{
	auto regen_buffer = std::make_unique<char[]>(uncompressed_data_size);
	size_t cur = 0;
	while (in.good() && cur < uncompressed_data_size) {
		// Get size of compressed data
		std::int32_t compressed_data_size;
		in.read(reinterpret_cast<char*>(&compressed_data_size), sizeof(compressed_data_size));

		// Decompress data
		auto compressed_data = std::make_unique<char[]>(compressed_data_size);
		in.read(compressed_data.get(), compressed_data_size);
		int const decompressed_size = LZ4_decompress_safe(
		    compressed_data.get(), regen_buffer.get() + cur, compressed_data_size,
		    static_cast<int>(uncompressed_data_size - cur));
		cur += decompressed_size;

		// Check if decompression successful
		if (0 > decompressed_size) {
			return false;
		}
	}

	// Write decompressed data to output stream
	out.write(regen_buffer.get(), uncompressed_data_size);

	return !in.fail();
}

bool decompressData(ReadBuffer& in, WriteBuffer& out,
                    std::uint64_t uncompressed_data_size)
{
	auto regen_buffer = std::make_unique<char[]>(uncompressed_data_size);
	size_t cur = 0;
	while (in.good() && cur < uncompressed_data_size) {
		// Get size of compressed data
		std::int32_t compressed_data_size;
		in.read(&compressed_data_size, sizeof(compressed_data_size));

		// Decompress data
		auto compressed_data = std::make_unique<char[]>(compressed_data_size);
		in.read(compressed_data.get(), compressed_data_size);
		int const decompressed_size = LZ4_decompress_safe(
		    compressed_data.get(), regen_buffer.get() + cur, compressed_data_size,
		    static_cast<int>(uncompressed_data_size - cur));
		cur += decompressed_size;

		// Check if decompression successful
		if (0 > decompressed_size) {
			return false;
		}
	}

	// Write decompressed data to output stream
	out.write(regen_buffer.get(), uncompressed_data_size);

	return !in.fail();
}
}  // namespace ufo::map
