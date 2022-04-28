/*
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * License: BSD 3
 */

/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, D. Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
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

// STL
#include <filesystem>
#include <fstream>
#include <istream>
#include <map>
#include <string>
#include <string_view>
#include <vector>

namespace ufo::map
{
static constexpr std::string_view FILE_HEADER = "# UFOMap file";  // File header
static constexpr std::string_view FILE_VERSION = "1.1.0";         // File version

using FileInfo = std::map<std::string, std::vector<std::string>>;

bool correctFileType(std::filesystem::path const& filename);

bool correctFileType(std::istream& in_stream);

FileInfo readHeader(std::filesystem::path const& filename);

FileInfo readHeader(std::istream& in_stream);

void writeHeader(std::ostream& out_stream, FileInfo const& info);

bool compressData(std::istream& in_stream, std::ostream& out_stream,
                  uint64_t uncompressed_data_size, int acceleration_level = 1,
                  int compression_level = 0);

bool decompressData(std::istream& in_stream, std::ostream& out_stream,
                    uint64_t uncompressed_data_size);
}  // namespace ufo::map
#endif  // UFO_MAP_IO_H