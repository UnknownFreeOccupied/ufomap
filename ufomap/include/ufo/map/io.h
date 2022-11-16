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
#include <cstring>
#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>

namespace ufo::map
{
// File options
struct FileOptions {
	bool compressed;
	node_size_t leaf_size;
	depth_t depth_levels;
};

// File header
struct FileHeader : FileOptions {
	static constexpr std::string_view FILE_HEADER = "# UFOMap file";
	static constexpr std::uint8_t CURRENT_MAJOR = 1;
	static constexpr std::uint8_t CURRENT_MINOR = 0;
	static constexpr std::uint8_t CURRENT_PATCH = 0;

	std::uint8_t major;
	std::uint8_t minor;
	std::uint8_t patch;
};

// Data identifiers
enum struct DataIdentifier : std::uint8_t {
	NO_DATA,
	OCCUPANCY,
	TIME,
	COLOR,
	INTENSITY,
	SEMANTIC,
	SURFEL,
	DISTANCE,
	COUNT,
	REFLECTION,
};

enum struct DataType : std::uint8_t {
	UINT8,
	UINT16,
	UINT32,
	UINT64,
	INT8,
	INT16,
	INT32,
	INT64,
	FLOAT32,
	FLOAT64
};

template <typename T>
[[nodiscard]] constexpr inline DataType dataType()
{
	if constexpr (std::is_same_v<std::uint8_t, T>) {
		return DataType::UINT8;
	} else if constexpr (std::is_same_v<std::uint16_t, T>) {
		return DataType::UINT16;
	} else if constexpr (std::is_same_v<std::uint32_t, T>) {
		return DataType::UINT32;
	} else if constexpr (std::is_same_v<std::uint64_t, T>) {
		return DataType::UINT64;
	} else if constexpr (std::is_same_v<std::int8_t, T>) {
		return DataType::INT8;
	} else if constexpr (std::is_same_v<std::int16_t, T>) {
		return DataType::INT16;
	} else if constexpr (std::is_same_v<std::int32_t, T>) {
		return DataType::INT32;
	} else if constexpr (std::is_same_v<std::int64_t, T>) {
		return DataType::INT64;
	} else if constexpr (std::is_same_v<float, T>) {
		return DataType::FLOAT32;
	} else if constexpr (std::is_same_v<double, T>) {
		return DataType::FLOAT64;
	} else {
		// TODO: Throw
	}
}

template <typename T>
[[nodiscard]] constexpr inline DataType dataType(T const)
{
	return dataType<T>();
}

// Data storage
class ReadBuffer
{
 public:
	virtual ~ReadBuffer() {}

	ReadBuffer& read(void* dest, std::size_t count)
	{
		if (size() < index_ + count) {
			// TODO: Fill in exception message
			throw std::out_of_range("");
		}

		std::memmove(dest, data_ + index_, count);
		index_ += count;

		return *this;
	}

	[[nodiscard]] virtual std::size_t size() const { return size_; }

	[[nodiscard]] constexpr std::size_t readIndex() const noexcept { return index_; }

	constexpr void skipRead(std::size_t count) noexcept { index_ += count; }

	constexpr void setReadIndex(std::size_t index) noexcept { index_ = index; }

	[[nodiscard]] std::size_t readLeft() const noexcept
	{
		return size_ < index_ ? 0 : index_ - size_;
	}

	[[nodiscard]] bool good() const noexcept
	{
		return true;  // TODO: Implement correctly
	}

	[[nodiscard]] bool fail() const noexcept
	{
		return false;  // TODO: Implement correctly
	}

 protected:
	std::uint8_t const* data_ = nullptr;
	std::size_t size_ = 0;
	std::size_t index_ = 0;
};

class WriteBuffer
{
 public:
	virtual ~WriteBuffer()
	{
		if (data_) {
			free(data_);
		}
	}

	WriteBuffer& write(void const* src, std::size_t count)
	{
		if (capacity() < index_ + count) {
			reserve(index_ + count);
		}

		std::memmove(data_ + index_, src, count);
		index_ += count;

		size_ = std::max(size_, index_);

		return *this;
	}

	virtual void reserve(std::size_t new_cap)
	{
		if (cap_ >= new_cap) {
			return;
		}

		if (auto p_new =
		        static_cast<std::uint8_t*>(realloc(data_, new_cap * sizeof(std::uint8_t)))) {
			data_ = p_new;
			cap_ = new_cap;
		} else {
			throw std::bad_alloc();
		}
	}

	[[nodiscard]] virtual std::size_t size() const { return size_; }

	[[nodiscard]] constexpr std::size_t capacity() const noexcept { return cap_; }

	[[nodiscard]] constexpr std::size_t writeIndex() const noexcept { return index_; }

	constexpr void setWriteIndex(std::size_t index) noexcept { index_ = index; }

 protected:
	std::uint8_t* data_ = nullptr;
	std::size_t size_ = 0;
	std::size_t cap_ = 0;
	std::size_t index_ = 0;
};

class Buffer : public ReadBuffer, public WriteBuffer
{
 public:
	virtual ~Buffer() {}

	// TODO: Implement

	[[nodiscard]] std::size_t size() const override { return WriteBuffer::size(); }

	void reserve(std::size_t new_cap) override
	{
		WriteBuffer::reserve(new_cap);
		ReadBuffer::data_ = WriteBuffer::data_;
	}
};

[[nodiscard]] bool isUFOMap(std::filesystem::path const& filename);

[[nodiscard]] bool isUFOMap(std::istream& in);

[[nodiscard]] bool isUFOMap(ReadBuffer& in);

[[nodiscard]] FileHeader readHeader(std::filesystem::path const& filename);

[[nodiscard]] FileHeader readHeader(std::istream& in);

[[nodiscard]] FileHeader readHeader(ReadBuffer& in);

void writeHeader(std::ostream& out, FileOptions const& options);

void writeHeader(WriteBuffer& out, FileOptions const& options);

std::size_t maxSizeCompressed(std::size_t uncompressed_size);

bool compressData(std::istream& in, std::ostream& out,
                  std::uint64_t uncompressed_data_size, int acceleration_level = 1,
                  int compression_level = 0);

bool compressData(ReadBuffer& in, WriteBuffer& out, std::uint64_t uncompressed_data_size,
                  int acceleration_level = 1, int compression_level = 0);

bool decompressData(std::istream& in, std::ostream& out,
                    std::uint64_t uncompressed_data_size);

bool decompressData(ReadBuffer& in, WriteBuffer& out,
                    std::uint64_t uncompressed_data_size);
}  // namespace ufo::map
#endif  // UFO_MAP_IO_H