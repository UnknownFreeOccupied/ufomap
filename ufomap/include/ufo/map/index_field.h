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

#ifndef UFO_MAP_INDEX_FIELD_H
#define UFO_MAP_INDEX_FIELD_H

// STL
#include <array>
#include <bit>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace ufo::map
{
using index_field_t = std::uint8_t;

struct IndexField {
	index_field_t field = 0;

	struct Reference {
		friend class IndexField;

	 public:
		~Reference() {}

		constexpr Reference& operator=(bool x) noexcept
		{
			field_ ^=
			    static_cast<index_field_t>(-static_cast<index_field_t>(x) ^ field_) & index_;
			return *this;
		}

		constexpr Reference& operator=(Reference const& x) noexcept { return operator=(!!x); }

		constexpr operator bool() const noexcept
		{
			return index_field_t(0) != (field_ & index_);
		}

		constexpr bool operator~() const noexcept
		{
			return index_field_t(0) == (field_ & index_);
		}

		constexpr Reference& flip() noexcept
		{
			field_ ^= index_;
			return *this;
		}

	 private:
		constexpr Reference(index_field_t& field, std::size_t pos) noexcept
		    : field_(field), index_(static_cast<index_field_t>(index_field_t(1) << pos))
		{
		}

	 private:
		index_field_t& field_;
		index_field_t index_;
	};

	constexpr IndexField() noexcept = default;

	constexpr IndexField(index_field_t val) noexcept : field(val) {}

	template <class CharT, class Traits, class Alloc>
	explicit IndexField(std::basic_string<CharT, Traits, Alloc> const& str,
	                    typename std::basic_string<CharT, Traits, Alloc>::size_type pos = 0,
	                    typename std::basic_string<CharT, Traits, Alloc>::size_type n =
	                        std::basic_string<CharT, Traits, Alloc>::npos,
	                    CharT zero = CharT('0'), CharT one = CharT('1'))
	{
		// TODO: Implement
	}

	template <class CharT>
	explicit IndexField(
	    CharT const* str,
	    typename std::basic_string<CharT>::size_type n = std::basic_string<CharT>::npos,
	    CharT zero = CharT('0'), CharT one = CharT('1'))
	{
		// TODO: Implement
	}

	constexpr bool operator==(IndexField rhs) const noexcept { return field == rhs.field; }

	constexpr bool operator!=(IndexField rhs) const noexcept { return !(*this == rhs); }

	[[nodiscard]] constexpr bool operator[](std::size_t pos) const
	{
		return (field >> pos) & index_field_t(1);
	}

	[[nodiscard]] Reference operator[](std::size_t pos) { return Reference(field, pos); }

	[[nodiscard]] bool test(std::size_t pos) const
	{
		if (size() <= pos) {
			std::out_of_range("position (which is " + std::to_string(pos) +
			                  ") >= size (which is " + std::to_string(size()) + ")");
		}
		return operator[](pos);
	}

	[[nodiscard]] constexpr bool all() const noexcept
	{
		return std::numeric_limits<index_field_t>::max() == field;
	}

	[[nodiscard]] constexpr bool any() const noexcept { return index_field_t(0) != field; }

	[[nodiscard]] constexpr bool none() const noexcept { return index_field_t(0) == field; }

	[[nodiscard]] constexpr std::size_t count() const noexcept
	{
		return std::popcount(field);
	}

	[[nodiscard]] static constexpr std::size_t size() noexcept { return 8; }

	constexpr IndexField& operator&=(IndexField const other) noexcept
	{
		field &= other.field;
		return *this;
	}

	constexpr IndexField& operator|=(IndexField const& other) noexcept
	{
		field |= other.field;
		return *this;
	}

	constexpr IndexField& operator^=(IndexField const& other) noexcept
	{
		field ^= other.field;
		return *this;
	}

	constexpr IndexField operator~() const noexcept
	{
		return IndexField(static_cast<index_field_t>(~field));
	}

	constexpr IndexField& set() noexcept
	{
		field = std::numeric_limits<index_field_t>::max();
		return *this;
	}

	constexpr IndexField& set(std::size_t pos, bool value = true)
	{
		if (size() <= pos) {
			std::out_of_range("position (which is " + std::to_string(pos) +
			                  ") >= size (which is " + std::to_string(size()) + ")");
		}
		field ^= static_cast<index_field_t>((-static_cast<index_field_t>(value) ^ field) &
		                                    (index_field_t(1) << pos));
		return *this;
	}

	constexpr IndexField& reset() noexcept
	{
		field = 0;
		return *this;
	}

	constexpr IndexField& reset(std::size_t pos) { return set(pos, false); }

	constexpr IndexField& flip() noexcept
	{
		field = static_cast<index_field_t>(~field);
		return *this;
	}

	constexpr IndexField& flip(std::size_t pos)
	{
		if (size() <= pos) {
			std::out_of_range("position (which is " + std::to_string(pos) +
			                  ") >= size (which is " + std::to_string(size()) + ")");
		}
		field ^= static_cast<index_field_t>(index_field_t(1) << pos);
		return *this;
	}
};

constexpr IndexField operator&(IndexField lhs, IndexField rhs) noexcept
{
	return IndexField(lhs.field & rhs.field);
}

constexpr IndexField operator|(IndexField lhs, IndexField rhs) noexcept
{
	return IndexField(lhs.field | rhs.field);
}

constexpr IndexField operator^(IndexField lhs, IndexField rhs) noexcept
{
	return IndexField(lhs.field ^ rhs.field);
}

template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& operator<<(std::basic_ostream<CharT, Traits>& os,
                                              IndexField x)
{
	return os << +x.field;
}

template <class CharT, class Traits>
std::basic_istream<CharT, Traits>& operator>>(std::basic_istream<CharT, Traits>& is,
                                              IndexField& x)
{
	// TODO: Implement
	return is;
}
}  // namespace ufo::map

namespace std
{
template <>
struct hash<ufo::map::IndexField> {
	std::size_t operator()(ufo::map::IndexField x) const { return x.field; }
};
}  // namespace std

#endif  // UFO_MAP_INDEX_FIELD_H