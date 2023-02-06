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
#include <type_traits>

namespace ufo::map
{

template <std::size_t N>
struct BitSet {
 private:
	using T = std::conditional_t<
	    8 >= N, std::uint8_t,
	    std::conditional_t<16 >= N, std::uint16_t,
	                       std::conditional_t<32 >= N, std::uint32_t, std::uint64_t>>>;

 public:
	T set = 0;

	struct Reference {
		friend class BitSet;

	 public:
		~Reference() {}

		constexpr Reference& operator=(bool x) noexcept
		{
			set_ ^= static_cast<T>(-static_cast<T>(x) ^ set_) & index_;
			return *this;
		}

		constexpr Reference& operator=(Reference const& x) noexcept { return operator=(!!x); }

		constexpr operator bool() const noexcept { return T(0) != (set_ & index_); }

		constexpr bool operator~() const noexcept { return T(0) == (set_ & index_); }

		constexpr Reference& flip() noexcept
		{
			set_ ^= index_;
			return *this;
		}

	 private:
		constexpr Reference(T& field, std::size_t pos) noexcept
		    : set_(field), index_(static_cast<T>(T(1) << pos))
		{
		}

	 private:
		T& set_;
		T index_;
	};

	constexpr BitSet() noexcept = default;

	constexpr BitSet(T val) noexcept : field(val) {}

	template <class CharT, class Traits, class Alloc>
	explicit BitSet(std::basic_string<CharT, Traits, Alloc> const& str,
	                typename std::basic_string<CharT, Traits, Alloc>::size_type pos = 0,
	                typename std::basic_string<CharT, Traits, Alloc>::size_type n =
	                    std::basic_string<CharT, Traits, Alloc>::npos,
	                CharT zero = CharT('0'), CharT one = CharT('1'))
	{
		// TODO: Implement
	}

	template <class CharT>
	explicit BitSet(
	    CharT const* str,
	    typename std::basic_string<CharT>::size_type n = std::basic_string<CharT>::npos,
	    CharT zero = CharT('0'), CharT one = CharT('1'))
	{
		// TODO: Implement
	}

	constexpr bool operator==(BitSet rhs) const noexcept { return field == rhs.field; }

	constexpr bool operator!=(BitSet rhs) const noexcept { return !(*this == rhs); }

	[[nodiscard]] constexpr bool operator[](std::size_t pos) const
	{
		return (field >> pos) & T(1);
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
		return std::numeric_limits<T>::max() == field;
	}

	[[nodiscard]] constexpr bool any() const noexcept { return T(0) != field; }

	[[nodiscard]] constexpr bool none() const noexcept { return T(0) == field; }

	[[nodiscard]] constexpr std::size_t count() const noexcept
	{
		return std::popcount(field);
	}

	[[nodiscard]] static constexpr std::size_t size() noexcept { return 8; }

	constexpr BitSet& operator&=(BitSet const other) noexcept
	{
		field &= other.field;
		return *this;
	}

	constexpr BitSet& operator|=(BitSet const& other) noexcept
	{
		field |= other.field;
		return *this;
	}

	constexpr BitSet& operator^=(BitSet const& other) noexcept
	{
		field ^= other.field;
		return *this;
	}

	constexpr BitSet operator~() const noexcept { return BitSet(static_cast<T>(~field)); }

	constexpr BitSet& set() noexcept
	{
		field = std::numeric_limits<T>::max();
		return *this;
	}

	constexpr BitSet& set(std::size_t pos, bool value = true)
	{
		if (size() <= pos) {
			std::out_of_range("position (which is " + std::to_string(pos) +
			                  ") >= size (which is " + std::to_string(size()) + ")");
		}
		field ^= static_cast<T>((-static_cast<T>(value) ^ field) & (T(1) << pos));
		return *this;
	}

	constexpr BitSet& reset() noexcept
	{
		field = 0;
		return *this;
	}

	constexpr BitSet& reset(std::size_t pos) { return set(pos, false); }

	constexpr BitSet& flip() noexcept
	{
		field = static_cast<T>(~field);
		return *this;
	}

	constexpr BitSet& flip(std::size_t pos)
	{
		if (size() <= pos) {
			std::out_of_range("position (which is " + std::to_string(pos) +
			                  ") >= size (which is " + std::to_string(size()) + ")");
		}
		field ^= static_cast<T>(T(1) << pos);
		return *this;
	}
};

constexpr BitSet operator&(BitSet lhs, BitSet rhs) noexcept
{
	return BitSet(lhs.field & rhs.field);
}

constexpr BitSet operator|(BitSet lhs, BitSet rhs) noexcept
{
	return BitSet(lhs.field | rhs.field);
}

constexpr BitSet operator^(BitSet lhs, BitSet rhs) noexcept
{
	return BitSet(lhs.field ^ rhs.field);
}

template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& operator<<(std::basic_ostream<CharT, Traits>& os,
                                              BitSet x)
{
	return os << +x.field;
}

template <class CharT, class Traits>
std::basic_istream<CharT, Traits>& operator>>(std::basic_istream<CharT, Traits>& is,
                                              BitSet& x)
{
	// TODO: Implement
	return is;
}
}  // namespace ufo::map

namespace std
{
template <>
struct hash<ufo::map::BitSet> {
	std::size_t operator()(ufo::map::BitSet x) const { return x.field; }
};
}  // namespace std

#endif  // UFO_MAP_INDEX_FIELD_H