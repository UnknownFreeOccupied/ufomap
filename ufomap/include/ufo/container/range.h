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

#ifndef UFO_CONTAINER_RANGE_H
#define UFO_CONTAINER_RANGE_H

// STL
#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <iostream>
#include <iterator>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <vector>

/*
 * Range, RangeSet, and RangeMap implementation
 *
 * Functions present in the respective STL implementation adhere to their
 * interface (std::set, std::map).
 *
 * A Range takes an arithmetic type T and defines an inclusive interval [T1, T2]
 *
 * A RangeSet is a container of unique ranges. If a Range is inserted into the
 * container and is adjacent or overlapping with a Range already present the two
 * are joined: e.g., [4,10] and [11, 40] becomes [4, 40].
 *
 * A RangeMap is a container where each Range (key) has an associated value.
 * Adjacent or overlapping ranges with the same value are joined.
 * Ranges within the container are not allowed to overlap, e.g., if
 * the range and value [0,10]:4 is present and [5,15]:6 is inserted
 * the resulting container will contain: [0,10]:4, [11,15]:6
 * There is support for overwriting, see insert_or_assign
 */

namespace ufo::container
{
template <typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
class Range
{
 public:
	struct Comparator {
		using range_t = std::pair<T, T>;
		using is_transparent = std::true_type;
		[[nodiscard]] constexpr bool operator()(Range lhs, range_t rhs) const noexcept
		{
			return lhs.upper() < rhs.first;
		}
		[[nodiscard]] constexpr bool operator()(range_t lhs, Range rhs) const noexcept
		{
			return lhs.second < rhs.upper();
		}
	};

 public:
	// Tags
	using value_type = T;

	Range() = default;

	Range(T value) : lower_(value), upper_(value) {}

	Range(T lower, T upper) : lower_(std::min(lower, upper)), upper_(std::max(lower, upper))
	{
	}

	template <typename V, typename = std::enable_if_t<std::is_arithmetic_v<V>>>
	Range(V value) : Range(value, value)
	{
	}

	template <typename V, typename = std::enable_if_t<std::is_arithmetic_v<V>>>
	Range(V lower, V upper)
	{
		setRange(lower, upper);
	}

	Range(Range const &other) = default;

	template <typename T2>
	Range(Range<T2> other) : Range(other.lower(), other.upper())
	{
	}

	Range(Range &&other) = default;

	Range &operator=(Range const &rhs) = default;

	template <typename T2>
	Range &operator=(Range<T2> rhs)
	{
		setRange(rhs.lower(), rhs.upper());
		return *this;
	}

	Range &operator=(Range &&rhs) = default;

	constexpr void setValue(T value)
	{
		lower_ = value;
		upper_ = value;
	}

	constexpr void setRange(T lower, T upper)
	{
		std::tie(lower_, upper_) = std::minmax(lower, upper);
	}

	template <typename V, typename = std::enable_if_t<std::is_arithmetic_v<V>>>
	constexpr void setValue(V value)
	{
		setRange(value, value);
	}

	template <typename V, typename = std::enable_if_t<std::is_arithmetic_v<V>>>
	constexpr void setRange(V lower, V upper)
	{
		// FIXME: Check that this is correct
		auto [min, max] = std::minmax(lower, upper);
		auto t_min = std::numeric_limits<T>::lowest();
		auto t_max = std::numeric_limits<T>::max();

		// Check for error
		if constexpr (std::is_floating_point_v<T> || std::is_floating_point_v<V> ||
		              std::is_signed_v<T> == std::is_signed_v<V>) {
			if (max < t_min || t_max < min) {
				// FIXME: Fix better message
				throw std::range_error("Outside representable range.");
			}
		} else if constexpr (std::is_unsigned_v<T>) {
			if (min < 0 || t_max < min) {
				// FIXME: Fix better message
				throw std::range_error("Outside representable range.");
			}
		} else if (t_max < min) {
			// FIXME: Fix better message
			throw std::range_error("Outside representable range.");
		}

		if constexpr (std::is_floating_point_v<T>) {
			lower_ = t_min <= min ? min : t_min;
			upper_ = t_max >= max ? max : t_max;
		} else if constexpr (std::is_floating_point_v<V>) {
			auto v_min = std::floor(min);
			auto v_max = std::ceil(max);
			lower_ = t_min <= v_min ? v_min : t_min;
			upper_ = t_max >= v_max ? v_max : t_max;
		} else if constexpr (std::is_signed_v<T> == std::is_signed_v<V>) {
			lower_ = t_min <= min ? min : t_min;
			upper_ = t_max >= max ? max : t_max;
		} else if constexpr (std::is_unsigned_v<T>) {
			lower_ = 0 > min ? 0 : (t_max >= min ? min : t_max);
			upper_ = 0 > max ? 0 : (t_max >= max ? max : t_max);
		} else {
			lower_ = t_max >= min ? min : t_max;
			upper_ = t_max >= max ? max : t_max;
		}
	}

	[[nodiscard]] constexpr T lower() const noexcept { return lower_; }

	[[nodiscard]] constexpr T upper() const noexcept { return upper_; }

	[[nodiscard]] constexpr bool contains(Range range) const noexcept
	{
		return lower_ <= range.lower_ && range.upper_ <= upper_;
	}

	template <typename T2>
	[[nodiscard]] constexpr bool contains(Range<T2> range) const noexcept
	{
		return contains(range.lower(), range.upper());
	}

	template <typename V, typename = std::enable_if_t<std::is_arithmetic_v<V>>>
	[[nodiscard]] constexpr bool contains(V value) const noexcept
	{
		return contains(value, value);
	}

	template <typename V, typename = std::enable_if_t<std::is_arithmetic_v<V>>>
	[[nodiscard]] constexpr bool contains(V lower, V upper) const noexcept
	{
		// FIXME: Is correct?
		auto [min, max] = std::minmax(lower, upper);
		if constexpr (std::is_floating_point_v<T> || std::is_floating_point_v<V> ||
		              (std::is_signed_v<T> == std::is_signed_v<V>)) {
			return lower_ <= min && max <= upper_;
		} else {
			if (min < 0 || upper_ < 0 || max > upper_) {
				return false;
			}
			return lower_ < 0 || lower_ <= min;
		}
	}

	void swap(Range other)
	{
		std::swap(lower_, other.lower_);
		std::swap(upper_, other.upper_);
	}

	std::ostream &writeData(std::ostream &out_stream) const
	{
		out_stream.write(reinterpret_cast<char *>(&lower_), sizeof(T));
		return out_stream.write(reinterpret_cast<char *>(&upper_), sizeof(T));
	}

	std::istream &readData(std::istream &in_stream)
	{
		in_stream.read(reinterpret_cast<char *>(&lower_), sizeof(T));
		return in_stream.read(reinterpret_cast<char *>(&upper_), sizeof(T));
	}

	//
	// Friends
	//

	template <typename T2>
	friend bool operator==(Range<T2> lhs, Range<T2> rhs);
	template <typename T2>
	friend bool operator!=(Range<T2> lhs, Range<T2> rhs);
	template <typename T2>
	friend bool operator<(Range<T2> lhs, Range<T2> rhs);
	template <typename T2>
	friend bool operator<=(Range<T2> lhs, Range<T2> rhs);
	template <typename T2>
	friend bool operator>(Range<T2> lhs, Range<T2> rhs);
	template <typename T2>
	friend bool operator>=(Range<T2> lhs, Range<T2> rhs);

	template <typename T2>
	friend void swap(Range<T2> &lhs, Range<T2> &rhs) noexcept(noexcept(lhs.swap(rhs)));

	template <typename T2>
	friend std::ostream &operator<<(std::ostream &os, Range<T2> range);

 private:
	T lower_;
	T upper_;
};

template <typename T>
bool operator==(Range<T> lhs, Range<T> rhs)
{
	return lhs.lower_ == rhs.lower_ && lhs.upper_ == rhs.upper_;
}

template <typename T>
bool operator!=(Range<T> lhs, Range<T> rhs)
{
	return lhs.lower_ != rhs.lower_ || lhs.upper_ != rhs.upper_;
}

template <typename T>
bool operator<(Range<T> lhs, Range<T> rhs)
{
	return lhs.upper_ < rhs.lower_;
}

template <typename T>
bool operator<=(Range<T> lhs, Range<T> rhs)
{
	return lhs.upper_ <= rhs.lower_;
}

template <typename T>
bool operator>(Range<T> lhs, Range<T> rhs)
{
	return lhs.lower_ > rhs.upper_;
}

template <typename T>
bool operator>=(Range<T> lhs, Range<T> rhs)
{
	return lhs.lower_ >= rhs.upper_;
}

template <typename T>
void swap(Range<T> &lhs, Range<T> &rhs) noexcept(noexcept(lhs.swap(rhs)))
{
	lhs.swap(rhs);
}

template <typename T>
std::ostream &operator<<(std::ostream &os, Range<T> range)
{
	if (range.lower() == range.upper()) {
		os << '[' << +range.lower() << ']';
	} else if constexpr (std::is_floating_point_v<T>) {
		os << '[' << +range.lower() << ',' << +range.upper() << ']';
	} else {
		os << '[' << +range.lower() << ".." << +range.upper() << ']';
	}
	return os;
}
}  // namespace ufo::container

#endif  // UFO_CONTAINER_RANGE_H