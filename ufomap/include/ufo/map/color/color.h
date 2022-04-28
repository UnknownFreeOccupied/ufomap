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

#ifndef UFO_MAP_COLOR_H
#define UFO_MAP_COLOR_H

// STL
#include <cmath>
#include <cstdint>
#include <iostream>
#include <utility>
#include <vector>

namespace ufo::map
{
using ColorType = uint8_t;

/**
 * @brief RGB color
 *
 */
class Color
{
 public:
	Color() : red_(0), green_(0), blue_(0) {}

	constexpr Color(ColorType red, ColorType green, ColorType blue)
	    : red_(red), green_(green), blue_(blue)
	{
	}

	bool operator==(Color const& other) const
	{
		return other.red() == red() && other.green() == green() && other.blue() == blue();
	}

	bool operator!=(Color const& other) const
	{
		return other.red() != red() || other.green() != green() || other.blue() != blue();
	}

	[[nodiscard]] bool isSet() const noexcept
	{
		return 0 != red() || 0 != green() || 0 != blue();
	}

	constexpr void clear() noexcept { red_ = green_ = blue_ = 0; }

	constexpr void setColor(Color const& color)
	{
		red_ = color.red_;
		green_ = color.green_;
		blue_ = color.blue_;
	}

	constexpr void setColor(ColorType red, ColorType green, ColorType blue)
	{
		red_ = red;
		green_ = green;
		blue_ = blue;
	}

	constexpr ColorType& red() noexcept { return red_; }
	constexpr ColorType& green() noexcept { return green_; }
	constexpr ColorType& blue() noexcept { return blue_; }

	constexpr ColorType red() const noexcept { return red_; }
	constexpr ColorType green() const noexcept { return green_; }
	constexpr ColorType blue() const noexcept { return blue_; }

	static Color averageColor(std::vector<Color> const& colors)
	{
		if (colors.empty()) {
			return {};
		} else if (1 == colors.size()) {
			return colors.front();
		}

		double r = 0.0;
		double g = 0.0;
		double b = 0.0;
		size_t total = 0;
		for (size_t i = 0; i != colors.size(); ++i) {
			if (!colors[i].isSet()) {
				continue;
			}

			++total;

			r += colors[i].red();
			g += colors[i].green();
			b += colors[i].blue();
		}
		return 0 == total ? Color() : Color(r / total, g / total, b / total);
	}

	static Color averageColor(std::vector<Color> const& colors,
	                          std::vector<double> const& weights)
	{
		if (colors.empty() || colors.size() != weights.size()) {
			return {};
		} else if (1 == colors.size()) {
			return colors.front();
		}

		double r = 0.0;
		double g = 0.0;
		double b = 0.0;
		double total_weight = 0.0;
		for (size_t i = 0; i != colors.size(); ++i) {
			if (!colors[i].isSet()) {
				continue;
			}

			total_weight += weights[i];

			r += colors[i].red() * weights[i];
			g += colors[i].green() * weights[i];
			b += colors[i].blue() * weights[i];
		}
		return 0.0 == total_weight
		           ? Color()
		           //  : Color(
		           // std::min(static_cast<double>(std::numeric_limits<ColorType>::max()),
		           //                 r / total_weight),
		           // std::min(static_cast<double>(std::numeric_limits<ColorType>::max()),
		           //                 g / total_weight),
		           // std::min(static_cast<double>(std::numeric_limits<ColorType>::max()),
		           //                 b / total_weight));
		           : Color(r / total_weight, g / total_weight, b / total_weight);
	}

	static Color averageColor2(std::vector<Color> const& colors)
	{
		if (colors.empty()) {
			return {};
		} else if (1 == colors.size()) {
			return colors.front();
		}

		double r = 0.0;
		double g = 0.0;
		double b = 0.0;
		size_t total = 0;
		for (size_t i = 0; i != colors.size(); ++i) {
			if (!colors[i].isSet()) {
				continue;
			}

			++total;

			r += colors[i].red() * colors[i].red();
			g += colors[i].green() * colors[i].green();
			b += colors[i].blue() * colors[i].blue();
		}
		return 0 == total
		           ? Color()
		           : Color(std::sqrt(r / total), std::sqrt(g / total), std::sqrt(b /
total));
	}

	static Color averageColor2(std::vector<Color> const& colors,
	                           std::vector<double> const& weights)
	{
		if (colors.empty() || colors.size() != weights.size()) {
			return {};
		} else if (1 == colors.size()) {
			return colors.front();
		}

		double r = 0.0;
		double g = 0.0;
		double b = 0.0;
		double total_weight = 0.0;
		for (size_t i = 0; i != colors.size(); ++i) {
			if (!colors[i].isSet()) {
				continue;
			}

			total_weight += weights[i];

			r += colors[i].red() * colors[i].red() * weights[i];
			g += colors[i].green() * colors[i].green() * weights[i];
			b += colors[i].blue() * colors[i].blue() * weights[i];
		}
		return 0.0 == total_weight
		           ? Color()
		           : Color(std::sqrt(r / total_weight), std::sqrt(g / total_weight),
		                   std::sqrt(b / total_weight));
	}

	std::ostream& writeData(std::ostream& out_stream) const
	{
		out_stream.write(reinterpret_cast<char const*>(&red_), sizeof(ColorType));
		out_stream.write(reinterpret_cast<char const*>(&green_), sizeof(ColorType));
		return out_stream.write(reinterpret_cast<char const*>(&blue_), sizeof(ColorType));
	}

	std::istream& readData(std::istream& in_stream)
	{
		in_stream.read(reinterpret_cast<char*>(&red_), sizeof(ColorType));
		in_stream.read(reinterpret_cast<char*>(&green_), sizeof(ColorType));
		return in_stream.read(reinterpret_cast<char*>(&blue_), sizeof(ColorType));
	}

	static constexpr size_t serializedSize() { return 3 * sizeof(ColorType); }

 protected:
	ColorType red_;
	ColorType green_;
	ColorType blue_;
};

// class Color
// {
//  public:
// 	Color() : color_(0) {}

// 	constexpr Color(ColorType red, ColorType green, ColorType blue)
// 	    : color_(red | (green << 8) | (blue << 16))
// 	{
// 	}

// 	bool operator==(Color const& other) const { return other.color_ == color_; }

// 	bool operator!=(Color const& other) const { return other.color_ != color_; }

// 	[[nodiscard]] bool isSet() const noexcept { return 0 != color_; }

// 	constexpr void clear() noexcept { color_ = 0; }

// 	constexpr void setColor(Color const& color) { color_ = color.color_; }

// 	constexpr void setColor(ColorType red, ColorType green, ColorType blue)
// 	{
// 		color_ = red | (green << 8) | (blue << 16);
// 	}

// 	constexpr ColorType red() const noexcept { return color_; }
// 	constexpr ColorType green() const noexcept { return color_ >> 8; }
// 	constexpr ColorType blue() const noexcept { return color_ >> 16; }

// 	static Color averageColor(std::vector<Color> const& colors)
// 	{
// 		if (colors.empty()) {
// 			return {};
// 		} else if (1 == colors.size()) {
// 			return colors.front();
// 		}

// 		double r = 0.0;
// 		double g = 0.0;
// 		double b = 0.0;
// 		size_t total = 0;
// 		for (size_t i = 0; i != colors.size(); ++i) {
// 			if (!colors[i].isSet()) {
// 				continue;
// 			}

// 			++total;

// 			r += colors[i].red();
// 			g += colors[i].green();
// 			b += colors[i].blue();
// 		}
// 		return 0 == total ? Color() : Color(r / total, g / total, b / total);
// 	}

// 	static Color averageColor(std::vector<Color> const& colors,
// 	                          std::vector<double> const& weights)
// 	{
// 		if (colors.empty() || colors.size() != weights.size()) {
// 			return {};
// 		} else if (1 == colors.size()) {
// 			return colors.front();
// 		}

// 		double r = 0.0;
// 		double g = 0.0;
// 		double b = 0.0;
// 		double total_weight = 0.0;
// 		for (size_t i = 0; i != colors.size(); ++i) {
// 			if (!colors[i].isSet()) {
// 				continue;
// 			}

// 			total_weight += weights[i];

// 			r += colors[i].red() * weights[i];
// 			g += colors[i].green() * weights[i];
// 			b += colors[i].blue() * weights[i];
// 		}
// 		return 0.0 == total_weight
// 		           ? Color()
// 		           //  : Color(
// 		           //        std::min(static_cast<double>(std::numeric_limits<ColorType>::max()),
// 		           //                 r / total_weight),
// 		           //        std::min(static_cast<double>(std::numeric_limits<ColorType>::max()),
// 		           //                 g / total_weight),
// 		           //        std::min(static_cast<double>(std::numeric_limits<ColorType>::max()),
// 		           //                 b / total_weight));
// 		           : Color(r / total_weight, g / total_weight, b / total_weight);
// 	}

// 	static Color averageColor2(std::vector<Color> const& colors)
// 	{
// 		if (colors.empty()) {
// 			return {};
// 		} else if (1 == colors.size()) {
// 			return colors.front();
// 		}

// 		double r = 0.0;
// 		double g = 0.0;
// 		double b = 0.0;
// 		size_t total = 0;
// 		for (size_t i = 0; i != colors.size(); ++i) {
// 			if (!colors[i].isSet()) {
// 				continue;
// 			}

// 			++total;

// 			r += colors[i].red() * colors[i].red();
// 			g += colors[i].green() * colors[i].green();
// 			b += colors[i].blue() * colors[i].blue();
// 		}
// 		return 0 == total
// 		           ? Color()
// 		           : Color(std::sqrt(r / total), std::sqrt(g / total), std::sqrt(b / total));
// 	}

// 	static Color averageColor2(std::vector<Color> const& colors,
// 	                           std::vector<double> const& weights)
// 	{
// 		if (colors.empty() || colors.size() != weights.size()) {
// 			return {};
// 		} else if (1 == colors.size()) {
// 			return colors.front();
// 		}

// 		double r = 0.0;
// 		double g = 0.0;
// 		double b = 0.0;
// 		double total_weight = 0.0;
// 		for (size_t i = 0; i != colors.size(); ++i) {
// 			if (!colors[i].isSet()) {
// 				continue;
// 			}

// 			total_weight += weights[i];

// 			r += colors[i].red() * colors[i].red() * weights[i];
// 			g += colors[i].green() * colors[i].green() * weights[i];
// 			b += colors[i].blue() * colors[i].blue() * weights[i];
// 		}
// 		return 0.0 == total_weight
// 		           ? Color()
// 		           : Color(std::sqrt(r / total_weight), std::sqrt(g / total_weight),
// 		                   std::sqrt(b / total_weight));
// 	}

// 	std::ostream& writeData(std::ostream& out_stream) const
// 	{
// 		return out_stream.write(reinterpret_cast<char const*>(&color_), sizeof(uint32_t));
// 	}

// 	std::istream& readData(std::istream& in_stream)
// 	{
// 		return in_stream.read(reinterpret_cast<char*>(&color_), sizeof(uint32_t));
// 	}

// 	static constexpr size_t serializedSize() { return sizeof(uint32_t); }

//  protected:
// 	uint32_t color_;
// };
}  // namespace ufo::map

#endif  // UFO_MAP_COLOR_H
