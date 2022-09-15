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

#ifndef UFO_MAP_POINT_H
#define UFO_MAP_POINT_H

// UFO
#include <ufo/map/color/color.h>
#include <ufo/map/semantic/semantic.h>
#include <ufo/map/types.h>
#include <ufo/math/vector3.h>

namespace ufo::map
{
using Point = math::Vector3<coord_t>;

// TODO: What to do with this?
struct Intensity {
	intensity_t intensity = 0;

	constexpr Intensity() = default;

	constexpr Intensity(intensity_t intensity) : intensity(intensity) {}
};

struct PointColor : Point, Color {
	constexpr PointColor() = default;

	constexpr PointColor(Point point, Color const& color = Color())
	    : Point(point), Color(color)
	{
	}

	constexpr PointColor(Point point, color_t red, color_t green, color_t blue)
	    : Point(point), Color(red, green, blue)
	{
	}

	constexpr PointColor(coord_t x, coord_t y, coord_t z, Color const& color = Color())
	    : Point(x, y, z), Color(color)
	{
	}

	constexpr PointColor(coord_t x, coord_t y, coord_t z, color_t red, color_t green,
	                     color_t blue)
	    : Point(x, y, z), Color(red, green, blue)
	{
	}
};

struct PointSemantic : Point, Semantic {
	constexpr PointSemantic() = default;

	constexpr PointSemantic(Point point, Semantic const& semantic = Semantic())
	    : Point(point), Semantic(semantic)
	{
	}

	constexpr PointSemantic(Point point, semantic_label_t label = 0,
	                        semantic_value_t value = 0)
	    : Point(point), Semantic(label, value)
	{
	}

	constexpr PointSemantic(coord_t x, coord_t y, coord_t z,
	                        Semantic const& semantic = Semantic())
	    : Point(x, y, z), Semantic(semantic)
	{
	}

	constexpr PointSemantic(coord_t x, coord_t y, coord_t z, semantic_label_t label = 0,
	                        semantic_value_t value = 0)
	    : Point(x, y, z), Semantic(label, value)
	{
	}
};

struct PointIntensity : Point, Intensity {
	constexpr PointIntensity() = default;

	constexpr PointIntensity(Point point, intensity_t intensity = 0)
	    : Point(point), Intensity(intensity)
	{
	}
};

struct PointColorSemantic : PointColor, Semantic {
	constexpr PointColorSemantic() = default;

	constexpr PointColorSemantic(Point point, Color const& color = Color(),
	                             Semantic const& semantic = Semantic())
	    : PointColor(point, color), Semantic(semantic)
	{
	}

	constexpr PointColorSemantic(Point point, color_t red, color_t green, color_t blue,
	                             semantic_label_t label = 0, semantic_value_t value = 0)
	    : PointColor(point, red, green, blue), Semantic(label, value)
	{
	}

	constexpr PointColorSemantic(coord_t x, coord_t y, coord_t z,
	                             Color const& color = Color(),
	                             Semantic const& semantic = Semantic())
	    : PointColor(x, y, z, color), Semantic(semantic)
	{
	}

	constexpr PointColorSemantic(coord_t x, coord_t y, coord_t z, color_t red,
	                             color_t green, color_t blue, semantic_label_t label = 0,
	                             semantic_value_t value = 0)
	    : PointColor(x, y, z, red, green, blue), Semantic(label, value)
	{
	}
};

struct PointColorIntensity : PointColor, Intensity {
	constexpr PointColorIntensity() = default;

	constexpr PointColorIntensity(Point point, Color const& color = Color(),
	                              intensity_t intensity = 0)
	    : PointColor(point, color), Intensity(intensity)
	{
	}

	constexpr PointColorIntensity(Point point, color_t red, color_t green, color_t blue,
	                              intensity_t intensity = 0)
	    : PointColor(point, red, green, blue), Intensity(intensity)
	{
	}

	constexpr PointColorIntensity(coord_t x, coord_t y, coord_t z,
	                              Color const& color = Color(), intensity_t intensity = 0)
	    : PointColor(x, y, z, color), Intensity(intensity)
	{
	}

	constexpr PointColorIntensity(coord_t x, coord_t y, coord_t z, color_t red,
	                              color_t green, color_t blue, intensity_t intensity = 0)
	    : PointColor(x, y, z, red, green, blue), Intensity(intensity)
	{
	}
};

struct PointColorSemanticIntensity : PointColorSemantic, Intensity {
	constexpr PointColorSemanticIntensity() = default;

	constexpr PointColorSemanticIntensity(Point point, Color const& color = Color(),
	                                      Semantic const& semantic = Semantic(),
	                                      intensity_t intensity = 0)
	    : PointColor(point, color), Semantic(semantic), Intensity(intensity)
	{
	}

	constexpr PointColorSemanticIntensity(Point point, color_t red, color_t green,
	                                      color_t blue, semantic_label_t label = 0,
	                                      semantic_value_t value = 0,
	                                      intensity_t intensity = 0)
	    : PointColor(point, red, green, blue), Semantic(label, value), Intensity(intensity)
	{
	}

	constexpr PointColorSemanticIntensity(coord_t x, coord_t y, coord_t z,
	                                      Color const& color = Color(),
	                                      Semantic const& semantic = Semantic(),
	                                      intensity_t intensity = 0)
	    : PointColor(x, y, z, color), Semantic(semantic), Intensity(intensity)
	{
	}

	constexpr PointColorSemanticIntensity(coord_t x, coord_t y, coord_t z, color_t red,
	                                      color_t green, color_t blue,
	                                      semantic_label_t label = 0,
	                                      semantic_value_t value = 0,
	                                      intensity_t intensity = 0)
	    : PointColor(x, y, z, red, green, blue),
	      Semantic(label, value),
	      Intensity(intensity)
	{
	}
};

}  // namespace ufo::map

#endif  // UFO_MAP_POINT_H