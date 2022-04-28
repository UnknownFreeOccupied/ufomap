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

#ifndef UFO_MAP_POINT_H
#define UFO_MAP_POINT_H

// UFO
#include <ufo/map/color/color.h>
#include <ufo/map/semantic/semantic.h>
#include <ufo/math/vector3.h>

// STL

namespace ufo::map
{
using Point3 = math::Vector3;

class Point3Color : public Point3, public Color
{
 public:
	Point3Color() = default;

	Point3Color(Point3 const& point, Color const& color) : Point3(point), Color(color) {}

	Point3Color(float x, float y, float z, uint8_t red, uint8_t green, uint8_t blue)
	    : Point3(x, y, z), Color(red, green, blue)
	{
	}

	Point3Color(Point3 const& point) : Point3(point) {}

	Point3Color(float x, float y, float z) : Point3(x, y, z) {}

	Point3Color(Color const& color) : Color(color) {}
};

class Point3Label : public Point3, public SemanticLabelC
{
 public:
	Point3Label() = default;

	Point3Label(Point3 const& point, SemanticLabel const& label)
	    : Point3(point), SemanticLabelC(label)
	{
	}

	Point3Label(float x, float y, float z, SemanticLabel label)
	    : Point3(x, y, z), SemanticLabelC(label)
	{
	}

	Point3Label(Point3 const& point) : Point3(point) {}

	Point3Label(float x, float y, float z) : Point3(x, y, z) {}

	Point3Label(SemanticLabel const& label) : SemanticLabelC(label) {}
};

class Point3Semantic : public Point3, public SemanticPair
{
 public:
	Point3Semantic() = default;

	Point3Semantic(Point3 const& point, SemanticPair const& semantic)
	    : Point3(point), SemanticPair(semantic)
	{
	}

	Point3Semantic(float x, float y, float z, SemanticLabel label, SemanticValue value)
	    : Point3(x, y, z), SemanticPair(label, value)
	{
	}

	Point3Semantic(Point3 const& point) : Point3(point) {}

	Point3Semantic(float x, float y, float z) : Point3(x, y, z) {}

	Point3Semantic(SemanticPair const& semantic) : SemanticPair(semantic) {}
};

class Point3ColorLabel : public Point3, public Color, public SemanticLabelC
{
 public:
	Point3ColorLabel() = default;

	Point3ColorLabel(Point3 const& point, Color const& color, SemanticLabel const& label)
	    : Point3(point), Color(color), SemanticLabelC(label)
	{
	}

	Point3ColorLabel(float x, float y, float z, ColorType red, ColorType green,
	                 ColorType blue, SemanticLabel label)
	    : Point3(x, y, z), Color(red, green, blue), SemanticLabelC(label)
	{
	}

	Point3ColorLabel(Point3 const& point) : Point3(point) {}

	Point3ColorLabel(float x, float y, float z) : Point3(x, y, z) {}

	Point3ColorLabel(Color const& color) : Color(color) {}

	Point3ColorLabel(SemanticLabel const& label) : SemanticLabelC(label) {}
};

class Point3ColorSemantic : public Point3, public Color, public SemanticPair
{
 public:
	Point3ColorSemantic() = default;

	Point3ColorSemantic(Point3 const& point, Color const& color,
	                    SemanticPair const& semantic)
	    : Point3(point), Color(color), SemanticPair(semantic)
	{
	}

	Point3ColorSemantic(float x, float y, float z, ColorType red, ColorType green,
	                    ColorType blue, SemanticLabel label, SemanticValue value)
	    : Point3(x, y, z), Color(red, green, blue), SemanticPair(label, value)
	{
	}

	Point3ColorSemantic(Point3 const& point) : Point3(point) {}

	Point3ColorSemantic(float x, float y, float z) : Point3(x, y, z) {}

	Point3ColorSemantic(Color const& color) : Color(color) {}

	Point3ColorSemantic(SemanticPair const& semantic) : SemanticPair(semantic) {}
};

}  // namespace ufo::map

#endif  // UFO_MAP_POINT_H