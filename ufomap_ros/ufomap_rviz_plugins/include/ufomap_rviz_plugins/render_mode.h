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

#ifndef UFOMAP_RVIZ_PLUGINS_RENDER_MODE_H
#define UFOMAP_RVIZ_PLUGINS_RENDER_MODE_H

// UFO
#include <ufo/geometry/aabb.h>
#include <ufo/geometry/obb.h>
#include <ufo/map/color/color.h>
#include <ufo/map/point.h>
#include <ufo/map/types.h>

// Qt
#include <QString>

// Ogre
#include <OGRE/OgreColourValue.h>

// STL
#include <string>
#include <string_view>
#include <unordered_map>

namespace ufomap_ros::rviz_plugins
{
using VoxelType = ufo::map::OccupancyState;

enum class ColoringMode {
	VOXEL,
	TIME,
	SEMANTIC,
	SURFEL_NORMAL,
	X_AXIS,
	Y_AXIS,
	Z_AXIS,
	OCCUPANCY,
	INTENSITY,
	FIXED
};

enum class RenderStyle { POINTS, SQUARES, FLAT_SQUARES, SPHERES, TILES, BOXES, SURFEL };

static inline QString getStr(VoxelType const& type)
{
	using namespace std::literals;
	switch (type) {
		case VoxelType::UNKNOWN:
			return "Unknown";
		case VoxelType::FREE:
			return "Free";
		case VoxelType::OCCUPIED:
			return "Occupied";
		default:
			return "";
	}
}

static inline QString getStr(ColoringMode const& mode)
{
	using namespace std::literals;
	switch (mode) {
		case ColoringMode::VOXEL:
			return "Voxel";
		case ColoringMode::TIME:
			return "Time";
		case ColoringMode::SEMANTIC:
			return "Semantic";
		case ColoringMode::SURFEL_NORMAL:
			return "Surfel Normal";
		case ColoringMode::X_AXIS:
			return "X-Axis";
		case ColoringMode::Y_AXIS:
			return "Y-Axis";
		case ColoringMode::Z_AXIS:
			return "Z-Axis";
		case ColoringMode::OCCUPANCY:
			return "Occupancy";
		case ColoringMode::INTENSITY:
			return "Intensity";
		case ColoringMode::FIXED:
			return "Fixed";
		default:
			return "";
	}
}

static inline QString getStr(RenderStyle const& style)
{
	using namespace std::literals;
	switch (style) {
		case RenderStyle::POINTS:
			return "Points";
		case RenderStyle::SQUARES:
			return "Squares";
		case RenderStyle::FLAT_SQUARES:
			return "Flat Squares";
		case RenderStyle::SPHERES:
			return "Spheres";
		case RenderStyle::TILES:
			return "Tiles";
		case RenderStyle::BOXES:
			return "Boxes";
		case RenderStyle::SURFEL:
			return "Surfel";
		default:
			return "";
	}
}

struct RenderMode {
	RenderMode()
	    : style(RenderStyle::BOXES), coloring_mode(ColoringMode::VOXEL), color(0, 0, 0)
	{
	}

	// Should render
	bool should_render;

	// Style
	RenderStyle style;

	// Coloring
	ColoringMode coloring_mode;

	// Color
	Ogre::ColourValue color;

	// Color factor
	float color_factor = 0.6;

	// Normalized
	bool normalized_value = false;
	double normalized_min_change = 0.1;
	double min_normalized_value = 0.0;
	double max_normalized_value = 0.0;

	// Alpha
	float alpha = 1.0;

	// Scale
	float scale = 1.0;

	// Depth
	ufo::map::depth_t depth = 0;

	bool operator==(RenderMode const& rhs) const
	{
		return rhs.should_render == should_render && rhs.style == style &&
		       rhs.coloring_mode == coloring_mode && rhs.color == color &&
		       rhs.color_factor == color_factor && rhs.alpha == alpha && rhs.scale == scale &&
		       rhs.depth == depth;
	}

	bool operator!=(RenderMode const& rhs) const { return !(*this == rhs); }
};

}  // namespace ufomap_ros::rviz_plugins

#endif  // UFOMAP_RVIZ_PLUGINS_RENDER_MODE_H