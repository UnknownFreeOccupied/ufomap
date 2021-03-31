/**
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * License: BSD 3
 *
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

#include <ufo/map/occupancy_map_color.h>

namespace ufo::map
{
OccupancyMapColor::OccupancyMapColor(double resolution, DepthType depth_levels,
                                     bool automatic_pruning, double occupied_thres,
                                     double free_thres, double prob_hit, double prob_miss,
                                     double clamping_thres_min, double clamping_thres_max)
    : OccupancyMapBase(resolution, depth_levels, automatic_pruning, occupied_thres,
                       free_thres, prob_hit, prob_miss, clamping_thres_min,
                       clamping_thres_max)
{
}

OccupancyMapColor::OccupancyMapColor(std::string const& filename, bool automatic_pruning,
                                     double occupied_thres, double free_thres,
                                     double prob_hit, double prob_miss,
                                     double clamping_thres_min, double clamping_thres_max)
    : OccupancyMapBase(filename, automatic_pruning, occupied_thres, free_thres, prob_hit,
                       prob_miss, clamping_thres_min, clamping_thres_max)
{
}

OccupancyMapColor::OccupancyMapColor(OccupancyMapColor const& other)
    : OccupancyMapBase(other)
{
}

//
// Set color
//

void OccupancyMapColor::setColor(Code const& code, Color color)
{
	auto path = Base::createNode(code);
	DepthType depth = code.getDepth();
	path[depth]->value.color = color;

	Base::updateParents(path, depth);
}

//
// Get color
//

Color OccupancyMapColor::getColor(Code const& code) const
{
	return Base::getNode(code).first->value.color;
}

//
// Integrate colors
//

void OccupancyMapColor::integrateColors(Point3 const& sensor_origin,
                                        PointCloudColor const& cloud, double max_range)
{
	CodeMap<std::vector<Color>> colors;
	for (Point3Color const& point : cloud) {
		if (0 > max_range || (point - sensor_origin).norm() < max_range) {
			colors[Base::toCode(point)].push_back(point.getColor());
		}
	}

	for (auto const& [code, color] : colors) {
		updateNodeColor(code, getAverageColor(color));
	}
}

//
// Update node
//

bool OccupancyMapColor::updateNode(INNER_NODE& node, DepthType depth)
{
	Color new_color = getAverageChildColor(node, depth);
	bool changed = Base::updateNode(node, depth);
	changed = changed || (node.value.color != new_color);
	node.value.color = new_color;
	return changed;
}

//
// Update node color
//

void OccupancyMapColor::updateNodeColor(Code code, Color update)
{
	if (!update.isSet()) {
		return;
	}

	auto path = Base::createNode(code);
	DepthType depth = code.getDepth();

	updateNodeColor(*path[depth], update, 1.0 - Base::getOccupancy(*path[depth]));

	Base::updateParents(path, depth);
}

void OccupancyMapColor::updateNodeColor(LEAF_NODE& node, Color update, double prob)
{
	Color& current = node.value.color;

	if (current == update) {
		return;
	}

	if (!current.isSet()) {
		current = update;
	} else {
		double total_prob = prob + Base::getOccupancy(node);
		prob /= total_prob;

		// double prob = std::max(0.0, std::min(2.0 * (Base::getOccupancy(node) - 0.5), 0.9));
		double prob_inv = 1.0 - prob;

		double c_r = static_cast<double>(current.r);
		double c_g = static_cast<double>(current.g);
		double c_b = static_cast<double>(current.b);

		double u_r = static_cast<double>(update.r);
		double u_g = static_cast<double>(update.g);
		double u_b = static_cast<double>(update.b);

		current.r = std::sqrt(((c_r * c_r) * prob_inv) + ((u_r * u_r) * prob));
		current.g = std::sqrt(((c_g * c_g) * prob_inv) + ((u_g * u_g) * prob));
		current.b = std::sqrt(((c_b * c_b) * prob_inv) + ((u_b * u_b) * prob));
	}
}

//
// Average child color
//

Color OccupancyMapColor::getAverageChildColor(INNER_NODE const& node,
                                              DepthType depth) const
{
	if (!hasChildren(node)) {
		return node.value.color;
	}

	std::vector<Color> colors;

	for (int i = 0; i < 8; ++i) {
		LEAF_NODE& child = getChild(node, depth - 1, i);
		if (child.value.color.isSet()) {
			colors.push_back(child.value.color);
		}
	}

	return getAverageColor(colors);
}

//
// Average color
//

Color OccupancyMapColor::getAverageColor(std::vector<Color> const& colors) const
{
	if (colors.empty()) {
		return Color();
	}

	// TODO: Update to LAB space?
	double r = 0;
	double g = 0;
	double b = 0;
	for (Color const& color : colors) {
		double color_r = static_cast<double>(color.r);
		double color_g = static_cast<double>(color.g);
		double color_b = static_cast<double>(color.b);

		r += (color_r * color_r);
		g += (color_g * color_g);
		b += (color_b * color_b);
	}
	double num_colors = static_cast<double>(colors.size());
	return Color(std::sqrt(r / num_colors), std::sqrt(g / num_colors),
	             std::sqrt(b / num_colors));
}
}  // namespace ufo::map