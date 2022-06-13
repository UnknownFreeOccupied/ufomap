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

#ifndef UFOMAP_RVIZ_PLUGINS_HEATMAP_H
#define UFOMAP_RVIZ_PLUGINS_HEATMAP_H

// UFO
#include <ufo/map/point.h>
#include <ufo/map/types.h>

struct Heatmap {
	// Position
	ufo::map::Point3 min_position;
	ufo::map::Point3 max_position;

	// Time step
	ufo::map::time_step_t  min_time_step;
	ufo::map::time_step_t  max_time_step;

	bool operator==(Heatmap const& rhs) const
	{
		return rhs.min_position == min_position && rhs.max_position == max_position &&
		       rhs.min_time_step == min_time_step && rhs.max_time_step == max_time_step;
	}

	bool operator!=(Heatmap const& rhs) const { return !(*this == rhs); }

	static Ogre::ColourValue getColor(double value, double min_value, double max_value,
	                                  double color_factor)
	{
		constexpr double s = 1;
		constexpr double v = 1;

		value = std::clamp((value - min_value) / (max_value - min_value), 0.0, 1.0);

		double h = (1 - value) * color_factor;
		h -= std::floor(h);
		h *= 6;

		int const i = std::floor(h);

		double const f = (i & 1) ? h - i : 1 - (h - i);

		double const m = v * (1 - s);

		double const n = v * (1 - s * f);

		switch (i) {
			case 6:
			case 0:
				return Ogre::ColourValue(v, n, m);
			case 1:
				return Ogre::ColourValue(n, v, m);
			case 2:
				return Ogre::ColourValue(m, v, n);
			case 3:
				return Ogre::ColourValue(m, n, v);
			case 4:
				return Ogre::ColourValue(n, m, v);
			case 5:
				return Ogre::ColourValue(v, m, n);
			default:
				return Ogre::ColourValue(1, 0.5, 0.5);
		}
	}
};

#endif  // UFOMAP_RVIZ_PLUGINS_HEATMAP_H