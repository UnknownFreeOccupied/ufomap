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

#ifndef UFO_MAP_OCCUPANCY_MAP_H
#define UFO_MAP_OCCUPANCY_MAP_H

#include <ufo/map/occupancy_map_base.h>

namespace ufo::map
{
class OccupancyMap : public OccupancyMapBase<OccupancyNode<float>>
{
 private:
	using DATA_TYPE = OccupancyNode<float>;
	using Base = OccupancyMapBase<DATA_TYPE>;

 public:
	//
	// Constructors
	//

	OccupancyMap(double resolution, DepthType depth_levels = 16,
	             bool automatic_pruning = true, double occupied_thres = 0.5,
	             double free_thres = 0.5, double prob_hit = 0.7, double prob_miss = 0.4,
	             double clamping_thres_min = 0.1192, double clamping_thres_max = 0.971);

	OccupancyMap(std::string const& filename, bool automatic_pruning = true,
	                 double occupied_thres = 0.5, double free_thres = 0.5,
	                 double prob_hit = 0.7, double prob_miss = 0.4,
	                 double clamping_thres_min = 0.1192, double clamping_thres_max = 0.971);

	OccupancyMap(OccupancyMap const& other);

	//
	// Destructor
	//

	virtual ~OccupancyMap() {}

	//
	// Tree Type
	//

	virtual std::string getTreeType() const noexcept override { return "occupancy_map"; }
};
}  // namespace ufo::map

#endif  // UFO_MAP_OCCUPANCY_MAP_H