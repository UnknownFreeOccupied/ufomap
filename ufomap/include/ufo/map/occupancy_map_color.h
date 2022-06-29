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

#ifndef UFO_MAP_OCCUPANCY_MAP_COLOR_H
#define UFO_MAP_OCCUPANCY_MAP_COLOR_H

// UFO
#include <ufo/map/color/color_map_base.h>
#include <ufo/map/occupancy/occupancy_map_base.h>
#include <ufo/map/occupancy/occupancy_node.h>
#include <ufo/map/octree/octree_base.h>

namespace ufo::map
{

template <class OccupancyType = float>
class OccupancyMapColor final
    : public OctreeBase<OccupancyMapColor<OccupancyType>,
                        OccupancyColorNode<OccupancyType>, OccupancyIndicators>,
      public OccupancyMapBase<OccupancyMapColor<OccupancyType>,
                              typename OctreeBase<OccupancyMapColor<OccupancyType>,
                                                  OccupancyColorNode<OccupancyType>,
                                                  OccupancyIndicators>::LeafNode,
                              typename OctreeBase<OccupancyMapColor<OccupancyType>,
                                                  OccupancyColorNode<OccupancyType>,
                                                  OccupancyIndicators>::InnerNode>,
      public ColorMapBase<OccupancyMapColor<OccupancyType>,
                          typename OctreeBase<OccupancyMapColor<OccupancyType>,
                                              OccupancyColorNode<OccupancyType>,
                                              OccupancyIndicators>::LeafNode,
                          typename OctreeBase<OccupancyMapColor<OccupancyType>,
                                              OccupancyColorNode<OccupancyType>,
                                              OccupancyIndicators>::InnerNode>
{
 protected:
	//
	// Tags
	//

	using OctreeBase = OctreeBase<OccupancyMapColor, OccupancyColorNode<OccupancyType>,
	                              OccupancyIndicators>;
	using LeafNode = typename OctreeBase::LeafNode;
	using InnerNode = typename OctreeBase::InnerNode;
	using OccupancyBase = OccupancyMapBase<OccupancyMapColor, LeafNode, InnerNode>;
	using ColorBase = ColorMapBase<OccupancyMapColor, LeafNode, InnerNode>;

	//
	// Friends
	//

	friend OctreeBase;
	friend OccupancyBase;
	friend ColorBase;

 public:
	//
	// Constructors
	//

	OccupancyMapColor(double resolution = 0.1, depth_t depth_levels = 16,
	                  bool automatic_pruning = true, float occupied_thres = 0.5,
	                  float free_thres = 0.5, float clamping_thres_min = 0.1192,
	                  float clamping_thres_max = 0.971)
	    : OctreeBase(resolution, depth_levels, automatic_pruning),
	      OccupancyBase(occupied_thres, free_thres, clamping_thres_min, clamping_thres_max),
	      ColorBase()
	{
		initRoot();
	}

	OccupancyMapColor(std::filesystem::path const& filename, bool automatic_pruning = true,
	                  float occupied_thres = 0.5, float free_thres = 0.5,
	                  float clamping_thres_min = 0.1192, float clamping_thres_max = 0.971)
	    : OccupancyMapColor(0.1, 16, automatic_pruning, occupied_thres, free_thres,
	                        clamping_thres_min, clamping_thres_max)
	{
		OctreeBase::read(filename);
	}

	OccupancyMapColor(std::istream& in_stream, bool automatic_pruning = true,
	                  float occupied_thres = 0.5, float free_thres = 0.5,
	                  float clamping_thres_min = 0.1192, float clamping_thres_max = 0.971)
	    : OccupancyMapColor(0.1, 16, automatic_pruning, occupied_thres, free_thres,
	                        clamping_thres_min, clamping_thres_max)
	{
		OctreeBase::read(in_stream);
	}

	OccupancyMapColor(OccupancyMapColor const& other)
	    : OctreeBase(other), OccupancyBase(other), ColorBase(other)
	{
		initRoot();

		std::stringstream io_stream(std::ios_base::in | std::ios_base::out |
		                            std::ios_base::binary);
		other.write(io_stream);
		OctreeBase::read(io_stream);
	}

	OccupancyMapColor(OccupancyMapColor&& other) = default;

	OccupancyMapColor& operator=(OccupancyMapColor const& rhs)
	{
		OctreeBase::operator=(rhs);
		OccupancyBase::operator=(rhs);
		ColorBase::operator=(rhs);

		initRoot();

		std::stringstream io_stream(std::ios_base::in | std::ios_base::out |
		                            std::ios_base::binary);
		rhs.write(io_stream);
		OctreeBase::read(io_stream);

		return *this;
	}

	OccupancyMapColor& operator=(OccupancyMapColor&& rhs) = default;

	//
	// Initilize root
	//

	void initRoot()
	{
		OctreeBase::initRoot();
		OccupancyBase::initRoot();
		ColorBase::initRoot();
	}

	//
	// Update node
	//

	void updateNode(InnerNode& node, depth_t depth)
	{
		OccupancyBase::updateNode(node, depth);
		ColorBase::updateNode(node, depth);
	}

	//
	// Input/output (read/write)
	//

	void addFileInfo(FileInfo& info) const
	{
		OccupancyBase::addFileInfo(info);
		ColorBase::addFileInfo(info);
	}

	bool readNodes(std::istream& in_stream, std::vector<LeafNode*> const& nodes,
	               std::string const& field, char type, uint64_t size, uint64_t num)
	{
		return OccupancyBase::readNodes(in_stream, nodes, field, type, size, num) ||
		       ColorBase::readNodes(in_stream, nodes, field, type, size, num);
	}

	void writeNodes(std::ostream& out_stream, std::vector<LeafNode const*> const& nodes,
	                bool compress, int compression_acceleration_level,
	                int compression_level) const
	{
		OccupancyBase::writeNodes(out_stream, nodes, compress, compression_acceleration_level,
		                          compression_level);
		ColorBase::writeNodes(out_stream, nodes, compress, compression_acceleration_level,
		                      compression_level);
	}
};

using OccupancyMapColorSmall = OccupancyMapColor<uint8_t>;
}  // namespace ufo::map

#endif  // UFO_MAP_OCCUPANCY_MAP_COLOR_H