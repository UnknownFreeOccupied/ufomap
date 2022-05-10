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

#ifndef UFO_MAP_OCCUPANCY_MAP_TIME_COLOR_H
#define UFO_MAP_OCCUPANCY_MAP_TIME_COLOR_H

// UFO
#include <ufo/map/color/color_map_base.h>
#include <ufo/map/occupancy/occupancy_map_time_base.h>
#include <ufo/map/occupancy/occupancy_node.h>

namespace ufo::map
{
class OccupancyMapTimeColor final
    : public OccupancyMapTimeBase<OccupancyMapTimeColor, OccupancyTimeColorNode>,
      public ColorMapBase<OccupancyMapTimeColor, OccupancyTimeColorNode,
                          OccupancyIndicators>
{
 protected:
	using OccupancyTimeBase =
	    OccupancyMapTimeBase<OccupancyMapTimeColor, OccupancyTimeColorNode>;
	using ColorBase =
	    ColorMapBase<OccupancyMapTimeColor, OccupancyTimeColorNode, OccupancyIndicators>;
	using OccupancyBase = typename OccupancyTimeBase::OccupancyBase;
	using OctreeBase = typename OccupancyTimeBase::OctreeBase;
	using LeafNode = typename OctreeBase::LeafNode;
	using InnerNode = typename OctreeBase::InnerNode;

 public:
	//
	// Constructors
	//

	// using OccupancyTimeBase::OccupancyTimeBase;

	OccupancyMapTimeColor(float resolution, Depth depth_levels = 16,
	                      bool automatic_pruning = true, float occupied_thres = 0.5,
	                      float free_thres = 0.5, float clamping_thres_min = 0.1192,
	                      float clamping_thres_max = 0.971)
	    : OctreeBase(resolution, depth_levels, automatic_pruning),
	      OccupancyTimeBase(resolution, depth_levels, automatic_pruning, occupied_thres,
	                        free_thres, clamping_thres_min, clamping_thres_max),
	      ColorBase(resolution, depth_levels, automatic_pruning)
	{
		printf("OccupancyMapTimeColor constructor\n");

		// TODO: Implement
		initRoot();
	}

	OccupancyMapTimeColor(std::filesystem::path const& filename,
	                      bool automatic_pruning = true, float occupied_thres = 0.5,
	                      float free_thres = 0.5, float clamping_thres_min = 0.1192,
	                      float clamping_thres_max = 0.971)
	    : OccupancyMapTimeColor(0.1, 16, automatic_pruning, occupied_thres, free_thres,
	                            clamping_thres_min, clamping_thres_max)
	{
		printf("OccupancyMapTimeColor constructor\n");

		// TODO: Implement
		OctreeBase::read(filename);
		// TODO: Throw if cannot read
	}

	OccupancyMapTimeColor(std::istream& in_stream, bool automatic_pruning = true,
	                      float occupied_thres = 0.5, float free_thres = 0.5,
	                      float clamping_thres_min = 0.1192,
	                      float clamping_thres_max = 0.971)
	    : OccupancyMapTimeColor(0.1, 16, automatic_pruning, occupied_thres, free_thres,
	                            clamping_thres_min, clamping_thres_max)
	{
		printf("OccupancyMapTimeColor constructor\n");

		// TODO: Implement
		OctreeBase::read(in_stream);
		// TODO: Throw if cannot read
	}

	OccupancyMapTimeColor(OccupancyMapTimeColor const& other)
	    : OctreeBase(other), OccupancyTimeBase(other), ColorBase(other)
	{
		printf("OccupancyMapTimeColor copy constructor\n");

		// TODO: Implement
		initRoot();
		std::stringstream io_stream(std::ios_base::in | std::ios_base::out |
		                            std::ios_base::binary);
		other.write(io_stream);
		OctreeBase::read(io_stream);
	}

	OccupancyMapTimeColor(OccupancyMapTimeColor&& other)
	    : OctreeBase(std::move(other)),
	      OccupancyTimeBase(std::move(other)),
	      ColorBase(std::move(other))
	{
		printf("OccupancyMapTimeColor move constructor\n");
		// TODO: Implement
	}

	OccupancyMapTimeColor& operator=(OccupancyMapTimeColor const& rhs)
	{
		printf("OccupancyMapTimeColor copy assignment\n");

		// TODO: Implement
		OctreeBase::operator=(rhs);
		OccupancyTimeBase::operator=(rhs);
		ColorBase::operator=(rhs);

		initRoot();
		std::stringstream io_stream(std::ios_base::in | std::ios_base::out |
		                            std::ios_base::binary);
		rhs.write(io_stream);
		OctreeBase::read(io_stream);

		return *this;
	}

	OccupancyMapTimeColor& operator=(OccupancyMapTimeColor&& rhs)
	{
		printf("OccupancyMapTimeColor move assignment\n");

		// TODO: Implement
		OctreeBase::operator=(std::move(rhs));
		OccupancyTimeBase::operator=(std::move(rhs));
		ColorBase::operator=(std::move(rhs));
		return *this;
	}

	//
	// Destructor
	//

	virtual ~OccupancyMapTimeColor() override
	{
		printf("OccupancyMapTimeColor destructor\n");
	}

	//
	// Initilize root
	//

	virtual void initRoot() override
	{
		printf("OccupancyMapTimeColor initRoot\n");
		OccupancyTimeBase::initRoot();
		ColorBase::initRoot();
	}

	virtual void updateNode(InnerNode& node, Depth depth) override
	{
		OccupancyTimeBase::updateNode(node, depth);
		ColorBase::updateNode(node, depth);
	}

	//
	// Get map type
	//

	static constexpr std::string_view mapType() { return "occupancy_map_time_color"; }

	//
	// Input/output (read/write)
	//

	virtual void addFileInfo(FileInfo& info) const override
	{
		OccupancyTimeBase::addFileInfo(info);
		ColorBase::addFileInfo(info);
		info["map_type"].emplace_back(mapType());
	}

	virtual bool readNodes(std::istream& in_stream, std::vector<LeafNode*> const& nodes,
	                       std::string const& field, char type, uint64_t size,
	                       uint64_t num) override
	{
		return OccupancyBase::readNodes(in_stream, nodes, field, type, size, num) ||
		       OccupancyTimeBase::readNodes(in_stream, nodes, field, type, size, num) ||
		       ColorBase::readNodes(in_stream, nodes, field, type, size, num);
	}

	virtual void writeNodes(std::ostream& out_stream,
	                        std::vector<LeafNode const*> const& nodes, bool compress,
	                        int compression_acceleration_level,
	                        int compression_level) const override
	{
		OccupancyBase::writeNodes(out_stream, nodes, compress, compression_acceleration_level,
		                          compression_level);
		OccupancyTimeBase::writeNodes(out_stream, nodes, compress,
		                              compression_acceleration_level, compression_level);
		ColorBase::writeNodes(out_stream, nodes, compress, compression_acceleration_level,
		                      compression_level);
	}
};
}  // namespace ufo::map

#endif  // UFO_MAP_OCCUPANCY_MAP_TIME_COLOR_H