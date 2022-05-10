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

#ifndef UFO_MAP_OCCUPANCY_MAP_TIME_BASE_H
#define UFO_MAP_OCCUPANCY_MAP_TIME_BASE_H

// UFO
#include <ufo/map/occupancy/occupancy_map_base.h>
#include <ufo/map/occupancy/occupancy_node.h>
#include <ufo/map/predicate/time.h>

// STL
#include <chrono>
#include <type_traits>

namespace ufo::map
{
template <class Derived, class DataType, class Indicators = OccupancyIndicators>
class OccupancyMapTimeBase : public OccupancyMapBase<Derived, DataType, Indicators>
{
	static constexpr bool PropagateMax = true;  // FIXME: Fix

 protected:
	using OccupancyBase = OccupancyMapBase<Derived, DataType, Indicators>;
	using OctreeBase = typename OccupancyBase::Base;
	using LeafNode = typename OctreeBase::LeafNode;
	using InnerNode = typename OctreeBase::InnerNode;

	static_assert(std::is_base_of_v<OccupancyTimeNode, LeafNode>);

 public:
	using TimeType = std::chrono::time_point<std::chrono::system_clock>;

 public:
	constexpr bool propagateTimeMax() const noexcept { return PropagateMax; }

	//
	// Get time step
	//

	constexpr TimeStepType getTimeStep(Node const& node) const noexcept
	{
		return OctreeBase::getLeafNode(node).time_step;
	}

	TimeStepType getTimeStep(Code code) const
	{
		return OctreeBase::getLeafNode(code).time_step;
	}

	TimeStepType getTimeStep(Key key) const { return getTimeStep(OctreeBase::toCode(key)); }

	TimeStepType getTimeStep(Point3 coord, Depth depth = 0) const
	{
		return getTimeStep(OctreeBase::toCode(coord, depth));
	}

	TimeStepType getTimeStep(Coord x, Coord y, Coord z, Depth depth = 0) const
	{
		return getTimeStep(OctreeBase::toCode(x, y, z, depth));
	}

	//
	// Set time step
	//

	void setTimeStep(Node& node, TimeStepType time_step, bool propagate = true)
	{
		OctreeBase::apply(
		    node, [this, time_step](DataType& node) { setTimeStepImpl(node, time_step); },
		    propagate);
	}

	void setTimeStep(Code code, TimeStepType time_step, bool propagate = true)
	{
		OctreeBase::apply(
		    code, [this, time_step](DataType& node) { setTimeStepImpl(node, time_step); },
		    propagate);
	}

	void setTimeStep(Key key, TimeStepType time_step, bool propagate = true)
	{
		setTimeStep(OctreeBase::toCode(key), time_step, propagate);
	}

	void setTimeStep(Point3 coord, TimeStepType time_step, bool propagate = true,
	                 Depth depth = 0)
	{
		setTimeStep(OctreeBase::toCode(coord, depth), time_step, propagate);
	}

	void setTimeStep(Coord x, Coord y, Coord z, TimeStepType time_step,
	                 bool propagate = true, Depth depth = 0)
	{
		setTimeStep(OctreeBase::toCode(x, y, z, depth), time_step, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setTimeStep(ExecutionPolicy policy, Node& node, TimeStepType time_step,
	                 bool propagate = true)
	{
		OctreeBase::apply(
		    policy, node,
		    [this, time_step](DataType& node) { setTimeStepImpl(node, time_step); },
		    propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setTimeStep(ExecutionPolicy policy, Code code, TimeStepType time_step,
	                 bool propagate = true)
	{
		OctreeBase::apply(
		    policy, code,
		    [this, time_step](DataType& node) { setTimeStepImpl(node, time_step); },
		    propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setTimeStep(ExecutionPolicy policy, Key key, TimeStepType time_step,
	                 bool propagate = true)
	{
		setTimeStep(policy, OctreeBase::toCode(key), time_step, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setTimeStep(ExecutionPolicy policy, Point3 coord, TimeStepType time_step,
	                 bool propagate = true, Depth depth = 0)
	{
		setTimeStep(policy, OctreeBase::toCode(coord, depth), time_step, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setTimeStep(ExecutionPolicy policy, Coord x, Coord y, Coord z,
	                 TimeStepType time_step, bool propagate = true, Depth depth = 0)
	{
		setTimeStep(policy, OctreeBase::toCode(x, y, z, depth), time_step, propagate);
	}

 protected:
	//
	// Constructors
	//

	// using OccupancyBase::OccupancyBase

	OccupancyMapTimeBase(float resolution, Depth depth_levels, bool automatic_pruning,
	                     float occupied_thres, float free_thres, float clamping_thres_min,
	                     float clamping_thres_max)
	    : OctreeBase(resolution, depth_levels, automatic_pruning),
	      OccupancyBase(resolution, depth_levels, automatic_pruning, occupied_thres,
	                    free_thres, clamping_thres_min, clamping_thres_max)
	{
		printf("OccupancyMapTimeBase constructor\n");
		// TODO: Implement?
	}

	OccupancyMapTimeBase(OccupancyMapTimeBase const& other)
	    : OctreeBase(other), OccupancyBase(other)
	{
		printf("OccupancyMapTimeBase copy constructor\n");
		// TODO: Implement?
	}

	OccupancyMapTimeBase(OccupancyMapTimeBase&& other)
	    : OctreeBase(std::move(other)), OccupancyBase(std::move(other))
	{
		printf("OccupancyMapTimeBase move constructor\n");
		// TODO: Implement?
	}

	OccupancyMapTimeBase& operator=(OccupancyMapTimeBase const& rhs)
	{
		printf("OccupancyMapTimeBase copy assignment\n");
		OctreeBase::operator=(rhs);
		OccupancyBase::operator=(rhs);
		return *this;
	}

	OccupancyMapTimeBase& operator=(OccupancyMapTimeBase&& rhs)
	{
		// TODO: Implement
		printf("OccupancyMapTimeBase move assignment\n");
		OctreeBase::operator=(std::move(rhs));
		OccupancyBase::operator=(std::move(rhs));
		return *this;
	}

	//
	// Destructor
	//

	virtual ~OccupancyMapTimeBase() override {}

	//
	// Initilize root
	//

	virtual void initRoot() override
	{
		printf("OccupancyMapTimeBase initRoot\n");
		OccupancyBase::initRoot();
		OctreeBase::getRootImpl().time_step = 0;
	}

	//
	// Set time step
	//

	static constexpr void setTimeStepImpl(DataType& node, TimeStepType time_step)
	{
		node.time_step = time_step;
	}

	//
	// Update node
	//

	virtual void updateNode(InnerNode& node, Depth depth) override
	{
		OccupancyBase::updateNode(node, depth);

		if constexpr (PropagateMax) {
			node.time_step = 0;
		} else {
			node.time_step = -1;
		}

		if (1 == depth) {
			for (auto const& child : OctreeBase::getLeafChildren(node)) {
				if constexpr (PropagateMax) {
					node.time_step = std::max(node.time_step, child.time_step);
				} else {
					node.time_step = std::min(node.time_step, child.time_step);
				}
			}
		} else {
			for (auto const& child : OctreeBase::getInnerChildren(node)) {
				if constexpr (PropagateMax) {
					node.time_step = std::max(node.time_step, child.time_step);
				} else {
					node.time_step = std::min(node.time_step, child.time_step);
				}
			}
		}
	}

	//
	// Input/output (read/write)
	//

	virtual void addFileInfo(FileInfo& info) const override
	{
		OccupancyBase::addFileInfo(info);
		info["fields"].emplace_back("time_step");
		info["type"].emplace_back("U");
		info["size"].emplace_back("4");
	}

	virtual bool readNodes(std::istream& in_stream, std::vector<LeafNode*> const& nodes,
	                       std::string const& field, char type, uint64_t size,
	                       uint64_t num) override
	{
		if ("time_step" != field) {
			return false;
		}
		// FIXME: Make parallel

		if ('U' == type && 4 == size) {
			std::vector<uint32_t> data(nodes.size());
			in_stream.read(reinterpret_cast<char*>(data.data()),
			               data.size() * sizeof(typename decltype(data)::value_type));

			for (size_t i = 0; i != data.size(); ++i) {
				nodes[i]->time_step = data[i];
			}
		} else {
			// FIXME: Error
			return false;
		}
		return true;
	}

	virtual void writeNodes(std::ostream& out_stream,
	                        std::vector<LeafNode const*> const& nodes, bool compress,
	                        int compression_acceleration_level,
	                        int compression_level) const override
	{
		std::vector<uint32_t> data(nodes.size());
		for (size_t i = 0; i != nodes.size(); ++i) {
			data[i] = nodes[i]->time_step;
		}

		uint64_t size = data.size();
		out_stream.write(reinterpret_cast<char*>(&size), sizeof(size));
		out_stream.write(reinterpret_cast<char const*>(data.data()),
		                 data.size() * sizeof(typename decltype(data)::value_type));
	}
};
}  // namespace ufo::map

#endif  // UFO_MAP_OCCUPANCY_MAP_TIME_BASE_H