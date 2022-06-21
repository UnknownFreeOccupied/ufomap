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
 protected:
	using OccupancyBase = OccupancyMapBase<Derived, DataType, Indicators>;
	using typename OccupancyBase::InnerNode;
	using typename OccupancyBase::LeafNode;

	static_assert(std::is_base_of_v<OccupancyTimeNode, LeafNode>);

 public:
	//
	// Get time step
	//

	static constexpr time_step_t getTimeStep(Node const& node) noexcept
	{
		return getTimeStep(Derived::getLeafNode(node));
	}

	time_step_t getTimeStep(Code code) const
	{
		return getTimeStep(Derived::getLeafNode(code));
	}

	time_step_t getTimeStep(Key key) const { return getTimeStep(Derived::toCode(key)); }

	time_step_t getTimeStep(Point3 coord, depth_t depth = 0) const
	{
		return getTimeStep(derived().toCode(coord, depth));
	}

	time_step_t getTimeStep(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return getTimeStep(derived().toCode(x, y, z, depth));
	}

	//
	// Set time step
	//

	void setTimeStep(Node& node, time_step_t time_step, bool propagate = true)
	{
		derived().apply(
		    node, [this, time_step](DataType& node) { setTimeStepImpl(node, time_step); },
		    propagate);
	}

	void setTimeStep(Code code, time_step_t time_step, bool propagate = true)
	{
		derived().apply(
		    code, [this, time_step](DataType& node) { setTimeStepImpl(node, time_step); },
		    propagate);
	}

	void setTimeStep(Key key, time_step_t time_step, bool propagate = true)
	{
		setTimeStep(Derived::toCode(key), time_step, propagate);
	}

	void setTimeStep(Point3 coord, time_step_t time_step, bool propagate = true,
	                 depth_t depth = 0)
	{
		setTimeStep(derived().toCode(coord, depth), time_step, propagate);
	}

	void setTimeStep(coord_t x, coord_t y, coord_t z, time_step_t time_step,
	                 bool propagate = true, depth_t depth = 0)
	{
		setTimeStep(derived().toCode(x, y, z, depth), time_step, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setTimeStep(ExecutionPolicy policy, Node& node, time_step_t time_step,
	                 bool propagate = true)
	{
		derived().apply(
		    policy, node,
		    [this, time_step](DataType& node) { setTimeStepImpl(node, time_step); },
		    propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setTimeStep(ExecutionPolicy policy, Code code, time_step_t time_step,
	                 bool propagate = true)
	{
		derived().apply(
		    policy, code,
		    [this, time_step](DataType& node) { setTimeStepImpl(node, time_step); },
		    propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setTimeStep(ExecutionPolicy policy, Key key, time_step_t time_step,
	                 bool propagate = true)
	{
		setTimeStep(policy, Derived::toCode(key), time_step, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setTimeStep(ExecutionPolicy policy, Point3 coord, time_step_t time_step,
	                 bool propagate = true, depth_t depth = 0)
	{
		setTimeStep(policy, derived().toCode(coord, depth), time_step, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setTimeStep(ExecutionPolicy policy, coord_t x, coord_t y, coord_t z,
	                 time_step_t time_step, bool propagate = true, depth_t depth = 0)
	{
		setTimeStep(policy, derived().toCode(x, y, z, depth), time_step, propagate);
	}

	//
	// Propagation criteria
	//

	constexpr PropagationCriteria getTimeStepPropagationCriteria() const noexcept
	{
		return time_step_prop_criteria_;
	}

	constexpr void setTimeStepPropagationCriteria(
	    PropagationCriteria time_step_prop_criteria, bool propagate = true) noexcept
	{
		if (time_step_prop_criteria_ == time_step_prop_criteria) {
			return;
		}

		time_step_prop_criteria_ = time_step_prop_criteria;

		// Set all inner nodes to modified
		// FIXME: Possible to optimize this to only set the ones with children
		Derived::setModified(1);

		if (propagate) {
			derived().updateModifiedNodes();
		}
	}

 protected:
	//
	// Constructors
	//

	OccupancyMapTimeBase(float occupied_thres, float free_thres, float clamping_thres_min,
	                     float clamping_thres_max)
	    : OccupancyBase(occupied_thres, free_thres, clamping_thres_min, clamping_thres_max)
	{
		// TODO: Implement?
	}

	OccupancyMapTimeBase(OccupancyMapTimeBase const& other) : OccupancyBase(other)
	{
		// TODO: Implement?
	}

	OccupancyMapTimeBase(OccupancyMapTimeBase&& other) : OccupancyBase(std::move(other))
	{
		// TODO: Implement?
	}

	OccupancyMapTimeBase& operator=(OccupancyMapTimeBase const& rhs)
	{
		OccupancyBase::operator=(rhs);
		return *this;
	}

	OccupancyMapTimeBase& operator=(OccupancyMapTimeBase&& rhs)
	{
		// TODO: Implement
		OccupancyBase::operator=(std::move(rhs));
		return *this;
	}

	//
	// Destructor
	//

	~OccupancyMapTimeBase() {}

	//
	// Derived
	//

	constexpr Derived& derived() { return *static_cast<Derived*>(this); }

	constexpr Derived const& derived() const { return *static_cast<Derived const*>(this); }

	//
	// Initilize root
	//

	void initRoot()
	{
		OccupancyBase::initRoot();
		setTimeStep(derived().getRoot(), 0);
	}

	//
	// Get time step
	//

	static constexpr time_step_t getTimeStep(LeafNode const& node)
	{
		return node.time_step;
	}

	//
	// Set time step
	//

	static constexpr void setTimeStep(LeafNode& node, time_step_t time_step)
	{
		node.time_step = time_step;
	}

	//
	// Update node
	//

	void updateNode(InnerNode& node, depth_t depth)
	{
		OccupancyBase::updateNode(node, depth);

		switch (time_step_prop_criteria_) {
			case PropagationCriteria::Max:
				setTimeStep(node, maxChildTimeStep(node, depth));
				break;
			case PropagationCriteria::Min:
				setTimeStep(node, minChildTimeStep(node, depth));
				break;
			case PropagationCriteria::Mean:
				setTimeStep(node, averageChildTimeStep(node, depth));
				break;
		}
	}

	//
	// Max child time step
	//

	static constexpr time_step_t maxChildTimeStep(InnerNode const& node, depth_t depth)
	{
		return 1 == depth ? std::max({getTimeStep(Derived::getLeafChild(node, 0)),
		                              getTimeStep(Derived::getLeafChild(node, 1)),
		                              getTimeStep(Derived::getLeafChild(node, 2)),
		                              getTimeStep(Derived::getLeafChild(node, 3)),
		                              getTimeStep(Derived::getLeafChild(node, 4)),
		                              getTimeStep(Derived::getLeafChild(node, 5)),
		                              getTimeStep(Derived::getLeafChild(node, 6)),
		                              getTimeStep(Derived::getLeafChild(node, 7))})
		                  : std::max({getTimeStep(Derived::getInnerChild(node, 0)),
		                              getTimeStep(Derived::getInnerChild(node, 1)),
		                              getTimeStep(Derived::getInnerChild(node, 2)),
		                              getTimeStep(Derived::getInnerChild(node, 3)),
		                              getTimeStep(Derived::getInnerChild(node, 4)),
		                              getTimeStep(Derived::getInnerChild(node, 5)),
		                              getTimeStep(Derived::getInnerChild(node, 6)),
		                              getTimeStep(Derived::getInnerChild(node, 7))});
	}

	//
	// Min child time step
	//

	static constexpr time_step_t minChildTimeStep(InnerNode const& node, depth_t depth)
	{
		return 1 == depth ? std::min({getTimeStep(Derived::getLeafChild(node, 0)),
		                              getTimeStep(Derived::getLeafChild(node, 1)),
		                              getTimeStep(Derived::getLeafChild(node, 2)),
		                              getTimeStep(Derived::getLeafChild(node, 3)),
		                              getTimeStep(Derived::getLeafChild(node, 4)),
		                              getTimeStep(Derived::getLeafChild(node, 5)),
		                              getTimeStep(Derived::getLeafChild(node, 6)),
		                              getTimeStep(Derived::getLeafChild(node, 7))})
		                  : std::min({getTimeStep(Derived::getInnerChild(node, 0)),
		                              getTimeStep(Derived::getInnerChild(node, 1)),
		                              getTimeStep(Derived::getInnerChild(node, 2)),
		                              getTimeStep(Derived::getInnerChild(node, 3)),
		                              getTimeStep(Derived::getInnerChild(node, 4)),
		                              getTimeStep(Derived::getInnerChild(node, 5)),
		                              getTimeStep(Derived::getInnerChild(node, 6)),
		                              getTimeStep(Derived::getInnerChild(node, 7))});
	}

	//
	// Average child time step
	//

	static constexpr time_step_t averageChildTimeStep(InnerNode const& node, depth_t depth)
	{
		time_step_t sum =
		    1 == depth ? std::accumulate(
		                     std::begin(Derived::getLeafChildren(node)),
		                     std::end(Derived::getLeafChildren(node)), time_step_t(0),
		                     [](auto cur, auto&& child) { return cur + getTimeStep(child); })
		               : std::accumulate(
		                     std::begin(Derived::getInnerChildren(node)),
		                     std::end(Derived::getInnerChildren(node)), time_step_t(0),
		                     [](auto cur, auto&& child) { return cur + getTimeStep(child); });

		return sum / time_step_t(8);
	}

	//
	// Input/output (read/write)
	//

	void addFileInfo(FileInfo& info) const
	{
		OccupancyBase::addFileInfo(info);
		info["fields"].emplace_back("time_step");
		info["type"].emplace_back("U");
		info["size"].emplace_back("4");
	}

	bool readNodes(std::istream& in_stream, std::vector<LeafNode*> const& nodes,
	               std::string const& field, char type, uint64_t size, uint64_t num)
	{
		if (OccupancyBase::readNodes(in_stream, nodes, field, type, size, num)) {
			return true;
		}

		if ("time_step" != field) {
			return false;
		}

		if ('U' == type && 4 == size) {
			auto data = std::make_unique<uint32_t[]>(nodes.size());
			in_stream.read(reinterpret_cast<char*>(data.get()),
			               nodes.size() * sizeof(uint32_t));

			for (size_t i = 0; i != data.size(); ++i) {
				setTimeStep(*nodes[i], data[i]);
			}
		} else {
			return false;
		}
		return true;
	}

	void writeNodes(std::ostream& out_stream, std::vector<LeafNode const*> const& nodes,
	                bool compress, int compression_acceleration_level,
	                int compression_level) const
	{
		OccupancyBase::writeNodes(out_stream, nodes, compress, compression_acceleration_level,
		                          compression_level);

		auto data = std::make_unique<uint32_t[]>(nodes.size());
		for (size_t i = 0; i != nodes.size(); ++i) {
			data[i] = getTimeStep(*nodes[i]);
		}

		uint64_t size = nodes.size();
		out_stream.write(reinterpret_cast<char*>(&size), sizeof(size));
		out_stream.write(reinterpret_cast<char const*>(data.get()),
		                 nodes.size() * sizeof(uint32_t));
	}

 protected:
	// Propagation criteria
	PropagationCriteria time_step_prop_criteria_ = PropagationCriteria::Max;
};
}  // namespace ufo::map

#endif  // UFO_MAP_OCCUPANCY_MAP_TIME_BASE_H