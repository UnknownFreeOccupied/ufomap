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

#ifndef UFO_MAP_TIME_MAP_BASE_H
#define UFO_MAP_TIME_MAP_BASE_H

// UFO
#include <ufo/map/predicate/time.h>
#include <ufo/map/types.h>

namespace ufo::map
{
template <class Derived, class LeafNode, class InnerNode>
class TimeMapBase
{
 public:
	//
	// Get time step
	//

	constexpr time_step_t getTimeStep(Node node) const noexcept
	{
		return getTimeStep(derived().getLeafNode(node));
	}

	time_step_t getTimeStep(Code code) const
	{
		return getTimeStep(derived().getLeafNode(code));
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

	void setTimeStep(Node node, time_step_t time_step, bool propagate = true)
	{
		derived().apply(
		    node, [this, time_step](auto&& node) { setTimeStep(node, time_step); },
		    propagate);
	}

	void setTimeStep(Code code, time_step_t time_step, bool propagate = true)
	{
		derived().apply(
		    code, [this, time_step](auto&& node) { setTimeStep(node, time_step); },
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
	void setTimeStep(ExecutionPolicy policy, Node node, time_step_t time_step,
	                 bool propagate = true)
	{
		derived().apply(
		    policy, node, [this, time_step](auto&& node) { setTimeStep(node, time_step); },
		    propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setTimeStep(ExecutionPolicy policy, Code code, time_step_t time_step,
	                 bool propagate = true)
	{
		derived().apply(
		    policy, code, [this, time_step](auto&& node) { setTimeStep(node, time_step); },
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
		derived().setModified(1);

		if (propagate) {
			derived().updateModifiedNodes();
		}
	}

 protected:
	//
	// Derived
	//

	constexpr Derived& derived() { return *static_cast<Derived*>(this); }

	constexpr Derived const& derived() const { return *static_cast<Derived const*>(this); }

	//
	// Initilize root
	//

	void initRoot() { setTimeStep(derived().getRoot(), 0); }

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
		switch (time_step_prop_criteria_) {
			case PropagationCriteria::MIN:
				setTimeStep(node, minChildTimeStep(node, depth));
				break;
			case PropagationCriteria::MAX:
				setTimeStep(node, maxChildTimeStep(node, depth));
				break;
			case PropagationCriteria::MEAN:
				setTimeStep(node, averageChildTimeStep(node, depth));
				break;
		}
	}

	//
	// Max child time step
	//

	constexpr time_step_t maxChildTimeStep(InnerNode const& node, depth_t depth) const
	{
		return 1 == depth ? std::max({getTimeStep(derived().getLeafChild(node, 0)),
		                              getTimeStep(derived().getLeafChild(node, 1)),
		                              getTimeStep(derived().getLeafChild(node, 2)),
		                              getTimeStep(derived().getLeafChild(node, 3)),
		                              getTimeStep(derived().getLeafChild(node, 4)),
		                              getTimeStep(derived().getLeafChild(node, 5)),
		                              getTimeStep(derived().getLeafChild(node, 6)),
		                              getTimeStep(derived().getLeafChild(node, 7))})
		                  : std::max({getTimeStep(derived().getInnerChild(node, 0)),
		                              getTimeStep(derived().getInnerChild(node, 1)),
		                              getTimeStep(derived().getInnerChild(node, 2)),
		                              getTimeStep(derived().getInnerChild(node, 3)),
		                              getTimeStep(derived().getInnerChild(node, 4)),
		                              getTimeStep(derived().getInnerChild(node, 5)),
		                              getTimeStep(derived().getInnerChild(node, 6)),
		                              getTimeStep(derived().getInnerChild(node, 7))});
	}

	//
	// Min child time step
	//

	constexpr time_step_t minChildTimeStep(InnerNode const& node, depth_t depth) const
	{
		return 1 == depth ? std::min({getTimeStep(derived().getLeafChild(node, 0)),
		                              getTimeStep(derived().getLeafChild(node, 1)),
		                              getTimeStep(derived().getLeafChild(node, 2)),
		                              getTimeStep(derived().getLeafChild(node, 3)),
		                              getTimeStep(derived().getLeafChild(node, 4)),
		                              getTimeStep(derived().getLeafChild(node, 5)),
		                              getTimeStep(derived().getLeafChild(node, 6)),
		                              getTimeStep(derived().getLeafChild(node, 7))})
		                  : std::min({getTimeStep(derived().getInnerChild(node, 0)),
		                              getTimeStep(derived().getInnerChild(node, 1)),
		                              getTimeStep(derived().getInnerChild(node, 2)),
		                              getTimeStep(derived().getInnerChild(node, 3)),
		                              getTimeStep(derived().getInnerChild(node, 4)),
		                              getTimeStep(derived().getInnerChild(node, 5)),
		                              getTimeStep(derived().getInnerChild(node, 6)),
		                              getTimeStep(derived().getInnerChild(node, 7))});
	}

	//
	// Average child time step
	//

	constexpr time_step_t averageChildTimeStep(InnerNode const& node, depth_t depth) const
	{
		time_step_t sum =
		    1 == depth ? std::accumulate(
		                     std::begin(derived().getLeafChildren(node)),
		                     std::end(derived().getLeafChildren(node)), time_step_t(0),
		                     [](auto cur, auto&& child) { return cur + getTimeStep(child); })
		               : std::accumulate(
		                     std::begin(derived().getInnerChildren(node)),
		                     std::end(derived().getInnerChildren(node)), time_step_t(0),
		                     [](auto cur, auto&& child) { return cur + getTimeStep(child); });

		return sum / time_step_t(8);
	}

	//
	// Input/output (read/write)
	//

	bool canReadData(DataIdentifier identifier) const noexcept
	{
		return DataIdentifier::TIME == identifier;
	}

	bool readNodes(std::istream& in_stream, std::vector<LeafNode*> const& nodes,
	               DataIdentifier data_identifier, uint64_t size)
	{
		if ("time_step" != field) {
			return false;
		}

		if ('U' == type && sizeof(uint32_t) == size) {
			auto data = std::make_unique<uint32_t[]>(nodes.size());
			in_stream.read(reinterpret_cast<char*>(data.get()),
			               nodes.size() * sizeof(uint32_t));

			for (size_t i = 0; i != nodes.size(); ++i) {
				setTimeStep(*nodes[i], data[i]);
			}
		} else {
			return false;
		}
		return true;
	}

	void writeNodes(std::ostream& out_stream, std::vector<LeafNode> const& nodes) const
	{
		uint64_t const size = nodes.size() * sizeof(uint32_t);
		out_stream.write(reinterpret_cast<char const*>(&size), sizeof(size));

		auto data = std::make_unique<uint32_t[]>(nodes.size());
		for (size_t i = 0; i != nodes.size(); ++i) {
			data[i] = getTimeStep(nodes[i]);
		}

		out_stream.write(reinterpret_cast<char const*>(data.get()),
		                 nodes.size() * sizeof(uint32_t));
	}

 protected:
	// Propagation criteria
	PropagationCriteria time_step_prop_criteria_ = PropagationCriteria::MAX;
};
}  // namespace ufo::map

#endif  // UFO_MAP_TIME_MAP_BASE_H