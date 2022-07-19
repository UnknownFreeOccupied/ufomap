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

#ifndef UFO_MAP_SIGNED_DISTANCE_MAP_BASE_H
#define UFO_MAP_SIGNED_DISTANCE_MAP_BASE_H

// UFO
#include <ufo/algorithm/algorithm.h>

// STL
#include <cstdint>
#include <type_traits>
#include <utility>

namespace ufo::map
{
template <class Derived, class LeafNode, class InnerNode>
class SignedDistanceMapBase
{
 public:
	using signed_distance_t = typename LeafNode::signed_distance_type;

 public:
	//
	// Get signed distance
	//

	constexpr signed_distance_t getSignedDistance(Node node) const noexcept
	{
		return getSignedDistance(derived().getLeafNode(node));
	}

	signed_distance_t getSignedDistance(Code code) const
	{
		return getSignedDistance(derived().getLeafNode(code));
	}

	signed_distance_t getSignedDistance(Key key) const
	{
		return getSignedDistance(Derived::toCode(key));
	}

	signed_distance_t getSignedDistance(Point3 coord, depth_t depth = 0) const
	{
		return getSignedDistance(derived().toCode(coord, depth));
	}

	signed_distance_t getSignedDistance(coord_t x, coord_t y, coord_t z,
	                                    depth_t depth = 0) const
	{
		return getSignedDistance(derived().toCode(x, y, z, depth));
	}

	//
	// Set signed distance
	//

	void setSignedDistance(Node node, signed_distance_t time_step, bool propagate = true)
	{
		derived().apply(
		    node, [this, time_step](auto&& node) { setSignedDistance(node, time_step); },
		    propagate);
	}

	void setSignedDistance(Code code, signed_distance_t time_step, bool propagate = true)
	{
		derived().apply(
		    code, [this, time_step](auto&& node) { setSignedDistance(node, time_step); },
		    propagate);
	}

	void setSignedDistance(Key key, signed_distance_t time_step, bool propagate = true)
	{
		setSignedDistance(Derived::toCode(key), time_step, propagate);
	}

	void setSignedDistance(Point3 coord, signed_distance_t time_step, bool propagate = true,
	                       depth_t depth = 0)
	{
		setSignedDistance(derived().toCode(coord, depth), time_step, propagate);
	}

	void setSignedDistance(coord_t x, coord_t y, coord_t z, signed_distance_t time_step,
	                       bool propagate = true, depth_t depth = 0)
	{
		setSignedDistance(derived().toCode(x, y, z, depth), time_step, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setSignedDistance(ExecutionPolicy policy, Node node, signed_distance_t time_step,
	                       bool propagate = true)
	{
		derived().apply(
		    policy, node,
		    [this, time_step](auto&& node) { setSignedDistance(node, time_step); },
		    propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setSignedDistance(ExecutionPolicy policy, Code code, signed_distance_t time_step,
	                       bool propagate = true)
	{
		derived().apply(
		    policy, code,
		    [this, time_step](auto&& node) { setSignedDistance(node, time_step); },
		    propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setSignedDistance(ExecutionPolicy policy, Key key, signed_distance_t time_step,
	                       bool propagate = true)
	{
		setSignedDistance(policy, Derived::toCode(key), time_step, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setSignedDistance(ExecutionPolicy policy, Point3 coord,
	                       signed_distance_t time_step, bool propagate = true,
	                       depth_t depth = 0)
	{
		setSignedDistance(policy, derived().toCode(coord, depth), time_step, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setSignedDistance(ExecutionPolicy policy, coord_t x, coord_t y, coord_t z,
	                       signed_distance_t time_step, bool propagate = true,
	                       depth_t depth = 0)
	{
		setSignedDistance(policy, derived().toCode(x, y, z, depth), time_step, propagate);
	}

	//
	// Propagation criteria
	//

	constexpr PropagationCriteria getSignedDistancePropagationCriteria() const noexcept
	{
		return signed_distance_prop_criteria_;
	}

	constexpr void setSignedDistancePropagationCriteria(
	    PropagationCriteria signed_distance_prop_criteria, bool propagate = true) noexcept
	{
		if (signed_distance_prop_criteria_ == signed_distance_prop_criteria) {
			return;
		}

		signed_distance_prop_criteria_ = signed_distance_prop_criteria;

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

	void initRoot()
	{
		setSignedDistance(derived().getRoot(), std::numeric_limits<signed_distance_t>::max());
	}

	//
	// Initilize root
	//

	void initRoot() { setSurfel(derived().getRoot(), nullptr); }

	//
	// Get signed distance
	//

	static constexpr signed_distance_t getSignedDistance(LeafNode const& node)
	{
		return node.signed_distance;
	}

	//
	// Set signed distance
	//

	static constexpr void setSignedDistance(LeafNode& node,
	                                        signed_distance_t signed_distance)
	{
		node.signed_distance = signed_distance;
	}

	//
	// Update node
	//

	void updateNode(InnerNode& node, depth_t depth)
	{
		switch (signed_distance_prop_criteria_) {
			case PropagationCriteria::MIN:
				setSignedDistance(node, minChildSignedDistance(node, depth));
				break;
			case PropagationCriteria::MAX:
				setSignedDistance(node, maxChildSignedDistance(node, depth));
				break;
			case PropagationCriteria::MEAN:
				setSignedDistance(node, averageChildSignedDistance(node, depth));
				break;
		}
	}

	//
	// Min child signed distance
	//

	constexpr signed_distance_t minChildSignedDistance(InnerNode const& node,
	                                                   depth_t depth) const
	{
		return 1 == depth ? std::min({getSignedDistance(derived().getLeafChild(node, 0)),
		                              getSignedDistance(derived().getLeafChild(node, 1)),
		                              getSignedDistance(derived().getLeafChild(node, 2)),
		                              getSignedDistance(derived().getLeafChild(node, 3)),
		                              getSignedDistance(derived().getLeafChild(node, 4)),
		                              getSignedDistance(derived().getLeafChild(node, 5)),
		                              getSignedDistance(derived().getLeafChild(node, 6)),
		                              getSignedDistance(derived().getLeafChild(node, 7))})
		                  : std::min({getSignedDistance(derived().getInnerChild(node, 0)),
		                              getSignedDistance(derived().getInnerChild(node, 1)),
		                              getSignedDistance(derived().getInnerChild(node, 2)),
		                              getSignedDistance(derived().getInnerChild(node, 3)),
		                              getSignedDistance(derived().getInnerChild(node, 4)),
		                              getSignedDistance(derived().getInnerChild(node, 5)),
		                              getSignedDistance(derived().getInnerChild(node, 6)),
		                              getSignedDistance(derived().getInnerChild(node, 7))});
	}

	//
	// Max child signed distance
	//

	constexpr signed_distance_t maxChildSignedDistance(InnerNode const& node,
	                                                   depth_t depth) const
	{
		return 1 == depth ? std::max({getSignedDistance(derived().getLeafChild(node, 0)),
		                              getSignedDistance(derived().getLeafChild(node, 1)),
		                              getSignedDistance(derived().getLeafChild(node, 2)),
		                              getSignedDistance(derived().getLeafChild(node, 3)),
		                              getSignedDistance(derived().getLeafChild(node, 4)),
		                              getSignedDistance(derived().getLeafChild(node, 5)),
		                              getSignedDistance(derived().getLeafChild(node, 6)),
		                              getSignedDistance(derived().getLeafChild(node, 7))})
		                  : std::max({getSignedDistance(derived().getInnerChild(node, 0)),
		                              getSignedDistance(derived().getInnerChild(node, 1)),
		                              getSignedDistance(derived().getInnerChild(node, 2)),
		                              getSignedDistance(derived().getInnerChild(node, 3)),
		                              getSignedDistance(derived().getInnerChild(node, 4)),
		                              getSignedDistance(derived().getInnerChild(node, 5)),
		                              getSignedDistance(derived().getInnerChild(node, 6)),
		                              getSignedDistance(derived().getInnerChild(node, 7))});
	}

	//
	// Average child signed distance
	//

	constexpr signed_distance_t averageChildSignedDistance(InnerNode const& node,
	                                                       depth_t depth) const
	{
		signed_distance_t sum =
		    1 == depth
		        ? std::accumulate(
		              std::begin(derived().getLeafChildren(node)),
		              std::end(derived().getLeafChildren(node)), signed_distance_t(0),
		              [](auto cur, auto&& child) { return cur + getSignedDistance(child); })
		        : std::accumulate(
		              std::begin(derived().getInnerChildren(node)),
		              std::end(derived().getInnerChildren(node)), signed_distance_t(0),
		              [](auto cur, auto&& child) { return cur + getSignedDistance(child); });

		return sum / signed_distance_t(8);
	}

	//
	// Input/output (read/write)
	//

	void addFileInfo(FileInfo& info) const
	{
		info["fields"].emplace_back("surfel");
		info["type"].emplace_back("U");  // TODO: Implement
		info["size"].emplace_back(std::to_string(sizeof(signed_distance_t)));
	}

	bool readNodes(std::istream& in_stream, std::vector<LeafNode*> const& nodes,
	               std::string const& field, char type, uint64_t size)
	{
		if ("surfel" != field) {
			return false;
		}

		if ('U' == type && sizeof(signed_distance_t) == size) {
			auto data = std::make_unique<signed_distance_t[]>(nodes.size());
			in_stream.read(reinterpret_cast<char*>(data.get()),
			               nodes.size() * sizeof(signed_distance_t));

			for (size_t i = 0; i != nodes.size(); ++i) {
				setSignedDistance(*nodes[i], reinterpret_cast<signed_distance_t const>(data[i]));
			}
		} else {
			return false;
		}
		return true;
	}

	void writeNodes(std::ostream& out_stream, std::vector<LeafNode> const& nodes,
	                bool compress, int compression_acceleration_level,
	                int compression_level) const
	{
		uint64_t const size = nodes.size() * sizeof(signed_distance_t);
		out_stream.write(reinterpret_cast<char const*>(&size), sizeof(size));

		auto data = std::make_unique<signed_distance_t[]>(nodes.size());
		for (size_t i = 0; i != nodes.size(); ++i) {
			data[i] = reinterpret_cast<signed_distance_t const>(getSignedDistance(nodes[i]));
		}

		out_stream.write(reinterpret_cast<char const*>(data.get()),
		                 nodes.size() * sizeof(signed_distance_t));
	}

 protected:
	// Propagation criteria
	PropagationCriteria signed_distance_prop_criteria_ = PropagationCriteria::MIN;
};
}  // namespace ufo::map

#endif  // UFO_MAP_SIGNED_DISTANCE_MAP_BASE_H