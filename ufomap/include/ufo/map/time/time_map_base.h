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

// STL
#include <vector>

namespace ufo::map
{
template <class Derived, class LeafNode, class InnerNode>
class TimeMapBase
{
 public:
	using time_t = typename LeafNode::time_t;

	//
	// Get time step
	//

	constexpr time_t getTime(Node node) const noexcept
	{
		return getTime(derived().getLeafNode(node));
	}

	time_t getTime(Code code) const { return getTime(derived().getLeafNode(code)); }

	time_t getTime(Key key) const { return getTime(Derived::toCode(key)); }

	time_t getTime(Point3 coord, depth_t depth = 0) const
	{
		return getTime(derived().toCode(coord, depth));
	}

	time_t getTime(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return getTime(derived().toCode(x, y, z, depth));
	}

	//
	// Set time step
	//

	void setTime(Node node, time_t time, bool propagate = true)
	{
		derived().apply(
		    node, [this, time](LeafNode& node) { setTime(node, time); }, propagate);
	}

	void setTime(Code code, time_t time, bool propagate = true)
	{
		derived().apply(
		    code, [this, time](LeafNode& node) { setTime(node, time); }, propagate);
	}

	void setTime(Key key, time_t time, bool propagate = true)
	{
		setTime(Derived::toCode(key), time, propagate);
	}

	void setTime(Point3 coord, time_t time, bool propagate = true, depth_t depth = 0)
	{
		setTime(derived().toCode(coord, depth), time, propagate);
	}

	void setTime(coord_t x, coord_t y, coord_t z, time_t time, bool propagate = true,
	             depth_t depth = 0)
	{
		setTime(derived().toCode(x, y, z, depth), time, propagate);
	}

	//
	// Propagation criteria
	//

	constexpr PropagationCriteria getTimePropagationCriteria() const noexcept
	{
		return time_prop_criteria_;
	}

	constexpr void setTimePropagationCriteria(PropagationCriteria time_prop_criteria,
	                                          bool propagate = true) noexcept
	{
		if (time_prop_criteria_ == time_prop_criteria) {
			return;
		}

		time_prop_criteria_ = time_prop_criteria;

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

	void initRoot() { setTime(derived().getRoot(), 0); }

	//
	// Get time step
	//

	static constexpr time_t getTime(LeafNode const& node) { return node.time; }

	//
	// Set time step
	//

	static constexpr void setTime(LeafNode& node, time_t time) { node.time = time; }

	//
	// Update node
	//

	void updateNode(InnerNode& node, depth_t depth)
	{
		switch (time_prop_criteria_) {
			case PropagationCriteria::MIN:
				setTime(node, minChildTime(node, depth));
				break;
			case PropagationCriteria::MAX:
				setTime(node, maxChildTime(node, depth));
				break;
			case PropagationCriteria::MEAN:
				setTime(node, averageChildTime(node, depth));
				break;
		}
	}

	//
	// Max child time step
	//

	constexpr time_t maxChildTime(InnerNode const& node, depth_t depth) const
	{
		return 1 == depth ? std::max({getTime(derived().getLeafChild(node, 0)),
		                              getTime(derived().getLeafChild(node, 1)),
		                              getTime(derived().getLeafChild(node, 2)),
		                              getTime(derived().getLeafChild(node, 3)),
		                              getTime(derived().getLeafChild(node, 4)),
		                              getTime(derived().getLeafChild(node, 5)),
		                              getTime(derived().getLeafChild(node, 6)),
		                              getTime(derived().getLeafChild(node, 7))})
		                  : std::max({getTime(derived().getInnerChild(node, 0)),
		                              getTime(derived().getInnerChild(node, 1)),
		                              getTime(derived().getInnerChild(node, 2)),
		                              getTime(derived().getInnerChild(node, 3)),
		                              getTime(derived().getInnerChild(node, 4)),
		                              getTime(derived().getInnerChild(node, 5)),
		                              getTime(derived().getInnerChild(node, 6)),
		                              getTime(derived().getInnerChild(node, 7))});
	}

	//
	// Min child time step
	//

	constexpr time_t minChildTime(InnerNode const& node, depth_t depth) const
	{
		return 1 == depth ? std::min({getTime(derived().getLeafChild(node, 0)),
		                              getTime(derived().getLeafChild(node, 1)),
		                              getTime(derived().getLeafChild(node, 2)),
		                              getTime(derived().getLeafChild(node, 3)),
		                              getTime(derived().getLeafChild(node, 4)),
		                              getTime(derived().getLeafChild(node, 5)),
		                              getTime(derived().getLeafChild(node, 6)),
		                              getTime(derived().getLeafChild(node, 7))})
		                  : std::min({getTime(derived().getInnerChild(node, 0)),
		                              getTime(derived().getInnerChild(node, 1)),
		                              getTime(derived().getInnerChild(node, 2)),
		                              getTime(derived().getInnerChild(node, 3)),
		                              getTime(derived().getInnerChild(node, 4)),
		                              getTime(derived().getInnerChild(node, 5)),
		                              getTime(derived().getInnerChild(node, 6)),
		                              getTime(derived().getInnerChild(node, 7))});
	}

	//
	// Average child time step
	//

	constexpr time_t averageChildTime(InnerNode const& node, depth_t depth) const
	{
		time_t sum =
		    1 == depth
		        ? std::accumulate(
		              std::begin(derived().getLeafChildren(node)),
		              std::end(derived().getLeafChildren(node)), time_t(0),
		              [](auto cur, LeafNode const& child) { return cur + getTime(child); })
		        : std::accumulate(
		              std::begin(derived().getInnerChildren(node)),
		              std::end(derived().getInnerChildren(node)), time_t(0),
		              [](auto cur, LeafNode const& child) { return cur + getTime(child); });

		return sum / time_t(8);
	}

	//
	// Input/output (read/write)
	//

	static constexpr DataIdentifier getDataIdentifier() noexcept
	{
		return DataIdentifier::TIME;
	}

	static constexpr bool canReadData(DataIdentifier identifier) noexcept
	{
		return getDataIdentifier() == identifier;
	}

	void readNodes(std::istream& in_stream, std::vector<LeafNode*> const& nodes)
	{
		uint8_t type;
		in_stream.read(reinterpret_cast<char*>(&type), sizeof(type));
		DataType dt = getDataType(type);

		auto const num_nodes = nodes.size();

		auto data = std::make_unique<time_t[]>(nodes.size());
		in_stream.read(reinterpret_cast<char*>(data.get()), num_nodes * sizeof(time_t));

		for (size_t i = 0; num_nodes != i; ++i) {
			setTime(*nodes[i], data[i]);
		}
	}

	void writeNodes(std::ostream& out_stream, std::vector<LeafNode> const& nodes) const
	{
		uint8_t type = util::enumToValue(getDataType<time_t>());
		out_stream.write(reinterpret_cast<char const*>(&type), sizeof(type));

		auto num_nodes = nodes.size();
		auto data = std::make_unique<time_t[]>(num_nodes);
		for (size_t i = 0; num_nodes != i; ++i) {
			data[i] = getTime(nodes[i]);
		}

		out_stream.write(reinterpret_cast<char const*>(data.get()),
		                 num_nodes * sizeof(time_t));
	}

 protected:
	// Propagation criteria
	PropagationCriteria time_prop_criteria_ = PropagationCriteria::MAX;
};
}  // namespace ufo::map

#endif  // UFO_MAP_TIME_MAP_BASE_H