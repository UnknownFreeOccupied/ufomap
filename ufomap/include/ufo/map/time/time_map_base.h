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
#include <ufo/map/time/time_node.h>
#include <ufo/map/types.h>

// STL
#include <functional>
#include <iostream>
#include <limits>
#include <utility>

namespace ufo::map
{
template <class Derived>
class TimeMapBase
{
 private:
	using BlockNode = std::pair<std::reference_wrapper<TimeNodeBlock>, std::size_t>;

	static constexpr bool BLOCK_BASED = Derived::BLOCK_BASED;

 public:
	//
	// Get time
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
	// Set time
	//

	void setTime(Node node, time_t time, bool propagate = true)
	{
		derived().apply(
		    node, [time](auto& node) { TimeMapBase::setTime(node, time); },
		    [time](auto& nodes) { TimeMapBase::setTime(nodes, time); }, propagate);
	}

	void setTime(Code code, time_t time, bool propagate = true)
	{
		derived().apply(
		    code, [time](auto& node) { TimeMapBase::setTime(node, time); },
		    [time](auto& nodes) { TimeMapBase::setTime(nodes, time); }, propagate);
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
	// Get time
	//

	static constexpr time_t getTime(TimeNode node) noexcept { return node.time; }

	static constexpr time_t getTime(BlockNode const& node) noexcept
	{
		return node.first.getTime(node.second);
	}

	//
	// Set time
	//

	static constexpr void setTime(TimeNode& node, time_t time) { node.time = time; }

	template <class T>
	static constexpr void setTime(T& nodes, time_t time)
	{
		for (TimeNode& node : nodes) {
			node.time = time;
		}
	}

	static constexpr void setTime(BlockNode& node, time_t time)
	{
		node.first.setTime(node.second, time);
	}

	static constexpr void setTime(TimeNodeBlock& nodes, time_t time)
	{
		nodes.setTime(time);
	}

	//
	// Update node
	//

	constexpr void updateNode(TimeNode) noexcept {}

	template <class T>
	void updateNode(TimeNode& node, T const& children)
	{
		switch (time_prop_criteria_) {
			case PropagationCriteria::MIN:
				setTime(node, minChildTime(children));
				break;
			case PropagationCriteria::MAX:
				setTime(node, maxChildTime(children));
				break;
			case PropagationCriteria::MEAN:
				setTime(node, averageChildTime(children));
				break;
		}
	}

	constexpr void updateNode(TimeNodeBlock&) noexcept {}

	template <class T>
	constexpr void updateNode(TimeNodeBlock& nodes, T const& children)
	{
		for (std::size_t i = 0; i !=) }

	constexpr void updateNode(BlockNode& node, ) {}

	//
	// Min child time
	//

	template <class T>
	constexpr time_t minChildTime(T const& children) const
	{
		time_t min = std::numeric_limits<time_t>::max();
		for (auto const& child : children) {
			min = std::min(min, getTime(child));
		}
		return min;
	}

	//
	// Max child time
	//

	template <class T>
	constexpr time_t maxChildTime(T const& children) const
	{
		time_t max = std::numeric_limits<time_t>::lowest();
		for (auto const& child : children) {
			max = std::max(max, getTime(child));
		}
		return max;
	}

	//
	// Average child time
	//

	template <class T>
	constexpr time_t averageChildTime(T const& children) const
	{
		// FIXME: Make sure not overflow
		time_t sum =
		    std::accumulate(std::cbegin(children), std::cend(children), time_t(0),
		                    [](auto cur, auto const& child) { return cur + getTime(child); });
		return sum / time_t(children.size());
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

	template <class InputIt>
	void readNodes(std::istream& in, InputIt first, InputIt last)
	{
		auto const num_nodes = std::distance(first, last);

		auto data = std::make_unique<time_t[]>(num_nodes);
		in.read(reinterpret_cast<char*>(data.get()),
		        num_nodes * sizeof(typename decltype(data)::element_type));

		for (std::size_t i = 0; num_nodes != i; ++i, std::advance(first, 1)) {
			setTime(*first, data[i]);
		}
	}

	template <class InputIt>
	void writeNodes(std::ostream& out, InputIt first, InputIt last)
	{
		auto const num_nodes = std::distance(first, last);

		auto data = std::make_unique<time_t[]>(num_nodes);
		for (std::size_t i = 0; num_nodes != i; ++i, std::advance(first, 1)) {
			data[i] = getTime(*first);
		}

		out.write(reinterpret_cast<char const*>(data.get()),
		          num_nodes * sizeof(typename decltype(data)::element_type));
	}

 protected:
	// Propagation criteria
	PropagationCriteria time_prop_criteria_ = PropagationCriteria::MAX;
};
}  // namespace ufo::map

#endif  // UFO_MAP_TIME_MAP_BASE_H