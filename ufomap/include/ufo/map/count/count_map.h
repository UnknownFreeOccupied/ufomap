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

#ifndef UFO_MAP_COUNT_MAP_BASE_H
#define UFO_MAP_COUNT_MAP_BASE_H

// UFO
#include <ufo/algorithm/algorithm.h>
#include <ufo/map/counter/counter_node.h>
#include <ufo/map/counter/counter_predicate.h>

// STL
#include <cstdint>
#include <iostream>
#include <type_traits>
#include <utility>

namespace ufo::map
{
template <class Derived>
class CounterMapBase
{
 public:
	//
	// Get counter
	//

	constexpr counter_t getCounter(Node node) const noexcept
	{
		return getCounter(derived().getLeafNode(node));
	}

	counter_t getCounter(Code code) const
	{
		return getCounter(derived().getLeafNode(code));
	}

	counter_t getCounter(Key key) const { return getCounter(Derived::toCode(key)); }

	counter_t getCounter(Point3 coord, depth_t depth = 0) const
	{
		return getCounter(derived().toCode(coord, depth));
	}

	counter_t getCounter(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return getCounter(derived().toCode(x, y, z, depth));
	}

	//
	// Set counter
	//

	void setCounter(Node node, counter_t counter, bool propagate = true)
	{
		derived().apply(
		    node, [this, counter](auto const& node) { setCounter(node, counter); },
		    propagate);
	}

	void setCounter(Code code, counter_t counter, bool propagate = true)
	{
		derived().apply(
		    code, [this, counter](auto const& node) { setCounter(node, counter); },
		    propagate);
	}

	void setCounter(Key key, counter_t counter, bool propagate = true)
	{
		setCounter(Derived::toCode(key), counter, propagate);
	}

	void setCounter(Point3 coord, counter_t counter, bool propagate = true,
	                depth_t depth = 0)
	{
		setCounter(derived().toCode(coord, depth), counter, propagate);
	}

	void setCounter(coord_t x, coord_t y, coord_t z, counter_t counter,
	                bool propagate = true, depth_t depth = 0)
	{
		setCounter(derived().toCode(x, y, z, depth), counter, propagate);
	}

	//
	// Propagation criteria
	//

	constexpr PropagationCriteria counterPropagationCriteria() const noexcept
	{
		return prop_criteria_;
	}

	constexpr void setCounterPropagationCriteria(PropagationCriteria prop_criteria,
	                                             bool propagate = true) noexcept
	{
		if (prop_criteria_ == prop_criteria) {
			return;
		}

		prop_criteria_ = prop_criteria;

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
		// FIXME: What should this be?
		setCounter(derived().getRoot(), 0);
	}

	//
	// Get distance
	//

	static constexpr counter_t getCounter(LeafNode const& node) { return node.counter; }

	//
	// Set distance
	//

	static constexpr void setDistance(LeafNode& node, distance_t distance)
	{
		node.distance = distance;
	}

	//
	// Update node
	//

	void updateNode(DistanceNode) {}

	template <class T>
	void updateNode(DistanceNode& node, T const& children)
	{
		switch (distance_prop_criteria_) {
			case PropagationCriteria::MIN:
				setDistance(node, minChildDistance(children));
				break;
			case PropagationCriteria::MAX:
				setDistance(node, maxChildDistance(children));
				break;
			case PropagationCriteria::MEAN:
				setDistance(node, averageChildDistance(children));
				break;
		}
	}

	//
	// Min child distance
	//

	template <class T>
	constexpr distance_t minChildDistance(T const& children) const
	{
		// TODO: Implement
	}

	//
	// Max child distance
	//

	template <class T>
	constexpr distance_t maxChildDistance(T const& children) const
	{
		// TODO: Implement
	}

	//
	// Average child distance
	//

	template <class T>
	constexpr distance_t averageChildDistance(T const& children) const
	{
		// TODO: Implement
	}

	//
	// Input/output (read/write)
	//

	static constexpr DataIdentifier getDataIdentifier() noexcept
	{
		return DataIdentifier::DISTANCE;
	}

	static constexpr bool canReadData(DataIdentifier identifier) noexcept
	{
		return getDataIdentifier() == identifier;
	}

	template <class InputIt>
	void readNodes(std::istream& in, InputIt first, InputIt last)
	{
		auto const num_nodes = std::distance(first, last);

		auto data = std::make_unique<distance_t[]>(num_nodes);
		in.read(reinterpret_cast<char*>(data.get()),
		        num_nodes * sizeof(typename decltype(data)::element_type));

		for (std::size_t i = 0; num_nodes != i; ++i, std::advance(first, 1)) {
			setDistance(*first, data[i]);
		}
	}

	template <class InputIt>
	void writeNodes(std::ostream& out, InputIt first, InputIt last, bool compress,
	                int compression_acceleration_level, int compression_level) const
	{
		auto const num_nodes = std::distance(first, last);

		auto data = std::make_unique<distance_t[]>(num_nodes);
		for (std::size_t i = 0; num_nodes != i; ++i, std::advance(first, 1)) {
			data[i] = getDistance(*first);
		}

		out.write(reinterpret_cast<char const*>(data.get()),
		          num_nodes * sizeof(typename decltype(data)::element_type));
	}

 protected:
	// Propagation criteria
	PropagationCriteria distance_prop_criteria_ = PropagationCriteria::MIN;
};
}  // namespace ufo::map

#endif  // UFO_MAP_COUNT_MAP_BASE_H