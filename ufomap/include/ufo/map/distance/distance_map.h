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

#ifndef UFO_MAP_DISTANCE_MAP_BASE_H
#define UFO_MAP_DISTANCE_MAP_BASE_H

// UFO
#include <ufo/algorithm/algorithm.h>
#include <ufo/map/distance/distance_node.h>
#include <ufo/map/distance/distance_predicate.h>

// STL
#include <cstdint>
#include <iostream>
#include <type_traits>
#include <utility>

namespace ufo::map
{
template <class Derived>
class DistanceMapBase
{
 public:
	//
	// Get distance
	//

	constexpr distance_t getDistance(Node node) const noexcept
	{
		return getDistance(derived().getLeafNode(node));
	}

	distance_t getDistance(Code code) const
	{
		return getDistance(derived().getLeafNode(code));
	}

	distance_t getDistance(Key key) const { return getDistance(Derived::toCode(key)); }

	distance_t getDistance(Point coord, depth_t depth = 0) const
	{
		return getDistance(derived().toCode(coord, depth));
	}

	distance_t getDistance(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return getDistance(derived().toCode(x, y, z, depth));
	}

	//
	// Set distance
	//

	void setDistance(Node node, distance_t time_step, bool propagate = true)
	{
		derived().apply(
		    node, [this, time_step](auto&& node) { setDistance(node, time_step); },
		    propagate);
	}

	void setDistance(Code code, distance_t time_step, bool propagate = true)
	{
		derived().apply(
		    code, [this, time_step](auto&& node) { setDistance(node, time_step); },
		    propagate);
	}

	void setDistance(Key key, distance_t time_step, bool propagate = true)
	{
		setDistance(Derived::toCode(key), time_step, propagate);
	}

	void setDistance(Point coord, distance_t time_step, bool propagate = true,
	                 depth_t depth = 0)
	{
		setDistance(derived().toCode(coord, depth), time_step, propagate);
	}

	void setDistance(coord_t x, coord_t y, coord_t z, distance_t time_step,
	                 bool propagate = true, depth_t depth = 0)
	{
		setDistance(derived().toCode(x, y, z, depth), time_step, propagate);
	}

	//
	// Propagation criteria
	//

	constexpr PropagationCriteria getDistancePropagationCriteria() const noexcept
	{
		return distance_prop_criteria_;
	}

	constexpr void setDistancePropagationCriteria(
	    PropagationCriteria distance_prop_criteria, bool propagate = true) noexcept
	{
		if (distance_prop_criteria_ == distance_prop_criteria) {
			return;
		}

		distance_prop_criteria_ = distance_prop_criteria;

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
		setDistance(derived().getRoot(), std::numeric_limits<distance_t>::max());
	}

	//
	// Get distance
	//

	static constexpr distance_t getDistance(LeafNode const& node) { return node.distance; }

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

	static constexpr DataIdentifier dataIdentifier() noexcept
	{
		return DataIdentifier::DISTANCE;
	}

	static constexpr bool canReadData(DataIdentifier identifier) noexcept
	{
		return dataIdentifier() == identifier;
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

#endif  // UFO_MAP_DISTANCE_MAP_BASE_H