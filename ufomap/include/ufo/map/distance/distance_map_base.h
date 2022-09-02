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
#include <ufo/map/predicate/signed_distance.h>

// STL
#include <cstdint>
#include <iostream>
#include <type_traits>
#include <utility>

namespace ufo::map
{
template <class Derived, class LeafNode>
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
		// FIXME: What should this be?
		setSignedDistance(derived().getRoot(), std::numeric_limits<signed_distance_t>::max());
	}

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

	void updateNode(SignedDistanceNode) {}

	template <class T>
	void updateNode(SignedDistanceNode& node, T const& children)
	{
		switch (signed_distance_prop_criteria_) {
			case PropagationCriteria::MIN:
				setSignedDistance(node, minChildSignedDistance(children));
				break;
			case PropagationCriteria::MAX:
				setSignedDistance(node, maxChildSignedDistance(children));
				break;
			case PropagationCriteria::MEAN:
				setSignedDistance(node, averageChildSignedDistance(children));
				break;
		}
	}

	//
	// Min child signed distance
	//

	template <class T>
	constexpr signed_distance_t minChildSignedDistance(T const& children) const
	{
		// TODO: Implement
	}

	//
	// Max child signed distance
	//

	template <class T>
	constexpr signed_distance_t maxChildSignedDistance(T const& children) const
	{
		// TODO: Implement
	}

	//
	// Average child signed distance
	//

	template <class T>
	constexpr signed_distance_t averageChildSignedDistance(T const& children) const
	{
		// TODO: Implement
	}

	//
	// Input/output (read/write)
	//

	static constexpr DataIdentifier getDataIdentifier() noexcept
	{
		return DataIdentifier::SIGNED_DISTANCE;
	}

	static constexpr bool canReadData(DataIdentifier identifier) noexcept
	{
		return getDataIdentifier() == identifier;
	}

	template <class InputIt>
	void readNodes(std::istream& in, InputIt first, InputIt last)
	{
		auto const num_nodes = std::distance(first, last);

		auto data = std::make_unique<signed_distance_t[]>(num_nodes);
		in.read(reinterpret_cast<char*>(data.get()),
		        num_nodes * sizeof(typename decltype(data)::element_type));

		for (std::size_t i = 0; num_nodes != i; ++i, std::advance(first, 1)) {
			setSignedDistance(*first, data[i]);
		}
	}

	template <class InputIt>
	void writeNodes(std::ostream& out, InputIt first, InputIt last, bool compress,
	                int compression_acceleration_level, int compression_level) const
	{
		auto const num_nodes = std::distance(first, last);

		auto data = std::make_unique<signed_distance_t[]>(num_nodes);
		for (std::size_t i = 0; num_nodes != i; ++i, std::advance(first, 1)) {
			data[i] = getSignedDistance(*first);
		}

		out.write(reinterpret_cast<char const*>(data.get()),
		          num_nodes * sizeof(typename decltype(data)::element_type));
	}

 protected:
	// Propagation criteria
	PropagationCriteria signed_distance_prop_criteria_ = PropagationCriteria::MIN;
};
}  // namespace ufo::map

#endif  // UFO_MAP_SIGNED_DISTANCE_MAP_BASE_H