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

#ifndef UFO_MAP_DISTANCE_MAP_H
#define UFO_MAP_DISTANCE_MAP_H

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
class DistanceMap
{
 public:
	//
	// Get distance
	//

	[[nodiscard]] constexpr distance_t distance(Node node) const noexcept
	{
		return derived().leafNode(node).distanceIndex(node.index());
	}

	[[nodiscard]] distance_t distance(Code code) const
	{
		return derived().leafNode(code).distanceIndex(code.index());
	}

	[[nodiscard]] distance_t distance(Key key) const
	{
		return distance(Derived::toCode(key));
	}

	[[nodiscard]] distance_t distance(Point coord, depth_t depth = 0) const
	{
		return distance(derived().toCode(coord, depth));
	}

	[[nodiscard]] distance_t distance(coord_t x, coord_t y, coord_t z,
	                                  depth_t depth = 0) const
	{
		return distance(derived().toCode(x, y, z, depth));
	}

	//
	// Set distance
	//

	void setDistance(Node node, distance_t distance, bool propagate = true)
	{
		derived().apply(
		    node,
		    [distance](auto& node, index_t const index) {
			    node.setDistanceIndex(index, distance);
		    },
		    [distance](auto& node) { node.setDistance(distance); }, propagate);
	}

	void setDistance(Code code, distance_t distance, bool propagate = true)
	{
		derived().apply(
		    code,
		    [distance](auto& node, index_t const index) {
			    node.setDistanceIndex(index, distance);
		    },
		    [distance](auto& node) { node.setDistance(distance); }, propagate);
	}

	void setDistance(Key key, distance_t distance, bool propagate = true)
	{
		setDistance(Derived::toCode(key), distance, propagate);
	}

	void setDistance(Point coord, distance_t distance, bool propagate = true,
	                 depth_t depth = 0)
	{
		setDistance(derived().toCode(coord, depth), distance, propagate);
	}

	void setDistance(coord_t x, coord_t y, coord_t z, distance_t distance,
	                 bool propagate = true, depth_t depth = 0)
	{
		setDistance(derived().toCode(x, y, z, depth), distance, propagate);
	}

	//
	// Propagation criteria
	//

	[[nodiscard]] constexpr PropagationCriteria distancePropagationCriteria() const noexcept
	{
		return prop_criteria_;
	}

	constexpr void setDistancePropagationCriteria(PropagationCriteria prop_criteria,
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
	// Constructors
	//

	DistanceMap() = default;

	DistanceMap(DistanceMap const& other) = default;

	DistanceMap(DistanceMap&& other) = default;

	template <class Derived2>
	DistanceMap(DistanceMap<Derived2> const& other)
	    : prop_criteria_(other.distancePropagationCriteria())
	{
	}

	//
	// Assignment operator
	//

	DistanceMap& operator=(DistanceMap const& rhs) = default;

	DistanceMap& operator=(DistanceMap&& rhs) = default;

	template <class Derived2>
	DistanceMap& operator=(DistanceMap<Derived2> const& rhs)
	{
		prop_criteria_ = rhs.distancePropagationCriteria();
		return *this;
	}

	//
	// Derived
	//

	[[nodiscard]] constexpr Derived& derived() { return *static_cast<Derived*>(this); }

	[[nodiscard]] constexpr Derived const& derived() const
	{
		return *static_cast<Derived const*>(this);
	}

	//
	// Initilize root
	//

	void initRoot()
	{
		// FIXME: What should this be?
		derived().root().setDistanceIndex(derived().rootIndex(), 0);
	}

	//
	// Update node
	//

	template <std::size_t N, class T>
	void updateNode(DistanceNode<N>& node, index_field_t const indices, T const& children)
	{
		if constexpr (1 == N) {
			auto fun = [](DistanceNode<1> const node) { return node.distance[0]; };
			switch (prop_criteria_) {
				case PropagationCriteria::MIN:
					node.distance[0] = min(children, fun);
					break;
				case PropagationCriteria::MAX:
					node.distance[0] = max(children, fun);
					break;
				case PropagationCriteria::MEAN:
					node.distance[0] = mean(children, fun);
					break;
			}
		} else {
			for (index_t index = 0; children.size() != index; ++index) {
				if ((indices >> index) & index_field_t(1)) {
					switch (prop_criteria_) {
						case PropagationCriteria::MIN:
							node.distance[index] = min(children[index].distance);
							break;
						case PropagationCriteria::MAX:
							node.distance[index] = max(children[index].distance);
							break;
						case PropagationCriteria::MEAN:
							node.distance[index] = mean(children[index].distance);
							break;
					}
				}
			}
		}
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr DataIdentifier dataIdentifier() noexcept
	{
		return DataIdentifier::DISTANCE;
	}

	[[nodiscard]] static constexpr bool canReadData(DataIdentifier identifier) noexcept
	{
		return dataIdentifier() == identifier;
	}

	template <class InputIt>
	[[nodiscard]] static constexpr uint8_t numData() noexcept
	{
		using typename std::iterator_traits<InputIt>::value_type;
		using typename value_type::node_type;
		return node_type::distanceSize();
	}

	template <class OutputIt>
	void readNodes(std::istream& in, OutputIt first, std::size_t num_nodes)
	{
		uint8_t n;
		in.read(reinterpret_cast<char*>(&n), sizeof(n));
		num_nodes *= n;

		auto data = std::make_unique<distance_t[]>(num_nodes);
		in.read(reinterpret_cast<char*>(data.get()),
		        num_nodes * sizeof(typename decltype(data)::element_type));

		auto const d = data.get();
		if constexpr (1 == numData<OutputIt>()) {
			if (1 == n) {
				for (std::size_t i = 0; i != num_nodes; ++first, ++i) {
					first->node.distance[0] = *(d + i);
				}
			} else {
				auto const prop_criteria = prop_criteria_;
				for (std::size_t i = 0; i != num_nodes; ++first, i += 8) {
					switch (prop_criteria) {
						case PropagationCriteria::MIN:
							first->node.distance[0] = min(d + i, d + i + 8);
							break;
						case PropagationCriteria::MAX:
							first->node.distance[0] = max(d + i, d + i + 8);
							break;
						case PropagationCriteria::MEAN:
							first->node.distance[0] = mean(d + i, d + i + 8);
							break;
					}
				}
			}
		} else {
			if (1 == n) {
				for (std::size_t i = 0; i != num_nodes; ++first, ++i) {
					if (std::numeric_limits<index_field_t>::max() == first->index_field) {
						first->node.distance.fill(*(d + i));
					} else {
						for (std::size_t index = 0; first->node.distance.size() != index; ++index) {
							if ((first.index_field >> index) & index_field_t(1)) {
								first->node.distance[index] = *(d + i);
							}
						}
					}
				}
			} else {
				for (std::size_t i = 0; i != num_nodes; ++first, i += 8) {
					if (std::numeric_limits<index_field_t>::max() == first->index_field) {
						std::copy(d + i, d + i + 8, first->node.distance.data());
					} else {
						for (index_t index = 0; first->node.distance.size() != index; ++i, ++index) {
							if ((first.index_field >> index) & index_field_t(1)) {
								first->node.distance[index] = *(d + i + index);
							}
						}
					}
				}
			}
		}
	}

	template <class InputIt>
	void writeNodes(std::ostream& out, InputIt first, std::size_t num_nodes)
	{
		constexpr uint8_t const n = numData<InputIt>();
		num_nodes *= n;

		auto data = std::make_unique<distance_t[]>(num_nodes);
		auto d = data.get();
		for (std::size_t i = 0; i != num_nodes; ++first, i += n) {
			std::copy(std::cbegin(first->node.distance), std::cend(first->node.distance),
			          d + i);
		}

		out.write(reinterpret_cast<char const*>(&n), sizeof(n));
		out.write(reinterpret_cast<char const*>(data.get()),
		          num_nodes * sizeof(typename decltype(data)::element_type));
	}

 protected:
	// Propagation criteria
	PropagationCriteria prop_criteria_ = PropagationCriteria::MIN;
};
}  // namespace ufo::map

#endif  // UFO_MAP_DISTANCE_MAP_H