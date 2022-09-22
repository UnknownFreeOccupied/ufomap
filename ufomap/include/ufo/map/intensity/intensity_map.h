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

#ifndef UFO_MAP_INTENSITY_MAP_H
#define UFO_MAP_INTENSITY_MAP_H

// UFO
#include <ufo/map/intensity/intensity_node.h>
#include <ufo/map/intensity/intensity_predicate.h>
#include <ufo/map/types.h>

// STL
#include <functional>
#include <iostream>
#include <limits>
#include <utility>

namespace ufo::map
{
template <class Derived>
class IntensityMap
{
 public:
	//
	// Get intensity
	//

	[[nodiscard]] constexpr intensity_t intensity(Node node) const
	{
		return derived().leafNode(node).intensityIndex(node.index());
	}

	[[nodiscard]] intensity_t intensity(Code code) const
	{
		return derived().leafNode(code).intensityIndex(code.index());
	}

	[[nodiscard]] intensity_t intensity(Key key) const
	{
		return intensity(Derived::toCode(key));
	}

	[[nodiscard]] intensity_t intensity(Point coord, depth_t depth = 0) const
	{
		return intensity(derived().toCode(coord, depth));
	}

	[[nodiscard]] intensity_t intensity(coord_t x, coord_t y, coord_t z,
	                                    depth_t depth = 0) const
	{
		return intensity(derived().toCode(x, y, z, depth));
	}

	//
	// Set intensity
	//

	void setIntensity(Node node, intensity_t intensity, bool propagate = true)
	{
		derived().apply(
		    node,
		    [intensity](auto& node, index_t const index) {
			    node.setIntensityIndex(index, intensity);
		    },
		    [intensity](auto& node) { node.setIntensity(intensity); }, propagate);
	}

	void setIntensity(Code code, intensity_t intensity, bool propagate = true)
	{
		derived().apply(
		    code,
		    [intensity](auto& node, index_t const index) {
			    node.setIntensityIndex(index, intensity);
		    },
		    [intensity](auto& node) { node.setIntensity(intensity); }, propagate);
	}

	void setIntensity(Key key, intensity_t intensity, bool propagate = true)
	{
		setIntensity(Derived::toCode(key), intensity, propagate);
	}

	void setIntensity(Point coord, intensity_t intensity, bool propagate = true,
	                  depth_t depth = 0)
	{
		setIntensity(derived().toCode(coord, depth), intensity, propagate);
	}

	void setIntensity(coord_t x, coord_t y, coord_t z, intensity_t intensity,
	                  bool propagate = true, depth_t depth = 0)
	{
		setIntensity(derived().toCode(x, y, z, depth), intensity, propagate);
	}

	//
	// Propagation criteria
	//

	[[nodiscard]] constexpr PropagationCriteria intensityPropagationCriteria()
	    const noexcept
	{
		return prop_criteria_;
	}

	constexpr void setIntensityPropagationCriteria(PropagationCriteria prop_criteria,
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

	IntensityMap() = default;

	IntensityMap(IntensityMap const& other) = default;

	IntensityMap(IntensityMap&& other) = default;

	template <class Derived2>
	IntensityMap(IntensityMap<Derived2> const& other)
	    : prop_criteria_(other.intensityPropagationCriteria())
	{
	}

	//
	// Assignment operator
	//

	IntensityMap& operator=(IntensityMap const& rhs) = default;

	IntensityMap& operator=(IntensityMap&& rhs) = default;

	template <class Derived2>
	IntensityMap& operator=(IntensityMap<Derived2> const& rhs)
	{
		prop_criteria_ = rhs.intensityPropagationCriteria();
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

	void initRoot() { derived().root().setIntensityIndex(derived().rootIndex(), 0); }

	//
	// Update node
	//

	template <std::size_t N, class T>
	void updateNode(IntensityNode<N>& node, IndexField const indices, T const& children)
	{
		if constexpr (1 == N) {
			switch (prop_criteria_) {
				case PropagationCriteria::MIN:
					node.intensity[0] = minIntensity(children);
					break;
				case PropagationCriteria::MAX:
					node.intensity[0] = maxIntensity(children);
					break;
				case PropagationCriteria::MEAN:
					node.intensity[0] = averageIntensity(children);
					break;
			}
		} else {
			for (index_t index = 0; children.size() != index; ++index) {
				if (indices[index]) {
					switch (prop_criteria_) {
						case PropagationCriteria::MIN:
							node.intensity[index] = minIntensity(children[index]);
							break;
						case PropagationCriteria::MAX:
							node.intensity[index] = maxIntensity(children[index]);
							break;
						case PropagationCriteria::MEAN:
							node.intensity[index] = averageIntensity(children[index]);
							break;
					}
				}
			}
		}
	}

	//
	// Min child intensity
	//

	template <class T>
	[[nodiscard]] constexpr intensity_t minIntensity(T const& nodes) const
	{
		intensity_t min = std::numeric_limits<intensity_t>::max();
		for (auto const& node : nodes) {
			min = std::min(min, node.intensity[0]);
		}
		return min;
	}

	[[nodiscard]] constexpr intensity_t minIntensity(IntensityNode<8> const& node) const
	{
		return minIntensity(std::cbegin(node.intensity), std::cend(node.intensity));
	}

	template <class InputIt>
	[[nodiscard]] constexpr intensity_t minIntensity(InputIt first, InputIt last) const
	{
		intensity_t min = std::numeric_limits::max();
		for (; first != last; ++first) {
			min = std::min(min, *first);
		}
		return min;
	}

	//
	// Max child intensity
	//

	template <class T>
	[[nodiscard]] constexpr intensity_t maxIntensity(T const& nodes) const
	{
		intensity_t max = std::numeric_limits<intensity_t>::lowest();
		for (auto const& node : nodes) {
			max = std::max(max, node.intensity[0]);
		}
		return max;
	}

	[[nodiscard]] constexpr intensity_t maxIntensity(IntensityNode<8> const& node) const
	{
		return maxIntensity(std::cbegin(node.intensity), std::cend(node.intensity));
	}

	template <class InputIt>
	[[nodiscard]] constexpr intensity_t maxIntensity(InputIt first, InputIt last) const
	{
		intensity_t max = std::numeric_limits::lowest();
		for (; first != last; ++first) {
			max = std::max(max, *first);
		}
		return max;
	}

	//
	// Average child intensity
	//

	template <class T>
	[[nodiscard]] static constexpr intensity_t averageIntensity(T const& nodes)
	{
		return std::accumulate(std::cbegin(nodes), std::cend(nodes), 0.0,
		                       [](double cur, IntensityNode<1> node) {
			                       return cur + node.intensity[0];
		                       }) /
		       double(nodes.size());
	}

	[[nodiscard]] static constexpr intensity_t averageIntensity(
	    IntensityNode<8> const& node)
	{
		return averageIntensity(std::cbegin(node.intensity), std::cend(node.intensity));
	}

	template <class InputIt>
	[[nodiscard]] static constexpr intensity_t averageIntensity(InputIt first, InputIt last)
	{
		return std::reduce(first, last, 0.0) / double(std::distance(first, last));
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr DataIdentifier dataIdentifier() noexcept
	{
		return DataIdentifier::INTENSITY;
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
		return node_type::intensitySize();
	}

	template <class OutputIt>
	void readNodes(std::istream& in, OutputIt first, std::size_t num_nodes)
	{
		uint8_t n;
		in.read(reinterpret_cast<char*>(&n), sizeof(n));
		num_nodes *= n;

		auto data = std::make_unique<intensity_t[]>(num_nodes);
		in.read(reinterpret_cast<char*>(data.get()),
		        num_nodes * sizeof(typename decltype(data)::element_type));

		auto const d = data.get();
		if constexpr (1 == numData<OutputIt>()) {
			if (1 == n) {
				for (std::size_t i = 0; i != num_nodes; ++first, ++i) {
					first->node.intensity[0] = *(d + i);
				}
			} else {
				auto const prop_criteria = prop_criteria_;
				for (std::size_t i = 0; i != num_nodes; ++first, i += 8) {
					switch (prop_criteria) {
						case PropagationCriteria::MIN:
							first->node.intensity[0] = minIntensity(d + i, d + i + 8);
							break;
						case PropagationCriteria::MAX:
							first->node.intensity[0] = maxIntensity(d + i, d + i + 8);
							break;
						case PropagationCriteria::MEAN:
							first->node.intensity[0] = averageIntensity(d + i, d + i + 8);
							break;
					}
				}
			}
		} else {
			if (1 == n) {
				for (std::size_t i = 0; i != num_nodes; ++first, ++i) {
					if (first->index_field.all()) {
						first->node.intensity.fill(*(d + i));
					} else {
						for (std::size_t index = 0; first->node.intensity.size() != index; ++index) {
							if (first.index_field[index]) {
								first->node.intensity[index] = *(d + i);
							}
						}
					}
				}
			} else {
				for (std::size_t i = 0; i != num_nodes; ++first, i += 8) {
					if (first->index_field.all()) {
						std::copy(d + i, d + i + 8, first->node.intensity.data());
					} else {
						for (index_t index = 0; first->node.intensity.size() != index; ++i, ++index) {
							if (first.index_field[index]) {
								first->node.intensity[index] = *(d + i + index);
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

		auto data = std::make_unique<intensity_t[]>(num_nodes);
		auto d = data.get();
		for (std::size_t i = 0; i != num_nodes; ++first, i += n) {
			std::copy(std::cbegin(first->node.intensity), std::cend(first->node.intensity),
			          d + i);
		}

		out.write(reinterpret_cast<char const*>(&n), sizeof(n));
		out.write(reinterpret_cast<char const*>(data.get()),
		          num_nodes * sizeof(typename decltype(data)::element_type));
	}

 protected:
	// Propagation criteria
	PropagationCriteria prop_criteria_ = PropagationCriteria::MAX;
};
}  // namespace ufo::map

#endif  // UFO_MAP_INTENSITY_MAP_H