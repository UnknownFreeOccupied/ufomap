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
#include <ufo/map/time/time_node.h>
#include <ufo/map/time/time_predicate.h>
#include <ufo/map/types.h>

// STL
#include <functional>
#include <iostream>
#include <limits>
#include <utility>

namespace ufo::map
{
template <class Derived>
class TimeMap
{
 public:
	//
	// Get time
	//

	[[nodiscard]] constexpr time_t time(Node node) const noexcept
	{
		return derived().leafNode(node).timeIndex(node.index());
	}

	[[nodiscard]] time_t time(Code code) const
	{
		return derived().leafNode(code).timeIndex(code.index());
	}

	[[nodiscard]] time_t time(Key key) const { return time(Derived::toCode(key)); }

	[[nodiscard]] time_t time(Point coord, depth_t depth = 0) const
	{
		return time(derived().toCode(coord, depth));
	}

	[[nodiscard]] time_t time(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return time(derived().toCode(x, y, z, depth));
	}

	//
	// Set time
	//

	void setTime(Node node, time_t time, bool propagate = true)
	{
		derived().apply(
		    node, [time](auto& node, index_t const index) { node.setTimeIndex(index, time); },
		    [time](auto& node) { node.setTime(time); }, propagate);
	}

	void setTime(Code code, time_t time, bool propagate = true)
	{
		derived().apply(
		    code, [time](auto& node, index_t const index) { node.setTimeIndex(index, time); },
		    [time](auto& node) { node.setTime(time); }, propagate);
	}

	void setTime(Key key, time_t time, bool propagate = true)
	{
		setTime(Derived::toCode(key), time, propagate);
	}

	void setTime(Point coord, time_t time, bool propagate = true, depth_t depth = 0)
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

	[[nodiscard]] constexpr PropagationCriteria timePropagationCriteria() const noexcept
	{
		return prop_criteria_;
	}

	constexpr void setTimePropagationCriteria(PropagationCriteria prop_criteria,
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

	TimeMap() = default;

	TimeMap(TimeMap const& other) = default;

	TimeMap(TimeMap&& other) = default;

	template <class Derived2>
	TimeMap(TimeMap<Derived2> const& other)
	    : prop_criteria_(other.timePropagationCriteria())
	{
	}

	//
	// Assignment operator
	//

	TimeMap& operator=(TimeMap const& rhs) = default;

	TimeMap& operator=(TimeMap&& rhs) = default;

	template <class Derived2>
	TimeMap& operator=(TimeMap<Derived2> const& rhs)
	{
		prop_criteria_ = rhs.timePropagationCriteria();
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

	void initRoot() { derived().root().setTimeIndex(derived().rootIndex(), 0); }

	//
	// Update node
	//

	template <std::size_t N, class T>
	void updateNode(TimeNode<N>& node, index_field_t const indices, T const& children)
	{
		if constexpr (1 == N) {
			switch (prop_criteria_) {
				case PropagationCriteria::MIN:
					setTime(node, minTime(children));
					break;
				case PropagationCriteria::MAX:
					setTime(node, maxTime(children));
					break;
				case PropagationCriteria::MEAN:
					setTime(node, averageTime(children));
					break;
			}
		} else {
			for (index_t index = 0; children.size() != index; ++index) {
				if ((indices >> index) & index_field_t(1)) {
					switch (prop_criteria_) {
						case PropagationCriteria::MIN:
							setTime(node, index, minTime(children[index]));
							break;
						case PropagationCriteria::MAX:
							setTime(node, index, maxTime(children[index]));
							break;
						case PropagationCriteria::MEAN:
							setTime(node, index, averageTime(children[index]));
							break;
					}
				}
			}
		}
	}

	//
	// Min child time
	//

	template <class T>
	[[nodiscard]] constexpr time_t minTime(T const& nodes) const
	{
		time_t min = std::numeric_limits<time_t>::max();
		for (auto const& node : nodes) {
			min = std::min(min, node.time[0]);
		}
		return min;
	}

	[[nodiscard]] constexpr time_t minTime(TimeNode<8> const& node) const
	{
		return minTime(std::cbegin(node.time), std::cend(node.time));
	}

	template <class InputIt>
	[[nodiscard]] constexpr time_t minTime(InputIt first, InputIt last) const
	{
		time_t min = std::numeric_limits::max();
		for (; first != last; ++first) {
			min = std::min(min, *first);
		}
		return min;
	}

	//
	// Max child time
	//

	template <class T>
	[[nodiscard]] constexpr time_t maxTime(T const& nodes) const
	{
		time_t max = std::numeric_limits<time_t>::lowest();
		for (auto const& node : nodes) {
			max = std::max(max, node.time[0]);
		}
		return max;
	}

	[[nodiscard]] constexpr time_t maxTime(TimeNode<8> const& node) const
	{
		return maxTime(std::cbegin(node.time), std::cend(node.time));
	}

	template <class InputIt>
	[[nodiscard]] constexpr time_t maxTime(InputIt first, InputIt last) const
	{
		time_t max = std::numeric_limits::lowest();
		for (; first != last; ++first) {
			max = std::max(max, *first);
		}
		return max;
	}

	//
	// Average child time
	//

	template <class T>
	[[nodiscard]] static constexpr time_t averageTime(T const& nodes)
	{
		return std::accumulate(
		           std::cbegin(nodes), std::cend(nodes), 0.0,
		           [](double cur, TimeNode<1> node) { return cur + node.time[0]; }) /
		       double(nodes.size());
	}

	[[nodiscard]] static constexpr time_t averageTime(TimeNode<8> const& node)
	{
		return averageTime(std::cbegin(node.time), std::cend(node.time));
	}

	template <class InputIt>
	[[nodiscard]] static constexpr time_t averageTime(InputIt first, InputIt last)
	{
		return std::reduce(first, last, 0.0) / double(std::distance(first, last));
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr DataIdentifier dataIdentifier() noexcept
	{
		return DataIdentifier::TIME;
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
		return node_type::timeSize();
	}

	template <class OutputIt>
	void readNodes(std::istream& in, OutputIt first, std::size_t num_nodes)
	{
		uint8_t n;
		in.read(reinterpret_cast<char*>(&n), sizeof(n));
		num_nodes *= n;

		auto data = std::make_unique<time_t[]>(num_nodes);
		in.read(reinterpret_cast<char*>(data.get()),
		        num_nodes * sizeof(typename decltype(data)::element_type));

		auto const d = data.get();
		if constexpr (1 == numData<OutputIt>()) {
			if (1 == n) {
				for (std::size_t i = 0; i != num_nodes; ++first, ++i) {
					first->node.time[0] = *(d + i);
				}
			} else {
				auto const prop_criteria = prop_criteria_;
				for (std::size_t i = 0; i != num_nodes; ++first, i += 8) {
					switch (prop_criteria) {
						case PropagationCriteria::MIN:
							first->node.time[0] = minTime(d + i, d + i + 8);
							break;
						case PropagationCriteria::MAX:
							first->node.time[0] = maxTime(d + i, d + i + 8);
							break;
						case PropagationCriteria::MEAN:
							first->node.time[0] = averageTime(d + i, d + i + 8);
							break;
					}
				}
			}
		} else {
			if (1 == n) {
				for (std::size_t i = 0; i != num_nodes; ++first, ++i) {
					if (std::numeric_limits<index_field_t>::max() == first->index_field) {
						first->node.time.fill(*(d + i));
					} else {
						for (std::size_t index = 0; first->node.red.size() != index; ++index) {
							if ((first.index_field >> index) & index_field_t(1)) {
								first->node.time[index] = *(d + i);
							}
						}
					}
				}
			} else {
				for (std::size_t i = 0; i != num_nodes; ++first, i += 8) {
					if (std::numeric_limits<index_field_t>::max() == first->index_field) {
						std::copy(d + i, d + i + 8, first->node.time.data());
					} else {
						for (index_t index = 0; first->node.time.size() != index; ++i, ++index) {
							if ((first.index_field >> index) & index_field_t(1)) {
								first->node.time[index] = *(d + i + index);
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

		auto data = std::make_unique<time_t[]>(num_nodes);
		auto d = data.get();
		for (std::size_t i = 0; i != num_nodes; ++first, i += n) {
			std::copy(std::cbegin(first->node.time), std::cend(first->node.time), d + i);
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

#endif  // UFO_MAP_TIME_MAP_BASE_H