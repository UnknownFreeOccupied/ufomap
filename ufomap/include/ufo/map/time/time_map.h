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

#ifndef UFO_MAP_TIME_MAP_H
#define UFO_MAP_TIME_MAP_H

// UFO
#include <ufo/algorithm/algorithm.h>
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

	[[nodiscard]] constexpr time_t time(Node node) const
	{
		return derived().leafNode(node).time(node.index());
	}

	[[nodiscard]] time_t time(Code code) const
	{
		auto [n, d] = derived().leafNodeAndDepth(code);
		return n.time(code.index(d));
	}

	[[nodiscard]] time_t time(Key key) const { return time(derived().toCode(key)); }

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
		    node, [time](auto& node, index_t index) { node.setTime(index, time); },
		    [time](auto& node) { node.setTime(time); }, propagate);
	}

	void setTime(Code code, time_t time, bool propagate = true)
	{
		derived().apply(
		    code, [time](auto& node, index_t index) { node.setTime(index, time); },
		    [time](auto& node) { node.setTime(time); }, propagate);
	}

	void setTime(Key key, time_t time, bool propagate = true)
	{
		setTime(derived().toCode(key), time, propagate);
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
	// Swap
	//

	constexpr void swap(CountMap& other) noexcept
	{
		std::swap(prop_criteria_, other.prop_criteria_);
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

	void initRoot() { derived().root().setTime(derived().rootIndex(), 0); }

	//
	// Update node
	//

	template <std::size_t N, class T>
	void updateNode(TimeNode<N>& node, IndexField const indices, T const& children)
	{
		auto prop = timePropagationCriteria();
		if constexpr (1 == N) {
			auto fun = [](TimeNode<1> const node) { return node.time[0]; };
			switch (prop) {
				case PropagationCriteria::MIN:
					node.time[0] = min(children, fun);
					break;
				case PropagationCriteria::MAX:
					node.time[0] = max(children, fun);
					break;
				case PropagationCriteria::MEAN:
					node.time[0] = mean(children, fun);
					break;
			}
		} else {
			for (std::size_t i = 0; children.size() != i; ++i) {
				if (indices[i]) {
					switch (prop) {
						case PropagationCriteria::MIN:
							node.setTime(index, min(children[i].beginTime(), children[i].endTime()));
							break;
						case PropagationCriteria::MAX:
							node.setTime(index, max(children[i].beginTime(), children[i].endTime()));
							break;
						case PropagationCriteria::MEAN:
							node.setTime(index, mean(children[i].beginTime(), children[i].endTime()));
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
		return DataIdentifier::TIME;
	}

	[[nodiscard]] static constexpr bool canReadData(DataIdentifier identifier) noexcept
	{
		return dataIdentifier() == identifier;
	}

	template <class InputIt>
	[[nodiscard]] static constexpr uint8_t numData() noexcept
	{
		using value_type = typename std::iterator_traits<InputIt>::value_type;
		using node_type = typename value_type::node_type;
		return node_type::timeSize();
	}

	template <class OutputIt>
	void readNodes(std::istream& in, OutputIt first, std::size_t num_nodes)
	{
		std::uint8_t dt;
		in.read(reinterpret_cast<char*>(&dt), sizeof(dt));
		// TODO: Do something with data type

		std::uint8_t n;
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
							first->node.time[0] = min(d + i, d + i + 8);
							break;
						case PropagationCriteria::MAX:
							first->node.time[0] = max(d + i, d + i + 8);
							break;
						case PropagationCriteria::MEAN:
							first->node.time[0] = mean(d + i, d + i + 8);
							break;
					}
				}
			}
		} else {
			if (1 == n) {
				for (std::size_t i = 0; i != num_nodes; ++first, ++i) {
					if (first->index_field.all()) {
						first->node.time.fill(*(d + i));
					} else {
						for (std::size_t index = 0; first->node.time.size() != index; ++index) {
							if (first.index_field[index]) {
								first->node.time[index] = *(d + i);
							}
						}
					}
				}
			} else {
				for (std::size_t i = 0; i != num_nodes; ++first, i += 8) {
					if (first->index_field.all()) {
						std::copy(d + i, d + i + 8, first->node.time.data());
					} else {
						for (index_t index = 0; first->node.time.size() != index; ++i, ++index) {
							if (first.index_field[index]) {
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
		constexpr std::uint8_t const n = numData<InputIt>();
		num_nodes *= n;

		auto data = std::make_unique<time_t[]>(num_nodes);
		auto d = data.get();
		for (std::size_t i = 0; i != num_nodes; ++first, i += n) {
			std::copy(first->node.beginTime(), first->node.endTime(), d + i);
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

#endif  // UFO_MAP_TIME_MAP_H