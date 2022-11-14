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
class TimeMapBase
{
 public:
	//
	// Get time
	//

	[[nodiscard]] constexpr time_t time(Node node) const
	{
		return derived().leafNode(node).time[node.index()];
	}

	[[nodiscard]] time_t time(Code code) const
	{
		auto [node, depth] = derived().leafNodeAndDepth(code);
		return node.time[code.index(depth)];
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

	Node setTime(Node node, time_t time, bool propagate = true)
	{
		return derived().apply(
		    node, [time](auto& node, index_t index) { node.time[index] = time; },
		    [time](auto& node) { node.time.fill(time); }, propagate);
	}

	Node setTime(Code code, time_t time, bool propagate = true)
	{
		return derived().apply(
		    code, [time](auto& node, index_t index) { node.time[index] = time; },
		    [time](auto& node) { node.time.fill(time); }, propagate);
	}

	Node setTime(Key key, time_t time, bool propagate = true)
	{
		return setTime(derived().toCode(key), time, propagate);
	}

	Node setTime(Point coord, time_t time, bool propagate = true, depth_t depth = 0)
	{
		return setTime(derived().toCode(coord, depth), time, propagate);
	}

	Node setTime(coord_t x, coord_t y, coord_t z, time_t time, bool propagate = true,
	             depth_t depth = 0)
	{
		return setTime(derived().toCode(x, y, z, depth), time, propagate);
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

	TimeMapBase() = default;

	TimeMapBase(TimeMapBase const& other) = default;

	TimeMapBase(TimeMapBase&& other) = default;

	template <class Derived2>
	TimeMapBase(TimeMapBase<Derived2> const& other)
	    : prop_criteria_(other.timePropagationCriteria())
	{
	}

	//
	// Assignment operator
	//

	TimeMapBase& operator=(TimeMapBase const& rhs) = default;

	TimeMapBase& operator=(TimeMapBase&& rhs) = default;

	template <class Derived2>
	TimeMapBase& operator=(TimeMapBase<Derived2> const& rhs)
	{
		prop_criteria_ = rhs.timePropagationCriteria();
		return *this;
	}

	//
	// Swap
	//

	void swap(TimeMapBase& other) noexcept
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

	void initRoot() { derived().root().time[derived().rootIndex()] = 0; }

	//
	// Update node
	//

	template <std::size_t N>
	void updateNode(TimeNode<N>& node, index_t index, TimeNode<N> const children)
	{
		switch (timePropagationCriteria()) {
			case PropagationCriteria::MIN:
				node.time[index] = min(children.time);
				return;
			case PropagationCriteria::MAX:
				node.time[index] = max(children.time);
				return;
			case PropagationCriteria::MEAN:
				node.time[index] = mean(children.time);
				return;
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
	[[nodiscard]] static constexpr std::size_t numData() noexcept
	{
		using value_type = typename std::iterator_traits<InputIt>::value_type;
		using node_type = typename value_type::node_type;
		return node_type::timeSize();
	}

	template <class InputIt>
	std::size_t serializedSize(InputIt first, InputIt last) const
	{
		return std::iterator_traits<InputIt>::value_type::timeSize() *
		       std::distance(first, last) * sizeof(time_t);
	}

	template <class OutputIt>
	void readNodes(std::istream& in, OutputIt first, OutputIt last)
	{
		constexpr auto N = numData<OutputIt>();

		auto size = std::distance(first, last) * N;

		auto data = std::make_unique<time_t[]>(size);
		in.read(reinterpret_cast<char*>(data.get()),
		        size * sizeof(typename decltype(data)::element_type));

		auto const d = data.get();
		for (; first != last; ++first, d += N) {
			if (first->index_field.all()) {
				std::copy(d, d + N, first->node.time.data());
			} else {
				for (index_t i = 0; N != i; ++i) {
					if (first->index_field[i]) {
						first->node.time[i] = *(d + i);
					}
				}
			}
		}
	}

	template <class OutputIt>
	void readNodes(ReadBuffer& in, OutputIt first, OutputIt last)
	{
		constexpr std::size_t const N = numData<OutputIt>();
		for (; first != last; ++first) {
			if (first->index_field.all()) {
				in.read(first->node.time.data(), N * sizeof(time_t));
			} else {
				for (index_t i = 0; N != i; ++i) {
					if (first->index_field[i]) {
						in.read(&first->node.time[i], sizeof(time_t));
					} else
						in.skipRead(sizeof(time_t));
				}
			}
		}
	}

	template <class InputIt>
	void writeNodes(std::ostream& out, InputIt first, InputIt last) const
	{
		auto size = std::distance(first, last) * 8;  // FIXME: numData<InputIt>();

		auto data = std::make_unique<time_t[]>(size);
		auto d = data.get();
		for (; first != last; ++first) {
			d = copy(first->time, d);
		}

		out.write(reinterpret_cast<char const*>(data.get()),
		          size * sizeof(typename decltype(data)::element_type));
	}

	template <class InputIt>
	void writeNodes(WriteBuffer& out, InputIt first, InputIt last) const
	{
		out.reserve(out.size() + serializedSize(first, last));
		for (; first != last; ++first) {
			out.write(first->time.data(), first->time.size() * sizeof(time_t));
		}
	}

 protected:
	// Propagation criteria
	PropagationCriteria prop_criteria_ = PropagationCriteria::MAX;
};
}  // namespace ufo::map

#endif  // UFO_MAP_TIME_MAP_H