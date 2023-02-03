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

#ifndef UFO_MAP_COUNT_MAP_H
#define UFO_MAP_COUNT_MAP_H

// UFO
#include <ufo/algorithm/algorithm.h>
#include <ufo/map/count/count_predicate.h>

// STL
#include <algorithm>
#include <array>
#include <cstdint>
#include <deque>
#include <iostream>
#include <type_traits>
#include <utility>

namespace ufo::map
{
template <class Derived, std::size_t N>
class CountMap
{
 public:
	//
	// Get count
	//

	[[nodiscard]] count_t count(Node node) const
	{
		auto [index, offset] = derived().indexAndOffset(node);
		return count_[index][offset];
	}

	[[nodiscard]] count_t count(Code code) const
	{
		auto [index, offset] = derived().indexAndOffset(code);
		return count_[index][offset];
	}

	[[nodiscard]] count_t count(Key key) const { return count(derived().toCode(key)); }

	[[nodiscard]] count_t count(Point coord, depth_t depth = 0) const
	{
		return count(derived().toCode(coord, depth));
	}

	[[nodiscard]] count_t count(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return count(derived().toCode(x, y, z, depth));
	}

	//
	// Set count
	//

	Node setCount(Node node, count_t count, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [&count_, count](index_t index, index_t offset) {
			    count_[index][offset] = count;
		    },
		    [&count_, count](index_t index) { count_[index].fill(count); }, propagate);
	}

	Node setCount(Code code, count_t count, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [&count_, count](index_t index, index_t offset) {
			    count_[index][offset] = count;
		    },
		    [&count_, count](index_t index) { count_[index].fill(count); }, propagate);
	}

	Node setCount(Key key, count_t count, bool propagate = true)
	{
		return setCount(derived().toCode(key), count, propagate);
	}

	Node setCount(Point coord, count_t count, bool propagate = true, depth_t depth = 0)
	{
		return setCount(derived().toCode(coord, depth), count, propagate);
	}

	Node setCount(coord_t x, coord_t y, coord_t z, count_t count, bool propagate = true,
	              depth_t depth = 0)
	{
		return setCount(derived().toCode(x, y, z, depth), count, propagate);
	}

	//
	// Propagation criteria
	//

	[[nodiscard]] constexpr PropagationCriteria countPropagationCriteria() const noexcept
	{
		return prop_criteria_;
	}

	void setCountPropagationCriteria(PropagationCriteria prop_criteria,
	                                 bool propagate = true)
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

	CountMap() = default;

	CountMap(CountMap const&) = default;

	CountMap(CountMap&&) = default;

	template <class Derived2>
	CountMap(CountMap<Derived2, N> const& other)
	    : count_(other.count_), prop_criteria_(other.countPropagationCriteria())
	{
	}

	//
	// Assignment operator
	//

	CountMap& operator=(CountMap const&) = default;

	CountMap& operator=(CountMap&&) = default;

	template <class Derived2>
	CountMap& operator=(CountMap<Derived2, N> const& rhs)
	{
		count_ = rhs.count_;
		prop_criteria_ = rhs.countPropagationCriteria();
		return *this;
	}

	//
	// Swap
	//

	void swap(CountMap& other) noexcept
	{
		std::swap(count_, other.count_);
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

	void initRoot() { count_[derived().rootIndex()][derived().rootOffset()] = 0; }

	//
	// Fill
	//

	void fill(index_t index, index_t parent_index, index_t parent_offset)
	{
		count_[index].fill(count_[parent_index][parent_offset]);
	}

	//
	// Clear
	//

	void clear(index_t index) {}

	//
	// Update node
	//

	void updateNode(index_t index, index_t offset, index_t children_index)
	{
		switch (countPropagationCriteria()) {
			case PropagationCriteria::MIN:
				count_[index][offset] = min(count_[children_index]);
				return;
			case PropagationCriteria::MAX:
				count_[index][offset] = max(count_[children_index]);
				return;
			case PropagationCriteria::MEAN:
				count_[index][offset] = mean(count_[children_index]);
				return;
		}
	}

	//
	// Is collapsible
	//

	[[nodiscard]] constexpr bool isCollapsible(index_t index) const
	{
		return std::all_of(std::begin(count_[index]) + 1, std::end(count_[index]),
		                   [a = count_[index].front()](auto b) { return a == b; });
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr MapType mapType() noexcept { return MapType::COUNT; }

	[[nodiscard]] static constexpr bool canReadData(MapType mt) noexcept
	{
		return mapType() == mt;
	}

	template <class InputIt>
	std::size_t serializedSize(InputIt first, InputIt last) const
	{
		return std::distance(first, last) * N * sizeof(count_t);
	}

	template <class OutputIt>
	void readNodes(ReadBuffer& in, OutputIt first, OutputIt last)
	{
		for (; first != last; ++first) {
			if (first->offsets.all()) {
				in.read(count_[first->index].data(), N * sizeof(count_t));
			} else {
				std::array<count_t, N> count;
				in.read(count.data(), N * sizeof(count_t));
				for (index_t i = 0; N != i; ++i) {
					if (first->offsets[i]) {
						count_[first->index][i] = count[i];
					}
				}
			}
		}
	}

	template <class InputIt>
	void writeNodes(WriteBuffer& out, InputIt first, InputIt last) const
	{
		out.reserve(out.size() + serializedSize(first, last));
		for (; first != last; ++first) {
			out.write(count_[*first].data(), N * sizeof(count_t));
		}
	}

 protected:
	// Data
	std::deque<std::array<count_t, N>> count_;

	// Propagation criteria
	PropagationCriteria prop_criteria_ = PropagationCriteria::MAX;

	template <class Derived2, std::size_t N2>
	friend class CountMap;
};
}  // namespace ufo::map

#endif  // UFO_MAP_COUNT_MAP_H