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
#include <ufo/map/count/count_node.h>
#include <ufo/map/count/count_predicate.h>

// STL
#include <cstdint>
#include <iostream>
#include <type_traits>
#include <utility>

namespace ufo::map
{
template <class Derived>
class CountMapBase
{
 public:
	//
	// Get count
	//

	[[nodiscard]] count_t count(Node node) const
	{
		return derived().leafNode(node).count[node.index()];
	}

	[[nodiscard]] count_t count(Code code) const
	{
		auto [node, depth] = derived().leafNodeAndDepth(code);
		return node.count[code.index(depth)];
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

	void setCount(Node node, count_t count, bool propagate = true)
	{
		derived().apply(
		    node, [count](auto& node, index_t index) { node.count[index] = count; },
		    [count](auto& node) { node.count.fill(count); }, propagate);
	}

	void setCount(Code code, count_t count, bool propagate = true)
	{
		derived().apply(
		    code, [count](auto& node, index_t index) { node.count[index] = count; },
		    [count](auto& node) { node.count.fill(count); }, propagate);
	}

	void setCount(Key key, count_t count, bool propagate = true)
	{
		setCount(derived().toCode(key), count, propagate);
	}

	void setCount(Point coord, count_t count, bool propagate = true, depth_t depth = 0)
	{
		setCount(derived().toCode(coord, depth), count, propagate);
	}

	void setCount(coord_t x, coord_t y, coord_t z, count_t count, bool propagate = true,
	              depth_t depth = 0)
	{
		setCount(derived().toCode(x, y, z, depth), count, propagate);
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

	CountMapBase() = default;

	CountMapBase(CountMapBase const&) = default;

	CountMapBase(CountMapBase&&) = default;

	template <class Derived2>
	CountMapBase(CountMapBase<Derived2> const& other)
	    : prop_criteria_(other.countPropagationCriteria())
	{
	}

	//
	// Assignment operator
	//

	CountMapBase& operator=(CountMapBase const&) = default;

	CountMapBase& operator=(CountMapBase&&) = default;

	template <class Derived2>
	CountMapBase& operator=(CountMapBase<Derived2> const& rhs)
	{
		prop_criteria_ = rhs.countPropagationCriteria();
		return *this;
	}

	//
	// Swap
	//

	void swap(CountMapBase& other) noexcept
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

	void initRoot() { derived().root().count[derived().rootIndex()] = 0; }

	//
	// Update node
	//

	template <std::size_t N>
	void updateNode(CountNode<N>& node, index_t index, CountNode<N> const children)
	{
		switch (countPropagationCriteria()) {
			case PropagationCriteria::MIN:
				node.count[index] = min(children.count);
				return;
			case PropagationCriteria::MAX:
				node.count[index] = max(children.count);
				return;
			case PropagationCriteria::MEAN:
				node.count[index] = mean(children.count);
				return;
		}
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr DataIdentifier dataIdentifier() noexcept
	{
		return DataIdentifier::COUNT;
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
		return node_type::countSize();
	}

	template <class InputIt>
	std::size_t serializedSize(InputIt first, InputIt last) const
	{
		return std::iterator_traits<InputIt>::value_type::countSize() *
		       std::distance(first, last) * sizeof(count_t);
	}

	template <class OutputIt>
	void readNodes(ReadBuffer& in, OutputIt first, OutputIt last)
	{
		for (; first != last; ++first) {
			if (first->index_field.all()) {
				in.read(first->node.count.data(),
				        first->node.count.size() *
				            sizeof(typename decltype(first->node.count)::value_type));
			} else {
				decltype(first->node.count) count;
				in.read(count.data(),
				        count.size() * sizeof(typename decltype(count)::value_type));
				for (index_t i = 0; count.size() != i; ++i) {
					if (first->index_field[i]) {
						first->node.count[i] = count[i];
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
			out.write(
			    first->count.data(),
			    first->count.size() * sizeof(typename decltype(first->count)::value_type));
		}
	}

 protected:
	// Propagation criteria
	PropagationCriteria prop_criteria_ = PropagationCriteria::MAX;
};
}  // namespace ufo::map

#endif  // UFO_MAP_COUNT_MAP_H