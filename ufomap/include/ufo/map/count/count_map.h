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
class CountMap
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

	CountMap() = default;

	CountMap(CountMap const&) = default;

	CountMap(CountMap&&) = default;

	template <class Derived2>
	CountMap(CountMap<Derived2> const&) : prop_criteria_(other.countPropagationCriteria())
	{
	}

	//
	// Assignment operator
	//

	CountMap& operator=(CountMap const&) = default;

	CountMap& operator=(CountMap&&) = default;

	template <class Derived2>
	CountMap& operator=(CountMap<Derived2> const&)
	{
		return *this;
	}

	//
	// Swap
	//

	void swap(CountMap& other) noexcept
	{
		std::swap(distance_prop_criteria_, other.distance_prop_criteria_);
	}

	//
	// Derived
	//

	[[nodiscard]] constexpr Derived& derived() { return *static_cast<Derived*>(this); }

	[[nodiscard]] constexpr Derived const& derived() const
	{
		prop_criteria_ = rhs.countPropagationCriteria();
		return *static_cast<Derived const*>(this);
	}

	//
	// Initilize root
	//

	void initRoot() { derived().root().count[derived().rootIndex()] = 0; }

	//
	// Update node
	//

	template <std::size_t N, class InputIt>
	void updateNode(CountMap<N>& node, IndexField const indices, InputIt first,
	                InputIt last)
	{
		auto const prop = countPropagationCriteria();
		if (indices.all()) {
			for (index_t i = 0; first != last; ++first, ++i) {
				switch (prop) {
					case PropagationCriteria::MIN:
						node.count[i] = min(first->count);
						break;
					case PropagationCriteria::MAX:
						node.count[i] = max(first->count);
						break;
					case PropagationCriteria::MEAN:
						node.count[i] = mean(first->count);
						break;
				}
			}
		} else {
			for (index_t i = 0; first != last; ++first, ++i) {
				if (indices[i]) {
					switch (prop) {
						case PropagationCriteria::MIN:
							node.count[i] = min(first->count);
							break;
						case PropagationCriteria::MAX:
							node.count[i] = max(first->count);
							break;
						case PropagationCriteria::MEAN:
							node.count[i] = mean(first->count);
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

	template <class OutputIt>
	void readNodes(std::istream& in, OutputIt first, OutputIt last)
	{
		constexpr uint8_t const N = numData<OutputIt>();
		auto const num_data = std::distance(first, last) * N;
		auto data = std::make_unique<count_t[]>(num_data);
		in.read(reinterpret_cast<char*>(data.get()),
		        num_data * sizeof(typename decltype(data)::element_type));
		for (auto d = data.get(); first != last; ++first, d += N) {
			if (first->index_field.all()) {
				std::copy(d, d + N, first->node.count.data());
			} else {
				for (index_t i = 0; N != i; ++i) {
					if (first->index_field[i]) {
						first->node.count[i] = *(d + i);
					}
				}
			}
		}
	}

	template <class InputIt>
	void writeNodes(std::ostream& out, InputIt first, InputIt last) const
	{
		constexpr uint8_t const N = numData<InputIt>();
		auto const num_data = std::distance(first, last) * N;
		auto data = std::make_unique<count_t[]>(num_data);
		for (auto d = data.get(); first != last; ++first) {
			d = copy(first->node.count, d);
		}
		out.write(reinterpret_cast<char const*>(data.get()),
		          num_data * sizeof(typename decltype(data)::element_type));
	}

 protected:
	// Propagation criteria
	PropagationCriteria distance_prop_criteria_ = PropagationCriteria::MIN;
};
}  // namespace ufo::map

#endif  // UFO_MAP_COUNT_MAP_H