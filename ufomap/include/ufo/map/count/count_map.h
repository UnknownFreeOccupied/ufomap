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
		return derived().leafNode(node).count(node.index());
	}

	[[nodiscard]] count_t count(Code code) const
	{
		auto [n, d] = derived().leafNodeAndDepth(code);
		return n.count(code.index(d));
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
		    node, [count](auto& node, index_t index) { node.setCount(index, count); },
		    [count](auto& node) { node.setCount(count); }, propagate);
	}

	void setCount(Code code, count_t count, bool propagate = true)
	{
		derived().apply(
		    code, [count](auto& node, index_t index) { node.setCount(index, count); },
		    [count](auto& node) { node.setCount(count); }, propagate);
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

	CountMap() = default

	    CountMap(CountMap const&) = default;

	CountMap(CountMap&&) = default;

	template <class Derived2>
	CountMap(CountMap<Derived2> const&)
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

	void initRoot() { derived().root().setCount(derived().rootIndex(), 0); }

	//
	// Update node
	//

	template <std::size_t N, class T>
	void updateNode(CountMap<N>& node, IndexField const indices, T const& children)
	{
		auto prop = countPropagationCriteria();
		if constexpr (1 == N) {
			auto fun = [](TimeNode<1> node) { return node.count(0); };
			switch (prop) {
				case PropagationCriteria::MIN:
					node.setCount(min(children, fun));
					break;
				case PropagationCriteria::MAX:
					node.setCount(max(children, fun));
					break;
				case PropagationCriteria::MEAN:
					node.setCount(mean(children, fun));
					break;
			}
		} else {
			for (std::size_t i = 0; children.size() != i; ++i) {
				if (indices[i]) {
					switch (prop) {
						case PropagationCriteria::MIN:
							node.setCount(i, min(children[i].beginCount(), children[i].endCound()));
							break;
						case PropagationCriteria::MAX:
							node.setCount(i, max(children[i].beginCount(), children[i].endCount()));
							break;
						case PropagationCriteria::MEAN:
							node.setCount(i, mean(children[i].beginCount(), children[i].endCount()));
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
	[[nodiscard]] static constexpr std::size_t numData() noexcept
	{
		using value_type = typename std::iterator_traits<InputIt>::value_type;
		using node_type = typename value_type::node_type;
		return node_type::countSize();
	}

	template <class OutputIt>
	void readNodes(std::istream& in, OutputIt first, std::size_t num_nodes)
	{
		std::uint8_t n;
		in.read(reinterpret_cast<char*>(&n), sizeof(n));
		num_nodes *= n;

		auto data = std::make_unique<count_t[]>(num_nodes);
		in.read(reinterpret_cast<char*>(data.get()),
		        num_nodes * sizeof(typename decltype(data)::element_type));

		auto const d = data.get();
		if constexpr (1 == numData<OutputIt>()) {
			if (1 == n) {
				for (std::size_t i = 0; i != num_nodes; ++first, ++i) {
					first->node.setCount(*(d + i));
				}
			} else {
				auto prop = countPropagationCriteria();
				for (std::size_t i = 0; i != num_nodes; ++first, i += 8) {
					switch (prop) {
						case PropagationCriteria::MIN:
							first->node.setCount(min(d + i, d + i + 8));
							break;
						case PropagationCriteria::MAX:
							first->node.setCount(max(d + i, d + i + 8));
							break;
						case PropagationCriteria::MEAN:
							first->node.setCount(mean(d + i, d + i + 8));
							break;
					}
				}
			}
		} else {
			if (1 == n) {
				for (std::size_t i = 0; i != num_nodes; ++first, ++i) {
					if (first->index_field.all()) {
						first->node.setCount(*(d + i));
					} else {
						for (std::size_t index = 0; numData<OutputIt>() != index; ++index) {
							if (first->index_field[index]) {
								first->node.setCount(index, *(d + i));
							}
						}
					}
				}
			} else {
				for (std::size_t i = 0; i != num_nodes; ++first, i += 8) {
					if (first->index_field.all()) {
						std::copy(d + i, d + i + 8, first->node.countData());
					} else {
						for (std::size_t index = 0; numData<OutputIt>() != index; ++index) {
							if (first->index_field[index]) {
								first->node.setCount(index, *(d + i + index));
							}
						}
					}
				}
			}
		}
	}

	template <class InputIt>
	void writeNodes(std::ostream& out, InputIt first, std::size_t num_nodes) const
	{
		constexpr std::uint8_t const n = numData<InputIt>();
		num_nodes *= n;

		auto data = std::make_unique<count_t[]>(num_nodes);
		auto d = data.get();
		for (std::size_t i = 0; i != num_nodes; ++first, i += n) {
			std::copy((first->node.beginCount(), first->node.endCount(), d + i);
		}

		out.write(reinterpret_cast<char const*>(&n), sizeof(n));
		out.write(reinterpret_cast<char const*>(data.get()),
		          num_nodes * sizeof(typename decltype(data)::element_type));
	}

 protected:
	// Propagation criteria
	PropagationCriteria distance_prop_criteria_ = PropagationCriteria::MIN;
};
}  // namespace ufo::map

#endif  // UFO_MAP_COUNT_MAP_H