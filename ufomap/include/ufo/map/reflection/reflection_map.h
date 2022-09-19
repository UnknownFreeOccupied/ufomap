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

#ifndef UFO_MAP_REFLECTION_MAP_H
#define UFO_MAP_REFLECTION_MAP_H

// UFO
#include <ufo/map/reflection/reflection_node.h>
#include <ufo/map/reflection/reflection_predicate.h>
#include <ufo/map/types.h>

// STL
#include <limits>

namespace ufo::map
{
template <class Derived>
class ReflectionMap
{
 public:
	//
	// Get hits
	//

	[[nodiscard]] constexpr count_t hits(Node node) const
	{
		return derived().leafNode(node).hitsIndex(node.index());
	}

	[[nodiscard]] count_t hits(Code code) const
	{
		return derived().leafNode(code).hitsIndex(code.index());
	}

	[[nodiscard]] count_t hits(Key key) const { return hits(Derived::toCode(key)); }

	[[nodiscard]] count_t hits(Point coord, depth_t depth = 0) const
	{
		return hits(derived().toCode(coord, depth));
	}

	[[nodiscard]] count_t hits(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return hits(derived().toCode(x, y, z, depth));
	}

	//
	// Set hits
	//

	void setHits(Node node, count_t hits, bool propagate = true)
	{
		derived().apply(
		    node, [hits](auto& node, index_t const index) { node.setHitsIndex(index, hits); },
		    [hits](auto& node) { node.setHits(hits); }, propagate);
	}

	void setHits(Code code, count_t hits, bool propagate = true)
	{
		derived().apply(
		    code, [hits](auto& node, index_t const index) { node.setHitsIndex(index, hits); },
		    [hits](auto& node) { node.setHits(hits); }, propagate);
	}

	void setHits(Key key, count_t hits, bool propagate = true)
	{
		setHits(Derived::toCode(key), hits, propagate);
	}

	void setHits(Point coord, count_t hits, bool propagate = true, depth_t depth = 0)
	{
		setHits(derived().toCode(coord, depth), hits, propagate);
	}

	void setHits(coord_t x, coord_t y, coord_t z, count_t hits, bool propagate = true,
	             depth_t depth = 0)
	{
		setHits(derived().toCode(x, y, z, depth), hits, propagate);
	}

	//
	// Increment hits
	//

	void incrementHits(Node node, count_t inc, bool propagate = true)
	{
		derived().apply(
		    node,
		    [inc](auto& node, index_t const index) { node.incrementHitsIndex(index, inc); },
		    [inc](auto& node) { node.incrementHits(inc); }, propagate);
	}

	void incrementHits(Code code, count_t inc, bool propagate = true)
	{
		derived().apply(
		    code,
		    [inc](auto& node, index_t const index) { node.incrementHitsIndex(index, inc); },
		    [inc](auto& node) { node.incrementHits(inc); }, propagate);
	}

	void incrementHits(Key key, count_t inc, bool propagate = true)
	{
		incrementHits(Derived::toCode(key), inc, propagate);
	}

	void incrementHits(Point coord, count_t inc, bool propagate = true, depth_t depth = 0)
	{
		incrementHits(derived().toCode(coord, depth), inc, propagate);
	}

	void incrementHits(coord_t x, coord_t y, coord_t z, count_t inc, bool propagate = true,
	                   depth_t depth = 0)
	{
		incrementHits(derived().toCode(x, y, z, depth), inc, propagate);
	}

	//
	// Decrement hits
	//

	void decrementHits(Node node, count_t dec, bool propagate = true)
	{
		derived().apply(
		    node,
		    [dec](auto& node, index_t const index) { node.decrementHitsIndex(index, dec); },
		    [dec](auto& node) { node.decrementHits(dec); }, propagate);
	}

	void decrementHits(Code code, count_t dec, bool propagate = true)
	{
		derived().apply(
		    code,
		    [dec](auto& node, index_t const index) { node.decrementHitsIndex(index, dec); },
		    [dec](auto& node) { node.decrementHits(dec); }, propagate);
	}

	void decrementHits(Key key, count_t dec, bool propagate = true)
	{
		decrementHits(Derived::toCode(key), dec, propagate);
	}

	void decrementHits(Point coord, count_t dec, bool propagate = true, depth_t depth = 0)
	{
		decrementHits(derived().toCode(coord, depth), dec, propagate);
	}

	void decrementHits(coord_t x, coord_t y, coord_t z, count_t dec, bool propagate = true,
	                   depth_t depth = 0)
	{
		decrementHits(derived().toCode(x, y, z, depth), dec, propagate);
	}

	//
	// Get misses
	//

	[[nodiscard]] constexpr count_t hits(Node node) const
	{
		return derived().leafNode(node).hitsIndex(node.index());
	}

	[[nodiscard]] count_t hits(Code code) const
	{
		return derived().leafNode(code).hitsIndex(code.index());
	}

	[[nodiscard]] count_t hits(Key key) const { return hits(Derived::toCode(key)); }

	[[nodiscard]] count_t hits(Point coord, depth_t depth = 0) const
	{
		return hits(derived().toCode(coord, depth));
	}

	[[nodiscard]] count_t hits(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return hits(derived().toCode(x, y, z, depth));
	}

	//
	// Set hits
	//

	void setMisses(Node node, count_t misses, bool propagate = true)
	{
		derived().apply(
		    node,
		    [misses](auto& node, index_t const index) { node.setMissesIndex(index, misses); },
		    [misses](auto& node) { node.setMisses(misses); }, propagate);
	}

	void setMisses(Code code, count_t misses, bool propagate = true)
	{
		derived().apply(
		    code,
		    [misses](auto& node, index_t const index) { node.setMissesIndex(index, misses); },
		    [misses](auto& node) { node.setMisses(misses); }, propagate);
	}

	void setMisses(Key key, count_t misses, bool propagate = true)
	{
		setMisses(Derived::toCode(key), misses, propagate);
	}

	void setMisses(Point coord, count_t misses, bool propagate = true, depth_t depth = 0)
	{
		setMisses(derived().toCode(coord, depth), misses, propagate);
	}

	void setMisses(coord_t x, coord_t y, coord_t z, count_t misses, bool propagate = true,
	               depth_t depth = 0)
	{
		setMisses(derived().toCode(x, y, z, depth), misses, propagate);
	}

	//
	// Increment misses
	//

	void incrementMisses(Node node, count_t inc, bool propagate = true)
	{
		derived().apply(
		    node,
		    [inc](auto& node, index_t const index) { node.incrementMissesIndex(index, inc); },
		    [inc](auto& node) { node.incrementMisses(inc); }, propagate);
	}

	void incrementMisses(Code code, count_t inc, bool propagate = true)
	{
		derived().apply(
		    code,
		    [inc](auto& node, index_t const index) { node.incrementMissesIndex(index, inc); },
		    [inc](auto& node) { node.incrementMisses(inc); }, propagate);
	}

	void incrementMisses(Key key, count_t inc, bool propagate = true)
	{
		incrementMisses(Derived::toCode(key), inc, propagate);
	}

	void incrementMisses(Point coord, count_t inc, bool propagate = true, depth_t depth = 0)
	{
		incrementMisses(derived().toCode(coord, depth), inc, propagate);
	}

	void incrementMisses(coord_t x, coord_t y, coord_t z, count_t inc,
	                     bool propagate = true, depth_t depth = 0)
	{
		incrementMisses(derived().toCode(x, y, z, depth), inc, propagate);
	}

	//
	// Decrement misses
	//

	void decrementMisses(Node node, count_t dec, bool propagate = true)
	{
		derived().apply(
		    node,
		    [dec](auto& node, index_t const index) { node.decrementMissesIndex(index, dec); },
		    [dec](auto& node) { node.decrementMisses(dec); }, propagate);
	}

	void decrementMisses(Code code, count_t dec, bool propagate = true)
	{
		derived().apply(
		    code,
		    [dec](auto& node, index_t const index) { node.decrementMissesIndex(index, dec); },
		    [dec](auto& node) { node.decrementMisses(dec); }, propagate);
	}

	void decrementMisses(Key key, count_t dec, bool propagate = true)
	{
		decrementMisses(Derived::toCode(key), dec, propagate);
	}

	void decrementMisses(Point coord, count_t dec, bool propagate = true, depth_t depth = 0)
	{
		decrementMisses(derived().toCode(coord, depth), dec, propagate);
	}

	void decrementMisses(coord_t x, coord_t y, coord_t z, count_t dec,
	                     bool propagate = true, depth_t depth = 0)
	{
		decrementMisses(derived().toCode(x, y, z, depth), dec, propagate);
	}

	//
	// Get reflectiveness
	//

	[[nodiscard]] constexpr double reflectiveness(Node node) const
	{
		auto const& n = derived().leafNode(node);
		double const hits = n.hitsIndex(node.index());
		double const misses = n.missesIndex(node.index());
		return hits / (hits + misses);
	}

	[[nodiscard]] double reflectiveness(Code code) const
	{
		auto const& n = derived().leafNode(code);
		double const hits = n.hitsIndex(code.index());
		double const misses = n.missesIndex(code.index());
		return hits / (hits + misses);
	}

	[[nodiscard]] double reflectiveness(Key key) const
	{
		return reflectiveness(Derived::toCode(key));
	}

	[[nodiscard]] double reflectiveness(Point coord, depth_t depth = 0) const
	{
		return reflectiveness(derived().toCode(coord, depth));
	}

	[[nodiscard]] double reflectiveness(coord_t x, coord_t y, coord_t z,
	                                    depth_t depth = 0) const
	{
		return reflectiveness(derived().toCode(x, y, z, depth));
	}

	//
	// Propagation criteria
	//

	[[nodiscard]] constexpr PropagationCriteria reflectionPropagationCriteria()
	    const noexcept
	{
		return prop_criteria_;
	}

	constexpr void setReflectionPropagationCriteria(PropagationCriteria prop_criteria,
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

	ReflectionMap() = default;

	ReflectionMap(ReflectionMap const& other) = default;

	ReflectionMap(ReflectionMap&& other) = default;

	template <class Derived2>
	ReflectionMap(ReflectionMap<Derived2> const& other)
	    : prop_criteria_(other.reflectionPropagationCriteria())
	{
	}

	//
	// Assignment operator
	//

	ReflectionMap& operator=(ReflectionMap const& rhs) = default;

	ReflectionMap& operator=(ReflectionMap&& rhs) = default;

	template <class Derived2>
	ReflectionMap& operator=(ReflectionMap<Derived2> const& rhs)
	{
		prop_criteria_ = rhs.reflectionPropagationCriteria();
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
		derived().root().setHitsIndex(derived().rootIndex(), 0);
		derived().root().setMissesIndex(derived().rootIndex(), 0);
	}

	//
	// Update node
	//

	template <std::size_t N, class T>
	void updateNode(ReflectionNode<N>& node, index_field_t const indices, T const& children)
	{
		if constexpr (1 == N) {
			switch (prop_criteria_) {
				case PropagationCriteria::MIN:
					node.hits[0] = min(children, [](auto const& e) { return e.hits[0]; });
					node.misses[0] = min(children, [](auto const& e) { return e.misses[0]; });
					break;
				case PropagationCriteria::MAX:
					node.hits[0] = max(children, [](auto const& e) { return e.hits[0]; });
					node.misses[0] = max(children, [](auto const& e) { return e.misses[0]; });
					break;
				case PropagationCriteria::MEAN:
					node.hits[0] = average(children, [](auto const& e) { return e.hits[0]; });
					node.misses[0] = average(children, [](auto const& e) { return e.misses[0]; });
					break;
				case PropagationCriteria::SUM:
					node.hits[0] = sum(children, [](auto const& e) { return e.hits[0]; });
					node.misses[0] = sum(children, [](auto const& e) { return e.misses[0]; });
					break;
			}
		} else {
			for (index_t index = 0; children.size() != index; ++index) {
				if ((indices >> index) & index_field_t(1)) {
					switch (prop_criteria_) {
						case PropagationCriteria::MIN:
							node.hits[index] = min(children[index].hits);
							node.misses[index] = min(children[index].misses);
							break;
						case PropagationCriteria::MAX:
							node.hits[index] = max(children[index].hits);
							node.misses[index] = max(children[index].misses);
							break;
						case PropagationCriteria::MEAN:
							node.hits[index] = average(children[index].hits);
							node.misses[index] = average(children[index].misses);
							break;
						case PropagationCriteria::SUM:
							node.hits[index] = sum(children[index].hits);
							node.misses[index] = sum(children[index].misses);
							break;
					}
				}
			}
		}
	}

	//
	// Min
	//

	template <class C>
	count_t min(C const x)
	{
		return min(std::cbegin(x), std::cend(x));
	}

	template <class InputIt>
	count_t min(InputIt first, InputIt last)
	{
		count_t m = std::numeric_limits<count_t>::max();
		for (; first != last; ++first) {
			m = std::min(m, *first);
		}
		return m
	}

	template <class C, class UnaryFunction>
	count_t min(C const& x, UnaryFunction f)
	{
		count_t m = std::numeric_limits<count_t>::max();
		for (auto const& e : x) {
			m = std::min(m, f(e));
		}
		return m;
	}

	//
	// Max
	//

	template <class C>
	count_t max(C const x)
	{
		return max(std::cbegin(x), std::cend(x));
	}

	template <class InputIt>
	count_t max(InputIt first, InputIt last)
	{
		count_t m = std::numeric_limits<count_t>::lowest();
		for (; first != last; ++first) {
			m = std::max(m, *first);
		}
		return m
	}

	template <class C, class UnaryFunction>
	count_t max(C const& x, UnaryFunction f)
	{
		count_t m = std::numeric_limits<count_t>::lowest();
		for (auto const& e : x) {
			m = std::max(m, f(e));
		}
		return m;
	}

	//
	// Average
	//

	template <class C>
	count_t average(C const x)
	{
		return average(std::cbegin(x), std::cend(x));
	}

	template <class InputIt>
	count_t average(InputIt first, InputIt last)
	{
		return std::reduce(first, last, 0.0) / double(std::distance(first, last));
	}

	template <class C, class UnaryFunction>
	count_t average(C const& x, UnaryFunction f)
	{
		return sum(x, f) / double(x.size());
	}

	//
	// Sum
	//

	template <class C>
	count_t sum(C const x)
	{
		return sum(std::cbegin(x), std::cend(x));
	}

	template <class InputIt>
	count_t sum(InputIt first, InputIt last)
	{
		return std::reduce(first, last);
	}

	template <class C, class UnaryFunction>
	count_t sum(C const& x, UnaryFunction f)
	{
		count_t s = 0;
		for (auto const& e : x) {
			s += f(e);
		}
		return s;
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr DataIdentifier dataIdentifier() noexcept
	{
		return DataIdentifier::REFLECTION;
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
		return node_type::reflectionSize();
	}

	template <class OutputIt>
	void readNodes(std::istream& in, OutputIt first, std::size_t num_nodes)
	{
		uint8_t n;
		in.read(reinterpret_cast<char*>(&n), sizeof(n));
		std::size_t const s = 2 * n;
		num_nodes *= s;

		auto data = std::make_unique<count_t[]>(num_nodes);
		in.read(reinterpret_cast<char*>(data.get()),
		        num_nodes * sizeof(typename decltype(data)::element_type));

		auto const d = data.get();
		if constexpr (1 == numData<OutputIt>()) {
			if (1 == n) {
				for (std::size_t i = 0; i != num_nodes; ++first, i += 2) {
					first->node.hits[0] = *(d + i);
					first->node.misses[0] = *(d + i + 1);
				}
			} else {
				auto const prop_criteria = prop_criteria_;
				for (std::size_t i = 0; i != num_nodes; ++first, i += 16) {
					switch (prop_criteria) {
						case PropagationCriteria::MIN:
							first->node.hits[0] = min(d + i, d + i + 8);
							first->node.misses[0] = min(d + i + 8, d + i + 16);
							break;
						case PropagationCriteria::MAX:
							first->node.hits[0] = max(d + i, d + i + 8);
							first->node.misses[0] = max(d + i + 8, d + i + 16);
							break;
						case PropagationCriteria::MEAN:
							first->node.hits[0] = average(d + i, d + i + 8);
							first->node.misses[0] = average(d + i + 8, d + i + 16);
							break;
						case PropagationCriteria::SUM:
							first->node.hits[0] = sum(d + i, d + i + 8);
							first->node.misses[0] = sum(d + i + 8, d + i + 16);
							break;
					}
				}
			}
		} else {
			if (1 == n) {
				for (std::size_t i = 0; i != num_nodes; ++first, i += 2) {
					if (std::numeric_limits<index_field_t>::max() == first->index_field) {
						first->node.hits.fill(*(d + i));
						first->node.misses.fill(*(d + i + 1));
					} else {
						for (std::size_t index = 0; first->node.hits.size() != index; ++index) {
							if ((first.index_field >> index) & index_field_t(1)) {
								first->node.hits[index] = *(d + i);
								first->node.misses[index] = *(d + i + 1);
							}
						}
					}
				}
			} else {
				for (std::size_t i = 0; i != num_nodes; ++first, i += 16) {
					if (std::numeric_limits<index_field_t>::max() == first->index_field) {
						std::copy(d + i, d + i + 8, first->node.hits.data());
						std::copy(d + i + 8, d + i + 16, first->node.misses.data());
					} else {
						for (index_t index = 0; first->node.hits.size() != index; ++i, ++index) {
							if ((first.index_field >> index) & index_field_t(1)) {
								first->node.hits[index] = *(d + i + index);
								first->node.misses[index] = *(d + i + index + 8);
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
		constexpr std::size_t const s = 2 * n;
		num_nodes *= s;

		auto data = std::make_unique<count_t[]>(num_nodes);
		auto d = data.get();
		for (std::size_t i = 0; i != num_nodes; ++first, i += s) {
			std::copy(std::cbegin(first->node.hits), std::cend(first->node.hits), d + i);
			std::copy(std::cbegin(first->node.misses), std::cend(first->node.misses),
			          d + i + n);
		}

		out.write(reinterpret_cast<char const*>(&n), sizeof(n));
		out.write(reinterpret_cast<char const*>(data.get()),
		          num_nodes * sizeof(typename decltype(data)::element_type));
	}

 protected:
	// Propagation criteria
	PropagationCriteria prop_criteria_ = PropagationCriteria::SUM;
};
}  // namespace ufo::map

#endif  // UFO_MAP_REFLECTION_MAP_H