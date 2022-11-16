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
class ReflectionMapBase
{
 public:
	//
	// Get hits
	//

	[[nodiscard]] count_t hits(Node node) const
	{
		return derived().leafNode(node).hits[node.index()];
	}

	[[nodiscard]] count_t hits(Code code) const
	{
		auto [node, depth] = derived().leafNodeAndDepth(code);
		return node.hits[code.index(depth)];
	}

	[[nodiscard]] count_t hits(Key key) const { return hits(derived().toCode(key)); }

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
		    node, [hits](auto& node, index_t index) { node.hits[index] = hits; },
		    [hits](auto& node) { node.hits.fill(hits); }, propagate);
	}

	void setHits(Code code, count_t hits, bool propagate = true)
	{
		derived().apply(
		    code, [hits](auto& node, index_t index) { node.hits[index] = hits; },
		    [hits](auto& node) { node.hits.fill(hits); }, propagate);
	}

	void setHits(Key key, count_t hits, bool propagate = true)
	{
		setHits(derived().toCode(key), hits, propagate);
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
		    node, [inc](auto& node, index_t index) { node.hits[index] += inc; },
		    [inc](auto& node) {
			    for (auto& h : node.hits) {
				    h += inc;
			    }
		    },
		    propagate);
	}

	void incrementHits(Code code, count_t inc, bool propagate = true)
	{
		derived().apply(
		    code, [inc](auto& node, index_t index) { node.hits[index] += inc; },
		    [inc](auto& node) {
			    for (auto& h : node.hits) {
				    h += inc;
			    }
		    },
		    propagate);
	}

	void incrementHits(Key key, count_t inc, bool propagate = true)
	{
		incrementHits(derived().toCode(key), inc, propagate);
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
		    node, [dec](auto& node, index_t index) { node.decrementHits(index, dec); },
		    [dec](auto& node) { node.decrementHits(dec); }, propagate);
	}

	void decrementHits(Code code, count_t dec, bool propagate = true)
	{
		derived().apply(
		    code, [dec](auto& node, index_t index) { node.decrementHits(index, dec); },
		    [dec](auto& node) { node.decrementHits(dec); }, propagate);
	}

	void decrementHits(Key key, count_t dec, bool propagate = true)
	{
		decrementHits(derived().toCode(key), dec, propagate);
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
	// Update hits
	//

	// TODO: Implement

	//
	// Get misses
	//

	[[nodiscard]] count_t misses(Node node) const
	{
		return derived().leafNode(node).misses(node.index());
	}

	[[nodiscard]] count_t misses(Code code) const
	{
		auto [node, depth] = derived().leafNodeAndDepth(code);
		return node.misses(code.index(depth));
	}

	[[nodiscard]] count_t misses(Key key) const { return misses(derived().toCode(key)); }

	[[nodiscard]] count_t misses(Point coord, depth_t depth = 0) const
	{
		return misses(derived().toCode(coord, depth));
	}

	[[nodiscard]] count_t misses(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return misses(derived().toCode(x, y, z, depth));
	}

	//
	// Set misses
	//

	void setMisses(Node node, count_t misses, bool propagate = true)
	{
		derived().apply(
		    node, [misses](auto& node, index_t index) { node.setMisses(index, misses); },
		    [misses](auto& node) { node.setMisses(misses); }, propagate);
	}

	void setMisses(Code code, count_t misses, bool propagate = true)
	{
		derived().apply(
		    code, [misses](auto& node, index_t index) { node.setMisses(index, misses); },
		    [misses](auto& node) { node.setMisses(misses); }, propagate);
	}

	void setMisses(Key key, count_t misses, bool propagate = true)
	{
		setMisses(derived().toCode(key), misses, propagate);
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
		    node, [inc](auto& node, index_t index) { node.incrementMisses(index, inc); },
		    [inc](auto& node) { node.incrementMisses(inc); }, propagate);
	}

	void incrementMisses(Code code, count_t inc, bool propagate = true)
	{
		derived().apply(
		    code, [inc](auto& node, index_t index) { node.incrementMisses(index, inc); },
		    [inc](auto& node) { node.incrementMisses(inc); }, propagate);
	}

	void incrementMisses(Key key, count_t inc, bool propagate = true)
	{
		incrementMisses(derived().toCode(key), inc, propagate);
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
		    node, [dec](auto& node, index_t index) { node.decrementMisses(index, dec); },
		    [dec](auto& node) { node.decrementMisses(dec); }, propagate);
	}

	void decrementMisses(Code code, count_t dec, bool propagate = true)
	{
		derived().apply(
		    code, [dec](auto& node, index_t index) { node.decrementMisses(index, dec); },
		    [dec](auto& node) { node.decrementMisses(dec); }, propagate);
	}

	void decrementMisses(Key key, count_t dec, bool propagate = true)
	{
		decrementMisses(derived().toCode(key), dec, propagate);
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
	// Update misses
	//

	// TODO: Implement

	//
	// Get reflectiveness
	//

	[[nodiscard]] double reflectiveness(Node node) const
	{
		auto const& n = derived().leafNode(node);
		auto const index = node.index();
		double const hits = n.hits[index];
		double const misses = n.misses[index];
		return hits / (hits + misses);
	}

	[[nodiscard]] double reflectiveness(Code code) const
	{
		auto [node, depth] = derived().leafNodeAndDepth(code);
		auto const index = code.index(depth);
		double const hits = node.hits[index];
		double const misses = node.misses[index];
		return hits / (hits + misses);
	}

	[[nodiscard]] double reflectiveness(Key key) const
	{
		return reflectiveness(derived().toCode(key));
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
	// Set reflectiveness
	//

	// TODO: Implement

	//
	// Update reflectiveness
	//

	// TODO: Implement

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

	ReflectionMapBase() = default;

	ReflectionMapBase(ReflectionMapBase const& other) = default;

	ReflectionMapBase(ReflectionMapBase&& other) = default;

	template <class Derived2>
	ReflectionMapBase(ReflectionMapBase<Derived2> const& other)
	    : prop_criteria_(other.reflectionPropagationCriteria())
	{
	}

	//
	// Assignment operator
	//

	ReflectionMapBase& operator=(ReflectionMapBase const& rhs) = default;

	ReflectionMapBase& operator=(ReflectionMapBase&& rhs) = default;

	template <class Derived2>
	ReflectionMapBase& operator=(ReflectionMapBase<Derived2> const& rhs)
	{
		prop_criteria_ = rhs.reflectionPropagationCriteria();
		return *this;
	}

	//
	// Swap
	//

	void swap(ReflectionMapBase& other) noexcept
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

	void initRoot()
	{
		derived().root().hits[derived().rootIndex()] = 0;
		derived().root().misses[derived().rootIndex()] = 0;
	}

	//
	// Update node
	//

	template <std::size_t N>
	void updateNode(ReflectionNode<N>& node, index_t index,
	                ReflectionNode<N> const& children)
	{
		switch (reflectionPropagationCriteria()) {
			case PropagationCriteria::MIN:
				node.hits[index] = min(children.hits);
				node.misses[index] = min(children.misses);
				return;
			case PropagationCriteria::MAX:
				node.hits[index] = max(children.hits);
				node.misses[index] = max(children.misses);
				return;
			case PropagationCriteria::MEAN:
				node.hits[index] = mean(children.hits);
				node.misses[index] = mean(children.misses);
				return;
		}
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
		using value_type = typename std::iterator_traits<InputIt>::value_type;
		using node_type = typename value_type::node_type;
		return node_type::reflectionSize();
	}

	template <class InputIt>
	std::size_t serializedSize(InputIt first, InputIt last) const
	{
		return std::iterator_traits<InputIt>::value_type::reflectionSize() *
		       std::distance(first, last) * 2 * sizeof(count_t);
	}

	template <class OutputIt>
	void readNodes(ReadBuffer& in, OutputIt first, OutputIt last)
	{
		for (; first != last; ++first) {
			if (first->index_field.all()) {
				in.read(first->node.hits.data(),
				        first->node.hits.size() *
				            sizeof(typename decltype(first->node.hits)::value_type));
				in.read(first->node.misses.data(),
				        first->node.misses.size() *
				            sizeof(typename decltype(first->node.misses)::value_type));
			} else {
				decltype(first->node.hits) hits;
				decltype(first->node.misses) misses;
				in.read(hits.data(), hits.size() * sizeof(typename decltype(hits)::value_type));
				in.read(misses.data(),
				        misses.size() * sizeof(typename decltype(misses)::value_type));
				for (index_t i = 0; hits.size() != i; ++i) {
					if (first->index_field[i]) {
						first->node.hits[i] = hits[i];
						first->node.misses[i] = misses[i];
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
			out.write(first->hits.data(),
			          first->hits.size() * sizeof(typename decltype(first->hits)::value_type));
			out.write(
			    first->misses.data(),
			    first->misses.size() * sizeof(typename decltype(first->misses)::value_type));
		}
	}

 protected:
	// Propagation criteria
	PropagationCriteria prop_criteria_ = PropagationCriteria::MAX;
};
}  // namespace ufo::map

#endif  // UFO_MAP_REFLECTION_MAP_H