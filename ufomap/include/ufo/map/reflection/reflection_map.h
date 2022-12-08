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
#include <ufo/map/reflection/reflection.h>
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
	// Get reflection
	//

	[[nodiscard]] Reflection reflection(Node node) const
	{
		return derived().leafNode(node).reflection[derived().dataIndex(node)];
	}

	[[nodiscard]] Reflection reflection(Code code) const
	{
		auto [node, depth] = derived().leafNodeAndDepth(code);
		return node.reflection[code.index(depth)];
	}

	[[nodiscard]] Reflection reflection(Key key) const
	{
		return reflection(derived().toCode(key));
	}

	[[nodiscard]] Reflection reflection(Point coord, depth_t depth = 0) const
	{
		return reflection(derived().toCode(coord, depth));
	}

	[[nodiscard]] Reflection reflection(coord_t x, coord_t y, coord_t z,
	                                    depth_t depth = 0) const
	{
		return reflection(derived().toCode(x, y, z, depth));
	}

	//
	// Set reflection
	//

	void setReflection(Node node, Reflection reflection, bool propagate = true)
	{
		derived().apply(
		    node,
		    [reflection](auto& node, index_t index) { node.reflection[index] = reflection; },
		    [reflection](auto& node) { node.reflection.fill(reflection); }, propagate);
	}

	void setReflection(Code code, Reflection reflection, bool propagate = true)
	{
		derived().apply(
		    code,
		    [reflection](auto& node, index_t index) { node.reflection[index] = reflection; },
		    [reflection](auto& node) { node.reflection.fill(reflection); }, propagate);
	}

	void setReflection(Key key, Reflection reflection, bool propagate = true)
	{
		setReflection(derived().toCode(key), reflection, propagate);
	}

	void setReflection(Point coord, Reflection reflection, bool propagate = true,
	                   depth_t depth = 0)
	{
		setReflection(derived().toCode(coord, depth), reflection, propagate);
	}

	void setReflection(coord_t x, coord_t y, coord_t z, Reflection reflection,
	                   bool propagate = true, depth_t depth = 0)
	{
		setReflection(derived().toCode(x, y, z, depth), reflection, propagate);
	}

	//
	// Get hits
	//

	[[nodiscard]] count_t hits(Node node) const { return reflection(node).hits; }

	[[nodiscard]] count_t hits(Code code) const { return reflection(code).hits; }

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
		    node, [hits](auto& node, index_t index) { node.reflection[index].hits = hits; },
		    [hits](auto& node) {
			    for (auto& r : node.reflection) {
				    r.hits = hits;
			    }
		    },
		    propagate);
	}

	void setHits(Code code, count_t hits, bool propagate = true)
	{
		derived().apply(
		    code, [hits](auto& node, index_t index) { node.reflection[index].hits = hits; },
		    [hits](auto& node) {
			    for (auto& r : node.reflection) {
				    r.hits = hits;
			    }
		    },
		    propagate);
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
		    node, [inc](auto& node, index_t index) { node.reflection[index].hits += inc; },
		    [inc](auto& node) {
			    for (auto& r : node.reflection) {
				    r.hits += inc;
			    }
		    },
		    propagate);
	}

	void incrementHits(Code code, count_t inc, bool propagate = true)
	{
		derived().apply(
		    code, [inc](auto& node, index_t index) { node.reflection[index].hits += inc; },
		    [inc](auto& node) {
			    for (auto& r : node.reflection) {
				    r.hits += inc;
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
		    node, [dec](auto& node, index_t index) { node.reflection[index].hits -= dec; },
		    [dec](auto& node) {
			    for (auto& r : node.reflection) {
				    r.hits -= dec;
			    }
		    },
		    propagate);
	}

	void decrementHits(Code code, count_t dec, bool propagate = true)
	{
		derived().apply(
		    code, [dec](auto& node, index_t index) { node.reflection[index].hits -= dec; },
		    [dec](auto& node) {
			    for (auto& r : node.reflection) {
				    r.hits -= dec;
			    }
		    },
		    propagate);
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
	// Get misses
	//

	[[nodiscard]] count_t misses(Node node) const { return reflection(node).misses; }

	[[nodiscard]] count_t misses(Code code) const { return reflection(code).hits; }

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
		    node,
		    [misses](auto& node, index_t index) { node.reflection[index].misses = misses; },
		    [misses](auto& node) {
			    for (auto& r : node.reflection) {
				    r.misses = misses;
			    }
		    },
		    propagate);
	}

	void setMisses(Code code, count_t misses, bool propagate = true)
	{
		derived().apply(
		    code,
		    [misses](auto& node, index_t index) { node.reflection[index].misses = misses; },
		    [misses](auto& node) {
			    for (auto& r : node.reflection) {
				    r.misses = misses;
			    }
		    },
		    propagate);
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
		    node, [inc](auto& node, index_t index) { node.reflection[index].misses += inc; },
		    [inc](auto& node) {
			    for (auto& r : node.reflection) {
				    r.misses += inc;
			    }
		    },
		    propagate);
	}

	void incrementMisses(Code code, count_t inc, bool propagate = true)
	{
		derived().apply(
		    code, [inc](auto& node, index_t index) { node.reflection[index].misses += inc; },
		    [inc](auto& node) {
			    for (auto& r : node.reflection) {
				    r.misses += inc;
			    }
		    },
		    propagate);
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
		    node, [dec](auto& node, index_t index) { node.reflection[index].misses -= dec; },
		    [dec](auto& node) {
			    for (auto& r : node.reflection) {
				    r.misses -= dec;
			    }
		    },
		    propagate);
	}

	void decrementMisses(Code code, count_t dec, bool propagate = true)
	{
		derived().apply(
		    code, [dec](auto& node, index_t index) { node.reflection[index].misses -= dec; },
		    [dec](auto& node) {
			    for (auto& r : node.reflection) {
				    r.misses -= dec;
			    }
		    },
		    propagate);
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
	// Get reflectiveness
	//

	[[nodiscard]] reflection_t reflectiveness(Node node) const
	{
		return reflection(node).reflectiveness();
	}

	[[nodiscard]] reflection_t reflectiveness(Code code) const
	{
		return reflection(code).reflectiveness();
	}

	[[nodiscard]] reflection_t reflectiveness(Key key) const
	{
		return reflectiveness(derived().toCode(key));
	}

	[[nodiscard]] reflection_t reflectiveness(Point coord, depth_t depth = 0) const
	{
		return reflectiveness(derived().toCode(coord, depth));
	}

	[[nodiscard]] reflection_t reflectiveness(coord_t x, coord_t y, coord_t z,
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
		derived().root().reflection[derived().rootIndex()] = Reflection(0, 0);
	}

	//
	// Fill
	//

	template <std::size_t N>
	void fill(ReflectionNode<N>& node, ReflectionNode<N> const& parent, index_t index)
	{
		node.reflection.fill(parent.reflection[index]);
	}

	//
	// Clear
	//

	template <std::size_t N>
	void clear(ReflectionNode<N>&)
	{
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
				node.reflection[index] = min(children);
				return;
			case PropagationCriteria::MAX:
				node.reflection[index] = max(children);
				return;
			case PropagationCriteria::MEAN:
				node.reflection[index] = mean(children);
				return;
		}
	}

	template <std::size_t N>
	Reflection min(ReflectionNode<N> const& children)
	{
		Reflection res(std::numeric_limits<count_t>::max(),
		               std::numeric_limits<count_t>::max());
		for (auto r : children.reflection) {
			if (r.hits < res.hits) {
				res.hits = r.hits;
			}
			if (r.misses < res.misses) {
				res.misses = r.misses;
			}
		}
		return res;
	}

	template <std::size_t N>
	Reflection max(ReflectionNode<N> const& children)
	{
		Reflection res(std::numeric_limits<count_t>::lowest(),
		               std::numeric_limits<count_t>::lowest());
		for (auto r : children.reflection) {
			if (r.hits > res.hits) {
				res.hits = r.hits;
			}
			if (r.misses > res.misses) {
				res.misses = r.misses;
			}
		}
		return res;
	}

	template <std::size_t N>
	Reflection mean(ReflectionNode<N> const& children)
	{
		Reflection res;
		for (auto r : children.reflection) {
			res.hits += r.hits;
			res.misses += r.misses;
		}
		res.hits /= N;
		res.misses /= N;
		return res;
	}

	//
	// Is collapsible
	//

	template <std::size_t N>
	[[nodiscard]] bool isCollapsible(ReflectionNode<N> const& node) const
	{
		// TODO: Use floor(log2(X))?
		return std::all_of(std::begin(node.reflection) + 1, std::end(node.reflection),
		                   [p = node.reflection.front()](auto e) { return e == p; });
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr MapType mapType() noexcept
	{
		return MapType::REFLECTION;
	}

	[[nodiscard]] static constexpr bool canReadData(MapType mt) noexcept
	{
		return mapType() == mt;
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
		       std::distance(first, last) * sizeof(Reflection);
	}

	template <class OutputIt>
	void readNodes(ReadBuffer& in, OutputIt first, OutputIt last)
	{
		for (; first != last; ++first) {
			if (first->indices.all()) {
				in.read(first->node->reflection.data(),
				        first->node->reflection.size() *
				            sizeof(typename decltype(first->node->reflection)::value_type));
			} else {
				decltype(first->node->reflection) reflection;
				in.read(reflection.data(),
				        reflection.size() * sizeof(typename decltype(reflection)::value_type));
				for (index_t i = 0; reflection.size() != i; ++i) {
					if (first->indices[i]) {
						first->node->reflection[i] = reflection[i];
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
			out.write(first->reflection.data(),
			          first->reflection.size() *
			              sizeof(typename decltype(first->reflection)::value_type));
		}
	}

 protected:
	// Propagation criteria
	PropagationCriteria prop_criteria_ = PropagationCriteria::MAX;
};
}  // namespace ufo::map

#endif  // UFO_MAP_REFLECTION_MAP_H