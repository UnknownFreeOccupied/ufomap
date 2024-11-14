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

#ifndef UFO_MAP_SEMANTIC_SET_MAP_H
#define UFO_MAP_SEMANTIC_SET_MAP_H

// UFO
#include <ufo/container/range_map.hpp>
#include <ufo/map/node.hpp>
#include <ufo/map/semantic/semantic.hpp>
#include <ufo/map/semantic/semantic_mapping.hpp>
#include <ufo/map/semantic_set/semantic_set.hpp>
#include <ufo/map/semantic_set/semantic_set_block.hpp>
#include <ufo/map/types.hpp>
#include <ufo/util/buffer.hpp>

// STL
#include <cstddef>
#include <limits>
#include <memory>

// STL parallel
#ifdef UFO_TBB
#include <execution>
#endif

namespace ufo
{
template <class Derived>
class SemanticSetMap : public SemanticMapping
{
 private:
	using Index                          = typename Derived::Index;
	using Node                           = typename Derived::Node;
	using Code                           = typename Derived::Code;
	using Key                            = typename Derived::Key;
	using Point                          = typename Derived::Point;
	static constexpr std::size_t const N = Derived::childrenPerParent();

 public:
	//
	// Get semantics
	//

	[[nodiscard]] SemanticSet<1> semantics(Index node) const
	{
		assert(semantics_.size() > node.pos && N > node.offset);

		if (semanticsEmpty(node)) {
			return {};
		}

		if (derived().isLeaf(node)) {
			// Return stuff from current node
			static_assert(N > 1);
			// TODO: Add function in SemanticSet<N> to retrive a sub
			return semantics_[node.pos].semantics.subset(node.offset);
		}

		// Traverse tree and get stuff from leaves
		std::vector<Semantic> tmp;
		derived().traverse(node, [this, &tmp, p = node.pos](Index node) -> bool {
			if (0 == node.offset && p != node.pos) {
				tmp.insert(std::end(tmp), std::cbegin(semantics_[node.pos].semantics),
				           std::cend(semantics_[node.pos].semantics));
			}
			return !semanticsEmpty(node);
		});

		std::sort(std::begin(tmp), std::end(tmp));

		switch (semanticsPropagationCriteria()) {
			case PropagationCriteria::MIN: {
				auto last = std::unique(std::begin(tmp), std::end(tmp),
				                        [](auto a, auto b) { return a.label == b.label; });
				return {std::begin(tmp), last};
			}
			case PropagationCriteria::MEAN: {
				// TODO: Implement
			}
			default: {
				// Default is to return the maximum value for a label
				auto r_last = std::unique(std::rbegin(tmp), std::rend(tmp),
				                          [](auto a, auto b) { return a.label == b.label; });
				return {r_last.base(), std::end(tmp)};
			}
		}
	}

	[[nodiscard]] SemanticSet<1> semantics(Node node) const
	{
		return semantics(derived().index(node));
	}

	[[nodiscard]] SemanticSet<1> semantics(Code code) const
	{
		return semantics(derived().index(code));
	}

	[[nodiscard]] SemanticSet<1> semantics(Key key) const
	{
		return semantics(derived().index(key));
	}

	[[nodiscard]] SemanticSet<1> semantics(Point coord, depth_t depth = 0) const
	{
		return semantics(derived().index(coord, depth));
	}

#ifdef UFO_TBB
	template <class ExecutionPolicy,
	          std::enable_if_t<std::is_execution_policy_v<ExecutionPolicy>,
	                           bool> = true>
	[[nodiscard]] SemanticSet<1> semantics(ExecutionPolicy&& policy, Index node) const
	{
		assert(semantics_.size() > node.pos && N > node.offset);

		if (semanticsEmpty(node)) {
			return {};
		}

		if (derived().isLeaf(node)) {
			// Return stuff from current node
			static_assert(N > 1);
			// TODO: Add function in SemanticSet<N> to retrive a sub
			return semantics_[node.pos].semantics.subset(node.offset);
		}

		// Traverse tree and get stuff from leaves
		std::vector<Semantic> tmp;
		derived().traverse(node, [this, &tmp, p = node.pos](Index node) -> bool {
			if (0 == node.offset && p != node.pos) {
				tmp.insert(std::end(tmp), std::cbegin(semantics_[node.pos].semantics),
				           std::cend(semantics_[node.pos].semantics));
			}
			return !semanticsEmpty(node);
		});

		std::sort(policy, std::begin(tmp), std::end(tmp));

		switch (semanticsPropagationCriteria()) {
			case PropagationCriteria::MIN: {
				auto last =
				    std::unique(std::forward<ExecutionPolicy>(policy), std::begin(tmp),
				                std::end(tmp), [](auto a, auto b) { return a.label == b.label; });
				return {std::begin(tmp), last};
			}
			case PropagationCriteria::MEAN: {
				// TODO: Implement
			}
			default: {
				// Default is to return the maximum value for a label
				auto r_last = std::unique(std::forward<ExecutionPolicy>(policy), std::rbegin(tmp),
				                          std::rend(tmp),
				                          [](auto a, auto b) { return a.label == b.label; });
				return {r_last.base(), std::end(tmp)};
			}
		}
	}

	template <class ExecutionPolicy,
	          std::enable_if_t<std::is_execution_policy_v<ExecutionPolicy>,
	                           bool> = true>
	[[nodiscard]] SemanticSet<1> semantics(ExecutionPolicy&& policy, Node node) const
	{
		return semantics(std::forward<ExecutionPolicy>(policy), derived().index(node));
	}

	template <class ExecutionPolicy,
	          std::enable_if_t<std::is_execution_policy_v<ExecutionPolicy>,
	                           bool> = true>
	[[nodiscard]] SemanticSet<1> semantics(ExecutionPolicy&& policy, Code code) const
	{
		return semantics(std::forward<ExecutionPolicy>(policy), derived().index(code));
	}

	template <class ExecutionPolicy,
	          std::enable_if_t<std::is_execution_policy_v<ExecutionPolicy>,
	                           bool> = true>
	[[nodiscard]] SemanticSet<1> semantics(ExecutionPolicy&& policy, Key key) const
	{
		return semantics(std::forward<ExecutionPolicy>(policy), derived().index(key));
	}

	template <class ExecutionPolicy,
	          std::enable_if_t<std::is_execution_policy_v<ExecutionPolicy>,
	                           bool> = true>
	[[nodiscard]] SemanticSet<1> semantics(ExecutionPolicy&& policy, Point coord,
	                                       depth_t depth = 0) const
	{
		return semantics(std::forward<ExecutionPolicy>(policy),
		                 derived().index(coord, depth));
	}
#endif

	//
	// Get semantics summary
	//

	[[nodiscard]] Semantic semanticsSummary(pos_t block) const
	{
		assert(semantics_.size() > block);
		return semantics_[block].summary;
	}

	[[nodiscard]] Semantic semanticsSummary(Index node) const
	{
		assert(semantics_.size() > node.pos && N > node.offset);
		return derived().isChild(node) ? semantics_[node.pos].summary(node.offset)
		                               : semanticsSummary(derived().children(node));
	}

	[[nodiscard]] Semantic semanticsSummary(Node node) const
	{
		return semanticsSummary(derived().index(node));
	}

	[[nodiscard]] Semantic semanticsSummary(Code code) const
	{
		return semanticsSummary(derived().index(code));
	}

	[[nodiscard]] Semantic semanticsSummary(Key key) const
	{
		return semanticsSummary(derived().index(key));
	}

	[[nodiscard]] Semantic semanticsSummary(Point coord, depth_t depth = 0) const
	{
		return semanticsSummary(derived().index(coord, depth));
	}

	//
	// Semantics empty
	//

	[[nodiscard]] bool semanticsEmpty(Index node) const
	{
		assert(semantics_.size() > node.pos && N > node.offset);
		return semantics_[node.pos].label_summary[node.offset];
	}

	[[nodiscard]] bool semanticsEmpty(Node node) const
	{
		return semanticsEmpty(derived().index(node));
	}

	[[nodiscard]] bool semanticsEmpty(Code code) const
	{
		return semanticsEmpty(derived().index(code));
	}

	[[nodiscard]] bool semanticsEmpty(Key key) const
	{
		return semanticsEmpty(derived().index(key));
	}

	[[nodiscard]] bool semanticsEmpty(Point coord, depth_t depth = 0) const
	{
		return semanticsEmpty(derived().index(coord, depth));
	}

	//
	// Semantics size
	//

	[[nodiscard]] std::size_t semanticsSize(Index node) const
	{
		assert(semantics_.size() > node.pos && N > node.offset);
		// TODO: Implement
	}

	[[nodiscard]] std::size_t semanticsSize(Node node) const
	{
		return semanticsSize(derived().index(node));
	}

	[[nodiscard]] std::size_t semanticsSize(Code code) const
	{
		return semanticsSize(derived().index(code));
	}

	[[nodiscard]] std::size_t semanticsSize(Key key) const
	{
		return semanticsSize(derived().index(key));
	}

	[[nodiscard]] std::size_t semanticsSize(Point coord, depth_t depth = 0) const
	{
		return semanticsSize(derived().index(coord, depth));
	}

	//
	// Get semantics count
	//

	[[nodiscard]] std::size_t semanticsCount(Index node, label_t label) const
	{
		assert(semantics_.size() > node.pos && N > node.offset);
		// TODO: Implement
	}

	[[nodiscard]] std::size_t semanticsCount(Node node, label_t label) const
	{
		return semanticsCount(derived().index(node), label);
	}

	[[nodiscard]] std::size_t semanticsCount(Code code, label_t label) const
	{
		return semanticsCount(derived().index(code), label);
	}

	[[nodiscard]] std::size_t semanticsCount(Key key, label_t label) const
	{
		return semanticsCount(derived().index(key), label);
	}

	[[nodiscard]] std::size_t semanticsCount(Point coord, label_t label,
	                                         depth_t depth = 0) const
	{
		return semanticsCount(derived().index(coord, depth), label);
	}

	//
	// Contains
	//

	[[nodiscard]] bool semanticsContains(Index node, label_t label) const
	{
		assert(semantics_.size() > node.pos && N > node.offset);
		// TODO: Implement
	}

	[[nodiscard]] bool semanticsContains(Node node, label_t label) const
	{
		return semanticsContains(derived().index(node), label);
	}

	[[nodiscard]] bool semanticsContains(Code code, label_t label) const
	{
		return semanticsContains(derived().index(code), label);
	}

	[[nodiscard]] bool semanticsContains(Key key, label_t label) const
	{
		return semanticsContains(derived().index(key), label);
	}

	[[nodiscard]] bool semanticsContains(Point coord, label_t label,
	                                     depth_t depth = 0) const
	{
		return semanticsContains(derived().index(coord, depth), label);
	}

	//
	// Contains class
	//

	[[nodiscard]] bool semanticsContainsClass(Index node, std::string tag) const
	{
		// TODO: Implement
	}

	// TODO: What type is class?
	[[nodiscard]] bool semanticsContainsClass(Index node, label_t tag) const
	{
		// TODO: Implement
	}

	//
	// Contains all
	//

	[[nodiscard]] bool semanticsContainsAll(Index node, std::string tag) const
	{
		// TODO: Implement
	}

	[[nodiscard]] bool semanticsContainsAll(Index node, SemanticRange range) const
	{
		assert(semantics_.size() > node.pos && N > node.offset);
		// TODO: Implement
	}

	[[nodiscard]] bool semanticsContainsAll(Index                   node,
	                                        SemanticRangeSet const& ranges) const
	{
		assert(semantics_.size() > node.pos && N > node.offset);
		// TODO: Implement
	}

	[[nodiscard]] bool semanticsContainsAll(Node node, SemanticRange range) const
	{
		return semanticsContainsAll(derived().index(node), range);
	}

	[[nodiscard]] bool semanticsContainsAll(Code code, SemanticRange range) const
	{
		return semanticsContainsAll(derived().index(code), range);
	}

	[[nodiscard]] bool semanticsContainsAll(Key key, SemanticRange range) const
	{
		return semanticsContainsAll(derived().index(key), range);
	}

	[[nodiscard]] bool semanticsContainsAll(Point coord, SemanticRange range,
	                                        depth_t depth = 0) const
	{
		return semanticsContainsAll(derived().index(coord, depth), range);
	}

	//
	// Contains any
	//

	[[nodiscard]] bool semanticsContainsAny(Index node, SemanticRange range) const
	{
		assert(semantics_.size() > node.pos && N > node.offset);
		// TODO: Implement
	}

	[[nodiscard]] bool semanticsContainsAny(Node node, SemanticRange range) const
	{
		return semanticsContainsAny(derived().index(node), range);
	}

	[[nodiscard]] bool semanticsContainsAny(Code code, SemanticRange range) const
	{
		return semanticsContainsAny(derived().index(code), range);
	}

	[[nodiscard]] bool semanticsContainsAny(Key key, SemanticRange range) const
	{
		return semanticsContainsAny(derived().index(key), range);
	}

	[[nodiscard]] bool semanticsContainsAny(Point coord, SemanticRange range,
	                                        depth_t depth = 0) const
	{
		return semanticsContainsAny(derived().index(coord, depth), range);
	}

	//
	// Contains none
	//

	[[nodiscard]] bool semanticsContainsNone(Index node, SemanticRange range) const
	{
		assert(semantics_.size() > node.pos && N > node.offset);
		// TODO: Implement
	}

	[[nodiscard]] bool semanticsContainsNone(Node node, SemanticRange range) const
	{
		return semanticsContainsNone(derived().index(node), range);
	}

	[[nodiscard]] bool semanticsContainsNone(Code code, SemanticRange range) const
	{
		return semanticsContainsNone(derived().index(code), range);
	}

	[[nodiscard]] bool semanticsContainsNone(Key key, SemanticRange range) const
	{
		return semanticsContainsNone(derived().index(key), range);
	}

	[[nodiscard]] bool semanticsContainsNone(Point coord, SemanticRange range,
	                                         depth_t depth = 0) const
	{
		return semanticsContainsNone(derived().index(coord, depth), range);
	}

	//
	// Contains some
	//

	[[nodiscard]] bool semanticsContainsSome(Index node, SemanticRange range) const
	{
		assert(semantics_.size() > node.pos && N > node.offset);
		// TODO: Implement
	}

	[[nodiscard]] bool semanticsContainsSome(Node node, SemanticRange range) const
	{
		return semanticsContainsSome(derived().index(node), range);
	}

	[[nodiscard]] bool semanticsContainsSome(Code code, SemanticRange range) const
	{
		return semanticsContainsSome(derived().index(code), range);
	}

	[[nodiscard]] bool semanticsContainsSome(Key key, SemanticRange range) const
	{
		return semanticsContainsSome(derived().index(key), range);
	}

	[[nodiscard]] bool semanticsContainsSome(Point coord, SemanticRange range,
	                                         depth_t depth = 0) const
	{
		return semanticsContainsSome(derived().index(coord, depth), range);
	}

	//
	// Set semantics
	//

	void semanticsSet(Index node, SemanticSet<1> const& semantics)
	{
		derived().apply(
		    node,
		    [this, &semantics](Index node) {
			    // TODO: Update summary
			    semantics_[node.pos].semantics.set(node.offset, semantics);
		    },
		    [this, &semantics](pos_t block) {
			    // TODO: Update summary
			    semantics_[block].semantics.set(semantics);
		    });
	}

	Node semanticsSet(Node node, SemanticSet<1> const& semantics, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, &semantics](Index node) {
			    // TODO: Update summary
			    semantics_[node.pos].semantics.set(node.offset, semantics);
		    },
		    [this, &semantics](pos_t block) {
			    // TODO: Update summary
			    semantics_[block].semantics.set(semantics);
		    },
		    propagate);
	}

	Node semanticsSet(Code code, SemanticSet<1> const& semantics, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, &semantics](Index node) {
			    // TODO: Update summary
			    semantics_[node.pos].semantics.set(node.offset, semantics);
		    },
		    [this, &semantics](pos_t block) {
			    // TODO: Update summary
			    semantics_[block].semantics.set(semantics);
		    },
		    propagate);
	}

	Node semanticsSet(Key key, SemanticSet<1> const& semantics, bool propagate = true)
	{
		return semanticsSet(derived().toCode(key), semantics, propagate);
	}

	Node semanticsSet(Point coord, SemanticSet<1> const& semantics, bool propagate = true,
	                  depth_t depth = 0)
	{
		return semanticsSet(derived().toCode(coord, depth), semantics, propagate);
	}

	//
	// Insert semantics
	//

	void semanticsInsert(Index node, Semantic sem)
	{
		return derived().apply(
		    node,
		    [this, sem](Index node) {
			    // TODO: Update summary
			    semantics_[node.pos].semantics.insert(node.offset, sem);
		    },
		    [this, sem](pos_t block) {
			    // TODO: Update summary
			    semantics_[block].semantics.insert(sem);
		    });
	}

	void semanticsInsert(Index node, label_t label, value_t value)
	{
		return semanticsInsert(node, Semantic(label, value));
	}

	template <class InputIt>
	void semanticsInsert(Index node, InputIt first, InputIt last)
	{
		// TODO: Implement
	}

	template <class SemanticRange>
	void semanticsInsert(Index node, SemanticRange const& sems)
	{
		semanticsInsert(node, std::cbegin(sems), std::cend(sems));
	}

	void semanticsInsert(Index node, std::initializer_list<Semantic> ilist)
	{
		semanticsInsert(node, std::cbegin(ilist), std::cend(ilist));
	}

	Node semanticsInsert(Node node, Semantic sem, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, sem](Index node) {
			    // TODO: Update summary
			    semantics_[node.pos].semantics.insert(node.offset, sem);
		    },
		    [this, sem](pos_t block) {
			    // TODO: Update summary
			    semantics_[block].semantics.insert(sem);
		    },
		    propagate);
	}

	Node semanticsInsert(Node node, label_t label, value_t value, bool propagate = true)
	{
		return semanticsInsert(node, Semantic(label, value), propagate);
	}

	Node semanticsInsert(Code code, Semantic sem, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, sem](Index node) {
			    // TODO: Update summary
			    semantics_[node.pos].semantics.insert(node.offset, sem);
		    },
		    [this, sem](pos_t block) {
			    // TODO: Update summary
			    semantics_[block].semantics.insert(sem);
		    },
		    propagate);
	}

	Node semanticsInsert(Code code, label_t label, value_t value, bool propagate = true)
	{
		return semanticsInsert(code, Semantic(label, value), propagate);
	}

	Node semanticsInsert(Key key, Semantic value, bool propagate = true)
	{
		return semanticsInsert(derived().toCode(key), value, propagate);
	}

	Node semanticsInsert(Key key, label_t label, value_t value, bool propagate = true)
	{
		return semanticsInsert(key, Semantic(label, value), propagate);
	}

	Node semanticsInsert(Point coord, Semantic value, bool propagate = true,
	                     depth_t depth = 0)
	{
		return semanticsInsert(derived().toCode(coord, depth), value, propagate);
	}

	Node semanticsInsert(Point coord, label_t label, value_t value, bool propagate = true,
	                     depth_t depth = 0)
	{
		return semanticsInsert(coord, Semantic(label, value), propagate, depth);
	}

	//
	// Insert or assign
	//

	void semanticsInsertOrAssign(Index node, Semantic sem)
	{
		// TODO: Implement
	}

	void semanticsInsertOrAssign(Index node, label_t label, value_t value)
	{
		semanticsInsertOrAssign(node, Semantic(label, value));
	}

	template <class InputIt>
	void semanticsInsertOrAssign(Index node, InputIt first, InputIt last)
	{
		// TODO: Implement
	}

	template <class SemanticRange>
	void semanticsInsertOrAssign(Index node, SemanticRange const& sems)
	{
		semanticsInsertOrAssign(node, std::cbegin(sems), std::cend(sems));
	}

	void semanticsInsertOrAssign(Index node, std::initializer_list<Semantic> ilist)
	{
		semanticsInsert(node, std::cbegin(ilist), std::cend(ilist));
	}

	//
	// Insert or update
	//

	template <
	    class UnaryFun,
	    std::enable_if_t<std::is_invocable_r_v<value_t, UnaryFun, Semantic>, bool> = true>
	void semanticsInsertOrUpdate(Index node, Semantic sem, UnaryFun f)
	{
		// TODO: Implement
	}

	template <
	    class UnaryFun,
	    std::enable_if_t<std::is_invocable_r_v<value_t, UnaryFun, Semantic>, bool> = true>
	void semanticsInsertOrUpdate(Index node, label_t label, value_t value, UnaryFun f)
	{
		semanticsInsertOrUpdate(node, Semantic(label, value), f);
	}

	template <
	    class InputIt, class UnaryFun,
	    std::enable_if_t<std::is_invocable_r_v<value_t, UnaryFun, Semantic>, bool> = true>
	void semanticsInsertOrUpdate(Index node, InputIt first, InputIt last, UnaryFun f)
	{
		// TODO: Implement
	}

	template <
	    class SemanticRange, class UnaryFun,
	    std::enable_if_t<std::is_invocable_r_v<value_t, UnaryFun, Semantic>, bool> = true>
	void semanticsInsertOrUpdate(Index node, SemanticRange const& sems, UnaryFun f)
	{
		semanticsInsertOrUpdate(node, std::cbegin(sems), std::cend(sems), f);
	}

	template <
	    class UnaryFun,
	    std::enable_if_t<std::is_invocable_r_v<value_t, UnaryFun, Semantic>, bool> = true>
	void semanticsInsertOrUpdate(Index node, std::initializer_list<Semantic> ilist,
	                             UnaryFun f)
	{
		semanticsInsertOrUpdate(node, std::cbegin(ilist), std::cend(ilist), f);
	}

	//
	// Assign
	//

	void semanticsAssign(Index node, value_t value)
	{
		// TODO: Implement
	}

	void semanticsAssign(Index node, std::string const& tag, value_t value)
	{
		// TODO: Implement
	}

	void semanticsAssign(Index node, SemanticRange range, value_t value)
	{
		// TODO: Implement
	}

	void semanticsAssign(Index node, SemanticRangeSet const& ranges, value_t value)
	{
		// TODO: Implement
	}

	//
	// Update
	//

	template <
	    class UnaryFun,
	    std::enable_if_t<std::is_invocable_r_v<value_t, UnaryFun, Semantic>, bool> = true>
	void semanticsUpdate(Index node, UnaryFun f)
	{
		// TODO: Implement
	}

	template <
	    class UnaryFun,
	    std::enable_if_t<std::is_invocable_r_v<value_t, UnaryFun, Semantic>, bool> = true>
	void semanticsUpdate(Index node, std::string const& tag, UnaryFun f)
	{
		// TODO: Implement
	}

	//
	// Erase
	//

	// TODO: Implement

	//
	// Erase if
	//

	// TODO: Implement

	//
	// Clear
	//

	// TODO: Implement

	void semanticsClear(bool propagate = true)
	{
		semanticsClear(derived().rootCode(), propagate);
	}

	void semanticsClear(Index node)
	{
		// TODO: Implement
	}

	Node semanticsClear(Node node, bool propagate = true)
	{
		// TODO: Implement
	}

	Node semanticsClear(Code code, bool propagate = true)
	{
		// TODO: Implement
	}

	Node semanticsClear(Key key, bool propagate = true)
	{
		return semanticsClear(derived().toCode(key), propagate);
	}

	Node semanticsClear(Point coord, bool propagate = true, depth_t depth = 0)
	{
		return semanticsClear(derived().toCode(coord, depth), propagate);
	}

	//
	// Change label
	//

	//
	// Integration
	//

	template <class InputIt>
	void insertPoints(Index node, InputIt first, InputIt last)
	{
		for (auto it = first; it != last; ++it) {
			insertSemantics(node, it->label, it->value);
		}
	}

	//
	// Contains
	//

	[[nodiscard]] bool containsSemantics(Index node, label_t label) const
	{
		switch (semanticsPropagationCriteria()) {
			case PropagationCriteria::MIN:
			case PropagationCriteria::MAX:
				return semantics_[node.pos].semantic_set.contains(node.offset, label);
			case PropagationCriteria::NONE:
			case PropagationCriteria::S_MIN:
			case PropagationCriteria::S_MAX: {
				if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
					// check content
					return semantics_[node.pos].semantic_set.contains(node.offset, label);
				} else {
					// traverse tree
					for (offset_t i = 0; i != N; ++i) {
						auto child = derived().child(node, i);
						if (containsSemantics(child, label)) {
							return true;
						}
					}
					return false;
				}
			}
		}
		return true;
	}

	//
	// All
	//

	// inputIt to tags (string)
	template <class InputIt>
	[[nodiscard]] bool allSemantics(Index node, InputIt first, InputIt last) const
	{
		SemanticRangeSet ranges;
		for (auto it = first; it != last; ++it) {
			ranges.insert(labels(*it));
		}
		return allSemantics(node, ranges);
	}

	[[nodiscard]] bool allSemantics(Index node, SemanticRangeSet ranges) const
	{
		switch (semanticsPropagationCriteria()) {
			case PropagationCriteria::MIN:
			case PropagationCriteria::MAX:
				return semantics_[node.pos].semantic_set.all(node.offset, ranges);

			case PropagationCriteria::NONE:
			case PropagationCriteria::S_MIN:
			case PropagationCriteria::S_MAX: {
				if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
					// check content
					return semantics_[node.pos].semantic_set.all(node.offset, ranges);
				} else {
					// traverse tree
					for (offset_t i = 0; i != N; ++i) {
						auto child = derived().child(node, i);
						if (allSemantics(child, ranges)) {
							return true;
						}
					}
					return false;
				}
			}
		}
		return true;
	}

	//
	// Any
	//

	// inputIt to tags (string)
	template <class InputIt>
	[[nodiscard]] bool anySemantics(Index node, InputIt first, InputIt last) const
	{
		SemanticRangeSet ranges;
		for (auto it = first; it != last; ++it) {
			auto ls = labels(*it);
			ranges.insert(ls.begin(), ls.end());
		}
		return anySemantics(node, ranges);
	}

	[[nodiscard]] bool anySemantics(Index node, SemanticRangeSet ranges) const
	{
		switch (semanticsPropagationCriteria()) {
			case PropagationCriteria::MIN:
			case PropagationCriteria::MAX:
				return semantics_[node.pos].semantic_set.any(node.offset, ranges);

			case PropagationCriteria::NONE:
			case PropagationCriteria::S_MIN:
			case PropagationCriteria::S_MAX: {
				if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
					// check content
					return semantics_[node.pos].semantic_set.any(node.offset, ranges);
				} else {
					// traverse tree
					for (offset_t i = 0; i != N; ++i) {
						auto child = derived().child(node, i);
						if (anySemantics(child, ranges)) {
							return true;
						}
					}
					return false;
				}
			}
		}
		return true;
	}

	//
	// None
	//

	// inputIt to tags (string)
	template <class InputIt>
	[[nodiscard]] bool noneSemantics(Index node, InputIt first, InputIt last) const
	{
		SemanticRangeSet ranges;
		for (auto it = first; it != last; ++it) {
			ranges.insert(labels(*it));
		}
		return noneSemantics(node, ranges);
	}

	[[nodiscard]] bool noneSemantics(Index node, SemanticRangeSet ranges) const
	{
		switch (semanticsPropagationCriteria()) {
			case PropagationCriteria::MIN:
			case PropagationCriteria::MAX:
				return semantics_[node.pos].semantic_set.none(node.offset, ranges);

			case PropagationCriteria::NONE:
			case PropagationCriteria::S_MIN:
			case PropagationCriteria::S_MAX: {
				if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
					// check content
					return semantics_[node.pos].semantic_set.none(node.offset, ranges);
				} else {
					// traverse tree
					for (offset_t i = 0; i != N; ++i) {
						auto child = derived().child(node, i);
						if (!noneSemantics(child, ranges)) {
							return false;
						}
					}
					return true;
				}
			}
		}
		return true;
	}

	//
	// Insert semantics
	//

	void insertSemantics(Index node, label_t label, value_t value)
	{
		if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
			derived().apply(
			    node,
			    [this, label, value](Index node) {
				    semantics_[node.pos].semantic_set.insert(node.offset, label, value);
			    },
			    [this, label, value](pos_t pos) {
				    semantics_[pos].semantic_set.insert(label, value);
			    });
		} else {
			for (offset_t i{}; i != N; ++i) {
				insertSemantics(derived().child(node, i), label, value);
			}
		}
	}

	Node insertSemantics(Node node, label_t label, value_t value, bool propagate = true)
	{
		if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
			return derived().apply(
			    node,
			    [this, label, value](Index node) {
				    semantics_[node.pos].semantic_set.insert(node.offset, label, value);
			    },
			    [this, label, value](pos_t pos) {
				    semantics_[pos].semantic_set.insert(label, value);
			    },
			    propagate);
		} else {
			for (offset_t i{}; i != N; ++i) {
				insertSemantics(derived().child(node, i), label, value, propagate);
			}
			return node;
		}
	}

	Node insertSemantics(Code code, label_t label, value_t value, bool propagate = true)
	{
		if (derived().isLeaf(code) || derived().isPureLeaf(code)) {
			return derived().apply(
			    code,
			    [this, label, value](Index node) {
				    semantics_[node.pos].semantic_set.insert(node.offset, label, value);
			    },
			    [this, label, value](pos_t pos) {
				    semantics_[pos].semantic_set.insert(label, value);
			    },
			    propagate);
		} else {
			for (offset_t i{}; i != N; ++i) {
				insertSemantics(code.child(i), label, value, propagate);
			}
			return derived()(code);
		}
	}

	Node insertSemantics(Key key, label_t label, value_t value, bool propagate = true)
	{
		return insertSemantics(derived().toCode(key), label, value, propagate);
	}

	Node insertSemantics(Point coord, label_t label, value_t value, bool propagate = true,
	                     depth_t depth = 0)
	{
		return insertSemantics(derived().toCode(coord, depth), label, value, propagate);
	}

	Node insertSemantics(Node node, Semantic semantic, bool propagate = true)
	{
		return insertSemantics(node, semantic.label, semantic.value, propagate);
	}

	Node insertSemantics(Code code, Semantic semantic, bool propagate = true)
	{
		return insertSemantics(code, semantic.label, semantic.value, propagate);
	}

	Node insertSemantics(Key key, Semantic semantic, bool propagate = true)
	{
		return insertSemantics(derived().toCode(key), semantic, propagate);
	}

	Node insertSemantics(Point coord, Semantic semantic, bool propagate = true,
	                     depth_t depth = 0)
	{
		return insertSemantics(derived().toCode(coord, depth), semantic, propagate);
	}

	template <class InputIt,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	void insertSemantics(Index node, InputIt first, InputIt last)
	{
		if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
			derived().apply(
			    node,
			    [this, first, last](Index node) {
				    semantics_[node.pos].semantic_set.insert(node.offset, first, last);
			    },
			    [this, first, last](pos_t pos) {
				    semantics_[pos].semantic_set.insert(first, last);
			    });
		} else {
			for (offset_t i{}; i != N; ++i) {
				insertSemantics(derived().child(node, i), first, last);
			}
		}
	}

	template <class InputIt,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	Node insertSemantics(Node node, InputIt first, InputIt last, bool propagate = true)
	{
		if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
			return derived().apply(
			    node,
			    [this, first, last](Index node) {
				    semantics_[node.pos].semantic_set.insert(node.offset, first, last);
			    },
			    [this, first, last](pos_t pos) {
				    semantics_[pos].semantic_set.insert(first, last);
			    },
			    propagate);
		} else {
			for (offset_t i{}; i != N; ++i) {
				insertSemantics(derived().child(node, i), first, last, propagate);
			}
			return node;
		}
	}

	template <class InputIt,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	Node insertSemantics(Code code, InputIt first, InputIt last, bool propagate = true)
	{
		if (derived().isLeaf(code) || derived().isPureLeaf(code)) {
			return derived().apply(
			    code,
			    [this, first, last](Index node) {
				    semantics_[node.pos].semantic_set.insert(node.offset, first, last);
			    },
			    [this, first, last](pos_t pos) {
				    semantics_[pos].semantic_set.insert(first, last);
			    },
			    propagate);
		} else {
			for (offset_t i{}; i != N; ++i) {
				insertSemantics(code.child(i), first, last, propagate);
			}
			return derived()(code);
		}
	}

	template <class InputIt,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	Node insertSemantics(Key key, InputIt first, InputIt last, bool propagate = true)
	{
		return insertSemantics(derived().toCode(key), first, last, propagate);
	}

	template <class InputIt,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	Node insertSemantics(Point coord, InputIt first, InputIt last, bool propagate = true,
	                     depth_t depth = 0)
	{
		return insertSemantics(derived().toCode(coord, depth), first, last, propagate);
	}

	Node insertSemantics(Node node, std::initializer_list<Semantic> ilist,
	                     bool propagate = true)
	{
		return insertSemantics(node, std::cbegin(ilist), std::cend(ilist), propagate);
	}

	Node insertSemantics(Code code, std::initializer_list<Semantic> ilist,
	                     bool propagate = true)
	{
		return insertSemantics(code, std::cbegin(ilist), std::cend(ilist), propagate);
	}

	Node insertSemantics(Key key, std::initializer_list<Semantic> ilist,
	                     bool propagate = true)
	{
		return insertSemantics(derived().toCode(key), ilist, propagate);
	}

	Node insertSemantics(Point coord, std::initializer_list<Semantic> ilist,
	                     bool propagate = true, depth_t depth = 0)
	{
		return insertSemantics(derived().toCode(coord, depth), ilist, propagate);
	}

	//
	// Insert or assign semantics
	//
	void insertOrAssignSemantics(Index node, label_t label, value_t value)
	{
		if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
			derived().apply(
			    node,
			    [this, label, value](Index node) {
				    semantics_[node.pos].semantic_set.insertOrAssign(node.offset, label, value);
			    },
			    [this, label, value](pos_t pos) {
				    semantics_[pos].semantic_set.insertOrAssign(label, value);
			    });
		} else {
			for (offset_t i{}; i != N; ++i) {
				insertOrAssignSemantics(derived().child(node, i), label, value);
			}
		}
	}

	Node insertOrAssignSemantics(Node node, label_t label, value_t value,
	                             bool propagate = true)
	{
		if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
			return derived().apply(
			           node,
			           [this, label, value](Index node) {
				           semantics_[node.pos].semantic_set.insertOrAssign(node.offset, label,
				                                                            value);
			           },
			           [this, label, value](pos_t pos) {
				           semantics_[pos].semantic_set.insertOrAssign(label, value);
			           }),
			       propagate;
		} else {
			for (offset_t i{}; i != N; ++i) {
				insertOrAssignSemantics(derived().child(node, i), label, value, propagate);
			}
			return node;
		}
	}

	Node insertOrAssignSemantics(Code code, label_t label, value_t value,
	                             bool propagate = true)
	{
		if (derived().isLeaf(code) || derived().isPureLeaf(code)) {
			return derived().apply(
			    code,
			    [this, label, value](Index node) {
				    semantics_[node.pos].semantic_set.insertOrAssign(node.offset, label, value);
			    },
			    [this, label, value](pos_t pos) {
				    semantics_[pos].semantic_set.insertOrAssign(label, value);
			    },
			    propagate);
		} else {
			for (offset_t i{}; i != N; ++i) {
				insertOrAssignSemantics(code.child(i), label, value, propagate);
			}
			return derived()(code);
		}
	}

	Node insertOrAssignSemantics(Key key, label_t label, value_t value,
	                             bool propagate = true)
	{
		return insertOrAssignSemantics(derived().toCode(key), label, value, propagate);
	}

	Node insertOrAssignSemantics(Point coord, label_t label, value_t value,
	                             bool propagate = true, depth_t depth = 0)
	{
		return insertOrAssignSemantics(derived().toCode(coord, depth), label, value,
		                               propagate);
	}

	Node insertOrAssignSemantics(Node node, Semantic semantic, bool propagate = true)
	{
		return insertOrAssignSemantics(node, semantic.label, semantic.value, propagate);
	}

	Node insertOrAssignSemantics(Code code, Semantic semantic, bool propagate = true)
	{
		return insertOrAssignSemantics(code, semantic.label, semantic.value, propagate);
	}

	Node insertOrAssignSemantics(Key key, Semantic semantic, bool propagate = true)
	{
		return insertOrAssignSemantics(derived().toCode(key), semantic, propagate);
	}

	Node insertOrAssignSemantics(Point coord, Semantic semantic, bool propagate = true,
	                             depth_t depth = 0)
	{
		return insertOrAssignSemantics(derived().toCode(coord, depth), semantic, propagate);
	}

	template <class InputIt,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	void insertOrAssignSemantics(Index node, InputIt first, InputIt last)
	{
		if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
			derived().apply(
			    node,
			    [this, first, last](Index node) {
				    semantics_[node.pos].semantic_set.insertOrAssign(node.offset, first, last);
			    },
			    [this, first, last](pos_t pos) {
				    semantics_[pos].semantic_set.insertOrAssign(first, last);
			    });
		} else {
			for (offset_t i{}; i != N; ++i) {
				insertOrAssignSemantics(derived().child(node, i), first, last);
			}
		}
	}

	template <class InputIt,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	Node insertOrAssignSemantics(Node node, InputIt first, InputIt last,
	                             bool propagate = true)
	{
		if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
			return derived().apply(
			    node,
			    [this, first, last](Index node) {
				    semantics_[node.pos].semantic_set.insertOrAssign(node.offset, first, last);
			    },
			    [this, first, last](pos_t pos) {
				    semantics_[pos].semantic_set.insertOrAssign(first, last);
			    },
			    propagate);
		} else {
			for (offset_t i{}; i != N; ++i) {
				insertOrAssignSemantics(derived().child(node, i), first, last, propagate);
			}
			return node;
		}
	}

	template <class InputIt,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	Node insertOrAssignSemantics(Code code, InputIt first, InputIt last,
	                             bool propagate = true)
	{
		if (derived().isLeaf(code) || derived().isPureLeaf(code)) {
			return derived().apply(
			    code,
			    [this, first, last](Index node) {
				    semantics_[node.pos].semantic_set.insertOrAssign(node.offset, first, last);
			    },
			    [this, first, last](pos_t pos) {
				    semantics_[pos].semantic_set.insertOrAssign(first, last);
			    },
			    propagate);
		} else {
			for (offset_t i{}; i != N; ++i) {
				insertOrAssignSemantics(code.child(i), first, last, propagate);
			}
			return derived()(code);
		}
	}

	template <class InputIt,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	Node insertOrAssignSemantics(Key key, InputIt first, InputIt last,
	                             bool propagate = true)
	{
		return insertOrAssignSemantics(derived().toCode(key), first, last, propagate);
	}

	template <class InputIt,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	Node insertOrAssignSemantics(Point coord, InputIt first, InputIt last,
	                             bool propagate = true, depth_t depth = 0)
	{
		return insertOrAssignSemantics(derived().toCode(coord, depth), first, last,
		                               propagate);
	}

	Node insertOrAssignSemantics(Node node, std::initializer_list<Semantic> ilist,
	                             bool propagate = true)
	{
		return insertOrAssignSemantics(node, std::begin(ilist), std::end(ilist), propagate);
	}

	Node insertOrAssignSemantics(Code code, std::initializer_list<Semantic> ilist,
	                             bool propagate = true)
	{
		return insertOrAssignSemantics(code, std::begin(ilist), std::end(ilist), propagate);
	}

	Node insertOrAssignSemantics(Key key, std::initializer_list<Semantic> ilist,
	                             bool propagate = true)
	{
		return insertOrAssignSemantics(derived().toCode(key), std::begin(ilist),
		                               std::end(ilist), propagate);
	}

	Node insertOrAssignSemantics(Point coord, std::initializer_list<Semantic> ilist,
	                             bool propagate = true, depth_t depth = 0)
	{
		return insertOrAssignSemantics(derived().toCode(coord, depth), std::begin(ilist),
		                               std::end(ilist), propagate);
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	void insertOrAssignSemantics(Index node, label_t label, UnaryFunction f)
	{
		if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
			derived().apply(
			    node,
			    [this, label, f](Index node) {
				    semantics_[node.pos].semantic_set.insertOrAssign(node.offset, label, f);
			    },
			    [this, label, f](pos_t pos) {
				    semantics_[pos].semantic_set.insertOrAssign(label, f);
			    });
		} else {
			for (offset_t i{}; i != N; ++i) {
				insertOrAssignSemantics(derived().child(node, i), label, f);
			}
		}
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	Node insertOrAssignSemantics(Node node, label_t label, UnaryFunction f,
	                             bool propagate = true)
	{
		if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
			return derived().apply(
			    node,
			    [this, label, f](Index node) {
				    semantics_[node.pos].semantic_set.insertOrAssign(node.offset, label, f);
			    },
			    [this, label, f](pos_t pos) {
				    semantics_[pos].semantic_set.insertOrAssign(label, f);
			    },
			    propagate);
		} else {
			for (offset_t i{}; i != N; ++i) {
				insertOrAssignSemantics(derived().child(node, i), label, f, propagate);
			}
			return node;
		}
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	Node insertOrAssignSemantics(Code code, label_t label, UnaryFunction f,
	                             bool propagate = true)
	{
		if (derived().isLeaf(code) || derived().isPureLeaf(code)) {
			return derived().apply(
			    code,
			    [this, label, f](Index node) {
				    semantics_[node.pos].semantic_set.insertOrAssign(node.offset, label, f);
			    },
			    [this, label, f](pos_t pos) {
				    semantics_[pos].semantic_set.insertOrAssign(label, f);
			    },
			    propagate);
		} else {
			for (offset_t i{}; i != N; ++i) {
				insertOrAssignSemantics(code.child(i), label, f, propagate);
			}
			return derived()(code);
		}
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	Node insertOrAssignSemantics(Key key, label_t label, UnaryFunction f,
	                             bool propagate = true)
	{
		return insertOrAssignSemantics(derived().toCode(key), label, f, propagate);
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	Node insertOrAssignSemantics(Point coord, label_t label, UnaryFunction f,
	                             bool propagate = true, depth_t depth = 0)
	{
		return insertOrAssignSemantics(derived().toCode(coord, depth), label, f, propagate);
	}

	template <class InputIt, class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	void insertOrAssignSemantics(Index node, InputIt first, InputIt last, UnaryFunction f)
	{
		if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
			derived().apply(
			    node,
			    [this, first, last, f](Index node) {
				    semantics_[node.pos].semantic_set.insertOrAssign(node.offset, first, last, f);
			    },
			    [this, first, last, f](pos_t pos) {
				    semantics_[pos].semantic_set.insertOrAssign(first, last, f);
			    });
		} else {
			for (offset_t i{}; i != N; ++i) {
				insertOrAssignSemantics(derived().child(node, i), first, last, f);
			}
		}
	}

	template <class InputIt, class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	Node insertOrAssignSemantics(Node node, InputIt first, InputIt last, UnaryFunction f,
	                             bool propagate = true)
	{
		if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
			return derived().apply(
			    node,
			    [this, first, last, f](Index node) {
				    semantics_[node.pos].semantic_set.insertOrAssign(node.offset, first, last, f);
			    },
			    [this, first, last, f](pos_t pos) {
				    semantics_[pos].semantic_set.insertOrAssign(first, last, f);
			    },
			    propagate);
		} else {
			for (offset_t i{}; i != N; ++i) {
				insertOrAssignSemantics(derived().child(node, i), first, last, f, propagate);
			}
			return node;
		}
	}

	template <class InputIt, class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	Node insertOrAssignSemantics(Code code, InputIt first, InputIt last, UnaryFunction f,
	                             bool propagate = true)
	{
		if (derived().isLeaf(code) || derived().isPureLeaf(code)) {
			return derived().apply(
			    code,
			    [this, first, last, f](Index node) {
				    semantics_[node.pos].semantic_set.insertOrAssign(node.offset, first, last, f);
			    },
			    [this, first, last, f](pos_t pos) {
				    semantics_[pos].semantic_set.insertOrAssign(first, last, f);
			    },
			    propagate);
		} else {
			for (offset_t i{}; i != N; ++i) {
				insertOrAssignSemantics(code.child(i), first, last, f, propagate);
			}
			return derived()(code);
		}
	}

	template <class InputIt, class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	Node insertOrAssignSemantics(Key key, InputIt first, InputIt last, UnaryFunction f,
	                             bool propagate = true)
	{
		return insertOrAssignSemantics(derived().toCode(key), first, last, f, propagate);
	}

	template <class InputIt, class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	Node insertOrAssignSemantics(Point coord, InputIt first, InputIt last, UnaryFunction f,
	                             bool propagate = true, depth_t depth = 0)
	{
		return insertOrAssignSemantics(derived().toCode(coord, depth), first, last, f,
		                               propagate);
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	Node insertOrAssignSemantics(Node node, std::initializer_list<label_t> ilist,
	                             UnaryFunction f, bool propagate = true)
	{
		return insertOrAssignSemantics(node, std::begin(ilist), std::end(ilist), f,
		                               propagate);
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	Node insertOrAssignSemantics(Code code, std::initializer_list<label_t> ilist,
	                             UnaryFunction f, bool propagate = true)
	{
		return insertOrAssignSemantics(code, std::begin(ilist), std::end(ilist), f,
		                               propagate);
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	Node insertOrAssignSemantics(Key key, std::initializer_list<label_t> ilist,
	                             UnaryFunction f, bool propagate = true)
	{
		return insertOrAssignSemantics(derived().toCode(key), std::begin(ilist),
		                               std::end(ilist), f, propagate);
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	Node insertOrAssignSemantics(Point coord, std::initializer_list<label_t> ilist,
	                             UnaryFunction f, bool propagate = true, depth_t depth = 0)
	{
		return insertOrAssignSemantics(derived().toCode(coord, depth), std::begin(ilist),
		                               std::end(ilist), f, propagate);
	}

	//
	// Assign
	//
	void assignSemantics(Index node, std::string const& tag, value_t value)
	{
		assignSemantics(node, labels(tag), value);
	}

	Node assignSemantics(Node node, std::string const& tag, value_t value,
	                     bool propagate = true)
	{
		return assignSemantics(node, labels(tag), value);
	}

	Node assignSemantics(Code code, std::string const& tag, value_t value,
	                     bool propagate = true)
	{
		return assignSemantics(code, labels(tag), value);
	}

	Node assignSemantics(Key key, std::string const& tag, value_t value,
	                     bool propagate = true)
	{
		return assignSemantics(derived().toCode(key), tag, value, propagate);
	}

	Node assignSemantics(Point coord, std::string const& tag, value_t value,
	                     bool propagate = true, depth_t depth = 0)
	{
		return assignSemantics(derived().toCode(coord, depth), tag, value, propagate);
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	void assignSemantics(Index node, std::string const& tag, UnaryFunction f)
	{
		assignSemantics(node, labels(tag), f);
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	Node assignSemantics(Node node, std::string const& tag, UnaryFunction f,
	                     bool propagate = true)
	{
		return assignSemantics(node, labels(tag), f);
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	Node assignSemantics(Code code, std::string const& tag, UnaryFunction f,
	                     bool propagate = true)
	{
		return assignSemantics(code, labels(tag), f);
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	Node assignSemantics(Key key, std::string const& tag, UnaryFunction f,
	                     bool propagate = true)
	{
		return assignSemantics(derived().toCode(key), tag, f, propagate);
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	Node assignSemantics(Point coord, std::string const& tag, UnaryFunction f,
	                     bool propagate = true, depth_t depth = 0)
	{
		return assignSemantics(derived().toCode(coord, depth), tag, f, propagate);
	}

	Node assignSemantics(Node node, SemanticRange range, value_t value,
	                     bool propagate = true)
	{
		return assignSemantics(node, SemanticRangeSet(range), value, propagate);
	}

	Node assignSemantics(Code code, SemanticRange range, value_t value,
	                     bool propagate = true)
	{
		return assignSemantics(code, SemanticRangeSet(range), value, propagate);
	}

	Node assignSemantics(Key key, SemanticRange range, value_t value, bool propagate = true)
	{
		return assignSemantics(derived().toCode(key), range, value, propagate);
	}

	Node assignSemantics(Point coord, SemanticRange range, value_t value,
	                     bool propagate = true, depth_t depth = 0)
	{
		return assignSemantics(derived().toCode(coord, depth), range, value, propagate);
	}

	void assignSemantics(Index node, SemanticRangeSet const& ranges, value_t value)
	{
		if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
			derived().apply(
			    node,
			    [this, ranges, value](Index node) {
				    semantics_[node.pos].semantic_set.assign(node.offset, ranges, value);
			    },
			    [this, ranges, value](pos_t pos) {
				    semantics_[pos].semantic_set.insertOrAssign(ranges, value);
			    });
		} else {
			for (offset_t i{}; i != N; ++i) {
				assignSemantics(derived().child(node, i), ranges, value);
			}
		}
	}

	Node assignSemantics(Node node, SemanticRangeSet const& ranges, value_t value,
	                     bool propagate = true)
	{
		if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
			return derived().apply(
			           node,
			           [this, ranges, value](Index node) {
				           semantics_[node.pos].semantic_set.assign(node.offset, ranges, value);
			           },
			           [this, ranges, value](pos_t pos) {
				           semantics_[pos].semantic_set.insertOrAssign(ranges, value);
			           }),
			       propagate;
		} else {
			for (offset_t i{}; i != N; ++i) {
				assignSemantics(derived().child(node, i), ranges, value, propagate);
			}
			return node;
		}
	}

	Node assignSemantics(Code code, SemanticRangeSet const& ranges, value_t value,
	                     bool propagate = true)
	{
		if (derived().isLeaf(code) || derived().isPureLeaf(code)) {
			return derived().apply(
			    code,
			    [this, ranges, value](Index node) {
				    semantics_[node.pos].semantic_set.assign(node.offset, ranges, value);
			    },
			    [this, ranges, value](pos_t pos) {
				    semantics_[pos].semantic_set.assign(ranges, value);
			    },
			    propagate);
		} else {
			for (offset_t i{}; i != N; ++i) {
				assignSemantics(code.child(i), ranges, value, propagate);
			}
			return derived()(code);
		}
	}

	Node assignSemantics(Key key, SemanticRangeSet const& ranges, value_t value,
	                     bool propagate = true)
	{
		return assignSemantics(derived().toCode(key), ranges, value, propagate);
	}

	Node assignSemantics(Point coord, SemanticRangeSet const& ranges, value_t value,
	                     bool propagate = true, depth_t depth = 0)
	{
		return assignSemantics(derived().toCode(coord, depth), ranges, value, propagate);
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	Node assignSemantics(Node node, SemanticRange range, UnaryFunction f,
	                     bool propagate = true)
	{
		return assignSemantics(node, SemanticRangeSet(range), f, propagate);
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	Node assignSemantics(Code code, SemanticRange range, UnaryFunction f,
	                     bool propagate = true)
	{
		return assignSemantics(code, SemanticRangeSet(range), f, propagate);
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	Node assignSemantics(Key key, SemanticRange range, UnaryFunction f,
	                     bool propagate = true)
	{
		return assignSemantics(derived().toCode(key), range, f, propagate);
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	Node assignSemantics(Point coord, SemanticRange range, UnaryFunction f,
	                     bool propagate = true, depth_t depth = 0)
	{
		return assignSemantics(derived().toCode(coord, depth), range, f, propagate);
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	void assignSemantics(Index node, SemanticRangeSet const& ranges, UnaryFunction f)
	{
		if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
			derived().apply(
			    node,
			    [this, ranges, f](Index node) {
				    semantics_[node.pos].semantic_set.assign(node.offset, ranges, f);
			    },
			    [this, ranges, f](pos_t pos) {
				    semantics_[pos].semantic_set.insertOrAssign(ranges, f);
			    });
		} else {
			for (offset_t i{}; i != N; ++i) {
				assignSemantics(derived().child(node, i), ranges, f);
			}
		}
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	Node assignSemantics(Node node, SemanticRangeSet const& ranges, UnaryFunction f,
	                     bool propagate = true)
	{
		if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
			return derived().apply(
			    node,
			    [this, ranges, f](Index node) {
				    semantics_[node.pos].semantic_set.assign(node.offset, ranges, f);
			    },
			    [this, ranges, f](pos_t pos) {
				    semantics_[pos].semantic_set.assign(ranges, f);
			    },
			    propagate);
		} else {
			for (offset_t i{}; i != N; ++i) {
				assignSemantics(derived().child(node, i), ranges, f, propagate);
			}
			return node;
		}
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	Node assignSemantics(Code code, SemanticRangeSet const& ranges, UnaryFunction f,
	                     bool propagate = true)
	{
		if (derived().isLeaf(code) || derived().isPureLeaf(code)) {
			return derived().apply(
			    code,
			    [this, ranges, f](Index node) {
				    semantics_[node.pos].semantic_set.assign(node.offset, ranges, f);
			    },
			    [this, ranges, f](pos_t pos) {
				    semantics_[pos].semantic_set.assign(ranges, f);
			    },
			    propagate);
		} else {
			for (offset_t i{}; i != N; ++i) {
				assignSemantics(code.child(i), ranges, f, propagate);
			}
			return derived()(code);
		}
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	Node assignSemantics(Key key, SemanticRangeSet const& ranges, UnaryFunction f,
	                     bool propagate = true)
	{
		return assignSemantics(derived().toCode(key), ranges, f, propagate);
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	Node assignSemantics(Point coord, SemanticRangeSet const& ranges, UnaryFunction f,
	                     bool propagate = true, depth_t depth = 0)
	{
		return assignSemantics(derived().toCode(coord, depth), ranges, f, propagate);
	}

	//
	// Erase
	//

	Node eraseSemantics(label_t label, bool propagate = true)
	{
		return eraseSemantics(derived().rootCode(), label, propagate);
	}

	void eraseSemantics(Index node, label_t label)
	{
		if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
			derived().apply(
			    node,
			    [this, label](Index node) {
				    semantics_[node.pos].semantic_set.erase(node.offset, label);
			    },
			    [this, label](pos_t pos) { semantics_[pos].semantic_set.erase(label); });
		} else {
			for (offset_t i{}; i != N; ++i) {
				eraseSemantics(derived().child(node, i), label);
			}
		}
	}

	Node eraseSemantics(Node node, label_t label, bool propagate = true)
	{
		if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
			return derived().apply(
			    node,
			    [this, label](Index node) {
				    semantics_[node.pos].semantic_set.erase(node.offset, label);
			    },
			    [this, label](pos_t pos) { semantics_[pos].semantic_set.erase(label); },
			    propagate);
		} else {
			for (offset_t i{}; i != N; ++i) {
				eraseSemantics(derived().child(node, i), label, propagate);
			}
			return node;
		}
	}

	Node eraseSemantics(Code code, label_t label, bool propagate = true)
	{
		if (derived().isLeaf(code) || derived().isPureLeaf(code)) {
			return derived().apply(
			    code,
			    [this, label](Index node) {
				    semantics_[node.pos].semantic_set.erase(node.offset, label);
			    },
			    [this, label](pos_t pos) { semantics_[pos].semantic_set.erase(label); },
			    propagate);
		} else {
			for (offset_t i{}; i != N; ++i) {
				eraseSemantics(code.child(i), label, propagate);
			}
			return derived()(code);
		}
	}

	Node eraseSemantics(Key key, label_t label, bool propagate = true)
	{
		return eraseSemantics(derived().toCode(key), label, propagate);
	}

	Node eraseSemantics(Point coord, label_t label, bool propagate = true,
	                    depth_t depth = 0)
	{
		return eraseSemantics(derived().toCode(coord, depth), label, propagate);
	}

	template <class InputIt,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	Node eraseSemantics(InputIt first, InputIt last, bool propagate = true)
	{
		return eraseSemantics(derived().rootCode(), first, last, propagate);
	}

	template <class InputIt,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	Node eraseSemantics(Index node, InputIt first, InputIt last)
	{
		return eraseSemantics(node, SemanticRangeSet(first, last));
	}

	template <class InputIt,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	Node eraseSemantics(Node node, InputIt first, InputIt last, bool propagate = true)
	{
		return eraseSemantics(node, SemanticRangeSet(first, last));
	}

	template <class InputIt,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	Node eraseSemantics(Code code, InputIt first, InputIt last, bool propagate = true)
	{
		return eraseSemantics(code, SemanticRangeSet(first, last));
	}

	template <class InputIt,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	Node eraseSemantics(Key key, InputIt first, InputIt last, bool propagate = true)
	{
		return eraseSemantics(derived().toCode(key), first, last, propagate);
	}

	template <class InputIt,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	Node eraseSemantics(Point coord, InputIt first, InputIt last, bool propagate = true,
	                    depth_t depth = 0)
	{
		return eraseSemantics(derived().toCode(coord, depth), first, last, propagate);
	}

	Node eraseSemantics(std::initializer_list<label_t> ilist, bool propagate = true)
	{
		return eraseSemantics(derived().rootCode(), ilist, propagate);
	}

	void eraseSemantics(Index node, std::initializer_list<label_t> ilist)
	{
		eraseSemantics(node, std::begin(ilist), std::end(ilist));
	}

	Node eraseSemantics(Node node, std::initializer_list<label_t> ilist,
	                    bool propagate = true)
	{
		return eraseSemantics(node, std::begin(ilist), std::end(ilist), propagate);
	}

	Node eraseSemantics(Code code, std::initializer_list<label_t> ilist,
	                    bool propagate = true)
	{
		return eraseSemantics(code, std::begin(ilist), std::end(ilist), propagate);
	}

	Node eraseSemantics(Key key, std::initializer_list<label_t> ilist,
	                    bool propagate = true)
	{
		return eraseSemantics(derived().toCode(key), ilist, propagate);
	}

	Node eraseSemantics(Point coord, std::initializer_list<label_t> ilist,
	                    bool propagate = true, depth_t depth = 0)
	{
		return eraseSemantics(derived().toCode(coord, depth), ilist, propagate);
	}

	Node eraseSemantics(SemanticRange range, bool propagate = true)
	{
		return eraseSemantics(derived().rootCode(), range, propagate);
	}

	void eraseSemantics(Index node, SemanticRange range)
	{
		eraseSemantics(node, SemanticRangeSet(range));
	}

	Node eraseSemantics(Node node, SemanticRange range, bool propagate = true)
	{
		return eraseSemantics(node, SemanticRangeSet(range), propagate);
	}

	Node eraseSemantics(Code code, SemanticRange range, bool propagate = true)
	{
		return eraseSemantics(code, SemanticRangeSet(range), propagate);
	}

	Node eraseSemantics(Key key, SemanticRange range, bool propagate = true)
	{
		return eraseSemantics(derived().toCode(key), range, propagate);
	}

	Node eraseSemantics(Point coord, SemanticRange range, bool propagate = true,
	                    depth_t depth = 0)
	{
		return eraseSemantics(derived().toCode(coord, depth), range, propagate);
	}

	Node eraseSemantics(SemanticRangeSet const& ranges, bool propagate = true)
	{
		return eraseSemantics(derived().rootCode(), ranges, propagate);
	}

	void eraseSemantics(Index node, SemanticRangeSet const& ranges)
	{
		if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
			derived().apply(
			    node,
			    [this, ranges](Index node) {
				    semantics_[node.pos].semantic_set.erase(node.offset, ranges);
			    },
			    [this, ranges](pos_t pos) { semantics_[pos].semantic_set.erase(ranges); });
		} else {
			for (offset_t i{}; i != N; ++i) {
				eraseSemantics(derived().child(node, i), ranges);
			}
		}
	}

	Node eraseSemantics(Node node, SemanticRangeSet const& ranges, bool propagate = true)
	{
		if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
			return derived().apply(
			    node,
			    [this, ranges](Index node) {
				    semantics_[node.pos].semantic_set.erase(node.offset, ranges);
			    },
			    [this, ranges](pos_t pos) { semantics_[pos].semantic_set.erase(ranges); },
			    propagate);
		} else {
			for (offset_t i{}; i != N; ++i) {
				eraseSemantics(derived().child(node, i), ranges, propagate);
			}
			return node;
		}
	}

	Node eraseSemantics(Code code, SemanticRangeSet const& ranges, bool propagate = true)
	{
		if (derived().isLeaf(code) || derived().isPureLeaf(code)) {
			return derived().apply(
			    code,
			    [this, ranges](Index node) {
				    semantics_[node.pos].semantic_set.erase(node.offset, ranges);
			    },
			    [this, ranges](pos_t pos) { semantics_[pos].semantic_set.erase(ranges); },
			    propagate);
		} else {
			for (offset_t i{}; i != N; ++i) {
				eraseSemantics(code.child(i), ranges, propagate);
			}
			return derived()(code);
		}
	}

	Node eraseSemantics(Key key, SemanticRangeSet const& ranges, bool propagate = true)
	{
		return eraseSemantics(derived().toCode(key), ranges, propagate);
	}

	Node eraseSemantics(Point coord, SemanticRangeSet const& ranges, bool propagate = true,
	                    depth_t depth = 0)
	{
		return eraseSemantics(derived().toCode(coord, depth), ranges, propagate);
	}

	Node eraseSemantics(std::string const& tag, bool propagate = true)
	{
		return eraseSemantics(derived().rootCode(), tag, propagate);
	}

	void eraseSemantics(Index node, std::string const& tag)
	{
		eraseSemantics(node, labels(tag));
	}

	Node eraseSemantics(Node node, std::string const& tag, bool propagate = true)
	{
		return eraseSemantics(node, labels(tag));
	}

	Node eraseSemantics(Code code, std::string const& tag, bool propagate = true)
	{
		return eraseSemantics(code, labels(tag));
	}

	Node eraseSemantics(Key key, std::string const& tag, bool propagate = true)
	{
		return eraseSemantics(derived().toCode(key), tag, propagate);
	}

	Node eraseSemantics(Point coord, std::string const& tag, bool propagate = true,
	                    depth_t depth = 0)
	{
		return eraseSemantics(derived().toCode(coord, depth), tag, propagate);
	}

	//
	// Erase if
	//

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	Node eraseIfSemantics(UnaryPredicate p, bool propagate = true)
	{
		return eraseIfSemantics(derived().rootCode(), p, propagate);
	}

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	void eraseIfSemantics(Index node, UnaryPredicate p)
	{
		if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
			derived().apply(
			    node,
			    [this, p](Index node) {
				    semantics_[node.pos].semantic_set.eraseIf(node.offset, p);
			    },
			    [this, p](pos_t pos) { semantics_[pos].semantic_set.eraseIf(p); });
		} else {
			for (offset_t i{}; i != N; ++i) {
				eraseIfSemantics(derived().child(node, i), p);
			}
		}
	}

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	Node eraseIfSemantics(Node node, UnaryPredicate p, bool propagate = true)
	{
		if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
			return derived().apply(
			    node,
			    [this, p](Index node) {
				    semantics_[node.pos].semantic_set.eraseIf(node.offset, p);
			    },
			    [this, p](pos_t pos) { semantics_[pos].semantic_set.eraseIf(p); }, propagate);
		} else {
			for (offset_t i{}; i != N; ++i) {
				eraseIfSemantics(derived().child(node, i), p, propagate);
			}
			return node;
		}
	}

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	Node eraseIfSemantics(Code code, UnaryPredicate p, bool propagate = true)
	{
		if (derived().isLeaf(code) || derived().isPureLeaf(code)) {
			return derived().apply(
			    code,
			    [this, p](Index node) {
				    semantics_[node.pos].semantic_set.eraseIf(node.offset, p);
			    },
			    [this, p](pos_t pos) { semantics_[pos].semantic_set.eraseIf(p); }, propagate);
		} else {
			for (offset_t i{}; i != N; ++i) {
				eraseIfSemantics(code.child(i), p, propagate);
			}
			return derived()(code);
		}
	}

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	Node eraseIfSemantics(Key key, UnaryPredicate p, bool propagate = true)
	{
		return eraseIfSemantics(derived().toCode(key), p, propagate);
	}

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	Node eraseIfSemantics(Point coord, UnaryPredicate p, bool propagate = true,
	                      depth_t depth = 0)
	{
		return eraseIfSemantics(derived().toCode(coord, depth), p, propagate);
	}

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	Node eraseIfSemantics(std::string const& tag, UnaryPredicate p, bool propagate = true)
	{
		return eraseIfSemantics(derived().rootCode(), tag, p, propagate);
	}

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	void eraseIfSemantics(Index node, std::string const& tag, UnaryPredicate p)
	{
		eraseIfSemantics(node, labels(tag), p);
	}

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	Node eraseIfSemantics(Node node, std::string const& tag, UnaryPredicate p,
	                      bool propagate = true)
	{
		return eraseIfSemantics(node, labels(tag), p);
	}

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	Node eraseIfSemantics(Code code, std::string const& tag, UnaryPredicate p,
	                      bool propagate = true)
	{
		return eraseIfSemantics(code, labels(tag), p);
	}

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	Node eraseIfSemantics(Key key, std::string const& tag, UnaryPredicate p,
	                      bool propagate = true)
	{
		return eraseIfSemantics(derived().toCode(key), tag, p, propagate);
	}

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	Node eraseIfSemantics(Point coord, std::string const& tag, UnaryPredicate p,
	                      bool propagate = true, depth_t depth = 0)
	{
		return eraseIfSemantics(derived().toCode(coord, depth), tag, p, propagate);
	}

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	Node eraseIfSemantics(SemanticRange range, UnaryPredicate p, bool propagate = true)
	{
		return eraseIfSemantics(derived().rootCode(), range, p, propagate);
	}

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	void eraseIfSemantics(Index node, SemanticRange range, UnaryPredicate p)
	{
		eraseIfSemantics(node, SemanticRangeSet(range), p);
	}

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	Node eraseIfSemantics(Node node, SemanticRange range, UnaryPredicate p,
	                      bool propagate = true)
	{
		return eraseIfSemantics(node, SemanticRangeSet(range), p, propagate);
	}

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	Node eraseIfSemantics(Code code, SemanticRange range, UnaryPredicate p,
	                      bool propagate = true)
	{
		return eraseIfSemantics(code, SemanticRangeSet(range), p, propagate);
	}

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	Node eraseIfSemantics(Key key, SemanticRange range, UnaryPredicate p,
	                      bool propagate = true)
	{
		return eraseIfSemantics(derived().toCode(key), range, p, propagate);
	}

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	Node eraseIfSemantics(Point coord, SemanticRange range, UnaryPredicate p,
	                      bool propagate = true, depth_t depth = 0)
	{
		return eraseIfSemantics(derived().toCode(coord, depth), range, p, propagate);
	}

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	Node eraseIfSemantics(SemanticRangeSet const& ranges, UnaryPredicate p,
	                      bool propagate = true)
	{
		return eraseIfSemantics(derived().rootCode(), ranges, p, propagate);
	}

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	void eraseIfSemantics(Index node, SemanticRangeSet const& ranges, UnaryPredicate p)
	{
		if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
			derived().apply(
			    node,
			    [this, ranges, p](Index node) {
				    semantics_[node.pos].semantic_set.eraseIf(node.offset, ranges, p);
			    },
			    [this, ranges, p](pos_t pos) {
				    semantics_[pos].semantic_set.eraseIf(ranges, p);
			    });
		} else {
			for (offset_t i{}; i != N; ++i) {
				eraseIfSemantics(derived().child(node, i), ranges, p);
			}
		}
	}

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	Node eraseIfSemantics(Node node, SemanticRangeSet const& ranges, UnaryPredicate p,
	                      bool propagate = true)
	{
		if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
			return derived().apply(
			    node,
			    [this, ranges, p](Index node) {
				    semantics_[node.pos].semantic_set.eraseIf(node.offset, ranges, p);
			    },
			    [this, ranges, p](pos_t pos) {
				    semantics_[pos].semantic_set.eraseIf(ranges, p);
			    },
			    propagate);
		} else {
			for (offset_t i{}; i != N; ++i) {
				eraseIfSemantics(derived().child(node, i), ranges, p, propagate);
			}
			return node;
		}
	}

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	Node eraseIfSemantics(Code code, SemanticRangeSet const& ranges, UnaryPredicate p,
	                      bool propagate = true)
	{
		if (derived().isLeaf(code) || derived().isPureLeaf(code)) {
			return derived().apply(
			    code,
			    [this, ranges, p](Index node) {
				    semantics_[node.pos].semantic_set.eraseIf(node.offset, ranges, p);
			    },
			    [this, ranges, p](pos_t pos) {
				    semantics_[pos].semantic_set.eraseIf(ranges, p);
			    },
			    propagate);
		} else {
			for (offset_t i{}; i != N; ++i) {
				eraseIfSemantics(code.child(i), ranges, p, propagate);
			}
			return derived()(code);
		}
	}

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	Node eraseIfSemantics(Key key, SemanticRangeSet const& ranges, UnaryPredicate p,
	                      bool propagate = true)
	{
		return eraseIfSemantics(derived().toCode(key), ranges, p, propagate);
	}

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	Node eraseIfSemantics(Point coord, SemanticRangeSet const& ranges, UnaryPredicate p,
	                      bool propagate = true, depth_t depth = 0)
	{
		return eraseIfSemantics(derived().toCode(coord, depth), ranges, p, propagate);
	}

	//
	// Clear
	//

	void clearImpl()
	{
		semantics_.resize(1);
		semantics_.assign(1, Data());
	}

	void clearImpl(pos_t block)
	{
		semantics_[block].semantic_set.clear();
		semantics_[block].summary = Semantic();
	}

	void clearSemantics(bool propagate = true)
	{
		clearSemantics(derived().rootCode(), propagate);
	}

	void clearSemantics(Index node)
	{
		if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
			derived().apply(
			    node,
			    [this](Index node) {
				    if (derived().isPureLeaf(node)) {
					    semantics_[node.pos].semantic_set.clear(node.offset);
				    }
			    },
			    [this](pos_t block) { clearImpl(block); });
		} else {
			for (offset_t i{}; i != N; ++i) {
				clearSemantics(derived().child(node, i));
			}
		}
	}

	void clearSemantics(Node node, bool propagate = true)
	{
		if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
			derived().apply(
			    node,
			    [this](Index node) {
				    if (derived().isPureLeaf(node)) {
					    semantics_[node.pos].semantic_set.clear(node.offset);
				    }
			    },
			    [this](pos_t block) { clearImpl(block); }, propagate);
		} else {
			for (offset_t i{}; i != N; ++i) {
				clearSemantics(derived().child(node, i), propagate);
			}
		}
	}

	void clearSemantics(Code code, bool propagate = true)
	{
		if (derived().isLeaf(code) || derived().isPureLeaf(code)) {
			derived().apply(
			    code,
			    [this](Index node) {
				    if (derived().isPureLeaf(node)) {
					    semantics_[node.pos].semantic_set.clear(node.offset);
				    }
			    },
			    [this](pos_t block) { clearImpl(block); }, propagate);
		} else {
			for (offset_t i{}; i != N; ++i) {
				clearSemantics(code.child(i), propagate);
			}
		}
	}

	void clearSemantics(Key key, bool propagate = true)
	{
		clearSemantics(derived().toCode(key), propagate);
	}

	void clearSemantics(Point coord, bool propagate = true, depth_t depth = 0)
	{
		clearSemantics(derived().toCode(coord, depth), propagate);
	}

	//
	// Change semantic label
	//

	void changeSemantics(label_t old_label, label_t new_label, bool propagate = true)
	{
		changeLabel(derived().rootCode(), old_label, new_label, propagate);
	}

	void changeSemantics(Node node, label_t old_label, label_t new_label)
	{
		if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
			derived().apply(
			    node,
			    [this, old_label, new_label](Index node) {
				    semantics_[node.pos].semantic_set.changeLabel(node.offset, old_label,
				                                                  new_label);
			    },
			    [this, old_label, new_label](pos_t pos) {
				    semantics_[pos].semantic_set.changeLabel(old_label, new_label);
			    });
		} else {
			for (offset_t i{}; i != N; ++i) {
				changeSemantics(derived().child(node, i), old_label, new_label);
			}
		}
	}

	void changeSemantics(Node node, label_t old_label, label_t new_label,
	                     bool propagate = true)
	{
		if (derived().isLeaf(node) || derived().isPureLeaf(node)) {
			derived().apply(
			    node,
			    [this, old_label, new_label](Index node) {
				    semantics_[node.pos].semantic_set.changeLabel(node.offset, old_label,
				                                                  new_label);
			    },
			    [this, old_label, new_label](pos_t pos) {
				    semantics_[pos].semantic_set.changeLabel(old_label, new_label);
			    },
			    propagate);
		} else {
			for (offset_t i{}; i != N; ++i) {
				changeSemantics(derived().child(node, i), old_label, new_label, propagate);
			}
		}
	}

	void changeSemantics(Code code, label_t old_label, label_t new_label,
	                     bool propagate = true)
	{
		if (derived().isLeaf(code) || derived().isPureLeaf(code)) {
			derived().apply(
			    code,
			    [this, old_label, new_label](Index node) {
				    semantics_[node.pos].semantic_set.changeLabel(node.offset, old_label,
				                                                  new_label);
			    },
			    [this, old_label, new_label](pos_t pos) {
				    semantics_[pos].semantic_set.changeLabel(old_label, new_label);
			    },
			    propagate);
		} else {
			for (offset_t i{}; i != N; ++i) {
				changeSemantics(code.child(i), old_label, new_label, propagate);
			}
		}
	}

	void changeSemantics(Key key, label_t old_label, label_t new_label,
	                     bool propagate = true)
	{
		changeSemantics(derived().toCode(key), old_label, new_label, propagate);
	}

	void changeSemantics(Point coord, label_t old_label, label_t new_label,
	                     bool propagate = true, depth_t depth = 0)
	{
		changeSemantics(derived().toCode(coord, depth), old_label, new_label, propagate);
	}

	//
	// Propagation criteria
	//

	[[nodiscard]] constexpr PropagationCriteria semanticsPropagationCriteria()
	    const noexcept
	{
		return prop_criteria_;
	}

	void setSemanticsPropagationCriteria(PropagationCriteria prop_criteria,
	                                     bool                propagate = true)
	{
		if (prop_criteria_ == prop_criteria) {
			return;
		}

		prop_criteria_ = prop_criteria;

		derived().setModified();

		if (propagate) {
			derived().propagateModified();
		}
	}

 protected:
	//
	// Constructors
	//

	SemanticSetMap()
	{
		// semantics_.reserve(10);
		semantics_.emplace_back();
	};

	SemanticSetMap(SemanticSetMap const& other) = default;

	SemanticSetMap(SemanticSetMap&& other) = default;

	// template <class Derived2>
	// SemanticSetMap(SemanticSetMap<Derived2> const& other) : Derived2  // TODO: Fill in
	// {
	// }

	//
	// Destructor
	//

	~SemanticSetMap() {}

	//
	// Assignment operator
	//

	SemanticSetMap& operator=(SemanticSetMap const& rhs) = default;

	SemanticSetMap& operator=(SemanticSetMap&& rhs) = default;

	template <class Derived2, size_t N2>
	SemanticSetMap& operator=(SemanticSetMap<Derived22> const& rhs)
	{
		// TODO: Fill in
		return *this;
	}

	//
	// Swap
	//

	void swap(SemanticSetMap& other) noexcept
	{
		std::swap(semantics_, other.semantics_);
		std::swap(mapping_, other.mapping_);
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
	// Create node block
	//

	void createBlock(Index node)
	{
		// TODO: Implement
		value_t value_summary{};
		switch (...) {
			for (auto s : ...) {
				value_summary;
			}
		}
		semantics_.emplace_back(semantics_[node.pos].semantics.subset(node.offset),
		                        semantics_[node.pos].label_summary[node.offset],
		                        value_summary);

		// TODO: Should the semantics in the parent be removed?
		semanticsClear(node);
	}

	//
	// Fill
	//

	void fill(Index node, pos_t children)
	{
		// TODO: Implement

		switch (semanticsPropagationCriteria()) {
			case PropagationCriteria::MIN:
			case PropagationCriteria::MAX:
				semantics_[children].semantic_set.fill(semantics_[node.pos].semantic_set,
				                                       node.offset);
				break;
			case PropagationCriteria::NONE:
			case PropagationCriteria::S_MIN:
			case PropagationCriteria::S_MAX:
				if (derived().allPureLeaf(children) || derived().allLeaf(children)) {
					// transfer semantics to all children
					semantics_[children].semantic_set.fill(semantics_[node.pos].semantic_set,
					                                       node.offset);
					semantics_[children].summary = semantics_[node.pos].summary;
					// remove semantics from node since it is now a inner node without semantic set
					// can keep the summary
					semantics_[node.pos].semantic_set.clear(node.offset);
				} else {
					// TODO: is fill only called when all children are not parents already?
					// can it happen that one of the children is not a leaf?
					assert(false);
				}
		}

		// TODO: Should the semantics in the parent be removed?
		semanticsClear(node);
	}

	//
	// Resize
	//

	void resize(std::size_t count) { semantics_.resize(count); }

	//
	// Reserve
	//

	void reserveImpl(std::size_t new_cap) { semantics_.reserve(new_cap); }

	//
	// Initilize root
	//

	void initRoot()
	{
		// semanticNode(derived().root()).clear();
		// auto node = derived().rootIndex();
		// semantics_[node.pos].clear();
		clearImpl();
	}

	//
	// Update block
	//

	void updateBlock(pos_t block, std::array<bool, N> modified_parent)
	{
		// TODO: Can we have this here?
		if (derived().noneParent(block)) {
			return;
		}

		for (offset_t i{}; N != i; ++i) {
			Index node{block, i};
			if (modified_parent[i]) {
				updateNode(node, derived().children(node));
			}
		}

		auto children = derived().children(block);
		// TODO constexpr 1 == N
		switch (semanticsPropagationCriteria()) {
			case PropagationCriteria::NONE: {
				break;
			}
			case PropagationCriteria::S_MAX: {
				// if nodes is leaf or pure leaf -> no offset has children
				// update its summary: or all semantics in each offset of nodes and put it into
				// summary of nodes

				// if nodes is inner node -> some offset might be a parent, some might be a Leaf
				// or pure leaf or summary of all children that are not NULLPOS (offset is parent,
				// we can use summary of child) and put it into summary of nodes if NULLPOS
				// (offset is not a parent, no summary we can use here) we need to look at
				// semantics, take care of max

				// TODO: possible improvement, check if all pure leaf or leaf, can treat all
				// semantics at the same time

				label_t summary_label = 0;
				value_t summary_value = std::numeric_limits<value_t>::min();

				for (offset_t i{}; i != N; ++i) {
					pos_t child = children[i];
					if (child == NULL_POS) {
						// i is not a parent, look at semantics of i
						std::vector vec(semantics_[block].semantic_set.begin(i),
						                semantics_[block].semantic_set.end(i));
						if (!vec.empty()) {
							// keep max value of all semantics
							std::sort(std::begin(vec), std::end(vec),
							          [](auto a, auto b) { return a.value > b.value; });
							summary_value = std::max(summary_value, vec.front().value);

							// make the summary or
							auto last = std::unique(std::begin(vec), std::end(vec),
							                        [](auto a, auto b) { return a.label == b.label; });

							label_t label = 0;
							for (auto it = vec.begin(); it != last; ++it) {
								label |= it->label;
							}
							summary_label |= label;
						}
					} else {
						// i is parent, look at summary of child
						summary_label |= semantics_[child].summary.label;
						summary_value = std::max(summary_value, semantics_[child].summary.value);
					}
				}

				semantics_[block].summary.label = summary_label;
				semantics_[block].summary.value = summary_value;

				break;
			}
			case PropagationCriteria::S_MIN: {
				label_t summary_label = 0;
				value_t summary_value = std::numeric_limits<value_t>::max();

				for (offset_t i{}; i != N; ++i) {
					pos_t child = children[i];
					if (child == NULL_POS) {
						// i is not a parent, look at semantics of i
						std::vector vec(semantics_[block].semantic_set.begin(i),
						                semantics_[block].semantic_set.end(i));

						if (!vec.empty()) {
							// keep min value of all semantics
							std::sort(std::begin(vec), std::end(vec),
							          [](auto a, auto b) { return a.value < b.value; });
							summary_value = std::min(summary_value, vec.front().value);

							// make the summary or
							auto last = std::unique(std::begin(vec), std::end(vec),
							                        [](auto a, auto b) { return a.label == b.label; });

							label_t label = 0;
							for (auto it = vec.begin(); it != last; ++it) {
								label |= it->label;
							}
							summary_label |= label;
						}
					} else {
						// i is parent, look at summary of child
						summary_label |= semantics_[child].summary.label;
						summary_value = std::min(summary_value, semantics_[child].summary.value);
					}
				}

				semantics_[block].summary.label = summary_label;
				semantics_[block].summary.value = summary_value;

				break;
			}
			case PropagationCriteria::MIN: {
				// label_t summary_label = 0;
				// value_t summary_value = std::numeric_limits<value_t>::max();

				for (offset_t i{}; i != N; ++i) {
					pos_t child = children[i];
					if (child == NULL_POS) {
						continue;
					}

					std::vector vec(semantics_[child].semantic_set.begin(),
					                semantics_[child].semantic_set.end());
					std::sort(std::begin(vec), std::end(vec), [](auto a, auto b) {
						return a.label < b.label || (a.label == b.label && a.value < b.value);
					});
					// summary_value = std::min(summary_value, vec.front().value);

					auto r_last = std::unique(std::begin(vec), std::end(vec),
					                          [](auto a, auto b) { return a.label == b.label; });
					// auto first  = r_last.base();
					// auto last   = std::end(vec);
					semantics_[block].semantic_set.set(i, std::begin(vec), r_last);

					// // summary
					// label_t label = 0;
					// for (auto it = vec.begin(); it != last; ++it) {
					// 	label |= it->label;
					// }
					// summary_label |= label;
				}

				// semantics_[nodes].summary.label = summary_label;
				// semantics_[nodes].summary.value = summary_value;
				break;
			}
			case PropagationCriteria::MAX: {
				// label_t summary_label = 0;
				// value_t summary_value = std::numeric_limits<value_t>::min();

				for (offset_t i{}; i != N; ++i) {
					pos_t child = children[i];
					if (child == NULL_POS) {
						continue;
					}

					std::vector vec(semantics_[child].semantic_set.begin(),
					                semantics_[child].semantic_set.end());
					std::sort(std::begin(vec), std::end(vec), [](auto a, auto b) {
						return a.label < b.label || (a.label == b.label && a.value > b.value);
					});
					// summary_value = std::max(summary_value, vec.front().value);

					auto r_last = std::unique(std::begin(vec), std::end(vec),
					                          [](auto a, auto b) { return a.label == b.label; });
					// auto first  = r_last.base();
					// auto last   = std::end(vec);
					// semantics_[nodes].semantic_set.set(i, first, last);
					semantics_[block].semantic_set.set(i, std::begin(vec), r_last);

					// // summary
					// label_t label = 0;
					// for (auto it = vec.begin(); it != last; ++it) {
					// 	label |= it->label;
					// }
					// summary_label |= label;
				}

				// semantics_[nodes].summary.label = summary_label;
				// semantics_[nodes].summary.value = summary_value;
				break;
			}
		}
	}

	//
	// Is prunable
	//

	[[nodiscard]] bool isPrunable(pos_t block) const
	{
		for (offset_t i{1}; N != i; ++i) {
			if (!std::equal(
			        semantics_[block].semantics.begin(0), semantics_[block].semantics.end(0),
			        semantics_[block].semantics.begin(i), semantics_[block].semantics.end(i))) {
				return false;
			}
		}
		return true;
	}

	void preparePrune(Index node)
	{
		auto children = derived().children(node);

		// move content of children to node, node will be a leaf afterwards and needs the
		// actual data
		// all children have the same data by definition, so we can just take the first
		semantics_[node.pos].semantic_set.set(node.offset,
		                                      semantics_[children].semantic_set.begin(0),
		                                      semantics_[children].semantic_set.end(0));
	}

	//
	// Memory
	//

	[[nodiscard]] std::size_t sizeofNodeTimesN(Index node) const
	{
		// TODO: Implement
	}

	[[nodiscard]] std::size_t sizeofBlock(pos_t block) const
	{
		// TODO: Implement
	}

	[[nodiscard]] static constexpr std::size_t sizeofBlockLowerBound() noexcept
	{
		return sizeof(typename decltype(semantics_)::value_type);
	}

	[[nodiscard]] static constexpr std::size_t sizeofMap() noexcept
	{
		return sizeof(semantics_) + sizeof(mapping_) + sizeof(prop_criteria_);
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr MapType mapType() noexcept
	{
		return MapType::SEMANTIC_SET;
	}

	template <class Container>
	constexpr std::size_t serializedSize(Container const& c) const
	{
		std::size_t s{};
		for (auto const [block, inner_offsets, value_offsets] : c) {
			if (value_offsets.none()) {
				continue;
			}
			s += sizeof(std::uint8_t);
			if (derived().allLeaf(block)) {
				s += semantics_[block].semantics.serializedSize();
			} else {
				for (offset_t i{}; N != i; ++i) {
					if (!value_offsets[i]) {
						continue;
					}

					// This should be correct in both cases
					s += semantics_[block].semantics.serializedSize(i);
				}
			}
		}
		return s;
	}

	template <class Container>
	void readNodes(ReadBuffer& in, Container const& c)
	{
		for (auto const [block, inner_offsets, value_offsets] : c) {
			if (value_offsets.none()) {
				return;
			}

			std::uint8_t mode;
			in.read(&mode, sizeof(mode));
			if (0 == mode) {
				// All are serialized together
				semantics_[block].semantics.read(in, value_offsets);
			} else {
				// Each serialized seperate

				// TODO: Optimize
				SemanticSet<1> s;
				for (offset_t i{}; N != i; ++i) {
					if (!value_offsets[i]) {
						continue;
					}

					s.read(in);
					semantics_[block].semantics.set(i, s);
				}
			}
		}
	}

	template <class BlockRange>
	void writeBlocks(WriteBuffer& out, BlockRange const& blocks) const
	{
		for (auto block : blocks) {
			if (value_offsets.none()) {
				continue;
			}

			if (derived().allLeaf(block)) {
				std::uint8_t mode = 0;  // All serialized together
				out.write(&mode, sizeof(mode));
				semantics_[block].semantics.write(out);
			} else {
				std::uint8_t mode = 1;  // Each serialized seperate
				out.write(&mode, sizeof(mode));

				for (offset_t i{}; N != i; ++i) {
					if (!value_offsets[i]) {
						continue;
					}

					Index node(block, i);
					if (derived().isLeaf(node)) {
						semantics_[block].semantics.write(out, i);
					} else {
						semantics(node).write(out);
					}
				}
			}
		}
	}

	//
	// Dot file info
	//

	void dotFileInfo(std::ostream& out, Index node) const
	{
		if (derived().isLeaf(node)) {
			out << "Semantics: " << semantics(node);
		} else {
			out << "Semantic Summary: " << semanticsSummary(node);
		}
	}

 protected:
	Container<SemanticSetBlock<N>> semantics_;

	// Propagation
	PropagationCriteria prop_criteria_ = PropagationCriteria::MAX;

	template <class Derived2, std::size_t N2>
	friend class SemanticSetMap;
};
}  // namespace ufo

namespace ufo
{
//
// Type traits
//

template <class Map>
struct is_semantic_set_map
    : std::conditional_t<is_map_type_v<Map, MapType::SEMANTIC_SET>, std::true_type,
                         std::false_type> {
};
template <class Map>
inline constexpr bool is_semantic_set_map_v = is_semantic_set_map<Map>::value;
}  // namespace ufo
#endif  // UFO_MAP_SEMANTIC_MAP_H