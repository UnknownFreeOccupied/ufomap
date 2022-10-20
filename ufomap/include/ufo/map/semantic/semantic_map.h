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

#ifndef UFO_MAP_SEMANTIC_MAP_H
#define UFO_MAP_SEMANTIC_MAP_H

// UFO
#include <ufo/container/range_map.h>
#include <ufo/map/semantic/semantic.h>
#include <ufo/map/semantic/semantic_node.h>
#include <ufo/map/semantic/semantic_predicate.h>
#include <ufo/map/semantic/semantics.h>
#include <ufo/map/types.h>

// STL
#include <algorithm>
#include <functional>
#include <set>
#include <unordered_map>

namespace ufo::map
{
template <class Derived>
class SemanticMap
{
 public:
	//
	// Get semantics
	//

	[[nodiscard]] Semantics semantics(Node node) const
	{
		return derived().leafNode(node).semantics(node.index());
	}

	[[nodiscard]] Semantics semantics(Code code) const
	{
		auto [node, depth] = derived().leafNodeAndDepth(code);
		return node.semantics(code.index(depth));
	}

	[[nodiscard]] Semantics semantics(Key key) const
	{
		return semantics(derived().toCode(key));
	}

	[[nodiscard]] Semantics semantics(Point coord, depth_t depth = 0) const
	{
		return semantics(derived().toCode(coord, depth));
	}

	[[nodiscard]] Semantics semantics(coord_t x, coord_t y, coord_t z,
	                                  depth_t depth = 0) const
	{
		return semantics(derived().toCode(x, y, z, depth));
	}

	//
	// Get semantic
	//

	[[nodiscard]] std::optional<Semantic> semantic(Node node, label_t label) const
	{
		return semanticNode(derived().leafNode(node)).at(node.index(), label);
	}

	[[nodiscard]] std::optional<Semantic> semantic(Code code, label_t label) const
	{
		auto [node, depth] = derived().leafNodeAndDepth(code);
		return semanticNode(node).at(code.index(depth), label);
	}

	[[nodiscard]] std::optional<Semantic> semantic(Key key, label_t label) const
	{
		return semantic(derived().toCode(key), label);
	}

	[[nodiscard]] std::optional<Semantic> semantic(Point coord, label_t label,
	                                               depth_t depth = 0) const
	{
		return semantic(derived().toCode(coord, depth), label);
	}

	[[nodiscard]] std::optional<Semantic> semantic(coord_t x, coord_t y, coord_t z,
	                                               label_t label, depth_t depth = 0) const
	{
		return semantic(derived().toCode(x, y, z, depth), label);
	}

	//
	// Set semantics
	//

	void setSemantics(Node node, Semantics const& semantics, bool propagate = true)
	{
		derived().apply(
		    node,
		    [&semantics](auto& node, index_t index) {
			    semanticNode(node).set(index, semantics);
		    },
		    [&semantics](auto& node) { semanticNode(node).set(semantics); }, propagate);
	}

	void setSemantics(Code code, Semantics const& semantics, bool propagate = true)
	{
		derived().apply(
		    code,
		    [&semantics](auto& node, index_t index) {
			    semanticNode(node).set(index, semantics);
		    },
		    [&semantics](auto& node) { semanticNode(node).set(semantics); }, propagate);
	}

	void setSemantics(Key key, Semantics const& semantics, bool propagate = true)
	{
		setSemantics(derived().toCode(key), semantics, propagate);
	}

	void setSemantics(Point coord, Semantics const& semantics, bool propagate = true,
	                  depth_t depth = 0)
	{
		setSemantics(derived().toCode(coord, depth), semantics, propagate);
	}

	void setSemantics(coord_t x, coord_t y, coord_t z, Semantics const& semantics,
	                  bool propagate = true, depth_t depth = 0)
	{
		setSemantics(derived().toCode(x, y, z, depth), semantics, propagate);
	}

	//
	// Insert semantics
	//

	void insertSemantics(Node node, label_t label, value_t value, bool propagate = true)
	{
		derived().apply(
		    node,
		    [label, value](auto& node, index_t index) {
			    semanticNode(node).insert(index, label, value);
		    },
		    [label, value](auto& node) { semanticNode(node).insert(label, value); },
		    propagate);
	}

	void insertSemantics(Code code, label_t label, value_t value, bool propagate = true)
	{
		derived().apply(
		    code,
		    [label, value](auto& node, index_t index) {
			    semanticNode(node).insert(index, label, value);
		    },
		    [label, value](auto& node) { semanticNode(node).insert(label, value); },
		    propagate);
	}

	void insertSemantics(Key key, label_t label, value_t value, bool propagate = true)
	{
		insertSemantics(derived().toCode(key), label, value, propagate);
	}

	void insertSemantics(Point coord, label_t label, value_t value, bool propagate = true,
	                     depth_t depth = 0)
	{
		insertSemantics(derived().toCode(coord, depth), label, value, propagate);
	}

	void insertSemantics(coord_t x, coord_t y, coord_t z, label_t label, value_t value,
	                     bool propagate = true, depth_t depth = 0)
	{
		insertSemantics(derived().toCode(x, y, z, depth), label, value, propagate);
	}

	void insertSemantics(Node node, Semantic semantic, bool propagate = true)
	{
		insertSemantics(node, semantic.label, semantic.value, propagate);
	}

	void insertSemantics(Code code, Semantic semantic, bool propagate = true)
	{
		insertSemantics(code, semantic.label, semantic.value, propagate);
	}

	void insertSemantics(Key key, Semantic semantic, bool propagate = true)
	{
		insertSemantics(derived().toCode(key), semantic, propagate);
	}

	void insertSemantics(Point coord, Semantic semantic, bool propagate = true,
	                     depth_t depth = 0)
	{
		insertSemantics(derived().toCode(coord, depth), semantic, propagate);
	}

	void insertSemantics(coord_t x, coord_t y, coord_t z, Semantic semantic,
	                     bool propagate = true, depth_t depth = 0)
	{
		insertSemantics(derived().toCode(x, y, z, depth), semantic, propagate);
	}

	template <class InputIt>
	void insertSemantics(Node node, InputIt first, InputIt last, bool propagate = true)
	{
		derived().apply(
		    node,
		    [first, last](auto& node, index_t index) {
			    semanticNode(node).insert(index, first, last);
		    },
		    [first, last](auto& node) { semanticNode(node).insert(first, last); }, propagate);
	}

	template <class InputIt>
	void insertSemantics(Code code, InputIt first, InputIt last, bool propagate = true)
	{
		derived().apply(
		    code,
		    [first, last](auto& node, index_t index) {
			    semanticNode(node).insert(index, first, last);
		    },
		    [first, last](auto& node) { semanticNode(node).insert(first, last); }, propagate);
	}

	template <class InputIt>
	void insertSemantics(Key key, InputIt first, InputIt last, bool propagate = true)
	{
		insertSemantics(derived().toCode(key), first, last, propagate);
	}

	template <class InputIt>
	void insertSemantics(coord_t x, coord_t y, coord_t z, InputIt first, InputIt last,
	                     bool propagate = true, depth_t depth = 0)
	{
		insertSemantics(derived().toCode(x, y, z, depth), first, last, propagate);
	}

	template <class InputIt>
	void insertSemantics(Point coord, InputIt first, InputIt last, bool propagate = true,
	                     depth_t depth = 0)
	{
		insertSemantics(derived().toCode(coord, depth), first, last, propagate);
	}

	void insertSemantics(Node node, std::initializer_list<Semantic> ilist,
	                     bool propagate = true)
	{
		insertSemantics(node, std::cbegin(ilist), std::cend(ilist), bool propagate = true);
	}

	void insertSemantics(Code code, std::initializer_list<Semantic> ilist,
	                     bool propagate = true)
	{
		insertSemantics(code, std::cbegin(ilist), std::cend(ilist), bool propagate = true);
	}

	void insertSemantics(Key key, std::initializer_list<Semantic> ilist,
	                     bool propagate = true)
	{
		insertSemantics(derived().toCode(key), ilist, propagate);
	}

	void insertSemantics(Point coord, std::initializer_list<Semantic> ilist,
	                     bool propagate = true, depth_t depth = 0)
	{
		insertSemantics(derived().toCode(coord, depth), ilist, propagate);
	}

	void insertSemantics(coord_t x, coord_t y, coord_t z,
	                     std::initializer_list<Semantic> ilist, bool propagate = true,
	                     depth_t depth = 0)
	{
		insertSemantics(derived().toCode(x, y, z, depth), ilist, propagate);
	}

	//
	// Insert or assign semantics
	//

	void insertOrAssignSemantics(Node node, label_t label, value_t value,
	                             bool propagate = true)
	{
		derived().apply(
		    node,
		    [label, value](auto& node, index_t index) {
			    semanticNode(node).insertOrAssign(index, label, value);
		    },
		    [label, value](auto& node) { semanticNode(node).insertOrAssign(label, value); },
		    propagate);
	}

	void insertOrAssignSemantics(Code code, label_t label, value_t value,
	                             bool propagate = true)
	{
		derived().apply(
		    code,
		    [label, value](auto& node, index_t index) {
			    semanticNode(node).insertOrAssign(index, label, value);
		    },
		    [label, value](auto& node) { semanticNode(node).insertOrAssign(label, value); },
		    propagate);
	}

	void insertOrAssignSemantics(Key key, label_t label, value_t value,
	                             bool propagate = true)
	{
		insertOrAssignSemantics(derived().toCode(key), label, value, propagate);
	}

	void insertOrAssignSemantics(Point coord, label_t label, value_t value,
	                             bool propagate = true, depth_t depth = 0)
	{
		insertOrAssignSemantics(derived().toCode(coord, depth), label, value, propagate);
	}

	void insertOrAssignSemantics(coord_t x, coord_t y, coord_t z, label_t label,
	                             value_t value, bool propagate = true, depth_t depth = 0)
	{
		insertOrAssignSemantics(derived().toCode(x, y, z, depth), label, value, propagate);
	}

	void insertOrAssignSemantics(Node node, Semantic semantic, bool propagate = true)
	{
		insertOrAssignSemantics(node, semantic.label, semantic.value, propagate);
	}

	void insertOrAssignSemantics(Code code, Semantic semantic, bool propagate = true)
	{
		insertOrAssignSemantics(code, semantic.label, semantic.value, propagate);
	}

	void insertOrAssignSemantics(Key key, Semantic semantic, bool propagate = true)
	{
		insertOrAssignSemantics(derived().toCode(key), semantic, propagate);
	}

	void insertOrAssignSemantics(Point coord, Semantic semantic, bool propagate = true,
	                             depth_t depth = 0)
	{
		insertOrAssignSemantics(derived().toCode(coord, depth), semantic, propagate);
	}

	void insertOrAssignSemantics(coord_t x, coord_t y, coord_t z, Semantic semantic,
	                             bool propagate = true, depth_t depth = 0)
	{
		insertOrAssignSemantics(derieved().toCode(x, y, z, depth), semantic, propagate);
	}

	//
	// TODO: Inser or assign custom function
	//

	//
	// TODO: Assign
	//

	//
	// TODO: Erase
	//

	//
	// TODO: Erase if
	//

	//
	// TODO: Clear
	//

	//
	// TODO: Contains
	//

	[[nodiscard]] bool containsAllSemantics(...) const
	{
		// TODO: Implement
	}

	[[nodiscard]] bool containsAnySemantics(...) const
	{
		// TODO: Implement
	}

	[[nodiscard]] bool containsNoneSemantics(...) const
	{
		// TODO: Implement
	}

	//
	// TODO: Counts
	//

	//
	// TODO: Size
	//

	[[nodiscard]] std::size_t semanticsSize(Node node) const
	{
		return semanticNode(derived().leafNode(node)).size(node.index());
	}

	[[nodiscard]] std::size_t semanticsSize(Code code) const
	{
		auto [node, depth] = derived().leafNodeAndDepth(code);
		return semanticNode(node).size(code.index(depth));
	}

	[[nodiscard]] std::size_t semanticsSize(Key key) const
	{
		return semanticsSize(derived().toCode(key));
	}

	[[nodiscard]] std::size_t semanticsSize(Point coord, depth_t depth = 0) const
	{
		return semanticsSize(derived().toCode(coord, depth));
	}

	[[nodiscard]] std::size_t semanticsSize(coord_t x, coord_t y, coord_t z,
	                                        depth_t depth = 0) const
	{
		return semanticsSize(derived().toCode(x, y, z, depth));
	}

	//
	// Empty
	//

	[[nodiscard]] bool semanticsEmpty(Node node) const
	{
		return semanticNode(derived().leafNode(node)).empty(node.index());
	}

	[[nodiscard]] bool semanticsEmpty(Code code) const
	{
		auto [node, depth] = derived().leafNodeAndDepth(code);
		return semanticNode(node).empty(code.index(depth));
	}

	[[nodiscard]] bool semanticsEmpty(Key key) const
	{
		return semanticsEmpty(derived().toCode(key));
	}

	[[nodiscard]] bool semanticsEmpty(Point coord, depth_t depth = 0) const
	{
		return semanticsEmpty(derived().toCode(coord, depth));
	}

	[[nodiscard]] bool semanticsEmpty(coord_t x, coord_t y, coord_t z,
	                                  depth_t depth = 0) const
	{
		return semanticsEmpty(derived().toCode(x, y, z, depth));
	}

	//
	// TODO: Iterators
	//

	//
	// TODO: Find, lower/upper bound
	//

	//
	// Change semantic label
	//

	void changeLabel(label_t old_label, label_t new_label, bool propagate = true)
	{
		changeLabel(derived().getRootCode(), old_label, new_label, propagate);
	}

	void changeLabel(Node node, label_t old_label, label_t new_label, bool propagate = true)
	{
		derived().apply(
		    node,
		    [old_label, new_label](auto& node, index_t index) {
			    node.changeLabel(index, old_label, new_label);
		    },
		    [old_label, new_label](auto& node) { node.changeLabel(old_label, new_label); },
		    propagate);
	}

	void changeLabel(Code code, label_t old_label, label_t new_label, bool propagate = true)
	{
		derived().apply(
		    code,
		    [old_label, new_label](auto& node, index_t index) {
			    node.changeLabel(index, old_label, new_label);
		    },
		    [old_label, new_label](auto& node) { node.changeLabel(old_label, new_label); },
		    propagate);
	}

	void changeLabel(Key key, label_t old_label, label_t new_label, bool propagate = true)
	{
		changeLabel(derived().toCode(key), old_label, new_label, propagate);
	}

	void changeLabel(Point coord, label_t old_label, label_t new_label,
	                 bool propagate = true, depth_t depth = 0)
	{
		changeLabel(derived().toCode(coord, depth), old_label, new_label, propagate);
	}

	void changeLabel(coord_t x, coord_t y, coord_t z, label_t old_label, label_t new_label,
	                 bool propagate = true, depth_t depth = 0)
	{
		changeLabel(derived().toCode(x, y, z, depth), old_label, new_label, propagate);
	}

	//
	// Delete semantic label
	//

	void deleteLabel(label_t label, bool propagate = true)
	{
		deleteLabel(derived().getRootCode(), label, propagate);
	}

	void deleteLabel(Node node, label_t label, bool propagate = true)
	{
		derived().apply(
		    node, [label](auto& node, index_t index) { node.deleteLabel(index, label); },
		    [label](auto& node) { node.deleteLabel(label); }, propagate);
	}

	void deleteLabel(Code code, label_t label, bool propagate = true)
	{
		derived().apply(
		    code, [label](auto& node, index_t index) { node.deleteLabel(index, label); },
		    [label](auto& node) { node.deleteLabel(label); }, propagate);
	}

	void deleteLabel(Key key, label_t label, bool propagate = true)
	{
		deleteLabel(derived().toCode(key), label, propagate);
	}

	void deleteLabel(Point coord, label_t label, bool propagate = true, depth_t depth = 0)
	{
		deleteLabel(derived().toCode(coord, depth), label, propagate);
	}

	void deleteLabel(coord_t x, coord_t y, coord_t z, label_t label, bool propagate = true,
	                 depth_t depth = 0)
	{
		deleteLabel(derived().toCode(x, y, z, depth), label, propagate);
	}

	//
	// Clear semantics
	//

	void clearSemantics(bool propagate = true)
	{
		clearSemantics(derived().getRootCode(), propagate);
	}

	void clearSemantics(Node node, bool propagate = true)
	{
		derived().apply(
		    node, [](auto& node, index_t index) { semanticNode(node).clear(index); },
		    [](auto& node) { semanticNode(node).clear(); }, propagate);
	}

	void clearSemantics(Code code, bool propagate = true)
	{
		derived().apply(
		    code, [](auto& node, index_t index) { semanticNode(node).clear(index); },
		    [](auto& node) { semanticNode(node).clear(); }, propagate);
	}

	void clearSemantics(Key key, bool propagate = true)
	{
		clearSemantics(derived().toCode(key), propagate);
	}

	void clearSemantics(Point coord, bool propagate = true, depth_t depth = 0)
	{
		clearSemantics(derived().toCode(coord, depth), propagate);
	}

	void clearSemantics(coord_t x, coord_t y, coord_t z, bool propagate = true,
	                    depth_t depth = 0)
	{
		clearSemantics(derived().toCode(x, y, z, depth), propagate);
	}

	//
	// Erase semantics if
	//

	template <class Pred>
	void eraseSemanticsIf(Pred pred, bool propagate = true)
	{
		eraseSemanticIf(derived().getRootCode(), pred, propagate);
	}

	template <class Pred>
	void eraseSemanticsIf(Node node, Pred pred, bool propagate = true)
	{
		derived().apply(
		    node, [pred](auto& node, index_t index) { node.eraseSemanticsIf(index, pred); },
		    [pred](auto& node) { node.eraseSemanticsIf(pred); }, propagate);
	}

	template <class Pred>
	void eraseSemanticsIf(Code code, Pred pred, bool propagate = true)
	{
		derived().apply(
		    code, [pred](auto& node, index_t index) { node.eraseSemanticsIf(index, pred); },
		    [pred](auto& node) { node.eraseSemanticsIf(pred); }, propagate);
	}

	template <class Pred>
	void eraseSemanticsIf(Key key, Pred pred, bool propagate = true)
	{
		eraseSemanticsIf(derived().toCode(key), pred, propagate);
	}

	template <class Pred>
	void eraseSemanticsIf(Point coord, Pred pred, bool propagate = true, depth_t depth = 0)
	{
		eraseSemanticsIf(derived().toCode(coord, depth), pred, propagate);
	}

	template <class Pred>
	void eraseSemanticsIf(coord_t x, coord_t y, coord_t z, Pred pred, bool propagate = true,
	                      depth_t depth = 0)
	{
		eraseSemanticsIf(derived().toCode(x, y, z, depth), pred, propagate);
	}

	//
	// Get label mapping
	//

	[[nodiscard]] SemanticRange getLabelMapping(std::string const& name) const
	{
		// TODO: Look at
		std::set<std::string> names;
		std::queue<std::string> queue;
		queue.push(name);
		while (!queue.empty()) {
			std::string cur = queue.front();
			queue.pop();
			for (auto const& str : label_mapping_.at(cur).strings) {
				if (names.insert(str).second) {
					queue.push(str);
				}
			}
		}
		container::RangeSet<label_t> labels;
		for (auto const& str : names) {
			labels.insert(std::cbegin(label_mapping_.ranges), std::cend(label_mapping_.ranges));
		}
		return labels;
	}

	//
	// Add label mapping
	//

	void addLabelMapping(std::string const& name, label_t label)
	{
		// TODO: Implement
	}

	void addLabelMapping(std::string const& name, SemanticRange label_range)
	{
		// TODO: Implement
	}

	void addLabelMapping(std::string const& name, std::string const& name_2)
	{
		// TODO: Implement
	}

	template <class InputIt>
	void addLabelMapping(std::string const& name, InputIt first, InputIt last)
	{
		while (first != last) {
			addLabelMapping(name, *first);
			std::advance(first, 1);
		}
	}

	//
	// Remove label mapping
	//

	void removeLabelMapping(std::string const& name)
	{
		// TODO: Implement
	}

	// void removeLabelMapping(std::string const& name)
	// {
	// 	auto it = label_mapping_.find(name);
	// 	if (string_label_mapping_.end() != it) {
	// 		for (auto const& l : it->second) {
	// 			label_string_mapping_.erase(l);
	// 		}
	// 		string_label_mapping_.erase(it);
	// 	}
	// }

	// void removeLabelMapping(label_t  label)
	// {
	// 	auto it = label_string_mapping_.find(label);
	// 	if (label_string_mapping_.end() != it) {
	// 		string_label_mapping_[it->second].erase(label);
	// 		label_string_mapping_.erase(it);
	// 	}
	// }

	//
	// Clear label mapping
	//

	void clearLabelMapping() { label_mapping_.clear(); }

	void clearLabelMapping(std::string const& name)
	{
		// TODO: Implement
	}

 protected:
	//
	// Constructors
	//

	SemanticMap() = default;

	SemanticMap(SemanticMap const& other) = default;

	SemanticMap(SemanticMap&& other) = default;

	template <class Derived2>
	SemanticMap(SemanticMap<Derived2> const& other) :  // TODO: Fill in
	{
	}

	//
	// Assignment operator
	//

	SemanticMap& operator=(SemanticMap const& rhs) = default;

	SemanticMap& operator=(SemanticMap&& rhs) = default;

	template <class Derived2>
	SemanticMap& operator=(SemanticMap<Derived2> const& rhs)
	{
		// TODO: Fill in
		return *this;
	}

	//
	// Swap
	//

	void swap(SemanticMap& other) noexcept
	{
		std::swap(label_mapping_, other.label_mapping_);
		std::swap(label_propagation_, other.label_propagation_);
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
	// Semantic node
	//

	template <class T>
	[[nodiscard]] constexpr auto& semanticNode(T& node)
	{
		return static_cast<SemanticNode<T::semanticSize()>&>(node);
	}

	template <class T>
	[[nodiscard]] constexpr auto const& semanticNode(T const& node) const
	{
		return static_cast<SemanticNode<T::semanticSize()> const&>(node);
	}

	//
	// Initilize root
	//

	void initRoot() { semanticNode(derived().root()).clear(); }

	//
	// Update node
	//

	template <std::size_t N, class T>
	void updateNode(SemanticNode<N>& node, IndexField const indices, T const& children)
	{
		if constexpr (1 == N) {
			// TODO: Implement
		} else if (indices.all()) {
			// TODO: Implement
		} else {
			// TODO: Implement
		}
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr DataIdentifier dataIdentifier() noexcept
	{
		return DataIdentifier::SEMANTIC;
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
		return node_type::semanticSize();
	}

	template <class OutputIt>
	void readNodes(std::istream& in, OutputIt first, std::size_t num_nodes)
	{
		mapping_.read(in);


		uint8_t n;
		in.read(reinterpret_cast<char*>(&n), sizeof(n));

		for (std::size_t i = 0; i != num_nodes; ++first, ++i) {
			first->node.read(in, first->indices, propagation_, n);
		}
	}

	template <class InputIt>
	void writeNodes(std::ostream& out, InputIt first, std::size_t num_nodes) const
	{
		mapping_.write(out);

		constexpr uint8_t const n = numData<InputIt>();
		out.write(reinterpret_cast<char const*>(&n), sizeof(n));

		for (std::size_t i = 0; i != num_nodes; ++first, ++i) {
			first->node.write(out);
		}
	}

 protected:
	// Label mapping
	SemanticMapping mapping_;

	// Propagation
	SemanticPropagation propagation_;
};
}  // namespace ufo::map

#endif  // UFO_MAP_SEMANTIC_MAP_H