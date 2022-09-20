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
#include <ufo/container/flat_map.h>
#include <ufo/container/range_map.h>
#include <ufo/map/semantic/semantic.h>
#include <ufo/map/semantic/semantic_node.h>
#include <ufo/map/types.h>

// STL
#include <algorithm>
#include <functional>
#include <set>
#include <unordered_map>

namespace ufo::map
{
template <class Derived, class LeafNode>
class SemanticMap
{
 public:
	using semantic_container_type = typename LeafNode::semantic_container_type;
	using semantic_label_type = typename LeafNode::semantic_label_type;
	using semantic_value_type = typename LeafNode::semantic_value_type;

	//
	// Semantics min/max
	//

	[[nodiscard]] static constexpr semantic_label_type getSemanticLabelMin() noexcept
	{
		return semantic_container_type::getSemanticLabelMin();
	}

	[[nodiscard]] static constexpr semantic_label_type getSemanticLabelMax() noexcept
	{
		return semantic_container_type::getSemanticLabelMax();
	}

	[[nodiscard]] static constexpr semantic_value_type getSemanticValueMin() noexcept
	{
		return semantic_container_type::getSemanticValueMin();
	}

	[[nodiscard]] static constexpr semantic_value_type getSemanticValueMax() noexcept
	{
		return semantic_container_type::getSemanticValueMin();
	}

	//
	// Get semantics
	//

	[[nodiscard]] semantic_container_type const& getSemantics(Node node) const
	{
		return getSemantics(derived().getLeafNode(node));
	}

	[[nodiscard]] semantic_container_type const& getSemantics(Code code) const
	{
		return getSemantics(derived().getLeafNode(code));
	}

	[[nodiscard]] semantic_container_type const& getSemantics(Key key) const
	{
		return getSemantics(derived().toCode(key));
	}

	[[nodiscard]] semantic_container_type const& getSemantics(Point coord,
	                                                          depth_t depth = 0) const
	{
		return getSemantics(derived().toCode(coord, depth));
	}

	[[nodiscard]] semantic_container_type const& getSemantics(coord_t x, coord_t y,
	                                                          coord_t z,
	                                                          depth_t depth = 0) const
	{
		return getSemantics(derived().toCode(x, y, z, depth));
	}

	//
	// Set semantics
	//

	void setSemantics(Node node, semantic_container_type const& semantics,
	                  bool propagate = true)
	{
		derived().apply(
		    node, [this, &semantics](LeafNode& node) { setSemantics(node, semantics); },
		    propagate);
	}

	void setSemantics(Code code, semantic_container_type const& semantics,
	                  bool propagate = true)
	{
		derived().apply(
		    code, [this, &semantics](LeafNode& node) { setSemantics(node, semantics); },
		    propagate);
	}

	void setSemantics(Key key, semantic_container_type const& semantics,
	                  bool propagate = true)
	{
		setSemantics(derived().toCode(key), semantics, propagate);
	}

	void setSemantics(Point coord, semantic_container_type const& semantics,
	                  bool propagate = true, depth_t depth = 0)
	{
		setSemantics(derived().toCode(coord, depth), semantics, propagate);
	}

	void setSemantics(coord_t x, coord_t y, coord_t z,
	                  semantic_container_type const& semantics, bool propagate = true,
	                  depth_t depth = 0)
	{
		setSemantics(derived().toCode(x, y, z, depth), semantics, propagate);
	}

	//
	// Insert semantics
	//

	void insertSemantics(Node node, semantic_label_type label, semantic_value_type value,
	                     bool propagate = true)
	{
		derived().apply(
		    node,
		    [this, label, value](LeafNode& node) { insertSemantics(node, label, value); },
		    propagate);
	}

	void insertSemantics(Code code, semantic_label_type label, semantic_value_type value,
	                     bool propagate = true)
	{
		derived().apply(
		    code,
		    [this, label, value](LeafNode& node) { insertSemantics(node, label, value); },
		    propagate);
	}

	void insertSemantics(Key key, semantic_label_type label, semantic_value_type value,
	                     bool propagate = true)
	{
		insertSemantics(derived().toCode(key), label, value, propagate);
	}

	void insertSemantics(Point coord, semantic_label_type label, semantic_value_type value,
	                     bool propagate = true, depth_t depth = 0)
	{
		insertSemantics(derived().toCode(coord, depth), label, value, propagate);
	}

	void insertSemantics(coord_t x, coord_t y, coord_t z, semantic_label_type label,
	                     semantic_value_type value, bool propagate = true,
	                     depth_t depth = 0)
	{
		insertSemantics(derived().toCode(x, y, z, depth), label, value, propagate);
	}

	void insertSemantics(Node node, SemanticPair semantic, bool propagate = true)
	{
		insertSemantics(node, semantic.label, semantic.value, propagate);
	}

	void insertSemantics(Code code, SemanticPair semantic, bool propagate = true)
	{
		insertSemantics(code, semantic.label, semantic.value, propagate);
	}

	void insertSemantics(Key key, SemanticPair semantic, bool propagate = true)
	{
		insertSemantics(derived().toCode(key), semantic, propagate);
	}

	void insertSemantics(Point coord, SemanticPair semantic, bool propagate = true,
	                     depth_t depth = 0)
	{
		insertSemantics(derived().toCode(coord, depth), semantic, propagate);
	}

	void insertSemantics(coord_t x, coord_t y, coord_t z, SemanticPair semantic,
	                     bool propagate = true, depth_t depth = 0)
	{
		insertSemantics(derived().toCode(x, y, z, depth), semantic, propagate);
	}

	template <class InputIt>
	void insertSemantics(Node node, InputIt first, InputIt last, bool propagate = true)
	{
		derived().apply(
		    node, [this, first, last](LeafNode& node) { insertSemantics(node, first, last); },
		    propagate);
	}

	template <class InputIt>
	void insertSemantics(Code code, InputIt first, InputIt last, bool propagate = true)
	{
		derived().apply(
		    code, [this, first, last](LeafNode& node) { insertSemantics(node, first, last); },
		    propagate);
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

	void insertSemantics(Node node, std::initializer_list<SemanticPair> ilist,
	                     bool propagate = true)
	{
		derived().apply(
		    node, [this, ilist](LeafNode& node) { insertSemantics(node, ilist); }, propagate)
	}

	void insertSemantics(Code code, std::initializer_list<SemanticPair> ilist,
	                     bool propagate = true)
	{
		derived().apply(
		    code, [this, ilist](LeafNode& node) { insertSemantics(node, ilist); }, propagate);
	}

	void insertSemantics(Key key, std::initializer_list<SemanticPair> ilist,
	                     bool propagate = true)
	{
		insertSemantics(derived().toCode(key), ilist, propagate);
	}

	void insertSemantics(Point coord, std::initializer_list<SemanticPair> ilist,
	                     bool propagate = true, depth_t depth = 0)
	{
		insertSemantics(derived().toCode(coord, depth), ilist, propagate);
	}

	void insertSemantics(coord_t x, coord_t y, coord_t z,
	                     std::initializer_list<SemanticPair> ilist, bool propagate = true,
	                     depth_t depth = 0)
	{
		insertSemantics(derived().toCode(x, y, z, depth), ilist, propagate);
	}

	//
	// Insert or assign semantics
	//

	void insertOrAssignSemantics(Node node, semantic_label_type label,
	                             semantic_value_type value, bool propagate = true)
	{
		derived().apply(
		    node,
		    [this, label, value](LeafNode& node) {
			    insertOrAssignSemantics(node, label, value);
		    },
		    propagate);
	}

	void insertOrAssignSemantics(Code code, semantic_label_type label,
	                             semantic_value_type value, bool propagate = true)
	{
		derived().apply(
		    code,
		    [this, label, value](LeafNode& node) {
			    insertOrAssignSemantics(node, label, value);
		    },
		    propagate);
	}

	void insertOrAssignSemantics(Key key, semantic_label_type label,
	                             semantic_value_type value, bool propagate = true)
	{
		insertOrAssignSemantics(derived().toCode(key), label, value, propagate);
	}

	void insertOrAssignSemantics(Point coord, semantic_label_type label,
	                             semantic_value_type value, bool propagate = true,
	                             depth_t depth = 0)
	{
		insertOrAssignSemantics(derived().toCode(coord, depth), label, value, propagate);
	}

	void insertOrAssignSemantics(coord_t x, coord_t y, coord_t z, semantic_label_type label,
	                             semantic_value_type value, bool propagate = true,
	                             depth_t depth = 0)
	{
		insertOrAssignSemantics(derived().toCode(x, y, z, depth), label, value, propagate);
	}

	void insertOrAssignSemantics(Node node, SemanticPair semantic, bool propagate = true)
	{
		insertOrAssignSemantics(node, semantic.label, semantic.value, propagate);
	}

	void insertOrAssignSemantics(Code code, SemanticPair semantic, bool propagate = true)
	{
		insertOrAssignSemantics(code, semantic.label, semantic.value, propagate);
	}

	void insertOrAssignSemantics(Key key, SemanticPair semantic, bool propagate = true)
	{
		insertOrAssignSemantics(derived().toCode(key), semantic, propagate);
	}

	void insertOrAssignSemantics(Point coord, SemanticPair semantic, bool propagate = true,
	                             depth_t depth = 0)
	{
		insertOrAssignSemantics(derived().toCode(coord, depth), semantic, propagate);
	}

	void insertOrAssignSemantics(coord_t x, coord_t y, coord_t z, SemanticPair semantic,
	                             bool propagate = true, depth_t depth = 0)
	{
		insertOrAssignSemantics(derieved().toCode(x, y, z, depth), semantic, propagate);
	}

	//
	// Increase semantics
	//

	void increaseSemantics(Node node, semantic_value_type inc, bool propagate = true)
	{
		derived().apply(
		    node, [this, inc](LeafNode& node) { increaseSemantics(node, inc); }, propagate);
	}

	void increaseSemantics(Code code, semantic_value_type inc, bool propagate = true)
	{
		derived().apply(
		    code, [this, inc](LeafNode& node) { increaseSemantics(node, inc); }, propagate);
	}

	void increaseSemantics(Key key, semantic_value_type inc, bool propagate = true)
	{
		increaseSemantics(derived().toCode(key), inc, propagate);
	}

	void increaseSemantics(Point coord, semantic_value_type inc, bool propagate = true,
	                       depth_t depth = 0)
	{
		increaseSemantics(derived().toCode(coord, depth), inc, propagate);
	}

	void increaseSemantics(coord_t x, coord_t y, coord_t z, semantic_value_type inc,
	                       bool propagate = true, depth_t depth = 0)
	{
		increaseSemantics(derived().toCode(x, y, z, depth), inc, propagate);
	}

	void increaseSemantics(Node node, semantic_label_type label, semantic_value_type inc,
	                       semantic_value_type init_value = getSemanticValueMin(),
	                       bool propagate = true)
	{
		derived().apply(
		    node,
		    [this, label, inc, init_value](LeafNode& node) {
			    increaseSemantics(node, label inc, init_value);
		    },
		    propagate);
	}

	void increaseSemantics(Code code, semantic_label_type label, semantic_value_type inc,
	                       semantic_value_type init_value = getSemanticValueMin(),
	                       bool propagate = true)
	{
		derived().apply(
		    code,
		    [this, label, inc, init_value](LeafNode& node) {
			    increaseSemantics(node, label inc, init_value);
		    },
		    propagate);
	}

	void increaseSemantics(Key key, semantic_label_type label, semantic_value_type inc,
	                       semantic_value_type init_value = getSemanticValueMin(),
	                       bool propagate = true)
	{
		increaseSemantics(derived().toCode(key), label, inc, init_value, propagate);
	}

	void increaseSemantics(Point coord, semantic_label_type label, semantic_value_type inc,
	                       semantic_value_type init_value = getSemanticValueMin(),
	                       bool propagate = true, depth_t depth = 0)
	{
		increaseSemantics(derived().toCode(coord, depth), label, inc, init_value, propagate);
	}

	void increaseSemantics(coord_t x, coord_t y, coord_t z, semantic_label_type label,
	                       semantic_value_type inc,
	                       semantic_value_type init_value = getSemanticValueMin(),
	                       bool propagate = true, depth_t depth = 0)
	{
		increaseSemantics(derived().toCode(x, y, z, depth), label, inc, init_value,
		                  propagate);
	}

	//
	// Decrease semantics
	//

	void decreaseSemantics(Node node, semantic_value_type dec, bool propagate = true)
	{
		derived().apply(
		    node, [this, dec](LeafNode& node) { decreaseSemantics(node, dec); }, propagate);
	}

	void decreaseSemantics(Code code, semantic_value_type dec, bool propagate = true)
	{
		derived().apply(
		    code, [this, dec](LeafNode& node) { decreaseSemantics(node, dec); }, propagate);
	}

	void decreaseSemantics(Key key, semantic_value_type dec, bool propagate = true)
	{
		decreaseSemantics(derived().toCode(key), dec, propagate);
	}

	void decreaseSemantics(Point coord, semantic_value_type dec, bool propagate = true,
	                       depth_t depth = 0)
	{
		decreaseSemantics(derived().toCode(coord, depth), dec, propagate);
	}

	void decreaseSemantics(coord_t x, coord_t y, coord_t z, semantic_value_type dec,
	                       bool propagate = true, depth_t depth = 0)
	{
		decreaseSemantics(derived().toCode(x, y, z, depth), dec, propagate);
	}

	void decreaseSemantics(Node node, semantic_label_type label, semantic_value_type dec,
	                       semantic_value_type init_value = getSemanticValueMin(),
	                       bool propagate = true)
	{
		derived().apply(
		    node,
		    [this, label, dec, init_value](LeafNode& node) {
			    decreaseSemantics(node, label dec, init_value);
		    },
		    propagate);
	}

	void decreaseSemantics(Code code, semantic_label_type label, semantic_value_type dec,
	                       semantic_value_type init_value = getSemanticValueMin(),
	                       bool propagate = true)
	{
		derived().apply(
		    code,
		    [this, label, dec, init_value](LeafNode& node) {
			    decreaseSemantics(node, label dec, init_value);
		    },
		    propagate);
	}

	void decreaseSemantics(Key key, semantic_label_type label, semantic_value_type dec,
	                       semantic_value_type init_value = getSemanticValueMin(),
	                       bool propagate = true)
	{
		decreaseSemantics(derived().toCode(key), label, dec, init_value, propagate);
	}

	void decreaseSemantics(Point coord, semantic_label_type label, semantic_value_type dec,
	                       semantic_value_type init_value = getSemanticValueMin(),
	                       bool propagate = true, depth_t depth = 0)
	{
		decreaseSemantics(derived().toCode(coord, depth), label, dec, init_value, propagate);
	}

	void decreaseSemantics(coord_t x, coord_t y, coord_t z, semantic_label_type label,
	                       semantic_value_type dec,
	                       semantic_value_type init_value = getSemanticValueMin(),
	                       bool propagate = true, depth_t depth = 0)
	{
		decreaseSemantics(derived().toCode(x, y, z, depth), label, dec, init_value,
		                  propagate);
	}

	//
	// Change semantic label
	//

	void changeSemanticLabel(semantic_label_type old_label, semantic_label_type new_label,
	                         bool propagate = true)
	{
		changeSemanticLabel(derived().getRootCode(), old_label, new_label, propagate);
	}

	void changeSemanticLabel(Node node, semantic_label_type old_label,
	                         semantic_label_type new_label, bool propagate = true)
	{
		derived().apply(
		    node,
		    [this, old_label, new_label](LeafNode& node) {
			    changeSemanticLabel(node, old_label, new_label);
		    },
		    propagate);
	}

	void changeSemanticLabel(Code code, semantic_label_type old_label,
	                         semantic_label_type new_label, bool propagate = true)
	{
		derived().apply(
		    code,
		    [this, old_label, new_label](LeafNode& node) {
			    changeSemanticLabel(node, old_label, new_label);
		    },
		    propagate);
	}

	void changeSemanticLabel(Key key, semantic_label_type old_label,
	                         semantic_label_type new_label, bool propagate = true)
	{
		changeSemanticLabel(derived().toCode(key), old_label, new_label, propagate);
	}

	void changeSemanticLabel(Point coord, semantic_label_type old_label,
	                         semantic_label_type new_label, bool propagate = true,
	                         depth_t depth = 0)
	{
		changeSemanticLabel(derived().toCode(coord, depth), old_label, new_label, propagate);
	}

	void changeLabel(coord_t x, coord_t y, coord_t z, semantic_label_type old_label,
	                 semantic_label_type new_label, bool propagate = true,
	                 depth_t depth = 0)
	{
		changeSemanticLabel(derived().toCode(x, y, z, depth), old_label, new_label,
		                    propagate);
	}

	//
	// Delete semantic label
	//

	void deleteSemanticLabel(semantic_label_type label, bool propagate = true)
	{
		deleteSemanticLabel(derived().getRootCode(), label, propagate);
	}

	void deleteSemanticLabel(Node node, semantic_label_type label, bool propagate = true)
	{
		derived().apply(
		    node, [this, label](LeafNode& node) { deleteSemanticLabel(node, label); },
		    propagate);
	}

	void deleteSemanticLabel(Code code, semantic_label_type label, bool propagate = true)
	{
		derived().apply(
		    code, [this, label](LeafNode& node) { deleteSemanticLabel(node, label); },
		    propagate);
	}

	void deleteSemanticLabel(Key key, semantic_label_type label, bool propagate = true)
	{
		deleteSemanticLabel(derived().toCode(key), label, propagate);
	}

	void deleteSemanticLabel(Point coord, semantic_label_type label, bool propagate = true,
	                         depth_t depth = 0)
	{
		deleteSemanticLabel(derived().toCode(coord, depth), label, propagate);
	}

	void deleteSemanticLabel(coord_t x, coord_t y, coord_t z, semantic_label_type label,
	                         bool propagate = true, depth_t depth = 0)
	{
		deleteSemanticLabel(derived().toCode(x, y, z, depth), label, propagate);
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
		    node, [this](LeafNode& node) { clearSemantics(node); }, propagate);
	}

	void clearSemantics(Code code, bool propagate = true)
	{
		derived().apply(
		    code, [this](LeafNode& node) { clearSemantics(node); }, propagate);
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
		    node, [this, pred](LeafNode& node) { eraseSemanticsIf(node, pred); }, propagate);
	}

	template <class Pred>
	void eraseSemanticsIf(Code code, Pred pred, bool propagate = true)
	{
		derived().apply(
		    code, [this, pred](LeafNode& node) { eraseSemanticsIf(node, pred); }, propagate);
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

	[[nodiscard]] semantic_label_range_t getLabelMapping(std::string const& name) const
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
		container::RangeSet<semantic_label_t> labels;
		for (auto const& str : names) {
			labels.insert(std::cbegin(label_mapping_.ranges), std::cend(label_mapping_.ranges));
		}
		return labels;
	}

	//
	// Add label mapping
	//

	void addLabelMapping(std::string const& name, semantic_label_type label)
	{
		// TODO: Implement
	}

	void addLabelMapping(std::string const& name, semantic_label_range_t label_range)
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

	// void removeLabelMapping(semantic_label_type  label)
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

	void clearLabelMaping() { label_mapping_.clear(); }

	void clearLabelMapping(std::string const& name)
	{
		// TODO: Implement
	}

 protected:
	//
	// Derived
	//

	constexpr Derived& derived() { return *static_cast<Derived*>(this); }

	constexpr Derived const& derived() const { return *static_cast<Derived const*>(this); }

	//
	// Initilize root
	//

	void initRoot() { clearSemantics(derived().getRoot()); }

	//
	// Get semantics
	//

	static constexpr semantic_container_type& getSemantics(LeafNode& node) noexcept
	{
		return node.semantics;
	}

	static constexpr semantic_container_type const& getSemantics(
	    LeafNode const& node) noexcept
	{
		return node.semantics;
	}

	//
	// Get semantic value
	//

	static std::optional<semantic_value_type> getSemanticValue(LeafNode const& node,
	                                                           semantic_label_type label)
	{
		return getSemantics(node).getValue(label);
	}

	//
	// Insert semantics
	//

	static void insertSemantics(LeafNode& node, semantic_label_type label,
	                            semantic_value_type value)
	{
		getSemantics(node).emplace(label, value);
	}

	template <class InputIt>
	static void insertSemantics(LeafNode& node, InputIt first, InputIt last)
	{
		getSemantics(node).insert(first, last);
	}

	static void insertSemantics(LeafNode& node, std::initializer_list<SemanticPair> ilist)
	{
		getSemantics(node).insert(ilist);
	}

	//
	// Insert or assign semantics
	//

	static void insertOrAssignSemantics(LeafNode& node, semantic_label_type label,
	                                    semantic_value_type value)
	{
		getSemantics(node).insert_or_assign(label, value);
	}

	//
	// Increase semantics
	//

	static void increaseSemantics(LeafNode& node, semantic_value_type inc)
	{
		for (auto&& elem : getSemantics(node)) {
			increaseSemantics(node, elem.getLabel(), inc, getSemanticValueMin());
		}
	}

	static void increaseSemantics(LeafNode& node, semantic_label_type label,
	                              semantic_value_type inc, semantic_value_type init_value)
	{
		getSemantics(node).increase_value(label, inc, init_value);
	}

	//
	// Decrease semantics
	//

	static void decreaseSemantics(LeafNode& node, semantic_value_type dec)
	{
		for (auto&& elem : getSemantics(node)) {
			decreaseSemantics(node, elem.getLabel(), dec, getSemanticValueMin());
		}
	}

	static void decreaseSemantics(LeafNode& node, semantic_label_type label,
	                              semantic_value_type dec, semantic_value_type init_value)
	{
		getSemantics(node).decrease_value(label, dec, init_value);
	}

	//
	// Change semantic label
	//

	static void changeSemanticLabel(LeafNode& node, semantic_label_type old_label,
	                                semantic_label_type new_label)
	{
		// TODO: Implement
	}

	//
	// Delete label
	//

	static void deleteSemanticLabel(LeafNode& node, semantic_label_type label)
	{
		getSemantics(node).erase(label);
	}

	//
	// Clear semantics
	//

	static void clearSemantics(LeafNode& node) { getSemantics(node).clear(); }

	//
	// Erase semantics if
	//

	template <class Pred>
	static void eraseSemanticsIf(LeafNode& node, Pred pred)
	{
		erase_if(getSemantics(node), pred);
	}

	//
	// Update node
	//

	void updateNode(LeafNode& node) {}

	template <class T>
	void updateNode(LeafNode& node, T const& children)
	{
		// Get size
		std::array<std::size_t, 8> sizes;
		std::size_t index = 0;
		for (auto const& child : children) {
			sizes[index++] = getSemantics(child).size();
		}

		// Create buffer
		std::vector<SemanticPair> data(std::reduce(std::cbegin(sizes), std::cend(sizes)));

		// Fill in data
		auto it = std::begin(data);
		for (auto const& child : children) {
			it = std::copy(std::begin(getSemantics(child)), std::end(getSemantics(child)), it);
		}

		// Sort data
		auto beg = std::begin(data);
		auto mid = std::next(beg, sizes[0]);
		for (std::size_t i = 1; sizes.size() != i; ++i) {
			auto end = std::next(mid, sizes[i]);
			std::inplace_merge(beg, mid, end, [](auto a, auto b) { return a.label < b.label; });
			mid = end;
		}

		// Handle duplicate labels
		auto new_last = removeDuplicateLabels(std::begin(data), std::end(data));

		// Copy over data
		getSemantics(node).resize(std::distance(std::begin(data), new_last));
		std::copy(std::cbegin(data), new_last, getSemantics(node).data());
	}

	template <class ForwardIt>
	ForwardIt removeDuplicateLabels(ForwardIt first, ForwardIt last)
	{
		if (first == last) {
			return last;
		}

		ForwardIt result = first;
		while (first != last) {
			auto const label = first->label;

			auto it = std::find_if(first, last, [label](auto v) { return label != v.label; });

			if (1 < std::distance(first, it)) {
				PropagationCriteria prop = label_propagation_.getPropCriteria(label);
				first->value = getValue(prop, first, it);
			}

			if (result != first) {
				*result = std::move(*first);
			}

			first = it;
			std::advance(result, 1);
		}

		return result;
	}

	template <class InputIt>
	semantic_value_t getValue(PropagationCriteria prop, InputIt first, InputIt last) const
	{
		switch (prop) {
			case PropagationCriteria::MIN:
				getMinValue(first, last);
				break;
			case PropagationCriteria::MAX:
				getMaxValue(first, last);
				break;
			case PropagationCriteria::MEAN:
				getMeanValue(first, last);
				break;
		}
	}

	template <class InputIt>
	semantic_value_t getMinValue(InputIt first, InputIt last) const
	{
		semantic_value_t min = first->value;
		std::advance(first, 1);
		while (first != last) {
			min = std::min(min, first->value);
			std::advance(first, 1);
		}
		return min;
	}

	template <class InputIt>
	semantic_value_t getMaxValue(InputIt first, InputIt last) const
	{
		semantic_value_t max = first->value;
		std::advance(first, 1);
		while (first != last) {
			max = std::max(max, first->value);
			std::advance(first, 1);
		}
		return max;
	}

	template <class InputIt>
	semantic_value_t getMeanValue(InputIt first, InputIt last) const
	{
		// FIXME: Make sure no overflow and how to round?
		semantic_value_t total = first->value;
		std::size_t num = 1;
		std::advance(first, 1);
		while (first != last) {
			total += first->value;
			++num;
			std::advance(first, 1);
		}
		return total / num;
	}

	//
	// Input/output (read/write)
	//

	static constexpr DataIdentifier dataIdentifier() noexcept
	{
		return DataIdentifier::SEMANTIC;
	}

	static constexpr bool canReadData(DataIdentifier identifier) noexcept
	{
		return dataIdentifier() == identifier;
	}

	void readNodes(std::istream& in_stream, std::vector<LeafNode*> const& nodes)
	{
		label_mapping_.read(in_stream);

		auto const num_nodes = nodes.size();

		// TODO: Read semantics
	}

	void writeNodes(std::ostream& out_stream, std::vector<LeafNode> const& nodes) const
	{
		label_mapping_.write(out_stream);

		auto const num_nodes = nodes.size();

		DataType const label_type = dataType<SemanticLabelType>();
		DataType const value_type = dataType<SeamnticValueType>();

		uint8_t const label_value = enumToValue(label_type);
		uint8_t const value_value = enumToValue(value_type);

		out_stream.write(reinterpret_cast<char const*>(&label_value), sizeof(label_value));
		out_stream.write(reinterpret_cast<char const*>(&value_value), sizeof(value_value));

		auto sizes = std::make_unique<SemanticLabelType>(num_nodes);
		uint64_t num_label_value_pairs = 0;
		for (std::size_t i = 0; num_nodes != i; ++i) {
			auto const size = getSemantics(nodes[i]).size();
			sizes[i] = size;
			num_label_value_pairs += size;
		}

		out_stream.write(reinterpret_cast<char const*>(&num_label_value_pairs),
		                 sizeof(num_label_value_pairs));
		out_stream.write(reinterpret_cast<char const*>(sizes.get()),
		                 num_nodes * sizeof(SemanticLabelType));

		auto labels = std::make_unique<SemanticLabelType>(num_label_value_pairs);
		auto values = std::make_unique<SemanticValueType>(num_label_value_pairs);

		std::size_t i = 0;
		for (auto const& node : nodes) {
			for (auto const& lv : getSemantics(node)) {
				labels[i] = lv.getLabel();
				values[i] = lv.getValue();
				++i;
			}
		}

		out_stream.write(reinterpret_cast<char const*>(&labels.get()),
		                 num_label_value_pairs * sizeof(SemanticLabelType));
		out_stream.write(reinterpret_cast<char const*>(&values.get()),
		                 num_label_value_pairs * sizeof(SemanticValueType));
	}

 protected:
	// Label mapping
	SemanticLabelMapping label_mapping_;

	// Propagation
	SemanticLabelPropagation label_propagation_;
};
}  // namespace ufo::map

#endif  // UFO_MAP_SEMANTIC_MAP_H