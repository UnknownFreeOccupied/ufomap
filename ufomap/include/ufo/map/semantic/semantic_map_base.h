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

#ifndef UFO_MAP_SEMANTIC_MAP_BASE_H
#define UFO_MAP_SEMANTIC_MAP_BASE_H

// UFO
#include <ufo/container/flat_map.h>
#include <ufo/container/range_map.h>
#include <ufo/map/semantic/semantic_node.h>
#include <ufo/map/types.h>

// STL
#include <algorithm>
#include <execution>
#include <set>
#include <unordered_map>

namespace ufo::map
{
template <class Derived, class LeafNode, class InnerNode>
class SemanticMapBase
{
 public:
	using semantic_container_type = typename LeafNode::semantic_container_type;
	using semantic_type = typename LeafNode::semantic_type;
	using semantic_label_type = typename LeafNode::semantic_label_type;
	using semantic_value_type = typename LeafNode::semantic_value_type;

	//
	// Semantics min/max
	//

	[[nodiscard]] static constexpr semantic_label_type getSemanticLabelMin() noexcept
	{
		return LeafNode::getSemanticLabelMin();
	}

	[[nodiscard]] static constexpr semantic_label_type getSemanticLabelMax() noexcept
	{
		return LeafNode::getSemanticLabelMax();
	}

	[[nodiscard]] static constexpr semantic_value_type getSemanticValueMin() noexcept
	{
		return LeafNode::getSemanticValueMin();
	}

	[[nodiscard]] static constexpr semantic_value_type getSemanticValueMax() noexcept
	{
		return LeafNode::getSemanticValueMin();
	}

	//
	// Get bits
	//

	[[nodiscard]] static constexpr std::size_t getSemanticLabelBits() noexcept
	{
		return LeafNode::getSemanticLabelBits();
	}

	[[nodiscard]] static constexpr std::size_t getSemanticValueBits() noexcept
	{
		return LeafNode::getSemanticValueBits();
	}

	//
	// Is fixed size
	//

	[[nodiscard]] static constexpr bool isSemanticFixedSize() noexcept
	{
		return LeafNode::isSemanticFixedSize();
	}

	//
	// Get semantics
	//

	static constexpr semantic_container_type const& getSemantics(Node node) noexcept
	{
		return getSemantics(derived().getLeafNode(node));
	}

	semantic_container_type const& getSemantics(Code code) const noexcept
	{
		return getSemantics(derived().getLeafNode(code));
	}

	semantic_container_type const& getSemantics(Key key) const noexcept
	{
		return getSemantics(derived().toCode(key));
	}

	semantic_container_type const& getSemantics(Point3 coord,
	                                            depth_t depth = 0) const noexcept
	{
		return getSemantics(derived().toCode(coord, depth));
	}

	semantic_container_type const& getSemantics(coord_t x, coord_t y, coord_t z,
	                                            depth_t depth = 0) const noexcept
	{
		return getSemantics(derived().toCode(x, y, z, depth));
	}

	//
	// Get semantic value
	//

	static std::optional<semantic_value_type> getSemanticValue(Node node,
	                                                           semantic_label_type label)
	{
		return getSemanticValue(derived().getLeafNode(node), label);
	}

	std::optional<semantic_value_type> getSemanticValue(Code code,
	                                                    semantic_label_type label) const
	{
		return getSemanticValue(derived().getLeafNode(code), label);
	}

	std::optional<semantic_value_type> getSemanticValue(Key key,
	                                                    semantic_label_type label) const
	{
		return getSemanticValue(derived().toCode(key), label);
	}

	std::optional<semantic_value_type> getSemanticValue(Point3 coord,
	                                                    semantic_label_type label,
	                                                    depth_t depth = 0) const
	{
		return getSemanticValue(derived().toCode(coord, depth), label);
	}

	std::optional<semantic_value_type> getSemanticValue(coord_t x, coord_t y, coord_t z,
	                                                    semantic_label_type label,
	                                                    depth_t depth = 0) const
	{
		return getSemanticValue(derived().toCode(x, y, z, depth), label);
	}

	//
	// Get min/max semantic value
	//

	SemanticPair getMinSemanticValue(Node node) const noexcept
	{
		return getMinSemanticValue(derived().getLeafNode(node));
	}

	SemanticPair getMinSemanticValue(Code code) const noexcept
	{
		return getMinSemanticValue(derived().getLeafNode(code));
	}

	SemanticPair getMinSemanticValue(Key key) const noexcept
	{
		return getMinSemanticValue(derived().toCode(key));
	}

	SemanticPair getMinSemanticValue(Point3 coord, depth_t depth = 0) const noexcept
	{
		return getMinSemanticValue(derived().toCode(coord, depth));
	}

	SemanticPair getMinSemanticValue(coord_t x, coord_t y, coord_t z,
	                                 depth_t depth = 0) const noexcept
	{
		return getMinSemanticValue(derived().toCode(x, y, z, depth));
	}

	SemanticPair getMaxSemanticValue(Node node) const noexcept
	{
		return getMaxSemanticValue(derived().getLeafNode(node));
	}

	SemanticPair getMaxSemanticValue(Code code) const noexcept
	{
		return getMaxSemanticValue(derived().getLeafNode(code));
	}

	SemanticPair getMaxSemanticValue(Key key) const noexcept
	{
		return getMaxSemanticValue(derived().toCode(key));
	}

	SemanticPair getMaxSemanticValue(Point3 coord, depth_t depth = 0) const noexcept
	{
		return getMaxSemanticValue(derived().toCode(coord, depth));
	}

	SemanticPair getMaxSemanticValue(coord_t x, coord_t y, coord_t z,
	                                 depth_t depth = 0) const noexcept
	{
		return getMaxSemanticValue(derived().toCode(x, y, z, depth));
	}

	//
	// Insert semantics
	//

	void insertSemantics(Node node, semantic_label_t label, semantic_value_t value,
	                     bool propagate = true)
	{
		derived().apply(
		    node,
		    [this, label, value](LeafNode& node) { insertSemantics(node, label, value); },
		    propagate);
	}

	void insertSemantics(Node node, SemanticPair semantic, bool propagate = true)
	{
		insertSemantics(node, semantic.label, semantic.value, propagate);
	}

	template <class InputIt>
	void insertSemantics(Node node, InputIt first, InputIt last, bool propagate = true)
	{
		derived().apply(
		    node, [this, first, last](LeafNode& node) { insertSemantics(node, first, last); },
		    propagate);
	}

	void insertSemantics(Node node, std::initializer_list<SemanticPair> ilist,
	                     bool propagate = true)
	{
		derived().apply(
		    node, [this, ilist](LeafNode& node) { insertSemantics(node, ilist); }, propagate)
	}

	void insertSemantics(Code code, semantic_label_t label, semantic_value_t value,
	                     bool propagate = true)
	{
		derived().apply(
		    code,
		    [this, label, value](LeafNode& node) { insertSemantics(node, label, value); },
		    propagate);
	}

	void insertSemantics(Code code, SemanticPair semantic, bool propagate = true)
	{
		insertSemantics(code, semantic.label, semantic.value, propagate);
	}

	template <class InputIt>
	void insertSemantics(Code code, InputIt first, InputIt last, bool propagate = true)
	{
		derived().apply(
		    code, [this, first, last](LeafNode& node) { insertSemantics(node, first, last); },
		    propagate);
	}

	void insertSemantics(Code code, std::initializer_list<SemanticPair> ilist,
	                     bool propagate = true)
	{
		derived().apply(
		    code, [this, ilist](LeafNode& node) { insertSemantics(node, ilist); }, propagate);
	}

	void insertSemantics(Key key, semantic_label_t label, semantic_value_t value,
	                     bool propagate = true)
	{
		insertSemantics(derived().toCode(key), label, value, propagate);
	}

	void insertSemantics(Key key, SemanticPair semantic, bool propagate = true)
	{
		insertSemantics(derived().toCode(key), semantic, propagate);
	}

	template <class InputIt>
	void insertSemantics(Key key, InputIt first, InputIt last, bool propagate = true)
	{
		insertSemantics(derived().toCode(key), first, last, propagate);
	}

	void insertSemantics(Key key, std::initializer_list<SemanticPair> ilist,
	                     bool propagate = true)
	{
		insertSemantics(derived().toCode(key), ilist, propagate);
	}

	void insertSemantics(Point3 coord, semantic_label_t label, semantic_value_t value,
	                     depth_t depth = 0, bool propagate = true)
	{
		insertSemantics(derived().toCode(coord, depth), label, value, propagate);
	}

	void insertSemantics(Point3 coord, SemanticPair semantic, depth_t depth = 0,
	                     bool propagate = true)
	{
		insertSemantics(derived().toCode(coord, depth), semantic, propagate);
	}

	template <class InputIt>
	void insertSemantics(Point3 coord, InputIt first, InputIt last, depth_t depth = 0,
	                     bool propagate = true)
	{
		insertSemantics(derived().toCode(coord, depth), first, last, propagate);
	}

	void insertSemantics(Point3 coord, std::initializer_list<SemanticPair> ilist,
	                     depth_t depth = 0, bool propagate = true)
	{
		insertSemantics(derived().toCode(coord, depth), ilist, propagate);
	}

	void insertSemantics(coord_t x, coord_t y, coord_t z, semantic_label_t label,
	                     semantic_value_t value, depth_t depth = 0, bool propagate = true)
	{
		insertSemantics(derived().toCode(x, y, z, depth), label, value, propagate);
	}

	void insertSemantics(coord_t x, coord_t y, coord_t z, SemanticPair semantic,
	                     depth_t depth = 0, bool propagate = true)
	{
		insertSemantics(derived().toCode(x, y, z, depth), semantic, propagate);
	}

	template <class InputIt>
	void insertSemantics(coord_t x, coord_t y, coord_t z, InputIt first, InputIt last,
	                     depth_t depth = 0, bool propagate = true)
	{
		insertSemantics(derived().toCode(x, y, z, depth), first, last, propagate);
	}

	void insertSemantics(coord_t x, coord_t y, coord_t z,
	                     std::initializer_list<SemanticPair> ilist, depth_t depth = 0,
	                     bool propagate = true)
	{
		insertSemantics(derived().toCode(x, y, z, depth), ilist, propagate);
	}

	//
	// Insert or assign semantics
	//

	void insertOrAssignSemantics(Node node, semantic_label_t label, semantic_value_t value,
	                             bool propagate = true)
	{
		derived().apply(
		    node,
		    [this, label, value](LeafNode& node) {
			    insertOrAssignSemantics(node, label, value);
		    },
		    propagate);
	}

	void insertOrAssignSemantics(Node node, SemanticPair semantic, bool propagate = true)
	{
		insertOrAssignSemantics(node, semantic.label, semantic.value, bool propagate = true);
	}

	void insertOrAssignSemantics(Code code, semantic_label_t label, semantic_value_t value,
	                             bool propagate = true)
	{
		derived().apply(
		    code,
		    [this, label, value](LeafNode& node) {
			    insertOrAssignSemantics(node, label, value);
		    },
		    propagate);
	}

	void insertOrAssignSemantics(Code code, SemanticPair semantic, bool propagate = true)
	{
		insertOrAssignSemantics(code, semantic.label, semantic.value, propagate);
	}

	void insertOrAssignSemantics(Key key, semantic_label_t label, semantic_value_t value,
	                             bool propagate = true)
	{
		insertOrAssignSemantics(derived().toCode(key), label, value, propagate);
	}

	void insertOrAssignSemantics(Key key, SemanticPair semantic, bool propagate = true)
	{
		insertOrAssignSemantics(derived().toCode(key), semantic, propagate);
	}

	void insertOrAssignSemantics(Point3 coord, semantic_label_t label,
	                             semantic_value_t value, depth_t depth = 0,
	                             bool propagate = true)
	{
		insertOrAssignSemantics(derived().toCode(coord, depth), label, value, propagate);
	}

	void insertOrAssignSemantics(Point3 coord, SemanticPair semantic, depth_t depth = 0,
	                             bool propagate = true)
	{
		insertOrAssignSemantics(derived().toCode(coord, depth), semantic, propagate);
	}

	void insertOrAssignSemantics(coord_t x, coord_t y, coord_t z, semantic_label_t label,
	                             semantic_value_t value, depth_t depth = 0,
	                             bool propagate = true)
	{
		insertOrAssignSemantics(derived().toCode(x, y, z, depth), label, value, propagate);
	}

	void insertOrAssignSemantics(coord_t x, coord_t y, coord_t z, SemanticPair semantic,
	                             depth_t depth = 0, bool propagate = true)
	{
		insertOrAssignSemantics(derieved().toCode(x, y, z, depth), semantic, propagate);
	}

	//
	// Increase semantics
	//

	void increaseSemantics(Node node, semantic_value_t inc, bool propagate = true)
	{
		derived().apply(
		    node, [this, inc](LeafNode& node) { increaseSemantics(node, inc); }, propagate);
	}

	void increaseSemantics(Code code, semantic_value_t inc, bool propagate = true)
	{
		derived().apply(
		    code, [this, inc](LeafNode& node) { increaseSemantics(node, inc); }, propagate);
	}

	void increaseSemantics(Key key, semantic_value_t inc, bool propagate = true)
	{
		increaseSemantics(derived().toCode(key), inc, propagate);
	}

	void increaseSemantics(Point3 coord, semantic_value_t inc, depth_t depth = 0,
	                       bool propagate = true)
	{
		increaseSemantics(derived().toCode(coord, depth), inc, propagate);
	}

	void increaseSemantics(coord_t x, coord_t y, coord_t z, semantic_value_t inc,
	                       depth_t depth = 0, bool propagate = true)
	{
		increaseSemantics(derived().toCode(x, y, z, depth), inc, propagate);
	}

	void increaseSemantics(Node node, semantic_label_t label, semantic_value_t inc,
	                       semantic_value_t init_value = getSemanticValueMin(),
	                       bool propagate = true)
	{
		derived().apply(
		    node,
		    [this, label, inc, init_value](LeafNode& node) {
			    increaseSemantics(node, label inc, init_value);
		    },
		    propagate);
	}

	void increaseSemantics(Code code, semantic_label_t label, semantic_value_t inc,
	                       semantic_value_t init_value = 0, bool propagate = true)
	{
		derived().apply(
		    code,
		    [this, label, inc, init_value](LeafNode& node) {
			    increaseSemantics(node, label inc, init_value);
		    },
		    propagate);
	}

	void increaseSemantics(Key key, semantic_label_t label, semantic_value_t inc,
	                       semantic_value_t init_value = 0, bool propagate = true)
	{
		increaseSemantics(derived().toCode(key), label, inc, init_value, propagate);
	}

	void increaseSemantics(Point3 coord, semantic_label_t label, semantic_value_t inc,
	                       semantic_value_t init_value = 0, depth_t depth = 0,
	                       bool propagate = true)
	{
		increaseSemantics(derived().toCode(coord, depth), label, inc, init_value, propagate);
	}

	void increaseSemantics(coord_t x, coord_t y, coord_t z, semantic_label_t label,
	                       semantic_value_t inc, semantic_value_t init_value = 0,
	                       depth_t depth = 0, bool propagate = true)
	{
		increaseSemantics(derived().toCode(x, y, z, depth), label, inc, init_value,
		                  propagate);
	}

	//
	// Decrease semantics
	//

	void decreaseSemantics(Node node, semantic_value_t dec, bool propagate = true)
	{
		derived().apply(
		    node, [this, dec](LeafNode& node) { decreaseSemantics(node, dec); }, propagate);
	}

	void decreaseSemantics(Code code, semantic_value_t dec, bool propagate = true)
	{
		derived().apply(
		    code, [this, dec](LeafNode& node) { decreaseSemantics(node, dec); }, propagate);
	}

	void decreaseSemantics(Key key, semantic_value_t dec, bool propagate = true)
	{
		decreaseSemantics(derived().toCode(key), dec, propagate);
	}

	void decreaseSemantics(Point3 coord, semantic_value_t dec, depth_t depth = 0,
	                       bool propagate = true)
	{
		decreaseSemantics(derived().toCode(coord, depth), dec, propagate);
	}

	void decreaseSemantics(coord_t x, coord_t y, coord_t z, semantic_value_t dec,
	                       depth_t depth = 0, bool propagate = true)
	{
		decreaseSemantics(derived().toCode(x, y, z, depth), dec, propagate);
	}

	void decreaseSemantics(Node node, semantic_label_t label, semantic_value_t dec,
	                       semantic_value_t init_value = getSemanticValueMin(),
	                       bool propagate = true)
	{
		derived().apply(
		    node,
		    [this, label, dec, init_value](LeafNode& node) {
			    decreaseSemantics(node, label dec, init_value);
		    },
		    propagate);
	}

	void decreaseSemantics(Code code, semantic_label_t label, semantic_value_t dec,
	                       semantic_value_t init_value = 0, bool propagate = true)
	{
		derived().apply(
		    code,
		    [this, label, dec, init_value](LeafNode& node) {
			    decreaseSemantics(node, label dec, init_value);
		    },
		    propagate);
	}

	void decreaseSemantics(Key key, semantic_label_t label, semantic_value_t dec,
	                       semantic_value_t init_value = 0, bool propagate = true)
	{
		decreaseSemantics(derived().toCode(key), label, dec, init_value, propagate);
	}

	void decreaseSemantics(Point3 coord, semantic_label_t label, semantic_value_t dec,
	                       semantic_value_t init_value = 0, depth_t depth = 0,
	                       bool propagate = true)
	{
		decreaseSemantics(derived().toCode(coord, depth), label, dec, init_value, propagate);
	}

	void decreaseSemantics(coord_t x, coord_t y, coord_t z, semantic_label_t label,
	                       semantic_value_t dec, semantic_value_t init_value = 0,
	                       depth_t depth = 0, bool propagate = true)
	{
		decreaseSemantics(derived().toCode(x, y, z, depth), label, dec, init_value,
		                  propagate);
	}

	//
	// Change semantic label
	//

	void changeSemanticLabel(semantic_label_t old_label, semantic_label_t new_label,
	                         bool propagate = true)
	{
		changeSemanticLabel(derived().getRootCode(), old_label, new_label, propagate);
	}

	void changeSemanticLabel(Node node, semantic_label_t old_label,
	                         semantic_label_t new_label, bool propagate = true)
	{
		derived().apply(
		    node,
		    [this, old_label, new_label](LeafNode& node) {
			    changeSemanticLabel(node, old_label, new_label);
		    },
		    propagate);
	}

	void changeSemanticLabel(Code code, semantic_label_t old_label,
	                         semantic_label_t new_label, bool propagate = true)
	{
		derived().apply(
		    code,
		    [this, old_label, new_label](LeafNode& node) {
			    changeSemanticLabel(node, old_label, new_label);
		    },
		    propagate);
	}

	void changeSemanticLabel(Key key, semantic_label_t old_label,
	                         semantic_label_t new_label, bool propagate = true)
	{
		changeSemanticLabel(derived().toCode(key), old_label, new_label, propagate);
	}

	void changeSemanticLabel(Point3 coord, semantic_label_t old_label,
	                         semantic_label_t new_label, depth_t depth = 0,
	                         bool propagate = true)
	{
		changeSemanticLabel(derived().toCode(coord, depth), old_label, new_label, propagate);
	}

	void changeLabel(coord_t x, coord_t y, coord_t z, semantic_label_t old_label,
	                 semantic_label_t new_label, depth_t depth = 0, bool propagate = true)
	{
		changeSemanticLabel(derived().toCode(x, y, z, depth), old_label, new_label,
		                    propagate);
	}

	//
	// Delete semantic label
	//

	void deleteSemanticLabel(semantic_label_t label, bool propagate = true)
	{
		deleteSemanticLabel(derived().getRootCode(), label, propagate);
	}

	void deleteSemanticLabel(Node node, semantic_label_t label, bool propagate = true)
	{
		derived().apply(
		    node, [this, label](LeafNode& node) { deleteSemanticLabel(node, label); },
		    propagate);
	}

	void deleteSemanticLabel(Code code, semantic_label_t label, bool propagate = true)
	{
		derived().apply(
		    code, [this, label](LeafNode& node) { deleteSemanticLabel(node, label); },
		    propagate);
	}

	void deleteSemanticLabel(Key key, semantic_label_t label, bool propagate = true)
	{
		deleteSemanticLabel(derived().toCode(key), label, propagate);
	}

	void deleteSemanticLabel(Point3 coord, semantic_label_t label, depth_t depth = 0,
	                         bool propagate = true)
	{
		deleteSemanticLabel(derived().toCode(coord, depth), label, propagate);
	}

	void deleteSemanticLabel(coord_t x, coord_t y, coord_t z, semantic_label_t label,
	                         depth_t depth = 0, bool propagate = true)
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

	void clearSemantics(Point3 coord, depth_t depth = 0, bool propagate = true)
	{
		clearSemantics(derived().toCode(coord, depth), propagate);
	}

	void clearSemantics(coord_t x, coord_t y, coord_t z, depth_t depth = 0,
	                    bool propagate = true)
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
	void eraseSemanticsIf(Point3 coord, Pred pred, depth_t depth = 0, bool propagate = true)
	{
		eraseSemanticsIf(derived().toCode(coord, depth), pred, propagate);
	}

	template <class Pred>
	void eraseSemanticsIf(coord_t x, coord_t y, coord_t z, Pred pred, depth_t depth = 0,
	                      bool propagate = true)
	{
		eraseSemanticsIf(derived().toCode(x, y, z, depth), pred, propagate);
	}

	//
	// Get label mapping
	//

	container::RangeSet<semantic_label_t> getLabelMapping(std::string const& name) const
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

	void addLabelMapping(std::string const& name, semantic_label_t label)
	{
		label_mapping_[name].ranges.insert(label);
	}

	void addLabelMapping(std::string const& name, std::string const& name_2)
	{
		// TODO: Look at
		label_mapping_.strings.insert(name_2);
	}

	//
	// Remove label mapping
	//

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

	// void removeLabelMapping(semantic_label_t  label)
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
	// Get min/max semantic value
	//

	SemanticPair getMinSemanticValue(LeafNode const& node) noexcept
	{
		auto semantics = getSemantics(node);
		auto const it =
		    std::min_element(std::cbegin(semantics), std::cend(semantics),
		                     [](auto const& a, auto const& b) { return a.value < b.value; });
		return SemanticPair(it->label, it->value);
	}

	SemanticPair getMaxSemanticValue(LeafNode const& node) noexcept
	{
		auto semantics = getSemantics(node);
		auto const it =
		    std::max_element(std::cbegin(semantics), std::cend(semantics),
		                     [](auto const& a, auto const& b) { return a.value < b.value; });
		return SemanticPair(it->label, it->value);
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

	static void increaseSemantics(LeafNode& node, semantic_value_t inc)
	{
		for (auto&& elem : getSemantics(node)) {
			increaseSemantics(node, elem.getLabel(), inc, 0);
		}
	}

	static void increaseSemantics(LeafNode& node, semantic_label_type label,
	                              semantic_value_type inc, semantic_value_t init_value)
	{
		getSemantics(node).increase_value(label, inc, init_value);
	}

	//
	// Decrease semantics
	//

	static void decreaseSemantics(LeafNode& node, semantic_value_t dec)
	{
		for (auto&& elem : getSemantics(node)) {
			decreaseSemantics(node, elem.getLabel(), dec, 0);
		}
	}

	static void decreaseSemantics(LeafNode& node, semantic_label_type label,
	                              semantic_value_type dec, semantic_value_t init_value)
	{
		getSemantics(node).decrease_value(label, dec, init_value);
	}

	//
	// Change semantic label
	//

	static void changeSemanticLabel(LeafNode& node, semantic_label_t old_label,
	                                semantic_label_t new_label)
	{
		// TODO: Implement
	}

	//
	// Delete label
	//

	static void deleteSemanticLabel(LeafNode& node, semantic_label_t label)
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
		std::vector<semantic_label_type> to_delete;
		for (auto&& elem : getSemantics(node)) {
			if (pred(elem.getLabel(), elem.getValue())) {
				to_delete.push_back(elem.getLabel());
			}
		}
		for (auto&& label : to_delete) {
			deleteSemanticLabel(node, label);
		}
	}

	//
	// Update node
	//

	void updateNode(InnerNode& node, depth_t const depth)
	{
		// TODO: Look at

		std::array<typename semantics_t::size_type, 8> sizes;
		for (size_t i = 0; i != sizes.size(); ++i) {
			sizes[i] = derived().getChild(node, depth - 1, i).semantics.size();
		}

		size_t total_size = std::reduce(sizes.cbegin(), sizes.cend());

		if (0 == total_size) {
			node.semantics.clear();
			return;
		}

		std::vector<typename semantics_t::Element> semantics(total_size);

		// Get childrens semantics
		auto it_beg = semantics.begin();
		for (size_t i = 0; i != sizes.size(); ++i) {
			if (0 != sizes[i]) {
				auto const& child_sem = derived().getChild(node, depth - 1, i).semantics;
				auto it_end = std::copy(child_sem.cbegin(), child_sem.cend(), it_beg);
				std::inplace_merge(semantics.begin(), it_beg, it_end);
				it_beg = std::move(it_end);
			}
		}

		// Remove duplicates (if duplicate, save highest value)
		semantics.erase(std::unique(semantics.begin(), semantics.end(),
		                            [](auto const& v1, auto const& v2) {
			                            return v1.getKey() == v2.getKey();
		                            }),
		                semantics.end());

		// Insert
		node.semantics.setOrdered(semantics.cbegin(), semantics.cend());
	}

	//
	// Input/output (read/write)
	//

	bool canReadData(DataIdentifier identifier) const noexcept
	{
		return DataIdentifier::SEMANTIC == identifier;
	}

	bool readNodes(std::istream& in_stream, std::vector<LeafNode*> const& nodes,
	               DataIdentifier identifier)
	{
		if ("semantics" != field) {
			return false;
		}

		if ('U' == type && sizeof(semantic_label_t) == size) {
			semantic_label_t bits_for_label;
			in_stream.read(reinterpret_cast<char*>(&bits_for_label), sizeof(bits_for_label));
			for (auto& node : nodes) {
				node->semantics.readData(in_stream);
			}
		} else {
			return false;
		}
		return true;
	}

	void writeNodes(std::ostream& out_stream, std::vector<LeafNode> const& nodes,
	                bool compress, int compression_acceleration_level,
	                int compression_level) const
	{
		std::vector<uint64_t> has_semantics((nodes.size() + 64 - 1) / 64);
		uint64_t semantics_size = 0;
		std::size_t index = 0;
		std::size_t element = 0;
		for (auto const& node : nodes) {
			if (!getSemantics(node).empty()) {
				has_semantics[index] |= 1U << element;
				semantics_size += getSemantics(node).size();
			}
			if (63 == element) {
				++index;
				element = 0;
			} else {
				++element;
			}
		}
		semantics_size *= sizeof(SemanticType);

		constexpr uint8_t value_bits = getSemanticValueBits();

		uint64_t mapping_size = 0;
		for (auto const& mapping : label_mapping_) {
			// TODO: Implement
		}

		uint64_t propagation_size = 0;
		for (auto const& propagation : label_propagation_) {
			// TODO: Implement
		}

		total_size = node_size + mapping_size + propagation_size;
		total_size += sizeof(SemanticValue);  // For number of bits

		out_stream.write(reinterpret_cast<char*>(&total_size), sizeof(total_size));
		out_stream.write(reinterpret_cast<char*>(&total_size), sizeof(total_size));

		// FIXME: Improve

		// FIXME: Write mappings

		// FIXME: Write propagation strategies
		out_stream.write(reinterpret_cast<char*>(&num), sizeof(num));

		semantic_value_t bits_for_value = LeafNode::getSemanticValueBits();
		out_stream.write(reinterpret_cast<char*>(&bits_for_value), sizeof(bits_for_value));

		for (auto const& node : nodes) {
			node->semantics.writeData(out_stream);
		}
	}

 protected:
	// Label mapping
	struct LabelMapping {
		container::RangeSet<semantic_t> ranges;
		std::unordered_set<std::string> strings;
		RGBColor color;
	};

	std::unordered_map<std::string, LabelMapping> label_mapping_;

	// Propagation
	container::RangeMap<semantic_t, PropagationCriteria> label_propagation_;

	// Deserialize
	// TODO: container::RangeMap<semantic_t, DeserializeCriteria> label_deserialize_;
};
}  // namespace ufo::map

#endif  // UFO_MAP_SEMANTIC_MAP_BASE_H