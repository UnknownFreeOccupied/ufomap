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
		return getSemantics(std::as_const(Derived::getLeafNode(node)));
	}

	semantic_container_type const& getSemantics(Code code) const noexcept
	{
		return getSemantics(derived().getLeafNode(code));
	}

	semantic_container_type const& getSemantics(Key key) const noexcept
	{
		return getSemantics(Derived::toCode(key));
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
		return getSemanticValue(Derived::getLeafNode(node), label);
	}

	std::optional<semantic_value_type> getSemanticValue(Code code,
	                                                    semantic_label_type label) const
	{
		return getSemanticValue(derived().getLeafNode(code), label);
	}

	std::optional<semantic_value_type> getSemanticValue(Key key,
	                                                    semantic_label_type label) const
	{
		return getSemanticValue(Derived::toCode(key), label);
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
	// Get max semantic value
	//

	SemanticPair getMaxSemanticValue(Node const& node) const noexcept
	{
		return getMaxSemanticValue(Derived::getLeafNode(node));
	}

	SemanticPair getMaxSemanticValue(Code code) const noexcept
	{
		return getMaxSemanticValue(derived().getLeafNode(code));
	}

	SemanticPair getMaxSemanticValue(Key key) const noexcept
	{
		return getMaxSemanticValue(Derived::toCode(key));
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

	void insertSemantics(semantic_label_t label, semantic_value_t value, Node node,
	                     bool propagate = true)
	{
		derived().apply(
		    node,
		    [this, label, value](LeafNode& node) { insertSemantics(node, label, value); },
		    propagate);
	}

	void insertSemantics(SemanticPair semantic, Node node, bool propagate = true)
	{
		insertSemantics(semantic.label, semantic.value, node, propagate);
	}

	template <class InputIt>
	void insertSemantics(InputIt first, InputIt last, Node node, bool propagate = true)
	{
		derived().apply(node, [this, first, last](LeafNode& node) {
			insertSemantics(node, first, last);
		});
	}

	void insertSemantics(semantic_label_t label, semantic_value_t value, Code code,
	                     bool propagate = true)
	{
		derived().apply(code, [this, label, value](LeafNode& node) {
			insertSemantics(node, label, value);
		});
	}

	void insertSemantics(SemanticPair semantic, Code code, bool propagate = true)
	{
		insertSemantics(semantic.label, semantic.value, code, propagate);
	}

	template <class InputIt>
	void insertSemantics(InputIt first, InputIt last, Code code, bool propagate = true)
	{
		derived().apply(code, [this, first, last](LeafNode& node) {
			insertSemantics(node, first, last);
		});
	}

	void insertSemantics(semantic_label_t label, semantic_value_t value, Key key,
	                     bool propagate = true)
	{
		insertSemantics(label, value, Derived::toCode(key), propagate);
	}

	void insertSemantics(SemanticPair semantic, Key key, bool propagate = true)
	{
		insertSemantics(semantic, Derived::toCode(key), propagate);
	}

	template <class InputIt>
	void insertSemantics(InputIt first, InputIt last, Key key, bool propagate = true)
	{
		insertSemantics(first, last, Derived::toCode(key), propagate);
	}

	void insertSemantics(semantic_label_t label, semantic_value_t value, Point3 coord,
	                     depth_t depth = 0, bool propagate = true)
	{
		insertSemantics(label, value, derived().toCode(coord, depth), propagate);
	}

	void insertSemantics(SemanticPair semantic, Point3 coord, depth_t depth = 0,
	                     bool propagate = true)
	{
		insertSemantics(semantic, derived().toCode(coord, depth), propagate);
	}

	template <class InputIt>
	void insertSemantics(InputIt first, InputIt last, Point3 coord, depth_t depth = 0,
	                     bool propagate = true)
	{
		insertSemantics(first, last, derived().toCode(coord, depth), propagate);
	}

	void insertSemantics(semantic_label_t label, semantic_value_t value, coord_t x,
	                     coord_t y, coord_t z, depth_t depth = 0, bool propagate = true)
	{
		insertSemantics(label, value, derived().toCode(x, y, z, depth), propagate);
	}

	void insertSemantics(SemanticPair semantic, coord_t x, coord_t y, coord_t z,
	                     depth_t depth = 0, bool propagate = true)
	{
		insertSemantics(semantic, derived().toCode(x, y, z, depth), propagate);
	}

	template <class InputIt>
	void insertSemantics(InputIt first, InputIt last, coord_t x, coord_t y, coord_t z,
	                     depth_t depth = 0, bool propagate = true)
	{
		insertSemantics(first, last, derived().toCode(x, y, z, depth), propagate);
	}

	//
	// Insert or assign semantics
	//

	void insertOrAssignSemantics(Node& node, semantic_label_t label, semantic_value_t value,
	                             bool propagate = true)
	{
		insertOrAssignSemantics(std::execution::seq, node, label, value, propagate);
	}

	void insertOrAssignSemantics(Node& node, SemanticPair semantic, bool propagate = true)
	{
		insertOrAssignSemantics(std::execution::seq, node, semantic, propagate);
	}

	template <class InputIt>
	void insertOrAssignSemantics(Node& node, InputIt first, InputIt last,
	                             bool propagate = true)
	{
		insertOrAssignSemantics(std::execution::seq, node, first, last, propagate);
	}

	void insertOrAssignSemantics(Code code, semantic_label_t label, semantic_value_t value,
	                             bool propagate = true)
	{
		insertOrAssignSemantics(std::execution::seq, code, label, value, propagate);
	}

	void insertOrAssignSemantics(Code code, SemanticPair semantic, bool propagate = true)
	{
		insertOrAssignSemantics(std::execution::seq, code, semantic, propagate);
	}

	template <class InputIt>
	void insertOrAssignSemantics(Code code, InputIt first, InputIt last,
	                             bool propagate = true)
	{
		insertOrAssignSemantics(std::execution::seq, code, first, last, propagate);
	}

	void insertOrAssignSemantics(Key key, semantic_label_t label, semantic_value_t value,
	                             bool propagate = true)
	{
		insertOrAssignSemantics(std::execution::seq, key, label, value, propagate);
	}

	void insertOrAssignSemantics(Key key, SemanticPair semantic, bool propagate = true)
	{
		insertOrAssignSemantics(std::execution::seq, key, semantic, propagate);
	}

	template <class InputIt>
	void insertOrAssignSemantics(Key key, InputIt first, InputIt last,
	                             bool propagate = true)
	{
		insertOrAssignSemantics(std::execution::seq, key, first, last, propagate);
	}

	void insertOrAssignSemantics(Point3 coord, semantic_label_t label,
	                             semantic_value_t value, bool propagate = true,
	                             depth_t depth = 0)
	{
		insertOrAssignSemantics(std::execution::seq, coord, label, value, propagate, depth);
	}

	void insertOrAssignSemantics(Point3 coord, SemanticPair semantic, bool propagate = true,
	                             depth_t depth = 0)
	{
		insertOrAssignSemantics(std::execution::seq, coord, semantic, propagate, depth);
	}

	template <class InputIt>
	void insertOrAssignSemantics(Point3 coord, InputIt first, InputIt last,
	                             bool propagate = true, depth_t depth = 0)
	{
		insertOrAssignSemantics(std::execution::seq, coord, first, last, propagate, depth);
	}

	void insertOrAssignSemantics(coord_t x, coord_t y, coord_t z, semantic_label_t label,
	                             semantic_value_t value, bool propagate = true,
	                             depth_t depth = 0)
	{
		insertOrAssignSemantics(std::execution::seq, x, y, z, label, value, propagate, depth);
	}

	void insertOrAssignSemantics(coord_t x, coord_t y, coord_t z, SemanticPair semantic,
	                             bool propagate = true, depth_t depth = 0)
	{
		insertOrAssignSemantics(std::execution::seq, x, y, z, semantic, propagate, depth);
	}

	template <class InputIt>
	void insertOrAssignSemantics(coord_t x, coord_t y, coord_t z, InputIt first,
	                             InputIt last, bool propagate = true, depth_t depth = 0)
	{
		insertOrAssignSemantics(std::execution::seq, x, y, z, first, last, propagate, depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void insertOrAssignSemantics(ExecutionPolicy policy, Node& node, semantic_label_t label,
	                             semantic_value_t value, bool propagate = true)
	{
		insertOrAssignSemantics(policy, node, SemanticPair(label, value), propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void insertOrAssignSemantics(ExecutionPolicy policy, Node& node, SemanticPair semantic,
	                             bool propagate = true)
	{
		derived().apply(
		    policy, node,
		    [this, semantic](LeafNode& node) { insertOrAssignSemanticsImpl(node, semantic); },
		    propagate);
	}

	template <class ExecutionPolicy, class InputIt,
	          typename = std::enable_if_t<
	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
	void insertOrAssignSemantics(ExecutionPolicy policy, Node& node, InputIt first,
	                             InputIt last, bool propagate = true)
	{
		derived().apply(
		    policy, node,
		    [this, first, last](LeafNode& node) {
			    insertOrAssignSemanticsImpl(node, first, last);
		    },
		    propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void insertOrAssignSemantics(ExecutionPolicy policy, Code code, semantic_label_t label,
	                             semantic_value_t value, bool propagate = true)
	{
		insertOrAssignSemantics(policy, code, SemanticPair(label, value), propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void insertOrAssignSemantics(ExecutionPolicy policy, Code code, SemanticPair semantic,
	                             bool propagate = true)
	{
		derived().apply(
		    policy, code,
		    [this, semantic](LeafNode& node) { insertOrAssignSemanticsImpl(node, semantic); },
		    propagate);
	}

	template <class ExecutionPolicy, class InputIt,
	          typename = std::enable_if_t<
	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
	void insertOrAssignSemantics(ExecutionPolicy policy, Code code, InputIt first,
	                             InputIt last, bool propagate = true)
	{
		derived().apply(
		    policy, code,
		    [this, first, last](LeafNode& node) {
			    insertOrAssignSemanticsImpl(node, first, last);
		    },
		    propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void insertOrAssignSemantics(ExecutionPolicy policy, Key key, semantic_label_t label,
	                             semantic_value_t value, bool propagate = true)
	{
		insertOrAssignSemantics(policy, Derived::toCode(key), label, value, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void insertOrAssignSemantics(ExecutionPolicy policy, Key key, SemanticPair semantic,
	                             bool propagate = true)
	{
		insertOrAssignSemantics(policy, Derived::toCode(key), semantic, propagate);
	}

	template <class ExecutionPolicy, class InputIt,
	          typename = std::enable_if_t<
	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
	void insertOrAssignSemantics(ExecutionPolicy policy, Key key, InputIt first,
	                             InputIt last, bool propagate = true)
	{
		insertOrAssignSemantics(policy, Derived::toCode(key), first, last, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void insertOrAssignSemantics(ExecutionPolicy policy, Point3 coord,
	                             semantic_label_t label, semantic_value_t value,
	                             bool propagate = true, depth_t depth = 0)
	{
		insertOrAssignSemantics(policy, derived().toCode(coord, depth), label, value,
		                        propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void insertOrAssignSemantics(ExecutionPolicy policy, Point3 coord,
	                             SemanticPair semantic, bool propagate = true,
	                             depth_t depth = 0)
	{
		insertOrAssignSemantics(policy, derived().toCode(coord, depth), semantic, propagate);
	}

	template <class ExecutionPolicy, class InputIt,
	          typename = std::enable_if_t<
	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
	void insertOrAssignSemantics(ExecutionPolicy policy, Point3 coord, InputIt first,
	                             InputIt last, bool propagate = true, depth_t depth = 0)
	{
		insertOrAssignSemantics(policy, derived().toCode(coord, depth), first, last,
		                        propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void insertOrAssignSemantics(ExecutionPolicy policy, coord_t x, coord_t y, coord_t z,
	                             semantic_label_t label, semantic_value_t value,
	                             bool propagate = true, depth_t depth = 0)
	{
		insertOrAssignSemantics(policy, derived().toCode(x, y, z, depth), label, value,
		                        propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void insertOrAssignSemantics(ExecutionPolicy policy, coord_t x, coord_t y, coord_t z,
	                             SemanticPair semantic, bool propagate = true,
	                             depth_t depth = 0)
	{
		insertOrAssignSemantics(policy, derived().toCode(x, y, z, depth), semantic,
		                        propagate);
	}

	template <class ExecutionPolicy, class InputIt,
	          typename = std::enable_if_t<
	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
	void insertOrAssignSemantics(ExecutionPolicy policy, coord_t x, coord_t y, coord_t z,
	                             InputIt first, InputIt last, bool propagate = true,
	                             depth_t depth = 0)
	{
		insertOrAssignSemantics(policy, derived().toCode(x, y, z, depth), first, last,
		                        propagate);
	}

	//
	// Increase semantics
	//

	void increaseSemantics(Code code, semantic_value_t inc, bool propagate = true)
	{
		increaseSemantics(std::execution::seq, code, inc, propagate);
	}

	void increaseSemantics(Code code, semantic_label_t label, semantic_value_t inc,
	                       semantic_value_t init_value = 0, bool propagate = true)
	{
		increaseSemantics(std::execution::seq, code, label, inc, init_value, propagate);
	}

	void increaseSemantics(Code code, SemanticPair semantic,
	                       semantic_value_t init_value = 0, bool propagate = true)
	{
		increaseSemantics(std::execution::seq, code, semantic, init_value, propagate);
	}

	template <class InputIt>
	void increaseSemantics(Code code, InputIt first, InputIt last,
	                       semantic_value_t init_value = 0, bool propagate = true)
	{
		increaseSemantics(std::execution::seq, code, first, last, init_value, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void increaseSemantics(ExecutionPolicy policy, Code code, semantic_value_t inc,
	                       bool propagate = true)
	{
		increaseSemantics(policy, code, inc, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void increaseSemantics(ExecutionPolicy policy, Code code, semantic_label_t label,
	                       semantic_value_t inc, semantic_value_t init_value = 0,
	                       bool propagate = true)
	{
		increaseSemantics(policy, code, SemanticPair(label, inc), init_value, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void increaseSemantics(ExecutionPolicy policy, Code code, SemanticPair semantic,
	                       semantic_value_t init_value = 0, bool propagate = true)
	{
		derived().apply(
		    policy, code,
		    [this, semantic, init_value](LeafNode& node) {
			    increaseSemanticsImpl(node, semantic, init_value);
		    },
		    propagate);
	}

	template <class ExecutionPolicy, class InputIt,
	          typename = std::enable_if_t<
	              std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>>>
	void increaseSemantics(ExecutionPolicy policy, Code code, InputIt first, InputIt last,
	                       semantic_value_t init_value = 0, bool propagate = true)
	{
		derived().apply(
		    policy, code,
		    [this, first, last, init_value](LeafNode& node) {
			    increaseSemanticsImpl(node, first, last, init_value);
		    },
		    propagate);
	}

	//
	// Decrease semantics
	//

	void decreaseSemantics(Code code, semantic_value_t dec, bool propagate = true)
	{
		// TODO: Implement
	}

	void decreaseSemantics(Code code, semantic_label_t label, semantic_value_t dec,
	                       bool propagate = true)
	{
		decreaseSemantics(code, label, dec, 0, propagate);
	}

	void decreaseSemantics(Code code, semantic_label_t label, semantic_value_t dec,
	                       semantic_value_t init_value, bool propagate = true)
	{
		// TODO: Implement
	}

	//
	// Change label
	//

	void changeLabel(semantic_label_t old_label, semantic_label_t new_label,
	                 bool propagate = true)
	{
		changeLabel(std::execution::seq, old_label, new_label, propagate);
	}

	void changeLabel(Node& node, semantic_label_t old_label, semantic_label_t new_label,
	                 bool propagate = true)
	{
		changeLabel(std::execution::seq, node, old_label, new_label, propagate);
	}

	void changeLabel(Code code, semantic_label_t old_label, semantic_label_t new_label,
	                 bool propagate = true)
	{
		changeLabel(std::execution::seq, code, old_label, new_label, propagate);
	}

	void changeLabel(Key key, semantic_label_t old_label, semantic_label_t new_label,
	                 bool propagate = true)
	{
		changeLabel(std::execution::seq, key, old_label, new_label, propagate);
	}

	void changeLabel(Point3 coord, semantic_label_t old_label, semantic_label_t new_label,
	                 bool propagate = true, depth_t depth = 0)
	{
		changeLabel(std::execution::seq, coord, old_label, new_label, propagate, depth);
	}

	void changeLabel(coord_t x, coord_t y, coord_t z, semantic_label_t old_label,
	                 semantic_label_t new_label, bool propagate = true, depth_t depth = 0)
	{
		changeLabel(std::execution::seq, x, y, z, old_label, new_label, propagate, depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void changeLabel(ExecutionPolicy policy, semantic_label_t old_label,
	                 semantic_label_t new_label, bool propagate = true)
	{
		changeLabel(policy, derived().getRootCode(), old_label, new_label, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void changeLabel(ExecutionPolicy policy, Node& node, semantic_label_t old_label,
	                 semantic_label_t new_label, bool propagate = true)
	{
		// TODO: Implement
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void changeLabel(ExecutionPolicy policy, Code code, semantic_label_t old_label,
	                 semantic_label_t new_label, bool propagate = true)
	{
		// TODO: Implement
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void changeLabel(ExecutionPolicy policy, Key key, semantic_label_t old_label,
	                 semantic_label_t new_label, bool propagate = true)
	{
		changeLabel(policy, Derived::toCode(key), old_label, new_label, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void changeLabel(ExecutionPolicy policy, Point3 coord, semantic_label_t old_label,
	                 semantic_label_t new_label, bool propagate = true, depth_t depth = 0)
	{
		changeLabel(policy, derived().toCode(coord, depth), old_label, new_label, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void changeLabel(ExecutionPolicy policy, coord_t x, coord_t y, coord_t z,
	                 semantic_label_t old_label, semantic_label_t new_label,
	                 bool propagate = true, depth_t depth = 0)
	{
		changeLabel(policy, derived().toCode(x, y, z, depth), old_label, new_label,
		            propagate);
	}

	//
	// Delete label
	//

	void deleteLabel(semantic_label_t label, bool propagate = true)
	{
		deleteLabel(std::execution::seq, label, propagate);
	}

	void deleteLabel(Node& node, semantic_label_t label, bool propagate = true)
	{
		deleteLabel(std::execution::seq, node, label, propagate);
	}

	void deleteLabel(Code code, semantic_label_t label, bool propagate = true)
	{
		deleteLabel(std::execution::seq, code, label, propagate);
	}

	void deleteLabel(Key key, semantic_label_t label, bool propagate = true)
	{
		deleteLabel(std::execution::seq, key, label, propagate);
	}

	void deleteLabel(Point3 coord, semantic_label_t label, bool propagate = true,
	                 depth_t depth = 0)
	{
		deleteLabel(std::execution::seq, coord, label, propagate, depth);
	}

	void deleteLabel(coord_t x, coord_t y, coord_t z, semantic_label_t label,
	                 bool propagate = true, depth_t depth = 0)
	{
		deleteLabel(std::execution::seq, x, y, z, label, propagate, depth);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void deleteLabel(ExecutionPolicy policy, semantic_label_t label, bool propagate = true)
	{
		deleteLabel(policy, derived().getRootCode(), label, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void deleteLabel(ExecutionPolicy policy, Node& node, semantic_label_t label,
	                 bool propagate = true)
	{
		// TODO: Implement
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void deleteLabel(ExecutionPolicy policy, Code code, semantic_label_t label,
	                 bool propagate = true)
	{
		// TODO: Implement
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void deleteLabel(ExecutionPolicy policy, Key key, semantic_label_t label,
	                 bool propagate = true)
	{
		deleteLabel(policy, Derived::toCode(key), label, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void deleteLabel(ExecutionPolicy policy, Point3 coord, semantic_label_t label,
	                 bool propagate = true, depth_t depth = 0)
	{
		deleteLabel(policy, derived().toCode(coord, depth), label, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void deleteLabel(ExecutionPolicy policy, coord_t x, coord_t y, coord_t z,
	                 semantic_label_t label, bool propagate = true, depth_t depth = 0)
	{
		deleteLabel(policy, derived().toCode(x, y, z, depth), label, propagate);
	}

	//
	// Clear semantics
	//

	// TODO: Add node

	void clearSemantics(bool propagate = true)
	{
		clearSemantics(std::execution::seq, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void clearSemantics(ExecutionPolicy policy, bool propagate = true)
	{
		clearSemantics(policy, propagate);
	}

	void clearSemantics(Code code, bool propagate = true)
	{
		clearSemantics(std::execution::seq, code, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void clearSemantics(ExecutionPolicy policy, Code code, bool propagate = true)
	{
		derived().apply(
		    policy, code, [this](LeafNode& node) { clearSemanticsImpl(node); }, propagate);
	}

	void clearSemantics(Key key, bool propagate = true)
	{
		clearSemantics(Derived::toCode(key), propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void clearSemantics(ExecutionPolicy policy, Key key, bool propagate = true)
	{
		clearSemantics(policy, Derived::toCode(key), propagate);
	}

	void clearSemantics(Point3 coord, bool propagate = true, depth_t depth = 0)
	{
		clearSemantics(derived().toCode(coord, depth), propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void clearSemantics(ExecutionPolicy policy, Point3 coord, bool propagate = true,
	                    depth_t depth = 0)
	{
		clearSemantics(policy, derived().toCode(coord, depth), propagate);
	}

	void clearSemantics(coord_t x, coord_t y, coord_t z, bool propagate = true,
	                    depth_t depth = 0)
	{
		clearSemantics(derived().toCode(x, y, z, depth), propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void clearSemantics(ExecutionPolicy policy, coord_t x, coord_t y, coord_t z,
	                    bool propagate = true, depth_t depth = 0)
	{
		clearSemantics(policy, derived().toCode(x, y, z, depth), propagate);
	}

	//
	// Filter semantics
	//

	// // TODO: Add node

	// void filterSemantics(float min_prob, bool propagate = true)
	// {
	// 	filterSemantics(std::execution::seq, min_prob, propagate);
	// }

	// template <class ExecutionPolicy, typename =
	// std::enable_if_t<std::is_execution_policy_v<
	//                                      std::decay_t<ExecutionPolicy>>>>
	// void filterSemantics(ExecutionPolicy policy, float min_prob, bool propagate = true)
	// {
	// 	filterSemantics(policy, derived().getRootCode(), min_prob, propagate);
	// }

	// void filterSemantics(Code code, float min_prob, bool propagate = true)
	// {
	// 	filterSemantics(std::execution::seq, code, min_prob, propagate);
	// }

	// template <class ExecutionPolicy, typename =
	// std::enable_if_t<std::is_execution_policy_v<
	//                                      std::decay_t<ExecutionPolicy>>>>
	// void filterSemantics(ExecutionPolicy policy, Code code, float min_prob,
	//                      bool propagate = true)
	// {
	// 	// TODO: Implement
	// }

	// void filterSemantics(Key key, float min_prob, bool propagate = true)
	// {
	// 	filterSemantics(Derived::toCode(key), min_prob, propagate);
	// }

	// template <class ExecutionPolicy, typename =
	// std::enable_if_t<std::is_execution_policy_v<
	//                                      std::decay_t<ExecutionPolicy>>>>
	// void filterSemantics(ExecutionPolicy policy, Key key, float min_prob,
	//                      bool propagate = true, depth_t depth = 0)
	// {
	// 	filterSemantics(policy, Derived::toCode(key), min_prob, propagate);
	// }

	// void filterSemantics(Point3 coord, float min_prob, bool propagate = true,
	//                      depth_t depth = 0)
	// {
	// 	filterSemantics(derived().toCode(coord, depth), min_prob, propagate);
	// }

	// template <class ExecutionPolicy, typename =
	// std::enable_if_t<std::is_execution_policy_v<
	//                                      std::decay_t<ExecutionPolicy>>>>
	// void filterSemantics(ExecutionPolicy policy, Point3 coord, float min_prob,
	//                      bool propagate = true, depth_t depth = 0)
	// {
	// 	filterSemantics(policy, derived().toCode(coord, depth), min_prob, propagate);
	// }

	// void filterSemantics(coord_t x, coord_t y, coord_t z, float min_prob, bool propagate
	// = true,
	//                      depth_t depth = 0)
	// {
	// 	filterSemantics(derived().toCode(x, y, z, depth), min_prob, propagate);
	// }

	// template <class ExecutionPolicy, typename =
	// std::enable_if_t<std::is_execution_policy_v<
	//                                      std::decay_t<ExecutionPolicy>>>>
	// void filterSemantics(ExecutionPolicy policy, coord_t x, coord_t y, coord_t z, float
	// min_prob,
	//                      bool propagate = true, depth_t depth = 0)
	// {
	// 	filterSemantics(policy, derived().toCode(x, y, z, depth), min_prob, propagate);
	// }

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
	// Clear semantics
	//

	static void clearSemantics(LeafNode& node) { getSemantics(node).clear(); }

	//
	// Get semantics
	//

	static constexpr semantic_container_type& getSemantics(LeafNode& node) noexcept
	{
		return node.semantics;
	}

	static constexpr semantic_container_type getSemantics(LeafNode const& node) noexcept
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

	//
	// Get max semantic value
	//

	SemanticPair getMaxSemanticValue(LeafNode const& node) noexcept
	{
		auto semantics = getSemantics(node);
		auto const it =
		    std::max_element(std::cbegin(semantics), std::cend(semantics),
		                     [](auto const& a, auto const& b) { return a.value < b.value; });
		return SemanticPair(it->label, it->value);
	}

	//
	// Update node
	//

	void updateNode(InnerNode& node, depth_t const depth)
	{
		std::array<typename semantics_t::size_type, 8> sizes;
		for (size_t i = 0; i != sizes.size(); ++i) {
			sizes[i] = Derived::getChild(node, depth - 1, i).semantics.size();
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
				auto const& child_sem = Derived::getChild(node, depth - 1, i).semantics;
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
	// Insert or assign semantics
	//

	void insertOrAssignSemanticsImpl(LeafNode& node, SemanticPair semantic)
	{
		node.semantics.insert_or_assign(semantic.label, semantic.value);
	}

	template <class InputIt>
	void insertOrAssignSemanticsImpl(LeafNode& node, InputIt first, InputIt last)
	{
		node.semantics.insert_or_assign(first, last);
	}

	//
	// Increase semantics
	//

	void increaseSemanticsImpl(LeafNode& node, semantic_value_t inc)
	{
		// FIXME: Improve?
		for (auto semantic : node.semantics) {
			node.semantics.increase_value(semantic.getKey(), inc, 0);
		}
	}

	void increaseSemanticsImpl(LeafNode& node, SemanticPair semantic,
	                           semantic_value_t init_value)
	{
		node.semantics.increase_value(semantic.label, semantic.value, init_value);
	}

	template <class InputIt>
	void increaseSemanticsImpl(LeafNode& node, InputIt first, InputIt last,
	                           semantic_value_t init_value)
	{
		auto it = node.semantics.begin();
		for (; first != last; ++first) {
			// TODO: Correct?
			if constexpr (std::is_same_v<SemanticElement, std::declval<decltype(*first)>>) {
				it = node.semantics.increase_value(it, first->label, first->value, init_value);
			} else {
				it = node.semantics.increase_value(it, first->first, first->second, init_value);
			}
		}
	}

	//
	// Decrease semantics
	//

	void decreaseSemanticsImpl(LeafNode& node, semantic_value_t dec)
	{
		// FIXME: Improve?
		for (auto semantic : node.semantics) {
			node.semantics.decrease_value(semantic.getKey(), dec, 0);
		}
	}

	void decreaseSemanticsImpl(LeafNode& node, SemanticPair semantic,
	                           semantic_value_t init_value)
	{
		node.semantics.decrease_value(semantic.label, semantic.value, init_value);
	}

	template <class InputIt>
	void decreaseSemanticsImpl(LeafNode& node, InputIt first, InputIt last,
	                           semantic_value_t init_value)
	{
		auto it = node.semantics.begin();
		for (; first != last; ++first) {
			// TODO: Correct?
			if constexpr (std::is_same_v<SemanticElement, std::declval<decltype(*first)>>) {
				it = node.semantics.decrease_value(it, first->label, first->value, init_value);
			} else {
				it = node.semantics.decrease_value(it, first->first, first->second, init_value);
			}
		}
	}

	//
	// Change label
	//

	void changeLabelImpl(LeafNode& node, semantic_label_t old_label,
	                     semantic_label_t new_label)
	{
		// TODO: Implement
	}

	//
	// Delete label
	//

	void deleteSemanticLabelImpl(LeafNode& node, semantic_label_t label)
	{
		node.semantics.erase(label);
	}

	//
	// Clear semantics
	//

	void clearSemanticsImpl(LeafNode& node) { node.semantics.clear(); }

	//
	// Filter semantics
	//

	// FIXME: Implement
	//  void filterSemanticsLogitImpl(LeafNode& node, semantic_value_t min_value)
	// {
	// 	node.semantics.filter(min_value);
	// }

	//
	// Input/output (read/write)
	//

	void addFileInfo(FileInfo& info) const
	{
		info["fields"].emplace_back("semantics");
		info["type"].emplace_back("U");
		// TODO: Fix
		info["size"].emplace_back(std::to_string(sizeof(semantic_label_t)));
	}

	bool readNodes(std::istream& in_stream, std::vector<LeafNode*> const& nodes,
	               std::string const& field, char type, uint64_t size, uint64_t num)
	{
		if ("semantics" != field) {
			return false;
		}

		if ('U' == type && sizeof(semantic_label_t) == size) {
			semantic_label_t bits_for_label;
			in_stream.read(reinterpret_cast<char*>(&bits_for_label), sizeof(semantic_label_t));
			for (auto& node : nodes) {
				node->semantics.readData(in_stream);
			}
		} else {
			return false;
		}
		return true;
	}

	virtual void writeNodes(std::ostream& out_stream, std::vector<LeafNode> const& nodes,
	                        bool compress, int compression_acceleration_level,
	                        int compression_level) const override
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

		out_stream.write(reinterpret_cast<char*>(&total_size), sizeof(uint64_t));
		out_stream.write(reinterpret_cast<char*>(&total_size), sizeof(uint64_t));

		// FIXME: Improve

		// FIXME: Write mappings

		// FIXME: Write propagation strategies
		out_stream.write(reinterpret_cast<char*>(&num), sizeof(uint64_t));

		semantic_value_t bits_for_value = LeafNode::getSemanticValueBits();
		out_stream.write(reinterpret_cast<char*>(&bits_for_value), sizeof(SemanticValue));

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