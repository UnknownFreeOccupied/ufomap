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

#ifndef UFO_MAP_OCTREE_BASE_H
#define UFO_MAP_OCTREE_BASE_H

// UFO
#include <ufo/algorithm/algorithm.h>
#include <ufo/map/code.h>
#include <ufo/map/index.h>
#include <ufo/map/index_field.h>
#include <ufo/map/io.h>
#include <ufo/map/key.h>
#include <ufo/map/node.h>
#include <ufo/map/octree/octree_iterator.h>
#include <ufo/map/octree/octree_node.h>
#include <ufo/map/octree/octree_predicate.h>
#include <ufo/map/point.h>
#include <ufo/map/predicate/predicates.h>
#include <ufo/map/predicate/spatial.h>
#include <ufo/map/types.h>
#include <ufo/math/util.h>
#include <ufo/util/iterator_wrapper.h>
#include <ufo/util/type_traits.h>

// STL
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <deque>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace ufo::map
{
// Utilizing curiously recurring template pattern (CRTP)
template <class Derived>
class OctreeBase
{
 public:
	//
	// Tags
	//

	using const_iterator = IteratorWrapper<Derived, Node>;
	using const_query_iterator = const_iterator;
	using const_bounding_volume_iterator = IteratorWrapper<Derived, NodeBV>;
	using const_bounding_volume_query_iterator = const_bounding_volume_iterator;
	using const_query_nearest_iterator = IteratorWrapper<Derived, NearestNode>;

	using Query = util::IteratorWrapper<const_query_iterator>;
	using QueryBV = util::IteratorWrapper<const_bounding_volume_query_iterator>;
	using QueryNearest = util::IteratorWrapper<const_query_nearest_iterator>;

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                       Octree                                        |
	|                                                                                     |
	**************************************************************************************/

	//
	// Clear
	//

	/*!
	 * @brief Erases the map. After this call, the map contains only the root node.
	 */
	void clear(bool prune = true) { clear(size(), depthLevels(), prune); }

	/*!
	 * @brief Erases the map and changes the leaf node size and the number of depth levels.
	 * After this call, the map contains only the root node.
	 *
	 * @param leaf_size The new leaf node size.
	 * @param depth_levels The new number of depth levels.
	 */
	void clear(node_size_t leaf_size, depth_t depth_levels, bool prune = true)
	{
		indices_.clear();
		free_indices_.clear();
		codes_.clear();
		modified_.clear();
		derived().clear();
		if (prune) {
			indices_.shrink_to_fit();
			// FIXME: free_indices_.shrink_to_fit();
			codes_.shrink_to_fit();
			modified_.shrink_to_fit();
			derived().shrinkToFit();
		}
		setNodeSizeAndDepthLevels(leaf_size, depth_levels);
		derived().initRoot();
	}

	//
	// Depth levels
	//

	/*!
	 * @brief The number of depth levels the octree currently have.
	 *
	 * @return The number of depth levels the octree currently have.
	 */
	[[nodiscard]] constexpr depth_t depthLevels() const noexcept { return depth_levels_; }

	/*!
	 * @brief The minimum depth levels an octree can have.
	 *
	 * @return The minimum depth levels an octree can have.
	 */
	[[nodiscard]] static constexpr depth_t minDepthLevels() noexcept
	{
		// TODO: Change to correct
		return 3;
	}

	/*!
	 * @brief The maximum depth levels an octree can have.
	 *
	 * @return The maximum depth levels an octree can have.
	 */
	[[nodiscard]] static constexpr depth_t maxDepthLevels() noexcept
	{
		// TODO: Change to correct
		return 22;
	}

	//
	// Size
	//

	/*!
	 * @brief Get the node size at a specific depth.
	 *
	 * @param depth The depth.
	 * @return The node size at the depth.
	 */
	[[nodiscard]] constexpr node_size_t size(depth_t depth = 0) const
	{
		return node_size_[depth];
	}

	//
	// Volume
	//

	/*!
	 * @brief The volume of the octree.
	 *
	 * @note Same as `size(rootDepth()) * size(rootDepth()) * size(rootDepth())`.
	 *
	 * @return The volume of the octree.
	 */
	[[nodiscard]] constexpr node_size_t volume() const
	{
		auto const s = size(rootDepth());
		return s * s * s;
	}

	//
	// Center
	//

	/*!
	 * @return The center of the octree.
	 */
	[[nodiscard]] Point center() const { return Point(0, 0, 0); }

	//
	// Bounding volume
	//

	/*!
	 * @return Minimum bounding volume convering the whole octree.
	 */
	[[nodiscard]] geometry::AAEBB boundingVolume() const
	{
		return geometry::AAEBB(center(), size(rootDepth() - 1));
	}

	//
	// Within
	//

	/*!
	 * @brief Check if a coordinate is within the octree bounds.
	 *
	 * @param coord The coordinate.
	 * @return Whether the coordinate is within the octree bounds.
	 */
	[[nodiscard]] constexpr bool isWithin(Point coord) const
	{
		return isWithin(coord.x, coord.y, coord.z);
	}

	/*!
	 * @brief Check if a coordinate is within the octree bounds.
	 *
	 * @param x,y,z The coordinate.
	 * @return Whether the coordinate is within the octree bounds.
	 */
	[[nodiscard]] constexpr bool isWithin(coord_t x, coord_t y, coord_t z) const
	{
		auto const max = size(rootDepth() - 1);
		auto const min = -max;
		return min <= x && min <= y && min <= z && max >= x && max >= y && max >= z;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Leaf                                         |
	|                                                                                     |
	**************************************************************************************/

	//
	// Pure leaf
	//

	/*!
	 * @brief Check if a node is a pure leaf node (i.e., can never have children).
	 *
	 * @param node The node to check.
	 * @return Whether the node is a pure leaf node.
	 */
	[[nodiscard]] static constexpr bool isPureLeaf(Node node) noexcept
	{
		return 0 == node.depth();
	}

	/*!
	 * @brief Check if a node corresponding to a code is a pure leaf node (i.e., can never
	 * have children).
	 *
	 * @note Only have to check if the depth of the code is 0.
	 *
	 * @param code The code of the node to check.
	 * @return Whether the node is a pure leaf node.
	 */
	[[nodiscard]] static constexpr bool isPureLeaf(Code code) noexcept
	{
		return 0 == code.depth();
	}

	/*!
	 * @brief Check if a node corresponding to a key is a pure leaf node (i.e., can never
	 * have children).
	 *
	 * @note Only have to check if the depth of the key is 0.
	 *
	 * @param key The key of the node to check.
	 * @return Whether the node is a pure leaf node.
	 */
	[[nodiscard]] static constexpr bool isPureLeaf(Key key) noexcept
	{
		return 0 == key.depth();
	}

	/*!
	 * @brief Check if a node corresponding to a coordinate at a specified depth is a
	 * pure leaf node (i.e., can never have children).
	 *
	 * @note Only have to check if the depth is 0.
	 *
	 * @param coord The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a pure leaf node.
	 */
	[[nodiscard]] static constexpr bool isPureLeaf(Point coord, depth_t depth = 0) noexcept
	{
		return 0 == depth;
	}

	/*!
	 * @brief Check if a node corresponding to a coordinate at a specified depth is a
	 * pure leaf node (i.e., can never have children).
	 *
	 * @note Only have to check if the depth is 0.
	 *
	 * @param x,y,z The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a pure leaf node.
	 */
	[[nodiscard]] static constexpr bool isPureLeaf(coord_t x, coord_t y, coord_t z,
	                                               depth_t depth = 0) noexcept
	{
		return 0 == depth;
	}

	//
	// Leaf
	//

	/*!
	 * @brief Check if a node is a leaf node (i.e., has no children).
	 *
	 * @param node The node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] constexpr bool isLeaf(Node node) const { return isLeaf(index(node)); }

	/*!
	 * @brief Check if a node corresponding to a code is a leaf node (i.e., has no
	 * children).
	 *
	 * @param code The code of the node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] bool isLeaf(Code code) const { return isLeaf(index(code)); }

	/*!
	 * @brief Check if a node corresponding to a key is a leaf node (i.e., has no children).
	 *
	 * @param key The key of the node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] bool isLeaf(Key key) const { return isLeaf(toCode(key)); }

	/*!
	 * @brief Check if a node corresponding to a coordinate at a specified depth is a leaf
	 * node (i.e., has no children).
	 *
	 * @param coord The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] bool isLeaf(Point coord, depth_t depth = 0) const
	{
		return isLeaf(toCode(coord, depth));
	}

	/*!
	 * @brief Check if a node corresponding to a coordinate at a specified depth is a leaf
	 * node (i.e., has no children).
	 *
	 * @param x,y,z The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] bool isLeaf(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return isLeaf(toCode(x, y, z, depth));
	}

	//
	// Parent
	//

	/*!
	 * @brief Check if a node is a parent (i.e., has children).
	 *
	 * @param node The node to check.
	 * @return Whether the node is a parent.
	 */
	[[nodiscard]] constexpr bool isParent(Node node) const { return !isLeaf(node); }

	/*!
	 * @brief Check if a node corresponding to a code is a parent (i.e., has
	 * children).
	 *
	 * @param code The code of the node to check.
	 * @return Whether the node is a parent.
	 */
	[[nodiscard]] constexpr bool isParent(Code code) const { return !isLeaf(code); }

	/*!
	 * @brief Check if a node corresponding to a key is a parent (i.e., has
	 * children).
	 *
	 * @param key The key of the node to check.
	 * @return Whether the node is a parent.
	 */
	[[nodiscard]] constexpr bool isParent(Key key) const { return !isLeaf(key); }

	/*!
	 * @brief Check if a node corresponding to a coordinate at a specified depth is a parent
	 * (i.e., has children).
	 *
	 * @param coord The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a parent.
	 */
	[[nodiscard]] constexpr bool isParent(Point coord, depth_t depth = 0) const
	{
		return !isLeaf(coord, depth);
	}

	/*!
	 * @brief Check if a node corresponding to a coordinate at a specified depth is a parent
	 * (i.e., has children).
	 *
	 * @param x,y,z The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a parent.
	 */
	[[nodiscard]] constexpr bool isParent(coord_t x, coord_t y, coord_t z,
	                                      depth_t depth = 0) const
	{
		return !isLeaf(x, y, z, depth);
	}

	//
	// Exists
	//

	// TODO: Add comment
	[[nodiscard]] constexpr bool exists(Node node) const
	{
		auto [index, _] = indexAndOffset(node);
		return codes_[index] == node.code().parent();
	}

	[[nodiscard]] constexpr bool exists(Code code) const
	{
		auto [index, _] = indexAndOffset(code);
		return codes_[index] == node.code().parent();
	}

	[[nodiscard]] constexpr bool exists(Key key) const { return exists(toCode(key)); }

	[[nodiscard]] constexpr bool exists(Point coord, depth_t depth = 0) const
	{
		return exists(toCode(coord, depth));
	}

	[[nodiscard]] constexpr bool exists(coord_t x, coord_t y, coord_t z,
	                                    depth_t depth = 0) const
	{
		return exists(toCode(x, y, z, depth));
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Modified                                       |
	|                                                                                     |
	**************************************************************************************/

	//
	// Modified
	//

	/*!
	 * @brief Check if the octree is in a modified state (i.e., at least one node has been
	 * modified).
	 *
	 * @return Whether the octree is in a modified state.
	 */
	[[nodiscard]] constexpr bool isModified() const
	{
		return modified_[rootIndex()][rootOffset()];
	}

	/*!
	 * @brief Check if a node of the octree is in a modified state (i.e., the node
	 * or one of its children has been modified).
	 *
	 * @param node The node to check.
	 * @return Whether the node is in a modified state.
	 */
	[[nodiscard]] constexpr bool isModified(Node node) const
	{
		return isModified(index(node));
	}

	/*!
	 * @brief Check if a node corresponding to a code is in a modified state (i.e., the node
	 * or one of its children has been modified).
	 *
	 * @param code The code of the node to check.
	 * @return Whether the node is in a modified state.
	 */
	[[nodiscard]] constexpr bool isModified(Code code) const
	{
		return isModified(index(code));
	}

	/*!
	 * @brief Check if a node corresponding to a key is in a modified state (i.e., the node
	 * or one of its children has been modified).
	 *
	 * @param key The key of the node to check.
	 * @return Whether the node is in a modified state.
	 */
	[[nodiscard]] constexpr bool isModified(Key key) const
	{
		return isModified(toCode(key));
	}

	/*!
	 * @brief Check if a node corresponding to a coordinate at a specified depth is in a
	 * modified state (i.e., the node or one of its children has been modified).
	 *
	 * @param coord The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is in a modified state.
	 */
	[[nodiscard]] constexpr bool isModified(Point coord, depth_t depth = 0) const
	{
		return isModified(toCode(coord, depth));
	}

	/*!
	 * @brief Check if a node corresponding to a coordinate at a specified depth is in a
	 * modified state (i.e., the node or one of its children has been modified).
	 *
	 * @param x,y,z The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is in a modified state.
	 */
	[[nodiscard]] constexpr bool isModified(coord_t x, coord_t y, coord_t z,
	                                        depth_t depth = 0) const
	{
		return isModified(toCode(x, y, z, depth));
	}

	//
	// Set modified
	//

	/*!
	 * @brief Set all nodes down to and including 'min_depth' to modified state.
	 *
	 * @param min_depth The minimum depth to set nodes to the modified state.
	 */
	void setModified(depth_t min_depth = 0)
	{
		if (rootDepth() >= min_depth) {
			setModified(rootIndex(), rootOffset(), rootDepth(), min_depth);
		}
	}

	/*!
	 * @brief Set a node and all its children down to and including 'min_depth' to the
	 * modified state. This will also set the parents of the node to the modified state.
	 *
	 * @param node The node.
	 * @param min_depth The minimum depth to set nodes to the modified state.
	 */
	void setModified(Node node, depth_t min_depth = 0)
	{
		if (rootDepth() < min_depth) {
			return;
		}

		if (node.depth() < min_depth) {
			setParentsModified(node.code().toDepth(min_depth - 1));
		} else {
			auto index = create(node);
			if (noneModified(index.index)) {
				setParentsModified(node.code());
			}
			setModified(index, node.depth(), min_depth);
		}
	}

	/*!
	 * @brief Set a node, corresponding to a code, and all its children down to and
	 * including 'min_depth' to the modified state. This will also set the parents of the
	 * node to the modified state.
	 *
	 * @param code The code of the node.
	 * @param min_depth The minimum depth to set nodes to the modified state.
	 */
	void setModified(Code code, depth_t min_depth = 0)
	{
		if (rootDepth() < min_depth) {
			return;
		}

		if (code.depth() < min_depth) {
			setParentsModified(code.toDepth(min_depth - 1));
		} else {
			Index index = create(code);
			if (noneModified(index.index)) {
				setParentsModified(code);
			}
			setModified(index, code.depth(), min_depth);
		}
	}

	/*!
	 * @brief Set a node, corresponding to a key, and all its children down to and including
	 * 'min_depth' to the modified state. This will also set the parents of the node to the
	 * modified state.
	 *
	 * @param key The key of the node.
	 * @param min_depth The minimum depth to set nodes to the modified state.
	 */
	void setModified(Key key, depth_t min_depth = 0)
	{
		setModified(toCode(key), min_depth);
	}

	/*!
	 * @brief Set a node, corresponding to a coordinate and a specified depth, and all its
	 * children down to and including 'min_depth' to the modified state. This will also set
	 * the parents of the node to the modified state.
	 *
	 * @param coord The coordinate of the node.
	 * @param min_depth The minimum depth to set nodes to the modified state.
	 * @param depth The depth of the node.
	 */
	void setModified(Point coord, depth_t min_depth = 0, depth_t depth = 0)
	{
		setModified(toCode(coord, depth), min_depth);
	}

	/*!
	 * @brief Set a node, corresponding to a coordinate and a specified depth, and all its
	 * children down to and including 'min_depth' to the modified state. This will also set
	 * the parents of the node to the modified state.
	 *
	 * @param x,y,z The coordinate of the node.
	 * @param min_depth The minimum depth to set nodes to the modified state.
	 * @param depth The depth of the node.
	 */
	void setModified(coord_t x, coord_t y, coord_t z, depth_t min_depth = 0,
	                 depth_t depth = 0)
	{
		setModified(toCode(x, y, z, depth), min_depth);
	}

	//
	// Set parents modified
	//

	void setParentsModified(depth_t min_depth = 0)
	{
		// TODO: Implement
	}

	//
	// Reset modified
	//

	/*!
	 * @brief Reset all nodes up to and including 'max_depth' from modified state.
	 *
	 * @note Only use this if you know what you are doing or if you do *not* plan to query
	 * for information in the octree. This function does not propagate information up the
	 * tree, hence queries will not function as expected. If you want to make queries then
	 * you should use 'propagateModified' instead.
	 *
	 * @param max_depth The maximum depth to reset nodes from the modified state.
	 */
	void resetModified(depth_t max_depth = maxDepthLevels())
	{
		resetModified(rootIndex(), rootOffset(), rootDepth(), max_depth);
	}

	/*!
	 * @brief Reset a node and its children up to and including 'max_depth' from modified
	 * state.
	 *
	 * @note Only use this if you know what you are doing or if you do *not* plan to query
	 * for information in the octree. This function does not propagate information up the
	 * tree, hence queries will not function as expected. If you want to make queries then
	 * you should use 'propagateModified' instead.
	 *
	 * @param node The node.
	 * @param max_depth The maximum depth to reset nodes from the modified state.
	 */
	void resetModified(Node node, depth_t max_depth = maxDepthLevels())
	{
		resetModified(create(node), node.depth(), max_depth);
	}

	/*!
	 * @brief Reset a node, corresponding to a code, and its children up to and including
	 * 'max_depth' from modified state.
	 *
	 * @note Only use this if you know what you are doing or if you do *not* plan to query
	 * for information in the octree. This function does not propagate information up the
	 * tree, hence queries will not function as expected. If you want to make queries then
	 * you should use 'propagateModified' instead.
	 *
	 * @param code The code of the node.
	 * @param max_depth The maximum depth to reset nodes from the modified state.
	 */
	void resetModified(Code code, depth_t max_depth = maxDepthLevels())
	{
		resetModified(create(code), code.depth(), max_depth);
	}

	/*!
	 * @brief Reset a node, corresponding to a key, and its children up to and including
	 * 'max_depth' from modified state.
	 *
	 * @note Only use this if you know what you are doing or if you do *not* plan to query
	 * for information in the octree. This function does not propagate information up the
	 * tree, hence queries will not function as expected. If you want to make queries then
	 * you should use 'propagateModified' instead.
	 *
	 * @param key The key of the node.
	 * @param max_depth The maximum depth to reset nodes from the modified state.
	 */
	void resetModified(Key key, depth_t max_depth = maxDepthLevels())
	{
		resetModified(toCode(key), max_depth);
	}

	/*!
	 * @brief Reset a node, corresponding to a coordinate and a specified depth, and its
	 * children up to and including 'max_depth' from modified state.
	 *
	 * @note Only use this if you know what you are doing or if you do *not* plan to query
	 * for information in the octree. This function does not propagate information up the
	 * tree, hence queries will not function as expected. If you want to make queries then
	 * you should use 'propagateModified' instead.
	 *
	 * @param coord The coordinate of the node.
	 * @param max_depth The maximum depth to reset nodes from the modified state.
	 * @param depth The depth of the node.
	 */
	void resetModified(Point coord, depth_t max_depth = maxDepthLevels(), depth_t depth = 0)
	{
		resetModified(toCode(coord, depth), max_depth);
	}

	/*!
	 * @brief Reset a node, corresponding to a coordinate and a specified depth, and its
	 * children up to and including 'max_depth' from modified state.
	 *
	 * @note Only use this if you know what you are doing or if you do *not* plan to query
	 * for information in the octree. This function does not propagate information up the
	 * tree, hence queries will not function as expected. If you want to make queries then
	 * you should use 'propagateModified' instead.
	 *
	 * @param x,y,z The coordinate of the node.
	 * @param max_depth The maximum depth to reset nodes from the modified state.
	 * @param depth The depth of the node.
	 */
	void resetModified(coord_t x, coord_t y, coord_t z,
	                   depth_t max_depth = maxDepthLevels(), depth_t depth = 0)
	{
		resetModified(toCode(x, y, z, depth), max_depth);
	}

	//
	// Propagate
	//

	/*!
	 * @brief Propagate modified information up the octree to and including 'max_depth'.
	 *
	 * @param keep_modified Whether propagated node's modified state should be reset.
	 * @param max_depth The maximum depth to propagate information.
	 */
	void propagateModified(bool keep_modified = false, depth_t max_depth = maxDepthLevels())
	{
		propagateModified(rootIndex(), rootOffset(), rootDepth(), keep_modified, max_depth);
	}

	/*!
	 * @brief Propagate a node's children modified information up to and including
	 * 'max_depth'.
	 *
	 * @param node The node.
	 * @param keep_modified Whether propagated node's modified state should be reset.
	 * @param max_depth The maximum depth to propagate information.
	 */
	void propagateModified(Node node, bool keep_modified = false,
	                       depth_t max_depth = maxDepthLevels())
	{
		propagateModified(index(node), node.depth(), keep_modified, max_depth);
	}

	/*!
	 * @brief Propagate a node's, corresponding to a code, children modified information up
	 * to and including 'max_depth'.
	 *
	 * @param code The code of the node.
	 * @param keep_modified Whether propagated node's modified state should be reset.
	 * @param max_depth The maximum depth to propagate information.
	 */
	void propagateModified(Code code, bool keep_modified = false,
	                       depth_t max_depth = maxDepthLevels())
	{
		propagateModified(index(code), code.depth(), keep_modified, max_depth);
	}

	/*!
	 * @brief Propagate a node's, corresponding to a key, children modified information up
	 * to and including 'max_depth'.
	 *
	 * @param key The key of the node.
	 * @param keep_modified Whether propagated node's modified state should be reset.
	 * @param max_depth The maximum depth to propagate information.
	 */
	void propagateModified(Key key, bool keep_modified = false,
	                       depth_t max_depth = maxDepthLevels())
	{
		propagateModified(toCode(key), keep_modified, max_depth);
	}

	/*!
	 * @brief Propagate a node's, corresponding to a coordinate and a specified depth,
	 * children modified information up to and including 'max_depth'.
	 *
	 * @param coord The coordinate of the node.
	 * @param keep_modified Whether propagated node's modified state should be reset.
	 * @param max_depth The maximum depth to propagate information.
	 * @param depth The depth of the node.
	 */
	void propagateModified(Point coord, bool keep_modified = false,
	                       depth_t max_depth = maxDepthLevels(), depth_t depth = 0)
	{
		propagateModified(toCode(coord, depth), depth, keep_modified, max_depth);
	}

	/*!
	 * @brief Propagate a node's, corresponding to a coordinate and a specified depth,
	 * children modified information up to and including 'max_depth'.
	 *
	 * @param x,y,z The coordinate of the node.
	 * @param keep_modified Whether propagated node's modified state should be reset.
	 * @param max_depth The maximum depth to propagate information.
	 * @param depth The depth of the node.
	 */
	void propagateModified(coord_t x, coord_t y, coord_t z, bool keep_modified = false,
	                       depth_t max_depth = maxDepthLevels(), depth_t depth = 0)
	{
		propagateModified(toCode(x, y, z, depth), keep_modified, max_depth);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Root                                         |
	|                                                                                     |
	**************************************************************************************/

	//
	// Is
	//

	/*!
	 * @brief Check if a node is the root node.
	 *
	 * @param node The node.
	 * @return Whether the node is the root node.
	 */
	[[nodiscard]] bool isRoot(Node node) const { return isRoot(node.code()); }

	/*!
	 * @brief Check if a node, corresponding to a code, is the root node.
	 *
	 * @param code The code of the node.
	 * @return Whether the node is the root node.
	 */
	[[nodiscard]] bool isRoot(Code code) const { return rootCode() == code; }

	/*!
	 * @brief Check if a node, corresponding to a key, is the root node.
	 *
	 * @param key The key of the node.
	 * @return Whether the node is the root node.
	 */
	[[nodiscard]] bool isRoot(Key key) const { return isRoot(toCode(key)); }

	/*!
	 * @brief Check if a node, corresponding to a coordinate at a specific depth, is the
	 * root node.
	 *
	 * @param coord The coordinate of the node.
	 * @param depth The depth of the node.
	 * @return Whether the node is the root node.
	 */
	[[nodiscard]] bool isRoot(Point coord, depth_t depth = 0) const
	{
		return isRoot(toCode(coord, depth));
	}

	/*!
	 * @brief Check if a node, corresponding to a coordinate at a specific depth, is the
	 * root node.
	 *
	 * @param x,y,z The coordinate of the node.
	 * @param depth The depth of the node.
	 * @return Whether the node is the root node.
	 */
	[[nodiscard]] bool isRoot(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return isRoot(toCode(x, y, z, depth));
	}

	//
	// Get
	//

	/*!
	 * @brief Get the root node.
	 *
	 * @return The root node.
	 */
	[[nodiscard]] constexpr Node rootNode() const { return Node(rootIndex(), rootCode()); }

	/*!
	 * @brief Get the root node with bounding volume.
	 *
	 * @return The root node with bounding volume.
	 */
	[[nodiscard]] constexpr NodeBV rootNodeBV() const
	{
		return NodeBV(rootNode(), rootBoundingVolume());
	}

	//
	// Code
	//

	/*!
	 * @brief Get the code for the root node.
	 *
	 * @return The root node code.
	 */
	[[nodiscard]] constexpr Code rootCode() const
	{
		return Code(rootIndex() << 3 * rootDepth(), rootDepth());
	}

	//
	// Depth
	//

	/*!
	 * @brief Get the depth of the root node.
	 *
	 * @return The depth of the root node.
	 */
	[[nodiscard]] constexpr depth_t rootDepth() const noexcept { return depthLevels() - 1; }

	//
	// Size
	//

	/*!
	 * @brief Get the size of the root node.
	 *
	 * @note This is the same as `size()`.
	 *
	 * @return The size of the root node.
	 */
	[[nodiscard]] constexpr node_size_t rootSize() const noexcept { return size(); }

	//
	// Center
	//

	/*!
	 * @brief Get the center of the root node.
	 *
	 * @note This is the same as `center()`.
	 *
	 * @return The center of the root node.
	 */
	[[nodiscard]] constexpr Point rootCenter() const noexcept { return center(); }

	//
	// Bounding volume
	//

	/*!
	 * @brief Get the bounding volume of the root node.
	 *
	 * @note This is the same as `boundingVolmue()`.
	 *
	 * @return The bounding volume of the root node.
	 */
	[[nodiscard]] constexpr geometry::AAEBB rootBoundingVolume() const noexcept
	{
		return boundingVolume();
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Node                                         |
	|                                                                                     |
	**************************************************************************************/

	//
	// Size
	//

	/*!
	 * @brief Get the size of a node.
	 *
	 * @param node The node.
	 * @return The size of the node.
	 */
	[[nodiscard]] constexpr node_size_t size(Node node) const { return size(node.depth()); }

	//
	// Center
	//

	/*!
	 * @brief The center of a node.
	 *
	 * @param node The node.
	 * @return The center of the node.
	 */
	template <class NodeType>
	[[nodiscard]] Point center(NodeType const& node) const
	{
		if constexpr (std::is_same_v<NodeBV, NodeType>) {
			return node.center();
		} else {
			return toCoord(node.code());
		}
	}

	//
	// Bounding volume
	//

	/*!
	 * @brief Bounding volume for a node.
	 *
	 * @param node The node
	 * @return Bounding volume for the node.
	 */
	template <class NodeType>
	[[nodiscard]] geometry::AAEBB boundingVolume(NodeType const& node) const
	{
		if constexpr (std::is_same_v<NodeBV, NodeType>) {
			return node.boundingVolume();

		} else {
			return geometry::AAEBB(center(node), size(node) / 2);
		}
	}

	//
	// At
	//

	/*!
	 * @brief Get the node corresponding to a code with bounds checking.
	 *
	 * @note The node can be higher up the tree than the specified depth. This happens if
	 * the node at a higher depth has no children. If it is neccessary that the node is at
	 * the specified depth, then the corresponding 'createNodeChecked' function can be used.
	 * The data inside the nodes returned by this function and 'createNodeChecked' will be
	 * the same, so it is only neccessary to use 'createNodeChecked' if you intend to alter
	 * what the node stores.
	 *
	 * @param code The code.
	 * @return The node.
	 */
	[[nodiscard]] std::optional<Node> at(Code code) const
	{
		return code.depth() <= rootDepth() ? std::optional<Node>(operator()(code))
		                                   : std::nullopt;
	}

	/*!
	 * @brief Get the node corresponding to a key with bounds checking.
	 *
	 * @note The node can be higher up the tree than the specified depth. This happens if
	 * the node at a higher depth has no children. If it is neccessary that the node is at
	 * the specified depth, then the corresponding 'createNodeChecked' function can be used.
	 * The data inside the nodes returned by this function and 'createNodeChecked' will be
	 * the same, so it is only neccessary to use 'createNodeChecked' if you intend to alter
	 * what the node stores.
	 *
	 * @param key The key.
	 * @return The node.
	 */
	[[nodiscard]] std::optional<Node> at(Key key) const { return at(toCode(key)); }

	/*!
	 * @brief Get the node corresponding to a coordinate at a specific depth with bounds
	 * checking.
	 *
	 * @note The node can be higher up the tree than the specified depth. This happens if
	 * the node at a higher depth has no children. If it is neccessary that the node is at
	 * the specified depth, then the corresponding 'createNodeChecked' function can be used.
	 * The data inside the nodes returned by this function and 'createNodeChecked' will be
	 * the same, so it is only neccessary to use 'createNodeChecked' if you intend to alter
	 * what the node stores.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The node.
	 */
	[[nodiscard]] std::optional<Node> at(Point coord, depth_t depth = 0) const
	{
		if (auto code = toCodeChecked(coord, depth)) {
			return std::optional<Node>(*code);
		} else {
			return std::nullopt;
		}
	}

	/*!
	 * @brief Get the node corresponding to a coordinate at a specific depth with bounds
	 * checking.
	 *
	 * @note The node can be higher up the tree than the specified depth. This happens if
	 * the node at a higher depth has no children. If it is neccessary that the node is at
	 * the specified depth, then the corresponding 'createNodeChecked' function can be used.
	 * The data inside the nodes returned by this function and 'createNodeChecked' will be
	 * the same, so it is only neccessary to use 'createNodeChecked' if you intend to alter
	 * what the node stores.
	 *
	 * @param x,y,z The coordinate.
	 * @param depth The depth.
	 * @return The node.
	 */
	[[nodiscard]] std::optional<Node> at(coord_t x, coord_t y, coord_t z,
	                                     depth_t depth = 0) const
	{
		return at(Point(x, y, z), depth);
	}

	//
	// Function call operator
	//

	// TODO: Add comment
	[[nodiscard]] Node operator()(Node node) const
	{
		auto [index, _] = indexAndOffset(node);
		return Node(index, node.code());
	}

	/*!
	 * @brief Get the node corresponding to a code.
	 *
	 * @note The node can be higher up the tree than the specified depth. This happens if
	 * the node at a higher depth has no children. If it is neccessary that the node is at
	 * the specified depth, then the corresponding 'createNode' function can be used. The
	 * data inside the nodes returned by this function and 'createNode' will be the same, so
	 * it is only neccessary to use 'createNode' if you intend to alter what the node
	 * stores.
	 *
	 * @param code The code.
	 * @return The node.
	 */
	[[nodiscard]] Node operator()(Code code) const
	{
		auto [index, _] = indexAndOffset(code);
		return Node(index, code);
	}

	/*!
	 * @brief Get the node corresponding to a key.
	 *
	 * @note The node can be higher up the tree than the specified depth. This happens if
	 * the node at a higher depth has no children. If it is neccessary that the node is at
	 * the specified depth, then the corresponding 'createNode' function can be used. The
	 * data inside the nodes returned by this function and 'createNode' will be the same, so
	 * it is only neccessary to use 'createNode' if you intend to alter what the node
	 * stores.
	 *
	 * @param key The key.
	 * @return The node.
	 */
	[[nodiscard]] Node operator()(Key key) const { return operator()(toCode(key)); }

	/*!
	 * @brief Get the node corresponding to a coordinate at a specific depth.
	 *
	 * @note The node can be higher up the tree than the specified depth. This happens if
	 * the node at a higher depth has no children. If it is neccessary that the node is at
	 * the specified depth, then the corresponding 'createNode' function can be used. The
	 * data inside the nodes returned by this function and 'createNode' will be the same, so
	 * it is only neccessary to use 'createNode' if you intend to alter what the node
	 * stores.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The node.
	 */
	[[nodiscard]] Node operator()(Point coord, depth_t depth = 0) const
	{
		return operator()(toCode(coord, depth));
	}

	/*!
	 * @brief Get the node corresponding to a coordinate at a specific depth.
	 *
	 * @note The node can be higher up the tree than the specified depth. This happens if
	 * the node at a higher depth has no children. If it is neccessary that the node is at
	 * the specified depth, then the corresponding 'createNode' function can be used. The
	 * data inside the nodes returned by this function and 'createNode' will be the same, so
	 * it is only neccessary to use 'createNode' if you intend to alter what the node
	 * stores.
	 *
	 * @param x,y,z The coordinate.
	 * @param depth The depth.
	 * @return The node.
	 */
	[[nodiscard]] Node operator()(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return operator()(toCode(x, y, z, depth));
	}

	//
	// Sibling
	//

	/*!
	 * @brief Get the sibling of a node.
	 *
	 * @param node The node.
	 * @param sibling_index The index of the sibling.
	 * @return The sibling.
	 */
	[[nodiscard]] Node sibling(Node node, index_t sibling_index) const
	{
		return Node(node.data(), node.code().sibling(sibling_index), node.dataDepth());
	}

	/*!
	 * @brief Get the sibling of a node.
	 *
	 * @param node The node.
	 * @param sibling_index The index of the sibling.
	 * @return The sibling.
	 */
	[[nodiscard]] NodeBV sibling(NodeBV const& node, index_t sibling_index) const
	{
		geometry::AAEBB aaebb(
		    siblingCenter(node.center(), node.halfSize(), node.index(), sibling_index),
		    node.halfSize());
		return NodeBV(sibling(static_cast<Node>(node), sibling_index), aaebb);
	}

	/*!
	 * @brief Get the sibling of a node with bounds checking.
	 *
	 * @param node The node.
	 * @param sibling_index The index of the sibling.
	 * @return The sibling.
	 */
	template <class Node>
	[[nodiscard]] Node siblingChecked(Node const& node, index_t sibling_index) const
	{
		if (!isRoot(node)) {
			throw std::out_of_range("Node has no siblings");
		} else if (7 < sibling_index) {
			throw std::out_of_range("sibling_index out of range");
		}
		return sibling(node, sibling_index);
	}

	//
	// Child
	//

	/*!
	 * @brief Get a child of a node.
	 *
	 * @param node The node.
	 * @param child_index The index of the child.
	 * @return The child.
	 */
	[[nodiscard]] Node child(Node node, offset_t child_idx) const
	{
		Index idx = toIndex(node);
		return Node(isLeaf(idx) ? child(idx) : idx, node.code().child(child_idx));
	}

	/*!
	 * @brief Get a child of a node.
	 *
	 * @param node The node.
	 * @param child_index The index of the child.
	 * @return The child.
	 */
	[[nodiscard]] NodeBV child(NodeBV const& node, index_t child_index) const
	{
		auto const child_half_size = node.halfSize() / 2;
		geometry::AAEBB child_aaebb(childCenter(node.center(), child_half_size, child_index),
		                            child_half_size);

		return NodeBV(child(static_cast<Node>(node), child_index), child_aaebb);
	}

	/*!
	 * @brief Get a child of a node with bounds checking.
	 *
	 * @param node The node.
	 * @param child_index The index of the child.
	 * @return The child.
	 */
	template <class Node>
	[[nodiscard]] Node childChecked(Node const& node, index_t child_index) const
	{
		if (!isParent(node)) {
			throw std::out_of_range("Node has no children");
		} else if (7 < child_index) {
			throw std::out_of_range("child_index out of range");
		}
		return child(node, child_index);
	}

	//
	// Parent
	//

	/*!
	 * @brief Get the parent of a node.
	 *
	 * @param node The node.
	 * @return The parent.
	 */
	[[nodiscard]] Node parent(Node node) const
	{
		return node.depth() == node.dataDepth() ? operator()(node.code().parent())
		                                        : Node(node.data(), node.code().parent(),
		                                               node.dataDepth());
	}

	/*!
	 * @brief Get the parent of a node.
	 *
	 * @param node The node.
	 * @return The parent.
	 */
	[[nodiscard]] NodeBV parent(NodeBV const& node) const
	{
		return toNodeBV(parent(static_cast<Node>(node)));
	}

	/*!
	 * @brief Get the parent of a node with bounds checking.
	 *
	 * @param node The node.
	 * @return The parent.
	 */
	template <class Node>
	[[nodiscard]] Node parentChecked(Node const& node) const
	{
		if (rootDepth() <= node.depth()) {
			throw std::out_of_range("Node has no parent");
		}
		return parent(node);
	}

	//
	// NodeBV
	//

	/*!
	 * @brief Convert a node to a node with bounding volume.
	 *
	 * @param node The node.
	 * @return The node with bounding volume.
	 */
	[[nodiscard]] NodeBV toNodeBV(Node node) const
	{
		return NodeBV(node, boundingVolume(node));
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Conversion                                      |
	|                                                                                     |
	**************************************************************************************/

	//
	// Code
	//

	/*!
	 * @brief Convert a key to a code.
	 *
	 * @param key The key to convert.
	 * @return The code corresponding to the key.
	 */
	[[nodiscard]] static constexpr Code toCode(Key key) noexcept { return Code(key); }

	/*!
	 * @brief Convert a coordinate at a specific depth to a code.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The code corresponding to the coordinate at the specific depth.
	 */
	[[nodiscard]] constexpr Code toCode(Point coord, depth_t depth = 0) const noexcept
	{
		return toCode(toKey(coord, depth));
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a code.
	 *
	 * @param x,y,z The coordinate.
	 * @param depth The depth.
	 * @return The code corresponding to the coordinate at the specific depth.
	 */
	[[nodiscard]] constexpr Code toCode(coord_t x, coord_t y, coord_t z,
	                                    depth_t depth = 0) const noexcept
	{
		return toCode(toKey(x, y, z, depth));
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a code with bounds check.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The code corresponding to the coordinate at the specific depth.
	 */
	[[nodiscard]] constexpr std::optional<Code> toCodeChecked(
	    Point coord, depth_t depth = 0) const noexcept
	{
		std::optional<key_t> key = toKeyChecked(coord, depth);
		return key ? std::optional<Code>(toCode(*key)) : std::nullopt;
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a code with bounds check.
	 *
	 * @param x,y,z The coordinate.
	 * @param depth The depth.
	 * @return The code corresponding to the coordinate at the specific depth.
	 */
	[[nodiscard]] constexpr std::optional<Code> toCodeChecked(
	    coord_t x, coord_t y, coord_t z, depth_t depth = 0) const noexcept
	{
		return toCodeChecked(Point(x, y, z), depth);
	}

	//
	// Key
	//

	/*!
	 * @brief Convert a code to a key.
	 *
	 * @param code The code to convert.
	 * @return The key corresponding to the code.
	 */
	[[nodiscard]] static constexpr Key toKey(Code code) noexcept { return code; }

	/*!
	 * @brief COnvert a coordinate at a specific depth to a key.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The corresponding key.
	 */
	[[nodiscard]] constexpr Key toKey(Point coord, depth_t depth = 0) const noexcept
	{
		return Key(toKey(coord.x, depth), toKey(coord.y, depth), toKey(coord.z, depth),
		           depth);
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a key.
	 *
	 * @param x,y,z The coordinate.
	 * @param depth The depth.
	 * @return The corresponding key.
	 */
	[[nodiscard]] constexpr Key toKey(coord_t x, coord_t y, coord_t z,
	                                  depth_t depth = 0) const noexcept
	{
		return Key(toKey(x, depth), toKey(y, depth), toKey(z, depth), depth);
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a key with bounds check.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The corresponding key.
	 */
	[[nodiscard]] constexpr std::optional<Key> toKeyChecked(
	    Point coord, depth_t depth = 0) const noexcept
	{
		return rootDepth() >= depth && isWithin(coord)
		           ? std::optional<Key>(toKey(coord, depth))
		           : std::nullopt;
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a key with bounds check.
	 *
	 * @param x,y,z The coordinate.
	 * @param depth The depth.
	 * @return The corresponding key.
	 */
	[[nodiscard]] constexpr std::optional<Key> toKeyChecked(
	    coord_t x, coord_t y, coord_t z, depth_t depth = 0) const noexcept
	{
		return toKeyChecked(Point(x, y, z), depth);
	}

	//
	// Coordinate
	//

	/*!
	 * @brief Convert a code to a coordinate.
	 *
	 * @param code The code.
	 * @return The corresponding coordinate.
	 */
	[[nodiscard]] constexpr Point toCoord(Code code) const noexcept
	{
		return toCoord(toKey(code));
	}

	/*!
	 * @brief Convert a key to a coordinate.
	 *
	 * @param key The key.
	 * @return The corresponding coordinate.
	 */
	[[nodiscard]] constexpr Point toCoord(Key key) const noexcept
	{
		return Point(toCoord(key[0], key.depth()), toCoord(key[1], key.depth()),
		             toCoord(key[2], key.depth()));
	}

	/*!
	 * @brief Convert a code to a coordinate with bounds check.
	 *
	 * @param Code The code.
	 * @return The corresponding coordinate.
	 */
	[[nodiscard]] constexpr std::optional<Point> toCoordChecked(Code code) const noexcept
	{
		return toCoordChecked(toKey(code));
	}

	/*!
	 * @brief Convert a key to a coordinate with bounds check.
	 *
	 * @param key The key.
	 * @return The corresponding coordinate.
	 */
	[[nodiscard]] constexpr std::optional<Point> toCoordChecked(Key key) const noexcept
	{
		return rootDepth() >= key.depth() ? std::optional<Point>(toCoord(key)) : std::nullopt;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Traverse                                       |
	|                                                                                     |
	**************************************************************************************/

	/*!
	 * @brief Depth first traversal of the octree, starting at the root node. The function
	 * 'f' will be called for each node traverse. If 'f' returns false then the children of
	 * the node will also be traverse, otherwise they will not.
	 *
	 * @param f The callback function to be called for each node traversed.
	 */
	template <class UnaryFunction>
	void traverse(UnaryFunction f) const
	{
		// FIXME: Should it be const also?
		if constexpr (std::is_same_v<Node, util::argument<UnaryFunction, 0>>) {
			traverseRecurs(rootNode(), f);
		} else {
			traverseRecurs(rootNodeBV(), f);
		}
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node. The function 'f'
	 * will be called for each node traverse. If 'f' returns false then the children of the
	 * node will also be traverse, otherwise they will not.
	 *
	 * @param node The node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 */
	template <class UnaryFunction>
	void traverse(Node node, UnaryFunction f) const
	{
		// FIXME: Should it be const also?
		if constexpr (std::is_same_v<util::argument<UnaryFunction, 0>, Node>) {
			traverseRecurs(node, f);
		} else {
			traverseRecurs(NodeBV(node, boundingVolume(node)), f);
		}
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node corresponding to the
	 * code. The function 'f' will be called for each node traverse. If 'f' returns false
	 * then the children of the node will also be traverse, otherwise they will not.
	 *
	 * @param code The code to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 */
	template <class UnaryFunction>
	void traverse(Code code, UnaryFunction f) const
	{
		traverse(operator()(code), f);
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node corresponding to the
	 * key. The function 'f' will be called for each node traverse. If 'f' returns false
	 * then the children of the node will also be traverse, otherwise they will not.
	 *
	 * @param key The key to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 */
	template <class UnaryFunction>
	void traverse(Key key, UnaryFunction f) const
	{
		traverse(toCode(key), f);
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node corresponding to the
	 * coordinate at a specified depth. The function 'f' will be called for each node
	 * traverse. If 'f' returns false then the children of the node will also be traverse,
	 * otherwise they will not.
	 *
	 * @param coord The coord to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 * @param depth The depth of the node.
	 */
	template <class UnaryFunction>
	void traverse(Point coord, UnaryFunction f, depth_t depth = 0) const
	{
		traverse(toCode(coord, depth), f);
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node corresponding to the
	 * coordinate at a specified depth. The function 'f' will be called for each node
	 * traverse. If 'f' returns false then the children of the node will also be traverse,
	 * otherwise they will not.
	 *
	 * @param x,y,z The coord to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 * @param depth The depth of the node.
	 */
	template <class UnaryFunction>
	void traverse(coord_t x, coord_t y, coord_t z, UnaryFunction f, depth_t depth = 0) const
	{
		traverse(toCode(x, y, z, depth), f);
	}

	/*! TODO: Update info for all nearest
	 * @brief Traverse the octree in the orderDepth first traversal of the octree, starting
	 * at the root node. The function 'f' will be called for each node traverse. If 'f'
	 * returns false then the children of the node will also be traverse, otherwise they
	 * will not.
	 *
	 * @param f The callback function to be called for each node traversed.
	 */
	template <class Geometry, class UnaryFunction>
	void traverseNearest(Geometry const& g, UnaryFunction f) const
	{
		traverseNearestRecurs(rootNodeBV(), g, f);
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node. The function 'f'
	 * will be called for each node traverse. If 'f' returns false then the children of the
	 * node will also be traverse, otherwise they will not.
	 *
	 * @param node The node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 */
	template <class Geometry, class UnaryFunction>
	void traverseNearest(Node node, Geometry const& g, UnaryFunction f) const
	{
		traverseNearestRecurs(NodeBV(node, boundingVolume(node)), g, f);
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node corresponding to the
	 * code. The function 'f' will be called for each node traverse. If 'f' returns false
	 * then the children of the node will also be traverse, otherwise they will not.
	 *
	 * @param code The code to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 */
	template <class Geometry, class UnaryFunction>
	void traverseNearest(Code code, Geometry const& g, UnaryFunction f) const
	{
		traverseNearest(operator()(code), g, f);
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node corresponding to the
	 * key. The function 'f' will be called for each node traverse. If 'f' returns false
	 * then the children of the node will also be traverse, otherwise they will not.
	 *
	 * @param key The key to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 */
	template <class Geometry, class UnaryFunction>
	void traverseNearest(Key key, Geometry const& g, UnaryFunction f) const
	{
		traverseNearest(toCode(key), g, f);
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node corresponding to the
	 * coordinate at a specified depth. The function 'f' will be called for each node
	 * traverse. If 'f' returns false then the children of the node will also be traverse,
	 * otherwise they will not.
	 *
	 * @param coord The coord to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 * @param depth The depth of the node.
	 */
	template <class Geometry, class UnaryFunction>
	void traverseNearest(Point coord, Geometry const& g, UnaryFunction f,
	                     depth_t depth = 0) const
	{
		traverseNearest(toCode(coord, depth), g, f);
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node corresponding to the
	 * coordinate at a specified depth. The function 'f' will be called for each node
	 * traverse. If 'f' returns false then the children of the node will also be traverse,
	 * otherwise they will not.
	 *
	 * @param x,y,z The coord to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 * @param depth The depth of the node.
	 */
	template <class Geometry, class UnaryFunction>
	void traverseNearest(coord_t x, coord_t y, coord_t z, Geometry const& g,
	                     UnaryFunction f, depth_t depth = 0) const
	{
		traverseNearest(toCode(x, y, z, depth), g, f);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Query                                        |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	//
	// Query
	//

	template <class Predicates>
	[[nodiscard]] Query query(Predicates&& predicates, bool early_stopping = false) const
	{
		return Query(beginQuery(std::forward<Predicates>(predicates), early_stopping),
		             endQuery());
	}

	template <class Predicates>
	[[nodiscard]] Query query(Node node, Predicates&& predicates,
	                          bool early_stopping = false) const
	{
		return Query(beginQuery(node, std::forward<Predicates>(predicates), early_stopping),
		             endQuery());
	}

	template <class Predicates>
	[[nodiscard]] Query query(Code code, Predicates&& predicates,
	                          bool early_stopping = false) const
	{
		return Query(beginQuery(code, std::forward<Predicates>(predicates), early_stopping),
		             endQuery());
	}

	template <class Predicates>
	[[nodiscard]] Query query(Key key, Predicates&& predicates,
	                          bool early_stopping = false) const
	{
		return query(toCode(key), std::forward<Predicates>(predicates), early_stopping);
	}

	template <class Predicates>
	[[nodiscard]] Query query(Point coord, depth_t depth, Predicates&& predicates,
	                          bool early_stopping = false) const
	{
		return query(toCode(coord, depth), std::forward<Predicates>(predicates),
		             early_stopping);
	}

	template <class Predicates>
	[[nodiscard]] Query query(coord_t x, coord_t y, coord_t z, depth_t depth,
	                          Predicates&& predicates, bool early_stopping = false) const
	{
		return query(toCode(x, y, z, depth), std::forward<Predicates>(predicates),
		             early_stopping);
	}

	//
	// Query bounding volume
	//

	template <class Predicates>
	[[nodiscard]] QueryBV queryBV(Predicates&& predicates,
	                              bool early_stopping = false) const
	{
		return QueryBV(beginQueryBV(std::forward<Predicates>(predicates), early_stopping),
		               endQueryBV());
	}

	template <class Predicates>
	[[nodiscard]] QueryBV queryBV(Node node, Predicates&& predicates,
	                              bool early_stopping = false) const
	{
		return QueryBV(
		    beginQueryBV(node, std::forward<Predicates>(predicates), early_stopping),
		    endQueryBV());
	}

	template <class Predicates>
	[[nodiscard]] QueryBV queryBV(Code code, Predicates&& predicates,
	                              bool early_stopping = false) const
	{
		return QueryBV(
		    beginQueryBV(code, std::forward<Predicates>(predicates), early_stopping),
		    endQueryBV());
	}

	template <class Predicates>
	[[nodiscard]] QueryBV queryBV(Key key, Predicates&& predicates,
	                              bool early_stopping = false) const
	{
		return queryBV(toCode(key), std::forward<Predicates>(predicates), early_stopping);
	}

	template <class Predicates>
	[[nodiscard]] QueryBV queryBV(Point coord, depth_t depth, Predicates&& predicates,
	                              bool early_stopping = false) const
	{
		return queryBV(toCode(coord, depth), std::forward<Predicates>(predicates),
		               early_stopping);
	}

	template <class Predicates>
	[[nodiscard]] QueryBV queryBV(coord_t x, coord_t y, coord_t z, depth_t depth,
	                              Predicates&& predicates,
	                              bool early_stopping = false) const
	{
		return queryBV(toCode(x, y, z, depth), std::forward<Predicates>(predicates),
		               early_stopping);
	}

	//
	// Query nearest
	//

	template <class Geometry, class Predicates>
	[[nodiscard]] QueryNearest queryNearest(Geometry&& geometry, Predicates&& predicates,
	                                        bool early_stopping = false) const
	{
		return QueryNearest(
		    beginQueryNearest(std::forward<Geometry>(geometry),
		                      std::forward<Predicates>(predicates), early_stopping),
		    endQueryNearest());
	}

	template <class Geometry, class Predicates>
	[[nodiscard]] QueryNearest queryNearest(Node node, Geometry&& geometry,
	                                        Predicates&& predicates,
	                                        bool early_stopping = false) const
	{
		return QueryNearest(
		    beginQueryNearest(node, std::forward<Geometry>(geometry),
		                      std::forward<Predicates>(predicates), early_stopping),
		    endQueryNearest());
	}

	template <class Geometry, class Predicates>
	[[nodiscard]] QueryNearest queryNearest(Code code, Geometry&& geometry,
	                                        Predicates&& predicates,
	                                        bool early_stopping = false) const
	{
		return QueryNearest(
		    beginQueryNearest(code, std::forward<Geometry>(geometry),
		                      std::forward<Predicates>(predicates), early_stopping),
		    endQueryNearest());
	}

	template <class Geometry, class Predicates>
	[[nodiscard]] QueryNearest queryNearest(Key key, Geometry&& geometry,
	                                        Predicates&& predicates,
	                                        bool early_stopping = false) const
	{
		return queryNearest(toCode(key), std::forward<Geometry>(geometry),
		                    std::forward<Predicates>(predicates), early_stopping);
	}

	template <class Geometry, class Predicates>
	[[nodiscard]] QueryNearest queryNearest(Point coord, depth_t depth, Geometry&& geometry,
	                                        Predicates&& predicates,
	                                        bool early_stopping = false) const
	{
		return queryNearest(toCode(coord, depth), std::forward<Geometry>(geometry),
		                    std::forward<Predicates>(predicates), early_stopping);
	}

	template <class Geometry, class Predicates>
	[[nodiscard]] QueryNearest queryNearest(coord_t x, coord_t y, coord_t z, depth_t depth,
	                                        Geometry&& geometry, Predicates&& predicates,
	                                        bool early_stopping = false) const
	{
		return queryNearest(toCode(x, y, z, depth), std::forward<Geometry>(geometry),
		                    std::forward<Predicates>(predicates), early_stopping);
	}

	//
	// Query to output
	//

	template <class Predicates, class OutputIt>
	OutputIt query(Predicates&& predicates, OutputIt d_first,
	               bool early_stopping = false) const
	{
		if constexpr (std::is_same_v<typename OutputIt::value_type, Node>) {
			return std::copy(beginQuery(std::forward<Predicates>(predicates), early_stopping),
			                 endQuery(), d_first);
		} else {
			return std::copy(beginQueryBV(std::forward<Predicates>(predicates), early_stopping),
			                 endQueryBV(), d_first);
		}
	}

	template <class Predicates, class OutputIt>
	OutputIt query(Node node, Predicates&& predicates, OutputIt d_first,
	               bool early_stopping = false) const
	{
		if constexpr (std::is_same_v<typename OutputIt::value_type, Node>) {
			return std::copy(
			    beginQuery(node, std::forward<Predicates>(predicates), early_stopping),
			    endQuery(), d_first);
		} else {
			return std::copy(
			    beginQueryBV(node, std::forward<Predicates>(predicates), early_stopping),
			    endQueryBV(), d_first);
		}
	}

	template <class Predicates, class OutputIt>
	OutputIt query(Code code, Predicates&& predicates, OutputIt d_first,
	               bool early_stopping = false) const
	{
		if constexpr (std::is_same_v<typename OutputIt::value_type, Node>) {
			return std::copy(
			    beginQuery(code, std::forward<Predicates>(predicates), early_stopping),
			    endQuery(), d_first);
		} else {
			return std::copy(
			    beginQueryBV(code, std::forward<Predicates>(predicates), early_stopping),
			    endQueryBV(), d_first);
		}
	}

	template <class Predicates, class OutputIt>
	OutputIt query(Key key, Predicates&& predicates, OutputIt d_first,
	               bool early_stopping = false) const
	{
		return query(toCode(key), std::forward<Predicates>(predicates), d_first,
		             early_stopping);
	}

	template <class Predicates, class OutputIt>
	OutputIt query(Point coord, depth_t depth, Predicates&& predicates, OutputIt d_first,
	               bool early_stopping = false) const
	{
		return query(toCode(coord, depth), std::forward<Predicates>(predicates), d_first,
		             early_stopping);
	}

	template <class Predicates, class OutputIt>
	OutputIt query(coord_t x, coord_t y, coord_t z, depth_t depth, Predicates&& predicates,
	               OutputIt d_first, bool early_stopping = false) const
	{
		return query(toCode(x, y, z, depth), std::forward<Predicates>(predicates), d_first,
		             early_stopping);
	}

	template <class Predicates, class OutputIt>
	OutputIt queryK(std::size_t k, Predicates&& predicates, OutputIt d_first,
	                bool early_stopping = false) const
	{
		std::size_t count = 0;
		if constexpr (std::is_same_v<typename OutputIt::value_type, Node>) {
			for (auto it = beginQuery(std::forward<Predicates>(predicates), early_stopping);
			     count < k && it != endQuery(); ++it, ++count) {
				*d_first++ = *it;
			}
		} else {
			for (auto it = beginQueryBV(std::forward<Predicates>(predicates), early_stopping);
			     count < k && it != endQueryBV(); ++it, ++count) {
				*d_first++ = *it;
			}
		}
		return d_first;
	}

	template <class Predicates, class OutputIt>
	OutputIt queryK(Node node, std::size_t k, Predicates&& predicates, OutputIt d_first,
	                bool early_stopping = false) const
	{
		std::size_t count = 0;
		if constexpr (std::is_same_v<typename OutputIt::value_type, Node>) {
			for (auto it =
			         beginQuery(node, std::forward<Predicates>(predicates), early_stopping);
			     count < k && it != endQuery(); ++it, ++count) {
				*d_first++ = *it;
			}
		} else {
			for (auto it =
			         beginQueryBV(node, std::forward<Predicates>(predicates), early_stopping);
			     count < k && it != endQueryBV(); ++it, ++count) {
				*d_first++ = *it;
			}
		}
		return d_first;
	}

	template <class Predicates, class OutputIt>
	OutputIt queryK(Code code, std::size_t k, Predicates&& predicates, OutputIt d_first,
	                bool early_stopping = false) const
	{
		std::size_t count = 0;
		if constexpr (std::is_same_v<typename OutputIt::value_type, Node>) {
			for (auto it =
			         beginQuery(code, std::forward<Predicates>(predicates), early_stopping);
			     count < k && it != endQuery(); ++it, ++count) {
				*d_first++ = *it;
			}
		} else {
			for (auto it =
			         beginQueryBV(code, std::forward<Predicates>(predicates), early_stopping);
			     count < k && it != endQueryBV(); ++it, ++count) {
				*d_first++ = *it;
			}
		}
		return d_first;
	}

	template <class Predicates, class OutputIt>
	OutputIt queryK(Key key, std::size_t k, Predicates&& predicates, OutputIt d_first,
	                bool early_stopping = false) const
	{
		return queryK(toCode(key), k, std::forward<Predicates>(predicates), d_first,
		              early_stopping);
	}

	template <class Predicates, class OutputIt>
	OutputIt queryK(Point coord, depth_t depth, std::size_t k, Predicates&& predicates,
	                OutputIt d_first, bool early_stopping = false) const
	{
		return queryK(toCode(coord, depth), k, std::forward<Predicates>(predicates), d_first,
		              early_stopping);
	}

	template <class Predicates, class OutputIt>
	OutputIt queryK(coord_t x, coord_t y, coord_t z, depth_t depth, std::size_t k,
	                Predicates&& predicates, OutputIt d_first,
	                bool early_stopping = false) const
	{
		return queryK(toCode(x, y, z, depth), k, std::forward<Predicates>(predicates),
		              d_first, early_stopping);
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearest(Geometry&& geometry, Predicates&& predicates, OutputIt d_first,
	                      double epsilon = 0.0, bool early_stopping = false) const
	{
		return std::copy(
		    beginQueryNearest(std::forward<Geometry>(geometry),
		                      std::forward<Predicates>(predicates), epsilon, early_stopping),
		    endQueryNearest(), d_first);
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearest(Node node, Geometry&& geometry, Predicates&& predicates,
	                      OutputIt d_first, double epsilon = 0.0,
	                      bool early_stopping = false) const
	{
		return std::copy(
		    beginQueryNearest(node, std::forward<Geometry>(geometry),
		                      std::forward<Predicates>(predicates), epsilon, early_stopping),
		    endQueryNearest(), d_first);
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearest(Code code, Geometry&& geometry, Predicates&& predicates,
	                      OutputIt d_first, double epsilon = 0.0,
	                      bool early_stopping = false) const
	{
		return std::copy(
		    beginQueryNearest(code, std::forward<Geometry>(geometry),
		                      std::forward<Predicates>(predicates), epsilon, early_stopping),
		    endQueryNearest(), d_first);
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearest(Key key, Geometry&& geometry, Predicates&& predicates,
	                      OutputIt d_first, double epsilon = 0.0,
	                      bool early_stopping = false) const
	{
		return queryNearest(toCode(key), std::forward<Geometry>(geometry),
		                    std::forward<Predicates>(predicates), d_first, epsilon,
		                    early_stopping);
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearest(Point coord, depth_t depth, Geometry&& geometry,
	                      Predicates&& predicates, OutputIt d_first, double epsilon = 0.0,
	                      bool early_stopping = false) const
	{
		return queryNearest(toCode(coord, depth), std::forward<Geometry>(geometry),
		                    std::forward<Predicates>(predicates), d_first, epsilon,
		                    early_stopping);
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearest(coord_t x, coord_t y, coord_t z, depth_t depth,
	                      Geometry&& geometry, Predicates&& predicates, OutputIt d_first,
	                      double epsilon = 0.0, bool early_stopping = false) const
	{
		return queryNearest(toCode(x, y, z, depth), std::forward<Geometry>(geometry),
		                    std::forward<Predicates>(predicates), d_first, epsilon,
		                    early_stopping);
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearestK(std::size_t k, Geometry&& geometry, Predicates&& predicates,
	                       OutputIt d_first, double epsilon = 0.0,
	                       bool early_stopping = false) const
	{
		std::size_t count = 0;
		for (auto it = beginQueryNearest(std::forward<Geometry>(geometry),
		                                 std::forward<Predicates>(predicates), epsilon,
		                                 early_stopping);
		     count < k && it != endQueryNearest(); ++it, ++count) {
			*d_first++ = *it;
		}
		return d_first;
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearestK(Node node, std::size_t k, Geometry&& geometry,
	                       Predicates&& predicates, OutputIt d_first, double epsilon = 0.0,
	                       bool early_stopping = false) const
	{
		std::size_t count = 0;
		for (auto it = beginQueryNearest(node, std::forward<Geometry>(geometry),
		                                 std::forward<Predicates>(predicates), epsilon,
		                                 early_stopping);
		     count < k && it != endQueryNearest(); ++it, ++count) {
			*d_first++ = *it;
		}
		return d_first;
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearestK(Code code, std::size_t k, Geometry&& geometry,
	                       Predicates&& predicates, OutputIt d_first, double epsilon = 0.0,
	                       bool early_stopping = false) const
	{
		std::size_t count = 0;
		for (auto it = beginQueryNearest(code, std::forward<Geometry>(geometry),
		                                 std::forward<Predicates>(predicates), epsilon,
		                                 early_stopping);
		     count < k && it != endQueryNearest(); ++it, ++count) {
			*d_first++ = *it;
		}
		return d_first;
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearestK(Key key, std::size_t k, Geometry&& geometry,
	                       Predicates&& predicates, OutputIt d_first, double epsilon = 0.0,
	                       bool early_stopping = false) const
	{
		return queryNearestK(toCode(key), k, std::forward<Geometry>(geometry),
		                     std::forward<Predicates>(predicates), d_first, epsilon,
		                     early_stopping);
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearestK(Point coord, depth_t depth, std::size_t k, Geometry&& geometry,
	                       Predicates&& predicates, OutputIt d_first, double epsilon = 0.0,
	                       bool early_stopping = false) const
	{
		return queryNearestK(toCode(coord, depth), k, std::forward<Geometry>(geometry),
		                     std::forward<Predicates>(predicates), d_first, epsilon,
		                     early_stopping);
	}

	template <class Geometry, class Predicates, class OutputIt>
	OutputIt queryNearestK(coord_t x, coord_t y, coord_t z, depth_t depth, std::size_t k,
	                       Geometry&& geometry, Predicates&& predicates, OutputIt d_first,
	                       double epsilon = 0.0, bool early_stopping = false) const
	{
		return queryNearestK(toCode(x, y, z, depth), k, std::forward<Geometry>(geometry),
		                     std::forward<Predicates>(predicates), d_first, epsilon,
		                     early_stopping);
	}

	//
	// Query iterator
	//

	template <class Predicates>
	[[nodiscard]] const_query_iterator beginQuery(Predicates&& predicates,
	                                              bool early_stopping = false) const
	{
		if (early_stopping) {
			if constexpr (predicate::contains_spatial_predicate_v<std::decay_t<Predicates>>) {
				return const_query_iterator(
				    new Iterator<Node, false, Derived, NodeBV, std::decay_t<Predicates>>(
				        &derived(), rootNodeBV(), std::forward<Predicates>(predicates)));
			} else {
				return const_query_iterator(
				    new Iterator<Node, false, Derived, Node, std::decay_t<Predicates>>(
				        &derived(), rootNode(), std::forward<Predicates>(predicates)));
			}
		} else {
			if constexpr (predicate::contains_spatial_predicate_v<std::decay_t<Predicates>>) {
				return const_query_iterator(
				    new Iterator<Node, true, Derived, NodeBV, std::decay_t<Predicates>>(
				        &derived(), rootNodeBV(), std::forward<Predicates>(predicates)));
			} else {
				return const_query_iterator(
				    new Iterator<Node, true, Derived, Node, std::decay_t<Predicates>>(
				        &derived(), rootNode(), std::forward<Predicates>(predicates)));
			}
		}
	}

	template <class Predicates>
	[[nodiscard]] const_query_iterator beginQuery(Node node, Predicates&& predicates,
	                                              bool early_stopping = false) const
	{
		if (early_stopping) {
			if constexpr (predicate::contains_spatial_predicate_v<std::decay_t<Predicates>>) {
				return const_query_iterator(
				    new Iterator<Node, false, Derived, NodeBV, std::decay_t<Predicates>>(
				        &derived(), toNodeBV(node), std::forward<Predicates>(predicates)));
			} else {
				return const_query_iterator(
				    new Iterator<Node, false, Derived, Node, std::decay_t<Predicates>>(
				        &derived(), node, std::forward<Predicates>(predicates)));
			}
		} else {
			if constexpr (predicate::contains_spatial_predicate_v<std::decay_t<Predicates>>) {
				return const_query_iterator(
				    new Iterator<Node, true, Derived, NodeBV, std::decay_t<Predicates>>(
				        &derived(), toNodeBV(node), std::forward<Predicates>(predicates)));
			} else {
				return const_query_iterator(
				    new Iterator<Node, true, Derived, Node, std::decay_t<Predicates>>(
				        &derived(), node, std::forward<Predicates>(predicates)));
			}
		}
	}

	template <class Predicates>
	[[nodiscard]] const_query_iterator beginQuery(Code code, Predicates&& predicates,
	                                              bool early_stopping = false) const
	{
		return beginQuery(operator()(code), std::forward<Predicates>(predicates),
		                  early_stopping);
	}

	template <class Predicates>
	[[nodiscard]] const_query_iterator beginQuery(Key key, Predicates&& predicates,
	                                              bool early_stopping = false) const
	{
		return beginQuery(toCode(key), std::forward<Predicates>(predicates), early_stopping);
	}

	template <class Predicates>
	[[nodiscard]] const_query_iterator beginQuery(Point coord, depth_t depth,
	                                              Predicates&& predicates,
	                                              bool early_stopping = false) const
	{
		return beginQuery(toCode(coord, depth), std::forward<Predicates>(predicates),
		                  early_stopping);
	}

	template <class Predicates>
	[[nodiscard]] const_query_iterator beginQuery(coord_t x, coord_t y, coord_t z,
	                                              depth_t depth, Predicates&& predicates,
	                                              bool early_stopping = false) const
	{
		return beginQuery(toCode(x, y, z, depth), std::forward<Predicates>(predicates),
		                  early_stopping);
	}

	[[nodiscard]] const_query_iterator endQuery() const
	{
		return const_query_iterator(new Iterator<Node, true, Derived>(&derived()));
	}

	template <class Predicates>
	[[nodiscard]] const_bounding_volume_query_iterator beginQueryBV(
	    Predicates&& predicates, bool early_stopping = false) const
	{
		if (early_stopping) {
			return const_bounding_volume_query_iterator(
			    new Iterator<NodeBV, false, Derived, NodeBV, Predicates>(
			        &derived(), rootNodeBV(), std::forward<Predicates>(predicates)));
		} else {
			return const_bounding_volume_query_iterator(
			    new Iterator<NodeBV, true, Derived, NodeBV, Predicates>(
			        &derived(), rootNodeBV(), std::forward<Predicates>(predicates)));
		}
	}

	template <class Predicates>
	[[nodiscard]] const_bounding_volume_query_iterator beginQueryBV(
	    Node node, Predicates&& predicates, bool early_stopping = false) const
	{
		if (early_stopping) {
			return const_bounding_volume_query_iterator(
			    new Iterator<NodeBV, false, Derived, NodeBV, Predicates>(
			        &derived(), toNodeBV(node), std::forward<Predicates>(predicates)));
		} else {
			return const_bounding_volume_query_iterator(
			    new Iterator<NodeBV, true, Derived, NodeBV, Predicates>(
			        &derived(), toNodeBV(node), std::forward<Predicates>(predicates)));
		}
	}

	template <class Predicates>
	[[nodiscard]] const_bounding_volume_query_iterator beginQueryBV(
	    Code code, Predicates&& predicates, bool early_stopping = false) const
	{
		return beginQueryBV(operator()(code), std::forward<Predicates>(predicates),
		                    early_stopping);
	}

	template <class Predicates>
	[[nodiscard]] const_bounding_volume_query_iterator beginQueryBV(
	    Key key, Predicates&& predicates, bool early_stopping = false) const
	{
		return beginQueryBV(toCode(key), std::forward<Predicates>(predicates),
		                    early_stopping);
	}

	template <class Predicates>
	[[nodiscard]] const_bounding_volume_query_iterator beginQueryBV(
	    Point coord, depth_t depth, Predicates&& predicates,
	    bool early_stopping = false) const
	{
		return beginQueryBV(toCode(coord, depth), std::forward<Predicates>(predicates),
		                    early_stopping);
	}

	template <class Predicates>
	[[nodiscard]] const_bounding_volume_query_iterator beginQueryBV(
	    coord_t x, coord_t y, coord_t z, depth_t depth, Predicates&& predicates,
	    bool early_stopping = false) const
	{
		return beginQueryBV(toCode(x, y, z, depth), std::forward<Predicates>(predicates),
		                    early_stopping);
	}

	[[nodiscard]] const_bounding_volume_query_iterator endQueryBV() const
	{
		return const_bounding_volume_query_iterator(
		    new Iterator<NodeBV, true, Derived, NodeBV>(&derived()));
	}

	template <class Geometry, class Predicates>
	[[nodiscard]] const_query_nearest_iterator beginQueryNearest(
	    Geometry&& geometry, Predicates&& predicates, double epsilon = 0.0,
	    bool early_stopping = false) const
	{
		if (early_stopping) {
			return const_query_nearest_iterator(
			    new NearestIterator<false, Derived, Geometry, Predicates>(
			        &derived(), rootNodeBV(), std::forward<Geometry>(geometry),
			        std::forward<Predicates>(predicates), epsilon));
		} else {
			return const_query_nearest_iterator(
			    new NearestIterator<true, Derived, Geometry, Predicates>(
			        &derived(), rootNodeBV(), std::forward<Geometry>(geometry),
			        std::forward<Predicates>(predicates), epsilon));
		}
	}

	template <class Geometry, class Predicates>
	[[nodiscard]] const_query_nearest_iterator beginQueryNearest(
	    Node node, Geometry&& geometry, Predicates&& predicates, double epsilon = 0.0,
	    bool early_stopping = false) const
	{
		if (early_stopping) {
			return const_query_nearest_iterator(
			    new NearestIterator<false, Derived, Geometry, Predicates>(
			        &derived(), toNodeBV(node), std::forward<Geometry>(geometry),
			        std::forward<Predicates>(predicates), epsilon));
		} else {
			return const_query_nearest_iterator(
			    new NearestIterator<true, Derived, Geometry, Predicates>(
			        &derived(), toNodeBV(node), std::forward<Geometry>(geometry),
			        std::forward<Predicates>(predicates), epsilon));
		}
	}

	template <class Geometry, class Predicates>
	[[nodiscard]] const_query_nearest_iterator beginQueryNearest(
	    Code code, Geometry&& geometry, Predicates&& predicates, double epsilon = 0.0,
	    bool early_stopping = false) const
	{
		return beginQueryNearest(operator()(code), std::forward<Geometry>(geometry),
		                         std::forward<Predicates>(predicates), epsilon,
		                         early_stopping);
	}

	template <class Geometry, class Predicates>
	[[nodiscard]] const_query_nearest_iterator beginQueryNearest(
	    Key key, Geometry&& geometry, Predicates&& predicates, double epsilon = 0.0,
	    bool early_stopping = false) const
	{
		return beginQueryNearest(toCode(key), std::forward<Geometry>(geometry),
		                         std::forward<Predicates>(predicates), epsilon,
		                         early_stopping);
	}

	template <class Geometry, class Predicates>
	[[nodiscard]] const_query_nearest_iterator beginQueryNearest(
	    Point coord, depth_t depth, Geometry&& geometry, Predicates&& predicates,
	    double epsilon = 0.0, bool early_stopping = false) const
	{
		return beginQueryNearest(toCode(coord, depth), std::forward<Geometry>(geometry),
		                         std::forward<Predicates>(predicates), epsilon,
		                         early_stopping);
	}

	template <class Geometry, class Predicates>
	[[nodiscard]] const_query_nearest_iterator beginQueryNearest(
	    coord_t x, coord_t y, coord_t z, depth_t depth, Geometry&& geometry,
	    Predicates&& predicates, double epsilon = 0.0, bool early_stopping = false) const
	{
		return beginQueryNearest(toCode(x, y, z, depth), std::forward<Geometry>(geometry),
		                         std::forward<Predicates>(predicates), epsilon,
		                         early_stopping);
	}

	[[nodiscard]] const_query_nearest_iterator endQueryNearest() const
	{
		return const_query_nearest_iterator(new NearestIterator<true, Derived>());
	}

	//
	// "Normal" iterator
	//

	[[nodiscard]] const_iterator begin(bool early_stopping = false) const
	{
		return beginQuery(predicate::Exists{}, early_stopping);
	}

	[[nodiscard]] const_iterator begin(Node node, bool early_stopping = false) const
	{
		return beginQuery(node, predicate::Exists{}, early_stopping);
	}

	[[nodiscard]] const_iterator begin(Code code, bool early_stopping = false) const
	{
		return beginQuery(code, predicate::Exists{}, early_stopping);
	}

	[[nodiscard]] const_iterator begin(Key key, bool early_stopping = false) const
	{
		return begin(toCode(key), early_stopping);
	}

	[[nodiscard]] const_iterator begin(Point coord, depth_t depth,
	                                   bool early_stopping = false) const
	{
		return begin(toCode(coord, depth), early_stopping);
	}

	[[nodiscard]] const_iterator begin(coord_t x, coord_t y, coord_t z, depth_t depth,
	                                   bool early_stopping = false) const
	{
		return begin(toCode(x, y, z, depth), early_stopping);
	}

	[[nodiscard]] const_iterator end() const { return endQuery(predicate::Exists{}); }

	[[nodiscard]] const_bounding_volume_iterator beginBV(bool early_stopping = false) const
	{
		return beginQueryBV(predicate::Exists{}, early_stopping);
	}

	[[nodiscard]] const_iterator beginBV(Node node, bool early_stopping = false) const
	{
		return beginQueryBV(node, predicate::Exists{}, early_stopping);
	}

	[[nodiscard]] const_iterator beginBV(Code code, bool early_stopping = false) const
	{
		return beginQueryBV(code, predicate::Exists{}, early_stopping);
	}

	[[nodiscard]] const_iterator beginBV(Key key, bool early_stopping = false) const
	{
		return beginBV(toCode(key), early_stopping);
	}

	[[nodiscard]] const_iterator beginBV(Point coord, depth_t depth,
	                                     bool early_stopping = false) const
	{
		return beginBV(toCode(coord, depth), early_stopping);
	}

	[[nodiscard]] const_iterator beginBV(coord_t x, coord_t y, coord_t z, depth_t depth,
	                                     bool early_stopping = false) const
	{
		return beginBV(toCode(x, y, z, depth), early_stopping);
	}

	[[nodiscard]] const_bounding_volume_iterator endBV() const
	{
		return endQueryBV(predicate::Exists{});
	}

	/**************************************************************************************
	|                                                                                     |
	|                                         I/O                                         |
	|                                                                                     |
	**************************************************************************************/

	void read(std::filesystem::path const& path, mt_t map_types = 0, bool propagate = true)
	{
		std::ifstream file;
		file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		file.imbue(std::locale());
		file.open(path, std::ios_base::in | std::ios_base::binary);

		read(file, map_types, propagate);
	}

	void read(std::istream& in, mt_t map_types = 0, bool propagate = true)
	{
		readData(in, readHeader(in), map_types, propagate);
	}

	void read(ReadBuffer& in, mt_t map_types = 0, bool propagate = true)
	{
		auto header = readHeader(in);
		readData(in, header, map_types, propagate);
	}

	void readData(std::istream& in, FileHeader const& header, mt_t map_types = 0,
	              bool propagate = true)
	{
		if (size() != header.leaf_size || depthLevels() != header.depth_levels) {
			clear(header.leaf_size, header.depth_levels);
		}

		auto nodes = readNodes(in);
		derived().readNodes(in, std::begin(nodes), std::end(nodes), header.compressed,
		                    map_types);

		if (propagate) {
			propagateModified();
		}
	}

	void readData(ReadBuffer& in, FileHeader const& header, mt_t map_types = 0,
	              bool propagate = true)
	{
		if (size() != header.leaf_size || depthLevels() != header.depth_levels) {
			clear(header.leaf_size, header.depth_levels);
		}

		auto nodes = readNodes(in);
		derived().readNodes(in, std::begin(nodes), std::end(nodes), false, header.compressed,
		                    map_types);

		if (propagate) {
			propagateModified();
		}
	}

	void write(std::filesystem::path const& path, depth_t min_depth = 0,
	           bool compress = false, mt_t map_types = 0,
	           int compression_acceleration_level = 1, int compression_level = 0) const
	{
		write(path, predicate::Exists(), min_depth, compress, map_types,
		      compression_acceleration_level, compression_level);
	}

	void write(std::ostream& out, depth_t min_depth = 0, bool compress = false,
	           mt_t map_types = 0, int compression_acceleration_level = 1,
	           int compression_level = 0) const
	{
		write(out, predicate::Exists(), min_depth, compress, map_types,
		      compression_acceleration_level, compression_level);
	}

	void write(WriteBuffer& out, depth_t min_depth = 0, bool compress = false,
	           mt_t map_types = 0, int compression_acceleration_level = 1,
	           int compression_level = 0) const
	{
		write(out, predicate::Exists(), min_depth, compress, map_types,
		      compression_acceleration_level, compression_level);
	}

	Buffer write(depth_t min_depth = 0, bool compress = false, mt_t map_types = 0,
	             int compression_acceleration_level = 1, int compression_level = 0) const
	{
		return write(predicate::Exists(), min_depth, compress, map_types,
		             compression_acceleration_level, compression_level);
	}

	template <class Predicates,
	          typename = std::enable_if_t<!std::is_scalar_v<std::decay_t<Predicates>>>>
	void write(std::filesystem::path const& path, Predicates&& predicates,
	           depth_t min_depth = 0, bool compress = false, mt_t map_types = 0,
	           int compression_acceleration_level = 1, int compression_level = 0) const
	{
		std::ofstream file;
		file.exceptions(std::ofstream::failbit | std::ofstream::badbit);
		file.imbue(std::locale());
		file.open(path, std::ios_base::out | std::ios_base::binary);

		write(file, std::forward<Predicates>(predicates), min_depth, compress, map_types,
		      compression_acceleration_level, compression_level);
	}

	template <class Predicates,
	          typename = std::enable_if_t<!std::is_scalar_v<std::decay_t<Predicates>>>>
	void write(std::ostream& out, Predicates&& predicates, depth_t min_depth = 0,
	           bool compress = false, mt_t map_types = 0,
	           int compression_acceleration_level = 1, int compression_level = 0) const
	{
		auto [tree_structure, nodes] =
		    data(predicate::Leaf() && predicate::DepthMin(min_depth) &&
		         std::forward<Predicates>(predicates));
		write(out, tree_structure, nodes, compress, map_types, compression_acceleration_level,
		      compression_level);
	}

	template <class Predicates,
	          typename = std::enable_if_t<!std::is_scalar_v<std::decay_t<Predicates>>>>
	void write(WriteBuffer& out, Predicates&& predicates, depth_t min_depth = 0,
	           bool compress = false, mt_t map_types = 0,
	           int compression_acceleration_level = 1, int compression_level = 0) const
	{
		auto [tree_structure, nodes] =
		    data(predicate::Leaf() && predicate::DepthMin(min_depth) &&
		         std::forward<Predicates>(predicates));
		write(out, tree_structure, nodes, compress, map_types, compression_acceleration_level,
		      compression_level);
	}

	template <class Predicates,
	          typename = std::enable_if_t<!std::is_scalar_v<std::decay_t<Predicates>>>>
	Buffer write(Predicates&& predicates, depth_t min_depth = 0, bool compress = false,
	             mt_t map_types = 0, int compression_acceleration_level = 1,
	             int compression_level = 0) const
	{
		Buffer buffer;
		write(buffer, predicates, min_depth, compress, map_types,
		      compression_acceleration_level, compression_level);
		return buffer;
	}

	void writeModifiedAndPropagate(std::filesystem::path const& filename,
	                               bool compress = false, mt_t map_types = 0,
	                               int compression_acceleration_level = 1,
	                               int compression_level = 0)
	{
		std::ofstream file;
		file.exceptions(std::ofstream::failbit | std::ofstream::badbit);
		file.imbue(std::locale());
		file.open(filename, std::ios_base::out | std::ios_base::binary);

		writeModifiedAndPropagate(file, compress, map_types, compression_acceleration_level,
		                          compression_level);
	}

	void writeModifiedAndPropagate(std::ostream& out, bool compress = false,
	                               mt_t map_types = 0,
	                               int compression_acceleration_level = 1,
	                               int compression_level = 0)
	{
		auto [modified_tree, modified_indices] = modifiedData<true>();
		write(out, modified_tree, modified_indices, compress, map_types,
		      compression_acceleration_level, compression_level);
	}

	void writeModifiedAndPropagate(WriteBuffer& out, bool compress = false,
	                               mt_t map_types = 0,
	                               int compression_acceleration_level = 1,
	                               int compression_level = 0)
	{
		auto [modified_tree, modified_indices] = modifiedData<true>();
		write(out, modified_tree, modified_indices, compress, map_types,
		      compression_acceleration_level, compression_level);
	}

	Buffer writeModifiedAndPropagate(bool compress = false, mt_t map_types = 0,
	                                 int compression_acceleration_level = 1,
	                                 int compression_level = 0)
	{
		Buffer buffer;
		writeModifiedAndPropagate(buffer, compress, map_types, compression_acceleration_level,
		                          compression_level);
		return buffer;
	}

	void writeModifiedAndReset(std::filesystem::path const& filename, bool compress = false,
	                           mt_t map_types = 0, int compression_acceleration_level = 1,
	                           int compression_level = 0)
	{
		std::ofstream file;
		file.exceptions(std::ofstream::failbit | std::ofstream::badbit);
		file.imbue(std::locale());
		file.open(filename, std::ios_base::out | std::ios_base::binary);

		writeModifiedAndReset(file, compress, map_types, compression_acceleration_level,
		                      compression_level);
	}

	void writeModifiedAndReset(std::ostream& out, bool compress = false, mt_t map_types = 0,
	                           int compression_acceleration_level = 1,
	                           int compression_level = 0)
	{
		auto [modified_tree, modified_indices] = modifiedData<false>();
		write(out, modified_tree, modified_indices, compress, map_types,
		      compression_acceleration_level, compression_level);
	}

	void writeModifiedAndReset(WriteBuffer& out, bool compress = false, mt_t map_types = 0,
	                           int compression_acceleration_level = 1,
	                           int compression_level = 0)
	{
		auto [modified_tree, modified_indices] = modifiedData<false>();
		write(out, modified_tree, modified_indices, compress, map_types,
		      compression_acceleration_level, compression_level);
	}

	Buffer writeModifiedAndReset(bool compress = false, mt_t map_types = 0,
	                             int compression_acceleration_level = 1,
	                             int compression_level = 0)
	{
		Buffer buffer;
		writeModifiedAndReset(buffer, compress, map_types, compression_acceleration_level,
		                      compression_level);
		return buffer;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Statistics                                      |
	|                                                                                     |
	**************************************************************************************/

	/*!
	 * @return Number of nodes in the octree.
	 */
	[[nodiscard]] std::size_t numNodes() const
	{
		return 8 * (indices_.size() - free_indices_.size());
	}

	/*!
	 * @brief This is lower bound memory usage of a node.
	 *
	 * @note Additional data accessed by pointers/references inside a node are not counted.
	 *
	 * @return Memory usage of a single node.
	 */
	[[nodiscard]] static constexpr std::size_t memoryNode() const
	{
		return derived().memoryNodeBlock() / 8;
	}

	/*!
	 * @brief Lower bound memory usage for all nodes.
	 *
	 * @note Does not account for pointed to data inside nodes.
	 *
	 * @return Memory usage of the octree.
	 */
	[[nodiscard]] std::size_t memoryUsage() const { return numNodes() * memoryNode(); }

	/*!
	 * @return Number of allocated nodes.
	 */
	[[nodiscard]] std::size_t numNodesAllocated() const { return 8 * indices_.size(); }

	/*!
	 * @brief Lower bound memory usage for all allocated nodes.
	 *
	 * @note Does not account for pointed to data inside nodes.
	 *
	 * @return Memory usage of the allocated octree.
	 */
	[[nodiscard]] std::size_t memoryUsageAllocated() const
	{
		return numNodesAllocated() * memoryNode();
	}

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	OctreeBase(node_size_t leaf_node_size = 0.1, depth_t depth_levels = 16)

	{
		setNodeSizeAndDepthLevels(leaf_node_size, depth_levels);

		init();
	}

	OctreeBase(OctreeBase const& other)
	    : depth_levels_(other.depth_levels_),
	      max_value_(other.max_value_),
	      node_size_(other.node_size_),
	      node_size_factor_(other.node_size_factor_)
	{
		init();
	}

	OctreeBase(OctreeBase&& other)
	    : depth_levels_(std::move(other.depth_levels_)),
	      max_value_(std::move(other.max_value_)),
	      indices_(std::move(other.indices_)),
	      free_indices_(std::move(other.free_indices_)),
	      codes_(std::move(other.codes_)),
	      modified_(std::move(other.modified_)),
	      node_size_(std::move(other.node_size_)),
	      node_size_factor_(std::move(other.node_size_factor_))
	{
		init();
	}

	template <class Derived2>
	OctreeBase(OctreeBase<Derived2, Data2, InnerData2> const& other)
	    : depth_levels_(other.depth_levels_),
	      max_value_(other.max_value_),
	      indices_(other.indices_),
	      free_indices_(other.free_indices_),
	      codes_(other.codes_),
	      modified_(other.modified_),
	      node_size_(other.node_size_),
	      node_size_factor_(other.node_size_factor_)
	{
		init();
	}

	//
	// Init
	//

	void init() { inner_node_.emplace_back(); }

	//
	// Destructor
	//

	~OctreeBase() {}

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	OctreeBase& operator=(OctreeBase const& rhs)
	{
		// TODO: Should this clear?
		clear(rhs.size(), rhs.depthLevels());

		depth_levels_ = rhs.depth_levels_;
		max_value_ = rhs.max_value_;
		node_size_ = rhs.node_size_;
		node_size_factor_ = rhs.node_size_factor_;
		return *this;
	}

	OctreeBase& operator=(OctreeBase&& rhs)
	{
		// TODO: Should this clear?
		clear(rhs.size(), rhs.depthLevels(), true);

		depth_levels_ = std::move(rhs.depth_levels_);
		max_value_ = std::move(rhs.max_value_);
		indices_ = std::move(rhs.indices_);
		free_indices_ = std::move(rhs.free_indices_);
		codes_ = std::move(rhs.codes_);
		modified_ = std::move(rhs.modified_);
		node_size_ = std::move(rhs.node_size_);
		node_size_factor_ = std::move(rhs.node_size_factor_);
		return *this;
	}

	template <class Derived2>
	OctreeBase& operator=(OctreeBase<Derived2> const& rhs)
	{
		// TODO: Should this clear?
		clear(rhs.size(), rhs.depthLevels());

		depth_levels_ = rhs.depth_levels_;
		max_value_ = rhs.max_value_;
		indices_ = rhs.indices_;
		free_indices_ = rhs.free_indices_;
		codes_ = rhs.codes_;
		modified_ = rhs.modified_;
		node_size_ = rhs.node_size_;
		node_size_factor_ = rhs.node_size_factor_;
		return *this;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                         Swap                                        |
	|                                                                                     |
	**************************************************************************************/

	void swap(OctreeBase& other)
	{
		std::swap(depth_levels_, other.depth_levels_);
		std::swap(max_value_, other.max_value_);
		std::swap(indices_, rhs.indices_);
		std::swap(free_indices_, rhs.free_indices_);
		std::swap(codes_, rhs.codes_);
		std::swap(modified_, rhs.modified_);
		std::swap(node_size_, other.node_size_);
		std::swap(node_size_factor_, other.node_size_factor_);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Derived                                       |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	[[nodiscard]] constexpr Derived& derived() { return *static_cast<Derived*>(this); }

	[[nodiscard]] constexpr Derived const& derived() const
	{
		return *static_cast<Derived const*>(this);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Octree                                        |
	|                                                                                     |
	**************************************************************************************/

	void setNodeSizeAndDepthLevels(node_size_t const leaf_node_size,
	                               depth_t const depth_levels)
	{
		if (minDepthLevels() > depth_levels || maxDepthLevels() < depth_levels) {
			throw std::invalid_argument("depth_levels have to be in range [" +
			                            std::to_string(+minDepthLevels()) + ".." +
			                            std::to_string(+maxDepthLevels()) + "], '" +
			                            std::to_string(+depth_levels) + "' was supplied.");
		}

		depth_levels_ = depth_levels;
		max_value_ = std::pow(2, depth_levels - 2);

		// For increased precision
		node_size_[0] = leaf_node_size;
		int const s = node_size_.size();
		for (int i = 1; s != i; ++i) {
			node_size_[i] = std::ldexp(leaf_node_size, i);
		}

		std::transform(std::cbegin(node_size_), std::cend(node_size_),
		               std::begin(node_size_factor_), [](auto const n) { return 1.0 / n; });
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Root                                         |
	|                                                                                     |
	**************************************************************************************/

	/*!
	 * @brief Initilize the root node.
	 */
	void initRoot()
	{
		modified_[rootIndex().index].reset();
		codes_[rootIndex().index] = rootCode().parent();  // TODO: Not really correct...
	}

	/*!
	 * @brief Get the root node index.
	 *
	 * @return The root node index.
	 */
	[[nodiscard]] constexpr index_t rootIndex() const noexcept { return {0, 0}; }

	/*!
	 * @brief Get the root node's index field.
	 *
	 * @return The root node's index field.
	 */
	[[nodiscard]] constexpr IndexField rootIndexField() const noexcept
	{
		return IndexField(1);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        TODO                                         |
	|                                                                                     |
	**************************************************************************************/

	void fill(index_t index, Index parent_index)
	{
		codes_[index] = codes_[parent_index.index].child(parent_index.offset);
		if (isModified(parent_index)) {
			modified_[index].set();
		} else {
			modified_[index].reset();
		}
		derived().fill(index, parent_index);
	}

	void clear(index_t index)
	{
		// TODO: Implement
		derived().clear(index);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Apply                                        |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	template <class UnaryFunction, class UnaryFunction2,
	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction> &&
	                                      std::is_copy_constructible_v<UnaryFunction2>>>
	void applyUnsafe(Index idx, UnaryFunction f, UnaryFunction2 f2)
	{
		if (isLeaf(idx)) {
			f(idx);
		} else {
			applyUnsafeRecurs(idx, f, f2);
		}
	}

	template <class UnaryFunction, class UnaryFunction2,
	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction> &&
	                                      std::is_copy_constructible_v<UnaryFunction2>>>
	void applyUnsafeRecurs(Index idx, UnaryFunction f, UnaryFunction2 f2)
	{
		if (allLeaf(child(idx))) {
			f2(child(idx));
		} else {
			// FIXME: Compare iterative and recursive version

			// // Recursive
			// idx.index = child(idx);
			// for (offset_t i{}; 8 != i; ++i) {
			// 	idx.offset = i;
			// 	if (isLeaf(idx)) {
			// 		f(idx);
			// 	} else {
			// 		applyRecurs(idx, f, f2);
			// 	}
			// }

			// Iterative
			std::array<Index, maxDepthLevels()> indices;
			indices[0] = child(idx, 0);
			for (int depth{}; 0 <= depth;) {
				idx = indices[depth];
				depth -= 7 < ++indices[depth].offset;
				if (isLeaf(idx)) {
					f(idx);
				} else if (allLeaf(child(idx))) {
					f2(child(idx));
				} else {
					indices[++depth] = child(idx, 0);
				}
			}
		}
	}

	template <class UnaryFunction, class UnaryFunction2,
	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction> &&
	                                      std::is_copy_constructible_v<UnaryFunction2>>>
	void apply(Index idx, UnaryFunction f, UnaryFunction2 f2, bool propagate)
	{
		setModified(idx);

		if (isLeaf(idx)) {
			f(idx);
		} else {
			applyRecurs(idx, f, f2);
		}

		if (propagate) {
			propagateModified();
		}
	}

	template <class UnaryFunction, class UnaryFunction2,
	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction> &&
	                                      std::is_copy_constructible_v<UnaryFunction2>>>
	void applyRecurs(Index idx, UnaryFunction f, UnaryFunction2 f2)
	{
		setModified(child(idx));
		if (allLeaf(child(idx))) {
			f2(child(idx));
		} else {
			// FIXME: Compare iterative and recursive version

			// // Recursive
			// idx.index = child(idx);
			// for (offset_t i{}; 8 != i; ++i) {
			// 	idx.offset = i;
			// 	if (isLeaf(idx)) {
			// 		f(idx);
			// 	} else {
			// 		applyRecurs(idx, f, f2);
			// 	}
			// }

			// Iterative
			std::array<Index, maxDepthLevels()> indices;
			indices[0] = child(idx, 0);
			for (int depth{}; 0 <= depth;) {
				idx = indices[depth];
				depth -= 7 < ++indices[depth].offset;
				if (isLeaf(idx)) {
					f(idx);
				} else {
					setModified(child(idx));
					if (allLeaf(child(idx))) {
						f2(child(idx));
					} else {
						indices[++depth] = child(idx, 0);
					}
				}
			}
		}
	}

	template <class UnaryFunction, class UnaryFunction2,
	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction> &&
	                                      std::is_copy_constructible_v<UnaryFunction2>>>
	Node apply(Node node, UnaryFunction f, UnaryFunction2 f2, bool propagate)
	{
		Index idx = create(node);
		apply(idx, f, f2, propagate);
		return {idx.index, node.code()};
	}

	template <class UnaryFunction, class UnaryFunction2,
	          typename = std::enable_if_t<std::is_copy_constructible_v<UnaryFunction> &&
	                                      std::is_copy_constructible_v<UnaryFunction2>>>
	Node apply(Code code, UnaryFunction f, UnaryFunction2 f2, bool propagate)
	{
		Index idx = create(code);
		apply(idx, f, f2, propagate);
		return {idx.index, code};
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Traverse                                       |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comment

	template <class NodeType, class UnaryFunction>
	void traverseRecurs(NodeType const& node, UnaryFunction f) const
	{
		if (f(node) || isLeaf(node)) {
			return;
		}

		// FIXME: Make iterative for performance?

		node = child(node, 0);
		for (index_t index = 0; 8 != index; ++index) {
			traverseRecurs(sibling(node, index), f);
		}
	}

	template <class Geometry, class UnaryFunction>
	void traverseNearestRecurs(NodeBV const& node, Geometry const& g, UnaryFunction f) const
	{
		// TODO: Implement
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Conversion                                      |
	|                                                                                     |
	**************************************************************************************/

	//
	// Key
	//

	/*!
	 * @brief Convert a coordinate at a specific depth to a key.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The corresponding key.
	 */
	[[nodiscard]] constexpr key_t toKey(coord_t coord, depth_t depth = 0) const
	{
		// FIXME: Make it possible to make a cloud of coordinates
		key_t val = std::floor(node_size_factor_[0] * coord);
		return ((val + max_value_) >> depth) << depth;
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a key with bounds check.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The corresponding key.
	 */
	[[nodiscard]] constexpr std::optional<key_t> toKeyChecked(coord_t coord,
	                                                          depth_t depth = 0) const
	{
		auto min = -size(rootDepth() - 1);
		auto max = -min;
		return min <= coord && max >= coord ? std::optional<key_t>(toKey(coord, depth))
		                                    : std::nullopt;
	}

	//
	// Coordinate
	//

	/*!
	 * @brief Convert a key to a coordinate at a specific depth.
	 *
	 * @param key The key.
	 * @param depth The depth of the coordinate.
	 * @return The corresponding coordinate.
	 */
	[[nodiscard]] constexpr coord_t toCoord(key_t key, depth_t depth = 0) const
	{
		constexpr auto sub64 = std::minus<int_fast64_t>{};
		return rootDepth() == depth
		           ? 0
		           : (std::floor(sub64(key, max_value_) / static_cast<coord_t>(1U << depth)) +
		              coord_t(0.5)) *
		                 size(depth);
		//  FIXME: Look at
		// return rootDepth() == depth
		//            ? 0
		//            : ((sub64(key, max_value_) >> depth) + coord_t(0.5)) * size(depth);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                    Access nodes                                     |
	|                                                                                     |
	**************************************************************************************/

	//
	// Correct
	//

	[[nodiscard]] bool isCorrect(Node node) const
	{
		return codes_[node.data()] == node.code().parent();
	}

	//
	// Valid
	//

	[[nodiscard]] bool isValid(Node node) const
	{
		Code rel_code = codes_[node.data()];
		Code code = node.code();

		depth_t rel_depth = rel_code.depth();
		depth_t depth = node.depth() + 1;

		return rel_depth >= depth && rel_code == code.toDepth(rel_depth);
	}

	//
	// Create
	//

	Index create(Node node)
	{
		if (!isCorrect(node)) {
			return create(node.code());
		}

		Index idx = node.index();
		if (noneModified(idx)) {
			setParentsModified(node.code());
		}
		return idx;
	}

	Index create(Code code)
	{
		Index idx = rootIndex();
		for (depth_t d = rootDepth(), min_d = code.depth() + 1; min_d < d;) {
			createChildren(idx);
			setModified(idx);
			idx = child(idx, code.offset(--d));
		}
		return idx;
	}

	//
	// Get node
	//

	[[nodiscard]] Index toIndex(Node node) const
	{
		if (!isValid(node)) {
			return toIndex(node.code());
		} else if (isCorrect(node) || isLeaf(node)) {
			return {node.index(), node.offset()};
		} else {
			// TODO: Correct?
			return toIndex(node.code(), node.index(), codes_[node.index()].depth());
		}
	}

	[[nodiscard]] Index toIndex(Code code) const
	{
		return toIndex(code, rootIndex(), rootDepth());
	}

	[[nodiscard]] Index toIndex(Code code, index_t index, depth_t depth) const
	{
		offset_t offset = code.index(depth);
		depth_t min_depth = code.depth();
		index_t child_index = child(index, offset);
		while (min_depth < depth && NULL_INDEX != child_index) {
			index = child_index;
			--depth;
			offset = code.index(depth);
			child_index = child(index, offset);
		}
		return {index, offset};
	}

	//
	// Get children
	//

	[[nodiscard]] index_t child(Index index) const
	{
		return indices_[index.index][index.offset];
	}

	[[nodiscard]] Index child(Index index, offset_t offset) const
	{
		return {child(index), offset};
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Center                                        |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	//
	// Child center
	//

	[[nodiscard]] static constexpr Point childCenter(Point parent_center,
	                                                 node_size_t child_half_size,
	                                                 index_t child_index)
	{
		parent_center[0] += child_index & index_t(1) ? child_half_size : -child_half_size;
		parent_center[1] += child_index & index_t(2) ? child_half_size : -child_half_size;
		parent_center[2] += child_index & index_t(4) ? child_half_size : -child_half_size;
		return parent_center;
	}

	//
	// Sibling center
	//

	[[nodiscard]] static constexpr Point siblingCenter(Point center, node_size_t half_size,
	                                                   index_t index, index_t sibling_index)
	{
		index_t const temp = index ^ sibling_index;
		node_size_t const size = 2 * half_size;
		if (temp & index_t(1)) {
			center[0] += sibling_index & index_t(1) ? size : -size;
		}
		if (temp & index_t(2)) {
			center[1] += sibling_index & index_t(2) ? size : -size;
		}
		if (temp & index_t(4)) {
			center[2] += sibling_index & index_t(4) ? size : -size;
		}
		return center;
	}

	//
	// Parent center
	//

	[[nodiscard]] static constexpr Point parentCenter(Point child_center,
	                                                  node_size_t child_half_size,
	                                                  index_t child_index)
	{
		child_center[0] -= child_index & index_t(1) ? child_half_size : -child_half_size;
		child_center[1] -= child_index & index_t(2) ? child_half_size : -child_half_size;
		child_center[2] -= child_index & index_t(4) ? child_half_size : -child_half_size;
		return child_center;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Leaf                                         |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Implement

	/**************************************************************************************
	|                                                                                     |
	|                                      Modified                                       |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Implement

	//
	// Set modified
	//

	void setModified(Index idx) { modified_[idx.index][idx.offset] = true; }

	void setModified(index_t index) { modified_[index].set(); }

	void setModified(Index idx, depth_t depth, depth_t min_depth)
	{
		modified_[idx.index][idx.offset] = min_depth <= depth;

		if (min_depth < depth && !isLeaf(idx)) {
			setModifiedRecurs(idx, depth - 1, min_depth);
		}
	}

	void setModifiedRecurs(Index idx, depth_t depth, depth_t min_depth)
	{
		index_t children_index = children(idx);
		modified_[children_index].set();

		if (min_depth < depth && !allLeaf(children_index)) {
			for (offset_t i = 0; 8 != i; ++i) {
				Index child_idx{children_index, i};
				if (!isLeaf(child_idx)) {
					setModifiedRecurs(child_idx, depth - 1, min_depth);
				}
			}
		}
	}

	//
	// Reset modified
	//

	void resetModified(Index idx) { modified_[idx.index][idx.offset] = false; }

	void resetModified(index_t index) { modified_[index].reset(); }

	void resetModified(Index index, depth_t depth, depth_t max_depth)
	{
		if (!isModified(index)) {
			return;
		}

		if (!isLeaf(index)) {
			resetModifiedRecurs(child(index), depth - 1, max_depth);
		}

		modified_[index][offset] = max_depth < depth || anyModified(child(index));
	}

	void resetModifiedRecurs(index_t index, depth_t depth, depth_t max_depth)
	{
		for (offset_t i = 0; 8 != i; ++i) {
			if (isModified(index, i) && !isLeaf(index, i)) {
				resetModifiedRecurs(child(index, i), depth - 1, max_depth);
			}
		}

		if (max_depth >= depth) {
			modified_[index].reset();
		}
	}

	//
	// Set parents modified
	//

	void setParentsModified(Code code)
	{
		setParentsModified(rootIndex(), rootDepth(), code);
	}

	void setParentsModified(Index idx, depth_t d, Code c)
	{
		for (depth_t min_d = c.depth() + 1; min_d < d && !isLeaf(idx);) {
			setModified(idx);
			idx = child(idx, code.offset(--d))
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Propagate                                      |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	void updateNode(Index index)
	{
		derived().updateNode(index, child(index));
		prune(index);  // TODO: Should this be here?
	}

	void propagateModified(Index index, bool keep_modified)
	{
		if (!isModified(index)) {
			return;
		}

		if (!isLeaf(index)) {
			if (keep_modified) {
				propagateModifiedRecurs<true>(child(index));
			} else {
				propagateModifiedRecurs<false>(child(index));
			}
			updateNode(index);
		}

		modified_[index][offset] = keep_modified;
	}

	template <bool KeepModified>
	void propagateModifiedRecurs(index_t index)
	{
		for (offset_t i = 0; 8 != i; ++i) {
			if (!isModified(index, i)) {
				continue;
			}

			if (!isLeaf(index, i)) {
				propagateModifiedRecurs<KeepModified>(child(index, i));
				updateNode(index, i);
			}

			if constexpr (!KeepModified) {
				modified_[index].reset();
			}
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Prune                                        |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	[[nodiscard]] bool isPrunable(Index index)
	{
		return isLeaf(index) || derived().isPrunable(child(index));
	}

	bool prune(Index index)
	{
		return isPrunable(index) ? deleteChildren(index), true : false;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                 Create/delete nodes                                 |
	|                                                                                     |
	**************************************************************************************/

	//
	// Create
	//

	index_t allocateNodeBlock()
	{
		index_t index = indices_.size();
		indices_.emplace_back();
		codes_.emplace_back();
		modified_.emplace_back();
		derived().allocateNodeBlock();
		return index;
	}

	void allocateChildren(Index idx)
	{
		if (!isLeaf(idx)) {
			return;
		}

		if (free_indices_.empty()) {
			indices_[idx.index][idx.offset] = allocateNodeBlock();
			indices_.back().fill(NULL_INDEX);
		} else {
			indices_[idx.index][idx.offset] = free_indices_.top();
			free_indices_.pop();
		}

		fill(child(idx), idx);
	}

	void createChildren(index_t index) { createChildren(index, ~IndexField()); }

	void createChildren(index_t index, IndexField const offsets)
	{
		for (offset_t i = 0; 8 != i; ++i) {
			if (offsets[i]) {
				allocateChildren(Index{index, i});
			}
		}
	}

	void createChildren(Index index) { allocateChildren(index); }

	//
	// Delete
	//

	void deallocateChildren(Index index)
	{
		index_t i = child(index);
		free_indices_.push(i);
		codes_[i] = Code();  // TODO: What should this be?
		indices_[index.index][index.offset] = NULL_INDEX;
	}

	void deleteChildren(index_t index) { deleteChildren(index, ~IndexField()); }

	void deleteChildren(index_t index, IndexField const offsets)
	{
		for (offset_t i = 0; 8 != i; ++i) {
			if (!isLeaf(index, i) && offsets[i]) {
				index_t child_index = child(index, i);
				deleteChildren(child_index);
				clear(child_index);
				deallocateChildren(index, i);
			}
		}
	}

	void deleteChildren(Index index)
	{
		if (isLeaf(index)) {
			return;
		}

		index_t child_index = child(index);
		deleteChildren(child_index);
		clear(child_index);
		deallocateChildren(index);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                         I/O                                         |
	|                                                                                     |
	**************************************************************************************/

	struct IndexAndOffsets {
		index_t index;
		IndexField offsets;

		IndexAndOffsets() = default;

		IndexAndOffsets(index_t index, IndexField offsets) : index(index), offsets(offsets) {}

		IndexAndOffsets(IndexAndOffsets const&) = default;

		IndexAndOffsets(IndexAndOffsets&&) = default;

		IndexAndOffsets& operator=(IndexAndOffsets const&) = default;

		IndexAndOffsets& operator=(IndexAndOffsets&&) = default;
	};

	[[nodiscard]] FileOptions fileOptions(bool const compress) const
	{
		FileOptions options;
		options.compressed = compress;
		options.leaf_size = size();
		options.depth_levels = depthLevels();
		return options;
	}

	[[nodiscard]] std::vector<IndexAndOffsets> readNodes(std::istream& in)
	{
		auto tree = readTreeStructure(in);
		auto num_nodes = readNum(in);
		return retrieveNodes(tree, num_nodes);
	}

	[[nodiscard]] std::vector<IndexAndOffsets> readNodes(ReadBuffer& in)
	{
		auto tree = readTreeStructure(in);
		auto num_nodes = readNum(in);
		return retrieveNodes(tree, num_nodes);
	}

	[[nodiscard]] std::unique_ptr<IndexField[]> readTreeStructure(std::istream& in)
	{
		std::uint64_t num = readNum(in);
		auto tree = std::make_unique<IndexField[]>(num);
		in.read(reinterpret_cast<char*>(tree.get()),
		        num * sizeof(typename decltype(tree)::element_type));
		return tree;
	}

	[[nodiscard]] std::unique_ptr<IndexField[]> readTreeStructure(ReadBuffer& in)
	{
		std::uint64_t num = readNum(in);
		auto tree = std::make_unique<IndexField[]>(num);
		in.read(tree.get(), num * sizeof(typename decltype(tree)::element_type));
		return tree;
	}

	[[nodiscard]] std::uint64_t readNum(std::istream& in)
	{
		std::uint64_t num;
		in.read(reinterpret_cast<char*>(&num), sizeof(num));
		return num;
	}

	[[nodiscard]] std::uint64_t readNum(ReadBuffer& in)
	{
		std::uint64_t num;
		in.read(&num, sizeof(num));
		return num;
	}

	[[nodiscard]] std::vector<IndexAndOffsets> retrieveNodes(
	    std::unique_ptr<IndexField[]> const& tree, std::uint64_t num_nodes)
	{
		std::vector<IndexAndOffsets> nodes;
		nodes.reserve(num_nodes);
		retrieveNodesRecurs(rootIndex(), tree.get(), nodes);
		return nodes;
	}

	void retriveNodesRecurs(index_t index, IndexField const*& tree,
	                        std::vector<IndexAndOffsets>& nodes)
	{
		IndexField const valid_return = *tree++;
		IndexField const valid_inner = *tree++;

		if (valid_return.any()) {
			nodes.emplace_back(index, valid_return);
		}

		if (valid_inner.any()) {
			createChildren(index, valid_inner);
			for (offset_t i = 0; 8 != i; ++i) {
				if (valid_inner[i]) {
					retriveNodesRecurs(child(index, i), tree, nodes);
				}
			}
		}

		modified_[index] |= valid_return | valid_inner;
	}

	template <class Predicates>
	[[nodiscard]] std::pair<std::vector<IndexField>, std::vector<index_t>> data(
	    Predicates const& predicates) const
	{
		std::vector<IndexField> tree;
		std::vector<index_t> indices;

		std::conditional_t<predicate::contains_spatial_predicate_v<Predicates>, NodeBV, Node>
		    root_node;
		if constexpr (predicate::contains_spatial_predicate_v<Predicates>) {
			root_node = rootNodeBV();
		} else {
			root_node = rootNode();
		}

		bool valid_return = predicate::PredicateValueCheck<Predicates>::apply(
		    predicates, derived(), root_node);
		bool valid_inner = !valid_return && predicate::PredicateInnerCheck<Predicates>::apply(
		                                        predicates, derived(), root_node);

		tree.emplace_back(valid_return ? 1U : 0U);
		tree.emplace_back(valid_inner ? 1U : 0U);

		if (valid_return) {
			indices.push_back(rootIndex());
		} else if (valid_inner) {
			dataRecurs(child(root_node, 0), predicates, tree, indices);
			if (indices.empty()) {
				tree.clear();
			}
		}

		return {std::move(tree), std::move(indices)};
	}

	template <class Predicates, class NodeType>
	void dataRecurs(NodeType const& node, Predicates const& predicates,
	                std::vector<IndexField>& tree, std::vector<index_t>& indices) const
	{
		IndexField valid_return;
		IndexField valid_inner;
		for (offset_t i = 0; 8 != i; ++i) {
			auto s = sibling(node, i);

			if (predicate::PredicateValueCheck<Predicates>::apply(predicates, derived(), s)) {
				valid_return[i] = true;
			} else if (predicate::PredicateInnerCheck<Predicates>::apply(predicates, derived(),
			                                                             s)) {
				valid_inner[i] = true;
			}
		}

		tree.push_back(valid_return);
		tree.push_back(valid_inner);

		auto cur_tree_size = tree.size();
		auto cur_indices_size = indices.size();

		if (valid_return.any()) {
			indices.push_back(node.data());
		}

		if (valid_inner.any()) {
			for (offset_t i = 0; 8 != i; ++i) {
				if (valid_inner[i]) {
					auto s = sibling(node, i);
					dataRecurs(child(s, 0), predicates, tree, indices);
				}
			}
		}

		if (indices.size() == cur_indices_size) {
			tree.resize(cur_tree_size);
			tree[tree.size() - 1] = 0;
			tree[tree.size() - 2] = 0;
		}
	}

	template <bool Propagate>
	[[nodiscard]] std::pair<std::vector<IndexField>, std::vector<index_t>> modifiedData()
	{
		std::vector<IndexField> modified_tree;
		std::vector<index_t> modified_indices;

		modified_tree.reserve(modified_tree_size_);
		modified_indices.reserve(modified_indices_size_);

		modifiedDataRecurs(rootIndex(), modified_tree, modified_indices);

		if (modified_indices.empty()) {
			modified_tree.clear();
		}

		modified_tree_size_ = std::max(modified_tree_size_, modified_tree.size());
		modified_indices_size_ = std::max(modified_indices_size_, modified_indices.size());

		return {std::move(modified_tree), std::move(modified_indices)};
	}

	template <bool Propagate>
	void modifiedDataRecurs(index_t index, std::vector<IndexField>& modified_tree,
	                        std::vector<index_t>& modified_indices)
	{
		IndexField const valid_return = leaves(index) & modified_[index];
		IndexField const valid_inner = ~leaves(index) & modified_[index];

		modified_tree.push_back(valid_return);
		modified_tree.push_back(valid_inner);

		auto cur_tree_size = modified_tree.size();
		auto cur_indices_size = modified_indices.size();

		if (valid_return.any()) {
			modified_indices.push_back(index);
		}

		if (valid_inner.any()) {
			for (offset_t i = 0; 8 != i; ++i) {
				if (valid_inner[i]) {
					modifiedDataRecurs(child(index, i));
				}
			}
		}

		if constexpr (Propagate) {
			propagate(index, valid_inner);
		}

		modified_[index].reset();

		if (modified_indices.size() == cur_indices_size) {
			modified_tree.resize(cur_tree_size);
			modified_tree[modified_tree.size() - 1] = 0;
			modified_tree[modified_tree.size() - 2] = 0;
		}
	}

	void write(std::ostream& out, std::vector<IndexField> const& tree,
	           std::vector<index_t> const& nodes, bool compress, mt_t map_types,
	           int compression_acceleration_level, int compression_level) const
	{
		writeHeader(out, fileOptions(compress));
		writeTreeStructure(out, tree);
		writeNumNodes(out, nodes.size());
		writeNodes(out, std::cbegin(nodes), std::cend(nodes), compress, map_types,
		           compression_acceleration_level, compression_level);
	}

	void write(WriteBuffer& out, std::vector<IndexField> const& tree,
	           std::vector<index_t> const& nodes, bool compress, mt_t map_types,
	           int compression_acceleration_level, int compression_level) const
	{
		writeHeader(out, fileOptions(compress));
		writeTreeStructure(out, tree);
		writeNumNodes(out, nodes.size());
		writeNodes(out, std::cbegin(nodes), std::cend(nodes), compress, map_types,
		           compression_acceleration_level, compression_level);
	}

	void writeTreeStructure(std::ostream& out, std::vector<IndexField> const& tree) const
	{
		std::uint64_t num = tree.size();
		out.write(reinterpret_cast<char const*>(&num), sizeof(num));
		out.write(reinterpret_cast<char const*>(tree.data()),
		          num * sizeof(typename std::decay_t<decltype(tree)>::value_type));
	}

	void writeTreeStructure(WriteBuffer& out, std::vector<IndexField> const& tree) const
	{
		std::uint64_t num = tree.size();
		out.write(&num, sizeof(num));
		out.write(tree.data(),
		          num * sizeof(typename std::decay_t<decltype(tree)>::value_type));
	}

	void writeNumNodes(std::ostream& out, std::uint64_t num_nodes) const
	{
		out.write(reinterpret_cast<char const*>(&num_nodes), sizeof(num_nodes));
	}

	void writeNumNodes(WriteBuffer& out, std::uint64_t num_nodes) const
	{
		out.write(&num_nodes, sizeof(num_nodes));
	}

	template <class InputIt>
	void writeNodes(std::ostream& out, InputIt first, InputIt last, bool const compress,
	                mt_t const map_types, int const compression_acceleration_level,
	                int const compression_level) const
	{
		derived().writeNodes(out, first, last, compress, map_types,
		                     compression_acceleration_level, compression_level);
	}

	template <class InputIt>
	void writeNodes(WriteBuffer& out, InputIt first, InputIt last, bool const compress,
	                mt_t const map_types, int const compression_acceleration_level,
	                int const compression_level) const
	{
		derived().writeNodes(out, first, last, compress, map_types,
		                     compression_acceleration_level, compression_level);
	}

 protected:
	// The number of depth levels
	depth_t depth_levels_;
	// The maximum coordinate value the octree can store
	key_t max_value_;

	// Node indices
	std::deque<std::array<index_t, 8>> indices_;
	// Free node indices
	std::stack<index_t> free_indices_;

	// Parent code for the nodes
	std::deque<Code> codes_;

	// Indicates wheter a node has been modified
	std::deque<IndexField> modified_;

	// Stores the node size at a given depth, where the depth is the index
	std::array<node_size_t, maxDepthLevels()> node_size_;
	// Reciprocal of the node size at a given depth, where the depth is the index
	std::array<node_size_t, maxDepthLevels()> node_size_factor_;

	//
	// Store for performance
	//

	std::size_t modified_tree_size_ = 0;
	std::size_t modified_indices_size_ = 0;

	//
	// Friends
	//

	friend Derived;

	template <class Derived2>
	friend class OctreeBase;
};

}  // namespace ufo::map

#endif  // UFO_MAP_OCTREE_BASE_H