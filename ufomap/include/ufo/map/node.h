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

#ifndef UFO_MAP_OCTREE_NODE_H
#define UFO_MAP_OCTREE_NODE_H

// UFO
#include <ufo/geometry/aaebb.h>
#include <ufo/geometry/point.h>
#include <ufo/map/code.h>
#include <ufo/map/types.h>

// STL
#include <utility>

namespace ufo::map
{
/*!
 * @brief A wrapper around a UFOMap inner/leaf node.
 *
 */
struct Node {
 public:
	//
	// Constructor
	//

	constexpr Node() = default;

	constexpr Node(Node const&) = default;

	constexpr Node(Node&&) = default;

	constexpr Node& operator=(Node const&) = default;

	constexpr Node& operator=(Node&&) = default;

	void swap(Node& other) noexcept
	{
		std::swap(data_, other.data_);
		std::swap(code_, other.code_);
	}

	/*!
	 * @brief Compare two nodes.
	 *
	 * @param lhs,rhs The nodes to compare.
	 * @return Whether the two nodes are equal.
	 */
	friend constexpr bool operator==(Node lhs, Node rhs) noexcept
	{
		return lhs.data_ == rhs.data_ && lhs.code_ == rhs.code_;
	}

	/*!
	 * @brief Compare two nodes.
	 *
	 * @param lhs,rhs The nodes to compare.
	 * @return Whether the two nodes are different.
	 */
	friend constexpr bool operator!=(Node lhs, Node rhs) noexcept { return !(lhs == rhs); }

	friend constexpr bool operator<(Node lhs, Node rhs) noexcept
	{
		return lhs.code() < rhs.code();
	}

	friend constexpr bool operator<=(Node lhs, Node rhs) noexcept
	{
		return lhs.code() <= rhs.code();
	}

	friend constexpr bool operator>(Node lhs, Node rhs) noexcept
	{
		return lhs.code() > rhs.code();
	}

	friend constexpr bool operator>=(Node lhs, Node rhs) noexcept
	{
		return lhs.code() >= rhs.code();
	}

	/*!
	 * @brief Get the code for the node.
	 *
	 * @return The code for the node.
	 */
	[[nodiscard]] constexpr Code code() const noexcept { return code_; }

	/*!
	 * @brief Get the depth of the node.
	 *
	 * @return The depth of the node.
	 */
	[[nodiscard]] constexpr depth_t depth() const noexcept { return code_.depth(); }

	/*!
	 * @brief Get the index of the node (i.e., the child index from the parent's
	 * perspective).
	 *
	 * @return The index of the node.
	 */
	[[nodiscard]] constexpr index_t index() const noexcept { return code_.index(); }

	struct Hash {
		static constexpr code_t hash(Node node) { return node.code().code(); }

		constexpr code_t operator()(Node node) const { return hash(node); }

		static constexpr bool equal(Node lhs, Node rhs) { return lhs == rhs; }
	};

 protected:
	constexpr Node(void* data, Code code, depth_t data_depth) noexcept
	    : data_(data), code_(code), data_depth_(data_depth)
	{
	}

	/*!
	 * @brief Get the corresponding data.
	 *
	 * @note Use the octree that generated the node to read the data.
	 *
	 * @return The corresponding data.
	 */
	[[nodiscard]] constexpr void* data() noexcept { return data_; }

	/*!
	 * @brief Get the corresponding data.
	 *
	 * @note Use the octree that generated the node to read the data.
	 *
	 * @return The corresponding data.
	 */
	[[nodiscard]] constexpr void const* data() const noexcept { return data_; }

	/*!
	 * @return Get the depth of the data.
	 */
	[[nodiscard]] constexpr depth_t dataDepth() const noexcept { return data_depth_; }

	/*!
	 * @return The index of the data.
	 */
	[[nodiscard]] constexpr index_t dataIndex() const noexcept
	{
		return code_.index(dataDepth());
	}

	/*!
	 * @return Whether data points to same node as the code.
	 */
	[[nodiscard]] constexpr bool isActualData() const noexcept
	{
		return depth() == dataDepth();
	}

 protected:
	// Pointer to the actual node
	void* data_ = nullptr;
	// The code for the node
	Code code_;
	// The depth of the data
	depth_t data_depth_ = 0;

	template <class Derived, class Data, class InnerData, bool ReuseNodes, bool LockLess,
	          bool CountNodes>
	friend class OctreeBase;
};

struct NodeBV : public Node {
 public:
	//
	// Constructor
	//

	constexpr NodeBV() = default;

	constexpr NodeBV(NodeBV const&) = default;

	constexpr NodeBV(NodeBV&&) = default;

	constexpr NodeBV& operator=(NodeBV const&) = default;

	constexpr NodeBV& operator=(NodeBV&&) = default;

	void swap(NodeBV& other)
	{
		std::swap(static_cast<Node&>(*this), static_cast<Node&>(other));
		std::swap(aaebb_, other.aaebb_);
	}

	friend constexpr bool operator==(NodeBV const& lhs, NodeBV const& rhs) noexcept
	{
		return static_cast<Node>(lhs) == static_cast<Node>(rhs);
	}

	friend constexpr bool operator!=(NodeBV const& lhs, NodeBV const& rhs) noexcept
	{
		return !(lhs == rhs);
	}

	/*!
	 * @brief The bounding volume of the node.
	 *
	 * @return The bounding volume of the node.
	 */
	[[nodiscard]] constexpr geometry::AAEBB boundingVolume() const noexcept
	{
		return aaebb_;
	}

	/*!
	 * @brief The center coordinate of the node.
	 *
	 * @return The center coordinate of the node.
	 */
	[[nodiscard]] constexpr geometry::Point center() const noexcept
	{
		return aaebb_.center;
	}

	/*!
	 * @brief The minimum coordinate of the node.
	 *
	 * @return The minimum coordinate of the node.
	 */
	[[nodiscard]] constexpr geometry::Point min() const noexcept { return aaebb_.min(); }

	/*!
	 * @brief The maximum coordinate of the node.
	 *
	 * @return The maximum coordinte of the node.
	 */
	[[nodiscard]] constexpr geometry::Point max() const noexcept { return aaebb_.max(); }

	/*!
	 * @brief Half the length of a side of the node.
	 *
	 * @return Half the length of a side of the node.
	 */
	[[nodiscard]] constexpr float halfSize() const noexcept { return aaebb_.half_size; }

	/*!
	 * @brief The length of a side of the node.
	 *
	 * @return The length of a side of the node.
	 */
	[[nodiscard]] constexpr float size() const noexcept { return 2 * halfSize(); }

	/*!
	 * @brief The center x coordinate of the node.
	 *
	 * @return The center x coordinate of the node.
	 */
	[[nodiscard]] constexpr float x() const noexcept { return aaebb_.center.x; }

	/*!
	 * @brief The center y coordinate of the node.
	 *
	 * @return The center y coordinate of the node.
	 */
	[[nodiscard]] constexpr float y() const noexcept { return aaebb_.center.y; }

	/*!
	 * @brief The center z coordinate of the node.
	 *
	 * @return The center z coordinate of the node.
	 */
	[[nodiscard]] constexpr float z() const noexcept { return aaebb_.center.z; }

 protected:
	constexpr NodeBV(void* data, Code code, depth_t data_depth,
	                 geometry::AAEBB aaebb) noexcept
	    : Node(data, code, data_depth), aaebb_(aaebb)
	{
	}

	constexpr NodeBV(Node node, geometry::AAEBB aaebb) noexcept : Node(node), aaebb_(aaebb)
	{
	}

 private:
	// The AAEBB for the node
	geometry::AAEBB aaebb_;

	template <class Derived, class Data, class InnerData, bool ReuseNodes, bool LockLess,
	          bool CountNodes>
	friend class OctreeBase;
};
}  // namespace ufo::map

#endif  // UFO_MAP_OCTREE_NODE_H