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

#ifndef UFO_MAP_SEEN_EMPTY_MAP_HPP
#define UFO_MAP_SEEN_EMPTY_MAP_HPP

// UFO
#include <ufo/map/seen_empty/is_seen_empty_map.hpp>
#include <ufo/map/seen_empty/seen_empty_propagation_criteria.hpp>
#include <ufo/map/types.hpp>
#include <ufo/utility/bit_set.hpp>
#include <ufo/utility/io/buffer.hpp>

namespace ufo
{
template <class Derived, offset_t N, class Index, class Node, class Code, class Key,
          class Coord>
class SeenEmptyMap
{
 public:
	using SeenEmptyBlock = BitSet<N>;

	//
	// Seen empty block
	//

	[[nodiscard]] SeenEmptyBlock& seenEmptyBlock(pos_t block) { return seen_empty_[block]; }

	[[nodiscard]] SeenEmptyBlock const& seenEmptyBlock(pos_t block) const
	{
		return seen_empty_[block];
	}

	//
	// Get seen free
	//

	[[nodiscard]] bool seenEmpty(Index node) const
	{
		return seen_empty_[node.pos][node.offset];
	}

	[[nodiscard]] bool seenEmpty(Node node) const
	{
		return seenEmpty(derived().index(node));
	}

	[[nodiscard]] bool seenEmpty(Code code) const
	{
		return seenEmpty(derived().index(code));
	}

	[[nodiscard]] bool seenEmpty(Key key) const { return seenEmpty(derived().index(key)); }

	[[nodiscard]] bool seenEmpty(Coord coord, depth_t depth = 0) const
	{
		return seenEmpty(derived().index(coord, depth));
	}

	//
	// Set seen free
	//

	void seenEmptySet(Index node)
	{
		derived().apply(
		    node, [this](Index node) { seen_empty_[node.pos].set(node.offset); },
		    [this](pos_t block) { seen_empty_[block].set(); });
	}

	Node seenEmptySet(Node node, bool propagate = true)
	{
		return derived().apply(
		    node, [this](Index node) { seen_empty_[node.pos].set(node.offset); },
		    [this](pos_t block) { seen_empty_[block].set(); }, propagate);
	}

	Node seenEmptySet(Code code, bool propagate = true)
	{
		return derived().apply(
		    code, [this](Index node) { seen_empty_[node.pos].set(node.offset); },
		    [this](pos_t block) { seen_empty_[block].set(); }, propagate);
	}

	Node seenEmptySet(Key key, bool propagate = true)
	{
		return seenEmptySet(derived().toCode(key), propagate);
	}

	Node seenEmptySet(Coord coord, bool propagate = true, depth_t depth = 0)
	{
		return seenEmptySet(derived().toCode(coord, depth), propagate);
	}

	//
	// Reset seen free
	//

	void seenEmptyReset(Index node)
	{
		derived().apply(
		    node, [this](Index node) { seen_empty_[node.pos].reset(node.offset); },
		    [this](pos_t block) { seen_empty_[block].reset(); });
	}

	Node seenEmptyReset(Node node, bool propagate = true)
	{
		return derived().apply(
		    node, [this](Index node) { seen_empty_[node.pos].reset(node.offset); },
		    [this](pos_t block) { seen_empty_[block].reset(); }, propagate);
	}

	Node seenEmptyReset(Code code, bool propagate = true)
	{
		return derived().apply(
		    code, [this](Index node) { seen_empty_[node.pos].reset(node.offset); },
		    [this](pos_t block) { seen_empty_[block].reset(); }, propagate);
	}

	Node seenEmptyReset(Key key, bool propagate = true)
	{
		return seenEmptyReset(derived().toCode(key), propagate);
	}

	Node seenEmptyReset(Coord coord, bool propagate = true, depth_t depth = 0)
	{
		return seenEmptyReset(derived().toCode(coord, depth), propagate);
	}

	//
	// Update seen free
	//

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, bool>, bool> = true>
	void seenEmptyUpdate(Index node, UnaryOp unary_op)
	{
		derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    seen_empty_[node.pos][node.offset] =
			        unary_op(std::as_const(seen_empty_[node.pos])[node.offset]);
		    },
		    [this, unary_op](pos_t block) {
			    for (std::size_t i{}; N != i; ++i) {
				    seen_empty_[block][i] = unary_op(std::as_const(seen_empty_[block])[i]);
			    }
		    });
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, bool>, bool> = true>
	void seenEmptyUpdate(Index node, BinaryOp binary_op)
	{
		derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    seen_empty_[node.pos][node.offset] =
			        binary_op(node, std::as_const(seen_empty_[node.pos])[node.offset]);
		    },
		    [this, binary_op](pos_t block) {
			    for (std::size_t i{}; N != i; ++i) {
				    seen_empty_[block][i] =
				        binary_op(Index(block, i++), std::as_const(seen_empty_[block])[i]);
			    }
		    });
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, bool>, bool> = true>
	Node seenEmptyUpdate(Node node, UnaryOp unary_op, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    seen_empty_[node.pos][node.offset] =
			        unary_op(std::as_const(seen_empty_[node.pos])[node.offset]);
		    },
		    [this, unary_op](pos_t block) {
			    for (std::size_t i{}; N != i; ++i) {
				    seen_empty_[block][i] = unary_op(std::as_const(seen_empty_[block])[i]);
			    }
		    },
		    propagate);
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, bool>, bool> = true>
	Node seenEmptyUpdate(Node node, BinaryOp binary_op, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    seen_empty_[node.pos][node.offset] =
			        binary_op(node, std::as_const(seen_empty_[node.pos])[node.offset]);
		    },
		    [this, binary_op](pos_t block) {
			    for (std::size_t i{}; N != i; ++i) {
				    seen_empty_[block][i] =
				        binary_op(Index(block, i++), std::as_const(seen_empty_[block])[i]);
			    }
		    },
		    propagate);
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, bool>, bool> = true>
	Node seenEmptyUpdate(Code code, UnaryOp unary_op, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, unary_op](Index node) {
			    seen_empty_[node.pos][node.offset] =
			        unary_op(std::as_const(seen_empty_[node.pos])[node.offset]);
		    },
		    [this, unary_op](pos_t block) {
			    for (std::size_t i{}; N != i; ++i) {
				    seen_empty_[block][i] = unary_op(std::as_const(seen_empty_[block])[i]);
			    }
		    },
		    propagate);
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, bool>, bool> = true>
	Node seenEmptyUpdate(Code code, BinaryOp binary_op, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, binary_op](Index node) {
			    seen_empty_[node.pos][node.offset] =
			        binary_op(node, std::as_const(seen_empty_[node.pos])[node.offset]);
		    },
		    [this, binary_op](pos_t block) {
			    for (std::size_t i{}; N != i; ++i) {
				    seen_empty_[block][i] =
				        binary_op(Index(block, i++), std::as_const(seen_empty_[block])[i]);
			    }
		    },
		    propagate);
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, bool>, bool> = true>
	Node seenEmptyUpdate(Key key, UnaryOp unary_op, bool propagate = true)
	{
		return seenEmptyUpdate(derived().toCode(key), unary_op, propagate);
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, bool>, bool> = true>
	Node seenEmptyUpdate(Key key, BinaryOp binary_op, bool propagate = true)
	{
		return seenEmptyUpdate(derived().toCode(key), binary_op, propagate);
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, bool>, bool> = true>
	Node seenEmptyUpdate(Coord coord, UnaryOp unary_op, bool propagate = true,
	                     depth_t depth = 0)
	{
		return seenEmptyUpdate(derived().toCode(coord, depth), unary_op, propagate);
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, bool>, bool> = true>
	Node seenEmptyUpdate(Coord coord, BinaryOp binary_op, bool propagate = true,
	                     depth_t depth = 0)
	{
		return seenEmptyUpdate(derived().toCode(coord, depth), binary_op, propagate);
	}

	//
	// Propagation criteria
	//

	[[nodiscard]] constexpr SeenEmptyPropagationCriteria seenEmptyPropagationCriteria()
	    const noexcept
	{
		return prop_criteria_;
	}

	void setSeenEmptyPropagationCriteria(SeenEmptyPropagationCriteria prop_criteria,
	                                     bool                         propagate = true)
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

	SeenEmptyMap()
	{
		seen_empty_.emplace_back();
		initRoot();
	}

	SeenEmptyMap(SeenEmptyMap const& other) = default;

	SeenEmptyMap(SeenEmptyMap&& other) = default;

	template <class Derived2>
	SeenEmptyMap(SeenEmptyMap<Derived2, N, Index, Node, Code, Key, Coord> const& other)
	    : seen_empty_(other.seen_empty_), prop_criteria_(other.prop_criteria_)
	{
	}

	template <class Derived2>
	SeenEmptyMap(SeenEmptyMap<Derived2, N, Index, Node, Code, Key, Coord>&& other)
	    : seen_empty_(std::move(other.seen_empty_))
	    , prop_criteria_(std::move(other.prop_criteria_))
	{
	}

	//
	// Destructor
	//

	~SeenEmptyMap() = default;

	//
	// Assignment operator
	//

	SeenEmptyMap& operator=(SeenEmptyMap const& rhs) = default;

	SeenEmptyMap& operator=(SeenEmptyMap&& rhs) = default;

	template <class Derived2>
	SeenEmptyMap& operator=(
	    SeenEmptyMap<Derived2, N, Index, Node, Code, Key, Coord> const& rhs)
	{
		seen_empty_    = rhs.seen_empty_;
		prop_criteria_ = rhs.prop_criteria_;
		return *this;
	}

	template <class Derived2>
	SeenEmptyMap& operator=(SeenEmptyMap<Derived2, N, Index, Node, Code, Key, Coord>&& rhs)
	{
		seen_empty_    = std::move(rhs.seen_empty_);
		prop_criteria_ = std::move(rhs.prop_criteria_);
		return *this;
	}

	//
	// Swap
	//

	void swap(SeenEmptyMap& other) noexcept
	{
		std::swap(seen_empty_, other.seen_empty_);
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
	// Init root
	//

	void initRoot() { seen_empty_[0][0] = 0; }

	//
	// Empty
	//

	[[nodiscard]] bool empty() const { return seen_empty_.empty(); }

	//
	// Resize
	//

	void resize(std::size_t count)
	{
		// TODO: Implement
		seen_empty_.resize(count);
	}

	//
	// Clear block
	//

	void clearBlock(pos_t) {}

	//
	// Create block
	//

	void createBlock(Index parent)
	{
		seen_empty_.emplace_back(seen_empty_[parent.pos][parent.offset] ? -1 : 0);
	}

	//
	// Fill block
	//

	void fillBlock(Index parent, pos_t block)
	{
		seen_empty_[block] = SeenEmptyBlock(seen_empty_[parent.pos][parent.offset] ? -1 : 0);
	}

	//
	// Update block
	//

	void updateBlock(pos_t block, std::array<bool, N> modified_parent)
	{
		auto const prop_criteria = seenEmptyPropagationCriteria();
		if (SeenEmptyPropagationCriteria::NONE == prop_criteria) {
			return;
		}

		for (offset_t i{}; N != i; ++i) {
			if (modified_parent[i]) {
				Index node{block, i};
				updateNode(node, derived().children(node), prop_criteria);
			}
		}
	}

	void updateNode(Index node, pos_t children,
	                SeenEmptyPropagationCriteria const prop_criteria)
	{
		switch (prop_criteria) {
			case SeenEmptyPropagationCriteria::ALL:
				seen_empty_[node.pos][node.offset] = seen_empty_[children].all();
			case SeenEmptyPropagationCriteria::ANY:
				seen_empty_[node.pos][node.offset] = seen_empty_[children].any();
			case SeenEmptyPropagationCriteria::SOME:
				seen_empty_[node.pos][node.offset] = seen_empty_[children].some();
			case SeenEmptyPropagationCriteria::NONE: break;
		}
	}

	//
	// Is prunable
	//

	[[nodiscard]] bool isPrunable(pos_t block) const
	{
		return seen_empty_[block].all() || seen_empty_[block].none();
	}

	void preparePrune(Index node) {}

	//
	// Memory
	//

	[[nodiscard]] static constexpr std::size_t sizeofNodeTimesN(Index) noexcept
	{
		return sizeofBlockLowerBound();
	}

	[[nodiscard]] static constexpr std::size_t sizeofBlock(pos_t) noexcept
	{
		return sizeofBlockLowerBound();
	}

	[[nodiscard]] static constexpr std::size_t sizeofBlockLowerBound() noexcept
	{
		return sizeof(typename decltype(seen_empty_)::value_type);
	}

	[[nodiscard]] static constexpr std::size_t sizeofMap() noexcept
	{
		return sizeof(seen_empty_);
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr MapType mapType() noexcept
	{
		return MapType::SEEN_EMPTY;
	}

	[[nodiscard]] static constexpr std::size_t serializedSizeBlock() noexcept
	{
		return sizeof(typename decltype(seen_empty_)::value_type);
	}

	[[nodiscard]] static constexpr std::size_t serializedSizeNode() noexcept
	{
		// TODO: Implement
		return 0;
	}

	template <class BlockOffsetRange>
	std::size_t serializedSize(BlockOffsetRange const& block_offset,
	                           std::size_t             num_nodes) const
	{
		return num_nodes * serializedSizeNode();
	}

	template <class BlockOffsetRange>
	void readNodes(ReadBuffer& in, BlockOffsetRange const& block_offset)
	{
		// TODO: Implement
	}

	template <class BlockOffsetRange>
	void writeNodes(WriteBuffer& out, BlockOffsetRange const& block_offset) const
	{
		// TODO: Implement
	}

	//
	// Dot file info
	//

	void dotFileInfo(std::ostream& out, Index node) const
	{
		out << "Seen empty: " << seen_empty_[node.pos][node.offset];
	}

 protected:
	Container<SeenEmptyBlock> seen_empty_;

	SeenEmptyPropagationCriteria prop_criteria_ = SeenEmptyPropagationCriteria::ALL;

	template <class Derived2, offset_t N2, class Index2, class Node2, class Code2,
	          class Key2, class Coord2>
	friend class SeenEmptyMap;
};
}  // namespace ufo

#endif  // UFO_MAP_SEEN_EMPTY_MAP_HPP