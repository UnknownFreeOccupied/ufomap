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

#ifndef UFO_MAP_COUNT_MAP_HPP
#define UFO_MAP_COUNT_MAP_HPP

// UFO
#include <ufo/map/types.hpp>
#include <ufo/utility/io/buffer.hpp>

// STL
#include <algorithm>
#include <array>
#include <cstdint>
#include <type_traits>
#include <utility>
#include <vector>

namespace ufo
{
template <class Derived, offset_t N, class Index, class Node, class Code, class Key,
          class Coord>
class CountMap
{
 public:
	using CountBlock = DataBlock<count_t, N>;

	//
	// Count block
	//

	[[nodiscard]] CountBlock& countBlock(pos_t block) { return count_[block]; }

	[[nodiscard]] CountBlock const& countBlock(pos_t block) const { return count_[block]; }

	//
	// Get count
	//

	[[nodiscard]] count_t count(Index node) const { return count_[node.pos][node.offset]; }

	[[nodiscard]] count_t count(Node node) const { return count(derived().index(node)); }

	[[nodiscard]] count_t count(Code code) const { return count(derived().index(code)); }

	[[nodiscard]] count_t count(Key key) const { return count(derived().index(key)); }

	[[nodiscard]] count_t count(Coord coord, depth_t depth = 0) const
	{
		return count(derived().index(coord, depth));
	}

	//
	// Set count
	//

	void setCount(Index node, count_t count)
	{
		return derived().apply(
		    node, [this, count](Index node) { count_[node.pos][node.offset] = count; },
		    [this, count](pos_t pos) { count_[pos].fill(count); });
	}

	Node setCount(Node node, count_t count, bool propagate = true)
	{
		return derived().apply(
		    node, [this, count](Index node) { count_[node.pos][node.offset] = count; },
		    [this, count](pos_t pos) { count_[pos].fill(count); }, propagate);
	}

	Node setCount(Code code, count_t count, bool propagate = true)
	{
		return derived().apply(
		    code, [this, count](Index node) { count_[node.pos][node.offset] = count; },
		    [this, count](pos_t pos) { count_[pos].fill(count); }, propagate);
	}

	Node setCount(Key key, count_t count, bool propagate = true)
	{
		return setCount(derived().toCode(key), count, propagate);
	}

	Node setCount(Coord coord, count_t count, bool propagate = true, depth_t depth = 0)
	{
		return setCount(derived().toCode(coord, depth), count, propagate);
	}

	//
	// Update count
	//

	void updateCount(Index node, int change)
	{
		derived().apply(
		    node, [this, change](Index node) { count_[node.pos][node.offset] += change; },
		    [this, change](pos_t pos) {
			    for (auto& e : count_[pos]) {
				    e += change;
			    }
		    });
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, count_t>, bool> = true>
	void updateCount(Index node, UnaryOp unary_op)
	{
		derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    count_[node.pos][node.offset] = unary_op(count_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : count_[pos]) {
				    e = unary_op(e);
			    }
		    });
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, count_t>, bool> = true>
	void updateCount(Index node, BinaryOp binary_op)
	{
		derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    count_[node.pos][node.offset] = binary_op(node, count_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    offset_t i{};
			    for (auto& e : count_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    });
	}

	Node updateCount(Node node, int change, bool propagate = true)
	{
		return derived().apply(
		    node, [this, change](Index node) { count_[node.pos][node.offset] += change; },
		    [this, change](pos_t pos) {
			    for (auto& e : count_[pos]) {
				    e += change;
			    }
		    },
		    propagate);
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, count_t>, bool> = true>
	Node updateCount(Node node, UnaryOp unary_op, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    count_[node.pos][node.offset] = unary_op(count_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : count_[pos]) {
				    e = unary_op(e);
			    }
		    },
		    propagate);
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, count_t>, bool> = true>
	Node updateCount(Node node, BinaryOp binary_op, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    count_[node.pos][node.offset] = binary_op(node, count_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    offset_t i{};
			    for (auto& e : count_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    },
		    propagate);
	}

	Node updateCount(Code code, int change, bool propagate = true)
	{
		return derived().apply(
		    code, [this, change](Index node) { count_[node.pos][node.offset] += change; },
		    [this, change](pos_t pos) {
			    for (auto& e : count_[pos]) {
				    e += change;
			    }
		    },
		    propagate);
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, count_t>, bool> = true>
	Node updateCount(Code code, UnaryOp unary_op, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, unary_op](Index node) {
			    count_[node.pos][node.offset] = unary_op(count_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : count_[pos]) {
				    e = unary_op(e);
			    }
		    },
		    propagate);
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, count_t>, bool> = true>
	Node updateCount(Code code, BinaryOp binary_op, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, binary_op](Index node) {
			    count_[node.pos][node.offset] = binary_op(node, count_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    offset_t i{};
			    for (auto& e : count_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    },
		    propagate);
	}

	Node updateCount(Key key, int change, bool propagate = true)
	{
		return updateCount(derived().toCode(key), change, propagate);
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, count_t>, bool> = true>
	Node updateCount(Key key, UnaryOp unary_op, bool propagate = true)
	{
		return updateCount(derived().toCode(key), unary_op, propagate);
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, count_t>, bool> = true>
	Node updateCount(Key key, BinaryOp binary_op, bool propagate = true)
	{
		return updateCount(derived().toCode(key), binary_op, propagate);
	}

	Node updateCount(Coord coord, int change, bool propagate = true, depth_t depth = 0)
	{
		return updateCount(derived().toCode(coord, depth), change, propagate);
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, count_t>, bool> = true>
	Node updateCount(Coord coord, UnaryOp unary_op, bool propagate = true,
	                 depth_t depth = 0)
	{
		return updateCount(derived().toCode(coord, depth), unary_op, propagate);
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, count_t>, bool> = true>
	Node updateCount(Coord coord, BinaryOp binary_op, bool propagate = true,
	                 depth_t depth = 0)
	{
		return updateCount(derived().toCode(coord, depth), binary_op, propagate);
	}

	//
	// Propagation criteria
	//

	[[nodiscard]] constexpr PropagationCriteria countPropagationCriteria() const noexcept
	{
		return prop_criteria_;
	}

	void setCountPropagationCriteria(PropagationCriteria prop_criteria,
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

	CountMap() { count_.emplace_back(); }

	CountMap(CountMap const&) = default;

	CountMap(CountMap&&) = default;

	template <class Derived2>
	CountMap(CountMap<Derived2, N, Index, Node, Code, Key, Coord> const& other)
	    : count_(other.count_), prop_criteria_(other.prop_criteria_)
	{
	}

	template <class Derived2>
	CountMap(CountMap<Derived2, N, Index, Node, Code, Key, Coord>&& other)
	    : count_(std::move(other.count_)), prop_criteria_(std::move(other.prop_criteria_))
	{
	}

	//
	// Destructor
	//

	~CountMap() = default;

	//
	// Assignment operator
	//

	CountMap& operator=(CountMap const&) = default;

	CountMap& operator=(CountMap&&) = default;

	template <class Derived2>
	CountMap& operator=(CountMap<Derived2, N, Index, Node, Code, Key, Coord> const& rhs)
	{
		count_         = rhs.count_;
		prop_criteria_ = rhs.prop_criteria_;
		return *this;
	}

	template <class Derived2>
	CountMap& operator=(CountMap<Derived2, N, Index, Node, Code, Key, Coord>&& rhs)
	{
		count_         = std::move(rhs.count_);
		prop_criteria_ = std::move(rhs.prop_criteria_);
		return *this;
	}

	//
	// Swap
	//

	void swap(CountMap& other) noexcept
	{
		std::swap(count_, other.count_);
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
		count_.emplace_back();
		count_.back().fill(count_[node.pos][node.offset]);
	}

	//
	// Resize
	//

	void resize(std::size_t count)
	{
		// TODO: Implement
		count_.resize(count);
	}

	//
	// Reserve
	//

	void reserveImpl(std::size_t new_cap) { count_.reserve(new_cap); }

	//
	// Initialize root
	//

	void initRoot()
	{
		auto node                     = derived().rootIndex();
		count_[node.pos][node.offset] = 0;
	}

	//
	// Fill
	//

	void fill(Index node, pos_t children)
	{
		count_[children].fill(count_[node.pos][node.offset]);
	}

	//
	// Clear
	//

	void clearImpl() { count_.resize(1); }

	void clearImpl(pos_t) {}

	//
	// Update node
	//

	void updateBlock(pos_t block, std::array<bool, N> modified_parent)
	{
		for (offset_t i{}; N != i; ++i) {
			if (modified_parent[i]) {
				Index node(block, i);
				updateNode(node, derived().children(node));
			}
		}
	}

	void updateNode(Index node, pos_t children)
	{
		switch (countPropagationCriteria()) {
			case PropagationCriteria::MIN:
				count_[node.pos][node.offset] = min(children);
				return;
			case PropagationCriteria::MAX:
				count_[node.pos][node.offset] = max(children);
				return;
			case PropagationCriteria::MEAN:
				count_[node.pos][node.offset] = mean(children);
				return;
			case PropagationCriteria::NONE: return;
		}
	}

	[[nodiscard]] constexpr count_t min(pos_t block) const
	{
		count_t ret = count_[block][0];
		for (auto e : count_[block]) {
			ret = std::min(ret, e);
		}
		return ret;
	}

	[[nodiscard]] constexpr count_t max(pos_t block) const
	{
		count_t ret = count_[block][0];
		for (auto e : count_[block]) {
			ret = std::max(ret, e);
		}
		return ret;
	}

	[[nodiscard]] constexpr count_t mean(pos_t block) const
	{
		// FIXME: What happens if count_t is not unsigned integer?
		if constexpr (std::is_unsigned_v<count_t>) {
			return std::accumulate(std::cbegin(count_[block]), std::cend(count_[block]),
			                       std::uint64_t(0)) /
			       N;
		} else {
			return std::accumulate(std::cbegin(count_[block]), std::cend(count_[block]), 0.0) /
			       N;
		}
	}

	//
	// Is prunable
	//

	[[nodiscard]] bool isPrunable(pos_t block) const
	{
		return std::all_of(std::cbegin(count_[block]) + 1, std::cend(count_[block]),
		                   [a = count_[block].front()](auto b) { return a == b; });
	}

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
		return sizeof(typename decltype(count_)::value_type);
	}

	[[nodiscard]] static constexpr std::size_t sizeofMap() noexcept
	{
		return sizeof(count_) + sizeof(prop_criteria_);
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr MapType mapType() noexcept { return MapType::COUNT; }

	[[nodiscard]] static constexpr std::size_t serializedSizeBlock() noexcept
	{
		return sizeof(typename decltype(count_)::value_type);
	}

	template <class Container>
	std::size_t serializedSize(Container const& c) const
	{
		return c.size() * serializedSizeBlock();
	}

	template <class Container>
	void readNodes(ReadBuffer& in, Container const& c)
	{
		for (auto const [pos, offsets] : c) {
			if (offsets.all()) {
				in.read(count_[pos].data(), serializedSizeBlock());
			} else {
				CountBlock count;
				in.read(count.data(), serializedSizeBlock());
				for (offset_t i{}; N != i; ++i) {
					count_[pos][i] = offsets[i] ? count[i] : count_[pos][i];
				}
			}
		}
	}

	template <class BlockRange>
	void writeBlocks(WriteBuffer& out, BlockRange const& blocks) const
	{
		for (auto block : blocks) {
			out.write(count_[block].data(), serializedSizeBlock());
		}
	}

 protected:
	// Data
	Container<CountBlock> count_;

	// Propagation criteria
	PropagationCriteria prop_criteria_ = PropagationCriteria::MAX;

	template <class Derived2, offset_t N2, class Index2, class Node2, class Code2,
	          class Key2, class Coord2>
	friend class CountMap;
};
}  // namespace ufo

#endif  // UFO_MAP_COUNT_MAP_HPP