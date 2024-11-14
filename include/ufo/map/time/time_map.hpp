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

#ifndef UFO_MAP_TIME_MAP_HPP
#define UFO_MAP_TIME_MAP_HPP

// UFO
#include <ufo/map/time/is_time_map.hpp>
#include <ufo/map/types.hpp>
#include <ufo/utility/io/buffer.hpp>

// STL
#include <algorithm>
#include <utility>

namespace ufo
{
template <class Derived, offset_t N, class Index, class Node, class Code, class Key,
          class Coord>
class TimeMap
{
 public:
	using TimeBlock = DataBlock<time_t, N>;

	//
	// Time block
	//

	[[nodiscard]] TimeBlock& timeBlock(pos_t block) { return time_[block]; }

	[[nodiscard]] TimeBlock const& timeBlock(pos_t block) const { return time_[block]; }

	//
	// Get time
	//

	[[nodiscard]] time_t time(Index node) const { return time_[node.pos][node.offset]; }

	[[nodiscard]] time_t time(Node node) const { return time(derived().index(node)); }

	[[nodiscard]] time_t time(Code code) const { return time(derived().index(code)); }

	[[nodiscard]] time_t time(Key key) const { return time(derived().index(key)); }

	[[nodiscard]] time_t time(Coord coord, depth_t depth = 0) const
	{
		return time(derived().index(coord, depth));
	}

	//
	// Set time
	//

	void setTime(Index node, time_t value)
	{
		derived().apply(
		    node, [this, value](Index node) { time_[node.pos][node.offset] = value; },
		    [this, value](pos_t pos) { time_[pos].fill(value); });
	}

	Node setTime(Node node, time_t value, bool propagate = true)
	{
		return derived().apply(
		    node, [this, value](Index node) { time_[node.pos][node.offset] = value; },
		    [this, value](pos_t pos) { time_[pos].fill(value); }, propagate);
	}

	Node setTime(Code code, time_t value, bool propagate = true)
	{
		return derived().apply(
		    code, [this, value](Index node) { time_[node.pos][node.offset] = value; },
		    [this, value](pos_t pos) { time_[pos].fill(value); }, propagate);
	}

	Node setTime(Key key, time_t value, bool propagate = true)
	{
		return setTime(derived().toCode(key), value, propagate);
	}

	Node setTime(Coord coord, time_t value, bool propagate = true, depth_t depth = 0)
	{
		return setTime(derived().toCode(coord, depth), value, propagate);
	}

	//
	// Update time
	//

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, time_t>, bool> = true>
	void updateTime(Index node, UnaryOp unary_op)
	{
		derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    time_[node.pos][node.offset] = unary_op(time_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : time_[pos]) {
				    e = unary_op(e);
			    }
		    });
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, time_t>, bool> = true>
	void updateTime(Index node, BinaryOp binary_op)
	{
		derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    time_[node.pos][node.offset] = binary_op(node, time_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    offset_t i{};
			    for (auto& e : time_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    });
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, time_t>, bool> = true>
	Node updateTime(Node node, UnaryOp unary_op, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    time_[node.pos][node.offset] = unary_op(time_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : time_[pos]) {
				    e = unary_op(e);
			    }
		    },
		    propagate);
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, time_t>, bool> = true>
	Node updateTime(Node node, BinaryOp binary_op, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    time_[node.pos][node.offset] = binary_op(node, time_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    offset_t i{};
			    for (auto& e : time_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    },
		    propagate);
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, time_t>, bool> = true>
	Node updateTime(Code code, UnaryOp unary_op, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, unary_op](Index node) {
			    time_[node.pos][node.offset] = unary_op(time_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : time_[pos]) {
				    e = unary_op(e);
			    }
		    },
		    propagate);
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, time_t>, bool> = true>
	Node updateTime(Code code, BinaryOp binary_op, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, binary_op](Index node) {
			    time_[node.pos][node.offset] = binary_op(node, time_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    offset_t i{};
			    for (auto& e : time_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    },
		    propagate);
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, time_t>, bool> = true>
	Node updateTime(Key key, UnaryOp unary_op, bool propagate = true)
	{
		return updateTime(derived().toCode(key), unary_op, propagate);
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, time_t>, bool> = true>
	Node updateTime(Key key, BinaryOp binary_op, bool propagate = true)
	{
		return updateTime(derived().toCode(key), binary_op, propagate);
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, time_t>, bool> = true>
	Node updateTime(Coord coord, UnaryOp unary_op, bool propagate = true, depth_t depth = 0)
	{
		return updateTime(derived().toCode(coord, depth), unary_op, propagate);
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, time_t>, bool> = true>
	Node updateTime(Coord coord, BinaryOp binary_op, bool propagate = true,
	                depth_t depth = 0)
	{
		return updateTime(derived().toCode(coord, depth), binary_op, propagate);
	}

	//
	// Propagation criteria
	//

	[[nodiscard]] constexpr PropagationCriteria timePropagationCriteria() const noexcept
	{
		return prop_criteria_;
	}

	void setTimePropagationCriteria(PropagationCriteria prop_criteria,
	                                bool                propagate = true) noexcept
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

	TimeMap() { time_.emplace_back(); }

	TimeMap(TimeMap const& other) = default;

	TimeMap(TimeMap&& other) = default;

	template <class Derived2>
	TimeMap(TimeMap<Derived2, N, Index, Node, Code, Key, Coord> const& other)
	    : time_(other.time_), prop_criteria_(other.prop_criteria_)
	{
	}

	template <class Derived2>
	TimeMap(TimeMap<Derived2, N, Index, Node, Code, Key, Coord>&& other)
	    : time_(std::move(other.time_)), prop_criteria_(std::move(other.prop_criteria_))
	{
	}

	//
	// Destructor
	//

	~TimeMap() = default;

	//
	// Assignment operator
	//

	TimeMap& operator=(TimeMap const& rhs) = default;

	TimeMap& operator=(TimeMap&& rhs) = default;

	template <class Derived2>
	TimeMap& operator=(TimeMap<Derived2, N, Index, Node, Code, Key, Coord> const& rhs)
	{
		time_          = rhs.time_;
		prop_criteria_ = rhs.prop_criteria_;
		return *this;
	}

	template <class Derived2>
	TimeMap& operator=(TimeMap<Derived2, N, Index, Node, Code, Key, Coord>&& rhs)
	{
		time_          = std::move(rhs.time_);
		prop_criteria_ = std::move(rhs.prop_criteria_);
		return *this;
	}

	//
	// Swap
	//

	void swap(TimeMap& other) noexcept
	{
		std::swap(time_, other.time_);
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
	// Initialize root
	//

	void initRoot()
	{
		auto node                    = derived().rootIndex();
		time_[node.pos][node.offset] = 0;
	}

	//
	// Create node block
	//

	void createBlock(Index node)
	{
		time_.emplace_back();
		time_.back().fill(time_[node.pos][node.offset]);
	}

	//
	// Fill
	//

	void fill(Index node, pos_t children)
	{
		time_[children].fill(time_[node.pos][node.offset]);
	}

	//
	// Resize
	//

	void resize(std::size_t count)
	{
		// TODO: Implement
		time_.resize(count);
	}

	//
	// Reserve
	//

	void reserveImpl(std::size_t new_cap) { time_.reserve(new_cap); }

	//
	// Clear
	//

	void clearImpl() { time_.resize(1); }

	void clearImpl(pos_t) {}

	//
	// Update block
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

	constexpr inline void updateNode(Index node, pos_t children)
	{
		switch (timePropagationCriteria()) {
			case PropagationCriteria::MIN: time_[node.pos][node.offset] = min(children); return;
			case PropagationCriteria::MAX: time_[node.pos][node.offset] = max(children); return;
			case PropagationCriteria::MEAN:
				time_[node.pos][node.offset] = mean(children);
				return;
			case PropagationCriteria::NONE: return;
		}
	}

	[[nodiscard]] constexpr inline time_t min(pos_t block) const
	{
		time_t ret = time_[block][0];
		for (time_t e : time_[block]) {
			ret = std::min(ret, e);
		}
		return ret;
	}

	[[nodiscard]] constexpr inline time_t max(pos_t block) const
	{
		time_t ret = time_[block][0];
		for (time_t e : time_[block]) {
			ret = std::max(ret, e);
		}
		return ret;
	}

	[[nodiscard]] constexpr inline time_t mean(pos_t block) const
	{
		return static_cast<time_t>(
		    std::accumulate(std::cbegin(time_[block]), std::cend(time_[block]), 0.0) / N);
	}

	//
	// Is prunable
	//

	[[nodiscard]] bool isPrunable(pos_t block) const
	{
		return std::all_of(std::cbegin(time_[block]) + 1, std::cend(time_[block]),
		                   [a = time_[block].front()](auto b) { return a == b; });
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
		return sizeof(typename decltype(time_)::value_type);
	}

	[[nodiscard]] static constexpr std::size_t sizeofMap() noexcept
	{
		return sizeof(time_) + sizeof(prop_criteria_);
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr MapType mapType() noexcept { return MapType::TIME; }

	[[nodiscard]] static constexpr std::size_t serializedSizeBlock() noexcept
	{
		return sizeof(typename decltype(time_)::value_type);
	}

	template <class Container>
	constexpr std::size_t serializedSize(Container const& c) const
	{
		return c.size() * serializedSizeBlock();
	}

	template <class Container>
	void readNodes(ReadBuffer& in, Container const& c)
	{
		for (auto const [pos, offsets] : c) {
			if (offsets.all()) {
				in.read(time_[pos].data(), serializedSizeBlock());
			} else {
				TimeBlock time;
				in.read(time.data(), serializedSizeBlock());
				for (offset_t i{}; N != i; ++i) {
					time_[pos][i] = offsets[i] ? time[i] : time_[pos][i];
				}
			}
		}
	}

	template <class BlockRange>
	void writeBlocks(WriteBuffer& out, BlockRange const& blocks) const
	{
		for (auto block : blocks) {
			out.write(time_[block].data(), serializedSizeBlock());
		}
	}

 protected:
	// Data
	Container<TimeBlock> time_;

	// Propagation criteria
	PropagationCriteria prop_criteria_ = PropagationCriteria::MAX;

	template <class Derived2, offset_t N2, class Index2, class Node2, class Code2,
	          class Key2, class Coord2>
	friend class TimeMap;
};
}  // namespace ufo

#endif  // UFO_MAP_TIME_MAP_HPP