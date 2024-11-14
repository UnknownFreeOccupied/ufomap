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

#ifndef UFO_MAP_LABEL_MAP_HPP
#define UFO_MAP_LABEL_MAP_HPP

// UFO
#include <ufo/map/label/is_label_map.hpp>
#include <ufo/map/types.hpp>
#include <ufo/utility/bit_set.hpp>
#include <ufo/utility/io/buffer.hpp>

namespace ufo
{
template <class Derived, offset_t N, class Index, class Node, class Code, class Key,
          class Coord>
class LabelMap
{
 public:
	using LabelBlock = DataBlock<label_t, N>;

	//
	// Label block
	//

	[[nodiscard]] LabelBlock& labelBlock(pos_t block) { return label_[block]; }

	[[nodiscard]] LabelBlock const& labelBlock(pos_t block) const { return label_[block]; }

	//
	// Get label
	//

	[[nodiscard]] label_t label(Index node) const { return label_[node.pos][node.offset]; }

	[[nodiscard]] label_t label(Node node) const { return label(derived().index(node)); }

	[[nodiscard]] label_t label(Code code) const { return label(derived().index(code)); }

	[[nodiscard]] label_t label(Key key) const { return label(derived().index(key)); }

	[[nodiscard]] label_t label(Coord coord, depth_t depth = 0) const
	{
		return label(derived().index(coord, depth));
	}

	//
	// Set label
	//

	void setLabel(Index node, label_t value)
	{
		derived().apply(
		    node, [this, value](Index node) { label_[node.pos][node.offset] = value; },
		    [this, value](pos_t pos) { label_[pos].fill(value); });
	}

	Node setLabel(Node node, label_t value, bool propagate = true)
	{
		return derived().apply(
		    node, [this, value](Index node) { label_[node.pos][node.offset] = value; },
		    [this, value](pos_t pos) { label_[pos].fill(value); }, propagate);
	}

	Node setLabel(Code code, label_t value, bool propagate = true)
	{
		return derived().apply(
		    code, [this, value](Index node) { label_[node.pos][node.offset] = value; },
		    [this, value](pos_t pos) { label_[pos].fill(value); }, propagate);
	}

	Node setLabel(Key key, label_t value, bool propagate = true)
	{
		return setLabel(derived().toCode(key), value, propagate);
	}

	Node setLabel(Coord coord, label_t value, bool propagate = true, depth_t depth = 0)
	{
		return setLabel(derived().toCode(coord, depth), value, propagate);
	}

	//
	// Update label
	//

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, label_t>, bool> = true>
	void updateLabel(Index node, UnaryOp unary_op)
	{
		derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    label_[node.pos][node.offset] = unary_op(label_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : label_[pos]) {
				    e = unary_op(e);
			    }
		    });
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, label_t>, bool> = true>
	void updateLabel(Index node, BinaryOp binary_op)
	{
		derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    label_[node.pos][node.offset] = binary_op(node, label_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    offset_t i{};
			    for (auto& e : label_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    });
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, label_t>, bool> = true>
	Node updateLabel(Node node, UnaryOp unary_op, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    label_[node.pos][node.offset] = unary_op(label_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : label_[pos]) {
				    e = unary_op(e);
			    }
		    },
		    propagate);
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, label_t>, bool> = true>
	Node updateLabel(Node node, BinaryOp binary_op, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    label_[node.pos][node.offset] = binary_op(node, label_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    offset_t i{};
			    for (auto& e : label_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    },
		    propagate);
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, label_t>, bool> = true>
	Node updateLabel(Code code, UnaryOp unary_op, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, unary_op](Index node) {
			    label_[node.pos][node.offset] = unary_op(label_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : label_[pos]) {
				    e = unary_op(e);
			    }
		    },
		    propagate);
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, label_t>, bool> = true>
	Node updateLabel(Code code, BinaryOp binary_op, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, binary_op](Index node) {
			    label_[node.pos][node.offset] = binary_op(node, label_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    offset_t i{};
			    for (auto& e : label_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    },
		    propagate);
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, label_t>, bool> = true>
	Node updateLabel(Key key, UnaryOp unary_op, bool propagate = true)
	{
		return updateLabel(derived().toCode(key), unary_op, propagate);
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, label_t>, bool> = true>
	Node updateLabel(Key key, BinaryOp binary_op, bool propagate = true)
	{
		return updateLabel(derived().toCode(key), binary_op, propagate);
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, label_t>, bool> = true>
	Node updateLabel(Coord coord, UnaryOp unary_op, bool propagate = true,
	                 depth_t depth = 0)
	{
		return updateLabel(derived().toCode(coord, depth), unary_op, propagate);
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, label_t>, bool> = true>
	Node updateLabel(Coord coord, BinaryOp binary_op, bool propagate = true,
	                 depth_t depth = 0)
	{
		return updateLabel(derived().toCode(coord, depth), binary_op, propagate);
	}

 protected:
	//
	// Constructors
	//

	LabelMap() { label_.emplace_back(); }

	LabelMap(LabelMap const& other) = default;

	LabelMap(LabelMap&& other) = default;

	template <class Derived2>
	LabelMap(LabelMap<Derived2, N, Index, Node, Code, Key, Coord> const& other)
	    : label_(other.label_)
	{
	}

	template <class Derived2>
	LabelMap(LabelMap<Derived2, N, Index, Node, Code, Key, Coord>&& other)
	    : label_(std::move(other.label_))
	{
	}

	//
	// Destructor
	//

	~LabelMap() = default;

	//
	// Assignment operator
	//

	LabelMap& operator=(LabelMap const& rhs) = default;

	LabelMap& operator=(LabelMap&& rhs) = default;

	template <class Derived2>
	LabelMap& operator=(LabelMap<Derived2, N, Index, Node, Code, Key, Coord> const& rhs)
	{
		label_ = rhs.label_;
		return *this;
	}

	template <class Derived2>
	LabelMap& operator=(LabelMap<Derived2, N, Index, Node, Code, Key, Coord>&& rhs)
	{
		label_ = std::move(rhs.label_);
		return *this;
	}

	//
	// Swap
	//

	void swap(LabelMap& other) noexcept { std::swap(label_, other.label_); }

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
		label_.emplace_back();
		label_.back().fill(label_[node.pos][node.offset]);
	}

	//
	// Resize
	//

	void resize(std::size_t count) { label_.resize(count, LabelBlock{}); }

	//
	// Reserve
	//

	void reserveImpl(std::size_t new_cap) { label_.reserve(new_cap); }

	//
	// Initialize root
	//

	void initRoot()
	{
		auto node                     = derived().rootIndex();
		label_[node.pos][node.offset] = 0;
	}

	//
	// Fill
	//

	void fill(Index node, pos_t children)
	{
		label_[children].fill(label_[node.pos][node.offset]);
	}

	//
	// Clear
	//

	void clearImpl() { label_.assign(1, 0); }

	void clearImpl(pos_t block) {}

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

	void updateNode(Index node, pos_t children)
	{
		label_t l{};
		for (auto e : label_[children]) {
			l |= e;
		}
		label_[node.pos][node.offset] = l;
	}

	//
	// Is prunable
	//

	[[nodiscard]] bool isPrunable(pos_t block) const
	{
		return std::all_of(std::cbegin(label_[block]) + 1, std::cend(label_[block]),
		                   [a = label_[block].front()](auto b) { return a == b; });
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
		return sizeof(typename decltype(label_)::value_type);
	}

	[[nodiscard]] static constexpr std::size_t sizeofMap() noexcept
	{
		return sizeof(label_);
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr MapType mapType() noexcept { return MapType::LABEL; }

	[[nodiscard]] static constexpr std::size_t serializedSizeBlock() noexcept
	{
		return sizeof(typename decltype(label_)::value_type);
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
				in.read(label_[pos].data(), serializedSizeBlock());
			} else {
				LabelBlock label;
				in.read(label.data(), serializedSizeBlock());
				for (offset_t i{}; N != i; ++i) {
					label_[pos][i] = offsets[i] ? label[i] : label_[pos][i];
				}
			}
		}
	}

	template <class BlockRange>
	void writeBlocks(WriteBuffer& out, BlockRange const& blocks) const
	{
		for (auto block : blocks) {
			out.write(label_[block].data(), serializedSizeBlock());
		}
	}

	//
	// Dot file info
	//

	void dotFileInfo(std::ostream& out, Index node) const
	{
		out << "Label: " << label_[node.pos][node.offset];
	}

 protected:
	Container<LabelBlock> label_;

	template <class Derived2, offset_t N2, class Index2, class Node2, class Code2,
	          class Key2, class Coord2>
	friend class LabelMap;
};
}  // namespace ufo

#endif  // UFO_MAP_LABEL_MAP_HPP