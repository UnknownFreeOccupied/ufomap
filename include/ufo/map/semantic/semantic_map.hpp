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

#ifndef UFO_MAP_SEMANTIC_MAP_HPP
#define UFO_MAP_SEMANTIC_MAP_HPP

// UFO
#include <ufo/map/semantic/semantic.hpp>
#include <ufo/map/types.hpp>
#include <ufo/utility/io/buffer.hpp>

// STL
#include <limits>

namespace ufo
{
enum class SemanticPropagationCriteria { MIN, MAX, NONE };

template <class Derived, offset_t N, class Index, class Node, class Code, class Key,
          class Coord>
class SemanticMap
{
 public:
	using SemanticBlock = DataBlock<Semantic, N>;

	//
	// Semantic block
	//

	[[nodiscard]] SemanticBlock& semanticBlock(pos_t block) { return semantic_[block]; }

	[[nodiscard]] SemanticBlock const& semanticBlock(pos_t block) const
	{
		return semantic_[block];
	}

	//
	// Get semantic
	//

	[[nodiscard]] Semantic semantic(Index node) const
	{
		return semantic_[node.pos][node.offset];
	}

	[[nodiscard]] Semantic semantic(Node node) const
	{
		return semantic(derived().index(node));
	}

	[[nodiscard]] Semantic semantic(Code code) const
	{
		return semantic(derived().index(code));
	}

	[[nodiscard]] Semantic semantic(Key key) const
	{
		return semantic(derived().index(key));
	}

	[[nodiscard]] Semantic semantic(Point coord, depth_t depth = 0) const
	{
		return semantic(derived().index(coord, depth));
	}

	//
	// Set semantic
	//

	void setSemantic(Index node, Semantic value)
	{
		derived().apply(
		    node, [this, value](Index node) { semantic_[node.pos][node.offset] = value; },
		    [this, value](pos_t pos) { semantic_[pos].fill(value); });
	}

	Node setSemantic(Node node, Semantic value, bool propagate = true)
	{
		return derived().apply(
		    node, [this, value](Index node) { semantic_[node.pos][node.offset] = value; },
		    [this, value](pos_t pos) { semantic_[pos].fill(value); }, propagate);
	}

	Node setSemantic(Code code, Semantic value, bool propagate = true)
	{
		return derived().apply(
		    code, [this, value](Index node) { semantic_[node.pos][node.offset] = value; },
		    [this, value](pos_t pos) { semantic_[pos].fill(value); }, propagate);
	}

	Node setSemantic(Key key, Semantic value, bool propagate = true)
	{
		return setSemantic(derived().toCode(key), value, propagate);
	}

	Node setSemantic(Point coord, Semantic value, bool propagate = true, depth_t depth = 0)
	{
		return setSemantic(derived().toCode(coord, depth), value, propagate);
	}

	//
	// Update semantic
	//

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, Semantic>, bool> = true>
	void updateSemantic(Index node, UnaryOp unary_op)
	{
		derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    semantic_[node.pos][node.offset] = unary_op(semantic_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : semantic_[pos]) {
				    e = unary_op(e);
			    }
		    });
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, Semantic>, bool> = true>
	void updateSemantic(Index node, BinaryOp binary_op)
	{
		derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    semantic_[node.pos][node.offset] =
			        binary_op(node, semantic_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    offset_t i{};
			    for (auto& e : semantic_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    });
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, Semantic>, bool> = true>
	Node updateSemantic(Node node, UnaryOp unary_op, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    semantic_[node.pos][node.offset] = unary_op(semantic_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : semantic_[pos]) {
				    e = unary_op(e);
			    }
		    },
		    propagate);
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, Semantic>, bool> = true>
	Node updateSemantic(Node node, BinaryOp binary_op, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    semantic_[node.pos][node.offset] =
			        binary_op(node, semantic_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    offset_t i{};
			    for (auto& e : semantic_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    },
		    propagate);
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, Semantic>, bool> = true>
	Node updateSemantic(Code code, UnaryOp unary_op, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, unary_op](Index node) {
			    semantic_[node.pos][node.offset] = unary_op(semantic_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : semantic_[pos]) {
				    e = unary_op(e);
			    }
		    },
		    propagate);
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, Semantic>, bool> = true>
	Node updateSemantic(Code code, BinaryOp binary_op, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, binary_op](Index node) {
			    semantic_[node.pos][node.offset] =
			        binary_op(node, semantic_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    offset_t i{};
			    for (auto& e : semantic_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    },
		    propagate);
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, Semantic>, bool> = true>
	Node updateSemantic(Key key, UnaryOp unary_op, bool propagate = true)
	{
		return updateSemantic(derived().toCode(key), unary_op, propagate);
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, Semantic>, bool> = true>
	Node updateSemantic(Key key, BinaryOp binary_op, bool propagate = true)
	{
		return updateSemantic(derived().toCode(key), binary_op, propagate);
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, Semantic>, bool> = true>
	Node updateSemantic(Point coord, UnaryOp unary_op, bool propagate = true,
	                    depth_t depth = 0)
	{
		return updateSemantic(derived().toCode(coord, depth), unary_op, propagate);
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, Semantic>, bool> = true>
	Node updateSemantic(Point coord, BinaryOp binary_op, bool propagate = true,
	                    depth_t depth = 0)
	{
		return updateSemantic(derived().toCode(coord, depth), binary_op, propagate);
	}

	//
	// Propagation criteria
	//

	[[nodiscard]] constexpr SemanticPropagationCriteria semanticPropagationCriteria()
	    const noexcept
	{
		return prop_criteria_;
	}

	constexpr void setSemanticPropagationCriteria(SemanticPropagationCriteria prop_criteria,
	                                              bool propagate = true) noexcept
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

	SemanticMap() { semantic_.emplace_back(); }

	SemanticMap(SemanticMap const& other) = default;

	SemanticMap(SemanticMap&& other) = default;

	template <class Derived2>
	SemanticMap(SemanticMap<Derived2, N, Index, Node, Code, Key, Coord> const& other)
	    : semantic_(other.semantic_), prop_criteria_(other.prop_criteria_)
	{
	}

	template <class Derived2>
	SemanticMap(SemanticMap<Derived2, N, Index, Node, Code, Key, Coord>&& other)
	    : semantic_(std::move(other.semantic_))
	    , prop_criteria_(std::move(other.prop_criteria_))
	{
	}

	//
	// Destructor
	//

	~SemanticMap() = default;

	//
	// Assignment operator
	//

	SemanticMap& operator=(SemanticMap const& rhs) = default;

	SemanticMap& operator=(SemanticMap&& rhs) = default;

	template <class Derived2>
	SemanticMap& operator=(
	    SemanticMap<Derived2, N, Index, Node, Code, Key, Coord> const& rhs)
	{
		semantic_      = rhs.semantic_;
		prop_criteria_ = rhs.prop_criteria_;
		return *this;
	}

	template <class Derived2>
	SemanticMap& operator=(SemanticMap<Derived2, N, Index, Node, Code, Key, Coord>&& rhs)
	{
		semantic_      = std::move(rhs.semantic_);
		prop_criteria_ = std::move(rhs.prop_criteria_);
		return *this;
	}

	//
	// Swap
	//

	void swap(SemanticMap& other) noexcept
	{
		std::swap(semantic_, other.semantic_);
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
		semantic_.emplace_back();
		semantic_.back().fill(semantic_[node.pos][node.offset]);
	}

	//
	// Resize
	//

	void resize(std::size_t count) { semantic_.resize(count, Semantic(0, 0)); }

	//
	// Reserve
	//

	void reserveImpl(std::size_t new_cap) { semantic_.reserve(new_cap); }

	//
	// Initialize root
	//

	void initRoot()
	{
		auto node                              = derived().rootIndex();
		semantic_[node.pos][node.offset].label = 0;
		semantic_[node.pos][node.offset].value = 0;
	}

	//
	// Fill
	//

	void fill(Index node, pos_t children)
	{
		semantic_[children].fill(semantic_[node.pos][node.offset]);
	}

	//
	// Clear
	//

	void clearImpl() { semantic_.assign(1, Semantic(0, 0)); }

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
		switch (semanticPropagationCriteria()) {
			case SemanticPropagationCriteria::MIN: {
				Semantic s(0, std::numeric_limits<value_t>::max());
				for (auto e : semantic_[children]) {
					s.label |= e.label;
					s.value = std::min(s.value, e.value);
				}
				semantic_[node.pos][node.offset] = s;
			}
				return;
			case SemanticPropagationCriteria::MAX: {
				Semantic s(0, std::numeric_limits<value_t>::lowest());
				for (auto e : semantic_[children]) {
					s.label |= e.label;
					s.value = std::max(s.value, e.value);
				}
				semantic_[node.pos][node.offset] = s;
			}
				return;
			case SemanticPropagationCriteria::NONE: return;
		}
	}

	//
	// Is prunable
	//

	[[nodiscard]] bool isPrunable(pos_t block) const
	{
		return std::all_of(std::cbegin(semantic_[block]) + 1, std::cend(semantic_[block]),
		                   [a = semantic_[block].front()](auto b) { return a == b; });
	}

	//
	// Memory node block
	//

	[[nodiscard]] static constexpr std::size_t memoryBlock() noexcept
	{
		return N * sizeof(Semantic);
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr MapType mapType() noexcept { return MapType::SEMANTIC; }

	template <class Container>
	constexpr std::size_t serializedSize(Container const& c) const
	{
		return c.size() * memoryBlock();
	}

	template <class Container>
	void readNodes(ReadBuffer& in, Container const& c)
	{
		for (auto const [pos, offsets] : c) {
			if (offsets.all()) {
				in.read(semantic_[pos].data(), memoryBlock());
			} else {
				SemanticBlock semantic;
				in.read(semantic.data(), memoryBlock());
				for (offset_t i{}; N != i; ++i) {
					semantic_[pos][i] = offsets[i] ? semantic[i] : semantic_[pos][i];
				}
			}
		}
	}

	template <class BlockRange>
	void writeBlocks(WriteBuffer& out, BlockRange const& blocks) const
	{
		for (auto block : blocks) {
			out.write(semantic_[block].data(), memoryBlock());
		}
	}

	//
	// Dot file info
	//

	void dotFileInfo(std::ostream& out, Index node) const
	{
		out << "Semantic: " << semantic_[node.pos][node.offset];
	}

 protected:
	Container<SemanticBlock> semantic_;

	// Propagation criteria
	SemanticPropagationCriteria prop_criteria_ = SemanticPropagationCriteria::MAX;

	template <class Derived2, offset_t N2, class Index2, class Node2, class Code2,
	          class Key2, class Coord2>
	friend class SemanticMap;
};
}  // namespace ufo

namespace ufo
{
//
// Type traits
//

template <class Map>
struct is_semantic_map
    : std::conditional_t<is_map_type_v<Map, MapType::SEMANTIC>, std::true_type,
                         std::false_type> {
};
template <class Map>
inline constexpr bool is_semantic_map_v = is_semantic_map<Map>::value;
}  // namespace ufo
#endif  // UFO_MAP_SEMANTIC_MAP_HPP