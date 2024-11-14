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

#ifndef UFO_MAP_INTENSITY_MAP_HPP
#define UFO_MAP_INTENSITY_MAP_HPP

// UFO
#include <ufo/map/types.hpp>
#include <ufo/utility/bit_set.hpp>
#include <ufo/utility/io/buffer.hpp>

// STL
#include <algorithm>
#include <array>
#include <deque>
#include <numeric>
#include <type_traits>
#include <utility>

namespace ufo
{
template <class Derived, offset_t N, class Index, class Node, class Code, class Key,
          class Coord>
class IntensityMap
{
 public:
	using IntensityBlock = DataBlock<intensity_t, N>;

	//
	// Intensity block
	//

	[[nodiscard]] IntensityBlock& intensityBlock(pos_t block) { return intensity_[block]; }

	[[nodiscard]] IntensityBlock const& intensityBlock(pos_t block) const
	{
		return intensity_[block];
	}

	//
	// Get intensity
	//

	[[nodiscard]] intensity_t intensity(Index node) const
	{
		return intensity_[node.pos][node.offset];
	}

	[[nodiscard]] constexpr intensity_t intensity(Node node) const
	{
		return intensity(derived().index(node));
	}

	[[nodiscard]] intensity_t intensity(Code code) const
	{
		return intensity(derived().index(code));
	}

	[[nodiscard]] intensity_t intensity(Key key) const
	{
		return intensity(derived().index(key));
	}

	[[nodiscard]] intensity_t intensity(Coord coord, depth_t depth = 0) const
	{
		return intensity(derived().index(coord, depth));
	}

	//
	// Set intensity
	//

	void setIntensity(Index node, intensity_t intensity)
	{
		derived().apply(
		    node,
		    [this, intensity](Index node) { intensity_[node.pos][node.offset] = intensity; },
		    [this, intensity](pos_t pos) { intensity_[pos].fill(intensity); });
	}

	Node setIntensity(Node node, intensity_t intensity, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, intensity](Index node) { intensity_[node.pos][node.offset] = intensity; },
		    [this, intensity](pos_t pos) { intensity_[pos].fill(intensity); }, propagate);
	}

	Node setIntensity(Code code, intensity_t intensity, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, intensity](Index node) { intensity_[node.pos][node.offset] = intensity; },
		    [this, intensity](pos_t pos) { intensity_[pos].fill(intensity); }, propagate);
	}

	Node setIntensity(Key key, intensity_t intensity, bool propagate = true)
	{
		return setIntensity(derived().toCode(key), intensity, propagate);
	}

	Node setIntensity(Coord coord, intensity_t intensity, bool propagate = true,
	                  depth_t depth = 0)
	{
		return setIntensity(derived().toCode(coord, depth), intensity, propagate);
	}

	//
	// Update intensity
	//

	void updateIntensity(Index node, double change)
	{
		derived().apply(
		    node, [this, change](Index node) { intensity_[node.pos][node.offset] += change; },
		    [this, change](pos_t pos) {
			    for (auto& e : intensity_[pos]) {
				    e += change;
			    }
		    });
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, intensity_t>, bool> = true>
	void updateIntensity(Index node, UnaryOp unary_op)
	{
		derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    intensity_[node.pos][node.offset] = unary_op(intensity_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : intensity_[pos]) {
				    e = unary_op(e);
			    }
		    });
	}

	template <
	    class BinaryOp,
	    std::enable_if_t<std::is_invocable_v<BinaryOp, Index, intensity_t>, bool> = true>
	void updateIntensity(Index node, BinaryOp binary_op)
	{
		derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    intensity_[node.pos][node.offset] =
			        binary_op(node, intensity_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    offset_t i{};
			    for (auto& e : intensity_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    });
	}

	Node updateIntensity(Node node, double change, bool propagate = true)
	{
		return derived().apply(
		    node, [this, change](Index node) { intensity_[node.pos][node.offset] += change; },
		    [this, change](pos_t pos) {
			    for (auto& e : intensity_[pos]) {
				    e += change;
			    }
		    },
		    propagate);
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, intensity_t>, bool> = true>
	Node updateIntensity(Node node, UnaryOp unary_op, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    intensity_[node.pos][node.offset] = unary_op(intensity_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : intensity_[pos]) {
				    e = unary_op(e);
			    }
		    },
		    propagate);
	}

	template <
	    class BinaryOp,
	    std::enable_if_t<std::is_invocable_v<BinaryOp, Index, intensity_t>, bool> = true>
	Node updateIntensity(Node node, BinaryOp binary_op, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    intensity_[node.pos][node.offset] =
			        binary_op(node, intensity_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    offset_t i{};
			    for (auto& e : intensity_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    },
		    propagate);
	}

	Node updateIntensity(Code code, double change, bool propagate = true)
	{
		return derived().apply(
		    code, [this, change](Index node) { intensity_[node.pos][node.offset] += change; },
		    [this, change](pos_t pos) {
			    for (auto& e : intensity_[pos]) {
				    e += change;
			    }
		    },
		    propagate);
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, intensity_t>, bool> = true>
	Node updateIntensity(Code code, UnaryOp unary_op, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, unary_op](Index node) {
			    intensity_[node.pos][node.offset] = unary_op(intensity_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : intensity_[pos]) {
				    e = unary_op(e);
			    }
		    },
		    propagate);
	}

	template <
	    class BinaryOp,
	    std::enable_if_t<std::is_invocable_v<BinaryOp, Index, intensity_t>, bool> = true>
	Node updateIntensity(Code code, BinaryOp binary_op, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, binary_op](Index node) {
			    intensity_[node.pos][node.offset] =
			        binary_op(node, intensity_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    offset_t i{};
			    for (auto& e : intensity_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    },
		    propagate);
	}

	Node updateIntensity(Key key, double change, bool propagate = true)
	{
		return updateIntensity(derived().toCode(key), change, propagate);
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, intensity_t>, bool> = true>
	Node updateIntensity(Key key, UnaryOp unary_op, bool propagate = true)
	{
		return updateIntensity(derived().toCode(key), unary_op, propagate);
	}

	template <
	    class BinaryOp,
	    std::enable_if_t<std::is_invocable_v<BinaryOp, Index, intensity_t>, bool> = true>
	Node updateIntensity(Key key, BinaryOp binary_op, bool propagate = true)
	{
		return updateIntensity(derived().toCode(key), binary_op, propagate);
	}

	Node updateIntensity(Coord coord, double change, bool propagate = true,
	                     depth_t depth = 0)
	{
		return updateIntensity(derived().toCode(coord, depth), change, propagate);
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, intensity_t>, bool> = true>
	Node updateIntensity(Coord coord, UnaryOp unary_op, bool propagate = true,
	                     depth_t depth = 0)
	{
		return updateIntensity(derived().toCode(coord, depth), unary_op, propagate);
	}

	template <
	    class BinaryOp,
	    std::enable_if_t<std::is_invocable_v<BinaryOp, Index, intensity_t>, bool> = true>
	Node updateIntensity(Coord coord, BinaryOp binary_op, bool propagate = true,
	                     depth_t depth = 0)
	{
		return updateIntensity(derived().toCode(coord, depth), binary_op, propagate);
	}

	//
	// Propagation criteria
	//

	[[nodiscard]] constexpr PropagationCriteria intensityPropagationCriteria()
	    const noexcept
	{
		return prop_criteria_;
	}

	constexpr void setIntensityPropagationCriteria(PropagationCriteria prop_criteria,
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

	IntensityMap() { intensity_.emplace_back(); }

	IntensityMap(IntensityMap const& other) = default;

	IntensityMap(IntensityMap&& other) = default;

	template <class Derived2>
	IntensityMap(IntensityMap<Derived2, N, Index, Node, Code, Key, Coord> const& other)
	    : intensity_(other.intensity_), prop_criteria_(other.prop_criteria_)
	{
	}

	template <class Derived2>
	IntensityMap(IntensityMap<Derived2, N, Index, Node, Code, Key, Coord>&& other)
	    : intensity_(std::move(other.intensity_))
	    , prop_criteria_(std::move(other.prop_criteria_))
	{
	}

	//
	// Destructor
	//

	~IntensityMap() = default;

	//
	// Assignment operator
	//

	IntensityMap& operator=(IntensityMap const& rhs) = default;

	IntensityMap& operator=(IntensityMap&& rhs) = default;

	template <class Derived2>
	IntensityMap& operator=(
	    IntensityMap<Derived2, N, Index, Node, Code, Key, Coord> const& rhs)
	{
		intensity_     = rhs.intensity_;
		prop_criteria_ = rhs.prop_criteria_;
		return *this;
	}

	template <class Derived2>
	IntensityMap& operator=(IntensityMap<Derived2, N, Index, Node, Code, Key, Coord>&& rhs)
	{
		intensity_     = std::move(rhs.intensity_);
		prop_criteria_ = std::move(rhs.prop_criteria_);
		return *this;
	}

	//
	// Swap
	//

	void swap(IntensityMap& other) noexcept
	{
		std::swap(intensity_, other.intensity_);
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
		intensity_.emplace_back();
		intensity_.back().fill(intensity_[node.pos][node.offset]);
	}

	//
	// Resize
	//

	void resize(std::size_t count)
	{
		// TODO: Implement
		intensity_.resize(count);
	}

	//
	// Reserve
	//

	void reserveImpl(std::size_t new_cap) { intensity_.reserve(new_cap); }

	//
	// Initialize root
	//

	void initRoot()
	{
		auto node                         = derived().rootIndex();
		intensity_[node.pos][node.offset] = 0;
	}

	//
	// Fill
	//

	void fill(Index node, pos_t children)
	{
		intensity_[children].fill(intensity_[node.pos][node.offset]);
	}

	//
	// Clear
	//

	void clearImpl() { intensity_.resize(1); }

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
		switch (intensityPropagationCriteria()) {
			case PropagationCriteria::MIN:
				intensity_[node.pos][node.offset] = min(children);
				return;
			case PropagationCriteria::MAX:
				intensity_[node.pos][node.offset] = max(children);
				return;
			case PropagationCriteria::MEAN:
				intensity_[node.pos][node.offset] = mean(children);
				return;
			case PropagationCriteria::NONE: return;
		}
	}

	[[nodiscard]] constexpr intensity_t min(pos_t block) const
	{
		intensity_t ret = intensity_[block][0];
		for (auto e : intensity_[block]) {
			ret = std::min(ret, e);
		}
		return ret;
	}

	[[nodiscard]] constexpr intensity_t max(pos_t block) const
	{
		intensity_t ret = intensity_[block][0];
		for (auto e : intensity_[block]) {
			ret = std::max(ret, e);
		}
		return ret;
	}

	[[nodiscard]] constexpr intensity_t mean(pos_t block) const
	{
		return std::accumulate(std::cbegin(intensity_[block]), std::cend(intensity_[block]),
		                       intensity_t{}) /
		       static_cast<intensity_t>(N);
	}

	//
	// Is prunable
	//

	[[nodiscard]] bool isPrunable(pos_t block) const
	{
		return std::all_of(std::cbegin(intensity_[block]) + 1, std::cend(intensity_[block]),
		                   [a = intensity_[block].front()](auto b) { return a == b; });
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
		return sizeof(typename decltype(intensity_)::value_type);
	}

	[[nodiscard]] static constexpr std::size_t sizeofMap() noexcept
	{
		return sizeof(intensity_) + sizeof(prop_criteria_);
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr MapType mapType() noexcept { return MapType::INTENSITY; }

	[[nodiscard]] static constexpr std::size_t serializedSizeBlock() noexcept
	{
		return sizeof(typename decltype(intensity_)::value_type);
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
				in.read(intensity_[pos].data(), serializedSizeBlock());
			} else {
				IntensityBlock intensity;
				in.read(intensity.data(), serializedSizeBlock());
				for (offset_t i{}; N != i; ++i) {
					intensity_[pos][i] = offsets[i] ? intensity[i] : intensity_[pos][i];
				}
			}
		}
	}

	template <class BlockRange>
	void writeBlocks(WriteBuffer& out, BlockRange const& blocks) const
	{
		for (auto block : blocks) {
			out.write(intensity_[block].data(), serializedSizeBlock());
		}
	}

 protected:
	// Data
	Container<IntensityBlock> intensity_;

	// Propagation criteria
	PropagationCriteria prop_criteria_ = PropagationCriteria::MAX;

	template <class Derived2, offset_t N2, class Index2, class Node2, class Code2,
	          class Key2, class Coord2>
	friend class IntensityMap;
};
}  // namespace ufo

#endif  // UFO_MAP_INTENSITY_MAP_HPP