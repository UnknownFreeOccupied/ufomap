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
#include <ufo/map/code.hpp>
#include <ufo/map/index.hpp>
#include <ufo/map/key.hpp>
#include <ufo/map/node.hpp>
#include <ufo/map/point.hpp>
#include <ufo/map/types.hpp>
#include <ufo/util/bit_set.hpp>
#include <ufo/util/buffer.hpp>

// STL
#include <algorithm>
#include <array>
#include <deque>
#include <functional>
#include <iostream>
#include <limits>
#include <utility>

namespace ufo
{
template <class Derived, std::size_t N>
class LabelMap
{
 public:
	//
	// Get label
	//

	[[nodiscard]] constexpr label_t label(Node node) const
	{
		auto [index, offset] = derived().indexAndOffset(node);
		return label_[index][offset];
	}

	[[nodiscard]] label_t label(Code code) const
	{
		auto [index, offset] = derived().indexAndOffset(code);
		return label_[index][offset];
	}

	[[nodiscard]] label_t label(Key key) const { return label(derived().toCode(key)); }

	[[nodiscard]] label_t label(Point coord, depth_t depth = 0) const
	{
		return label(derived().toCode(coord, depth));
	}

	//
	// TODO: Add label
	//

	//
	// TODO: Remove label
	//

 protected:
	//
	// Constructors
	//

	LabelMap() = default;

	LabelMap(LabelMap const& other) = default;

	LabelMap(LabelMap&& other) = default;

	template <class Derived2>
	LabelMap(LabelMap<Derived2> const& other) : label_(other.label_)
	{
	}

	//
	// Assignment operator
	//

	LabelMap& operator=(LabelMap const& rhs) = default;

	LabelMap& operator=(LabelMap&& rhs) = default;

	template <class Derived2>
	LabelMap& operator=(LabelMap<Derived2> const& rhs)
	{
		label_ = rhs.label_;
		return *this;
	}

	//
	// Swap
	//

	void swap(LabelMap& other) noexcept { std::swap(prop_criteria_, other.prop_criteria_); }

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

	void initRoot() { label_[derived().rootIndex()][derived().rootOffset()].clear(); }

	//
	// Fill
	//

	void fill(index_t index, index_t parent_index, index_t parent_offset)
	{
		intensity_[index].fill(intensity_[parent_index][parent_offset]);
	}

	//
	// Clear
	//

	void clearImpl() {}

	void clearImpl(index_t index) {}

	//
	// Update block
	//

	void updateBlock(pos_t block)
	{
		// TODO implement
	}

	void updateNode(index_t index, index_t offset, index_t children_index)
	{
		switch (intensityPropagationCriteria()) {
			case PropagationCriteria::MIN:
				intensity_[index][offset] = min(intensity_[children_index]);
				return;
			case PropagationCriteria::MAX:
				intensity_[index][offset] = max(intensity_[children_index]);
				return;
			case PropagationCriteria::MEAN:
				intensity_[index][offset] = mean(intensity_[children_index]);
				return;
		}
	}

	//
	// Is collapsible
	//

	[[nodiscard]] constexpr bool isCollapsible(index_t index) const
	{
		return std::all_of(std::begin(intensity_[index]) + 1, std::end(intensity_[index]),
		                   [t = intensity_[index].front()](auto e) { return e == t; });
	}

	void preparePrune(Index node) {}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr MapType mapType() noexcept { return MapType::INTENSITY; }

	[[nodiscard]] static constexpr bool canReadData(MapType mt) noexcept
	{
		return mapType() == mt;
	}

	template <class InputIt>
	std::size_t serializedSize(InputIt first, InputIt last) const
	{
		return std::distance(first, last) * N * sizeof(intensity_t);
	}

	template <class OutputIt>
	void readNodes(ReadBuffer& in, OutputIt first, OutputIt last)
	{
		for (; first != last; ++first) {
			if (first->offsets.all()) {
				in.read(intensity_[first->index].data(), N * sizeof(intensity_t));
			} else {
				std::array<intensity_t, N> intensity;
				in.read(intensity.data(), N * sizeof(intensity_t));
				for (index_t i = 0; N != i; ++i) {
					if (first->offsets[i]) {
						intensity_[first->index][i] = intensity[i];
					}
				}
			}
		}
	}

	template <class BlockRange>
	void writeBlocks(WriteBuffer& out, BlockRange const& blocks) const
	{
		for (auto block : blocks) {
			// TODO: Implement
		}
	}

 protected:
	// Data
	std::deque<LabelSet> label_;

	template <class Derived2, std::size_t N2>
	friend class LabelMap;
};
}  // namespace ufo

namespace ufo
{
//
// Type traits
//

template <class Map>
struct is_labels_map
    : std::conditional_t<is_map_type_v<Map, MapType::LABEL>, std::true_type,
                         std::false_type> {
};
template <class Map>
inline constexpr bool is_labels_map_v = is_labels_map<Map>::value;
}  // namespace ufo
#endif  // UFO_MAP_INTENSITY_MAP_HPP