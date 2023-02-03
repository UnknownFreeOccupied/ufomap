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

#ifndef UFO_MAP_INTENSITY_MAP_H
#define UFO_MAP_INTENSITY_MAP_H

// UFO
#include <ufo/map/intensity/intensity_predicate.h>
#include <ufo/map/types.h>

// STL
#include <algorithm>
#include <array>
#include <deque>
#include <functional>
#include <iostream>
#include <limits>
#include <utility>

namespace ufo::map
{
template <class Derived, std::size_t N>
class IntensityMap
{
 public:
	//
	// Get intensity
	//

	[[nodiscard]] constexpr intensity_t intensity(Node node) const
	{
		auto [index, offset] = derived().indexAndOffset(node);
		return intensity_[index][offset];
	}

	[[nodiscard]] intensity_t intensity(Code code) const
	{
		auto [index, offset] = derived().indexAndOffset(code);
		return intensity_[index][offset];
	}

	[[nodiscard]] intensity_t intensity(Key key) const
	{
		return intensity(derived().toCode(key));
	}

	[[nodiscard]] intensity_t intensity(Point coord, depth_t depth = 0) const
	{
		return intensity(derived().toCode(coord, depth));
	}

	[[nodiscard]] intensity_t intensity(coord_t x, coord_t y, coord_t z,
	                                    depth_t depth = 0) const
	{
		return intensity(derived().toCode(x, y, z, depth));
	}

	//
	// Set intensity
	//

	void setIntensity(Node node, intensity_t intensity, bool propagate = true)
	{
		derived().apply(
		    node,
		    [&intensity_, intensity](index_t index, index_t offset) {
			    intensity_[index][offset] = intensity;
		    },
		    [&intensity_, intensity](index_t index) { intensity_[index].fill(intensity); },
		    propagate);
	}

	void setIntensity(Code code, intensity_t intensity, bool propagate = true)
	{
		derived().apply(
		    code,
		    [&intensity_, intensity](index_t index, index_t offset) {
			    intensity_[index][offset] = intensity;
		    },
		    [&intensity_, intensity](index_t index) { intensity_[index].fill(intensity); },
		    propagate);
	}

	void setIntensity(Key key, intensity_t intensity, bool propagate = true)
	{
		setIntensity(derived().toCode(key), intensity, propagate);
	}

	void setIntensity(Point coord, intensity_t intensity, bool propagate = true,
	                  depth_t depth = 0)
	{
		setIntensity(derived().toCode(coord, depth), intensity, propagate);
	}

	void setIntensity(coord_t x, coord_t y, coord_t z, intensity_t intensity,
	                  bool propagate = true, depth_t depth = 0)
	{
		setIntensity(derived().toCode(x, y, z, depth), intensity, propagate);
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

		// Set all inner nodes to modified
		// FIXME: Possible to optimize this to only set the ones with children
		derived().setModified(1);

		if (propagate) {
			derived().updateModifiedNodes();
		}
	}

 protected:
	//
	// Constructors
	//

	IntensityMap() = default;

	IntensityMap(IntensityMap const& other) = default;

	IntensityMap(IntensityMap&& other) = default;

	template <class Derived2>
	IntensityMap(IntensityMap<Derived2, N> const& other)
	    : intensity_(other.intensity_), prop_criteria_(other.intensityPropagationCriteria())
	{
	}

	//
	// Assignment operator
	//

	IntensityMap& operator=(IntensityMap const& rhs) = default;

	IntensityMap& operator=(IntensityMap&& rhs) = default;

	template <class Derived2>
	IntensityMap& operator=(IntensityMap<Derived2, N> const& rhs)
	{
		intensity_ = rhs.intensity_;
		prop_criteria_ = rhs.intensityPropagationCriteria();
		return *this;
	}

	//
	// Swap
	//

	void swap(IntensityMap& other) noexcept
	{
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
	// Initilize root
	//

	void initRoot() { intensity_[derived().rootIndex()][derived().rootOffset()] = 0; }

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

	void clear(index_t index) {}

	//
	// Update node
	//

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

	template <class InputIt>
	void writeNodes(WriteBuffer& out, InputIt first, InputIt last) const
	{
		out.reserve(out.size() + serializedSize(first, last));
		for (; first != last; ++first) {
			out.write(intensity_[*first].data(), N * sizeof(intensity_t));
		}
	}

 protected:
	// Data
	std::deque<std::array<intensity_t, N>> intensity_;

	// Propagation criteria
	PropagationCriteria prop_criteria_ = PropagationCriteria::MAX;

	template <class Derived2, std::size_t N2>
	friend class IntensityMap;
};
}  // namespace ufo::map

#endif  // UFO_MAP_INTENSITY_MAP_H