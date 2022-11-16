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
#include <ufo/map/intensity/intensity_node.h>
#include <ufo/map/intensity/intensity_predicate.h>
#include <ufo/map/types.h>

// STL
#include <functional>
#include <iostream>
#include <limits>
#include <utility>

namespace ufo::map
{
template <class Derived>
class IntensityMapBase
{
 public:
	//
	// Get intensity
	//

	[[nodiscard]] constexpr intensity_t intensity(Node node) const
	{
		return derived().leafNode(node).intensity[node.index()];
	}

	[[nodiscard]] intensity_t intensity(Code code) const
	{
		auto [node, depth] = derived().leafNodeAndDepth(code);
		return node.intensity[code.index(depth)];
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
		    [intensity](auto& node, index_t index) { node.intensity[index] = intensity; },
		    [intensity](auto& node) { node.intensity.fill(intensity); }, propagate);
	}

	void setIntensity(Code code, intensity_t intensity, bool propagate = true)
	{
		derived().apply(
		    code,
		    [intensity](auto& node, index_t index) { node.intensity[index] = intensity; },
		    [intensity](auto& node) { node.intensity.fill(intensity); }, propagate);
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

	IntensityMapBase() = default;

	IntensityMapBase(IntensityMapBase const& other) = default;

	IntensityMapBase(IntensityMapBase&& other) = default;

	template <class Derived2>
	IntensityMapBase(IntensityMapBase<Derived2> const& other)
	    : prop_criteria_(other.intensityPropagationCriteria())
	{
	}

	//
	// Assignment operator
	//

	IntensityMapBase& operator=(IntensityMapBase const& rhs) = default;

	IntensityMapBase& operator=(IntensityMapBase&& rhs) = default;

	template <class Derived2>
	IntensityMapBase& operator=(IntensityMapBase<Derived2> const& rhs)
	{
		prop_criteria_ = rhs.intensityPropagationCriteria();
		return *this;
	}

	//
	// Swap
	//

	void swap(IntensityMapBase& other) noexcept
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

	void initRoot() { derived().root().intensity[derived().rootIndex()] = 0; }

	//
	// Update node
	//

	template <std::size_t N>
	void updateNode(IntensityNode<N>& node, index_t index, IntensityNode<N> const children)
	{
		switch (intensityPropagationCriteria()) {
			case PropagationCriteria::MIN:
				node.intensity[index] = min(children.intensity);
				return;
			case PropagationCriteria::MAX:
				node.intensity[index] = max(children.intensity);
				return;
			case PropagationCriteria::MEAN:
				node.intensity[index] = mean(children.intensity);
				return;
		}
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr DataIdentifier dataIdentifier() noexcept
	{
		return DataIdentifier::INTENSITY;
	}

	[[nodiscard]] static constexpr bool canReadData(DataIdentifier identifier) noexcept
	{
		return dataIdentifier() == identifier;
	}

	template <class InputIt>
	[[nodiscard]] static constexpr uint8_t numData() noexcept
	{
		using value_type = typename std::iterator_traits<InputIt>::value_type;
		using node_type = typename value_type::node_type;
		return node_type::intensitySize();
	}

	template <class InputIt>
	std::size_t serializedSize(InputIt first, InputIt last) const
	{
		return std::iterator_traits<InputIt>::value_type::intensitySize() *
		       std::distance(first, last) * sizeof(intensity_t);
	}

	template <class OutputIt>
	void readNodes(std::istream& in, OutputIt first, OutputIt last)
	{
		constexpr auto N = numData<OutputIt>();

		auto size = std::distance(first, last) * N;

		auto data = std::make_unique<intensity_t[]>(size);
		in.read(reinterpret_cast<char*>(data.get()),
		        size * sizeof(typename decltype(data)::element_type));

		auto const d = data.get();
		for (; first != last; ++first, d += N) {
			if (first->index_field.all()) {
				std::copy(d, d + N, first->node.intensity.data());
			} else {
				for (index_t i = 0; N != i; ++i) {
					if (first->index_field[i]) {
						first->node.intensity[i] = *(d + i);
					}
				}
			}
		}
	}

	template <class OutputIt>
	void readNodes(ReadBuffer& in, OutputIt first, OutputIt last)
	{
		constexpr std::size_t const N = numData<OutputIt>();
		for (; first != last; ++first) {
			if (first->index_field.all()) {
				in.read(first->node.intensity.data(), N * sizeof(intensity_t));
			} else {
				for (index_t i = 0; N != i; ++i) {
					if (first->index_field[i]) {
						in.read(&first->node.intensity[i], sizeof(intensity_t));
					} else
						in.skipRead(sizeof(intensity_t));
				}
			}
		}
	}

	template <class InputIt>
	void writeNodes(std::ostream& out, InputIt first, InputIt last) const
	{
		auto size = std::distance(first, last) * 8;  // FIXME: numData<InputIt>();

		auto data = std::make_unique<intensity_t[]>(size);
		auto d = data.get();
		for (; first != last; ++first) {
			d = copy(first->intensity, d);
		}

		out.write(reinterpret_cast<char const*>(data.get()),
		          size * sizeof(typename decltype(data)::element_type));
	}

	template <class InputIt>
	void writeNodes(WriteBuffer& out, InputIt first, InputIt last) const
	{
		out.reserve(out.size() + serializedSize(first, last));
		for (; first != last; ++first) {
			out.write(first->intensity.data(), first->intensity.size() * sizeof(intensity_t));
		}
	}

 protected:
	// Propagation criteria
	PropagationCriteria prop_criteria_ = PropagationCriteria::MAX;
};
}  // namespace ufo::map

#endif  // UFO_MAP_INTENSITY_MAP_H