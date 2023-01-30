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

#ifndef UFO_MAP_TIME_MAP_H
#define UFO_MAP_TIME_MAP_H

// UFO
#include <ufo/algorithm/algorithm.h>
#include <ufo/map/time/time_predicate.h>
#include <ufo/map/types.h>

// STL
#include <algorithm>
#include <array>
#include <functional>
#include <iostream>
#include <limits>
#include <utility>

namespace ufo::map
{
template <class Derived, std::size_t N>
class TimeMapBase
{
 public:
	//
	// Get time
	//

	[[nodiscard]] time_t timeUnsafe(Index index) const
	{
		return time_[index.index][index.offset];
	}

	[[nodiscard]] time_t time(Node node) const { return time(derived().index(node)); }

	[[nodiscard]] time_t time(Code code) const { return time(derived().index(code)); }

	[[nodiscard]] time_t time(Key key) const { return time(derived().toCode(key)); }

	[[nodiscard]] time_t time(Point coord, depth_t depth = 0) const
	{
		return time(derived().toCode(coord, depth));
	}

	[[nodiscard]] time_t time(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return time(derived().toCode(x, y, z, depth));
	}

	//
	// Set time
	//

	void setTimeUnsafe(Index index, time_t time)
	{
		derived().applyUnsafe(
		    index, [&time_, time](Index idx) { time_[idx.index][idx.offset] = time; },
		    [&time_, time](index_t i) { time_[i].fill(time); });
	}

	void setTimeUnsafe(IndexFam const index, time_t time)
	{
		if (derived().allLeaf(index.index)) {
			for (offset_t i{}; N != i; ++i) {
				time_[index.index][i] = index.offset[i] ? time : time_[index.index][i];
			}
		} else {
			for (offset_t i{}; N != i; ++i) {
				if (index.offset[i]) {
					setTimeUnsafe(Index(index.index, i), time);
				}
			}
		}
	}

	Node setTime(Node node, time_t time, bool propagate = true)
	{
		return derived().apply(
		    node, [&time_, time](Index idx) { time_[idx.index][idx.offset] = time; },
		    [&time_, time](index_t i) { time_[i].fill(time); }, propagate);
	}

	Node setTime(Code code, time_t time, bool propagate = true)
	{
		return derived().apply(
		    code, [&time_, time](Index idx) { time_[idx.index][idx.offset] = time; },
		    [&time_, time](index_t i) { time_[i].fill(time); }, propagate);
	}

	Node setTime(Key key, time_t time, bool propagate = true)
	{
		return setTime(derived().toCode(key), time, propagate);
	}

	Node setTime(Point coord, time_t time, bool propagate = true, depth_t depth = 0)
	{
		return setTime(derived().toCode(coord, depth), time, propagate);
	}

	Node setTime(coord_t x, coord_t y, coord_t z, time_t time, bool propagate = true,
	             depth_t depth = 0)
	{
		return setTime(derived().toCode(x, y, z, depth), time, propagate);
	}

	//
	// Propagation criteria
	//

	[[nodiscard]] constexpr PropagationCriteria timePropagationCriteria() const noexcept
	{
		return prop_criteria_;
	}

	void setTimePropagationCriteria(PropagationCriteria prop_criteria,
	                                bool propagate = true) noexcept
	{
		if (prop_criteria_ == prop_criteria) {
			return;
		}

		prop_criteria_ = prop_criteria;

		// Set all inner nodes to modified
		// FIXME: Possible to optimize this to only set the ones with children
		derived().setModified(1);
		// TODO: derived().setParentsModified();

		if (propagate) {
			derived().updateModifiedNodes();
		}
	}

 protected:
	//
	// Constructors
	//

	TimeMapBase() = default;

	TimeMapBase(TimeMapBase const& other) = default;

	TimeMapBase(TimeMapBase&& other) = default;

	template <class Derived2>
	TimeMapBase(TimeMapBase<Derived2, N> const& other)
	    : time_(other.time_), prop_criteria_(other.prop_criteria_)
	{
	}

	template <class Derived2>
	TimeMapBase(TimeMapBase<Derived2, N>&& other)
	    : time_(std::move(other.time_)), prop_criteria_(std::move(other.prop_criteria_))
	{
	}

	//
	// Destructor
	//

	~TimeMapBase() = default;

	//
	// Assignment operator
	//

	TimeMapBase& operator=(TimeMapBase const& rhs) = default;

	TimeMapBase& operator=(TimeMapBase&& rhs) = default;

	template <class Derived2>
	TimeMapBase& operator=(TimeMapBase<Derived2, N> const& rhs)
	{
		time_ = rhs.time_;
		prop_criteria_ = rhs.prop_criteria_;
		return *this;
	}

	template <class Derived2>
	TimeMapBase& operator=(TimeMapBase<Derived2, N>&& rhs)
	{
		time_ = std::move(rhs.time_);
		prop_criteria_ = std::move(rhs.prop_criteria_);
		return *this;
	}

	//
	// Swap
	//

	void swap(TimeMapBase& other) noexcept
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
	// Allocate node block
	//

	void allocateNodeBlock() { time_.emplace_back(); }

	//
	// Initilize root
	//

	void initRoot() { setTimeUnsafe(derived().rootIndex(), 0); }

	//
	// Fill
	//

	void fill(index_t index, Index parent_idx)
	{
		time_[index].fill(time_[parent_idx.index][parent_idx.offset]);
	}

	//
	// Clear
	//

	void clear() { time_.clear(); }

	void clear(index_t index) {}

	//
	// Shrink to fit
	//

	void shrinkToFit() { time_.shrink_to_fit(); }

	//
	// Update node
	//

	void updateNode(Index idx, index_t children_index)
	{
		switch (timePropagationCriteria()) {
			case PropagationCriteria::MIN:
				time_[idx.index][idx.offset] = min(time_[children_index]);
				return;
			case PropagationCriteria::MAX:
				time_[idx.index][idx.offset] = max(time_[children_index]);
				return;
			case PropagationCriteria::MEAN:
				time_[idx.index][idx.offset] = mean(time_[children_index]);
				return;
		}
	}

	//
	// Is prunable
	//

	[[nodiscard]] bool isPrunable(index_t index) const
	{
		return std::all_of(std::cbegin(time_[index]) + 1, std::cend(time_[index]),
		                   [t = time_[index].front()](auto e) { return e == t; });
	}

	//
	// Memory node block
	//

	[[nodiscard]] static constexpr std::size_t memoryNodeBlock() const noexcept
	{
		return N * sizeof(time_t);
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr MapType mapType() noexcept { return MapType::TIME; }

	[[nodiscard]] static constexpr bool canReadData(MapType mt) noexcept
	{
		return mapType() == mt;
	}

	template <class InputIt>
	constexpr std::size_t serializedSize(InputIt first, InputIt last) const
	{
		return std::distance(first, last) * memoryNodeBlock();
	}

	template <class OutputIt>
	void readNodes(ReadBuffer& in, OutputIt first, OutputIt last)
	{
		for (; first != last; ++first) {
			if (first->indices.all()) {
				in.read(time_[first->index].data(), memoryNodeBlock());
			} else {
				std::array<time_t, N> time;
				in.read(time.data(), memoryNodeBlock());
				for (offset_t i{}; N != i; ++i) {
					if (first->indices[i]) {
						time_[first->index][i] = time[i];
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
			out.write(time_[*first].data(), memoryNodeBlock());
		}
	}

 protected:
	// Data
	std::deque<std::array<time_t, N>> time_;

	// Propagation criteria
	PropagationCriteria prop_criteria_ = PropagationCriteria::MAX;

	template <class Derived2, std::size_t N2>
	friend class TimeMapBase;
};
}  // namespace ufo::map

#endif  // UFO_MAP_TIME_MAP_H