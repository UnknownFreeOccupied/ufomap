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

#ifndef UFO_MAP_TIME_MAP_BASE_H
#define UFO_MAP_TIME_MAP_BASE_H

// UFO
#include <ufo/map/predicate/time.h>
#include <ufo/map/time/time_node.h>
#include <ufo/map/types.h>

// STL
#include <functional>
#include <iostream>
#include <limits>
#include <utility>

namespace ufo::map
{
template <class Derived>
class TimeMapBase
{
 public:
	//
	// Get time
	//

	constexpr time_t getTime(Node node) const noexcept
	{
		return getTime(derived().getLeafNode(node), node.index());
	}

	time_t getTime(Code code) const
	{
		return getTime(derived().getLeafNode(code), code.index());
	}

	time_t getTime(Key key) const { return getTime(Derived::toCode(key)); }

	time_t getTime(Point3 coord, depth_t depth = 0) const
	{
		return getTime(derived().toCode(coord, depth));
	}

	time_t getTime(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return getTime(derived().toCode(x, y, z, depth));
	}

	//
	// Set time
	//

	void setTime(Node node, time_t time, bool propagate = true)
	{
		derived().apply(
		    node,
		    [time](auto& node, index_t const index) {
			    TimeMapBase::setTime(node, index, time);
		    },
		    [time](auto& node) { TimeMapBase::setTime(node, time); }, propagate);
	}

	void setTime(Code code, time_t time, bool propagate = true)
	{
		derived().apply(
		    code,
		    [time](auto& node, index_t const index) {
			    TimeMapBase::setTime(node, index, time);
		    },
		    [time](auto& node) { TimeMapBase::setTime(node, time); }, propagate);
	}

	void setTime(Key key, time_t time, bool propagate = true)
	{
		setTime(Derived::toCode(key), time, propagate);
	}

	void setTime(Point3 coord, time_t time, bool propagate = true, depth_t depth = 0)
	{
		setTime(derived().toCode(coord, depth), time, propagate);
	}

	void setTime(coord_t x, coord_t y, coord_t z, time_t time, bool propagate = true,
	             depth_t depth = 0)
	{
		setTime(derived().toCode(x, y, z, depth), time, propagate);
	}

	//
	// Propagation criteria
	//

	constexpr PropagationCriteria getTimePropagationCriteria() const noexcept
	{
		return time_prop_criteria_;
	}

	constexpr void setTimePropagationCriteria(PropagationCriteria time_prop_criteria,
	                                          bool propagate = true) noexcept
	{
		if (time_prop_criteria_ == time_prop_criteria) {
			return;
		}

		time_prop_criteria_ = time_prop_criteria;

		// Set all inner nodes to modified
		// FIXME: Possible to optimize this to only set the ones with children
		derived().setModified(1);

		if (propagate) {
			derived().updateModifiedNodes();
		}
	}

 protected:
	//
	// Derived
	//

	constexpr Derived& derived() { return *static_cast<Derived*>(this); }

	constexpr Derived const& derived() const { return *static_cast<Derived const*>(this); }

	//
	// Initilize root
	//

	void initRoot() { setTime(derived().getRoot(), derived().getRootIndex(), 0); }

	//
	// Get time
	//

	template <bool Single>
	static constexpr time_t getTime(TimeNode<Single> const node,
	                                index_t const index) noexcept
	{
		return node.getTime(index);
	}

	//
	// Set time
	//

	template <bool Single>
	static constexpr void setTime(TimeNode<Single>& node, time_t const time)
	{
		node.setTime(time);
	}

	template <bool Single>
	static constexpr void setTime(TimeNode<Single>& node, index_t const index,
	                              time_t const time)
	{
		node.setTime(index, time);
	}

	//
	// Update node
	//

	template <bool Single>
	constexpr void updateNode(TimeNode<Single> const&) noexcept
	{
	}

	template <bool Single, class T>
	void updateNode(TimeNode<Single>& node, index_field_t const indices, T const& children)
	{
		if constexpr (Single) {
			switch (time_prop_criteria_) {
				case PropagationCriteria::MIN:
					setTime(node, minTime(children));
					break;
				case PropagationCriteria::MAX:
					setTime(node, maxTime(children));
					break;
				case PropagationCriteria::MEAN:
					setTime(node, averageTime(children));
					break;
			}
		} else {
			for (index_t index = 0; children.size() != index; ++index) {
				if ((indices >> index) & index_field_t(1)) {
					switch (time_prop_criteria_) {
						case PropagationCriteria::MIN:
							setTime(node, index, minTime(children[index]));
							break;
						case PropagationCriteria::MAX:
							setTime(node, index, maxTime(children[index]));
							break;
						case PropagationCriteria::MEAN:
							setTime(node, index, averageTime(children[index]));
							break;
					}
				}
			}
		}
	}

	//
	// Min child time
	//

	constexpr time_t minTime(TimeNode<false> const& node) const
	{
		time_t min = std::numeric_limits<time_t>::max();
		for (time_t time : node.time) {
			min = std::min(min, time);
		}
		return min;
	}

	template <class T>
	constexpr time_t minTime(T const& nodes) const
	{
		time_t min = std::numeric_limits<time_t>::max();
		for (auto const& node : nodes) {
			min = std::min(min, node.time);
		}
		return min;
	}

	constexpr time_t minTime(std::vector<time_t> const& times) const
	{
		time_t min = std::numeric_limits::max();
		for (time_t time : times) {
			min = std::min(min, time);
		}
		return min;
	}

	//
	// Max child time
	//

	constexpr time_t maxTime(TimeNode<false> const& node) const
	{
		time_t max = std::numeric_limits<time_t>::lowest();
		for (time_t time : node.time) {
			max = std::max(max, time);
		}
		return max;
	}

	template <class T>
	constexpr time_t maxTime(T const& nodes) const
	{
		time_t max = std::numeric_limits<time_t>::lowest();
		for (auto const& node : nodes) {
			max = std::max(max, node.time);
		}
		return max;
	}

	constexpr time_t maxTime(std::vector<time_t> const& times) const
	{
		time_t max = std::numeric_limits::lowest();
		for (time_t time : times) {
			max = std::max(max, time);
		}
		return max;
	}

	//
	// Average child time
	//

	constexpr time_t averageTime(TimeNode<false> const& node) const
	{
		// FIXME: Make sure not overflow
		return std::accumulate(std::cbegin(node.time), std::cend(node.time), time_t(0),
		                       [](time_t cur, time_t x) { return cur + x; }) /
		       double(node.time.size());
	}

	template <class T>
	constexpr time_t averageTime(T const& nodes) const
	{
		// FIXME: Make sure not overflow
		return std::accumulate(std::cbegin(nodes), std::cend(nodes), time_t(0),
		                       [](time_t cur, auto const& node) { return cur + node.time; }) /
		       double(nodes.size());
	}

	constexpr time_t averageTime(std::vector<time_t> const& times) const
	{
		return std::reduce(std::begin(times), std::end(times)) / double(times.size());
	}

	//
	// Input/output (read/write)
	//

	static constexpr DataIdentifier getDataIdentifier() noexcept
	{
		return DataIdentifier::TIME;
	}

	static constexpr bool canReadData(DataIdentifier identifier) noexcept
	{
		return getDataIdentifier() == identifier;
	}

	template <class InputIt>
	static constexpr uint8_t isSingle() noexcept
	{
		using typename std::iterator_traits<InputIt>::value_type;
		using typename value_type::node_type;
		if constexpr (std::is_base_of_v(TimeNode<true>, node_type)) {
			return 1;
		} else {
			return 0;
		}
	}

	std::size_t numBitsSet(index_field_t const index_field)
	{
		std::size_t num = 0;
		for (std::size_t i = 0; 8 != i; ++i) {
			if ((index_field >> i) & index_field_t(1)) {
				++num;
			}
		}
		return num;
	}

	template <class InputIt>
	void readNodes(std::istream& in, InputIt first, InputIt last, std::size_t num_nodes)
	{
		uint8_t single;
		in.read(reinterpret_cast<char*>(&single), sizeof(single));

		if (single) {
			// Calculate it as we have single nodes here
			num_nodes = std::distance(first, last);
		}

		auto data = std::make_unique<time_t[]>(num_nodes);
		in.read(reinterpret_cast<char*>(data.get()),
		        num_nodes * sizeof(typename decltype(data)::element_type));

		if constexpr (isSingle<InputIt>()) {
			if (single) {
				for (std::size_t i = 0; first != last; ++i, std::advance(first, 1)) {
					setTime(first->node, data[i]);
				}
			} else {
				// FIXME: This is weird, as it is not the same if only some of nodes sent

				for (std::size_t i = 0; first != last; ++i, std::advance(first, 1)) {
					auto const num = numBitsSet(first->index_field);
					std::vector<time_t> times;
					times.reserve(num);
					for (index_t index = 0; 8 != index; ++index) {
						if ((first.index_field >> index) & index_field_t(1)) {
							times.push_back(getTime(first->node, index));
						}
					}
					switch (time_prop_criteria_) {
						case PropagationCriteria::MIN:
							setTime(first->node, minTime(times));
							break;
						case PropagationCriteria::MAX:
							setTime(first->node, maxTime(times));
							break;
						case PropagationCriteria::MEAN:
							setTime(first->node, averageTime(times));
							break;
					}
					setTime(first->node, average...);
				}
			}
		} else {
			if (single) {
				for (std::size_t i = 0; first != last; ++i, std::advance(first, 1)) {
					setTime(first->node, data[i]);
				}
			} else {
				for (std::size_t i = 0; first != last; std::advance(first, 1)) {
					if (std::numeric_limits<index_field_t>::max() == first->index_field) {
						for (time_t& time : first->node.time) {
							time = data[i];
							++i;
						}
					} else {
						for (index_t index = 0; first->node.time.size() != index; ++index) {
							if ((first.index_field >> index) & index_field_t(1)) {
								setTime(first->node, index, data[i]);
								++i;
							}
						}
					}
				}
			}
		}
	}

	template <class InputIt>
	void writeNodes(std::ostream& out, InputIt first, InputIt last, std::size_t num_nodes)
	{
		constexpr uint8_t single = isSingle<InputIt>();
		out.write(reinterpret_cast<char const*>(&single), sizeof(single));

		if constexpr (single) {
			// Calculate it as we have single nodes here
			num_nodes = std::distance(first, last);
		}

		auto data = std::make_unique<time_t[]>(num_nodes);

		if constexpr (single) {
			for (std::size_t i = 0; first != last; ++i, std::advance(first, 1)) {
				data[i] = getTime(first->node, 0);
			}
		} else {
			for (std::size_t i = 0; first != last; std::advance(first, 1)) {
				if (std::numeric_limits<index_field_t>::max() == first.index_field) {
					// All nodes should be written
					for (time_t const time : first->node.time) {
						data[i] = time;
						++i;
					}
				} else {
					for (index_t index = 0; first->node.time.size() != index; ++index) {
						if ((first.index_field >> index) & index_field_t(1)) {
							data[i] = getTime(first->node, index);
							++i;
						}
					}
				}
			}
		}

		out.write(reinterpret_cast<char const*>(data.get()),
		          num_nodes * sizeof(typename decltype(data)::element_type));
	}

 protected:
	// Propagation criteria
	PropagationCriteria time_prop_criteria_ = PropagationCriteria::MAX;
};
}  // namespace ufo::map

#endif  // UFO_MAP_TIME_MAP_BASE_H