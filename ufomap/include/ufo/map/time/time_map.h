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
#include <ufo/map/time/time_node.h>
#include <ufo/map/time/time_predicate.h>
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

	time_t getTime(Point coord, depth_t depth = 0) const
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

	void setTime(Point coord, time_t time, bool propagate = true, depth_t depth = 0)
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
	constexpr void updateNode(TimeNode<Single> const&, index_field_t const) noexcept
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

	template <class InputIt>
	constexpr time_t minTime(InputIt first, InputIt last) const
	{
		time_t min = std::numeric_limits::max();
		for (; first != last; ++first) {
			min = std::min(min, *first);
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

	template <class InputIt>
	constexpr time_t maxTime(InputIt first, InputIt last) const
	{
		time_t max = std::numeric_limits::lowest();
		for (; first != last; ++first) {
			max = std::max(max, *first);
		}
		return max;
	}

	//
	// Average child time
	//

	static constexpr time_t averageTime(TimeNode<false> const& node)
	{
		return std::reduce(std::cbegin(node.time), std::cend(node.time), 0.0) /
		       double(node.time.size());
	}

	template <class T>
	static constexpr time_t averageTime(T const& nodes)
	{
		return std::accumulate(std::cbegin(nodes), std::cend(nodes), 0.0,
		                       [](double cur, auto const& node) { return cur + node.time; }) /
		       double(nodes.size());
	}

	template <class InputIt>
	static constexpr time_t averageTime(InputIt first, InputIt last)
	{
		return std::reduce(first, last, 0.0) / double(std::distance(first, last));
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

	template <class InputIt>
	void readNodes(std::istream& in, InputIt first, InputIt last, std::size_t num_nodes)
	{
		uint8_t single;
		in.read(reinterpret_cast<char*>(&single), sizeof(single));

		num_nodes = std::distance(first, last);
		if (!single) {
			num_nodes *= 8;
		}

		auto data = std::make_unique<time_t[]>(num_nodes);
		in.read(reinterpret_cast<char*>(data.get()),
		        num_nodes * sizeof(typename decltype(data)::element_type));

		if constexpr (isSingle<InputIt>()) {
			if (single) {
				for (std::size_t i = 0; first != last; ++i, ++first) {
					setTime(first->node, data[i]);
				}
			} else {
				auto const time_prop_criteria = time_prop_criteria_;
				for (auto d_first = data.get(); first != last; ++first) {
					auto d_last = std::next(d_first, 8);
					switch (time_prop_criteria) {
						case PropagationCriteria::MIN:
							setTime(first->node, minTime(d_first, d_last));
							break;
						case PropagationCriteria::MAX:
							setTime(first->node, maxTime(d_first, d_last));
							break;
						case PropagationCriteria::MEAN:
							setTime(first->node, averageTime(d_first, d_last));
							break;
					}
					d_first = d_last;
				}
			}
		} else {
			if (single) {
				for (std::size_t i = 0; first != last; ++i, ++first) {
					setTime(first->node, data[i]);
				}
			} else {
				for (std::size_t i = 0; first != last; ++first) {
					if (std::numeric_limits<index_field_t>::max() == first->index_field) {
						auto d_first = &(data[i]);
						auto d_last = std::next(d_first, first->node.time.size());
						std::copy(d_first, d_last, first->node.time.data());
						i += first->node.time.size();
					} else {
						for (index_t index = 0; first->node.time.size() != index; ++i, ++index) {
							if ((first.index_field >> index) & index_field_t(1)) {
								setTime(first->node, index, data[i]);
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
		constexpr uint8_t const single = isSingle<InputIt>();
		out.write(reinterpret_cast<char const*>(&single), sizeof(single));

		num_nodes = std::distance(first, last);
		if constexpr (!single) {
			num_nodes *= 8;
		}

		auto data = std::make_unique<time_t[]>(num_nodes);
		if constexpr (single) {
			for (std::size_t i = 0; first != last; ++i, ++first) {
				data[i] = getTime(first->node, 0);
			}
		} else {
			for (auto d = data.get(); first != last; ++first) {
				d = std::copy(std::cbegin(first->node.time), std::cend(first->node.time), d);
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