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

#ifndef UFO_MAP_SURFEL_MAP_H
#define UFO_MAP_SURFEL_MAP_H

// UFO
#include <ufo/algorithm/algorithm.h>
#include <ufo/map/surfel/surfel.h>
#include <ufo/map/surfel/surfel_node.h>
#include <ufo/map/surfel/surfel_predicate.h>
#include <ufo/map/surfel/surfel_util.h>
#include <ufo/util/enum.h>

// STL
#include <cstdint>
#include <deque>
#include <functional>
#include <type_traits>
#include <utility>

namespace ufo::map
{
template <class Derived, std::size_t N>
class SurfelMapBase
{
 public:
	//
	// Get surfel
	//

	[[nodiscard]] Surfel surfel(Node node) const
	{
		auto index = derived().leafNode(node).surfel_index[derived().dataIndex(node)];
		return NULL_INDEX == index ? Surfel() : surfel_[index];
	}

	[[nodiscard]] Surfel surfel(Code code) const
	{
		auto [node, depth] = derived().leafNodeAndDepth(code);
		auto index = node.surfel_index[depth];
		return NULL_INDEX == index ? Surfel() : surfel_[index];
	}

	[[nodiscard]] Surfel surfel(Key key) const { return surfel(derived().toCode(key)); }

	[[nodiscard]] Surfel surfel(Point coord, depth_t depth = 0) const
	{
		return surfel(derived().toCode(coord, depth));
	}

	[[nodiscard]] Surfel surfel(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return surfel(derived().toCode(x, y, z, depth));
	}

	//
	// Has surfel
	//

	[[nodiscard]] bool hasSurfel(Node node) const
	{
		auto index = derived().leafNode(node).surfel_index[derived().dataIndex(node)];
		return NULL_INDEX == index ? false : !surfel_[index].empty();
	}

	[[nodiscard]] bool hasSurfel(Code code) const
	{
		auto [node, depth] = derived().leafNodeAndDepth(code);
		auto index = node.surfel_index[depth];
		return NULL_INDEX == index ? false : !surfel_[index].empty();
	}

	[[nodiscard]] bool hasSurfel(Key key) const { return hasSurfel(derived().toCode(key)); }

	[[nodiscard]] bool hasSurfel(Point coord, depth_t depth = 0) const
	{
		return hasSurfel(derived().toCode(coord, depth));
	}

	[[nodiscard]] bool hasSurfel(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return hasSurfel(derived().toCode(x, y, z, depth));
	}

	//
	// Set surfel
	//

	void setSurfel(Node node, Surfel const& surfel, bool propagate = true)
	{
		derived().apply(
		    node,
		    [sum = surfel.sum(), sum_squares = surfel.sumSquares(),
		     num_points = surfel.numPoints()](auto& node, index_t const index) {
			    node.sum[index] = sum;
			    node.sum_squares[index] = sum_squares;
			    node.num_points[index] = num_points;
		    },
		    [sum = surfel.sum(), sum_squares = surfel.sumSquares(),
		     num_points = surfel.numPoints()](auto& node) {
			    node.sum.fill(sum);
			    node.sum_squares.fill(sum_squares);
			    node.num_points.fill(num_points);
		    },
		    propagate);
	}

	void setSurfel(Code code, Surfel const& surfel, bool propagate = true)
	{
		derived().apply(
		    code,
		    [sum = surfel.sum(), sum_squares = surfel.sumSquares(),
		     num_points = surfel.numPoints()](auto& node, index_t const index) {
			    node.sum[index] = sum;
			    node.sum_squares[index] = sum_squares;
			    node.num_points[index] = num_points;
		    },
		    [sum = surfel.sum(), sum_squares = surfel.sumSquares(),
		     num_points = surfel.numPoints()](auto& node) {
			    node.sum.fill(sum);
			    node.sum_squares.fill(sum_squares);
			    node.num_points.fill(num_points);
		    },
		    propagate);
	}

	void setSurfel(Key key, Surfel const& surfel, bool propagate = true)
	{
		setSurfel(derived().toCode(key), surfel, propagate);
	}

	void setSurfel(Point coord, Surfel const& surfel, depth_t depth = 0,
	               bool propagate = true)
	{
		setSurfel(derived().toCode(coord, depth), surfel, propagate);
	}

	void setSurfel(coord_t x, coord_t y, coord_t z, Surfel const& surfel, depth_t depth = 0,
	               bool propagate = true)
	{
		setSurfel(derived().toCode(x, y, z, depth), surfel, propagate);
	}

	//
	// Insert surfel
	//

	void insertSurfel(Node node, Surfel const& surfel, bool propagate = true)
	{
		if (surfel.empty()) {
			return;
		}

		derived().apply(
		    node,
		    [sum = surfel.sum(), sum_squares = surfel.sumSquares(),
		     num_points = surfel.numPoints()](auto& node, index_t const index) {
			    surfel::add(node.sum[index], node.sum_squares[index], node.num_points[index],
			                sum, sum_squares, num_points);
		    },
		    [sum = surfel.sum(), sum_squares = surfel.sumSquares(),
		     num_points = surfel.numPoints()](auto& node) {
			    for (index_t i = 0; node.surfelSize() != i; ++i) {
				    surfel::add(node.sum[i], node.sum_squares[i], node.num_points[i], sum,
				                sum_squares, num_points);
			    }
		    },
		    propagate);
	}

	void insertSurfel(Code code, Surfel const& surfel, bool propagate = true)
	{
		if (surfel.empty()) {
			return;
		}

		derived().apply(
		    code,
		    [sum = surfel.sum(), sum_squares = surfel.sumSquares(),
		     num_points = surfel.numPoints()](auto& node, index_t const index) {
			    surfel::add(node.sum[index], node.sum_squares[index], node.num_points[index],
			                sum, sum_squares, num_points);
		    },
		    [sum = surfel.sum(), sum_squares = surfel.sumSquares(),
		     num_points = surfel.numPoints()](auto& node) {
			    for (index_t i = 0; node.surfelSize() != i; ++i) {
				    surfel::add(node.sum[i], node.sum_squares[i], node.num_points[i], sum,
				                sum_squares, num_points);
			    }
		    },
		    propagate);
	}

	void insertSurfel(Key key, Surfel const& surfel, bool propagate = true)
	{
		insertSurfel(derived().toCode(key), surfel, propagate);
	}

	void insertSurfel(Point coord, Surfel const& surfel, depth_t depth = 0,
	                  bool propagate = true)
	{
		insertSurfel(derived().toCode(coord, depth), surfel, propagate);
	}

	void insertSurfel(coord_t x, coord_t y, coord_t z, Surfel const& surfel,
	                  depth_t depth = 0, bool propagate = true)
	{
		insertSurfel(derived().toCode(x, y, z, depth), surfel, propagate);
	}

	//
	// Erase surfel
	//

	void eraseSurfel(Node node, Surfel const& surfel, bool propagate = true)
	{
		if (surfel.empty()) {
			return;
		}

		derived().apply(
		    node,
		    [sum = surfel.sum(), sum_squares = surfel.sumSquares(),
		     num_points = surfel.numPoints()](auto& node, index_t const index) {
			    surfel::remove(node.sum[index], node.sum_squares[index], node.num_points[index],
			                   sum, sum_squares, num_points);
		    },
		    [sum = surfel.sum(), sum_squares = surfel.sumSquares(),
		     num_points = surfel.numPoints()](auto& node) {
			    for (index_t i = 0; node.surfelSize() != i; ++i) {
				    surfel::remove(node.sum[i], node.sum_squares[i], node.num_points[i], sum,
				                   sum_squares, num_points);
			    }
		    },
		    propagate);
	}

	void eraseSurfel(Code code, Surfel const& surfel, bool propagate = true)
	{
		if (surfel.empty()) {
			return;
		}

		derived().apply(
		    code,
		    [sum = surfel.sum(), sum_squares = surfel.sumSquares(),
		     num_points = surfel.numPoints()](auto& node, index_t const index) {
			    surfel::remove(node.sum[index], node.sum_squares[index], node.num_points[index],
			                   sum, sum_squares, num_points);
		    },
		    [sum = surfel.sum(), sum_squares = surfel.sumSquares(),
		     num_points = surfel.numPoints()](auto& node) {
			    for (index_t i = 0; node.surfelSize() != i; ++i) {
				    surfel::remove(node.sum[i], node.sum_squares[i], node.num_points[i], sum,
				                   sum_squares, num_points);
			    }
		    },
		    propagate);
	}

	void eraseSurfel(Key key, Surfel const& surfel, bool propagate = true)
	{
		eraseSurfel(derived().toCode(key), surfel, propagate);
	}

	void eraseSurfel(Point coord, Surfel const& surfel, depth_t depth = 0,
	                 bool propagate = true)
	{
		eraseSurfel(derived().toCode(coord, depth), surfel, propagate);
	}

	void eraseSurfel(coord_t x, coord_t y, coord_t z, Surfel const& surfel,
	                 depth_t depth = 0, bool propagate = true)
	{
		eraseSurfel(derived().toCode(x, y, z, depth), surfel, propagate);
	}

	//
	// Insert surfel point
	//

	void insertSurfelPoint(Point point, depth_t depth = 0, bool propagate = true)
	{
		derived().apply(
		    derived().toCode(point, depth),
		    [point](auto& node, index_t const index) {
			    surfel::addPoint(node.sum[index], node.sum_squares[index],
			                     node.num_points[index], point);
		    },
		    [point](auto& node) {
			    for (index_t i = 0; node.surfelSize() != i; ++i) {
				    surfel::addPoint(node.sum[i], node.sum_squares[i], node.num_points[i], point);
			    }
		    },
		    propagate);
	}

	template <class InputIt>
	void insertSurfelPoint(InputIt first, InputIt last, depth_t depth = 0,
	                       bool propagate = true)
	{
		std::vector<std::pair<Code, Point>> codes;
		codes.reserve(std::distance(first, last));

		for (; first != last; ++first) {
			codes.emplace_back(derived().toCode(*first, depth), *first);
		}

		std::sort(std::begin(codes), std::end(codes),
		          [](auto&& a, auto&& b) { return a.first < b.first; });

		std::vector<Point> points;
		for (auto it = std::cbegin(codes); it != std::cend(codes);) {
			auto it_end = std::find_if(std::next(it), std::cend(codes),
			                           [it](auto&& v) { return it->first != v.first; });

			points.reserve(std::distance(it, it_end));
			std::transform(it, it_end, std::back_insert_iterator(points),
			               [](auto&& elem) { return elem.second; });

			derived().apply(
			    it->first,
			    [first = std::cbegin(points), last = std::cend(points)](auto& node,
			                                                            index_t const index) {
				    surfel::addPoint(node.sum[index], node.sum_squares[index],
				                     node.num_points[index], first, last);
			    },
			    [first = std::cbegin(points), last = std::cend(points)](auto& node) {
				    for (index_t i = 0; node.surfelSize() != i; ++i) {
					    surfel::addPoint(node.sum[i], node.sum_squares[i], node.num_points[i],
					                     first, last);
				    }
			    },
			    false);

			points.clear();
			it = it_end;
		}

		if (propagate) {
			derived().updateModifiedNodes();
		}
	}

	void insertSurfelPoint(std::initializer_list<Point> points, depth_t depth = 0,
	                       bool propagate = true)
	{
		insertSurfelPoint(std::begin(points), std::end(points), depth, propagate);
	}

	//
	// Erase surfel point
	//

	void eraseSurfelPoint(Point point, depth_t depth = 0, bool propagate = true)
	{
		derived().apply(
		    derived().toCode(point, depth),
		    [point](auto& node, index_t const index) {
			    surfel::removePoint(node.sum[index], node.sum_squares[index],
			                        node.num_points[index], point);
		    },
		    [point](auto& node) {
			    for (index_t i = 0; node.surfelSize() != i; ++i) {
				    surfel::removePoint(node.sum[i], node.sum_squares[i], node.num_points[i],
				                        point);
			    }
		    },
		    propagate);
	}

	template <class InputIt>
	void eraseSurfelPoint(InputIt first, InputIt last, depth_t depth = 0,
	                      bool propagate = true)
	{
		std::vector<std::pair<Code, Point>> codes;
		codes.reserve(std::distance(first, last));

		for (; first != last; ++first) {
			codes.emplace_back(derived().toCode(*first, depth), *first);
		}

		std::sort(std::begin(codes), std::end(codes),
		          [](auto const& a, auto const& b) { return a.first < b.first; });

		std::vector<Point> points;
		for (auto it = std::cbegin(codes); it != std::cend(codes);) {
			auto it_end = std::find_if(std::next(it), std::cend(codes),
			                           [it](auto&& v) { return it->first != v.first; });

			points.reserve(std::distance(it, it_end));
			std::transform(it, it_end, std::back_insert_iterator(points),
			               [](auto&& elem) { return elem.second; });

			derived().apply(
			    it->first,
			    [first = std::cbegin(points), last = std::cend(points)](auto& node,
			                                                            index_t const index) {
				    surfel::removePoint(node.sum[index], node.sum_squares[index],
				                        node.num_points[index], first, last);
			    },
			    [first = std::cbegin(points), last = std::cend(points)](auto& node) {
				    for (index_t i = 0; node.surfelSize() != i; ++i) {
					    surfel::removePoint(node.sum[i], node.sum_squares[i], node.num_points[i],
					                        first, last);
				    }
			    },
			    false);

			points.clear();
			it = it_end;
		}

		if (propagate) {
			derived().updateModifiedNodes();
		}
	}

	void eraseSurfelPoint(std::initializer_list<Point> points, depth_t depth = 0,
	                      bool propagate = true)
	{
		eraseSurfelPoint(std::begin(points), std::end(points), depth, propagate);
	}

	//
	// Clear surfel
	//

	void clearSurfel(Node node, bool propagate = true)
	{
		derived().apply(
		    node,
		    [](auto& node, index_t const index) {
			    surfel::clear(node.sum[index], node.sum_squares[index], node.num_points[index]);
		    },
		    [](auto& node) {
			    for (index_t i = 0; node.surfelSize() != i; ++i) {
				    surfel::clear(node.sum[i], node.sum_squares[i], node.num_points[i]);
			    }
		    },
		    propagate);
	}

	void clearSurfel(Code code, bool propagate = true)
	{
		derived().apply(
		    code,
		    [](auto& node, index_t const index) {
			    surfel::clear(node.sum[index], node.sum_squares[index], node.num_points[index]);
		    },
		    [](auto& node) {
			    for (index_t i = 0; node.surfelSize() != i; ++i) {
				    surfel::clear(node.sum[i], node.sum_squares[i], node.num_points[i]);
			    }
		    },
		    propagate);
	}

	void clearSurfel(Key key, bool propagate = true)
	{
		clearSurfel(derived().toCode(key), propagate);
	}

	void clearSurfel(Point coord, depth_t depth = 0, bool propagate = true)
	{
		clearSurfel(derived().toCode(coord, depth), propagate);
	}

	void clearSurfel(coord_t x, coord_t y, coord_t z, depth_t depth = 0,
	                 bool propagate = true)
	{
		clearSurfel(derived().toCode(x, y, z, depth), propagate);
	}

 protected:
	//
	// Constructors
	//

	SurfelMapBase() = default;

	SurfelMapBase(SurfelMapBase const& other) = default;

	SurfelMapBase(SurfelMapBase&& other) = default;

	template <class Derived2>
	SurfelMapBase(SurfelMapBase<Derived2, N> const& other)
	{
	}

	//
	// Assignment operator
	//

	SurfelMapBase& operator=(SurfelMapBase const& rhs) = default;

	SurfelMapBase& operator=(SurfelMapBase&& rhs) = default;

	template <class Derived2>
	SurfelMapBase& operator=(SurfelMapBase<Derived2, N> const& rhs)
	{
		return *this;
	}

	//
	// Swap
	//

	void swap(SurfelMapBase& other) noexcept {}

	//
	// Derived
	//

	constexpr Derived& derived() { return *static_cast<Derived*>(this); }

	constexpr Derived const& derived() const { return *static_cast<Derived const*>(this); }

	//
	// Initilize root
	//

	void initRoot()
	{
		derived().root().surfel_index[derived().rootIndex()] = NULL_INDEX;
		surfel_.clear();
		while (!free_indices_.empty()) {
			free_indices_.pop();
		}
	}

	//
	// Fill
	//

	template <std::size_t N>
	void fill(SurfelNode<N>& node, SurfelNode<N> const& parent, index_t const index)
	{
		if (NULL_INDEX == parent.surfel_index[index]) {
			node.surfel_index.fill(NULL_INDEX);
		} else {
			Surfel surfel = surfel_[parent.surfel_index[index]];
			for (auto& x : node.surfel_index) {
				x = createSurfel(surfel);
			}
		}
	}

	//
	// Clear
	//

	template <std::size_t N>
	void clear(SurfelNode<N>& node)
	{
		for (auto x : node.surfel_index) {
			if (NULL_INDEX != x) {
				surfel_[x].clear();
				free_indices_.push(x);
			}
		}
		node.surfel_index.fill(NULL_INDEX);
	}

	//
	// Update node
	//

	template <std::size_t N>
	void updateNode(SurfelNode<N>& node, index_t const index, SurfelNode<N> const& children)
	{
		if (all_of(children.surfel_index, [](auto x) { return NULL_INDEX == x; })) {
			auto s_index = node.surfel_index[index];
			if (NULL_INDEX != s_index) {
				surfel_[s_index].clear();
				free_indices_.push(s_index);
				node.surfel_index[index] = NULL_INDEX;
			}
			return;
		}

		if (NULL_INDEX == node.surfel_index[index]) {
			node.surfel_index[index] = createSurfel();
		} else {
			surfel_[node.surfel_index[index]].clear();
		}

		auto& s = surfel_[node.surfel_index[index]];
		for (auto x : children.surfel_index) {
			if (NULL_INDEX != x) {
				s += surfel_[x];
			}
		}
	}

	//
	// Is collapsible
	//

	template <std::size_t N>
	[[nodiscard]] bool isCollapsible(SurfelNode<N> const& node) const
	{
		if (all_of(node.surfel_index, [](auto x) { return NULL_INDEX == x; })) {
			return true;
		}

		if (any_of(node.surfel_index, [](auto x) { return NULL_INDEX == x; })) {
			return false;
		}

		auto num = surfel_[node.surfel_index[0]].numPoints();
		for (std::size_t i = 1; N != i; ++i) {
			if (surfel_[node.surfel_index[i]].numPoints() != num) {
				return false;
			}
		}

		return true;
	}

	//
	// Create surfel
	//

	[[nodiscard]] index_t createSurfel()
	{
		index_t index;
		if (free_indices_.empty()) {
			index = surfel_.size();
			surfel_.emplace_back();
		} else {
			index = free_indices_.top();
			free_indices_.pop();
		}
		return index;
	}

	[[nodiscard]] index_t createSurfel(Surfel const& surfel)
	{
		index_t index;
		if (free_indices_.empty()) {
			index = surfel_.size();
			surfel_.push_back(surfel);
		} else {
			index = free_indices_.top();
			free_indices_.pop();
			surfel_[index] = surfel;
		}
		return index;
	}

	//
	// Input/output (read/write)
	//

	static constexpr MapType mapType() noexcept { return MapType::SURFEL; }

	static constexpr bool canReadData(MapType mt) noexcept { return mapType() == mt; }

	template <class InputIt>
	std::size_t serializedSize(InputIt first, InputIt last) const
	{
		// TODO: Implement
		return std::iterator_traits<InputIt>::value_type::surfelSize() *
		       std::distance(first, last) *
		       (sizeof(math::Vector3<surfel_scalar_t>) +
		        sizeof(std::array<surfel_scalar_t, 6>) + sizeof(count_t));
	}

	template <class OutputIt>
	void readNodes(ReadBuffer& in, OutputIt first, OutputIt last)
	{
		IndexField indices;
		for (; first != last; ++first) {
			in.read(&indices, sizeof(indices));
			if (first->indices.all()) {
				// TODO: Implement
				in.read(first->node->sum.data(),
				        first->node->sum.size() *
				            sizeof(typename decltype(first->node->sum)::value_type));
				in.read(first->node->sum_squares.data(),
				        first->node->sum_squares.size() *
				            sizeof(typename decltype(first->node->sum_squares)::value_type));
				in.read(first->node->num_points.data(),
				        first->node->num_points.size() *
				            sizeof(typename decltype(first->node->num_points)::value_type));
			} else {
				// TODO: Implement
				decltype(first->node->sum) sum;
				decltype(first->node->sum_squares) sum_squares;
				decltype(first->node->num_points) num_points;
				in.read(sum.data(), sum.size() * sizeof(typename decltype(sum)::value_type));
				in.read(sum_squares.data(),
				        sum_squares.size() * sizeof(typename decltype(sum_squares)::value_type));
				in.read(num_points.data(),
				        num_points.size() * sizeof(typename decltype(num_points)::value_type));
				for (index_t i = 0; sum.size() != i; ++i) {
					if (first->indices[i]) {
						first->node->sum[i] = sum[i];
						first->node->sum_squares[i] = sum_squares[i];
						first->node->num_points[i] = num_points[i];
					}
				}
			}
		}
	}

	template <class InputIt>
	void writeNodes(WriteBuffer& out, InputIt first, InputIt last) const
	{
		out.reserve(out.size() + serializedSize(first, last));
		IndexField indices;
		for (; first != last; ++first) {
			for (index_t index{}; auto x : first->surfel_index) {
				indices[index] = NULL_INDEX != x;
			}
			out.write(&indices, sizeof(indices));
			for (auto x : first->surfel_index) {
				if (NULL_INDEX != x) {
					out.write(&surfel_[x], sizeof(Surfel));
				}
			}
		}
	}

 private:
	std::deque<std::array<index_t, N>> indices_;
	std::deque<Surfel> surfel_;
	std::stack<index_t> free_surfel_;

	template <class Derived2, std::size_t N2>
	friend class SurfelMapBase;
};
}  // namespace ufo::map

#endif  // UFO_MAP_SURFEL_MAP_H