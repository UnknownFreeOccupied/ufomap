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
#include <ufo/util/enum.h>

// STL
#include <cstdint>
#include <deque>
#include <functional>
#include <type_traits>
#include <utility>

namespace ufo::map
{
template <class Derived>
class SurfelMapBase
{
 public:
	//
	// Get surfel
	//

	SurfelReference surfel(Node node) const
	{
		return SurfelReference(derived().leafNode(node), node.index());
	}

	SurfelReference surfel(Code code) const
	{
		auto [node, depth] = derived().leafNodeAndDepth(code);
		return SurfelReference(node, code.index(depth));
	}

	SurfelReference surfel(Key key) const { return surfel(derived().toCode(key)); }

	SurfelReference surfel(Point coord, depth_t depth = 0) const
	{
		return surfel(derived().toCode(coord, depth));
	}

	SurfelReference surfel(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return surfel(derived().toCode(x, y, z, depth));
	}

	//
	// Set surfel
	//

	void setSurfel(Node node, Surfel const& surfel, bool propagate = true)
	{
		derived().apply(
		    node,
		    [&surfel](auto& node, index_t const index) {
			    node.sum[index] = surfel.sum();
			    node.sum_squares[index] = surfel.sumSquares();
			    node.num_points[index] = surfel.numPoints();
		    },
		    [&surfel](auto& node) {
			    node.sum.fill(surfel.sum());
			    node.sum_squares.fill(surfel.sumSquares());
			    node.num_points.fill(surfel.numPoints());
		    },
		    propagate);
	}

	void setSurfel(Code code, Surfel const& surfel, bool propagate = true)
	{
		derived().apply(
		    code,
		    [&surfel](auto& node, index_t const index) {
			    node.sum[index] = surfel.sum();
			    node.sum_squares[index] = surfel.sumSquares();
			    node.num_points[index] = surfel.numPoints();
		    },
		    [&surfel](auto& node) {
			    node.sum.fill(surfel.sum());
			    node.sum_squares.fill(surfel.sumSquares());
			    node.num_points.fill(surfel.numPoints());
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

	void insertSurfel(Surfel const& surfel, depth_t depth = 0, bool propagate = true)
	{
		if (surfel.empty()) {
			return;
		}

		insertSurfel(surfel.getSum() / surfel.getNumPoints(), surfel, depth, propagate);
	}

	void insertSurfel(Node node, Surfel const& surfel, bool propagate = true)
	{
		if (surfel.empty()) {
			return;
		}

		derived().apply(
		    node,
		    [&surfel](auto& node, index_t const index) { insertSurfel(node, index, surfel); },
		    [&surfel](auto& node) { insertSurfel(node, surfel); }, propagate);
	}

	void insertSurfel(Code code, Surfel const& surfel, bool propagate = true)
	{
		if (surfel.empty()) {
			return;
		}

		derived().apply(
		    code,
		    [&surfel](auto& node, index_t const index) { insertSurfel(node, index, surfel); },
		    [&surfel](auto& node) { insertSurfel(node, surfel); }, propagate);
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

	void eraseSurfel(Surfel const& surfel, depth_t depth = 0, bool propagate = true)
	{
		if (surfel.empty()) {
			return;
		}

		eraseSurfel(surfel.getSum() / surfel.getNumPoints(), surfel, depth, propagate);
	}

	void eraseSurfel(Node node, Surfel const& surfel, bool propagate = true)
	{
		if (surfel.empty()) {
			return;
		}

		derived().apply(
		    node,
		    [&surfel](auto& node, index_t const index) { eraseSurfel(node, index, surfel); },
		    [&surfel](auto& node) { eraseSurfel(node, surfel); }, propagate);
	}

	void eraseSurfel(Code code, Surfel const& surfel, bool propagate = true)
	{
		if (surfel.empty()) {
			return;
		}

		derived().apply(
		    code,
		    [&surfel](auto& node, index_t const index) { eraseSurfel(node, index, surfel); },
		    [&surfel](auto& node) { eraseSurfel(node, surfel); }, propagate);
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
		    [point](LeafNode const& node) { insertSurfelPoint(node, point); }, propagate);
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
			    [&points](auto& node, index_t const index) {
				    insertSurfelPoint(node, index, std::begin(points), std::end(points));
			    },
			    [&points](auto& node) {
				    insertSurfelPoint(node, std::begin(points), std::end(points));
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
			    eraseSurfelPoint(node, index, point);
		    },
		    [point](auto& node) { eraseSurfelPoint(node, point); }, propagate);
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
			    [&points](auto& node, index_t const index) {
				    eraseSurfelPoint(node, index, std::begin(points), std::end(points));
			    },
			    [&points](auto& node) {
				    eraseSurfelPoint(node, std::begin(points), std::end(points));
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
			    node.sum[index] = ...;
			    node.sum_squares[index] = ...;
			    node.num_points[index] = 0;
		    },
		    [](auto& node) {
			    node.sum.fill(...);
			    node.sum_squares.fill(...);
			    node.num_points.fill(0);
		    },
		    propagate);
	}

	void clearSurfel(Code code, bool propagate = true)
	{
		derived().apply(
		    code,
		    [](auto& node, index_t const index) {
			    node.sum[index] = ...;
			    node.sum_squares[index] = ...;
			    node.num_points[index] = 0;
		    },
		    [](auto& node) {
			    node.sum.fill(...);
			    node.sum_squares.fill(...);
			    node.num_points.fill(0);
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

	SurfelMap() = default;

	SurfelMap(SurfelMap const& other) = default;

	SurfelMap(SurfelMap&& other) = default;

	template <class Derived2>
	SurfelMap(SurfelMap<Derived2> const& other)
	{
	}

	//
	// Assignment operator
	//

	SurfelMap& operator=(SurfelMap const& rhs) = default;

	SurfelMap& operator=(SurfelMap&& rhs) = default;

	template <class Derived2>
	SurfelMap& operator=(SurfelMap<Derived2> const& rhs)
	{
		return *this;
	}

	//
	// Swap
	//

	void swap(SurfelMap&) noexcept {}

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
		derived().root().sum[derived().rootIndex()] = ...;
		derived().root().sum_squares[derived().rootIndex()] = ...;
		derived().root().num_points[derived().rootIndex()] = 0;
	}

	//
	// Update node
	//

	template <std::size_t N>
	void updateNode(SurfelNode<N>& node, index_t const index, SurfelNode<N> const& children)
	{
		node.sum[index] = ...;
		node.sum_squares[index] = ...;
		node.num_points[index] = 0;

		// TODO: Implement
	}

	//
	// Input/output (read/write)
	//

	static constexpr DataIdentifier dataIdentifier() noexcept
	{
		return DataIdentifier::SURFEL;
	}

	static constexpr bool canReadData(DataIdentifier identifier) noexcept
	{
		return dataIdentifier() == identifier;
	}

	template <class InputIt>
	std::size_t serializedSize(InputIt first, InputIt last) const
	{
		return std::iterator_traits<InputIt>::value_type::surfelSize() *
		       std::distance(first, last) *
		       (sizeof(math::Vector3<surfel_scalar_t>) +
		        sizeof(std::array<surfel_scalar_t, 6>) + sizeof(count_t));
	}

	template <class OutputIt>
	void readNodes(ReadBuffer& in, OutputIt first, OutputIt last)
	{
		for (; first != last; ++first) {
			if (first->index_field.all()) {
				in.read(first->node.sum.data(),
				        first->node.sum.size() *
				            sizeof(typename decltype(first->node.sum)::value_type));
				in.read(first->node.sum_squares.data(),
				        first->node.sum_squares.size() *
				            sizeof(typename decltype(first->node.sum_squares)::value_type));
				in.read(first->node.num_points.data(),
				        first->node.num_points.size() *
				            sizeof(typename decltype(first->node.num_points)::value_type));
			} else {
				decltype(first->node.sum) sum;
				decltype(first->node.sum_squares) sum_squares;
				decltype(first->node.num_points) num_points;
				in.read(sum.data(), sum.size() * sizeof(typename decltype(sum)::value_type));
				in.read(sum_squares.data(),
				        sum_squares.size() * sizeof(typename decltype(sum_squares)::value_type));
				in.read(num_points.data(),
				        num_points.size() * sizeof(typename decltype(num_points)::value_type));
				for (index_t i = 0; sum.size() != i; ++i) {
					if (first->index_field[i]) {
						first->node.sum[i] = sum[i];
						first->node.sum_squares[i] = sum_squares[i];
						first->node.num_points[i] = num_points[i];
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
			out.write(first->sum.data(),
			          first->sum.size() * sizeof(typename decltype(first->sum)::value_type));
			out.write(first->sum_squares.data(),
			          first->sum_squares.size() *
			              sizeof(typename decltype(first->sum_squares)::value_type));
			out.write(first->num_points.data(),
			          first->num_points.size() *
			              sizeof(typename decltype(first->num_points)::value_type));
		}
	}
};
}  // namespace ufo::map

#endif  // UFO_MAP_SURFEL_MAP_H