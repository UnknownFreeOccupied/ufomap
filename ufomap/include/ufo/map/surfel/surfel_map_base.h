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

#ifndef UFO_MAP_SURFEL_MAP_BASE_H
#define UFO_MAP_SURFEL_MAP_BASE_H

// UFO
#include <ufo/algorithm/algorithm.h>
#include <ufo/map/code/code_unordered_map.h>
#include <ufo/map/predicate/surfel.h>
// #include <ufo/map/types.h>

// STL
#include <cstdint>
#include <functional>
#include <type_traits>
#include <utility>

namespace ufo::map
{
template <class Derived, class LeafNode, class InnerNode>
class SurfelMapBase
{
 public:
	using Surfel = typename LeafNode::surfel_type;

 public:
	//
	// Get surfel
	//

	Surfel getSurfel(Node node) const { return getSurfel(derived().getLeafNode(node)); }

	Surfel getSurfel(Code code) const { return getSurfel(derived().getLeafNode(code)); }

	Surfel getSurfel(Key key) const { return getSurfel(Derived::toCode(key)); }

	Surfel getSurfel(Point3 coord, depth_t depth = 0) const
	{
		return getSurfel(derived().toCode(coord, depth));
	}

	Surfel getSurfel(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return getSurfel(derived().toCode(x, y, z, depth));
	}

	//
	// Has surfel
	//

	[[nodiscard]] bool hasSurfel(Node node) const
	{
		return hasSurfel(derived().getLeafNode(node));
	}

	[[nodiscard]] bool hasSurfel(Code code) const
	{
		return hasSurfel(derived().getLeafNode(code));
	}

	[[nodiscard]] bool hasSurfel(Key key) const { return hasSurfel(Derived::toCode(key)); }

	[[nodiscard]] bool hasSurfel(Point3 coord, depth_t depth = 0) const
	{
		return hasSurfel(derived().toCode(coord, depth));
	}

	[[nodiscard]] bool hasSurfel(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return hasSurfel(derived().toCode(x, y, z, depth));
	}

	//
	// Get number of surfel points
	//

	[[nodiscard]] std::size_t getNumSurfelPoints(Node node) const
	{
		return getNumSurfelPoints(derived().getLeafNode(node));
	}

	[[nodiscard]] std::size_t getNumSurfelPoints(Code code) const
	{
		return getNumSurfelPoints(derived().getLeafNode(code));
	}

	[[nodiscard]] std::size_t getNumSurfelPoints(Key key) const
	{
		return getNumSurfelPoints(derived().toCode(key));
	}

	[[nodiscard]] std::size_t getNumSurfelPoints(Point3 coord, depth_t depth = 0) const
	{
		return getNumSurfelPoints(derived().toCode(coord, depth));
	}

	[[nodiscard]] std::size_t getNumSurfelPoints(coord_t x, coord_t y, coord_t z,
	                                             depth_t depth = 0) const
	{
		return getNumSurfelPoints(derived().toCode(x, y, z, depth));
	}

	//
	// Set surfel
	//

	void setSurfel(Node node, Surfel const& surfel, bool propagate = true)
	{
		derived().apply(
		    node, [&surfel](auto&& node) { setSurfel(node, surfel); }, propagate);
	}

	void setSurfel(Code code, Surfel const& surfel, bool propagate = true)
	{
		derived().apply(
		    code, [&surfel](auto&& node) { setSurfel(node, surfel); }, propagate);
	}

	void setSurfel(Key key, Surfel const& surfel, bool propagate = true)
	{
		setSurfel(derived().toCode(key), surfel, propagate);
	}

	void setSurfel(Point3 coord, Surfel const& surfel, depth_t depth = 0,
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

		insertSurfel(surfel.getSum() / surfel.numPoints(), surfel, depth, propagate);
	}

	void insertSurfel(Node node, Surfel const& surfel, bool propagate = true)
	{
		if (surfel.empty()) {
			return;
		}

		derived().apply(
		    node, [&surfel](LeafNode& node) { insertSurfel(node, surfel); }, propagate);
	}

	void insertSurfel(Code code, Surfel const& surfel, bool propagate = true)
	{
		if (surfel.empty()) {
			return;
		}

		derived().apply(
		    code, [&surfel](LeafNode& node) { insertSurfel(node, surfel); }, propagate);
	}

	void insertSurfel(Key key, Surfel const& surfel, bool propagate = true)
	{
		insertSurfel(derived().toCode(key), surfel, propagate);
	}

	void insertSurfel(Point3 coord, Surfel const& surfel, depth_t depth = 0,
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

		eraseSurfel(surfel.getSum() / surfel.numPoints(), surfel, depth, propagate);
	}

	void eraseSurfel(Node node, Surfel const& surfel, bool propagate = true)
	{
		if (surfel.empty()) {
			return;
		}

		derived().apply(
		    node, [&surfel](LeafNode& node) { eraseSurfel(node, surfel); }, propagate);
	}

	void eraseSurfel(Code code, Surfel const& surfel, bool propagate = true)
	{
		if (surfel.empty()) {
			return;
		}

		derived().apply(
		    code, [&surfel](LeafNode& node) { eraseSurfel(node, surfel); }, propagate);
	}

	void eraseSurfel(Key key, Surfel const& surfel, bool propagate = true)
	{
		eraseSurfel(derived().toCode(key), surfel, propagate);
	}

	void eraseSurfel(Point3 coord, Surfel const& surfel, depth_t depth = 0,
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

	void insertSurfelPoint(Point3 point, depth_t depth = 0, bool propagate = true)
	{
		derived().apply(
		    derived().toCode(point, depth),
		    [point](LeafNode const& node) { insertSurfelPoint(node, point); }, propagate);
	}

	template <class InputIt>
	void insertSurfelPoint(InputIt first, InputIt last, depth_t depth = 0,
	                       bool propagate = true)
	{
		std::vector<std::pair<Code, Point3>> codes;
		codes.reserve(std::distance(first, last));

		for (; first != last; ++first) {
			codes.emplace_back(derived().toCode(*first, depth), *first);
		}

		std::sort(std::begin(codes), std::end(codes),
		          [](auto&& a, auto&& b) { return a.first < b.first; });

		std::vector<Point3> points;
		for (auto it = std::cbegin(codes); it != std::cend(codes);) {
			auto it_end = std::find_if(std::next(it), std::cend(codes),
			                           [it](auto&& v) { return it->first != v.first; });

			points.reserve(std::distance(it, it_end));
			std::transform(it, it_end, std::back_insert_iterator(points),
			               [](auto&& elem) { return elem.second; });

			derived().apply(
			    it->first,
			    [&points](LeafNode& node) {
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

	void insertSurfelPoint(std::initializer_list<Point3> points, depth_t depth = 0,
	                       bool propagate = true)
	{
		insertSurfelPoint(std::begin(points), std::end(points), depth, propagate);
	}

	//
	// Erase surfel point
	//

	void eraseSurfelPoint(Point3 point, depth_t depth = 0, bool propagate = true)
	{
		derived().apply(
		    derived().toCode(point, depth),
		    [point](auto&& node) { eraseSurfelPoint(node, point); }, propagate);
	}

	template <class InputIt>
	void eraseSurfelPoint(InputIt first, InputIt last, depth_t depth = 0,
	                      bool propagate = true)
	{
		std::vector<std::pair<Code, Point3>> codes;
		codes.reserve(std::distance(first, last));

		for (; first != last; ++first) {
			codes.emplace_back(derived().toCode(*first, depth), *first);
		}

		std::sort(std::begin(codes), std::end(codes),
		          [](auto&& a, auto&& b) { return a.first < b.first; });

		std::vector<Point3> points;
		for (auto it = std::cbegin(codes); it != std::cend(codes);) {
			auto it_end = std::find_if(std::next(it), std::cend(codes),
			                           [it](auto&& v) { return it->first != v.first; });

			points.reserve(std::distance(it, it_end));
			std::transform(it, it_end, std::back_insert_iterator(points),
			               [](auto&& elem) { return elem.second; });

			derived().apply(
			    it->first,
			    [&points](LeafNode& node) {
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

	void eraseSurfelPoint(std::initializer_list<Point3> points, depth_t depth = 0,
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
		    node, [](LeafNode& node) { clearSurfel(node); }, propagate);
	}

	void clearSurfel(Code code, bool propagate = true)
	{
		derived().apply(
		    code, [](LeafNode& node) { clearSurfel(node); }, propagate);
	}

	void clearSurfel(Key key, bool propagate = true)
	{
		clearSurfel(derived().toCode(key), propagate);
	}

	void clearSurfel(Point3 coord, depth_t depth = 0, bool propagate = true)
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
	// Derived
	//

	constexpr Derived& derived() { return *static_cast<Derived*>(this); }

	constexpr Derived const& derived() const { return *static_cast<Derived const*>(this); }

	//
	// Initilize root
	//

	void initRoot() { clearSurfel(derived().getRoot()); }

	//
	// Get surfel
	//

	static constexpr Surfel& getSurfel(LeafNode& node) { return node.surfel; }

	static constexpr Surfel const& getSurfel(LeafNode const& node) { return node.surfel; }

	//
	// Set surfel
	//

	static constexpr void setSurfel(LeafNode& node, Surfel const& surfel) noexcept
	{
		node.surfel = surfel;
	}

	template <class... Args>
	static constexpr void setSurfel(LeafNode& node, Args&&... args)
	{
		node.surfel = Surfel(std::forward<Args>(args)...);
	}

	//
	// Clear surfel
	//

	static constexpr void clearSurfel(LeafNode& node) noexcept { node.surfel.clear(); }

	//
	// Has surfel
	//

	static constexpr bool hasSurfel(LeafNode const& node) noexcept
	{
		return 0 != getNumSurfelPoints(node);
	}

	//
	// Get num surfel points
	//

	static constexpr std::size_t getNumSurfelPoints(LeafNode const& node) noexcept
	{
		return node.surfel.numPoints();
	}

	//
	// Insert surfel
	//

	static void insertSurfel(LeafNode& node, Surfel const& surfel)
	{
		getSurfel(node).addSurfel(surfel);
	}

	template <class InputIt>
	static void insertSurfel(LeafNode& node, InputIt first, InputIt last)
	{
		for (; first != last; ++first) {
			getSurfel(node).addSurfel(*first);
		}
	}

	//
	// Erase surfel
	//

	static void eraseSurfel(LeafNode& node, Surfel const& surfel)
	{
		getSurfel(node).removeSurfel(surfel);
	}

	//
	// Insert surfel point
	//

	static void insertSurfelPoint(LeafNode& node, Point3 point)
	{
		getSurfel(node).addPoint(point);
	}

	template <class InputIt>
	static void insertSurfelPoint(LeafNode& node, InputIt first, InputIt last)
	{
		getSurfel(node).addPoint(first, last);
	}

	//
	// Erase surfel point
	//

	static void eraseSurfelPoint(LeafNode& node, Point3 point)
	{
		getSurfel(node).removePoint(point);
	}

	template <class InputIt>
	static void eraseSurfelPoint(LeafNode& node, InputIt first, InputIt last)
	{
		getSurfel(node).removePoint(first, last);
	}

	//
	// Update node
	//

	void updateNode(InnerNode& node, depth_t depth)
	{
		clearSurfel(node);

		if (1 == depth) {
			for (LeafNode const& child : derived().getLeafChildren(node)) {
				if (hasSurfel(child)) {
					insertSurfel(node, getSurfel(child));
				}
			}
		} else {
			for (InnerNode const& child : derived().getInnerChildren(node)) {
				if (hasSurfel(child)) {
					insertSurfel(node, getSurfel(child));
				}
			}
		}
	}

	//
	// FIXME: REMOVE Update node indicators
	//

	void updateNodeIndicators(LeafNode&) {}

	void updateNodeIndicators(InnerNode&, depth_t) {}

	//
	// Input/output (read/write)
	//

	bool canReadData(DataIdentifier identifier) const noexcept
	{
		return DataIdentifier::SURFEL == identifier;
	}

	bool readNodes(std::istream& in_stream, std::vector<LeafNode*> const& nodes,
	               DataIdentifier data_identifier, uint64_t size)
	{
		if ("surfel" != field) {
			return false;
		}

		if ('U' == type && sizeof(Surfel) == size) {
			auto data = std::make_unique<Surfel[]>(nodes.size());
			in_stream.read(reinterpret_cast<char*>(data.get()), nodes.size() * sizeof(Surfel));

			for (size_t i = 0; i != nodes.size(); ++i) {
				setSurfel(*nodes[i], data[i]);
			}
		} else {
			return false;
		}
		return true;
	}

	void writeNodes(std::ostream& out_stream, std::vector<LeafNode> const& nodes) const
	{
		uint64_t const size = nodes.size() * sizeof(Surfel);
		out_stream.write(reinterpret_cast<char const*>(&size), sizeof(size));

		auto data = std::make_unique<Surfel[]>(nodes.size());
		for (size_t i = 0; i != nodes.size(); ++i) {
			data[i] = getSurfel(nodes[i]);
		}

		out_stream.write(reinterpret_cast<char const*>(data.get()),
		                 nodes.size() * sizeof(Surfel));
	}

 protected:
	//  Surfel depth
	depth_t surfel_depth_ = 0;
};
}  // namespace ufo::map

#endif  // UFO_MAP_SURFEL_MAP_BASE_H