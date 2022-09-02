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
#include <ufo/util/enum.h>

// STL
#include <cstdint>
#include <deque>
#include <functional>
#include <type_traits>
#include <utility>

namespace ufo::map
{
template <class Derived, class LeafNode>
class SurfelMapBase
{
 public:
	using Surfel = typename LeafNode::surfel_type;

 public:
	//
	// Get surfel
	//

	Surfel getSurfel(Node node) const
	{
		return getSurfel(derived().getLeafNode(node), node.index());
	}

	Surfel getSurfel(Code code) const
	{
		return getSurfel(derived().getLeafNode(code), code.index());
	}

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
		return hasSurfel(derived().getLeafNode(node), node.index());
	}

	[[nodiscard]] bool hasSurfel(Code code) const
	{
		return hasSurfel(derived().getLeafNode(code), code.index());
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
		return getNumSurfelPoints(derived().getLeafNode(node), node.index());
	}

	[[nodiscard]] std::size_t getNumSurfelPoints(Code code) const
	{
		return getNumSurfelPoints(derived().getLeafNode(code), code.index());
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
		    node,
		    [&surfel](auto& node, index_t const index) { setSurfel(node, index, surfel); },
		    [&surfel](auto& node) { setSurfel(node, surfel); }, propagate);
	}

	void setSurfel(Code code, Surfel const& surfel, bool propagate = true)
	{
		derived().apply(
		    code,
		    [&surfel](auto& node, index_t const index) { setSurfel(node, index, surfel); },
		    [&surfel](auto& node) { setSurfel(node, surfel); }, propagate);
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
		    [point](auto& node, index_t const index) {
			    eraseSurfelPoint(node, index, point);
		    },
		    [point](auto& node) { eraseSurfelPoint(node, point); }, propagate);
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
		    node, [](auto& node, index_t const index) { clearSurfel(node, index); },
		    [](auto& node) { clearSurfel(node); }, propagate);
	}

	void clearSurfel(Code code, bool propagate = true)
	{
		derived().apply(
		    code, [](auto& node, index_t const index) { clearSurfel(node, index); },
		    [](auto& node) { clearSurfel(node); }, propagate);
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

	void initRoot() { clearSurfel(derived().getRoot(), derived().getRootIndex()); }

	//
	// Get surfel
	//

	template <bool Single>
	static constexpr Surfel& getSurfel(SurfelNode<Single>& node, index_t const index)
	{
		return node.getSurfel(index);
	}

	template <bool Single>
	static constexpr Surfel const& getSurfel(SurfelNode<Single> const& node,
	                                         index_t const index)
	{
		return node.getSurfel(index);
	}

	//
	// Set surfel
	//

	// TODO: Continue from here

	template <bool Single, typename T>
	static constexpr void setSurfel(SurfelNode<Single>& node,
	                                Surfel<T> const& surfel) noexcept
	{
		node.setSurfel(surfel);
	}

	template <bool Single, typename T>
	static constexpr void setSurfel(SurfelNode<Single>& node, index_t const index,
	                                Surfel<T> const& surfel) noexcept
	{
		node.setSurfel(index, surfel);
	}

	template <bool Single, class... Args>
	static constexpr void setSurfel(SurfelNode<Single>& node, Args&&... args)
	{
		node.setSurfel(Surfel(std::forward<Args>(args)...));
	}

	template <bool Single, class... Args>
	static constexpr void setSurfel(SurfelNode<Single>& node, index_t const index,
	                                Args&&... args)
	{
		node.setSurfel(index, Surfel(std::forward<Args>(args)...));
	}

	//
	// Clear surfel
	//

	template <bool Single>
	static constexpr void clearSurfel(SurfelNode<Single>& node) noexcept
	{
		node.clearSurfel();
	}

	template <bool Single>
	static constexpr void clearSurfel(SurfelNode<Single>& node,
	                                  index_t const index) noexcept
	{
		node.clearSurfel(index);
	}

	//
	// Has surfel
	//

	template <bool Single>
	static constexpr bool hasSurfel(SurfelNode<Single> const& node,
	                                index_t const index) noexcept
	{
		return 0 != getNumSurfelPoints(node, index);
	}

	//
	// Get num surfel points
	//

	template <bool Single>
	static constexpr std::size_t getNumSurfelPoints(SurfelNode<Single> const& node,
	                                                index_t const index) noexcept
	{
		return node.getSurfel(index).getNumPoints();
	}

	//
	// Insert surfel
	//

	template <bool Single>
	static void insertSurfel(SurfelNode<Single>& node, Surfel const& surfel)
	{
		// TODO: Implement
		getSurfel(node).addSurfel(surfel);
	}

	template <bool Single>
	static void insertSurfel(SurfelNode<Single>& node, index_t const index,
	                         Surfel const& surfel)
	{
		// TODO: Implement
	}

	template <bool Single, class InputIt>
	static void insertSurfel(SurfelNode<Single>& node, InputIt first, InputIt last)
	{
		// TODO: Implement
		for (; first != last; ++first) {
			getSurfel(node).addSurfel(*first);
		}
	}

	template <bool Single, class InputIt>
	static void insertSurfel(SurfelNode<Single>& node, InputIt first, InputIt last)
	{
		// TODO: Implement
	}

	//
	// Erase surfel
	//

	template <bool Single>
	static void eraseSurfel(SurfelNode<Single>& node, Surfel const& surfel)
	{
		// TODO: Implement
		getSurfel(node).removeSurfel(surfel);
	}

	template <bool Single>
	static void eraseSurfel(SurfelNode<Single>& node, index_t const index,
	                        Surfel const& surfel)
	{
		// TODO: Implement
	}

	//
	// Insert surfel point
	//

	template <bool Single>
	static void insertSurfelPoint(SurfelNode<Single>& node, Point3 point)
	{
		// TODO: Implement
		getSurfel(node).addPoint(point);
	}

	template <bool Single>
	static void insertSurfelPoint(SurfelNode<Single>& node, index_t const index,
	                              Point3 point)
	{
		// TODO: Implement
	}

	template <bool Single, class InputIt>
	static void insertSurfelPoint(SurfelNode<Single>& node, InputIt first, InputIt last)
	{
		// TODO: Implement
		getSurfel(node).addPoint(first, last);
	}

	template <bool Single, class InputIt>
	static void insertSurfelPoint(SurfelNode<Single>& node, index_t const index,
	                              InputIt first, InputIt last)
	{
		// TODO: Implement
	}

	//
	// Erase surfel point
	//

	template <bool Single>
	static void eraseSurfelPoint(SurfelNode<Single>& node, Point3 point)
	{
		// TODO: Implement
		getSurfel(node).removePoint(point);
	}

	template <bool Single>
	static void eraseSurfelPoint(SurfelNode<Single>& node, index_t const index,
	                             Point3 point)
	{
		// TODO: Implement
	}

	template <bool Single, class InputIt>
	static void eraseSurfelPoint(SurfelNode<Single>& node, InputIt first, InputIt last)
	{
		// TODO: Implement
		getSurfel(node).removePoint(first, last);
	}

	template <bool Single, class InputIt>
	static void eraseSurfelPoint(SurfelNode<Single>& node, index_t const index,
	                             InputIt first, InputIt last)
	{
		// TODO: Implement
	}

	//
	// Update node
	//

	template <bool Single>
	void updateNode(SurfelNode<Single> const&, index_field_t const)
	{
	}

	template <bool Single, class T>
	void updateNode(SurfelNode<Single>& node, index_field_t const indices,
	                T const& children)
	{
		// TODO: Implement

		clearSurfel(node);

		for (auto const& child : children) {
			if (hasSurfel(child)) {
				insertSurfel(node, getSurfel((child)));
			}
		}
	}

	//
	// Input/output (read/write)
	//

	static constexpr DataIdentifier getDataIdentifier() noexcept
	{
		return DataIdentifier::SURFEL;
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
		if constexpr (std::is_base_of_v(SurfelNode<true>, node_type)) {
			return 1;
		} else {
			return 0;
		}
	}

	template <class InputIt>
	void readNodes(std::istream& in, InputIt first, InputIt last, std::size_t num_nodes)
	{
		// TODO: Implement

		uint8_t type;
		in_stream.read(reinterpret_cast<char*>(&type), sizeof(type));
		DataType dt = getDataType(type);

		auto const num_nodes = nodes.size();

		switch (dt) {
			case DataType::FLOAT32: {
				auto data = std::make_unique<Surfel<float>[]>(nodes.size());
				in_stream.read(reinterpret_cast<char*>(data.get()),
				               nodes.size() * sizeof(Surfel<float>));
				for (size_t i = 0; num_nodes != i; ++i) {
					setSurfel(*nodes[i], data[i]);
				}
			} break;
			case DataType::FLOAT64: {
				auto data = std::make_unique<Surfel<double>[]>(nodes.size());
				in_stream.read(reinterpret_cast<char*>(data.get()),
				               nodes.size() * sizeof(Surfel<double>));
				for (size_t i = 0; num_nodes != i; ++i) {
					setSurfel(*nodes[i], data[i]);
				}
			} break;
			default:
				// TODO: Throw
		}
	}

	template <class InputIt>
	void writeNodes(std::ostream& out, InputIt first, InputIt last,
	                std::size_t num_nodes) const
	{
		// TODO: Implement

		uint8_t type = util::enumToValue(getDataType<typename Surfel::scalar_t>());
		out_stream.write(reinterpret_cast<char const*>(&type), sizeof(type));

		auto const num_nodes = nodes.size();
		auto data = std::make_unique<Surfel[]>(num_nodes);
		for (size_t i = 0; num_nodes != i; ++i) {
			data[i] = getSurfel(nodes[i]);
		}

		out_stream.write(reinterpret_cast<char const*>(data.get()),
		                 num_nodes * sizeof(Surfel));
	}

 protected:
	//  Surfel depth
	depth_t surfel_depth_ = 0;
};
}  // namespace ufo::map

#endif  // UFO_MAP_SURFEL_MAP_BASE_H