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
// #include <ufo/map/predicate/surfel.h>
// #include <ufo/map/types.h>

// STL
#include <cstdint>
#include <type_traits>
#include <utility>

namespace ufo::map
{
template <class Derived, class LeafNode, class InnerNode>
class SurfelMapBase
{
 public:
	using Surfel = std::remove_pointer_t<typename LeafNode::surfel>;

 public:
	//
	// Get surfel
	//

	std::optional<Surfel> getSurfel(Node node) const
	{
		// TODO: Implement
	}

	std::optional<Surfel> getSurfel(Code code) const
	{
		// TODO: Implement
	}

	std::optional<Surfel> getSurfel(Key key) const
	{
		return getTimeStep(Derived::toCode(key));
	}

	std::optional<Surfel> getSurfel(Point3 coord, depth_t depth = 0) const
	{
		return getTimeStep(derived().toCode(coord, depth));
	}

	std::optional<Surfel> getSurfel(coord_t x, coord_t y, coord_t z,
	                                depth_t depth = 0) const
	{
		return getTimeStep(derived().toCode(x, y, z, depth));
	}

	//
	// Get surfel points
	//

	std::vector<std::pair<Point3, std::size_t>> getSurfelPoints(Node node) const
	{
		// TODO: Implement
	}

	std::vector<std::pair<Point3, std::size_t>> getSurfelPoints(Code code) const
	{
		// TODO: Implement
	}

	std::vector<std::pair<Point3, std::size_t>> getSurfelPoints(Key key) const
	{
		return getSurfelPoints(Derived::toCode(key));
	}

	std::vector<std::pair<Point3, std::size_t>> getSurfelPoints(Point3 coord,
	                                                            depth_t depth = 0) const
	{
		return getSurfelPoints(derived().toCode(coord, depth));
	}

	std::vector<std::pair<Point3, std::size_t>> getSurfelPoints(coord_t x, coord_t y,
	                                                            coord_t z,
	                                                            depth_t depth = 0) const
	{
		return getSurfelPoints(derived().toCode(x, y, z, depth));
	}

	//
	// Insert surfel point
	//

	void insertSurfelPoint(Node node, bool propagate = true)
	{
		// TODO: Implement
	}

	void insertSurfelPoint(Code code, bool propagate = true)
	{
		// TODO: Implement
	}

	void insertSurfelPoint(Key key, bool propagate = true)
	{
		insertSurfelPoint(Derived::toCode(key), propagate);
	}

	void insertSurfelPoint(Point3 point, depth_t depth = 0, bool propagate = true)
	{
		insertSurfelPoint(derived().toCode(point, depth), propagate);
	}

	void insertSurfelPoint(coord_t x, coord_t y, coord_t z, depth_t depth = 0,
	                       bool propagate = true)
	{
		insertSurfelPoint(derived().toCode(x, y, z, depth), propagate);
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

	void initRoot() { setSurfel(derived().getRoot(), nullptr); }

	//
	// Get surfel
	//

	static constexpr Surfel& getSurfel(LeafNode& node) { return *node.surfel; }

	static constexpr Surfel const& getSurfel(LeafNode const& node) { return *node.surfel; }

	//
	// Set surfel
	//

	static constexpr void setSurfel(LeafNode& node, Surfel* surfel) noexcept
	{
		node.surfel = surfel;
	}

	//
	// Has surfel
	//

	static constexpr bool hasSurfel(LeafNode const& node) noexcept
	{
		return nullptr != node.surfel;
	}

	//
	// Add surfels
	//

	Surfel addSurfel(Surfel surfel, Sufel const& other)
	{
		auto const alpha = Surfel::scalar_t(1) / (surfel.num_points * other.num_points *
		                                          (other.num_points + surfel.num_points));
		auto const beta = other.num_points * surfel.sum - surfel.num_points * other.sum;

		for (std::size_t i = 0; i != surfel.sum_squares.size(); ++i) {
			for (std::size_t j = 0; j != surfel.sum_squares[i].size(); ++j) {
				surfel.sum_squares[i][j] += other.sum_squares[i][j] + alpha * beta[i] * beta[j];
			}
		}
		surfel.sum += other.sum;
		surfel.num_points += other.num_points;

		return surfel;
	}

	//
	// Calculate surfel from surfels
	//

	std::optional<Surfel> calculateSurfelFromSurfels(InnerNode const& node, depth_t depth)
	{
		if (isLeaf(node)) {
			return std::nullopt;
		}

		if (1 == depth) {
			if (none_of(derived().getLeafChildren(node),
			            [this](auto const& child) { return hasSurfel(child); })) {
				return std::nullopt;
			}
		} else {
			if (none_of(derived().getInnerChildren(node),
			            [this](auto const& child) { return hasSurfel(child); })) {
				return std::nullopt;
			}
		}

		Surfel surfel;

		bool first = true;
		if (1 == depth) {
			for (LeafNode const& node : derived().getLeafChildren(node)) {
				if (!hasSurfel(child)) {
					continue;
				}

				if (first) {
					first = false;
					surfel = getSurfel(child);
					continue;
				} else {
					surfel = addSurfel(surfel, getSurfel(child));
				}
			}
		} else {
			for (InnerNode const& node : derived().getInnerChildren(node)) {
				if (!hasSurfel(child)) {
					continue;
				}

				if (first) {
					first = false;
					surfel = getSurfel(child);
					continue;
				} else {
					surfel = addSurfel(surfel, getSurfel(child));
				}
			}
		}

		return surfel;
	}

	//
	// Calculate surfel from points
	//

	std::optional<Surfel> calculateSurfelFromPoints(InnerNode const& node, depth_t depth)
	{
		if (isLeaf(node)) {
			return std::nullopt;
		}

		surfel_point_counter_type total_points;
		if (1 == depth) {
			surfel_point_counter_type =
			    reduce(derived().getLeafChildren(node), surfel_point_counter_type(0),
			           [](auto const& a, auto const& b) { return a + b.num_sufel_points; });
		} else {
			surfel_point_counter_type =
			    reduce(derived().getInnerChildren(node), surfel_point_counter_type(0),
			           [](auto const& a, auto const& b) { return a + b.num_sufel_points; });
		}

		if (0 == total_points) {
			return std::nullopt;
		}

		std::vector<std::pair<Point3, uintptr_t>> points_with_weight;
		points_with_weight.reserve(total_points);
		getSurfelPointsRecurs(points_with_weight, node, Point3(0, 0, 0), depth);

		// if (points_with_weight.empty()) {
		// 	return std::nullopt;
		// }

		Surfel surfel;

		Point3 position(0, 0, 0);
		// uintptr_t total_weight = 0;
		for (auto [point, weight] : points_with_weight) {
			position.x += point.x * weight;
			position.y += point.y * weight;
			position.z += point.z * weight;
			// total_weight += weight;
		}
		// position /= total_weight;
		position /= total_points;

		Point3 normal(0, 0, 0);
		// TODO: Implement

		return surfel;
	}

	void getSurfelPointsRecurs(std::vector<std::pair<Point3, uintptr_t>>& points,
	                           InnerNode const& node, Point3 center, depth_t depth)
	{
		if (nullptr == getSurfel(node) || 0 == reinterpret_cast<uintptr_t>(getSurfel(node))) {
			return;
		}

		if (isLeaf(node)) {
			points.emplace_back(center, reinterpret_cast<uintptr_t>(getSurfel(node)));
		} else if (1 == depth) {
			for (std::size_t i = 0; i != 8; ++i) {
				LeafNode const& child = derived().getLeafChild(node, i);

				if (nullptr != getSurfel(child) &&
				    0 != reinterpret_cast<uintptr_t>(getSurfel(child))) {
					points.emplace_back(Derived::childCenter(center, derived().getNodeSize(0), i),
					                    reinterpret_cast<uintptr_t>(getSurfel(child)));
				}
			}
		} else {
			for (std::size_t i = 0; i != 8; ++i) {
				getSurfelPointsRecurs(points, derived().getInnerChild(node, i),
				                      Derived::childCenter(center, derived().getNodeSize(0), i),
				                      depth - 1);
			}
		}
	}

	//
	// Update node
	//

	void updateNode(InnerNode& node, depth_t depth)
	{
		if (depth > surfel_depth_) {
			setSurfel(node, calculateSurfelFromSurfels(node, depth));
		} else if (depth == surfel_depth_) {
			setSurfel(node, calculateSurfelFromPoints(node, depth));
		} else {
			setSurfel(node, reinterpret_cast<Surfel*>(numChildrenSurfelPoints(node, depth)));
		}
	}

	//
	// Has children surfel points
	//

	uintptr_t numChildrenSurfelPoints(InnerNode const& node, depth_t depth)
	{
		uintptr_t num_points = 0;

		if (1 == depth) {
			for (LeafNode const& child : derived().getLeafChildren(node)) {
				if (nullptr != getSurfel(child)) {
					num_points += reinterpret_cast<uintptr_t>(getSurfel(child));
				}
			}
		} else {
			for (InnerNode const& child : derived().getInnerChildren(node)) {
				if (nullptr != getSurfel(child)) {
					num_points += reinterpret_cast<uintptr_t>(getSurfel(child));
				}
			}
		}

		return num_points;
	}

	//
	// Input/output (read/write)
	//

	void addFileInfo(FileInfo& info) const
	{
		info["fields"].emplace_back("surfel");
		info["type"].emplace_back("U");
		info["size"].emplace_back(std::to_string(sizeof(uintptr_t)));
	}

	bool readNodes(std::istream& in_stream, std::vector<LeafNode*> const& nodes,
	               std::string const& field, char type, uint64_t size, uint64_t num)
	{
		if ("surfel" != field) {
			return false;
		}

		if ('U' == type && sizeof(uintptr_t) == size) {
			auto data = std::make_unique<uintptr_t[]>(nodes.size());
			in_stream.read(reinterpret_cast<char*>(data.get()),
			               nodes.size() * sizeof(uintptr_t));

			for (size_t i = 0; i != nodes.size(); ++i) {
				setSurfel(*nodes[i], reinterpret_cast<Surfel*>(data[i]));
			}
		} else {
			return false;
		}
		return true;
	}

	void writeNodes(std::ostream& out_stream, std::vector<LeafNode> const& nodes,
	                bool compress, int compression_acceleration_level,
	                int compression_level) const
	{
		uint64_t const size = nodes.size();
		out_stream.write(reinterpret_cast<char const*>(&size), sizeof(size));

		auto data = std::make_unique<uintptr_t[]>(nodes.size());
		for (size_t i = 0; i != nodes.size(); ++i) {
			data[i] = reinterpret_cast<uintptr_t>(getSurfel(nodes[i]));
		}

		out_stream.write(reinterpret_cast<char const*>(data.get()),
		                 nodes.size() * sizeof(uintptr_t));
	}

 protected:
	//  Surfel depth
	depth_t surfel_depth_ = 0;
};
}  // namespace ufo::map

#endif  // UFO_MAP_SURFEL_MAP_BASE_H