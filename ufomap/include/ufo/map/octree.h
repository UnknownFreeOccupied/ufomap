/**
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * License: BSD 3
 *
 */

/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, D. Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
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

#ifndef UFO_MAP_OCTREE_H
#define UFO_MAP_OCTREE_H

// UFO
#include <ufo/map/code.h>
#include <ufo/map/iterator/octree.h>
#include <ufo/map/iterator/octree_nearest.h>
#include <ufo/map/key.h>
#include <ufo/map/octree_node.h>
#include <ufo/map/types.h>

// STD
#include <algorithm>
#include <bitset>
#include <cstring>
#include <fstream>
#include <future>
#include <numeric>
#include <optional>
#include <sstream>
#include <type_traits>
#include <vector>

// Compression
#include <lz4.h>
#include <lz4hc.h>

namespace ufo::map
{
template <typename DATA_TYPE, typename INNER_NODE = OctreeInnerNode<DATA_TYPE>,
          typename LEAF_NODE = OctreeLeafNode<DATA_TYPE>>
class Octree
{
 public:
	using OctreeTreeIterator =
	    OctreeIterator<Octree, DATA_TYPE, INNER_NODE, LEAF_NODE, false>;
	using OctreeLeafIterator =
	    OctreeIterator<Octree, DATA_TYPE, INNER_NODE, LEAF_NODE, true>;
	using OctreeTreeNNIterator =
	    OctreeNearestIterator<Octree, DATA_TYPE, INNER_NODE, LEAF_NODE, false>;
	using OctreeLeafNNIterator =
	    OctreeNearestIterator<Octree, DATA_TYPE, INNER_NODE, LEAF_NODE, true>;

 protected:
	// TODO: Should these be inline or constexpr?
	static inline const DepthType MIN_DEPTH_LEVELS = 2;   // Minimum number of depth levels
	static inline const DepthType MAX_DEPTH_LEVELS = 21;  // Maximum number of depth levels

	using Path = std::array<LEAF_NODE*, MAX_DEPTH_LEVELS>;

	// class Node
	// {
	//  protected:
	// 	Path path_;
	// 	std::array<ufo::geometry::AABB, MAX_DEPTH_LEVELS>;
	// 	DepthType depth_;
	// };

 public:
	//
	// Destructor
	//

	virtual ~Octree() { clear(); }

	//
	// General information
	//

	virtual std::string getTreeType() const noexcept = 0;

	static constexpr DepthType getMinDepthLevels() noexcept { return MIN_DEPTH_LEVELS; }

	static constexpr DepthType getMaxDepthLevels() noexcept { return MAX_DEPTH_LEVELS; }

	static std::string getFileVersion() noexcept { return FILE_VERSION; }

	//
	// Root code
	//

	Code getRootCode() const noexcept { return Code(0, getTreeDepthLevels()); }

	//
	// Automatic pruning
	//

	void enableAutomaticPruning(bool enable) noexcept
	{
		automatic_pruning_enabled_ = enable;
	}

	bool isAutomaticPruningEnabled() const noexcept { return automatic_pruning_enabled_; }

	//
	// "Normal" iterators
	//

	OctreeTreeIterator beginTree(DepthType min_depth = 0) const noexcept
	{
		return OctreeTreeIterator(this, getRoot(), ufo::geometry::BoundingVolume(),
		                          min_depth);
	}

	OctreeTreeIterator endTree() const noexcept { return OctreeTreeIterator(); }

	OctreeTreeIterator beginTreeBounding(ufo::geometry::BoundingVar const& bounding_volume,
	                                     DepthType min_depth = 0) const noexcept
	{
		ufo::geometry::BoundingVolume bv;
		bv.add(bounding_volume);
		return OctreeTreeIterator(this, getRoot(), bv, min_depth);
	}

	OctreeTreeIterator beginTreeBounding(
	    ufo::geometry::BoundingVolume const& bounding_volume,
	    DepthType min_depth = 0) const noexcept
	{
		return OctreeTreeIterator(this, getRoot(), bounding_volume, min_depth);
	}

	OctreeLeafIterator beginLeaves(DepthType min_depth = 0) const noexcept
	{
		return OctreeLeafIterator(this, getRoot(), ufo::geometry::BoundingVolume(),
		                          min_depth);
	}

	OctreeLeafIterator endLeaves() const noexcept { return OctreeLeafIterator(); }

	OctreeLeafIterator beginLeavesBounding(
	    ufo::geometry::BoundingVar const& bounding_volume,
	    DepthType min_depth = 0) const noexcept
	{
		ufo::geometry::BoundingVolume bv;
		bv.add(bounding_volume);
		return OctreeLeafIterator(this, getRoot(), bv, min_depth);
	}

	OctreeLeafIterator beginLeavesBounding(
	    ufo::geometry::BoundingVolume const& bounding_volume,
	    DepthType min_depth = 0) const noexcept
	{
		return OctreeLeafIterator(this, getRoot(), bounding_volume, min_depth);
	}

	//
	// Nearest neighbor iterators
	//

	// OctreeTreeNNIterator beginTreeNN(Point3 const& coord,
	//                                  DepthType min_depth = 0) const noexcept
	// {
	// 	return OctreeTreeNNIterator(this, getRoot(), coord,
	// ufo::geometry::BoundingVolume(), 	                            min_depth);
	// }

	// OctreeTreeNNIterator endTreeNN() const noexcept { return OctreeTreeNNIterator(); }

	// OctreeTreeNNIterator beginTreeNNBounding(
	//     Point3 const& coord, ufo::geometry::BoundingVar const& bounding_volume,
	//     DepthType min_depth = 0) const noexcept
	// {
	// 	ufo::geometry::BoundingVolume bv;
	// 	bv.add(bounding_volume);
	// 	return OctreeTreeNNIterator(this, getRoot(), coord, bv, min_depth);
	// }

	// OctreeTreeNNIterator beginTreeNNBounding(
	//     Point3 const& coord, ufo::geometry::BoundingVolume const& bounding_volume,
	//     DepthType min_depth = 0) const noexcept
	// {
	// 	return OctreeTreeNNIterator(this, getRoot(), coord, bounding_volume,
	// min_depth);
	// }

	// OctreeLeafNNIterator beginLeavesNN(Point3 const& coord,
	//                                    DepthType min_depth = 0) const noexcept
	// {
	// 	return OctreeLeafNNIterator(this, getRoot(), coord,
	// ufo::geometry::BoundingVolume(), 	                            min_depth);
	// }

	// OctreeLeafNNIterator endLeavesNN() const noexcept { return OctreeLeafNNIterator(); }

	// OctreeLeafNNIterator beginLeavesNNBounding(
	//     Point3 const& coord, ufo::geometry::BoundingVar const& bounding_volume,
	//     DepthType min_depth = 0) const noexcept
	// {
	// 	ufo::geometry::BoundingVolume bv;
	// 	bv.add(bounding_volume);
	// 	return OctreeLeafNNIterator(this, getRoot(), coord, bv, min_depth);
	// }

	// OctreeLeafNNIterator beginLeavesNNBounding(
	//     Point3 const& coord, ufo::geometry::BoundingVolume const& bounding_volume,
	//     DepthType min_depth = 0) const noexcept
	// {
	// 	return OctreeLeafNNIterator(this, getRoot(), coord, bounding_volume,
	// min_depth);
	// }

	//
	// Pruning
	//

	bool prune() { return prune(getRootCode()); }

	bool pruneNode(Code const& code)
	{
		if (0 == code.getDepth()) {
			return false;
		}

		auto [node, depth] = getNode(code);
		if (code.getDepth() != depth ||
		    !isNodeCollapsible(static_cast<INNER_NODE const&>(*node), code.getDepth())) {
			return false;
		}
		return deleteChildren(static_cast<INNER_NODE&>(node), code.getDepth(), true);
	}

	//
	// Search
	//

	DATA_TYPE const* search(Code const& code) const
	{
		auto [node, depth] = getNode(code);
		return code.getDepth() == depth ? &(node->value) : nullptr;
	}

	//
	// Check if node exists
	//

	bool nodeExists(Code const& code) const { return nullptr != getNode(code); }

	//
	// Get child/parent code
	//

	Code getChild(Code const& code, unsigned int child_idx) const
	{
		// TODO: Have check for depth?
		return code.getChild(child_idx);
	}

	Code getParent(Code const& code) const
	{
		// TODO: Have check for depth?
		return code.toDepth(code.getDepth() + 1);
	}

	//
	// To code
	//

	Code toCode(Key const& key) const noexcept { return Code(key); }

	Code toCode(Point3 const& coord, DepthType depth = 0) const noexcept
	{
		return toCode(toKey(coord, depth));
	}

	Code toCode(double x, double y, double z, DepthType depth = 0) const noexcept
	{
		return toCode((toKey(x, y, z, depth)));
	}

	//
	// To key
	//

	Key toKey(Code const& code) const noexcept { return code.toKey(); }

	KeyType toKey(double coord, DepthType depth = 0) const noexcept
	{
		int key_value = (int)std::floor(resolution_factor_ * coord);
		if (0 == depth) {
			return key_value + max_value_;  // FIXME: Can this cause problems?
		}
		return ((key_value >> depth) << depth) + (1 << (depth - 1)) + max_value_;
	}

	Key toKey(Point3 const& coord, DepthType depth = 0) const noexcept
	{
		return Key(toKey(coord[0], depth), toKey(coord[1], depth), toKey(coord[2], depth),
		           depth);
	}

	Key toKey(double x, double y, double z, DepthType depth = 0) const noexcept
	{
		return Key(toKey(x, depth), toKey(y, depth), toKey(z, depth), depth);
	}

	std::optional<KeyType> toKeyChecked(double coord, DepthType depth = 0) const noexcept
	{
		if (getMin()[0] > coord || getMax()[0] < coord) {
			return std::nullopt;
		}
		return toKey(coord, depth);
	}

	std::optional<Key> toKeyChecked(Point3 const& coord, DepthType depth = 0) const noexcept
	{
		std::optional<KeyType> x = toKeyChecked(coord[0], depth);
		if (!x.has_value()) {
			return std::nullopt;
		}
		std::optional<KeyType> y = toKeyChecked(coord[1], depth);
		if (!y.has_value()) {
			return std::nullopt;
		}
		std::optional<KeyType> z = toKeyChecked(coord[2], depth);
		if (!z.has_value()) {
			return std::nullopt;
		}
		return Key(x.value(), y.value(), z.value(), depth);
	}

	std::optional<Key> toKeyChecked(double x, double y, double z,
	                                DepthType depth = 0) const noexcept
	{
		return toKeyChecked(Point3(x, y, z), depth);
	}

	//
	// To coordinate
	//

	Point3 toCoord(Code const& code) const noexcept { return toCoord(toKey(code)); }

	double toCoord(KeyType key, DepthType depth = 0) const noexcept
	{
		if (getTreeDepthLevels() == depth) {
			return 0.0;
		}

		double divider = double(1 << depth);
		return (floor((double(key) - double(max_value_)) / divider) + 0.5) *
		       getNodeSize(depth);
	}

	Point3 toCoord(Key const& key) const noexcept
	{
		return Point3(toCoord(key[0], key.getDepth()), toCoord(key[1], key.getDepth()),
		              toCoord(key[2], key.getDepth()));
	}

	Point3 toCoord(Key const& key, DepthType depth) const noexcept
	{
		return Point3(toCoord(key[0], depth), toCoord(key[1], depth), toCoord(key[2], depth));
	}

	std::optional<Point3> toCoordChecked(Key const& key, DepthType depth) const noexcept
	{
		if (key.getDepth() > depth) {
			return std::nullopt;
		}
		return toCoord(key, depth);
	}

	//
	// Memory
	//

	/**
	 * @return std::size_t number of inner nodes in the tree
	 */
	std::size_t getNumInnerNodes() const noexcept
	{
		return num_inner_nodes_ + num_inner_leaf_nodes_;
	}

	/**
	 * @return std::size_t number of leaf nodes in the tree
	 */
	std::size_t getNumLeafNodes() const noexcept { return num_leaf_nodes_; }

	/**
	 * @return std::size_t memory usage of a single inner node
	 */
	std::size_t memoryUsageInnerNode() const noexcept { return sizeof(INNER_NODE); }

	/**
	 * @return std::size_t memory usage of a single leaf node
	 */
	std::size_t memoryUsageLeafNode() const noexcept { return sizeof(LEAF_NODE); }

	/**
	 * @return std::size_t memory usage of the octree
	 */
	std::size_t memoryUsage() const noexcept
	{
		return (getNumInnerNodes() * memoryUsageInnerNode()) +
		       (getNumLeafNodes() * memoryUsageLeafNode());
	}

	/**
	 * @return std::size_t number of nodes in the tree
	 */
	std::size_t size() const noexcept { return getNumInnerNodes() + getNumLeafNodes(); }

	//
	// Ray casting
	//

	CodeRay computeRay(Point3 const& origin, Point3 const& end, double max_range = -1,
	                   DepthType depth = 0) const
	{
		KeyRay key_ray = computeRayKeys(origin, end, max_range, depth);

		CodeRay ray;
		ray.reserve(key_ray.size());
		for (Key const& key : key_ray) {
			ray.emplace_back(key);
		}
		return ray;
	}

	KeyRay computeRayKeys(Point3 const& origin, Point3 end, double max_range = -1,
	                      DepthType depth = 0) const
	{
		const double min_allowed = getMin()[0];
		const double max_allowed = getMax()[0];
		if (min_allowed <= origin.min() && max_allowed >= origin.max() &&
		    min_allowed <= end.min() && max_allowed >= end.max()) {
			std::out_of_range("Coordinate out of range of octree");
		}

		KeyRay ray;

		Point3 direction = (end - origin);
		double distance = direction.norm();
		direction /= distance;
		if (0 <= max_range && distance > max_range) {
			end = origin + (direction * max_range);
			distance = max_range;
		}

		Key current;
		Key ending;
		std::array<int, 3> step;
		Point3 t_delta;
		Point3 t_max;

		computeRayInit(origin, end, direction, current, ending, step, t_delta, t_max, depth);
		// Increment
		while (current != ending && t_max.min() <= distance) {
			ray.push_back(current);
			computeRayTakeStep(current, step, t_delta, t_max);
		}

		return ray;
	}

	//
	// Delete node
	//

	bool deleteNodeChildren(Code const& code)
	{
		if (0 == code.getDepth()) {
			return false;
		}

		auto [node, depth] = getNode(code);
		if (code.getDepth() != depth) {
			return false;
		}
		return deleteChildren(static_cast<INNER_NODE&>(node), code.getDepth(), true);
	}

	//
	// Get min/max coordinate octree can store
	//

	/**
	 * @return The minimum point the octree can store
	 */
	Point3 getMin() const noexcept
	{
		double half_size = -getNodeHalfSize(getTreeDepthLevels());
		return Point3(half_size, half_size, half_size);
	}

	/**
	 * @return The maximum point the octree can store
	 */
	Point3 getMax() const noexcept
	{
		double half_size = getNodeHalfSize(getTreeDepthLevels());
		return Point3(half_size, half_size, half_size);
	}

	//
	// Clear
	//

	void clear() { clear(resolution_, getTreeDepthLevels()); }

	// FIXME: Make virtual
	void clear(double new_resolution, DepthType new_depth_levels)
	{
		if (MIN_DEPTH_LEVELS > new_depth_levels || MAX_DEPTH_LEVELS < new_depth_levels) {
			throw std::invalid_argument("depth_levels can be minimum " +
			                            std::to_string(MIN_DEPTH_LEVELS) + " and maximum " +
			                            std::to_string(MAX_DEPTH_LEVELS));
		}

		// TODO: Should they be manually deleted?
		deleteChildren(getRoot(), getTreeDepthLevels(), true);
		getRoot() = INNER_NODE();
		// TODO: Have to call update node

		depth_levels_ = new_depth_levels;
		max_value_ = std::pow(2, getTreeDepthLevels() - 1);

		if (new_resolution != resolution_) {
			resolution_ = new_resolution;
			resolution_factor_ = 1.0 / new_resolution;

			nodes_half_sizes_[0] = resolution_ / 2.0;
			nodes_half_sizes_[1] = resolution_;
			for (std::size_t i = 2; i <= getTreeDepthLevels(); ++i) {
				nodes_half_sizes_[i] = nodes_half_sizes_[i - 1] * 2.0;
			}
		}
	}

	//
	// Node size and resolution
	//

	double getNodeSize(DepthType depth) const { return getNodeHalfSize(depth + 1); }

	double getNodeHalfSize(DepthType depth) const { return nodes_half_sizes_[depth]; }

	double getResolution() const noexcept { return resolution_; }

	//
	// Tree depth
	//

	DepthType getTreeDepthLevels() const noexcept { return depth_levels_; }

	//
	// Checking for children
	//

	bool isLeaf(Code const& code) const
	{
		if (0 == code.getDepth()) {
			return true;
		}

		auto [node, depth] = getNode(code);
		return isLeaf(static_cast<INNER_NODE const&>(*node));
	}

	bool hasChildren(Code const& code) const { return !isLeaf(code); }

	//
	// Is node collapsible
	//

	bool isNodeCollapsible(Code const& code) const
	{
		if (0 == code.getDepth()) {
			return false;
		}

		auto [node, depth] = getNode(code);
		if (code.getDepth() != depth) {
			return false;
		}
		return isNodeCollapsible(static_cast<INNER_NODE const&>(*node), code.getDepth());
	}

	//
	// Get child center
	//

	static Point3 getChildCenter(Point3 const& parent_center, double child_half_size,
	                             unsigned int child_idx)
	{
		Point3 child_center(parent_center);
		child_center[0] += ((child_idx & 1) ? child_half_size : -child_half_size);
		child_center[1] += ((child_idx & 2) ? child_half_size : -child_half_size);
		child_center[2] += ((child_idx & 4) ? child_half_size : -child_half_size);
		return child_center;
	}

	//
	// Input/output (read/write)
	//

	static std::string readType(std::string const& filename)
	{
		std::ifstream file(filename.c_str(), std::ios_base::in | std::ios_base::binary);
		if (!file.is_open()) {
			return "";
		}
		// TODO: Check is_good of finished stream, warn?
		return readType(file);
	}

	static std::string readType(std::istream& s)
	{
		// check if first line valid:
		std::string line;
		std::getline(s, line);
		if (0 != line.compare(0, FILE_HEADER.length(), FILE_HEADER)) {
			return "";
		}

		std::string id = "";

		std::string token;
		bool header_read = false;
		while (s.good() && !header_read) {
			s >> token;
			if ("data" == token) {
				header_read = true;
				// Skip forward to the end of line
				char c;
				do {
					c = s.get();
				} while (s.good() && ('\n' != c));
			} else if (0 == token.compare(0, 1, "#")) {
				// Comment line, skip forward to the end of line
				char c;
				do {
					c = s.get();
				} while (s.good() && ('\n' != c));
			} else if ("id" == token) {
				s >> id;
			} else {
				// Other token
				char c;
				do {
					c = s.get();
				} while (s.good() && ('\n' != c));
			}
		}

		return id;
	}

	virtual bool read(std::string const& filename)
	{
		std::ifstream file(filename.c_str(), std::ios_base::in | std::ios_base::binary);
		if (!file.is_open()) {
			return false;
		}
		// TODO: Check is_good of finished stream, warn?
		return read(file);
	}

	virtual bool read(std::istream& s)
	{
		// check if first line valid:
		std::string line;
		std::getline(s, line);
		if (0 != line.compare(0, FILE_HEADER.length(), FILE_HEADER)) {
			return false;
		}

		std::string file_version;
		std::string id;
		double resolution;
		DepthType depth_levels;
		bool compressed;
		int uncompressed_data_size;
		if (!readHeader(s, file_version, id, resolution, depth_levels, compressed,
		                uncompressed_data_size)) {
			return false;
		}

		if (compressed) {
			std::stringstream uncompressed_s(std::ios_base::in | std::ios_base::out |
			                                 std::ios_base::binary);
			if (!decompressData(s, uncompressed_s, uncompressed_data_size)) {
				return false;
			}
			return readData(uncompressed_s, resolution, depth_levels, uncompressed_data_size,
			                compressed);
		} else {
			return readData(s, resolution, depth_levels, uncompressed_data_size, compressed);
		}
	}

	virtual bool readData(std::istream& s, double resolution, DepthType depth_levels,
	                      int uncompressed_data_size = 1, bool compressed = false)
	{
		return readData(s, ufo::geometry::BoundingVolume(), resolution, depth_levels,
		                uncompressed_data_size, compressed);
	}

	virtual bool readData(std::istream& s,
	                      ufo::geometry::BoundingVar const& bounding_volume,
	                      double resolution, DepthType depth_levels,
	                      int uncompressed_data_size = 1, bool compressed = false)
	{
		ufo::geometry::BoundingVolume bv;
		bv.add(bounding_volume);
		return readData(s, bv, resolution, depth_levels, uncompressed_data_size, compressed);
	}

	virtual bool readData(std::istream& s,
	                      ufo::geometry::BoundingVolume const& bounding_volume,
	                      double resolution, DepthType depth_levels,
	                      int uncompressed_data_size = 1, bool compressed = false)
	{
		if (!s.good()) {
			// TODO: Warning
		}

		if (getResolution() != resolution || getTreeDepthLevels() != depth_levels) {
			clear(resolution, depth_levels);
		}

		if (compressed) {
			std::stringstream uncompressed_s(std::ios_base::in | std::ios_base::out |
			                                 std::ios_base::binary);
			if (!decompressData(s, uncompressed_s, uncompressed_data_size)) {
				return false;
			}
			return readNodes(uncompressed_s, bounding_volume);
		} else {
			return readNodes(s, bounding_volume);
		}
	}

	virtual bool write(std::string const& filename, bool compress = false,
	                   DepthType min_depth = 0, int compression_acceleration_level = 1,
	                   int compression_level = 0) const
	{
		return write(filename, ufo::geometry::BoundingVolume(), compress, min_depth,
		             compression_acceleration_level, compression_level);
	}

	virtual bool write(std::string const& filename,
	                   ufo::geometry::BoundingVar const& bounding_volume,
	                   bool compress = false, DepthType min_depth = 0,
	                   int compression_acceleration_level = 1,
	                   int compression_level = 0) const
	{
		ufo::geometry::BoundingVolume bv;
		bv.add(bounding_volume);
		return write(filename, bv, compress, min_depth, compression_acceleration_level,
		             compression_level);
	}

	virtual bool write(std::string const& filename,
	                   ufo::geometry::BoundingVolume const& bounding_volume,
	                   bool compress = false, DepthType min_depth = 0,
	                   int compression_acceleration_level = 1,
	                   int compression_level = 0) const
	{
		std::ofstream file(filename.c_str(), std::ios_base::out | std::ios_base::binary);

		if (!file.is_open()) {
			return false;
		}
		// TODO: check is_good of finished stream, return
		const bool success = write(file, bounding_volume, compress, min_depth,
		                           compression_acceleration_level, compression_level);
		file.close();
		return success;
	}

	virtual bool write(std::ostream& s, bool compress = false, DepthType min_depth = 0,
	                   int compression_acceleration_level = 1,
	                   int compression_level = 0) const
	{
		return write(s, ufo::geometry::BoundingVolume(), compress, min_depth,
		             compression_acceleration_level, compression_level);
	}

	virtual bool write(std::ostream& s, ufo::geometry::BoundingVar const& bounding_volume,
	                   bool compress = false, DepthType min_depth = 0,
	                   int compression_acceleration_level = 1,
	                   int compression_level = 0) const
	{
		ufo::geometry::BoundingVolume bv;
		bv.add(bounding_volume);
		return write(s, bv, compress, min_depth, compression_acceleration_level,
		             compression_level);
	}

	virtual bool write(std::ostream& s,
	                   ufo::geometry::BoundingVolume const& bounding_volume,
	                   bool compress = false, DepthType min_depth = 0,
	                   int compression_acceleration_level = 1,
	                   int compression_level = 0) const
	{
		std::stringstream data(std::ios_base::in | std::ios_base::out |
		                       std::ios_base::binary);

		int uncompressed_data_size =
		    writeData(data, bounding_volume, compress, min_depth,
		              compression_acceleration_level, compression_level);

		if (0 > uncompressed_data_size) {
			return false;
		}

		// Write header
		s << FILE_HEADER;
		s << "\n# (feel free to add / change comments, but leave the first line as "
		     "it "
		     "is!)\n#\n";
		s << "version " << getFileVersion() << std::endl;
		s << "id " << getTreeType() << std::endl;
		s << "resolution " << getResolution() << std::endl;
		s << "depth_levels " << getTreeDepthLevels() << std::endl;
		s << "compressed " << compress << std::endl;
		s << "uncompressed_data_size " << uncompressed_data_size << std::endl;
		s << "data" << std::endl;

		// Write data
		s << data.rdbuf();  // Is correct or do I have to change to begin first?

		return s.good();
	}

	virtual int writeData(std::ostream& s, bool compress = false, DepthType min_depth = 0,
	                      int compression_acceleration_level = 1,
	                      int compression_level = 0) const
	{
		return writeData(s, ufo::geometry::BoundingVolume(), compress, min_depth,
		                 compression_acceleration_level, compression_level);
	}

	virtual int writeData(std::ostream& s,
	                      ufo::geometry::BoundingVar const& bounding_volume,
	                      bool compress = false, DepthType min_depth = 0,
	                      int compression_acceleration_level = 1,
	                      int compression_level = 0) const
	{
		ufo::geometry::BoundingVolume bv;
		bv.add(bounding_volume);
		return writeData(s, bv, compress, min_depth, compression_acceleration_level,
		                 compression_level);
	}

	virtual int writeData(std::ostream& s,
	                      ufo::geometry::BoundingVolume const& bounding_volume,
	                      bool compress = false, DepthType min_depth = 0,
	                      int compression_acceleration_level = 1,
	                      int compression_level = 0) const
	{
		const std::streampos initial_write_position = s.tellp();

		if (compress) {
			std::stringstream data(std::ios_base::in | std::ios_base::out |
			                       std::ios_base::binary);
			int uncompressed_data_size =
			    writeData(data, bounding_volume, false, min_depth,
			              compression_acceleration_level, compression_level);
			if (0 > uncompressed_data_size ||
			    !compressData(data, s, uncompressed_data_size, compression_acceleration_level,
			                  compression_level)) {
				return -1;
			}
			return uncompressed_data_size;
		} else {
			if (!writeNodes(s, bounding_volume, min_depth)) {
				return -1;
			}
		}

		// Return size of data
		return s.tellp() - initial_write_position;
	}

 protected:
	//
	// Constructors
	//

	Octree(double resolution, DepthType depth_levels, bool automatic_pruning)
	    : resolution_(resolution),
	      resolution_factor_(1.0 / resolution),
	      depth_levels_(depth_levels),
	      max_value_(std::pow(2, depth_levels - 1)),
	      automatic_pruning_enabled_(automatic_pruning)
	{
		if (getMinDepthLevels() > depth_levels || getMaxDepthLevels() < depth_levels) {
			throw std::invalid_argument("depth_levels has to be [" +
			                            std::to_string(getMinDepthLevels()) + ", " +
			                            std::to_string(getMaxDepthLevels()) + "]");
		}

		// Precompute sizes
		nodes_half_sizes_[0] = resolution_ / 2.0;
		nodes_half_sizes_[1] = resolution_;
		for (std::size_t i = 2; i <= depth_levels_; ++i) {
			nodes_half_sizes_[i] = nodes_half_sizes_[i - 1] * 2.0;
		}
	}

	//
	// Get root
	//

	INNER_NODE const& getRoot() const { return root_; }

	INNER_NODE& getRoot() { return root_; }

	//
	// Get node
	//

	std::pair<Path, DepthType> getNodePath(Code const& code)
	{
		Path path;
		path[getTreeDepthLevels()] = static_cast<LEAF_NODE*>(&getRoot());
		DepthType depth = getTreeDepthLevels();
		for (; depth > code.getDepth(); --depth) {
			INNER_NODE& node = static_cast<INNER_NODE&>(*path[depth]);
			if (isLeaf(node)) {
				break;
			}
			DepthType child_depth = depth - 1;
			path[child_depth] = static_cast<LEAF_NODE*>(
			    &getChild(node, child_depth, code.getChildIdx(child_depth)));
		}
		return std::make_pair(path, depth);
	}

	std::pair<LEAF_NODE const*, DepthType> getNode(Code const& code) const
	{
		LEAF_NODE const* node = &getRoot();
		for (DepthType depth = getTreeDepthLevels() - 1; depth > code.getDepth(); --depth) {
			INNER_NODE const& inner_node = static_cast<INNER_NODE const&>(*node);
			if (!hasChildren(inner_node)) {
				return std::make_pair(node, depth + 1);
			}
			node = &getChild(inner_node, depth, code.getChildIdx(depth));
		}
		return std::make_pair(node, code.getDepth());
	}

	std::pair<LEAF_NODE*, DepthType> getNode(Code const& code)
	{
		auto [const_node, depth] = std::as_const(*this).getNode(code);
		return std::make_pair(const_cast<LEAF_NODE*>(const_node), depth);
	}

	//
	// Create node
	//

	Path createNode(Code const& code)
	{
		Path path;
		path[getTreeDepthLevels()] = static_cast<LEAF_NODE*>(&getRoot());
		createNode(code, path, getTreeDepthLevels());
		return path;
	}

	void createNode(Code const& code, Path& path, DepthType depth)
	{
		for (; depth > code.getDepth(); --depth) {
			INNER_NODE& node = static_cast<INNER_NODE&>(*path[depth]);
			if (!hasChildren(node)) {
				createChildren(node, depth);  // TODO: Add depth
			}
			DepthType child_depth = depth - 1;
			path[child_depth] = static_cast<LEAF_NODE*>(
			    &getChild(node, child_depth, code.getChildIdx(child_depth)));
		}
	}

	//
	// Create / delete children
	//

	bool createChildren(INNER_NODE& node, DepthType depth)
	{
		if (!node.is_leaf) {
			return false;
		}

		if (!node.children) {
			// Allocate children
			if (1 == depth) {
				// Children are leaf nodes
				node.children = new std::array<LEAF_NODE, 8>();
				num_leaf_nodes_ += 8;
				num_inner_leaf_nodes_ -= 1;
			} else {
				// Children are inner nodes
				// Get 8 new and 1 is made into a inner node
				node.children = new std::array<INNER_NODE, 8>();
				num_inner_leaf_nodes_ += 7;
			}
			num_inner_nodes_ += 1;
		}

		if (1 == depth) {
			for (LEAF_NODE& child : getLeafChildren(node)) {
				child = static_cast<LEAF_NODE&>(node);
			}
		} else {
			for (INNER_NODE& child : getInnerChildren(node)) {
				decltype(child.children) children = child.children;
				child = node;
				child.children = children;
			}
		}

		node.is_leaf = false;
		return true;
	}

	void deleteChildren(INNER_NODE& node, DepthType depth, bool manual_pruning = false)
	{
		node.is_leaf = true;

		if (!node.children || (!manual_pruning && !automatic_pruning_enabled_)) {
			return;
		}

		if (1 == depth) {
			// Deleting leaf nodes
			delete &getLeafChildren(node);
			num_leaf_nodes_ -= 8;
			num_inner_leaf_nodes_ += 1;
		} else {
			// Deleting inner nodes
			std::array<INNER_NODE, 8>& children = getInnerChildren(node);
			for (INNER_NODE& child : children) {
				// Manual pruning is true in case automatic_pruning_enabled_ changes between calls
				deleteChildren(child, depth - 1, true);
			}
			delete &children;
			// Remove 8 and 1 inner node is made into a inner leaf node
			num_inner_leaf_nodes_ -= 7;
		}
		num_inner_nodes_ -= 1;
		node.children = nullptr;
	}

	//
	// Get children
	//

	static constexpr std::array<LEAF_NODE, 8>& getLeafChildren(INNER_NODE const& inner_node)
	{
		return *static_cast<std::array<LEAF_NODE, 8>*>(inner_node.children);
	}

	static constexpr std::array<INNER_NODE, 8>& getInnerChildren(
	    INNER_NODE const& inner_node)
	{
		return *static_cast<std::array<INNER_NODE, 8>*>(inner_node.children);
	}

	static LEAF_NODE& getLeafChild(INNER_NODE const& inner_node, unsigned int idx)
	{
		return getLeafChildren(inner_node)[idx];
	}

	static INNER_NODE& getInnerChild(INNER_NODE const& inner_node, unsigned int idx)
	{
		return getInnerChildren(inner_node)[idx];
	}

	static LEAF_NODE& getChild(INNER_NODE const& inner_node, DepthType child_depth,
	                           unsigned int idx)
	{
		if (0 == child_depth) {
			return getLeafChild(inner_node, idx);
		} else {
			return getInnerChild(inner_node, idx);
		}
	}

	//
	// Checking for children
	//

	static bool isLeaf(INNER_NODE const& node) noexcept { return node.is_leaf; }

	static bool isLeaf(LEAF_NODE const* node, DepthType depth) noexcept
	{
		return 0 == depth || isLeaf(static_cast<INNER_NODE const&>(*node));
	}

	static bool hasChildren(INNER_NODE const& node) noexcept { return !isLeaf(node); }

	static bool hasChildren(LEAF_NODE const* node, DepthType depth) noexcept
	{
		return 0 != depth && hasChildren(static_cast<INNER_NODE const&>(*node));
	}

	//
	// Node collapsible
	//

	bool isNodeCollapsible(INNER_NODE const& node, DepthType depth)
	{
		if (1 < depth) {
			for (int i = 0; i < 8; ++i) {
				if (hasChildren(getInnerChild(node, i))) {
					return false;
				}
			}
		}
		auto const& first_child = getChild(node, depth - 1, 0).value;
		for (int i = 1; i < 8; ++i) {
			if (first_child != getChild(node, depth - 1, i).value) {
				return false;
			}
		}

		return true;
	}

	//
	// Ray casting
	//

	// // TODO: Is correct?
	// void computeRayInitDiscrete(Point3 const& direction_normalized,
	//                             std::array<int, 3>& step, Point3& t_delta, Point3& t_max,
	//                             DepthType depth = 0) const
	// {
	// 	double node_size = getNodeSize(depth);
	// 	double node_half_size = getNodeHalfSize(depth);
	// 	for (std::size_t i = 0; i < 3; ++i) {
	// 		if (0 < direction_normalized[i]) {
	// 			step[i] = static_cast<int>(1U << depth);
	// 			t_delta[i] = node_size / std::abs(direction_normalized[i]);
	// 			t_max[i] = node_half_size / direction_normalized[i];
	// 		} else if (0 > direction_normalized[i]) {
	// 			step[i] = -static_cast<int>(1U << depth);
	// 			t_delta[i] = node_size / std::abs(direction_normalized[i]);
	// 			t_max[i] = -node_half_size / direction_normalized[i];
	// 		} else {
	// 			step[i] = 0;
	// 			t_delta[i] = std::numeric_limits<double>::max();
	// 			t_max[i] = std::numeric_limits<double>::max();
	// 		}
	// 	}
	// }

	void computeRayInit(Point3 const& origin, Point3 const& end,
	                    Point3 const& direction_normalized, Key& current, Key& ending,
	                    std::array<int, 3>& step, Point3& t_delta, Point3& t_max,
	                    DepthType depth = 0) const
	{
		current = toKey(origin, depth);
		ending = toKey(end, depth);

		if (current == ending) {
			return;
		}

		double node_size = getNodeSize(depth);
		double node_half_size = getNodeHalfSize(depth);
		Point3 voxel_border = toCoord(current) - origin;

		for (unsigned int i = 0; i < 3; ++i) {
			if (0 < direction_normalized[i]) {
				step[i] = static_cast<int>(1U << depth);
				voxel_border[i] += node_half_size;
				t_delta[i] = node_size / std::abs(direction_normalized[i]);
				t_max[i] = voxel_border[i] / direction_normalized[i];
			} else if (0 > direction_normalized[i]) {
				step[i] = -static_cast<int>(1U << depth);
				voxel_border[i] -= node_half_size;
				t_delta[i] = node_size / std::abs(direction_normalized[i]);
				t_max[i] = voxel_border[i] / direction_normalized[i];
			} else {
				step[i] = 0;
				t_delta[i] = std::numeric_limits<double>::max();
				t_max[i] = std::numeric_limits<double>::max();
			}
		}
	}

	static void computeRayTakeStep(Key& current, std::array<int, 3> const& step,
	                               Point3 const& t_delta, Point3& t_max)
	{
		std::size_t advance_dim = t_max.minElementIndex();
		current[advance_dim] += step[advance_dim];
		t_max[advance_dim] += t_delta[advance_dim];
	}

	//
	// Move line into BBX (octree bounds)
	//

	// TODO: Make one where only end is moved
	bool moveLineInside(Point3& origin, Point3& end) const
	{
		Point3 bbx_min = getMin();
		Point3 bbx_max = getMax();

		if ((origin[0] < bbx_min[0] && end[0] < bbx_min[0]) ||
		    (origin[0] > bbx_max[0] && end[0] > bbx_max[0]) ||
		    (origin[1] < bbx_min[1] && end[1] < bbx_min[1]) ||
		    (origin[1] > bbx_max[1] && end[1] > bbx_max[1]) ||
		    (origin[2] < bbx_min[2] && end[2] < bbx_min[2]) ||
		    (origin[2] > bbx_max[2] && end[2] > bbx_max[2])) {
			return false;
		}

		if (inBBX(origin, bbx_min, bbx_max) && inBBX(end, bbx_min, bbx_max)) {
			return true;
		}

		int hits = 0;
		std::array<Point3, 2> hit;
		for (int i = 0; i < 3 && hits < 2; ++i) {
			if (getIntersection(origin[i] - bbx_min[i], end[i] - bbx_min[i], origin, end,
			                    &hit[hits]) &&
			    inBBX(hit[hits], i, bbx_min, bbx_max)) {
				++hits;
			}
		}
		for (int i = 0; i < 3 && hits < 2; ++i) {
			if (getIntersection(origin[i] - bbx_max[i], end[i] - bbx_max[i], origin, end,
			                    &hit[hits]) &&
			    inBBX(hit[hits], i, bbx_min, bbx_max)) {
				++hits;
			}
		}

		switch (hits) {
			case 1:
				if (inBBX(origin, bbx_min, bbx_max)) {
					end = hit[0];
				} else {
					origin = hit[0];
				}
				break;
			case 2:
				if (((origin - hit[0]).squaredNorm() + (end - hit[1]).squaredNorm()) <=
				    ((origin - hit[1]).squaredNorm() + (end - hit[0]).squaredNorm())) {
					origin = hit[0];
					end = hit[1];
				} else {
					origin = hit[1];
					end = hit[0];
				}
		}

		return true;
	}

	bool isInside(Point3 const& point) const { return inBBX(point, getMin(), getMax()); }

	static bool inBBX(Point3 const& point, Point3 const& bbx_min, Point3 const& bbx_max)
	{
		return bbx_min.x() <= point.x() && bbx_max.x() >= point.x() &&
		       bbx_min.y() <= point.y() && bbx_max.y() >= point.y() &&
		       bbx_min.z() <= point.z() && bbx_max.z() >= point.z();
	}

	static bool getIntersection(double d_1, double d_2, Point3 const& p_1,
	                            Point3 const& p_2, Point3* hit)
	{
		if (0 <= (d_1 * d_2)) {
			return false;
		}
		*hit = p_1 + (p_2 - p_1) * (-d_1 / (d_2 - d_1));
		return true;
	}

	static bool inBBX(Point3 const& point, int axis, Point3 const& bbx_min,
	                  Point3 const& bbx_max)
	{
		if (0 == axis && point[2] > bbx_min[2] && point[2] < bbx_max[2] &&
		    point[1] > bbx_min[1] && point[1] < bbx_max[1]) {
			return true;
		}
		if (1 == axis && point[2] > bbx_min[2] && point[2] < bbx_max[2] &&
		    point[0] > bbx_min[0] && point[0] < bbx_max[0]) {
			return true;
		}
		if (2 == axis && point[0] > bbx_min[0] && point[0] < bbx_max[0] &&
		    point[1] > bbx_min[1] && point[1] < bbx_max[1]) {
			return true;
		}
		return false;
	}

	//
	// Input/output (read/write)
	//

	virtual bool readHeader(std::istream& s, std::string& file_version, std::string& id,
	                        double& resolution, DepthType& depth_levels, bool& compressed,
	                        int& uncompressed_data_size) const
	{
		file_version = "";
		id = "";
		resolution = 0.0;
		depth_levels = 0;
		compressed = false;
		uncompressed_data_size = -1;

		std::string token;
		bool header_read = false;
		while (s.good() && !header_read) {
			s >> token;
			if ("data" == token) {
				header_read = true;
				// Skip forward to the end of line
				char c;
				do {
					c = s.get();
				} while (s.good() && ('\n' != c));
			} else if (0 == token.compare(0, 1, "#")) {
				// Comment line, skip forward to the end of line
				char c;
				do {
					c = s.get();
				} while (s.good() && ('\n' != c));
			} else if ("version" == token) {
				s >> file_version;
			} else if ("id" == token) {
				s >> id;
			} else if ("resolution" == token) {
				s >> resolution;
			} else if ("depth_levels" == token) {
				s >> depth_levels;
			} else if ("compressed" == token) {
				s >> compressed;
			} else if ("uncompressed_data_size" == token) {
				s >> uncompressed_data_size;
			} else {
				// Other token
				char c;
				do {
					c = s.get();
				} while (s.good() && ('\n' != c));
			}
		}

		if (!header_read) {
			return false;
		}

		if ("" == file_version) {
			return false;
		}

		if ("" == id) {
			return false;
		}

		if (0.0 >= resolution) {
			return false;
		}

		if (0 == depth_levels) {
			return false;
		}

		if (0 > uncompressed_data_size) {
			return false;
		}

		if (getTreeType() != id) {
			// Wrong tree type
			return false;
		}

		return true;
	}

	virtual bool readNodes(std::istream& s,
	                       ufo::geometry::BoundingVolume const& bounding_volume) = 0;

	virtual bool writeNodes(std::ostream& s,
	                        ufo::geometry::BoundingVolume const& bounding_volume,
	                        DepthType min_depth) const = 0;

	//
	// Compress/decompress
	//

	bool compressData(std::istream& s_in, std::ostream& s_out, int uncompressed_data_size,
	                  int acceleration_level = 1, int compression_level = 0) const
	{
		// Compress data
		char* data = new char[uncompressed_data_size];
		s_in.read(data, uncompressed_data_size);
		const int max_dst_size = LZ4_compressBound(uncompressed_data_size);
		char* compressed_data = new char[max_dst_size];
		int compressed_data_size;
		if (0 >= compression_level) {
			compressed_data_size =
			    LZ4_compress_fast(data, compressed_data, uncompressed_data_size, max_dst_size,
			                      acceleration_level);
		} else {
			compressed_data_size = LZ4_compress_HC(
			    data, compressed_data, uncompressed_data_size, max_dst_size, compression_level);
		}

		// Check if compression successful
		if (0 <= compressed_data_size) {
			// Write compressed data to output stream
			s_out.write(compressed_data, compressed_data_size);
		}

		// Clean up
		delete[] data;
		delete[] compressed_data;
		return 0 <= compressed_data_size;
	}

	bool decompressData(std::istream& s_in, std::iostream& s_out,
	                    int uncompressed_data_size) const
	{
		// Get size of compressed data
		const std::streampos initial_read_position = s_in.tellg();
		s_in.seekg(0, s_in.end);
		const int compressed_data_size = s_in.tellg() - initial_read_position;
		s_in.seekg(initial_read_position);

		// Decompress data
		char* compressed_data = new char[compressed_data_size];
		s_in.read(compressed_data, compressed_data_size);
		char* regen_buffer = new char[uncompressed_data_size];
		const int decompressed_size = LZ4_decompress_safe(
		    compressed_data, regen_buffer, compressed_data_size, uncompressed_data_size);

		// Check if decompression successful
		if (0 <= decompressed_size) {
			// Write decompressed data to output stream
			s_out.write(regen_buffer, decompressed_size);
		}

		// Clean up
		delete[] compressed_data;
		delete[] regen_buffer;
		return 0 <= decompressed_size;
	}

 protected:
	double resolution_;         // The voxel size of the leaf nodes
	double resolution_factor_;  // Reciprocal of the resolution
	DepthType depth_levels_;    // The maximum depth of the octree
	KeyType max_value_;         // The maximum coordinate value the octree can store

	INNER_NODE root_;  // The root of the octree

	// Stores the half size of a node at a given depth, where the depth is the index
	std::array<double, MAX_DEPTH_LEVELS + 1> nodes_half_sizes_;

	// Automatic pruning
	bool automatic_pruning_enabled_ = true;

	// Memory
	size_t num_inner_nodes_ = 0;       // Current number of inner nodes
	size_t num_inner_leaf_nodes_ = 1;  // Current number of inner leaf nodes
	size_t num_leaf_nodes_ = 0;        // Current number of leaf nodes

	inline static const std::string FILE_HEADER = "# UFOMap file";  // File header
	inline static const std::string FILE_VERSION = "1.0.0";         // File version

	// TODO: Is this needed? I think so
	template <typename T, typename D, typename I, typename L, bool O>
	friend class OctreeIterator;
	template <typename T, typename D, typename I, typename L, bool O>
	friend class OctreeNearestIterator;
};
}  // namespace ufo::map

#endif  // UFO_MAP_OCTREE_H