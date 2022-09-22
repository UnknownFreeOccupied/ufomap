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

#ifndef UFO_MAP_OCCUPANCY_MAP_H
#define UFO_MAP_OCCUPANCY_MAP_H

// UFO
#include <ufo/geometry/aabb.h>
#include <ufo/map/io.h>
#include <ufo/map/node.h>
#include <ufo/map/occupancy/occupancy_node.h>
#include <ufo/map/occupancy/occupancy_predicate.h>
#include <ufo/map/point.h>
#include <ufo/map/predicate/satisfies.h>
#include <ufo/map/types.h>
#include <ufo/math/logit.h>

// STL
#include <cstdint>
#include <iostream>
#include <limits>
#include <type_traits>

namespace ufo::map
{
template <class Derived>
class OccupancyMap
{
 public:
	//
	// Sensor model
	//

	[[nodiscard]] constexpr occupancy_t occupancyClampingThresMin() const noexcept
	{
		return math::probability(occupancyClampingThresMinLogit());
	}

	[[nodiscard]] constexpr double occupancyClampingThresMinLogit() const noexcept
	{
		return clamping_thres_min_logit_;
	}

	[[nodiscard]] constexpr occupancy_t occupancyClampingThresMax() const noexcept
	{
		return math::probability(occupancyClampingThresMaxLogit());
	}

	[[nodiscard]] constexpr double occupancyClampingThresMaxLogit() const noexcept
	{
		return clamping_thres_max_logit_;
	}

	[[nodiscard]] constexpr occupancy_t occupiedThres() const noexcept
	{
		return toOccupancyProbability(occupiedThresLogit());
	}

	[[nodiscard]] constexpr logit_t occupiedThresLogit() const noexcept
	{
		return occupied_thres_logit_;
	}

	[[nodiscard]] constexpr occupancy_t freeThres() const noexcept
	{
		return toOccupancyProbability(freeThresLogit());
	}

	[[nodiscard]] constexpr logit_t freeThresLogit() const noexcept
	{
		return free_thres_logit_;
	}

	//
	// Probability <-> logit
	//

	[[nodiscard]] constexpr logit_t toOccupancyLogit(occupancy_t probability) const
	{
		return math::logit<logit_t>(probability, occupancyClampingThresMinLogit(),
		                            occupancyClampingThresMaxLogit());
	}

	[[nodiscard]] constexpr occupancy_t toOccupancyProbability(logit_t logit) const
	{
		if constexpr (std::is_floating_point_v<logit_t>) {
			return math::probability(logit);
		} else {
			return math::probability(logit, occupancyClampingThresMinLogit(),
			                         occupancyClampingThresMaxLogit());
		}
	}

	[[nodiscard]] constexpr logit_t toOccupancyChangeLogit(occupancy_t probability) const
	{
		if constexpr (std::is_floating_point_v<logit_t>) {
			return math::logit(probability);
		} else {
			return math::logitChangeValue<logit_t>(probability,
			                                       occupancyClampingThresMinLogit(),
			                                       occupancyClampingThresMaxLogit());
		}
	}

	//
	// Set sensor model
	//

	void setOccupiedFreeThres(occupancy_t new_occupied_thres, occupancy_t new_free_thres,
	                          bool propagate = true)
	{
		// FIXME: Should add a warning that these are very computational expensive to
		// call since the whole tree has to be updated

		setOccupiedFreeThresLogit(toOccupancyLogit(new_occupied_thres),
		                          toOccupancyLogit(new_free_thres), propagate);
	}

	// FIXME: Look at
	void setOccupiedFreeThresLogit(logit_t new_occupied_thres, logit_t new_free_thres,
	                               bool propagate = true)
	{
		// FIXME: Should add a warning that these are very computational expensive to
		// call since the whole tree has to be updated

		occupied_thres_logit_ = new_occupied_thres;
		free_thres_logit_ = new_free_thres;

		derived().setModifiedChildren();

		if (propagate) {
			derived().updateModifiedNodes();
		}
	}

	constexpr void setOccupancyClampingThres(occupancy_t min_thres_probability,
	                                         occupancy_t max_thres_probability)
	{
		setOccupancyClampingThresMin(min_thres_probability);
		setOccupancyClampingThresMax(max_thres_probability);
	}

	constexpr void setOccupancyClampingThresMin(occupancy_t probability)
	{
		clamping_thres_min_logit_ = math::logit(probability);
		// FIXME: Should this update occupancy values?
	}

	constexpr void setOccupancyClampingThresMax(occupancy_t probability)
	{
		clamping_thres_max_logit_ = math::logit(probability);
		// FIXME: Should this update occupancy values?
	}

	//
	// Propagation criteria
	//

	[[nodiscard]] constexpr PropagationCriteria occupancyPropagationCriteria()
	    const noexcept
	{
		return prop_criteria_;
	}

	constexpr void setOccupancyPropagationCriteria(
	    PropagationCriteria occupancy_prop_criteria, bool propagate = true) noexcept
	{
		if (prop_criteria_ == occupancy_prop_criteria) {
			return;
		}

		prop_criteria_ = occupancy_prop_criteria;

		// Set all inner nodes to modified
		// FIXME: Possible to optimize this to only set the ones with children
		derived().setModified(1);

		if (propagate) {
			derived().updateModifiedNodes();
		}
	}

	//
	// Get bounding volume containing all known, i.e., occupied and free space
	//

	[[nodiscard]] geometry::AABB knownBBX(depth_t min_depth = 0) const
	{
		if (!containsOccupied(derived().rootNode()) && !containsFree(derived().rootNode())) {
			return geometry::AABB();
		}

		Point min = derived().max();
		Point max = derived().min();

		auto pred = predicate::Leaf(min_depth) &&
		            predicate::OccupancyStates(false, true, true) &&
		            predicate::SatisfiesInner([this, &min, &max](auto const& node) {
			            auto node_min = derived().nodeMin(node);
			            auto node_max = derived().nodeMax(node);
			            return node_min.x < min.x || node_min.y < min.y || node_min.z < min.z ||
			                   node_max.x > max.x || node_max.y > max.y || node_max.z > max.z;
		            });

		for (auto const node : derived().queryBV(pred)) {
			auto node_min = derived().nodeMin(node);
			auto node_max = derived().nodeMax(node);
			min.x = std::min(min.x, node_min.x);
			min.y = std::min(min.y, node_min.y);
			min.z = std::min(min.z, node_min.z);
			max.x = std::max(max.x, node_max.x);
			max.y = std::max(max.y, node_max.y);
			max.z = std::max(max.z, node_max.z);
		}

		return geometry::AABB(min, max);
	}

	//
	// Get occupancy state
	//

	[[nodiscard]] OccupancyState occupancyState(Node node) const
	{
		auto const& leaf_node = derived().leafNode(node);
		auto const index = node.index();
		if (isUnknown(leaf_node, index)) {
			return OccupancyState::UNKNOWN;
		} else if (isFree(leaf_node, index)) {
			return OccupancyState::FREE;
		} else {
			return OccupancyState::OCCUPIED;
		}
	}

	[[nodiscard]] OccupancyState occupancyState(Code code) const
	{
		auto const& node = derived().leafNode(code);
		auto const index = code.index();
		if (isUnknown(node, index)) {
			return OccupancyState::UNKNOWN;
		} else if (isFree(node, index)) {
			return OccupancyState::FREE;
		} else {
			return OccupancyState::OCCUPIED;
		}
	}

	[[nodiscard]] OccupancyState occupancyState(Key key) const
	{
		return occupancyState(derived().toCode(key));
	}

	[[nodiscard]] OccupancyState occupancyState(Point coord, depth_t depth = 0) const
	{
		return occupancyState(derived().toCode(coord, depth));
	}

	[[nodiscard]] OccupancyState occupancyState(coord_t x, coord_t y, coord_t z,
	                                            depth_t depth = 0) const
	{
		return occupancyState(derived().toCode(x, y, z, depth));
	}

	//
	// Contains occupancy state
	//

	[[nodiscard]] bool containsOccupancyState(Node node, OccupancyState state) const
	{
		switch (state) {
			case OccupancyState::UNKNOWN:
				return containsUnknown(node);
			case OccupancyState::FREE:
				return containsFree(node);
			case OccupancyState::OCCUPIED:
				return containsOccupied(node);
		}
	}

	[[nodiscard]] bool containsOccupancyState(Code code, OccupancyState state) const
	{
		switch (state) {
			case OccupancyState::UNKNOWN:
				return containsUnknown(code);
			case OccupancyState::FREE:
				return containsFree(code);
			case OccupancyState::OCCUPIED:
				return containsOccupied(code);
		}
	}

	[[nodiscard]] bool containsOccupancyState(Key key, OccupancyState state) const
	{
		return containsOccupancyState(derived().toCode(key), state);
	}

	[[nodiscard]] bool containsOccupancyState(Point coord, OccupancyState state,
	                                          depth_t depth = 0) const
	{
		return containsOccupancyState(derived().toCode(coord, depth), state);
	}

	[[nodiscard]] bool containsOccupancyState(coord_t x, coord_t y, coord_t z,
	                                          OccupancyState state, depth_t depth = 0) const
	{
		return containsOccupancyState(derived().toCode(x, y, z, depth), state);
	}

	//
	// Is unknown
	//

	[[nodiscard]] bool isUnknown(Node node) const
	{
		return isUnknown(derived().leafNode(node), node.index());
	}

	[[nodiscard]] bool isUnknown(Code code) const
	{
		return isUnknown(derived().leafNode(code), code.index());
	}

	[[nodiscard]] bool isUnknown(Key key) const { return isUnknown(derived().toCode(key)); }

	[[nodiscard]] bool isUnknown(Point coord, depth_t depth = 0) const
	{
		return isUnknown(derived().toCode(coord, depth));
	}

	[[nodiscard]] bool isUnknown(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return isUnknown(derived().toCode(x, y, z, depth));
	}

	//
	// Is free
	//

	[[nodiscard]] bool isFree(Node node) const
	{
		return isFree(derived().leafNode(node), node.index());
	}

	[[nodiscard]] bool isFree(Code code) const
	{
		return isFree(derived().leafNode(code), code.index());
	}

	[[nodiscard]] bool isFree(Key key) const { return isFree(derived().toCode(key)); }

	[[nodiscard]] bool isFree(Point coord, depth_t depth = 0) const
	{
		return isFree(derived().toCode(coord, depth));
	}

	[[nodiscard]] bool isFree(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return isFree(derived().toCode(x, y, z, depth));
	}

	//
	// Is occupied
	//

	[[nodiscard]] bool isOccupied(Node node) const
	{
		return isOccupied(derived().leafNode(node), node.index());
	}

	[[nodiscard]] bool isOccupied(Code code) const
	{
		return isOccupied(derived().leafNode(code), code.index());
	}

	[[nodiscard]] bool isOccupied(Key key) const
	{
		return isOccupied(derived().toCode(key));
	}

	[[nodiscard]] bool isOccupied(Point coord, depth_t depth = 0) const
	{
		return isOccupied(derived().toCode(coord, depth));
	}

	[[nodiscard]] bool isOccupied(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return isOccupied(derived().toCode(x, y, z, depth));
	}

	//
	// Contains unknown
	//

	[[nodiscard]] bool containsUnknown(Node node) const
	{
		return 0 == node.depth()
		           ? isUnknown(node)
		           : derived().innerNode(node).containsUnknownIndex(node.index());
	}

	[[nodiscard]] bool containsUnknown(Code code) const
	{
		if (0 == code.depth()) {
			return isUnknown(code);
		}
		auto [n, d] = derived().innerNodeAndDepth(code);
		return n.containsUnknownIndex(code.index(d));
	}

	[[nodiscard]] bool containsUnknown(Key key) const
	{
		return containsUnknown(derived().toCode(key));
	}

	[[nodiscard]] bool containsUnknown(Point coord, depth_t depth = 0) const
	{
		return containsUnknown(derived().toCode(coord, depth));
	}

	[[nodiscard]] bool containsUnknown(coord_t x, coord_t y, coord_t z,
	                                   depth_t depth = 0) const
	{
		return containsUnknown(derived().toCode(x, y, z, depth));
	}

	//
	// Contains free
	//

	[[nodiscard]] bool containsFree(Node node) const
	{
		return containsFree(derived().leafNode(node), node.index());
	}

	[[nodiscard]] bool containsFree(Code code) const
	{
		return containsFree(derived().leafNode(code), code.index());
	}

	[[nodiscard]] bool containsFree(Key key) const
	{
		return containsFree(derived().toCode(key));
	}

	[[nodiscard]] bool containsFree(Point coord, depth_t depth = 0) const
	{
		return containsFree(derived().toCode(coord, depth));
	}

	[[nodiscard]] bool containsFree(coord_t x, coord_t y, coord_t z,
	                                depth_t depth = 0) const
	{
		return containsFree(derived().toCode(x, y, z, depth));
	}

	//
	// Contains occupied
	//

	[[nodiscard]] bool containsOccupied(Node node) const
	{
		return containsOccupied(derived().leafNode(node), node.index());
	}

	[[nodiscard]] bool containsOccupied(Code code) const
	{
		return containsOccupied(derived().leafNode(code), code.index());
	}

	[[nodiscard]] bool containsOccupied(Key key) const
	{
		return containsOccupied(derived().toCode(key));
	}

	[[nodiscard]] bool containsOccupied(Point coord, depth_t depth = 0) const
	{
		return containsOccupied(derived().toCode(coord, depth));
	}

	[[nodiscard]] bool containsOccupied(coord_t x, coord_t y, coord_t z,
	                                    depth_t depth = 0) const
	{
		return containsOccupied(derived().toCode(x, y, z, depth));
	}

	//
	// Get occupancy
	//

	[[nodiscard]] occupancy_t occupancy(Node node) const
	{
		return occupancy(derived().leafNode(node), node.index());
	}

	[[nodiscard]] occupancy_t occupancy(Code code) const
	{
		return occupancy(derived().leafNode(code), code.index());
	}

	[[nodiscard]] occupancy_t occupancy(Key key) const
	{
		return occupancy(derived().toCode(key));
	}

	[[nodiscard]] occupancy_t occupancy(Point coord, depth_t depth = 0) const
	{
		return occupancy(derived().toCode(coord, depth));
	}

	[[nodiscard]] occupancy_t occupancy(coord_t x, coord_t y, coord_t z,
	                                    depth_t depth = 0) const
	{
		return occupancy(derived().toCode(x, y, z, depth));
	}

	//
	// Get occupancy logit
	//

	[[nodiscard]] logit_t occupancyLogit(Node node) const
	{
		return occupancyLogit(derived().leafNode(node), node.index());
	}

	[[nodiscard]] logit_t occupancyLogit(Code code) const
	{
		return occupancyLogit(derived().leafNode(code), code.index());
	}

	[[nodiscard]] logit_t occupancyLogit(Key key) const
	{
		return occupancyLogit(derived().toCode(key));
	}

	[[nodiscard]] logit_t occupancyLogit(Point coord, depth_t depth = 0) const
	{
		return occupancyLogit(derived().toCode(coord, depth));
	}

	[[nodiscard]] logit_t occupancyLogit(coord_t x, coord_t y, coord_t z,
	                                     depth_t depth = 0) const
	{
		return occupancyLogit(derived().toCode(x, y, z, depth));
	}

	//
	// Set occupancy
	//

	void setOccupancy(Node node, occupancy_t occupancy, bool propagate = true)
	{
		logit_t logit = toOccupancyLogit(occupancy);
		setOccupancyLogit(node, logit, propagate);
	}

	void setOccupancy(Code code, occupancy_t occupancy, bool propagate = true)
	{
		logit_t logit = toOccupancyLogit(occupancy);
		setOccupancyLogit(code, logit, propagate);
	}

	void setOccupancy(Key key, occupancy_t occupancy, bool propagate = true)
	{
		setOccupancy(derived().toCode(key), occupancy, propagate);
	}

	void setOccupancy(Point coord, occupancy_t occupancy, bool propagate = true,
	                  depth_t depth = 0)
	{
		setOccupancy(derived().toCode(coord, depth), occupancy, propagate);
	}

	void setOccupancy(coord_t x, coord_t y, coord_t z, occupancy_t occupancy,
	                  bool propagate = true, depth_t depth = 0)
	{
		setOccupancy(derived().toCode(x, y, z, depth), occupancy, propagate);
	}

	//
	// Set occupancy logit
	//

	void setOccupancyLogit(Node node, logit_t logit, bool propagate = true)
	{
		derived().apply(
		    node,
		    [this, logit](auto& node, index_t const index) {
			    setOccupancyLogit(node, index, logit);
		    },
		    [this, logit](auto& node) { setOccupancyLogit(node, logit); }, propagate);
	}

	void setOccupancyLogit(Code code, logit_t logit, bool propagate = true)
	{
		derived().apply(
		    code,
		    [this, logit](auto& node, index_t const index) {
			    setOccupancyLogit(node, index, logit);
		    },
		    [this, logit](auto& node) { setOccupancyLogit(node, logit); }, propagate);
	}

	void setOccupancyLogit(Key key, logit_t logit, bool propagate = true)
	{
		setOccupancyLogit(derived().toCode(key), logit, propagate);
	}

	void setOccupancyLogit(Point coord, logit_t logit, bool propagate = true,
	                       depth_t depth = 0)
	{
		setOccupancyLogit(derived().toCode(coord, depth), logit, propagate);
	}

	void setOccupancyLogit(coord_t x, coord_t y, coord_t z, logit_t logit,
	                       bool propagate = true, depth_t depth = 0)
	{
		setOccupancyLogit(derived().toCode(x, y, z, depth), logit, propagate);
	}

	//
	// Increase occupancy logit
	//

	void increaseOccupancyLogit(Node node, logit_t inc, bool propagate = true)
	{
		derived().apply(
		    node,
		    [this, inc](auto& node, index_t const index) {
			    increaseOccupancyLogit(node, index, inc);
		    },
		    [this, inc](auto& node) { increaseOccupancyLogit(node, inc); }, propagate);
	}

	void increaseOccupancyLogit(Code code, logit_t inc, bool propagate = true)
	{
		derived().apply(
		    code,
		    [this, inc](auto& node, index_t const index) {
			    increaseOccupancyLogit(node, index, inc);
		    },
		    [this, inc](auto& node) { increaseOccupancyLogit(node, inc); }, propagate);
	}

	void increaseOccupancyLogit(Key key, logit_t inc, bool propagate = true)
	{
		increaseOccupancyLogit(derived().toCode(key), inc, propagate);
	}

	void increaseOccupancyLogit(Point coord, logit_t inc, bool propagate = true,
	                            depth_t depth = 0)
	{
		increaseOccupancyLogit(derived().toCode(coord, depth), inc, propagate);
	}

	void increaseOccupancyLogit(coord_t x, coord_t y, coord_t z, logit_t inc,
	                            bool propagate = true, depth_t depth = 0)
	{
		increaseOccupancyLogit(derived().toCode(x, y, z, depth), inc, propagate);
	}

	//
	// Decrease occupancy logit
	//

	void decreaseOccupancyLogit(Node node, logit_t dec, bool propagate = true)
	{
		derived().apply(
		    node,
		    [this, dec](auto& node, index_t const index) {
			    decreaseOccupancyLogit(node, index, dec);
		    },
		    [this, dec](auto& node) { decreaseOccupancyLogit(node, dec); }, propagate);
	}

	void decreaseOccupancyLogit(Code code, logit_t dec, bool propagate = true)
	{
		derived().apply(
		    code,
		    [this, dec](auto& node, index_t const index) {
			    decreaseOccupancyLogit(node, index, dec);
		    },
		    [this, dec](auto& node) { decreaseOccupancyLogit(node, dec); }, propagate);
	}

	void decreaseOccupancyLogit(Key key, logit_t dec, bool propagate = true)
	{
		decreaseOccupancyLogit(derived().toCode(key), dec, propagate);
	}

	void decreaseOccupancyLogit(Point coord, logit_t dec, bool propagate = true,
	                            depth_t depth = 0)
	{
		decreaseOccupancyLogit(derived().toCode(coord, depth), dec, propagate);
	}

	void decreaseOccupancyLogit(coord_t x, coord_t y, coord_t z, logit_t dec,
	                            bool propagate = true, depth_t depth = 0)
	{
		decreaseOccupancyLogit(derived().toCode(x, y, z, depth), dec, propagate);
	}

	//
	// Increase occupancy
	//

	void increaseOccupancy(Node node, occupancy_t inc, bool propagate = true)
	{
		increaseOccupancyLogit(node, toOccupancyChangeLogit(inc), propagate);
	}

	void increaseOccupancy(Code code, occupancy_t inc, bool propagate = true)
	{
		increaseOccupancyLogit(code, toOccupancyChangeLogit(inc), propagate);
	}

	void increaseOccupancy(Key key, occupancy_t inc, bool propagate = true)
	{
		increaseOccupancy(derived().toCode(key), inc, propagate);
	}

	void increaseOccupancy(Point coord, occupancy_t inc, bool propagate = true,
	                       depth_t depth = 0)
	{
		increaseOccupancy(derived().toCode(coord, depth), inc, propagate);
	}

	void increaseOccupancy(coord_t x, coord_t y, coord_t z, occupancy_t inc,
	                       bool propagate = true, depth_t depth = 0)
	{
		increaseOccupancy(derived().toCode(x, y, z, depth), inc, propagate);
	}

	//
	// Decrease occupancy
	//

	void decreaseOccupancy(Node node, occupancy_t dec, bool propagate = true)
	{
		decreaseOccupancyLogit(node, toOccupancyChangeLogit(dec), propagate);
	}

	void decreaseOccupancy(Code code, occupancy_t dec, bool propagate = true)
	{
		decreaseOccupancyLogit(code, toOccupancyChangeLogit(dec), propagate);
	}

	void decreaseOccupancy(Key key, occupancy_t dec, bool propagate = true)
	{
		decreaseOccupancy(derived().toCode(key), dec, propagate);
	}

	void decreaseOccupancy(Point coord, occupancy_t dec, bool propagate = true,
	                       depth_t depth = 0)
	{
		decreaseOccupancy(derived().toCode(coord, depth), dec, propagate);
	}

	void decreaseOccupancy(coord_t x, coord_t y, coord_t z, occupancy_t dec,
	                       bool propagate = true, depth_t depth = 0)
	{
		decreaseOccupancy(derived().toCode(x, y, z, depth), dec, propagate);
	}

 protected:
	//
	// Constructors
	//

	OccupancyMap() = default;

	OccupancyMap(OccupancyMap const& other) = default;

	OccupancyMap(OccupancyMap&& other) = default;

	template <class Derived2>
	OccupancyMap(OccupancyMap<Derived2> const& other)
	    : clamping_thres_min_logit_(other.occupancyClampingThresMinLogit()),
	      clamping_thres_max_logit_(other.occupancyClampingThresMaxLogit()),
	      occupied_thres_logit_(other.occupiedThresLogit()),
	      free_thres_logit_(other.freeThresLogit())
	{
	}

	//
	// Assignment operator
	//

	OccupancyMap& operator=(OccupancyMap const& rhs) = default;

	OccupancyMap& operator=(OccupancyMap&& rhs) = default;

	template <class Derived2>
	OccupancyMap& operator=(OccupancyMap<Derived2> const& rhs)
	{
		clamping_thres_min_logit_ = rhs.occupancyClampingThresMinLogit();
		clamping_thres_max_logit_ = rhs.occupancyClampingThresMaxLogit();
		occupied_thres_logit_ = rhs.occupiedThresLogit();
		free_thres_logit_ = rhs.freeThresLogit();
		return *this;
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

	void initRoot()
	{
		auto const occ = toOccupancyLogit(0.5);
		derived().root().setOccupancyIndex(derived().rootIndex(), occ);
		derived().root().setContainsUnknownIndex(derived().rootIndex(), isUnknown(occ));
		derived().root().setContainsFreeIndex(derived().rootIndex(), isFree(occ));
		derived().root().setContainsOccupiedIndex(derived().rootIndex(), isOccupied(occ));
	}

	//
	// Update node
	//

	template <std::size_t N, class T>
	void updateNode(OccupancyNode<N>& node, IndexField const indices, T const& children)
	{
		if constexpr (1 == N) {
			auto fun = [](OccupancyNode<1> const node) { return node.occupancy[0]; };
			switch (prop_criteria_) {
				case PropagationCriteria::MIN:
					node.occupancy[0] = min(children, fun);
					break;
				case PropagationCriteria::MAX:
					node.occupancy[0] = max(children, fun);
					break;
				case PropagationCriteria::MEAN:
					node.occupancy[0] = mean(children, fun);
					break;
			}

			node.contains_unknown =
			    any_of(children, [this](auto const& child) { return containsUnknown(child); });
			node.contains_free =
			    any_of(children, [this](auto const& child) { return containsFree(child); });
			node.contains_occupied =
			    any_of(children, [this](auto const& child) { return containsOccupied(child); });
		} else {
			for (std::size_t index = 0; children.size() != index; ++index) {
				if (!indices[index]) {
					continue;
				}

				switch (prop_criteria_) {
					case PropagationCriteria::MIN:
						node.occupancy[index] = min(children[index].occupancy);
						break;
					case PropagationCriteria::MAX:
						node.occupancy[index] = max(children[index].occupancy);
						break;
					case PropagationCriteria::MEAN:
						node.occupancy[index] = mean(children[index].occupancy);
						break;
				}

				if (containsUnknown(children[index])) {
					node.setContainsUnknownIndex(index);
				} else {
					node.resetContainsUnknownIndex(index);
				}
				if (containsFree(children[index])) {
					node.setContainsFreeIndex(index);
				} else {
					node.resetContainsFreeIndex(index);
				}
				if (containsOccupied(children[index])) {
					node.setContainsOccupiedIndex(index);
				} else {
					node.resetContainsOccupiedIndex(index);
				}
			}
		}
	}

	//
	// Is
	//

	[[nodiscard]] constexpr bool isUnknown(logit_t const logit) const noexcept
	{
		return freeThresLogit() <= logit && occupiedThresLogit() >= logit;
	}

	[[nodiscard]] constexpr bool isFree(logit_t const logit) const noexcept
	{
		return freeThresLogit() > logit;
	}

	[[nodiscard]] constexpr bool isOccupied(logit_t const logit) const noexcept
	{
		return occupiedThresLogit() < logit;
	}

	//
	// Contains
	//

	template <class T>
	[[nodiscard]] bool containsUnknown(T const& node)
	{
		if constexpr (std::is_base_of_v<ContainsOccupancy<node.occupancySize()>, T>) {
			return node.containsUnknown();
		} else {
			return any_of(node.occupancy, [this](auto const occ) { isUnknown(occ); });
		}
	}

	template <class T>
	[[nodiscard]] bool containsFree(T const& node)
	{
		if constexpr (std::is_base_of_v<ContainsOccupancy<node.occupancySize()>, T>) {
			return node.containsFree();
		} else {
			return any_of(node.occupancy, [this](auto const occ) { isFree(occ); });
		}
	}

	template <class T>
	[[nodiscard]] bool containsOccupied(T const& node)
	{
		if constexpr (std::is_base_of_v<ContainsOccupancy<node.occupancySize()>, T>) {
			return node.containsOccupied();
		} else {
			return any_of(node.occupancy, [this](auto const occ) { isOccupied(occ); });
		}
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr DataIdentifier dataIdentifier() noexcept
	{
		return DataIdentifier::OCCUPANCY;
	}

	[[nodiscard]] static constexpr bool canReadData(DataIdentifier identifier) noexcept
	{
		return dataIdentifier() == identifier;
	}

	template <class OutputIt>
	void readNodes(std::istream& in, OutputIt first, std::size_t num_nodes)
	{
		uint8_t n;
		in.read(reinterpret_cast<char*>(&n), sizeof(n));
		num_nodes *= n;

		auto data = std::make_unique<logit_t[]>(num_nodes);
		in.read(reinterpret_cast<char*>(data.get()),
		        num_nodes * sizeof(typename decltype(data)::element_type));

		auto const d = data.get();
		if constexpr (1 == numData<OutputIt>()) {
			if (1 == n) {
				for (std::size_t i = 0; i != num_nodes; ++first, ++i) {
					first->node.occupancy[0] = *(d + i);
				}
			} else {
				auto const prop_criteria = prop_criteria_;
				for (std::size_t i = 0; i != num_nodes; ++first, i += 8) {
					switch (prop_criteria) {
						case PropagationCriteria::MIN:
							first->node.occupancy[0] = min(d + i, d + i + 8);
							break;
						case PropagationCriteria::MAX:
							first->node.occupancy[0] = max(d + i, d + i + 8);
							break;
						case PropagationCriteria::MEAN:
							first->node.occupancy[0] = mean(d + i, d + i + 8);
							break;
					}
				}
			}
		} else {
			if (1 == n) {
				for (std::size_t i = 0; i != num_nodes; ++first, ++i) {
					if (first->index_field.all()) {
						first->node.occupancy.fill(*(d + i));
					} else {
						for (std::size_t index = 0; first->node.occupancy.size() != index; ++index) {
							if (first.index_field[index]) {
								first->node.occupancy[index] = *(d + i);
							}
						}
					}
				}
			} else {
				for (std::size_t i = 0; i != num_nodes; ++first, i += 8) {
					if (first->index_field.all()) {
						std::copy(d + i, d + i + 8, first->node.occupancy.data());
					} else {
						for (index_t index = 0; first->node.occupancy.size() != index; ++i, ++index) {
							if (first.index_field[index]) {
								first->node.occupancy[index] = *(d + i + index);
							}
						}
					}
				}
			}
		}
	}

	template <class InputIt>
	void writeNodes(std::ostream& out, InputIt first, std::size_t num_nodes)
	{
		constexpr uint8_t const n = numData<InputIt>();
		num_nodes *= n;

		auto data = std::make_unique<logit_t[]>(num_nodes);
		auto d = data.get();
		for (std::size_t i = 0; i != num_nodes; ++first, i += n) {
			std::copy(std::cbegin(first->node.occupancy), std::cend(first->node.occupancy),
			          d + i);
		}

		out.write(reinterpret_cast<char const*>(&n), sizeof(n));
		out.write(reinterpret_cast<char const*>(data.get()),
		          num_nodes * sizeof(typename decltype(data)::element_type));
	}

 protected:
	double clamping_thres_min_logit_ = math::logit(0.1192);  // Min logit value
	double clamping_thres_max_logit_ = math::logit(0.971);   // Max logit value

	logit_t occupied_thres_logit_ = toOccupancyLogit(0.5);  // Threshold for occupied
	logit_t free_thres_logit_ = toOccupancyLogit(0.5);      // Threshold for free

	// Propagation criteria
	PropagationCriteria prop_criteria_ = PropagationCriteria::MAX;
};
}  // namespace ufo::map

#endif  // UFO_MAP_OCCUPANCY_MAP_H