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

#ifndef UFO_MAP_OCCUPANCY_MAP_BASE_H
#define UFO_MAP_OCCUPANCY_MAP_BASE_H

// UFO
#include <ufo/geometry/aabb.h>
#include <ufo/map/io.h>
#include <ufo/map/occupancy/occupancy_indicators.h>
#include <ufo/map/occupancy/occupancy_node.h>
#include <ufo/map/octree/node.h>
#include <ufo/map/point.h>
#include <ufo/map/predicate/occupancy.h>
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
class OccupancyMapBase
{
 public:
	//
	// Get sensor model
	//

	constexpr occupancy_t getOccupancyClampingThresMin() const noexcept
	{
		return math::probability(getOccupancyClampingThresMinLogit());
	}

	constexpr occupancy_t getOccupancyClampingThresMinLogit() const noexcept
	{
		return clamping_thres_min_logit_;
	}

	constexpr occupancy_t getOccupancyClampingThresMax() const noexcept
	{
		return math::probability(getOccupancyClampingThresMaxLogit());
	}

	constexpr occupancy_t getOccupancyClampingThresMaxLogit() const noexcept
	{
		return clamping_thres_max_logit_;
	}

	constexpr occupancy_t getOccupiedThres() const noexcept
	{
		return toOccupancyProbability(getOccupiedThresLogit());
	}

	constexpr logit_t getOccupiedThresLogit() const noexcept
	{
		return occupied_thres_logit_;
	}

	constexpr occupancy_t getFreeThres() const noexcept
	{
		return toOccupancyProbability(getFreeThresLogit());
	}

	constexpr logit_t getFreeThresLogit() const noexcept { return free_thres_logit_; }

	//
	// Probability <-> logit
	//

	constexpr logit_t toOccupancyLogit(occupancy_t probability) const
	{
		return math::logit<logit_t>(probability, getOccupancyClampingThresMinLogit(),
		                            getOccupancyClampingThresMaxLogit());
	}

	constexpr occupancy_t toOccupancyProbability(logit_t logit) const
	{
		if constexpr (std::is_same_v<logit_t, uint8_t>) {
			return math::probability(logit, getOccupancyClampingThresMinLogit(),
			                         getOccupancyClampingThresMaxLogit());
		} else {
			return math::probability(logit);
		}
	}

	constexpr logit_t toOccupancyChangeLogit(occupancy_t probability) const
	{
		if constexpr (std::is_same_v<logit_t, uint8_t>) {
			return math::logitChangeValue<uint8_t>(probability,
			                                       getOccupancyClampingThresMinLogit(),
			                                       getOccupancyClampingThresMaxLogit());
		} else {
			return math::logit(probability);
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

	constexpr PropagationCriteria getOccupancyPropagationCriteria() const noexcept
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

	geometry::AABB getKnownBBX() const
	{
		if (!containsOccupied(derived().getRootNode()) &&
		    !containsFree(derived().getRootNode())) {
			return geometry::AABB();
		}

		Point3 min = derived().max();
		Point3 max = derived().min();

		auto pred = predicate::Leaf() && predicate::OccupancyStates(false, true, true) &&
		            predicate::SatisfiesInner([this, &min, &max](auto const& node) {
			            auto node_min = derived().getNodeMin(node);
			            auto node_max = derived().getNodeMax(node);
			            return node_min.x < min.x || node_min.y < min.y || node_min.z < min.z ||
			                   node_max.x > max.x || node_max.y > max.y || node_max.z > max.z;
		            });

		for (auto const node : derived().queryBV(pred)) {
			auto node_min = derived().getNodeMin(node);
			auto node_max = derived().getNodeMax(node);
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

	OccupancyState getOccupancyState(Node node) const
	{
		if (isUnknown(derived().getLeafNode(node))) {
			return OccupancyState::UNKNOWN;
		} else if (isFree(derived().getLeafNode(node))) {
			return OccupancyState::FREE;
		} else {
			return OccupancyState::OCCUPIED;
		}
	}

	OccupancyState getOccupancyState(Code code) const
	{
		auto const& node = derived().getLeafNode(code);
		if (isUnknown(node)) {
			return OccupancyState::UNKNOWN;
		} else if (isFree(node)) {
			return OccupancyState::FREE;
		} else {
			return OccupancyState::OCCUPIED;
		}
	}

	OccupancyState getOccupancyState(Key key) const
	{
		return getOccupancyState(derived().toCode(key));
	}

	OccupancyState getOccupancyState(Point3 coord, depth_t depth = 0) const
	{
		return getOccupancyState(derived().toCode(coord, depth));
	}

	OccupancyState getOccupancyState(coord_t x, coord_t y, coord_t z,
	                                 depth_t depth = 0) const
	{
		return getOccupancyState(derived().toCode(x, y, z, depth));
	}

	//
	// Contains occupancy state
	//

	bool containsOccupancyState(Node node, OccupancyState state) const
	{
		switch (state) {
			case OccupancyState::UNKNOWN:
				return containsUnknown(derived().getLeafNode(node));
			case OccupancyState::FREE:
				return containsFree(derived().getLeafNode(node));
			case OccupancyState::OCCUPIED:
				return containsOccupied(derived().getLeafNode(node));
		}
	}

	bool containsOccupancyState(Code code, OccupancyState state) const
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

	bool containsOccupancyState(Key key, OccupancyState state) const
	{
		return containsOccupancyState(derived().toCode(key), state);
	}

	bool containsOccupancyState(Point3 coord, OccupancyState state, depth_t depth = 0) const
	{
		return containsOccupancyState(derived().toCode(coord, depth), state);
	}

	bool containsOccupancyState(coord_t x, coord_t y, coord_t z, OccupancyState state,
	                            depth_t depth = 0) const
	{
		return containsOccupancyState(derived().toCode(x, y, z, depth), state);
	}

	//
	// Is unknown
	//

	bool isUnknown(Node node) const { return isUnknown(derived().getLeafNode(node)); }

	bool isUnknown(Code code) const { return isUnknown(derived().getLeafNode(code)); }

	bool isUnknown(Key key) const { return isUnknown(derived().toCode(key)); }

	bool isUnknown(Point3 coord, depth_t depth = 0) const
	{
		return isUnknown(derived().toCode(coord, depth));
	}

	bool isUnknown(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return isUnknown(derived().toCode(x, y, z, depth));
	}

	//
	// Is free
	//

	bool isFree(Node node) const { return isFree(derived().getLeafNode(node)); }

	bool isFree(Code code) const { return isFree(derived().getLeafNode(code)); }

	bool isFree(Key key) const { return isFree(derived().toCode(key)); }

	bool isFree(Point3 coord, depth_t depth = 0) const
	{
		return isFree(derived().toCode(coord, depth));
	}

	bool isFree(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return isFree(derived().toCode(x, y, z, depth));
	}

	//
	// Is occupied
	//

	bool isOccupied(Node node) const { return isOccupied(derived().getLeafNode(node)); }

	bool isOccupied(Code code) const { return isOccupied(derived().getLeafNode(code)); }

	bool isOccupied(Key key) const { return isOccupied(derived().toCode(key)); }

	bool isOccupied(Point3 coord, depth_t depth = 0) const
	{
		return isOccupied(derived().toCode(coord, depth));
	}

	bool isOccupied(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return isOccupied(derived().toCode(x, y, z, depth));
	}

	//
	// Contains unknown
	//

	bool containsUnknown(Node node) const
	{
		return containsUnknown(derived().getLeafNode(node));
	}

	bool containsUnknown(Code code) const { containsUnknown(derived().getLeafNode(code)); }

	bool containsUnknown(Key key) const { return containsUnknown(derived().toCode(key)); }

	bool containsUnknown(Point3 coord, depth_t depth = 0) const
	{
		return containsUnknown(derived().toCode(coord, depth));
	}

	bool containsUnknown(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return containsUnknown(derived().toCode(x, y, z, depth));
	}

	//
	// Contains free
	//

	bool containsFree(Node node) const { return containsFree(derived().getLeafNode(node)); }

	bool containsFree(Code code) const { return containsFree(derived().getLeafNode(code)); }

	bool containsFree(Key key) const { return containsFree(derived().toCode(key)); }

	bool containsFree(Point3 coord, depth_t depth = 0) const
	{
		return containsFree(derived().toCode(coord, depth));
	}

	bool containsFree(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return containsFree(derived().toCode(x, y, z, depth));
	}

	//
	// Contains occupied
	//

	bool containsOccupied(Node node) const
	{
		return containsOccupied(derived().getLeafNode(node));
	}

	bool containsOccupied(Code code) const
	{
		return containsOccupied(derived().getLeafNode(code));
	}

	bool containsOccupied(Key key) const { return containsOccupied(derived().toCode(key)); }

	bool containsOccupied(Point3 coord, depth_t depth = 0) const
	{
		return containsOccupied(derived().toCode(coord, depth));
	}

	bool containsOccupied(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return containsOccupied(derived().toCode(x, y, z, depth));
	}

	//
	// Get occupancy
	//

	occupancy_t getOccupancy(Node node) const
	{
		return getOccupancy(derived().getLeafNode(node));
	}

	occupancy_t getOccupancy(Code code) const
	{
		return getOccupancy(derived().getLeafNode(code));
	}

	occupancy_t getOccupancy(Key key) const { return getOccupancy(derived().toCode(key)); }

	occupancy_t getOccupancy(Point3 coord, depth_t depth = 0) const
	{
		return getOccupancy(derived().toCode(coord, depth));
	}

	occupancy_t getOccupancy(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return getOccupancy(derived().toCode(x, y, z, depth));
	}

	//
	// Get occupancy logit
	//

	logit_t getOccupancyLogit(Node node) const
	{
		return getOccupancyLogit(derived().getLeafNode(node));
	}

	logit_t getOccupancyLogit(Code code) const
	{
		return getOccupancyLogit(derived().getLeafNode(code));
	}

	logit_t getOccupancyLogit(Key key) const
	{
		return getOccupancyLogit(derived().toCode(key));
	}

	logit_t getOccupancyLogit(Point3 coord, depth_t depth = 0) const
	{
		return getOccupancyLogit(derived().toCode(coord, depth));
	}

	logit_t getOccupancyLogit(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return getOccupancyLogit(derived().toCode(x, y, z, depth));
	}

	//
	// Set occupancy
	//

	void setOccupancy(Node& node, occupancy_t occupancy, bool propagate = true)
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

	void setOccupancy(Point3 coord, occupancy_t occupancy, bool propagate = true,
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
		    node, [this, logit](OccupancyNode& node) { setOccupancyLogit(node, logit); },
		    propagate);
	}

	void setOccupancyLogit(Code code, logit_t logit, bool propagate = true)
	{
		derived().apply(
		    code, [this, logit](OccupancyNode& node) { setOccupancyLogit(node, logit); },
		    propagate);
	}

	void setOccupancyLogit(Key key, logit_t logit, bool propagate = true)
	{
		setOccupancyLogit(derived().toCode(key), logit, propagate);
	}

	void setOccupancyLogit(Point3 coord, logit_t logit, bool propagate = true,
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

	void increaseOccupancyLogit(Node& node, logit_t inc, bool propagate = true)
	{
		derived().apply(
		    node, [this, inc](OccupancyNode& node) { increaseOccupancyLogit(node, inc); },
		    propagate);
	}

	void increaseOccupancyLogit(Code code, logit_t inc, bool propagate = true)
	{
		derived().apply(
		    code, [this, inc](OccupancyNode& node) { increaseOccupancyLogit(node, inc); },
		    propagate);
	}

	void increaseOccupancyLogit(Key key, logit_t inc, bool propagate = true)
	{
		increaseOccupancyLogit(derived().toCode(key), inc, propagate);
	}

	void increaseOccupancyLogit(Point3 coord, logit_t inc, bool propagate = true,
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

	void decreaseOccupancyLogit(Node& node, logit_t dec, bool propagate = true)
	{
		derived().apply(
		    node, [this, dec](OccupancyNode& node) { decreaseOccupancyLogit(node, dec); },
		    propagate);
	}

	void decreaseOccupancyLogit(Code code, logit_t dec, bool propagate = true)
	{
		derived().apply(
		    code, [this, dec](OccupancyNode& node) { decreaseOccupancyLogit(node, dec); },
		    propagate);
	}

	void decreaseOccupancyLogit(Key key, logit_t dec, bool propagate = true)
	{
		decreaseOccupancyLogit(derived().toCode(key), dec, propagate);
	}

	void decreaseOccupancyLogit(Point3 coord, logit_t dec, bool propagate = true,
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

	void increaseOccupancy(Node& node, occupancy_t inc, bool propagate = true)
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

	void increaseOccupancy(Point3 coord, occupancy_t inc, bool propagate = true,
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

	void decreaseOccupancy(Node& node, occupancy_t dec, bool propagate = true)
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

	void decreaseOccupancy(Point3 coord, occupancy_t dec, bool propagate = true,
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
	// Derived
	//

	constexpr Derived& derived() { return *static_cast<Derived*>(this); }

	constexpr Derived const& derived() const { return *static_cast<Derived const*>(this); }

	//
	// Initilize root
	//

	void initRoot()
	{
		setOccupancy(derived().getRoot(), 0.5);
		updateNodeIndicators(derived().getRoot());
	}

	//
	// Get occupancy
	//

	constexpr occupancy_t getOccupancy(OccupancyNode node) const noexcept
	{
		return toOccupancyProbability(getOccupancyLogit(node));
	}

	static constexpr logit_t getOccupancyLogit(OccupancyNode node) noexcept
	{
		return node.occupancy;
	}

	//
	// Set occupancy
	//

	constexpr void setOccupancy(OccupancyNode& node, occupancy_t new_occupancy)
	{
		setOccupancyLogit(node, toOccupancyLogit(new_occupancy));
	}

	constexpr void setOccupancyLogit(OccupancyNode& node, logit_t new_occupancy)
	{
		if constexpr (std::is_same_v<logit_t, uint8_t>) {
			node.occupancy = new_occupancy;
		} else {
			node.occupancy = std::clamp(new_occupancy, getOccupancyClampingThresMinLogit(),
			                            getOccupancyClampingThresMaxLogit());
		}
	}

	//
	// Checking state
	//

	constexpr bool isUnknown(OccupancyNode node) const noexcept
	{
		return !isFree(node) && !isOccupied(node);
	}

	constexpr bool isFree(OccupancyNode node) const noexcept
	{
		return getOccupancyLogit(node) < getFreeThresLogit();
	}

	constexpr bool isOccupied(OccupancyNode node) const noexcept
	{
		return getOccupancyLogit(node) > getOccupiedThresLogit();
	}

	constexpr bool containsUnknown(OccupancyNode node) const noexcept
	{
		return node.contains_unknown;
	}

	constexpr bool containsFree(OccupancyNode node) const noexcept { node.contains_free; }

	constexpr bool containsOccupied(OccupancyNode node) const noexcept
	{
		node.contains_occupied;
	}

	//
	// Increase occupancy
	//

	constexpr void increaseOccupancyLogit(OccupancyNode& node, logit_t inc)
	{
		if constexpr (std::is_same_v<logit_t, uint8_t>) {
			node.occupancy = math::increaseLogit(static_cast<uint8_t>(node.occupancy), inc);
		} else {
			node.occupancy =
			    std::clamp(node.occupancy + inc, getOccupancyClampingThresMinLogit(),
			               getOccupancyClampingThresMaxLogit());
		}
	}

	//
	// Decrease occupancy
	//

	constexpr void decreaseOccupancyLogit(OccupancyNode& node, logit_t dec)
	{
		if constexpr (std::is_same_v<logit_t, uint8_t>) {
			node.occupancy = math::decreaseLogit(static_cast<uint8_t>(node.occupancy), dec);
		} else {
			node.occupancy =
			    std::clamp(node.occupancy + dec, getOccupancyClampingThresMinLogit(),
			               getOccupancyClampingThresMaxLogit());
		}
	}

	//
	// Update node
	//

	void updateNode(OccupancyNode& node)
	{
		node.contains_unknown = isUnknown(node);
		node.contains_free = isFree(node);
		node.contains_occupied = isOccupied(node);
	}

	template <class T>
	void updateNode(OccupancyNode& node, T const& children)
	{
		switch (prop_criteria_) {
			case PropagationCriteria::MIN:
				setOccupancyLogit(node, minChildOccupancyLogit(children));
				break;
			case PropagationCriteria::MAX:
				setOccupancyLogit(node, maxChildOccupancyLogit(children));
				break;
			case PropagationCriteria::MEAN:
				setOccupancyLogit(node, averageChildOccupancyLogit(children));
				break;
		}

		node.contains_unknown =
		    any_of(children, [this](auto const& child) { return containsUnknown(child); });
		node.contains_free =
		    any_of(children, [this](auto const& child) { return containsFree(child); });
		node.contains_occupied =
		    any_of(children, [this](auto const& child) { return containsOccupied(child); });
	}

	//
	// Min child occupancy logit
	//

	template <class T>
	constexpr logit_t minChildOccupancyLogit(T const& children) const
	{
		logit_t min = std::numeric_limits<logit_t>::max();
		for (auto const& child : children) {
			min = std::min(min, getOccupancyLogit(child));
		}
		return min;
	}

	//
	// Max child occupancy logit
	//

	template <class T>
	constexpr logit_t maxChildOccupancyLogit(T const& children) const
	{
		logit_t max = std::numeric_limits<logit_t>::lowest();
		for (auto const& child : children) {
			max = std::max(max, getOccupancyLogit(child));
		}
		return max;
	}

	//
	// Average child occupancy logit
	//

	template <class T>
	constexpr logit_t averageChildOccupancyLogit(T const& children) const
	{
		if constexpr (std::is_same_v<logit_t, uint8_t>) {
			unsigned int sum = std::accumulate(
			    std::cbegin(children), std::cend(children), 0u,
			    [](auto cur, auto const& child) { return cur + getOccupancyLogit(child); });
			return sum / children.size();
		} else {
			logit_t sum = std::accumulate(
			    std::cbegin(children), std::cend(children), logit_t(0),
			    [](auto cur, auto const& child) { return cur + getOccupancyLogit(child); });

			return sum / children.size();
		}
	}

	//
	// Input/output (read/write)
	//

	static constexpr DataIdentifier getDataIdentifier() noexcept
	{
		return DataIdentifier::OCCUPANCY;
	}

	static constexpr bool canReadData(DataIdentifier identifier) noexcept
	{
		return getDataIdentifier() == identifier;
	}

	template <class InputIt>
	void readNodes(std::istream& in, InputIt first, InputIt last)
	{
		auto const num_nodes = std::distance(first, last);

		auto data = std::make_unique<logit_t[]>(num_nodes);
		in.read(reinterpret_cast<char*>(data.get()),
		        num_nodes * sizeof(typename decltype(data)::element_type));

		for (std::size_t i = 0; num_nodes != i; ++i, std::advance(first, 1)) {
			setOccupancyLogit(*first, data[i]);
		}
	}

	template <class InputIt>
	void writeNodes(std::ostream& out, InputIt first, InputIt last)
	{
		auto const num_nodes = std::distance(first, last);

		auto data = std::make_unique<logit_t[]>(num_nodes);
		for (std::size_t i = 0; num_nodes != i; ++i, std::advance(first, 1)) {
			data[i] = getOccupancyLogit(*first);
		}

		out.write(reinterpret_cast<char const*>(data.get()),
		          num_nodes * sizeof(typename decltype(data)::element_type));
	}

	// void readNodes(std::istream& in_stream, std::vector<LeafNode*> const& nodes)
	// {
	// 	uint8_t type;
	// 	in_stream.read(reinterpret_cast<char*>(&type), sizeof(type));

	// 	// Get min/max threshold
	// 	decltype(clamping_thres_min_logit_) min_logit;
	// 	decltype(clamping_thres_max_logit_) max_logit;
	// 	in_stream.read(reinterpret_cast<char*>(&min_logit), sizeof(min_logit));
	// 	in_stream.read(reinterpret_cast<char*>(&max_logit), sizeof(max_logit));

	// 	auto const num_nodes = nodes.size();

	// 	if (0 == type) {
	// 		// uint8_t

	// 		auto data = std::make_unique<uint8_t[]>(num_nodes);
	// 		in_stream.read(reinterpret_cast<char*>(data.get()), num_nodes * sizeof(uint8_t));

	// 		if constexpr (std::is_same_v<logit_t, uint8_t>) {
	// 			if (getOccupancyClampingThresMinLogit() == min_logit &&
	// 			    getOccupancyClampingThresMaxLogit() == max_logit) {
	// 				for (size_t i = 0; num_nodes != i; ++i) {
	// 					setOccupancyLogit(*nodes[i], data[i]);
	// 				}
	// 			} else {
	// 				for (size_t i = 0; num_nodes != i; ++i) {
	// 					setOccupancyLogit(*nodes[i],
	// 					                  math::convertLogit<uint8_t>(
	// 					                      math::convertLogit(data[i], min_logit, max_logit),
	// 					                      getOccupancyClampingThresMinLogit(),
	// 					                      getOccupancyClampingThresMaxLogit()));
	// 				}
	// 			}
	// 		} else {
	// 			for (size_t i = 0; num_nodes != i; ++i) {
	// 				setOccupancyLogit(*nodes[i],
	// 				                  std::clamp(math::convertLogit(data[i], min_logit,
	// max_logit), 				                             getOccupancyClampingThresMinLogit(),
	// 				                             getOccupancyClampingThresMaxLogit()));
	// 			}
	// 		}

	// 	} else {
	// 		// float

	// 		auto data = std::make_unique<float[]>(num_nodes);
	// 		in_stream.read(reinterpret_cast<char*>(data.get()), num_nodes * sizeof(float));

	// 		for (size_t i = 0; num_nodes != i; ++i) {
	// 			if constexpr (std::is_same_v<logit_t, uint8_t>) {
	// 				setOccupancyLogit(*nodes[i],
	// 				                  math::convertLogit<uint8_t>(
	// 				                      std::clamp(data[i], getOccupancyClampingThresMinLogit(),
	// 				                                 getOccupancyClampingThresMaxLogit()),
	// 				                      getOccupancyClampingThresMinLogit(),
	// 				                      getOccupancyClampingThresMaxLogit()));
	// 			} else {
	// 				setOccupancyLogit(*nodes[i],
	// 				                  std::clamp(data[i], getOccupancyClampingThresMinLogit(),
	// 				                             getOccupancyClampingThresMaxLogit()));
	// 			}
	// 		}
	// 	}
	// }

	// void writeNodes(std::ostream& out_stream, std::vector<LeafNode> const& nodes) const
	// {
	// 	constexpr uint8_t type = std::is_same_v<logit_t, uint8_t> ? 0 : 1;

	// 	out_stream.write(reinterpret_cast<char const*>(&type), sizeof(type));
	// 	out_stream.write(reinterpret_cast<char const*>(&clamping_thres_min_logit_),
	// 	                 sizeof(clamping_thres_min_logit_));
	// 	out_stream.write(reinterpret_cast<char const*>(&clamping_thres_max_logit_),
	// 	                 sizeof(clamping_thres_max_logit_));

	// 	auto const num_nodes = nodes.size();
	// 	auto data = std::make_unique<logit_t[]>(num_nodes);
	// 	for (size_t i = 0; num_nodes != i; ++i) {
	// 		data[i] = getOccupancyLogit(nodes[i]);
	// 	}

	// 	out_stream.write(reinterpret_cast<char const*>(data.get()),
	// 	                 num_nodes * sizeof(logit_t));
	// }

 protected:
	occupancy_t clamping_thres_min_logit_ = math::logit(0.1192);  // Min logit value
	occupancy_t clamping_thres_max_logit_ = math::logit(0.971);   // Max logit value

	logit_t occupied_thres_logit_ = toOccupancyLogit(0.5);  // Threshold for occupied
	logit_t free_thres_logit_ = toOccupancyLogit(0.5);      // Threshold for free

	// Propagation criteria
	PropagationCriteria prop_criteria_ = PropagationCriteria::MAX;
};

std::false_type is_occupancy_map_base_impl(...);

template <class Derived, class LeafNode>
std::true_type is_occupancy_map_base_impl(
    OccupancyMapBase<Derived, LeafNode> const volatile&);

template <typename T>
using is_occupancy_map_base = decltype(is_occupancy_map_base_impl(std::declval<T&>()));

// Helper variable template
template <class T>
inline constexpr bool is_occupancy_map_base_v = is_occupancy_map_base<T>::value;
}  // namespace ufo::map

#endif  // UFO_MAP_OCCUPANCY_MAP_BASE_H