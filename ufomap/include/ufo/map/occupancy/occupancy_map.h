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
#include <ufo/util/type_traits.h>

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
	// Get occupancy logit
	//

	[[nodiscard]] logit_t occupancyLogit(Node node) const
	{
		return derived().leafNode(node).occupancy[node.index()];
	}

	[[nodiscard]] logit_t occupancyLogit(Code code) const
	{
		auto [node, depth] = derived().leafNodeAndDepth(code);
		return node.occupancy[code.index(depth)];
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
	// Get occupancy
	//

	[[nodiscard]] occupancy_t occupancy(Node node) const
	{
		return toOccupancyProbability(occupancyLogit(node));
	}

	[[nodiscard]] occupancy_t occupancy(Code code) const
	{
		return toOccupancyProbability(occupancyLogit(code));
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
	// Set occupancy logit
	//

	void setOccupancyLogit(Node node, logit_t logit, bool propagate = true)
	{
		derived().apply(
		    node, [logit](auto& node, index_t index) { node.occupancy[index] = logit; },
		    [logit](auto& node) { node.occupancy.fill(logit); }, propagate);
	}

	void setOccupancyLogit(Code code, logit_t logit, bool propagate = true)
	{
		derived().apply(
		    code, [logit](auto& node, index_t index) { node.occupancy[index] = logit; },
		    [logit](auto& node) { node.occupancy.fill(logit); }, propagate);
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
	// Set occupancy
	//

	void setOccupancy(Node node, occupancy_t occupancy, bool propagate = true)
	{
		setOccupancyLogit(node, toOccupancyLogit(occupancy), propagate);
	}

	void setOccupancy(Code code, occupancy_t occupancy, bool propagate = true)
	{
		setOccupancyLogit(code, toOccupancyLogit(occupancy), propagate);
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
	// Increase occupancy logit
	//

	void increaseOccupancyLogit(Node node, logit_t inc, bool propagate = true)
	{
		derived().apply(
		    node,
		    [inc](auto& node, index_t index) {
			    node.occupancy[index] =
			        std::numeric_limits<logit_t>::max() - inc > node.occupancy[index]
			            ? node.occupancy[index] + inc
			            : std::numeric_limits<logit_t>::max();
		    },
		    [inc](auto& node) {
			    for (auto& occ : node.occupancy) {
				    occ = std::numeric_limits<logit_t>::max() - inc > occ
				              ? occ + inc
				              : std::numeric_limits<logit_t>::max();
			    }
		    },
		    propagate);
	}

	void increaseOccupancyLogit(Code code, logit_t inc, bool propagate = true)
	{
		derived().apply(
		    code,
		    [inc](auto& node, index_t index) {
			    node.occupancy[index] =
			        std::numeric_limits<logit_t>::max() - inc > node.occupancy[index]
			            ? node.occupancy[index] + inc
			            : std::numeric_limits<logit_t>::max();
		    },
		    [inc](auto& node) {
			    for (auto& occ : node.occupancy) {
				    occ = std::numeric_limits<logit_t>::max() - inc > occ
				              ? occ + inc
				              : std::numeric_limits<logit_t>::max();
			    }
		    },
		    propagate);
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
		    [dec](auto& node, index_t index) {
			    node.occupancy[index] =
			        std::numeric_limits<logit_t>::lowest() + dec < node.occupancy[index]
			            ? node.occupancy[index] - dec
			            : std::numeric_limits<logit_t>::lowest();
		    },
		    [dec](auto& node) {
			    for (auto& occ : node.occupancy) {
				    occ = std::numeric_limits<logit_t>::lowest() + dec < occ
				              ? occ - dec
				              : std::numeric_limits<logit_t>::lowest();
			    }
		    },
		    propagate);
	}

	void decreaseOccupancyLogit(Code code, logit_t dec, bool propagate = true)
	{
		derived().apply(
		    code,
		    [dec](auto& node, index_t index) {
			    node.occupancy[index] =
			        std::numeric_limits<logit_t>::lowest() + dec < node.occupancy[index]
			            ? node.occupancy[index] - dec
			            : std::numeric_limits<logit_t>::lowest();
		    },
		    [dec](auto& node) {
			    for (auto& occ : node.occupancy) {
				    occ = std::numeric_limits<logit_t>::lowest() + dec < occ
				              ? occ - dec
				              : std::numeric_limits<logit_t>::lowest();
			    }
		    },
		    propagate);
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
	// Update occupancy
	//

	void updateOccupancy(Node node, occupancy_t update, bool propagate = true)
	{
		if (0.5 < update) {
			increaseOccupancyLogit(node, toOccupancyChangeLogit(update), propagate);
		} else if (0.5 > update) {
			decreaseOccupancyLogit(node, toOccupancyChangeLogit(update), propagate);
		}
	}

	void updateOccupancy(Code code, occupancy_t update, bool propagate = true)
	{
		if (0.5 < update) {
			increaseOccupancyLogit(code, toOccupancyChangeLogit(update), propagate);
		} else if (0.5 > update) {
			decreaseOccupancyLogit(code, toOccupancyChangeLogit(update), propagate);
		}
	}

	void updateOccupancy(Key key, occupancy_t update, bool propagate = true)
	{
		updateOccupancy(derived().toCode(key), update, propagate);
	}

	void updateOccupancy(Point coord, occupancy_t update, bool propagate = true,
	                     depth_t depth = 0)
	{
		updateOccupancy(derived().toCode(coord, depth), update, propagate);
	}

	void updateOccupancy(coord_t x, coord_t y, coord_t z, occupancy_t update,
	                     bool propagate = true, depth_t depth = 0)
	{
		updateOccupancy(derived().toCode(x, y, z, depth), update, propagate);
	}

	//
	// Get occupancy state
	//

	[[nodiscard]] OccupancyState occupancyState(Node node) const
	{
		auto occ = occupancyLogit(node);
		if (isUnknown(occ)) {
			return OccupancyState::UNKNOWN;
		} else if (isFree(occ)) {
			return OccupancyState::FREE;
		} else {
			return OccupancyState::OCCUPIED;
		}
	}

	[[nodiscard]] OccupancyState occupancyState(Code code) const
	{
		auto occ = occupancyLogit(code);
		if (isUnknown(occ)) {
			return OccupancyState::UNKNOWN;
		} else if (isFree(occ)) {
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
		return isUnknown(occupancyLogit(node));
	}

	[[nodiscard]] bool isUnknown(Code code) const
	{
		return isUnknown(occupancyLogit(code));
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

	[[nodiscard]] bool isFree(Node node) const { return isFree(occupancyLogit(node)); }

	[[nodiscard]] bool isFree(Code code) const { return isFree(occupancyLogit(code)); }

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
		return isOccupied(occupancyLogit(node));
	}

	[[nodiscard]] bool isOccupied(Code code) const
	{
		return isOccupied(occupancyLogit(code));
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
		return 0 == node.depth() ? isUnknown(node)
		                         : containsUnknown(node.index(), derived().innerNode(node));
	}

	[[nodiscard]] bool containsUnknown(Code code) const
	{
		if (0 == code.depth()) {
			return isUnknown(code);
		}

		auto [node, depth] = derived().innerNodeAndDepth(code);
		return containsUnknown(code.index(depth), node);
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
		return 0 == node.depth() ? isFree(node)
		                         : containsFree(node.index(), derived().innerNode(node));
	}

	[[nodiscard]] bool containsFree(Code code) const
	{
		if (0 == code.depth()) {
			return isFree(code);
		}

		auto [node, depth] = derived().innerNodeAndDepth(code);
		return containsFree(code.index(depth), node);
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
		return 0 == node.depth() ? isOccupied(node)
		                         : containsOccupied(node.index(), derived().innerNode(node));
	}

	[[nodiscard]] bool containsOccupied(Code code) const
	{
		if (0 == code.depth()) {
			return isFree(code);
		}

		auto [node, depth] = derived().innerNodeAndDepth(code);
		return containsFree(code.index(depth), node);
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
		return math::logit<logit_t>(static_cast<double>(probability),
		                            occupancyClampingThresMinLogit(),
		                            occupancyClampingThresMaxLogit());
	}

	template <class logit_t>
	[[nodiscard]] constexpr occupancy_t toOccupancyProbability(logit_t logit) const
	{
		if constexpr (std::is_floating_point_v<logit_t>) {
			return math::probability(logit);
		} else {
			return math::probability(logit, occupancyClampingThresMinLogit(),
			                         occupancyClampingThresMaxLogit());
		}
	}

	template <class occupancy_t>
	[[nodiscard]] constexpr logit_t toOccupancyChangeLogit(occupancy_t probability) const
	{
		if constexpr (std::is_floating_point_v<logit_t>) {
			return math::logit(probability);
		} else {
			return math::logitChangeValue<logit_t>(static_cast<double>(probability),
			                                       occupancyClampingThresMinLogit(),
			                                       occupancyClampingThresMaxLogit());
		}
	}

	//
	// Set sensor model
	//

	void setOccupancyThres(occupancy_t occupied_thres, occupancy_t free_thres,
	                       bool propagate = true)
	{
		// FIXME: Should add a warning that these are very computational expensive to
		// call since the whole tree has to be updated

		setOccupancyThresLogit(toOccupancyLogit(occupied_thres), toOccupancyLogit(free_thres),
		                       propagate);
	}

	// FIXME: Look at
	void setOccupancyThresLogit(logit_t occupied_thres, logit_t free_thres,
	                            bool propagate = true)
	{
		// FIXME: Should add a warning that these are very computational expensive to
		// call since the whole tree has to be updated

		occupied_thres_logit_ = occupied_thres;
		free_thres_logit_ = free_thres;

		derived().setModified(1);

		if (propagate) {
			derived().updateModifiedNodes();
		}
	}

	void setOccupancyClampingThres(occupancy_t min_thres_probability,
	                               occupancy_t max_thres_probability)
	{
		setOccupancyClampingThresMin(min_thres_probability);
		setOccupancyClampingThresMax(max_thres_probability);
	}

	void setOccupancyClampingThresMin(occupancy_t probability)
	{
		clamping_thres_min_logit_ = math::logit(probability);
		// FIXME: Should this update occupancy values?
	}

	void setOccupancyClampingThresMax(occupancy_t probability)
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

 protected:
	//
	// Constructors
	//

	OccupancyMapBase() = default;

	OccupancyMapBase(OccupancyMapBase const& other) = default;

	OccupancyMapBase(OccupancyMapBase&& other) = default;

	template <class Derived2>
	OccupancyMapBase(OccupancyMapBase<Derived2> const& other)
	    : clamping_thres_min_logit_(other.occupancyClampingThresMinLogit()),
	      clamping_thres_max_logit_(other.occupancyClampingThresMaxLogit()),
	      occupied_thres_logit_(other.occupiedThresLogit()),
	      free_thres_logit_(other.freeThresLogit())
	{
	}

	//
	// Assignment operator
	//

	OccupancyMapBase& operator=(OccupancyMapBase const& rhs) = default;

	OccupancyMapBase& operator=(OccupancyMapBase&& rhs) = default;

	template <class Derived2>
	OccupancyMapBase& operator=(OccupancyMapBase<Derived2> const& rhs)
	{
		clamping_thres_min_logit_ = rhs.occupancyClampingThresMinLogit();
		clamping_thres_max_logit_ = rhs.occupancyClampingThresMaxLogit();
		occupied_thres_logit_ = rhs.occupiedThresLogit();
		free_thres_logit_ = rhs.freeThresLogit();
		return *this;
	}

	//
	// Swap
	//

	void swap(OccupancyMapBase& other) noexcept
	{
		std::swap(clamping_thres_min_logit_, other.clamping_thres_min_logit_);
		std::swap(clamping_thres_max_logit_, other.clamping_thres_max_logit_);
		std::swap(occupied_thres_logit_, other.occupied_thres_logit_);
		std::swap(free_thres_logit_, other.free_thres_logit_);
		std::swap(prop_criteria_, other.prop_criteria_);
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
		derived().root().occupancy[derived().rootIndex()] = toOccupancyLogit(0.5);
		// auto const occ = toOccupancyLogit(0.5);
		// auto const index = derived().rootIndex();
		// derived().root().occupancy[index] = occ;
		// derived().root().contains_unknown.set(index, isUnknown(occ));
		// derived().root().contains_free.set(index, isFree(occ));
		// derived().root().contains_occupied.set(index, isOccupied(occ));
	}

	//
	// Update node
	//

	template <class T, class InputIt>
	void updateNode(T& node, IndexField const indices, InputIt first, InputIt last)
	{
		auto prop = occupancyPropagationCriteria();
		if (indices.all()) {
			for (index_t i = 0; first != last; ++first, ++i) {
				switch (prop) {
					case PropagationCriteria::MIN:
						node.occupancy[i] = min(first->occupancy);
						break;
					case PropagationCriteria::MAX:
						node.occupancy[i] = max(first->occupancy);
						break;
					case PropagationCriteria::MEAN:
						node.occupancy[i] = mean(first->occupancy);
						break;
				}

				node.contains_unknown.set(i, containsUnknown(*first));
				node.contains_free.set(i, containsFree(*first));
				node.contains_occupied.set(i, containsOccupied(*first));
			}
		} else {
			for (index_t i = 0; first != last; ++first, ++i) {
				if (!indices[i]) {
					continue;
				}

				switch (prop) {
					case PropagationCriteria::MIN:
						node.occupancy[i] = min(first->occupancy);
						break;
					case PropagationCriteria::MAX:
						node.occupancy[i] = max(first->occupancy);
						break;
					case PropagationCriteria::MEAN:
						node.occupancy[i] = mean(first->occupancy);
						break;
				}

				node.contains_unknown.set(i, containsUnknown(*first));
				node.contains_free.set(i, containsFree(*first));
				node.contains_occupied.set(i, containsOccupied(*first));
			}
		}
	}

	//
	// Is
	//

	[[nodiscard]] constexpr bool isUnknown(logit_t const logit) const noexcept
	{
		return !isFree(logit) && !isOccupied(logit);
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
	[[nodiscard]] bool containsUnknown(T const& node) const
	{
		if constexpr (std::is_base_of_v<ContainsOccupancy<T::occupancySize()>, T>) {
			if (node.leaf.all()) {
				return any_of(node.occupancy, [this](auto occ) { return isUnknown(occ); });
			} else if ((node.contains_unknown & ~node.leaf).any()) {
				return true;
			}

			for (index_t i = 0; T::occupancySize() != i; ++i) {
				if (node.leaf[i] && isUnknown(node.occupancy[i])) {
					return true;
				}
			}
			return false;
		} else {
			return any_of(node.occupancy, [this](auto occ) { return isUnknown(occ); });
		}
	}

	template <class T>
	[[nodiscard]] bool containsUnknown(index_t const index, T const& node) const
	{
		if constexpr (std::is_base_of_v<ContainsOccupancy<T::occupancySize()>, T>) {
			if (!node.leaf[index]) {
				return node.contains_unknown[index];
			}
		}
		return isUnknown(node.occupancy[index]);
	}

	template <class T>
	[[nodiscard]] bool containsFree(T const& node) const
	{
		if constexpr (std::is_base_of_v<ContainsOccupancy<T::occupancySize()>, T>) {
			if (node.leaf.all()) {
				return any_of(node.occupancy, [this](auto occ) { return isFree(occ); });
			} else if ((node.contains_free & ~node.leaf).any()) {
				return true;
			}

			for (index_t i = 0; T::occupancySize() != i; ++i) {
				if (node.leaf[i] && isFree(node.occupancy[i])) {
					return true;
				}
			}
			return false;
		} else {
			return any_of(node.occupancy, [this](auto occ) { return isFree(occ); });
		}
	}

	template <class T>
	[[nodiscard]] bool containsFree(index_t const index, T const& node) const
	{
		if constexpr (std::is_base_of_v<ContainsOccupancy<T::occupancySize()>, T>) {
			if (!node.leaf[index]) {
				return node.contains_free[index];
			}
		}
		return isFree(node.occupancy[index]);
	}

	template <class T>
	[[nodiscard]] bool containsOccupied(T const& node) const
	{
		if constexpr (std::is_base_of_v<ContainsOccupancy<T::occupancySize()>, T>) {
			if (node.leaf.all()) {
				return any_of(node.occupancy, [this](auto occ) { return isOccupied(occ); });
			} else if ((node.contains_occupied & ~node.leaf).any()) {
				return true;
			}

			for (index_t i = 0; T::occupancySize() != i; ++i) {
				if (node.leaf[i] && isOccupied(node.occupancy[i])) {
					return true;
				}
			}
			return false;
		} else {
			return any_of(node.occupancy, [this](auto occ) { return isOccupied(occ); });
		}
	}

	template <class T>
	[[nodiscard]] bool containsOccupied(index_t const index, T const& node) const
	{
		if constexpr (std::is_base_of_v<ContainsOccupancy<T::occupancySize()>, T>) {
			if (!node.leaf[index]) {
				return node.contains_occupied[index];
			}
		}
		return isOccupied(node.occupancy[index]);
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

	template <class InputIt>
	[[nodiscard]] static constexpr uint8_t numData() noexcept
	{
		using value_type = typename std::iterator_traits<InputIt>::value_type;
		using node_type = typename value_type::node_type;
		return node_type::occupancySize();
	}

	template <class InputIt>
	std::size_t serializedSize(InputIt first, InputIt last) const
	{
		return sizeof(clamping_thres_min_logit_) + sizeof(clamping_thres_max_logit_) +
		       std::iterator_traits<InputIt>::value_type::occupancySize() *
		           std::distance(first, last) * sizeof(logit_t);
	}

	template <typename T>
	static bool equal(T a, T b)
	{
		return (std::abs(a - b) <=
		        std::numeric_limits<T>::epsilon() * std::max(std::abs(a), std::abs(b)));
	}

	template <class OutputIt>
	void readNodes(std::istream& in, OutputIt first, OutputIt last)
	{
		double clamping_thres_min_logit;
		double clamping_thres_max_logit;
		in.read(reinterpret_cast<char*>(&clamping_thres_min_logit),
		        sizeof(clamping_thres_min_logit));
		in.read(reinterpret_cast<char*>(&clamping_thres_max_logit),
		        sizeof(clamping_thres_max_logit));

		constexpr uint8_t const N = numData<OutputIt>();
		auto const num_data = std::distance(first, last) * N;
		auto data = std::make_unique<logit_t[]>(num_data);
		in.read(reinterpret_cast<char*>(data.get()),
		        num_data * sizeof(typename decltype(data)::element_type));
		if (!equal(clamping_thres_min_logit, clamping_thres_min_logit_) ||
		    !equal(clamping_thres_max_logit, clamping_thres_max_logit_)) {
			for (auto d = data.get(); first != last; ++first, d += N) {
				if (first->index_field.all()) {
					std::copy(d, d + N, first->node.occupancy.data());
					// TODO: Implement
				} else {
					for (index_t i = 0; N != i; ++i) {
						if (first->index_field[i]) {
							first->node.occupancy[i] = *(d + i);
							// TODO: Implement
						}
					}
				}
			}
		} else {
			for (auto d = data.get(); first != last; ++first, d += N) {
				if (first->index_field.all()) {
					std::copy(d, d + N, first->node.occupancy.data());
				} else {
					for (index_t i = 0; N != i; ++i) {
						if (first->index_field[i]) {
							first->node.occupancy[i] = *(d + i);
						}
					}
				}
			}
		}
	}

	template <class OutputIt>
	void readNodes(ReadBuffer&, OutputIt, OutputIt)
	{
		// TODO: Implement
	}

	template <class InputIt>
	void writeNodes(std::ostream& out, InputIt first, InputIt last) const
	{
		constexpr uint8_t const N = 8;  // FIXME: numData<InputIt>();
		auto const num_data = std::distance(first, last) * N;
		auto data = std::make_unique<logit_t[]>(num_data);
		for (auto d = data.get(); first != last; ++first) {
			d = copy(first->occupancy, d);
		}
		out.write(reinterpret_cast<char const*>(&clamping_thres_min_logit_),
		          sizeof(clamping_thres_min_logit_));
		out.write(reinterpret_cast<char const*>(&clamping_thres_max_logit_),
		          sizeof(clamping_thres_max_logit_));
		out.write(reinterpret_cast<char const*>(data.get()),
		          num_data * sizeof(typename decltype(data)::element_type));
	}

	template <class InputIt>
	void writeNodes(WriteBuffer& out, InputIt first, InputIt last) const
	{
		out.reserve(out.size() + serializedSize(first, last));
		out.write(&clamping_thres_min_logit_, sizeof(clamping_thres_min_logit_));
		out.write(&clamping_thres_max_logit_, sizeof(clamping_thres_max_logit_));
		for (; first != last; ++first) {
			out.write(first->occupancy.data(), first->occupancy.size() * sizeof(logit_t));
		}
	}

 protected:
	double clamping_thres_min_logit_ = math::logit(0.1192);  // Min logit value
	double clamping_thres_max_logit_ = math::logit(0.971);   // Max logit value

	logit_t occupied_thres_logit_ = toOccupancyLogit(0.5);  // Threshold for occupied
	logit_t free_thres_logit_ = toOccupancyLogit(0.5);      // Threshold for free

	// TODO: Maybe add lookup tables?

	// Propagation criteria
	PropagationCriteria prop_criteria_ = PropagationCriteria::MAX;
};
}  // namespace ufo::map

#endif  // UFO_MAP_OCCUPANCY_MAP_H