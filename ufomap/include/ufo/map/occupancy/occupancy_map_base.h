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
#include <ufo/map/occupancy/occupancy_node.h>
#include <ufo/map/octree/node.h>
#include <ufo/map/octree/octree_base.h>
#include <ufo/map/point.h>
#include <ufo/map/predicate/occupancy.h>
#include <ufo/map/types.h>
#include <ufo/math/logit.h>

// STL
#include <cstdint>
#include <execution>
#include <numeric>
#include <sstream>
#include <type_traits>

namespace ufo::map
{
template <class Derived, class LeafNode, class InnerNode>
class OccupancyMapBase
{
 protected:
	// Support float and uint8_t Occupancy value right now.
	// (and uint32_t given that it is packed with Time)
	static_assert(std::is_base_of_v<OccupancyNode<float>, LeafNode> ||
	              std::is_base_of_v<OccupancyNode<uint8_t>, LeafNode> ||
	              std::is_base_of_v<OccupancyTimeNode, LeafNode>);

 public:
	using logit_t = std::conditional_t<std::is_base_of_v<OccupancyTimeNode, LeafNode>,
	                                   uint8_t, decltype(LeafNode::occupancy)>;

 public:
	//
	// Get sensor model
	//

	constexpr float getOccupancyClampingThresMin() const noexcept
	{
		return math::probability(getOccupancyClampingThresMinLogit());
	}

	constexpr float getOccupancyClampingThresMinLogit() const noexcept
	{
		return occupancy_clamping_thres_min_log_;
	}

	constexpr float getOccupancyClampingThresMax() const noexcept
	{
		return math::probability(getOccupancyClampingThresMaxLogit());
	}

	constexpr float getOccupancyClampingThresMaxLogit() const noexcept
	{
		return occupancy_clamping_thres_max_log_;
	}

	constexpr float getOccupiedThres() const noexcept
	{
		return toOccupancyProbability(getOccupiedThresLogit());
	}

	constexpr logit_t getOccupiedThresLogit() const noexcept { return occupied_thres_log_; }

	constexpr float getFreeThres() const noexcept
	{
		return toOccupancyProbability(getFreeThresLogit());
	}

	constexpr logit_t getFreeThresLogit() const noexcept { return free_thres_log_; }

	//
	// Probability <-> logit
	//

	constexpr logit_t toOccupancyLogit(float probability) const
	{
		return math::logit<logit_t>(probability, getOccupancyClampingThresMinLogit(),
		                            getOccupancyClampingThresMaxLogit());
	}

	constexpr float toOccupancyProbability(logit_t logit) const
	{
		if constexpr (std::is_same_v<logit_t, uint8_t>) {
			return math::probability(logit, getOccupancyClampingThresMinLogit(),
			                         getOccupancyClampingThresMaxLogit());
		} else {
			return math::probability(logit);
		}
	}

	//
	// Set sensor model
	//

	void setOccupiedFreeThres(float new_occupied_thres, float new_free_thres,
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

		occupied_thres_log_ = new_occupied_thres;
		free_thres_log_ = new_free_thres;

		derived().setModifiedChildren();

		if (propagate) {
			derived().updateModifiedNodes();
		}
	}

	constexpr void setOccupancyClampingThres(float min_thres_probability,
	                                         float max_thres_probability)
	{
		// FIXME: Should this be applied?
		setOccupancyClampingThresLogit(math::logit(min_thres_probability),
		                               math::logit(max_thres_probability));
	}

	constexpr void setOccupancyClampingThresLogit(logit_t min_thres, logit_t max_thres)
	{
		// FIXME: What happens if uint8_t?
		setOccupancyClampingThresMinLogit(min_thres);
		setOccupancyClampingThresMaxLogit(max_thres);
	}

	constexpr void setOccupancyClampingThresMin(float probability)
	{
		setOccupancyClampingThresMinLogit(math::logit(probability));
	}

	constexpr void setOccupancyClampingThresMinLogit(logit_t min_thres)
	{
		// FIXME: What happens if uint8_t?
		occupancy_clamping_thres_min_log_ = min_thres;
	}

	constexpr void setOccupancyClampingThresMax(float probability)
	{
		setOccupancyClampingThresMaxLogit(math::logit(probability));
	}

	constexpr void setOccupancyClampingThresMaxLogit(logit_t max_thres)
	{
		// FIXME: What happens if uint8_t?
		occupancy_clamping_thres_max_log_ = max_thres;
	}

	//
	// Propagation criteria
	//

	constexpr PropagationCriteria getOccupancyPropagationCriteria() const noexcept
	{
		return occupancy_prop_criteria_;
	}

	constexpr void setOccupancyPropagationCriteria(
	    PropagationCriteria occupancy_prop_criteria, bool propagate = true) noexcept
	{
		if (occupancy_prop_criteria_ == occupancy_prop_criteria) {
			return;
		}

		occupancy_prop_criteria_ = occupancy_prop_criteria;

		// Set all inner nodes to modified
		// FIXME: Possible to optimize this to only set the ones with children
		derived().setModified(1);

		if (propagate) {
			derived().updateModifiedNodes();
		}
	}

	//
	// Get bounding volume containing all known
	//

	geometry::AABB getKnownBBX() const
	{
		// TODO: ement
	}

	//
	// Get occupancy state
	//

	OccupancyState getOccupancyState(Node const& node) const
	{
		if (isUnknown(Derived::getLeafNode(node))) {
			return OccupancyState::UNKNOWN;
		} else if (isFree(Derived::getLeafNode(node))) {
			return OccupancyState::FREE;
		} else {
			return OccupancyState::OCCUPIED;
		}
	}

	OccupancyState getOccupancyState(Code code) const
	{
		LeafNode const& node = derived().getLeafNode(code);
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
		return getOccupancyState(Derived::toCode(key));
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

	bool containsOccupancyState(OccupancyState state, Node const& node) const
	{
		switch (state) {
			case OccupancyState::UNKNOWN:
				return containsUnknown(Derived::getLeafNode(node));
			case OccupancyState::FREE:
				return containsFree(Derived::getLeafNode(node));
			case OccupancyState::OCCUPIED:
				return containsOccupied(Derived::getLeafNode(node));
		}
	}

	bool containsOccupancyState(OccupancyState state, Code code) const
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

	bool containsOccupancyState(OccupancyState state, Key key) const
	{
		return containsOccupancyState(state, Derived::toCode(key));
	}

	bool containsOccupancyState(OccupancyState state, Point3 coord, depth_t depth = 0) const
	{
		return containsOccupancyState(state, derived().toCode(coord, depth));
	}

	bool containsOccupancyState(OccupancyState state, coord_t x, coord_t y, coord_t z,
	                            depth_t depth = 0) const
	{
		return containsOccupancyState(state, derived().toCode(x, y, z, depth));
	}

	//
	// Is unknown
	//

	bool isUnknown(Node const& node) const { return isUnknown(Derived::getLeafNode(node)); }

	bool isUnknown(Code code) const { return isUnknown(derived().getLeafNode(code)); }

	bool isUnknown(Key key) const { return isUnknown(Derived::toCode(key)); }

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

	bool isFree(Node const& node) const { return isFree(Derived::getLeafNode(node)); }

	bool isFree(Code code) const { return isFree(derived().getLeafNode(code)); }

	bool isFree(Key key) const { return isFree(Derived::toCode(key)); }

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

	bool isOccupied(Node const& node) const
	{
		return isOccupied(Derived::getLeafNode(node));
	}

	bool isOccupied(Code code) const { return isOccupied(derived().getLeafNode(code)); }

	bool isOccupied(Key key) const { return isOccupied(Derived::toCode(key)); }

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

	bool containsUnknown(Node const& node) const
	{
		if (derived().isLeaf(node)) {
			return isUnknown(node);
		}
		return containsUnknown(derived().getInnerNode(node));
	}

	bool containsUnknown(Code code) const
	{
		if (0 == code.depth()) {
			return isUnknown(code);
		}
		return containsUnknown(derived().getInnerNode(code));
	}

	bool containsUnknown(Key key) const { return containsUnknown(Derived::toCode(key)); }

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

	bool containsFree(Node const& node) const
	{
		if (derived().isLeaf(node)) {
			return isFree(node);
		}
		return containsFree(derived().getInnerNode(node));
	}

	bool containsFree(Code code) const
	{
		if (0 == code.depth()) {
			return isFree(code);
		}
		return containsFree(derived().getInnerNode(code));
	}

	bool containsFree(Key key) const { return containsFree(Derived::toCode(key)); }

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

	bool containsOccupied(Node const& node) const
	{
		if (derived().isLeaf(node)) {
			return isOccupied(node);
		}
		return containsOccupied(derived().getInnerNode(node));
	}

	bool containsOccupied(Code code) const
	{
		if (0 == code.depth()) {
			return isOccupied(code);
		}
		return containsOccupied(derived().getInnerNode(code));
	}

	bool containsOccupied(Key key) const { return containsOccupied(Derived::toCode(key)); }

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

	float getOccupancy(Node const& node) const
	{
		return getOccupancy(Derived::getLeafNode(node));
	}

	float getOccupancy(Code code) const
	{
		return getOccupancy(derived().getLeafNode(code));
	}

	float getOccupancy(Key key) const { return getOccupancy(Derived::toCode(key)); }

	float getOccupancy(Point3 coord, depth_t depth = 0) const
	{
		return getOccupancy(derived().toCode(coord, depth));
	}

	float getOccupancy(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return getOccupancy(derived().toCode(x, y, z, depth));
	}

	//
	// Get occupancy logit
	//

	logit_t getOccupancyLogit(Node const& node) const
	{
		return getOccupancyLogit(derived().leafNode(node));
	}

	logit_t getOccupancyLogit(Code code) const
	{
		return getOccupancyLogit(derived().leafNode(code));
	}

	logit_t getOccupancyLogit(Key key) const
	{
		return getOccupancyLogit(Derived::toCode(key));
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

	void setOccupancy(Node& node, float occupancy, bool propagate = true)
	{
		logit_t logit = toOccupancyLogit(occupancy);
		setOccupancyLogit(node, logit, propagate);
	}

	void setOccupancy(Code code, float occupancy, bool propagate = true)
	{
		logit_t logit = toOccupancyLogit(occupancy);
		setOccupancyLogit(code, logit, propagate);
	}

	void setOccupancy(Key key, float occupancy, bool propagate = true)
	{
		setOccupancy(Derived::toCode(key), occupancy, propagate);
	}

	void setOccupancy(Point3 coord, float occupancy, bool propagate = true,
	                  depth_t depth = 0)
	{
		setOccupancy(derived().toCode(coord, depth), occupancy, propagate);
	}

	void setOccupancy(coord_t x, coord_t y, coord_t z, float occupancy,
	                  bool propagate = true, depth_t depth = 0)
	{
		setOccupancy(derived().toCode(x, y, z, depth), occupancy, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setOccupancy(ExecutionPolicy policy, Node& node, float occupancy,
	                  bool propagate = true)
	{
		logit_t logit = toOccupancyLogit(occupancy);
		setOccupancyLogit(policy, node, logit, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setOccupancy(ExecutionPolicy policy, Code code, float occupancy,
	                  bool propagate = true)
	{
		logit_t logit = toOccupancyLogit(occupancy);
		setOccupancyLogit(policy, code, logit, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setOccupancy(ExecutionPolicy policy, Point3 coord, float occupancy,
	                  bool propagate = true, depth_t depth = 0)
	{
		setOccupancy(policy, derived().toCode(coord, depth), occupancy, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setOccupancy(ExecutionPolicy policy, coord_t x, coord_t y, coord_t z,
	                  float occupancy, bool propagate = true, depth_t depth = 0)
	{
		setOccupancy(policy, derived().toCode(x, y, z, depth), occupancy, propagate);
	}

	//
	// Set occupancy logit
	//

	void setOccupancyLogit(Node& node, logit_t logit, bool propagate = true)
	{
		derived().apply(
		    node, [this, logit](LeafNode& node) { setOccupancyLogit(node, logit); },
		    propagate);
	}

	void setOccupancyLogit(Code code, logit_t logit, bool propagate = true)
	{
		derived().apply(
		    code, [this, logit](LeafNode& node) { setOccupancyLogit(node, logit); },
		    propagate);
	}

	void setOccupancyLogit(Key key, logit_t logit, bool propagate = true)
	{
		setOccupancyLogit(Derived::toCode(key), logit, propagate);
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

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setOccupancyLogit(ExecutionPolicy policy, Node& node, logit_t logit,
	                       bool propagate = true)
	{
		derived().apply(
		    policy, node, [this, logit](LeafNode& node) { setOccupancyLogit(node, logit); },
		    propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setOccupancyLogit(ExecutionPolicy policy, Code code, logit_t logit,
	                       bool propagate = true)
	{
		derived().apply(
		    policy, code, [this, logit](LeafNode& node) { setOccupancyLogit(node, logit); },
		    propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setOccupancyLogit(ExecutionPolicy policy, Key key, logit_t logit,
	                       bool propagate = true)
	{
		setOccupancyLogit(policy, Derived::toCode(key), logit, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setOccupancyLogit(ExecutionPolicy policy, Point3 coord, logit_t logit,
	                       bool propagate = true, depth_t depth = 0)
	{
		setOccupancyLogit(policy, derived().toCode(coord, depth), logit, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setOccupancyLogit(ExecutionPolicy policy, coord_t x, coord_t y, coord_t z,
	                       logit_t logit, bool propagate = true, depth_t depth = 0)
	{
		setOccupancyLogit(policy, derived().toCode(x, y, z, depth), logit, propagate);
	}

	//
	// Increase occupancy
	//

	void increaseOccupancy(Node& node, float inc, bool propagate = true)
	{
		logit_t inc_logit;
		if constexpr (std::is_same_v<logit_t, uint8_t>) {
			inc_logit = math::logitChangeValue(inc, getOccupancyClampingThresMinLogit(),
			                                   getOccupancyClampingThresMaxLogit());
		} else {
			inc_logit = math::logit(inc, getOccupancyClampingThresMinLogit(),
			                        getOccupancyClampingThresMaxLogit());
		}

		increaseOccupancyLogit(node, inc_logit, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void increaseOccupancy(ExecutionPolicy policy, Node& node, float inc,
	                       bool propagate = true)
	{
		logit_t inc_logit;
		if constexpr (std::is_same_v<logit_t, uint8_t>) {
			inc_logit = math::logitChangeValue(inc, getOccupancyClampingThresMinLogit(),
			                                   getOccupancyClampingThresMaxLogit());
		} else {
			inc_logit = math::logit(inc, getOccupancyClampingThresMinLogit(),
			                        getOccupancyClampingThresMaxLogit());
		}

		increaseOccupancyLogit(policy, node, inc_logit, propagate);
	}

	void increaseOccupancyLogit(Node& node, logit_t inc, bool propagate = true)
	{
		derived().apply(
		    node, [this, inc](LeafNode& node) { increaseOccupancyLogit(node, inc); },
		    propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void increaseOccupancyLogit(ExecutionPolicy policy, Node& node, logit_t inc,
	                            bool propagate = true)
	{
		derived().apply(
		    policy, node, [this, inc](LeafNode& node) { increaseOccupancyLogit(node, inc); },
		    propagate);
	}

	void increaseOccupancy(Code code, float inc, bool propagate = true)
	{
		logit_t inc_logit;
		if constexpr (std::is_same_v<logit_t, uint8_t>) {
			inc_logit = math::logitChangeValue(inc, getOccupancyClampingThresMinLogit(),
			                                   getOccupancyClampingThresMaxLogit());
		} else {
			inc_logit = math::logit(inc, getOccupancyClampingThresMinLogit(),
			                        getOccupancyClampingThresMaxLogit());
		}

		increaseOccupancyLogit(code, inc_logit, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void increaseOccupancy(ExecutionPolicy policy, Code code, float inc,
	                       bool propagate = true)
	{
		logit_t inc_logit;
		if constexpr (std::is_same_v<logit_t, uint8_t>) {
			inc_logit = math::logitChangeValue(inc, getOccupancyClampingThresMinLogit(),
			                                   getOccupancyClampingThresMaxLogit());
		} else {
			inc_logit = math::logit(inc, getOccupancyClampingThresMinLogit(),
			                        getOccupancyClampingThresMaxLogit());
		}

		increaseOccupancyLogit(policy, code, inc_logit, propagate);
	}

	void increaseOccupancyLogit(Code code, logit_t inc, bool propagate = true)
	{
		derived().apply(
		    code, [this, inc](LeafNode& node) { increaseOccupancyLogit(node, inc); },
		    propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void increaseOccupancyLogit(ExecutionPolicy policy, Code code, logit_t inc,
	                            bool propagate = true)
	{
		derived().apply(
		    policy, code, [this, inc](LeafNode& node) { increaseOccupancyLogit(node, inc); },
		    propagate);
	}

	void increaseOccupancy(Key key, float inc, bool propagate = true)
	{
		increaseOccupancy(Derived::toCode(key), inc, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void increaseOccupancy(ExecutionPolicy policy, Key key, float inc,
	                       bool propagate = true)
	{
		increaseOccupancy(policy, Derived::toCode(key), inc, propagate);
	}

	void increaseOccupancyLogit(Key key, logit_t inc, bool propagate = true)
	{
		increaseOccupancyLogit(Derived::toCode(key), inc, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void increaseOccupancyLogit(ExecutionPolicy policy, Key key, logit_t inc,
	                            bool propagate = true)
	{
		increaseOccupancyLogit(policy, Derived::toCode(key), inc, propagate);
	}

	void increaseOccupancy(Point3 coord, float inc, bool propagate = true,
	                       depth_t depth = 0)
	{
		increaseOccupancy(derived().toCode(coord, depth), inc, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void increaseOccupancy(ExecutionPolicy policy, Point3 coord, float inc,
	                       bool propagate = true, depth_t depth = 0)
	{
		increaseOccupancy(policy, derived().toCode(coord, depth), inc, propagate);
	}

	void increaseOccupancyLogit(Point3 coord, logit_t inc, bool propagate = true,
	                            depth_t depth = 0)
	{
		increaseOccupancyLogit(derived().toCode(coord, depth), inc, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void increaseOccupancyLogit(ExecutionPolicy policy, Point3 coord, logit_t inc,
	                            bool propagate = true, depth_t depth = 0)
	{
		increaseOccupancyLogit(policy, derived().toCode(coord, depth), inc, propagate);
	}

	void increaseOccupancy(coord_t x, coord_t y, coord_t z, float inc,
	                       bool propagate = true, depth_t depth = 0)
	{
		increaseOccupancy(derived().toCode(x, y, z, depth), inc, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void increaseOccupancy(ExecutionPolicy policy, coord_t x, coord_t y, coord_t z,
	                       float inc, bool propagate = true, depth_t depth = 0)
	{
		increaseOccupancy(policy, derived().toCode(x, y, z, depth), inc, propagate);
	}

	void increaseOccupancyLogit(coord_t x, coord_t y, coord_t z, logit_t inc,
	                            bool propagate = true, depth_t depth = 0)
	{
		increaseOccupancyLogit(derived().toCode(x, y, z, depth), inc, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void increaseOccupancyLogit(ExecutionPolicy policy, coord_t x, coord_t y, coord_t z,
	                            logit_t inc, bool propagate = true, depth_t depth = 0)
	{
		increaseOccupancyLogit(policy, derived().toCode(x, y, z, depth), inc, propagate);
	}

	//
	// Decrease occupancy
	//

	void decreaseOccupancy(Node& node, float dec, bool propagate = true)
	{
		logit_t dec_logit;
		if constexpr (std::is_same_v<logit_t, uint8_t>) {
			dec_logit = math::logitChangeValue(dec, getOccupancyClampingThresMinLogit(),
			                                   getOccupancyClampingThresMaxLogit());
		} else {
			dec_logit = math::logit(dec, getOccupancyClampingThresMinLogit(),
			                        getOccupancyClampingThresMaxLogit());
		}

		decreaseOccupancyLogit(node, dec_logit, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void decreaseOccupancy(ExecutionPolicy policy, Node& node, float dec,
	                       bool propagate = true)
	{
		logit_t dec_logit;
		if constexpr (std::is_same_v<logit_t, uint8_t>) {
			dec_logit = math::logitChangeValue(dec, getOccupancyClampingThresMinLogit(),
			                                   getOccupancyClampingThresMaxLogit());
		} else {
			dec_logit = math::logit(dec, getOccupancyClampingThresMinLogit(),
			                        getOccupancyClampingThresMaxLogit());
		}

		decreaseOccupancyLogit(policy, node, dec_logit, propagate);
	}

	void decreaseOccupancyLogit(Node& node, logit_t dec, bool propagate = true)
	{
		derived().apply(
		    node, [this, dec](LeafNode& node) { decreaseOccupancyLogit(node, dec); },
		    propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void decreaseOccupancyLogit(ExecutionPolicy policy, Node& node, logit_t dec,
	                            bool propagate = true)
	{
		derived().apply(
		    policy, node, [this, dec](LeafNode& node) { decreaseOccupancyLogit(node, dec); },
		    propagate);
	}

	void decreaseOccupancy(Code code, float dec, bool propagate = true)
	{
		logit_t dec_logit;
		if constexpr (std::is_same_v<logit_t, uint8_t>) {
			dec_logit = math::logitChangeValue(dec, getOccupancyClampingThresMinLogit(),
			                                   getOccupancyClampingThresMaxLogit());
		} else {
			dec_logit = math::logit(dec, getOccupancyClampingThresMinLogit(),
			                        getOccupancyClampingThresMaxLogit());
		}

		decreaseOccupancyLogit(code, dec_logit, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void decreaseOccupancy(ExecutionPolicy policy, Code code, float dec,
	                       bool propagate = true)
	{
		logit_t dec_logit;
		if constexpr (std::is_same_v<logit_t, uint8_t>) {
			dec_logit = math::logitChangeValue(dec, getOccupancyClampingThresMinLogit(),
			                                   getOccupancyClampingThresMaxLogit());
		} else {
			dec_logit = math::logit(dec, getOccupancyClampingThresMinLogit(),
			                        getOccupancyClampingThresMaxLogit());
		}

		decreaseOccupancyLogit(policy, code, dec_logit, propagate);
	}

	void decreaseOccupancyLogit(Code code, logit_t dec, bool propagate = true)
	{
		derived().apply(
		    code, [this, dec](LeafNode& node) { decreaseOccupancyLogit(node, dec); },
		    propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void decreaseOccupancyLogit(ExecutionPolicy policy, Code code, logit_t dec,
	                            bool propagate = true)
	{
		derived().apply(
		    policy, code, [this, dec](LeafNode& node) { decreaseOccupancyLogit(node, dec); },
		    propagate);
	}

	void decreaseOccupancy(Key key, float dec, bool propagate = true)
	{
		decreaseOccupancy(Derived::toCode(key), dec, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void decreaseOccupancy(ExecutionPolicy policy, Key key, float dec,
	                       bool propagate = true)
	{
		decreaseOccupancy(policy, Derived::toCode(key), dec, propagate);
	}

	void decreaseOccupancyLogit(Key key, logit_t dec, bool propagate = true)
	{
		decreaseOccupancyLogit(Derived::toCode(key), dec, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void decreaseOccupancyLogit(ExecutionPolicy policy, Key key, logit_t dec,
	                            bool propagate = true)
	{
		decreaseOccupancyLogit(policy, Derived::toCode(key), dec, propagate);
	}

	void decreaseOccupancy(Point3 coord, float dec, bool propagate = true,
	                       depth_t depth = 0)
	{
		decreaseOccupancy(derived().toCode(coord, depth), dec, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void decreaseOccupancy(ExecutionPolicy policy, Point3 coord, float dec,
	                       bool propagate = true, depth_t depth = 0)
	{
		decreaseOccupancy(policy, derived().toCode(coord, depth), dec, propagate);
	}

	void decreaseOccupancyLogit(Point3 coord, logit_t dec, bool propagate = true,
	                            depth_t depth = 0)
	{
		decreaseOccupancyLogit(derived().toCode(coord, depth), dec, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void decreaseOccupancyLogit(ExecutionPolicy policy, Point3 coord, logit_t dec,
	                            bool propagate = true, depth_t depth = 0)
	{
		decreaseOccupancyLogit(policy, derived().toCode(coord, depth), dec, propagate);
	}

	void decreaseOccupancy(coord_t x, coord_t y, coord_t z, float dec,
	                       bool propagate = true, depth_t depth = 0)
	{
		decreaseOccupancy(derived().toCode(x, y, z, depth), dec, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void decreaseOccupancy(ExecutionPolicy policy, coord_t x, coord_t y, coord_t z,
	                       float dec, bool propagate = true, depth_t depth = 0)
	{
		decreaseOccupancy(policy, derived().toCode(x, y, z, depth), dec, propagate);
	}

	void decreaseOccupancyLogit(coord_t x, coord_t y, coord_t z, logit_t dec,
	                            bool propagate = true, depth_t depth = 0)
	{
		decreaseOccupancyLogit(derived().toCode(x, y, z, depth), dec, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void decreaseOccupancyLogit(ExecutionPolicy policy, coord_t x, coord_t y, coord_t z,
	                            logit_t dec, bool propagate = true, depth_t depth = 0)
	{
		decreaseOccupancyLogit(policy, derived().toCode(x, y, z, depth), dec, propagate);
	}

 protected:
	//
	// Constructors
	//

	OccupancyMapBase(float occupied_thres = 0.5, float free_thres = 0.5,
	                 float clamping_thres_min = 0.1192, float clamping_thres_max = 0.971)
	    : occupancy_clamping_thres_min_log_(math::logit(clamping_thres_min)),
	      occupancy_clamping_thres_max_log_(math::logit(clamping_thres_max))
	{
		occupied_thres_log_ = toOccupancyLogit(occupied_thres);
		free_thres_log_ = toOccupancyLogit(free_thres);
	}

	OccupancyMapBase(OccupancyMapBase const& other)
	    : occupancy_clamping_thres_min_log_(other.occupancy_clamping_thres_min_log_),
	      occupancy_clamping_thres_max_log_(other.occupancy_clamping_thres_max_log_),
	      occupied_thres_log_(other.occupied_thres_log_),
	      free_thres_log_(other.free_thres_log_),
	      occupancy_prop_criteria_(other.occupancy_prop_criteria_)
	{
	}

	template <class Derived2, class DataType2, class Indicators2>
	OccupancyMapBase(OccupancyMapBase<Derived2, DataType2, Indicators2> const& other)
	    : occupancy_clamping_thres_min_log_(other.occupancy_clamping_thres_min_log_),
	      occupancy_clamping_thres_max_log_(other.occupancy_clamping_thres_max_log_),
	      occupancy_prop_criteria_(other.occupancy_prop_criteria_)
	{
		occupied_thres_log_ = toOccupancyLogit(other.getOccupiedThres());
		free_thres_log_ = toOccupancyLogit(other.getFreeThres());
	}

	OccupancyMapBase(OccupancyMapBase&& other)
	    : occupancy_clamping_thres_min_log_(
	          std::move(other.occupancy_clamping_thres_min_log_)),
	      occupancy_clamping_thres_max_log_(
	          std::move(other.occupancy_clamping_thres_max_log_)),
	      occupied_thres_log_(std::move(other.occupied_thres_log_)),
	      free_thres_log_(std::move(other.free_thres_log_)),
	      occupancy_prop_criteria_(std::move(other.occupancy_prop_criteria_))
	{
	}

	OccupancyMapBase& operator=(OccupancyMapBase const& rhs)
	{
		occupancy_clamping_thres_min_log_ = rhs.occupancy_clamping_thres_min_log_;
		occupancy_clamping_thres_max_log_ = rhs.occupancy_clamping_thres_max_log_;
		occupied_thres_log_ = rhs.occupied_thres_log_;
		free_thres_log_ = rhs.free_thres_log_;
		occupancy_prop_criteria_ = rhs.occupancy_prop_criteria_;
		return *this;
	}

	template <class Derived2, class DataType2, class Indicators2>
	OccupancyMapBase& operator=(
	    OccupancyMapBase<Derived2, DataType2, Indicators2> const& rhs)
	{
		occupancy_clamping_thres_min_log_ = rhs.occupancy_clamping_thres_min_log_;
		occupancy_clamping_thres_max_log_ = rhs.occupancy_clamping_thres_max_log_;

		occupied_thres_log_ = toOccupancyLogit(rhs.getOccupiedThres());
		free_thres_log_ = toOccupancyLogit(rhs.getFreeThres());

		return *this;
	}

	OccupancyMapBase& operator=(OccupancyMapBase&& rhs)
	{
		occupancy_clamping_thres_min_log_ = std::move(rhs.occupancy_clamping_thres_min_log_);
		occupancy_clamping_thres_max_log_ = std::move(rhs.occupancy_clamping_thres_max_log_);
		occupied_thres_log_ = std::move(rhs.occupied_thres_log_);
		free_thres_log_ = std::move(rhs.free_thres_log_);
		return *this;
	}

	//
	// Destructor
	//

	~OccupancyMapBase() {}

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

	constexpr float getOccupancy(LeafNode const& node) const noexcept
	{
		return toOccupancyProbability(getOccupancyLogit(node));
	}

	static constexpr logit_t getOccupancyLogit(LeafNode const& node) noexcept
	{
		return node.occupancy;
	}

	//
	// Set occupancy
	//

	constexpr void setOccupancy(LeafNode& node, float new_occupancy)
	{
		setOccupancyLogit(node, toOccupancyLogit(new_occupancy));
	}

	constexpr void setOccupancyLogit(LeafNode& node, logit_t new_occupancy)
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

	constexpr bool isUnknown(LeafNode const& node) noexcept
	{
		return !isFree(node) && !isOccupied(node);
	}

	constexpr bool isFree(LeafNode const& node) noexcept
	{
		return getOccupancyLogit(node) < getFreeThresLogit();
	}

	constexpr bool isOccupied(LeafNode const& node) noexcept
	{
		return getOccupancyLogit(node) > getOccupiedThresLogit();
	}

	static constexpr bool containsUnknown(LeafNode const& node) noexcept
	{
		return node.contains_unknown;
	}

	static constexpr bool containsFree(LeafNode const& node) noexcept
	{
		return node.contains_free;
	}

	static constexpr bool containsOccupied(LeafNode const& node) noexcept
	{
		return node.contains_occupied;
	}

	//
	// Increase occupancy
	//

	constexpr void increaseOccupancyLogit(LeafNode& node, logit_t inc)
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

	constexpr void decreaseOccupancyLogit(LeafNode& node, logit_t dec)
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

	// NOTE: Only called when node has children
	void updateNode(InnerNode& node, depth_t depth)
	{
		switch (occupancy_prop_criteria_) {
			case PropagationCriteria::Max:
				setOccupancyLogit(node, maxChildOccupancyLogit(node, depth));
				break;
			case PropagationCriteria::Min:
				setOccupancyLogit(node, minChildOccupancyLogit(node, depth));
				break;
			case PropagationCriteria::Mean:
				setOccupancyLogit(node, averageChildOccupancyLogit(node, depth));
				break;
		}
	}

	//
	// Max child occupancy logit
	//

	static constexpr logit_t maxChildOccupancyLogit(InnerNode const& node, depth_t depth)
	{
		return 1 == depth ? std::max({getOccupancyLogit(Derived::getLeafChild(node, 0)),
		                              getOccupancyLogit(Derived::getLeafChild(node, 1)),
		                              getOccupancyLogit(Derived::getLeafChild(node, 2)),
		                              getOccupancyLogit(Derived::getLeafChild(node, 3)),
		                              getOccupancyLogit(Derived::getLeafChild(node, 4)),
		                              getOccupancyLogit(Derived::getLeafChild(node, 5)),
		                              getOccupancyLogit(Derived::getLeafChild(node, 6)),
		                              getOccupancyLogit(Derived::getLeafChild(node, 7))})
		                  : std::max({getOccupancyLogit(Derived::getInnerChild(node, 0)),
		                              getOccupancyLogit(Derived::getInnerChild(node, 1)),
		                              getOccupancyLogit(Derived::getInnerChild(node, 2)),
		                              getOccupancyLogit(Derived::getInnerChild(node, 3)),
		                              getOccupancyLogit(Derived::getInnerChild(node, 4)),
		                              getOccupancyLogit(Derived::getInnerChild(node, 5)),
		                              getOccupancyLogit(Derived::getInnerChild(node, 6)),
		                              getOccupancyLogit(Derived::getInnerChild(node, 7))});
	}

	//
	// Min child occupancy logit
	//

	static constexpr logit_t minChildOccupancyLogit(InnerNode const& node, depth_t depth)
	{
		return 1 == depth ? std::min({getOccupancyLogit(Derived::getLeafChild(node, 0)),
		                              getOccupancyLogit(Derived::getLeafChild(node, 1)),
		                              getOccupancyLogit(Derived::getLeafChild(node, 2)),
		                              getOccupancyLogit(Derived::getLeafChild(node, 3)),
		                              getOccupancyLogit(Derived::getLeafChild(node, 4)),
		                              getOccupancyLogit(Derived::getLeafChild(node, 5)),
		                              getOccupancyLogit(Derived::getLeafChild(node, 6)),
		                              getOccupancyLogit(Derived::getLeafChild(node, 7))})
		                  : std::min({getOccupancyLogit(Derived::getInnerChild(node, 0)),
		                              getOccupancyLogit(Derived::getInnerChild(node, 1)),
		                              getOccupancyLogit(Derived::getInnerChild(node, 2)),
		                              getOccupancyLogit(Derived::getInnerChild(node, 3)),
		                              getOccupancyLogit(Derived::getInnerChild(node, 4)),
		                              getOccupancyLogit(Derived::getInnerChild(node, 5)),
		                              getOccupancyLogit(Derived::getInnerChild(node, 6)),
		                              getOccupancyLogit(Derived::getInnerChild(node, 7))});
	}

	//
	// Average child occupancy logit
	//

	static constexpr logit_t averageChildOccupancyLogit(InnerNode const& node,
	                                                    depth_t depth)
	{
		if constexpr (std::is_same_v<logit_t, uint8_t>) {
			unsigned int sum =
			    1 == depth ? std::accumulate(std::begin(Derived::getLeafChildren(node)),
			                                 std::end(Derived::getLeafChildren(node)), 0u,
			                                 [](auto cur, auto&& child) {
				                                 return cur + getOccupancyLogit(child);
			                                 })
			               : std::accumulate(std::begin(Derived::getInnerChildren(node)),
			                                 std::end(Derived::getInnerChildren(node)), 0u,
			                                 [](auto cur, auto&& child) {
				                                 return cur + getOccupancyLogit(child);
			                                 });

			return sum / 8u;
		} else {
			float sum = 1 == depth
			                ? std::accumulate(std::begin(Derived::getLeafChildren(node)),
			                                  std::end(Derived::getLeafChildren(node)), 0.0f,
			                                  [](auto cur, auto&& child) {
				                                  return cur + getOccupancyLogit(child);
			                                  })
			                : std::accumulate(std::begin(Derived::getInnerChildren(node)),
			                                  std::end(Derived::getInnerChildren(node)), 0.0f,
			                                  [](auto cur, auto&& child) {
				                                  return cur + getOccupancyLogit(child);
			                                  });

			return sum / 8.0f;
		}
	}

	// NOTE: Only called when node has no children
	void updateNodeIndicators(LeafNode& node)
	{
		node.contains_unknown = isUnknown(node);
		node.contains_free = isFree(node);
		node.contains_occupied = isOccupied(node);
	}

	// NOTE: Only called when node has children
	void updateNodeIndicators(InnerNode& node, depth_t depth)
	{
		node.contains_unknown = false;
		node.contains_free = false;
		node.contains_occupied = false;

		if (1 == depth) {
			for (auto const& child : Derived::getLeafChildren(node)) {
				node.contains_unknown = node.contains_unknown || child.contains_unknown;
				node.contains_free = node.contains_free || child.contains_free;
				node.contains_occupied = node.contains_occupied || child.contains_occupied;
			}
		} else {
			for (auto const& child : Derived::getInnerChildren(node)) {
				node.contains_unknown = node.contains_unknown || child.contains_unknown;
				node.contains_free = node.contains_free || child.contains_free;
				node.contains_occupied = node.contains_occupied || child.contains_occupied;
			}
		}
	}

	//
	// Input/output (read/write)
	//

	void addFileInfo(FileInfo& info) const
	{
		info["fields"].emplace_back("occupancy");
		if constexpr (std::is_same_v<logit_t, uint8_t>) {
			info["type"].emplace_back("U");
			info["size"].emplace_back("1");
		} else {
			info["type"].emplace_back("F");
			info["size"].emplace_back("4");
		}
	}

	bool readNodes(std::istream& in_stream, std::vector<LeafNode*> const& nodes,
	               std::string const& field, char type, uint64_t size, uint64_t num)
	{
		if ("occupancy" != field) {
			return false;
		}

		if ('U' == type && 1 == size) {
			// Get min/max threshold
			float min_logit;
			float max_logit;
			in_stream.read(reinterpret_cast<char*>(&min_logit), sizeof(min_logit));
			in_stream.read(reinterpret_cast<char*>(&max_logit), sizeof(max_logit));

			auto data = std::make_unique<uint8_t[]>(nodes.size());
			in_stream.read(reinterpret_cast<char*>(data.get()), nodes.size() * sizeof(uint8_t));

			if constexpr (std::is_same_v<logit_t, uint8_t>) {
				if (getOccupancyClampingThresMinLogit() == min_logit &&
				    getOccupancyClampingThresMaxLogit() == max_logit) {
					for (size_t i = 0; i != nodes.size(); ++i) {
						setOccupancyLogit(*nodes[i], data[i]);
					}
				} else {
					for (size_t i = 0; i != nodes.size(); ++i) {
						setOccupancyLogit(*nodes[i],
						                  math::convertLogit<uint8_t>(
						                      math::convertLogit(data[i], min_logit, max_logit),
						                      getOccupancyClampingThresMinLogit(),
						                      getOccupancyClampingThresMaxLogit()));
					}
				}
			} else {
				for (size_t i = 0; i != nodes.size(); ++i) {
					setOccupancyLogit(*nodes[i],
					                  std::clamp(math::convertLogit(data[i], min_logit, max_logit),
					                             getOccupancyClampingThresMinLogit(),
					                             getOccupancyClampingThresMaxLogit()));
				}
			}
		} else if ('F' == type && 4 == size) {
			auto data = std::make_unique<float[]>(nodes.size());
			in_stream.read(reinterpret_cast<char*>(data.get()), nodes.size() * sizeof(float));

			for (size_t i = 0; i != nodes.size(); ++i) {
				if constexpr (std::is_same_v<logit_t, uint8_t>) {
					setOccupancyLogit(*nodes[i],
					                  math::convertLogit<uint8_t>(
					                      std::clamp(data[i], getOccupancyClampingThresMinLogit(),
					                                 getOccupancyClampingThresMaxLogit()),
					                      getOccupancyClampingThresMinLogit(),
					                      getOccupancyClampingThresMaxLogit()));
				} else {
					setOccupancyLogit(*nodes[i],
					                  std::clamp(data[i], getOccupancyClampingThresMinLogit(),
					                             getOccupancyClampingThresMaxLogit()));
				}
			}
		} else {
			return false;
		}

		return true;
	}

	void writeNodes(std::ostream& out_stream, std::vector<LeafNode const*> const& nodes,
	                bool compress, int compression_acceleration_level,
	                int compression_level) const
	{
		auto data = std::make_unique<logit_t[]>(nodes.size());
		for (size_t i = 0; i != nodes.size(); ++i) {
			data[i] = getOccupancyLogit(*nodes[i]);
		}

		if constexpr (std::is_same_v<logit_t, uint8_t>) {
			uint64_t const size = nodes.size() + sizeof(occupancy_clamping_thres_min_log_) +
			                      sizeof(occupancy_clamping_thres_max_log_);

			out_stream.write(reinterpret_cast<char const*>(&size), sizeof(size));

			out_stream.write(reinterpret_cast<char const*>(&occupancy_clamping_thres_min_log_),
			                 sizeof(occupancy_clamping_thres_min_log_));
			out_stream.write(reinterpret_cast<char const*>(&occupancy_clamping_thres_max_log_),
			                 sizeof(occupancy_clamping_thres_max_log_));
		} else {
			uint64_t const size = nodes.size();
			out_stream.write(reinterpret_cast<char const*>(&size), sizeof(size));
		}

		out_stream.write(reinterpret_cast<char const*>(data.get()),
		                 nodes.size() * sizeof(logit_t));
	}

 protected:
	//  Sensor model
	float occupancy_clamping_thres_min_log_;  // Min logit value
	float occupancy_clamping_thres_max_log_;  // Max logit value
	logit_t occupied_thres_log_;              // Threshold for occupied
	logit_t free_thres_log_;                  // Threshold for free

	// Occupancy propagation criteria
	PropagationCriteria occupancy_prop_criteria_ = PropagationCriteria::Max;

	template <class Derived2, class DataType2, class Indicators2>
	friend class OccupancyMapBase;
};

// std::false_type is_occupancy_map_base_impl(...);
// template <class Derived2, class DataType2, class Indicators2>
// std::true_type is_occupancy_map_base_impl(
//     OccupancyMapBase<Derived2, DataType2, Indicators2> const volatile&);

// template <typename T>
// using is_occupancy_map_base = decltype(is_occupancy_map_base_impl(std::declval<T&>()));

// // Helper variable template
// template <class T>
// inline constexpr bool is_occupancy_map_base_v = is_occupancy_map_base<T>::value;
}  // namespace ufo::map

#endif  // UFO_MAP_OCCUPANCY_MAP_BASE_H