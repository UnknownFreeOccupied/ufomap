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
#include <execution>
#include <numeric>
#include <sstream>
#include <type_traits>
#include <deque>

namespace ufo::map
{
template <class Derived, class LeafNode, class InnerNode>
class OccupancyMapBase
{
 protected:
	static_assert(std::is_base_of_v<LeafNode, InnerNode>);

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

	constexpr logit_t toOccupancyChangeLogit(float probability) const
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
		setOccupancyClampingThresMin(min_thres_probability);
		setOccupancyClampingThresMax(max_thres_probability);
	}

	constexpr void setOccupancyClampingThresMin(float probability)
	{
		occupancy_clamping_thres_min_log_ = math::logit(probability);
		// FIXME: Should this update occupancy values?
	}

	constexpr void setOccupancyClampingThresMax(float probability)
	{
		occupancy_clamping_thres_max_log_ = math::logit(probability);
		// FIXME: Should this update occupancy values?
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

	OccupancyState getOccupancyState(Node const& node) const
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
				return containsUnknown(derived().getLeafNode(node));
			case OccupancyState::FREE:
				return containsFree(derived().getLeafNode(node));
			case OccupancyState::OCCUPIED:
				return containsOccupied(derived().getLeafNode(node));
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

	bool isUnknown(Node const& node) const
	{
		return isUnknown(derived().getLeafNode(node));
	}

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

	bool isFree(Node const& node) const { return isFree(derived().getLeafNode(node)); }

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
		return isOccupied(derived().getLeafNode(node));
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
		return getOccupancy(derived().getLeafNode(node));
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
		return getOccupancyLogit(derived().getLeafNode(node));
	}

	logit_t getOccupancyLogit(Code code) const
	{
		return getOccupancyLogit(derived().getLeafNode(code));
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
		increaseOccupancyLogit(node, toOccupancyChangeLogit(inc), propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void increaseOccupancy(ExecutionPolicy policy, Node& node, float inc,
	                       bool propagate = true)
	{
		increaseOccupancyLogit(policy, node, toOccupancyChangeLogit(inc), propagate);
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
		increaseOccupancyLogit(code, toOccupancyChangeLogit(inc), propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void increaseOccupancy(ExecutionPolicy policy, Code code, float inc,
	                       bool propagate = true)
	{
		increaseOccupancyLogit(policy, code, toOccupancyChangeLogit(inc), propagate);
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
		decreaseOccupancyLogit(node, toOccupancyChangeLogit(dec), propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void decreaseOccupancy(ExecutionPolicy policy, Node& node, float dec,
	                       bool propagate = true)
	{
		decreaseOccupancyLogit(policy, node, toOccupancyChangeLogit(dec), propagate);
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
		decreaseOccupancyLogit(code, toOccupancyChangeLogit(dec), propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void decreaseOccupancy(ExecutionPolicy policy, Code code, float dec,
	                       bool propagate = true)
	{
		decreaseOccupancyLogit(policy, code, toOccupancyChangeLogit(dec), propagate);
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

	OccupancyMapBase(OccupancyMapBase const& other) = default;

	OccupancyMapBase(OccupancyMapBase&& other) = default;

	OccupancyMapBase& operator=(OccupancyMapBase const& rhs) = default;

	OccupancyMapBase& operator=(OccupancyMapBase&& rhs) = default;

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

	constexpr bool isUnknown(LeafNode const& node) const noexcept
	{
		return !isFree(node) && !isOccupied(node);
	}

	constexpr bool isFree(LeafNode const& node) const noexcept
	{
		return getOccupancyLogit(node) < getFreeThresLogit();
	}

	constexpr bool isOccupied(LeafNode const& node) const noexcept
	{
		return getOccupancyLogit(node) > getOccupiedThresLogit();
	}

	constexpr bool containsUnknown(LeafNode const& node) const noexcept
	{
		// return node.contains_unknown;
		return derived().isLeaf(node) ? isUnknown(node) : node.contains_unknown;
	}

	constexpr bool containsFree(LeafNode const& node) const noexcept
	{
		// node.contains_free;
		return derived().isLeaf(node) ? isFree(node) : node.contains_free;
	}

	constexpr bool containsOccupied(LeafNode const& node) const noexcept
	{
		// node.contains_occupied;
		return derived().isLeaf(node) ? isOccupied(node) : node.contains_occupied;
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
			case PropagationCriteria::MIN:
				setOccupancyLogit(node, minChildOccupancyLogit(node, depth));
				break;
			case PropagationCriteria::MAX:
				setOccupancyLogit(node, maxChildOccupancyLogit(node, depth));
				break;
			case PropagationCriteria::MEAN:
				setOccupancyLogit(node, averageChildOccupancyLogit(node, depth));
				break;
		}
	}

	//
	// Min child occupancy logit
	//

	constexpr logit_t minChildOccupancyLogit(InnerNode const& node, depth_t depth) const
	{
		logit_t min = std::numeric_limits<logit_t>::max();

		if (1 == depth) {
			for (LeafNode const& child : derived().getLeafChildren(node)) {
				min = std::min(min, getOccupancyLogit(child));
			}
		} else {
			for (InnerNode const& child : derived().getInnerChildren(node)) {
				min = std::min(min, getOccupancyLogit(child));
			}
		}

		return min;
	}

	//
	// Max child occupancy logit
	//

	constexpr logit_t maxChildOccupancyLogit(InnerNode const& node, depth_t depth) const
	{
		logit_t max = std::numeric_limits<logit_t>::lowest();

		if (1 == depth) {
			for (LeafNode const& child : derived().getLeafChildren(node)) {
				max = std::max(max, getOccupancyLogit(child));
			}
		} else {
			for (InnerNode const& child : derived().getInnerChildren(node)) {
				max = std::max(max, getOccupancyLogit(child));
			}
		}

		return max;
	}

	//
	// Average child occupancy logit
	//

	constexpr logit_t averageChildOccupancyLogit(InnerNode const& node, depth_t depth) const
	{
		if constexpr (std::is_same_v<logit_t, uint8_t>) {
			unsigned int sum =
			    1 == depth ? std::accumulate(std::begin(derived().getLeafChildren(node)),
			                                 std::end(derived().getLeafChildren(node)), 0u,
			                                 [](auto cur, auto const& child) {
				                                 return cur + getOccupancyLogit(child);
			                                 })
			               : std::accumulate(std::begin(derived().getInnerChildren(node)),
			                                 std::end(derived().getInnerChildren(node)), 0u,
			                                 [](auto cur, auto const& child) {
				                                 return cur + getOccupancyLogit(child);
			                                 });

			return sum / 8u;
		} else {
			float sum = 1 == depth
			                ? std::accumulate(std::begin(derived().getLeafChildren(node)),
			                                  std::end(derived().getLeafChildren(node)), 0.0f,
			                                  [](auto cur, auto const& child) {
				                                  return cur + getOccupancyLogit(child);
			                                  })
			                : std::accumulate(std::begin(derived().getInnerChildren(node)),
			                                  std::end(derived().getInnerChildren(node)), 0.0f,
			                                  [](auto cur, auto const& child) {
				                                  return cur + getOccupancyLogit(child);
			                                  });

			return sum / 8.0f;
		}
	}

	// NOTE: Only called when node has no children
	void updateNodeIndicators(LeafNode& node)
	{
		// node.contains_unknown = isUnknown(node);
		// node.contains_free = isFree(node);
		// node.contains_occupied = isOccupied(node);
	}

	// NOTE: Only called when node has children
	void updateNodeIndicators(InnerNode& node, depth_t depth)
	{
		if (1 == depth) {
			any_of(derived().getLeafChildren(node),
			       [this](LeafNode const& child) { return containsUnknown(child); });
			node.contains_free =
			    any_of(derived().getLeafChildren(node),
			           [this](LeafNode const& child) { return containsFree(child); });
			node.contains_occupied =
			    any_of(derived().getLeafChildren(node),
			           [this](LeafNode const& child) { return containsOccupied(child); });
		} else {
			node.contains_unknown =
			    any_of(derived().getInnerChildren(node),
			           [this](InnerNode const& child) { return containsUnknown(child); });
			node.contains_free =
			    any_of(derived().getInnerChildren(node),
			           [this](InnerNode const& child) { return containsFree(child); });
			node.contains_occupied =
			    any_of(derived().getInnerChildren(node),
			           [this](InnerNode const& child) { return containsOccupied(child); });
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
		} else {
			info["type"].emplace_back("F");
		}
		info["size"].emplace_back(std::to_string(sizeof(logit_t)));
	}

	bool readNodes(std::istream& in_stream, std::vector<LeafNode*> const& nodes,
	               std::string const& field, char type, uint64_t size)
	{
		if ("occupancy" != field) {
			return false;
		}

		if ('U' == type && sizeof(uint8_t) == size) {
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
		} else if ('F' == type && sizeof(float) == size) {
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

	void writeNodes(std::ostream& out_stream, std::vector<LeafNode> const& nodes) const
	{
		if constexpr (std::is_same_v<logit_t, uint8_t>) {
			uint64_t const size = (nodes.size() * sizeof(logit_t)) +
			                      sizeof(occupancy_clamping_thres_min_log_) +
			                      sizeof(occupancy_clamping_thres_max_log_);

			out_stream.write(reinterpret_cast<char const*>(&size), sizeof(size));

			out_stream.write(reinterpret_cast<char const*>(&occupancy_clamping_thres_min_log_),
			                 sizeof(occupancy_clamping_thres_min_log_));
			out_stream.write(reinterpret_cast<char const*>(&occupancy_clamping_thres_max_log_),
			                 sizeof(occupancy_clamping_thres_max_log_));
		} else {
			uint64_t const size = nodes.size() * sizeof(logit_t);
			out_stream.write(reinterpret_cast<char const*>(&size), sizeof(size));
		}

		auto data = std::make_unique<logit_t[]>(nodes.size());
		for (size_t i = 0; i != nodes.size(); ++i) {
			data[i] = getOccupancyLogit(nodes[i]);
		}

		out_stream.write(reinterpret_cast<char const*>(data.get()),
		                 nodes.size() * sizeof(logit_t));
	}

	void writeNodes(std::ostream& out_stream, std::deque<LeafNode> const& nodes) const
	{
		if constexpr (std::is_same_v<logit_t, uint8_t>) {
			uint64_t const size = (nodes.size() * sizeof(logit_t)) +
			                      sizeof(occupancy_clamping_thres_min_log_) +
			                      sizeof(occupancy_clamping_thres_max_log_);

			out_stream.write(reinterpret_cast<char const*>(&size), sizeof(size));

			out_stream.write(reinterpret_cast<char const*>(&occupancy_clamping_thres_min_log_),
			                 sizeof(occupancy_clamping_thres_min_log_));
			out_stream.write(reinterpret_cast<char const*>(&occupancy_clamping_thres_max_log_),
			                 sizeof(occupancy_clamping_thres_max_log_));
		} else {
			uint64_t const size = nodes.size() * sizeof(logit_t);
			out_stream.write(reinterpret_cast<char const*>(&size), sizeof(size));
		}

		auto data = std::make_unique<logit_t[]>(nodes.size());
		for (size_t i = 0; i != nodes.size(); ++i) {
			data[i] = getOccupancyLogit(nodes[i]);
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
	PropagationCriteria occupancy_prop_criteria_ = PropagationCriteria::MAX;
};

std::false_type is_occupancy_map_base_impl(...);

template <class Derived, class LeafNode, class InnerNode>
std::true_type is_occupancy_map_base_impl(
    OccupancyMapBase<Derived, LeafNode, InnerNode> const volatile&);

template <typename T>
using is_occupancy_map_base = decltype(is_occupancy_map_base_impl(std::declval<T&>()));

// Helper variable template
template <class T>
inline constexpr bool is_occupancy_map_base_v = is_occupancy_map_base<T>::value;
}  // namespace ufo::map

#endif  // UFO_MAP_OCCUPANCY_MAP_BASE_H