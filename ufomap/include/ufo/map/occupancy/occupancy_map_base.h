/*
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * License: BSD 3
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

#ifndef UFO_MAP_OCCUPANCY_MAP_BASE_H
#define UFO_MAP_OCCUPANCY_MAP_BASE_H

// UFO
#include <ufo/geometry/aabb.h>
#include <ufo/map/map_base.h>
#include <ufo/map/occupancy/occupancy_node.h>
#include <ufo/map/octree/octree_base.h>
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
template <class Derived, class DataType, class Indicators = OccupancyIndicators>
class OccupancyMapBase : virtual public OctreeBase<Derived, DataType, Indicators>
{
 protected:
	using Base = OctreeBase<Derived, DataType, Indicators>;
	using LeafNode = typename Base::LeafNode;
	using InnerNode = typename Base::InnerNode;

	// Support float and uint8_t Occupancy value right now.
	// (and uint32_t given that it is packed with Time)
	static_assert(std::is_base_of_v<OccupancyNode<float>, LeafNode> ||
	              std::is_base_of_v<OccupancyNode<uint8_t>, LeafNode> ||
	              std::is_base_of_v<OccupancyTimeNode, LeafNode>);

 public:
	using LogitType = std::conditional_t<std::is_base_of_v<OccupancyTimeNode, DataType>,
	                                     uint8_t, decltype(DataType::occupancy)>;

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

	constexpr LogitType getOccupiedThresLogit() const noexcept
	{
		return occupied_thres_log_;
	}

	constexpr float getFreeThres() const noexcept
	{
		return toOccupancyProbability(getFreeThresLogit());
	}

	constexpr LogitType getFreeThresLogit() const noexcept { return free_thres_log_; }

	//
	// Probability <-> logit
	//

	constexpr LogitType toOccupancyLogit(float probability) const
	{
		return math::logit<LogitType>(probability, getOccupancyClampingThresMinLogit(),
		                              getOccupancyClampingThresMaxLogit());
	}

	constexpr float toOccupancyProbability(LogitType logit) const
	{
		if constexpr (std::is_same_v<LogitType, uint8_t>) {
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
		// TODO: Should add a warning that these are very computational expensive to
		// call since the whole tree has to be updated

		setOccupiedFreeThresLogit(toOccupancyLogit(new_occupied_thres),
		                          toOccupancyLogit(new_free_thres), propagate);
	}

	// FIXME: Look at
	void setOccupiedFreeThresLogit(LogitType new_occupied_thres, LogitType new_free_thres,
	                               bool propagate = true)
	{
		// TODO: Should add a warning that these are very computational expensive to
		// call since the whole tree has to be updated

		occupied_thres_log_ = new_occupied_thres;
		free_thres_log_ = new_free_thres;

		Base::setModifiedChildren();

		if (propagate) {
			Base::updateModifiedNodes();
		}
	}

	constexpr void setOccupancyClampingThres(float min_thres_probability,
	                                         float max_thres_probability)
	{
		// TODO: Should this be applied?
		setOccupancyClampingThresLogit(math::logit(min_thres_probability),
		                               math::logit(max_thres_probability));
	}

	constexpr void setOccupancyClampingThresLogit(LogitType min_thres, LogitType max_thres)
	{
		// TODO: What happens if uint8_t?
		setOccupancyClampingThresMinLogit(min_thres);
		setOccupancyClampingThresMaxLogit(max_thres);
	}

	constexpr void setOccupancyClampingThresMin(float probability)
	{
		setOccupancyClampingThresMinLogit(math::logit(probability));
	}

	constexpr void setOccupancyClampingThresMinLogit(LogitType min_thres)
	{
		// TODO: What happens if uint8_t?
		occupancy_clamping_thres_min_log_ = min_thres;
	}

	constexpr void setOccupancyClampingThresMax(float probability)
	{
		setOccupancyClampingThresMaxLogit(math::logit(probability));
	}

	constexpr void setOccupancyClampingThresMaxLogit(LogitType max_thres)
	{
		// TODO: What happens if uint8_t?
		occupancy_clamping_thres_max_log_ = max_thres;
	}

	//
	// Get bounding volume containing all known
	//

	geometry::AABB getKnownBBX() const
	{
		// TODO: Implement
	}

	//
	// Get occupancy state
	//

	OccupancyState getOccupancyState(MinimalNode const& node) const
	{
		if (isUnknown(Base::getLeafNode(node))) {
			return OccupancyState::UNKNOWN;
		} else if (isFree(Base::getLeafNode(node))) {
			return OccupancyState::FREE;
		} else {
			return OccupancyState::OCCUPIED;
		}
	}

	OccupancyState getOccupancyState(Code code) const
	{
		LeafNode const& node = Base::getLeafNode(code);
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
		return getOccupancyState(Base::toCode(key));
	}

	OccupancyState getOccupancyState(Point3 coord, DepthType depth = 0) const
	{
		return getOccupancyState(Base::toCode(coord, depth));
	}

	OccupancyState getOccupancyState(float x, float y, float z, DepthType depth = 0) const
	{
		return getOccupancyState(Base::toCode(x, y, z, depth));
	}

	//
	// Contains occupancy state
	//

	bool containsOccupancyState(OccupancyState state, MinimalNode const& node) const
	{
		switch (state) {
			case OccupancyState::UNKNOWN:
				return containsUnknown(Base::getLeafNode(node));
			case OccupancyState::FREE:
				return containsFree(Base::getLeafNode(node));
			case OccupancyState::OCCUPIED:
				return containsOccupied(Base::getLeafNode(node));
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
		return containsOccupancyState(state, Base::toCode(key));
	}

	bool containsOccupancyState(OccupancyState state, Point3 coord,
	                            DepthType depth = 0) const
	{
		return containsOccupancyState(state, Base::toCode(coord, depth));
	}

	bool containsOccupancyState(OccupancyState state, float x, float y, float z,
	                            DepthType depth = 0) const
	{
		return containsOccupancyState(state, Base::toCode(x, y, z, depth));
	}

	//
	// Is unknown
	//

	bool isUnknown(MinimalNode const& node) const
	{
		return isUnknown(Base::getLeafNode(node));
	}

	bool isUnknown(Code code) const { return isUnknown(Base::getLeafNode(code)); }

	bool isUnknown(Key key) const { return isUnknown(Base::toCode(key)); }

	bool isUnknown(Point3 coord, DepthType depth = 0) const
	{
		return isUnknown(Base::toCode(coord, depth));
	}

	bool isUnknown(float x, float y, float z, DepthType depth = 0) const
	{
		return isUnknown(Base::toCode(x, y, z, depth));
	}

	//
	// Is free
	//

	bool isFree(MinimalNode const& node) const { return isFree(Base::getLeafNode(node)); }

	bool isFree(Code code) const { return isFree(Base::getLeafNode(code)); }

	bool isFree(Key key) const { return isFree(Base::toCode(key)); }

	bool isFree(Point3 coord, DepthType depth = 0) const
	{
		return isFree(Base::toCode(coord, depth));
	}

	bool isFree(float x, float y, float z, DepthType depth = 0) const
	{
		return isFree(Base::toCode(x, y, z, depth));
	}

	//
	// Is occupied
	//

	bool isOccupied(MinimalNode const& node) const
	{
		return isOccupied(Base::getLeafNode(node));
	}

	bool isOccupied(Code code) const { return isOccupied(Base::getLeafNode(code)); }

	bool isOccupied(Key key) const { return isOccupied(Base::toCode(key)); }

	bool isOccupied(Point3 coord, DepthType depth = 0) const
	{
		return isOccupied(Base::toCode(coord, depth));
	}

	bool isOccupied(float x, float y, float z, DepthType depth = 0) const
	{
		return isOccupied(Base::toCode(x, y, z, depth));
	}

	//
	// Contains unknown
	//

	bool containsUnknown(MinimalNode const& node) const
	{
		if (Base::isLeaf(node)) {
			return isUnknown(node);
		}
		return containsUnknown(Base::getInnerNode(node));
	}

	bool containsUnknown(Code code) const
	{
		if (0 == code.getDepth()) {
			return isUnknown(code);
		}
		return containsUnknown(Base::getInnerNode(code));
	}

	bool containsUnknown(Key key) const { return containsUnknown(Base::toCode(key)); }

	bool containsUnknown(Point3 coord, DepthType depth = 0) const
	{
		return containsUnknown(Base::toCode(coord, depth));
	}

	bool containsUnknown(float x, float y, float z, DepthType depth = 0) const
	{
		return containsUnknown(Base::toCode(x, y, z, depth));
	}

	//
	// Contains free
	//

	bool containsFree(MinimalNode const& node) const
	{
		if (Base::isLeaf(node)) {
			return isFree(node);
		}
		return containsFree(Base::getInnerNode(node));
	}

	bool containsFree(Code code) const
	{
		if (0 == code.getDepth()) {
			return isFree(code);
		}
		return containsFree(Base::getInnerNode(code));
	}

	bool containsFree(Key key) const { return containsFree(Base::toCode(key)); }

	bool containsFree(Point3 coord, DepthType depth = 0) const
	{
		return containsFree(Base::toCode(coord, depth));
	}

	bool containsFree(float x, float y, float z, DepthType depth = 0) const
	{
		return containsFree(Base::toCode(x, y, z, depth));
	}

	//
	// Contains occupied
	//

	bool containsOccupied(MinimalNode const& node) const
	{
		if (Base::isLeaf(node)) {
			return isOccupied(node);
		}
		return containsOccupied(Base::getInnerNode(node));
	}

	bool containsOccupied(Code code) const
	{
		if (0 == code.getDepth()) {
			return isOccupied(code);
		}
		return containsOccupied(Base::getInnerNode(code));
	}

	bool containsOccupied(Key key) const { return containsOccupied(Base::toCode(key)); }

	bool containsOccupied(Point3 coord, DepthType depth = 0) const
	{
		return containsOccupied(Base::toCode(coord, depth));
	}

	bool containsOccupied(float x, float y, float z, DepthType depth = 0) const
	{
		return containsOccupied(Base::toCode(x, y, z, depth));
	}

	//
	// Get occupancy
	//

	float getOccupancy(MinimalNode const& node) const
	{
		return getOccupancy(Base::getLeafNode(node));
	}

	float getOccupancy(Code code) const { return getOccupancy(Base::getLeafNode(code)); }

	float getOccupancy(Key key) const { return getOccupancy(Base::toCode(key)); }

	float getOccupancy(Point3 coord, DepthType depth = 0) const
	{
		return getOccupancy(Base::toCode(coord, depth));
	}

	float getOccupancy(float x, float y, float z, DepthType depth = 0) const
	{
		return getOccupancy(Base::toCode(x, y, z, depth));
	}

	//
	// Get occupancy logit
	//

	LogitType getOccupancyLogit(MinimalNode const& node) const
	{
		return getOccupancyLogit(Base::getLeafNode(node));
	}

	LogitType getOccupancyLogit(Code code) const
	{
		return getOccupancyLogit(Base::getLeafNode(code));
	}

	LogitType getOccupancyLogit(Key key) const
	{
		return getOccupancyLogit(Base::toCode(key));
	}

	LogitType getOccupancyLogit(Point3 coord, DepthType depth = 0) const
	{
		return getOccupancyLogit(Base::toCode(coord, depth));
	}

	LogitType getOccupancyLogit(float x, float y, float z, DepthType depth = 0) const
	{
		return getOccupancyLogit(Base::toCode(x, y, z, depth));
	}

	//
	// Set occupancy
	//

	void setOccupancy(MinimalNode& node, float occupancy, bool propagate = true)
	{
		LogitType logit = toOccupancyLogit(occupancy);
		setOccupancyLogit(node, logit, propagate);
	}

	void setOccupancy(Code code, float occupancy, bool propagate = true)
	{
		LogitType logit = toOccupancyLogit(occupancy);
		setOccupancyLogit(code, logit, propagate);
	}

	void setOccupancy(Key key, float occupancy, bool propagate = true)
	{
		setOccupancy(Base::toCode(key), occupancy, propagate);
	}

	void setOccupancy(Point3 coord, float occupancy, bool propagate = true,
	                  DepthType depth = 0)
	{
		setOccupancy(Base::toCode(coord, depth), occupancy, propagate);
	}

	void setOccupancy(float x, float y, float z, float occupancy, bool propagate = true,
	                  DepthType depth = 0)
	{
		setOccupancy(Base::toCode(x, y, z, depth), occupancy, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setOccupancy(ExecutionPolicy policy, MinimalNode& node, float occupancy,
	                  bool propagate = true)
	{
		LogitType logit = toOccupancyLogit(occupancy);
		setOccupancyLogit(policy, node, logit, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setOccupancy(ExecutionPolicy policy, Code code, float occupancy,
	                  bool propagate = true)
	{
		LogitType logit = toOccupancyLogit(occupancy);
		setOccupancyLogit(policy, code, logit, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setOccupancy(ExecutionPolicy policy, Point3 coord, float occupancy,
	                  bool propagate = true, DepthType depth = 0)
	{
		setOccupancy(policy, Base::toCode(coord, depth), occupancy, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setOccupancy(ExecutionPolicy policy, float x, float y, float z, float occupancy,
	                  bool propagate = true, DepthType depth = 0)
	{
		setOccupancy(policy, Base::toCode(x, y, z, depth), occupancy, propagate);
	}

	//
	// Set occupancy logit
	//

	void setOccupancyLogit(MinimalNode& node, LogitType logit, bool propagate = true)
	{
		Base::apply(
		    node, [this, logit](LeafNode& node) { setOccupancyLogitImpl(node, logit); },
		    propagate);
	}

	void setOccupancyLogit(Code code, LogitType logit, bool propagate = true)
	{
		Base::apply(
		    code, [this, logit](LeafNode& node) { setOccupancyLogitImpl(node, logit); },
		    propagate);
	}

	void setOccupancyLogit(Key key, LogitType logit, bool propagate = true)
	{
		setOccupancyLogit(Base::toCode(key), logit, propagate);
	}

	void setOccupancyLogit(Point3 coord, LogitType logit, bool propagate = true,
	                       DepthType depth = 0)
	{
		setOccupancyLogit(Base::toCode(coord, depth), logit, propagate);
	}

	void setOccupancyLogit(float x, float y, float z, LogitType logit,
	                       bool propagate = true, DepthType depth = 0)
	{
		setOccupancyLogit(Base::toCode(x, y, z, depth), logit, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setOccupancyLogit(ExecutionPolicy policy, MinimalNode& node, LogitType logit,
	                       bool propagate = true)
	{
		Base::apply(
		    policy, node,
		    [this, logit](LeafNode& node) { setOccupancyLogitImpl(node, logit); }, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setOccupancyLogit(ExecutionPolicy policy, Code code, LogitType logit,
	                       bool propagate = true)
	{
		Base::apply(
		    policy, code,
		    [this, logit](LeafNode& node) { setOccupancyLogitImpl(node, logit); }, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setOccupancyLogit(ExecutionPolicy policy, Key key, LogitType logit,
	                       bool propagate = true)
	{
		setOccupancyLogit(policy, Base::toCode(key), logit, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setOccupancyLogit(ExecutionPolicy policy, Point3 coord, LogitType logit,
	                       bool propagate = true, DepthType depth = 0)
	{
		setOccupancyLogit(policy, Base::toCode(coord, depth), logit, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setOccupancyLogit(ExecutionPolicy policy, float x, float y, float z,
	                       LogitType logit, bool propagate = true, DepthType depth = 0)
	{
		setOccupancyLogit(policy, Base::toCode(x, y, z, depth), logit, propagate);
	}

	//
	// Increase occupancy
	//

	void increaseOccupancy(MinimalNode& node, float inc, bool propagate = true)
	{
		LogitType inc_logit;
		if constexpr (std::is_same_v<LogitType, uint8_t>) {
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
	void increaseOccupancy(ExecutionPolicy policy, MinimalNode& node, float inc,
	                       bool propagate = true)
	{
		LogitType inc_logit;
		if constexpr (std::is_same_v<LogitType, uint8_t>) {
			inc_logit = math::logitChangeValue(inc, getOccupancyClampingThresMinLogit(),
			                                   getOccupancyClampingThresMaxLogit());
		} else {
			inc_logit = math::logit(inc, getOccupancyClampingThresMinLogit(),
			                        getOccupancyClampingThresMaxLogit());
		}

		increaseOccupancyLogit(policy, node, inc_logit, propagate);
	}

	void increaseOccupancyLogit(MinimalNode& node, LogitType inc, bool propagate = true)
	{
		Base::apply(
		    node, [this, inc](LeafNode& node) { increaseOccupancyLogitImpl(node, inc); },
		    propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void increaseOccupancyLogit(ExecutionPolicy policy, MinimalNode& node, LogitType inc,
	                            bool propagate = true)
	{
		Base::apply(
		    policy, node,
		    [this, inc](LeafNode& node) { increaseOccupancyLogitImpl(node, inc); },
		    propagate);
	}

	void increaseOccupancy(Code code, float inc, bool propagate = true)
	{
		LogitType inc_logit;
		if constexpr (std::is_same_v<LogitType, uint8_t>) {
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
		LogitType inc_logit;
		if constexpr (std::is_same_v<LogitType, uint8_t>) {
			inc_logit = math::logitChangeValue(inc, getOccupancyClampingThresMinLogit(),
			                                   getOccupancyClampingThresMaxLogit());
		} else {
			inc_logit = math::logit(inc, getOccupancyClampingThresMinLogit(),
			                        getOccupancyClampingThresMaxLogit());
		}

		increaseOccupancyLogit(policy, code, inc_logit, propagate);
	}

	void increaseOccupancyLogit(Code code, LogitType inc, bool propagate = true)
	{
		Base::apply(
		    code, [this, inc](LeafNode& node) { increaseOccupancyLogitImpl(node, inc); },
		    propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void increaseOccupancyLogit(ExecutionPolicy policy, Code code, LogitType inc,
	                            bool propagate = true)
	{
		Base::apply(
		    policy, code,
		    [this, inc](LeafNode& node) { increaseOccupancyLogitImpl(node, inc); },
		    propagate);
	}

	void increaseOccupancy(Key key, float inc, bool propagate = true)
	{
		increaseOccupancy(Base::toCode(key), inc, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void increaseOccupancy(ExecutionPolicy policy, Key key, float inc,
	                       bool propagate = true)
	{
		increaseOccupancy(policy, Base::toCode(key), inc, propagate);
	}

	void increaseOccupancyLogit(Key key, LogitType inc, bool propagate = true)
	{
		increaseOccupancyLogit(Base::toCode(key), inc, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void increaseOccupancyLogit(ExecutionPolicy policy, Key key, LogitType inc,
	                            bool propagate = true)
	{
		increaseOccupancyLogit(policy, Base::toCode(key), inc, propagate);
	}

	void increaseOccupancy(Point3 coord, float inc, bool propagate = true,
	                       DepthType depth = 0)
	{
		increaseOccupancy(Base::toCode(coord, depth), inc, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void increaseOccupancy(ExecutionPolicy policy, Point3 coord, float inc,
	                       bool propagate = true, DepthType depth = 0)
	{
		increaseOccupancy(policy, Base::toCode(coord, depth), inc, propagate);
	}

	void increaseOccupancyLogit(Point3 coord, LogitType inc, bool propagate = true,
	                            DepthType depth = 0)
	{
		increaseOccupancyLogit(Base::toCode(coord, depth), inc, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void increaseOccupancyLogit(ExecutionPolicy policy, Point3 coord, LogitType inc,
	                            bool propagate = true, DepthType depth = 0)
	{
		increaseOccupancyLogit(policy, Base::toCode(coord, depth), inc, propagate);
	}

	void increaseOccupancy(float x, float y, float z, float inc, bool propagate = true,
	                       DepthType depth = 0)
	{
		increaseOccupancy(Base::toCode(x, y, z, depth), inc, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void increaseOccupancy(ExecutionPolicy policy, float x, float y, float z, float inc,
	                       bool propagate = true, DepthType depth = 0)
	{
		increaseOccupancy(policy, Base::toCode(x, y, z, depth), inc, propagate);
	}

	void increaseOccupancyLogit(float x, float y, float z, LogitType inc,
	                            bool propagate = true, DepthType depth = 0)
	{
		increaseOccupancyLogit(Base::toCode(x, y, z, depth), inc, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void increaseOccupancyLogit(ExecutionPolicy policy, float x, float y, float z,
	                            LogitType inc, bool propagate = true, DepthType depth = 0)
	{
		increaseOccupancyLogit(policy, Base::toCode(x, y, z, depth), inc, propagate);
	}

	//
	// Decrease occupancy
	//

	void decreaseOccupancy(MinimalNode& node, float dec, bool propagate = true)
	{
		LogitType dec_logit;
		if constexpr (std::is_same_v<LogitType, uint8_t>) {
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
	void decreaseOccupancy(ExecutionPolicy policy, MinimalNode& node, float dec,
	                       bool propagate = true)
	{
		LogitType dec_logit;
		if constexpr (std::is_same_v<LogitType, uint8_t>) {
			dec_logit = math::logitChangeValue(dec, getOccupancyClampingThresMinLogit(),
			                                   getOccupancyClampingThresMaxLogit());
		} else {
			dec_logit = math::logit(dec, getOccupancyClampingThresMinLogit(),
			                        getOccupancyClampingThresMaxLogit());
		}

		decreaseOccupancyLogit(policy, node, dec_logit, propagate);
	}

	void decreaseOccupancyLogit(MinimalNode& node, LogitType dec, bool propagate = true)
	{
		Base::apply(
		    node, [this, dec](LeafNode& node) { decreaseOccupancyLogitImpl(node, dec); },
		    propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void decreaseOccupancyLogit(ExecutionPolicy policy, MinimalNode& node, LogitType dec,
	                            bool propagate = true)
	{
		Base::apply(
		    policy, node,
		    [this, dec](LeafNode& node) { decreaseOccupancyLogitImpl(node, dec); },
		    propagate);
	}

	void decreaseOccupancy(Code code, float dec, bool propagate = true)
	{
		LogitType dec_logit;
		if constexpr (std::is_same_v<LogitType, uint8_t>) {
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
		LogitType dec_logit;
		if constexpr (std::is_same_v<LogitType, uint8_t>) {
			dec_logit = math::logitChangeValue(dec, getOccupancyClampingThresMinLogit(),
			                                   getOccupancyClampingThresMaxLogit());
		} else {
			dec_logit = math::logit(dec, getOccupancyClampingThresMinLogit(),
			                        getOccupancyClampingThresMaxLogit());
		}

		decreaseOccupancyLogit(policy, code, dec_logit, propagate);
	}

	void decreaseOccupancyLogit(Code code, LogitType dec, bool propagate = true)
	{
		Base::apply(
		    code, [this, dec](LeafNode& node) { decreaseOccupancyLogitImpl(node, dec); },
		    propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void decreaseOccupancyLogit(ExecutionPolicy policy, Code code, LogitType dec,
	                            bool propagate = true)
	{
		Base::apply(
		    policy, code,
		    [this, dec](LeafNode& node) { decreaseOccupancyLogitImpl(node, dec); },
		    propagate);
	}

	void decreaseOccupancy(Key key, float dec, bool propagate = true)
	{
		decreaseOccupancy(Base::toCode(key), dec, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void decreaseOccupancy(ExecutionPolicy policy, Key key, float dec,
	                       bool propagate = true)
	{
		decreaseOccupancy(policy, Base::toCode(key), dec, propagate);
	}

	void decreaseOccupancyLogit(Key key, LogitType dec, bool propagate = true)
	{
		decreaseOccupancyLogit(Base::toCode(key), dec, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void decreaseOccupancyLogit(ExecutionPolicy policy, Key key, LogitType dec,
	                            bool propagate = true)
	{
		decreaseOccupancyLogit(policy, Base::toCode(key), dec, propagate);
	}

	void decreaseOccupancy(Point3 coord, float dec, bool propagate = true,
	                       DepthType depth = 0)
	{
		decreaseOccupancy(Base::toCode(coord, depth), dec, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void decreaseOccupancy(ExecutionPolicy policy, Point3 coord, float dec,
	                       bool propagate = true, DepthType depth = 0)
	{
		decreaseOccupancy(policy, Base::toCode(coord, depth), dec, propagate);
	}

	void decreaseOccupancyLogit(Point3 coord, LogitType dec, bool propagate = true,
	                            DepthType depth = 0)
	{
		decreaseOccupancyLogit(Base::toCode(coord, depth), dec, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void decreaseOccupancyLogit(ExecutionPolicy policy, Point3 coord, LogitType dec,
	                            bool propagate = true, DepthType depth = 0)
	{
		decreaseOccupancyLogit(policy, Base::toCode(coord, depth), dec, propagate);
	}

	void decreaseOccupancy(float x, float y, float z, float dec, bool propagate = true,
	                       DepthType depth = 0)
	{
		decreaseOccupancy(Base::toCode(x, y, z, depth), dec, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void decreaseOccupancy(ExecutionPolicy policy, float x, float y, float z, float dec,
	                       bool propagate = true, DepthType depth = 0)
	{
		decreaseOccupancy(policy, Base::toCode(x, y, z, depth), dec, propagate);
	}

	void decreaseOccupancyLogit(float x, float y, float z, LogitType dec,
	                            bool propagate = true, DepthType depth = 0)
	{
		decreaseOccupancyLogit(Base::toCode(x, y, z, depth), dec, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void decreaseOccupancyLogit(ExecutionPolicy policy, float x, float y, float z,
	                            LogitType dec, bool propagate = true, DepthType depth = 0)
	{
		decreaseOccupancyLogit(policy, Base::toCode(x, y, z, depth), dec, propagate);
	}

 protected:
	//
	// Constructors
	//

	OccupancyMapBase(float resolution, DepthType depth_levels = 16,
	                 bool automatic_pruning = true, float occupied_thres = 0.5,
	                 float free_thres = 0.5, float clamping_thres_min = 0.1192,
	                 float clamping_thres_max = 0.971)
	    : Base(resolution, depth_levels, automatic_pruning),
	      occupancy_clamping_thres_min_log_(math::logit(clamping_thres_min)),
	      occupancy_clamping_thres_max_log_(math::logit(clamping_thres_max))
	{
		occupied_thres_log_ = toOccupancyLogit(occupied_thres);
		free_thres_log_ = toOccupancyLogit(free_thres);

		printf("OccupancyMapBase constructor\n");
		// TODO: Implement?
	}

	OccupancyMapBase(OccupancyMapBase const& other)
	    : Base(other),
	      occupancy_clamping_thres_min_log_(other.occupancy_clamping_thres_min_log_),
	      occupancy_clamping_thres_max_log_(other.occupancy_clamping_thres_max_log_),
	      occupied_thres_log_(other.occupied_thres_log_),
	      free_thres_log_(other.free_thres_log_)
	{
		printf("OccupancyMapBase copy constructor\n");
		// TODO: Implement?
	}

	template <class D1, class D2, class I>
	OccupancyMapBase(OccupancyMapBase<D1, D2, I> const& other)
	    : Base(other),
	      occupancy_clamping_thres_min_log_(other.occupancy_clamping_thres_min_log_),
	      occupancy_clamping_thres_max_log_(other.occupancy_clamping_thres_max_log_)
	{
		printf("OccupancyMapBase template copy constructor\n");

		occupied_thres_log_ = toOccupancyLogit(other.getOccupiedThres());
		free_thres_log_ = toOccupancyLogit(other.getFreeThres());
	}

	OccupancyMapBase(OccupancyMapBase&& other)
	    : Base(std::move(other)),
	      occupancy_clamping_thres_min_log_(
	          std::move(other.occupancy_clamping_thres_min_log_)),
	      occupancy_clamping_thres_max_log_(
	          std::move(other.occupancy_clamping_thres_max_log_)),
	      occupied_thres_log_(std::move(other.occupied_thres_log_)),
	      free_thres_log_(std::move(other.free_thres_log_))
	{
		printf("OccupancyMapBase move constructor\n");
		// TODO: Implement?
	}

	OccupancyMapBase& operator=(OccupancyMapBase const& rhs)
	{
		printf("OccupancyMapBase copy assignment\n");
		// Base::operator=(rhs);
		occupancy_clamping_thres_min_log_ = rhs.occupancy_clamping_thres_min_log_;
		occupancy_clamping_thres_max_log_ = rhs.occupancy_clamping_thres_max_log_;
		occupied_thres_log_ = rhs.occupied_thres_log_;
		free_thres_log_ = rhs.free_thres_log_;
		return *this;
	}

	template <class D1, class D2, class I>
	OccupancyMapBase& operator=(OccupancyMapBase<D1, D2, I> const& rhs)
	{
		printf("OccupancyMapBase template copy assignment\n");
		// Base::operator=(rhs);
		occupancy_clamping_thres_min_log_ = rhs.occupancy_clamping_thres_min_log_;
		occupancy_clamping_thres_max_log_ = rhs.occupancy_clamping_thres_max_log_;

		occupied_thres_log_ = toOccupancyLogit(rhs.getOccupiedThres());
		free_thres_log_ = toOccupancyLogit(rhs.getFreeThres());

		return *this;
	}

	OccupancyMapBase& operator=(OccupancyMapBase&& rhs)
	{
		printf("OccupancyMapBase move assignment\n");
		// Base::operator=(std::move(rhs));
		occupancy_clamping_thres_min_log_ = std::move(rhs.occupancy_clamping_thres_min_log_);
		occupancy_clamping_thres_max_log_ = std::move(rhs.occupancy_clamping_thres_max_log_);
		occupied_thres_log_ = std::move(rhs.occupied_thres_log_);
		free_thres_log_ = std::move(rhs.free_thres_log_);
		return *this;
	}

	//
	// Destructor
	//

	virtual ~OccupancyMapBase() override { printf("OccupancyMapBase destructor\n"); }

	//
	// Initilize root
	//

	virtual void initRoot() override
	{
		printf("OccupancyMapBase initRoot\n");
		Base::initRoot();
		Base::getRoot().occupancy = toOccupancyLogit(0.5);
		OccupancyMapBase::updateNodeIndicators(Base::getRoot());
	}

	//
	// Get occupancy
	//

	constexpr float getOccupancy(LeafNode const& node) const noexcept
	{
		return toOccupancyProbability(getOccupancyLogit(node));
	}

	constexpr LogitType getOccupancyLogit(LeafNode const& node) const noexcept
	{
		return node.occupancy;
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
		return node.occupancy < getFreeThresLogit();
	}

	constexpr bool isOccupied(LeafNode const& node) const noexcept
	{
		return node.occupancy > getOccupiedThresLogit();
	}

	constexpr bool containsUnknown(LeafNode const& node) const noexcept
	{
		return node.contains_unknown;
	}

	constexpr bool containsFree(LeafNode const& node) const noexcept
	{
		return node.contains_free;
	}

	constexpr bool containsOccupied(LeafNode const& node) const noexcept
	{
		return node.contains_occupied;
	}

	//
	// Set occupancy
	//

	constexpr void setOccupancyLogitImpl(LeafNode& node, LogitType new_occupancy)
	{
		if constexpr (std::is_same_v<LogitType, uint8_t>) {
			node.occupancy = new_occupancy;
		} else {
			node.occupancy = std::clamp(new_occupancy, getOccupancyClampingThresMinLogit(),
			                            getOccupancyClampingThresMaxLogit());
		}
	}

	//
	// Increase occupancy
	//

	constexpr void increaseOccupancyLogitImpl(LeafNode& node, LogitType inc)
	{
		if constexpr (std::is_same_v<LogitType, uint8_t>) {
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

	constexpr void decreaseOccupancyLogitImpl(LeafNode& node, LogitType dec)
	{
		if constexpr (std::is_same_v<LogitType, uint8_t>) {
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
	virtual void updateNode(InnerNode& node, DepthType depth) override
	{
		node.occupancy = std::numeric_limits<LogitType>::lowest();

		if (1 == depth) {
			for (auto const& child : Base::getLeafChildren(node)) {
				node.occupancy = std::max(node.occupancy, child.occupancy);
			}
		} else {
			for (auto const& child : Base::getInnerChildren(node)) {
				node.occupancy = std::max(node.occupancy, child.occupancy);
			}
		}
	}

	// NOTE: Only called when node has no children
	virtual void updateNodeIndicators(LeafNode& node) override
	{
		node.contains_unknown = isUnknown(node);
		node.contains_free = isFree(node);
		node.contains_occupied = isOccupied(node);
	}

	// NOTE: Only called when node has children
	virtual void updateNodeIndicators(InnerNode& node, DepthType depth) override
	{
		node.contains_unknown = false;
		node.contains_free = false;
		node.contains_occupied = false;

		if (1 == depth) {
			for (auto const& child : Base::getLeafChildren(node)) {
				node.contains_unknown = node.contains_unknown || child.contains_unknown;
				node.contains_free = node.contains_free || child.contains_free;
				node.contains_occupied = node.contains_occupied || child.contains_occupied;
			}
		} else {
			for (auto const& child : Base::getInnerChildren(node)) {
				node.contains_unknown = node.contains_unknown || child.contains_unknown;
				node.contains_free = node.contains_free || child.contains_free;
				node.contains_occupied = node.contains_occupied || child.contains_occupied;
			}
		}
	}

	//
	// Input/output (read/write)
	//

	virtual void addFileInfo(FileInfo& info) const override
	{
		info["fields"].emplace_back("occupancy");
		if constexpr (std::is_same_v<LogitType, uint8_t>) {
			info["type"].emplace_back("U");
			info["size"].emplace_back("1");
		} else {
			info["type"].emplace_back("F");
			info["size"].emplace_back("4");
		}
	}

	virtual bool readNodes(std::istream& in_stream, std::vector<LeafNode*> const& nodes,
	                       std::string const& field, char type, uint64_t size,
	                       uint64_t num) override
	{
		if ("occupancy" != field) {
			return false;
		}

		// TODO: Make parallel
		if ('U' == type && 1 == size) {
			// Get min/max threshold
			float min_logit;
			float max_logit;
			in_stream.read(reinterpret_cast<char*>(&min_logit), sizeof(min_logit));
			in_stream.read(reinterpret_cast<char*>(&max_logit), sizeof(max_logit));

			std::vector<uint8_t> data(nodes.size());
			in_stream.read(reinterpret_cast<char*>(data.data()), data.size() * sizeof(uint8_t));

			if constexpr (std::is_same_v<LogitType, uint8_t>) {
				if (getOccupancyClampingThresMinLogit() == min_logit &&
				    getOccupancyClampingThresMaxLogit() == max_logit) {
					for (size_t i = 0; i != nodes.size(); ++i) {
						nodes[i]->occupancy = data[i];
					}
				} else {
					for (size_t i = 0; i != nodes.size(); ++i) {
						nodes[i]->occupancy = math::convertLogit<uint8_t>(
						    math::convertLogit(data[i], min_logit, max_logit),
						    getOccupancyClampingThresMinLogit(), getOccupancyClampingThresMaxLogit());
					}
				}
			} else {
				for (size_t i = 0; i != nodes.size(); ++i) {
					nodes[i]->occupancy = std::clamp(
					    math::convertLogit(data[i], min_logit, max_logit),
					    getOccupancyClampingThresMinLogit(), getOccupancyClampingThresMaxLogit());
				}
			}
		} else if ('F' == type && 4 == size) {
			std::vector<float> data(nodes.size());
			in_stream.read(reinterpret_cast<char*>(data.data()), data.size() * sizeof(float));

			for (size_t i = 0; i != nodes.size(); ++i) {
				if constexpr (std::is_same_v<LogitType, uint8_t>) {
					nodes[i]->occupancy = math::convertLogit<uint8_t>(
					    std::clamp(data[i], getOccupancyClampingThresMinLogit(),
					               getOccupancyClampingThresMaxLogit()),
					    getOccupancyClampingThresMinLogit(), getOccupancyClampingThresMaxLogit());
				} else {
					nodes[i]->occupancy = std::clamp(data[i], getOccupancyClampingThresMinLogit(),
					                                 getOccupancyClampingThresMaxLogit());
				}
			}
		} else {
			// TODO: Error
			return false;
		}

		return true;
	}

	virtual void writeNodes(std::ostream& out_stream,
	                        std::vector<LeafNode const*> const& nodes, bool compress,
	                        int compression_acceleration_level,
	                        int compression_level) const override
	{
		std::vector<LogitType> data(nodes.size());
		for (size_t i = 0; i != nodes.size(); ++i) {
			data[i] = nodes[i]->occupancy;
		}

		if constexpr (std::is_same_v<LogitType, uint8_t>) {
			uint64_t size = nodes.size() + sizeof(occupancy_clamping_thres_min_log_) +
			                sizeof(occupancy_clamping_thres_max_log_);

			out_stream.write(reinterpret_cast<char*>(&size), sizeof(size));

			out_stream.write(reinterpret_cast<char const*>(&occupancy_clamping_thres_min_log_),
			                 sizeof(occupancy_clamping_thres_min_log_));
			out_stream.write(reinterpret_cast<char const*>(&occupancy_clamping_thres_max_log_),
			                 sizeof(occupancy_clamping_thres_max_log_));
		} else {
			uint64_t size = nodes.size();
			out_stream.write(reinterpret_cast<char*>(&size), sizeof(size));
		}

		out_stream.write(reinterpret_cast<char const*>(data.data()),
		                 data.size() * sizeof(typename decltype(data)::value_type));
	}

 protected:
	//  Sensor model
	float occupancy_clamping_thres_min_log_;  // Min logit value
	float occupancy_clamping_thres_max_log_;  // Max logit value
	LogitType occupied_thres_log_;            // Threshold for occupied
	LogitType free_thres_log_;                // Threshold for free

	// Propagation criteria
	PropagationCriteria prop_criteria;
	// TODO: Add prop_function

	template <class D1, class D2, class I>
	friend class OccupancyMapBase;
};

std::false_type is_occupancy_map_base_impl(...);
template <class D1, class D2, class I>
std::true_type is_occupancy_map_base_impl(OccupancyMapBase<D1, D2, I> const volatile&);

template <typename T>
using is_occupancy_map_base = decltype(is_occupancy_map_base_impl(std::declval<T&>()));

// Helper variable template
template <class T>
inline constexpr bool is_occupancy_map_base_v = is_occupancy_map_base<T>::value;
}  // namespace ufo::map

#endif  // UFO_MAP_OCCUPANCY_MAP_BASE_H