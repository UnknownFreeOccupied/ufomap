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

#ifndef UFO_MAP_OCCUPANCY_MAP_HPP
#define UFO_MAP_OCCUPANCY_MAP_HPP

// UFO
#include <ufo/container/tree/container.hpp>
#include <ufo/container/tree/tree.hpp>
#include <ufo/map/occupancy/block.hpp>
#include <ufo/map/occupancy/predicate.hpp>
#include <ufo/map/occupancy/propagation_criteria.hpp>
#include <ufo/map/occupancy/state.hpp>
#include <ufo/map/tree/map_block.hpp>
#include <ufo/map/type.hpp>
#include <ufo/math/transform3.hpp>
#include <ufo/utility/bit_set.hpp>
#include <ufo/utility/io/buffer.hpp>
#include <ufo/utility/macros.hpp>
#include <ufo/utility/type_traits.hpp>

// STL
#include <array>
#include <iostream>
#include <limits>
#include <string_view>
#include <type_traits>

namespace ufo
{
template <class Derived, class Tree>
class OccupancyMap
{
 private:
	template <class Derived2, class Tree2>
	friend class OccupancyMap;

	static constexpr auto const BF  = Tree::branchingFactor();
	static constexpr auto const Dim = Tree::dimensions();

	using Block = OccupancyBlock<BF>;

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                        Tags                                         |
	|                                                                                     |
	**************************************************************************************/

	static constexpr MapType const Type = MapType::OCCUPANCY;

	// Container
	using Index    = typename Tree::Index;
	using Node     = typename Tree::Node;
	using Code     = typename Tree::Code;
	using Key      = typename Tree::Key;
	using Point    = typename Tree::Point;
	using Coord    = typename Tree::Coord;
	using coord_t  = typename Tree::coord_t;
	using depth_t  = typename Tree::depth_t;
	using offset_t = typename Tree::offset_t;
	using length_t = typename Tree::length_t;
	using pos_t    = typename Tree::pos_t;

	// Occupancy
	using logit_t     = typename Block::logit_t;
	using occupancy_t = float;

 private:
	static constexpr occupancy_t MaxOccupancy = 1.0;
	static constexpr occupancy_t MinOccupancy = 0.0;
	static constexpr logit_t     MaxLogit     = OccupancyElement::MAX_VALUE;
	static constexpr logit_t     MinLogit     = -MaxLogit;

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                       Access                                        |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] logit_t occupancyLogit(occupancy_t probability) const
	{
		assert(0 <= probability && 1 >= probability);

		occupancy_t c = occupancyClampingThresLogit();
		occupancy_t l = logit(probability, -c, c);
		return std::round(l / c * static_cast<occupancy_t>(MaxLogit));
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] logit_t occupancyLogit(NodeType node) const
	{
		Index n = derived().index(node);
		return occupancyBlock(n.pos)[n.offset].logit();
	}

	[[nodiscard]] occupancy_t occupancy(logit_t logit) const
	{
		assert(MinLogit <= logit && MaxLogit >= logit);

		occupancy_t max   = static_cast<occupancy_t>(MaxLogit);
		occupancy_t value = static_cast<occupancy_t>(logit);
		return probability(occupancyClampingThresLogit() * (value / max));
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] occupancy_t occupancy(NodeType node) const
	{
		return occupancy(occupancyLogit(node));
	}

	/**************************************************************************************
	|                                                                                     |
	|                                         GPU                                         |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] WGPUBuffer gpuOccupancyBuffer() const
	{
		return derived().template gpuBuffer<Block>();
	}

	[[nodiscard]] std::size_t gpuOccupancyBufferSize() const
	{
		return derived().template gpuBufferSize<Block>();
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Modifiers                                      |
	|                                                                                     |
	**************************************************************************************/

	/**
	 * @brief
	 *
	 * @note If `NodeType` is `Index`, only propagates up to `node` and it does not set
	 * modified if propagation is `false`.
	 *
	 * @tparam NodeType Should be of type `Node`, `Code`, `Key`, or `Coord`
	 * @param node
	 * @param value
	 * @param propagate
	 */
	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	void occupancySetLogit(NodeType node, logit_t value, bool propagate = true)
	{
		assert(MinLogit <= value && MaxLogit >= value);

		OccupancyElement elem(value, occupancyUnknownLogit(value), occupancyFreeLogit(value),
		                      occupancyOccupiedLogit(value));

		auto node_f = [this, elem](Index node) {
			occupancyBlock(node.pos)[node.offset] = elem;
		};
		auto block_f = [this, elem](pos_t pos) { occupancyBlock(pos).fill(elem); };

		if constexpr (std::is_same_v<Index, std::decay_t<NodeType>>) {
			if (propagate) {
				derived().recursParentFirst(node, node_f, block_f);
			} else {
				derived().recursLeaves(node, node_f, block_f);
			}
		} else {
			if (propagate) {
				auto propagate_f = [this](Index node, pos_t children) {
					onPropagateChildren(node, children);
				};

				derived().recursLeaves(derived().code(node), node_f, block_f, propagate_f,
				                       propagate);
			} else {
				derived().recursParentFirst(derived().code(node), node_f, block_f);
			}
		}
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	void occupancySet(NodeType node, occupancy_t value, bool propagate = true)
	{
		assert(MinOccupancy <= value && MaxOccupancy >= value);

		occupancySetLogit(node, occupancyLogit(value), propagate);
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	void occupancyUpdateLogit(NodeType node, int change, bool propagate = true)
	{
		assert(MaxLogit - MinLogit >= std::abs(change));

		occupancyUpdateLogit(
		    node,
		    [this, change](Index node) {
			    return static_cast<logit_t>(UFO_CLAMP(
			        static_cast<int>(occupancyBlock(node.pos)[node.offset].logit()) + change,
			        static_cast<int>(MinLogit), static_cast<int>(MaxLogit)));
		    },
		    propagate);
	}

	template <class NodeType, class UnaryOp,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool>        = true,
	          std::enable_if_t<std::is_invocable_r_v<logit_t, UnaryOp, Index>, bool> = true>
	void occupancyUpdateLogit(NodeType node, UnaryOp unary_op, bool propagate = true)
	{
		auto node_f = [this, unary_op](Index node) {
			logit_t value = unary_op(node);

			assert(MinLogit <= value && MaxLogit >= value);

			occupancyBlock(node.pos)[node.offset] =
			    OccupancyElement(value, occupancyUnknownLogit(value), occupancyFreeLogit(value),
			                     occupancyOccupiedLogit(value));
		};

		auto block_f = [this, node_f](pos_t pos) {
			for (std::size_t i{}; BF > i; ++i) {
				node_f(Index(pos, i));
			}
		};

		auto propagate_f = [this](Index node, pos_t children) {
			onPropagateChildren(node, children);
		};

		if constexpr (std::is_same_v<Index, std::decay_t<NodeType>>) {
			if (propagate) {
				derived().recursLeaves(node, node_f, block_f, propagate_f);
			} else {
				derived().recursLeaves(node, node_f, block_f);
			}
		} else {
			derived().recursLeaves(derived().code(node), node_f, block_f, propagate_f,
			                       propagate);
		}
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	void occupancyUpdate(NodeType node, occupancy_t change, bool propagate = true)
	{
		occupancyUpdateLogit(node, occupancyLogit(change), propagate);
	}

	template <
	    class NodeType, class UnaryOp,
	    std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool>            = true,
	    std::enable_if_t<std::is_invocable_r_v<occupancy_t, UnaryOp, Index>, bool> = true>
	void occupancyUpdate(NodeType node, UnaryOp unary_op, bool propagate = true)
	{
		occupancyUpdateLogit(
		    node, [this, unary_op](Index node) { return occupancyLogit(unary_op(node)); },
		    propagate);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Lookup                                        |
	|                                                                                     |
	**************************************************************************************/

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] OccupancyState occupancyState(NodeType node) const
	{
		// Avoid using `occupancyUnknown` here since it calls both `occupancyFree` and
		// `occupancyOccupied`
		auto occ = occupancyLogit(node);
		if (occupancyOccupiedLogit(occ)) {
			return OccupancyState::OCCUPIED;
		} else if (occupancyFreeLogit(occ)) {
			return OccupancyState::FREE;
		} else {
			return OccupancyState::UNKNOWN;
		}
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool occupancyContainsState(NodeType node, OccupancyState state) const
	{
		Index n = derived().index(node);
		switch (state) {
			case OccupancyState::UNKNOWN: return containsUnknown(n);
			case OccupancyState::FREE: return containsFree(n);
			case OccupancyState::OCCUPIED: return containsOccupied(n);
		}
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool occupancyUnknown(NodeType node) const
	{
		return occupancyUnknownLogit(occupancyLogit(node));
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool occupancyFree(NodeType node) const
	{
		return occupancyFreeLogit(occupancyLogit(node));
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool occupancyOccupied(NodeType node) const
	{
		return occupancyOccupiedLogit(occupancyLogit(node));
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool containsUnknown(NodeType node) const
	{
		Index n = derived().index(node);
		return static_cast<bool>(occupancyBlock(n.pos)[n.offset].unknown());
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool containsFree(NodeType node) const
	{
		Index n = derived().index(node);
		return static_cast<bool>(occupancyBlock(n.pos)[n.offset].free());
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool containsOccupied(NodeType node) const
	{
		Index n = derived().index(node);
		return static_cast<bool>(occupancyBlock(n.pos)[n.offset].occupied());
	}

	//
	// Sensor model
	//

	[[nodiscard]] constexpr occupancy_t occupancyClampingThres() const noexcept
	{
		return probability(occupancyClampingThresLogit());
	}

	[[nodiscard]] constexpr occupancy_t occupancyClampingThresLogit() const noexcept
	{
		return clamping_thres_logit_;
	}

	[[nodiscard]] constexpr occupancy_t occupancyMax() const noexcept
	{
		return occupancyClampingThres();
	}
	[[nodiscard]] constexpr occupancy_t occupancyMin() const noexcept
	{
		return -occupancyClampingThres();
	}

	[[nodiscard]] constexpr logit_t occupancyMaxLogit() const noexcept
	{
		return std::numeric_limits<logit_t>::max();
	}

	[[nodiscard]] constexpr logit_t occupancyMinLogit() const noexcept
	{
		return -occupancyMaxLogit();
	}

	[[nodiscard]] constexpr occupancy_t occupiedThres() const noexcept
	{
		return occupancy(occupiedThresLogit());
	}

	[[nodiscard]] constexpr logit_t occupiedThresLogit() const noexcept
	{
		return occupied_thres_logit_;
	}

	[[nodiscard]] constexpr occupancy_t freeThres() const noexcept
	{
		return occupancy(freeThresLogit());
	}

	[[nodiscard]] constexpr logit_t freeThresLogit() const noexcept
	{
		return free_thres_logit_;
	}

	//
	// Set sensor model
	//

	void occupancySetThres(occupancy_t occupied_thres, occupancy_t free_thres,
	                       bool propagate = true)
	{
		// FIXME: Should add a warning that these are very computational expensive to
		// call since the whole tree has to be updated

		occupancySetThresLogit(occupancyLogit(occupied_thres), occupancyLogit(free_thres),
		                       propagate);
	}

	// FIXME: Look at
	void occupancySetThresLogit(logit_t occupied_thres, logit_t free_thres,
	                            bool propagate = true)
	{
		// FIXME: Should add a warning that these are very computational expensive to
		// call since the whole tree has to be updated

		occupied_thres_logit_ = occupied_thres;
		free_thres_logit_     = free_thres;

		// TODO: Implement

		// derived().setModified();

		// if (propagate) {
		// 	derived().propagateModified();
		// }
	}

	void occupancySetClampingThres(occupancy_t probability)
	{
		clamping_thres_logit_ = logit(probability);
	}

	//
	// Propagation criteria
	//

	[[nodiscard]] constexpr OccupancyPropagationCriteria occupancyPropagationCriteria()
	    const noexcept
	{
		return prop_criteria_;
	}

	void occupancySetPropagationCriteria(OccupancyPropagationCriteria prop_criteria,
	                                     bool                         propagate = true)
	{
		if (OccupancyPropagationCriteria() == prop_criteria) {
			return;
		}

		prop_criteria_ = prop_criteria;

		// Set all inner nodes to modified
		// FIXME: Possible to optimize this to only set the ones with children
		derived().setModified();

		if (propagate) {
			derived().propagateModified();
		}
	}

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	OccupancyMap() { onInitRoot(); }

	OccupancyMap(OccupancyMap const& other) = default;

	OccupancyMap(OccupancyMap&& other) = default;

	template <class Derived2, class Tree2>
	OccupancyMap(OccupancyMap<Derived2, Tree2> const& other)
	    : clamping_thres_logit_(other.clamping_thres_logit_)
	    , occupied_thres_logit_(other.occupied_thres_logit_)
	    , free_thres_logit_(other.free_thres_logit_)
	    , prop_criteria_(other.prop_criteria_)
	{
	}

	template <class Derived2, class Tree2>
	OccupancyMap(OccupancyMap<Derived2, Tree2>&& other)
	    : clamping_thres_logit_(std::move(other.clamping_thres_logit_))
	    , occupied_thres_logit_(std::move(other.occupied_thres_logit_))
	    , free_thres_logit_(std::move(other.free_thres_logit_))
	    , prop_criteria_(std::move(other.prop_criteria_))
	{
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Destructor                                      |
	|                                                                                     |
	**************************************************************************************/

	~OccupancyMap() = default;

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	OccupancyMap& operator=(OccupancyMap const& rhs) = default;

	OccupancyMap& operator=(OccupancyMap&& rhs) = default;

	template <class Derived2, class Tree2>
	OccupancyMap& operator=(OccupancyMap<Derived2, Tree2> const& rhs)
	{
		clamping_thres_logit_ = rhs.clamping_thres_logit_;
		occupied_thres_logit_ = rhs.occupied_thres_logit_;
		free_thres_logit_     = rhs.free_thres_logit_;
		prop_criteria_        = rhs.prop_criteria_;
		return *this;
	}

	template <class Derived2, class Tree2>
	OccupancyMap& operator=(OccupancyMap<Derived2, Tree2>&& rhs)
	{
		clamping_thres_logit_ = std::move(rhs.clamping_thres_logit_);
		occupied_thres_logit_ = std::move(rhs.occupied_thres_logit_);
		free_thres_logit_     = std::move(rhs.free_thres_logit_);
		prop_criteria_        = std::move(rhs.prop_criteria_);
		return *this;
	}

	//
	// Swap
	//

	void swap(OccupancyMap& other) noexcept
	{
		std::swap(clamping_thres_logit_, other.clamping_thres_logit_);
		std::swap(occupied_thres_logit_, other.occupied_thres_logit_);
		std::swap(free_thres_logit_, other.free_thres_logit_);
		std::swap(prop_criteria_, other.prop_criteria_);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Derived                                       |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] constexpr Derived& derived() { return *static_cast<Derived*>(this); }

	[[nodiscard]] constexpr Derived const& derived() const
	{
		return *static_cast<Derived const*>(this);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Block                                        |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] OccupancyBlock<BF>& occupancyBlock(pos_t pos)
	{
		return derived().template data<OccupancyBlock<BF>>(pos);
	}

	[[nodiscard]] OccupancyBlock<BF> const& occupancyBlock(pos_t pos) const
	{
		return derived().template data<OccupancyBlock<BF>>(pos);
	}

	/**************************************************************************************
	|                                                                                     |
	|                              Functions Derived expect                               |
	|                                                                                     |
	**************************************************************************************/

	void onInitRoot()
	{
		auto& block = occupancyBlock(0);

		logit_t value{};
		block[0] = OccupancyElement(value, occupancyUnknownLogit(value),
		                            occupancyFreeLogit(value), occupancyOccupiedLogit(value));
	}

	void onInitChildren(Index node, pos_t children)
	{
		occupancyBlock(children).fill(occupancyBlock(node.pos)[node.offset]);
	}

	void onPruneChildren(Index node, pos_t children)
	{
		// Ensure that the indicators of `node` are correct
		auto&   v     = occupancyBlock(node.pos)[node.offset];
		logit_t value = v.logit();
		v = OccupancyElement(value, occupancyUnknownLogit(value), occupancyFreeLogit(value),
		                     occupancyOccupiedLogit(value));
	}

	void onPropagateChildren(Index node, pos_t children)
	{
		auto const& children_block = occupancyBlock(children);

		std::uint32_t indicators{};
		std::uint32_t value{};
		switch (prop_criteria_) {
			case OccupancyPropagationCriteria::MAX:
				for (auto child : children_block.data) {
					indicators |= child.indicators();
					value = UFO_MAX(value, child.value());
				}
				break;
			case OccupancyPropagationCriteria::MIN:
				value = std::numeric_limits<std::uint32_t>::max();
				for (auto child : children_block.data) {
					indicators |= child.indicators();
					value = UFO_MIN(value, child.value());
				}
				break;
			case OccupancyPropagationCriteria::MEAN: {
				for (auto child : children_block.data) {
					indicators |= child.indicators();
					value += child.value();
				}
				value /= BF;
				break;
			}
			case OccupancyPropagationCriteria::ONLY_INDICATORS: return;
			case OccupancyPropagationCriteria::NONE: return;
		}

		occupancyBlock(node.pos)[node.offset] = OccupancyElement(value, indicators);
	}

	//
	// Is prunable
	//

	[[nodiscard]] bool prunable(pos_t block) const
	{
		using std::begin;
		using std::end;
		return std::all_of(
		    begin(occupancyBlock(block).data) + 1, end(occupancyBlock(block).data),
		    [v = occupancyBlock(block)[0].value_and_indicators](auto const& e) {
			    return v == e.value_and_indicators;
		    });
	}

	//
	// Is
	//

	[[nodiscard]] constexpr bool occupancyUnknownLogit(logit_t logit) const noexcept
	{
		return !occupancyFreeLogit(logit) && !occupancyOccupiedLogit(logit);
	}

	[[nodiscard]] constexpr bool occupancyFreeLogit(logit_t logit) const noexcept
	{
		return freeThresLogit() > logit;
	}

	[[nodiscard]] constexpr bool occupancyOccupiedLogit(logit_t logit) const noexcept
	{
		return occupiedThresLogit() < logit;
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr std::size_t serializedSizeNode() noexcept
	{
		return sizeof(logit_t);
	}

	[[nodiscard]] constexpr std::size_t serializedSize(
	    std::vector<std::pair<pos_t, BitSet<BF>>> const& /* nodes */,
	    std::size_t num_nodes) const
	{
		return num_nodes * serializedSizeNode();
	}

	void onRead(ReadBuffer& in, std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes)
	{
		for (auto [block, offset] : nodes) {
			auto& occ = occupancyBlock(block);

			if (offset.all()) {
				std::array<logit_t, BF> logit;
				in.read(logit);
				for (offset_t i{}; BF > i; ++i) {
					auto u = occupancyUnknownLogit(logit[i]);
					auto f = occupancyFreeLogit(logit[i]);
					auto o = occupancyOccupiedLogit(logit[i]);
					occ[i] = OccupancyElement(logit[i], u, f, o);
				}
			} else {
				for (offset_t i{}; BF > i; ++i) {
					if (offset[i]) {
						logit_t value;
						in.read(value);
						auto u = occupancyUnknownLogit(value);
						auto f = occupancyFreeLogit(value);
						auto o = occupancyOccupiedLogit(value);
						occ[i] = OccupancyElement(value, u, f, o);
					}
				}
			}
		}
	}

	void onWrite(WriteBuffer& out, std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes)
	{
		for (auto [block, offset] : nodes) {
			auto const& occ = occupancyBlock(block);

			if (offset.all()) {
				std::array<logit_t, BF> logit;
				for (offset_t i{}; BF > i; ++i) {
					logit[i] = occ[i].logit();
				}
				out.write(logit);
			} else {
				for (offset_t i{}; BF > i; ++i) {
					if (offset[i]) {
						out.write(occ[i].logit());
					}
				}
			}
		}
	}

	//
	// Dot file info
	//

	void onDotFileInfo(std::ostream& out, Index node) const
	{
		out << "Occupancy: " << occupancy(node);
	}

	//
	// Probability functions
	//

	[[nodiscard]] static occupancy_t probability(occupancy_t logit)
	{
		return occupancy_t(1) / (occupancy_t(1) + std::exp(-logit));
	}

	//
	// Logit functions
	//

	[[nodiscard]] static occupancy_t logit(occupancy_t probability)
	{
		return std::log(probability / (occupancy_t(1) - probability));
	}

	[[nodiscard]] static occupancy_t logit(occupancy_t probability, occupancy_t min,
	                                       occupancy_t max)
	{
		return std::clamp(logit(probability), min, max);
	}

 private:
	occupancy_t clamping_thres_logit_ = logit(0.971);

	logit_t occupied_thres_logit_{};  // Threshold for occupied
	logit_t free_thres_logit_{};      // Threshold for free

	// TODO: Maybe add lookup tables?

	// Propagation criteria
	OccupancyPropagationCriteria prop_criteria_ = OccupancyPropagationCriteria::MAX;
};

template <std::size_t Dim, std::size_t BF>
struct map_block<OccupancyMap, Dim, BF> {
	using type = OccupancyBlock<BF>;
};
}  // namespace ufo

#endif  // UFO_MAP_OCCUPANCY_MAP_HPP