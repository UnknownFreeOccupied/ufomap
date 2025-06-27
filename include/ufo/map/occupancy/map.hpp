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
#include <ufo/map/block.hpp>
#include <ufo/map/occupancy/block.hpp>
#include <ufo/map/occupancy/predicate.hpp>
#include <ufo/map/occupancy/propagation_criteria.hpp>
#include <ufo/map/occupancy/state.hpp>
#include <ufo/map/type.hpp>
#include <ufo/math/transform3.hpp>
#include <ufo/utility/bit_set.hpp>
#include <ufo/utility/io/buffer.hpp>
#include <ufo/utility/macros.hpp>
#include <ufo/utility/type_traits.hpp>

// STL
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

	using logit_t     = typename Block::logit_t;
	using occupancy_t = logit_t;

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                       Access                                        |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] logit_t occupancyLogit() const
	{
		return occupancyLogit(derived().index());
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] logit_t occupancyLogit(NodeType node) const
	{
		Index n = derived().index(node);
		return occupancyBlock(n.pos)[n.offset].logit;
	}

	[[nodiscard]] occupancy_t occupancy() const { return occupancy(derived().index()); }

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] occupancy_t occupancy(NodeType node) const
	{
		return occupancy(occupancyLogit(node));
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
		OccupancyElement elem(value, occupancyUnknownLogit(value), occupancyFreeLogit(value),
		                      occupancyOccupiedLogit(value));

		auto node_f = [this, elem](Index node) {
			occupancyBlock(node.pos)[node.offset] = elem;
		};

		auto block_f = [this, elem](pos_t pos) { occupancyBlock(pos) = elem; };

		auto update_f = [this](Index node, pos_t children) {
			onPropagateChildren(node, children);
		};

		derived().recursParentFirst(node, node_f, block_f, update_f, propagate);
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	void occupancySet(NodeType node, occupancy_t value, bool propagate = true)
	{
		assert(0 <= value && 1 >= value);

		occupancySetLogit(node, occupancyLogit(value), propagate);
	}

	template <class NodeType, class UnaryOp,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool>        = true,
	          std::enable_if_t<std::is_invocable_r_v<logit_t, UnaryOp, Index>, bool> = true>
	void occupancyUpdateLogit(NodeType node, UnaryOp&& unary_op, bool propagate = true)
	{
		auto node_f = [this, &unary_op](Index node) {
			logit_t value = unary_op(node);
			occupancyBlock(node.pos)[node.offset] =
			    OccupancyElement(value, occupancyUnknownLogit(value), occupancyFreeLogit(value),
			                     occupancyOccupiedLogit(value));
		};

		auto block_f = [this, node_f](pos_t pos) {
			for (std::size_t i{}; BF > i; ++i) {
				node_f(Index(pos, i));
			}
		};

		auto update_f = [this](Index node, pos_t children) {
			onPropagateChildren(node, children);
		};

		derived().recursParentFirst(node, node_f, block_f, update_f, propagate);
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	void occupancyUpdateLogit(NodeType node, logit_t change, bool propagate = true)
	{
		occupancyUpdateLogit(
		    node, [this, change](Index node) { return occupancyLogit(node) + change; },
		    propagate);
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	void occupancyUpdateLogit(NodeType node, logit_t change, logit_t min_clamp_thres,
	                          logit_t max_clamp_thres, bool propagate = true)
	{
		occupancyUpdateLogit(
		    node,
		    [this, change, min_clamp_thres, max_clamp_thres](Index node) {
			    return std::clamp(occupancyLogit(node) + change, min_clamp_thres,
			                      max_clamp_thres);
		    },
		    propagate);
	}

	template <
	    class NodeType, class UnaryOp,
	    std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool>            = true,
	    std::enable_if_t<std::is_invocable_r_v<occupancy_t, UnaryOp, Index>, bool> = true>
	void occupancyUpdate(NodeType node, UnaryOp&& unary_op, bool propagate = true)
	{
		occupancyUpdateLogit(
		    node, [this, &unary_op](Index node) { return occupancyLogit(unary_op(node)); },
		    propagate);
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	void occupancyUpdate(NodeType node, occupancy_t change, bool propagate = true)
	{
		occupancyUpdateLogit(node, occupancyLogit(change), propagate);
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	void occupancyUpdate(NodeType node, occupancy_t change, occupancy_t min_clamp_thres,
	                     occupancy_t max_clamp_thres, bool propagate = true)
	{
		occupancyUpdateLogit(node, occupancyLogit(change), occupancyLogit(min_clamp_thres),
		                     occupancyLogit(max_clamp_thres), propagate);
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
		return occupancyBlock(n.pos)[n.offset].contains_unknown;
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool containsFree(NodeType node) const
	{
		Index n = derived().index(node);
		return occupancyBlock(n.pos)[n.offset].contains_free;
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool containsOccupied(NodeType node) const
	{
		Index n = derived().index(node);
		return occupancyBlock(n.pos)[n.offset].contains_occupied;
	}

	//
	// Sensor model
	//

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
	    : occupied_thres_logit_(other.occupied_thres_logit_)
	    , free_thres_logit_(other.free_thres_logit_)
	    , prop_criteria_(other.prop_criteria_)
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
		occupied_thres_logit_ = rhs.occupied_thres_logit_;
		free_thres_logit_     = rhs.free_thres_logit_;
		prop_criteria_        = rhs.prop_criteria_;
		return *this;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Swap                                         |
	|                                                                                     |
	**************************************************************************************/

	void swap(OccupancyMap& other) noexcept
	{
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
	|                              Functions Derived expects                              |
	|                                                                                     |
	**************************************************************************************/

	void onInitRoot()
	{
		logit_t value{};
		occupancyBlock(0) =
		    OccupancyElement(value, occupancyUnknownLogit(value), occupancyFreeLogit(value),
		                     occupancyOccupiedLogit(value));
	}

	void onInitChildren(Index node, pos_t children)
	{
		occupancyBlock(children) = occupancyBlock(node.pos)[node.offset];
	}

	void onPropagateChildren(Index node, pos_t children)
	{
		auto&       o   = occupancyBlock(node.pos)[node.offset];
		auto const& ocb = occupancyBlock(children);

		o.contains_unknown  = false;
		o.contains_free     = false;
		o.contains_occupied = false;
		switch (prop_criteria_) {
			case OccupancyPropagationCriteria::MAX:
				o.logit = std::numeric_limits<logit_t>::lowest();
				for (auto child : ocb.data) {
					o.logit             = UFO_MAX(o.logit, child.logit);
					o.contains_unknown  = o.contains_unknown || child.contains_unknown;
					o.contains_free     = o.contains_free || child.contains_free;
					o.contains_occupied = o.contains_occupied || child.contains_occupied;
				}
				break;
			case OccupancyPropagationCriteria::MIN:
				o.logit = std::numeric_limits<logit_t>::max();
				for (auto child : ocb.data) {
					o.logit             = UFO_MIN(o.logit, child.logit);
					o.contains_unknown  = o.contains_unknown || child.contains_unknown;
					o.contains_free     = o.contains_free || child.contains_free;
					o.contains_occupied = o.contains_occupied || child.contains_occupied;
				}
				break;
			case OccupancyPropagationCriteria::MEAN: {
				o.logit = {};
				for (auto child : ocb.data) {
					o.logit += child.logit;
					o.contains_unknown  = o.contains_unknown || child.contains_unknown;
					o.contains_free     = o.contains_free || child.contains_free;
					o.contains_occupied = o.contains_occupied || child.contains_occupied;
				}
				o.logit /= BF;
				break;
			}
			case OccupancyPropagationCriteria::ONLY_INDICATORS: {
				for (auto child : ocb.data) {
					o.contains_unknown  = o.contains_unknown || child.contains_unknown;
					o.contains_free     = o.contains_free || child.contains_free;
					o.contains_occupied = o.contains_occupied || child.contains_occupied;
				}
				break;
			}
			case OccupancyPropagationCriteria::NONE: return;
		}
	}

	[[nodiscard]] bool onIsPrunable(pos_t block) const
	{
		return std::all_of(occupancyBlock(block).begin() + 1, occupancyBlock(block).end(),
		                   [v = occupancyBlock(block)[0]](auto const& e) { return v == e; });
	}

	void onPruneChildren(Index node, pos_t /* children */)
	{
		// Ensure that the indicators of `node` are correct
		auto&   v     = occupancyBlock(node.pos)[node.offset];
		logit_t value = v.logit;
		v = OccupancyElement(value, occupancyUnknownLogit(value), occupancyFreeLogit(value),
		                     occupancyOccupiedLogit(value));
	}

	[[nodiscard]] std::size_t onSerializedSize(
	    std::vector<std::pair<pos_t, BitSet<BF>>> const& /* nodes */,
	    std::size_t num_nodes) const
	{
		return num_nodes * sizeof(OccupancyElement);
	}

	void onRead(ReadBuffer& in, std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes)
	{
		for (auto [block, offset] : nodes) {
			auto& ob = occupancyBlock(block);

			if (offset.all()) {
				for (std::size_t i{}; BF > i; ++i) {
					logit_t l;
					in.read(l);
					ob[i] = OccupancyElement(l, occupancyUnknownLogit(l), occupancyFreeLogit(l),
					                         occupancyOccupiedLogit(l));
				}
			} else {
				for (std::size_t i{}; BF > i; ++i) {
					if (offset[i]) {
						logit_t l;
						in.read(l);
						ob[i] = OccupancyElement(l, occupancyUnknownLogit(l), occupancyFreeLogit(l),
						                         occupancyOccupiedLogit(l));
					}
				}
			}
		}
	}

	void onWrite(WriteBuffer&                                     out,
	             std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes) const
	{
		for (auto [block, offset] : nodes) {
			auto const& ob = occupancyBlock(block);

			if (offset.all()) {
				for (std::size_t i{}; BF > i; ++i) {
					out.write(ob[i].logit);
				}
			} else {
				for (std::size_t i{}; BF > i; ++i) {
					if (offset[i]) {
						out.write(ob[i].logit);
					}
				}
			}
		}
	}

	void onDotFile(std::ostream& out, Index node) const
	{
		out << "Occupancy: " << occupancy(node);
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

 private:
	logit_t occupied_thres_logit_{};  // Threshold for occupied
	logit_t free_thres_logit_{};      // Threshold for free

	// Propagation criteria
	OccupancyPropagationCriteria prop_criteria_ = OccupancyPropagationCriteria::MAX;
};

template <std::size_t Dim, std::size_t BF>
struct map_block<OccupancyMap, Dim, BF> {
	using type = OccupancyBlock<BF>;
};
}  // namespace ufo

#endif  // UFO_MAP_OCCUPANCY_MAP_HPP