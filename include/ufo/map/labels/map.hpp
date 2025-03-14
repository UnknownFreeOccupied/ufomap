/*!
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author Daniel Duberg (dduberg@kth.se), Ramona Häuselmann (ramonaha@kth.se)
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

#ifndef UFO_MAP_LABELS_MAP_HPP
#define UFO_MAP_LABELS_MAP_HPP

// UFO
#include <functional>
#include <numeric>
#include <ufo/container/tree/container.hpp>
#include <ufo/container/tree/tree.hpp>
#include <ufo/map/block.hpp>
#include <ufo/map/labels/block.hpp>
#include <ufo/map/labels/predicate.hpp>
#include <ufo/map/labels/propagation_criteria.hpp>
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

#include "ufo/core/label.hpp"

namespace ufo
{
template <class Derived, class Tree>
class LabelsMap
{
 private:
	template <class Derived2, class Tree2>
	friend class LabelsMap;

	static constexpr auto const BF  = Tree::branchingFactor();
	static constexpr auto const Dim = Tree::dimensions();

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                        Tags                                         |
	|                                                                                     |
	**************************************************************************************/

	static constexpr MapType const Type = MapType::LABEL_SET;

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

	// Labels
	using labels_t = typename LabelsBlock<BF>::labels_t;

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                       Access                                        |
	|                                                                                     |
	**************************************************************************************/

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] labels_t labels(NodeType node) const
	{
		Index n = derived().index(node);
		return labelsBlock(n.pos)[n.offset].labels;
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
	void labelsSet(NodeType node, labels_t value, bool propagate = true)
	{
		LabelsElement elem(value);

		auto node_f = [this, elem](Index node) { labelsBlock(node.pos)[node.offset] = elem; };

		auto block_f = [this, elem](pos_t pos) { labelsBlock(pos).fill(elem); };

		auto update_f = [this](Index node, pos_t children) {
			onPropagateChildren(node, children);
		};

		derived().recursParentFirst(node, node_f, block_f, update_f, propagate);
	}

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	void labelsUpdate(NodeType node, label_t value, bool propagate = true)
	{
		labelsUpdate(
		    node,
		    [this, value](Index node) {
			    labelsBlock(node.pos)[node.offset].labels.insert(value);
			    return labelsBlock(node.pos)[node.offset].labels;
		    },
		    propagate);
	}

	template <
	    class NodeType, class UnaryOp,
	    std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool>         = true,
	    std::enable_if_t<std::is_invocable_r_v<labels_t, UnaryOp, Index>, bool> = true>
	void labelsUpdate(NodeType node, UnaryOp unary_op, bool propagate = true)
	{
		auto node_f = [this, unary_op](Index node) {
			labels_t value = unary_op(node);

			auto& lb  = labelsBlock(node.pos)[node.offset];
			lb.labels = value;
		};

		auto block_f = [this, node_f](pos_t pos) {
			for (std::size_t i{}; BF > i; ++i) {
				node_f(Index(pos, i));
			}
		};

		auto update_f = [this](Index node, pos_t children) {
			onPropagateChildren(node, children);
		};

		derived().recursLeaves(node, node_f, block_f, update_f, propagate);
	}

	//
	// Propagation criteria
	//

	[[nodiscard]] constexpr LabelsPropagationCriteria labelsPropagationCriteria()
	    const noexcept
	{
		return prop_criteria_;
	}

	void labelsSetPropagationCriteria(LabelsPropagationCriteria prop_criteria,
	                                  bool                      propagate = true)
	{
		if (LabelsPropagationCriteria() == prop_criteria) {
			return;
		}

		prop_criteria_ = prop_criteria;

		// Set all inner nodes to modified
		// FIXME: Possible to optimize this to only set the ones with children
		// TODO: Enable below
		// derived().setModified();

		// if (propagate) {
		// 	derived().propagateModified();
		// }
	}

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	LabelsMap() { onInitRoot(); }

	LabelsMap(LabelsMap const& other) = default;

	LabelsMap(LabelsMap&& other) = default;

	template <class Derived2, class Tree2>
	LabelsMap(LabelsMap<Derived2, Tree2> const& other)
	    : prop_criteria_(other.prop_criteria_)
	{
	}

	template <class Derived2, class Tree2>
	LabelsMap(LabelsMap<Derived2, Tree2>&& other)
	    : prop_criteria_(std::move(other.prop_criteria_))
	{
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Destructor                                      |
	|                                                                                     |
	**************************************************************************************/

	~LabelsMap() = default;

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	LabelsMap& operator=(LabelsMap const& rhs) = default;

	LabelsMap& operator=(LabelsMap&& rhs) = default;

	template <class Derived2, class Tree2>
	LabelsMap& operator=(LabelsMap<Derived2, Tree2> const& rhs)
	{
		prop_criteria_ = rhs.prop_criteria_;
		return *this;
	}

	template <class Derived2, class Tree2>
	LabelsMap& operator=(LabelsMap<Derived2, Tree2>&& rhs)
	{
		prop_criteria_ = std::move(rhs.prop_criteria_);
		return *this;
	}

	//
	// Swap
	//

	void swap(LabelsMap& other) noexcept
	{
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

	[[nodiscard]] LabelsBlock<BF>& labelsBlock(pos_t pos)
	{
		return derived().template data<LabelsBlock<BF>>(pos);
	}

	[[nodiscard]] LabelsBlock<BF> const& labelsBlock(pos_t pos) const
	{
		return derived().template data<LabelsBlock<BF>>(pos);
	}

	/**************************************************************************************
	|                                                                                     |
	|                              Functions Derived expect                               |
	|                                                                                     |
	**************************************************************************************/

	void onInitRoot()
	{
		auto& block = labelsBlock(0);

		labels_t value{};
		block[0].labels = value;
	}

	void onInitChildren(Index node, pos_t children)
	{
		labelsBlock(children).fill(labelsBlock(node.pos)[node.offset]);
	}

	void onPruneChildren(Index node, pos_t children)
	{
		// TODO: implement
		// auto& v = labelsBlock(node.pos)[node.offset];
	}

	void onPropagateChildren(Index node, pos_t children)
	{
		auto const& children_block = labelsBlock(children);
		auto&       lb             = labelsBlock(node.pos)[node.offset];

		switch (prop_criteria_) {
			case LabelsPropagationCriteria::ALL: {
				for (std::size_t i{}; BF > i; ++i) {
					lb.labels.insert(children_block[i].labels.begin(),
					                 children_block[i].labels.end());
				}
				break;
			}
			case LabelsPropagationCriteria::SUMMARY: {
				label_t value{};
				for (std::size_t i{}; BF > i; ++i) {
					value |=
					    std::accumulate(children_block[i].labels.begin(),
					                    children_block[i].labels.end(), 0, std::bit_or<label_t>());
				}
				lb.labels = {value};
				break;
			}
			case LabelsPropagationCriteria::MAX: {
				label_t value = std::numeric_limits<label_t>::lowest();
				for (std::size_t i{}; BF > i; ++i) {
					value = UFO_MAX(value, children_block[i].labels.empty()
					                           ? std::numeric_limits<label_t>::lowest()
					                           : *children_block[i].labels.rbegin());
				}
				lb.labels = {value};
				break;
			}
			case LabelsPropagationCriteria::MIN: {
				label_t value = std::numeric_limits<label_t>::max();
				for (std::size_t i{}; BF > i; ++i) {
					value = UFO_MIN(value, children_block[i].labels.empty()
					                           ? std::numeric_limits<label_t>::max()
					                           : *children_block[i].labels.rbegin());
				}
				lb.labels = {value};
				break;
			}
			case LabelsPropagationCriteria::NONE: return;
		}
	}

	//
	// Is prunable
	//

	[[nodiscard]] bool onIsPrunable(pos_t block) const
	{
		using std::begin;
		using std::end;
		return std::all_of(
		    begin(labelsBlock(block).data) + 1, end(labelsBlock(block).data),
		    [v = labelsBlock(block)[0].labels](auto const& e) { return v == e.labels; });
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr std::size_t serializedSizeNode() noexcept
	{
		return sizeof(labels_t);
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
			auto& cb = labelsBlock(block);

			if (offset.all()) {
				std::array<labels_t, BF> labels;
				in.read(labels);
				for (offset_t i{}; BF > i; ++i) {
					cb[i] = LabelsElement(labels[i]);
				}
			} else {
				for (offset_t i{}; BF > i; ++i) {
					if (offset[i]) {
						labels_t value;
						in.read(value);
						cb[i] = LabelsElement(value);
					}
				}
			}
		}
	}

	void onWrite(WriteBuffer& out, std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes)
	{
		for (auto [block, offset] : nodes) {
			auto const& lb = labelsBlock(block);

			if (offset.all()) {
				std::array<labels_t, BF> labels;
				for (offset_t i{}; BF > i; ++i) {
					labels[i] = lb[i].labels;
				}
				out.write(labels);
			} else {
				for (offset_t i{}; BF > i; ++i) {
					if (offset[i]) {
						out.write(lb[i].labels);
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
		out << "Labels: ";

		if (labelsBlock(node.pos)[node.offset].labels.empty()) return;

		auto it = labelsBlock(node.pos)[node.offset].labels.begin();
		out << *it++;

		while (it != labelsBlock(node.pos)[node.offset].labels.end()) {
			out << ',' << *it++;
		}
	}

 private:
	// Propagation criteria
	LabelsPropagationCriteria prop_criteria_ = LabelsPropagationCriteria::ALL;
};

template <std::size_t Dim, std::size_t BF>
struct map_block<LabelsMap, Dim, BF> {
	using type = LabelsBlock<BF>;
};
}  // namespace ufo

#endif  // UFO_MAP_LABELS_MAP_HPP