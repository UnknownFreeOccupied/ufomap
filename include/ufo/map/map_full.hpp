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

#ifndef UFO_MAP_FULL_HPP
#define UFO_MAP_FULL_HPP

// UFO
#include <ufo/container/tree/block.hpp>
#include <ufo/container/tree/predicate.hpp>
#include <ufo/container/tree/tree.hpp>
#include <ufo/map/block.hpp>
#include <ufo/map/header.hpp>
#include <ufo/map/type.hpp>
#include <ufo/map/utility.hpp>
#include <ufo/utility/enum.hpp>
#include <ufo/utility/io/buffer.hpp>
#include <ufo/utility/macros.hpp>

// STD
#include <algorithm>
#include <array>
#include <bitset>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <ios>
#include <limits>
#include <tuple>
#include <type_traits>

namespace ufo
{
// All your base are belong to us
template <std::size_t Dim, MapUtility Utility, template <class, class> class... Maps>
class MapFull final
    : public Tree<
          MapFull<Dim, Utility, Maps...>, Dim, true,
          TreeBlock<Dim, std::size_t(1) << Dim,
                    MapUtility::WITH_CENTER == (MapUtility::WITH_CENTER & Utility)>,
          map_block_t<Maps, Dim>...>
    , public Maps<
          MapFull<Dim, Utility, Maps...>,
          Tree<MapFull<Dim, Utility, Maps...>, Dim, true,
               TreeBlock<Dim, std::size_t(1) << Dim,
                         MapUtility::WITH_CENTER == (MapUtility::WITH_CENTER & Utility)>,
               map_block_t<Maps, Dim>...>>...
{
	using Base =
	    Tree<MapFull<Dim, Utility, Maps...>, Dim, true,
	         TreeBlock<Dim, std::size_t(1) << Dim,
	                   MapUtility::WITH_CENTER == (MapUtility::WITH_CENTER & Utility)>,
	         map_block_t<Maps, Dim>...>;

	//
	// Friends
	//

	template <std::size_t Dim2, MapUtility Utility2, template <class, class> class... Maps2>
	friend class MapFull;

	// First base friends :)
	friend Base;

#define UFO_MAP_FULL_FRIEND(F)                                          \
	friend std::tuple_element_t<std::min(static_cast<std::size_t>(F + 1), \
	                                     sizeof...(Maps)),                \
	                            std::tuple<void, Maps<MapFull, Base>...>>;
	UFO_REPEAT_128(UFO_MAP_FULL_FRIEND, 0)

	static constexpr auto const BF = Base::branchingFactor();

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                        Tags                                         |
	|                                                                                     |
	**************************************************************************************/

	using Index    = typename Base::Index;
	using Node     = typename Base::Node;
	using Code     = typename Base::Code;
	using Key      = typename Base::Key;
	using Point    = typename Base::Point;
	using Coord    = typename Base::Coord;
	using Bounds   = typename Base::Bounds;
	using Length   = typename Base::Length;
	using coord_t  = typename Base::coord_t;
	using depth_t  = typename Base::depth_t;
	using offset_t = typename Base::offset_t;
	using length_t = typename Base::length_t;
	using pos_t    = typename Base::pos_t;

	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	MapFull(Length  leaf_node_length = Length(0.1),
	        depth_t num_depth_levels = std::min(depth_t(17), Base::maxNumDepthLevels()))
	    : Base(leaf_node_length, num_depth_levels)
	{
	}

	MapFull(length_t leaf_node_length,
	        depth_t  num_depth_levels = std::min(depth_t(17), Base::maxNumDepthLevels()))
	    : MapFull(Length(leaf_node_length), num_depth_levels)
	{
	}

	MapFull(std::filesystem::path const& file) : MapFull() { read(file); }

	MapFull(std::istream& in) : MapFull() { read(in); }

	MapFull(ReadBuffer& in) : MapFull() { read(in); }

	MapFull(MapFull const&) = default;

	MapFull(MapFull&&) = default;

	template <MapUtility Utility2, template <class, class> class... Maps2>
	MapFull(MapFull<Dim, Utility2, Maps2...> const& other)
	    : Base(static_cast<Base const&>(other))
	{
		// TODO: Implement
		// std::size_t size = Base::numBlocks();
		// (init<Maps<MapFull, Base>>(other, size), ...);
	}

	template <MapUtility Utility2, template <class, class> class... Maps2>
	MapFull(MapFull<Dim, Utility2, Maps2...>&& other) : Base(static_cast<Base&&>(other))
	{
		// TODO: Implement
		// std::size_t size = Base::numBlocks();
		// (init<Maps<MapFull, Base>>(std::move(other), size), ...);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Destructor                                      |
	|                                                                                     |
	**************************************************************************************/

	~MapFull() = default;

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	MapFull& operator=(MapFull const&) = default;

	MapFull& operator=(MapFull&&) = default;

	template <MapUtility Utility2, template <class, class> class... Maps2>
	MapFull& operator=(MapFull<Dim, Utility2, Maps2...> const& rhs)
	{
		Base::operator=(static_cast<Base const&>(rhs));

		// TODO: Implement
		// std::size_t size = Base::numBlocks();
		// (init<Maps<MapFull, Base>>(rhs, size), ...);

		return *this;
	}

	template <MapUtility Utility2, template <class, class> class... Maps2>
	MapFull& operator=(MapFull<Dim, Utility2, Maps2...>&& rhs)
	{
		Base::operator=(static_cast<Base&&>(rhs));

		// TODO: Implement
		// std::size_t size = Base::numBlocks();
		// (init<Maps<MapFull, Base>>(std::move(rhs), size), ...);

		return *this;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                         Swap                                        |
	|                                                                                     |
	**************************************************************************************/

	friend void swap(MapFull& lhs, MapFull& rhs)
	{
		static_cast<Base&>(lhs).swap(static_cast<Base&>(rhs));
		(static_cast<Maps<MapFull, Base>&>(lhs).swap(rhs), ...);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Map Types                                      |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] static constexpr MapType mapTypes() noexcept
	{
		return (mapType<Maps<MapFull, Base>>() | ...);
	}

	[[nodiscard]] static std::size_t numMapTypes() noexcept
	{
		auto mt = to_underlying(mapTypes());
		return std::bitset<std::numeric_limits<decltype(mt)>::digits>(mt).count();
	}

	[[nodiscard]] static constexpr bool hasMapTypes(MapType map_types) noexcept
	{
		return (mapTypes() & map_types) == map_types;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Propagate                                      |
	|                                                                                     |
	**************************************************************************************/

	void propagate(bool prune = true, MapType map_types = MapType::ALL)
	{
		propagate(Base::index(), prune, map_types);
	}

	void propagate(pos_t block, bool prune = true, MapType map_types = MapType::ALL)
	{
		assert(Base::valid(block));

		for (std::size_t i{}; BF > i; ++i) {
			auto n = Index(block, i);
			if (Base::isLeaf(n)) {
				continue;
			}

			auto c = Base::children(n);

			propagate(c, prune, map_types);
			onPropagateChildren(n, c, map_types);

			if (prune && onIsPrunable(c)) {
				Base::pruneChildren(n, c);
			}
		}
	}

	template <class NodeType,
	          std::enable_if_t<Base::template is_node_type_v<NodeType>, bool> = true>
	void propagate(NodeType node, bool prune = true, MapType map_types = MapType::ALL)
	{
		assert(Base::valid(node));

		auto n = Base::index(node);

		if (Base::isLeaf(n)) {
			return;
		}

		auto c = Base::children(n);

		propagate(c, prune, map_types);
		onPropagateChildren(n, c, map_types);

		if (prune && onIsPrunable(c)) {
			Base::pruneChildren(n, c);
		}
	}

	template <
	    class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void propagate(ExecutionPolicy&& policy, bool prune = true,
	               MapType map_types = MapType::ALL)
	{
		// TODO: Optimize
		propagate(prune, map_types);
	}

	/*!
	 * @brief Propagate modified information up the tree.
	 *
	 * @param prune Whether the tree should be pruned also
	 * @param reset_modified Whether propagated node's modified state should be reset
	 */
	void modifiedPropagate(bool prune = true, bool reset_modified = true,
	                       MapType map_types = MapType::ALL)
	{
		modifiedPropagate(Base::index(), prune, reset_modified, map_types);
	}

	void modifiedPropagate(pos_t block, bool prune = true, bool reset_modified = true,
	                       MapType map_types = MapType::ALL)
	{
		assert(Base::valid(block));

		auto m = reset_modified ? Base::treeBlock(block).modifiedExchange(0)
		                        : Base::treeBlock(block).modified();

		if (0 == m) {
			return;
		}

		for (std::size_t i{}; BF > i; ++i) {
			auto n = Index(block, i);
			if (0u == (m & (1u << i)) || Base::isLeaf(n)) {
				continue;
			}

			auto c = Base::children(n);

			modifiedPropagate(c, prune, reset_modified, map_types);
			onPropagateChildren(n, c, map_types);

			if (prune && onIsPrunable(c)) {
				Base::pruneChildren(n, c);
			}
		}
	}

	template <class NodeType,
	          std::enable_if_t<Base::template is_node_type_v<NodeType>, bool> = true>
	void modifiedPropagate(NodeType node, bool prune = true, bool reset_modified = true,
	                       MapType map_types = MapType::ALL)
	{
		assert(Base::valid(node));

		auto n = Base::index(node);

		if (!Base::modified(n)) {
			return;
		}

		if (reset_modified) {
			Base::treeBlock(n.pos).modifiedReset(n.offset);
		}

		if (Base::isLeaf(n)) {
			return;
		}

		auto c = Base::children(n);

		modifiedPropagate(c, prune, reset_modified, map_types);
		onPropagateChildren(n, c, map_types);

		if (prune && onIsPrunable(c)) {
			Base::pruneChildren(n, c);
		}
	}

	template <
	    class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void modifiedPropagate(ExecutionPolicy&& policy, bool prune = true,
	                       bool reset_modified = true, MapType map_types = MapType::ALL)
	{
		modifiedPropagate(std::forward<ExecutionPolicy>(policy), Base::index(), prune,
		                  reset_modified, map_types);
	}

	template <
	    class ExecutionPolicy, class NodeType,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true,
	    std::enable_if_t<Base::template is_node_type_v<NodeType>, bool>           = true>
	void modifiedPropagate(ExecutionPolicy&& policy, NodeType node, bool prune = true,
	                       bool reset_modified = true, MapType map_types = MapType::ALL)
	{
		// TODO: Optimize
		modifiedPropagate(node, prune, reset_modified, map_types);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                         I/O                                         |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] MapHeader header(MapType map_types = MapType::ALL) const
	{
		return MapHeader(Base::length(0), Base::numDepthLevels(), mapTypes() & map_types);
	}

	[[nodiscard]] static bool isMap(std::filesystem::path const& file)
	{
		return MapHeader::isMap(file);
	}

	[[nodiscard]] static bool isMap(std::istream& in) { return MapHeader::isMap(in); }

	[[nodiscard]] static bool isMap(ReadBuffer& in) { return MapHeader::isMap(in); }

	void read(std::filesystem::path const& file, MapType map_types = MapType::ALL,
	          bool propagate = true)
	{
		std::ifstream f = MapHeader::openRead(file);
		read(f, map_types, propagate);
	}

	void read(std::istream& in, MapType map_types = MapType::ALL, bool propagate = true)
	{
		readData(in, readHeader(in), map_types, propagate);
	}

	void read(ReadBuffer& in, MapType map_types = MapType::ALL, bool propagate = true)
	{
		readData(in, readHeader(in), map_types, propagate);
	}

	[[nodiscard]] MapHeader readHeader(std::filesystem::path const& file) const
	{
		return MapHeader{file};
	}

	[[nodiscard]] MapHeader readHeader(std::istream& in) const { return MapHeader{in}; }

	[[nodiscard]] MapHeader readHeader(ReadBuffer& in) const { return MapHeader{in}; }

	void readData(std::istream& in, MapHeader const& header,
	              MapType map_types = MapType::ALL, bool propagate = true)
	{
		if (Dim != header.leaf_node_length.size()) {
			throw std::runtime_error("Trying to read map with dimension " +
			                         std::to_string(header.leaf_node_length.size()) +
			                         " into a map with dimension " + std::to_string(Dim));
		}

		Length leaf_node_length;
		for (std::size_t i{}; Dim > i; ++i) {
			leaf_node_length[i] = header.leaf_node_length[i];
		}

		if (Base::length(0) != leaf_node_length ||
		    Base::numDepthLevels() != header.num_depth_levels) {
			Base::clear(leaf_node_length, header.num_depth_levels);
		}

		readMaps(in, readNodes(in, header), header, map_types);

		if (propagate) {
			// TODO: What to do here?
			modifiedPropagate();
		}
	}

	void readData(ReadBuffer& in, MapHeader const& header, MapType map_types = MapType::ALL,
	              bool propagate = true)
	{
		if (Dim != header.leaf_node_length.size()) {
			throw std::runtime_error("Trying to read map with dimension " +
			                         std::to_string(header.leaf_node_length.size()) +
			                         " into a map with dimension " + std::to_string(Dim));
		}

		Length leaf_node_length;
		for (std::size_t i{}; Dim > i; ++i) {
			leaf_node_length[i] = header.leaf_node_length[i];
		}

		if (Base::length(0) != leaf_node_length ||
		    Base::numDepthLevels() != header.num_depth_levels) {
			Base::clear(leaf_node_length, header.num_depth_levels);
		}

		readMaps(in, readNodes(in, header), header, map_types);

		if (propagate) {
			// TODO: What to do here?
			modifiedPropagate();
		}
	}

	template <class Predicate                                             = pred::Leaf,
	          std::enable_if_t<pred::is_pred_v<Predicate, MapFull>, bool> = true>
	void write(std::filesystem::path const& file, Predicate const& pred = pred::Leaf{},
	           MapType map_types = MapType::ALL) const
	{
		std::ofstream f = MapHeader::openWrite(file);
		return write(f, pred, map_types);
	}

	template <class Predicate                                             = pred::Leaf,
	          std::enable_if_t<pred::is_pred_v<Predicate, MapFull>, bool> = true>
	void write(std::ostream& out, Predicate const& pred = pred::Leaf{},
	           MapType map_types = MapType::ALL) const
	{
		auto [tree, nodes] = writeNodes(pred);

		MapHeader h        = header(map_types);
		h.num_blocks       = nodes.size();
		h.num_nodes        = numNodes(nodes);
		h.map_info[0].size = tree.size();
		for (std::size_t i = 1; h.map_info.size() > i; ++i) {
			h.map_info[i].size = onSerializedSize(nodes, h.num_nodes, h.map_info[i].type);
		}
		h.write(out);

		if (!tree.empty()) {
			out.write(reinterpret_cast<char*>(tree.data()), tree.size() * sizeof(BitSet<BF>));
		}

		if (!nodes.empty()) {
			writeMaps(out, nodes, h);
		}
	}

	template <class Predicate                                             = pred::Leaf,
	          std::enable_if_t<pred::is_pred_v<Predicate, MapFull>, bool> = true>
	void write(WriteBuffer& out, Predicate const& pred = pred::Leaf{},
	           MapType map_types = MapType::ALL) const
	{
		auto [tree, nodes] = writeNodes(pred);

		MapHeader h        = header(map_types);
		h.num_blocks       = nodes.size();
		h.num_nodes        = numNodes(nodes);
		h.map_info[0].size = tree.size();
		for (std::size_t i = 1; h.map_info.size() > i; ++i) {
			h.map_info[i].size = onSerializedSize(nodes, h.num_nodes, h.map_info[i].type);
		}
		h.write(out);

		if (!tree.empty()) {
			out.write(tree.data(), tree.size() * sizeof(BitSet<BF>));
		}

		if (!nodes.empty()) {
			writeMaps(out, nodes, h);
		}
	}

	template <class Predicate                                             = pred::Leaf,
	          std::enable_if_t<pred::is_pred_v<Predicate, MapFull>, bool> = true>
	[[nodiscard]] Buffer write(Predicate const& pred      = pred::Leaf{},
	                           MapType          map_types = MapType::ALL) const
	{
		Buffer buffer;
		write(buffer, pred, map_types);
		return buffer;
	}

	template <class Predicate                                             = pred::Leaf,
	          std::enable_if_t<pred::is_pred_v<Predicate, MapFull>, bool> = true>
	[[nodiscard]] MapHeader writeData(std::filesystem::path const& file,
	                                  Predicate const&             pred = pred::Leaf{},
	                                  MapType map_types = MapType::ALL) const
	{
		std::ofstream f = MapHeader::openWrite(file);
		return writeData(f, pred, map_types);
	}

	template <class Predicate                                             = pred::Leaf,
	          std::enable_if_t<pred::is_pred_v<Predicate, MapFull>, bool> = true>
	[[nodiscard]] MapHeader writeData(std::ostream&    out,
	                                  Predicate const& pred      = pred::Leaf{},
	                                  MapType          map_types = MapType::ALL) const
	{
		auto [tree, nodes] = writeNodes(pred);

		MapHeader h        = header(map_types);
		h.num_blocks       = nodes.size();
		h.num_nodes        = numNodes(nodes);
		h.map_info[0].size = tree.size();
		for (std::size_t i = 1; h.map_info.size() > i; ++i) {
			h.map_info[i].size = onSerializedSize(nodes, h.num_nodes, h.map_info[i].type);
		}

		if (!tree.empty()) {
			out.write(reinterpret_cast<char*>(tree.data()), tree.size() * sizeof(BitSet<BF>));
		}

		if (!nodes.empty()) {
			writeMaps(out, nodes, h);
		}

		return h;
	}

	template <class Predicate                                             = pred::Leaf,
	          std::enable_if_t<pred::is_pred_v<Predicate, MapFull>, bool> = true>
	[[nodiscard]] MapHeader writeData(WriteBuffer&     out,
	                                  Predicate const& pred      = pred::Leaf{},
	                                  MapType          map_types = MapType::ALL) const
	{
		auto [tree, nodes] = writeNodes(pred);

		MapHeader h        = header(map_types);
		h.num_blocks       = nodes.size();
		h.num_nodes        = numNodes(nodes);
		h.map_info[0].size = tree.size();
		for (std::size_t i = 1; h.map_info.size() > i; ++i) {
			h.map_info[i].size = onSerializedSize(nodes, h.num_nodes, h.map_info[i].type);
		}

		if (!tree.empty()) {
			out.write(tree.data(), tree.size() * sizeof(BitSet<BF>));
		}

		if (!nodes.empty()) {
			writeMaps(out, nodes, h);
		}

		return h;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Dot file                                       |
	|                                                                                     |
	**************************************************************************************/

	template <class Predicate                                             = pred::True,
	          std::enable_if_t<pred::is_pred_v<Predicate, MapFull>, bool> = true>
	void saveDotFile(std::filesystem::path const& file,
	                 Predicate const&             pred      = pred::True{},
	                 MapType                      map_types = MapType::ALL) const
	{
		saveDotFile(Base::node(), file, pred, map_types);
	}

	template <class Predicate                                             = pred::True,
	          std::enable_if_t<pred::is_pred_v<Predicate, MapFull>, bool> = true>
	void saveDotFile(std::ostream& out, Predicate const& pred = pred::True{},
	                 MapType map_types = MapType::ALL) const
	{
		saveDotFile(Base::node(), out, pred, map_types);
	}

	template <class NodeType, class Predicate = pred::True,
	          std::enable_if_t<Base::template is_node_type_v<NodeType>, bool> = true,
	          std::enable_if_t<pred::is_pred_v<Predicate, MapFull>, bool>     = true>
	void saveDotFile(NodeType node, std::filesystem::path const& file,
	                 Predicate const& pred      = pred::True{},
	                 MapType          map_types = MapType::ALL) const
	{
		std::ofstream f = MapHeader::openWrite(file);
		saveDotFile(node, f, pred, map_types);
	}

	template <class NodeType, class Predicate = pred::True,
	          std::enable_if_t<Base::template is_node_type_v<NodeType>, bool> = true,
	          std::enable_if_t<pred::is_pred_v<Predicate, MapFull>, bool>     = true>
	void saveDotFile(NodeType node, std::ostream& out, Predicate const& pred = pred::True{},
	                 MapType map_types = MapType::ALL) const
	{
		using Filter = pred::Filter<Predicate>;

		Node n = Base::node(node);

		std::string const parent_shape = "shape=polygon,sides=" + std::to_string(BF) + ' ';

		out << std::boolalpha << "graph UFOMap {\n";
		out << "fontname=\"Helvetica,Arial,sans-serif\"\n";
		out << "node [fontname=\"Helvetica,Arial,sans-serif\" shape=ellipse color=lightblue2 "
		       "style=filled]\n";
		out << "edge [fontname=\"Helvetica,Arial,sans-serif\"]\n";

		Filter::init(pred, *this);
		bool valid_return = Base::exists(n) && Filter::returnable(pred, *this, n);
		bool valid_inner  = Base::isParent(n) && Filter::traversable(pred, *this, n);

		if (!valid_return && !valid_inner) {
			out << "}";
			return;
		}

		std::string id =
		    std::to_string(node.index.pos) + '.' + std::to_string(node.index.offset) + '.' +
		    std::to_string(node.code.depth()) + '.' + std::to_string(node.code.offset());
		out << id << " [";

		if (valid_inner) {
			out << parent_shape;
		}

		out << "label=<";
		if (valid_return) {
			// We need to write this node regardless, otherwise we would have disconnected nodes
			onDotFile(out, n.index, map_types);
		} else {
			out << ' ';
		}

		out << ">";
		if (valid_inner) {
			out << " color=darkorange1";
		}
		out << "]\n";

		if (valid_inner) {
			saveDotFileRecurs(out, n, id, pred, map_types, parent_shape);
		}

		out << "}";
	}

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                             Functions Maps Should Have                              |
	|                                                                                     |
	**************************************************************************************/

	void onInitRoot() { (onInitRoot<Maps<MapFull, Base>>(), ...); }

	template <class Map>
	void onInitRoot()
	{
		Map::onInitRoot();
	}

	void onInitChildren(Index node, pos_t children)
	{
		(onInitChildren<Maps<MapFull, Base>>(node, children), ...);
	}

	template <class Map>
	void onInitChildren(Index node, pos_t children)
	{
		Map::onInitChildren(node, children);
	}

	void onPropagateChildren(Index node, pos_t children, MapType map_types)
	{
		(onPropagateChildren<Maps<MapFull, Base>>(node, children, map_types), ...);
	}

	template <class Map>
	void onPropagateChildren(Index node, pos_t children, MapType map_types)
	{
		if (!isMapType<Map>(map_types)) {
			return;
		}

		Map::onPropagateChildren(node, children);
	}

	[[nodiscard]] bool onIsPrunable(pos_t block) const
	{
		return (onIsPrunable<Maps<MapFull, Base>>(block) && ...);
	}

	template <class Map>
	[[nodiscard]] bool onIsPrunable(pos_t block) const
	{
		return Map::onIsPrunable(block);
	}

	void onPruneChildren(Index node, pos_t children)
	{
		(onPruneChildren<Maps<MapFull, Base>>(node, children), ...);
	}

	template <class Map>
	void onPruneChildren(Index node, pos_t children)
	{
		Map::onPruneChildren(node, children);
	}

	[[nodiscard]] std::size_t onSerializedSize(
	    std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes, std::size_t num_nodes,
	    MapType map_types) const
	{
		return (onSerializedSize<Maps<MapFull, Base>>(nodes, num_nodes, map_types) + ...);
	}

	template <class Map>
	[[nodiscard]] std::size_t onSerializedSize(
	    std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes, std::size_t num_nodes,
	    MapType map_types) const
	{
		return isMapType<Map>(map_types) ? Map::onSerializedSize(nodes, num_nodes) : 0;
	}

	void onRead(ReadBuffer& in, std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes,
	            MapType map_type, std::uint64_t data_size)
	{
		(onRead<Maps<MapFull, Base>>(in, nodes, map_type, data_size) || ...);
	}

	template <class Map>
	bool onRead(ReadBuffer& in, std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes,
	            MapType map_type, std::uint64_t data_size)
	{
		if (!isMapType<Map>(map_type)) {
			return false;
		}

		Map::onRead(in, nodes);
		return true;
	}

	void onWrite(WriteBuffer& out, std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes,
	             MapType map_type) const
	{
		(onWrite<Maps<MapFull, Base>>(out, nodes, map_type), ...);
	}

	template <class Map>
	void onWrite(WriteBuffer& out, std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes,
	             MapType map_type) const
	{
		if (!isMapType<Map>(map_type)) {
			return;
		}

		Map::onWrite(out, nodes);
	}

	void onDotFile(std::ostream& out, Index node, MapType map_types) const
	{
		out << "<br/>Center: " << Base::center(node);
		out << "<br/>Depth: " << Base::depth(node) << " | Length: " << Base::length(node);
		if (Base::modified(node)) {
			out << "Modified: <font color='green'><b>true</b></font>";
		} else {
			out << "Modified: <font color='red'>false</font>";
		}

		(onDotFile<Maps<MapFull, Base>>(out, node, map_types), ...);
	}

	template <class Map>
	void onDotFile(std::ostream& out, Index node, MapType map_types) const
	{
		if (MapType::NONE == (mapType<Map>() & map_types)) {
			return;
		}

		Map::onDotFile((out << "<br/>"), node);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Map Types                                      |
	|                                                                                     |
	**************************************************************************************/

	template <class Map>
	[[nodiscard]] static constexpr MapType mapType() noexcept
	{
		return Map::Type;
	}

	template <class Map>
	[[nodiscard]] static constexpr bool isMapType(MapType map_type) noexcept
	{
		return mapType<Map>() == (mapType<Map>() & map_type);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                         I/O                                         |
	|                                                                                     |
	**************************************************************************************/

	template <class Input>
	[[nodiscard]] std::vector<std::pair<pos_t, BitSet<BF>>> readNodes(
	    Input& in, MapHeader const& header)
	{
		std::vector<BitSet<BF>>                   tree(header.map_info[0].size);
		std::vector<std::pair<pos_t, BitSet<BF>>> nodes;

		if (0 < header.map_info[0].size) {
			if constexpr (std::is_base_of_v<std::istream, remove_cvref_t<Input>>) {
				in.read(reinterpret_cast<char*>(tree.data()),
				        header.map_info[0].size * sizeof(BitSet<BF>));
			} else if constexpr (std::is_base_of_v<ReadBuffer, remove_cvref_t<Input>>) {
				in.read(tree.data(), header.map_info[0].size * sizeof(BitSet<BF>));
			} else {
				static_assert(dependent_false_v<Input>, "Wrong input type");
			}
		}

		if (0 < header.num_blocks) {
			nodes.reserve(header.num_blocks);
			readNodesRecurs(Base::block(), tree.begin(), nodes);
		}

		return nodes;
	}

	typename std::vector<BitSet<BF>>::const_iterator readNodesRecurs(
	    pos_t block, typename std::vector<BitSet<BF>>::const_iterator tree_it,
	    std::vector<std::pair<pos_t, BitSet<8>>>& nodes)
	{
		BitSet<BF> const valid_return         = *tree_it++;
		BitSet<BF> const valid_inner          = *tree_it++;
		BitSet<BF> const valid_erase_children = valid_return & ~valid_inner;

		if (valid_return.any()) {
			nodes.emplace_back(block, valid_return);
		}

		if (valid_erase_children.any()) {
			for (std::size_t i{}; BF > i; ++i) {
				Index node(block, i);
				if (valid_erase_children[i]) {
					Base::eraseChildren(node);
				}
			}
		}

		if (valid_inner.any()) {
			for (std::size_t i{}; BF > i; ++i) {
				Index node(block, i);
				if (valid_inner[i]) {
					auto children = Base::createChildren(node);
					tree_it       = readNodesRecurs(children, tree_it, nodes);
				}
			}
		}

		return tree_it;
	}

	void readMaps(std::istream& in, std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes,
	              MapHeader const& header, MapType map_types)
	{
		Buffer buffer;
		for (std::size_t i = 1; header.map_info.size() > i; ++i) {
			auto map_type  = header.map_info[i].type;
			auto data_size = header.map_info[i].size;

			if (hasMapTypes(map_types & map_type)) {
				buffer.clear();
				buffer.write(in, data_size);
				onRead(buffer, nodes, map_type, data_size);
			} else {
				// Skip forward
				in.seekg(static_cast<std::istream::off_type>(data_size), std::istream::cur);
			}
		}
	}

	void readMaps(ReadBuffer& in, std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes,
	              MapHeader const& header, MapType map_types)
	{
		for (std::size_t i = 1; header.map_info.size() > i; ++i) {
			auto map_type  = header.map_info[i].type;
			auto data_size = header.map_info[i].size;

			if (hasMapTypes(map_types & map_type)) {
				onRead(in, nodes, map_type, data_size);
			} else {
				// Skip forward
				in.readSkip(in.readPos() + data_size);
			}
		}
	}

	template <class Predicate,
	          std::enable_if_t<pred::is_pred_v<Predicate, MapFull>, bool> = true>
	[[nodiscard]] std::pair<std::vector<BitSet<BF>>,
	                        std::vector<std::pair<pos_t, BitSet<BF>>>>
	writeNodes(Predicate pred) const
	{
		std::pair<std::vector<BitSet<BF>>, std::vector<std::pair<pos_t, BitSet<BF>>>> res;

		auto& tree  = res.first;
		auto& nodes = res.second;

		if constexpr ((std::is_same_v<decltype(ufo::pred::Leaf() && ufo::pred::Modified()),
		                              ufo::remove_cvref_t<Predicate>> ||
		               std::is_same_v<decltype(ufo::pred::Modified() && ufo::pred::Leaf()),
		                              ufo::remove_cvref_t<Predicate>>)) {
			auto root = Base::index();

			bool leaf     = Base::isLeaf(root);
			bool modified = Base::modified(root);

			bool valid_return = leaf && modified;
			bool valid_inner  = !leaf && modified;

			tree.emplace_back(valid_return ? 1u : 0u);
			tree.emplace_back(valid_inner ? 1u : 0u);

			if (valid_return) {
				nodes.emplace_back(root.pos, BitSet<BF>(1u));
			} else if (valid_inner) {
				writeNodesModifiedRecurs(Base::children(root), tree, nodes);
			}
		} else {
			using Filter = pred::Filter<Predicate>;

			Filter::init(pred, *this);

			auto root = Base::node();

			bool valid_return = Filter::returnable(pred, *this, root);
			bool valid_inner =
			    Base::isParent(root.index) && Filter::traversable(pred, *this, root);

			tree.emplace_back(valid_return ? 1u : 0u);
			tree.emplace_back(valid_inner ? 1u : 0u);

			if (valid_return) {
				nodes.emplace_back(root.index.pos, BitSet<BF>(1u));
			}
			if (valid_inner) {
				writeNodesRecurs(root, pred, tree, nodes);
			}
		}

		if (nodes.empty()) {
			tree.clear();
		}

		return res;
	}

	template <class Predicate,
	          std::enable_if_t<pred::is_pred_v<Predicate, MapFull>, bool> = true>
	void writeNodesRecurs(Node parent, Predicate const& pred, std::vector<BitSet<BF>>& tree,
	                      std::vector<std::pair<pos_t, BitSet<BF>>>& nodes) const
	{
		using Filter = pred::Filter<Predicate>;

		BitSet<BF> valid_return;
		BitSet<BF> valid_inner;

		for (std::size_t i{}; BF > i; ++i) {
			Node node(Base::child(parent.code, i), Base::child(parent.index, i));

			valid_return[i] = Filter::returnable(pred, *this, node);
			valid_inner[i] =
			    Base::isParent(node.index) && Filter::traversable(pred, *this, node);
		}

		tree.push_back(valid_return);
		std::size_t valid_inner_pos = tree.size();
		tree.push_back(valid_inner);

		if (valid_return.any()) {
			nodes.emplace_back(Base::children(parent.index), valid_return);
		}

		if (valid_inner.any()) {
			for (std::size_t i{}; BF > i; ++i) {
				Node node(Base::child(parent.code, i), Base::child(parent.index, i));
				if (valid_inner[i]) {
					auto const tree_size  = tree.size();
					auto const nodes_size = nodes.size();
					writeNodesRecurs(node, pred, tree, nodes);
					if (nodes.size() == nodes_size) {
						tree.resize(tree_size);
						tree[valid_inner_pos][i] = false;
					}
				}
			}
		}
	}

	void writeNodesModifiedRecurs(pos_t block, std::vector<BitSet<BF>>& tree,
	                              std::vector<std::pair<pos_t, BitSet<BF>>>& nodes) const
	{
		BitSet<BF> valid_return;
		BitSet<BF> valid_inner;

		for (std::size_t i{}; BF > i; ++i) {
			Index node(block, i);
			bool  leaf      = Base::isLeaf(node);
			bool  modified  = Base::modified(node);
			valid_return[i] = leaf && modified;
			valid_inner[i]  = !leaf && modified;
		}

		tree.push_back(valid_return);
		std::size_t valid_inner_pos = tree.size();
		tree.push_back(valid_inner);

		if (valid_return.any()) {
			nodes.emplace_back(block, valid_return);
		}

		if (valid_inner.any()) {
			for (std::size_t i{}; BF > i; ++i) {
				Index node(block, i);
				if (valid_inner[i]) {
					auto const tree_size  = tree.size();
					auto const nodes_size = nodes.size();
					writeNodesModifiedRecurs(Base::children(node), tree, nodes);
					if (nodes.size() == nodes_size) {
						tree.resize(tree_size);
						tree[valid_inner_pos][i] = false;
					}
				}
			}
		}
	}

	[[nodiscard]] static std::size_t numNodes(
	    std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes)
	{
		return std::reduce(nodes.begin(), nodes.end(), std::size_t(0),
		                   [](auto const& a, auto const& b) {
			                   using A = remove_cvref_t<decltype(a)>;
			                   using B = remove_cvref_t<decltype(b)>;
			                   std::size_t res;
			                   if constexpr (std::is_same_v<std::pair<pos_t, BitSet<BF>>, A>) {
				                   res = a.second.count();
			                   } else {
				                   res = a;
			                   }
			                   if constexpr (std::is_same_v<std::pair<pos_t, BitSet<BF>>, B>) {
				                   res += b.second.count();
			                   } else {
				                   res += b;
			                   }
			                   return res;
		                   });
	}

	void writeMaps(std::ostream&                                    out,
	               std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes,
	               MapHeader const&                                 header) const
	{
		if (nodes.empty()) {
			return;
		}

		std::size_t max_size{};
		for (std::size_t i = 1; header.map_info.size() > i; ++i) {
			max_size = std::max(max_size, header.map_info[i].size);
		}

		Buffer buffer;
		buffer.reserve(max_size);

		for (std::size_t i = 1; header.map_info.size() > i; ++i) {
			buffer.clear();
			onWrite(buffer, nodes, header.map_info[i].type);
			if (!buffer.empty()) {
				buffer.read(out, buffer.size());
			}
			// TODO: Fill in compressed size
		}
	}

	void writeMaps(WriteBuffer& out, std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes,
	               MapHeader const& header) const
	{
		if (nodes.empty()) {
			return;
		}

		std::size_t total_size{};
		for (std::size_t i = 1; header.map_info.size() > i; ++i) {
			total_size += header.map_info[i].size;
		}

		out.reserve(out.size() + total_size);

		for (std::size_t i = 1; header.map_info.size() > i; ++i) {
			onWrite(out, nodes, header.map_info[i].type);
			// TODO: Fill in compressed size
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Dot file                                       |
	|                                                                                     |
	**************************************************************************************/

	template <class Predicate,
	          std::enable_if_t<pred::is_pred_v<Predicate, MapFull>, bool> = true>
	void saveDotFileRecurs(std::ostream& out, Node node, std::string const& id,
	                       Predicate const& pred, MapType map_types,
	                       std::string const& parent_shape) const
	{
		using Filter = pred::Filter<Predicate>;

		for (std::size_t i{}; BF > i; ++i) {
			Node child        = Base::child(node, i);
			bool valid_return = Filter::returnable(pred, *this, child);
			bool valid_inner = Base::isParent(child) && Filter::traversable(pred, *this, child);

			if (!valid_return && !valid_inner) {
				continue;
			}

			std::string child_id = std::to_string(child.index.pos) + '.' +
			                       std::to_string(child.index.offset) + '.' +
			                       std::to_string(child.code.depth()) + '.' +
			                       std::to_string(child.code.offset());
			out << child_id << " [";

			if (valid_inner) {
				out << parent_shape;
			}

			out << "label=<";
			if (valid_return) {
				// We need to write this node regardless, otherwise we would have disconnected
				// nodes
				onDotFile(out, child.index, map_types);
			} else {
				out << ' ';
			}

			out << ">";
			if (valid_inner) {
				out << " color=darkorange1";
			}
			out << "]\n";
			out << id << " -- " << child_id << '\n';

			if (valid_inner) {
				saveDotFileRecurs(out, child, child_id, pred, map_types, parent_shape);
			}
		}
	}
};

template <std::size_t Dim, template <class, class> class... Maps>
using Map = MapFull<Dim, MapUtility::NONE, Maps...>;
}  // namespace ufo

#endif  // UFO_MAP_FULL_HPP