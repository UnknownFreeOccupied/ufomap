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

#ifndef UFO_TREE_MAP_HPP
#define UFO_TREE_MAP_HPP

// UFO
#include <ufo/container/tree/predicate.hpp>
#include <ufo/execution/execution.hpp>
#include <ufo/map/tree/block.hpp>
#include <ufo/map/tree/map_block.hpp>
#include <ufo/map/tree/map_utility.hpp>
#include <ufo/map/tree/tree.hpp>
#include <ufo/map/type.hpp>
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
// NOTE: Remove this when it is possible to friend varidic number of classes
#define UFO_REPEAT_2(M, N)   M(N) M(N + 1)
#define UFO_REPEAT_4(M, N)   UFO_REPEAT_2(M, N) UFO_REPEAT_2(M, N + 2)
#define UFO_REPEAT_8(M, N)   UFO_REPEAT_4(M, N) UFO_REPEAT_4(M, N + 4)
#define UFO_REPEAT_16(M, N)  UFO_REPEAT_8(M, N) UFO_REPEAT_8(M, N + 8)
#define UFO_REPEAT_32(M, N)  UFO_REPEAT_16(M, N) UFO_REPEAT_16(M, N + 16)
#define UFO_REPEAT_64(M, N)  UFO_REPEAT_32(M, N) UFO_REPEAT_32(M, N + 32)
#define UFO_REPEAT_128(M, N) UFO_REPEAT_64(M, N) UFO_REPEAT_64(M, N + 64)

// All your base are belong to us
template <std::size_t Dim, mu_t Utility, template <class, class> class... Maps>
class Map
    : public MapTree<Map<Dim, Utility, Maps...>, Dim, Utility, map_block_t<Maps, Dim>...>
    , public Maps<
          Map<Dim, Utility, Maps...>,
          MapTree<Map<Dim, Utility, Maps...>, Dim, Utility, map_block_t<Maps, Dim>...>>...
{
	using Base = MapTree<Map, Dim, Utility, map_block_t<Maps, Dim>...>;

	//
	// Friends
	//

	// First base friends :)
	friend Base;
	// Second base friends ;)
	friend typename Base::Base;

#define UFO_TREE_MAP_FRIEND(F)                                          \
	friend std::tuple_element_t<std::min(static_cast<std::size_t>(F + 1), \
	                                     sizeof...(Maps)),                \
	                            std::tuple<void, Maps<Map, Base>...>>;
	UFO_REPEAT_128(UFO_TREE_MAP_FRIEND, 0)

	static constexpr auto const BF = Base::branchingFactor();

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                        Tags                                         |
	|                                                                                     |
	**************************************************************************************/

	// UFO stuff
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
	using mt_t     = typename Base::mt_t;

	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	Map(Length  leaf_node_length = Length(0.1),
	    depth_t num_depth_levels = std::min(depth_t(17), Base::maxNumDepthLevels()))
	    : Base(leaf_node_length, num_depth_levels)
	{
	}

	Map(length_t leaf_node_length,
	    depth_t  num_depth_levels = std::min(depth_t(17), Base::maxNumDepthLevels()))
	    : Map(Length(leaf_node_length), num_depth_levels)
	{
	}

	Map(std::filesystem::path const& file) : Map()
	{
		// TODO: read(file);
	}

	Map(std::istream& in) : Map()
	{
		// TODO: read(in);
	}

	Map(ReadBuffer& in) : Map()
	{
		// TODO: read(in);
	}

	Map(Map const&) = default;
	Map(Map&&)      = default;

	template <mu_t Utility2, template <class, class> class... Maps2>
	Map(Map<Dim, Utility2, Maps2...> const& other) : Base(static_cast<Base const&>(other))
	{
		// TODO: Implement
		// std::size_t size = Base::numBlocks();
		// (init<Maps<Map, Base>>(other, size), ...);
	}

	template <mu_t Utility2, template <class, class> class... Maps2>
	Map(Map<Dim, Utility2, Maps2...>&& other) : Base(static_cast<Base&&>(other))
	{
		// TODO: Implement
		// std::size_t size = Base::numBlocks();
		// (init<Maps<Map, Base>>(std::move(other), size), ...);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Destructor                                      |
	|                                                                                     |
	**************************************************************************************/

	~Map() = default;

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	Map& operator=(Map const&) = default;
	Map& operator=(Map&&)      = default;

	template <mu_t Utility2, template <class, class> class... Maps2>
	Map& operator=(Map<Dim, Utility2, Maps2...> const& rhs)
	{
		Base::operator=(static_cast<Base const&>(rhs));

		// TODO: Implement
		// std::size_t size = Base::numBlocks();
		// (init<Maps<Map, Base>>(rhs, size), ...);

		return *this;
	}

	template <mu_t Utility2, template <class, class> class... Maps2>
	Map& operator=(Map<Dim, Utility2, Maps2...>&& rhs)
	{
		Base::operator=(static_cast<Base&&>(rhs));

		// TODO: Implement
		// std::size_t size = Base::numBlocks();
		// (init<Maps<Map, Base>>(std::move(rhs), size), ...);

		return *this;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                         Swap                                        |
	|                                                                                     |
	**************************************************************************************/

	void swap(Map& other)
	{
		static_cast<Base&>(*this).swap(static_cast<Base&>(other));
		(static_cast<Maps<Map, Base>&>(*this).swap(other), ...);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Map Types                                      |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] static std::size_t numMapTypes() noexcept
	{
		return std::bitset<std::numeric_limits<mt_t>::digits>(mapTypes()).count();
	}

	[[nodiscard]] static constexpr mt_t mapTypes() noexcept
	{
		return (to_underlying(mapType<Maps<Map, Base>>()) | ...);
	}

	[[nodiscard]] static constexpr bool mapType(MapType map_type)
	{
		return 0 != (to_underlying(map_type) & mapTypes());
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Dot file                                       |
	|                                                                                     |
	**************************************************************************************/

	template <class Pred                                         = pred::True,
	          std::enable_if_t<pred::is_pred_v<Pred, Map>, bool> = true>
	void saveDotFile(std::filesystem::path const& file, Pred const& pred = pred::True{},
	                 mt_t map_types = to_underlying(MapType::ALL)) const
	{
		saveDotFile(Base::node(), file, pred, map_types);
	}

	template <class Pred                                         = pred::True,
	          std::enable_if_t<pred::is_pred_v<Pred, Map>, bool> = true>
	void saveDotFile(std::ostream& out, Pred const& pred = pred::True{},
	                 mt_t map_types = to_underlying(MapType::ALL)) const
	{
		saveDotFile(Base::node(), out, pred, map_types);
	}

	template <class Pred                                         = pred::True,
	          std::enable_if_t<pred::is_pred_v<Pred, Map>, bool> = true>
	void saveDotFile(Index node, std::filesystem::path const& file,
	                 Pred const& pred      = pred::True{},
	                 mt_t        map_types = to_underlying(MapType::ALL)) const
	{
		saveDotFile(Base::node(node), file, pred, map_types);
	}

	template <class Pred                                         = pred::True,
	          std::enable_if_t<pred::is_pred_v<Pred, Map>, bool> = true>
	void saveDotFile(Index node, std::ostream& out, Pred const& pred = pred::True{},
	                 mt_t map_types = to_underlying(MapType::ALL)) const
	{
		saveDotFile(Base::node(node), out, pred, map_types);
	}

	template <class Pred                                         = pred::True,
	          std::enable_if_t<pred::is_pred_v<Pred, Map>, bool> = true>
	void saveDotFile(Node node, std::filesystem::path const& file,
	                 Pred const& pred      = pred::True{},
	                 mt_t        map_types = to_underlying(MapType::ALL)) const
	{
		std::ofstream f;
		Base::openFileOut(file, f);
		saveDotFile(node, f, pred, map_types);
	}

	template <class Pred                                         = pred::True,
	          std::enable_if_t<pred::is_pred_v<Pred, Map>, bool> = true>
	void saveDotFile(Node node, std::ostream& out, Pred pred = pred::True{},
	                 mt_t map_types = to_underlying(MapType::ALL)) const
	{
		std::string const parent_shape = "shape=polygon,sides=" + std::to_string(BF) + ' ';

		out << std::boolalpha << "graph UFOMap {\n";
		out << "fontname=\"Helvetica,Arial,sans-serif\"\n";
		out << "node [fontname=\"Helvetica,Arial,sans-serif\" shape=ellipse color=lightblue2 "
		       "style=filled]\n";
		out << "edge [fontname=\"Helvetica,Arial,sans-serif\"]\n";

		// To ensure index point to correct node
		node = Base::node(node);

		pred::Filter<Pred>::init(pred, *this);
		bool valid_return =
		    Base::exists(node) && pred::Filter<Pred>::returnable(pred, *this, node);
		bool valid_inner =
		    Base::isParent(node) && pred::Filter<Pred>::traversable(pred, *this, node);

		if (!valid_return && !valid_inner) {
			out << "}";
			return;
		}

		std::string id = std::to_string(node.code[2]) + std::to_string(node.code[1]) +
		                 std::to_string(node.code[0]) + '.' +
		                 std::to_string(node.code.depth());
		out << id << " [";

		if (valid_inner) {
			out << parent_shape;
		}

		out << "label=<";
		if (valid_return) {
			// We need to write this node regardless, otherwise we would have disconnected nodes
			Base::onDotFileInfo(out, node.index);
			(onDotFileInfo<Maps<Map, Base>>(out, node.index, map_types), ...);
		} else {
			out << ' ';
		}

		out << ">";
		if (valid_inner) {
			out << " color=darkorange1";
		}
		out << "]\n";

		if (valid_inner) {
			dotFileInfoRecurs(out, node, id, pred, map_types, parent_shape);
		}

		out << "}";
	}

	template <class Pred                                         = pred::True,
	          std::enable_if_t<pred::is_pred_v<Pred, Map>, bool> = true>
	void saveDotFile(Code node, std::filesystem::path const& file,
	                 Pred const& pred      = pred::True{},
	                 mt_t        map_types = to_underlying(MapType::ALL)) const
	{
		saveDotFile(Base::node(node), file, pred, map_types);
	}

	template <class Pred                                         = pred::True,
	          std::enable_if_t<pred::is_pred_v<Pred, Map>, bool> = true>
	void saveDotFile(Code node, std::ostream& out, Pred const& pred = pred::True{},
	                 mt_t map_types = to_underlying(MapType::ALL)) const
	{
		saveDotFile(Base::node(node), out, pred, map_types);
	}

	template <class Pred                                         = pred::True,
	          std::enable_if_t<pred::is_pred_v<Pred, Map>, bool> = true>
	void saveDotFile(Key node, std::filesystem::path const& file,
	                 Pred const& pred      = pred::True{},
	                 mt_t        map_types = to_underlying(MapType::ALL)) const
	{
		saveDotFile(Base::node(node), file, pred, map_types);
	}

	template <class Pred                                         = pred::True,
	          std::enable_if_t<pred::is_pred_v<Pred, Map>, bool> = true>
	void saveDotFile(Key node, std::ostream& out, Pred const& pred = pred::True{},
	                 mt_t map_types = to_underlying(MapType::ALL)) const
	{
		saveDotFile(Base::node(node), out, pred, map_types);
	}

	template <class Pred                                         = pred::True,
	          std::enable_if_t<pred::is_pred_v<Pred, Map>, bool> = true>
	void saveDotFile(Coord node, std::filesystem::path const& file,
	                 Pred const& pred      = pred::True{},
	                 mt_t        map_types = to_underlying(MapType::ALL)) const
	{
		saveDotFile(Base::node(node), file, pred, map_types);
	}

	template <class Pred                                         = pred::True,
	          std::enable_if_t<pred::is_pred_v<Pred, Map>, bool> = true>
	void saveDotFile(Coord node, std::ostream& out, Pred const& pred = pred::True{},
	                 mt_t map_types = to_underlying(MapType::ALL)) const
	{
		saveDotFile(Base::node(node), out, pred, map_types);
	}

 protected:
	void onInitRoot()
	{
		// Call MapTree
		Base::onInitRoot();

		// Call other Maps
		(onInitRoot<Maps<Map, Base>>(), ...);
	}

	template <class Map>
	void onInitRoot()
	{
		Map::onInitRoot();
	}

	void onInitChildren(Index node, pos_t children)
	{
		// Call MapTree
		Base::onInitChildren(node, children);

		// Call other Maps
		(onInitChildren<Maps<Map, Base>>(node, children), ...);
	}

	template <class Map>
	void onInitChildren(Index node, pos_t children)
	{
		Map::onInitChildren(node, children);
	}

	void onPruneChildren(Index node, pos_t children)
	{
		// Call MapTree
		Base::onPruneChildren(node, children);

		// Call other Maps
		(onPruneChildren<Maps<Map, Base>>(node, children), ...);
	}

	template <class Map>
	void onPruneChildren(Index node, pos_t children)
	{
		Map::onPruneChildren(node, children);
	}

	void onPropagateChildren(Index node, pos_t children, mt_t map_types)
	{
		(onPropagateChildren<Maps<Map, Base>>(node, children, map_types), ...);
	}

	template <class Map>
	void onPropagateChildren(Index node, pos_t children, mt_t map_types)
	{
		// TODO: Add some check here
		if (!isValidMapType<Map>(map_types)) {
			return;
		}

		Map::onPropagateChildren(node, children);
	}

	[[nodiscard]] bool prunable(pos_t block) const
	{
		return (prunable<Maps<Map, Base>>(block) && ...);
	}

	template <class Map>
	[[nodiscard]] bool prunable(pos_t block) const
	{
		return Map::prunable(block);
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
	[[nodiscard]] constexpr bool isValidMapType(mt_t map_types) const
	{
		return 0 != (to_underlying(mapType<Map>()) && map_types);
	}

	//
	// Memory node block
	//

	// [[nodiscard]] std::size_t sizeofNodeTimesN(TreeIndex node) const
	// {
	// 	return (sizeOfNodeTimesN<Maps<Map, Base>>(node) + ...);
	// }

	// template <class Map>
	// [[nodiscard]] std::size_t sizeOfNodeTimesN(TreeIndex node) const
	// {
	// 	if (!isMapTypesEnabled(Map::mapType())) {
	// 		return 0;
	// 	}

	// 	return Map::sizeOfNodeTimesN(node);
	// }

	// [[nodiscard]] std::size_t sizeofBlock(pos_t block) const
	// {
	// 	return (sizeofBlock<Maps<Map, Base>>(block) + ...);
	// }

	// template <class Map>
	// [[nodiscard]] std::size_t sizeofBlock(pos_t block) const
	// {
	// 	if (!isMapTypesEnabled(Map::mapType())) {
	// 		return 0;
	// 	}

	// 	return Map::sizeofBlock(block);
	// }

	// [[nodiscard]] static constexpr std::size_t sizeofBlockLowerBound() noexcept
	// {
	// 	return (Maps<Map, Base>::sizeofBlockLowerBound() + ...);
	// }

	// [[nodiscard]] std::size_t sizeofMap() const { return (Maps<Map, Base>::sizeofMap() +
	// ...);
	// }

	// //
	// // Input/output (read/write)
	// //

	// template <class Range>
	// [[nodiscard]] std::size_t serializedSize(
	//     std::vector<std::pair<pos_t, BitSet<N>>> const& block_offset, std::size_t
	//     num_nodes, bool compress, Range const& map_types) const
	// {
	// 	return (serializedSize<Maps<Map, Base>>(block_offset, num_nodes, compress,
	// map_types) +
	// 	        ...);
	// }

	// template <class Map, class Range>
	// [[nodiscard]] std::size_t serializedSize(
	//     std::vector<std::pair<pos_t, BitSet<N>>> const& block_offset, std::size_t
	//     num_nodes, bool compress, Range const& map_types) const
	// {
	// 	if (!isValidMapType<Map>(map_types)) {
	// 		return 0;
	// 	}

	// 	// TODO: Implement

	// 	return 0;

	// 	// if (compress) {
	// 	// 	return sizeof(MapType) + sizeof(std::uint64_t) + sizeof(std::uint64_t) +
	// 	// 	       maxSizeCompressed(Map::serializedSize(block_offset, num_nodes));
	// 	// } else {
	// 	// 	return sizeof(MapType) + sizeof(std::uint64_t) +
	// 	// 	       Map::serializedSize(block_offset, num_nodes);
	// 	// }
	// }

	// template <class Container, class Range>
	// void readNodes(std::istream& in, Container const& c, bool const compressed,
	//                Range const& map_types)
	// {
	// 	// TODO: Implement

	// 	// auto cur_pos = in.tellg();
	// 	// in.seekg(0, std::ios_base::end);
	// 	// auto end_pos = in.tellg();
	// 	// in.seekg(cur_pos);

	// 	// map_types &= mapTypesEnabled();

	// 	// Buffer in_buf;
	// 	// Buffer compress_buf;
	// 	// while (in.tellg() != end_pos && in.good()) {
	// 	// 	MapType       mt;
	// 	// 	std::uint64_t data_size;

	// 	// 	in.read(reinterpret_cast<char*>(&mt), sizeof(mt));
	// 	// 	in.read(reinterpret_cast<char*>(&data_size), sizeof(data_size));

	// 	// 	if (mt & map_types) {
	// 	// 		in_buf.clear();
	// 	// 		in_buf.write(in, data_size);

	// 	// 		(readNodes<Maps<Map, Base>>(
	// 	// 		     in_buf, compress_buf, c, mt, data_size, compressed) ||
	// 	// 		 ...);
	// 	// 	} else {
	// 	// 		// Skip forward
	// 	// 		in.seekg(static_cast<std::istream::off_type>(data_size), std::istream::cur);
	// 	// 	}
	// 	// }
	// }

	// template <class Container, class Range>
	// void readNodes(ReadBuffer& in, Container const& c, bool const compressed,
	//                Range const& map_types)
	// {
	// 	// TODO: Implement

	// 	// map_types &= mapTypesEnabled();

	// 	// Buffer compress_buf;
	// 	// while (in.readIndex() < in.size()) {
	// 	// 	MapType       mt;
	// 	// 	std::uint64_t data_size;

	// 	// 	in.read(&mt, sizeof(mt));
	// 	// 	in.read(&data_size, sizeof(data_size));

	// 	// 	std::uint64_t next_index = in.readIndex() + data_size;

	// 	// 	if (mt & map_types) {
	// 	// 		(readNodes<Maps<Map, Base>>(
	// 	// 		     in, compress_buf, c, mt, data_size, compressed) ||
	// 	// 		 ...);
	// 	// 	}

	// 	// 	// Skip forward
	// 	// 	in.setReadIndex(next_index);
	// 	// }
	// }

	// template <class Map, class Container>
	// bool readNodes(ReadBuffer& in, Buffer& compress_buf, Container const& c,
	//                std::string_view map_type, uint64_t const data_size,
	//                bool const compressed)
	// {
	// 	// TODO: Implement
	// 	return true;

	// 	// if (Map::mapType() != map_type) {
	// 	// 	return false;
	// 	// }

	// 	// if (compressed) {
	// 	// 	compress_buf.clear();

	// 	// 	std::uint64_t uncompressed_size;
	// 	// 	in.read(&uncompressed_size, sizeof(uncompressed_size));

	// 	// 	decompressMultiLz4(in, compress_buf, uncompressed_size);

	// 	// 	Map::readNodes(compress_buf, c);
	// 	// } else {
	// 	// 	Map::readNodes(in, c);
	// 	// }

	// 	// return true;
	// }

	// template <class Range>
	// void writeNodes(std::ostream&                                   out,
	//                 std::vector<std::pair<pos_t, BitSet<N>>> const& block_offset,
	//                 bool const compress, Range const& map_types,
	//                 int const compression_acceleration_level,
	//                 int const compression_level) const
	// {
	// 	// TODO: Implement

	// 	// std::size_t num_nodes{};
	// 	// for (auto [_, offset] : block_offset) {
	// 	// 	num_nodes += offset.count();
	// 	// }

	// 	// map_types &= mapTypesEnabled();
	// 	// Buffer out_buf;
	// 	// Buffer compress_buf;
	// 	// (writeNodes<Maps<Map, Base>>(
	// 	//      out, out_buf, compress_buf, block_offset, num_nodes, compress, map_types,
	// 	//      compression_acceleration_level, compression_level),
	// 	//  ...);
	// }

	// template <class Range>
	// void writeNodes(WriteBuffer&                                    out,
	//                 std::vector<std::pair<pos_t, BitSet<N>>> const& block_offset,
	//                 bool const compress, Range const& map_types,
	//                 int const compression_acceleration_level,
	//                 int const compression_level) const
	// {
	// 	// TODO: Implement

	// 	// std::size_t num_nodes{};
	// 	// for (auto [_, offset] : block_offset) {
	// 	// 	num_nodes += offset.count();
	// 	// }

	// 	// map_types &= mapTypesEnabled();
	// 	// out.reserve(out.size() +
	// 	//             serializedSize(block_offset, num_nodes, compress, map_types));
	// 	// Buffer compress_buf;
	// 	// (writeNodes<Maps<Map, Base>>(
	// 	//      out, compress_buf, block_offset, num_nodes, compress, map_types,
	// 	//      compression_acceleration_level, compression_level),
	// 	//  ...);
	// }

	// template <class Map, class Range>
	// void writeNodes(std::ostream& out, Buffer& out_buf, Buffer& compress_buf,
	//                 std::vector<std::pair<pos_t, BitSet<N>>> const& block_offset,
	//                 std::size_t num_nodes, bool const compress, Range const& map_types,
	//                 int const compression_acceleration_level,
	//                 int const compression_level) const
	// {
	// 	// TODO: Implement

	// 	// out_buf.clear();
	// 	// out_buf.reserve(Map::serializedSize(block_offset, num_nodes));

	// 	// writeNodes<Map>(out_buf, compress_buf, block_offset, num_nodes, compress,
	// 	// map_types,
	// 	//                 compression_acceleration_level, compression_level);
	// 	// if (!out_buf.empty()) {
	// 	// 	out.write(reinterpret_cast<char const*>(out_buf.data()),
	// 	// 	          static_cast<std::streamsize>(out_buf.size()));
	// 	// }
	// }

	// template <class Map, class Range>
	// void writeNodes(WriteBuffer& out, Buffer& compress_buf,
	//                 std::vector<std::pair<pos_t, BitSet<N>>> const& block_offset,
	//                 std::size_t num_nodes, bool const compress, Range const& map_types,
	//                 int const compression_acceleration_level,
	//                 int const compression_level) const
	// {
	// 	// TODO: Implement

	// 	// constexpr MapType mt = Map::mapType();
	// 	// if constexpr (MapType::NONE == mt) {
	// 	// 	return;
	// 	// } else if (0 == (mt & map_types)) {
	// 	// 	return;
	// 	// }

	// 	// out.write(&mt, sizeof(mt));

	// 	// std::uint64_t size;
	// 	// auto          size_index = out.writeIndex();
	// 	// out.setWriteIndex(size_index + sizeof(size));

	// 	// if (compress) {
	// 	// 	compress_buf.clear();
	// 	// 	compress_buf.reserve(Map::serializedSize(block_offset, num_nodes));
	// 	// 	Map::writeNodes(compress_buf, block_offset);

	// 	// 	compressMultiLz4(compress_buf, out, compress_buf.size(),
	// 	// 	                 compression_acceleration_level, compression_level);
	// 	// } else {
	// 	// 	Map::writeNodes(out, block_offset);
	// 	// }

	// 	// auto cur_index = out.writeIndex();
	// 	// size           = cur_index - (size_index + sizeof(size));
	// 	// out.setWriteIndex(size_index);
	// 	// out.write(&size, sizeof(size));
	// 	// out.setWriteIndex(cur_index);
	// }

	/**************************************************************************************
	|                                                                                     |
	|                                      Dot file                                       |
	|                                                                                     |
	**************************************************************************************/

	void onDotFileInfo(std::ostream& out, Index node, mt_t map_types) const
	{
		(onDotFileInfo<Maps<Map, Base>>(out, node, map_types), ...);
	}

	template <class Map>
	void onDotFileInfo(std::ostream& out, Index node, mt_t map_types) const
	{
		if (0 == (to_underlying(mapType<Map>()) & map_types)) {
			return;
		}

		Map::onDotFileInfo((out << "<br/>"), node);
	}

	template <class Pred, std::enable_if_t<pred::is_pred_v<Pred, Map>, bool> = true>
	void dotFileInfoRecurs(std::ostream& out, Node node, std::string const& id,
	                       Pred const& pred, mt_t map_types,
	                       std::string const& parent_shape) const
	{
		for (std::size_t i{}; BF > i; ++i) {
			Node child        = Base::child(node, i);
			bool valid_return = pred::Filter<Pred>::returnable(pred, *this, child);
			bool valid_inner =
			    Base::isParent(child) && pred::Filter<Pred>::traversable(pred, *this, child);

			if (!valid_return && !valid_inner) {
				continue;
			}

			std::string child_id =
			    std::to_string(child.code[2]) + std::to_string(child.code[1]) +
			    std::to_string(child.code[0]) + '.' + std::to_string(child.code.depth());
			out << child_id << " [";

			if (valid_inner) {
				out << parent_shape;
			}

			out << "label=<";
			if (valid_return) {
				// We need to write this node regardless, otherwise we would have disconnected
				// nodes
				Base::onDotFileInfo(out, child.index);
				(onDotFileInfo<Maps<Map, Base>>(out, child.index, map_types), ...);
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
				dotFileInfoRecurs(out, child, child_id, pred, map_types, parent_shape);
			}
		}
	}

 private:
	mt_t enabled_map_types_ = mapTypes();
};
}  // namespace ufo

#endif  // UFO_TREE_MAP_HPP