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

#ifndef UFO_MAP_H
#define UFO_MAP_H

// UFO
#include <ufo/map/octree/octree_base.h>

// STL
#include <future>

namespace ufo::map
{
// NOTE: Remove this when it is possible to friend varidic number of classes
#define REPEAT_2(M, N) M(N) M(N + 1)
#define REPEAT_4(M, N) REPEAT_2(M, N) REPEAT_2(M, N + 2)
#define REPEAT_8(M, N) REPEAT_4(M, N) REPEAT_4(M, N + 4)
#define REPEAT_16(M, N) REPEAT_8(M, N) REPEAT_8(M, N + 8)
#define REPEAT_32(M, N) REPEAT_16(M, N) REPEAT_16(M, N + 16)
#define REPEAT_64(M, N) REPEAT_32(M, N) REPEAT_32(M, N + 32)
#define REPEAT_128(M, N) REPEAT_64(M, N) REPEAT_64(M, N + 64)

// All your base are belong to us
template <class Data, class InnerData, template <class> class... Bases>
class OctreeMap
    : public OctreeBase<OctreeMap<Data, InnerData, Bases...>, Data, InnerData>,
      public Bases<OctreeMap<Data, InnerData, Bases...>>...
{
 protected:
	//
	// Tags
	//

	using Octree = OctreeBase<OctreeMap, Data, InnerData>;

	//
	// Friends
	//

	friend Octree;
#define FRIEND(N)                                                       \
	friend std::tuple_element_t<std::min(static_cast<std::size_t>(N + 1), \
	                                     sizeof...(Bases)),               \
	                            std::tuple<void, Bases<OctreeMap>...>>;
	REPEAT_128(FRIEND, 0)

 public:
	//
	// Constructors
	//

	OctreeMap(node_size_t leaf_node_size = 0.1, depth_t depth_levels = 16,
	          bool auto_prune = false)
	    : Octree(leaf_node_size, depth_levels, auto_prune)
	{
		initRoot();
	}

	OctreeMap(std::filesystem::path const& file, bool auto_prune = false)
	    : OctreeMap(0.1, 16, auto_prune)
	{
		Octree::read(file);
	}

	OctreeMap(std::istream& in, bool auto_prune = false) : OctreeMap(0.1, 16, auto_prune)
	{
		Octree::read(in);
	}

	OctreeMap(ReadBuffer& in, bool auto_prune = false) : OctreeMap(0.1, 16, auto_prune)
	{
		Octree::read(in);
	}

	OctreeMap(OctreeMap const& other) : Octree(other), Bases<OctreeMap>(other)...
	{
		readFromOtherMap(other);
	}

	template <class Data2, class InnerData2, template <class> class... Bases2>
	OctreeMap(OctreeMap<Data2, InnerData2, Bases2...> const& other)
	    : Octree(other), Bases<OctreeMap>(other)...
	{
		readFromOtherMap(other);
	}

	OctreeMap(OctreeMap&& other) = default;

	OctreeMap& operator=(OctreeMap const& rhs)
	{
		Octree::operator=(rhs);
		(Bases<OctreeMap>::operator=(rhs), ...);

		readFromOtherMap(rhs);

		return *this;
	}

	template <class Data2, class InnerData2, template <class> class... Bases2>
	OctreeMap& operator=(OctreeMap<Data2, InnerData2, Bases2...> const& rhs)
	{
		Octree::operator=(rhs);
		(Bases<OctreeMap>::operator=(rhs), ...);

		readFromOtherMap(rhs);

		return *this;
	}

	OctreeMap& operator=(OctreeMap&& rhs) = default;

	//
	// Swap
	//

	void swap(OctreeMap& other)  // TODO: Add noexcept thing
	{
		Octree::swap(other);
		(Bases<OctreeMap>::swap(other), ...);
	}

 protected:
	//
	// Read from other map
	//

	template <class Other>
	void readFromOtherMap(Other const& other)
	{
		initRoot();

		// for (auto node : other.query(predicate::Leaf(), true)) {
		// 	// TODO: Do something
		// 	map.set(node.code(), other.get(node));
		// }

		// TODO: Make it possible to only write the parts that they share
		auto buf = other.write();
		Octree::read(buf);

		// std::stringstream io(std::ios_base::in | std::ios_base::out |
		// std::ios_base::binary); io.exceptions(std::ifstream::failbit |
		// std::ifstream::badbit); io.imbue(std::locale());

		// other.write(io);
		// Octree::read(io);
	}

	//
	// Initilize root
	//

	void initRoot()
	{
		Octree::initRoot();
		(Bases<OctreeMap>::initRoot(), ...);
	}

	//
	// Fill
	//

	template <class NodeT, class ParentT>
	void fill(NodeT& node, ParentT const& parent, index_t const index)
	{
		(Bases<OctreeMap>::fill(node, parent, index), ...);
	}

	//
	// Clear
	//

	template <class NodeT>
	void clear(NodeT& node)
	{
		(Bases<OctreeMap>::clear(node), ...);
	}

	//
	// Update node
	//

	template <class NodeT, class ChildT>
	void updateNode(NodeT& node, index_t index, ChildT const& children)
	{
		(Bases<OctreeMap>::updateNode(node, index, children), ...);
	}

	//
	// Is collapsible
	//

	template <class NodeT>
	[[nodiscard]] bool isCollapsible(NodeT const& node) const
	{
		return (Bases<OctreeMap>::isCollapsible(node) && ...);
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr MapType mapType() noexcept
	{
		return (Bases<OctreeMap>::mapType() | ...);
	}

	template <class InputIt>
	[[nodiscard]] std::size_t serializedSize(InputIt first, InputIt last,
	                                         bool compress) const
	{
		return (serializedSize<Bases<OctreeMap>>(first, last, compress) + ...);
	}

	template <class OutputIt>
	void readNodes(std::istream& in, OutputIt first, OutputIt last, bool const compressed)
	{
		auto cur_pos = in.tellg();
		in.seekg(0, std::ios_base::end);
		auto end_pos = in.tellg();
		in.seekg(cur_pos);

		Buffer buf;
		Buffer compress_buf;
		while (in.tellg() != end_pos && in.good()) {
			MapType mt;
			std::uint64_t data_size;

			in.read(reinterpret_cast<char*>(&mt), sizeof(mt));
			in.read(reinterpret_cast<char*>(&data_size), sizeof(data_size));

			if ((Bases<OctreeMap>::canReadData(mt) || ...)) {
				(readNodes<Bases<OctreeMap>>(in, buf, compress_buf, first, last, mt, data_size,
				                             compressed) ||
				 ...);
			} else {
				// Skip forward
				in.seekg(data_size, std::istream::cur);
			}
		}
	}

	template <class OutputIt>
	void readNodes(ReadBuffer& in, OutputIt first, OutputIt last, bool const parallel,
	               bool const compressed)
	{
		// std::vector<std::future<void>> res;

		Buffer compress_buf;
		while (in.readIndex() < in.size()) {
			std::cout << "Start: " << in.readIndex() << " vs " << in.size() << '\n';
			MapType mt;
			std::uint64_t data_size;

			in.read(&mt, sizeof(mt));
			in.read(&data_size, sizeof(data_size));

			std::cout << "Data size " << data_size << '\n';
			std::cout << "Map type " << mt << '\n';

			std::uint64_t next_index = in.readIndex() + data_size;

			(readNodes<Bases<OctreeMap>>(in, compress_buf, first, last, mt, data_size,
			                             compressed) ||
			 ...);

			// Skip forward
			in.setReadIndex(next_index);
			std::cout << "End: " << in.readIndex() << " vs " << in.size() << '\n';
		}

		// for (auto const& r : res) {
		// 	r.wait();
		// }
	}

	template <class InputIt>
	void writeNodes(std::ostream& out, InputIt first, InputIt last, bool const compress,
	                int const compression_acceleration_level,
	                int const compression_level) const
	{
		Buffer buf;
		(writeNodes<Bases<OctreeMap>>(out, buf, first, last, compress,
		                              compression_acceleration_level, compression_level),
		 ...);
	}

	template <class InputIt>
	void writeNodes(WriteBuffer& out, InputIt first, InputIt last, bool const parallel,
	                bool const compress, int const compression_acceleration_level,
	                int const compression_level) const
	{
		out.reserve(out.size() + serializedSize(first, last, compress));
		(writeNodes<Bases<OctreeMap>>(out, first, last, compress,
		                              compression_acceleration_level, compression_level),
		 ...);
	}

 private:
	//
	// Input/output (read/write)
	//

	template <class Base, class InputIt>
	std::size_t serializedSize(InputIt first, InputIt last, bool compress) const
	{
		if (compress) {
			return sizeof(MapType) + sizeof(std::uint64_t) + sizeof(std::uint64_t) +
			       maxSizeCompressed(Base::serializedSize(first, last));
		} else {
			return sizeof(MapType) + sizeof(std::uint64_t) + Base::serializedSize(first, last);
		}
	}

	template <class Base, class OutputIt>
	bool readNodes(std::istream& in, Buffer& buf, Buffer& compress_buf, OutputIt first,
	               OutputIt last, MapType const mt, uint64_t const data_size,
	               bool const compressed)
	{
		if (!Base::canReadData(mt)) {
			return false;
		}

		buf.clear();
		// TODO: Implement better (should probably be resize?)
		buf.reserve(data_size);
		in.read(reinterpret_cast<char*>(buf.data()), data_size);
		return readNodes(buf, compress_buf, first, last, mt, data_size, compressed);
	}

	template <class Base, class OutputIt>
	bool readNodes(ReadBuffer& in, Buffer& compress_buf, OutputIt first, OutputIt last,
	               MapType const mt, uint64_t const data_size, bool const compressed)
	{
		if (!Base::canReadData(mt)) {
			return false;
		}

		if (compressed) {
			compress_buf.clear();

			std::uint64_t uncompressed_size;
			in.read(&uncompressed_size, sizeof(uncompressed_size));

			decompressData(in, compress_buf, uncompressed_size);

			Base::readNodes(compress_buf, first, last);
		} else {
			Base::readNodes(in, first, last);
		}

		return true;
	}

	template <class Base, class InputIt>
	void writeNodes(std::ostream& out, Buffer& buf, InputIt first, InputIt last,
	                bool const compress, int const compression_acceleration_level,
	                int const compression_level) const
	{
		buf.clear();
		writeNodes(buf, first, last, compress, compression_acceleration_level,
		           compression_level);

		if (!buf.empty()) {
			out.write(reinterpret_cast<char const*>(buf.data()), buf.size());
		}
	}

	template <class Base, class InputIt>
	void writeNodes(WriteBuffer& out, InputIt first, InputIt last, bool const compress,
	                int const compression_acceleration_level,
	                int const compression_level) const
	{
		constexpr MapType mt = Base::mapType();
		if constexpr (MapType::NONE == mt) {
			return;
		}

		out.write(&mt, sizeof(mt));

		std::uint64_t size;
		auto size_index = out.writeIndex();
		out.setWriteIndex(size_index + sizeof(size));

		if (compress) {
			Buffer data;
			data.reserve(Base::serializedSize(first, last));
			Base::writeNodes(data, first, last);

			compressData(data, out, compression_acceleration_level, compression_level);
		} else {
			Base::writeNodes(out, first, last);
		}

		auto cur_index = out.writeIndex();
		size = cur_index - (size_index + sizeof(size));
		out.setWriteIndex(size_index);
		out.write(&size, sizeof(size));
		out.setWriteIndex(cur_index);
	}
};
}  // namespace ufo::map

#endif  // UFO_MAP_H