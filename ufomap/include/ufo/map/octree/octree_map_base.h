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

#ifndef UFO_MAP_BASE_H
#define UFO_MAP_BASE_H

// UFO
#include <ufo/map/octree/octree_base.h>
#include <ufo/map/octree/octree_node_base.h>

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
template <class Data, class InnerData, bool ReuseNodes, bool LockLess, bool CountNodes,
          template <class> class... Bases>
class OctreeMapBase
    : public OctreeBase<
          OctreeMapBase<Data, InnerData, ReuseNodes, LockLess, CountNodes, Bases...>,
          Data, InnerData, ReuseNodes, LockLess, CountNodes>,
      public Bases<
          OctreeMapBase<Data, InnerData, ReuseNodes, LockLess, CountNodes, Bases...>>...
{
 protected:
	//
	// Tags
	//

	using Octree =
	    OctreeBase<OctreeMapBase, Data, InnerData, ReuseNodes, LockLess, CountNodes>;

	//
	// Friends
	//

	friend Octree;
#define FRIEND(N)                                                       \
	friend std::tuple_element_t<std::min(static_cast<std::size_t>(N + 1), \
	                                     sizeof...(Bases)),               \
	                            std::tuple<void, Bases<OctreeMapBase>...>>;
	REPEAT_128(FRIEND, 0)

 public:
	//
	// Constructors
	//

	OctreeMapBase(node_size_t leaf_node_size = 0.1, depth_t depth_levels = 16,
	              bool auto_prune = true)
	    : Octree(leaf_node_size, depth_levels, auto_prune)
	{
		initRoot();
	}

	OctreeMapBase(std::filesystem::path const& file, bool auto_prune = true)
	    : OctreeMapBase(0.1, 16, auto_prune)
	{
		Octree::read(file);
	}

	OctreeMapBase(std::istream& in, bool auto_prune = true)
	    : OctreeMapBase(0.1, 16, auto_prune)
	{
		Octree::read(in);
	}

	OctreeMapBase(OctreeMapBase const& other)
	    : Octree(other), Bases<OctreeMapBase>(other)...
	{
		readFromOtherMap(other);
	}

	template <class Data2, class InnerData2, bool ReuseNodes2, bool LockLess2,
	          bool CountNodes2, template <class> class... Bases2>
	OctreeMapBase(OctreeMapBase<Data2, InnerData2, ReuseNodes2, LockLess2, CountNodes2,
	                            Bases2...> const& other)
	    : Octree(other), Bases<OctreeMapBase>(other)...
	{
		readFromOtherMap(other);
	}

	OctreeMapBase(OctreeMapBase&& other) = default;

	OctreeMapBase& operator=(OctreeMapBase const& rhs)
	{
		Octree::operator=(rhs);
		(Bases<OctreeMapBase>::operator=(rhs), ...);

		readFromOtherMap(rhs);

		return *this;
	}

	template <class Data2, class InnerData2, bool ReuseNodes2, bool LockLess2,
	          bool CountNodes2m template <class> class... Bases2>
	OctreeMapBase& operator=(OctreeMapBase<Data2, InnerData2, ReuseNodes2, LockLess2,
	                                       CountNodes2, Bases2...> const& rhs)
	{
		Octree::operator=(rhs);
		(Bases<OctreeMapBase>::operator=(rhs), ...);

		readFromOtherMap(rhs);

		return *this;
	}

	OctreeMapBase& operator=(OctreeMapBase&& rhs) = default;

 protected:
	//
	// Read from other map
	//

	template <class Other>
	void readFromOtherMap(Other const& other)
	{
		initRoot();

		std::stringstream io(std::ios_base::in | std::ios_base::out | std::ios_base::binary);
		io.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		io.imbue(std::locale());

		other.write(io);
		Octree::read(io);
	}

	//
	// Initilize root
	//

	void initRoot()
	{
		Octree::initRoot();
		(Bases<OctreeMapBase>::initRoot(), ...);
	}

	//
	// Update node
	//

	template <class NodeT, class T>
	void updateNode(NodeT& node, index_field_t const indices, T const& children)
	{
		(Bases<OctreeMapBase>::updateNode(node, indices, children), ...);
	}

	//
	// Input/output (read/write)
	//

	template <class InputIt>
	void readNodes(std::istream& in, InputIt first, InputIt last, bool const compressed)
	{
		auto cur_pos = in.tellg();
		in.seekg(0, std::ios_base::end);
		auto end_pos = in.tellg();
		in.seekg(cur_pos);
		while (in.tellg() != end_pos && in.good()) {
			DataIdentifier identifier;
			uint64_t data_size;

			in.read(reinterpret_cast<char*>(&identifier), sizeof(identifier));
			in.read(reinterpret_cast<char*>(&data_size), sizeof(data_size));

			if (!(readNodes<Bases<OctreeMapBase>>(in, first, last, identifier, data_size,
			                                      compressed) ||
			      ...)) {
				// Skip forward
				in.seekg(data_size, std::istream::cur);
			}
		}
	}

	template <class InputIt>
	void writeNodes(std::ostream& out, InputIt first, InputIt last, bool const compress,
	                int const compression_acceleration_level,
	                int const compression_level) const
	{
		(writeNodes<Bases<OctreeMapBase>>(out, first, last, compress,
		                                  compression_acceleration_level, compression_level),
		 ...);
	}

 private:
	//
	// Input/output (read/write)
	//

	template <class Base, class InputIt>
	bool readNodes(std::istream& in, InputIt first, InputIt last,
	               DataIdentifier const identifier, uint64_t const data_size,
	               bool const compressed)
	{
		if (!Base::canReadData(identifier)) {
			return false;
		}

		if (compressed) {
			std::stringstream data_stream(std::ios_base::in | std::ios_base::out |
			                              std::ios_base::binary);
			data_stream.exceptions(std::stringstream::failbit | std::stringstream::badbit);
			data_stream.imbue(std::locale());

			uint64_t compressed_data_size = 0;

			decompressData(in, data_stream, data_size, compressed_data_size);

			Base::readNodes(data_stream, first, last);
		} else {
			Base::readNodes(in, first, last);
		}

		return true;
	}

	template <class Base, class InputIt>
	void writeNodes(std::ostream& out, InputIt first, InputIt last, bool const compress,
	                int const compression_acceleration_level,
	                int const compression_level) const
	{
		constexpr DataIdentifier identifier = Base::getDataIdentifier();
		if constexpr (DataIdentifier::NO_DATA == identifier) {
			return;
		}

		out.write(reinterpret_cast<char const*>(&identifier), sizeof(identifier));

		if (compress) {
			std::stringstream buf(std::ios_base::in | std::ios_base::out |
			                      std::ios_base::binary);
			buf.exceptions(std::stringstream::failbit | std::stringstream::badbit);
			buf.imbue(std::locale());

			Base::writeNodes(buf, first, last);

			uint64_t size = buf.tellp();
			out.write(reinterpret_cast<char const*>(&size), sizeof(size));
			compressData(buf, out, size, compression_acceleration_level, compression_level);
		} else {
			uint64_t size;
			auto size_pos = out.tellp();
			out.write(reinterpret_cast<char const*>(&size), sizeof(size));

			Base::writeNodes(out, first, last);

			size = out.tellp() - size_pos - sizeof(size);
			out.seekp(size_pos);
			out.write(reinterpret_cast<char const*>(&size), sizeof(size));
			out.seekp(0, std::ios_base::end);
		}
	}
};
}  // namespace ufo::map

#endif  // UFO_MAP_BASE_H