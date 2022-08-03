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

// STL
#include <deque>

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
template <class Node, class Indicators, bool ReuseNodes, bool LockLess,
          template <typename, typename, typename> typename... Bases>
class OctreeMapBase
    : public OctreeBase<OctreeMapBase<Node, Indicators, ReuseNodes, LockLess, Bases...>,
                        Node, Indicators, ReuseNodes, LockLess>,
      public Bases<OctreeMapBase<Node, Indicators, ReuseNodes, LockLess, Bases...>,
                   typename OctreeBase<
                       OctreeMapBase<Node, Indicators, ReuseNodes, LockLess, Bases...>,
                       Node, Indicators, ReuseNodes, LockLess>::LeafNode,
                   typename OctreeBase<
                       OctreeMapBase<Node, Indicators, ReuseNodes, LockLess, Bases...>,
                       Node, Indicators, ReuseNodes, LockLess>::InnerNode>...
{
 protected:
	//
	// Tags
	//

	using Octree = OctreeBase<OctreeMapBase, Node, Indicators, ReuseNodes, LockLess>;
	using LeafNode = typename Octree::LeafNode;
	using InnerNode = typename Octree::InnerNode;

	//
	// Friends
	//

	friend Octree;
#define FRIEND(N)                                                  \
	friend std::tuple_element_t<                                     \
	    std::min(static_cast<std::size_t>(N + 1), sizeof...(Bases)), \
	    std::tuple<void, Bases<OctreeMapBase, LeafNode, InnerNode>...>>;
	REPEAT_128(FRIEND, 0)

 public:
	//
	// Constructors
	//

	OctreeMapBase(double resolution = 0.1, depth_t depth_levels = 16,
	              bool automatic_pruning = true)
	    : Octree(resolution, depth_levels, automatic_pruning)
	{
		initRoot();
	}

	OctreeMapBase(std::filesystem::path const& filename, bool automatic_pruning = true)
	    : OctreeMapBase(0.1, 16, automatic_pruning)
	{
		Octree::read(filename);
	}

	OctreeMapBase(std::istream& in_stream, bool automatic_pruning = true)
	    : OctreeMapBase(0.1, 16, automatic_pruning)
	{
		Octree::read(in_stream);
	}

	OctreeMapBase(OctreeMapBase const& other)
	    : Octree(other), Bases<OctreeMapBase, LeafNode, InnerNode>(other)...
	{
		initRoot();

		std::stringstream io_stream(std::ios_base::in | std::ios_base::out |
		                            std::ios_base::binary);
		io_stream.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		io_stream.imbue(std::locale());

		other.write(io_stream);
		Octree::read(io_stream);
	}

	template <class Node2, class Indicators2, bool ReuseNodes2, bool LockLess2,
	          template <typename, typename, typename> typename... Bases2>
	OctreeMapBase(
	    OctreeMapBase<Node2, Indicators2, ReuseNodes2, LockLess2, Bases2...> const& other)
	    : Octree(other.resolution(), other.depthLevels(), other.automaticPruning())
	{
		initRoot();

		std::stringstream io_stream(std::ios_base::in | std::ios_base::out |
		                            std::ios_base::binary);
		io_stream.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		io_stream.imbue(std::locale());

		other.write(io_stream);
		Octree::read(io_stream);
	}

	OctreeMapBase(OctreeMapBase&& other) = default;

	OctreeMapBase& operator=(OctreeMapBase const& rhs)
	{
		Octree::operator=(rhs);
		(Bases<OctreeMapBase, LeafNode, InnerNode>::operator=(rhs), ...);

		initRoot();

		std::stringstream io_stream(std::ios_base::in | std::ios_base::out |
		                            std::ios_base::binary);
		io_stream.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		io_stream.imbue(std::locale());

		rhs.write(io_stream);
		Octree::read(io_stream);

		return *this;
	}

	template <class Node2, class Indicators2, bool ReuseNodes2, bool LockLess2,
	          template <typename, typename, typename> typename... Bases2>
	OctreeMapBase& operator=(
	    OctreeMapBase<Node2, Indicators2, ReuseNodes2, LockLess2, Bases2...> const& rhs)
	{
		initRoot();

		std::stringstream io_stream(std::ios_base::in | std::ios_base::out |
		                            std::ios_base::binary);
		io_stream.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		io_stream.imbue(std::locale());

		rhs.write(io_stream);
		Octree::read(io_stream);

		return *this;
	}

	OctreeMapBase& operator=(OctreeMapBase&& rhs) = default;

 protected:
	//
	// Initilize root
	//

	void initRoot()
	{
		Octree::initRoot();
		(Bases<OctreeMapBase, LeafNode, InnerNode>::initRoot(), ...);
	}

	//
	// Update node
	//

	void updateNode(InnerNode& node, depth_t depth)
	{
		(Bases<OctreeMapBase, LeafNode, InnerNode>::updateNode(node, depth), ...);
	}

	//
	// Input/output (read/write)
	//

	static constexpr bool canReadData(DataIdentifier identifier) noexcept
	{
		return (Bases<OctreeMapBase, LeafNode, InnerNode>::canReadData(identifier) || ...);
	}

	void readNodes(std::istream& in_stream, std::vector<LeafNode*> const& nodes,
	               bool const compressed)
	{
		auto cur_pos = in_stream.tellg();
		in_stream.seekg(0, std::ios_base::end);
		auto end_pos = in_stream.tellg();
		in_stream.seekg(cur_pos);
		while (in_stream.tellg() != end_pos && in_stream.good()) {
			DataIdentifier identifier;
			in_stream.read(reinterpret_cast<char*>(&identifier), sizeof(identifier));
			uint64_t data_size;
			in_stream.read(reinterpret_cast<char*>(&data_size), sizeof(data_size));

			if (!(readNodes<Bases<OctreeMapBase, LeafNode, InnerNode>>(
			          in_stream, nodes, identifier, data_size, compressed) ||
			      ...)) {
				// Skip forward
				in_stream.seekg(data_size, std::istream::cur);
			}
		}
	}

	void writeNodes(std::ostream& out_stream, std::vector<LeafNode> const& nodes,
	                bool const compress, int const compression_acceleration_level,
	                int const compression_level) const
	{
		(writeNodes<Bases<OctreeMapBase, LeafNode, InnerNode>>(
		     out_stream, nodes, compress, compression_acceleration_level, compression_level),
		 ...);
	}

 private:
	//
	// Input/output (read/write)
	//

	template <class Base>
	bool readNodes(std::istream& in_stream, std::vector<LeafNode*> const& nodes,
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

			decompressData(in_stream, data_stream, data_size, compressed_data_size);

			Base::readNodes(data_stream, nodes);
		} else {
			Base::readNodes(in_stream, nodes);
		}

		return true;
	}

	template <class Base>
	void writeNodes(std::ostream& out_stream, std::vector<LeafNode> const& nodes,
	                bool const compress, int const compression_acceleration_level,
	                int const compression_level) const
	{
		constexpr DataIdentifier identifier = Base::getDataIdentifier();
		if constexpr (DataIdentifier::NO_DATA != identifier) {
			out_stream.write(reinterpret_cast<char const*>(&identifier), sizeof(identifier));

			if (compress) {
				std::stringstream data_stream(std::ios_base::in | std::ios_base::out |
				                              std::ios_base::binary);
				data_stream.exceptions(std::stringstream::failbit | std::stringstream::badbit);
				data_stream.imbue(std::locale());

				Base::writeNodes(data_stream, nodes);

				uint64_t size = data_stream.tellp();
				out_stream.write(reinterpret_cast<char const*>(&size), sizeof(size));
				compressData(data_stream, out_stream, size, compression_acceleration_level,
				             compression_level);
			} else {
				uint64_t size;
				auto size_pos = out_stream.tellp();
				out_stream.write(reinterpret_cast<char const*>(&size), sizeof(size));

				Base::writeNodes(out_stream, nodes);

				size = out_stream.tellp() - size_pos - sizeof(size);
				out_stream.seekp(size_pos);
				out_stream.write(reinterpret_cast<char const*>(&size), sizeof(size));
				out_stream.seekp(0, std::ios_base::end);
			}
		}
	}
};
}  // namespace ufo::map

#endif  // UFO_MAP_BASE_H