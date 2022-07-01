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
template <class Node, class Indicators, bool LockLess, MemoryModel NodeMemoryModel,
          depth_t StaticallyAllocatedDepths,
          template <typename, typename, typename> typename... Bases>
class OctreeMapBase
    : public OctreeBase<OctreeMapBase<Node, Indicators, LockLess, NodeMemoryModel,
                                      StaticallyAllocatedDepths, Bases...>,
                        Node, Indicators, LockLess, NodeMemoryModel,
                        StaticallyAllocatedDepths>,
      public Bases<
          OctreeMapBase<Node, Indicators, LockLess, NodeMemoryModel,
                        StaticallyAllocatedDepths, Bases...>,
          typename OctreeBase<OctreeMapBase<Node, Indicators, LockLess, NodeMemoryModel,
                                            StaticallyAllocatedDepths, Bases...>,
                              Node, Indicators, LockLess, NodeMemoryModel,
                              StaticallyAllocatedDepths>::LeafNode,
          typename OctreeBase<OctreeMapBase<Node, Indicators, LockLess, NodeMemoryModel,
                                            StaticallyAllocatedDepths, Bases...>,
                              Node, Indicators, LockLess, NodeMemoryModel,
                              StaticallyAllocatedDepths>::InnerNode>...
{
 protected:
	//
	// Tags
	//

	using Octree = OctreeBase<OctreeMapBase, Node, Indicators, LockLess, NodeMemoryModel,
	                          StaticallyAllocatedDepths>;
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
		other.write(io_stream);
		Octree::read(io_stream);
	}

	template <class Node2, class Indicators2, bool LockLess2, MemoryModel NodeMemoryModel2,
	          depth_t StaticallyAllocatedDepths2,
	          template <typename, typename, typename> typename... Bases2>
	OctreeMapBase(OctreeMapBase<Node2, Indicators2, LockLess2, NodeMemoryModel2,
	                            StaticallyAllocatedDepths2, Bases2...> const& other)
	    : Octree(other.resolution(), other.depthLevels(), other.automaticPruning())
	{
		initRoot();

		std::stringstream io_stream(std::ios_base::in | std::ios_base::out |
		                            std::ios_base::binary);
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
		rhs.write(io_stream);
		Octree::read(io_stream);

		return *this;
	}

	template <class Node2, class Indicators2, bool LockLess2, MemoryModel NodeMemoryModel2,
	          depth_t StaticallyAllocatedDepths2,
	          template <typename, typename, typename> typename... Bases2>
	OctreeMapBase& operator=(
	    OctreeMapBase<Node2, Indicators2, LockLess2, NodeMemoryModel2,
	                  StaticallyAllocatedDepths2, Bases2...> const& rhs)
	{
		initRoot();

		std::stringstream io_stream(std::ios_base::in | std::ios_base::out |
		                            std::ios_base::binary);
		rhs.write(io_stream);
		Octree::read(io_stream);

		return *this;
	}

	OctreeMapBase& operator=(OctreeMapBase&& rhs) = default;

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

	void addFileInfo(FileInfo& info) const
	{
		(Bases<OctreeMapBase, LeafNode, InnerNode>::addFileInfo(info), ...);
	}

	bool readNodes(std::istream& in_stream, std::vector<LeafNode*> const& nodes,
	               std::string const& field, char type, uint64_t size, uint64_t num)
	{
		return (Bases<OctreeMapBase, LeafNode, InnerNode>::readNodes(in_stream, nodes, field,
		                                                             type, size, num) ||
		        ...);
	}

	void writeNodes(std::ostream& out_stream, std::vector<LeafNode const*> const& nodes,
	                bool compress, int compression_acceleration_level,
	                int compression_level) const
	{
		(Bases<OctreeMapBase, LeafNode, InnerNode>::writeNodes(
		     out_stream, nodes, compress, compression_acceleration_level, compression_level),
		 ...);
	}
};
}  // namespace ufo::map

#endif  // UFO_MAP_BASE_H