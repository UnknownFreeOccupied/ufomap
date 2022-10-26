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

#ifndef UFO_MAP_SIMPLE_SEMANTIC_MAP_BASE_H
#define UFO_MAP_SIMPLE_SEMANTIC_MAP_BASE_H

// UFO
#include <ufo/map/predicate/simple_semantic.h>

// STL
#include <deque>

namespace ufo::map
{
template <class Derived, class LeafNode, class InnerNode>
class SimpleSemanticMapBase
{
 public:
	//
	// Get time step
	//

	constexpr std::vector<SemanticPair>& getSimpleSemantic(Node node) noexcept
	{
		return derived().getLeafNode(node).simple_semantic;
	}

	std::vector<SemanticPair>& getSimpleSemantic(Code code)
	{
		return derived().getLeafNode(code).simple_semantic;
	}

	std::vector<SemanticPair>& getSimpleSemantic(Key key)
	{
		return getSimpleSemantic(Derived::toCode(key));
	}

	std::vector<SemanticPair>& getSimpleSemantic(Point3 coord, depth_t depth = 0)
	{
		return getSimpleSemantic(derived().toCode(coord, depth));
	}

	std::vector<SemanticPair>& getSimpleSemantic(coord_t x, coord_t y, coord_t z,
	                                             depth_t depth = 0)
	{
		return getSimpleSemantic(derived().toCode(x, y, z, depth));
	}

	constexpr std::vector<SemanticPair> const& getSimpleSemantic(Node node) const noexcept
	{
		return derived().getLeafNode(node).simple_semantic;
	}

	std::vector<SemanticPair> const& getSimpleSemantic(Code code) const
	{
		return derived().getLeafNode(code).simple_semantic;
	}

	std::vector<SemanticPair> const& getSimpleSemantic(Key key) const
	{
		return getSimpleSemantic(Derived::toCode(key));
	}

	std::vector<SemanticPair> const& getSimpleSemantic(Point3 coord,
	                                                   depth_t depth = 0) const
	{
		return getSimpleSemantic(derived().toCode(coord, depth));
	}

	std::vector<SemanticPair> const& getSimpleSemantic(coord_t x, coord_t y, coord_t z,
	                                                   depth_t depth = 0) const
	{
		return getSimpleSemantic(derived().toCode(x, y, z, depth));
	}

 protected:
	//
	// Derived
	//

	constexpr Derived& derived() { return *static_cast<Derived*>(this); }

	constexpr Derived const& derived() const { return *static_cast<Derived const*>(this); }

	//
	// Initilize root
	//

	void initRoot() { derived().getRoot().simple_semantic.clear(); }

	//
	// Update node
	//

	void updateNode(InnerNode& node, depth_t depth)
	{
		node.simple_semantic.clear();
		if (1 == depth) {
			std::size_t total_size = 0;
			for (auto const& child : derived().getLeafChildren(node)) {
				total_size += child.simple_semantic.size();
			}
			if (0 != total_size) {
				node.simple_semantic.resize(total_size);
				auto it = node.simple_semantic.data();
				for (auto const& child : derived().getLeafChildren(node)) {
					it = std::copy(child.simple_semantic.data(),
					               child.simple_semantic.data() + child.simple_semantic.size(), it);
				}
			}
		} else {
			std::size_t total_size = 0;
			for (auto const& child : derived().getInnerChildren(node)) {
				total_size += child.simple_semantic.size();
			}
			if (0 != total_size) {
				node.simple_semantic.resize(total_size);
				auto it = node.simple_semantic.data();
				for (auto const& child : derived().getInnerChildren(node)) {
					it = std::copy(child.simple_semantic.data(),
					               child.simple_semantic.data() + child.simple_semantic.size(), it);
				}
			}
		}
		if (!node.simple_semantic.empty()) {
			std::sort(std::begin(node.simple_semantic), std::end(node.simple_semantic),
			          [](auto a, auto b) {
				          return a.label < b.label || (a.label == b.label && a.value > b.value);
			          });
			auto last =
			    std::unique(std::begin(node.simple_semantic), std::end(node.simple_semantic),
			                [](auto a, auto b) { return a.label == b.label; });
			node.simple_semantic.resize(std::distance(std::begin(node.simple_semantic), last));
		}
		// node.simple_semantic.shrink_to_fit();
	}

	//
	// Input/output (read/write)
	//

	static constexpr DataIdentifier getDataIdentifier() noexcept
	{
		return DataIdentifier::SIMPLE_SEMANTIC;
	}

	static constexpr bool canReadData(DataIdentifier identifier) noexcept
	{
		return getDataIdentifier() == identifier;
	}

	void readNodes(std::istream& in_stream, std::vector<LeafNode*> const& nodes)
	{
		auto const num_nodes = nodes.size();

		std::uint64_t total_size;
		in_stream.read(reinterpret_cast<char*>(&total_size), sizeof(total_size));

		auto data = std::make_unique<SemanticPair[]>(num_nodes + total_size);
		in_stream.read(reinterpret_cast<char*>(data.get()),
		               (num_nodes + total_size) * sizeof(SemanticPair));

		auto it = data.get();
		for (auto node : nodes) {
			auto size = it->label;
			++it;
			node->simple_semantic.resize(size);
			if (0 != size) {
				std::copy(it, it + size, node->simple_semantic.data());
				it += size;
			}
			// node->simple_semantic.shrink_to_fit();
		}
	}

	void writeNodes(std::ostream& out_stream, std::vector<LeafNode> const& nodes) const
	{
		auto num_nodes = nodes.size();

		std::uint64_t total_size = 0;
		for (auto const& node : nodes) {
			total_size += node.simple_semantic.size();
		}

		auto data = std::make_unique<SemanticPair[]>(num_nodes + total_size);
		auto it = data.get();
		std::size_t i = 0;
		for (auto const& node : nodes) {
			it->label = node.simple_semantic.size();
			++it;
			if (0 != it->label) {
				it = std::copy(node.simple_semantic.data(),
				               node.simple_semantic.data() + node.simple_semantic.size(), it);
			}
		}

		out_stream.write(reinterpret_cast<char const*>(&total_size), sizeof(total_size));
		out_stream.write(reinterpret_cast<char const*>(data.get()),
		                 (num_nodes + total_size) * sizeof(SemanticPair));
	}

 protected:
	// Propagation criteria
	PropagationCriteria time_step_prop_criteria_ = PropagationCriteria::MAX;
};
}  // namespace ufo::map

#endif  // UFO_MAP_SIMPLE_SEMANTIC_MAP_BASE_H