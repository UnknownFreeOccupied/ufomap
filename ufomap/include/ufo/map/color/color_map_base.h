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

#ifndef UFO_MAP_COLOR_MAP_BASE_H
#define UFO_MAP_COLOR_MAP_BASE_H

// UFO
#include <ufo/map/color/color_node.h>
#include <ufo/map/octree/octree_base.h>

// STL
#include <type_traits>

namespace ufo::map
{
template <class Derived, class LeafNode, class InnerNode>
class ColorMapBase
{
 protected:
	static_assert(std::is_base_of_v<LeafNode, InnerNode>);
	static_assert(std::is_base_of_v<ColorNode, LeafNode>);

 public:
	//
	// Get color
	//

	constexpr RGBColor getColor(Node node) const
	{
		return getColor(derived().getLeafNode(node));
	}

	constexpr RGBColor getColor(Code code) const
	{
		return getColor(derived().getLeafNode(code));
	}

	constexpr RGBColor getColor(Key key) const { return getColor(Derived::toCode(key)); }

	constexpr RGBColor getColor(Point3 coord, depth_t depth = 0) const
	{
		return getColor(derived().toCode(coord, depth));
	}

	constexpr RGBColor getColor(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return getColor(derived().toCode(x, y, z, depth));
	}

	//
	// Set color
	//

	constexpr void setColor(Node& node, RGBColor color, bool propagate = true)
	{
		derived().apply(
		    node, [this, &color](LeafNode& node) { setColor(node, color); }, propagate);
	}

	constexpr void setColor(Code code, RGBColor color, bool propagate = true)
	{
		derived().apply(
		    code, [this, color](LeafNode& node) { setColor(node, color); }, propagate);
	}

	constexpr void setColor(Key key, RGBColor color, bool propagate = true)
	{
		setColor(Derived::toCode(key), color, propagate);
	}

	constexpr void setColor(Point3 coord, RGBColor color, bool propagate = true,
	                        depth_t depth = 0)
	{
		setColor(derived().toCode(coord, depth), color, propagate);
	}

	constexpr void setColor(coord_t x, coord_t y, coord_t z, RGBColor color,
	                        bool propagate = true, depth_t depth = 0)
	{
		setColor(derived().toCode(x, y, z, depth), color, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	constexpr void setColor(ExecutionPolicy policy, Node& node, RGBColor color,
	                        bool propagate = true)
	{
		derived().apply(
		    policy, node, [this, color](LeafNode& node) { setColor(node, color); },
		    propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	constexpr void setColor(ExecutionPolicy policy, Code code, RGBColor color,
	                        bool propagate = true)
	{
		derived().apply(
		    policy, code, [this, color](LeafNode& node) { setColor(node, color); },
		    propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	constexpr void setColor(ExecutionPolicy policy, Key key, RGBColor color,
	                        bool propagate = true)
	{
		setColor(policy, Derived::toCode(key), color, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	constexpr void setColor(ExecutionPolicy policy, Point3 coord, RGBColor color,
	                        bool propagate = true, depth_t depth = 0)
	{
		setColor(policy, derived().toCode(coord, depth), color, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	constexpr void setColor(ExecutionPolicy policy, coord_t x, coord_t y, coord_t z,
	                        RGBColor color, bool propagate = true, depth_t depth = 0)
	{
		setColor(policy, derived().toCode(x, y, z, depth), color, propagate);
	}

	//
	// Clear color
	//

	constexpr void clearColor(Node& node, bool propagate = true)
	{
		derived().apply(
		    node, [this](LeafNode& node) { clearColor(node); }, propagate);
	}

	constexpr void clearColor(Code code, bool propagate = true)
	{
		derived().apply(
		    code, [this](LeafNode& node) { clearColor(node); }, propagate);
	}

	constexpr void clearColor(Key key, bool propagate = true)
	{
		clearColor(Derived::toCode(key), propagate);
	}

	constexpr void setColor(Point3 coord, bool propagate = true, depth_t depth = 0)
	{
		clearColor(derived().toCode(coord, depth), propagate);
	}

	constexpr void clearColor(coord_t x, coord_t y, coord_t z, bool propagate = true,
	                          depth_t depth = 0)
	{
		clearColor(derived().toCode(x, y, z, depth), propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	constexpr void clearColor(ExecutionPolicy policy, Node& node, bool propagate = true)
	{
		derived().apply(
		    policy, node, [this](LeafNode& node) { clearColor(node); }, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	constexpr void clearColor(ExecutionPolicy policy, Code code, bool propagate = true)
	{
		derived().apply(
		    policy, code, [this](LeafNode& node) { clearColor(node); }, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	constexpr void clearColor(ExecutionPolicy policy, Key key, bool propagate = true)
	{
		clearColor(policy, Derived::toCode(key), propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	constexpr void clearColor(ExecutionPolicy policy, Point3 coord, bool propagate = true,
	                          depth_t depth = 0)
	{
		clearColor(policy, derived().toCode(coord, depth), propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	constexpr void clearColor(ExecutionPolicy policy, coord_t x, coord_t y, coord_t z,
	                          bool propagate = true, depth_t depth = 0)
	{
		clearColor(policy, derived().toCode(x, y, z, depth), propagate);
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

	void initRoot() { clearColor(derived().getRoot()); }

	//
	// Get color
	//

	static constexpr RGBColor getColor(LeafNode const& node) noexcept { return node.color; }

	//
	// Set color
	//

	static constexpr void setColor(LeafNode& node, RGBColor color) noexcept
	{
		node.color = color;
	}

	//
	// Clear color
	//

	static constexpr void clearColor(LeafNode& node) noexcept { node.color.clear(); }

	//
	// Update node
	//

	void updateNode(InnerNode& node, depth_t depth)
	{
		setColor(node, averageChildColor(node, depth));
	}

	//
	// Average child color
	//

	constexpr RGBColor averageChildColor(InnerNode const& node, depth_t depth) const
	{
		return 1 == depth ? RGBColor::average({derived().getLeafChild(node, 0).color,
		                                       derived().getLeafChild(node, 1).color,
		                                       derived().getLeafChild(node, 2).color,
		                                       derived().getLeafChild(node, 3).color,
		                                       derived().getLeafChild(node, 4).color,
		                                       derived().getLeafChild(node, 5).color,
		                                       derived().getLeafChild(node, 6).color,
		                                       derived().getLeafChild(node, 7).color})
		                  : RGBColor::average({derived().getInnerChild(node, 0).color,
		                                       derived().getInnerChild(node, 1).color,
		                                       derived().getInnerChild(node, 2).color,
		                                       derived().getInnerChild(node, 3).color,
		                                       derived().getInnerChild(node, 4).color,
		                                       derived().getInnerChild(node, 5).color,
		                                       derived().getInnerChild(node, 6).color,
		                                       derived().getInnerChild(node, 7).color});
	}

	//
	// Input/output (read/write)
	//

	void addFileInfo(FileInfo& info) const
	{
		info["fields"].emplace_back("color");
		info["type"].emplace_back("U");
		info["size"].emplace_back(std::to_string(sizeof(RGBColor)));
	}

	bool readNodes(std::istream& in_stream, std::vector<LeafNode*> const& nodes,
	               std::string const& field, char type, uint64_t size, uint64_t num)
	{
		if ("color" != field) {
			return false;
		}

		if ('U' == type && sizeof(RGBColor) == size) {
			auto data = std::make_unique<RGBColor[]>(nodes.size());
			in_stream.read(reinterpret_cast<char*>(data.get()),
			               nodes.size() * sizeof(RGBColor));

			for (size_t i = 0; i != nodes.size(); ++i) {
				setColor(*nodes[i], data[i]);
			}
		} else {
			return false;
		}
		return true;
	}

	void writeNodes(std::ostream& out_stream, std::vector<LeafNode> const& nodes,
	                bool compress, int compression_acceleration_level,
	                int compression_level) const
	{
		uint64_t const size = nodes.size();
		out_stream.write(reinterpret_cast<char const*>(&size), sizeof(uint64_t));

		auto data = std::make_unique<RGBColor[]>(nodes.size());
		for (size_t i = 0; i != nodes.size(); ++i) {
			data[i] = getColor(nodes[i]);
		}

		out_stream.write(reinterpret_cast<char const*>(data.get()),
		                 nodes.size() * sizeof(RGBColor));
	}
};
}  // namespace ufo::map

#endif  // UFO_MAP_COLOR_MAP_BASE_H