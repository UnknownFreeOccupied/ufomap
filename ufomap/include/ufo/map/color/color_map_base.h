/*
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * License: BSD 3
 */

/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, D. Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
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
#include <ufo/map/map_base.h>
#include <ufo/map/octree/octree_base.h>

// STL
#include <type_traits>

namespace ufo::map
{
template <class Derived, class DataType = ColorNode, class Indicators = OctreeIndicators>
class ColorMapBase : virtual public OctreeBase<Derived, DataType, Indicators>
{
 protected:
	using Base = OctreeBase<Derived, DataType, Indicators>;
	using LeafNode = typename Base::LeafNode;
	using InnerNode = typename Base::InnerNode;

	static_assert(std::is_base_of_v<ColorNode, LeafNode>);

 public:
	//
	// Get color
	//

	Color getColor(MinimalNode const& node) const noexcept
	{
		return getColor(Base::getLeafNode(node));
	}

	Color getColor(Code code) const { return getColor(Base::getLeafNode(code)); }

	Color getColor(Key key) const { return getColor(Base::toCode(key)); }

	Color getColor(Point3 coord, DepthType depth = 0) const
	{
		return getColor(Base::toCode(coord, depth));
	}

	Color getColor(float x, float y, float z, DepthType depth = 0) const
	{
		return getColor(Base::toCode(x, y, z, depth));
	}

	//
	// Set color
	//

	void setColor(MinimalNode& node, Color const& color, bool propagate = true)
	{
		Base::apply(
		    node, [this, &color](LeafNode& node) { setColorImpl(node, color); }, propagate);
	}

	void setColor(Code code, Color const& color, bool propagate = true)
	{
		Base::apply(
		    code, [this, color](LeafNode& node) { setColorImpl(node, color); }, propagate);
	}

	void setColor(Key key, Color const& color, bool propagate = true)
	{
		setColor(Base::toCode(key), color, propagate);
	}

	void setColor(Point3 coord, Color const& color, bool propagate = true,
	              DepthType depth = 0)
	{
		setColor(Base::toCode(coord, depth), color, propagate);
	}

	void setColor(float x, float y, float z, Color const& color, bool propagate = true,
	              DepthType depth = 0)
	{
		setColor(Base::toCode(x, y, z, depth), color, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setColor(ExecutionPolicy policy, MinimalNode& node, Color const& color,
	              bool propagate = true)
	{
		Base::apply(
		    policy, node, [this, color](LeafNode& node) { setColorImpl(node, color); },
		    propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setColor(ExecutionPolicy policy, Code code, Color const& color,
	              bool propagate = true)
	{
		Base::apply(
		    policy, code, [this, color](LeafNode& node) { setColorImpl(node, color); },
		    propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setColor(ExecutionPolicy policy, Key key, Color const& color,
	              bool propagate = true)
	{
		setColor(policy, Base::toCode(key), color, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setColor(ExecutionPolicy policy, Point3 coord, Color const& color,
	              bool propagate = true, DepthType depth = 0)
	{
		setColor(policy, Base::toCode(coord, depth), color, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void setColor(ExecutionPolicy policy, float x, float y, float z, Color const& color,
	              bool propagate = true, DepthType depth = 0)
	{
		setColor(policy, Base::toCode(x, y, z, depth), color, propagate);
	}

	//
	// Update color
	//

	void updateColor(MinimalNode& node, Color const& color, float weight,
	                 bool propagate = true)
	{
		Base::apply(
		    node,
		    [this, color, weight](LeafNode& node) { updateColorImpl(node, color, weight); },
		    propagate);
	}

	void updateColor(Code code, Color const& color, float weight, bool propagate = true)
	{
		Base::apply(
		    code,
		    [this, color, weight](LeafNode& node) { updateColorImpl(node, color, weight); },
		    propagate);
	}

	void updateColor(Key key, Color const& color, float weight, bool propagate = true)
	{
		updateColor(Base::toCode(key), color, weight, propagate);
	}

	void updateColor(Point3 coord, Color const& color, float weight, bool propagate = true,
	                 DepthType depth = 0)
	{
		updateColor(Base::toCode(coord, depth), color, weight, propagate);
	}

	void updateColor(float x, float y, float z, Color const& color, float weight,
	                 bool propagate = true, DepthType depth = 0)
	{
		updateColor(Base::toCode(x, y, z, depth), color, weight, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void updateColor(ExecutionPolicy policy, MinimalNode& node, Color const& color,
	                 float weight, bool propagate = true)
	{
		Base::apply(
		    policy, node,
		    [this, color, weight](LeafNode& node) { updateColorImpl(node, color, weight); },
		    propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void updateColor(ExecutionPolicy policy, Code code, Color const& color, float weight,
	                 bool propagate = true)
	{
		Base::apply(
		    policy, code,
		    [this, color, weight](LeafNode& node) { updateColorImpl(node, color, weight); },
		    propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void updateColor(ExecutionPolicy policy, Key key, Color const& color, float weight,
	                 bool propagate = true)
	{
		updateColor(policy, Base::toCode(key), color, weight, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void updateColor(ExecutionPolicy policy, Point3 coord, Color const& color, float weight,
	                 bool propagate = true, DepthType depth = 0)
	{
		updateColor(policy, Base::toCode(coord, depth), color, weight, propagate);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void updateColor(ExecutionPolicy policy, float x, float y, float z, Color const& color,
	                 float weight, bool propagate = true, DepthType depth = 0)
	{
		updateColor(policy, Base::toCode(x, y, z, depth), color, weight, propagate);
	}

 protected:
	//
	// Constructor
	//

	// using Base::Base;

	ColorMapBase(double resolution, DepthType depth_levels, bool automatic_pruning)
	    : Base(resolution, depth_levels, automatic_pruning)
	{
		printf("ColorMapBase constructor\n");
	}

	ColorMapBase(ColorMapBase const& other) : Base(other)
	{
		printf("ColorMapBase copy constructor\n");
	}

	template <class D1, class D2, class I>
	ColorMapBase(ColorMapBase<D1, D2, I> const& other) : Base(other)
	{
		printf("ColorMapBase template copy constructor\n");
	}

	ColorMapBase(ColorMapBase&& other) : Base(std::move(other))
	{
		printf("ColorMapBase move constructor\n");
	}

	ColorMapBase& operator=(ColorMapBase const& rhs)
	{
		printf("ColorMapBase copy assignment\n");
		Base::operator=(rhs);
	}

	template <class D1, class D2, class I>
	ColorMapBase& operator=(ColorMapBase<D1, D2, I> const& rhs)
	{
		printf("ColorMapBase template copy assignment\n");
		Base::operator=(rhs);
	}

	ColorMapBase& operator=(ColorMapBase&& rhs)
	{
		printf("ColorMapBase move assignment\n");
		Base::operator=(std::move(rhs));
	}

	//
	// Destructor
	//

	virtual ~ColorMapBase() override {}

	//
	// Initilize root
	//

	virtual void initRoot() override
	{
		printf("ColorMapBase initRoot\n");
		Base::initRoot();
		Base::getRoot().color.clear();
	}

	//
	// Update node
	//

	virtual void updateNode(InnerNode& node, DepthType depth) override
	{
		node.color.setColor(averageChildColor(node, depth));
	}

	//
	// Input/output (read/write)
	//

	virtual void addFileInfo(FileInfo& info) const override
	{
		info["fields"].emplace_back("color");
		info["type"].emplace_back("U");
		info["size"].emplace_back(std::to_string(sizeof(Color)));
	}

	virtual bool readNodes(std::istream& in_stream, std::vector<LeafNode*> const& nodes,
	                       std::string const& field, char type, uint64_t size,
	                       uint64_t num) override
	{
		if ("color" != field) {
			return false;
		}
		// TODO: Make parallel

		if ('U' == type && sizeof(Color) == size) {
			std::vector<Color> data(nodes.size());
			in_stream.read(reinterpret_cast<char*>(data.data()),
			               data.size() * sizeof(typename decltype(data)::value_type));

			for (size_t i = 0; i != data.size(); ++i) {
				nodes[i]->color = data[i];
			}
		} else {
			// TODO: Error
			return false;
		}
		return true;
	}

	virtual void writeNodes(std::ostream& out_stream,
	                        std::vector<LeafNode const*> const& nodes, bool compress,
	                        int compression_acceleration_level,
	                        int compression_level) const override
	{
		std::vector<Color> data(nodes.size());
		for (size_t i = 0; i != nodes.size(); ++i) {
			data[i] = nodes[i]->color;
		}

		uint64_t size = data.size();
		out_stream.write(reinterpret_cast<char const*>(&size), sizeof(uint64_t));
		out_stream.write(reinterpret_cast<char const*>(data.data()),
		                 data.size() * sizeof(typename decltype(data)::value_type));
	}

	//
	// Get color
	//

	Color getColor(LeafNode const& node) const { return node.color; }

	//
	// Set color
	//

	static constexpr void setColorImpl(LeafNode& node, Color const& color)
	{
		node.color.setColor(color);
	}

	//
	// Update color
	//

	static void updateColorImpl(LeafNode& node, Color const& color, float weight)
	{
		if (!node.color.isSet()) {
			node.color = color;
		} else {
			node.color = Color::averageColor({node.color, color}, {1.0 - weight, weight});
		}
	}

	//
	// Average child color
	//

	Color averageChildColor(InnerNode const& node, DepthType depth) const
	{
		std::vector<Color> colors;
		colors.reserve(8);

		if (1 == depth) {
			for (auto const& child : Base::getLeafChildren(node)) {
				colors.push_back(child.color);
			}
		} else {
			for (auto const& child : Base::getInnerChildren(node)) {
				colors.push_back(child.color);
			}
		}

		return Color::averageColor(colors);
	}

	template <class D1, class D2, class I>
	friend class ColorMapBase;
};
}  // namespace ufo::map

#endif  // UFO_MAP_COLOR_MAP_BASE_H