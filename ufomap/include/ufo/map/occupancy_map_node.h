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

#ifndef UFO_MAP_OCCUPANCY_MAP_NODE_H
#define UFO_MAP_OCCUPANCY_MAP_NODE_H

// UFO
#include <ufo/map/color/color_node.h>
#include <ufo/map/occupancy/occupancy_node.h>
#include <ufo/map/semantic/semantic_node.h>

namespace ufo::map
{
template <class T>
struct OccupancyNodeColor : OccupancyNode<T>, ColorNode {
	bool operator==(OccupancyNodeColor const& rhs) const noexcept
	{
		return OccupancyNode<T>::operator==(rhs) && ColorNode::operator==(rhs);
	}

	bool operator!=(OccupancyNodeColor const& rhs) const noexcept
	{
		return OccupancyNode<T>::operator!=(rhs) || ColorNode::operator!=(rhs);
	}

	/*!
	 * @brief Write the data from this node to stream s
	 *
	 * @param s The stream to write the data to
	 * @return std::ostream&
	 */
	std::ostream& writeData(std::ostream& s) const
	{
		return s.write(reinterpret_cast<char const*>(this), sizeof(OccupancyNodeColor));
	}

	/*!
	 * @brief Read the data for this node from stream s
	 *
	 * @param s The stream to read the data from
	 * @return std::istream&
	 */
	std::istream& readData(std::istream& s)
	{
		return s.read(reinterpret_cast<char*>(this), sizeof(OccupancyNodeColor));
	}
};

struct OccupancyTimeNodeColor : OccupancyTimeNode, ColorNode {
	bool operator==(OccupancyTimeNodeColor const& rhs) const noexcept
	{
		return OccupancyTimeNode::operator==(rhs) && ColorNode::operator==(rhs);
	}

	bool operator!=(OccupancyTimeNodeColor const& rhs) const noexcept
	{
		return OccupancyTimeNode::operator!=(rhs) && ColorNode::operator!=(rhs);
	}

	/*!
	 * @brief Write the data from this node to stream s
	 *
	 * @param s The stream to write the data to
	 * @return std::ostream&
	 */
	std::ostream& writeData(std::ostream& s) const
	{
		return s.write(reinterpret_cast<char const*>(this), sizeof(OccupancyTimeNodeColor));
	}

	/*!
	 * @brief Read the data for this node from stream s
	 *
	 * @param s The stream to read the data from
	 * @return std::istream&
	 */
	std::istream& readData(std::istream& s)
	{
		return s.read(reinterpret_cast<char*>(this), sizeof(OccupancyTimeNodeColor));
	}
};

template <typename T>
struct OccupancyNodeSemantic : SemanticNode, OccupancyNode<T> {
	bool operator==(OccupancyNodeSemantic const& rhs) const noexcept
	{
		return OccupancyNode<T>::operator==(rhs) && SemanticNode::operator==(rhs);
	}

	bool operator!=(OccupancyNodeSemantic const& rhs) const noexcept
	{
		return OccupancyNode<T>::operator!=(rhs) || SemanticNode::operator!=(rhs);
	}

	/*!
	 * @brief Write the data from this node to stream s
	 *
	 * @param s The stream to write the data to
	 * @return std::ostream&
	 */
	std::ostream& writeData(std::ostream& s) const
	{
		return SemanticNode::writeData(OccupancyNode<T>::writeData(s));
	}

	/*!
	 * @brief Read the data for this node from stream s
	 *
	 * @param s The stream to read the data from
	 * @return std::istream&
	 */
	std::istream& readData(std::istream& s)
	{
		return SemanticNode::readData(OccupancyNode<T>::readData(s));
	}
};

struct OccupancyTimeNodeSemantic : SemanticNode, OccupancyTimeNode {
	bool operator==(OccupancyTimeNodeSemantic const& rhs) const noexcept
	{
		return OccupancyTimeNode::operator==(rhs) && SemanticNode::operator==(rhs);
	}

	bool operator!=(OccupancyTimeNodeSemantic const& rhs) const noexcept
	{
		return OccupancyTimeNode::operator!=(rhs) || SemanticNode::operator!=(rhs);
	}

	/*!
	 * @brief Write the data from this node to stream s
	 *
	 * @param s The stream to write the data to
	 * @return std::ostream&
	 */
	std::ostream& writeData(std::ostream& s) const
	{
		return SemanticNode::writeData(OccupancyTimeNode::writeData(s));
	}

	/*!
	 * @brief Read the data for this node from stream s
	 *
	 * @param s The stream to read the data from
	 * @return std::istream&
	 */
	std::istream& readData(std::istream& s)
	{
		return SemanticNode::readData(OccupancyTimeNode::readData(s));
	}
};

template <typename T>
struct OccupancyNodeColorSemantic : SemanticNode, OccupancyNodeColor<T> {
	bool operator==(OccupancyNodeColorSemantic const& rhs) const noexcept
	{
		return OccupancyNodeColor<T>::operator==(rhs) && SemanticNode::operator==(rhs);
	}

	bool operator!=(OccupancyNodeColorSemantic const& rhs) const noexcept
	{
		return OccupancyNodeColor<T>::operator!=(rhs) || SemanticNode::operator!=(rhs);
	}

	/*!
	 * @brief Write the data from this node to stream s
	 *
	 * @param s The stream to write the data to
	 * @return std::ostream&
	 */
	std::ostream& writeData(std::ostream& s) const
	{
		return SemanticNode::writeData(OccupancyNodeColor<T>::writeData(s));
	}

	/*!
	 * @brief Read the data for this node from stream s
	 *
	 * @param s The stream to read the data from
	 * @return std::istream&
	 */
	std::istream& readData(std::istream& s)
	{
		return SemanticNode::readData(OccupancyNodeColor<T>::readData(s));
	}
};

struct OccupancyTimeNodeColorSemantic : SemanticNode, OccupancyTimeNodeColor {
	bool operator==(OccupancyTimeNodeColorSemantic const& rhs) const noexcept
	{
		return OccupancyTimeNodeColor::operator==(rhs) && SemanticNode::operator==(rhs);
	}

	bool operator!=(OccupancyTimeNodeColorSemantic const& rhs) const noexcept
	{
		return OccupancyTimeNodeColor::operator!=(rhs) || SemanticNode::operator!=(rhs);
	}

	/*!
	 * @brief Write the data from this node to stream s
	 *
	 * @param s The stream to write the data to
	 * @return std::ostream&
	 */
	std::ostream& writeData(std::ostream& s) const
	{
		return SemanticNode::writeData(OccupancyTimeNodeColor::writeData(s));
	}

	/*!
	 * @brief Read the data for this node from stream s
	 *
	 * @param s The stream to read the data from
	 * @return std::istream&
	 */
	std::istream& readData(std::istream& s)
	{
		return SemanticNode::readData(OccupancyTimeNodeColor::readData(s));
	}
};

}  // namespace ufo::map

#endif  // UFO_MAP_OCCUPANCY_MAP_NODE_H