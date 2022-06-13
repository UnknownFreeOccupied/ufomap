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

#ifndef UFOMAP_ROS_MSGS_CONVERSIONS_H
#define UFOMAP_ROS_MSGS_CONVERSIONS_H

// UFO ROS
#include <ufomap_msgs/UFOMap.h>

// STL
#include <sstream>
#include <type_traits>

namespace ufomap_msgs
{
//
// ROS message type to UFO type
//

template <class Map>
void msgToUfo(ufomap_msgs::UFOMap const& msg, Map& map, bool propagate = true)
{
	if (msg.data.empty()) {
		return;
	}

	std::stringstream data(std::ios_base::in | std::ios_base::out | std::ios_base::binary);
	data.exceptions(std::stringstream::failbit | std::stringstream::badbit);

	data.write(reinterpret_cast<char const*>(msg.data.data()),
	           sizeof(typename decltype(msg.data)::value_type) * msg.data.size());

	map.read(data, propagate);
}

//
// UFO type to ROS message type
//

template <class Map, class Predicates,
          typename = std::enable_if_t<!std::is_scalar_v<Predicates>>>
ufomap_msgs::UFOMap ufoToMsg(Map const& map, Predicates const& predicates,
                             unsigned int depth = 0, bool compress = false,
                             int compression_acceleration_level = 1,
                             int compression_level = 0)
{
	std::stringstream data(std::ios_base::in | std::ios_base::out | std::ios_base::binary);
	data.exceptions(std::stringstream::failbit | std::stringstream::badbit);

	map.write(data, predicates, depth, compress, compression_acceleration_level,
	          compression_level);

	ufomap_msgs::UFOMap msg;
	msg.data.resize(data.tellp());
	data.seekg(0, std::ios_base::beg);
	data.read(reinterpret_cast<char*>(msg.data.data()), msg.data.size());
	return msg;
}

template <class Map>
ufomap_msgs::UFOMap ufoToMsg(Map const& map, unsigned int depth = 0,
                             bool compress = false,
                             int compression_acceleration_level = 1,
                             int compression_level = 0)
{
	std::stringstream data(std::ios_base::in | std::ios_base::out | std::ios_base::binary);
	data.exceptions(std::stringstream::failbit | std::stringstream::badbit);

	map.write(data, depth, compress, compression_acceleration_level, compression_level);

	ufomap_msgs::UFOMap msg;
	msg.data.resize(data.tellp());
	data.seekg(0, std::ios_base::beg);
	data.read(reinterpret_cast<char*>(msg.data.data()), msg.data.size());
	return msg;
}

}  // namespace ufomap_msgs

#endif  // UFOMAP_ROS_MSGS_CONVERSIONS_H