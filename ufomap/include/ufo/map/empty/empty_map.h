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

#ifndef UFO_MAP_EMPTY_MAP_H
#define UFO_MAP_EMPTY_MAP_H

// UFO
#include <ufo/map/empty/empty_node.h>
#include <ufo/map/types.h>

// STL
#include <cstdint>
#include <iostream>

namespace ufo::map
{
template <std::uint64_t Num, class Derived>
class EmptyMap
{
 protected:
	//
	// Constructors
	//

	EmptyMap() = default;

	EmptyMap(EmptyMap const&) = default;

	EmptyMap(EmptyMap&&) = default;

	template <std::uint64_t Num2, class Derived2>
	EmptyMap(EmptyMap<Num2, Derived2> const&)
	{
	}

	//
	// Assignment operator
	//

	EmptyMap& operator=(EmptyMap const&) = default;

	EmptyMap& operator=(EmptyMap&&) = default;

	template <std::uint64_t Num2, class Derived2>
	EmptyMap& operator=(EmptyMap<Num2, Derived2> const&)
	{
		return *this;
	}

	//
	// Swap
	//

	void swap(EmptyMap&) noexcept {}

	//
	// Initilize root
	//

	static constexpr void initRoot() noexcept {}

	//
	// Update node
	//

	template <class T, class InputIt>
	static constexpr void updateNode(T const&, IndexField const, InputIt, InputIt) noexcept
	{
	}

	//
	// Input/ouput (read/write)
	//

	static constexpr DataIdentifier dataIdentifier() noexcept
	{
		return DataIdentifier::NO_DATA;
	}

	static constexpr bool canReadData(DataIdentifier identifier) noexcept
	{
		return dataIdentifier() == identifier;
	}

	template <class InputIt>
	static constexpr std::size_t serializedSize(InputIt first, InputIt last) noexcept
	{
		return 0;
	}

	template <class OutputIt>
	static constexpr void readNodes(std::istream&, OutputIt, OutputIt) noexcept
	{
	}

	template <class OutputIt>
	static constexpr void readNodes(ReadBuffer&, OutputIt, OutputIt) noexcept
	{
	}

	template <class InputIt>
	static constexpr void writeNodes(std::ostream&, InputIt, InputIt) noexcept
	{
	}

	template <class InputIt>
	static constexpr void writeNodes(WriteBuffer&, InputIt, InputIt) noexcept
	{
	}
};
}  // namespace ufo::map

#endif  // UFO_MAP_EMPTY_MAP_H