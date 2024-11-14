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

#ifndef UFO_MAP_INTEGRATION_GRID_HPP
#define UFO_MAP_INTEGRATION_GRID_HPP

// UFO
// #include <ufo/map/hextree/hextree_code.hpp>
#include <ufo/map/octree/octree_code.hpp>
// #include <ufo/map/quadtree/quadtree_code.hpp>
#include <ufo/map/types.hpp>
#include <ufo/math/util.hpp>

// STL
#include <array>
#include <atomic>
#include <cstdint>
#include <type_traits>

namespace ufo
{
template <class Code>
class IntegrationGrid
{
};

// template <>
// class IntegrationGrid<QuadCode>
// {
//  private:
// 	// NOTE: Fits perfectly in 32 KiB L1 cache
// 	static constexpr depth_t Depth = 9;

// 	static constexpr std::size_t NumIndices = ipow(ipow(2, QuadCode::size()), Depth);

// 	static constexpr code_t Mask =
// 	    ~((code_t(-1) >> QuadCode::size() * Depth) << QuadCode::size() * Depth);

// 	using DataType               = std::array<std::uint64_t, NumIndices / 64>;
// 	using iterator               = typename DataType::iterator;
// 	using const_iterator         = typename DataType::const_iterator;
// 	using reverse_iterator       = typename DataType::reverse_iterator;
// 	using const_reverse_iterator = typename DataType::const_reverse_iterator;

//  public:
// 	using value_type = typename DataType::value_type;
// 	using reference  = typename DataType::reference;

//  public:
// 	constexpr iterator begin() { return std::begin(indices_); }

// 	constexpr const_iterator begin() const { return std::begin(indices_); }

// 	constexpr const_iterator cbegin() const { return begin(); }

// 	constexpr iterator end() { return std::end(indices_); }

// 	constexpr const_iterator end() const { return std::end(indices_); }

// 	constexpr const_iterator cend() const { return end(); }

// 	constexpr reverse_iterator rbegin() { return std::rbegin(indices_); }

// 	constexpr const_reverse_iterator rbegin() const { return std::rbegin(indices_); }

// 	constexpr const_reverse_iterator crbegin() const { return rbegin(); }

// 	constexpr reverse_iterator rend() { return std::rend(indices_); }

// 	constexpr const_reverse_iterator rend() const { return std::rend(indices_); }

// 	constexpr const_reverse_iterator crend() const { return rend(); }

// 	constexpr std::uint64_t operator[](std::uint_fast32_t index) const
// 	{
// 		return indices_[index >> 6];
// 	}

// 	constexpr std::uint64_t& operator[](std::uint_fast32_t index)
// 	{
// 		return indices_[index >> 6];
// 	}

// 	constexpr std::uint64_t operator[](QuadCode code) const
// 	{
// 		return operator[](index(code));
// 	}

// 	constexpr std::uint64_t& operator[](QuadCode code) { return operator[](index(code)); }

// 	constexpr bool test(std::uint_fast32_t index) const
// 	{
// 		return indices_[index >> 6] & (value_type(1) << (index & 0x3F));
// 	}

// 	constexpr bool test(QuadCode code) const { return test(index(code)); }

// 	constexpr void set(std::uint_fast32_t index)
// 	{
// 		indices_[index >> 6] |= value_type(1) << (index & 0x3F);
// 	}

// 	constexpr void set(QuadCode code) { set(index(code)); }

// 	constexpr void reset(std::uint_fast32_t index)
// 	{
// 		indices_[index >> 6] &= ~(value_type(1) << (index & 0x3F));
// 	}

// 	constexpr void reset(QuadCode code) { reset(index(code)); }

// 	void clear() { indices_.fill(0); }

// 	constexpr std::size_t size() const noexcept { return indices_.size() * 64; }

// 	static constexpr std::uint_fast32_t index(QuadCode code)
// 	{
// 		return (code.code() >> QuadCode::size() * code.depth()) & Mask;
// 	}

// 	static constexpr QuadCode code(std::uint_fast32_t index, depth_t depth)
// 	{
// 		return QuadCode(index << QuadCode::size() * depth, depth);
// 	}

// 	static constexpr QuadCode code(code_t prefix, std::uint_fast32_t index, depth_t depth)
// 	{
// 		return QuadCode(prefix | (index << QuadCode::size() * depth), depth);
// 	}

// 	static constexpr depth_t depth() { return Depth; }

//  private:
// 	DataType indices_{};
// };

template <>
class IntegrationGrid<OctCode>
{
 private:
	// NOTE: Fits perfectly in 32 KiB L1 cache
	static constexpr depth_t Depth = 6;

	static constexpr std::size_t NumIndices = ipow(ipow(2, OctCode::size()), Depth);

	static constexpr code_t Mask =
	    ~((code_t(-1) >> OctCode::size() * Depth) << OctCode::size() * Depth);

	using DataType               = std::array<std::uint64_t, NumIndices / 64>;
	using iterator               = typename DataType::iterator;
	using const_iterator         = typename DataType::const_iterator;
	using reverse_iterator       = typename DataType::reverse_iterator;
	using const_reverse_iterator = typename DataType::const_reverse_iterator;

 public:
	using value_type = typename DataType::value_type;
	using reference  = typename DataType::reference;

 public:
	constexpr iterator begin() { return std::begin(indices_); }

	constexpr const_iterator begin() const { return std::begin(indices_); }

	constexpr const_iterator cbegin() const { return begin(); }

	constexpr iterator end() { return std::end(indices_); }

	constexpr const_iterator end() const { return std::end(indices_); }

	constexpr const_iterator cend() const { return end(); }

	constexpr reverse_iterator rbegin() { return std::rbegin(indices_); }

	constexpr const_reverse_iterator rbegin() const { return std::rbegin(indices_); }

	constexpr const_reverse_iterator crbegin() const { return rbegin(); }

	constexpr reverse_iterator rend() { return std::rend(indices_); }

	constexpr const_reverse_iterator rend() const { return std::rend(indices_); }

	constexpr const_reverse_iterator crend() const { return rend(); }

	constexpr std::uint64_t operator[](std::uint_fast32_t index) const
	{
		return indices_[index >> 6];
	}

	constexpr std::uint64_t& operator[](std::uint_fast32_t index)
	{
		return indices_[index >> 6];
	}

	constexpr std::uint64_t operator[](OctCode code) const
	{
		return operator[](index(code));
	}

	constexpr std::uint64_t& operator[](OctCode code) { return operator[](index(code)); }

	constexpr bool test(std::uint_fast32_t index) const
	{
		return indices_[index >> 6] & (value_type(1) << (index & 0x3F));
	}

	constexpr bool test(OctCode code) const { return test(index(code)); }

	constexpr void set(std::uint_fast32_t index)
	{
		indices_[index >> 6] |= value_type(1) << (index & 0x3F);
	}

	constexpr void set(OctCode code) { set(index(code)); }

	constexpr void reset(std::uint_fast32_t index)
	{
		indices_[index >> 6] &= ~(value_type(1) << (index & 0x3F));
	}

	constexpr void reset(OctCode code) { reset(index(code)); }

	void clear() { indices_.fill(0); }

	constexpr std::size_t size() const noexcept { return indices_.size() * 64; }

	static constexpr std::uint_fast32_t index(OctCode code)
	{
		return (code.code() >> OctCode::size() * code.depth()) & Mask;
	}

	static constexpr OctCode code(std::uint_fast32_t index, depth_t depth)
	{
		return OctCode(index << OctCode::size() * depth, depth);
	}

	static constexpr OctCode code(code_t prefix, std::uint_fast32_t index, depth_t depth)
	{
		return OctCode(prefix | (index << OctCode::size() * depth), depth);
	}

	static constexpr depth_t depth() { return Depth; }

 private:
	DataType indices_{};
};

// template <>
// class IntegrationGrid<HexCode>
// {
// };
}  // namespace ufo

#endif  // UFO_MAP_INTEGRATION_GRID_HPP