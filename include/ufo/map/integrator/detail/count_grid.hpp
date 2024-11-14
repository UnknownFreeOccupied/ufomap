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

#ifndef UFO_MAP_INTEGRATION_COUNT_GRID_HPP
#define UFO_MAP_INTEGRATION_COUNT_GRID_HPP

// UFO
#include <ufo/container/tree/code.hpp>
#include <ufo/math/math.hpp>

// STL
#include <array>
#include <atomic>
#include <cstddef>

namespace ufo
{
template <std::size_t Dim, unsigned Depth, bool Atomic = false>
class CountGrid
{
 public:
	using Code    = TreeCode<Dim>;
	using code_t  = typename Code::code_t;
	using depth_t = typename Code::depth_t;

 private:
	static constexpr std::size_t NumIndices = ipow(std::size_t(2), Dim* Depth);

	static constexpr code_t Mask = ~(((~code_t(0)) >> Dim * Depth) << Dim * Depth);

	using DataType = std::array<int, NumIndices>;

 public:
	using value_type             = typename DataType::value_type;
	using reference              = typename DataType::reference;
	using const_reference        = typename DataType::const_reference;
	using iterator               = typename DataType::iterator;
	using const_iterator         = typename DataType::const_iterator;
	using reverse_iterator       = typename DataType::reverse_iterator;
	using const_reverse_iterator = typename DataType::const_reverse_iterator;

 public:
	iterator begin() { return grid_.begin(); }

	const_iterator begin() const { return grid_.begin(); }

	const_iterator cbegin() const { return grid_.cbegin(); }

	iterator end() { return grid_.end(); }

	const_iterator end() const { return grid_.end(); }

	const_iterator cend() const { return grid_.cend(); }

	reference operator[](std::size_t pos) { return grid_[pos]; }

	const_reference operator[](std::size_t pos) const { return grid_[pos]; }

	reference operator[](Code const code) { return operator[](pos(code)); }

	const_reference operator[](Code const code) const { return operator[](pos(code)); }

	int inc(std::size_t pos) { return grid_[pos]++; }

	int inc(Code const& code) { return inc(pos(code)); }

	int dec(std::size_t pos) { return grid_[pos]--; }

	int dec(Code const& code) { return dec(pos(code)); }

	void clear() { grid_.fill(int(0)); }

	[[nodiscard]] static constexpr std::size_t size() noexcept { return NumIndices; }

	[[nodiscard]] static constexpr std::size_t pos(Code const& code) noexcept
	{
		return (code.code() >> Dim * code.depth()) & Mask;
	}

	[[nodiscard]] static constexpr Code code(std::size_t pos, depth_t depth)
	{
		return Code(pos << Dim * depth, depth);
	}

	[[nodiscard]] static constexpr Code code(code_t prefix, std::size_t pos, depth_t depth,
	                                         depth_t depth_offset)
	{
		return Code(prefix | (pos << Dim * depth), depth + depth_offset);
	}

	[[nodiscard]] static constexpr depth_t depth() noexcept { return Depth; }

 private:
	DataType grid_{};
};

template <std::size_t Dim, unsigned Depth>
class CountGrid<Dim, Depth, true>
{
 public:
	using Code    = TreeCode<Dim>;
	using code_t  = typename Code::code_t;
	using depth_t = typename Code::depth_t;

 private:
	static constexpr std::size_t NumIndices = ipow(std::size_t(2), Dim* Depth);

	static constexpr code_t Mask = ~(((~code_t(0)) >> Dim * Depth) << Dim * Depth);

	using DataType = std::array<std::atomic_int, NumIndices>;

 public:
	using value_type             = typename DataType::value_type;
	using reference              = typename DataType::reference;
	using const_reference        = typename DataType::const_reference;
	using iterator               = typename DataType::iterator;
	using const_iterator         = typename DataType::const_iterator;
	using reverse_iterator       = typename DataType::reverse_iterator;
	using const_reverse_iterator = typename DataType::const_reverse_iterator;

 public:
	CountGrid() = default;

	CountGrid(CountGrid const& other)
	{
		for (std::size_t i{}; grid_.size() > i; ++i) {
			grid_[i] = other.grid_[i].load();
		}
	}

	CountGrid(CountGrid&&) = default;

	CountGrid& operator=(CountGrid const& rhs)
	{
		for (std::size_t i{}; grid_.size() > i; ++i) {
			grid_[i] = rhs.grid_[i].load();
		}
		return *this;
	}

	CountGrid& operator=(CountGrid&&) = default;

	iterator begin() { return grid_.begin(); }

	const_iterator begin() const { return grid_.begin(); }

	const_iterator cbegin() const { return grid_.cbegin(); }

	iterator end() { return grid_.end(); }

	const_iterator end() const { return grid_.end(); }

	const_iterator cend() const { return grid_.cend(); }

	reference operator[](std::size_t pos) { return grid_[pos]; }

	const_reference operator[](std::size_t pos) const { return grid_[pos]; }

	reference operator[](Code const code) { return operator[](pos(code)); }

	const_reference operator[](Code const code) const { return operator[](pos(code)); }

	int inc(std::size_t pos) { return grid_[pos]++; }

	int inc(Code const& code) { return inc(pos(code)); }

	int dec(std::size_t pos) { return grid_[pos]--; }

	int dec(Code const& code) { return dec(pos(code)); }

	void clear()
	{
		for (auto& x : grid_) {
			x = 0;
		}
	}

	[[nodiscard]] static constexpr std::size_t size() noexcept { return NumIndices; }

	[[nodiscard]] static constexpr std::size_t pos(Code const& code) noexcept
	{
		return (code.code() >> Dim * code.depth()) & Mask;
	}

	[[nodiscard]] static constexpr Code code(std::size_t pos, depth_t depth)
	{
		return Code(pos << Dim * depth, depth);
	}

	[[nodiscard]] static constexpr Code code(code_t prefix, std::size_t pos, depth_t depth,
	                                         depth_t depth_offset)
	{
		return Code(prefix | (pos << Dim * depth), depth + depth_offset);
	}

	[[nodiscard]] static constexpr depth_t depth() noexcept { return Depth; }

 private:
	DataType grid_{};
};
}  // namespace ufo

#endif  // UFO_MAP_INTEGRATION_COUNT_GRID_HPP