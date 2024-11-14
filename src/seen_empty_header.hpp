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

#ifndef UFO_MAP_INTEGRATION_MISSES_HPP
#define UFO_MAP_INTEGRATION_MISSES_HPP

// UFO
#include <ufo/util/bit_set.hpp>
#include <ufo/map/code.hpp>
#include <ufo/map/integration/grid.hpp>
#include <ufo/map/types.hpp>

// OMP
#include <omp.h>

// STL
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <thread>
#include <utility>
#include <vector>

namespace ufo::impl
{

struct Miss : public CodeOrIndex {
	BitSet<8> sibling;
	freedom_t freedom{};

	constexpr Miss(Code code, BitSet<8> sibling, freedom_t freedom = 0)
	    : CodeOrIndex(code), sibling(sibling), freedom(freedom)
	{
	}
};

using Misses = std::vector<Miss>;

template <std::size_t UnknownInflate = 0>
Misses getMisses(CodeMap<Grid> const& free_grids, CodeMap<Grid> const& hit_grids,
                 depth_t const depth, std::size_t const num_threads)
{
	Misses misses;

	std::vector<Code> codes;
	codes.reserve(free_grids.size());
	for (auto const& [code, _] : free_grids) {
		codes.push_back(code);
	}

	auto isMaskSet = [](std::uint64_t const x, std::uint64_t const mask) {
		return (x & mask) == mask;
	};

	code_t const inc = code_t(64) << 3 * depth;
#pragma omp parallel num_threads(num_threads)
	{
		Misses thread_misses;

#pragma omp for schedule(static)
		for (auto const code : codes) {
			auto thread_id = omp_get_thread_num();

			auto const& free_grid = free_grids.find(code)->second;
			// TODO: Add hit grid

			code_t i = code.code();
			for (auto e : free_grid) {
				if (0 == e) {
					i += inc;
					continue;
				}

				std::array<std::uint64_t, 5> fg{};

				std::array<std::uint64_t, 27> g{};
				std::array<code_t, 27>        c;

				Key k_o = Code(i, depth + 2);

				for (int z{-1}, s = k_o.step(), i{}; 2 != z; ++z) {
					for (int y{-1}; 2 != y; ++y) {
						for (int x{-1}; 2 != x; ++x, ++i) {
							auto k = k_o;
							k.x() += x * s;
							k.y() += y * s;
							k.z() += z * s;
							Code c_k = k;
							if (c_k.toDepth(code.depth()) == code) {
								g[i] = free_grid[c_k.toDepth(depth)];
							} else if (auto it = grids.find(c_k.toDepth(code.depth()));
							           std::end(grids) != it) {
								g[i] = it->second[c_k.toDepth(depth)];
							}
							c[i] = c_k.code();
						}
					}
				}

		// Checks
