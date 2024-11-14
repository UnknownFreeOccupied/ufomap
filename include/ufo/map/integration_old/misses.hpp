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
#include <ufo/map/integration/integration_grid.hpp>
#include <ufo/map/types.hpp>
#include <ufo/utility/bit_set.hpp>

// STL
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <thread>
#include <utility>
#include <vector>

#ifdef UFO_OMP
// OMP
#include <omp.h>
#endif

namespace ufo::impl
{
struct Miss : public Code {
	BitSet<8> sibling;
	bool      seen_empty;

	constexpr Miss(Code code, BitSet<8> sibling, bool seen_empty)
	    : Code(code), sibling(sibling), seen_empty(seen_empty)
	{
	}
};

struct Miss : public CodeOrIndex {
	BitSet<8> sibling;
	freedom_t freedom{};

	constexpr Miss(Code code, BitSet<8> sibling, freedom_t freedom = 0)
	    : CodeOrIndex(code), sibling(sibling), freedom(freedom)
	{
	}
};

using Misses = std::vector<Miss>;

template <std::size_t UnknownInflate>
Misses getMisses(std::unordered_map<Code, Grid> const& free_grids,
                 std::unordered_map<Code, Grid> const& hit_grids, depth_t const depth,
                 std::size_t const num_threads)
{
	Misses misses;

	auto isMaskSet = [](auto const x, auto const mask) { return (x & mask) == mask; };

	code_t const inc = code_t(64) << 3 * depth;

	Grid const empty_grid;

#ifdef UFO_OMP
	if (1 < num_threads) {
		std::vector<Code> codes;
		codes.reserve(free_grids.size());
		for (auto const& [code, _] : free_grids) {
			codes.push_back(code);
		}

#pragma omp parallel num_threads(num_threads)
		{
			Misses thread_misses;

#pragma omp for schedule(static)
			for (auto const code : codes) {
				auto thread_id = omp_get_thread_num();

				auto        it       = hit_grids.find(code);
				Grid const& hit_grid = std::cend(hit_grids) != it ? it->second : empty_grid;

				Grid const& free_grid = free_grids.find(code)->second;

				auto   hg_it = std::cbegin(hit_grid);
				code_t i     = code.code();
				for (auto const m : free_grid) {
					if (0 == m) {
						i += inc;
						++hg_it;
						continue;
					}

					auto const h = ~(*hg_it);

					// std::array<std::uint_fast8_t, 8> mf{
					//     std::uint_fast8_t(m & 0xFFu),         std::uint_fast8_t((m >> 8) &
					//     0xFFu), std::uint_fast8_t((m >> 16) & 0xFFu), std::uint_fast8_t((m >> 24)
					//     & 0xFFu), std::uint_fast8_t((m >> 32) & 0xFFu), std::uint_fast8_t((m >>
					//     40) & 0xFFu), std::uint_fast8_t((m >> 48) & 0xFFu), std::uint_fast8_t((m
					//     >> 56) & 0xFFu)};

					std::array<std::uint_fast8_t, 8> mf{};

#include <ufo/map/integration/misses/1.hpp>

					BitSet<8> const field(static_cast<BitSet<8>::T>(
					    // clang-format off
							 std::uint8_t(isMaskSet(mf[0], 0xFFu)) 
						| (std::uint8_t(isMaskSet(mf[1], 0xFFu)) << 1) 
						| (std::uint8_t(isMaskSet(mf[2], 0xFFu)) << 2) 
						| (std::uint8_t(isMaskSet(mf[3], 0xFFu)) << 3) 
						| (std::uint8_t(isMaskSet(mf[4], 0xFFu)) << 4) 
						| (std::uint8_t(isMaskSet(mf[5], 0xFFu)) << 5) 
						| (std::uint8_t(isMaskSet(mf[6], 0xFFu)) << 6) 
						| (std::uint8_t(isMaskSet(mf[7], 0xFFu)) << 7)
					    // clang-format on
					    ));

					if (field.any()) {
						thread_misses.emplace_back(Code(i, depth + 1), field, 1);
					}

					if (!field.all()) {
						for (code_t j{}, k{}; 64 != j; j += 8, ++k) {
							if (!std::as_const(field)[k] && (0xFF & mf[k])) {
								BitSet<8> f(static_cast<BitSet<8>::T>(
								    // clang-format off
										(0x1u  & mf[k]) 
									| (0x2u  & mf[k]) 
									| (0x4u  & mf[k]) 
									| (0x8u  & mf[k]) 
									| (0x10u & mf[k]) 
									| (0x20u & mf[k]) 
									| (0x40u & mf[k]) 
									| (0x80u & mf[k])
								    // clang-format on
								    ));

								thread_misses.emplace_back(Code(i | (j << 3 * depth), depth), f, 1);
							}
						}
					}

					i += inc;
					++hg_it;
				}
			}

#pragma omp critical
			{
				misses.insert(std::end(misses), std::cbegin(thread_misses),
				              std::cend(thread_misses));
			}
		}
	} else
#endif
	{
		// TODO: Implement
	}

	return misses;
}

inline Misses getMisses(std::size_t const                     unknown_inflate,
                        std::unordered_map<Code, Grid> const& free_grids,
                        std::unordered_map<Code, Grid> const& hit_grids,
                        depth_t const depth, std::size_t const num_threads)
{
#include <ufo/map/integration/misses/2.hpp>
	return Misses();
}
}  // namespace ufo::impl

#endif  // UFO_MAP_INTEGRATION_MISSES_HPP