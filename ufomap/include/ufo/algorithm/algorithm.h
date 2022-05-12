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

#ifndef UFO_ALGORITHM_H
#define UFO_ALGORITHM_H

// STL
#include <algorithm>
#include <vector>

namespace ufo
{
using Permuation = std::vector<std::size_t>;

template <class RandomIt>
Permuation sortPermutation(RandomIt first, RandomIt last)
{
	Permuation p(std::distance(first, last));
	std::iota(std::begin(p), std::end(p), 0);
	std::sort(std::begin(p), std::end(p), [first, comp](std::size_t i, std::size_t j) {
		return *std::next(first, i) < *std::next(first, j);
	});
	return p;
}

template <class RandomIt, class Compare>
Permuation sortPermutation(RandomIt first, RandomIt last, Compare comp)
{
	Permuation p(std::distance(first, last));
	std::iota(std::begin(p), std::end(p), 0);
	std::sort(std::begin(p), std::end(p), [first, comp](std::size_t i, std::size_t j) {
		return comp(*std::next(first, i), *std::next(first, j));
	});
	return p;
}

template <class RandomIt>
void applyPermutation(RandomIt first, RandomIt last, Permuation const& perm)
{
	auto const size = perm.size();

	assert(std::distance(first, last) == size);

	std::vector<uint8_t> done(size, false);
	for (std::size_t i = 0; i != size; ++i) {
		if (done[i]) {
			continue;
		}

		for (std::size_t prev_j = i, j = perm[i]; i != j; prev_j = j, j = perm[j]) {
			std::iter_swap(std::next(first, prev_j), std::next(first, j));
			done[j] = true;
		}
	}
}
}  // namespace ufo

#endif  // UFO_ALGORITHM_H