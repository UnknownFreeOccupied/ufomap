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

#ifndef UFO_MAP_SEMANTIC_H
#define UFO_MAP_SEMANTIC_H

// UFO
#include <ufo/container/range.h>
#include <ufo/container/range_map.h>
#include <ufo/container/range_set.h>
#include <ufo/map/types.h>

namespace ufo::map
{
using label_range_t = container::Range<label_t>;
using value_range_t = container::Range<value_t>;

using label_range_set_t = container::RangeSet<label_t>;
using value_range_set_t = container::RangeSet<value_t>;

using semantic_range_map_t = container::RangeMap<label_t, value_t>;

struct Semantic {
	label_t label = 0;
	value_t value = 0;

	constexpr Semantic() noexcept = default;

	constexpr Semantic(Semantic const&) noexcept = default;

	constexpr Semantic(Semantic&&) noexcept = default;

	constexpr Semantic(label_t label, value_t value = 0) noexcept
	    : label(label), value(value)
	{
	}

	constexpr Semantic& operator=(Semantic const&) = default;

	constexpr Semantic& operator=(Semantic&&) = default;

	friend bool operator==(Semantic lhs, Semantic rhs)
	{
		return lhs.label == rhs.label && lhs.value == rhs.value;
	}

	friend bool operator!=(Semantic lhs, Semantic rhs) { return !(lhs == rhs); }

	friend bool operator<(Semantic lhs, Semantic rhs)
	{
		return lhs.label < rhs.label || (lhs.label == rhs.label && lhs.value < rhs.value);
	}

	friend bool operator<=(Semantic lhs, Semantic rhs) { return !(rhs < lhs); }

	friend bool operator>(Semantic lhs, Semantic rhs) { return rhs < lhs; }

	friend bool operator>=(Semantic lhs, Semantic rhs) { return !(lhs < rhs); }

	friend std::ostream& operator<<(std::ostream& out, Semantic s)
	{
		return out << s.label << ": " << s.value;
	}
};
}  // namespace ufo::map

#endif  // UFO_MAP_SEMANTIC_H