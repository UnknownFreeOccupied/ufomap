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

#ifndef UFO_MAP_TYPES_HPP
#define UFO_MAP_TYPES_HPP

// UFO
#include <ufo/map/tree/tree_container.hpp>

// STL
#include <array>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace ufo
{
enum class Device { CPU, GPU };

using coord_t         = float;
using node_size_t     = double;
using time_t          = float;
using color_t         = std::uint8_t;
using label_t         = std::uint32_t;
using value_t         = float;
using intensity_t     = float;
using count_t         = std::int32_t;
using reflection_t    = double;
using distance_t      = float;
using surfel_scalar_t = float;
using freedom_t       = std::uint8_t;

//
// Map utility
//

using mu_t = std::uint64_t;

enum MapUtility : mu_t {
	ENABLE_DISABLE      = mu_t(1),  // FIXME: Maybe call 'ABILITY' or 'TOGGLE'
	ACCELERATED_COMPUTE = mu_t(1) << 1,
	SNAPSHOT            = mu_t(1) << 2,
	WITH_CENTER         = mu_t(1) << 3
};

//
// Accelerated compute indices
//

enum class AcceleratedComputeIndices : std::size_t {
	DATASTRUCTURE = 0,
	MODIFIED      = 1,
	OCCUPANCY     = 2,
	TIME          = 3,
	COLOR         = 4,
	LABEL         = 5,
	SEMANTIC      = 6,
	SURFEL        = 7,
	INTENSITY     = 8,
	COUNT         = 9,
	REFLECTION    = 10,
	SEEN_FREE     = 11,
	TSDF          = 12,
};
}  // namespace ufo

#endif  // UFO_MAP_TYPES_HPP