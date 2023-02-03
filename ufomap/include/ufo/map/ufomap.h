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

#ifndef UFO_MAP_UFO_MAP_H
#define UFO_MAP_UFO_MAP_H

// UFO
#include <ufo/map/color/color_map.h>
// #include <ufo/map/distance/distance_map.h>
#include <ufo/map/empty/empty_map.h>
#include <ufo/map/io.h>
#include <ufo/map/occupancy/occupancy_map.h>
#include <ufo/map/octree/octree_map.h>
// #include <ufo/map/semantic/semantic_map.h>
#include <ufo/map/count/count_map.h>
#include <ufo/map/intensity/intensity_map.h>
#include <ufo/map/label/label_map.h>
#include <ufo/map/reflection/reflection_map.h>
#include <ufo/map/surfel/surfel_map.h>
#include <ufo/map/time/time_map.h>
#include <ufo/map/types.h>

// STL
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <type_traits>

namespace ufo::map
{
//
// Helpers
//

template <bool C, mt_t T, template <class> class Map>
struct cond_map {
	template <class D, std::size_t N>
	using type = Map<D, N>;
};

template <mt_t T, template <class> class Map>
struct cond_map<false, T, Map> {
	template <class D, std::size_t N>
	using type = EmptyMap<T, D, N>;
};

//
// UFOMap
//

template <mt_t MapType>
class UFOMap
    : public OctreeMap<
          // clang-format off
          cond_map<MapType & OCCUPANCY,  OCCUPANCY,  OccupancyMap>::template type,
          cond_map<MapType & COLOR,      COLOR,      ColorMap>::template type,
          cond_map<MapType & TIME,       TIME,       TimeMap>::template type,
          cond_map<MapType & INTENSITY,  INTENSITY,  IntensityMap>::template type,
          cond_map<MapType & COUNT,      COUNT,      CountMap>::template type,
          cond_map<MapType & REFLECTION, REFLECTION, ReflectionMap>::template type,
          cond_map<MapType & SURFEL,     SURFEL,     SurfelMap>::template type,
					cond_map<MapType & LABEL,      LABEL,      LabelMap>::template type
          // cond_map<MapType & SEMANTIC,   SEMANTIC,   SemanticMap>::template type,
          // cond_map<MapType & DISTANCE,   DISTANCE,   DistanceMap>::template type,
          // clang-format on
          >
{
 private:
	using Base = OctreeMap<
	    // clang-format off
			cond_map<MapType & OCCUPANCY,  OCCUPANCY,  OccupancyMap>::template type,
			cond_map<MapType & COLOR,      COLOR,      ColorMap>::template type,
			cond_map<MapType & TIME,       TIME,       TimeMap>::template type,
			cond_map<MapType & INTENSITY,  INTENSITY,  IntensityMap>::template type,
			cond_map<MapType & COUNT,      COUNT,      CountMap>::template type,
			cond_map<MapType & REFLECTION, REFLECTION, ReflectionMap>::template type,
			cond_map<MapType & SURFEL,     SURFEL,     SurfelMap>::template type,
			cond_map<MapType & LABEL,      LABEL,      LabelMap>::template type
			// cond_map<MapType & SEMANTIC,   SEMANTIC,   SemanticMap>::template type,
			// cond_map<MapType & DISTANCE,   DISTANCE,   DistanceMap>::template type,
	    // clang-format on
	    >;

 public:
	//
	// Constructors
	//

	UFOMap(node_size_t leaf_node_size = 0.1, depth_t depth_levels = 17)
	    : Base(leaf_node_size, depth_levels)
	{
	}

	UFOMap(std::filesystem::path const& file) : Base(file) {}

	UFOMap(std::istream& in) : Base(in) {}

	UFOMap(ReadBuffer& in) : Base(in) {}

	UFOMap(UFOMap const& other) = default;

	UFOMap(UFOMap&& other) = default;

	template <mt_t MapType2>
	UFOMap(UFOMap<MapType2> const& other) : Base(other)
	{
	}

	template <mt_t MapType2>
	UFOMap(UFOMap<MapType2>&& other) : Base(std::move(other))
	{
	}

	//
	// Operator assignment
	//

	UFOMap& operator=(UFOMap const& rhs) = default;

	UFOMap& operator=(UFOMap&& rhs) = default;

	template <mt_t MapType2>
	UFOMap& operator=(UFOMap<MapType2> const& rhs)
	{
		Base::operator=(rhs);
		return *this;
	}

	template <mt_t MapType2>
	UFOMap& operator=(UFOMap<MapType2>&& rhs)
	{
		Base::operator=(std::move(rhs));
		return *this;
	}

	//
	// Swap
	//

	void swap(UFOMap& other) noexcept(noexcept(Base::swap(other))) { Base::swap(other); }
};
}  // namespace ufo::map

namespace std
{
template <ufo::map::mt_t MapType>
void swap(ufo::map::UFOMap<MapType>& lhs,
          ufo::map::UFOMap<MapType>& rhs) noexcept(noexcept(lhs.swap(rhs)))
{
	lhs.swap(rhs);
}
}  // namespace std

#endif  // UFO_MAP_UFO_MAP_H