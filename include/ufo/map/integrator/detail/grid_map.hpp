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

#ifndef UFO_MAP_INTEGRATION_DETAIL_GRID_MAP_HPP
#define UFO_MAP_INTEGRATION_DETAIL_GRID_MAP_HPP

// UFO
#include <ufo/container/tree/code.hpp>
#include <ufo/map/integrator/detail/bool_grid.hpp>
#include <ufo/map/integrator/detail/count_grid.hpp>

// STL
#include <cassert>
#include <cstddef>
#include <utility>
#include <vector>

namespace ufo::detail
{
template <std::size_t Dim, unsigned Depth>
struct GridMapKey : TreeCode<Dim> {
	constexpr GridMapKey() = default;

	explicit constexpr GridMapKey(TreeCode<Dim> const& code)
	    : TreeCode<Dim>(code.toDepth(Depth + code.depth()))
	{
	}
};

template <template <std::size_t, unsigned> class Grid, std::size_t Dim, unsigned Depth>
class GridMap
{
 public:
	using Code = TreeCode<Dim>;

	using key_type        = GridMapKey<Dim, Depth>;
	using mapped_type     = Grid<Dim, Depth>;
	using value_type      = std::pair<key_type, mapped_type>;
	using size_type       = std::size_t;
	using difference_type = std::ptrdiff_t;
	using reference       = value_type&;
	using const_reference = value_type const&;
	using pointer         = value_type*;
	using const_pointer   = value_type const*;

 private:
	using Container = std::vector<value_type>;

 public:
	using iterator       = typename Container::iterator;
	using const_iterator = typename Container::const_iterator;

	iterator begin() { return maps_.begin(); }

	const_iterator begin() const { return maps_.begin(); }

	const_iterator cbegin() const { return maps_.cbegin(); }

	iterator end() { return begin() + size_; }

	const_iterator end() const { return begin() + size_; }

	const_iterator cend() const { return cbegin() + size_; }

	mapped_type& operator[](key_type const& key)
	{
		if (last_key_ == key) {
			return maps_[last_].second;
		}

		last_key_ = key;

		for (last_ = {}; size_ > last_; ++last_) {
			if (maps_[last_].first == key) {
				return maps_[last_].second;
			}
		}

		if (maps_.size() <= last_) {
			maps_.emplace_back();
			assert(maps_.size() >= last_);
		}

		++size_;

		maps_[last_].first = key;
		return maps_[last_].second;
	}

	mapped_type& operator[](Code const& code) { return operator[](key(code)); }

	mapped_type& operator[](std::size_t pos)
	{
		assert(size_ > pos);
		return maps_[pos].second;
	}

	void addKey(key_type const& key)
	{
		if (maps_.size() <= size_) {
			maps_.emplace_back();
			assert(maps_.size() > size_);
		}

		maps_[size_].first = key;
		++size_;
	}

	[[nodiscard]] bool contains(key_type const& key) const
	{
		return std::any_of(begin(), end(), [&key](auto const& v) { return v.first == key; });
	}

	[[nodiscard]] constexpr inline key_type key(Code const& code) const
	{
		return key_type(code);
	}

	void clear()
	{
		for (auto& v : *this) {
			v.second.clear();
		}
		size_     = 0u;
		last_     = {};
		last_key_ = {};
	}

	[[nodiscard]] size_type size() const { return size_; }

 private:
	Container   maps_;
	std::size_t size_{};
	std::size_t last_{};
	key_type    last_key_{};
};
}  // namespace ufo::detail

#endif  // UFO_MAP_INTEGRATION_DETAIL_GRID_MAP_HPP