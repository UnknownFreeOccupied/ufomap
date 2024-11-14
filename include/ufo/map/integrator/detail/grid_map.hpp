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
#include <ufo/container/tree/container.hpp>
#include <ufo/utility/spinlock.hpp>

// STL
#include <algorithm>
#include <stdexcept>
#include <utility>

namespace ufo::detail
{
template <class GridMiss, class GridHit, std::size_t NumBuckets = 100,
          std::size_t NumBlocksPerBucket = 128>
class GridMap
{
 public:
	using Code            = typename GridMiss::Code;
	using key_type        = typename Code::code_t;
	using mapped_type     = std::pair<GridMiss, GridHit>;
	using value_type      = std::pair<key_type, mapped_type>;
	using size_type       = std::size_t;
	using difference_type = std::ptrdiff_t;
	using reference       = value_type&;
	using const_reference = value_type const&;
	using pointer         = value_type*;
	using const_pointer   = value_type const*;

 private:
	// using Container = TreeContainer<value_type, NumBuckets, NumBlocksPerBucket>;
	using Container = std::vector<value_type>;

 public:
	using iterator               = typename Container::iterator;
	using const_iterator         = typename Container::const_iterator;
	using reverse_iterator       = std::reverse_iterator<iterator>;
	using const_reverse_iterator = std::reverse_iterator<const_iterator>;

	GridMap(std::size_t size)
	{
		// map_.reserve(128);
		map_.resize(size);
	}

	mapped_type& operator[](Code const& code) { return operator[](key(code)); }

	mapped_type& operator[](key_type const& key)
	{
		auto first = begin();
		auto last  = end();
		if (auto it = find(key, first, last); last != it) {
			return it->second;
		}

		std::lock_guard lock(mutex_);

		first = last;
		last  = end();
		if (auto it = find(key, first, last); last != it) {
			return it->second;
		}

		// TODO: Fix

		// return map_.emplace_back(key, mapped_type{}).second;

		// map_.setSize(map_.size() + 1);
		// map_.back().first = key;
		// return map_.back().second;

		last->first = key;
		++size_;
		return last->second;
	}

	mapped_type& at(Code const& code) { return at(key(code)); }

	mapped_type const& at(Code const& code) const { return at(key(code)); }

	mapped_type& at(key_type const& key)
	{
		auto first = begin();
		auto last  = end();

		if (auto it = find(key, first, last); last != it) {
			return it->second;
		}

		throw std::out_of_range("GridMap::at: key not found");
	}

	mapped_type const& at(key_type const& key) const
	{
		auto first = begin();
		auto last  = end();

		if (auto it = find(key, first, last); last != it) {
			return it->second;
		}

		throw std::out_of_range("GridMap::at: key not found");
	}

	iterator begin() { return map_.begin(); }

	const_iterator begin() const { return map_.begin(); }

	const_iterator cbegin() const { return map_.cbegin(); }

	iterator end()
	{
		// return map_.end();
		return map_.begin() + size();
	}

	const_iterator end() const
	{
		// return map_.end();
		return map_.begin() + size();
	}

	const_iterator cend() const
	{
		// return map_.cend();
		return map_.cbegin() + size();
	}

	reverse_iterator rbegin() { return map_.rbegin(); }

	const_reverse_iterator rbegin() const { return map_.rbegin(); }

	const_reverse_iterator crbegin() const { return map_.crbegin(); }

	reverse_iterator rend() { return map_.rend(); }

	const_reverse_iterator rend() const { return map_.rend(); }

	const_reverse_iterator crend() const { return map_.crend(); }

	[[nodiscard]] constexpr inline key_type key(Code const& code) const
	{
		return code.toDepth(GridMiss::depth() + code.depth()).code();
	}

	void clear()
	{
		// map_.clear();
		size_ = 0;
	}

	[[nodiscard]] size_type size() const
	{
		//  return map_.size();
		return size_;
	}

 private:
	[[nodiscard]] inline iterator find(key_type const& key, iterator first, iterator last)
	{
		return std::find_if(first, last,
		                    [key](value_type const& v) { return v.first == key; });
	}

	[[nodiscard]] const_iterator find(key_type const& key, const_iterator first,
	                                  const_iterator last) const
	{
		return std::find_if(first, last,
		                    [key](value_type const& v) { return v.first == key; });
	}

 private:
	Container          map_{};
	Spinlock           mutex_{};
	std::atomic_size_t size_{};
};
}  // namespace ufo::detail

#endif  // UFO_MAP_INTEGRATION_DETAIL_GRID_MAP_HPP