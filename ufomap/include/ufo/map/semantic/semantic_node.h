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

#ifndef UFO_MAP_SEMANTIC_NODE_H
#define UFO_MAP_SEMANTIC_NODE_H

// UFO
#include <ufo/map/semantic/semantics.h>

// STL
#include <algorithm>
#include <array>

namespace ufo::map
{
template <std::size_t N>
struct SemanticNode {
	// Tags
	using size_type = label_t;
	using difference_type = std::ptrdiff_t;
	using reference = Semantic &;  // TODO: Make label const
	using const_reference = Semantic const &;
	using pointer = Semantic *;
	using const_pointer = Semantic const *;
	using iterator = Semantic *;  // TODO: Make label const
	using const_iterator = Semantic const *;
	using reverse_iterator = std::reverse_iterator<iterator>;
	using const_reverse_iterator = std::reverse_iterator<const_iterator>;

	// -1 half rounded up (i.e., the number of elements needed to store the sizes of the N
	// semantic containers)
	static constexpr std::size_t N_H = 1 + (N - 1) / 2;

	// Data
	std::unique_ptr<Semantic[]> semantics;

	//
	// Size
	//

	[[nodiscard]] static constexpr std::size_t semanticSize() { return N; }

	//
	// Fill
	//

	void fill(SemanticNode const &parent, index_t const index)
	{
		resizeLazy(parent.size(index));
		auto first = parent.begin(index);
		auto last = parent.end(index);
		for (index_t i = 0; N != i; ++i) {
			std::copy(first, last, begin(i));
		}
	}

	//
	// Is collapsible
	//

	[[nodiscard]] bool isCollapsible(SemanticNode const &parent, index_t const index) const
	{
		auto first = parent.begin(index);
		auto last = parent.end(index);
		for (index_t i = 0; N != i; ++i) {
			if (!std::equal(first, last, cbegin(i), cend(i))) {
				return false;
			}
		}
		return true;
	}

	//
	// Iterators
	//

	iterator begin() noexcept { return empty() ? nullptr : semantics.get() + N_H; }

	const_iterator begin() const noexcept
	{
		return empty() ? nullptr : semantics.get() + N_H;
	}

	const_iterator cbegin() const noexcept { return begin(); }

	iterator end() noexcept
	{
		auto const s = allocSize();
		return 0 == s ? nullptr : semantics.get() + s;
	}

	const_iterator end() const noexcept
	{
		auto const s = allocSize();
		return 0 == s ? nullptr : semantics.get() + s;
	}

	const_iterator cend() const noexcept { return end(); }

	//
	// Reverse iterators
	//

	reverse_iterator rbegin() noexcept { return std::make_reverse_iterator(end()); }

	const_reverse_iterator rbegin() const noexcept
	{
		return std::make_reverse_iterator(end());
	}

	const_reverse_iterator crbegin() const noexcept { return rbegin(); }

	reverse_iterator rend() noexcept { return std::make_reverse_iterator(begin()); }

	const_reverse_iterator rend() const noexcept
	{
		return std::make_reverse_iterator(begin());
	}

	const_reverse_iterator crend() const noexcept { return rend(); }

	//
	// Index iterators
	//

	iterator begin(index_t const index) noexcept
	{
		return empty() ? nullptr : semantics.get() + N_H + offset(index);
	}

	const_iterator begin(index_t const index) const noexcept
	{
		return empty() ? nullptr : semantics.get() + N_H + offset(index);
	}

	const_iterator cbegin(index_t const index) const noexcept { return begin(index); }

	iterator end(index_t const index) noexcept
	{
		return empty() ? nullptr : semantics.get() + N_H + offset(index) + size(index);
	}

	const_iterator end(index_t const index) const noexcept
	{
		return empty() ? nullptr : semantics.get() + N_H + offset(index) + size(index);
	}

	const_iterator cend(index_t const index) const noexcept { return end(index); }

	//
	// Reverse index iterators
	//

	reverse_iterator rbegin(index_t const index) noexcept
	{
		return std::make_reverse_iterator(end(index));
	}

	const_reverse_iterator rbegin(index_t const index) const noexcept
	{
		return std::make_reverse_iterator(end(index));
	}

	const_reverse_iterator crbegin(index_t const index) const noexcept
	{
		return rbegin(index);
	}

	reverse_iterator rend(index_t const index) noexcept
	{
		return std::make_reverse_iterator(begin(index));
	}

	const_reverse_iterator rend(index_t const index) const noexcept
	{
		return std::make_reverse_iterator(begin(index));
	}

	const_reverse_iterator crend(index_t const index) const noexcept { return rend(index); }

	//
	// Empty
	//

	[[nodiscard]] bool empty() const noexcept { return nullptr == semantics; }

	[[nodiscard]] bool empty(index_t const index) const { return 0 == size(index); }

	//
	// Size
	//

	[[nodiscard]] std::size_t size() const
	{
		auto const s = sizes();
		return std::accumulate(std::begin(s), std::end(s), std::size_t(0));
	}

	[[nodiscard]] size_type size(index_t const index) const
	{
		return empty()
		           ? 0
		           : (index % 2
		                  ? semantics[index / 2].label
		                  : reinterpret_cast<label_t const &>(semantics[index / 2].value));
	}

	[[nodiscard]] std::array<size_type, N> sizes() const
	{
		if (empty()) {
			return std::array<size_type, N>{};
		}

		std::array<size_type, N> s;
		std::copy(semantics.get(), semantics.get() + N_H,
		          reinterpret_cast<Semantic *>(s.data()));
		return s;
	}

	//
	// Offset
	//

	[[nodiscard]] std::size_t offset(index_t const index) const
	{
		auto const s = sizes();
		return std::accumulate(std::begin(s), std::begin(s) + index + 1, std::size_t(0));
	}

	//
	// Clear
	//

	void clear() noexcept { semantics.reset(); }

	void clear(index_t const index) { resize(index, 0); }

	//
	// Set
	//

	void set(Semantics const &semantics)
	{
		resizeLazy(semantics.size());
		auto first = std::begin(semantics);
		auto last = std::end(semantics);
		for (index_t i = 0; N != i; ++i) {
			std::copy(first, last, begin(i));
		}
	}

	void set(index_t index, Semantics const &semantics)
	{
		resize(index, semantics.size());
		std::copy(std::begin(semantics), std::end(semantics), begin(index));
	}

	//
	// TODO: Change label
	//

	void changeLabel(label_t old_label, label_t new_label)
	{
		// TODO: Implement
	}

	void changeLabel(index_t index, label_t old_label, label_t new_label)
	{
		// TODO: Implement
	}

	//
	// TODO: Insert
	//

	void insert(label_t label, value_t value)
	{
		// TODO: Implement
	}

	void insert(index_t index, label_t label, value_t value)
	{
		// TODO: Implement
	}

	template <class InputIt>
	void insert(InputIt first, InputIt last)
	{
		// TODO: Implement
	}

	template <class InputIt>
	void insert(index_t index, InputIt first, InputIt last)
	{
		// TODO: Implement
	}

	//
	// TODO: Insert or assign
	//

	void insertOrAssign(label_t label, value_t value)
	{
		// TODO: Implement
	}

	void insertOrAssign(index_t index, label_t label, value_t value)
	{
		// TODO: Implement
	}

	template <class UnaryFunction>
	void insertOrAssign(label_t label, UnaryFunction f)
	{
		// TODO: Implement
	}

	template <class UnaryFunction>
	void insertOrAssign(index_t index, label_t label, UnaryFunction f)
	{
		// TODO: Implement
	}

	template <class InputIt>
	void insertOrAssign(InputIt first, InputIt last)
	{
		// TODO: Implement
	}

	template <class InputIt>
	void insertOrAssign(index_t index, InputIt first, InputIt last)
	{
		// TODO: Implement
	}

	template <class InputIt, class UnaryFunction>
	void insertOrAssign(InputIt first, InputIt last, UnaryFunction f)
	{
		// TODO: Implement
	}

	template <class InputIt, class UnaryFunction>
	void insertOrAssign(index_t index, InputIt first, InputIt last, UnaryFunction f)
	{
		// TODO: Implement
	}

	//
	// TODO: Assign
	//

	//
	// TODO: Clear
	//

	void clear() { semantics.reset(); }

	void clear(index_t index) { resize(index, 0); }

	//
	// TODO: Erase
	//

	//
	// TODO: Erase if
	//

 private:
	//
	// Resize
	//

	/*!
	 * @brief Set the Size
	 *
	 * @note Only call if already allocated.
	 *
	 * @param index
	 * @param size
	 */
	void setSize(index_t const index, label_t const size)
	{
		if (index % 2) {
			semantics[index / 2].label = size;
		} else {
			semantics[index / 2].value = reinterpret_cast<value_t const &>(size);
		}
	}

	void resizeLazy(std::array<size_type, N> const &sizes)
	{
		auto const new_size = std::accumulate(std::begin(sizes), std::end(sizes), N_H);
		if (0 == new_size) {
			clear();
			return;
		}

		auto const cur_sizes = sizes();
		if (cur_sizes == sizes) {
			return;
		}

		if (std::accumulate(std::begin(cur_sizes), std::end(cur_sizes), N_H) != new_size) {
			pointer p_cur = data_.release();
			pointer p_new = static_cast<pointer>(realloc(p_cur, new_size * sizeof(Semantic)));

			if (!p_new) {
				data_.reset(p_cur);
				throw std::bad_alloc();
			}

			data_.reset(p_new);
		}

		for (index_t i = 0; N != i; ++i) {
			setSize(i, sizes[i]);
		}
	}

	void resizeLazy(size_type const size)
	{
		std::array<size_type, N> sizes;
		sizes.fill(size);
		resizeLazy(sizes);
	}

	void resizeLazy(index_t const index, std::size_t const new_size)
	{
		// FIXME: Optimize
		std::array<size_type, N> s = sizes();
		s[index] = new_size;
		resizeLazy(s);
	}

	void resize(std::array<size_type, N> const &sizes)
	{
		auto const new_size = std::accumulate(std::begin(sizes), std::end(sizes), N_H);
		if (0 == new_size) {
			clear();
			return;
		}

		auto const cur_sizes = sizes();
		if (cur_sizes == sizes) {
			return;
		}

		for (int i = N - 1; 0 <= i; --i) {
			// Reduce the indices that should be reduced
			if (cur_sizes[i] <= sizes[i]) {
				continue;
			}

			std::move(end(i), end(), begin(i) + sizes[i]);
			setSize(i, sizes[i]);
		}

		if (std::accumulate(std::begin(cur_sizes), std::end(cur_sizes), N_H) != new_size) {
			pointer p_cur = data_.release();
			pointer p_new = static_cast<pointer>(realloc(p_cur, new_size * sizeof(Semantic)));

			if (!p_new) {
				data_.reset(p_cur);
				throw std::bad_alloc();
			}

			data_.reset(p_new);
		}

		if (0 == cur_size) {
			for (index_t i = 0; N != i; ++i) {
				setSize(i, sizes[i]);
			}
		} else {
			// Increase the indices that should be increased
			auto last = data_.get() + new_size;
			for (index_t i = N - 1; 0 != i; --i) {
				setSize(i, sizes[i]);

				if (0 == sizes[i] || 0 == cur_sizes[i]) {
					continue;
				}

				auto first_i = begin(i);
				auto last_i = end(i);
				last = last != last_i ? std::move_backward(first_i, last_i, last) : first_i;
			}
		}
	}

	void resize(size_type const size)
	{
		std::array<size_type, N> sizes;
		sizes.fill(size);
		resize(sizes);
	}

	void resize(index_t const index, std::size_t const new_size)
	{
		// FIXME: Optimize
		std::array<size_type, N> s = sizes();
		s[index] = new_size;
		resize(s);
	}
};
}  // namespace ufo::map

#endif  // UFO_MAP_SEMANTIC_NODE_H