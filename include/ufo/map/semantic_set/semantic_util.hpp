#ifndef UFO_MAP_SEMANTIC_UTIL_HPP
#define UFO_MAP_SEMANTIC_UTIL_HPP

#include <algorithm>
#if __cplusplus >= 202002L
#include <bit>
#endif
#include <cassert>
#include <cstddef>
#include <cstring>
#include <iterator>
#include <memory>
#include <optional>
#include <sstream>
#include <ufo/map/semantic/semantic.hpp>
#include <ufo/map/types.hpp>

namespace ufo::semantic
{

using size_type              = label_t;
using difference_type        = std::ptrdiff_t;
using reference              = Semantic&;  // TODO: Make label const
using const_reference        = Semantic const&;
using pointer                = Semantic*;
using const_pointer          = Semantic const*;
using iterator               = Semantic*;  // TODO: Make label const
using const_iterator         = Semantic const*;
using reverse_iterator       = std::reverse_iterator<iterator>;
using const_reverse_iterator = std::reverse_iterator<const_iterator>;

//
// Clear
//

template <std::size_t N>
void clear(std::unique_ptr<Semantic[]>& semantics) noexcept
{
	semantics.reset();
}

//
// Empty
//

template <std::size_t N>
[[nodiscard]] bool empty(std::unique_ptr<Semantic[]> const& semantics) noexcept
{
	return nullptr == semantics;
}

//
// Size
//

template <std::size_t N>
[[nodiscard]] size_type size(std::unique_ptr<Semantic[]> const& semantics,
                             offset_t const                     index)
{
	// TODO: ignore 2 msb that are used for change detection
	// return 0 == size<N>(semantics, index)

	// even index stored in label
	// odd index stored in value

	// this gives a compiler warning in release mode, therefore memcpy
	// dereferencing type-punned pointer will break strict-aliasing rules
	// [-Werror=strict-aliasing] semantics[index / 2].value = reinterpret_cast<value_t
	// const&>(size);
	// return empty<N>(semantics) ? 0
	//                            : (index % 2 ? reinterpret_cast<label_t const&>(
	//                                               semantics[index / 2].value)  // odd
	//                                               index
	//                                         : semantics[index / 2].label);     // even
	//                                         index

	if (empty<N>(semantics)) {
		return 0;
	} else {
		if (index % 2) {
			// std::size_t size;
			// std::memcpy(&size, &semantics[index / 2].value, sizeof(value_t));
			// size = reinterpret_cast<label_t const&>(semantics[index / 2].value);
			// return size;
			// return std::bit_cast<label_t>(semantics[index / 2].value);
			label_t dst;
			std::memcpy(&dst, &semantics[index / 2].value, sizeof(label_t));
			return dst;
		} else {
			return semantics[index / 2].label;
		}
	}
}

template <std::size_t N>
[[nodiscard]] std::array<size_type, N> sizes(std::unique_ptr<Semantic[]> const& semantics)
{
	if (empty<N>(semantics)) {
		return std::array<size_type, N>{};
	}

	// static constexpr std::size_t N_H = 1 + (N - 1) / 2;

	std::array<size_type, N> s;
	// std::copy(semantics.get(), semantics.get() + N_H,
	//           reinterpret_cast<Semantic *>(s.data())); // FIXME stack smashing detected

	for (offset_t i = 0; i < N; ++i) {
		s[i] = size<N>(semantics, i);
	}
	return s;
}

template <std::size_t N>
[[nodiscard]] size_type size(std::unique_ptr<Semantic[]> const& semantics)
{
	auto const s = sizes<N>(semantics);
	return std::accumulate(std::begin(s), std::end(s), size_type(0));
}

template <std::size_t N>
[[nodiscard]] std::size_t allocSize(std::unique_ptr<Semantic[]> const& semantics)
{
	static constexpr std::size_t N_H = 1 + (N - 1) / 2;

	return empty<N>(semantics) ? 0 : size<N>(semantics) + N_H;
}

//
// Offset
//

template <std::size_t N>
[[nodiscard]] std::size_t offset(std::unique_ptr<Semantic[]> const& semantics,
                                 offset_t const                     index)
{
	auto const s = sizes<N>(semantics);
	return std::accumulate(std::begin(s), std::begin(s) + index, std::size_t(0));
}

template <std::size_t N>
[[nodiscard]] bool empty(std::unique_ptr<Semantic[]> const& semantics,
                         offset_t const                     index)
{
	return 0 == size<N>(semantics, index);
}

//
// Iterators
//

template <std::size_t N>
iterator begin(std::unique_ptr<Semantic[]> const& semantics) noexcept
{
	static constexpr std::size_t N_H = 1 + (N - 1) / 2;

	return empty<N>(semantics) ? nullptr : semantics.get() + N_H;
}

template <std::size_t N>
const_iterator cbegin(std::unique_ptr<Semantic[]> const& semantics) noexcept
{
	return begin<N>(semantics);
}

template <std::size_t N>
iterator end(std::unique_ptr<Semantic[]> const& semantics) noexcept
{
	auto const s = allocSize<N>(semantics);
	return 0 == s ? nullptr : semantics.get() + s;
}

template <std::size_t N>
const_iterator cend(std::unique_ptr<Semantic[]> const& semantics) noexcept
{
	return end<N>(semantics);
}

//
// Index iterators
//

template <std::size_t N>
iterator begin(std::unique_ptr<Semantic[]> const& semantics,
               offset_t const                     index) noexcept
{
	static constexpr std::size_t N_H = 1 + (N - 1) / 2;

	return empty<N>(semantics, index) ? nullptr
	                                  : semantics.get() + N_H + offset<N>(semantics, index);
}

template <std::size_t N>
const_iterator cbegin(std::unique_ptr<Semantic[]> const& semantics,
                      offset_t const                     index) noexcept
{
	return begin<N>(semantics, index);
}

template <std::size_t N>
iterator end(std::unique_ptr<Semantic[]> const& semantics, offset_t const index) noexcept
{
	static constexpr std::size_t N_H = 1 + (N - 1) / 2;

	return empty<N>(semantics, index)
	           ? nullptr
	           : semantics.get() + N_H + offset<N>(semantics, index) +
	                 size<N>(semantics, index);
}

template <std::size_t N>
const_iterator cend(std::unique_ptr<Semantic[]> const& semantics,
                    offset_t const                     index) noexcept
{
	return end<N>(semantics, index);
}

//
// Lower bound
//

template <class InputIt, typename = std::enable_if_t<std::is_base_of_v<
                             std::input_iterator_tag,
                             typename std::iterator_traits<InputIt>::iterator_category>>>
[[nodiscard]] InputIt lower_bound(InputIt first, InputIt last, label_t label)
{
	return std::lower_bound(first, last,
	                        Semantic(label, std::numeric_limits<value_t>::lowest()));
}

template <std::size_t N>
[[nodiscard]] iterator lower_bound(std::unique_ptr<Semantic[]> const& semantics,
                                   offset_t index, label_t label)
{
	// return std::lower_bound(begin<N>(semantics, index), end<N>(semantics, index), label);
	return lower_bound(begin<N>(semantics, index), end<N>(semantics, index), label);
}

//
// Upper bound
//

template <class InputIt, typename = std::enable_if_t<std::is_base_of_v<
                             std::input_iterator_tag,
                             typename std::iterator_traits<InputIt>::iterator_category>>>
[[nodiscard]] InputIt upper_bound(InputIt first, InputIt last, label_t label)
{
	return std::upper_bound(first, last,
	                        Semantic(label, std::numeric_limits<value_t>::max()));
}

template <std::size_t N>
[[nodiscard]] iterator upper_bound(std::unique_ptr<Semantic[]> const& semantics,
                                   offset_t index, label_t label)
{
	// return std::upper_bound(begin<N>(semantics, index), end<N>(semantics, index), label);
	return upper_bound(begin<N>(semantics, index), end<N>(semantics, index), label);
}

//
// Equal range
//

template <std::size_t N>
[[nodiscard]] std::pair<iterator, iterator> equal_range(
    std::unique_ptr<Semantic[]> const& semantics, offset_t index, label_t label)
{
	return std::make_pair(lower_bound<N>(semantics, index, label),
	                      upper_bound<N>(semantics, index, label));
}

//
// Find
//

template <std::size_t N>
[[nodiscard]] iterator find(std::unique_ptr<Semantic[]> const& semantics, offset_t index,
                            label_t label)
{
	auto it = lower_bound<N>(semantics, index, label);
	return end<N>(semantics, index) != it && it->label == label ? it
	                                                            : end<N>(semantics, index);
}

// template<std::size_t N>
// [[nodiscard]] const_iterator find(std::unique_ptr<Semantic[]> const& semantics,
// offset_t index, label_t label)
// {
// 	auto it = lower_bound<N>(semantics, index, label);
// 	return end<N>(semantics, index) != it && it->label == label ? it : end<N>(semantics,
// index);
// }

//
// Contains
//

template <std::size_t N>
[[nodiscard]] bool contains(std::unique_ptr<Semantic[]> const& semantics, offset_t index,
                            label_t label)
{
	return end<N>(semantics, index) != find<N>(semantics, index, label);
}

//
// At
//
template <std::size_t N>
[[nodiscard]] std::optional<Semantic> at(std::unique_ptr<Semantic[]> const& semantics,
                                         offset_t index, label_t label)
{
	iterator it = find<N>(semantics, index, label);
	return end<N>(semantics, index) != it ? std::optional<Semantic>(*it) : std::nullopt;
}

//
// Value
//
template <std::size_t N>
[[nodiscard]] std::optional<value_t> value(std::unique_ptr<Semantic[]> const& semantics,
                                           offset_t index, label_t label)
{
	auto it = find<N>(semantics, index, label);
	return end<N>(semantics, index) != it ? std::optional<value_t>(it->value)
	                                      : std::nullopt;
}

//
// Count
//
template <std::size_t N>
[[nodiscard]] size_type count(std::unique_ptr<Semantic[]> const& semantics,
                              offset_t index, label_t label)
{
	return contains<N>(semantics, index, label) ? 1 : 0;
}

//
// All
//

template <std::size_t N>
[[nodiscard]] bool all(std::unique_ptr<Semantic[]> const& semantics, offset_t index,
                       SemanticRangeSet const& ranges)
{
	if (ranges.empty()) {
		return true;
	} else if (size<N>(semantics, index) < ranges.numValues()) {
		return false;
	}

	auto first = cbegin<N>(semantics, index);
	auto last  = cend<N>(semantics, index);
	for (auto range : ranges) {
		auto lower      = lower_bound(first, last, range.lower());
		first           = upper_bound(lower, last, range.upper());
		auto range_dist = range.upper() - range.lower() + 1;
		auto sem_dist   = std::distance(lower, first);
		if (first == last || range_dist != sem_dist) {
			return false;
		}
	}
	return true;
}

template <std::size_t N>
[[nodiscard]] bool all(std::unique_ptr<Semantic[]> const& semantics, offset_t index,
                       SemanticRange range)
{
	return all<N>(semantics, index, SemanticRangeSet(range));
}

template <std::size_t N, class UnaryPredicate>
[[nodiscard]] bool all(std::unique_ptr<Semantic[]> const& semantics, offset_t index,
                       UnaryPredicate p)
{
	return std::all_of(cbegin<N>(semantics, index), cend<N>(semantics, index), p);
}

template <std::size_t N, class UnaryPredicate>
[[nodiscard]] bool all(std::unique_ptr<Semantic[]> const& semantics, UnaryPredicate p)
{
	return std::all_of(cbegin<N>(semantics), cend<N>(semantics), p);
}

template <std::size_t N, class UnaryPredicate>
[[nodiscard]] bool all(std::unique_ptr<Semantic[]> const& semantics, offset_t index,
                       SemanticRangeSet const& ranges, UnaryPredicate p)
{
	if (ranges.empty()) {
		return true;
	} else if (size<N>(semantics, index) < ranges.numValues()) {
		return false;
	}

	auto first = cbegin<N>(semantics, index);
	auto last  = cend<N>(semantics, index);
	for (auto range : ranges) {
		auto lower      = lower_bound(first, last, range.lower());
		first           = upper_bound(lower, last, range.upper());
		auto range_dist = range.upper() - range.lower() + 1;
		auto sem_dist   = std::distance(lower, first);
		if (first == last || range_dist != sem_dist) {
			return false;
		}
		for (; lower != first; ++lower) {
			if (!p(*lower)) {
				return false;
			}
		}
	}
	return true;
}

template <std::size_t N, class UnaryPredicate>
[[nodiscard]] bool all(std::unique_ptr<Semantic[]> const& semantics, offset_t index,
                       SemanticRange range, UnaryPredicate p)
{
	return all<N>(semantics, index, SemanticRangeSet(range), p);
}

//
// Any
//

template <std::size_t N>
[[nodiscard]] bool any(std::unique_ptr<Semantic[]> const& semantics, offset_t index,
                       SemanticRangeSet const& ranges)
{
	if (ranges.empty()) {
		return true;
	}

	if (size<N>(semantics, index) < ranges.size()) {
		for (auto it = cbegin<N>(semantics, index), last = cend<N>(semantics, index);
		     it != last; ++it) {
			if (ranges.contains(it->label)) {
				return true;
			}
		}
	} else {
		auto first = cbegin<N>(semantics, index);
		auto last  = cend<N>(semantics, index);
		for (auto range : ranges) {
			first = lower_bound(first, last, range.lower());
			if (first == last) {
				return false;
			} else if (first->label <= range.upper()) {
				return true;
			}
		}
	}
	return false;
}

template <std::size_t N>
[[nodiscard]] bool any(std::unique_ptr<Semantic[]> const& semantics, offset_t index,
                       SemanticRange range)
{
	return any<N>(semantics, index, SemanticRangeSet(range));
}

template <std::size_t N, class UnaryPredicate>
[[nodiscard]] bool any(std::unique_ptr<Semantic[]> const& semantics, offset_t index,
                       UnaryPredicate p)
{
	return std::any_of(cbegin<N>(semantics, index), cend<N>(semantics, index), p);
}

template <std::size_t N, class UnaryPredicate>
[[nodiscard]] bool any(std::unique_ptr<Semantic[]> const& semantics, offset_t index,
                       SemanticRange range, UnaryPredicate p)
{
	return any<N>(semantics, index, SemanticRangeSet(range), p);
}

template <std::size_t N, class UnaryPredicate>
[[nodiscard]] bool any(std::unique_ptr<Semantic[]> const& semantics, offset_t index,
                       SemanticRangeSet const& ranges, UnaryPredicate p)
{
	if (ranges.empty()) {
		return true;
	}

	if (size<N>(semantics, index) < ranges.size()) {
		for (auto it = cbegin<N>(semantics, index), last = cend<N>(semantics, index);
		     it != last; ++it) {
			if (ranges.contains(it->label) && p(*it)) {
				return true;
			}
		}
	} else {
		auto first = cbegin<N>(semantics, index);
		auto last  = cend<N>(semantics, index);
		for (auto range : ranges) {
			first = lower_bound(first, last, range.lower());
			for (; first != last && first->label <= range.upper(); ++first) {
				if (p(*first)) {
					return true;
				}
			}
			if (first == last) {
				return false;
			}
		}
	}
}

//
// None
//

template <std::size_t N>
[[nodiscard]] bool none(std::unique_ptr<Semantic[]> const& semantics, offset_t index,
                        SemanticRangeSet const& ranges)
{
	return !any<N>(semantics, index, ranges);
}

template <std::size_t N>
[[nodiscard]] bool none(std::unique_ptr<Semantic[]> const& semantics, offset_t index,
                        SemanticRange range)
{
	return none<N>(semantics, index, SemanticRangeSet(range));
}

template <std::size_t N, class UnaryPredicate>
[[nodiscard]] bool none(std::unique_ptr<Semantic[]> const& semantics, offset_t index,
                        UnaryPredicate p)
{
	return std::none_of(cbegin<N>(semantics, index), cend<N>(semantics, index), p);
}

template <std::size_t N, class UnaryPredicate>
[[nodiscard]] bool none(std::unique_ptr<Semantic[]> const& semantics, offset_t index,
                        SemanticRange range, UnaryPredicate p)
{
	return none<UnaryPredicate, N>(semantics, index, SemanticRangeSet(range), p);
}

template <std::size_t N, class UnaryPredicate>
[[nodiscard]] bool none(std::unique_ptr<Semantic[]> const& semantics, offset_t index,
                        SemanticRangeSet const& ranges, UnaryPredicate p)
{
	return !any<N>(semantics, index, ranges, p);
}

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
template <std::size_t N>
void setSize(std::unique_ptr<Semantic[]> const& semantics, offset_t const index,
             size_type const size)
{
	// even index stored in label
	// odd index stored in value
	if (index % 2) {
		// this gives a compiler warning in release mode, therefore memcpy
		// dereferencing type-punned pointer will break strict-aliasing rules
		// [-Werror=strict-aliasing]

		// semantics[index / 2].value = reinterpret_cast<value_t const&>(size);

		// value_t v;
		// std::memcpy(&v, &size, sizeof(value_t));
		// semantics[index / 2].value = v;
		// semantics[index / 2].value = std::bit_cast<value_t>(size);
		std::memcpy(&semantics[index / 2].value, &size, sizeof(value_t));
	} else {
		semantics[index / 2].label = static_cast<label_t>(size);
	}
}

template <std::size_t N>
void resizeLazy(std::unique_ptr<Semantic[]>&    semantics,
                std::array<size_type, N> const& new_sizes)
{
	static constexpr std::size_t N_H = 1 + (N - 1) / 2;

	auto new_size = std::accumulate(std::begin(new_sizes), std::end(new_sizes), 0);
	if (0 == new_size) {
		clear<N>(semantics);
		return;
	}

	new_size += N_H;

	auto const cur_sizes = sizes<N>(semantics);
	if (cur_sizes == new_sizes) {
		return;
	}

	if (std::accumulate(std::begin(cur_sizes), std::end(cur_sizes), N_H) != new_size) {
		pointer p_cur = semantics.release();
		pointer p_new = static_cast<pointer>(realloc(p_cur, new_size * sizeof(Semantic)));

		if (!p_new) {
			semantics.reset(p_cur);
			throw std::bad_alloc();
		}

		semantics.reset(p_new);
	}

	for (offset_t i = 0; N != i; ++i) {
		setSize<N>(semantics, i, new_sizes[i]);
	}
}

template <std::size_t N>
void resizeLazy(std::unique_ptr<Semantic[]>& semantics, size_type const size)
{
	std::array<size_type, N> sizes;
	sizes.fill(size);
	resizeLazy<N>(semantics, sizes);
}

template <std::size_t N>
void resizeLazy(std::unique_ptr<Semantic[]>& semantics, offset_t const index,
                size_type const new_size)
{
	// FIXME: Optimize
	std::array<size_type, N> s = sizes<N>(semantics);
	s[index]                   = new_size;
	resizeLazy<N>(semantics, s);
}

template <std::size_t N>
void resize(std::unique_ptr<Semantic[]>&    semantics,
            std::array<size_type, N> const& new_sizes)
{
	static constexpr std::size_t N_H = 1 + (N - 1) / 2;

	auto const new_size = std::accumulate(std::begin(new_sizes), std::end(new_sizes), N_H);
	if (0 == new_size) {
		clear<N>(semantics);
		return;
	}

	auto const cur_sizes = sizes<N>(semantics);
	if (cur_sizes == new_sizes) {
		return;
	}

	for (int i = N - 1; 0 <= i; --i) {
		// Reduce the indices that should be reduced
		if (cur_sizes[i] <= new_sizes[i]) {
			continue;
		}

		std::move(end<N>(semantics, static_cast<offset_t>(i)), end<N>(semantics),
		          begin<N>(semantics, static_cast<offset_t>(i)) + new_sizes[i]);
		setSize<N>(semantics, static_cast<offset_t>(i), new_sizes[i]);
	}

	if (std::accumulate(std::begin(cur_sizes), std::end(cur_sizes), N_H) != new_size) {
		pointer p_cur = semantics.release();
		pointer p_new = static_cast<pointer>(realloc(p_cur, new_size * sizeof(Semantic)));

		if (!p_new) {
			semantics.reset(p_cur);
			throw std::bad_alloc();
		}

		semantics.reset(p_new);
	}

	if (0 == std::accumulate(std::begin(cur_sizes), std::end(cur_sizes), N_H) ||
	    std::accumulate(std::begin(cur_sizes), std::end(cur_sizes), 0) == 0) {
		for (offset_t i = 0; N != i; ++i) {
			setSize<N>(semantics, i, new_sizes[i]);
		}
	} else {
		// Increase the indices that should be increased
		auto const cur_size =
		    std::accumulate(std::begin(cur_sizes), std::end(cur_sizes), N_H);
		auto cur_last = semantics.get() + cur_size;

		// auto last = semantics.get() + new_size;
		for (int i = N - 1; i >= 0; --i) {
			// auto first_i_old = begin<N>(semantics, i);
			auto last_i_old = end<N>(semantics, static_cast<offset_t>(i));

			setSize<N>(semantics, static_cast<offset_t>(i), new_sizes[i]);

			// if (0 == new_sizes[i] || 0 == cur_sizes[i]) {
			// if (0 == new_sizes[i]) {
			if (cur_sizes[i] >= new_sizes[i]) {
				continue;
			}

			// move if new size is bigger than old size

			// auto first_i = begin<N>(semantics, i);
			// auto last_i = end<N>(semantics, i);
			// last = last != last_i ? std::move_backward(first_i, last_i, last) : first_i;

			auto first_i = begin<N>(semantics, static_cast<offset_t>(i));
			// auto last_i = end<N>(semantics, i);

			auto start = first_i;

			// if old size = 0, use start_i after new size
			// else use end_i of old size
			if (cur_sizes[i] == 0) {
				start = first_i;
			} else {
				start = last_i_old;
			}
			// if (cur_sizes[i] > 0) {
			// 	start = last_i_old;
			// }
			// cur_last = std::move_backward(start, cur_last, cur_last +
			// (new_sizes[i]-cur_sizes[i]));
			auto new_last = cur_last + (new_sizes[i] - cur_sizes[i]);
			std::move_backward(start, cur_last, new_last);
			cur_last = new_last;
		}
	}
}

template <std::size_t N>
void resize(std::unique_ptr<Semantic[]>& semantics, size_type const size)
{
	std::array<size_type, N> sizes;
	sizes.fill(size);
	resize<N>(semantics, sizes);
}

template <std::size_t N>
void resize(std::unique_ptr<Semantic[]>& semantics, offset_t const index,
            size_type const new_size)
{
	// FIXME: Optimize
	std::array<size_type, N> s = sizes<N>(semantics);
	s[index]                   = new_size;
	resize<N>(semantics, s);
}

template <std::size_t N>
void clear(std::unique_ptr<Semantic[]>& semantics, offset_t const index)
{
	resize<N>(semantics, index, 0);
}

template <std::size_t N, class InputIt,
          typename = std::enable_if_t<std::is_base_of_v<
              std::input_iterator_tag,
              typename std::iterator_traits<InputIt>::iterator_category>>>
size_type numAlreadyExists(std::unique_ptr<Semantic[]> const& semantics,
                           offset_t const index, InputIt first, InputIt last)
{
	size_type num = 0;

	auto first_index = cbegin<N>(semantics, index);
	auto last_index  = cend<N>(semantics, index);
	for (; first != last && first_index != last_index; ++first) {
		label_t label;
		if constexpr (std::is_same_v<Semantic,
		                             typename std::iterator_traits<InputIt>::value_type>) {
			label = first->label;
		} else {
			label = *first;
		}
		first_index = lower_bound(first_index, last_index, label);
		if (first_index != last_index && first_index->label == label) {
			++num;
		}
	}

	return num;
}

//
// Insert or assign helper
//

// just insert, no allocation
/*!
 * @brief
 *
 * @note Memory assumed already allocated
 *
 * @param index
 * @param cur_size
 * @param new_size
 * @param first Iterator to Semantic
 * @param last Iterator to Semantic
 */
template <std::size_t N, bool Assign, class InputIt,
          typename = std::enable_if_t<std::is_base_of_v<
              std::input_iterator_tag,
              typename std::iterator_traits<InputIt>::iterator_category>>>
void insertOrAssignImpl(std::unique_ptr<Semantic[]>& semantics, offset_t index,
                        size_type cur_size, size_type new_size, InputIt first,
                        InputIt last)
{
	if (0 == new_size) {
		return;
	} else if (0 == cur_size) {
		std::copy(first, last, begin<N>(semantics, index));
		return;
	}

	if constexpr (!Assign) {
		if (cur_size == new_size) {
			return;
		}
	}

	auto cur         = end<N>(semantics, index);
	auto first_index = begin<N>(semantics, index);
	auto last_index  = first_index + cur_size;
	while (first != last && first_index != last_index) {
		if constexpr (Assign) {
			if ((last_index - 1)->label == (last - 1)->label) {
				(--cur)->label = (--last_index)->label;
				cur->value     = (--last)->value;
				continue;
			}
		}
		if ((last_index - 1)->label < (last - 1)->label) {
			*(--cur) = *(--last);
			if constexpr (!Assign) {
				++cur_size;
				// FIXME: Does this actually improve performance?
				if (cur_size == new_size) {
					return;
				}
			}
		} else {
			// FIXME: Can this be a move? What happens if it is the same?
			*(--cur) = *(--last_index);
		}
	}

	// Copy the remaining to the beginning
	std::copy(first, last, first_index);
}

/*!
 * @brief
 *
 * @note Memory assumed already allocated
 *
 * @param index
 * @param cur_size
 * @param new_size
 * @param first Iterator to label_t
 * @param last Iterator to label_t
 * @param f
 */
template <std::size_t N, bool Assign, class InputIt, class UnaryFunction,
          class    = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>,
          typename = std::enable_if_t<std::is_base_of_v<
              std::input_iterator_tag,
              typename std::iterator_traits<InputIt>::iterator_category>>>
void insertOrAssignImpl(std::unique_ptr<Semantic[]>& semantics, offset_t index,
                        size_type cur_size, size_type new_size, InputIt first,
                        InputIt last, UnaryFunction f)
{
	if (0 == new_size) {
		return;
	} else if (0 == cur_size) {
		for (auto it = begin<N>(semantics, index); first != last; ++it, ++first) {
			it->label = *first;
			it->value = f(Semantic(*first));
		}
		return;
	}

	if constexpr (!Assign) {
		if (cur_size == new_size) {
			return;
		}
	}

	auto cur         = end<N>(semantics, index);
	auto first_index = begin<N>(semantics, index);
	auto last_index  = first_index + cur_size;
	while (first != last && first_index != last_index) {
		if constexpr (Assign) {
			if ((last_index - 1)->label == *(last - 1)) {
				(--cur)->label = (--last_index)->label;
				cur->value     = f(*cur);
				--last;
				continue;
			}
		}
		if ((last_index - 1)->label < *(last - 1)) {
			--cur;
			cur->label = *(--last);
			cur->value = f(Semantic(cur->label));
			if constexpr (!Assign) {
				++cur_size;
				// FIXME: Does this actually improve performance?
				if (cur_size == new_size) {
					return;
				}
			}
		} else {
			// FIXME: Can this be a move? What happens if it is the same?
			*(--cur) = *(--last_index);
		}
	}

	// Copy the remaining to the beginning
	for (auto it = begin<N>(semantics, index); first != last; ++it, ++first) {
		it->label = *first;
		it->value = f(Semantic(*first));
	}
}

// Allocates space for the element and inserts the element at the correct position
// Assign: if true value of semantic is overwritten if label is already present,
// if false nothing happens
// UnaryFunction: function that is applied to Semantic with label
//
// return iterator to element and a bool if insertion happened
template <std::size_t N, bool Assign, class UnaryFunction,
          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
std::pair<iterator, bool> insertOrAssignImpl(std::unique_ptr<Semantic[]>& semantics,
                                             offset_t const index, label_t label,
                                             UnaryFunction f)
{
	if (empty<N>(semantics, index)) {
		resize<N>(semantics, index, 1);
		auto it   = begin<N>(semantics, index);
		it->label = label;
		it->value = f(Semantic(label));
		return {it, true};
	}

	auto it = lower_bound<N>(semantics, index, label);
	if (end<N>(semantics, index) != it && it->label == label) {
		// Label already exists
		if constexpr (Assign) {
			it->value = f(*it);
		}
		return {it, false};
	}

	// insert at right position
	auto i = std::distance(begin<N>(semantics, index), it);
	resize<N>(semantics, index, size<N>(semantics, index) + 1);
	it              = begin<N>(semantics, index) + i;
	auto last_index = end<N>(semantics, index);
	std::move_backward(it, last_index - 1, last_index);
	it->label = label;
	it->value = f(Semantic(label));
	return {it, true};
}

// allocate the space and insert/assign to all nodes
template <std::size_t N, bool Assign, class UnaryFunction,
          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
void insertOrAssignImpl(std::unique_ptr<Semantic[]>& semantics, label_t label,
                        UnaryFunction f)
{
	if (empty<N>(semantics)) {
		std::array<size_type, N> s;
		s.fill(1);
		resize<N>(semantics, s);
		std::fill(begin<N>(semantics), end<N>(semantics),
		          Semantic(label, f(Semantic(label))));
		return;
	}

	std::array<size_type, N>       new_sizes = sizes<N>(semantics);
	std::array<difference_type, N> dist;
	for (offset_t index = 0; N != index; ++index) {
		auto it = lower_bound<N>(semantics, index, label);
		if (end<N>(semantics, index) != it && it->label == label) {
			// Label already exists
			if constexpr (Assign) {
				it->value = f(*it);
			}
			dist[index] = -1;  // don't need to insert this label
		} else {
			++new_sizes[index];
			dist[index] = std::distance(begin<N>(semantics, index), it);
		}
	}

	// if (0 == std::accumulate(std::begin(dist), std::end(dist), 0)) {
	// 	return;
	// }

	resize<N>(semantics, new_sizes);

	for (offset_t index = 0; N != index; ++index) {
		if (dist[index] < 0) {
			continue;  // don't need to insert this label, was already assigned
		}
		auto it         = begin<N>(semantics, index) + dist[index];
		auto last_index = end<N>(semantics, index);
		std::move_backward(it, last_index - 1, last_index);
		it->label = label;
		it->value = f(Semantic(label));
	}
}

// allocate the space and insert/assign to index, iterators
template <std::size_t N, bool Assign, class InputIt,
          typename = std::enable_if_t<std::is_base_of_v<
              std::input_iterator_tag,
              typename std::iterator_traits<InputIt>::iterator_category>>>
void insertOrAssignImpl(std::unique_ptr<Semantic[]>& semantics, offset_t const index,
                        InputIt first, InputIt last)
{
	std::vector vec(first, last);

	std::sort(std::begin(vec), std::end(vec));

	// Erase duplicate labels, saving highest value for each label
	auto r_last = std::unique(std::rbegin(vec), std::rend(vec),
	                          [](auto a, auto b) { return a.label == b.label; });

	auto f = r_last.base();
	auto l = std::end(vec);
	auto s = static_cast<size_type>(std::distance(f, l));

	if (empty<N>(semantics, index)) {
		// Optimized insert
		resize<N>(semantics, index, s);
		std::copy(f, l, begin<N>(semantics, index));
	} else {
		auto cur_size = size<N>(semantics, index);
		auto new_size = cur_size + s - numAlreadyExists<N>(semantics, index, f, l);

		resize<N>(semantics, index, new_size);

		// Do insert where memory already has been allocated
		insertOrAssignImpl<N, Assign>(semantics, index, cur_size, new_size, f, l);
	}
}

// allocate the space and insert/assign to all nodes, iterators
// allocate all space for all nodes at the same time, then insert
template <std::size_t N, bool Assign, class InputIt,
          typename = std::enable_if_t<std::is_base_of_v<
              std::input_iterator_tag,
              typename std::iterator_traits<InputIt>::iterator_category>>>
void insertOrAssignImpl(std::unique_ptr<Semantic[]>& semantics, InputIt first,
                        InputIt last)
{
	std::vector vec(first, last);

	std::sort(std::begin(vec), std::end(vec));

	// Erase duplicate labels, saving highest value for each label
	auto r_last = std::unique(std::rbegin(vec), std::rend(vec),
	                          [](auto a, auto b) { return a.label == b.label; });

	auto f = r_last.base();
	auto l = std::end(vec);
	auto s = static_cast<size_type>(std::distance(f, l));

	std::array<size_type, N> new_sizes;
	new_sizes.fill(s);

	if (empty<N>(semantics)) {
		resize<N>(semantics, new_sizes);
		for (offset_t index = 0; N != index; ++index) {
			std::copy(f, l, begin<N>(semantics, index));
		}
	} else {
		std::array<size_type, N> cur_sizes = sizes<N>(semantics);
		for (offset_t index = 0; N != index; ++index) {
			new_sizes[index] =
			    s + cur_sizes[index] - numAlreadyExists<N>(semantics, index, f, l);
		}

		resize<N>(semantics, new_sizes);

		// Do insert where memory already has been allocated
		for (offset_t index = 0; N != index; ++index) {
			insertOrAssignImpl<N, Assign>(semantics, index, cur_sizes[index], new_sizes[index],
			                              f, l);
		}
	}
}

// allocate and insert/assign with hint
template <std::size_t N, bool Assign, class UnaryFunction,
          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
std::pair<iterator, bool> insertOrAssignImpl(std::unique_ptr<Semantic[]>& semantics,
                                             offset_t const index, const_iterator hint,
                                             label_t label, UnaryFunction f)
{
	if (empty<N>(semantics, index)) {
		resize<N>(semantics, index, 1);
		auto it   = begin<N>(semantics, index);
		it->label = label;
		it->value = f(Semantic(label));
		return {it, true};
	}

	auto first_index = begin<N>(semantics, index);
	auto last_index  = end<N>(semantics, index);

	auto first =
	    first_index != hint && std::prev(hint, 1)->label < label ? hint : first_index;
	auto last = last_index != hint && hint->label >= label ? hint : last_index;
	hint      = lower_bound(first, last, label);

	auto i = std::distance<const_iterator>(first_index, hint);

	if (last_index != hint && hint->label == label) {
		// Label already exists
		auto it = first_index + i;
		if constexpr (Assign) {
			it->value = f(*it);
		}
		return {it, false};
	}

	resize<N>(semantics, index, size<N>(semantics, index) + 1);
	auto it    = begin<N>(semantics, index) + i;
	last_index = end<N>(semantics, index);
	std::move_backward(it, last_index - 1, last_index);
	it->label = label;
	it->value = f(Semantic(label));
	return {it, true};
}

// InputIt to label_t
template <std::size_t N, bool Assign, class InputIt, class UnaryFunction,
          class    = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>,
          typename = std::enable_if_t<std::is_base_of_v<
              std::input_iterator_tag,
              typename std::iterator_traits<InputIt>::iterator_category>>>
void insertOrAssignImpl(std::unique_ptr<Semantic[]>& semantics, offset_t const index,
                        InputIt first, InputIt last, UnaryFunction fun)
{
	std::vector vec(first, last);

	std::sort(std::begin(vec), std::end(vec));

	// Erase duplicate labels, saving highest value for each label
	auto r_last = std::unique(std::rbegin(vec), std::rend(vec),
	                          [](auto a, auto b) { return a == b; });

	auto f = r_last.base();
	auto l = std::end(vec);
	auto s = std::distance(f, l);

	if (empty<N>(semantics, index)) {
		// Optimized insert
		resize<N>(semantics, index, s);
		for (auto it = begin<N>(semantics, index); f != l; ++it, ++f) {
			it->label = *f;
			it->value = fun(Semantic(*f));
		}
	} else {
		auto cur_size = size<N>(semantics, index);
		auto new_size = cur_size + s - numAlreadyExists<N>(semantics, index, f, l);

		resize<N>(semantics, index, new_size);

		// Do insert where memory already has been allocated
		insertOrAssignImpl<N, Assign>(semantics, index, cur_size, new_size, f, l, fun);
	}
}

// TODO: enable if InputIt is iterator, otherwise this function is called instead of
// insertOrAssign(index, label, f)
template <std::size_t N, bool Assign, class InputIt, class UnaryFunction,
          class    = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>,
          typename = std::enable_if_t<std::is_base_of_v<
              std::input_iterator_tag,
              typename std::iterator_traits<InputIt>::iterator_category>>>
void insertOrAssignImpl(std::unique_ptr<Semantic[]>& semantics, InputIt first,
                        InputIt last, UnaryFunction fun)
{
	std::vector vec(first, last);

	std::sort(std::begin(vec), std::end(vec));

	// Erase duplicate labels
	auto r_last = std::unique(std::rbegin(vec), std::rend(vec),
	                          [](auto a, auto b) { return a == b; });

	auto f = r_last.base();
	auto l = std::end(vec);
	auto s = std::distance(f, l);

	std::array<size_type, N> new_sizes;
	new_sizes.fill(s);

	if (empty<N>(semantics)) {
		// Optimized insert
		std::vector<Semantic> sem;
		sem.reserve(s);
		for (; f != l; ++first) {
			sem.emplace_back(*f, fun(Semantic(*f)));
		}
		resize<N>(semantics, new_sizes);
		for (offset_t index = 0; N != index; ++index) {
			std::copy(std::cbegin(sem), std::cend(sem), begin<N>(semantics, index));
		}
	} else {
		// Calculate how many elements already exists per index
		std::array<size_type, N> cur_sizes = sizes<N>(semantics);

		for (offset_t index = 0; N != index; ++index) {
			new_sizes[index] =
			    cur_sizes[index] + s - numAlreadyExists<N>(semantics, index, f, l);
			// new_sizes[index] -= numAlreadyExists<N>(semantics, index, f, l);
		}

		resize<N>(semantics, new_sizes);

		// Do insert where memory already has been allocated
		for (offset_t index = 0; N != index; ++index) {
			insertOrAssignImpl<N, Assign>(semantics, index, cur_sizes[index], new_sizes[index],
			                              f, l, fun);
		}
	}
}

//
// Insert
//

// all
template <std::size_t N>
void insert(std::unique_ptr<Semantic[]>& semantics, label_t label, value_t value)
{
	insertOrAssignImpl<N, false>(semantics, label, [value](auto) { return value; });
}

// template<std::size_t N>
// void insert(std::unique_ptr<Semantic[]> & semantics, Semantic semantic) { return
// insert<N>(semantic.label, semantic.value); }

template <std::size_t N, class InputIt,
          typename = std::enable_if_t<std::is_base_of_v<
              std::input_iterator_tag,
              typename std::iterator_traits<InputIt>::iterator_category>>>
void insert(std::unique_ptr<Semantic[]>& semantics, InputIt first, InputIt last)
{
	insertOrAssignImpl<N, false>(semantics, first, last);
}

// index
template <std::size_t N>
std::pair<iterator, bool> insert(std::unique_ptr<Semantic[]>& semantics,
                                 offset_t const index, label_t label, value_t value)
{
	return insertOrAssignImpl<N, false>(semantics, index, label,
	                                    [value](auto) { return value; });
}

template <std::size_t N, class InputIt,
          typename = std::enable_if_t<std::is_base_of_v<
              std::input_iterator_tag,
              typename std::iterator_traits<InputIt>::iterator_category>>>
void insert(std::unique_ptr<Semantic[]>& semantics, offset_t const index, InputIt first,
            InputIt last)
{
	insertOrAssignImpl<N, false>(semantics, index, first, last);
}

template <std::size_t N>
std::pair<iterator, bool> insert(std::unique_ptr<Semantic[]>& semantics,
                                 offset_t const index, const_iterator hint, label_t label,
                                 value_t value)
{
	return insertOrAssignImpl<N, false>(semantics, index, hint, label,
	                                    [value](auto) { return value; });
}

//
// InsertOrAssign
//

// all
template <std::size_t N>
void insertOrAssign(std::unique_ptr<Semantic[]>& semantics, label_t label, value_t value)
{
	insertOrAssignImpl<N, true>(semantics, label, [value](auto) { return value; });
}

template <std::size_t N, class InputIt,
          typename = std::enable_if_t<std::is_base_of_v<
              std::input_iterator_tag,
              typename std::iterator_traits<InputIt>::iterator_category>>>
void insertOrAssign(std::unique_ptr<Semantic[]>& semantics, InputIt first, InputIt last)
{
	insertOrAssignImpl<N, true>(semantics, first, last);
}

// InputIt to sematic
template <std::size_t N, class UnaryFunction,
          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
void insertOrAssign(std::unique_ptr<Semantic[]>& semantics, label_t label,
                    UnaryFunction f)
{
	insertOrAssignImpl<N, true>(semantics, label, f);
}

// InputIt to label
template <std::size_t N, class InputIt, class UnaryFunction,
          class    = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>,
          typename = std::enable_if_t<std::is_base_of_v<
              std::input_iterator_tag,
              typename std::iterator_traits<InputIt>::iterator_category>>>
void insertOrAssign(std::unique_ptr<Semantic[]>& semantics, InputIt first, InputIt last,
                    UnaryFunction f)
{
	insertOrAssignImpl<N, true>(semantics, first, last, f);
}

// index
template <std::size_t N>
std::pair<iterator, bool> insertOrAssign(std::unique_ptr<Semantic[]>& semantics,
                                         offset_t index, label_t label, value_t value)
{
	return insertOrAssignImpl<N, true>(semantics, index, label,
	                                   [value](auto) { return value; });
}

template <std::size_t N, class InputIt,
          typename = std::enable_if_t<std::is_base_of_v<
              std::input_iterator_tag,
              typename std::iterator_traits<InputIt>::iterator_category>>>
void insertOrAssign(std::unique_ptr<Semantic[]>& semantics, offset_t index, InputIt first,
                    InputIt last)
{
	insertOrAssignImpl<N, true>(semantics, index, first, last);
}

template <std::size_t N>
std::pair<iterator, bool> insertOrAssign(std::unique_ptr<Semantic[]>& semantics,
                                         offset_t index, const_iterator hint,
                                         label_t label, value_t value)
{
	return insertOrAssignImpl<N, true>(semantics, index, hint, label,
	                                   [value](auto) { return value; });
}

template <std::size_t N, class UnaryFunction,
          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
std::pair<iterator, bool> insertOrAssign(std::unique_ptr<Semantic[]>& semantics,
                                         offset_t index, label_t label, UnaryFunction f)
{
	return insertOrAssignImpl<N, true>(semantics, index, label, f);
}

template <std::size_t N, class InputIt, class UnaryFunction,
          class    = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>,
          typename = std::enable_if_t<std::is_base_of_v<
              std::input_iterator_tag,
              typename std::iterator_traits<InputIt>::iterator_category>>>
void insertOrAssign(std::unique_ptr<Semantic[]>& semantics, offset_t index, InputIt first,
                    InputIt last, UnaryFunction f)
{
	insertOrAssignImpl<N, true>(semantics, index, first, last, f);
}

//
// Assign
//

// index
// apply function f to each label of node with index which occurs in the SemanticRangeSet
// ranges
template <std::size_t N, class UnaryFunction,
          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
void assign(std::unique_ptr<Semantic[]>& semantics, offset_t const index,
            SemanticRangeSet const& ranges, UnaryFunction f)
{
	auto first = begin<N>(semantics, index);
	auto last  = end<N>(semantics, index);
	for (auto range : ranges) {
		if (first == last) {
			break;
		}

		first      = lower_bound(first, last, range.lower());
		auto upper = upper_bound(first, last, range.upper());
		for (; first != upper; ++first) {
			first->value = f(*first);
		}
	}
}

template <std::size_t N>
void assign(std::unique_ptr<Semantic[]>& semantics, offset_t const index,
            SemanticRangeSet const& ranges, value_t value)
{
	assign<N>(semantics, index, ranges, [value](auto) { return value; });
}

template <std::size_t N, class InputIt, class UnaryPredicate, class UnaryFunction,
          class    = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>,
          class    = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>,
          typename = std::enable_if_t<std::is_base_of_v<
              std::input_iterator_tag,
              typename std::iterator_traits<InputIt>::iterator_category>>>
void assign(InputIt first, InputIt last, UnaryPredicate p, UnaryFunction f)
{
	for (auto it = first; it != last; ++it) {
		if (p(*it)) {
			it->value = f(*it);
		}
	}
}

template <std::size_t N, class UnaryPredicate, class UnaryFunction,
          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>,
          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
void assign(std::unique_ptr<Semantic[]>& semantics, offset_t index, UnaryPredicate p,
            UnaryFunction f)
{
	assign<N>(begin<N>(semantics, index), end<N>(semantics, index), p, f);
}

// all
//  apply function f to each label of all nodes which occurs in the SemanticRangeSet
template <std::size_t N, class UnaryFunction,
          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
void assign(std::unique_ptr<Semantic[]>& semantics, SemanticRangeSet const& ranges,
            UnaryFunction f)
{
	for (offset_t i = 0; N != i; ++i) {
		assign<N>(semantics, i, ranges, f);
	}
}

template <std::size_t N>
void assign(std::unique_ptr<Semantic[]>& semantics, SemanticRangeSet const& ranges,
            value_t value)
{
	for (offset_t i = 0; N != i; ++i) {
		assign<N>(semantics, i, ranges, value);
	}
}

template <std::size_t N, class UnaryPredicate, class UnaryFunction,
          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>,
          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
void assign(std::unique_ptr<Semantic[]>& semantics, UnaryPredicate p, UnaryFunction f)
{
	assign<N>(begin<N>(semantics), end<N>(semantics), p, f);
}

//
// Erase Impl
//

// just removes, no resize
template <std::size_t N>
size_type eraseImpl(std::unique_ptr<Semantic[]>& semantics, offset_t index, label_t label)
{
	auto first = begin<N>(semantics, index);
	auto last  = end<N>(semantics, index);

	first = lower_bound(first, last, label);

	if (first == last || first->label != label) {
		return 0;
	}

	std::move(first + 1, last, first);

	return 1;
}

// just removes, no resize
template <std::size_t N>
size_type eraseImpl(std::unique_ptr<Semantic[]>& semantics, offset_t index,
                    SemanticRangeSet ranges)
{
	auto first = begin<N>(semantics, index);
	auto last  = end<N>(semantics, index);

	size_type num = 0;
	for (auto range : ranges) {
		if (first == last) {
			break;
		}

		first      = lower_bound(first, last, range.lower());
		auto upper = upper_bound(first, last, range.upper());

		if (first != upper) {
			num += static_cast<size_type>(std::distance(first, upper));
			last = std::move(upper, last, first);
		}
	}

	return num;
}

//
// Index
//

// get index to which it belongs
template <std::size_t N>
[[nodiscard]] offset_t index(std::unique_ptr<Semantic[]> const& semantics,
                             const_iterator                     it)
{
	assert(it);

	auto     s    = sizes<N>(semantics);
	auto     dist = std::distance(cbegin<N>(semantics), it);
	offset_t i    = 0;
	for (auto offset = s[0]; (N != i && offset <= static_cast<offset_t>(dist)) || s[i] == 0;
	     ++i) {
		offset += s[i];
	}
	return i;
}

//
// Erase
//

// Removes the elements in the range [first; last)
// return Iterator following the last removed element, end() if nothing removed?
template <std::size_t N>
iterator erase(std::unique_ptr<Semantic[]>& semantics, const_iterator first,
               const_iterator last)
{
	auto first_index = index<N>(semantics, first);
	auto last_index  = index<N>(semantics, last);

	if (first == last || cend<N>(semantics) == first ||
	    cend<N>(semantics, first_index) == first) {
		return end<N>(semantics);
	} else if (cbegin<N>(semantics) == first && cend<N>(semantics) == last) {
		clear<N>(semantics);
		return end<N>(semantics);
	}

	auto s = sizes<N>(semantics);

	std::size_t r_offset = offset<N>(semantics, first_index);

	if (first_index == last_index) {
		s[first_index] -= static_cast<size_type>(std::distance(first, last));
		if (0 != s[first_index] && cend<N>(semantics, first_index) != last) {
			r_offset = std::distance(cbegin<N>(semantics), first);
			auto beg = begin<N>(semantics) + std::distance(cbegin<N>(semantics), last);
			auto dst = begin<N>(semantics) + std::distance(cbegin<N>(semantics), first);
			std::move(beg, end<N>(semantics, first_index), dst);
			// r_offset = std::distance(begin<N>(semantics), l);
		} else {
			// r_offset = std::distance(begin<N>(semantics), begin<N>(semantics, first_index));
			r_offset = std::distance(begin<N>(semantics),
			                         begin<N>(semantics, first_index) + s[first_index]);
		}
	} else {
		// Handle first
		s[first_index] -=
		    static_cast<size_type>(std::distance(first, cend<N>(semantics, first_index)));

		r_offset += s[first_index];

		// Handle middle
		for (; ++first_index != last_index;) {
			s[first_index] = 0;
		}

		// Handle last
		auto dist = std::distance(cbegin<N>(semantics, last_index), last);
		s[last_index] -= static_cast<size_type>(dist);
		if (0 != dist && 0 != s[last_index]) {
			auto beg = begin<N>(semantics) + std::distance(cbegin<N>(semantics), last);
			auto l =
			    std::move(beg, end<N>(semantics, last_index), begin<N>(semantics, last_index));
			r_offset += std::distance(begin<N>(semantics, last_index), l);
		}
	}

	resize<N>(semantics, s);

	return begin<N>(semantics) + r_offset;
}

template <std::size_t N>
size_type erase(std::unique_ptr<Semantic[]>& semantics, label_t label)
{
	if (empty<N>(semantics)) {
		return 0;
	}

	auto      s   = sizes<N>(semantics);
	size_type sum = 0;
	for (offset_t i = 0; N != i; ++i) {
		auto t = eraseImpl<N>(semantics, i, label);
		s[i] -= t;
		sum += t;
	}

	resize<N>(semantics, s);

	return sum;
}

template <std::size_t N>
size_type erase(std::unique_ptr<Semantic[]>& semantics, SemanticRangeSet const& ranges)
{
	if (ranges.empty() || empty<N>(semantics)) {
		return 0;
	}

	auto      s   = sizes<N>(semantics);
	size_type sum = 0;
	for (offset_t i = 0; N != i; ++i) {
		auto t = eraseImpl<N>(semantics, i, ranges);
		s[i] -= t;
		sum += t;
	}

	resize<N>(semantics, s);

	return sum;
}

template <std::size_t N>
size_type erase(std::unique_ptr<Semantic[]>& semantics, offset_t const index,
                label_t label)
{
	auto s = eraseImpl<N>(semantics, index, label);
	resize<N>(semantics, index, size<N>(semantics, index) - s);
	return s;
}

template <std::size_t N>
size_type erase(std::unique_ptr<Semantic[]>& semantics, offset_t const index,
                SemanticRangeSet const& ranges)
{
	if (ranges.empty() || empty<N>(semantics, index)) {
		return 0;
	}

	auto s = eraseImpl<N>(semantics, index, ranges);
	resize<N>(semantics, index, size<N>(semantics, index) - s);
	return s;
}

//
// Erase if Impl
//

// just removes, no resize
template <std::size_t N, class UnaryPredicate,
          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
size_type eraseIfImpl(std::unique_ptr<Semantic[]>& semantics, offset_t index,
                      UnaryPredicate p)
{
	auto first = begin<N>(semantics, index);
	auto last  = end<N>(semantics, index);

	first = std::find_if(first, last, p);

	if (first == last) {
		return 0;
	}

	size_type num = 1;
	for (auto it = first; ++it != last;) {
		if (p(*it)) {
			++num;
		} else {
			*first++ = std::move(*it);
		}
	}

	return num;
}

template <std::size_t N, class UnaryPredicate,
          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
size_type eraseIfImpl(std::unique_ptr<Semantic[]>& semantics, offset_t index,
                      SemanticRangeSet ranges, UnaryPredicate p)
{
	auto first = begin<N>(semantics, index);
	auto last  = end<N>(semantics, index);

	size_type num = 0;
	for (auto range : ranges) {
		if (first == last) {
			break;
		}

		first      = lower_bound(first, last, range.lower());
		auto upper = upper_bound(first, last, range.upper());

		first = std::find_if(first, upper, p);

		if (first != upper) {
			++num;
			for (auto it = first; ++it != upper;) {
				if (p(*it)) {
					++num;
				} else {
					*first++ = std::move(*it);
				}
			}

			if (first != upper) {
				last = std::move(upper, last, first);
			}
		}
	}

	return num;
}

//
// Erase if
//

template <std::size_t N, class UnaryPredicate,
          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
size_type eraseIf(std::unique_ptr<Semantic[]>& semantics, UnaryPredicate p)
{
	if (empty<N>(semantics)) {
		return 0;
	}

	auto      s   = sizes<N>(semantics);
	size_type sum = 0;
	for (offset_t i = 0; N != i; ++i) {
		auto t = eraseIfImpl<N>(semantics, i, p);
		s[i] -= t;
		sum += t;
	}

	resize<N>(semantics, s);

	return sum;
}

template <std::size_t N, class UnaryPredicate,
          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
size_type eraseIf(std::unique_ptr<Semantic[]>& semantics, SemanticRangeSet const& ranges,
                  UnaryPredicate p)
{
	if (ranges.empty() || empty<N>(semantics)) {
		return 0;
	}

	auto      s   = sizes<N>(semantics);
	size_type sum = 0;
	for (offset_t i = 0; N != i; ++i) {
		auto t = eraseIfImpl<N>(semantics, i, ranges, p);
		s[i] -= t;
		sum += t;
	}

	resize<N>(semantics, s);

	return sum;
}

template <std::size_t N, class UnaryPredicate,
          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
size_type eraseIf(std::unique_ptr<Semantic[]>& semantics, offset_t const index,
                  UnaryPredicate p)
{
	auto s = eraseIfImpl<N>(semantics, index, p);
	resize<N>(semantics, index, size<N>(semantics, index) - s);
	return s;
}

template <std::size_t N, class UnaryPredicate,
          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
size_type eraseIf(std::unique_ptr<Semantic[]>& semantics, offset_t const index,
                  SemanticRangeSet const& ranges, UnaryPredicate p)
{
	if (ranges.empty() || empty<N>(semantics, index)) {
		return 0;
	}

	auto s = eraseIfImpl<N>(semantics, index, ranges, p);
	resize<N>(semantics, index, size<N>(semantics, index) - s);
	return s;
}

//
// to string
//

// template <std::size_t N>
// std::string toString(std::unique_ptr<Semantic[]> const& semantics, offset_t index)
// {
// 	std::ostringstream result;

// 	for (auto it = begin<N>(semantics, index); it != end<N>(semantics, index); ++it) {
// 		result << it->label << ":" << it->value << " \n";
// 	}

// 	return result.str();
// }

template <std::size_t N>
std::string toString(std::unique_ptr<Semantic[]> const& semantics, offset_t index)
{
	std::ostringstream result;
	result << "{";

	// if (semantic::empty<N>(semantics, index)) {
	// 	return {};
	// } else
	// {
	auto b = begin<N>(semantics, index);
	auto e = end<N>(semantics, index);

	// for (auto it = begin<N>(semantics, index); it != end<N>(semantics, index); ++it) {
	for (auto it = b; it != e; ++it) {
		// result << it->label << ":" << it->value;
		result << *it;
		if ((it + 1) != e) {
			result << " | ";
		}
	}
	// }
	result << "}";
	return result.str();
}

// template <std::size_t N>
// std::string toString(std::unique_ptr<Semantic[]> const& semantics)
// {
// 	std::ostringstream result;
// 	result << N << " Nodes\n";

// 	auto s = sizes<N>(semantics);
// 	for (std::size_t i = 0; i < s.size(); ++i) {
// 		result << "Node " << i << ": " << s[i] << "\n";
// 		result << toString<N>(semantics, static_cast<offset_t>(i));
// 	}
// 	return result.str();
// }

template <std::size_t N>
std::string toString(std::unique_ptr<Semantic[]> const& semantics)
{
	// if (semantic::empty<N>(semantics)) {
	// 	return {};
	// }

	std::ostringstream result;

	result << "{";

	auto s = sizes<N>(semantics);
	for (size_type i = 0; i < s.size(); ++i) {
		result << toString<N>(semantics, static_cast<offset_t>(i));
		if (i < s.size() - 1) {
			result << "} {";
		}
	}

	result << "}";
	return result.str();
}

}  // namespace ufo::semantic

#endif  // UFO_MAP_SEMANTIC_UTIL_H