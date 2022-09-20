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

#ifndef UFO_ALGORITHM_H
#define UFO_ALGORITHM_H

// STL
#include <algorithm>
#include <cassert>
#include <iterator>
#include <numeric>
#include <type_traits>
#include <vector>

namespace ufo
{
using Permuation = std::vector<std::size_t>;

/*!
 * Retrieve the permutation to sort the elements in the range [first, last) in
 * non-descending order. The order of equal elements is not guaranteed.
 *
 * @param first,last The range of elements to sort.
 * @return The permutation.
 */
template <class RandomIt>
Permuation sortPermutation(RandomIt first, RandomIt last)
{
	Permuation p(std::distance(first, last));
	std::iota(std::begin(p), std::end(p), 0);
	std::sort(std::begin(p), std::end(p), [first](std::size_t i, std::size_t j) {
		return *std::next(first, i) < *std::next(first, j);
	});
	return p;
}

/*!
 * Retrieve the permutation to sort the elements in the range [first, last) in
 * non-descending order. The order of equal elements is not guaranteed.
 *
 * @param first,last The range of elements to sort.
 * @param comp Comparison function object which returns true if the first argument is less
 * than (i.e., is ordered before) the second.
 * @return The permutation.
 */
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

/*!
 * Apply permutation to the elements in the range [first, last).
 *
 * @code
 * auto perm = sortPermutation(a, b);
 * applyPermutation(a, b, perm);
 * @endcode
 *
 * @param first,last The range of elements to permutate.
 * @param perm The permutation.
 */
template <class RandomIt>
void applyPermutation(RandomIt first, RandomIt last, Permuation const& perm)
{
	auto const size = perm.size();

	assert(std::distance(first, last) == size);

	std::vector<bool> done(size, false);
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

//
// Min
//

template <class InputIt>
[[nodiscard]] typename std::iterator_traits<InputIt>::value_type min(InputIt first,
                                                                     InputIt last)
{
	using value_type = typename std::iterator_traits<InputIt>::value_type;
	auto m = std::numeric_limits<value_type>::max();
	for (; first != last; ++first) {
		if (*first < m) {
			m = *first;
		}
	}
	return m;
}

template <class InputIt, class UnaryFunction>
[[nodiscard]] std::invoke_result_t<UnaryFunction,
                                   typename std::iterator_traits<InputIt>::value_type>
min(InputIt first, InputIt last, UnaryFunction f)
{
	using value_type = typename std::iterator_traits<InputIt>::value_type;
	auto m = std::numeric_limits<value_type>::max();
	for (; first != last; ++first) {
		auto const c = f(*first);
		if (c < m) {
			m = c;
		}
	}
	return m;
}

template <class Container>
[[nodiscard]] auto min(Container const& data)
{
	return min(std::cbegin(data), std::cend(data));
}

template <class Container, class UnaryFunction>
[[nodiscard]] auto min(Container const& data, UnaryFunction f)
{
	return min(std::cbegin(data), std::cend(data), f);
}

//
// Max
//

template <class InputIt>
[[nodiscard]] typename std::iterator_traits<InputIt>::value_type max(InputIt first,
                                                                     InputIt last)
{
	using value_type = typename std::iterator_traits<InputIt>::value_type;
	auto m = std::numeric_limits<value_type>::lowest();
	for (; first != last; ++first) {
		if (*first > m) {
			m = *first;
		}
	}
	return m;
}

template <class InputIt, class UnaryFunction>
[[nodiscard]] std::invoke_result_t<UnaryFunction,
                                   typename std::iterator_traits<InputIt>::value_type>
max(InputIt first, InputIt last, UnaryFunction f)
{
	using value_type = typename std::iterator_traits<InputIt>::value_type;
	auto m = std::numeric_limits<value_type>::lowest();
	for (; first != last; ++first) {
		auto const c = f(*first);
		if (c > m) {
			m = c;
		}
	}
	return m;
}

template <class Container>
[[nodiscard]] auto max(Container const& data)
{
	return max(std::cbegin(data), std::cend(data));
}

template <class Container, class UnaryFunction>
[[nodiscard]] auto max(Container const& data, UnaryFunction f)
{
	return max(std::cbegin(data), std::cend(data), f);
}

//
// Sum
//

template <class InputIt>
[[nodiscard]] typename std::iterator_traits<InputIt>::value_type sum(InputIt first,
                                                                     InputIt last)
{
	return std::reduce(first, last);
}

template <class InputIt, class UnaryFunction>
[[nodiscard]] std::invoke_result_t<UnaryFunction,
                                   typename std::iterator_traits<InputIt>::value_type>
sum(InputIt first, InputIt last, UnaryFunction f)
{
	using value_type =
	    std::invoke_result_t<UnaryFunction,
	                         typename std::iterator_traits<InputIt>::value_type>;
	value_type s{};
	for (; first != last; ++first) {
		s += f(*first);
	}
	return s;
}

template <class Container>
[[nodiscard]] auto sum(Container const& data)
{
	return sum(std::cbegin(data), std::cend(data));
}

template <class Container, class UnaryFunction>
[[nodiscard]] auto sum(Container const& data, UnaryFunction f)
{
	return sum(std::cbegin(data), std::cend(data), f);
}

//
// Mean
//

template <class InputIt>
[[nodiscard]] typename std::iterator_traits<InputIt>::value_type mean(InputIt first,
                                                                      InputIt last)
{
	// TODO: Improve
	double s = 0.0;
	std::size_t num = 0;
	for (; first != last; ++first, ++num) {
		s += *first;
	}
	return s / num;
}

template <class InputIt, class UnaryFunction>
[[nodiscard]] std::invoke_result_t<UnaryFunction,
                                   typename std::iterator_traits<InputIt>::value_type>
mean(InputIt first, InputIt last, UnaryFunction f)
{
	// TODO: Improve
	double s = 0.0;
	std::size_t num = 0;
	for (; first != last; ++first, ++num) {
		s += f(*first);
	}
	return s / num;
}

template <class Container>
[[nodiscard]] auto mean(Container const& data)
{
	return mean(std::cbegin(data), std::cend(data));
}

template <class Container, class UnaryFunction>
[[nodiscard]] auto mean(Container const& data, UnaryFunction f)
{
	return mean(std::cbegin(data), std::cend(data), f);
}

//
// All of
//

template <class Container, class UnaryPredicate>
bool all_of(Container const& data, UnaryPredicate p)
{
	return std::all_of(std::begin(data), std::end(data), p);
}

template <class ExecutionPolicy, class Container, class UnaryPredicate>
bool all_of(ExecutionPolicy&& policy, Container const& data, UnaryPredicate p)
{
	return std::all_of(std::forward<ExecutionPolicy>(policy), std::begin(data),
	                   std::end(data), p);
}

//
// Any of
//

template <class Container, class UnaryPredicate>
bool any_of(Container const& data, UnaryPredicate p)
{
	return std::any_of(std::begin(data), std::end(data), p);
}

template <class ExecutionPolicy, class Container, class UnaryPredicate>
bool any_of(ExecutionPolicy&& policy, Container const& data, UnaryPredicate p)
{
	return std::any_of(std::forward<ExecutionPolicy>(policy), std::begin(data),
	                   std::end(data), p);
}

//
// None of
//

template <class Container, class UnaryPredicate>
bool none_of(Container const& data, UnaryPredicate p)
{
	return std::none_of(std::begin(data), std::end(data), p);
}

template <class ExecutionPolicy, class Container, class UnaryPredicate>
bool none_of(ExecutionPolicy&& policy, Container const& data, UnaryPredicate p)
{
	return std::none_of(std::forward<ExecutionPolicy>(policy), std::begin(data),
	                    std::end(data), p);
}

//
// For each
//

template <class Container, class UnaryFunction>
UnaryFunction for_each(Container const& data, UnaryFunction f)
{
	return std::for_each(std::begin(data), std::end(data), f);
}

template <class ExecutionPolicy, class Container, class UnaryFunction>
void for_each(ExecutionPolicy&& policy, Container const& data, UnaryFunction f)
{
	std::for_each(std::forward<ExecutionPolicy>(policy), std::begin(data), std::end(data),
	              f);
}

//
// For each n
//

// FIXME: Return type
template <class Container, class Size, class UnaryFunction>
auto for_each_n(Container const& first, Size n, UnaryFunction f)
{
	return std::for_each_n(std::begin(first), n, f);
}

// FIXME: Return type
template <class ExecutionPolicy, class Container, class Size, class UnaryFunction>
auto for_each_n(ExecutionPolicy&& policy, Container const& first, Size n, UnaryFunction f)
{
	return std::for_each_n(std::forward<ExecutionPolicy>(policy), std::begin(first), n, f);
}

//
// Count
//

// FIXME: Return type
template <class Container, class T>
auto count(Container const& data, T const& value)
{
	return std::count(std::begin(data), std::end(data), value);
}

// FIXME: Return type
template <class ExecutionPolicy, class Container, class T>
auto count(ExecutionPolicy&& policy, Container const& data, T const& value)
{
	return std::count(std::forward<ExecutionPolicy>(policy), std::begin(data),
	                  std::end(data), value);
}

//
// Count if
//

// FIXME: Return type
template <class Container, class UnaryPredicate>
auto count_if(Container const& data, UnaryPredicate p)
{
	return std::count_if(std::begin(data), std::end(data), p);
}

// FIXME: Return type
template <class ExecutionPolicy, class Container, class UnaryPredicate>
auto count_if(ExecutionPolicy&& policy, Container const& data, UnaryPredicate p)
{
	return std::count_if(std::forward<ExecutionPolicy>(policy), std::begin(data),
	                     std::end(data), p);
}

//
// Mismatch
//

// // FIXME: Return type
// template <class Container1, class Container2>
// auto mismatch(Container1&& data1, Container2&& data2)
// {
// 	return std::mismatch(std::begin(data1), std::end(data1), std::begin(data2));
// }

// FIXME: Return type
template <class ExecutionPolicy, class Container1, class Container2>
auto mismatch(ExecutionPolicy&& policy, Container1&& data1, Container2&& data2)
{
	return std::mismatch(std::forward<ExecutionPolicy>(policy), std::begin(data1),
	                     std::end(data1), std::begin(data2));
}

// FIXME: Return type
template <class Container1, class Container2, class BinaryPredicate>
auto mismatch(Container1&& data1, Container2&& data2, BinaryPredicate p)
{
	return std::mismatch(std::begin(data1), std::end(data1), std::begin(data2), p);
}

// FIXME: Return type
template <class ExecutionPolicy, class Container1, class Container2,
          class BinaryPredicate>
auto mismatch(ExecutionPolicy&& policy, Container1&& data1, Container2&& data2,
              BinaryPredicate p)
{
	return std::mismatch(std::forward<ExecutionPolicy>(policy), std::begin(data1),
	                     std::end(data1), std::begin(data2), p);
}

// FIXME: Return type
template <class Container1, class Container2>
auto mismatch(Container1&& data1, Container2&& data2)
{
	return std::mismatch(std::begin(data1), std::end(data1), std::begin(data2),
	                     std::end(data2));
}

//
// Find
//

// FIXME: Return type
template <class Container, class T>
auto find(Container const& data, T const& value)
{
	return std::find(std::begin(data), std::end(data), value);
}

// FIXME: Return type
template <class ExecutionPolicy, class Container, class T>
auto find(ExecutionPolicy&& policy, Container const& data, T const& value)
{
	return std::find(std::forward<ExecutionPolicy>(policy), std::begin(data),
	                 std::end(data), value);
}

//
// Find if
//

// FIXME: Return type
template <class Container, class UnaryPredicate>
auto find_if(Container const& data, UnaryPredicate p)
{
	return std::find_if(std::begin(data), std::end(data), p);
}

// FIXME: Return type
template <class ExecutionPolicy, class Container, class UnaryPredicate>
auto find_if(ExecutionPolicy&& policy, Container const& data, UnaryPredicate p)
{
	return std::find_if(std::forward<ExecutionPolicy>(policy), std::begin(data),
	                    std::end(data), p);
}

//
// Find if not
//

// FIXME: Return type
template <class Container, class UnaryPredicate>
auto find_if_not(Container const& data, UnaryPredicate q)
{
	return std::find_if_not(std::begin(data), std::end(data), q);
}

// FIXME: Return type
template <class ExecutionPolicy, class Container, class UnaryPredicate>
auto find_if_not(ExecutionPolicy&& policy, Container const& data, UnaryPredicate q)
{
	return std::find_if_not(std::forward<ExecutionPolicy>(policy), std::begin(data),
	                        std::end(data), q);
}

//
// Find end
//

// FIXME: Return type
template <class Container1, class Container2>
auto find_end(Container1&& data, Container2&& s_data)
{
	return std::find_end(std::begin(data), std::end(data), std::begin(s_data),
	                     std::end(s_data));
}

// FIXME: Return type
template <class ExecutionPolicy, class Container1, class Container2>
auto find_end(ExecutionPolicy&& policy, Container1&& data, Container2&& s_data)
{
	return std::find_end(std::forward<ExecutionPolicy>(policy), std::begin(data),
	                     std::end(data), std::begin(s_data), std::end(s_data));
}

// FIXME: Return type
template <class Container1, class Container2, class BinaryPredicate>
auto find_end(Container1&& data, Container2&& s_data, BinaryPredicate p)
{
	return std::find_end(std::begin(data), std::end(data), std::begin(s_data),
	                     std::end(s_data), p);
}

// FIXME: Return type
template <class ExecutionPolicy, class Container1, class Container2,
          class BinaryPredicate>
auto find_end(ExecutionPolicy&& policy, Container1&& data, Container2&& s_data,
              BinaryPredicate p)
{
	return std::find_end(std::forward<ExecutionPolicy>(policy), std::begin(data),
	                     std::end(data), std::begin(s_data), std::end(s_data), p);
}

//
// Find first of
//

// FIXME: Return type
template <class Container1, class Container2>
auto find_first_of(Container1&& data, Container2&& s_data)
{
	return std::find_first_of(std::begin(data), std::end(data), std::begin(s_data),
	                          std::end(s_data));
}

// FIXME: Return type
template <class ExecutionPolicy, class Container1, class Container2>
auto find_first_of(ExecutionPolicy&& policy, Container1&& data, Container2&& s_data)
{
	return std::find_first_of(std::forward<ExecutionPolicy>(policy), std::begin(data),
	                          std::end(data), std::begin(s_data), std::end(s_data));
}

// FIXME: Return type
template <class Container1, class Container2, class BinaryPredicate>
auto find_first_of(Container1&& data, Container2&& s_data, BinaryPredicate p)
{
	return std::find_first_of(std::begin(data), std::end(data), std::begin(s_data),
	                          std::end(s_data), p);
}

// FIXME: Return type
template <class ExecutionPolicy, class Container1, class Container2,
          class BinaryPredicate>
auto find_first_of(ExecutionPolicy&& policy, Container1&& data, Container2&& s_data,
                   BinaryPredicate p)
{
	return std::find_first_of(std::forward<ExecutionPolicy>(policy), std::begin(data),
	                          std::end(data), std::begin(s_data), std::end(s_data), p);
}

//
// Adjecent find
//

// FIXME: Return type
template <class Container>
auto adjacent_find(Container const& data)
{
	return std::adjacent_find(std::begin(data), std::end(data));
}

// FIXME: Return type
template <class ExecutionPolicy, class Container>
auto adjacent_find(ExecutionPolicy&& policy, Container const& data)
{
	return std::adjacent_find(std::forward<ExecutionPolicy>(policy), std::begin(data),
	                          std::end(data));
}

// FIXME: Return type
template <class Container, class BinaryPredicate>
auto adjacent_find(Container const& data, BinaryPredicate p)
{
	return std::adjacent_find(std::begin(data), std::end(data), p);
}

// FIXME: Return type
template <class ExecutionPolicy, class Container, class BinaryPredicate>
auto adjacent_find(ExecutionPolicy&& policy, Container const& data, BinaryPredicate p)
{
	return std::adjacent_find(std::forward<ExecutionPolicy>(policy), std::begin(data),
	                          std::end(data), p);
}

//
// Search
//

// FIXME: Return type
template <class Container1, class Container2>
auto search(Container1&& data, Container2&& s_data)
{
	return std::search(std::begin(data), std::end(data), std::begin(s_data),
	                   std::end(s_data));
}

// FIXME: Return type
template <class ExecutionPolicy, class Container1, class Container2>
auto search(ExecutionPolicy&& policy, Container1&& data, Container2&& s_data)
{
	return std::search(std::forward<ExecutionPolicy>(policy), std::begin(data),
	                   std::end(data), std::begin(s_data), std::end(s_data));
}

template <class Container1, class Container2, class BinaryPredicate>
auto search(Container1&& data, Container2&& s_data, BinaryPredicate p)
{
	return std::search(std::begin(data), std::end(data), std::begin(s_data),
	                   std::end(s_data), p);
}

// FIXME: Return type
template <class ExecutionPolicy, class Container1, class Container2,
          class BinaryPredicate>
auto search(ExecutionPolicy&& policy, Container1&& data, Container2&& s_data,
            BinaryPredicate p)
{
	return std::search(std::forward<ExecutionPolicy>(policy), std::begin(data),
	                   std::end(data), std::begin(s_data), std::end(s_data), p);
}

// FIXME: Return type
template <class Container, class Searcher>
auto search(Container const& data, Searcher const& searcher)
{
	return std::search(std::begin(data), std::end(data), searcher);
}

//
// Search n
//

// FIXME: Return type
template <class Container, class Size, class T>
auto search_n(Container const& data, Size count, T const& value)
{
	return std::search_n(std::begin(data), std::end(data), count, value);
}

// FIXME: Return type
template <class ExecutionPolicy, class Container, class Size, class T>
auto search_n(ExecutionPolicy&& policy, Container const& data, Size count, T const& value)
{
	return std::search_n(std::forward<ExecutionPolicy>(policy), std::begin(data),
	                     std::end(data), count, value);
}

// FIXME: Return type
template <class Container, class Size, class T, class BinaryPredicate>
auto search_n(Container const& data, Size count, T const& value, BinaryPredicate p)
{
	return std::search_n(std::begin(data), std::end(data), count, value, p);
}

// FIXME: Return type
template <class ExecutionPolicy, class Container, class Size, class T,
          class BinaryPredicate>
auto search_n(ExecutionPolicy&& policy, Container const& data, Size count, T const& value,
              BinaryPredicate p)
{
	return std::search_n(std::forward<ExecutionPolicy>(policy), std::begin(data),
	                     std::end(data), count, value, p);
}
}  // namespace ufo

#endif  // UFO_ALGORITHM_H