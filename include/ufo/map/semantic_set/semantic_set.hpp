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

#ifndef UFO_MAP_SEMANTIC_SET_HPP
#define UFO_MAP_SEMANTIC_SET_HPP

// UFO
#include <ufo/container/range.hpp>
#include <ufo/container/range_map.hpp>
#include <ufo/util/iterator_wrapper.hpp>
#include <ufo/util/type_traits.hpp>
// #include <ufo/map/semantic/semantics_reference.h>
#include <ufo/util/bit_set.hpp>
#include <ufo/map/semantic/semantic.hpp>
#include <ufo/map/semantic_set/semantic_util.hpp>

// STL
#include <algorithm>
#include <cstddef>  // For std::ptrdiff_t
#include <functional>
#include <initializer_list>
#include <istream>
#include <iterator>  // For std::random_access_iterator_tag / std::contiguous_iterator_tag
#include <memory>
#include <ostream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace ufo
{
template <std::size_t N = 1>
class SemanticSet
{
 public:
	// Tags
	using value_type             = Semantic;
	using size_type              = label_t;
	using difference_type        = std::ptrdiff_t;
	using reference              = Semantic &;  // TODO: Make label const
	using const_reference        = Semantic const &;
	using pointer                = Semantic *;
	using const_pointer          = Semantic const *;
	using iterator               = Semantic *;  // TODO: Make label const
	using const_iterator         = Semantic const *;
	using reverse_iterator       = std::reverse_iterator<iterator>;
	using const_reverse_iterator = std::reverse_iterator<const_iterator>;

	// -1 half rounded up (i.e., the number of elements needed to store the sizes of the N
	// semantic containers)
	static constexpr std::size_t N_H = 1 + (N - 1) / 2;

	// Data
	// TODO: Most significate size bits indicates whether the labels have change and the
	// second most significate if the values have changed for the particular index.
	std::unique_ptr<Semantic[]> semantics;

	SemanticSet() {}

	// copy constructor
	SemanticSet(SemanticSet const &other)
	{
		for (offset_t i{}; i != N; ++i) {
			set(i, other.begin(i), other.end(i));
		}
	}

	// move constructor
	SemanticSet(SemanticSet &&other) noexcept : semantics(std::move(other.semantics)) {}

	// move assignment operator
	SemanticSet &operator=(SemanticSet &&other) noexcept
	{
		semantics = std::move(other.semantics);
		return *this;
	}

	// copy assignment operator
	SemanticSet &operator=(SemanticSet const &other)
	{
		for (offset_t i{}; i != N; ++i) {
			set(i, other.begin(i), other.end(i));
		}
		return *this;
	}

	~SemanticSet() {}

	//
	// Size
	//

	[[nodiscard]] static constexpr std::size_t semanticSize() { return N; }

	[[nodiscard]] size_type semanticAllocSize() const
	{
		return empty() ? 0 : semanticSize() + N_H;
	}

	//
	// Fill
	//

	// fill all our nodes with data from parent node with specified index
	void fill(SemanticSet const &parent, offset_t const index)
	{
		semantic::resizeLazy<N>(semantics, parent.size(index));
		auto first = parent.begin(index);
		auto last  = parent.end(index);
		for (offset_t i = 0; N != i; ++i) {
			std::copy(first, last, begin(i));
		}
	}

	//
	// Is collapsible
	//

	// check if all nodes have exactly the same contents
	[[nodiscard]] bool isPrunable() const
	{
		auto first = cbegin(0);
		auto last  = cend(0);
		for (offset_t i = 1; N != i; ++i) {
			if (!std::equal(first, last, cbegin(i), cend(i))) {
				return false;
			}
		}
		return true;
	}

	// // check if all children have exactly the same elements as parent at specified index
	// [[nodiscard]] bool isCollapsible(SemanticSet const &parent, offset_t const index)
	// const
	// {
	// 	auto first = parent.begin(index);
	// 	auto last = parent.end(index);
	// 	for (offset_t i = 0; N != i; ++i) {
	// 		if (!std::equal(first, last, cbegin(i), cend(i))) {
	// 			return false;
	// 		}
	// 	}
	// 	return true;
	// }

	//
	// Iterators
	//

	iterator begin() noexcept { return semantic::begin<N>(semantics); }

	const_iterator begin() const noexcept { return semantic::begin<N>(semantics); }

	const_iterator cbegin() const noexcept { return begin(); }

	iterator end() noexcept { return semantic::end<N>(semantics); }

	const_iterator end() const noexcept { return semantic::end<N>(semantics); }

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

	iterator begin(offset_t const index) noexcept
	{
		return semantic::begin<N>(semantics, index);
	}

	const_iterator begin(offset_t const index) const noexcept
	{
		return semantic::begin<N>(semantics, index);
	}

	const_iterator cbegin(offset_t const index) const noexcept { return begin(index); }

	iterator end(offset_t const index) noexcept
	{
		return semantic::end<N>(semantics, index);
	}

	const_iterator end(offset_t const index) const noexcept
	{
		return semantic::end<N>(semantics, index);
	}

	const_iterator cend(offset_t const index) const noexcept { return end(index); }

	//
	// Reverse index iterators
	//

	reverse_iterator rbegin(offset_t const index) noexcept
	{
		return std::make_reverse_iterator(end(index));
	}

	const_reverse_iterator rbegin(offset_t const index) const noexcept
	{
		return std::make_reverse_iterator(end(index));
	}

	const_reverse_iterator crbegin(offset_t const index) const noexcept
	{
		return rbegin(index);
	}

	reverse_iterator rend(offset_t const index) noexcept
	{
		return std::make_reverse_iterator(begin(index));
	}

	const_reverse_iterator rend(offset_t const index) const noexcept
	{
		return std::make_reverse_iterator(begin(index));
	}

	const_reverse_iterator crend(offset_t const index) const noexcept
	{
		return rend(index);
	}

	//
	// Empty
	//

	[[nodiscard]] bool empty() const noexcept { return semantic::empty<N>(semantics); }

	[[nodiscard]] bool empty(offset_t const index) const
	{
		return semantic::empty<N>(semantics, index);
	}

	//
	// Size
	//

	[[nodiscard]] size_type size() const { return semantic::size<N>(semantics); }

	[[nodiscard]] size_type size(offset_t const index) const
	{
		return semantic::size<N>(semantics, index);
	}

	[[nodiscard]] std::array<size_type, N> sizes() const
	{
		return semantic::sizes<N>(semantics);
	}

	//
	// Resize
	//

	// resize data container without copying existing elements to the correct location
	void resizeLazy(std::array<size_type, N> const &new_sizes)
	{
		semantic::resizeLazy<N>(semantics, new_sizes);
	}

	//
	// Offset
	//

	[[nodiscard]] std::size_t offset(offset_t const index) const
	{
		return semantic::offset<N>(semantics, index);
	}

	//
	// Clear
	//

	void clear() noexcept { semantic::clear<N>(semantics); }

	void clear(offset_t const index) { semantic::clear<N>(semantics, index); }

	//
	// Set
	//

	void set(SemanticSet const &semantics)
	{
		semantic::resizeLazy<N>(this->semantics, semantics.size());
		auto first = std::begin(semantics);
		auto last  = std::end(semantics);
		for (offset_t i = 0; N != i; ++i) {
			std::copy(first, last, begin(i));
		}
	}

	void set(offset_t index, SemanticSet const &semantics)
	{
		semantic::resize<N>(this->semantics, index, semantics.size());
		std::copy(std::begin(semantics), std::end(semantics), begin(index));
	}

	template <class InputIt>
	void set(offset_t index, InputIt first, InputIt last)
	{
		set(index, SemanticSet(first, last));

		// semantic::resize<N>(this->semantics, index,
		//                     static_cast<size_type>(std::distance(first, last)));
		// // FIXME: assumes first and last is sorted, should it?
		// std::copy(first, last, begin(index));
	}

	//
	// TODO: Change label
	//

	// void changeLabel(label_t old_label, label_t new_label)
	// {
	// 	// TODO: Implement
	// }

	// void changeLabel(offset_t index, label_t old_label, label_t new_label)
	// {
	// 	// TODO: Implement
	// }

	//
	// Insert
	//

	void insert(label_t label, value_t value)
	{
		semantic::insert<N>(semantics, label, value);
	}

	void insert(offset_t index, label_t label, value_t value)
	{
		semantic::insert<N>(semantics, index, label, value);
	}

	template <class InputIt,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	void insert(InputIt first, InputIt last)
	{
		semantic::insert<N>(semantics, first, last);
	}

	template <class InputIt,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	void insert(offset_t index, InputIt first, InputIt last)
	{
		semantic::insert<N>(semantics, index, first, last);
	}

	//
	// Insert or assign
	//

	void insertOrAssign(label_t label, value_t value)
	{
		semantic::insertOrAssign<N>(semantics, label, value);
	}

	void insertOrAssign(offset_t index, label_t label, value_t value)
	{
		semantic::insertOrAssign<N>(semantics, index, label, value);
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	void insertOrAssign(label_t label, UnaryFunction f)
	{
		semantic::insertOrAssign<N>(semantics, label, f);
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	void insertOrAssign(offset_t index, label_t label, UnaryFunction f)
	{
		semantic::insertOrAssign<N>(semantics, index, label, f);
	}

	// iterator to semantics
	template <class InputIt,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	void insertOrAssign(InputIt first, InputIt last)
	{
		semantic::insertOrAssign<N>(semantics, first, last);
	}

	// iterator to semantics
	template <class InputIt,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	void insertOrAssign(offset_t index, InputIt first, InputIt last)
	{
		semantic::insertOrAssign<N>(semantics, index, first, last);
	}

	// TODO: enable if InputIt is iterator, otherwise this function is called instead of
	// insertOrAssign(index, label, f) iterator to label
	template <class InputIt, class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	void insertOrAssign(InputIt first, InputIt last, UnaryFunction f)
	{
		semantic::insertOrAssign<N>(semantics, first, last, f);
	}

	// iterator to label
	template <class InputIt, class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>,
	          typename = std::enable_if_t<std::is_base_of_v<
	              std::input_iterator_tag,
	              typename std::iterator_traits<InputIt>::iterator_category>>>
	void insertOrAssign(offset_t index, InputIt first, InputIt last, UnaryFunction f)
	{
		semantic::insertOrAssign<N>(semantics, index, first, last, f);
	}

	//
	// Assign
	//

	// all
	void assign(SemanticRange range, value_t value)
	{
		assign(semantics, SemanticRangeSet{range}, value);
	}

	void assign(SemanticRangeSet const &ranges, value_t value)
	{
		semantic::assign<N>(semantics, ranges, value);
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	void assign(SemanticRange range, UnaryFunction f)
	{
		assign(SemanticRangeSet{range}, f);
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	void assign(SemanticRangeSet const &ranges, UnaryFunction f)
	{
		semantic::assign<N>(semantics, ranges, f);
	}

	template <class UnaryPredicate, class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	void assign(UnaryPredicate p, UnaryFunction f)
	{
		semantic::assign<N>(semantics, p, f);
	}

	// index
	void assign(offset_t const index, SemanticRange range, value_t value)
	{
		assign(index, SemanticRangeSet{range}, value);
	}

	void assign(offset_t const index, SemanticRangeSet const &ranges, value_t value)
	{
		semantic::assign<N>(semantics, index, ranges, value);
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	void assign(offset_t const index, SemanticRangeSet const &ranges, UnaryFunction f)
	{
		semantic::assign<N>(semantics, index, ranges, f);
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	void assign(offset_t const index, SemanticRange range, UnaryFunction f)
	{
		assign(index, SemanticRangeSet{range}, f);
	}

	template <class UnaryPredicate, class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	void assign(offset_t const index, UnaryPredicate p, UnaryFunction f)
	{
		semantic::assign<N>(semantics, index, p, f);
	}

	//
	// Erase
	//

	iterator erase(const_iterator pos)
	{
		return semantic::erase<N>(semantics, pos, std::next(pos));
	}

	iterator erase(iterator pos)
	{
		return semantic::erase<N>(semantics, pos, std::next(pos));
	}

	// Removes the elements in the range [first, last)
	iterator erase(const_iterator first, const_iterator last)
	{
		return semantic::erase<N>(semantics, first, last);
	}

	size_type erase(label_t label) { return semantic::erase<N>(semantics, label); }

	size_type erase(SemanticRangeSet const &ranges)
	{
		return semantic::erase<N>(semantics, ranges);
	}

	size_type erase(SemanticRange range) { return erase(SemanticRangeSet{range}); }

	size_type erase(offset_t const index, label_t label)
	{
		return semantic::erase<N>(semantics, index, label);
	}

	size_type erase(offset_t const index, SemanticRangeSet const &ranges)
	{
		return semantic::erase<N>(semantics, index, ranges);
	}

	size_type erase(offset_t const index, SemanticRange range)
	{
		return erase(index, SemanticRangeSet{range});
	}

	//
	// Erase if
	//

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	size_type eraseIf(UnaryPredicate p)
	{
		return semantic::eraseIf<N>(semantics, p);
	}

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	size_type eraseIf(SemanticRangeSet const &ranges, UnaryPredicate p)
	{
		return semantic::eraseIf<N>(semantics, ranges, p);
	}

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	size_type eraseIf(SemanticRange range, UnaryPredicate p)
	{
		return eraseIf(SemanticRangeSet{range}, p);
	}

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	size_type eraseIf(offset_t const index, UnaryPredicate p)
	{
		return semantic::eraseIf<N>(semantics, index, p);
	}

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	size_type eraseIf(offset_t const index, SemanticRangeSet const &ranges,
	                  UnaryPredicate p)
	{
		return semantic::eraseIf<N>(semantics, index, ranges, p);
	}

	template <class UnaryPredicate,
	          class = std::enable_if_t<std::is_invocable<UnaryPredicate, Semantic>::value>>
	size_type eraseIf(offset_t const index, SemanticRange range, UnaryPredicate p)
	{
		return eraseIf(index, SemanticRangeSet{range}, p);
	}

	//
	// At
	//

	std::optional<Semantic> at(offset_t index, label_t label) const
	{
		return semantic::at<N>(semantics, index, label);
	}

	//
	// Value
	//

	std::optional<value_t> value(offset_t index, label_t label) const
	{
		return semantic::value<N>(semantics, index, label);
	}

	//
	// Count
	//

	size_type count(offset_t index, label_t label) const
	{
		return semantic::count<N>(semantics, index, label);
	}

	//
	// Find
	//

	const_iterator find(offset_t index, label_t label) const
	{
		return semantic::find<N>(semantics, index, label);
	}

	//
	// Contains
	//

	bool contains(offset_t index, label_t label) const
	{
		return semantic::contains<N>(semantics, index, label);
	}

	//
	// Equal range
	//

	std::pair<const_iterator, const_iterator> equal_range(offset_t index,
	                                                      label_t  label) const
	{
		return semantic::equal_range<N>(semantics, index, label);
	}

	//
	// Lower bound
	//

	[[nodiscard]] const_iterator lower_bound(offset_t index, label_t label) const
	{
		return semantic::lower_bound<N>(semantics, index, label);
	}

	//
	// Upper bound
	//

	[[nodiscard]] const_iterator upper_bound(offset_t index, label_t label) const
	{
		return semantic::upper_bound<N>(semantics, index, label);
	}

	//
	// All
	//

	[[nodiscard]] bool all(offset_t index, SemanticRange range) const
	{
		return semantic::all<N>(semantics, index, range);
	}

	[[nodiscard]] bool all(offset_t index, SemanticRangeSet const &ranges) const
	{
		return semantic::all<N>(semantics, index, ranges);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool all(offset_t index, UnaryPredicate p) const
	{
		return semantic::all<N>(semantics, index, p);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool all(offset_t index, SemanticRange range, UnaryPredicate p) const
	{
		return semantic::all<N>(semantics, index, range, p);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool all(offset_t index, SemanticRangeSet const &ranges,
	                       UnaryPredicate p) const
	{
		return semantic::all<N>(semantics, index, ranges, p);
	}

	//
	// Any
	//

	[[nodiscard]] bool any(offset_t index, SemanticRange range) const
	{
		return semantic::any<N>(semantics, index, range);
	}

	[[nodiscard]] bool any(offset_t index, SemanticRangeSet const &ranges) const
	{
		return semantic::any<N>(semantics, index, ranges);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool any(offset_t index, UnaryPredicate p) const
	{
		return semantic::any<N>(semantics, index, p);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool any(offset_t index, SemanticRange range, UnaryPredicate p) const
	{
		return semantic::any<N>(semantics, index, range, p);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool any(offset_t index, SemanticRangeSet const &ranges,
	                       UnaryPredicate p) const
	{
		return semantic::any<N>(semantics, index, ranges, p);
	}

	//
	// None
	//

	[[nodiscard]] bool none(offset_t index, SemanticRange range) const
	{
		return semantic::none<N>(semantics, index, range);
	}

	[[nodiscard]] bool none(offset_t index, SemanticRangeSet const &ranges) const
	{
		return semantic::none<N>(semantics, index, ranges);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool none(offset_t index, UnaryPredicate p) const
	{
		return semantic::none<N>(semantics, index, p);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool none(offset_t index, SemanticRange range, UnaryPredicate p) const
	{
		return semantic::none<N>(semantics, index, range, p);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool none(offset_t index, SemanticRangeSet const &ranges,
	                        UnaryPredicate p) const
	{
		return semantic::none<N>(semantics, index, ranges, p);
	}

	std::string toString() const { return semantic::toString<N>(semantics); }
	std::string toString(offset_t index) const
	{
		return semantic::toString<N>(semantics, index);
	}

	//
	// Memory usage
	//

	[[nodiscard]] std::size_t memoryUsage() const
	{
		if (semantic::empty<N>(semantics)) {
			return 0;
		} else {
			auto sizes = semantic::sizes<N>(semantics);
			auto s     = std::accumulate(sizes.begin(), sizes.end(), N_H);
			return s * sizeof(Semantic);
		}
	}

	// [[nodiscard]] std::size_t memoryUsage(offset_t index) const
	// {
	// 	return semantic::empty<N>(semantics, index) ? 0 : semantic::size<N>(semantics,
	// index) * sizeof(Semantic);
	// }

	//
	// Input/output (read/write)
	//

	void read(ReadBuffer &in)
	{
		// read number of semantics in this node
		std::uint64_t s;
		in.read(&s, sizeof(s));

		// resize container accordingly
		std::array<size_type, N> sizes{};
		sizes[0] = s;
		resizeLazy(sizes);

		if (s) {
			// copy data directly to container
			in.read(semantics.get(), memoryUsage());
		}
	}

	void read(ReadBuffer &in, BitSet<N> rf)
	{
		if (rf.all()) {
			read(in);
			return;
		}

		SemanticSet temp;
		temp.read(in);
		auto cur_sizes = sizes();
		auto new_sizes = temp.sizes();
		for (std::size_t i{}; N != i; ++i) {
			new_sizes[i] = rf[i] ? new_sizes[i] : cur_sizes[i];
		}

		resizeLazy(new_sizes);

		for (std::size_t i{}; N != i; ++i) {
			if (rf[i]) {
				std::copy(temp.begin(i), temp.end(i), begin(i));
			}
		}
	}

	void read(ReadBuffer &in, offset_t pos)
	{
		std::uint64_t s;
		in.read(&s, sizeof(s));
		semantic::resize<N>(semantics, pos, s);
		if (s) {
			in.read(semantic::begin<N>(semantics, pos), memoryUsage(pos));
		}
	}

	void write(WriteBuffer &out) const
	{
		auto s = size();
		out.write(&s, sizeof(s));
		if (s) {
			out.write(semantics.get(), memoryUsage());
		}
	}

	void write(WriteBuffer &out, BitSet<N> wf)
	{
		// TODO: Implement
	}

	void write(WriteBuffer &out, offset_t index) const
	{
		std::uint64_t s = size(index);
		out.write(&s, sizeof(s));
		if (s) {
			out.write(begin(index), memoryUsage(index));
		}
	}
};

template <>
class SemanticSet<1>
{
 public:
	//  Tags
	using value_type = Semantic;
	// using size_type              = std::size_t;
	using size_type              = label_t;
	using difference_type        = std::ptrdiff_t;
	using reference              = Semantic &;  // TODO: Make label const
	using const_reference        = Semantic const &;
	using pointer                = Semantic *;
	using const_pointer          = Semantic const *;
	using iterator               = Semantic *;  // TODO: Make label const
	using const_iterator         = Semantic const *;
	using reverse_iterator       = std::reverse_iterator<iterator>;
	using const_reverse_iterator = std::reverse_iterator<const_iterator>;

	//
	// Constructors
	//

	constexpr SemanticSet() = default;

	SemanticSet(SemanticSet const &other) { *this = other; }

	SemanticSet(SemanticSet &&other) noexcept = default;

	// SemanticSet(SemanticsReference other)
	// {
	// 	resize(other.size());
	// 	std::copy(std::begin(other), std::end(other), begin());
	// }

	template <class InputIt>
	SemanticSet(InputIt first, InputIt last)
	{
		insert(first, last);
	}

	SemanticSet(std::initializer_list<Semantic> init)
	    : SemanticSet(std::begin(init), std::end(init))
	{
	}

	//
	// Assignment operator
	//

	SemanticSet &operator=(SemanticSet const &rhs)
	{
		if (rhs.empty()) {
			clear();
		} else {
			resize(rhs.size());
			std::copy(std::begin(rhs), std::end(rhs), begin());
		}
		return *this;
	}

	SemanticSet &operator=(SemanticSet &&rhs) noexcept = default;

	// SemanticSet &operator=(SemanticsReference rhs)
	// {
	// 	resize(rhs.size());
	// 	std::copy(std::begin(rhs), std::end(rhs), begin());
	// }

	//
	// Data
	//

	[[nodiscard]] const_pointer data() const { return empty() ? nullptr : data_.get() + 1; }

	//
	// Iterators
	//

	iterator begin() noexcept { return empty() ? nullptr : data_.get() + 1; }

	const_iterator begin() const noexcept { return empty() ? nullptr : data_.get() + 1; }

	const_iterator cbegin() const noexcept { return begin(); }

	iterator end() noexcept { return empty() ? nullptr : data_.get() + allocSize(); }

	const_iterator end() const noexcept
	{
		return empty() ? nullptr : data_.get() + allocSize();
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
	// Empty
	//

	[[nodiscard]] bool empty() const noexcept { return nullptr == data_; }

	//
	// Size
	//

	[[nodiscard]] size_type size() const { return empty() ? 0 : data_[0].label; }

	[[nodiscard]] static constexpr size_type maxSize() noexcept
	{
		return std::numeric_limits<label_t>::max();
	}

	[[nodiscard]] size_type allocSize() const { return empty() ? 0 : size() + 1; }

	//
	// At
	//

	[[nodiscard]] std::optional<Semantic> at(label_t label) const
	{
		return semantic::at<1>(data_, 0, label);
	}

	//
	// Value
	//

	[[nodiscard]] std::optional<value_t> value(label_t label) const
	{
		return semantic::value<1>(data_, 0, label);
	}

	//
	// Count
	//

	[[nodiscard]] size_type count(label_t label) const
	{
		return semantic::count<1>(data_, 0, label);
	}

	//
	// Find
	//

	[[nodiscard]] iterator find(label_t label)
	{
		return semantic::find<1>(data_, 0, label);
	}

	[[nodiscard]] const_iterator find(label_t label) const
	{
		return semantic::find<1>(data_, 0, label);
	}

	//
	// Contains
	//

	[[nodiscard]] bool contains(label_t label) const
	{
		return semantic::contains<1>(data_, 0, label);
	}

	//
	// Equal range
	//

	[[nodiscard]] std::pair<iterator, iterator> equal_range(label_t label)
	{
		return semantic::equal_range<1>(data_, 0, label);
	}

	[[nodiscard]] std::pair<const_iterator, const_iterator> equal_range(label_t label) const
	{
		return semantic::equal_range<1>(data_, 0, label);
	}

	//
	// Lower bound
	//

	[[nodiscard]] iterator lower_bound(label_t label)
	{
		return semantic::lower_bound<1>(data_, 0, label);
	}

	[[nodiscard]] const_iterator lower_bound(label_t label) const
	{
		return semantic::lower_bound<1>(data_, 0, label);
	}

	//
	// Upper bound
	//

	[[nodiscard]] iterator upper_bound(label_t label)
	{
		return semantic::upper_bound<1>(data_, 0, label);
	}

	[[nodiscard]] const_iterator upper_bound(label_t label) const
	{
		return semantic::upper_bound<1>(data_, 0, label);
	}

	//
	// All
	//

	[[nodiscard]] bool all(SemanticRange range) const
	{
		return all(SemanticRangeSet(range));
	}

	[[nodiscard]] bool all(SemanticRangeSet const &ranges) const
	{
		return semantic::all<1>(data_, 0, ranges);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool all(UnaryPredicate p) const
	{
		return semantic::all<1>(data_, 0, p);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool all(SemanticRange range, UnaryPredicate p) const
	{
		return all(SemanticRangeSet(range), p);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool all(SemanticRangeSet const &ranges, UnaryPredicate p) const
	{
		return semantic::all<1>(data_, 0, ranges, p);
	}

	//
	// Any
	//

	[[nodiscard]] bool any(SemanticRange range) const
	{
		return any(SemanticRangeSet(range));
	}

	[[nodiscard]] bool any(SemanticRangeSet const &ranges) const
	{
		return semantic::any<1>(data_, 0, ranges);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool any(UnaryPredicate p) const
	{
		return semantic::any<1>(data_, 0, p);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool any(SemanticRange range, UnaryPredicate p) const
	{
		return any(SemanticRangeSet(range), p);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool any(SemanticRangeSet const &ranges, UnaryPredicate p) const
	{
		return semantic::any<1>(data_, 0, ranges, p);
	}

	//
	// None
	//

	[[nodiscard]] bool none(SemanticRange range) const
	{
		return none(SemanticRangeSet(range));
	}

	[[nodiscard]] bool none(SemanticRangeSet const &ranges) const
	{
		return semantic::none<1>(data_, 0, ranges);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool none(UnaryPredicate p) const
	{
		return semantic::none<1>(data_, 0, p);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool none(SemanticRange range, UnaryPredicate p) const
	{
		return none(SemanticRangeSet(range), p);
	}

	template <class UnaryPredicate>
	[[nodiscard]] bool none(SemanticRangeSet const &ranges, UnaryPredicate p) const
	{
		return semantic::none<1>(data_, 0, ranges, p);
	}

	//
	// TODO: Some
	//

	//
	// Clear
	//

	void clear() noexcept { data_.reset(); }

	//
	// Insert
	//

	std::pair<iterator, bool> insert(Semantic semantic)
	{
		return insert(semantic.label, semantic.value);
	}

	std::pair<iterator, bool> insert(label_t label, value_t value)
	{
		return semantic::insert<1>(data_, 0, label, value);
	}

	std::pair<iterator, bool> insert(const_iterator hint, Semantic semantic)
	{
		return insert(hint, semantic.label, semantic.value);
	}

	std::pair<iterator, bool> insert(const_iterator hint, label_t label, value_t value)
	{
		return semantic::insert<1>(data_, 0, hint, label, value);
	}

	template <class InputIt>
	void insert(InputIt first, InputIt last)
	{
		semantic::insert<1>(data_, 0, first, last);
	}

	void insert(std::initializer_list<Semantic> ilist)
	{
		insert(std::cbegin(ilist), std::cend(ilist));
	}

	//
	// Insert or assign
	//

	std::pair<iterator, bool> insertOrAssign(Semantic semantic)
	{
		return insertOrAssign(semantic.label, semantic.value);
	}

	std::pair<iterator, bool> insertOrAssign(label_t label, value_t value)
	{
		return semantic::insertOrAssign<1>(data_, 0, label, value);
	}

	std::pair<iterator, bool> insertOrAssign(const_iterator hint, Semantic semantic)
	{
		return insertOrAssign(hint, semantic.label, semantic.value);
	}

	std::pair<iterator, bool> insertOrAssign(const_iterator hint, label_t label,
	                                         value_t value)
	{
		return semantic::insertOrAssign<1>(data_, 0, hint, label, value);
	}

	template <class InputIt>
	void insertOrAssign(InputIt first, InputIt last)
	{
		semantic::insertOrAssign<1>(data_, 0, first, last);
	}

	void insertOrAssign(std::initializer_list<Semantic> ilist)
	{
		insertOrAssign(std::cbegin(ilist), std::cend(ilist));
	}

	//
	// Insert or assign custom function
	//

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	void insertOrAssign(label_t label, UnaryFunction f)
	{
		semantic::insertOrAssign<1>(data_, 0, label, f);
	}

	// InputIt to label_t
	template <class InputIt, class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	void insertOrAssign(InputIt first, InputIt last, UnaryFunction f)
	{
		semantic::insertOrAssign<1>(data_, 0, first, last, f);
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	void insertOrAssign(std::initializer_list<label_t> ilist, UnaryFunction f)
	{
		insertOrAssign(std::cbegin(ilist), std::cend(ilist), f);
	}

	//
	// Assign
	//

	void assign(SemanticRange range, value_t value)
	{
		assign(SemanticRangeSet{range}, value);
	}

	void assign(SemanticRangeSet const &ranges, value_t value)
	{
		semantic::assign<1>(data_, 0, ranges, value);
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	void assign(SemanticRange range, UnaryFunction f)
	{
		semantic::assign<1>(data_, 0, SemanticRangeSet{range}, f);
	}

	template <class UnaryPredicate>
	void assign(UnaryPredicate p, value_t value)
	{
		assign(p, [value](auto) { return value; });
	}

	template <class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	void assign(SemanticRangeSet const &ranges, UnaryFunction f)
	{
		semantic::assign<1>(data_, 0, ranges, f);
	}

	template <class UnaryPredicate, class UnaryFunction,
	          class = std::enable_if_t<std::is_invocable<UnaryFunction, Semantic>::value>>
	void assign(UnaryPredicate p, UnaryFunction f)
	{
		semantic::assign<1>(data_, 0, p, f);
	}

	//
	// Erase
	//

	iterator erase(const_iterator pos)
	{
		return semantic::erase<1>(data_, pos, std::next(pos));
	}

	iterator erase(iterator pos) { return semantic::erase<1>(data_, pos, std::next(pos)); }

	iterator erase(const_iterator first, const_iterator last)
	{
		return semantic::erase<1>(data_, first, last);
	}

	size_type erase(label_t label) { return semantic::erase<1>(data_, 0, label); }

	size_type erase(SemanticRangeSet const &ranges)
	{
		return semantic::erase<1>(data_, 0, ranges);
	}

	size_type erase(SemanticRange range) { return erase(SemanticRangeSet{range}); }

	//
	// Erase if
	//

	template <class UnaryPredicate>
	size_type eraseIf(UnaryPredicate p)
	{
		return semantic::eraseIf<1>(data_, 0, p);
	}

	template <class UnaryPredicate>
	size_type eraseIf(SemanticRangeSet const &ranges, UnaryPredicate p)
	{
		return semantic::eraseIf<1>(data_, 0, ranges, p);
	}

	template <class UnaryPredicate>
	size_type eraseIf(SemanticRange range, UnaryPredicate p)
	{
		return eraseIf(SemanticRangeSet{range}, p);
	}

	//
	// Swap
	//

	void swap(SemanticSet &other) noexcept { std::swap(data_, other.data_); }

	std::string toString() const { return semantic::toString<1>(data_); }

 protected:
	//
	// Data
	//

	[[nodiscard]] pointer data() { return empty() ? nullptr : data_.get() + 1; }

	//
	// Resize
	//

	void resize(size_type size)
	{
		if (0 == size) {
			clear();
			return;
		} else if (this->size() == size) {
			return;
		}

		pointer p_cur = data_.release();
		pointer p_new = static_cast<pointer>(realloc(p_cur, (size + 1) * sizeof(Semantic)));

		if (!p_new) {
			data_.reset(p_cur);
			throw std::bad_alloc();
		}

		data_.reset(p_new);
		data_[0].label = static_cast<label_t>(size);  // TODO: is this cast ok?
	}

 private:
	std::unique_ptr<Semantic[]> data_;
};
}  // namespace ufo

namespace std
{
template <std::size_t N>
inline void swap(ufo::SemanticSet<N> &lhs,
                 ufo::SemanticSet<N> &rhs) noexcept(noexcept(lhs.swap(rhs)))
{
	lhs.swap(rhs);
}

template <std::size_t N>
inline std::ostream &operator<<(std::ostream &out, ufo::SemanticSet<N> const &s)
{
	return out << s.toString();
}
}  // namespace std

#endif  // UFO_MAP_SEMANTICS_H