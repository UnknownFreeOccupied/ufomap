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

#ifndef UFO_MAP_OCTREE_BLOCK_H
#define UFO_MAP_OCTREE_BLOCK_H

// STL
#include <cmath>
#include <memory>
#include <stack>
#include <vector>

namespace ufo::map
{
template <class T, std::size_t N>
using Block = std::array<T, N>;

// template <class Node, std::size_t NumNodes>
// class Block
// {
//  public:
// 	//
// 	// Tags
// 	//

// 	using value_type = typename std::array<Node, NumNodes>::value_type;
// 	using size_type = typename std::array<Node, NumNodes>::size_type;
// 	using difference_type = typename std::array<Node, NumNodes>::difference_type;
// 	using reference = typename std::array<Node, NumNodes>::reference;
// 	using const_reference = typename std::array<Node, NumNodes>::const_reference;
// 	using pointer = typename std::array<Node, NumNodes>::pointer;
// 	using const_pointer = typename std::array<Node, NumNodes>::const_pointer;
// 	using iterator = typename std::array<Node, NumNodes>::iterator;
// 	using const_iterator = typename std::array<Node, NumNodes>::const_iterator;
// 	using reverse_iterator = typename std::array<Node, NumNodes>::reverse_iterator;
// 	using const_reverse_iterator =
// 	    typename std::array<Node, NumNodes>::const_reverse_iterator;

// 	//
// 	// Node access
// 	//

// 	constexpr reference at(size_type pos) { return nodes_.at(pos); }

// 	constexpr const_reference at(size_type pos) const { return nodes_.at(pos); }

// 	constexpr reference operator[](size_type pos) { return nodes_[pos]; }

// 	constexpr const_reference operator[](size_type pos) const { return nodes_[pos]; }

// 	constexpr reference front() { return nodes_.front(); }

// 	constexpr const_reference front() const { return nodes_.front(); }

// 	constexpr reference back() { return nodes_.back(); }

// 	constexpr const_reference back() const { return nodes_.back(); }

// 	constexpr Node* data() noexcept { return nodes_.data(); }

// 	constexpr const Node* data() const noexcept { return nodes_.data(); }

// 	//
// 	// Iterators
// 	//

// 	constexpr iterator begin() noexcept { return nodes_.begin(); }

// 	constexpr const_iterator begin() const noexcept { return nodes_.begin(); }

// 	constexpr const_iterator cbegin() const noexcept { return nodes_.cbegin(); }

// 	constexpr iterator end() noexcept { return nodes_.end(); }

// 	constexpr const_iterator end() const noexcept { return nodes_.end(); }

// 	constexpr const_iterator cend() const noexcept { return nodes_.cend(); }

// 	constexpr reverse_iterator rbegin() noexcept { return nodes_.rbegin(); }

// 	constexpr const_reverse_iterator rbegin() const noexcept { return nodes_.rbegin(); }

// 	constexpr const_reverse_iterator crbegin() const noexcept { return nodes_.crbegin(); }

// 	constexpr reverse_iterator rend() noexcept { return nodes_.rend(); }

// 	constexpr const_reverse_iterator rend() const noexcept { return nodes_.rend(); }

// 	constexpr const_reverse_iterator crend() const noexcept { return nodes_.crend(); }

// 	//
// 	// Capacity
// 	//

// 	[[nodiscard]] constexpr bool empty() const noexcept { return nodes_.empty(); }

// 	[[nodiscard]] constexpr size_type size() const noexcept { return nodes_.size(); }

// 	[[nodiscard]] constexpr size_type max_size() const noexcept
// 	{
// 		return nodes_.max_size();
// 	}

// 	//
// 	// Operations
// 	//

// 	void fill(Node const& value) { nodes_.fill(value); }

// 	void swap(Block& other) noexcept(noexcept(std::swap(std::declval<Node&>(),
// 	                                                    std::declval<Node&>())))
// 	{
// 		nodes_.swap(other);
// 	}

//  private:
// 	std::array<Node, NumNodes> nodes_;
// };

template <class T, std::size_t N>
class Blocks
{
 public:
	//
	//  Tags
	//

	using value_type = T;
	using size_type = std::size_t;
	using difference_type = std::ptrdiff_t;
	using reference = T&;
	using const_reference = T const&;
	using pointer = T*;
	using const_pointer = T const*;
	// 	using iterator = ...;
	// 	using const_iterator = ...;
	// 	using reverse_iterator = ...;
	// 	using const_reverse_iterator = ...;

	//
	// Constructor
	//

	Blocks() {}

	explicit Blocks(size_type count) { blocks_.reserve(count); }

	Blocks(Blocks const& other)
	    : blocks_(other.blocks_),
	      free_indices_(other.free_indices_),
	      new_index_(other.new_index_)
	{
	}

	Blocks(Blocks&& other)
	    : blocks_(std::move(other.blocks_)),
	      free_indices_(std::move(other.free_indices_)),
	      new_index_(std::move(other.new_index_))
	{
	}

	//
	// Destructor
	//

	~Blocks() {}

	//
	// operator=
	//

	Blocks& operator=(Blocks const& other)
	{
		blocks_ = other.blocks_;
		free_indices_ = other.free_indices_;
		new_index_ = other.new_index_;
		return *this;
	}

	Blocks& operator=(Blocks&& other)
	{
		blocks_ = std::move(other.blocks_);
		free_indices_ = std::move(other.free_indices_);
		new_index_ = std::move(other.new_index_);
		return *this;
	}

	//
	// Node access
	//

	reference at(size_type pos)
	{
		auto block_index = pos / N;
		auto node_index = pos % N;
		return blocks_.at(block_index)->at(node_index);
	}

	const_reference at(size_type pos) const
	{
		auto block_index = pos / N;
		auto node_index = pos % N;
		return blocks_.at(block_index)->at(node_index);
	}

	reference operator[](size_type pos)
	{
		auto block_index = pos / N;
		auto node_index = pos % N;
		return (*blocks_[block_index])[node_index];
	}

	const_reference operator[](size_type pos) const
	{
		auto block_index = pos / N;
		auto node_index = pos % N;
		return (*blocks_[block_index])[node_index];
	}

	//
	// Iterators
	//

	// 	iterator begin() noexcept
	// 	{
	// 		// TODO: Implement
	// 	}

	// 	const_iterator begin() const noexcept
	// 	{
	// 		// TODO: Implement
	// 	}

	// 	const_iterator cbegin() const noexcept
	// 	{
	// 		// TODO: Implement
	// 	}

	// 	iterator end() noexcept
	// 	{
	// 		// TODO: Implement
	// 	}

	// 	const_iterator end() const noexcept
	// 	{
	// 		// TODO: Implement
	// 	}

	// 	const_iterator cend() const noexcept
	// 	{
	// 		// TODO: Implement
	// 	}

	// 	reverse_iterator rbegin() noexcept
	// 	{
	// 		// TODO: Implement
	// 	}

	// 	const_reverse_iterator rbegin() const noexcept
	// 	{
	// 		// TODO: Implement
	// 	}

	// 	const_reverse_iterator crbegin() const noexcept
	// 	{
	// 		// TODO: Implement
	// 	}

	// 	reverse_iterator rend() noexcept
	// 	{
	// 		// TODO: Implement
	// 	}

	// 	const_reverse_iterator rend() const noexcept
	// 	{
	// 		// TODO: Implement
	// 	}

	// 	const_reverse_iterator crend() const noexcept
	// 	{
	// 		// TODO: Implement
	// 	}

	//
	// Capacity
	//

	[[nodiscard]] bool empty() const noexcept
	{
		// TODO: Implement
		return false;
	}

	[[nodiscard]] size_type size() const noexcept { return blocks_.size() * N; }

	[[nodiscard]] size_type max_size() const noexcept { return blocks_.max_size() * N; }

	void reserve(size_type new_cap)
	{
		blocks_.reserve(std::ceil(new_cap / static_cast<double>(N)));
	}

	[[nodiscard]] size_type capacity() const noexcept { return blocks_.capacity() * N; }

	void shrink_to_fit() { blocks_.shrink_to_fit(); }

	//
	// Modifiers
	//

	void clear() noexcept
	{
		blocks_.clear();
		while (!free_indices_.empty()) {
			free_indices_.pop();
		}
		new_index_ = 0;
	}

	std::pair<size_type, reference> create()
	{
		auto index = getFreeIndex();
		return {index, operator[](index)};
	}

	void erase(size_type pos) { free_indices_.push(pos); }

	void swap(Blocks& other)
	{
		blocks_.swap(other.blocks_);
		free_indices_.swap(other.free_indices_);
		std::swap(new_index_, other.new_index_);
	}

 private:
	size_type getFreeIndex()
	{
		if (free_indices_.empty()) {
			if (blocks_.size() * N == new_index_) {
				blocks_.push_back(std::make_unique<Block<T, N>>());
			}
			return new_index_++;
		} else {
			auto index = free_indices_.top();
			free_indices_.pop();
			return index;
		}
	}

 private:
	// Allocated blocks
	std::vector<std::unique_ptr<Block<T, N>>> blocks_;

	// Indices without nodes
	std::stack<size_type, std::vector<size_type>> free_indices_;

	// New index
	size_type new_index_ = 0;
};
}  // namespace ufo::map

#endif  // UFO_MAP_OCTREE_BLOCK_H