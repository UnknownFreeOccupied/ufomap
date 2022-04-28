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

#ifndef UFO_MAP_POINT_CLOUD_H
#define UFO_MAP_POINT_CLOUD_H

// UFO
#include <ufo/map/point.h>
#include <ufo/math/pose6.h>

// STL
#include <algorithm>
#include <execution>
#include <type_traits>
#include <vector>

namespace ufo::map
{
/**
 * @brief A collection of 3D coordinates of type T
 *
 * @tparam T The type of points to store in the point cloud. Has to inhert from
 * Point3
 */
template <typename T, class Allocator = std::allocator<T>,
          typename = std::enable_if_t<std::is_base_of_v<Point3, T>>>
class PointCloudT
{
 public:
	//  Tags
	using point_type = T;
	using value_type = typename std::vector<T, Allocator>::value_type;
	using allocator_type = typename std::vector<T, Allocator>::allocator_type;
	using size_type = typename std::vector<T, Allocator>::size_type;
	using difference_type = typename std::vector<T, Allocator>::difference_type;
	using reference = typename std::vector<T, Allocator>::reference;
	using const_reference = typename std::vector<T, Allocator>::const_reference;
	using pointer = typename std::vector<T, Allocator>::pointer;
	using const_pointer = typename std::vector<T, Allocator>::const_pointer;
	using iterator = typename std::vector<T, Allocator>::iterator;
	using const_iterator = typename std::vector<T, Allocator>::const_iterator;
	using reverse_iterator = typename std::vector<T, Allocator>::reverse_iterator;
	using const_reverse_iterator =
	    typename std::vector<T, Allocator>::const_reverse_iterator;

 public:
	PointCloudT() noexcept(noexcept(Allocator())) {}

	explicit PointCloudT(Allocator const& alloc) noexcept : cloud_(alloc) {}

	PointCloudT(size_type count, T const& value, Allocator const& alloc = Allocator())
	    : cloud_(count, value, alloc)
	{
	}

	explicit PointCloudT(size_type count, Allocator const& alloc = Allocator())
	    : cloud_(count, alloc)
	{
	}

	template <class InputIt>
	PointCloudT(InputIt first, InputIt last, Allocator const& alloc = Allocator())
	    : cloud_(first, last, alloc)
	{
	}

	PointCloudT(PointCloudT const& other) : cloud_(other.cloud_) {}

	PointCloudT(PointCloudT const& other, Allocator const& alloc)
	    : cloud_(other.cloud_, alloc)
	{
	}

	PointCloudT(PointCloudT&& other) noexcept : cloud_(std::move(other.cloud_)) {}

	PointCloudT(PointCloudT&& other, Allocator const& alloc)
	    : cloud_(std::move(other.cloud_), alloc)
	{
	}

	PointCloudT(std::initializer_list<T> init, Allocator const& alloc = Allocator())
	    : cloud_(init, alloc)
	{
	}

	// TODO: Add ~PointCloudT?

	// TODO: Add operator=?

	void assign(size_type count, T const& value) { cloud_.assign(count, value); }

	template <class InputIt>
	void assign(InputIt first, InputIt last)
	{
		cloud_.assign(first, last);
	}

	void assign(std::initializer_list<T> ilist) { cloud_.assign(ilist); }

	allocator_type get_allocator() const noexcept { return cloud_.get_allocator(); }

	/**
	 * @brief Returns an iterator to the first point of the point cloud
	 *
	 * @return std::vector<T>::iterator Iterator to the first point
	 */
	iterator begin() { return cloud_.begin(); }

	/**
	 * @brief Returns an iterator to the last element following the last point of
	 * the point cloud
	 *
	 * @return std::vector<T>::iterator Iterator to the element following the last
	 * point
	 */
	iterator end() { return cloud_.end(); }

	/**
	 * @brief Returns an iterator to the first point of the point cloud
	 *
	 * @return std::vector<T>::const_iterator Iterator to the first point
	 */
	const_iterator begin() const { return cloud_.begin(); }

	/**
	 * @brief Returns an iterator to the last element following the last point of
	 * the point cloud
	 *
	 * @return std::vector<T>::const_iterator Iterator to the element following
	 * the last point
	 */
	const_iterator end() const { return cloud_.end(); }

	/**
	 * @brief Returns an iterator to the first point of the point cloud
	 *
	 * @return std::vector<T>::const_iterator Iterator to the first point
	 */
	const_iterator cbegin() const { return cloud_.cbegin(); }

	/**
	 * @brief Returns an iterator to the last element following the last point of
	 * the point cloud
	 *
	 * @return std::vector<T>::const_iterator Iterator to the element following
	 * the last point
	 */
	const_iterator cend() const { return cloud_.cend(); }

	/**
	 * @brief Returns a reverse iterator to the first point of the point cloud
	 *
	 * @return std::vector<T>::reverse_iterator Reverse iterator to the first
	 * point
	 */
	reverse_iterator rbegin() { return cloud_.rbegin(); }

	/**
	 * @brief Returns a reverse iterator to the element following the last point
	 * of the reversed point cloud
	 *
	 * @return std::vector<T>::reverse_iterator Reverse iterator to the element
	 * following the last point
	 */
	reverse_iterator rend() { return cloud_.rend(); }

	/**
	 * @brief Returns a reverse iterator to the first point of the point cloud
	 *
	 * @return std::vector<T>::const_reverse_iterator Reverse iterator to the
	 * first point
	 */
	const_reverse_iterator rbegin() const { return cloud_.rbegin(); }

	/**
	 * @brief Returns a reverse iterator to the element following the last point
	 * of the reversed point cloud
	 *
	 * @return std::vector<T>::const_reverse_iterator Reverse iterator to the
	 * element following the last point
	 */
	const_reverse_iterator rend() const { return cloud_.rend(); }

	/**
	 * @brief Returns a reverse iterator to the first point of the point cloud
	 *
	 * @return std::vector<T>::const_reverse_iterator Reverse iterator to the
	 * first point
	 */
	const_reverse_iterator crbegin() const { return cloud_.rbegin(); }

	/**
	 * @brief Returns a reverse iterator to the element following the last point
	 * of the reversed point cloud
	 *
	 * @return std::vector<T>::const_reverse_iterator Reverse iterator to the
	 * element following the last point
	 */
	const_reverse_iterator crend() const { return cloud_.rend(); }

	reference at(size_type pos) { return cloud_.at(pos); }

	const_reference at(size_type pos) const { return cloud_.at(pos); }

	/**
	 * @brief Access specified point
	 *
	 * @param index Index of the point to return
	 * @return const T& Reference to the requested point
	 */
	reference operator[](size_type index) { return cloud_[index]; }

	/**
	 * @brief Access specified point
	 *
	 * @param index Index of the point to return
	 * @return const T& Reference to the requested point
	 */
	const_reference operator[](size_type index) const { return cloud_[index]; }

	reference front() { return cloud_.front(); }

	const_reference front() const { return cloud_.front(); }

	reference back() { return cloud_.back(); }

	const_reference back() const { return cloud_.back(); }

	T* data() noexcept { return cloud_.data(); }

	T const* data() const noexcept { return cloud_.data(); }

	[[nodiscard]] constexpr bool empty() const noexcept { return cloud_.empty(); }

	/**
	 * @brief Return the number of points in the point cloud
	 *
	 * @return size_t The number of points in the point cloud
	 */
	size_type size() const noexcept { return cloud_.size(); }

	size_type max_size() const noexcept { return cloud_.max_size(); }

	/**
	 * @brief Increase the capacity of the point cloud to a value that is greater
	 * or equal to new_cap
	 *
	 * @param new_cap New capacity of the point cloud
	 */
	void reserve(size_type new_cap) { cloud_.reserve(new_cap); }

	size_type capacity() const noexcept { return cloud_.capacity(); }

	void shrink_to_fit() { cloud_.shrink_to_fit(); }

	/**
	 * @brief Erases all points from the point cloud
	 *
	 */
	void clear() noexcept { cloud_.clear(); }

	iterator insert(const_iterator pos, T const& value)
	{
		return cloud_.insert(pos, value);
	}

	iterator insert(const_iterator pos, T&& value)
	{
		return cloud_.insert(pos, std::move(value));
	}

	iterator insert(const_iterator pos, size_type count, T const& value)
	{
		return cloud_.insert(pos, count, value);
	}

	template <class InputIt>
	iterator insert(const_iterator pos, InputIt first, InputIt last)
	{
		return cloud_.insert(pos, first, last);
	}

	iterator insert(const_iterator pos, std::initializer_list<T> ilist)
	{
		return cloud_.insert(pos, ilist);
	}

	/**
	 * @brief Construct a point to the point cloud
	 *
	 * @param args Arguments to construct the point
	 */
	template <class... Args>
	reference emplace_back(Args&&... args)
	{
		return cloud_.emplace_back(std::forward<Args>(args)...);
	}

	iterator erase(const_iterator pos) { return cloud_.erase(pos); }

	iterator erase(const_iterator first, const_iterator last)
	{
		return cloud_.erase(first, last);
	}

	/**
	 * @brief Adds a point to the point cloud
	 *
	 * @param point The point to add to the point cloud
	 */
	void push_back(T const& point) { cloud_.push_back(point); }

	void push_back(T&& value) { cloud_.push_back(std::move(value)); }

	void pop_back() { cloud_.pop_back(); }

	/**
	 * @brief Resizes the container to contain count elements.
	 * If the current size is greater than count, the container is reduced to its first
	 * count elements. If the current size is less than count, 1) additional
	 * default-inserted elements are appended 2) additional copies of value are appended.
	 *
	 * @param count New size of the point cloud
	 */
	void resize(size_type count) { cloud_.resize(count); }

	/**
	 * @brief Resizes the container to contain count elements.
	 * If the current size is greater than count, the container is reduced to its first
	 * count elements. If the current size is less than count, 1) additional
	 * default-inserted elements are appended 2) additional copies of value are appended.
	 *
	 * @param count New size of the point cloud
	 * @param value The value to initialize the new elements with
	 */
	void resize(size_type count, value_type const& value) { cloud_.resize(count, value); }

	void swap(PointCloudT& other) noexcept(
	    std::allocator_traits<Allocator>::propagate_on_container_swap::value ||
	    std::allocator_traits<Allocator>::is_always_equal::value)
	{
		cloud_.swap(other);
	}

	/**
	 * @brief Transform each point in the point cloud
	 *
	 * @param transform The transformation to be applied to each point
	 */
	void transform(math::Pose6 const& transform)
	{
		this->transform(std::execution::seq, transform);
	}

	template <class ExecutionPolicy, typename = std::enable_if_t<std::is_execution_policy_v<
	                                     std::decay_t<ExecutionPolicy>>>>
	void transform(ExecutionPolicy policy, math::Pose6 const& transform)
	{
		std::for_each(
		    policy, begin(), end(),
		    [t = transform.translation(), r = transform.rotation().getRotMatrix()](auto& p) {
			    auto const a = p;
			    p.x() = r[0] * a.x() + r[1] * a.y() + r[2] * a.z() + t.x();
			    p.y() = r[3] * a.x() + r[4] * a.y() + r[5] * a.z() + t.y();
			    p.z() = r[6] * a.x() + r[7] * a.y() + r[8] * a.z() + t.z();
		    });
	}

	void removeNaN()
	{
		auto it = std::remove_if(begin(), end(), [](auto const& point) {
			return std::isnan(point.x()) || std::isnan(point.y()) || std::isnan(point.z());
		});
		erase(it, end());
	}

 private:
	std::vector<T, Allocator> cloud_;  // The point cloud
};

template <class T, class Alloc>
bool operator==(PointCloudT<T, Alloc> const& lhs, PointCloudT<T, Alloc> const& rhs)
{
	return std::equal(std::begin(lhs), std::end(lhs), std::begin(rhs));
}

template <class T, class Alloc>
bool operator!=(PointCloudT<T, Alloc> const& lhs, PointCloudT<T, Alloc> const& rhs)
{
	return !std::equal(std::begin(lhs), std::end(lhs), std::begin(rhs));
}

template <class T, class Alloc>
bool operator<(PointCloudT<T, Alloc> const& lhs, PointCloudT<T, Alloc> const& rhs)
{
	return std::lexicographical_compare(std::begin(lhs), std::end(lhs), std::begin(rhs),
	                                    std::end(rhs));
}

template <class T, class Alloc>
bool operator<=(PointCloudT<T, Alloc> const& lhs, PointCloudT<T, Alloc> const& rhs)
{
	return std::lexicographical_compare(std::begin(lhs), std::end(lhs), std::begin(rhs),
	                                    std::end(rhs), std::less_equal<>());
}

template <class T, class Alloc>
bool operator>(PointCloudT<T, Alloc> const& lhs, PointCloudT<T, Alloc> const& rhs)
{
	return std::lexicographical_compare(std::begin(lhs), std::end(lhs), std::begin(rhs),
	                                    std::end(rhs), std::greater<>());
}

template <class T, class Alloc>
bool operator>=(PointCloudT<T, Alloc> const& lhs, PointCloudT<T, Alloc> const& rhs)
{
	return std::lexicographical_compare(std::begin(lhs), std::end(lhs), std::begin(rhs),
	                                    std::end(rhs), std::greater_equal<>());
}

template <class T, class Alloc>
void swap(PointCloudT<T, Alloc>& lhs,
          PointCloudT<T, Alloc>& rhs) noexcept(noexcept(lhs.swap(rhs)))
{
	lhs.swap(rhs);
}

using PointCloud = PointCloudT<Point3>;
using PointCloudColor = PointCloudT<Point3Color>;
using PointCloudLabel = PointCloudT<Point3Label>;
using PointCloudSemantic = PointCloudT<Point3Semantic>;
using PointCloudColorLabel = PointCloudT<Point3ColorLabel>;
using PointCloudColorSemantic = PointCloudT<Point3ColorSemantic>;

}  // namespace ufo::map

#endif  // UFO_MAP_POINT_CLOUD_H