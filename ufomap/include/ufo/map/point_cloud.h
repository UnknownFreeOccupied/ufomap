/**
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * License: BSD 3
 *
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
#include <ufo/map/types.h>
#include <ufo/math/pose6.h>

// STD
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
template <typename T, typename = std::enable_if_t<std::is_base_of_v<Point3, T>>>
class PointCloudT
{
 public:
	PointCloudT() {}

	PointCloudT(const PointCloudT& other)
	{
		cloud_.reserve(other.size());
		push_back(other);
	}

	~PointCloudT() {}

	/**
	 * @brief Access specified point
	 *
	 * @param index Index of the point to return
	 * @return const T& Reference to the requested point
	 */
	const T& operator[](size_t index) const { return cloud_[index]; }

	/**
	 * @brief Access specified point
	 *
	 * @param index Index of the point to return
	 * @return const T& Reference to the requested point
	 */
	T& operator[](size_t index) { return cloud_[index]; }

	/**
	 * @brief Erases all points from the point cloud
	 *
	 */
	void clear() { cloud_.clear(); }

	/**
	 * @brief Increase the capacity of the point cloud to a value that is greater
	 * or equal to new_cap
	 *
	 * @param new_cap New capacity of the point cloud
	 */
	void reserve(size_t new_cap) { cloud_.reserve(new_cap); }

	/**
	 * @brief Resizes the container to contain count elements.
	 * If the current size is greater than count, the container is reduced to its first
	 * count elements. If the current size is less than count, 1) additional
	 * default-inserted elements are appended 2) additional copies of value are appended.
	 *
	 * @param count New size of the point cloud
	 */
	constexpr void resize(size_t count) { cloud_.resize(count); }

	/**
	 * @brief Resizes the container to contain count elements.
	 * If the current size is greater than count, the container is reduced to its first
	 * count elements. If the current size is less than count, 1) additional
	 * default-inserted elements are appended 2) additional copies of value are appended.
	 *
	 * @param count New size of the point cloud
	 * @param value The value to initialize the new elements with
	 */
	constexpr void resize(size_t count, T const& value) { cloud_.resize(count, value); }

	/**
	 * @brief Return the number of points in the point cloud
	 *
	 * @return size_t The number of points in the point cloud
	 */
	size_t size() const { return cloud_.size(); }

	/**
	 * @brief Adds a point to the point cloud
	 *
	 * @param point The point to add to the point cloud
	 */
	void push_back(const T& point) { cloud_.push_back(point); }

	/**
	 * @brief Adds all points from another point cloud to this point cloud
	 *
	 * @param other The other point cloud that points should be added from
	 */
	void push_back(const PointCloudT& other)
	{
		cloud_.insert(cloud_.end(), other.begin(), other.end());
	}

	/**
	 * @brief Transform each point in the point cloud
	 *
	 * @param transform The transformation to be applied to each point
	 */
	void transform(const ufo::math::Pose6& transform, bool parallel = false)
	{
		if (parallel) {
			std::for_each(std::execution::par, cloud_.begin(), cloud_.end(),
			              [&transform](auto&& point) { point = transform.transform(point); });
		} else {
			std::for_each(std::execution::seq, cloud_.begin(), cloud_.end(),
			              [&transform](auto&& point) { point = transform.transform(point); });
		}
	}

	/**
	 * @brief Returns an iterator to the first point of the point cloud
	 *
	 * @return std::vector<T>::iterator Iterator to the first point
	 */
	typename std::vector<T>::iterator begin() { return cloud_.begin(); }

	/**
	 * @brief Returns an iterator to the last element following the last point of
	 * the point cloud
	 *
	 * @return std::vector<T>::iterator Iterator to the element following the last
	 * point
	 */
	typename std::vector<T>::iterator end() { return cloud_.end(); }

	/**
	 * @brief Returns an iterator to the first point of the point cloud
	 *
	 * @return std::vector<T>::const_iterator Iterator to the first point
	 */
	typename std::vector<T>::const_iterator begin() const { return cloud_.begin(); }

	/**
	 * @brief Returns an iterator to the last element following the last point of
	 * the point cloud
	 *
	 * @return std::vector<T>::const_iterator Iterator to the element following
	 * the last point
	 */
	typename std::vector<T>::const_iterator end() const { return cloud_.end(); }

	/**
	 * @brief Returns an iterator to the first point of the point cloud
	 *
	 * @return std::vector<T>::const_iterator Iterator to the first point
	 */
	typename std::vector<T>::const_iterator cbegin() const { return cloud_.cbegin(); }

	/**
	 * @brief Returns an iterator to the last element following the last point of
	 * the point cloud
	 *
	 * @return std::vector<T>::const_iterator Iterator to the element following
	 * the last point
	 */
	typename std::vector<T>::const_iterator cend() const { return cloud_.cend(); }

	/**
	 * @brief Returns a reverse iterator to the first point of the point cloud
	 *
	 * @return std::vector<T>::reverse_iterator Reverse iterator to the first
	 * point
	 */
	typename std::vector<T>::reverse_iterator rbegin() { return cloud_.rbegin(); }

	/**
	 * @brief Returns a reverse iterator to the element following the last point
	 * of the reversed point cloud
	 *
	 * @return std::vector<T>::reverse_iterator Reverse iterator to the element
	 * following the last point
	 */
	typename std::vector<T>::reverse_iterator rend() { return cloud_.rend(); }

	/**
	 * @brief Returns a reverse iterator to the first point of the point cloud
	 *
	 * @return std::vector<T>::const_reverse_iterator Reverse iterator to the
	 * first point
	 */
	typename std::vector<T>::const_reverse_iterator rbegin() const
	{
		return cloud_.rbegin();
	}

	/**
	 * @brief Returns a reverse iterator to the element following the last point
	 * of the reversed point cloud
	 *
	 * @return std::vector<T>::const_reverse_iterator Reverse iterator to the
	 * element following the last point
	 */
	typename std::vector<T>::const_reverse_iterator rend() const { return cloud_.rend(); }

	/**
	 * @brief Returns a reverse iterator to the first point of the point cloud
	 *
	 * @return std::vector<T>::const_reverse_iterator Reverse iterator to the
	 * first point
	 */
	typename std::vector<T>::const_reverse_iterator crbegin() const
	{
		return cloud_.rbegin();
	}

	/**
	 * @brief Returns a reverse iterator to the element following the last point
	 * of the reversed point cloud
	 *
	 * @return std::vector<T>::const_reverse_iterator Reverse iterator to the
	 * element following the last point
	 */
	typename std::vector<T>::const_reverse_iterator crend() const { return cloud_.rend(); }

 private:
	std::vector<T> cloud_;  // The point cloud
};

using PointCloud = PointCloudT<Point3>;
using PointCloudColor = PointCloudT<Point3Color>;

}  // namespace ufo::map

#endif  // UFO_MAP_POINT_CLOUD_H