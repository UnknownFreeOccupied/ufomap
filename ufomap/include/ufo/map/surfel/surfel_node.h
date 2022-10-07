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

#ifndef UFO_MAP_SURFEL_NODE_H
#define UFO_MAP_SURFEL_NODE_H

// UFO
#include <ufo/map/surfel/surfel.h>
#include <ufo/map/types.h>

// STL
#include <array>
#include <cstdint>
#include <memory>
#include <type_traits>

namespace ufo::map
{
template <std::size_t N = 8>
struct SurfelNode {
	//
	// Size
	//

	[[nodiscard]] static constexpr std::size_t surfelSize() { return N; }

	//
	// Fill
	//

	void fill(SurfelNode const& parent, index_t const index)
	{
		setSum(parent.sum(index));
		setSumSquares(parent.sumSquares(index));
		setNumPoints(parent.numPoints(index));
	}

	//
	// Is collapsible
	//

	[[nodiscard]] bool isCollapsible(SurfelNode const& parent, index_t const index) const
	{
		return all_of(sum_, [s = parent.sum(index)](auto const e) { return e == s; }) &&
		       all_of(sum_squares_,
		              [s = parent.sumSquares(index)](auto const e) { return e == s; }) &&
		       all_of(num_points_,
		              [s = parent.numPoints(index)](auto const e) { return e == s; });
	}

	//
	// Get surfel
	//

	[[nodiscard]] constexpr Surfel surfel(index_t const index) const
	{
		return Surfel(sum_, sum_squares_, num_points_);
	}

	//
	// Set surfel
	//

	void setSurfel(Surfel const& surfel)
	{
		setSum(surfel.sum());
		setSumSquares(surfel.sumSquares());
		setNumPoints(surfel.numPoints());
	}

	void setSurfel(index_t const index, Surfel const& surfel)
	{
		if constexpr (1 == N) {
			setSurfel(surfel);
		} else {
			setSum(index, surfel.sum());
			setSumSquares(index, surfel.sumSquares());
			setNumPoints(index, surfel.numPoints());
		}
	}

	//
	// Get sum
	//

	[[nodiscard]] constexpr math::Vector3<surfel_scalar_t> sum(index_t const index) const
	{
		if constexpr (1 == N) {
			return sum_[0];
		} else {
			return sum_[index];
		}
	}

	//
	// Set sum
	//

	void setSum(math::Vector3<surfel_scalar_t> value) { sum_.fill(value); }

	void setSum(index_t const index, math::Vector3<surfel_scalar_t> value)
	{
		if constexpr (1 == N) {
			setSum(value);
		} else {
			sum_[index] = value;
		}
	}

	//
	// Get sum squares
	//

	[[nodiscard]] constexpr std::array<surfel_scalar_t, 6> sumSquares(
	    index_t const index) const
	{
		if constexpr (1 == N) {
			return sum_squares_[0];
		} else {
			return sum_squares_[index];
		}
	}

	//
	// Set sum squares
	//

	void setSumSquares(std::array<surfel_scalar_t, 6> value) { sum_squares_.fill(value); }

	void setSumSquares(index_t const index, std::array<surfel_scalar_t, 6> value)
	{
		if constexpr (1 == N) {
			setSumSquares(value);
		} else {
			sum_squares_[index] = value;
		}
	}

	//
	// Get num points
	//

	[[nodiscard]] constexpr std::uint32_t numPoints(index_t const index) const
	{
		if constexpr (1 == N) {
			return num_points_[0];
		} else {
			return num_points_[index];
		}
	}

	//
	// Set num points
	//

	void setNumPoints(std::uint32_t value) { num_points_.fill(value); }

	void setNumPoints(index_t const index, std::uint32_t value)
	{
		if constexpr (1 == N) {
			setNumPoints(value);
		} else {
			num_points_[index] = value;
		}
	}

	//
	// Clear
	//

	void clear()
	{
		setSum(math::Vector3<surfel_scslar_t>());
		setSumSquares({0, 0, 0, 0, 0, 0});
		setNumPoints(0);
	}

	void clear(index_t const index)
	{
		if constexpr (1 == N) {
			clear();
		} else {
			setSum(index, math::Vector3<surfel_scslar_t>());
			setSumSquares(index, {0, 0, 0, 0, 0, 0});
			setNumPoints(index, 0);
		}
	}

	//
	// Get mean
	//

	[[nodiscard]] constexpr math::Vector3<surfel_scalar_t> mean(index_t const index) const
	{
		return sum(index) / numPoints(index);
	}

	//
	// Get normal
	//

	[[nodiscard]] constexpr math::Vector3<double> normal(index_t const index) const
	{
		return eigenVectors(symmetricCovariance(index))[0].normalize();
	}

	//
	// Get planarity
	//

	[[nodiscard]] constexpr double planarity(index_t const index) const
	{
		auto const e = eigenValues(index);
		return 2 * (e[1] - e[0]) / (e[0] + e[1] + e[2]);
	}

	//
	// Get covariance
	//

	[[nodiscard]] constexpr std::array<std::array<double, 3>, 3> covariance(
	    index_t const index) const
	{
		using as = std::array<surfel_scalar_t, 3>;
		using cov = std::array<as, 3>;

		double const n = numPoints(index) - 1;
		return cov{as{sum_squares_[index][0] / n, sum_squares_[index][1] / n,
		              sum_squares_[index][2] / n},
		           as{sum_squares_[index][1] / n, sum_squares_[index][3] / n,
		              sum_squares_[index][4] / n},
		           as{sum_squares_[index][2] / n, sum_squares_[index][4] / n,
		              sum_squares_[index][5] / n}};
	}

	//
	// Get eigenvalues
	//

	[[nodiscard]] constexpr math::Vector3<double> eigenValues(index_t const index) const
	{
		return eigenValues(symmetricCovariance(index));
	}

	//
	// Get eigen vectors
	//

	[[nodiscard]] constexpr std::array<math::Vector3<double>, 3> eigenVectors(
	    index_t const index) const
	{
		auto eigen_vectors = eigenVectors(symmetricCovariance(index));
		for_each(eigen_vectors, [](auto& v) { v.normalize(); });  // FIXME: Needed?
		return eigen_vectors;
	}

 private:
	std::array<math::Vector3<surfel_scalar_t>, N> sum_;
	std::array<std::array<surfel_scalar_t, 6>, N> sum_squares_;
	std::array<std::uint32_t, N> num_points_;
	// std::array<math::Vector3<surfel_scalar_t>, N> eigen_values_;
};
}  // namespace ufo::map

#endif  // UFO_MAP_SURFEL_NODE_H