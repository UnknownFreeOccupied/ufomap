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

#ifndef UFO_MAP_SURFEL_REFERENCE_H
#define UFO_MAP_SURFEL_REFERENCE_H

// UFO
#include <ufo/map/surfel/surfel.h>
#include <ufo/map/surfel/surfel_node.h>
#include <ufo/map/surfel/surfel_util.h>

namespace ufo::map
{
class SurfelReference
{
 public:
	//
	// Constructor
	//

	constexpr SurfelReference() = default;

	constexpr SurfelReference(SurfelReference const&) = default;

	constexpr SurfelReference(SurfelReference&&) = default;

	//
	// Assignment operator
	//

	SurfelReference& operator=(SurfelReference const&) = default;

	SurfelReference& operator=(SurfelReference&&) = default;

	//
	// Conversion
	//

	operator Surfel() const { return Surfel(sum(), sumSquares(), numPoints()); }

	//
	// Swap
	//

	void swap(SurfelReference& other)
	{
		std::swap(surfel_, other.surfel_);
		std::swap(index_, other.index_);
	}

	//
	// Empty
	//

	[[nodiscard]] constexpr bool empty() const { return 0 == numPoints(); }

	//
	// Mean
	//

	[[nodiscard]] constexpr math::Vector3<surfel_scalar_t> mean() const
	{
		return surfel::mean(sum(), numPoints());
	}

	//
	// Normal
	//

	constexpr math::Vector3d normal() const
	{
		return surfel::normal(sumSquares(), numPoints());
	}

	//
	// Planarity
	//

	constexpr double planarity() const
	{
		return surfel::planarity(sumSquares(), numPoints());
	}

	//
	// Covariance
	//

	constexpr std::array<std::array<double, 3>, 3> covariance() const
	{
		return surfel::covariance(sumSquares(), numPoints());
	}

	//
	// Eigenvalues
	//

	constexpr math::Vector3d eigenValues() const
	{
		return surfel::eigenValues(sumSquares(), numPoints());
	}

	//
	// Eigen vectors
	//

	constexpr std::array<math::Vector3d, 3> eigenVectors() const
	{
		return surfel::eigenVectors(sumSquares(), numPoints());
	}

	//
	// Num points
	//

	constexpr count_t numPoints() const { return surfel_->num_points[index_]; }

	//
	// Sum
	//

	constexpr math::Vector3<surfel_scalar_t> sum() const { return surfel_->sum[index_]; }

	//
	// Sum squares
	//

	constexpr std::array<surfel_scalar_t, 6> const& sumSquares() const
	{
		return surfel_->sum_squares[index_];
	}

 private:
	SurfelReference(SurfelNode<8> const& node, index_t const index)
	    : surfel_(&node), index_(index)
	{
	}

	SurfelReference(SurfelNode<8> const* node, index_t const index)
	    : surfel_(node), index_(index)
	{
	}

 private:
	SurfelNode<8> const* surfel_ = nullptr;
	index_t index_ = 0;

	template <class Derived>
	friend class SurfelMapBase;
};
}  // namespace ufo::map

#endif  // UFO_MAP_SURFEL_REFERENCE_H