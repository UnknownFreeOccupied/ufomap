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

#ifndef UFO_MAP_SURFEL_MAP_HPP
#define UFO_MAP_SURFEL_MAP_HPP

// UFO
#include <ufo/map/surfel/surfel.hpp>
#include <ufo/util/buffer.hpp>
#include <ufo/util/enum.hpp>

// STL
#include <cstdint>
#include <deque>
#include <functional>
#include <stack>
#include <type_traits>
#include <utility>

namespace ufo
{
template <bool Cached, class Derived, offset_t N, class Index, class Node, class Code,
          class Key, class Point>
class SurfelMap
{
 private:
	using SurfelT = std::conditional_t<Cached, SurfelCached, Surfel>;

 public:
	//
	// Surfel block
	//

	// [[nodiscard]] DataBlock<count_t, N>& countBlock(pos_t block) { return surfel_[block];
	// }

	// [[nodiscard]] DataBlock<count_t, N> const& countBlock(pos_t block) const
	// {
	// 	return surfel_[block];
	// }

	//
	// Get surfel
	//

	[[nodiscard]] Surfel surfel(Index node) const
	{
		auto pos = indices_[node.pos][node.offset];
		return NULL_POS != pos ? surfel_[pos] : Surfel{};
	}

	[[nodiscard]] Surfel surfel(Node node) const { return surfel(derived().index(node)); }

	[[nodiscard]] Surfel surfel(Code code) const { return surfel(derived().index(code)); }

	[[nodiscard]] Surfel surfel(Key key) const { return surfel(derived().index(key)); }

	[[nodiscard]] Surfel surfel(Point coord, depth_t depth = 0) const
	{
		return surfel(derived().index(coord, depth));
	}

	//
	// Surfel empty
	//

	[[nodiscard]] bool surfelEmpty(Index node) const
	{
		auto pos = indices_[node.pos][node.offset];
		return NULL_POS == pos || surfel_[pos].empty();
	}

	[[nodiscard]] bool surfelEmpty(Node node) const
	{
		return surfelEmpty(derived().index(node));
	}

	[[nodiscard]] bool surfelEmpty(Code code) const
	{
		return surfelEmpty(derived().index(code));
	}

	[[nodiscard]] bool surfelEmpty(Key key) const
	{
		return surfelEmpty(derived().index(key));
	}

	[[nodiscard]] bool surfelEmpty(Point coord, depth_t depth = 0) const
	{
		return surfelEmpty(derived().index(coord, depth));
	}

	//
	// Surfel mean
	//

	[[nodiscard]] decltype(std::declval<Surfel>().mean()) surfelMean(Index node) const
	{
		return surfel_[indices_[node.pos][node.offset]].mean();
	}

	[[nodiscard]] decltype(std::declval<Surfel>().mean()) surfelMean(Node node) const
	{
		return surfelMean(derived().index(node));
	}

	[[nodiscard]] decltype(std::declval<Surfel>().mean()) surfelMean(Code code) const
	{
		return surfelMean(derived().index(code));
	}

	[[nodiscard]] decltype(std::declval<Surfel>().mean()) surfelMean(Key key) const
	{
		return surfelMean(derived().index(key));
	}

	[[nodiscard]] decltype(std::declval<Surfel>().mean()) surfelMean(
	    Point coord, depth_t depth = 0) const
	{
		return surfelMean(derived().index(coord, depth));
	}

	//
	// Surfel normal
	//

	[[nodiscard]] decltype(std::declval<Surfel>().normal()) surfelNormal(Index node) const
	{
		return surfel_[indices_[node.pos][node.offset]].normal();
	}

	[[nodiscard]] decltype(std::declval<Surfel>().normal()) surfelNormal(Node node) const
	{
		return surfelNormal(derived().index(node));
	}

	[[nodiscard]] decltype(std::declval<Surfel>().normal()) surfelNormal(Code code) const
	{
		return surfelNormal(derived().index(code));
	}

	[[nodiscard]] decltype(std::declval<Surfel>().normal()) surfelNormal(Key key) const
	{
		return surfelNormal(derived().index(key));
	}

	[[nodiscard]] decltype(std::declval<Surfel>().normal()) surfelNormal(
	    Point coord, depth_t depth = 0) const
	{
		return surfelNormal(derived().index(coord, depth));
	}

	//
	// Surfel planarity
	//

	[[nodiscard]] decltype(std::declval<Surfel>().planarity()) surfelPlanarity(
	    Index node) const
	{
		return surfel_[indices_[node.pos][node.offset]].planarity();
	}

	[[nodiscard]] decltype(std::declval<Surfel>().planarity()) surfelPlanarity(
	    Node node) const
	{
		return surfelPlanarity(derived().index(node));
	}

	[[nodiscard]] decltype(std::declval<Surfel>().planarity()) surfelPlanarity(
	    Code code) const
	{
		return surfelPlanarity(derived().index(code));
	}

	[[nodiscard]] decltype(std::declval<Surfel>().planarity()) surfelPlanarity(
	    Key key) const
	{
		return surfelPlanarity(derived().index(key));
	}

	[[nodiscard]] decltype(std::declval<Surfel>().planarity()) surfelPlanarity(
	    Point coord, depth_t depth = 0) const
	{
		return surfelPlanarity(derived().index(coord, depth));
	}

	//
	// Surfel covariance
	//

	[[nodiscard]] decltype(std::declval<Surfel>().covariance()) surfelCovariance(
	    Index node) const
	{
		return surfel_[indices_[node.pos][node.offset]].covariance();
	}

	[[nodiscard]] decltype(std::declval<Surfel>().covariance()) surfelCovariance(
	    Node node) const
	{
		return surfelCovariance(derived().index(node));
	}

	[[nodiscard]] decltype(std::declval<Surfel>().covariance()) surfelCovariance(
	    Code code) const
	{
		return surfelCovariance(derived().index(code));
	}

	[[nodiscard]] decltype(std::declval<Surfel>().covariance()) surfelCovariance(
	    Key key) const
	{
		return surfelCovariance(derived().index(key));
	}

	[[nodiscard]] decltype(std::declval<Surfel>().covariance()) surfelCovariance(
	    Point coord, depth_t depth = 0) const
	{
		return surfelCovariance(derived().index(coord, depth));
	}

	//
	// Surfel eigenvalues
	//

	[[nodiscard]] decltype(std::declval<Surfel>().eigenValues()) surfelEigenValues(
	    Index node) const
	{
		return surfel_[indices_[node.pos][node.offset]].eigenValues();
	}

	[[nodiscard]] decltype(std::declval<Surfel>().eigenValues()) surfelEigenValues(
	    Node node) const
	{
		return surfelEigenValues(derived().index(node));
	}

	[[nodiscard]] decltype(std::declval<Surfel>().eigenValues()) surfelEigenValues(
	    Code code) const
	{
		return surfelEigenValues(derived().index(code));
	}

	[[nodiscard]] decltype(std::declval<Surfel>().eigenValues()) surfelEigenValues(
	    Key key) const
	{
		return surfelEigenValues(derived().index(key));
	}

	[[nodiscard]] decltype(std::declval<Surfel>().eigenValues()) surfelEigenValues(
	    Point coord, depth_t depth = 0) const
	{
		return surfelEigenValues(derived().index(coord, depth));
	}

	//
	// Surfel eigen vectors
	//

	[[nodiscard]] decltype(std::declval<Surfel>().eigenVectors()) surfelEigenVectors(
	    Index node) const
	{
		return surfel_[indices_[node.pos][node.offset]].eigenVectors();
	}

	[[nodiscard]] decltype(std::declval<Surfel>().eigenVectors()) surfelEigenVectors(
	    Node node) const
	{
		return surfelEigenVectors(derived().index(node));
	}

	[[nodiscard]] decltype(std::declval<Surfel>().eigenVectors()) surfelEigenVectors(
	    Code code) const
	{
		return surfelEigenVectors(derived().index(code));
	}

	[[nodiscard]] decltype(std::declval<Surfel>().eigenVectors()) surfelEigenVectors(
	    Key key) const
	{
		return surfelEigenVectors(derived().index(key));
	}

	[[nodiscard]] decltype(std::declval<Surfel>().eigenVectors()) surfelEigenVectors(
	    Point coord, depth_t depth = 0) const
	{
		return surfelEigenVectors(derived().index(coord, depth));
	}

	//
	// Surfel num points
	//

	[[nodiscard]] decltype(std::declval<Surfel>().numPoints()) surfelNumPoints(
	    Index node) const
	{
		return surfel_[indices_[node.pos][node.offset]].numPoints();
	}

	[[nodiscard]] decltype(std::declval<Surfel>().numPoints()) surfelNumPoints(
	    Node node) const
	{
		return surfelNumPoints(derived().index(node));
	}

	[[nodiscard]] decltype(std::declval<Surfel>().numPoints()) surfelNumPoints(
	    Code code) const
	{
		return surfelNumPoints(derived().index(code));
	}

	[[nodiscard]] decltype(std::declval<Surfel>().numPoints()) surfelNumPoints(
	    Key key) const
	{
		return surfelNumPoints(derived().index(key));
	}

	[[nodiscard]] decltype(std::declval<Surfel>().numPoints()) surfelNumPoints(
	    Point coord, depth_t depth = 0) const
	{
		return surfelNumPoints(derived().index(coord, depth));
	}

	//
	// Surfel sum
	//

	[[nodiscard]] decltype(std::declval<Surfel>().sum()) surfelSum(Index node) const
	{
		return surfel_[indices_[node.pos][node.offset]].sum();
	}

	[[nodiscard]] decltype(std::declval<Surfel>().sum()) surfelSum(Node node) const
	{
		return surfelSum(derived().index(node));
	}

	[[nodiscard]] decltype(std::declval<Surfel>().sum()) surfelSum(Code code) const
	{
		return surfelSum(derived().index(code));
	}

	[[nodiscard]] decltype(std::declval<Surfel>().sum()) surfelSum(Key key) const
	{
		return surfelSum(derived().index(key));
	}

	[[nodiscard]] decltype(std::declval<Surfel>().sum()) surfelSum(Point   coord,
	                                                               depth_t depth = 0) const
	{
		return surfelSum(derived().index(coord, depth));
	}

	//
	// Surfel sum squares
	//

	[[nodiscard]] decltype(std::declval<Surfel>().sumSquares()) surfelSumSquares(
	    Index node) const
	{
		return surfel_[indices_[node.pos][node.offset]].sumSquares();
	}

	[[nodiscard]] decltype(std::declval<Surfel>().sumSquares()) surfelSumSquares(
	    Node node) const
	{
		return surfelSumSquares(derived().index(node));
	}

	[[nodiscard]] decltype(std::declval<Surfel>().sumSquares()) surfelSumSquares(
	    Code code) const
	{
		return surfelSumSquares(derived().index(code));
	}

	[[nodiscard]] decltype(std::declval<Surfel>().sumSquares()) surfelSumSquares(
	    Key key) const
	{
		return surfelSumSquares(derived().index(key));
	}

	[[nodiscard]] decltype(std::declval<Surfel>().sumSquares()) surfelSumSquares(
	    Point coord, depth_t depth = 0) const
	{
		return surfelSumSquares(derived().index(coord, depth));
	}

	//
	// Surfel set
	//

	void surfelSet(Index node, Surfel const& surfel)
	{
		derived().apply(
		    node, [this, &surfel](Index node) { createOrAssignSurfel(node, surfel); },
		    [this, &surfel](pos_t block) {
			    for (offset_t i{}; N != i; ++i) {
				    createOrAssignSurfel(Index{block, i}, surfel);
			    }
		    });
	}

	Node surfelSet(Node node, Surfel const& surfel, bool propagate = true)
	{
		return derived().apply(
		    node, [this, &surfel](Index node) { createOrAssignSurfel(node, surfel); },
		    [this, &surfel](pos_t block) {
			    for (offset_t i{}; N != i; ++i) {
				    createOrAssignSurfel(Index{block, i}, surfel);
			    }
		    },
		    propagate);
	}

	Node surfelSet(Code code, Surfel const& surfel, bool propagate = true)
	{
		return derived().apply(
		    code, [this, &surfel](Index node) { createOrAssignSurfel(node, surfel); },
		    [this, &surfel](pos_t block) {
			    for (offset_t i{}; N != i; ++i) {
				    createOrAssignSurfel(Index{block, i}, surfel);
			    }
		    },
		    propagate);
	}

	Node surfelSet(Key key, Surfel const& surfel, bool propagate = true)
	{
		return surfelSet(derived().toCode(key), surfel, propagate);
	}

	Node surfelSet(Point coord, Surfel const& surfel, bool propagate = true,
	               depth_t depth = 0)
	{
		return surfelSet(derived().toCode(coord, depth), surfel, propagate);
	}

	//
	// Surfel add
	//

	void surfelAdd(Index node, Surfel const& surfel)
	{
		derived().apply(
		    node,
		    [this, &surfel](Index node) {
			    auto pos = createSurfel(node);
			    surfel_[pos] += surfel;
		    },
		    [this, &surfel](pos_t block) {
			    for (offset_t i{}; N != i; ++i) {
				    auto pos = createSurfel(Index(block, i));
				    surfel_[pos] += surfel;
			    }
		    });
	}

	Node surfelAdd(Node node, Surfel const& surfel, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, &surfel](Index node) {
			    auto pos = createSurfel(node);
			    surfel_[pos] += surfel;
		    },
		    [this, &surfel](pos_t block) {
			    for (offset_t i{}; N != i; ++i) {
				    auto pos = createSurfel(Index(block, i));
				    surfel_[pos] += surfel;
			    }
		    },
		    propagate);
	}

	Node surfelAdd(Code code, Surfel const& surfel, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, &surfel](Index node) {
			    auto pos = createSurfel(node);
			    surfel_[pos] += surfel;
		    },
		    [this, &surfel](pos_t block) {
			    for (offset_t i{}; N != i; ++i) {
				    auto pos = createSurfel(Index(block, i));
				    surfel_[pos] += surfel;
			    }
		    },
		    propagate);
	}

	Node surfelAdd(Key key, Surfel const& surfel, bool propagate = true)
	{
		return surfelAdd(derived().toCode(key), surfel, propagate);
	}

	Node surfelAdd(Point coord, Surfel const& surfel, bool propagate = true,
	               depth_t depth = 0)
	{
		return surfelAdd(derived().toCode(coord, depth), surfel, propagate);
	}

	void surfelAdd(Point point, bool propagate = true, depth_t depth = 0)
	{
		surfelAdd(point, point, propagate, depth);
	}

	template <class InputIt>
	void surfelAdd(InputIt first, InputIt last, bool propagate = true, depth_t depth = 0)
	{
		struct CodeOrIndexPoint
		    : CodeOrIndex
		    , Point {
			constexpr CodeOrIndexPoint() = default;
			CodeOrIndexPoint(Code code, Point point) : CodeOrIndexPoint(code), Point(point) {}
		};

		std::vector<CodeOrIndexPoint> data;
		data.reserve(std::distance(first, last));

		for (; first != last; ++first) {
			data.emplace_back(derived().toCode(*first, depth), *first);
		}

		std::sort(std::begin(data), std::end(data),
		          [](auto a, auto b) { return a.code > b.code; });

		derived().createIndicesFromCodes(data);

		for (auto it = std::cbegin(data), last = std::cend(data); it != last;) {
			auto it_end = std::find_if(std::next(it), last,
			                           [idx = it->index](auto v) { return idx != v.index; });

			surfelAdd(it->index, it, it_end);
		}

		if (propagate) {
			derived().propagateModified();
		}
	}

	template <class PointRange>
	void surfelAdd(PointRange const& points, bool propagate = true, depth_t depth = 0)
	{
		surfelAdd(std::cbegin(points), std::cend(points), propagate, depth);
	}

	void surfelAdd(std::initializer_list<Point> points, bool propagate = true,
	               depth_t depth = 0)
	{
		surfelAdd(std::cbegin(points), std::cend(points), propagate, depth);
	}

	void surfelAdd(Index node, Point point)
	{
		derived().apply(
		    node,
		    [this, point](Index node) {
			    auto pos = createSurfel(node);
			    surfel_[pos] += point;
		    },
		    [this, point](pos_t block) {
			    for (offset_t i{}; N != i; ++i) {
				    auto pos = createSurfel(Index{block, i});
				    surfel_[pos] += point;
			    }
		    });
	}

	template <class InputIt>
	void surfelAdd(Index node, InputIt first, InputIt last)
	{
		derived().apply(
		    node,
		    [this, first, last](Index node) {
			    auto pos = createSurfel(node);
			    surfel_[pos].addPoint(first, last);
		    },
		    [this, first, last](pos_t block) {
			    for (offset_t i{}; N != i; ++i) {
				    auto pos = createSurfel(Index{block, i});
				    surfel_[pos].addPoint(first, last);
			    }
		    });
	}

	template <class PointRange>
	void surfelAdd(Index node, PointRange const& points)
	{
		surfelAdd(node, std::cbegin(points), std::cend(points));
	}

	void surfelAdd(Index node, std::initializer_list<Point> points)
	{
		surfelAdd(node, std::cbegin(points), std::cend(points));
	}

	Node surfelAdd(Node node, Point point, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, point](Index node) {
			    auto pos = createSurfel(node);
			    surfel_[pos] += point;
		    },
		    [this, point](pos_t block) {
			    for (offset_t i{}; N != i; ++i) {
				    auto pos = createSurfel(Index{block, i});
				    surfel_[pos] += point;
			    }
		    },
		    propagate);
	}

	template <class InputIt>
	Node surfelAdd(Node node, InputIt first, InputIt last, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, first, last](Index node) {
			    auto pos = createSurfel(node);
			    surfel_[pos].addPoint(first, last);
		    },
		    [this, first, last](pos_t block) {
			    for (offset_t i{}; N != i; ++i) {
				    auto pos = createSurfel(Index{block, i});
				    surfel_[pos].addPoint(first, last);
			    }
		    },
		    propagate);
	}

	template <class PointRange>
	Node surfelAdd(Node node, PointRange const& points, bool propagate = true)
	{
		return surfelAdd(node, std::cbegin(points), std::cend(points), propagate);
	}

	Node surfelAdd(Node node, std::initializer_list<Point> points, bool propagate = true)
	{
		return surfelAdd(node, std::cbegin(points), std::cend(points), propagate);
	}

	Node surfelAdd(Code code, Point point, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, point](Index node) {
			    auto pos = createSurfel(node);
			    surfel_[pos] += point;
		    },
		    [this, point](pos_t block) {
			    for (offset_t i{}; N != i; ++i) {
				    auto pos = createSurfel(Index{block, i});
				    surfel_[pos] += point;
			    }
		    },
		    propagate);
	}

	template <class InputIt>
	Node surfelAdd(Code code, InputIt first, InputIt last, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, first, last](Index node) {
			    auto pos = createSurfel(node);
			    surfel_[pos].addPoint(first, last);
		    },
		    [this, first, last](pos_t block) {
			    for (offset_t i{}; N != i; ++i) {
				    auto pos = createSurfel(Index{block, i});
				    surfel_[pos].addPoint(first, last);
			    }
		    },
		    propagate);
	}

	template <class PointRange>
	Node surfelAdd(Code code, PointRange const& points, bool propagate = true)
	{
		return surfelAdd(code, std::cbegin(points), std::cend(points), propagate);
	}

	Node surfelAdd(Code code, std::initializer_list<Point> points, bool propagate = true)
	{
		return surfelAdd(code, std::cbegin(points), std::cend(points), propagate);
	}

	Node surfelAdd(Key key, Point point, bool propagate = true)
	{
		return surfelAdd(derived().toCode(key), point, propagate);
	}

	template <class InputIt>
	Node surfelAdd(Key key, InputIt first, InputIt last, bool propagate = true)
	{
		return surfelAdd(derived().toCode(key), first, last, propagate);
	}

	template <class PointRange>
	Node surfelAdd(Key key, PointRange const& points, bool propagate = true)
	{
		return surfelAdd(key, std::cbegin(points), std::cend(points), propagate);
	}

	Node surfelAdd(Key key, std::initializer_list<Point> points, bool propagate = true)
	{
		return surfelAdd(key, std::cbegin(points), std::cend(points), propagate);
	}

	Node surfelAdd(Point coord, Point point, bool propagate = true, depth_t depth = 0)
	{
		return surfelAdd(derived().toCode(coord, depth), point, propagate);
	}

	template <class InputIt>
	Node surfelAdd(Point coord, InputIt first, InputIt last, bool propagate = true,
	               depth_t depth = 0)
	{
		return surfelAdd(derived().toCode(coord, depth), first, last, propagate);
	}

	template <class PointRange>
	Node surfelAdd(Point coord, PointRange const& points, bool propagate = true,
	               depth_t depth = 0)
	{
		return surfelAdd(coord, std::cbegin(points), std::cend(points), propagate, depth);
	}

	Node surfelAdd(Point coord, std::initializer_list<Point> points, bool propagate = true,
	               depth_t depth = 0)
	{
		return surfelAdd(coord, std::cbegin(points), std::cend(points), propagate, depth);
	}

	//
	// Surfel remove
	//

	void surfelRemove(Index node, Surfel const& surfel)
	{
		derived().apply(
		    node,
		    [this, &surfel](Index node) {
			    auto pos = indices_[node.pos][node.offset];
			    if (NULL_POS != pos) {
				    surfel_[pos] -= surfel;
				    if (surfel_[pos].empty()) {
					    deleteSurfel(node);
				    }
			    }
		    },
		    [this, &surfel](pos_t block) {
			    for (offset_t i{}; N != i; ++i) {
				    auto pos = indices_[block][i];
				    if (NULL_POS != pos) {
					    surfel_[pos] -= surfel;
					    if (surfel_[pos].empty()) {
						    deleteSurfel(Index{block, i});
					    }
				    }
			    }
		    });
	}

	Node surfelRemove(Node node, Surfel const& surfel, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, &surfel](Index node) {
			    auto pos = indices_[node.pos][node.offset];
			    if (NULL_POS != pos) {
				    surfel_[pos] -= surfel;
				    if (surfel_[pos].empty()) {
					    deleteSurfel(node);
				    }
			    }
		    },
		    [this, &surfel](pos_t block) {
			    for (offset_t i{}; N != i; ++i) {
				    auto pos = indices_[block][i];
				    if (NULL_POS != pos) {
					    surfel_[pos] -= surfel;
					    if (surfel_[pos].empty()) {
						    deleteSurfel(Index{block, i});
					    }
				    }
			    }
		    },
		    propagate);
	}

	Node surfelRemove(Code code, Surfel const& surfel, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, &surfel](Index node) {
			    auto pos = indices_[node.pos][node.offset];
			    if (NULL_POS != pos) {
				    surfel_[pos] -= surfel;
				    if (surfel_[pos].empty()) {
					    deleteSurfel(node);
				    }
			    }
		    },
		    [this, &surfel](pos_t block) {
			    for (offset_t i{}; N != i; ++i) {
				    auto pos = indices_[block][i];
				    if (NULL_POS != pos) {
					    surfel_[pos] -= surfel;
					    if (surfel_[pos].empty()) {
						    deleteSurfel(Index{block, i});
					    }
				    }
			    }
		    },
		    propagate);
	}

	Node surfelRemove(Key key, Surfel const& surfel, bool propagate = true)
	{
		return surfelRemove(derived().toCode(key), surfel, propagate);
	}

	Node surfelRemove(Point coord, Surfel const& surfel, bool propagate = true,
	                  depth_t depth = 0)
	{
		return surfelRemove(derived().toCode(coord, depth), surfel, propagate);
	}

	void surfelRemove(Point point, bool propagate = true, depth_t depth = 0)
	{
		surfelRemove(point, point, propagate, depth);
	}

	template <class InputIt>
	void surfelRemove(InputIt first, InputIt last, bool propagate = true, depth_t depth = 0)
	{
		struct CodeOrIndexPoint
		    : CodeOrIndex
		    , Point {
			constexpr CodeOrIndexPoint() = default;
			CodeOrIndexPoint(Code code, Point point) : CodeOrIndexPoint(code), Point(point) {}
		};

		std::vector<CodeOrIndexPoint> data;
		data.reserve(std::distance(first, last));

		for (; first != last; ++first) {
			data.emplace_back(derived().toCode(*first, depth), *first);
		}

		std::sort(std::begin(data), std::end(data),
		          [](auto a, auto b) { return a.code > b.code; });

		derived().createIndicesFromCodes(data);

		for (auto it = std::cbegin(data), last = std::cend(data); it != last;) {
			auto it_end = std::find_if(std::next(it), last,
			                           [idx = it->index](auto v) { return idx != v.index; });

			surfelRemove(it->index, it, it_end);
		}

		if (propagate) {
			derived().propagateModified();
		}
	}

	template <class PointRange>
	void surfelRemove(PointRange const& points, bool propagate = true, depth_t depth = 0)
	{
		surfelRemove(std::cbegin(points), std::cend(points), propagate, depth);
	}

	void surfelRemove(std::initializer_list<Point> points, bool propagate = true,
	                  depth_t depth = 0)
	{
		surfelRemove(std::cbegin(points), std::cend(points), propagate, depth);
	}

	void surfelRemove(Index node, Point point)
	{
		derived().apply(
		    node,
		    [this, point](Index node) {
			    auto pos = indices_[node.pos][node.offset];
			    if (NULL_POS != pos) {
				    surfel_[pos] -= point;
				    if (surfel_[pos].empty()) {
					    deleteSurfel(node);
				    }
			    }
		    },
		    [this, point](pos_t block) {
			    for (offset_t i{}; N != i; ++i) {
				    auto pos = indices_[block][i];
				    if (NULL_POS != pos) {
					    surfel_[pos] -= point;
					    if (surfel_[pos].empty()) {
						    deleteSurfel(Index{block, i});
					    }
				    }
			    }
		    });
	}

	template <class InputIt>
	void surfelRemove(Index node, InputIt first, InputIt last)
	{
		derived().apply(
		    node,
		    [this, first, last](Index node) {
			    auto pos = indices_[node.pos][node.offset];
			    if (NULL_POS != pos) {
				    surfel_[pos].removePoint(first, last);
				    if (surfel_[pos].empty()) {
					    deleteSurfel(node);
				    }
			    }
		    },
		    [this, first, last](pos_t block) {
			    for (offset_t i{}; N != i; ++i) {
				    auto pos = indices_[block][i];
				    if (NULL_POS != pos) {
					    surfel_[pos].removePoint(first, last);
					    if (surfel_[pos].empty()) {
						    deleteSurfel(Index{block, i});
					    }
				    }
			    }
		    });
	}

	template <class PointRange>
	void surfelRemove(Index node, PointRange const& points)
	{
		surfelRemove(node, std::cbegin(points), std::cend(points));
	}

	void surfelRemove(Index node, std::initializer_list<Point> points)
	{
		surfelRemove(node, std::cbegin(points), std::cend(points));
	}

	Node surfelRemove(Node node, Point point, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, point](Index node) {
			    auto pos = indices_[node.pos][node.offset];
			    if (NULL_POS != pos) {
				    surfel_[pos] -= point;
				    if (surfel_[pos].empty()) {
					    deleteSurfel(node);
				    }
			    }
		    },
		    [this, point](pos_t block) {
			    for (offset_t i{}; N != i; ++i) {
				    auto pos = indices_[block][i];
				    if (NULL_POS != pos) {
					    surfel_[pos] -= point;
					    if (surfel_[pos].empty()) {
						    deleteSurfel(Index{block, i});
					    }
				    }
			    }
		    },
		    propagate);
	}

	template <class InputIt>
	Node surfelRemove(Node node, InputIt first, InputIt last, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, first, last](Index node) {
			    auto pos = indices_[node.pos][node.offset];
			    if (NULL_POS != pos) {
				    surfel_[pos].removePoint(first, last);
				    if (surfel_[pos].empty()) {
					    deleteSurfel(node);
				    }
			    }
		    },
		    [this, first, last](pos_t block) {
			    for (offset_t i{}; N != i; ++i) {
				    auto pos = indices_[block][i];
				    if (NULL_POS != pos) {
					    surfel_[pos].removePoint(first, last);
					    if (surfel_[pos].empty()) {
						    deleteSurfel(Index{block, i});
					    }
				    }
			    }
		    },
		    propagate);
	}

	template <class PointRange>
	Node surfelRemove(Node node, PointRange const& points, bool propagate = true)
	{
		return surfelRemove(node, std::cbegin(points), std::cend(points), propagate);
	}

	Node surfelRemove(Node node, std::initializer_list<Point> points, bool propagate = true)
	{
		return surfelRemove(node, std::cbegin(points), std::cend(points), propagate);
	}

	Node surfelRemove(Code code, Point point, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, point](Index node) {
			    auto pos = indices_[node.pos][node.offset];
			    if (NULL_POS != pos) {
				    surfel_[pos] -= point;
				    if (surfel_[pos].empty()) {
					    deleteSurfel(node);
				    }
			    }
		    },
		    [this, point](pos_t block) {
			    for (offset_t i{}; N != i; ++i) {
				    auto pos = indices_[block][i];
				    if (NULL_POS != pos) {
					    surfel_[pos] -= point;
					    if (surfel_[pos].empty()) {
						    deleteSurfel(Index{block, i});
					    }
				    }
			    }
		    },
		    propagate);
	}

	template <class InputIt>
	Node surfelRemove(Code code, InputIt first, InputIt last, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, first, last](Index node) {
			    auto pos = indices_[node.pos][node.offset];
			    if (NULL_POS != pos) {
				    surfel_[pos].removePoint(first, last);
				    if (surfel_[pos].empty()) {
					    deleteSurfel(node);
				    }
			    }
		    },
		    [this, first, last](pos_t block) {
			    for (offset_t i{}; N != i; ++i) {
				    auto pos = indices_[block][i];
				    if (NULL_POS != pos) {
					    surfel_[pos].removePoint(first, last);
					    if (surfel_[pos].empty()) {
						    deleteSurfel(Index{block, i});
					    }
				    }
			    }
		    },
		    propagate);
	}

	template <class PointRange>
	Node surfelRemove(Code code, PointRange const& points, bool propagate = true)
	{
		return surfelRemove(code, std::cbegin(points), std::cend(points), propagate);
	}

	Node surfelRemove(Code code, std::initializer_list<Point> points, bool propagate = true)
	{
		return surfelRemove(code, std::cbegin(points), std::cend(points), propagate);
	}

	Node surfelRemove(Key key, Point point, bool propagate = true)
	{
		return surfelRemove(derived().toCode(key), point, propagate);
	}

	template <class InputIt>
	Node surfelRemove(Key key, InputIt first, InputIt last, bool propagate = true)
	{
		return surfelRemove(derived().toCode(key), first, last, propagate);
	}

	template <class PointRange>
	Node surfelRemove(Key key, PointRange const& points, bool propagate = true)
	{
		return surfelRemove(key, std::cbegin(points), std::cend(points), propagate);
	}

	Node surfelRemove(Key key, std::initializer_list<Point> points, bool propagate = true)
	{
		return surfelRemove(key, std::cbegin(points), std::cend(points), propagate);
	}

	Node surfelRemove(Point coord, Point point, bool propagate = true, depth_t depth = 0)
	{
		return surfelRemove(derived().toCode(coord, depth), point, propagate);
	}

	template <class InputIt>
	Node surfelRemove(Point coord, InputIt first, InputIt last, bool propagate = true,
	                  depth_t depth = 0)
	{
		return surfelRemove(derived().toCode(coord, depth), first, last, propagate);
	}

	template <class PointRange>
	Node surfelRemove(Point coord, PointRange const& points, bool propagate = true,
	                  depth_t depth = 0)
	{
		return surfelRemove(coord, std::cbegin(points), std::cend(points), propagate, depth);
	}

	Node surfelRemove(Point coord, std::initializer_list<Point> points,
	                  bool propagate = true, depth_t depth = 0)
	{
		return surfelRemove(coord, std::cbegin(points), std::cend(points), propagate, depth);
	}

	//
	// Surfel update
	//

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, Surfel>, bool> = true>
	void surfelUpdate(Index node, UnaryOp unary_op)
	{
		derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    auto pos = indices_[node.pos][node.offset];
			    if (NULL_POS == pos) {
				    if (auto s = unary_op(Surfel{}); !s.empty()) {
					    createOrAssignSurfel(node, s);
				    }
			    } else {
				    if (auto s = unary_op(surfel_[pos]); !s.empty()) {
					    surfel_[pos] = s;
				    } else {
					    deleteSurfel(node);
				    }
			    }
		    },
		    [this, unary_op](pos_t block) {
			    for (offset_t i{}; N != i; ++i) {
				    Index node(block, i);
				    auto  pos = indices_[node.pos][node.offset];
				    if (NULL_POS == pos) {
					    if (auto s = unary_op(Surfel{}); !s.empty()) {
						    createOrAssignSurfel(node, s);
					    }
				    } else {
					    if (auto s = unary_op(surfel_[pos]); !s.empty()) {
						    surfel_[pos] = s;
					    } else {
						    deleteSurfel(node);
					    }
				    }
			    }
		    });
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, Surfel>, bool> = true>
	void surfelUpdate(Index node, BinaryOp binary_op)
	{
		derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    auto pos = indices_[node.pos][node.offset];
			    if (NULL_POS == pos) {
				    if (auto s = binary_op(node, Surfel{}); !s.empty()) {
					    createOrAssignSurfel(node, s);
				    }
			    } else {
				    if (auto s = binary_op(node, surfel_[pos]); !s.empty()) {
					    surfel_[pos] = s;
				    } else {
					    deleteSurfel(node);
				    }
			    }
		    },
		    [this, binary_op](pos_t block) {
			    for (offset_t i{}; N != i; ++i) {
				    Index node(block, i);
				    auto  pos = indices_[node.pos][node.offset];
				    if (NULL_POS == pos) {
					    if (auto s = binary_op(node, Surfel{}); !s.empty()) {
						    createOrAssignSurfel(node, s);
					    }
				    } else {
					    if (auto s = binary_op(node, surfel_[pos]); !s.empty()) {
						    surfel_[pos] = s;
					    } else {
						    deleteSurfel(node);
					    }
				    }
			    }
		    });
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, Surfel>, bool> = true>
	Node surfelUpdate(Node node, UnaryOp unary_op, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    auto pos = indices_[node.pos][node.offset];
			    if (NULL_POS == pos) {
				    if (auto s = unary_op(Surfel{}); !s.empty()) {
					    createOrAssignSurfel(node, s);
				    }
			    } else {
				    if (auto s = unary_op(surfel_[pos]); !s.empty()) {
					    surfel_[pos] = s;
				    } else {
					    deleteSurfel(node);
				    }
			    }
		    },
		    [this, unary_op](pos_t block) {
			    for (offset_t i{}; N != i; ++i) {
				    Index node(block, i);
				    auto  pos = indices_[node.pos][node.offset];
				    if (NULL_POS == pos) {
					    if (auto s = unary_op(Surfel{}); !s.empty()) {
						    createOrAssignSurfel(node, s);
					    }
				    } else {
					    if (auto s = unary_op(surfel_[pos]); !s.empty()) {
						    surfel_[pos] = s;
					    } else {
						    deleteSurfel(node);
					    }
				    }
			    }
		    },
		    propagate);
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, Surfel>, bool> = true>
	Node surfelUpdate(Node node, BinaryOp binary_op, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    auto pos = indices_[node.pos][node.offset];
			    if (NULL_POS == pos) {
				    if (auto s = binary_op(node, Surfel{}); !s.empty()) {
					    createOrAssignSurfel(node, s);
				    }
			    } else {
				    if (auto s = binary_op(node, surfel_[pos]); !s.empty()) {
					    surfel_[pos] = s;
				    } else {
					    deleteSurfel(node);
				    }
			    }
		    },
		    [this, binary_op](pos_t block) {
			    for (offset_t i{}; N != i; ++i) {
				    Index node(block, i);
				    auto  pos = indices_[node.pos][node.offset];
				    if (NULL_POS == pos) {
					    if (auto s = binary_op(node, Surfel{}); !s.empty()) {
						    createOrAssignSurfel(node, s);
					    }
				    } else {
					    if (auto s = binary_op(node, surfel_[pos]); !s.empty()) {
						    surfel_[pos] = s;
					    } else {
						    deleteSurfel(node);
					    }
				    }
			    }
		    },
		    propagate);
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, Surfel>, bool> = true>
	Node surfelUpdate(Code code, UnaryOp unary_op, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, unary_op](Index node) {
			    auto pos = indices_[node.pos][node.offset];
			    if (NULL_POS == pos) {
				    if (auto s = unary_op(Surfel{}); !s.empty()) {
					    createOrAssignSurfel(node, s);
				    }
			    } else {
				    if (auto s = unary_op(surfel_[pos]); !s.empty()) {
					    surfel_[pos] = s;
				    } else {
					    deleteSurfel(node);
				    }
			    }
		    },
		    [this, unary_op](pos_t block) {
			    for (offset_t i{}; N != i; ++i) {
				    Index node(block, i);
				    auto  pos = indices_[node.pos][node.offset];
				    if (NULL_POS == pos) {
					    if (auto s = unary_op(Surfel{}); !s.empty()) {
						    createOrAssignSurfel(node, s);
					    }
				    } else {
					    if (auto s = unary_op(surfel_[pos]); !s.empty()) {
						    surfel_[pos] = s;
					    } else {
						    deleteSurfel(node);
					    }
				    }
			    }
		    },
		    propagate);
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, Surfel>, bool> = true>
	Node surfelUpdate(Code code, BinaryOp binary_op, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, binary_op](Index node) {
			    auto pos = indices_[node.pos][node.offset];
			    if (NULL_POS == pos) {
				    if (auto s = binary_op(node, Surfel{}); !s.empty()) {
					    createOrAssignSurfel(node, s);
				    }
			    } else {
				    if (auto s = binary_op(node, surfel_[pos]); !s.empty()) {
					    surfel_[pos] = s;
				    } else {
					    deleteSurfel(node);
				    }
			    }
		    },
		    [this, binary_op](pos_t block) {
			    for (offset_t i{}; N != i; ++i) {
				    Index node(block, i);
				    auto  pos = indices_[node.pos][node.offset];
				    if (NULL_POS == pos) {
					    if (auto s = binary_op(node, Surfel{}); !s.empty()) {
						    createOrAssignSurfel(node, s);
					    }
				    } else {
					    if (auto s = binary_op(node, surfel_[pos]); !s.empty()) {
						    surfel_[pos] = s;
					    } else {
						    deleteSurfel(node);
					    }
				    }
			    }
		    },
		    propagate);
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, Surfel>, bool> = true>
	Node surfelUpdate(Key key, UnaryOp unary_op, bool propagate = true)
	{
		return surfelUpdate(derived().toCode(key), unary_op, propagate);
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, Surfel>, bool> = true>
	Node surfelUpdate(Key key, BinaryOp binary_op, bool propagate = true)
	{
		return surfelUpdate(derived().toCode(key), binary_op, propagate);
	}

	template <class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, Surfel>, bool> = true>
	Node surfelUpdate(Point coord, UnaryOp unary_op, bool propagate = true,
	                  depth_t depth = 0)
	{
		return surfelUpdate(derived().toCode(coord, depth), unary_op, propagate);
	}

	template <class BinaryOp,
	          std::enable_if_t<std::is_invocable_v<BinaryOp, Index, Surfel>, bool> = true>
	Node surfelUpdate(Point coord, BinaryOp binary_op, bool propagate = true,
	                  depth_t depth = 0)
	{
		return surfelUpdate(derived().toCode(coord, depth), binary_op, propagate);
	}

	//
	// Surfel clear
	//

	void surfelClear(Index node)
	{
		derived().apply(
		    node, [this](Index node) { deleteSurfel(node); },
		    [this](pos_t block) { clearImpl(block); });
	}

	Node surfelClear(Node node, bool propagate = true)
	{
		return derived().apply(
		    node, [this](Index node) { deleteSurfel(node); },
		    [this](pos_t block) { clearImpl(block); }, propagate);
	}

	Node surfelClear(Code code, bool propagate = true)
	{
		return derived().apply(
		    code, [this](Index node) { deleteSurfel(node); },
		    [this](pos_t block) { clearImpl(block); }, propagate);
	}

	Node surfelClear(Key key, bool propagate = true)
	{
		return surfelClear(derived().toCode(key), propagate);
	}

	Node surfelClear(Point coord, bool propagate = true, depth_t depth = 0)
	{
		return surfelClear(derived().toCode(coord, depth), propagate);
	}

 protected:
	//
	// Constructors
	//

	SurfelMap() { indices_.emplace_back(); }

	SurfelMap(SurfelMap const& other) = default;

	SurfelMap(SurfelMap&& other) = default;

	template <class Derived2>
	SurfelMap(SurfelMap<Cached, Derived2> const& other)
	    : indices_(other.indices_), surfel_(other.surfel_), free_surfel_(other.free_surfel_)
	{
	}

	template <class Derived2>
	SurfelMap(SurfelMap<Cached, Derived2>&& other)
	    : indices_(std::move(other.indices_))
	    , surfel_(std::move(other.surfel_))
	    , free_surfel_(std::move(other.free_surfel_))
	{
	}

	template <bool Cached2, class Derived2>
	SurfelMap(SurfelMap<Cached2, Derived2> const& other)
	    : indices_(other.indices_)
	    , surfel_(std::cbegin(other.surfel_), std::cend(other.surfel_))
	    , free_surfel_(other.free_surfel_)
	{
	}

	template <bool Cached2, class Derived2>
	SurfelMap(SurfelMap<Cached2, Derived2>&& other)
	    : indices_(std::move(other.indices_))
	    , surfel_(std::cbegin(other.surfel_), std::cend(other.surfel_))
	    , free_surfel_(std::move(other.free_surfel_))
	{
	}

	//
	// Destructor
	//

	~SurfelMap() = default;

	//
	// Assignment operator
	//

	SurfelMap& operator=(SurfelMap const& rhs) = default;

	SurfelMap& operator=(SurfelMap&& rhs) = default;

	template <class Derived2>
	SurfelMap& operator=(SurfelMap<Cached, Derived2> const& rhs)
	{
		indices_     = rhs.indices_;
		surfel_      = rhs.surfel_;
		free_surfel_ = rhs.free_surfel_;
		return *this;
	}

	template <class Derived2>
	SurfelMap& operator=(SurfelMap<Cached, Derived2>&& rhs)
	{
		indices_     = std::move(rhs.indices_);
		surfel_      = std::move(rhs.surfel_);
		free_surfel_ = std::move(rhs.free_surfel_);
		return *this;
	}

	template <bool Cached2, class Derived2>
	SurfelMap& operator=(SurfelMap<Cached2, Derived2> const& rhs)
	{
		indices_     = rhs.indices_;
		surfel_      = decltype(surfel_)(std::cbegin(rhs.surfel_), std::cend(rhs.surfel_));
		free_surfel_ = rhs.free_surfel_;
		return *this;
	}

	template <bool Cached2, class Derived2>
	SurfelMap& operator=(SurfelMap<Cached2, Derived2>&& rhs)
	{
		indices_     = std::move(rhs.indices_);
		surfel_      = decltype(surfel_)(std::cbegin(rhs.surfel_), std::cend(rhs.surfel_));
		free_surfel_ = std::move(rhs.free_surfel_);
		return *this;
	}

	//
	// Swap
	//

	void swap(SurfelMap& other)
	{
		std::swap(indices_, other.indices_);
		std::swap(surfel_, other.surfel_);
		std::swap(free_surfel_, other.free_surfel_);
	}

	//
	// Derived
	//

	[[nodiscard]] constexpr Derived& derived() { return *static_cast<Derived*>(this); }

	[[nodiscard]] constexpr Derived const& derived() const
	{
		return *static_cast<Derived const*>(this);
	}

	//
	// Initialize root
	//

	void initRoot()
	{
		auto node                       = derived().rootIndex();
		indices_[node.pos][node.offset] = NULL_POS;
	}

	//
	// Create node block
	//

	void createBlock(Index node)
	{
		auto children_block = indices_.size();
		indices_.emplace_back();
		auto pos = indices_[node.pos][node.offset];
		if (NULL_POS == pos) {
			indices_.back().fill(NULL_POS);
		} else {
			for (offset_t i{}; N != i; ++i) {
				createOrAssignSurfel(Index{children_block, i}, surfel_[pos]);
			}
		}
	}

	//
	// Fill
	//

	void fill(Index node, pos_t children_block)
	{
		auto pos = indices_[node.pos][node.offset];
		if (NULL_POS == pos) {
			indices_.back().fill(NULL_POS);
		} else {
			for (offset_t i{}; N != i; ++i) {
				createOrAssignSurfel(Index{children_block, i}, surfel_[pos]);
			}
		}
	}

	//
	// Resize
	//

	void resize(std::size_t count)
	{
		typename decltype(indices_)::value_type idx;
		idx.fill(NULL_POS);
		indices_.resize(count, idx);
	}

	//
	// Reserve
	//

	void reserveImpl(std::size_t new_cap) { indices_.reserve(new_cap); }

	//
	// Clear
	//

	void clearImpl()
	{
		indices_.resize(1);
		indices_[0].fill(NULL_POS);
		surfel_.clear();
		free_surfel_ = decltype(free_surfel_)();
	}

	void clearImpl(pos_t block)
	{
		for (offset_t i{}; N != i; ++i) {
			deleteSurfel(Index{block, i});
		}
	}

	//
	// Update block
	//

	void updateBlock(pos_t block, std::array<bool, N> modified_parent)
	{
		for (offset_t i{}; N != i; ++i) {
			if (modified_parent[i]) {
				Index node(block, i);
				updateNode(node, derived().children(node));
			}
		}
	}

	void updateNode(Index node)
	{
		Surfel surfel;

		auto children = derived().children(node);
		for (std::size_t i{}; N != i; ++i) {
			auto pos = indices_[children][i];
			if (NULL_POS != pos) {
				surfel += surfel_[pos];
			}
		}

		if (surfel.empty()) {
			deleteSurfel(node);
		} else {
			createOrAssignSurfel(node, surfel);
		}
	}

	//
	// Is prunable
	//

	[[nodiscard]] bool isPrunable(pos_t block) const
	{
		return std::all_of(std::cbegin(indices_[block]) + 1, std::cend(indices_[block]),
		                   [a = indices_[block].front()](auto b) {
			                   // FIXME: Do we need to actually look at the surfels?
			                   // The intended use of the surfels would mean that it is
			                   // impossible that two nodes have the exact same surfel. The only
			                   // time it can happen is when neither contains a surfel and hence
			                   // the surfel-index for both is NULL_POS.
			                   return a == b;
		                   });
	}

	void preparePrune(Index)
	{
		// FIXME: If the comment above is true then it is impossible to prune while we still
		// have surfels in the node; hence, we do not need to do anything here.
	}

	//
	// Memory
	//

	[[nodiscard]] std::size_t sizeofNodeTimesN(Index node)
	{
		auto pos = indices_[node.pos][node.offset];
		return N * (sizeof(pos) + (NULL_POS != pos ? sizeof(surfel_[pos]) : 0));
	}

	[[nodiscard]] std::size_t sizeofBlock(pos_t block)
	{
		std::size_t size{};
		for (offset_t i{}; N != i; ++i) {
			auto pos = indices_[block][i];
			size += sizeof(pos) + (NULL_POS != pos ? sizeof(surfel_[pos]) : 0);
		}
		return size;
	}

	[[nodiscard]] static constexpr std::size_t sizeofBlockLowerBound() noexcept
	{
		return sizeof(typename decltype(indices_)::value_type);
	}

	[[nodiscard]] static constexpr std::size_t sizeofMap() noexcept
	{
		return sizeof(indices_) + sizeof(surfel_) + sizeof(free_surfel_);
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr MapType mapType() noexcept { return MapType::SURFEL; }

	[[nodiscard]] constexpr std::size_t serializedSizeBlock(pos_t block)
	{
		std::size_t num_surfels{};
		for (auto i : indices_[block]) {
			num_surfels += NULL_POS != indices_[block][i];
		}
		return sizeof(BitSet<N>) + num_surfels * sizeof(Surfel);
	}

	template <class Container>
	constexpr std::size_t serializedSize(Container const& c) const
	{
		std::size_t size{};
		for (auto block : c) {
			size += serializedSizeBlock(block);
		}
		return size;
	}

	template <class Container>
	void readNodes(ReadBuffer& in, Container const& c)
	{
		// TODO: Implement
		// for (auto const [pos, offsets] : c) {
		// 	if (offsets.all()) {
		// 		in.read(time_[pos].data(), serializedSizeBlock());
		// 	} else {
		// 		DataBlock<time_t, N> time;
		// 		in.read(time.data(), serializedSizeBlock());
		// 		for (offset_t i{}; N != i; ++i) {
		// 			time_[pos][i] = offsets[i] ? time[i] : time_[pos][i];
		// 		}
		// 	}
		// }
	}

	template <class BlockRange>
	void writeBlocks(WriteBuffer& out, BlockRange const& blocks) const
	{
		// TODO: Implement
		// for (auto block : c) {
		// 	BitSet<N> has_surfel;
		// 	for (std::size_t i{}; N != i; ++i) {
		// 		has_surfel[i] = NULL_POS != indices_[block][i];
		// 	}
		// 	out.write(has_surfel, sizeof(BitSet<N>));
		// 	for (std::size_t i{}; N != i; ++i) {
		// 		auto pos = indices_[block][i];
		// 		if (NULL_POS != pos) {
		// 			out.write(static_cast<Surfel const&>(surfel_[pos]), sizeof(Surfel));
		// 		}
		// 	}
		// }
	}

	//
	// Create surfel
	//

	pos_t createSurfel(Index node)
	{
		auto pos = indices_[node.pos][node.offset];
		if (NULL_POS != pos) {
			return pos;
		}

		pos = free_surfel_.empty() ? surfel_.size() : free_surfel_.top();
		indices_[node.pos][node.offset] = pos;
		if (free_surfel_.empty()) {
			surfel_.emplace_back();
		} else {
			free_surfel_.pop();
		}
		return pos;
	}

	pos_t createOrAssignSurfel(Index node, Surfel const& surfel)
	{
		// TODO: What happens if surfel is empty?

		auto pos = indices_[node.pos][node.offset];
		if (NULL_POS != pos) {
			surfel_[pos] = surfel;
			return pos;
		}

		pos = free_surfel_.empty() ? surfel_.size() : free_surfel_.top();
		indices_[node.pos][node.offset] = pos;
		if (free_surfel_.empty()) {
			surfel_.push_back(surfel);
		} else {
			free_surfel_.pop();
			surfel_[pos] = surfel;
		}
		return pos;
	}

	//
	// Delete surfel
	//

	void deleteSurfel(Index node)
	{
		auto pos = indices_[node.pos][node.offset];
		if (NULL_POS == pos) {
			return;
		}

		free_surfel_.push(pos);
		indices_[node.pos][node.offset] = NULL_POS;
	}

 private:
	Container<DataBlock<pos_t, N>> indices_;
	std::deque<SurfelT>            surfel_;
	std::stack<pos_t>              free_surfel_;

	template <class Derived2, std::size_t N2>
	friend class SurfelMap;
};
}  // namespace ufo

namespace ufo
{
//
// Type traits
//

template <class Map>
struct is_surfel_map
    : std::conditional_t<is_map_type_v<Map, MapType::SURFEL>, std::true_type,
                         std::false_type> {
};
template <class Map>
inline constexpr bool is_surfel_map_v = is_surfel_map<Map>::value;
}  // namespace ufo
#endif  // UFO_MAP_SURFEL_MAP_HPP