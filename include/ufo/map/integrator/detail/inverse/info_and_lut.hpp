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

#ifndef UFO_MAP_INTEGRATOR_DETAIL_INVERSE_INFO_AND_LUT_HPP
#define UFO_MAP_INTEGRATOR_DETAIL_INVERSE_INFO_AND_LUT_HPP

// UFO
#include <ufo/map/integrator/detail/inverse/info.hpp>

// STL
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace ufo::detail
{
template <std::size_t Dim, std::size_t NumLevels>
class InverseInfoAndLut
{
 public:
	template <class... Args>
	void leafEmplaceBack(Args&&... args)
	{
		info_leaf_.emplace_back(std::forward<Args>(args)...);
	}

	template <class... Args>
	void emplaceBack(unsigned depth, Args&&... args)
	{
		info_[depth - 1u].emplace_back(std::forward<Args>(args)...);
	}

	[[nodiscard]] auto begin() { return info_.back().begin(); }

	[[nodiscard]] auto begin() const { return info_.back().begin(); }

	[[nodiscard]] auto cbegin() const { return info_.back().cbegin(); }

	[[nodiscard]] auto end()
	{
		return info_.back().empty() ? info_.back().end() : info_.back().end() - 1;
	}

	[[nodiscard]] auto end() const
	{
		return info_.back().empty() ? info_.back().end() : info_.back().end() - 1;
	}

	[[nodiscard]] auto cend() const
	{
		return info_.back().empty() ? info_.back().cend() : info_.back().cend() - 1;
	}

	[[nodiscard]] auto* data() { return info_.back().data(); }

	[[nodiscard]] auto const* data() const { return info_.back().data(); }

	[[nodiscard]] std::uint32_t size() const
	{
		return info_.back().empty() ? info_.back().size() : info_.back().size() - 1;
	}

	[[nodiscard]] static constexpr std::size_t numLevels() noexcept { return NumLevels; }

	void clear()
	{
		for (auto& v : info_) {
			v.clear();
		}
		info_leaf_.clear();
		for (auto& v : lut_) {
			v.clear();
		}
	}

	[[nodiscard]] constexpr Vec<Dim, float> const& point(std::uint32_t index) const noexcept
	{
		return info_leaf_[index].point();
	}

	[[nodiscard]] constexpr Vec<Dim, float> const& point(std::uint32_t index,
	                                                     unsigned      depth) const noexcept
	{
		return 0u == depth ? info_leaf_[index].point() : info_[depth - 1u][index].point();
	}

	[[nodiscard]] constexpr float distance(std::uint32_t index) const noexcept
	{
		return info_leaf_[index].distance();
	}

	[[nodiscard]] constexpr float& minDistance(std::uint32_t index, unsigned depth) noexcept
	{
		return info_[depth - 1u][index].minDistance();
	}

	[[nodiscard]] constexpr float minDistance(std::uint32_t index,
	                                          unsigned      depth) const noexcept
	{
		return info_[depth - 1u][index].minDistance();
	}

	[[nodiscard]] constexpr float& maxDistance(std::uint32_t index, unsigned depth) noexcept
	{
		return info_[depth - 1u][index].maxDistance();
	}

	[[nodiscard]] constexpr float maxDistance(std::uint32_t index,
	                                          unsigned      depth) const noexcept
	{
		return info_[depth - 1u][index].maxDistance();
	}

	[[nodiscard]] constexpr std::uint32_t firstChild(std::uint32_t index,
	                                                 unsigned      depth) const
	{
		return info_[depth - 1u][index].firstChild();
	}

	[[nodiscard]] constexpr std::uint32_t lastChild(std::uint32_t index,
	                                                unsigned      depth) const
	{
		return firstChild(index + 1, depth);
	}

	[[nodiscard]] std::uint32_t infoSize(unsigned depth) const
	{
		return 0u == depth ? info_leaf_.size() : info_[depth - 1u].size();
	}

	[[nodiscard]] constexpr std::uint32_t& firstLut(std::uint32_t index, unsigned depth)
	{
		return 0u == depth ? info_leaf_[index].firstLut()
		                   : info_[depth - 1u][index].firstLut();
	}

	[[nodiscard]] constexpr std::uint32_t firstLut(std::uint32_t index,
	                                               unsigned      depth) const
	{
		return 0u == depth ? info_leaf_[index].firstLut()
		                   : info_[depth - 1u][index].firstLut();
	}

	[[nodiscard]] constexpr std::uint32_t lastLut(std::uint32_t index, unsigned depth) const
	{
		return firstLut(index + 1, depth);
	}

	[[nodiscard]] std::vector<std::uint32_t>::iterator beginLut(std::uint32_t index,
	                                                            unsigned      depth)
	{
		return lut_[depth].begin() + firstLut(index, depth);
	}

	[[nodiscard]] std::vector<std::uint32_t>::const_iterator beginLut(std::uint32_t index,
	                                                                  unsigned depth) const
	{
		return lut_[depth].begin() + firstLut(index, depth);
	}

	[[nodiscard]] std::vector<std::uint32_t>::const_iterator endLut(std::uint32_t index,
	                                                                unsigned depth) const
	{
		return lut_[depth].begin() + lastLut(index, depth);
	}

	[[nodiscard]] std::vector<std::uint32_t>::iterator endLut(std::uint32_t index,
	                                                          unsigned      depth)
	{
		return lut_[depth].begin() + lastLut(index, depth);
	}

	[[nodiscard]] std::uint32_t lutSize(unsigned depth) const { return lut_[depth].size(); }

	[[nodiscard]] std::uint32_t lutSize(std::uint32_t index, unsigned depth) const
	{
		return lastLut(index, depth) - firstLut(index, depth);
	}

	template <class InputIt>
	void lutInsert(unsigned depth, InputIt first, InputIt last)
	{
		lut_[depth].insert(lut_[depth].end(), first, last);
	}

	template <class Range>
	void lutInsert(unsigned depth, Range const& range)
	{
		using std::begin;
		using std::end;
		lutInsert(depth, begin(range), end(range));
	}

	[[nodiscard]] constexpr std::uint32_t& count(std::uint32_t index,
	                                             unsigned      depth) noexcept
	{
		return info_[depth - 1u][index].count();
	}

	[[nodiscard]] constexpr std::uint32_t count(std::uint32_t index,
	                                            unsigned      depth) const noexcept
	{
		return info_[depth - 1u][index].count();
	}

	void shrinkToFit()
	{
		for (auto& v : info_) {
			v.shrink_to_fit();
		}
		info_leaf_.shrink_to_fit();
		for (auto& v : lut_) {
			v.shrink_to_fit();
		}
	}

	void reserveInfo(std::size_t size, unsigned depth)
	{
		if (0u == depth) {
			info_leaf_.reserve(size);
		} else {
			info_[depth - 1u].reserve(size);
		}
	}

	void reserveLut(std::size_t size, unsigned depth) { lut_[depth].reserve(size); }

	void resizeInfo(std::size_t size, unsigned depth)
	{
		if (0u == depth) {
			info_leaf_.resize(size);
		} else {
			info_[depth - 1u].resize(size);
		}
	}

	void resizeLut(std::size_t size, unsigned depth) { lut_[depth].resize(size); }

	void printMemoryUsage() const
	{
		// FIXME: Make nicer and calculate digits instead of having it hardcoded
		std::printf("Depth    Info        LUT      Info (MiB)   LUT (MiB)  Total (MiB)\n");
		double total_mib{};
		for (unsigned depth{}; NumLevels > depth;) {
			unsigned info_nodes = infoSize(depth);
			unsigned lut_nodes  = lutSize(depth);
			double   info_size  = 0u == depth ? sizeof(detail::InverseInfoLeaf<Dim>)
			                                  : sizeof(detail::InverseInfo<Dim>);
			info_size *= info_nodes / (1024.0 * 1024.0);
			double lut_size   = lut_nodes * sizeof(unsigned) / (1024.0 * 1024.0);
			double total_size = info_size + lut_size;
			total_mib += total_size;
			std::printf("  %u   %9u  %9u  %10.2f  %10.2f  %10.2f\n", depth++, info_nodes,
			            lut_nodes, info_size, lut_size, total_size);
		}
		std::printf("Total memory %f MiB\n", total_mib);
	}

 private:
	std::array<std::vector<InverseInfo<Dim>>, NumLevels - 1> info_;
	std::vector<InverseInfoLeaf<Dim>>                        info_leaf_;

	std::array<std::vector<std::uint32_t>, NumLevels> lut_;
};
}  // namespace ufo::detail

#endif  // UFO_MAP_INTEGRATOR_DETAIL_INVERSE_INFO_AND_LUT_HPP