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

#ifndef UFO_MAP_SEMANTIC_PROPAGATION_HPP
#define UFO_MAP_SEMANTIC_PROPAGATION_HPP

// UFO
#include <ufo/container/range.hpp>
#include <ufo/map/semantic/semantic.hpp>
#include <ufo/map/types.hpp>

// STL
#include <algorithm>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace ufo
{
class SemanticPropagation
{
 public:
	constexpr void setDefault(PropagationCriteria prop_criteria)
	{
		default_prop_criteria_ = prop_criteria;
	}

	void set(SemanticRange range, PropagationCriteria prop_criteria)
	{
		prop_criteria_.insert_or_assign(range, prop_criteria);
	}

	template <class InputIt>
	void set(InputIt first, InputIt last, PropagationCriteria prop_criteria)
	{
		std::for_each(first, last, [this, prop_criteria](auto r) { set(r, prop_criteria); });
	}

	void erase(SemanticRange range) { prop_criteria_.erase(range); }

	template <class InputIt>
	void erase(InputIt first, InputIt last)
	{
		std::for_each(first, last, [this](auto e) { erase(e); });
	}

	void clear() { prop_criteria_.clear(); }

	[[nodiscard]] bool empty() const { return prop_criteria_.empty(); }

	[[nodiscard]] constexpr PropagationCriteria defaultPropCriteria() const noexcept
	{
		return default_prop_criteria_;
	}

	[[nodiscard]] PropagationCriteria propCriteria(label_t label) const
	{
		auto it = prop_criteria_.find(label);
		return prop_criteria_.end() == it ? defaultPropCriteria() : it->second;
	}

 private:
	PropagationCriteria                    default_prop_criteria_;
	RangeMap<label_t, PropagationCriteria> prop_criteria_;
};
}  // namespace ufo

#endif  // UFO_MAP_SEMANTIC_PROPAGATION_HPP