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

#ifndef UFO_MAP_SEMANTIC_LABEL_PROPAGATION_H
#define UFO_MAP_SEMANTIC_LABEL_PROPAGATION_H

// UFO
#include <ufo/container/range.h>
#include <ufo/map/semantic/semantic.h>
#include <ufo/map/types.h>

// STL
#include <algorithm>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace ufo::map
{
class SemanticLabelPropagation
{
 public:
	void setDefault(PropagationCriteria prop_criteria)
	{
		default_prop_criteria_ = prop_criteria;
	}

	void set(semantic_label_range_t label_range, PropagationCriteria prop_criteria)
	{
		prop_criteria_.insert_or_assign(label_range, prop_criteria);
	}

	template <class InputIt>
	void set(InputIt first, InputIt last, PropagationCriteria prop_criteria)
	{
		while (first != last) {
			set(*first, prop_criteria);
			std::advance(first, 1);
		}
	}

	void erase(semantic_label_range_t label_range) { prop_criteria_.erase(label_range); }

	template <class InputIt>
	void erase(InputIt first, InputIt last)
	{
		while (first != last) {
			erase(*first);
			std::advance(first, 1);
		}
	}

	void clear() { prop_criteria_.clear(); }

	[[nodiscard]] bool empty() const { return prop_criteria_.empty(); }

	[[nodiscard]] constexpr PropagationCriteria getDefaultPropCriteria() const noexcept
	{
		return default_prop_criteria_;
	}

	[[nodiscard]] semantic_label_range_set_t const& getPropCriteria(
	    semantic_label_t label) const
	{
		auto it = prop_criteria_.find(label);
		return prop_criteria_.end() == it ? getDefaultPropCriteria() : it->second;
	}

 private:
	PropagationCriteria default_prop_criteria_;
	container::RangeMap<semantic_label_t, PropagationCriteria> prop_criteria_;
};
}  // namespace ufo::map

#endif  // UFO_MAP_SEMANTIC_LABEL_PROPAGATION_H