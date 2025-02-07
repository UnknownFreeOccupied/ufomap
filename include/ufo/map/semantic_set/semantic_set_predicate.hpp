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

#ifndef UFO_MAP_SEMANTIC_SET_PREDICATE_HPP
#define UFO_MAP_SEMANTIC_SET_PREDICATE_HPP

// UFO
#include <ufo/map/node.hpp>
#include <ufo/map/predicate/predicate.hpp>
// #include <ufo/map/semantic/semantic.hpp>
#include <ufo/map/semantic_set/semantic_set_map.hpp>
#include <ufo/map/types.hpp>

namespace ufo::pred
{
template <bool Negated = false>
struct SemanticEmpty {
};

//
// SemanticSetMap
//

struct SemanticSetMap {
};

template <>
struct ValueCheck<SemanticSetMap> {
	using Pred = SemanticSetMap;

	template <class Map, class Node>
	static constexpr bool apply(Pred, Map const&, Node)
	{
		return is_semantic_set_map_v<Map>;
	}
};

template <class PredPost>
struct ValueCheck<Then<SemanticSetMap, PredPost>> {
	using Pred = Then<SemanticSetMap, PredPost>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (ValueCheck<SemanticSetMap>::apply(p.pre, m, n)) {
			return ValueCheck<PredPost>::apply(p.post, m, n);
		} else {
			return true;
		}
	}
};

template <>
struct InnerCheck<SemanticSetMap> {
	using Pred = SemanticSetMap;

	template <class Map, class Node>
	static constexpr bool apply(Pred, Map const&, Node)
	{
		return is_semantic_set_map_v<Map>;
	}
};

template <class PredPost>
struct InnerCheck<Then<SemanticSetMap, PredPost>> {
	using Pred = Then<SemanticSetMap, PredPost>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (InnerCheck<SemanticSetMap>::apply(p.pre, m, n)) {
			return InnerCheck<PredPost>::apply(p.post, m, n);
		} else {
			return true;
		}
	}
};

//
// Semantic
//

template <bool Negated = false>
struct SemanticSetLabel {
	constexpr SemanticSetLabel(label_t label) : label(label) {}

	label_t label;
};

template <bool Negated>
constexpr SemanticSetLabel<!Negated> operator!(SemanticSetLabel<Negated> p)
{
	return SemanticSetLabel<!Negated>(p.label);
}

template <bool Negated>
struct ValueCheck<SemanticSetLabel<Negated>> {
	using Pred = SemanticSetLabel<Negated>;

	template <class Map, class Node>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		if constexpr (Negated) {
			// return p.label != (m.semantic(n.index()).label & p.label);
			// return !m.semantics(n.index()).contains(p.label);
			return !m.containsSemantics(n.index(), p.label);
		} else {
			// return p.label == (m.semantic(n.index()).label & p.label);
			// return m.semantics(n.index()).contains(p.label);
			return m.containsSemantics(n.index(), p.label);
		}
	}
};

template <bool Negated>
struct InnerCheck<SemanticSetLabel<Negated>> {
	using Pred = SemanticSetLabel<Negated>;

	template <class Map, class Node>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		if constexpr (Negated) {
			return true;  // FIXME: Can this be better?
		} else {
			switch (m.semanticSetPropagationCriteria()) {
				case ufo::impl::SemanticSetPropagationCriteria::MIN:
				case ufo::impl::SemanticSetPropagationCriteria::MAX:
				case ufo::impl::SemanticSetPropagationCriteria::NONE:
					return m.semantics(n.index()).contains(p.label);

				case ufo::impl::SemanticSetPropagationCriteria::S_MIN:
				case ufo::impl::SemanticSetPropagationCriteria::S_MAX:
					return p.label == (m.semanticSetSummary(n.index()).label & p.label);
			}
			return true;
		}
	}
};

//
// Any
//

// Tag

template <bool Negated = false>
struct AnySemanticTag {
	constexpr AnySemanticTag(std::initializer_list<std::string> tag) : tags(tag) {}

	std::vector<std::string> tags;
};

template <bool Negated>
constexpr AnySemanticTag<!Negated> operator!(AnySemanticTag<Negated> p)
{
	return AnySemanticTag<!Negated>(p.tags);
}

template <bool Negated>
struct ValueCheck<AnySemanticTag<Negated>> {
	using Pred = AnySemanticTag<Negated>;

	template <class Map, class Node>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		if constexpr (Negated) {
			return !m.anySemantics(n.index(), p.tags.begin(), p.tags.end());
		} else {
			return m.anySemantics(n.index(), p.tags.begin(), p.tags.end());
		}
	}
};

template <bool Negated>
struct InnerCheck<AnySemanticTag<Negated>> {
	using Pred = AnySemanticTag<Negated>;

	template <class Map, class Node>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		if constexpr (Negated) {
			return true;  // FIXME: Can this be better?
		} else {
			switch (m.semanticSetPropagationCriteria()) {
				case ufo::impl::SemanticSetPropagationCriteria::MIN:
				case ufo::impl::SemanticSetPropagationCriteria::MAX:
				case ufo::impl::SemanticSetPropagationCriteria::NONE:
					return m.anySemantics(n.index(), p.tags.begin(), p.tags.end());

				case ufo::impl::SemanticSetPropagationCriteria::S_MIN:
				case ufo::impl::SemanticSetPropagationCriteria::S_MAX:
					auto summary_label = m.semanticSetSummary(n.index()).label;
					for (auto tag : p.tags) {
						auto labelset = m.labels(tag);  // rangeset
						for (auto labels : labelset) {  // range
							for (auto l = labels.lower(); l <= labels.upper(); ++l) {
								if (l == (summary_label & l)) {
									return true;
								}
							}
						}
					}
					return false;
			}
			return true;
		}
	}
};

// Label

template <bool Negated = false>
struct AnySemanticLabel {
	constexpr AnySemanticLabel(std::initializer_list<label_t> label) : labels(label) {}

	std::vector<label_t> labels;
};

template <bool Negated>
constexpr AnySemanticLabel<!Negated> operator!(AnySemanticLabel<Negated> p)
{
	return AnySemanticLabel<!Negated>(p.labels);
}

template <bool Negated>
struct ValueCheck<AnySemanticLabel<Negated>> {
	using Pred = AnySemanticLabel<Negated>;

	template <class Map, class Node>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		if constexpr (Negated) {
			return !m.anySemantics(n.index(),
			                       SemanticRangeSet(p.labels.begin(), p.labels.end()));
		} else {
			return m.anySemantics(n.index(),
			                      SemanticRangeSet(p.labels.begin(), p.labels.end()));
		}
	}
};

template <bool Negated>
struct InnerCheck<AnySemanticLabel<Negated>> {
	using Pred = AnySemanticLabel<Negated>;

	template <class Map, class Node>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		if constexpr (Negated) {
			return true;  // FIXME: Can this be better?
		} else {
			switch (m.semanticSetPropagationCriteria()) {
				case ufo::impl::SemanticSetPropagationCriteria::MIN:
				case ufo::impl::SemanticSetPropagationCriteria::MAX:
				case ufo::impl::SemanticSetPropagationCriteria::NONE:
					return m.anySemantics(n.index(),
					                      SemanticRangeSet(p.labels.begin(), p.labels.end()));

				case ufo::impl::SemanticSetPropagationCriteria::S_MIN:
				case ufo::impl::SemanticSetPropagationCriteria::S_MAX:
					auto summary_label = m.semanticSetSummary(n.index()).label;
					for (auto l : p.labels) {  // range
						if (l == (summary_label & l)) {
							return true;
						}
					}
					return false;
			}
			return true;
		}
	}
};

//
// All
//

// Tag

template <bool Negated = false>
struct AllSemanticTag {
	constexpr AllSemanticTag(std::initializer_list<std::string> tag) : tags(tag) {}

	std::vector<std::string> tags;
};

template <bool Negated>
constexpr AllSemanticTag<!Negated> operator!(AllSemanticTag<Negated> p)
{
	return AllSemanticTag<!Negated>(p.tags);
}

template <bool Negated>
struct ValueCheck<AllSemanticTag<Negated>> {
	using Pred = AllSemanticTag<Negated>;

	template <class Map, class Node>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		if constexpr (Negated) {
			return !m.allSemantics(n.index(), p.tags.begin(), p.tags.end());
		} else {
			return m.allSemantics(n.index(), p.tags.begin(), p.tags.end());
		}
	}
};

template <bool Negated>
struct InnerCheck<AllSemanticTag<Negated>> {
	using Pred = AllSemanticTag<Negated>;

	template <class Map, class Node>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		if constexpr (Negated) {
			return true;  // FIXME: Can this be better?
		} else {
			switch (m.semanticSetPropagationCriteria()) {
				case ufo::impl::SemanticSetPropagationCriteria::MIN:
				case ufo::impl::SemanticSetPropagationCriteria::MAX:
				case ufo::impl::SemanticSetPropagationCriteria::NONE:
					return m.allSemantics(n.index(), p.tags.begin(), p.tags.end());

				case ufo::impl::SemanticSetPropagationCriteria::S_MIN:
				case ufo::impl::SemanticSetPropagationCriteria::S_MAX:
					auto summary_label = m.semanticSetSummary(n.index()).label;
					for (auto tag : p.tags) {
						auto labelset = m.labels(tag);  // rangeset
						for (auto labels : labelset) {  // range
							for (auto l = labels.lower(); l <= labels.upper(); ++l) {
								if (l != (summary_label & l)) {
									return false;
								}
							}
						}
					}
					return true;
			}
			return true;
		}
	}
};

// Label

template <bool Negated = false>
struct AllSemanticLabel {
	constexpr AllSemanticLabel(std::initializer_list<label_t> label) : labels(label) {}

	std::vector<label_t> labels;
};

template <bool Negated>
constexpr AllSemanticLabel<!Negated> operator!(AllSemanticLabel<Negated> p)
{
	return AllSemanticLabel<!Negated>(p.labels);
}

template <bool Negated>
struct ValueCheck<AllSemanticLabel<Negated>> {
	using Pred = AllSemanticLabel<Negated>;

	template <class Map, class Node>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		if constexpr (Negated) {
			return !m.allSemantics(n.index(),
			                       SemanticRangeSet(p.labels.begin(), p.labels.end()));
		} else {
			return m.allSemantics(n.index(),
			                      SemanticRangeSet(p.labels.begin(), p.labels.end()));
		}
	}
};

template <bool Negated>
struct InnerCheck<AllSemanticLabel<Negated>> {
	using Pred = AllSemanticLabel<Negated>;

	template <class Map, class Node>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		if constexpr (Negated) {
			return true;  // FIXME: Can this be better?
		} else {
			switch (m.semanticSetPropagationCriteria()) {
				case ufo::impl::SemanticSetPropagationCriteria::MIN:
				case ufo::impl::SemanticSetPropagationCriteria::MAX:
				case ufo::impl::SemanticSetPropagationCriteria::NONE:
					return m.allSemantics(n.index(),
					                      SemanticRangeSet(p.labels.begin(), p.labels.end()));

				case ufo::impl::SemanticSetPropagationCriteria::S_MIN:
				case ufo::impl::SemanticSetPropagationCriteria::S_MAX:
					auto summary_label = m.semanticSetSummary(n.index()).label;
					for (auto l : p.labels) {  // range
						if (l != (summary_label & l)) {
							return false;
						}
					}
					return true;
			}
			return true;
		}
	}
};

//
// None
//

// Tag

template <bool Negated = false>
struct NoneSemanticTag {
	constexpr NoneSemanticTag(std::initializer_list<std::string> tag) : tags(tag) {}

	std::vector<std::string> tags;
};

template <bool Negated>
constexpr NoneSemanticTag<!Negated> operator!(NoneSemanticTag<Negated> p)
{
	return NoneSemanticTag<!Negated>(p.tags);
}

template <bool Negated>
struct ValueCheck<NoneSemanticTag<Negated>> {
	using Pred = NoneSemanticTag<Negated>;

	template <class Map, class Node>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		if constexpr (Negated) {
			return !m.noneSemantics(n.index(), p.tags.begin(), p.tags.end());
		} else {
			return m.noneSemantics(n.index(), p.tags.begin(), p.tags.end());
		}
	}
};

template <bool Negated>
struct InnerCheck<NoneSemanticTag<Negated>> {
	using Pred = NoneSemanticTag<Negated>;

	template <class Map, class Node>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		if constexpr (Negated) {
			return true;  // FIXME: Can this be better?
		} else {
			switch (m.semanticSetPropagationCriteria()) {
				case ufo::impl::SemanticSetPropagationCriteria::MIN:
				case ufo::impl::SemanticSetPropagationCriteria::MAX:
				case ufo::impl::SemanticSetPropagationCriteria::NONE:
					return m.noneSemantics(n.index(), p.tags.begin(), p.tags.end());

				case ufo::impl::SemanticSetPropagationCriteria::S_MIN:
				case ufo::impl::SemanticSetPropagationCriteria::S_MAX: return true;
			}
			return true;
		}
	}
};

// Label

template <bool Negated = false>
struct NoneSemanticLabel {
	constexpr NoneSemanticLabel(std::initializer_list<label_t> label) : labels(label) {}

	std::vector<label_t> labels;
};

template <bool Negated>
constexpr NoneSemanticLabel<!Negated> operator!(NoneSemanticLabel<Negated> p)
{
	return NoneSemanticLabel<!Negated>(p.labels);
}

template <bool Negated>
struct ValueCheck<NoneSemanticLabel<Negated>> {
	using Pred = NoneSemanticLabel<Negated>;

	template <class Map, class Node>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		if constexpr (Negated) {
			return !m.noneSemantics(n.index(),
			                        SemanticRangeSet(p.labels.begin(), p.labels.end()));
		} else {
			return m.noneSemantics(n.index(),
			                       SemanticRangeSet(p.labels.begin(), p.labels.end()));
		}
	}
};

template <bool Negated>
struct InnerCheck<NoneSemanticLabel<Negated>> {
	using Pred = NoneSemanticLabel<Negated>;

	template <class Map, class Node>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		if constexpr (Negated) {
			return true;  // FIXME: Can this be better?
		} else {
			switch (m.semanticSetPropagationCriteria()) {
				case ufo::impl::SemanticSetPropagationCriteria::MIN:
				case ufo::impl::SemanticSetPropagationCriteria::MAX:
				case ufo::impl::SemanticSetPropagationCriteria::NONE:
					return m.noneSemantics(n.index(),
					                       SemanticRangeSet(p.labels.begin(), p.labels.end()));

				case ufo::impl::SemanticSetPropagationCriteria::S_MIN:
				case ufo::impl::SemanticSetPropagationCriteria::S_MAX: return true;
			}
			return true;
		}
	}
};

}  // namespace ufo::pred

#endif  // UFO_MAP_SEMANTIC_SET_PREDICATE_HPP