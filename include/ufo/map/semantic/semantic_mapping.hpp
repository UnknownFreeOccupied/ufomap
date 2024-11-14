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

#ifndef UFO_MAP_SEMANTIC_LABEL_MAPPING_HPP
#define UFO_MAP_SEMANTIC_LABEL_MAPPING_HPP

// UFO
#include <ios>
#include <ufo/container/range.hpp>
#include <ufo/map/color/color.hpp>
#include <ufo/map/semantic/semantic.hpp>

// STL
#include <algorithm>
#include <cassert>
#include <cstring>
#include <memory>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace ufo
{
class SemanticMapping
{
 public:
	void addLabel(std::string const& tag, label_t label)
	{
		addLabels(tag, SemanticRange(label));
	}

	void addLabels(std::string const& tag, SemanticRange labels)
	{
		addLabels(tag, SemanticRangeSet(labels));
	}

	void addLabels(std::string const& tag, SemanticRangeSet labels)
	{
		// change detection
		// only need to check mapping_ for changes since only this will be serialized
		// iterate over all ranges in the labels set that should be added
		for (auto l_insert = std::begin(labels); l_insert != std::end(labels); ++l_insert) {
			bool found_label = includesRange(mapping_[tag].ranges, *l_insert);

			// // iterate over all ranges in mapping and check im l_insert is included in one of
			// them auto const& l_mapping = mapping_[tag].ranges; for (auto r =
			// std::begin(l_mapping); r != std::end(l_mapping); ++r) { 	if
			// (r->contains(*l_insert)) { 		found_label = true; // found the current label
			// range in the mapping, so no change from inserting this 		break;
			// 	}
			// }

			if (!found_label) {
				// if we don't find a label of labels we know mapping will change
				changed_ = true;
				break;
			}
		}

		consumer_mapping_[tag].ranges.insert(std::begin(labels), std::end(labels));
		mapping_[tag].ranges.insert(std::begin(labels), std::end(labels));

		// remove from removed list
		removed_tags.erase(tag);

		for (auto iter = std::begin(labels); iter != std::end(labels); iter++) {
			removed_labels[tag].erase(*iter);
		}
	}

	void addLink(std::string const& tag, std::string const& link)
	{
		if (consumer_mapping_[tag].links.insert(link).second) {
			if (mapping_[tag].links.insert(link).second) {
				changed_ = true;
			}
		}

		// remove from removed list
		removed_links[tag].erase(link);
		removed_tags.erase(tag);
		removed_tags.erase(link);
	}

	template <class InputIt>
	void addLinks(std::string const& tag, InputIt first, InputIt last)
	{
		for (auto& iter = first; iter != last; iter++) {
			addLink(tag, *iter);
		}
	}

	void addLinks(std::string const& tag, std::initializer_list<std::string> links)
	{
		addLinks(tag, links.begin(), links.end());
	}

	void setSemanticColor(std::string const& tag, Color color)
	{
		if (0 == consumer_mapping_.count(tag) || consumer_mapping_[tag].color != color) {
			consumer_mapping_[tag].color = color;
			if (0 == mapping_.count(tag) || mapping_[tag].color != color) {
				mapping_[tag].color = color;
				changed_            = true;
			}
		}

		// remove from removed_list
		removed_colors.erase(tag);
		removed_tags.erase(tag);
	}

	////////////////////remove
	/// Labels//////////////////////////////////////////////////////////////

	// remove a range from a tag
	void removeLabel(std::string const& tag, SemanticRange const& labels)
	{
		// change detection
		if (includesRange(mapping_[tag].ranges, labels)) {
			changed_ = true;
		}

		consumer_mapping_[tag].ranges.erase(labels);
		mapping_[tag].ranges.erase(labels);

		removed_labels[tag].insert(labels);
	}

	// remove a set of ranges from a tag
	void removeLabel(std::string const& tag, SemanticRangeSet const& labels)
	{
		for (auto const& l : labels) {
			removeLabel(tag, l);
		}
	}

	// remove a label from a tag
	void removeLabel(std::string const& tag, label_t label)
	{
		removeLabel(tag, SemanticRange(label));
	}

	// remove all labels from tag
	// stores which labels got removed so that they don't reappear after a read from the
	// producer if producer has a new label that was not in the set when labels got cleared
	// it will appear in the mapping after a read from the producer.
	void clearLabels(std::string const& tag)
	{
		// change detection
		if (mapping_[tag].ranges.size() > 0) {
			changed_ = true;
		}

		for (auto const& l : mapping_[tag].ranges) {
			removed_labels[tag].insert(l);
		}

		consumer_mapping_[tag].ranges.clear();
		mapping_[tag].ranges.clear();
	}

	// remove all labels from all tags
	// stores which labels got removed so that they don't reappear after a read from the
	// producer if producer has a new label that was not in the set when labels got cleared
	// it will appear in the mapping after a read from the producer.
	void clearLabels()
	{
		for (auto const& [tag, data] : consumer_mapping_) {
			for (auto const& l : consumer_mapping_[tag].ranges) {
				removed_labels[tag].insert(l);
			}
			consumer_mapping_[tag].ranges.clear();
		}

		for (auto const& [tag, data] : mapping_) {
			// change detection
			if (mapping_[tag].ranges.size() > 0) {
				changed_ = true;
			}

			for (auto const& l : mapping_[tag].ranges) {
				removed_labels[tag].insert(l);
			}
			mapping_[tag].ranges.clear();
		}
	}

	///////////////////remove
	/// Links/////////////////////////////////////////////////////////////////////////////

	// removes a link from a tag
	void removeLink(std::string const& tag, std::string const& link)
	{
		consumer_mapping_[tag].links.erase(link);
		auto removed_count = mapping_[tag].links.erase(link);

		if (removed_count > 0) {
			changed_ = true;
		}

		removed_links[tag].insert(link);
	}

	// remove all links from tag
	void clearLinks(std::string const& tag)
	{
		// change detection
		if (mapping_[tag].links.size() > 0) {
			changed_ = true;
		}

		for (auto const& l : mapping_[tag].links) {
			removed_links[tag].insert(l);
		}

		consumer_mapping_[tag].links.clear();
		mapping_[tag].links.clear();
	}

	// remove all links from all tags
	void clearLinks()
	{
		for (auto const& [tag, data] : consumer_mapping_) {
			for (auto const& l : consumer_mapping_[tag].links) {
				removed_links[tag].insert(l);
			}
			consumer_mapping_[tag].links.clear();
		}

		for (auto const& [tag, data] : mapping_) {
			// change detection
			if (mapping_[tag].links.size() > 0) {
				changed_ = true;
			}

			for (auto const& l : mapping_[tag].links) {
				removed_links[tag].insert(l);
			}
			mapping_[tag].links.clear();
		}
	}

	// remove multiple links from tag
	template <class InputIt>
	void removeLink(std::string const& tag, InputIt first, InputIt last)
	{
		for (auto& iter = first; iter != last; iter++) {
			removeLink(tag, *iter);
		}
	}

	void removeLink(std::string const& tag, std::initializer_list<std::string> links)
	{
		removeLink(tag, links.begin(), links.end());
	}

	////////////remove color////////////////////////////////////////////////

	void clearSemanticColor(std::string const& tag)
	{
		// change detection
		if (!mapping_[tag].color.empty()) {
			changed_ = true;
		}

		consumer_mapping_[tag].color.clear();
		mapping_[tag].color.clear();

		removed_colors.insert(tag);
	}

	// remove color from all tags
	void clearSemanticColors()
	{
		for (auto const& [tag, data] : consumer_mapping_) {
			consumer_mapping_[tag].color.clear();
			removed_colors.insert(tag);
		}

		for (auto const& [tag, data] : mapping_) {
			// change detection
			if (!mapping_[tag].color.empty()) {
				changed_ = true;
			}
			mapping_[tag].color.clear();
			removed_colors.insert(tag);
		}
	}

	/////////////////////////////////////////////////////////////////////////

	// remove all ranges, links and color
	// removeLabels + removeLinks + removeColor
	void clearTag(std::string const& tag)
	{
		clearLabels(tag);
		clearLinks(tag);
		clearSemanticColor(tag);
	}

	// remove tag entirely: need to remove all links to this tag
	void removeTag(std::string const& tag)
	{
		if (mapping_.erase(tag) > 0) {
			changed_ = true;
		}

		consumer_mapping_.erase(tag);

		removed_tags.insert(tag);

		for (auto const& [t, data] : consumer_mapping_) {
			consumer_mapping_[t].links.erase(tag);
		}

		for (auto const& [t, data] : mapping_) {
			if (mapping_[t].links.erase(tag) > 0) {
				changed_ = true;
			}
		}
	}

	bool hasChanged() { return changed_; }

	SemanticRangeSet labels(std::string const& tag, bool recursive = true) const
	{
		if (!recursive) {
			if (auto it = mapping_.find(tag); mapping_.end() != it) {
				return it->second.ranges;
			}
			return SemanticRangeSet();
		}

		SemanticRangeSet ls;

		if (auto it = mapping_.find(tag); mapping_.end() != it) {
			ls.insert(std::cbegin(it->second.ranges), std::cend(it->second.ranges));
		}

		for (auto const& str : links(tag, true)) {
			if (auto it = mapping_.find(str); mapping_.end() != it) {
				ls.insert(std::cbegin(it->second.ranges), std::cend(it->second.ranges));
			}
		}
		return ls;
	}

	std::unordered_set<std::string> links(std::string const& tag,
	                                      bool               recursive = true) const
	{
		if (!recursive) {
			if (auto it = mapping_.find(tag); mapping_.end() != it) {
				return it->second.links;
			}
			return std::unordered_set<std::string>();
		}

		// assuming no cycles
		std::unordered_set<std::string> tags;
		std::queue<std::string>         queue;
		queue.push(tag);
		while (!queue.empty()) {
			std::string cur = queue.front();
			queue.pop();
			auto it = mapping_.find(cur);
			if (mapping_.end() == it) {
				continue;
			}
			for (auto const& str : it->second.links) {
				if (tags.insert(str).second) {
					queue.push(str);
				}
			}
		}

		return tags;
	}

	Color semanticColor(std::string const& tag) const
	{
		if (auto it = mapping_.find(tag); mapping_.end() != it) {
			return it->second.color;
		}
		return Color();
	}

	void readMapping(std::istream& in)
	{
		std::uint64_t size;
		// std::streamsize size;
		in.read(reinterpret_cast<char*>(&size), sizeof(size));
		std::vector<char> producer_serialized(size);
		in.read(producer_serialized.data(), static_cast<std::streamsize>(size));

		if (producer_serialized == producer_serialized_) {
			return;
		}

		producer_serialized_.swap(producer_serialized);

		producer_mapping_.clear();
		const char* first = producer_serialized_.data();
		const char* last  = first + producer_serialized_.size();
		std::string str;
		while (first != last) {
			std::memcpy(&size, first, sizeof(size));
			first += sizeof(size);
			str.resize(size);
			std::memcpy(str.data(), first, size);
			first += size;
			first = producer_mapping_[str].read(first);
		}

		mapping_ = producer_mapping_;

		// NOTE: Important that consumer is after producer so it overwrites
		for (auto const& [tag, data] : consumer_mapping_) {
			auto& mapping_data = mapping_[tag];

			mapping_data.ranges.insert(std::begin(data.ranges), std::end(data.ranges));
			mapping_data.links.insert(std::begin(data.links), std::end(data.links));
			if (!data.color.empty()) {
				mapping_data.color = data.color;
			}
		}

		// remove things that was removed by consumer
		// but might have reappeared from the producer

		// remove labels
		for (auto const& [tag, labels] : removed_labels) {
			for (auto const& l : labels) {
				mapping_[tag].ranges.erase(l);
			}
		}

		// remove links
		for (auto const& [tag, links] : removed_links) {
			for (auto const& link : links) {
				mapping_[tag].links.erase(link);
			}
		}

		// remove colors
		for (auto const& tag : removed_colors) {
			mapping_[tag].color.clear();
		}

		// remove tags
		for (auto const& tag : removed_tags) {
			mapping_.erase(tag);
			for (auto const& [tag, data] : mapping_) {
				mapping_[tag].links.erase(tag);
			}
		}

		// TODO: change detection
		changed_ = true;
	}

	void writeMapping(std::ostream& out)
	{
		if (changed_) {
			std::uint64_t size = 0;
			for (auto const& [tag, data] : mapping_) {
				size += sizeof(size) + tag.length() + data.serializedSize();
			}

			mapping_serialized_.resize(size);
			char* ptr = mapping_serialized_.data();
			for (auto const& [tag, data] : mapping_) {
				size = tag.length();
				std::memcpy(ptr, &size, sizeof(size));
				ptr += sizeof(size);
				std::memcpy(ptr, tag.data(), size);
				ptr += size;
				ptr = data.write(ptr);
			}
		}

		std::uint64_t size = mapping_serialized_.size();
		out.write(reinterpret_cast<char const*>(&size), sizeof(size));
		out.write(mapping_serialized_.data(),
		          static_cast<std::streamsize>(mapping_serialized_.size()));
		changed_ = false;
	}

	friend std::ostream& operator<<(std::ostream& os, SemanticMapping const& mapping)
	{
		for (auto const& [str, data] : mapping.mapping_) {
			os << str << ": " << data.ranges << '\n';
			if (!data.links.empty()) {
				// std::prev does not work on unordered_set iterator, iterator needs to be
				// bidirectional std::copy(std::cbegin(data.strings), std::cend(data.strings),
				//           std::ostream_iterator<std::string>(os, ", "));
				// os << *std::prev(std::end(data.strings));
				os << "{";
				for (auto iter = std::cbegin(data.links); iter != std::cend(data.links); ++iter) {
					if (iter == std::cbegin(data.links)) {
						os << "\"" << *iter << "\"";
					} else {
						os << ", \"" << *iter << "\"";
					}
				}
				os << "}" << '\n';
			}
			// os << data.color << '\n';
		}
		return os;
	}

 private:
	struct Data {
		SemanticRangeSet                ranges;
		std::unordered_set<std::string> links;
		Color                           color;

		void clear()
		{
			ranges.clear();
			links.clear();
			color.clear();
		}

		std::uint64_t serializedSize() const
		{
			std::uint64_t size = sizeof(color);
			size += sizeof(size) + ranges.size() * sizeof(SemanticRangeSet::value_type);
			size += sizeof(size);
			for (auto const& link : links) {
				size += sizeof(size) + link.length();
			}
			return size;
		}

		char const* read(char const* in)
		{
			// Ranges
			ranges.clear();
			std::uint64_t size;
			std::memcpy(&size, in, sizeof(size));
			in += sizeof(size);
			for (std::uint64_t i = 0; i != size; ++i) {
				SemanticRange r;
				std::memcpy(&r, in, sizeof(r));
				in += sizeof(r);
				ranges.insert(r);
			}

			// Strings
			links.clear();
			std::memcpy(&size, in, sizeof(size));
			in += sizeof(size);
			for (std::uint64_t i = 0; i != size; ++i) {
				std::uint64_t length;
				std::memcpy(&length, in, sizeof(length));
				in += sizeof(length);
				auto str = std::make_unique<char[]>(length);
				std::memcpy(str.get(), in, length);
				in += length;
				links.emplace(str.get(), length);
			}

			// Color
			std::memcpy(&color, in, sizeof(color));
			in += sizeof(color);

			return in;
		}

		char* write(char* out) const
		{
			// Ranges
			std::uint64_t size = ranges.size();
			std::memcpy(out, &size, sizeof(size));
			out += sizeof(size);
			for (SemanticRange r : ranges) {
				std::memcpy(out, &r, sizeof(r));
				out += sizeof(r);
			}

			// Strings
			size = links.size();
			std::memcpy(out, &size, sizeof(size));
			out += sizeof(size);
			for (auto const& link : links) {
				size = link.length();
				std::memcpy(out, &size, sizeof(size));
				out += sizeof(size);
				std::memcpy(out, link.data(), size);
				out += size;
			}

			// Color
			std::memcpy(out, &color, sizeof(color));
			out += sizeof(color);

			return out;
		}
	};

	// checks if range_set includes range in one of its ranges (also just partially)
	// range_set.contains only checks if the entire range is in the set
	// but for the change detection we need to know if range is included also just partially
	// in one of the sets in range_set
	bool includesRange(SemanticRangeSet const& range_set, SemanticRange const& range)
	{
		// iterate over all ranges in mapping and check im l_insert is included in one of them
		for (auto r = std::begin(range_set); r != std::end(range_set); ++r) {
			if (r->contains(range)) {
				return true;
			}
		}
		return false;
	}

 protected:
	// Updated by consumer
	std::unordered_map<std::string, Data> consumer_mapping_;
	// Updated by producer
	std::unordered_map<std::string, Data> producer_mapping_;
	// Combined mapping
	std::unordered_map<std::string, Data> mapping_;

	// Producer mapping serialized form
	std::vector<char> producer_serialized_;
	// Combined mapping serialized form
	std::vector<char> mapping_serialized_;

	// store removed stuff to incorporate again after a read
	std::unordered_map<std::string, SemanticRangeSet>                removed_labels;
	std::unordered_map<std::string, std::unordered_set<std::string>> removed_links;
	std::unordered_set<std::string>                                  removed_colors;
	std::unordered_set<std::string>                                  removed_tags;

	// Change detection
	bool changed_ = false;
};
}  // namespace ufo

#endif  // UFO_MAP_SEMANTIC_LABEL_MAPPING_H