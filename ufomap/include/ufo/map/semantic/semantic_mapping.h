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

#ifndef UFO_MAP_SEMANTIC_LABEL_MAPPING_H
#define UFO_MAP_SEMANTIC_LABEL_MAPPING_H

// UFO
#include <ufo/container/range.h>
#include <ufo/map/color/color.h>
#include <ufo/map/semantic/semantic.h>

// STL
#include <algorithm>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace ufo::map
{
class SemanticLabelMapping
{
 public:
	void add(std::string const& name, label_t label)
	{
		mapping_[name].ranges.insert(label);
	}

	void add(std::string const& name, container::Range<label_t> labels)
	{
		mapping_[name].ranges.insert(labels);
	}

	void add(std::string const& name, container::RangeSet<label_t> const& labels)
	{
		// TODO: Correct?
		mapping_[name].ranges.insert(std::begin(labels), std::end(labels));
	}

	void add(std::string const& name, std::string const& other_name)
	{
		mapping_[name].strings.insert(other_name);
	}

	void setColor(std::string const& name, Color color) { mapping_[name].color = color; }

	void clear() { mapping_.clear(); }

	void clear(std::string const& name) { mapping_[name].clear(); }

	void clearLabel()
	{
		std::for_each(std::begin(mapping_), std::end(mapping_),
		              [](auto const& m) { m.ranges.clear(); });
	}

	void clearLabel(std::string const& name) { mapping_[name].ranges.clear(); }

	void clearString()
	{
		std::for_each(std::begin(mapping_), std::end(mapping_),
		              [](auto const& m) { m.strings.clear(); });
	}

	void clearString(std::string const& name) { mapping_[name].strings.clear(); }

	void clearColor()
	{
		std::for_each(std::begin(mapping_), std::end(mapping_),
		              [](auto const& m) { m.color.clear(); });
	}

	void clearColor(std::string const& name) { mapping_[name].color.clear(); }

	void erase(std::string const& name, label_t label)
	{
		mapping_[name].ranges.erase(label);
	}

	void erase(std::string const& name, container::Range<label_t> labels)
	{
		mapping_[name].ranges.erase(labels);
	}

	void erase(std::string const& name, container::RangeSet<label_t> labels)
	{
		for (auto const& label : labels) {
			mapping_[name].ranges.erase(label);
		}
	}

	void erase(std::string const& name, std::string const& other_name)
	{
		mapping_[name].strings.erase(other_name);
	}

	container::RangeSet<label_t> getRanges(std::string const& name) const
	{
		if (0 == mapping_.count(name)) {
			return container::RangeSet<label_t>();
		}

		std::unordered_set<std::string> names;
		names.insert(name);
		std::queue<std::string> queue;
		queue.push(name);
		while (!queue.empty()) {
			std::string cur = queue.front();
			queue.pop();
			auto it = mapping_.find(cur);
			if (mapping_.end() == it) {
				continue;
			}
			for (auto const& str : it->second.strings) {
				if (auto it = mapping_.find(str); mapping_.end() != it) {
					if (names.insert(str).second) {
						queue.push(str);
					}
				}
			}
		}

		container::RangeSet<label_t> labels;
		for (auto const& str : names) {
			if (auto it = mapping_.find(str); mapping_.end() != it) {
				labels.insert(std::cbegin(it->second.ranges), std::cend(it->second.ranges));
			}
		}
		return labels;
	}

	std::unordered_set<std::string> getStrings(std::string const& name) const
	{
		if (auto it = mapping_.find(name); mapping_.end() != it) {
			return it->second.strings;
		}
		return std::unordered_set<std::string>();
	}

	Color getColor(std::string const& name) const
	{
		if (auto it = mapping_.find(name); mapping_.end() != it) {
			return it->second.color;
		}
		return Color();
	}

	std::ostream& writeData(std::ostream& out_stream) const
	{
		// Total
		uint64_t size = mapping_.size();
		out_stream.write(reinterpret_cast<char*>(&size), sizeof(uint64_t));

		for (auto const& [str, data] : mapping_) {
			// String label
			uint64_t length = str.length();
			out_stream.write(reinterpret_cast<char*>(&length), sizeof(uint64_t));
			out_stream.write(str.data(), length);

			data.writeData(out_stream);
		}
	}

	std::istream& readData(std::istream& in_stream)
	{
		// Total
		uint64_t size;
		in_stream.read(reinterpret_cast<char*>(&size), sizeof(uint64_t));
		if (0 == size) {
			if (!producer_mapping_.empty()) {
				changed_ = true;
				producer_mapping_.clear();
				mapping_ = consumer_mapping_;
			}
			return in_stream;
		}

		decltype(producer_mapping_) new_producer_mapping;

		std::vector<char> tmp;
		for (uint64_t i = 0; i != size; ++i) {
			// String label
			uint64_t length;
			in_stream.read(reinterpret_cast<char*>(&length), sizeof(uint64_t));
			tmp.resize(length);
			in_stream.read(tmp.data(), length);
			std::string str;
			str.assign(tmp.data(), length);

			auto& data = new_producer_mapping[std::move(str)];
			data.readData(in_stream);
		}

		if (producer_mapping_ == new_producer_mapping) {
			return in_stream;
		}

		producer_mapping_.swap(new_producer_mapping);

		decltype(mapping_) new_mapping_ = producer_mapping_;

		for (auto const& [str, data] : consumer_mapping_) {
			auto& cur_data = new_mapping_[str];

			// Ranges
			cur_data.ranges.insert(std::begin(data.ranges), std::end(data.ranges));

			// Strings
			cur_data.strings.insert(std::begin(data.strings), std::end(data.strings));

			// Color
			if (data.color.isSet()) {
				cur_data.color = data.color;
			}
		}

		if (mapping_ == new_mapping_) {
			return in_stream;
		}

		mapping_.swap(new_mapping_);

		changed_ = true;
		return in_stream;
	}

	constexpr uint64_t getSerializedSize() const { return serialized_size_; }

	friend std::ostream& operator<<(std::ostream& os, SemanticLabelMapping const& mapping)
	{
		for (auto const& [str, data] : mapping.mapping_) {
			os << str << ': ' << data.ranges << '\n';
			if (!data.strings.empty()) {
				std::copy(std::cbegin(data.strings), std::prev(std::cend(data.strings)),
				          std::ostream_iterator<std::string>(os, ", "));
				os << *std::prev(std::end(data.strings));
			}
		}
	}

 private:
	struct Data {
		void clear()
		{
			ranges.clear();
			strings.clear();
			color.clear();
		}

		std::ostream& writeData(std::ostream& out_stream) const
		{
			// Ranges
			ranges.writeData(out_stream);

			// Strings
			uint64_t size = strings.size();
			out_stream.write(reinterpret_cast<char*>(&size), sizeof(uint64_t));
			for (auto const& str : strings) {
				uint64_t length = str.length();
				out_stream.write(reinterpret_cast<char*>(&length), sizeof(uint64_t));
				out_stream.write(str.data(), length);
			}

			// Color
			out_stream.write(reinterpret_cast<char const*>(&color), sizeof(Color));
		}

		std::istream& readData(std::istream& in_stream)
		{
			// Ranges
			ranges.readData(in_stream);

			// Strings
			std::vector<char> tmp;
			uint64_t size;
			in_stream.read(reinterpret_cast<char*>(&size), sizeof(uint64_t));
			for (uint64_t i = 0; i != size; ++i) {
				uint64_t length;
				in_stream.read(reinterpret_cast<char*>(&length), sizeof(uint64_t));
				tmp.resize(length);
				in_stream.read(tmp.data(), length);
				std::string str;
				str.assign(tmp.data(), length);
				strings.insert(std::move(str));
			}

			// Color
			in_stream.read(reinterpret_cast<char*>(color), sizeof(Color));
		}

		container::RangeSet<label_t> ranges;
		std::unordered_set<std::string> strings;
		Color color;
	};

	// Combined mapping
	std::unordered_map<std::string, Data> mapping_;

	// Updated by consumer
	std::unordered_map<std::string, Data> consumer_mapping_;
	// Updated by producer
	std::unordered_map<std::string, Data> producer_mapping_;

	// Size taken
	uint64_t serialized_size_ = 0;

	// Change detection
	bool changed_ = false;
};
}  // namespace ufo::map

#endif  // UFO_MAP_SEMANTIC_LABEL_MAPPING_H