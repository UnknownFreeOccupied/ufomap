/*
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * License: BSD 3
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

#ifndef UFO_UTIL_TIMING_H
#define UFO_UTIL_TIMING_H

// STL
#include <chrono>
#include <limits>
#include <map>
#include <queue>
#include <string>
#include <vector>
#include <cmath>

namespace ufo::util
{
class TimerObject
{
 private:
	using Duration = std::chrono::duration<double, std::nano>;

 public:
	void start() { time_.push(std::chrono::high_resolution_clock::now()); }

	void stop()
	{
		std::chrono::time_point<std::chrono::high_resolution_clock, Duration> now =
		    std::chrono::high_resolution_clock::now();
		last_ = Duration(now - time_.front()).count();
		time_.pop();

		++samples_;

		double delta = last_ - mean_time_;
		mean_time_ += delta / samples_;
		double delta_2 = last_ - mean_time_;
		variance_time_ += delta * delta_2;

		total_time_ += last_;
		min_time_ = std::min(min_time_, last_);
		max_time_ = std::max(max_time_, last_);
	}

	bool isTiming() const { return !time_.empty(); }

	std::optional<double> getCurrentTiming() const
	{
		if (!isTiming()) {
			return std::nullopt;
		}
		return Duration(std::chrono::high_resolution_clock::now() - time_.front()).count();
	}

	double getLast() const { return last_; }

	std::size_t getNumSamples() const { return samples_; }

	double getTotal() const { return total_time_; }

	std::optional<double> getMin() const
	{
		if (0 == samples_) {
			return std::nullopt;
		}
		return min_time_;
	}

	std::optional<double> getMax() const
	{
		if (0 == samples_) {
			return std::nullopt;
		}
		return max_time_;
	}

	double getMean() const { return mean_time_; }

	std::optional<double> getVariance() const { return getSampleVariance(); }

	std::optional<double> getSampleVariance() const
	{
		if (2 > samples_) {
			return std::nullopt;
		}
		return variance_time_ / (samples_ - 1);
	}

	std::optional<double> getPopulationVariance() const
	{
		if (2 > samples_) {
			return std::nullopt;
		}
		return variance_time_ / (samples_);
	}

 private:
	std::queue<std::chrono::time_point<std::chrono::high_resolution_clock, Duration>> time_;

	std::size_t samples_ = 0;
	double last_;
	double total_time_ = 0.0;
	double mean_time_ = 0.0;
	double variance_time_ = 0.0;
	double min_time_ = std::numeric_limits<double>::max();
	double max_time_ = std::numeric_limits<double>::lowest();

	// std::chrono::duration<double> last_;
	// std::chrono::duration<double> total_time_ = std::chrono::duration<double>::zero();
	// std::chrono::duration<double> mean_time_ = std::chrono::duration<double>::zero();
	// std::chrono::duration<double> variance_time_ = std::chrono::duration<double>::zero();
	// std::chrono::duration<double> min_time_ = std::chrono::duration<double>::max();
	// std::chrono::duration<double> max_time_ = std::chrono::duration<double>::min();
};

class Timing
{
 public:
	void start(std::string const& tag) { timers_[tag].start(); }

	void stop(std::string const& tag) { timers_.at(tag).stop(); }

	bool contains(std::string const& tag) const { return 0 < timers_.count(tag); }

	bool isTiming(std::string const& tag) const { return timers_.at(tag).isTiming(); }

	double getTotalWithCurrentSeconds(std::string const& tag) const
	{
		return getTotalSeconds(tag) + getCurrentTimingSeconds(tag);
	}

	double getTotalWithCurrentMilliseconds(std::string const& tag) const
	{
		return getTotalMilliseconds(tag) + getCurrentTimingMilliseconds(tag);
	}

	double getTotalWithCurrentMicroseconds(std::string const& tag) const
	{
		return getTotalMicroseconds(tag) + getCurrentTimingMicroseconds(tag);
	}

	double getCurrentTimingSeconds(std::string const& tag) const
	{
		auto ct = timers_.at(tag).getCurrentTiming();
		if (!ct) {
			return -1;
		}
		return *ct / 1000000000.0;
	}

	double getCurrentTimingMilliseconds(std::string const& tag) const
	{
		auto ct = timers_.at(tag).getCurrentTiming();
		if (!ct) {
			return -1;
		}
		return *ct / 1000000.0;
	}

	double getCurrentTimingMicroseconds(std::string const& tag) const
	{
		auto ct = timers_.at(tag).getCurrentTiming();
		if (!ct) {
			return -1;
		}
		return *ct / 1000.0;
	}

	double getLastSeconds(std::string const& tag) const
	{
		return timers_.at(tag).getLast() / 1000000000.0;
	}
	double getLastMilliseconds(std::string const& tag) const
	{
		return timers_.at(tag).getLast() / 1000000.0;
	}
	double getLastMicroseconds(std::string const& tag) const
	{
		return timers_.at(tag).getLast() / 1000.0;
	}

	std::size_t getNumSamples(std::string const& tag) const
	{
		return timers_.at(tag).getNumSamples();
	}

	double getTotalSeconds(std::string const& tag) const
	{
		return timers_.at(tag).getTotal() / 1000000000.0;
	}
	double getTotalMilliseconds(std::string const& tag) const
	{
		return timers_.at(tag).getTotal() / 1000000.0;
	}
	double getTotalMicroseconds(std::string const& tag) const
	{
		return timers_.at(tag).getTotal() / 1000.0;
	}

	double getMinSeconds(std::string const& tag) const
	{
		auto min = timers_.at(tag).getMin();
		if (!min) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		return *min / 1000000000.0;
	}
	double getMinMilliseconds(std::string const& tag) const
	{
		auto min = timers_.at(tag).getMin();
		if (!min) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		return *min / 1000000.0;
	}
	double getMinMicroseconds(std::string const& tag) const
	{
		auto min = timers_.at(tag).getMin();
		if (!min) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		return *min / 1000.0;
	}

	double getMaxSeconds(std::string const& tag) const
	{
		auto max = timers_.at(tag).getMax();
		if (!max) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		return *max / 1000000000.0;
	}
	double getMaxMilliseconds(std::string const& tag) const
	{
		auto max = timers_.at(tag).getMax();
		if (!max) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		return *max / 1000000.0;
	}
	double getMaxMicroseconds(std::string const& tag) const
	{
		auto max = timers_.at(tag).getMax();
		if (!max) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		return *max / 1000.0;
	}

	double getMeanSeconds(std::string const& tag) const
	{
		return timers_.at(tag).getMean() / 1000000000.0;
	}
	double getMeanMilliseconds(std::string const& tag) const
	{
		return timers_.at(tag).getMean() / 1000000.0;
	}
	double getMeanMicroseconds(std::string const& tag) const
	{
		return timers_.at(tag).getMean() / 1000.0;
	}

	double getStdSeconds(std::string const& tag) const
	{
		auto variance = timers_.at(tag).getVariance();
		if (!variance) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		return std::sqrt(*variance) / 1000000000.0;
	}
	double getStdMilliseconds(std::string const& tag) const
	{
		auto variance = timers_.at(tag).getVariance();
		if (!variance) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		return std::sqrt(*variance) / 1000000.0;
	}
	double getStdMicroseconds(std::string const& tag) const
	{
		auto variance = timers_.at(tag).getVariance();
		if (!variance) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		return std::sqrt(*variance) / 1000.0;
	}

	std::vector<std::string> getTags() const
	{
		std::vector<std::string> tags;
		tags.reserve(timers_.size());
		for (auto const& [tag, _] : timers_) {
			tags.emplace_back(tag);
		}
		return tags;
	}

 private:
	std::map<std::string, TimerObject, std::less<std::string>> timers_;
};
}  // namespace ufo::util

#endif  // UFO_UTIL_TIMING_H