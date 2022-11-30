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

#ifndef UFO_UTIL_TIMING_H
#define UFO_UTIL_TIMING_H

// STL
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <string>
#include <vector>

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

	std::optional<double> currentTiming() const
	{
		if (!isTiming()) {
			return std::nullopt;
		}
		return Duration(std::chrono::high_resolution_clock::now() - time_.front()).count();
	}

	double last() const { return last_; }

	std::size_t numSamples() const { return samples_; }

	double total() const { return total_time_; }

	std::optional<double> min() const
	{
		if (0 == samples_) {
			return std::nullopt;
		}
		return min_time_;
	}

	std::optional<double> max() const
	{
		if (0 == samples_) {
			return std::nullopt;
		}
		return max_time_;
	}

	double mean() const { return mean_time_; }

	std::optional<double> variance() const { return sampleVariance(); }

	std::optional<double> sampleVariance() const
	{
		if (2 > samples_) {
			return std::nullopt;
		}
		return variance_time_ / (samples_ - 1);
	}

	std::optional<double> populationVariance() const
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

	double totalWithCurrentSeconds(std::string const& tag) const
	{
		return totalSeconds(tag) + currentTimingSeconds(tag);
	}

	double totalWithCurrentMilliseconds(std::string const& tag) const
	{
		return totalMilliseconds(tag) + currentTimingMilliseconds(tag);
	}

	double totalWithCurrentMicroseconds(std::string const& tag) const
	{
		return totalMicroseconds(tag) + currentTimingMicroseconds(tag);
	}

	double currentTimingSeconds(std::string const& tag) const
	{
		auto ct = timers_.at(tag).currentTiming();
		if (!ct) {
			return -1;
		}
		return *ct / 1000000000.0;
	}

	double currentTimingMilliseconds(std::string const& tag) const
	{
		auto ct = timers_.at(tag).currentTiming();
		if (!ct) {
			return -1;
		}
		return *ct / 1000000.0;
	}

	double currentTimingMicroseconds(std::string const& tag) const
	{
		auto ct = timers_.at(tag).currentTiming();
		if (!ct) {
			return -1;
		}
		return *ct / 1000.0;
	}

	double lastSeconds(std::string const& tag) const
	{
		return timers_.at(tag).last() / 1000000000.0;
	}
	double lastMilliseconds(std::string const& tag) const
	{
		return timers_.at(tag).last() / 1000000.0;
	}
	double lastMicroseconds(std::string const& tag) const
	{
		return timers_.at(tag).last() / 1000.0;
	}

	std::size_t numSamples(std::string const& tag) const
	{
		return timers_.at(tag).numSamples();
	}

	double totalSeconds(std::string const& tag) const
	{
		return timers_.at(tag).total() / 1000000000.0;
	}
	double totalMilliseconds(std::string const& tag) const
	{
		return timers_.at(tag).total() / 1000000.0;
	}
	double totalMicroseconds(std::string const& tag) const
	{
		return timers_.at(tag).total() / 1000.0;
	}

	double minSeconds(std::string const& tag) const
	{
		auto min = timers_.at(tag).min();
		if (!min) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		return *min / 1000000000.0;
	}
	double minMilliseconds(std::string const& tag) const
	{
		auto min = timers_.at(tag).min();
		if (!min) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		return *min / 1000000.0;
	}
	double minMicroseconds(std::string const& tag) const
	{
		auto min = timers_.at(tag).min();
		if (!min) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		return *min / 1000.0;
	}

	double maxSeconds(std::string const& tag) const
	{
		auto max = timers_.at(tag).max();
		if (!max) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		return *max / 1000000000.0;
	}
	double maxMilliseconds(std::string const& tag) const
	{
		auto max = timers_.at(tag).max();
		if (!max) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		return *max / 1000000.0;
	}
	double maxMicroseconds(std::string const& tag) const
	{
		auto max = timers_.at(tag).max();
		if (!max) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		return *max / 1000.0;
	}

	double meanSeconds(std::string const& tag) const
	{
		return timers_.at(tag).mean() / 1000000000.0;
	}
	double meanMilliseconds(std::string const& tag) const
	{
		return timers_.at(tag).mean() / 1000000.0;
	}
	double meanMicroseconds(std::string const& tag) const
	{
		return timers_.at(tag).mean() / 1000.0;
	}

	double stdSeconds(std::string const& tag) const
	{
		auto variance = timers_.at(tag).variance();
		if (!variance) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		return std::sqrt(*variance) / 1000000000.0;
	}
	double stdMilliseconds(std::string const& tag) const
	{
		auto variance = timers_.at(tag).variance();
		if (!variance) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		return std::sqrt(*variance) / 1000000.0;
	}
	double stdMicroseconds(std::string const& tag) const
	{
		auto variance = timers_.at(tag).variance();
		if (!variance) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		return std::sqrt(*variance) / 1000.0;
	}

	std::vector<std::string> tags() const
	{
		std::vector<std::string> tags;
		tags.reserve(timers_.size());
		for (auto const& [tag, _] : timers_) {
			tags.emplace_back(tag);
		}
		return tags;
	}

	bool empty() const { return timers_.empty(); }

	std::string color(std::string const& tag) { return color_[tag]; }

	void setColor(std::string const& tag, std::string const& color) { color_[tag] = color; }

	static constexpr char const* resetColor() { return "\033[0m"; }
	static constexpr char const* blackColor() { return "\033[30m"; }
	static constexpr char const* redColor() { return "\033[31m"; }
	static constexpr char const* greenColor() { return "\033[32m"; }
	static constexpr char const* yellowColor() { return "\033[33m"; }
	static constexpr char const* blueColor() { return "\033[34m"; }
	static constexpr char const* magentaColor() { return "\033[35m"; }
	static constexpr char const* cyanColor() { return "\033[36m"; }
	static constexpr char const* whiteColor() { return "\033[37m"; }
	static constexpr char const* boldBlackColor() { return "\033[1m\033[30m"; }
	static constexpr char const* boldRedColor() { return "\033[1m\033[31m"; }
	static constexpr char const* boldGreenColor() { return "\033[1m\033[32m"; }
	static constexpr char const* boldYellowColor() { return "\033[1m\033[33m"; }
	static constexpr char const* boldBlueColor() { return "\033[1m\033[34m"; }
	static constexpr char const* boldMagentaColor() { return "\033[1m\033[35m"; }
	static constexpr char const* boldCyanColor() { return "\033[1m\033[36m"; }
	static constexpr char const* boldWhiteColor() { return "\033[1m\033[37m"; }

 private:
	std::map<std::string, TimerObject> timers_;
	std::map<std::string, std::string> color_;
};
}  // namespace ufo::util

#endif  // UFO_UTIL_TIMING_H