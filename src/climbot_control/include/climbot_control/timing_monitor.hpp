// Copyright 2026 jerry
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CLIMBOT_CONTROL__TIMING_MONITOR_HPP_
#define CLIMBOT_CONTROL__TIMING_MONITOR_HPP_

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace climbot_control
{

/// Fixed thresholds shared by the localization monitor and line tracker.
inline constexpr std::array<int64_t, 4> kTimingThresholdNs{
  50'000'000LL, 100'000'000LL, 200'000'000LL, 250'000'000LL};

/// The result of adding one duration without retaining any per-message data.
struct TimingObservation
{
  int64_t duration_ns{0};
  std::array<bool, kTimingThresholdNs.size()> crossed{};
};

/// Constant-space duration distribution, rounded up to one millisecond bins.
///
/// The rounded quantiles deliberately make a reported bound conservative.  A
/// monitor exists to catch rare long pauses; it must not allocate or write one
/// record for every 50--100 Hz message in order to estimate their percentile.
class DurationStatistics
{
public:
  static constexpr int64_t kBucketWidthNs = 1'000'000LL;
  static constexpr std::size_t kLastBucket = 1000U;

  TimingObservation add(int64_t duration_ns)
  {
    TimingObservation observation;
    observation.duration_ns = std::max<int64_t>(0, duration_ns);
    ++sample_count_;
    max_ns_ = std::max(max_ns_, observation.duration_ns);
    const auto bucket = static_cast<std::size_t>(std::min<int64_t>(
      kLastBucket,
      std::max<int64_t>(1, (observation.duration_ns + kBucketWidthNs - 1) /
      kBucketWidthNs)));
    ++histogram_[bucket];
    for (std::size_t index = 0; index < kTimingThresholdNs.size(); ++index) {
      if (observation.duration_ns >= kTimingThresholdNs[index]) {
        ++threshold_counts_[index];
        observation.crossed[index] = true;
      }
    }
    return observation;
  }

  [[nodiscard]] uint64_t sampleCount() const {return sample_count_;}
  [[nodiscard]] int64_t maxNs() const {return max_ns_;}
  [[nodiscard]] uint64_t thresholdCount(std::size_t index) const
  {
    return threshold_counts_.at(index);
  }

  /// Returns a conservative one-millisecond-rounded quantile.
  [[nodiscard]] int64_t quantileNs(double quantile) const
  {
    if (sample_count_ == 0U) {
      return 0;
    }
    const auto bounded = std::clamp(quantile, 0.0, 1.0);
    const auto rank = static_cast<uint64_t>(
      std::ceil(bounded * static_cast<double>(sample_count_)));
    uint64_t accumulated = 0U;
    for (std::size_t bucket = 1U; bucket < histogram_.size(); ++bucket) {
      accumulated += histogram_[bucket];
      if (accumulated >= std::max<uint64_t>(1U, rank)) {
        return static_cast<int64_t>(bucket) * kBucketWidthNs;
      }
    }
    return static_cast<int64_t>(kLastBucket) * kBucketWidthNs;
  }

private:
  uint64_t sample_count_{0U};
  int64_t max_ns_{0};
  std::array<uint64_t, kTimingThresholdNs.size()> threshold_counts_{};
  std::array<uint64_t, kLastBucket + 1U> histogram_{};
};

/// Adds ordering semantics to DurationStatistics for a clock or message stamp.
class TimingSeries
{
public:
  struct Record
  {
    bool initialized{false};
    bool regressed{false};
    TimingObservation observation{};
  };

  Record record(int64_t timestamp_ns)
  {
    if (!have_last_) {
      have_last_ = true;
      last_ns_ = timestamp_ns;
      Record record;
      record.initialized = true;
      return record;
    }
    if (timestamp_ns < last_ns_) {
      ++rollback_count_;
      last_ns_ = timestamp_ns;
      Record record;
      record.regressed = true;
      return record;
    }
    const auto observation = statistics_.add(timestamp_ns - last_ns_);
    last_ns_ = timestamp_ns;
    Record record;
    record.observation = observation;
    return record;
  }

  [[nodiscard]] const DurationStatistics & statistics() const {return statistics_;}
  [[nodiscard]] uint64_t rollbackCount() const {return rollback_count_;}
  [[nodiscard]] bool hasLast() const {return have_last_;}
  [[nodiscard]] int64_t lastNs() const {return last_ns_;}

private:
  bool have_last_{false};
  int64_t last_ns_{0};
  uint64_t rollback_count_{0U};
  DurationStatistics statistics_;
};

}  // namespace climbot_control

#endif  // CLIMBOT_CONTROL__TIMING_MONITOR_HPP_
