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

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <optional>
#include <functional>
#include <sstream>
#include <string>

#include "climbot_control/timing_monitor.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace
{

constexpr std::size_t kHistorySize = 256U;

int64_t steadyNowNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

double milliseconds(int64_t nanoseconds)
{
  return static_cast<double>(nanoseconds) / 1'000'000.0;
}

template<typename StampT>
int64_t stampNs(const StampT & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1'000'000'000LL +
         static_cast<int64_t>(stamp.nanosec);
}

bool crossedAny(const climbot_control::TimingObservation & observation)
{
  for (const bool crossed : observation.crossed) {
    if (crossed) {
      return true;
    }
  }
  return false;
}

std::string crossedThresholds(const climbot_control::TimingObservation & observation)
{
  std::ostringstream result;
  bool first = true;
  for (std::size_t index = 0; index < observation.crossed.size(); ++index) {
    if (!observation.crossed[index]) {
      continue;
    }
    if (!first) {
      result << ',';
    }
    first = false;
    result << milliseconds(climbot_control::kTimingThresholdNs[index]);
  }
  return result.str();
}

class StampedReceiptHistory
{
public:
  void add(int64_t stamp_ns, int64_t receipt_ns)
  {
    entries_[next_] = {stamp_ns, receipt_ns, true};
    next_ = (next_ + 1U) % entries_.size();
  }

  std::optional<int64_t> delayNs(int64_t stamp_ns, int64_t receipt_ns) const
  {
    for (std::size_t offset = 0U; offset < entries_.size(); ++offset) {
      const auto index = (next_ + entries_.size() - 1U - offset) % entries_.size();
      const auto & entry = entries_[index];
      if (entry.valid && entry.stamp_ns == stamp_ns && receipt_ns >= entry.receipt_ns) {
        return receipt_ns - entry.receipt_ns;
      }
    }
    return std::nullopt;
  }

private:
  struct Entry
  {
    int64_t stamp_ns{0};
    int64_t receipt_ns{0};
    bool valid{false};
  };
  std::array<Entry, kHistorySize> entries_{};
  std::size_t next_{0U};
};

struct StreamMetrics
{
  StreamMetrics(std::string stream_name, int64_t expected_interval_ns)
  : name(std::move(stream_name)), expected_interval_ns(expected_interval_ns) {}

  std::string name;
  int64_t expected_interval_ns;
  climbot_control::TimingSeries header;
  climbot_control::TimingSeries receipt;
  climbot_control::DurationStatistics header_gap;
  climbot_control::DurationStatistics receipt_gap;
  StampedReceiptHistory history;
  climbot_control::DurationStatistics upstream_delay;
  uint64_t paired_count{0U};
  uint64_t unpaired_count{0U};
  int64_t last_receipt_event_ns{0};
  int64_t last_header_event_ns{0};
  int64_t last_upstream_event_ns{0};
};

class LocalizationTimingMonitor : public rclcpp::Node
{
public:
  LocalizationTimingMonitor()
  : Node("localization_timing_monitor"),
    raw_wheel_("raw_wheel", expectedIntervalNs("raw_wheel_expected_period_ms", 20.0)),
    wheel_("wheel", expectedIntervalNs("wheel_expected_period_ms", 20.0)),
    raw_imu_("raw_imu", expectedIntervalNs("raw_imu_expected_period_ms", 10.0)),
    imu_("imu", expectedIntervalNs("imu_expected_period_ms", 10.0)),
    total_station_("total_station",
      expectedIntervalNs("total_station_expected_period_ms", 1000.0 / 12.0)),
    filtered_("filtered", expectedIntervalNs("filtered_expected_period_ms", 20.0))
  {
    const auto summary_period_s = declare_parameter("summary_period_s", 10.0);
    if (!std::isfinite(summary_period_s) || summary_period_s <= 0.0) {
      throw std::invalid_argument("summary_period_s must be finite and positive.");
    }
    summary_period_ns_ = static_cast<int64_t>(summary_period_s * 1'000'000'000.0);
    const auto qos = rclcpp::QoS(20).best_effort();
    raw_wheel_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/model/climbot/odometry", qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        record(raw_wheel_, stampNs(message->header.stamp), std::nullopt);
      });
    wheel_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/wheel_odom", qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        record(wheel_, stampNs(message->header.stamp), raw_wheel_);
      });
    raw_imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", qos,
      [this](const sensor_msgs::msg::Imu::SharedPtr message) {
        record(raw_imu_, stampNs(message->header.stamp), std::nullopt);
      });
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu_wall", qos,
      [this](const sensor_msgs::msg::Imu::SharedPtr message) {
        record(imu_, stampNs(message->header.stamp), raw_imu_);
      });
    total_station_subscription_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/total_station/pose", qos,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
        record(total_station_, stampNs(message->header.stamp), std::nullopt);
      });
    filtered_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        record(filtered_, stampNs(message->header.stamp), std::nullopt);
      });
    summary_timer_ = create_wall_timer(
      std::chrono::nanoseconds(summary_period_ns_), [this]() {publishSummary();});
    RCLCPP_INFO(
      get_logger(),
      "Localization timing monitor active: six small topics, %0.1f s wall-time summaries.",
      summary_period_s);
  }

private:
  int64_t expectedIntervalNs(const std::string & parameter, double default_ms)
  {
    const auto value_ms = declare_parameter(parameter, default_ms);
    if (!std::isfinite(value_ms) || value_ms <= 0.0) {
      throw std::invalid_argument(parameter + " must be finite and positive.");
    }
    return static_cast<int64_t>(value_ms * 1'000'000.0);
  }

  void record(
    StreamMetrics & stream, int64_t header_ns,
    std::optional<std::reference_wrapper<StreamMetrics>> upstream)
  {
    const auto receipt_ns = steadyNowNs();
    const auto receipt_record = stream.receipt.record(receipt_ns);
    const auto header_record = stream.header.record(header_ns);
    if (upstream.has_value()) {
      const auto delay = upstream->get().history.delayNs(header_ns, receipt_ns);
      if (delay.has_value()) {
        ++stream.paired_count;
        const auto delay_observation = stream.upstream_delay.add(*delay);
        if (crossedAny(delay_observation)) {
          logEvent(stream, "upstream_delay", delay_observation);
        }
      } else {
        ++stream.unpaired_count;
      }
    }
    stream.history.add(header_ns, receipt_ns);
    logRecord(stream, "receipt", receipt_record, stream.receipt_gap);
    logRecord(stream, "header", header_record, stream.header_gap);
  }

  void logRecord(
    StreamMetrics & stream, const char * kind,
    const climbot_control::TimingSeries::Record & record,
    climbot_control::DurationStatistics & gap_statistics)
  {
    if (record.regressed) {
      const auto rollbacks = std::string(kind) == "header" ?
        stream.header.rollbackCount() : stream.receipt.rollbackCount();
      RCLCPP_WARN(
        get_logger(), "LOCALIZATION_TIMING event=clock_rollback stream=%s kind=%s count=%lu",
        stream.name.c_str(), kind, rollbacks);
      return;
    }
    if (!record.initialized) {
      const auto excess_ns = std::max<int64_t>(
        0, record.observation.duration_ns - stream.expected_interval_ns);
      const auto gap = gap_statistics.add(excess_ns);
      if (crossedAny(gap)) {
        logEvent(stream, kind, gap);
      }
    }
  }

  void logEvent(
    StreamMetrics & stream, const char * kind,
    const climbot_control::TimingObservation & observation)
  {
    auto * last_event_ns = &stream.last_receipt_event_ns;
    if (std::string(kind) == "header") {
      last_event_ns = &stream.last_header_event_ns;
    } else if (std::string(kind) == "upstream_delay") {
      last_event_ns = &stream.last_upstream_event_ns;
    }
    const auto now_ns = steadyNowNs();
    constexpr int64_t kEventSnapshotMinimumIntervalNs = 1'000'000'000LL;
    if (*last_event_ns != 0 && now_ns - *last_event_ns < kEventSnapshotMinimumIntervalNs) {
      return;
    }
    *last_event_ns = now_ns;
    RCLCPP_WARN(
      get_logger(),
      "LOCALIZATION_TIMING event=gap stream=%s kind=%s duration_ms=%.3f thresholds_ms=%s",
      stream.name.c_str(), kind, milliseconds(observation.duration_ns),
      crossedThresholds(observation).c_str());
  }

  void publishSummary()
  {
    const auto now_ns = steadyNowNs();
    if (last_summary_ns_.has_value()) {
      const auto lateness_ns = now_ns - *last_summary_ns_ - summary_period_ns_;
      if (lateness_ns > 0) {
        const auto observation = summary_lateness_.add(lateness_ns);
        if (crossedAny(observation)) {
          RCLCPP_WARN(
            get_logger(),
            "LOCALIZATION_TIMING event=monitor_unscheduled lateness_ms=%.3f thresholds_ms=%s",
            milliseconds(observation.duration_ns), crossedThresholds(observation).c_str());
        }
      }
    }
    last_summary_ns_ = now_ns;
    for (const auto * stream : streams()) {
      RCLCPP_INFO(
        get_logger(),
        "LOCALIZATION_TIMING summary stream=%s header_n=%lu header_max_ms=%.3f "
        "header_p999_ms=%.3f header_p9999_ms=%.3f header_ge_50=%lu header_ge_100=%lu "
        "header_ge_200=%lu header_ge_250=%lu header_rollbacks=%lu header_gap_ge_50=%lu "
        "header_gap_ge_100=%lu header_gap_ge_200=%lu header_gap_ge_250=%lu receipt_n=%lu "
        "receipt_max_ms=%.3f receipt_p999_ms=%.3f receipt_p9999_ms=%.3f "
        "receipt_ge_50=%lu receipt_ge_100=%lu receipt_ge_200=%lu receipt_ge_250=%lu "
        "receipt_rollbacks=%lu receipt_gap_ge_50=%lu receipt_gap_ge_100=%lu "
        "receipt_gap_ge_200=%lu receipt_gap_ge_250=%lu paired=%lu unpaired=%lu "
        "upstream_max_ms=%.3f",
        stream->name.c_str(), stream->header.statistics().sampleCount(),
        milliseconds(stream->header.statistics().maxNs()),
        milliseconds(stream->header.statistics().quantileNs(0.999)),
        milliseconds(stream->header.statistics().quantileNs(0.9999)),
        stream->header.statistics().thresholdCount(0U),
          stream->header.statistics().thresholdCount(1U),
        stream->header.statistics().thresholdCount(2U),
          stream->header.statistics().thresholdCount(3U),
        stream->header.rollbackCount(), stream->header_gap.thresholdCount(0U),
        stream->header_gap.thresholdCount(1U), stream->header_gap.thresholdCount(2U),
        stream->header_gap.thresholdCount(3U), stream->receipt.statistics().sampleCount(),
        milliseconds(stream->receipt.statistics().maxNs()),
        milliseconds(stream->receipt.statistics().quantileNs(0.999)),
        milliseconds(stream->receipt.statistics().quantileNs(0.9999)),
        stream->receipt.statistics().thresholdCount(0U),
          stream->receipt.statistics().thresholdCount(1U),
        stream->receipt.statistics().thresholdCount(2U),
          stream->receipt.statistics().thresholdCount(3U),
        stream->receipt.rollbackCount(), stream->receipt_gap.thresholdCount(0U),
        stream->receipt_gap.thresholdCount(1U), stream->receipt_gap.thresholdCount(2U),
        stream->receipt_gap.thresholdCount(3U), stream->paired_count, stream->unpaired_count,
        milliseconds(stream->upstream_delay.maxNs()));
    }
  }

  std::array<const StreamMetrics *, 6> streams() const
  {
    return {&raw_wheel_, &wheel_, &raw_imu_, &imu_, &total_station_, &filtered_};
  }

  int64_t summary_period_ns_{10'000'000'000LL};
  std::optional<int64_t> last_summary_ns_;
  climbot_control::DurationStatistics summary_lateness_;
  StreamMetrics raw_wheel_;
  StreamMetrics wheel_;
  StreamMetrics raw_imu_;
  StreamMetrics imu_;
  StreamMetrics total_station_;
  StreamMetrics filtered_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_wheel_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr raw_imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    total_station_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr filtered_subscription_;
  rclcpp::TimerBase::SharedPtr summary_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LocalizationTimingMonitor>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("localization_timing_monitor"), "Node failed: %s",
      exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
