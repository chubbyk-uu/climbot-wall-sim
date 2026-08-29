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

#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>

#include "climbot_gazebo/clock_throttle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

namespace
{

class ClockThrottleNode : public rclcpp::Node
{
public:
  ClockThrottleNode()
  : Node("clock_throttle"),
    throttle_(declare_parameter("publish_rate_hz", 500.0)),
    report_delay_s_(declare_parameter("report_delay_s", 5.0))
  {
    // This node produces the clock every other node waits on. Running it on
    // simulation time would make it wait on its own output, and the whole
    // system would stop with no node reporting a fault. The invariant is
    // cheap to state here and impossible to flip silently in a launch file.
    if (get_parameter("use_sim_time").as_bool()) {
      throw std::invalid_argument(
              "clock_throttle must run on wall time: it publishes the simulation clock");
    }
    if (!std::isfinite(report_delay_s_) || report_delay_s_ <= 0.0) {
      throw std::invalid_argument("report_delay_s must be positive and finite");
    }

    const auto qos = rclcpp::ClockQoS();
    publisher_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", qos);
    subscription_ = create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock_raw", qos, [this](const rosgraph_msgs::msg::Clock::SharedPtr message) {
        // Steady time, never simulation time: this measures whether the clock
        // itself was late, so it cannot be derived from the clock.
        const auto arrival = std::chrono::steady_clock::now();
        if (last_input_) {
          throttle_.recordInputGap(
            std::chrono::duration<double>(arrival - *last_input_).count());
        }
        last_input_ = arrival;
        const int64_t stamp_ns = rclcpp::Time(message->clock).nanoseconds();
        if (throttle_.shouldPublish(stamp_ns)) {
          publisher_->publish(*message);
          const auto sent = std::chrono::steady_clock::now();
          if (last_output_) {
            throttle_.recordOutputGap(
              std::chrono::duration<double>(sent - *last_output_).count());
          }
          last_output_ = sent;
        }
      });
    // Deliberately reports the request, not a delivery. What is delivered is
    // decided by the input step and gets measured below.
    RCLCPP_INFO(
      get_logger(),
      "Requested %.3f Hz simulation clock (period %.3f ms); measuring the delivered rate.",
      throttle_.requestedRateHz(), static_cast<double>(throttle_.periodNs()) / 1.0e6);
    report_timer_ = create_wall_timer(
      std::chrono::duration<double>(report_delay_s_), [this]() {report();});
    gap_timer_ = create_wall_timer(
      std::chrono::duration<double>(
        declare_parameter("gap_summary_period_s", 10.0)), [this]() {reportGaps();});
  }

private:
  void report()
  {
    report_timer_->cancel();
    if (throttle_.outputs() < 2U) {
      RCLCPP_WARN(
        get_logger(),
        "No usable simulation clock arrived on /clock_raw within %.1f s; /clock is not being "
        "published.", report_delay_s_);
      return;
    }
    const double input_hz = throttle_.measuredInputRateHz();
    const double output_hz = throttle_.measuredOutputRateHz();
    RCLCPP_INFO(
      get_logger(),
      "CLOCK_THROTTLE measured input=%.2f Hz delivered=%.2f Hz requested=%.2f Hz "
      "published=%lu of %lu.",
      input_hz, output_hz, throttle_.requestedRateHz(),
      static_cast<unsigned long>(throttle_.outputs()),
      static_cast<unsigned long>(throttle_.inputs()));
    if (std::abs(output_hz - throttle_.requestedRateHz()) >
      kRateTolerance * throttle_.requestedRateHz())
    {
      RCLCPP_WARN(
        get_logger(),
        "Requested %.2f Hz but the stream delivers %.2f Hz. A %.3f ms input step reaches only "
        "its own integer divisions, so the request rounded down to the next reachable rate. "
        "Ask for one of those rates instead of relying on this rounding.",
        throttle_.requestedRateHz(), output_hz, input_hz > 0.0 ? 1000.0 / input_hz : 0.0);
    }
  }

  void reportGaps()
  {
    const auto & in = throttle_.inputGaps();
    const auto & out = throttle_.outputGaps();
    // Both sides in one line so a reader never has to align two timestamps:
    // equal maxima mean the stall arrived from upstream, an output maximum
    // above the input maximum means this node was the blockage.
    RCLCPP_INFO(
      get_logger(),
      "CLOCK_THROTTLE gaps in_n=%lu in_max_ms=%.1f in_ge_50=%lu in_ge_100=%lu in_ge_200=%lu "
      "in_ge_250=%lu out_n=%lu out_max_ms=%.1f out_ge_50=%lu out_ge_100=%lu out_ge_200=%lu "
      "out_ge_250=%lu",
      static_cast<unsigned long>(in.samples()), in.maxS() * 1000.0,
      static_cast<unsigned long>(in.atLeast(0)), static_cast<unsigned long>(in.atLeast(1)),
      static_cast<unsigned long>(in.atLeast(2)), static_cast<unsigned long>(in.atLeast(3)),
      static_cast<unsigned long>(out.samples()), out.maxS() * 1000.0,
      static_cast<unsigned long>(out.atLeast(0)), static_cast<unsigned long>(out.atLeast(1)),
      static_cast<unsigned long>(out.atLeast(2)), static_cast<unsigned long>(out.atLeast(3)));
  }

  static constexpr double kRateTolerance = 0.01;

  climbot_gazebo::ClockThrottle throttle_;
  double report_delay_s_{};
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr report_timer_;
  rclcpp::TimerBase::SharedPtr gap_timer_;
  std::optional<std::chrono::steady_clock::time_point> last_input_;
  std::optional<std::chrono::steady_clock::time_point> last_output_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ClockThrottleNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("clock_throttle"), "Node failed: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
