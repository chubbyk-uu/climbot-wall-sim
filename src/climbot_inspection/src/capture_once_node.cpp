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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>

#include "climbot_interfaces/srv/capture_once.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{

using SteadyClock = std::chrono::steady_clock;

int64_t stampKey(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

template<typename T>
T requiredPositive(rclcpp::Node & node, const std::string & name, T default_value)
{
  const T value = node.declare_parameter<T>(name, default_value);
  if constexpr (std::is_floating_point_v<T>) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(name + " must be positive and finite.");
    }
  } else if (value <= 0) {
    throw std::invalid_argument(name + " must be positive.");
  }
  return value;
}

std::string requiredString(
  rclcpp::Node & node, const std::string & name, const std::string & default_value)
{
  const auto value = node.declare_parameter<std::string>(name, default_value);
  if (value.empty()) {
    throw std::invalid_argument(name + " must not be empty.");
  }
  return value;
}

}  // namespace

class CaptureOnceNode : public rclcpp::Node
{
public:
  using CaptureOnce = climbot_interfaces::srv::CaptureOnce;

  CaptureOnceNode()
  : Node("capture_once_node"),
    expected_frame_(requiredString(*this, "expected_frame_id", "inspection_camera_optical_frame")),
    expected_width_(requiredPositive<int64_t>(*this, "expected_width", 1920)),
    expected_height_(requiredPositive<int64_t>(*this, "expected_height", 1080)),
    capture_timeout_(seconds(requiredPositive<double>(*this, "capture_timeout_s", 5.0))),
    discovery_settle_(seconds(requiredPositive<double>(*this, "discovery_settle_s", 0.5))),
    warmup_retry_(seconds(requiredPositive<double>(*this, "warmup_retry_s", 1.0))),
    warmup_quiet_(seconds(requiredPositive<double>(*this, "warmup_quiet_s", 0.25))),
    slow_capture_warning_(seconds(requiredPositive<double>(
        *this, "slow_capture_warning_s", 0.5)))
  {
    const auto source_image = requiredString(
      *this, "source_image_topic", "/simulation/inspection_camera/image_raw");
    const auto source_info = requiredString(
      *this, "source_camera_info_topic", "/simulation/inspection_camera/camera_info");
    const auto trigger_topic = requiredString(
      *this, "trigger_topic", "/simulation/inspection_camera/trigger");
    const auto output_image = requiredString(
      *this, "output_image_topic", "/inspection/camera/image_raw");
    const auto output_info = requiredString(
      *this, "output_camera_info_topic", "/inspection/camera/camera_info");
    const auto receipt_topic = requiredString(
      *this, "capture_receipt_topic", "/inspection/capture_receipt");
    const auto service_name = requiredString(
      *this, "capture_service", "/inspection/capture_once");
    const auto reset_service_name = requiredString(
      *this, "capture_reset_service", "/inspection/capture_reset");
    const auto state_topic = requiredString(
      *this, "capture_state_topic", "/inspection/capture_state");
    enforce_trigger_stamp_ = declare_parameter<bool>("enforce_trigger_stamp", true);

    const auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    trigger_publisher_ = create_publisher<std_msgs::msg::Bool>(trigger_topic, image_qos);
    image_publisher_ = create_publisher<sensor_msgs::msg::Image>(output_image, image_qos);
    // Intrinsics are immutable for one recorder process. Latch the latest
    // snapshot instead of making archive integrity depend on 660 individual
    // CameraInfo deliveries in a large mission.
    info_publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      output_info, rclcpp::QoS(1).reliable().transient_local());
    receipt_publisher_ = create_publisher<std_msgs::msg::Header>(
      receipt_topic, rclcpp::QoS(10).reliable());
    state_publisher_ = create_publisher<std_msgs::msg::UInt8>(
      state_topic, rclcpp::QoS(1).reliable().transient_local());

    source_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    service_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions options;
    options.callback_group = source_group_;
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      source_image, image_qos,
      std::bind(&CaptureOnceNode::onImage, this, std::placeholders::_1), options);
    info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      source_info, image_qos,
      std::bind(&CaptureOnceNode::onInfo, this, std::placeholders::_1), options);
    service_ = create_service<CaptureOnce>(
      service_name,
      std::bind(
        &CaptureOnceNode::capture, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), service_group_);
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      reset_service_name,
      std::bind(
        &CaptureOnceNode::reset, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), service_group_);
    warmup_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&CaptureOnceNode::warmupTick, this), source_group_);
    publishState();
    RCLCPP_INFO(
      get_logger(), "Waiting for the camera trigger transport; warm-up frames will be discarded.");
  }

private:
  enum class State : uint8_t {WARMING, READY, CAPTURING, DRAINING};

  static SteadyClock::duration seconds(double value)
  {
    return std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(value));
  }

  bool validImage(const sensor_msgs::msg::Image & message) const
  {
    return message.width == static_cast<uint32_t>(expected_width_) &&
           message.height == static_cast<uint32_t>(expected_height_) &&
           message.header.frame_id == expected_frame_;
  }

  bool validInfo(const sensor_msgs::msg::CameraInfo & message) const
  {
    return message.width == static_cast<uint32_t>(expected_width_) &&
           message.height == static_cast<uint32_t>(expected_height_) &&
           message.header.frame_id == expected_frame_;
  }

  void publishState()
  {
    std_msgs::msg::UInt8 message;
    switch (state_) {
      case State::READY:
        message.data = CaptureOnce::Response::OK;
        break;
      case State::WARMING:
        message.data = CaptureOnce::Response::WARMING;
        break;
      case State::CAPTURING:
        message.data = CaptureOnce::Response::BUSY;
        break;
      case State::DRAINING:
        message.data = CaptureOnce::Response::DRAINING;
        break;
    }
    state_publisher_->publish(message);
  }

  void enterDrainingLocked()
  {
    images_.clear();
    infos_.clear();
    capture_image_.reset();
    capture_info_.reset();
    capture_ready_ = false;
    state_ = State::DRAINING;
    last_drain_frame_ = SteadyClock::now();
    publishState();
  }

  void onImage(const sensor_msgs::msg::Image::SharedPtr message)
  {
    if (!validImage(*message)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Rejected image with unexpected size or frame_id.");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == State::DRAINING) {
      // A frame from the timed-out request is not evidence for a later request.
      // Extend the quiet interval each time one arrives and discard it.
      last_drain_frame_ = SteadyClock::now();
      return;
    }
    if (state_ != State::WARMING && state_ != State::CAPTURING) {
      return;
    }
    if (state_ == State::CAPTURING && enforce_trigger_stamp_ &&
      stampKey(message->header.stamp) < trigger_stamp_key_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Discarded an image older than the capture trigger.");
      return;
    }
    images_[stampKey(message->header.stamp)] = message;
    matchPairLocked(stampKey(message->header.stamp));
  }

  void onInfo(const sensor_msgs::msg::CameraInfo::SharedPtr message)
  {
    if (!validInfo(*message)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Rejected CameraInfo with unexpected size or frame_id.");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == State::DRAINING) {
      last_drain_frame_ = SteadyClock::now();
      return;
    }
    if (state_ != State::WARMING && state_ != State::CAPTURING) {
      return;
    }
    if (state_ == State::CAPTURING && enforce_trigger_stamp_ &&
      stampKey(message->header.stamp) < trigger_stamp_key_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Discarded CameraInfo older than the capture trigger.");
      return;
    }
    infos_[stampKey(message->header.stamp)] = message;
    matchPairLocked(stampKey(message->header.stamp));
  }

  void matchPairLocked(int64_t key)
  {
    const auto image = images_.find(key);
    const auto info = infos_.find(key);
    if (image == images_.end() || info == infos_.end()) {
      return;
    }
    const auto image_message = image->second;
    const auto info_message = info->second;
    images_.clear();
    infos_.clear();
    if (state_ == State::WARMING) {
      warmup_frame_seen_ = true;
      last_warmup_frame_ = SteadyClock::now();
      return;
    }
    capture_image_ = image_message;
    capture_info_ = info_message;
    capture_ready_ = true;
    capture_pair_received_ = SteadyClock::now();
    condition_.notify_all();
  }

  void publishTrigger()
  {
    std_msgs::msg::Bool trigger;
    trigger.data = true;
    trigger_publisher_->publish(trigger);
  }

  void warmupTick()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == State::DRAINING) {
      if (SteadyClock::now() - last_drain_frame_ >= warmup_quiet_) {
        state_ = State::READY;
        images_.clear();
        infos_.clear();
        publishState();
        RCLCPP_INFO(get_logger(), "Late-frame drain complete; capture service is ready.");
      }
      return;
    }
    if (state_ != State::WARMING) {
      return;
    }
    const auto now = SteadyClock::now();
    if (warmup_frame_seen_) {
      if (now - last_warmup_frame_ >= warmup_quiet_) {
        state_ = State::READY;
        images_.clear();
        infos_.clear();
        publishState();
        RCLCPP_INFO(get_logger(), "Camera warm-up complete; capture service is ready.");
      }
      return;
    }
    if (trigger_publisher_->get_subscription_count() == 0) {
      discovery_seen_ = false;
      return;
    }
    if (!discovery_seen_) {
      discovery_seen_ = true;
      discovery_time_ = now;
      return;
    }
    if (now - discovery_time_ < discovery_settle_) {
      return;
    }
    if (!warmup_trigger_sent_ || now - last_warmup_trigger_ >= warmup_retry_) {
      publishTrigger();
      warmup_trigger_sent_ = true;
      last_warmup_trigger_ = now;
    }
  }

  void capture(
    const CaptureOnce::Request::SharedPtr,
    const CaptureOnce::Response::SharedPtr response)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (state_ == State::WARMING) {
      response->success = false;
      response->reason = CaptureOnce::Response::WARMING;
      response->message = "Camera transport is still warming up.";
      return;
    }
    if (state_ == State::CAPTURING) {
      response->success = false;
      response->reason = CaptureOnce::Response::BUSY;
      response->message = "Another capture request is already pending.";
      return;
    }
    if (state_ == State::DRAINING) {
      response->success = false;
      response->reason = CaptureOnce::Response::DRAINING;
      response->message = "Capture transport is draining possible late frames.";
      return;
    }

    state_ = State::CAPTURING;
    const uint64_t capture_id = ++capture_sequence_;
    const auto capture_started = SteadyClock::now();
    images_.clear();
    infos_.clear();
    capture_image_.reset();
    capture_info_.reset();
    capture_ready_ = false;
    trigger_stamp_key_ = get_clock()->now().nanoseconds();
    publishState();
    publishTrigger();
    // A shutdown cannot safely rely on a callback which captures this node:
    // executor teardown may outlive its service worker. Polling in bounded
    // slices lets the worker observe rclcpp::ok() without a dangling callback
    // or a full capture_timeout_s shutdown delay.
    const auto deadline = SteadyClock::now() + capture_timeout_;
    bool received = capture_ready_;
    while (!received && rclcpp::ok() && SteadyClock::now() < deadline) {
      const auto left = deadline - SteadyClock::now();
      condition_.wait_for(lock, std::min(
        left, std::chrono::duration_cast<SteadyClock::duration>(std::chrono::milliseconds(100))));
      received = capture_ready_;
    }
    if (!received || !capture_ready_) {
      enterDrainingLocked();
      response->success = false;
      response->reason = CaptureOnce::Response::TIMEOUT;
      response->message =
        "Timed out waiting for a matching image/CameraInfo pair; draining possible late frames.";
      return;
    }
    const auto image = capture_image_;
    const auto info = capture_info_;
    const auto pair_received = capture_pair_received_;
    const double simulation_lag_ms = 1e-6 * static_cast<double>(
      stampKey(image->header.stamp) - trigger_stamp_key_);
    lock.unlock();

    const auto publish_started = SteadyClock::now();
    info_publisher_->publish(*info);
    image_publisher_->publish(*image);
    // The trigger controller needs correlation, not another full image copy.
    // Emit the lightweight receipt only after both canonical outputs were
    // handed to their reliable DDS publishers.
    receipt_publisher_->publish(image->header);
    const auto publish_finished = SteadyClock::now();

    lock.lock();
    capture_image_.reset();
    capture_info_.reset();
    capture_ready_ = false;
    state_ = State::READY;
    publishState();
    lock.unlock();
    response->success = true;
    response->header = image->header;
    response->reason = CaptureOnce::Response::OK;
    response->message = "Published one matched inspection image and CameraInfo pair.";
    const auto total = publish_finished - capture_started;
    const double pair_ms = 1000.0 * std::chrono::duration<double>(
      pair_received - capture_started).count();
    const double publish_ms = 1000.0 * std::chrono::duration<double>(
      publish_finished - publish_started).count();
    const double total_ms = 1000.0 * std::chrono::duration<double>(total).count();
    if (total > slow_capture_warning_) {
      RCLCPP_WARN(
        get_logger(),
        "Slow capture %lu: source pair %.1f ms, simulation lag %.1f ms, publish %.1f ms, "
        "total %.1f ms.",
        static_cast<unsigned long>(capture_id), pair_ms, simulation_lag_ms, publish_ms, total_ms);
    } else if (capture_id == 1U || capture_id % 50U == 0U) {
      RCLCPP_INFO(
        get_logger(),
        "Capture %lu timing: source pair %.1f ms, simulation lag %.1f ms, publish %.1f ms, "
        "total %.1f ms.",
        static_cast<unsigned long>(capture_id), pair_ms, simulation_lag_ms, publish_ms, total_ms);
    }
  }

  void reset(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    const std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == State::WARMING) {
      response->success = false;
      response->message = "Camera transport is warming up; reset cannot bypass warm-up.";
      return;
    }
    if (state_ == State::CAPTURING) {
      response->success = false;
      response->message = "A capture request is pending; it cannot be reset safely.";
      return;
    }
    enterDrainingLocked();
    response->success = true;
    response->message = "Capture reset accepted; draining possible late frames before re-arming.";
  }

  const std::string expected_frame_;
  const int64_t expected_width_;
  const int64_t expected_height_;
  const SteadyClock::duration capture_timeout_;
  const SteadyClock::duration discovery_settle_;
  const SteadyClock::duration warmup_retry_;
  const SteadyClock::duration warmup_quiet_;
  const SteadyClock::duration slow_capture_warning_;
  bool enforce_trigger_stamp_{};

  std::mutex mutex_;
  std::condition_variable condition_;
  State state_{State::WARMING};
  bool discovery_seen_{false};
  bool warmup_trigger_sent_{false};
  bool warmup_frame_seen_{false};
  bool capture_ready_{false};
  uint64_t capture_sequence_{};
  SteadyClock::time_point discovery_time_{};
  SteadyClock::time_point last_warmup_trigger_{};
  SteadyClock::time_point last_warmup_frame_{};
  SteadyClock::time_point last_drain_frame_{};
  SteadyClock::time_point capture_pair_received_{};
  int64_t trigger_stamp_key_{};
  std::unordered_map<int64_t, sensor_msgs::msg::Image::SharedPtr> images_;
  std::unordered_map<int64_t, sensor_msgs::msg::CameraInfo::SharedPtr> infos_;
  sensor_msgs::msg::Image::SharedPtr capture_image_;
  sensor_msgs::msg::CameraInfo::SharedPtr capture_info_;

  rclcpp::CallbackGroup::SharedPtr source_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trigger_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_publisher_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr receipt_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_subscription_;
  rclcpp::Service<CaptureOnce>::SharedPtr service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr warmup_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<CaptureOnceNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("capture_once_node"), "Node failed: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
