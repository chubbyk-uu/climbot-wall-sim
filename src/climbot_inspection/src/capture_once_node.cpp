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
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
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
  CaptureOnceNode()
  : Node("capture_once_node"),
    expected_frame_(requiredString(*this, "expected_frame_id", "inspection_camera_optical_frame")),
    expected_width_(requiredPositive<int64_t>(*this, "expected_width", 1920)),
    expected_height_(requiredPositive<int64_t>(*this, "expected_height", 1080)),
    capture_timeout_(seconds(requiredPositive<double>(*this, "capture_timeout_s", 5.0))),
    discovery_settle_(seconds(requiredPositive<double>(*this, "discovery_settle_s", 0.5))),
    warmup_retry_(seconds(requiredPositive<double>(*this, "warmup_retry_s", 1.0))),
    warmup_quiet_(seconds(requiredPositive<double>(*this, "warmup_quiet_s", 0.25)))
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
    const auto service_name = requiredString(
      *this, "capture_service", "/inspection/capture_once");

    const auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    trigger_publisher_ = create_publisher<std_msgs::msg::Bool>(trigger_topic, image_qos);
    image_publisher_ = create_publisher<sensor_msgs::msg::Image>(output_image, image_qos);
    info_publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(output_info, image_qos);

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
    service_ = create_service<std_srvs::srv::Trigger>(
      service_name,
      std::bind(
        &CaptureOnceNode::capture, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), service_group_);
    warmup_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&CaptureOnceNode::warmupTick, this), source_group_);
    RCLCPP_INFO(
      get_logger(), "Waiting for the camera trigger transport; warm-up frames will be discarded.");
  }

private:
  enum class State {WARMING, READY, CAPTURING, FAULTED};

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

  void onImage(const sensor_msgs::msg::Image::SharedPtr message)
  {
    if (!validImage(*message)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Rejected image with unexpected size or frame_id.");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != State::WARMING && state_ != State::CAPTURING) {
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
    if (state_ != State::WARMING && state_ != State::CAPTURING) {
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
    if (state_ != State::WARMING) {
      return;
    }
    const auto now = SteadyClock::now();
    if (warmup_frame_seen_) {
      if (now - last_warmup_frame_ >= warmup_quiet_) {
        state_ = State::READY;
        images_.clear();
        infos_.clear();
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
    const std_srvs::srv::Trigger::Request::SharedPtr,
    const std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (state_ == State::WARMING) {
      response->success = false;
      response->message = "Camera transport is still warming up.";
      return;
    }
    if (state_ == State::CAPTURING) {
      response->success = false;
      response->message = "Another capture request is already pending.";
      return;
    }
    if (state_ == State::FAULTED) {
      response->success = false;
      response->message =
        "Capture is faulted after a timeout; restart the node to exclude a late frame.";
      return;
    }

    state_ = State::CAPTURING;
    images_.clear();
    infos_.clear();
    capture_image_.reset();
    capture_info_.reset();
    capture_ready_ = false;
    publishTrigger();
    const bool received = condition_.wait_for(
      lock, capture_timeout_, [this]() {return capture_ready_ || !rclcpp::ok();});
    if (!received || !capture_ready_) {
      state_ = State::FAULTED;
      response->success = false;
      response->message =
        "Timed out waiting for a matching image/CameraInfo pair; capture is now faulted.";
      return;
    }
    const auto image = capture_image_;
    const auto info = capture_info_;
    lock.unlock();

    info_publisher_->publish(*info);
    image_publisher_->publish(*image);

    lock.lock();
    capture_image_.reset();
    capture_info_.reset();
    capture_ready_ = false;
    state_ = State::READY;
    lock.unlock();
    response->success = true;
    response->message = "Published one matched inspection image and CameraInfo pair.";
  }

  const std::string expected_frame_;
  const int64_t expected_width_;
  const int64_t expected_height_;
  const SteadyClock::duration capture_timeout_;
  const SteadyClock::duration discovery_settle_;
  const SteadyClock::duration warmup_retry_;
  const SteadyClock::duration warmup_quiet_;

  std::mutex mutex_;
  std::condition_variable condition_;
  State state_{State::WARMING};
  bool discovery_seen_{false};
  bool warmup_trigger_sent_{false};
  bool warmup_frame_seen_{false};
  bool capture_ready_{false};
  SteadyClock::time_point discovery_time_{};
  SteadyClock::time_point last_warmup_trigger_{};
  SteadyClock::time_point last_warmup_frame_{};
  std::unordered_map<int64_t, sensor_msgs::msg::Image::SharedPtr> images_;
  std::unordered_map<int64_t, sensor_msgs::msg::CameraInfo::SharedPtr> infos_;
  sensor_msgs::msg::Image::SharedPtr capture_image_;
  sensor_msgs::msg::CameraInfo::SharedPtr capture_info_;

  rclcpp::CallbackGroup::SharedPtr source_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trigger_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
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
