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
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>

#include "climbot_interfaces/msg/execution_reference.hpp"
#include "climbot_interfaces/msg/inspection_capture.hpp"
#include "climbot_interfaces/msg/inspection_capture_gate.hpp"
#include "climbot_interfaces/srv/capture_once.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

namespace
{

double finitePositive(rclcpp::Node & node, const std::string & name, double fallback)
{
  const double value = node.declare_parameter(name, fallback);
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(name + " must be positive and finite.");
  }
  return value;
}

double finite(rclcpp::Node & node, const std::string & name, double fallback)
{
  const double value = node.declare_parameter(name, fallback);
  if (!std::isfinite(value)) {
    throw std::invalid_argument(name + " must be finite.");
  }
  return value;
}

std::string nonEmpty(rclcpp::Node & node, const std::string & name, const std::string & fallback)
{
  const auto value = node.declare_parameter(name, fallback);
  if (value.empty()) {
    throw std::invalid_argument(name + " must not be empty.");
  }
  return value;
}

std::chrono::steady_clock::duration steadySeconds(double value)
{
  return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(value));
}

double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + 1e-9 * static_cast<double>(stamp.nanosec);
}

double yaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double wrap(double value)
{
  return std::atan2(std::sin(value), std::cos(value));
}

geometry_msgs::msg::Quaternion multiply(
  const geometry_msgs::msg::Quaternion & a, const geometry_msgs::msg::Quaternion & b)
{
  geometry_msgs::msg::Quaternion result;
  result.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  result.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  result.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  result.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  return result;
}

geometry_msgs::msg::Quaternion rpyQuaternion(double roll, double pitch, double heading)
{
  const double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
  const double cy = std::cos(heading * 0.5), sy = std::sin(heading * 0.5);
  geometry_msgs::msg::Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

}  // namespace

class AutomaticCaptureNode : public rclcpp::Node
{
public:
  using Reference = climbot_interfaces::msg::ExecutionReference;
  using Metadata = climbot_interfaces::msg::InspectionCapture;
  using CaptureGate = climbot_interfaces::msg::InspectionCaptureGate;
  using CaptureOnce = climbot_interfaces::srv::CaptureOnce;

  AutomaticCaptureNode()
  : Node("automatic_capture_node"),
    footprint_length_(finitePositive(*this, "effective_length_m", 0.28125)),
    overlap_(finite(*this, "image_overlap_ratio", 0.20)),
    mount_x_(finite(*this, "camera_mount_x_m", 0.340)),
    mount_y_(finite(*this, "camera_mount_y_m", 0.0)),
    mount_z_(finite(*this, "camera_mount_z_m", 0.275)),
    mount_roll_(finite(*this, "camera_mount_roll_rad", std::acos(-1.0))),
    mount_pitch_(finite(*this, "camera_mount_pitch_rad", 0.0)),
    mount_yaw_(finite(*this, "camera_mount_yaw_rad", -0.5 * std::acos(-1.0))),
    reference_timeout_(finitePositive(*this, "reference_timeout_s", 0.5)),
    capture_response_timeout_(finitePositive(*this, "capture_response_timeout_s", 6.0)),
    image_wait_timeout_(finitePositive(*this, "image_wait_timeout_s", 1.0)),
    slow_capture_warning_(finitePositive(*this, "slow_capture_warning_s", 0.5)),
    pose_wait_timeout_(finitePositive(*this, "pose_wait_timeout_s", 0.5)),
    cache_duration_(finitePositive(*this, "pose_cache_duration_s", 3.0))
  {
    if (overlap_ < 0.0 || overlap_ >= 1.0) {
      throw std::invalid_argument("image_overlap_ratio must be within [0, 1).");
    }
    spacing_ = footprint_length_ * (1.0 - overlap_);
    const auto reference_topic = nonEmpty(
      *this, "execution_reference_topic", "/control/execution_reference");
    const auto odometry_topic = nonEmpty(*this, "odometry_topic", "/odometry/filtered");
    const auto receipt_topic = nonEmpty(
      *this, "capture_receipt_topic", "/inspection/capture_receipt");
    const auto service = nonEmpty(*this, "capture_service", "/inspection/capture_once");
    const auto metadata_topic = nonEmpty(
      *this, "metadata_topic", "/inspection/capture_metadata");
    const auto gate_topic = nonEmpty(
      *this, "capture_gate_topic", "/inspection/capture_gate");

    reference_subscription_ = create_subscription<Reference>(
      reference_topic, rclcpp::QoS(10).reliable(),
      std::bind(&AutomaticCaptureNode::onReference, this, std::placeholders::_1));
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic, rclcpp::SensorDataQoS(),
      std::bind(&AutomaticCaptureNode::onOdometry, this, std::placeholders::_1));
    receipt_subscription_ = create_subscription<std_msgs::msg::Header>(
      receipt_topic, rclcpp::QoS(10).reliable(),
      std::bind(&AutomaticCaptureNode::onReceipt, this, std::placeholders::_1));
    capture_client_ = create_client<CaptureOnce>(service);
    metadata_publisher_ = create_publisher<Metadata>(metadata_topic, rclcpp::QoS(10).reliable());
    gate_publisher_ = create_publisher<CaptureGate>(
      gate_topic, rclcpp::QoS(1).reliable().transient_local());
    timeout_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&AutomaticCaptureNode::checkTimeouts, this));
    RCLCPP_INFO(
      get_logger(), "Automatic inspection spacing is %.4f m (length %.4f m, overlap %.1f%%).",
      spacing_, footprint_length_, overlap_ * 100.0);
  }

private:
  struct Key
  {
    std::string task_id;
    uint32_t revision{};
    int32_t segment{};
    bool operator==(const Key & other) const
    {
      return task_id == other.task_id && revision == other.revision && segment == other.segment;
    }
  };

  struct Pending
  {
    Metadata metadata;
    std::chrono::steady_clock::time_point requested;
    uint64_t request_id{};
    std::optional<std::chrono::steady_clock::time_point> response_received;
    std::optional<bool> response_succeeded;
    std::optional<rclcpp::Time> image_stamp;
  };

  void onReference(const Reference::SharedPtr message)
  {
    const auto receipt_time = std::chrono::steady_clock::now();
    if (last_reference_receipt_) {
      const double gap = std::chrono::duration<double>(
        receipt_time - *last_reference_receipt_).count();
      if (gap > 0.5) {
        RCLCPP_WARN(
          get_logger(), "Execution-reference callback gap was %.1f ms.", gap * 1000.0);
      }
    }
    last_reference_receipt_ = receipt_time;
    latest_reference_time_ = receipt_time;
    if (!message->inspection_enabled || message->segment_index < 0) {
      reference_.reset();
      // A transition/entry reference is not an inspection heartbeat. Publishing
      // a healthy heartbeat with the forthcoming SCAN's identity here would
      // make the tracker treat that segment as capture-capable for
      // capture_gate_timeout_s.
      // If the following enabled reference is rejected (for example because
      // its optical offset is wrong), the robot could therefore drive before
      // the missing-heartbeat safeguard took effect.
      return;
    }
    const double dx = message->end.x - message->start.x;
    const double dy = message->end.y - message->start.y;
    const double length = std::hypot(dx, dy);
    if (!std::isfinite(length) || length <= 1e-6) {
      RCLCPP_ERROR(get_logger(), "Rejected zero or non-finite execution reference.");
      reference_.reset();
      return;
    }
    if (!std::isfinite(message->detection_forward_offset) ||
      std::abs(message->detection_forward_offset - mount_x_) > 1e-4)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Task detection_forward_offset does not match the camera mount; automatic capture disabled.");
      reference_.reset();
      // Deliberately no heartbeat. The mismatch is a configuration fault that
      // cannot resolve while the task runs, so every exposure on this line
      // would be missed. Withholding the heartbeat makes the tracker's
      // supervision stop the SCAN in capture_gate_start_timeout_s instead of
      // driving the whole line and discovering the empty archive at finalization.
      return;
    }
    const Key key{message->task_id, message->revision, message->segment_index};
    if (!key_ || !(key == *key_)) {
      key_ = key;
      next_trigger_ = 0U;
      // The reference is the user-bounded base_link route.  The final target
      // intentionally remains one interval before its terminal pose: the
      // tracker may complete inside its endpoint tolerance, and asking the
      // camera for a frame exactly at that pose used to lose one frame per
      // SCAN.  This count/interval rule is mirrored by archive_core.py.
      first_trigger_ = message->detection_forward_offset;
      trigger_count_ = std::max(
        1U, static_cast<uint32_t>(std::ceil(length / spacing_)));
      trigger_interval_ = length / static_cast<double>(trigger_count_);
    } else if (reference_) {
      const double shift = std::hypot(
        message->start.x - reference_->start.x, message->start.y - reference_->start.y) +
        std::hypot(message->end.x - reference_->end.x, message->end.y - reference_->end.y);
      if (shift > 1e-4) {
        RCLCPP_ERROR(
          get_logger(),
          "Frozen SCAN reference changed after inspection was enabled; disabling segment.");
        disabled_key_ = key;
        reference_.reset();
        return;
      }
    }
    reference_ = *message;
    publishCaptureHeartbeat();
    tryTrigger();
  }

  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const auto receipt_time = std::chrono::steady_clock::now();
    if (last_odometry_receipt_) {
      const double gap = std::chrono::duration<double>(
        receipt_time - *last_odometry_receipt_).count();
      if (gap > 0.25) {
        RCLCPP_WARN(
          get_logger(), "Odometry callback gap was %.1f ms.", gap * 1000.0);
      }
    }
    last_odometry_receipt_ = receipt_time;
    const double t = stampSeconds(message->header.stamp);
    if (!std::isfinite(t) || !std::isfinite(message->pose.pose.position.x) ||
      !std::isfinite(message->pose.pose.position.y))
    {
      return;
    }
    if (!poses_.empty() && t <= stampSeconds(poses_.back()->header.stamp)) {
      if (t < stampSeconds(poses_.back()->header.stamp)) {
        poses_.clear();
      } else {
        poses_.back() = message;
        return;
      }
    }
    poses_.push_back(message);
    while (poses_.size() > 2U && t - stampSeconds(poses_.front()->header.stamp) > cache_duration_) {
      poses_.pop_front();
    }
    resolvePending();
    tryTrigger();
  }

  double cameraProgress(const nav_msgs::msg::Odometry & odometry) const
  {
    const double heading = yaw(odometry.pose.pose.orientation);
    const double camera_x = odometry.pose.pose.position.x +
      std::cos(heading) * mount_x_ - std::sin(heading) * mount_y_;
    const double camera_y = odometry.pose.pose.position.y +
      std::sin(heading) * mount_x_ + std::cos(heading) * mount_y_;
    const double dx = reference_->end.x - reference_->start.x;
    const double dy = reference_->end.y - reference_->start.y;
    const double length = std::hypot(dx, dy);
    return ((camera_x - reference_->start.x) * dx +
           (camera_y - reference_->start.y) * dy) / length;
  }

  void tryTrigger()
  {
    if (!reference_ || pending_ || poses_.empty() || !key_ ||
      (disabled_key_ && *disabled_key_ == *key_) ||
      !latest_reference_time_ ||
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - *latest_reference_time_).count() > reference_timeout_ ||
      next_trigger_ >= trigger_count_ || !capture_client_->service_is_ready())
    {
      return;
    }
    const double progress = cameraProgress(*poses_.back());
    const double target = first_trigger_ + trigger_interval_ * static_cast<double>(next_trigger_);
    if (!std::isfinite(progress) || progress + 1e-6 < target) {
      return;
    }
    const double odometry_age_ms = 1000.0 * (
      now().seconds() - stampSeconds(poses_.back()->header.stamp));
    const double trigger_lateness = progress - target;
    const auto log_trigger_decision = [this, next_trigger = next_trigger_, progress, target,
        trigger_lateness, odometry_age_ms](bool warning) {
        const char * format =
          "Capture trigger %u decision: progress %.4f m, target %.4f m, late %.1f mm, "
          "odometry simulation age %.1f ms.";
        if (warning) {
          RCLCPP_WARN(
            get_logger(), format, next_trigger, progress, target,
            trigger_lateness * 1000.0, odometry_age_ms);
        } else {
          RCLCPP_INFO(
            get_logger(), format, next_trigger, progress, target,
            trigger_lateness * 1000.0, odometry_age_ms);
        }
      };
    if (trigger_lateness > 0.005 || odometry_age_ms > 50.0) {
      log_trigger_decision(true);
    } else if (next_trigger_ == 0U) {
      log_trigger_decision(false);
    }
    Pending pending;
    pending.requested = std::chrono::steady_clock::now();
    pending.request_id = ++next_request_id_;
    pending.metadata.task_id = key_->task_id;
    pending.metadata.revision = key_->revision;
    pending.metadata.segment_index = key_->segment;
    pending.metadata.trigger_index = next_trigger_;
    pending.metadata.target_along_track = target;
    pending.metadata.actual_along_track = progress;
    pending.metadata.reference_start = reference_->start;
    pending.metadata.reference_end = reference_->end;
    pending_ = pending;
    ++next_trigger_;
    auto request = std::make_shared<CaptureOnce::Request>();
    const uint64_t request_id = pending.request_id;
    capture_client_->async_send_request(
      request, [this, request_id](rclcpp::Client<CaptureOnce>::SharedFuture future) {
        const auto response = future.get();
        if (!pending_ || pending_->request_id != request_id) {
          return;
        }
        pending_->response_received = std::chrono::steady_clock::now();
        pending_->response_succeeded = response->success;
        if (!response->success && pending_ && !pending_->image_stamp) {
          RCLCPP_WARN(get_logger(), "Automatic capture rejected (reason %u): %s",
          response->reason, response->message.c_str());
          // A normal exposure never controls travel speed. Once a trigger is
          // rejected, however, the robot may already be past its target, so a
          // blind retry would create an unbounded coverage gap. Withhold the
          // heartbeat and let the tracker fail the active SCAN instead.
          const Key request_key{
            pending_->metadata.task_id,
            pending_->metadata.revision,
            pending_->metadata.segment_index};
          pending_.reset();
          if (key_ && request_key == *key_) {
            disabled_key_ = request_key;
            reference_.reset();
          }
        }
      });
  }

  void onReceipt(const std_msgs::msg::Header::SharedPtr message)
  {
    if (!pending_ || pending_->image_stamp) {
      return;
    }
    pending_->metadata.header = *message;
    pending_->image_stamp = rclcpp::Time(message->stamp, get_clock()->get_clock_type());
    const double request_to_image = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - pending_->requested).count();
    if (request_to_image > slow_capture_warning_) {
      const double response_ms = pending_->response_received ?
        1000.0 * std::chrono::duration<double>(
        *pending_->response_received - pending_->requested).count() : -1.0;
      RCLCPP_WARN(
        get_logger(),
        "Slow capture trigger %u: completion receipt after %.1f ms (service response %.1f ms).",
        pending_->metadata.trigger_index, request_to_image * 1000.0, response_ms);
    }
    // The image may arrive after both surrounding EKF samples. Resolve here
    // as well as from onOdometry(), otherwise no later odometry is available
    // to complete a valid interpolation bracket.
    resolvePending();
  }

  std::optional<nav_msgs::msg::Odometry> interpolatedPose(const rclcpp::Time & stamp) const
  {
    const double target = stamp.seconds();
    for (std::size_t index = 1U; index < poses_.size(); ++index) {
      const double first_t = stampSeconds(poses_[index - 1U]->header.stamp);
      const double second_t = stampSeconds(poses_[index]->header.stamp);
      if (target < first_t || target > second_t || second_t <= first_t) {
        continue;
      }
      const double ratio = (target - first_t) / (second_t - first_t);
      nav_msgs::msg::Odometry result = *poses_[index - 1U];
      result.header.stamp = stamp;
      auto & pose = result.pose.pose;
      const auto & a = poses_[index - 1U]->pose.pose;
      const auto & b = poses_[index]->pose.pose;
      pose.position.x = a.position.x + ratio * (b.position.x - a.position.x);
      pose.position.y = a.position.y + ratio * (b.position.y - a.position.y);
      pose.position.z = a.position.z + ratio * (b.position.z - a.position.z);
      const double angle = yaw(a.orientation) + ratio *
        wrap(yaw(b.orientation) - yaw(a.orientation));
      pose.orientation = rpyQuaternion(0.0, 0.0, angle);
      for (std::size_t i = 0U; i < result.pose.covariance.size(); ++i) {
        result.pose.covariance[i] = poses_[index - 1U]->pose.covariance[i] + ratio *
          (poses_[index]->pose.covariance[i] - poses_[index - 1U]->pose.covariance[i]);
      }
      return result;
    }
    return std::nullopt;
  }

  void resolvePending()
  {
    if (!pending_ || !pending_->image_stamp) {
      return;
    }
    const auto interpolated = interpolatedPose(*pending_->image_stamp);
    if (!interpolated) {
      return;
    }
    auto & output = pending_->metadata.camera_pose;
    const auto & base = interpolated->pose.pose;
    const double heading = yaw(base.orientation);
    pending_->metadata.wall_heading_rad = heading;
    output.pose.position.x = base.position.x +
      std::cos(heading) * mount_x_ - std::sin(heading) * mount_y_;
    output.pose.position.y = base.position.y +
      std::sin(heading) * mount_x_ + std::cos(heading) * mount_y_;
    // This controller's wall pose is planar: x/y/yaw describe base_link on
    // the work plane.  The EKF's z can contain the Gazebo canonical-link
    // height, which is not an extra camera standoff and must not be added to
    // the frozen optical mount.  A future full-SE(3) real-robot pipeline needs
    // an explicit base-frame transform rather than changing this 2-D contract.
    output.pose.position.z = mount_z_;
    output.pose.orientation = multiply(
      base.orientation,
      rpyQuaternion(mount_roll_, mount_pitch_, mount_yaw_));
    const double reference_dx =
      pending_->metadata.reference_end.x - pending_->metadata.reference_start.x;
    const double reference_dy =
      pending_->metadata.reference_end.y - pending_->metadata.reference_start.y;
    const double reference_length = std::hypot(reference_dx, reference_dy);
    pending_->metadata.actual_along_track =
      ((output.pose.position.x - pending_->metadata.reference_start.x) * reference_dx +
      (output.pose.position.y - pending_->metadata.reference_start.y) * reference_dy) /
      reference_length;

    // Propagate the 6x6 base covariance through the planar camera lever arm.
    std::array<double, 36> jacobian{};
    for (std::size_t i = 0U; i < 6U; ++i) {
      jacobian[i * 6U + i] = 1.0;
    }
    // Camera wall distance is the frozen planar mount, independent of EKF z.
    jacobian[2U * 6U + 2U] = 0.0;
    jacobian[0U * 6U + 5U] =
      -std::sin(heading) * mount_x_ - std::cos(heading) * mount_y_;
    jacobian[1U * 6U + 5U] =
      std::cos(heading) * mount_x_ - std::sin(heading) * mount_y_;
    std::array<double, 36> intermediate{};
    for (std::size_t row = 0U; row < 6U; ++row) {
      for (std::size_t column = 0U; column < 6U; ++column) {
        for (std::size_t inner = 0U; inner < 6U; ++inner) {
          intermediate[row * 6U + column] += jacobian[row * 6U + inner] *
            interpolated->pose.covariance[inner * 6U + column];
        }
      }
    }
    for (std::size_t row = 0U; row < 6U; ++row) {
      for (std::size_t column = 0U; column < 6U; ++column) {
        for (std::size_t inner = 0U; inner < 6U; ++inner) {
          output.covariance[row * 6U + column] +=
            intermediate[row * 6U + inner] * jacobian[column * 6U + inner];
        }
      }
    }
    metadata_publisher_->publish(pending_->metadata);
    pending_.reset();
    // Deliberately no heartbeat here. The gate must age on the same clock as
    // the execution reference that produces it: a capture completing after the
    // reference stream stalled would otherwise reset the gate while the
    // reference kept ageing, and the tracker would stop the robot later than
    // this node stopped triggering. The only cost is that the gate's reason
    // text stays "in flight" until the next reference, at most one heartbeat.
    tryTrigger();
  }

  void checkTimeouts()
  {
    if (!pending_) {
      // The retry delay is wall-clock based; a paused simulation must not
      // permanently strand an exposure after a temporary camera response.
      tryTrigger();
      return;
    }
    const double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - pending_->requested).count();
    if (!pending_->response_received && age > capture_response_timeout_) {
      RCLCPP_ERROR(
        get_logger(),
        "Capture service did not respond within %.0f ms for trigger %u; disabling segment.",
        capture_response_timeout_ * 1000.0, pending_->metadata.trigger_index);
      disableCurrentSegment();
      pending_.reset();
      return;
    }
    if (pending_->response_succeeded && !pending_->image_stamp &&
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - *pending_->response_received).count() >
      image_wait_timeout_)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Capture service responded successfully for trigger %u but its inspection image did "
        "not arrive within %.0f ms; disabling segment.",
        pending_->metadata.trigger_index, image_wait_timeout_ * 1000.0);
      disableCurrentSegment();
      pending_.reset();
      return;
    }
    if (pending_->image_stamp && age > pose_wait_timeout_) {
      RCLCPP_ERROR(
        get_logger(),
        "No EKF interpolation bracket arrived for an inspection image; segment disabled.");
      disableCurrentSegment();
      pending_.reset();
    }
  }

  void disableCurrentSegment()
  {
    if (key_) {
      disabled_key_ = *key_;
    }
    reference_.reset();
  }

  void publishCaptureHeartbeat()
  {
    if (!reference_ || !key_ || (disabled_key_ && *disabled_key_ == *key_)) {
      return;
    }
    // This is a health heartbeat, not a per-exposure motion barrier. Normal
    // captures stay asynchronous so a regular scan remains at cruise speed.
    // A concrete capture fault instead stops this heartbeat and is handled by
    // the tracker's existing missing-heartbeat safety stop.
    if (pending_) {
      publishInactiveGate(key_, "inspection capture in flight");
    } else if (next_trigger_ >= trigger_count_) {
      publishInactiveGate(key_, "all captures for frozen SCAN reference are complete");
    } else {
      publishInactiveGate(key_, "inspection capture ready");
    }
  }

  /// Publish the heartbeat for one identified SCAN.
  void publishInactiveGate(const std::optional<Key> & identity, const std::string & reason)
  {
    CaptureGate gate;
    gate.header.stamp = now();
    if (identity) {
      gate.task_id = identity->task_id;
      gate.revision = identity->revision;
      gate.segment_index = identity->segment;
    }
    gate.active = false;
    gate.reason = reason;
    gate_publisher_->publish(gate);
  }

  const double footprint_length_;
  const double overlap_;
  const double mount_x_, mount_y_, mount_z_;
  const double mount_roll_, mount_pitch_, mount_yaw_;
  const double reference_timeout_, capture_response_timeout_, image_wait_timeout_;
  const double slow_capture_warning_, pose_wait_timeout_, cache_duration_;
  double spacing_{};
  std::optional<Key> key_, disabled_key_;
  std::optional<Reference> reference_;
  std::optional<Pending> pending_;
  uint32_t next_trigger_{};
  uint64_t next_request_id_{};
  uint32_t trigger_count_{};
  double trigger_interval_{};
  double first_trigger_{};
  std::optional<std::chrono::steady_clock::time_point> latest_reference_time_;
  std::optional<std::chrono::steady_clock::time_point> last_reference_receipt_;
  std::optional<std::chrono::steady_clock::time_point> last_odometry_receipt_;
  std::deque<nav_msgs::msg::Odometry::SharedPtr> poses_;
  rclcpp::Subscription<Reference>::SharedPtr reference_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr receipt_subscription_;
  rclcpp::Client<CaptureOnce>::SharedPtr capture_client_;
  rclcpp::Publisher<Metadata>::SharedPtr metadata_publisher_;
  rclcpp::Publisher<CaptureGate>::SharedPtr gate_publisher_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<AutomaticCaptureNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("automatic_capture_node"), "Node failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
