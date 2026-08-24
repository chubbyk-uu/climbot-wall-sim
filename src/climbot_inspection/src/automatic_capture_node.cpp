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
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/trigger.hpp"

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

  AutomaticCaptureNode()
  : Node("automatic_capture_node"),
    footprint_length_(finitePositive(*this, "effective_length_m", 0.28125)),
    overlap_(finite(*this, "image_overlap_ratio", 0.25)),
    mount_x_(finite(*this, "camera_mount_x_m", 0.300)),
    mount_y_(finite(*this, "camera_mount_y_m", 0.0)),
    mount_z_(finite(*this, "camera_mount_z_m", 0.275)),
    mount_roll_(finite(*this, "camera_mount_roll_rad", std::acos(-1.0))),
    mount_pitch_(finite(*this, "camera_mount_pitch_rad", 0.0)),
    mount_yaw_(finite(*this, "camera_mount_yaw_rad", -0.5 * std::acos(-1.0))),
    reference_timeout_(finitePositive(*this, "reference_timeout_s", 0.5)),
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
    const auto image_topic = nonEmpty(*this, "image_topic", "/inspection/camera/image_raw");
    const auto service = nonEmpty(*this, "capture_service", "/inspection/capture_once");
    const auto metadata_topic = nonEmpty(
      *this, "metadata_topic", "/inspection/capture_metadata");

    reference_subscription_ = create_subscription<Reference>(
      reference_topic, rclcpp::QoS(10).reliable(),
      std::bind(&AutomaticCaptureNode::onReference, this, std::placeholders::_1));
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic, rclcpp::SensorDataQoS(),
      std::bind(&AutomaticCaptureNode::onOdometry, this, std::placeholders::_1));
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::QoS(1).reliable(),
      std::bind(&AutomaticCaptureNode::onImage, this, std::placeholders::_1));
    capture_client_ = create_client<std_srvs::srv::Trigger>(service);
    metadata_publisher_ = create_publisher<Metadata>(metadata_topic, rclcpp::QoS(10).reliable());
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
    rclcpp::Time requested;
    std::optional<rclcpp::Time> image_stamp;
  };

  void onReference(const Reference::SharedPtr message)
  {
    latest_reference_time_ = now();
    if (!message->inspection_enabled || message->segment_index < 0) {
      reference_.reset();
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
      return;
    }
    const Key key{message->task_id, message->revision, message->segment_index};
    if (!key_ || !(key == *key_)) {
      key_ = key;
      next_trigger_ = 0U;
      trigger_count_ = std::max<uint32_t>(
        1U, static_cast<uint32_t>(std::ceil(length / spacing_))) + 1U;
      trigger_interval_ = length / static_cast<double>(trigger_count_ - 1U);
      first_trigger_ = message->detection_forward_offset;
    } else if (reference_) {
      const double shift = std::hypot(
        message->start.x - reference_->start.x, message->start.y - reference_->start.y) +
        std::hypot(message->end.x - reference_->end.x, message->end.y - reference_->end.y);
      if (shift > 1e-4) {
        RCLCPP_ERROR(
          get_logger(),
          "Frozen SCAN reference changed after inspection was enabled; disabling segment.");
        disabled_key_ = key;
      }
    }
    reference_ = *message;
    tryTrigger();
  }

  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
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
      (now() - latest_reference_time_).seconds() > reference_timeout_ ||
      next_trigger_ >= trigger_count_ || !capture_client_->service_is_ready())
    {
      return;
    }
    const double progress = cameraProgress(*poses_.back());
    const double target = first_trigger_ + trigger_interval_ * static_cast<double>(next_trigger_);
    if (!std::isfinite(progress) || progress + 1e-6 < target) {
      return;
    }
    Pending pending;
    pending.requested = now();
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
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    capture_client_->async_send_request(
      request, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        if (!future.get()->success && pending_ && !pending_->image_stamp) {
          RCLCPP_WARN(get_logger(), "Automatic capture rejected: %s",
          future.get()->message.c_str());
          // A busy or warming camera has not consumed this spatial target.
          // Put the same number back rather than silently creating a hole.
          next_trigger_ = std::min(next_trigger_, pending_->metadata.trigger_index);
          if (future.get()->message.find("faulted") != std::string::npos ||
          future.get()->message.find("restart the node") != std::string::npos)
          {
            disabled_key_ = key_;
          }
          pending_.reset();
        }
      });
  }

  void onImage(const sensor_msgs::msg::Image::SharedPtr message)
  {
    if (!pending_ || pending_->image_stamp) {
      return;
    }
    pending_->metadata.header = message->header;
    pending_->image_stamp = rclcpp::Time(message->header.stamp, get_clock()->get_clock_type());
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
    output.pose.position.z = base.position.z + mount_z_;
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
    tryTrigger();
  }

  void checkTimeouts()
  {
    if (!pending_) {
      return;
    }
    const double age = (now() - pending_->requested).seconds();
    if (pending_->image_stamp && age > pose_wait_timeout_) {
      RCLCPP_ERROR(
        get_logger(),
        "No EKF interpolation bracket arrived for an inspection image; segment disabled.");
      disabled_key_ = key_;
      pending_.reset();
    }
  }

  const double footprint_length_;
  const double overlap_;
  const double mount_x_, mount_y_, mount_z_;
  const double mount_roll_, mount_pitch_, mount_yaw_;
  const double reference_timeout_, pose_wait_timeout_, cache_duration_;
  double spacing_{};
  std::optional<Key> key_, disabled_key_;
  std::optional<Reference> reference_;
  std::optional<Pending> pending_;
  uint32_t next_trigger_{};
  uint32_t trigger_count_{};
  double trigger_interval_{};
  double first_trigger_{};
  rclcpp::Time latest_reference_time_{0, 0, RCL_ROS_TIME};
  std::deque<nav_msgs::msg::Odometry::SharedPtr> poses_;
  rclcpp::Subscription<Reference>::SharedPtr reference_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr capture_client_;
  rclcpp::Publisher<Metadata>::SharedPtr metadata_publisher_;
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
