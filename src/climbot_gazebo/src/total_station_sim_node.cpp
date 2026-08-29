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

// Derive a delayed, noisy total-station position from Gazebo truth.
//
// Replaces the Python node of the same name. Its 200 Hz delivery timer made it
// the one genuine hot path among the remaining Python nodes, and it needs the
// simulation clock, so it could not be taken off that path by configuration
// alone the way camera_distortion_adapter could.
//
// The wall transform is climbot_description::WallFrame, the same
// implementation the Python package binds to, so no convention is duplicated.
//
// The noise stream is NOT the Python one. Python drew from Mersenne Twister
// through random.Random.gauss(); this draws from std::mt19937 through
// std::normal_distribution. Distributions, independence structure and seed
// parameters are preserved -- drops and position noise share one stream and
// timestamp jitter keeps its own -- but a given seed no longer reproduces the
// earlier sample values, so archived runs are not sample-for-sample comparable
// across this change.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "climbot_description/geometry.hpp"
#include "climbot_description/wall_frame.hpp"
#include "climbot_gazebo/total_station_model.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

double finite(const rclcpp::Node & node, const std::string & name, double value)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument(name + " must be finite");
  }
  (void)node;
  return value;
}

class TotalStationSimulator : public rclcpp::Node
{
public:
  TotalStationSimulator()
  : Node("total_station_sim")
  {
    rate_ = finite(*this, "publish_rate_hz", declare_parameter("publish_rate_hz", 12.0));
    stddev_ = finite(
      *this, "position_stddev_m", declare_parameter("position_stddev_m", 0.001));
    const double delay_s = finite(
      *this, "fixed_delay_s", declare_parameter("fixed_delay_s", 0.01));
    drop_probability_ = finite(
      *this, "drop_probability", declare_parameter("drop_probability", 0.0));
    profile_ = declare_parameter("localization_profile", std::string("precision"));
    prism_error_enabled_ = declare_parameter("prism_extrinsic_error_enabled", false);
    const auto prism = declare_parameter(
      "prism_extrinsic_error_robot_m", std::vector<double>{0.020, -0.010, 0.0});
    timestamp_error_enabled_ = declare_parameter(
      "measurement_timestamp_error_enabled", false);
    timestamp_bias_s_ = finite(
      *this, "measurement_timestamp_bias_s",
      declare_parameter("measurement_timestamp_bias_s", 0.020));
    timestamp_jitter_stddev_s_ = finite(
      *this, "measurement_timestamp_jitter_stddev_s",
      declare_parameter("measurement_timestamp_jitter_stddev_s", 0.002));
    frame_id_ = declare_parameter("frame_id", std::string("odom"));
    const auto wall_config = declare_parameter("wall_config", std::string(""));

    if (rate_ <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be positive.");
    }
    if (stddev_ < 0.0) {
      throw std::invalid_argument("position_stddev_m cannot be negative.");
    }
    if (delay_s < 0.0) {
      throw std::invalid_argument("fixed_delay_s cannot be negative.");
    }
    if (drop_probability_ < 0.0 || drop_probability_ > 1.0) {
      throw std::invalid_argument("drop_probability must be within [0, 1].");
    }
    if (!climbot_gazebo::isLocalizationProfile(profile_)) {
      throw std::invalid_argument("localization_profile must be precision or realistic.");
    }
    if (prism.size() != 3U) {
      throw std::invalid_argument("prism_extrinsic_error_robot_m needs three values.");
    }
    for (std::size_t index = 0; index < prism.size(); ++index) {
      prism_error_robot_[index] = finite(
        *this, "prism_extrinsic_error_robot_m", prism[index]);
    }
    if (timestamp_jitter_stddev_s_ < 0.0) {
      throw std::invalid_argument(
              "measurement_timestamp_jitter_stddev_s cannot be negative.");
    }
    if (wall_config.empty()) {
      throw std::invalid_argument("wall_config must name the shared wall description.");
    }

    delay_ns_ = static_cast<int64_t>(delay_s * 1.0e9);
    // Two independent streams, as in the Python node: enabling timestamp
    // jitter must not shift the position samples a precision run would draw.
    noise_engine_.seed(
      static_cast<std::mt19937::result_type>(declare_parameter("random_seed", 42)));
    jitter_engine_.seed(
      static_cast<std::mt19937::result_type>(
        declare_parameter("measurement_timestamp_jitter_seed", 20260827)));
    wall_frame_ = std::make_unique<climbot_description::WallFrame>(
      climbot_description::WallFrame::fromYaml(wall_config));

    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/model/climbot/ground_truth", 20,
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {latest_truth_ = message;});
    publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/total_station/pose", 20);
    // create_timer, never create_wall_timer: both of these follow the
    // simulation clock, as the rclpy node this replaces did. On wall time the
    // station would keep sampling a frozen truth while the simulator is
    // paused, would sample at 12 Hz of wall time rather than of simulation
    // time whenever the real-time factor is not 1.0, and would stamp repeat
    // observations with one frozen source time.
    sample_timer_ = create_timer(
      std::chrono::duration<double>(1.0 / rate_), [this]() {sampleTruth();});
    // Delivery stays a 200 Hz poll of the pending queue, matching the node
    // this replaces. Scheduling each measurement on its own one-shot timer
    // would trade a known, uniform poll for per-measurement timer jitter.
    delivery_timer_ = create_timer(
      std::chrono::milliseconds(5), [this]() {publishDueMeasurements();});
  }

private:
  void sampleTruth()
  {
    if (!latest_truth_) {
      return;
    }
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    if (unit(noise_engine_) < drop_probability_) {
      return;
    }
    const auto source = latest_truth_;
    geometry_msgs::msg::PoseWithCovarianceStamped observation;
    const int64_t source_ns =
      static_cast<int64_t>(source->header.stamp.sec) * 1000000000LL +
      source->header.stamp.nanosec;
    int64_t stamped_ns = source_ns;
    if (timestamp_error_enabled_) {
      std::normal_distribution<double> jitter(0.0, timestamp_jitter_stddev_s_);
      stamped_ns = climbot_gazebo::timestampWithClockErrorNs(
        source_ns, timestamp_bias_s_ + jitter(jitter_engine_));
    }
    observation.header.stamp.sec = static_cast<int32_t>(stamped_ns / 1000000000LL);
    observation.header.stamp.nanosec = static_cast<uint32_t>(stamped_ns % 1000000000LL);
    observation.header.frame_id = frame_id_;

    const auto & truth = source->pose.pose.position;
    const climbot_description::Vector3 wall = wall_frame_->positionFromWorld(
      climbot_description::Vector3{truth.x, truth.y, truth.z});
    std::array<double, 3> residual_wall{0.0, 0.0, 0.0};
    if (prism_error_enabled_) {
      const auto & rotation = source->pose.pose.orientation;
      const double yaw = climbot_description::yawFromQuaternion(
        wall_frame_->orientationFromWorld(
          climbot_description::Quaternion{rotation.x, rotation.y, rotation.z, rotation.w}));
      residual_wall = climbot_gazebo::rotateRobotResidualToWall(prism_error_robot_, yaw);
    }
    std::normal_distribution<double> noise(0.0, stddev_);
    observation.pose.pose.position.x = wall.x + residual_wall[0] + noise(noise_engine_);
    observation.pose.pose.position.y = wall.y + residual_wall[1] + noise(noise_engine_);
    observation.pose.pose.position.z = wall.z + residual_wall[2] + noise(noise_engine_);
    // Identity rather than an all-zero quaternion, which is not a rotation.
    observation.pose.pose.orientation.w = 1.0;

    const double variance = stddev_ * stddev_;
    observation.pose.covariance.fill(0.0);
    observation.pose.covariance[0] = variance;
    observation.pose.covariance[7] = variance;
    observation.pose.covariance[14] = variance;
    // Orientation is intentionally unobserved by the total station.
    observation.pose.covariance[21] = 1e6;
    observation.pose.covariance[28] = 1e6;
    observation.pose.covariance[35] = 1e6;

    // Delivery remains a separately known transport delay from the truth
    // sample time. A clock-error stamp must not alter when a packet arrives.
    pending_.emplace_back(source_ns + delay_ns_, std::move(observation));
  }

  void publishDueMeasurements()
  {
    const int64_t now = get_clock()->now().nanoseconds();
    while (!pending_.empty() && pending_.front().first <= now) {
      publisher_->publish(pending_.front().second);
      pending_.pop_front();
    }
  }

  double rate_{};
  double stddev_{};
  double drop_probability_{};
  std::string profile_;
  std::string frame_id_;
  bool prism_error_enabled_{};
  bool timestamp_error_enabled_{};
  double timestamp_bias_s_{};
  double timestamp_jitter_stddev_s_{};
  std::array<double, 3> prism_error_robot_{};
  int64_t delay_ns_{};
  std::mt19937 noise_engine_;
  std::mt19937 jitter_engine_;
  std::unique_ptr<climbot_description::WallFrame> wall_frame_;
  nav_msgs::msg::Odometry::SharedPtr latest_truth_;
  std::deque<std::pair<int64_t, geometry_msgs::msg::PoseWithCovarianceStamped>> pending_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr sample_timer_;
  rclcpp::TimerBase::SharedPtr delivery_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TotalStationSimulator>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("total_station_sim"), "Node failed: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
