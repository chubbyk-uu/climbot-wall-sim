#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "climbot_coverage/coverage_geometry.hpp"
#include "climbot_interfaces/msg/coverage_task.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace climbot_coverage
{
namespace
{

geometry_msgs::msg::Point markerPoint(const Point2 & point, double height)
{
  geometry_msgs::msg::Point output;
  output.x = point.x;
  output.y = point.y;
  output.z = height;
  return output;
}

geometry_msgs::msg::Point32 polygonPoint(const Point2 & point)
{
  geometry_msgs::msg::Point32 output;
  output.x = static_cast<float>(point.x);
  output.y = static_cast<float>(point.y);
  return output;
}

geometry_msgs::msg::Quaternion yawQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion output;
  output.z = std::sin(0.5 * yaw);
  output.w = std::cos(0.5 * yaw);
  return output;
}

visualization_msgs::msg::Marker lineMarker(
  const Polygon & polygon, const std_msgs::msg::Header & header, int id,
  const std::string & marker_namespace, float red, float green, float blue,
  double height)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = marker_namespace;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.035;
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = 1.0F;
  for (const auto & point : polygon) {
    marker.points.push_back(markerPoint(point, height));
  }
  if (!polygon.empty()) {
    marker.points.push_back(markerPoint(polygon.front(), height));
  }
  return marker;
}

}  // namespace

class CoveragePlannerNode : public rclcpp::Node
{
public:
  CoveragePlannerNode()
  : Node("coverage_planner")
  {
    frame_id_ = declare_parameter("frame_id", "odom");
    region_type_ = declare_parameter("region_type", "rectangle");
    input_mode_ = declare_parameter("input_mode", "parameters");
    sweep_direction_ = declare_parameter("sweep_direction", "horizontal");
    start_corner_ = declare_parameter("start_corner", "lower_left");
    lower_left_ = pointParameter("lower_left", {-3.0, 0.5});
    upper_right_ = pointParameter("upper_right", {3.0, 6.5});
    lower_right_ = pointParameter("lower_right", {3.0, 0.5});
    task_id_ = declare_parameter("task_id", "coverage-task");
    detection_width_ = declare_parameter("detection_width", 0.50);
    detection_length_ = declare_parameter("detection_length", 0.01);
    overlap_ratio_ = declare_parameter("overlap_ratio", 0.20);
    robot_length_ = declare_parameter("robot_length", -1.0);
    robot_width_ = declare_parameter("robot_width", -1.0);
    edge_clearance_ = declare_parameter("edge_clearance", -1.0);
    wall_width_ = declare_parameter("wall_width", -1.0);
    wall_height_ = declare_parameter("wall_height", -1.0);
    path_height_ = declare_parameter("path_height", 0.06);
    bottom_warning_tolerance_ = declare_parameter("bottom_warning_tolerance", 0.05);
    validatePhysicalParameters();
    row_spacing_ = detection_width_ * (1.0 - overlap_ratio_);
    // The robot turns in place at waypoints, so the inset must contain its
    // footprint at every yaw, not only while aligned with a scan line.
    safety_margin_ = 0.5 * std::hypot(robot_length_, robot_width_) + edge_clearance_;

    path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/coverage/path", rclcpp::QoS(1).reliable().transient_local());
    task_publisher_ = create_publisher<climbot_interfaces::msg::CoverageTask>(
      "/coverage/task", rclcpp::QoS(1).reliable().transient_local());
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/coverage/markers", rclcpp::QoS(1).transient_local());
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/coverage/status", rclcpp::QoS(1).transient_local());
    clicked_point_subscription_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10,
      std::bind(&CoveragePlannerNode::clickedPointCallback, this, std::placeholders::_1));
    clear_service_ = create_service<std_srvs::srv::Trigger>(
      "/coverage/clear_points",
      std::bind(
        &CoveragePlannerNode::clearPoints, this, std::placeholders::_1,
        std::placeholders::_2));
    replan_service_ = create_service<std_srvs::srv::Trigger>(
      "/coverage/replan",
      std::bind(
        &CoveragePlannerNode::replan, this, std::placeholders::_1,
        std::placeholders::_2));

    if (input_mode_ == "parameters") {
      planFromPoints();
    } else {
      publishStatus("Waiting for RViz points: A=lower-left, B=upper-right" +
        std::string(region_type_ == "trapezoid" ? ", C=lower-right." : "."));
      publishTask(emptyTask());
      publishMarkers({}, {}, {});
    }
  }

private:
  void validatePhysicalParameters() const
  {
    if (task_id_.empty()) {
      throw std::invalid_argument("task_id cannot be empty.");
    }
    if (detection_width_ <= 0.0 || detection_length_ <= 0.0) {
      throw std::invalid_argument("Detection footprint dimensions must be positive.");
    }
    if (overlap_ratio_ < 0.0 || overlap_ratio_ >= 1.0) {
      throw std::invalid_argument("overlap_ratio must be within [0, 1).");
    }
    if (robot_length_ <= 0.0 || robot_width_ <= 0.0 || edge_clearance_ < 0.0) {
      throw std::invalid_argument(
              "Robot dimensions must be positive and edge_clearance non-negative.");
    }
    if (wall_width_ <= 0.0 || wall_height_ <= 0.0) {
      throw std::invalid_argument("Wall dimensions must be positive.");
    }
    if (sweep_direction_ != "horizontal" && sweep_direction_ != "vertical") {
      throw std::invalid_argument("Sweep direction must be horizontal or vertical.");
    }
  }

  Point2 pointParameter(const std::string & name, const std::vector<double> & default_value)
  {
    const auto values = declare_parameter(name, default_value);
    if (values.size() != 2) {
      throw std::invalid_argument(name + " must contain exactly two coordinates.");
    }
    return {values[0], values[1]};
  }

  void publishStatus(const std::string & text)
  {
    std_msgs::msg::String message;
    message.data = text;
    status_publisher_->publish(message);
    RCLCPP_INFO(get_logger(), "%s", text.c_str());
  }

  void clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr message)
  {
    if (input_mode_ != "rviz") {
      return;
    }
    if (message->header.frame_id != frame_id_) {
      publishStatus(
        "Rejected clicked point: expected frame " + frame_id_ + ", received " +
        message->header.frame_id + ".");
      return;
    }
    const std::size_t required_points = region_type_ == "trapezoid" ? 3U : 2U;
    if (clicked_points_.size() >= required_points) {
      clicked_points_.clear();
    }
    clicked_points_.push_back({message->point.x, message->point.y});
    publishTask(emptyTask());
    publishMarkers({}, {}, {});
    // The coordinates are logged so a mirrored or rotated RViz camera is
    // visible immediately instead of surfacing later as a geometry error.
    const std::vector<std::string> roles{"A lower-left", "B upper-right", "C lower-right"};
    std::ostringstream accepted;
    accepted << "Accepted point " << clicked_points_.size() << " of " << required_points <<
      " (" << roles[clicked_points_.size() - 1U] << ") at " << frame_id_ << " (" <<
      message->point.x << ", " << message->point.y << ").";
    publishStatus(accepted.str());
    if (clicked_points_.size() == required_points) {
      lower_left_ = clicked_points_[0];
      upper_right_ = clicked_points_[1];
      if (required_points == 3U) {
        lower_right_ = clicked_points_[2];
      }
      planFromPoints();
    }
  }

  void clearPoints(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    clicked_points_.clear();
    publishTask(emptyTask());
    publishMarkers({}, {}, {});
    response->success = true;
    response->message = "Clicked points cleared.";
    publishStatus(response->message);
  }

  void replan(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    response->success = planFromPoints();
    response->message = response->success ? "Coverage path regenerated." : last_error_;
  }

  bool planFromPoints()
  {
    try {
      RegionResult region;
      if (region_type_ == "rectangle") {
        region = makeRectangle(lower_left_, upper_right_);
      } else if (region_type_ == "trapezoid") {
        region = makeIsoscelesTrapezoid(lower_left_, upper_right_, lower_right_);
      } else {
        throw std::invalid_argument("region_type must be rectangle or trapezoid.");
      }
      const auto motion = motionRegion();
      const auto path = generateFootprintAwareBoustrophedonPath(
        region.polygon, motion, detection_width_, detection_length_, row_spacing_,
        sweep_direction_, start_corner_);
      const double coverage_ratio = sampledCoverageRatio(
        region.polygon, path, detection_width_, detection_length_);
      if (coverage_ratio < 0.98) {
        throw std::invalid_argument("Nominal detection footprint covers less than 98 percent.");
      }
      publishTask(makeTask(region.polygon, motion, path));
      publishMarkers(region.polygon, motion, path);
      std::ostringstream status;
      status << "Generated " << sweep_direction_ << " " << region_type_ <<
        " coverage path with " << path.size() << " waypoints; row spacing <= " <<
        row_spacing_ << " m, nominal footprint coverage " << coverage_ratio * 100.0 <<
        "% and safety margin " << safety_margin_ << " m.";
      if (region.bottom_height_correction > bottom_warning_tolerance_) {
        status << " Bottom clicks differed by " << region.bottom_height_correction <<
          " m and were corrected to their mean height.";
      }
      last_error_.clear();
      publishStatus(status.str());
      return true;
    } catch (const std::exception & exception) {
      last_error_ = exception.what();
      publishTask(emptyTask());
      publishMarkers({}, {}, {});
      publishStatus("Planning failed: " + last_error_);
      return false;
    }
  }

  climbot_interfaces::msg::CoverageTask emptyTask()
  {
    climbot_interfaces::msg::CoverageTask task;
    task.header.stamp = now();
    task.header.frame_id = frame_id_;
    task.task_id = task_id_;
    task.revision = ++revision_;
    task.sweep_direction = sweep_direction_ == "horizontal" ?
      climbot_interfaces::msg::CoverageTask::SWEEP_HORIZONTAL :
      climbot_interfaces::msg::CoverageTask::SWEEP_VERTICAL;
    task.detection_width = detection_width_;
    task.detection_length = detection_length_;
    return task;
  }

  geometry_msgs::msg::Polygon polygonMessage(const Polygon & polygon) const
  {
    geometry_msgs::msg::Polygon message;
    for (const auto & point : polygon) {
      message.points.push_back(polygonPoint(point));
    }
    return message;
  }

  Polygon motionRegion() const
  {
    const RegionResult wall = makeRectangle(
      {-0.5 * wall_width_, 0.0}, {0.5 * wall_width_, wall_height_});
    return insetConvexPolygon(wall.polygon, safety_margin_);
  }

  climbot_interfaces::msg::CoverageTask makeTask(
    const Polygon & original, const Polygon & effective,
    const std::vector<Point2> & points)
  {
    auto task = emptyTask();
    task.coverage_region = polygonMessage(original);
    task.motion_region = polygonMessage(effective);
    for (std::size_t index = 0; index < points.size(); ++index) {
      const auto & point = points[index];
      geometry_msgs::msg::Pose pose;
      pose.position.x = point.x;
      pose.position.y = point.y;
      if (points.size() > 1U) {
        const std::size_t other = index + 1U < points.size() ? index + 1U : index - 1U;
        const double direction = index + 1U < points.size() ? 1.0 : -1.0;
        const double delta_x = direction * (points[other].x - point.x);
        const double delta_y = direction * (points[other].y - point.y);
        pose.orientation = yawQuaternion(std::atan2(delta_y, delta_x));
      } else {
        pose.orientation.w = 1.0;
      }
      task.waypoints.push_back(pose);
      if (index + 1U < points.size()) {
        task.segment_types.push_back(index % 2U == 0U ?
          climbot_interfaces::msg::CoverageTask::SEGMENT_SCAN :
          climbot_interfaces::msg::CoverageTask::SEGMENT_TRANSITION);
      }
    }
    return task;
  }

  void publishTask(const climbot_interfaces::msg::CoverageTask & task)
  {
    nav_msgs::msg::Path path;
    path.header = task.header;
    for (const auto & waypoint : task.waypoints) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose = waypoint;
      pose.pose.position.z = path_height_;
      path.poses.push_back(pose);
    }
    task_publisher_->publish(task);
    path_publisher_->publish(path);
  }

  void publishMarkers(
    const Polygon & original, const Polygon & effective,
    const std::vector<Point2> & path)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = frame_id_;

    visualization_msgs::msg::Marker wall;
    wall.header = header;
    wall.ns = "wall";
    wall.id = 0;
    wall.type = visualization_msgs::msg::Marker::CUBE;
    wall.action = visualization_msgs::msg::Marker::ADD;
    wall.pose.position.y = 0.5 * wall_height_;
    wall.pose.position.z = -0.02;
    wall.pose.orientation.w = 1.0;
    wall.scale.x = wall_width_;
    wall.scale.y = wall_height_;
    wall.scale.z = 0.02;
    wall.color.r = 0.45F;
    wall.color.g = 0.48F;
    wall.color.b = 0.52F;
    wall.color.a = 0.35F;
    markers.markers.push_back(wall);

    if (!original.empty()) {
      markers.markers.push_back(lineMarker(original, header, 0, "original", 1.0F, 0.45F, 0.05F,
          0.03));
      const std::vector<std::string> labels{"A", "C", "B", "D"};
      for (std::size_t index = 0; index < original.size(); ++index) {
        visualization_msgs::msg::Marker label;
        label.header = header;
        label.ns = "vertex_labels";
        label.id = static_cast<int>(index);
        label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.action = visualization_msgs::msg::Marker::ADD;
        label.pose.position = markerPoint(original[index], 0.12);
        label.pose.orientation.w = 1.0;
        label.scale.z = 0.22;
        label.color.r = 1.0F;
        label.color.g = 0.9F;
        label.color.b = 0.15F;
        label.color.a = 1.0F;
        label.text = labels[index];
        markers.markers.push_back(label);
      }
    }
    if (!effective.empty()) {
      markers.markers.push_back(lineMarker(effective, header, 0, "effective", 0.1F, 1.0F, 0.3F,
          0.04));
    }
    if (!path.empty()) {
      visualization_msgs::msg::Marker path_marker;
      path_marker.header = header;
      path_marker.ns = "coverage_path";
      path_marker.id = 0;
      path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      path_marker.action = visualization_msgs::msg::Marker::ADD;
      path_marker.pose.orientation.w = 1.0;
      path_marker.scale.x = 0.055;
      path_marker.color.r = 0.1F;
      path_marker.color.g = 0.55F;
      path_marker.color.b = 1.0F;
      path_marker.color.a = 1.0F;
      for (const auto & point : path) {
        path_marker.points.push_back(markerPoint(point, path_height_));
      }
      markers.markers.push_back(path_marker);
      for (std::size_t index = 0; index + 1 < path.size(); ++index) {
        visualization_msgs::msg::Marker arrow;
        arrow.header = header;
        arrow.ns = "path_direction";
        arrow.id = static_cast<int>(index);
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.pose.orientation.w = 1.0;
        arrow.scale.x = 0.025;
        arrow.scale.y = 0.07;
        arrow.scale.z = 0.10;
        arrow.color.r = 0.15F;
        arrow.color.g = 0.85F;
        arrow.color.b = 1.0F;
        arrow.color.a = 0.75F;
        arrow.points.push_back(markerPoint(path[index], path_height_ + 0.01));
        arrow.points.push_back(markerPoint(path[index + 1], path_height_ + 0.01));
        markers.markers.push_back(arrow);
      }
    }

    int point_id = 0;
    for (const auto & point : clicked_points_) {
      visualization_msgs::msg::Marker sphere;
      sphere.header = header;
      sphere.ns = "clicked_points";
      sphere.id = point_id++;
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose.position = markerPoint(point, 0.08);
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.12;
      sphere.color.r = 1.0F;
      sphere.color.g = 0.1F;
      sphere.color.b = 0.9F;
      sphere.color.a = 1.0F;
      markers.markers.push_back(sphere);
    }
    marker_publisher_->publish(markers);
  }

  std::string frame_id_;
  std::string task_id_;
  std::string region_type_;
  std::string input_mode_;
  std::string sweep_direction_;
  std::string start_corner_;
  Point2 lower_left_;
  Point2 upper_right_;
  Point2 lower_right_;
  double detection_width_;
  double detection_length_;
  double overlap_ratio_;
  double robot_length_;
  double robot_width_;
  double edge_clearance_;
  double wall_width_;
  double wall_height_;
  double safety_margin_;
  double row_spacing_;
  double path_height_;
  double bottom_warning_tolerance_;
  std::uint32_t revision_{0U};
  std::string last_error_;
  std::vector<Point2> clicked_points_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<climbot_interfaces::msg::CoverageTask>::SharedPtr task_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr replan_service_;
};

}  // namespace climbot_coverage

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<climbot_coverage::CoveragePlannerNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("coverage_planner"), "Failed to start: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
