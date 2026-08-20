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
#include "climbot_interfaces/msg/coverage_config.hpp"
#include "climbot_interfaces/msg/coverage_task.hpp"
#include "climbot_interfaces/srv/configure_coverage.hpp"
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
    // Pitch of the reference grid drawn over the wall in RViz, matching the
    // one painted on the wall face in Gazebo. 0 publishes nothing; the
    // display can also be unticked in RViz, which is the live switch.
    wall_grid_spacing_ = declare_parameter("wall_grid_spacing", 1.0);
    path_height_ = declare_parameter("path_height", 0.06);
    bottom_warning_tolerance_ = declare_parameter("bottom_warning_tolerance", 0.05);
    // Above the acceptance gate by more than execution costs. Measured loss
    // from nominal to executed footprint coverage is 0.37 to 1.09 points
    // across the three baselines, against a 95 percent acceptance gate.
    minimum_nominal_coverage_ratio_ = declare_parameter(
      "minimum_nominal_coverage_ratio", 0.965);
    top_edge_scan_ = declare_parameter("top_edge_scan", "auto");
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
    // Its own topic rather than another namespace inside /coverage/markers:
    // the grid is scenery an operator turns on and off while looking at a
    // plan, and a display can only be ticked off as a whole.
    grid_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/coverage/wall_grid", rclcpp::QoS(1).transient_local());
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/coverage/status", rclcpp::QoS(1).transient_local());
    config_publisher_ = create_publisher<climbot_interfaces::msg::CoverageConfig>(
      "/coverage/config", rclcpp::QoS(1).reliable().transient_local());
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
    configure_service_ = create_service<climbot_interfaces::srv::ConfigureCoverage>(
      "/coverage/configure",
      std::bind(
        &CoveragePlannerNode::configure, this, std::placeholders::_1,
        std::placeholders::_2));
    publishConfig();
    publishWallGrid();

    if (input_mode_ == "parameters") {
      planFromPoints();
    } else {
      publishStatus("Waiting for RViz points: A=lower-left, B=upper-right" +
        std::string(region_type_ == "trapezoid" ? ", C=lower-right." : "."));
      publishTask(emptyTask());
      publishMarkers({}, {});
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
    if (minimum_nominal_coverage_ratio_ <= 0.0 ||
      minimum_nominal_coverage_ratio_ > 1.0)
    {
      throw std::invalid_argument(
              "minimum_nominal_coverage_ratio must be within (0, 1].");
    }
    if (top_edge_scan_ != "auto" && top_edge_scan_ != "always" &&
      top_edge_scan_ != "never")
    {
      throw std::invalid_argument("top_edge_scan must be auto, always, or never.");
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
    const std::size_t required_points = requiredPoints();
    if (clicked_points_.size() >= required_points) {
      clicked_points_.clear();
    }
    clicked_points_.push_back({message->point.x, message->point.y});
    publishTask(emptyTask());
    publishMarkers({}, {});
    // The coordinates are logged so a mirrored or rotated RViz camera is
    // visible immediately instead of surfacing later as a geometry error.
    const std::vector<std::string> roles{"A lower-left", "B upper-right", "C lower-right"};
    // requiredPoints() returns 2 or 3 today, so the index is always in range;
    // at() rather than [] so that a fourth shape needing a fourth point fails
    // where it is introduced instead of reading past this vector.
    const std::size_t role = clicked_points_.size() - 1U;
    std::ostringstream accepted;
    accepted << "Accepted point " << clicked_points_.size() << " of " << required_points <<
      " (" << (role < roles.size() ? roles.at(role) : std::string("unnamed")) <<
      ") at " << frame_id_ << " (" <<
      message->point.x << ", " << message->point.y << ").";
    publishStatus(accepted.str());
    if (clicked_points_.size() == required_points) {
      applySelectedPoints();
      planFromPoints();
    }
    publishConfig();
  }

  std::size_t requiredPoints() const
  {
    return region_type_ == "trapezoid" ? 3U : 2U;
  }

  // The single place that decides whether a replan can be accepted, so the
  // panel's greying, the service response and the replan guard can never
  // disagree with each other.
  std::string planBlockedReason() const
  {
    if (input_mode_ != "rviz") {
      return {};
    }
    const std::size_t required = requiredPoints();
    if (clicked_points_.size() < required) {
      std::ostringstream reason;
      reason << "Select " << (required - clicked_points_.size()) << " more point" <<
        (required - clicked_points_.size() == 1U ? "" : "s") << " for a " <<
        region_type_ << " (" << clicked_points_.size() << " of " << required << ").";
      return reason.str();
    }
    return {};
  }

  climbot_interfaces::msg::CoverageConfig currentConfig(const std::string & note) const
  {
    climbot_interfaces::msg::CoverageConfig config;
    config.header.stamp = now();
    config.header.frame_id = frame_id_;
    config.region_type = region_type_;
    config.sweep_direction = sweep_direction_;
    config.input_mode = input_mode_;
    config.required_points = static_cast<uint8_t>(requiredPoints());
    config.selected_points = static_cast<uint8_t>(
      std::min<std::size_t>(clicked_points_.size(), 255U));
    const std::string blocked = planBlockedReason();
    config.can_plan = blocked.empty();
    config.message = note.empty() ? blocked : note;
    return config;
  }

  void publishConfig(const std::string & note = {})
  {
    config_publisher_->publish(currentConfig(note));
  }

  void configure(
    const std::shared_ptr<climbot_interfaces::srv::ConfigureCoverage::Request> request,
    std::shared_ptr<climbot_interfaces::srv::ConfigureCoverage::Response> response)
  {
    // An empty field means "leave this one alone", so a panel changing only the
    // sweep direction cannot clobber a shape someone else just set.
    const std::string region = request->region_type.empty() ?
      region_type_ : request->region_type;
    const std::string sweep = request->sweep_direction.empty() ?
      sweep_direction_ : request->sweep_direction;
    if (region != "rectangle" && region != "trapezoid") {
      response->success = false;
      response->message = "region_type must be rectangle or trapezoid, received '" +
        region + "'.";
      response->config = currentConfig(response->message);
      return;
    }
    if (sweep != "horizontal" && sweep != "vertical") {
      response->success = false;
      response->message = "sweep_direction must be horizontal or vertical, received '" +
        sweep + "'.";
      response->config = currentConfig(response->message);
      return;
    }
    const bool unchanged = region == region_type_ && sweep == sweep_direction_;
    const bool shape_changed = region != region_type_;
    region_type_ = region;
    sweep_direction_ = sweep;

    std::ostringstream note;
    note << (unchanged ? "Configuration unchanged: " : "Configuration set to ") <<
      sweep << " " << region << ".";
    // A shape change keeps the points; see the withdrawal below for why.
    const std::string blocked = planBlockedReason();
    if (!blocked.empty()) {
      note << " " << blocked;
      response->success = true;
      response->message = note.str();
      response->config = currentConfig(response->message);
      publishConfig(response->message);
      publishStatus(response->message);
      return;
    }
    // A shape change never builds a preview on its own, even when the points
    // already in hand would be enough for the new shape. Going from trapezoid
    // to rectangle silently reinterpreted three clicks as two and drew a
    // different trajectory, which reads as the drop-down having planned
    // something nobody asked for. The points survive - clearing them would
    // punish a mis-click on a drop-down, and A and B mean the same corner in
    // both shapes - but the trajectory is withdrawn until the operator either
    // finishes selecting or asks for a replan.
    if (shape_changed && input_mode_ == "rviz") {
      publishTask(emptyTask());
      publishMarkers({}, {});
      last_error_.clear();
      note << " Preview withdrawn: press Replan to rebuild it from the " <<
        clicked_points_.size() << " selected point" <<
        (clicked_points_.size() == 1U ? "" : "s") <<
        ", or clear them and select again.";
      response->success = true;
      response->message = note.str();
      response->config = currentConfig(response->message);
      publishConfig(response->message);
      publishStatus(response->message);
      return;
    }
    if (input_mode_ == "rviz" && clicked_points_.size() > requiredPoints()) {
      note << " Using the first " << requiredPoints() << " of " <<
        clicked_points_.size() << " selected points.";
    }
    if (!unchanged || input_mode_ != "rviz") {
      applySelectedPoints();
      planFromPoints();
    }
    response->success = true;
    response->message = note.str() + " " + (last_error_.empty() ?
      std::string("Preview regenerated.") : "Planning failed: " + last_error_);
    response->config = currentConfig(response->message);
    publishConfig(response->message);
  }

  void applySelectedPoints()
  {
    if (input_mode_ != "rviz" || clicked_points_.size() < requiredPoints()) {
      return;
    }
    lower_left_ = clicked_points_[0];
    upper_right_ = clicked_points_[1];
    if (requiredPoints() == 3U) {
      lower_right_ = clicked_points_[2];
    }
  }

  void clearPoints(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    clicked_points_.clear();
    publishTask(emptyTask());
    publishMarkers({}, {});
    response->success = true;
    response->message = "Clicked points cleared.";
    publishStatus(response->message);
    publishConfig();
  }

  void replan(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    // The corners keep their configured values in rviz mode until enough
    // points are clicked, and clearing the selection does not reset them.
    // Replanning from those would hand the operator a startable task over a
    // region nobody selected.
    const std::string blocked = planBlockedReason();
    if (!blocked.empty()) {
      response->success = false;
      response->message = blocked;
      publishStatus(response->message);
      publishConfig(response->message);
      return;
    }
    // A shape change can leave the newest clicks unapplied, so adopt them here
    // rather than replanning the region a previous shape happened to store.
    applySelectedPoints();
    response->success = planFromPoints();
    response->message = response->success ? "Coverage path regenerated." : last_error_;
    publishConfig();
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
      auto path = generateFootprintAwareBoustrophedonPath(
        region.polygon, motion, detection_width_, detection_length_, row_spacing_,
        sweep_direction_, start_corner_);
      double coverage_ratio = sampledCoverageRatio(
        region.polygon, path, detection_width_, detection_length_);
      const std::string finishing_scan =
        appendTopEdgeFinishingScan(region.polygon, motion, path, coverage_ratio);
      if (coverage_ratio < minimum_nominal_coverage_ratio_) {
        std::ostringstream reason;
        reason << "Nominal detection footprint covers " << coverage_ratio * 100.0 <<
          " percent, below the required " << minimum_nominal_coverage_ratio_ * 100.0 <<
          " percent.";
        throw std::invalid_argument(reason.str());
      }
      publishTask(makeTask(region.polygon, motion, path));
      publishMarkers(region.polygon, path);
      std::ostringstream status;
      status << "Generated " << sweep_direction_ << " " << region_type_ <<
        " coverage path with " << path.size() << " waypoints; row spacing <= " <<
        row_spacing_ << " m, nominal footprint coverage " << coverage_ratio * 100.0 <<
        "% and safety margin " << safety_margin_ << " m.";
      if (!finishing_scan.empty()) {
        status << " " << finishing_scan;
      }
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
      publishMarkers({}, {});
      publishStatus("Planning failed: " + last_error_);
      return false;
    }
  }

  // Appends the finishing scan when the mode asks for it, updating the path and
  // the predicted coverage in place.  Returns what to tell the operator, or an
  // empty string when nothing was added.
  std::string appendTopEdgeFinishingScan(
    const Polygon & coverage_region, const Polygon & motion,
    std::vector<Point2> & path, double & coverage_ratio)
  {
    if (top_edge_scan_ == "never" || path.empty()) {
      return {};
    }
    // A horizontal sweep places its topmost line half a footprint below the top
    // edge already, so a finishing line there would duplicate it exactly.
    if (sweep_direction_ != "vertical") {
      return top_edge_scan_ == "always" ?
             "Top-edge scan skipped: a horizontal sweep already tops out on the edge." :
             std::string{};
    }
    if (top_edge_scan_ == "auto" && coverage_ratio >= minimum_nominal_coverage_ratio_) {
      return {};
    }
    const auto finishing = makeTopEdgeFinishingScan(
      coverage_region, motion, detection_width_, detection_length_, path.back());
    if (finishing.empty()) {
      return "Top-edge scan skipped: the finishing line does not fit in motion_region.";
    }
    const double before = coverage_ratio;
    std::vector<Point2> extended = path;
    extended.insert(extended.end(), finishing.begin(), finishing.end());
    const double after = sampledCoverageRatio(
      coverage_region, extended, detection_width_, detection_length_);
    path = std::move(extended);
    coverage_ratio = after;
    std::ostringstream note;
    note << "Added a top-edge finishing scan: nominal coverage " << before * 100.0 <<
      "% -> " << after * 100.0 << "%.";
    return note.str();
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
    // The work frame's origin is the wall's lower-left corner, so the surface
    // is the first quadrant and no region coordinate is ever negative.
    const RegionResult wall = makeRectangle(
      {0.0, 0.0}, {wall_width_, wall_height_});
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

  /// Draw the wall's reference grid once, on a topic of its own.
  ///
  /// Same pitch and the same lines as the grid painted on the wall face in
  /// Gazebo, so a coordinate read off one view is the coordinate in the other,
  /// and so an operator who switched the painted grid off for a photography
  /// run still has one to plan against. It never changes, so it is published
  /// once and latched for late subscribers rather than rebuilt per plan.
  void publishWallGrid()
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    if (wall_grid_spacing_ > 0.0) {
      std_msgs::msg::Header header;
      header.stamp = now();
      header.frame_id = frame_id_;
      visualization_msgs::msg::Marker grid;
      grid.header = header;
      grid.ns = "wall_grid";
      grid.id = 0;
      grid.type = visualization_msgs::msg::Marker::LINE_LIST;
      grid.action = visualization_msgs::msg::Marker::ADD;
      grid.pose.orientation.w = 1.0;
      grid.scale.x = 0.015;
      grid.color.r = 0.55F;
      grid.color.g = 0.58F;
      grid.color.b = 0.62F;
      grid.color.a = 0.9F;
      // Clear of the translucent wall slab below and of the region outlines
      // above, so it never fights either for the same pixels.
      const double height = 0.004;
      // The rule the wall face is painted with, in climbot_wall.sdf.xacro:
      // lines at whole multiples of the spacing measured from the work frame's
      // origin, which is the wall's lower-left corner, and only the interior
      // ones. The epsilon keeps a line off the far edge when the span divides
      // exactly, which is the ordinary case - 10 m at 1 m pitch is nine lines.
      const auto interior = [this](double span) {
          return static_cast<int>((span - 1e-9) / wall_grid_spacing_);
        };
      for (int index = 1; index <= interior(wall_width_); ++index) {
        const double x = index * wall_grid_spacing_;
        grid.points.push_back(markerPoint(Point2{x, 0.0}, height));
        grid.points.push_back(markerPoint(Point2{x, wall_height_}, height));
      }
      for (int index = 1; index <= interior(wall_height_); ++index) {
        const double y = index * wall_grid_spacing_;
        grid.points.push_back(markerPoint(Point2{0.0, y}, height));
        grid.points.push_back(markerPoint(Point2{wall_width_, y}, height));
      }
      markers.markers.push_back(grid);
    }
    grid_publisher_->publish(markers);
  }

  /// The motion region is deliberately not taken from the caller. It is the
  /// wall inset by the safety margin and does not depend on what was clicked,
  /// so it is the one thing an operator needs to see *before* clicking - a
  /// point outside it yields a region the footprint cannot cover, and drawing
  /// the boundary only once a plan succeeds hid it exactly when it was needed.
  void publishMarkers(
    const Polygon & original,
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
    // Centre of a surface that now runs from the origin, not across it.
    wall.pose.position.x = 0.5 * wall_width_;
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
      // A rectangle and a trapezoid both come out of the planner with four
      // vertices, so this covers every shape there is; the bound stops a fifth
      // one from labelling itself out of range rather than assuming there
      // will never be one.
      const std::vector<std::string> labels{"A", "C", "B", "D"};
      for (std::size_t index = 0; index < std::min(original.size(), labels.size());
        ++index)
      {
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
    const auto reachable = motionRegion();
    if (!reachable.empty()) {
      markers.markers.push_back(lineMarker(reachable, header, 0, "effective", 0.1F, 1.0F, 0.3F,
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
  rclcpp::Publisher<climbot_interfaces::msg::CoverageConfig>::SharedPtr config_publisher_;
  rclcpp::Service<climbot_interfaces::srv::ConfigureCoverage>::SharedPtr configure_service_;
  std::string input_mode_;
  std::string sweep_direction_;
  std::string start_corner_;
  Point2 lower_left_;
  Point2 upper_right_;
  Point2 lower_right_;
  double detection_width_;
  double detection_length_;
  double overlap_ratio_;
  double minimum_nominal_coverage_ratio_;
  std::string top_edge_scan_;
  double robot_length_;
  double robot_width_;
  double edge_clearance_;
  double wall_width_;
  double wall_height_;
  double wall_grid_spacing_;
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
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr grid_publisher_;
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
