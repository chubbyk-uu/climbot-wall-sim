#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "climbot_control/coverage_execution.hpp"
#include "climbot_interfaces/action/execute_coverage.hpp"
#include "climbot_interfaces/msg/coverage_status.hpp"
#include "climbot_interfaces/msg/coverage_task.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"

class CoverageManagerNode : public rclcpp::Node
{
public:
  using ExecuteCoverage = climbot_interfaces::action::ExecuteCoverage;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteCoverage>;
  using Status = climbot_interfaces::msg::CoverageStatus;

  CoverageManagerNode()
  : Node("coverage_manager"), frame_id_(declare_parameter("frame_id", "odom")),
    start_response_timeout_s_(declare_parameter("start_response_timeout_s", 5.0)),
    feedback_publish_period_s_(declare_parameter("feedback_publish_period_s", 0.2))
  {
    if (!(start_response_timeout_s_ > 0.0)) {
      throw std::invalid_argument("start_response_timeout_s must be positive.");
    }
    if (!(feedback_publish_period_s_ >= 0.0)) {
      throw std::invalid_argument("feedback_publish_period_s must be non-negative.");
    }
    status_.current_segment = -1;
    // Created before the task subscription so no preview can ever be handled
    // before there is somewhere to report it.
    status_publisher_ = create_publisher<Status>(
      "/coverage/manager_status", rclcpp::QoS(1).reliable().transient_local());
    task_subscription_ = create_subscription<climbot_interfaces::msg::CoverageTask>(
      "/coverage/task", rclcpp::QoS(1).reliable().transient_local(),
      [this](const climbot_interfaces::msg::CoverageTask::SharedPtr task) {
        if (const auto error = climbot_control::validateCoverageTask(*task, frame_id_)) {
          cached_task_.reset();
          status_.task_id.clear();
          status_.revision = 0U;
          status_.total_segments = 0U;
          // The planner publishes an empty task to clear the preview after a
          // click or a clear request, so reporting that as a malformed task
          // would make the operator hunt for a fault that does not exist.
          if (task->waypoints.empty()) {
            publishStatus(Status::IDLE, "Idle: no coverage region selected.");
          } else {
            publishStatus(Status::INVALID, "No executable preview: " + *error);
          }
          return;
        }
        cached_task_ = *task;
        status_.task_id = task->task_id;
        status_.revision = task->revision;
        status_.total_segments = static_cast<uint32_t>(task->segment_types.size());
        publishStatus(
          Status::READY,
          "Ready: " + task->task_id + " revision " + std::to_string(task->revision));
      });
    action_client_ = rclcpp_action::create_client<ExecuteCoverage>(this, "/coverage/execute");
    start_service_ = create_service<std_srvs::srv::Trigger>(
      "/coverage/start",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      const std_srvs::srv::Trigger::Response::SharedPtr response) {start(response);});
    cancel_service_ = create_service<std_srvs::srv::Trigger>(
      "/coverage/cancel",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      const std_srvs::srv::Trigger::Response::SharedPtr response) {cancel(response);});
    publishStatus(Status::IDLE, "Idle: waiting for a valid coverage preview.");
  }

private:
  void publishStatus(uint8_t state, const std::string & text)
  {
    status_.header.stamp = now();
    status_.state = state;
    status_.message = text;
    status_publisher_->publish(status_);
    last_publish_ = now();
    RCLCPP_INFO(get_logger(), "%s", text.c_str());
  }

  /// Republish the current status without logging, for the executor feedback
  /// stream. That arrives at the control loop rate, which no operator display
  /// needs and no log should carry, so it is rate limited here.
  void publishProgress()
  {
    const auto current = now();
    if ((current - last_publish_).seconds() < feedback_publish_period_s_) {
      return;
    }
    status_.header.stamp = current;
    status_publisher_->publish(status_);
    last_publish_ = current;
  }

  /// Release a start request whose goal response never arrived, so a crashed or
  /// restarted executor cannot lock the manager out until it is itself restarted.
  void expireStalePending()
  {
    if (!start_pending_since_) {
      return;
    }
    if ((now() - *start_pending_since_).seconds() < start_response_timeout_s_) {
      return;
    }
    start_pending_since_.reset();
    publishStatus(
      cached_task_ ? Status::READY : Status::IDLE,
      "Start request timed out before the executor answered.");
  }

  void start(const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    expireStalePending();
    if (!cached_task_) {
      response->success = false;
      response->message = "No valid coverage task is available.";
      return;
    }
    if (active_goal_ || start_pending_since_) {
      response->success = false;
      response->message = "A coverage task is already starting or executing.";
      return;
    }
    if (!action_client_->action_server_is_ready()) {
      response->success = false;
      response->message = "Coverage executor Action server is unavailable.";
      return;
    }
    ExecuteCoverage::Goal goal;
    goal.task = *cached_task_;
    const auto task_id = goal.task.task_id;
    const auto revision = goal.task.revision;
    rclcpp_action::Client<ExecuteCoverage>::SendGoalOptions options;
    options.goal_response_callback = [this, task_id,
        revision](const GoalHandle::SharedPtr & goal_handle) {
        start_pending_since_.reset();
        if (!goal_handle) {
          publishStatus(
            Status::READY,
            "Executor rejected " + task_id + " revision " + std::to_string(revision));
          return;
        }
        active_goal_ = goal_handle;
        publishStatus(
          Status::EXECUTING, "Executing " + task_id + " revision " + std::to_string(revision));
      };
    options.feedback_callback = [this](
      GoalHandle::SharedPtr, const std::shared_ptr<const ExecuteCoverage::Feedback> feedback) {
        status_.current_segment = feedback->current_segment;
        status_.progress = feedback->progress;
        status_.executor_state = feedback->state;
        publishProgress();
      };
    options.result_callback = [this](const GoalHandle::WrappedResult & result) {
        active_goal_.reset();
        status_.result_code = result.result->result_code;
        status_.progress = result.result->result_code == ExecuteCoverage::Result::SUCCESS ?
          1.0F : status_.progress;
        status_.current_segment = -1;
        status_.executor_state = ExecuteCoverage::Feedback::STOPPED;
        publishStatus(Status::FINISHED, "Execution finished: " + result.result->message);
      };
    status_.current_segment = -1;
    status_.progress = 0.0F;
    status_.executor_state = ExecuteCoverage::Feedback::WAITING;
    start_pending_since_ = now();
    // Announce STARTING before sending, so the goal response can never be
    // overtaken by this line and leave a live goal reported as still starting.
    publishStatus(
      Status::STARTING,
      "Start requested for " + task_id + " revision " + std::to_string(revision));
    action_client_->async_send_goal(goal, options);
    response->success = true;
    response->message = "Start request accepted for " + task_id + " revision " +
      std::to_string(revision);
  }

  void cancel(const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    expireStalePending();
    if (!active_goal_) {
      response->success = false;
      response->message =
        start_pending_since_ ? "Goal is still being accepted." : "No active coverage task.";
      return;
    }
    action_client_->async_cancel_goal(active_goal_);
    response->success = true;
    response->message = "Cancellation requested; executor will stop before returning.";
  }

  std::string frame_id_;
  double start_response_timeout_s_;
  double feedback_publish_period_s_;
  Status status_;
  rclcpp::Time last_publish_{0, 0, RCL_ROS_TIME};
  std::optional<rclcpp::Time> start_pending_since_;
  std::optional<climbot_interfaces::msg::CoverageTask> cached_task_;
  GoalHandle::SharedPtr active_goal_;
  rclcpp::Subscription<climbot_interfaces::msg::CoverageTask>::SharedPtr task_subscription_;
  rclcpp_action::Client<ExecuteCoverage>::SharedPtr action_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp::Publisher<Status>::SharedPtr status_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<CoverageManagerNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("coverage_manager"), "Startup failed: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
