#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "climbot_control/coverage_execution.hpp"
#include "climbot_interfaces/action/execute_coverage.hpp"
#include "climbot_interfaces/msg/coverage_task.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

class CoverageManagerNode : public rclcpp::Node
{
public:
  using ExecuteCoverage = climbot_interfaces::action::ExecuteCoverage;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteCoverage>;

  CoverageManagerNode()
  : Node("coverage_manager"), frame_id_(declare_parameter("frame_id", "odom")),
    start_response_timeout_s_(declare_parameter("start_response_timeout_s", 5.0))
  {
    if (!(start_response_timeout_s_ > 0.0)) {
      throw std::invalid_argument("start_response_timeout_s must be positive.");
    }
    task_subscription_ = create_subscription<climbot_interfaces::msg::CoverageTask>(
      "/coverage/task", rclcpp::QoS(1).reliable().transient_local(),
      [this](const climbot_interfaces::msg::CoverageTask::SharedPtr task) {
        if (const auto error = climbot_control::validateCoverageTask(*task, frame_id_)) {
          cached_task_.reset();
          publishStatus("No executable preview: " + *error);
          return;
        }
        cached_task_ = *task;
        publishStatus("Ready: " + task->task_id + " revision " +
          std::to_string(task->revision));
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
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/coverage/manager_status", rclcpp::QoS(1).reliable().transient_local());
    publishStatus("Idle: waiting for a valid coverage preview.");
  }

private:
  void publishStatus(const std::string & text)
  {
    std_msgs::msg::String message;
    message.data = text;
    status_publisher_->publish(message);
    RCLCPP_INFO(get_logger(), "%s", text.c_str());
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
    publishStatus("Start request timed out before the executor answered.");
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
          publishStatus("Executor rejected " + task_id + " revision " + std::to_string(revision));
          return;
        }
        active_goal_ = goal_handle;
        publishStatus("Executing " + task_id + " revision " + std::to_string(revision));
      };
    options.result_callback = [this](const GoalHandle::WrappedResult & result) {
        active_goal_.reset();
        publishStatus("Execution finished: " + result.result->message);
      };
    start_pending_since_ = now();
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
  std::optional<rclcpp::Time> start_pending_since_;
  std::optional<climbot_interfaces::msg::CoverageTask> cached_task_;
  GoalHandle::SharedPtr active_goal_;
  rclcpp::Subscription<climbot_interfaces::msg::CoverageTask>::SharedPtr task_subscription_;
  rclcpp_action::Client<ExecuteCoverage>::SharedPtr action_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
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
