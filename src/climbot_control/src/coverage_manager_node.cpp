#include <chrono>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "climbot_control/control_clock.hpp"
#include "climbot_control/coverage_execution.hpp"
#include "climbot_interfaces/action/execute_coverage.hpp"
#include "climbot_interfaces/msg/coverage_status.hpp"
#include "climbot_interfaces/msg/coverage_task.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class CoverageManagerNode : public rclcpp::Node
{
public:
  using ExecuteCoverage = climbot_interfaces::action::ExecuteCoverage;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteCoverage>;
  using Status = climbot_interfaces::msg::CoverageStatus;

  CoverageManagerNode()
  : Node("coverage_manager"), frame_id_(declare_parameter("frame_id", "odom")),
    start_response_timeout_s_(declare_parameter("start_response_timeout_s", 5.0)),
    feedback_publish_period_s_(declare_parameter("feedback_publish_period_s", 0.2)),
    executor_timeout_s_(declare_parameter("executor_timeout_s", 5.0))
  {
    if (!(start_response_timeout_s_ > 0.0)) {
      throw std::invalid_argument("start_response_timeout_s must be positive.");
    }
    if (!(feedback_publish_period_s_ >= 0.0)) {
      throw std::invalid_argument("feedback_publish_period_s must be non-negative.");
    }
    if (!(executor_timeout_s_ > 0.0)) {
      throw std::invalid_argument("executor_timeout_s must be positive.");
    }
    // The feedback throttle and the start-response deadline are elapsed times,
    // and off sim time the node clock is the settable system clock. A backward
    // step there stretched both silently: status stopped updating and a start
    // request that never got an answer took the step's length longer to be
    // reported as such.
    control_clock_ = climbot_control::controlClock(this);
    last_publish_ = rclcpp::Time(0, 0, control_clock_->get_clock_type());
    status_.current_segment = -1;
    // Created before the task subscription so no preview can ever be handled
    // before there is somewhere to report it.
    status_publisher_ = create_publisher<Status>(
      "/coverage/manager_status", rclcpp::QoS(1).reliable().transient_local());
    task_subscription_ = create_subscription<climbot_interfaces::msg::CoverageTask>(
      "/coverage/task", rclcpp::QoS(1).reliable().transient_local(),
      [this](const climbot_interfaces::msg::CoverageTask::SharedPtr task) {
        const auto error = climbot_control::validateCoverageTask(*task, frame_id_);
        if (error) {
          cached_task_.reset();
        } else {
          cached_task_ = *task;
        }
        if (busy()) {
          // A preview arriving mid-run only changes what a later start would
          // use; the executor holds its own copy of the running task. Letting
          // it rewrite the state here reported a moving robot as Ready or Idle
          // and took the cancel button away from the operator.
          publishStatus(
            status_.state,
            error ?
            "Preview cleared while " + status_.task_id + " keeps running." :
            "Cached newer preview " + task->task_id + " revision " +
            std::to_string(task->revision) + " while " + status_.task_id +
            " keeps running.");
          return;
        }
        if (error) {
          status_.task_id.clear();
          status_.revision = 0U;
          status_.total_segments = 0U;
          // The planner publishes an empty task to clear the preview after a
          // click or a clear request, so reporting that as a malformed task
          // would make the operator hunt for a fault that does not exist. It
          // also publishes one when planning fails, and the manager cannot
          // tell those apart: the reason is on the planner's own status topic.
          if (task->waypoints.empty()) {
            publishStatus(Status::IDLE, "Idle: no coverage region selected.");
          } else {
            publishStatus(Status::INVALID, "No executable preview: " + *error);
          }
          return;
        }
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
    // Steady time, and a wall timer, so a paused or stopped simulation clock
    // cannot freeze the checks that release a stuck task.
    supervision_timer_ = create_wall_timer(500ms, [this]() {superviseExecution();});
    publishStatus(Status::IDLE, "Idle: waiting for a valid coverage preview.");
  }

private:
  /// A goal is in flight, either awaiting acceptance or executing.
  bool busy() const
  {
    return active_goal_ != nullptr || start_pending_since_.has_value();
  }

  /// Publish what this manager would accept right now, using the same
  /// preconditions its services apply, so an interface renders the decision
  /// instead of guessing it from the state.
  void refreshPermissions()
  {
    status_.can_start = cached_task_.has_value() && !busy();
    status_.can_cancel = active_goal_ != nullptr;
  }

  void publishStatus(uint8_t state, const std::string & text)
  {
    status_.header.stamp = now();
    status_.state = state;
    status_.message = text;
    refreshPermissions();
    status_publisher_->publish(status_);
    last_publish_ = control_clock_->now();
    RCLCPP_INFO(get_logger(), "%s", text.c_str());
  }

  /// Republish the current status without logging, for the executor feedback
  /// stream. That arrives at the control loop rate, which no operator display
  /// needs and no log should carry, so it is rate limited here.
  void publishProgress()
  {
    const auto current = control_clock_->now();
    if ((current - last_publish_).seconds() < feedback_publish_period_s_) {
      return;
    }
    status_.header.stamp = now();
    refreshPermissions();
    status_publisher_->publish(status_);
    last_publish_ = current;
  }

  /// An accepted goal is only ever finished by its result callback, which a
  /// dead executor never delivers: the manager then reports EXECUTING forever
  /// and refuses every later start until it is itself restarted. Releasing the
  /// goal after the server has been gone for a while restores the operator's
  /// control. The robot is already safe by then, because the speed watchdog
  /// zeroes the command as soon as one stops arriving.
  void superviseExecution()
  {
    expireStalePending();
    if (!active_goal_ || action_client_->action_server_is_ready()) {
      executor_missing_since_.reset();
      return;
    }
    const auto current = std::chrono::steady_clock::now();
    if (!executor_missing_since_) {
      executor_missing_since_ = current;
      return;
    }
    if (std::chrono::duration<double>(current - *executor_missing_since_).count() <
      executor_timeout_s_)
    {
      return;
    }
    executor_missing_since_.reset();
    active_goal_.reset();
    status_.result_code = ExecuteCoverage::Result::CONTROL_TIMEOUT;
    status_.current_segment = -1;
    status_.executor_state = ExecuteCoverage::Feedback::STOPPED;
    publishStatus(
      Status::FINISHED,
      "Executor disappeared while running " + status_.task_id +
      "; released the task so it can be started again.");
  }

  /// Release a start request whose goal response never arrived, so a crashed or
  /// restarted executor cannot lock the manager out until it is itself restarted.
  void expireStalePending()
  {
    if (!start_pending_since_) {
      return;
    }
    if ((control_clock_->now() - *start_pending_since_).seconds() <
      start_response_timeout_s_)
    {
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
    if (busy()) {
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
    // The identity always describes whatever the current state is about. A
    // preview cached during the previous run only becomes the reported task
    // here, when it is the one actually being sent.
    status_.task_id = task_id;
    status_.revision = revision;
    status_.total_segments = static_cast<uint32_t>(goal.task.segment_types.size());
    status_.current_segment = -1;
    status_.progress = 0.0F;
    status_.executor_state = ExecuteCoverage::Feedback::WAITING;
    start_pending_since_ = control_clock_->now();
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
  double executor_timeout_s_;
  Status status_;
  // Durations here are measured on control_clock_, so this carries that
  // clock's type and is set in the constructor. Message stamps stay on
  // ROS time; only the elapsed-time arithmetic moves.
  rclcpp::Clock::SharedPtr control_clock_;
  rclcpp::Time last_publish_;
  std::optional<rclcpp::Time> start_pending_since_;
  std::optional<std::chrono::steady_clock::time_point> executor_missing_since_;
  std::optional<climbot_interfaces::msg::CoverageTask> cached_task_;
  GoalHandle::SharedPtr active_goal_;
  rclcpp::Subscription<climbot_interfaces::msg::CoverageTask>::SharedPtr task_subscription_;
  rclcpp_action::Client<ExecuteCoverage>::SharedPtr action_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp::Publisher<Status>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr supervision_timer_;
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
