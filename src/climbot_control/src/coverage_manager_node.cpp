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
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "climbot_control/control_clock.hpp"
#include "climbot_control/coverage_execution.hpp"
#include "climbot_interfaces/action/execute_coverage.hpp"
#include "climbot_interfaces/msg/coverage_status.hpp"
#include "climbot_interfaces/msg/coverage_task.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
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
    executor_timeout_s_(declare_parameter("executor_timeout_s", 5.0)),
    // How long /control/cmd_vel has to carry no motion before the manager will
    // accept that the robot is not being driven. Longer than the speed
    // watchdog's own command timeout, so an executor that is still commanding
    // between two of these checks cannot read as quiet.
    command_quiet_s_(declare_parameter("command_quiet_s", 1.0)),
    hold_response_timeout_s_(declare_parameter("hold_response_timeout_s", 1.0))
  {
    if (!std::isfinite(start_response_timeout_s_) || !(start_response_timeout_s_ > 0.0)) {
      throw std::invalid_argument("start_response_timeout_s must be positive.");
    }
    if (!std::isfinite(feedback_publish_period_s_) || !(feedback_publish_period_s_ >= 0.0)) {
      throw std::invalid_argument("feedback_publish_period_s must be non-negative.");
    }
    if (!std::isfinite(executor_timeout_s_) || !(executor_timeout_s_ > 0.0)) {
      throw std::invalid_argument("executor_timeout_s must be positive.");
    }
    if (!std::isfinite(command_quiet_s_) || !(command_quiet_s_ > 0.0)) {
      throw std::invalid_argument("command_quiet_s must be positive.");
    }
    if (!std::isfinite(hold_response_timeout_s_) || !(hold_response_timeout_s_ > 0.0)) {
      throw std::invalid_argument("hold_response_timeout_s must be positive.");
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
    // What the robot is being told to do, watched directly rather than inferred
    // from whether the Action server answers. Those are different questions,
    // and losing the second one while the first keeps happening is the whole
    // reason this manager has a STOPPING state.
    command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/control/cmd_vel", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr command) {
        if (command->linear.x != 0.0 || command->angular.z != 0.0) {
          last_motion_command_ = std::chrono::steady_clock::now();
        }
      });
    // The stop that does not go through the executor. Engaging it is what lets
    // this manager say the robot is stopped without having to be told so by the
    // thing it just lost contact with.
    hold_client_ = create_client<std_srvs::srv::SetBool>("/control/hold");
    hold_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/control/hold_active", rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        hold_active_ = message->data;
        // The watchdog's own state is stronger than a service response. It also
        // retires a request whose response was lost after the watchdog had
        // already applied it, so a later retry cannot race the state backwards.
        if (hold_request_value_ && *hold_request_value_ == message->data) {
          if (!message->data) {
            hold_release_confirmed_ = true;
          }
          ++hold_generation_;
          hold_request_value_.reset();
          hold_request_since_.reset();
        }
      });
    start_service_ = create_service<std_srvs::srv::Trigger>(
      "/coverage/start",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      const std_srvs::srv::Trigger::Response::SharedPtr response) {start(response);});
    cancel_service_ = create_service<std_srvs::srv::Trigger>(
      "/coverage/cancel",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      const std_srvs::srv::Trigger::Response::SharedPtr response) {cancel(response);});
    // Steady time, and a wall timer, so a paused or stopped simulation clock
    // cannot freeze either safety deadline: detecting an unanswered start
    // request and detecting a vanished executor. Both measure their own
    // elapsed time on std::chrono::steady_clock rather than on control_clock_,
    // which follows simulation time whenever it is active.
    supervision_timer_ = create_wall_timer(500ms, [this]() {superviseExecution();});
    publishStatus(Status::IDLE, "Idle: waiting for a valid coverage preview.");
  }

private:
  /// A goal is in flight, either awaiting acceptance or executing.
  bool busy() const
  {
    return active_goal_ != nullptr || start_pending_since_.has_value() ||
           stopping_since_.has_value() || queued_goal_.has_value();
  }

  /// Publish what this manager would accept right now, using the same
  /// preconditions its services apply, so an interface renders the decision
  /// instead of guessing it from the state.
  void refreshPermissions()
  {
    status_.can_start = cached_task_.has_value() && !busy();
    // The stop entry survives losing the executor. It used to be withdrawn at
    // the same moment contact was lost, which is the one moment an operator
    // watching the robot still move has nothing else to reach for.
    status_.can_cancel = active_goal_ != nullptr || stopping_since_.has_value();
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
  /// and refuses every later start until it is itself restarted. Losing the
  /// server is therefore something this has to act on - but acting on it used
  /// to mean calling the task finished on the spot, and that is a claim about
  /// the robot which nothing here had checked.
  //
  // "The server is undiscoverable" and "the robot has stopped" are two
  // different facts. A dead executor gives both at once. A DDS discovery
  // hiccup, a wedged Action channel or a cancel request that never lands give
  // only the first: the executor is alive, /control/cmd_vel keeps being
  // refreshed, the speed watchdog has nothing to time out, and the robot drives
  // on. Reporting FINISHED there dropped the goal handle and took the operator
  // stop entry away at that exact moment.
  //
  // So the loss now opens a STOPPING state instead. It is left only when
  // nothing has commanded motion for command_quiet_s, or the executor answers
  // after all. A hold protects current output but does not terminate the task.
  void superviseExecution()
  {
    continueQueuedStart();
    expireStalePending();
    if (stopping_since_) {
      continueStopping();
      return;
    }
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
    beginStopping();
  }

  void beginStopping()
  {
    stopping_since_ = std::chrono::steady_clock::now();
    engageHold();
    // Still worth asking, and the goal handle is kept so the answer has
    // somewhere to land: the server being undiscoverable is what brought us
    // here, but that can be the transport rather than the executor.
    action_client_->async_cancel_goal(active_goal_);
    publishStatus(
      Status::STOPPING,
      "Lost contact with the executor while running " + status_.task_id +
      "; stopping the robot before releasing the task.");
    continueStopping();
  }

  /// Leave STOPPING only for evidence about the robot, never about the server.
  void continueStopping()
  {
    engageHold();
    // A send-goal request that timed out has no handle yet. Until its response
    // arrives there is nothing to cancel and no proof it was rejected. Calling
    // the task finished here recreates the late-acceptance hole under a new
    // name, so this state deliberately has no elapsed-time escape hatch.
    if (unresolved_goal_response_) {
      return;
    }
    if (motionCommandsHaveStopped()) {
      finishStopping("nothing has commanded motion since");
      return;
    }
    // A hold proves the actuator output is zero now, not that the task which
    // keeps asking for motion has ended. The watchdog can restart and forget
    // its process-local bool. Staying STOPPING while commands keep arriving
    // makes that restart observable: hold_active becomes false and the next
    // tick applies the hold again instead of releasing an orphaned task.
  }

  bool motionCommandsHaveStopped() const
  {
    if (!stopping_since_) {
      return false;
    }
    const auto last_evidence = last_motion_command_ && *last_motion_command_ > *stopping_since_ ?
      *last_motion_command_ : *stopping_since_;
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_evidence).count() >= command_quiet_s_;
  }

  void finishStopping(const std::string & evidence)
  {
    stopping_since_.reset();
    // Nothing this goal says counts from here. An undiscoverable server is not
    // a dead one, and its result or feedback can still turn up long after the
    // operator has started the next task.
    ++goal_generation_;
    active_goal_.reset();
    unresolved_goal_response_ = false;
    status_.result_code = ExecuteCoverage::Result::EXECUTOR_LOST;
    status_.current_segment = -1;
    status_.executor_state = ExecuteCoverage::Feedback::STOPPED;
    publishStatus(
      Status::FINISHED,
      "Released " + status_.task_id + " after losing the executor: " + evidence + ".");
  }

  /// Ask the speed watchdog to force /cmd_vel to zero, whatever is publishing.
  void engageHold()
  {
    if (hold_active_.value_or(false)) {
      return;
    }
    maintainHoldRequest(true);
  }

  /// Keep one desired hold state in flight, retiring and retrying a request
  /// whose response never arrives. SetBool is idempotent, so a retry is safe;
  /// allowing one missing response to suppress every retry is not.
  void maintainHoldRequest(bool held)
  {
    const auto current = std::chrono::steady_clock::now();
    if (hold_request_value_) {
      const bool same_request = *hold_request_value_ == held;
      const bool still_live = hold_request_since_ &&
        std::chrono::duration<double>(current - *hold_request_since_).count() <
        hold_response_timeout_s_;
      if (same_request && still_live) {
        return;
      }
      // A different desired state supersedes immediately. The same state is
      // retried only after its deadline. Either way, a late callback carries
      // the retired generation and cannot clear the new request.
      ++hold_generation_;
      hold_request_value_.reset();
      hold_request_since_.reset();
    }
    if (!hold_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = held;
    const auto generation = ++hold_generation_;
    hold_request_value_ = held;
    hold_request_since_ = current;
    hold_client_->async_send_request(
      request,
      [this, generation, held](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
        if (generation != hold_generation_) {
          return;
        }
        try {
          const auto response = future.get();
          if (!response->success) {
            RCLCPP_ERROR(get_logger(), "The speed hold refused a request: %s",
              response->message.c_str());
          } else if (!held) {
            // SetBool answers only after the watchdog changed its process-local
            // state, so this is a release confirmation even if the latched
            // status sample is delivered after the response.
            hold_release_confirmed_ = true;
          }
        } catch (const std::exception & exception) {
          RCLCPP_ERROR(get_logger(), "The speed hold request failed: %s", exception.what());
        }
        // The supervision timer retries unless /control/hold_active confirms
        // the desired state. Clearing this merely permits that retry.
        hold_request_value_.reset();
        hold_request_since_.reset();
      });
  }

  /// Escalate an unanswered start request into supervised STOPPING.
  void expireStalePending()
  {
    if (!start_pending_since_) {
      return;
    }
    // Steady time, like the executor check above and for the same reason. This
    // used to be the node clock, which under use_sim_time is the simulation
    // clock: a paused simulation still fires this wall timer but stops the
    // difference from growing, so a start request that had already gone
    // unanswered would never expire - exactly what the comment on the timer
    // says is avoided.
    if (std::chrono::duration<double>(
        std::chrono::steady_clock::now() - *start_pending_since_).count() <
      start_response_timeout_s_)
    {
      return;
    }
    start_pending_since_.reset();
    unresolved_goal_response_ = true;
    stopping_since_ = std::chrono::steady_clock::now();
    engageHold();
    publishStatus(
      Status::STOPPING,
      "Start request timed out before the executor answered; holding the robot "
      "and waiting for the request's real outcome.");
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
    const auto generation = ++goal_generation_;

    // The identity always describes whatever the current state is about. A
    // preview cached during the previous run only becomes the reported task
    // here, when it is the one actually being sent.
    status_.task_id = task_id;
    status_.revision = revision;
    status_.total_segments = static_cast<uint32_t>(goal.task.segment_types.size());
    status_.current_segment = -1;
    status_.progress = 0.0F;
    status_.planned_total_s = 0.0;
    status_.schedule_lag_s = 0.0;
    status_.estimated_remaining_s = 0.0;
    status_.executor_state = ExecuteCoverage::Feedback::WAITING;

    // Whatever is held has to be confirmed released before the Action request
    // exists. Sending both asynchronously made EXECUTING mean only that the
    // controller had a goal, not that the actuator path could move, and a late
    // engage response could win after the new task started.
    const bool hold_may_engage = hold_active_.value_or(false) ||
      (hold_request_value_ && *hold_request_value_);
    if (hold_may_engage) {
      queued_goal_ = goal;
      queued_goal_generation_ = generation;
      hold_release_confirmed_ = false;
      maintainHoldRequest(false);
      publishStatus(
        Status::STARTING,
        "Releasing the speed hold before starting " + task_id + " revision " +
        std::to_string(revision) + ".");
    } else {
      // STARTING must already be busy in the status message. dispatchGoal()
      // refreshes this timestamp immediately before the asynchronous send, but
      // setting it here closes the one-message permission gap.
      start_pending_since_ = std::chrono::steady_clock::now();
      publishStatus(
        Status::STARTING,
        "Start requested for " + task_id + " revision " + std::to_string(revision));
      dispatchGoal(goal, generation);
    }
    response->success = true;
    response->message = "Start request accepted for " + task_id + " revision " +
      std::to_string(revision);
  }

  void continueQueuedStart()
  {
    if (!queued_goal_) {
      return;
    }
    if (!hold_release_confirmed_) {
      maintainHoldRequest(false);
      return;
    }
    if (!action_client_->action_server_is_ready()) {
      const auto task_id = queued_goal_->task.task_id;
      queued_goal_.reset();
      ++goal_generation_;
      publishStatus(
        cached_task_ ? Status::READY : Status::IDLE,
        "Released the speed hold, but the executor disappeared before " + task_id +
        " could be sent.");
      return;
    }
    auto goal = *queued_goal_;
    const auto generation = queued_goal_generation_;
    queued_goal_.reset();
    hold_release_confirmed_ = false;
    dispatchGoal(goal, generation);
  }

  void dispatchGoal(const ExecuteCoverage::Goal & goal, uint64_t generation)
  {
    const auto task_id = goal.task.task_id;
    const auto revision = goal.task.revision;
    // Every callback is stamped with the request that created it. A response
    // timeout no longer retires that generation: the request still exists and
    // remains the current safety problem until its real response arrives.
    rclcpp_action::Client<ExecuteCoverage>::SendGoalOptions options;
    options.goal_response_callback = [this, task_id, revision,
        generation](const GoalHandle::SharedPtr & goal_handle) {
        if (generation != goal_generation_) {
          RCLCPP_WARN(
            get_logger(),
            "Received a retired goal response for %s revision %u.",
            task_id.c_str(), revision);
          if (goal_handle) {
            // Defensive even though this manager no longer retires an
            // unresolved request: a replacement Action server may permit more
            // than one goal, so never rely on this project's tracker refusing
            // the second one.
            maintainHoldRequest(true);
            action_client_->async_cancel_goal(goal_handle);
          }
          return;
        }
        start_pending_since_.reset();
        const bool response_was_uncertain = unresolved_goal_response_;
        unresolved_goal_response_ = false;
        if (!goal_handle) {
          if (response_was_uncertain) {
            stopping_since_.reset();
            ++goal_generation_;
          }
          publishStatus(
            Status::READY,
            (response_was_uncertain ? "The timed-out request was ultimately rejected for " :
            "Executor rejected ") + task_id + " revision " + std::to_string(revision));
          return;
        }
        active_goal_ = goal_handle;
        if (response_was_uncertain || stopping_since_) {
          action_client_->async_cancel_goal(active_goal_);
          publishStatus(
            Status::STOPPING,
            "The timed-out request was accepted for " + task_id + " revision " +
            std::to_string(revision) + "; holding and canceling it.");
          continueStopping();
          return;
        }
        publishStatus(
          Status::EXECUTING, "Executing " + task_id + " revision " + std::to_string(revision));
      };
    options.feedback_callback = [this, generation](
      GoalHandle::SharedPtr, const std::shared_ptr<const ExecuteCoverage::Feedback> feedback) {
        if (generation != goal_generation_) {
          return;
        }
        status_.current_segment = feedback->current_segment;
        status_.progress = feedback->progress;
        status_.executor_state = feedback->state;
        status_.planned_total_s = feedback->planned_total_s;
        status_.schedule_lag_s = feedback->schedule_lag_s;
        status_.estimated_remaining_s = feedback->estimated_remaining_s;
        publishProgress();
      };
    options.result_callback = [this, generation](const GoalHandle::WrappedResult & result) {
        if (generation != goal_generation_) {
          return;
        }
        // Reached while STOPPING too, and welcome there: the executor answering
        // is the strongest evidence the run is over, and it carries a real
        // outcome rather than EXECUTOR_LOST. The generation is retired here in
        // either case, so nothing else from this goal can follow it.
        ++goal_generation_;
        stopping_since_.reset();
        unresolved_goal_response_ = false;
        active_goal_.reset();
        status_.result_code = result.result->result_code;
        status_.progress = result.result->result_code == ExecuteCoverage::Result::SUCCESS ?
          1.0F : status_.progress;
        status_.current_segment = -1;
        status_.executor_state = ExecuteCoverage::Feedback::STOPPED;
        publishStatus(Status::FINISHED, "Execution finished: " + result.result->message);
      };
    start_pending_since_ = std::chrono::steady_clock::now();
    action_client_->async_send_goal(goal, options);
  }

  void cancel(const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    expireStalePending();
    if (stopping_since_) {
      if (active_goal_) {
        action_client_->async_cancel_goal(active_goal_);
      }
      engageHold();
      response->success = true;
      response->message = unresolved_goal_response_ ?
        "Start outcome is still unknown; holding while its response is awaited." :
        "Already stopping; cancellation and the speed hold were requested again.";
      return;
    }
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
  double command_quiet_s_;
  double hold_response_timeout_s_;
  Status status_;
  // Durations here are measured on control_clock_, so this carries that
  // clock's type and is set in the constructor. Message stamps stay on
  // ROS time; only the elapsed-time arithmetic moves.
  rclcpp::Clock::SharedPtr control_clock_;
  rclcpp::Time last_publish_;
  std::optional<std::chrono::steady_clock::time_point> start_pending_since_;
  std::optional<std::chrono::steady_clock::time_point> executor_missing_since_;
  std::optional<std::chrono::steady_clock::time_point> stopping_since_;
  std::optional<std::chrono::steady_clock::time_point> last_motion_command_;
  std::optional<std::chrono::steady_clock::time_point> hold_request_since_;
  // No value until the speed watchdog has published: absent and false are
  // different answers, and only the second one is evidence of anything.
  std::optional<bool> hold_active_;
  std::optional<bool> hold_request_value_;
  bool hold_release_confirmed_{false};
  bool unresolved_goal_response_{false};
  uint64_t hold_generation_{0};
  // Identifies callbacks from one Action request. A timed-out request keeps its
  // generation until its true outcome or a supervised stop retires it.
  uint64_t goal_generation_{0};
  uint64_t queued_goal_generation_{0};
  std::optional<climbot_interfaces::msg::CoverageTask> cached_task_;
  std::optional<ExecuteCoverage::Goal> queued_goal_;
  GoalHandle::SharedPtr active_goal_;
  rclcpp::Subscription<climbot_interfaces::msg::CoverageTask>::SharedPtr task_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hold_subscription_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr hold_client_;
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
    RCLCPP_FATAL(rclcpp::get_logger("coverage_manager"), "Node failed: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
