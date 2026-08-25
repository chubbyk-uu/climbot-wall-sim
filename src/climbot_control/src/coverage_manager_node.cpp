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
#include "climbot_interfaces/msg/inspection_archive_status.hpp"
#include "climbot_interfaces/msg/coverage_status.hpp"
#include "climbot_interfaces/msg/coverage_task.hpp"
#include "climbot_interfaces/srv/finalize_inspection_archive.hpp"
#include "climbot_interfaces/srv/prepare_inspection_archive.hpp"
#include "climbot_interfaces/srv/start_coverage.hpp"
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
  using ArchiveStatus = climbot_interfaces::msg::InspectionArchiveStatus;
  using PrepareArchive = climbot_interfaces::srv::PrepareInspectionArchive;
  using FinalizeArchive = climbot_interfaces::srv::FinalizeInspectionArchive;
  using StartCoverage = climbot_interfaces::srv::StartCoverage;

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
    hold_response_timeout_s_(declare_parameter("hold_response_timeout_s", 1.0)),
    hold_discovery_grace_s_(declare_parameter("hold_discovery_grace_s", 1.0)),
    archive_finalize_timeout_s_(declare_parameter("archive_finalize_timeout_s", 5.0)),
    // A bare control-node process stays motion-only for compatibility and
    // safety. The complete mission launch explicitly enables inspection.
    inspection_default_enabled_(declare_parameter("inspection_default_enabled", false)),
    inspection_output_root_(declare_parameter("inspection_output_root", "~/climbot_data"))
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
    if (!std::isfinite(hold_discovery_grace_s_) || !(hold_discovery_grace_s_ > 0.0)) {
      throw std::invalid_argument("hold_discovery_grace_s must be positive.");
    }
    if (!std::isfinite(archive_finalize_timeout_s_) || !(archive_finalize_timeout_s_ > 0.0)) {
      throw std::invalid_argument("archive_finalize_timeout_s must be positive.");
    }
    if (inspection_output_root_.empty()) {
      throw std::invalid_argument("inspection_output_root must not be empty.");
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
    archive_prepare_client_ = create_client<PrepareArchive>("/inspection/archive/prepare");
    archive_finalize_client_ = create_client<FinalizeArchive>("/inspection/archive/finalize");
    archive_status_subscription_ = create_subscription<ArchiveStatus>(
      "/inspection/archive/status", rclcpp::QoS(1).reliable().transient_local(),
      [this](const ArchiveStatus::SharedPtr archive) {onArchiveStatus(*archive);});
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
        last_hold_status_ = std::chrono::steady_clock::now();
        // The watchdog's own state is stronger than a service response. It also
        // retires a request whose response was lost after the watchdog had
        // already applied it, so a later retry cannot race the state backwards.
        if (hold_request_value_ && *hold_request_value_ == message->data) {
          // A latched false sample can belong to the previous run. It proves
          // only that the watchdog was released at some point, not that this
          // Start's explicit release request reached it. The service response
          // below carries that causal confirmation; accepting this sample here
          // could dispatch a Goal while an old hold was still in force.
          if (!message->data) {
            return;
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
    configured_start_service_ = create_service<StartCoverage>(
      "/coverage/start_configured",
      [this](const StartCoverage::Request::SharedPtr request,
      const StartCoverage::Response::SharedPtr response) {startConfigured(*request, response);});
    cancel_service_ = create_service<std_srvs::srv::Trigger>(
      "/coverage/cancel",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      const std_srvs::srv::Trigger::Response::SharedPtr response) {cancel(response);});
    force_abandon_service_ = create_service<std_srvs::srv::Trigger>(
      "/coverage/force_abandon",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      const std_srvs::srv::Trigger::Response::SharedPtr response) {forceAbandon(response);});
    rearm_service_ = create_service<std_srvs::srv::Trigger>(
      "/coverage/rearm",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      const std_srvs::srv::Trigger::Response::SharedPtr response) {rearm(response);});
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
           stopping_since_.has_value() || queued_goal_.has_value() || recovery_locked_ ||
           archive_prepare_pending_ || archive_finalize_pending_;
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
    status_.can_cancel = active_goal_ != nullptr || stopping_since_.has_value() ||
      queued_goal_.has_value() || archive_prepare_pending_;
    status_.can_force_abandon = stopping_since_.has_value() && unresolved_goal_response_;
    status_.can_rearm = recovery_locked_;
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

  void resetArchiveStatus(bool enabled)
  {
    status_.inspection_enabled = enabled;
    status_.archive_state = ArchiveStatus::IDLE;
    status_.archive_run_id.clear();
    status_.archive_directory.clear();
    status_.archive_preflight_expected_images = 0U;
    status_.archive_expected_images = 0U;
    status_.archive_saved_images = 0U;
    status_.archive_failed_images = 0U;
    status_.archive_estimated_bytes = 0U;
    status_.archive_message =
      enabled ? "Archive preparation has not started." : "Inspection disabled.";
  }

  void onArchiveStatus(const ArchiveStatus & archive)
  {
    if (!status_.inspection_enabled || archive.task_id != status_.task_id ||
      archive.revision != status_.revision ||
      (!status_.archive_run_id.empty() && !archive.run_id.empty() &&
      archive.run_id != status_.archive_run_id))
    {
      return;
    }
    // A canceled prepare may still finish on the recorder, which then emits
    // READY before the detached-finalize request reaches it. There is no
    // motion Goal and no owned run id in that state, so accepting the late
    // update would repaint a canceled, safely idle task as a ready archive.
    // The generation guard below sends that run a CANCELED finalize; its
    // intermediate status is not an operator-visible state of this task.
    if (status_.archive_state == ArchiveStatus::CANCELED &&
      !archive_prepare_pending_ && !active_inspection_enabled_ &&
      !archive_finalize_pending_)
    {
      return;
    }
    status_.archive_state = archive.state;
    if (!archive.run_id.empty()) {
      status_.archive_run_id = archive.run_id;
    }
    if (!archive.task_directory.empty()) {
      status_.archive_directory = archive.task_directory;
    }
    status_.archive_expected_images = archive.expected_images;
    status_.archive_saved_images = archive.saved_images;
    status_.archive_failed_images = archive.failed_images;
    status_.archive_estimated_bytes = archive.estimated_bytes;
    status_.archive_message = archive.message;
    if (archive.state == ArchiveStatus::FAILED &&
      (active_goal_ || start_pending_since_ || queued_goal_))
    {
      archive_failure_stop_ = true;
      stopping_since_ = std::chrono::steady_clock::now();
      engageHold();
      if (active_goal_) {
        action_client_->async_cancel_goal(active_goal_);
      }
      publishStatus(
        Status::STOPPING,
        "Inspection archive failed; holding and stopping the coverage task before release.");
      return;
    }
    publishProgress();
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
    expireArchiveFinalization();
    if (recovery_locked_) {
      engageHold();
      return;
    }
    expireQueuedStart();
    if (recovery_locked_) {
      return;
    }
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

  /// A recorder can disappear after accepting the finalization RPC but before
  /// replying.  ROS service futures do not fail merely because that server
  /// vanishes, so without this deadline the manager would remain busy forever
  /// after motion had already stopped.  The incomplete directory is retained
  /// for recovery; this only releases the manager and truthfully marks the
  /// formal archive failed.  The generation retires any late response.
  void expireArchiveFinalization()
  {
    if (!archive_finalize_pending_ || !archive_finalize_since_) {
      return;
    }
    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - *archive_finalize_since_).count();
    if (elapsed < archive_finalize_timeout_s_) {
      return;
    }
    archive_finalize_pending_ = false;
    archive_finalize_since_.reset();
    ++archive_finalize_generation_;
    active_inspection_enabled_ = false;
    status_.archive_state = ArchiveStatus::FAILED;
    status_.archive_message = "Archive finalization timed out after " +
      std::to_string(archive_finalize_timeout_s_) + " s; partial data was retained.";
    status_.result_code = ExecuteCoverage::Result::ARCHIVE_FAILED;
    publishStatus(
      Status::FINISHED,
      archive_finalize_message_ + " Archive finalization failed: " + status_.archive_message);
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

  bool holdStatusIsRecent() const
  {
    return last_hold_status_ && std::chrono::duration<double>(
      std::chrono::steady_clock::now() - *last_hold_status_).count() <
           hold_discovery_grace_s_;
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
    const auto result_code = archive_failure_stop_ ?
      ExecuteCoverage::Result::ARCHIVE_FAILED : ExecuteCoverage::Result::EXECUTOR_LOST;
    finalizeRun(
      result_code,
      "Released " + status_.task_id + " after losing the executor: " + evidence + ".",
      FinalizeArchive::Request::FAILED);
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

  /// A queued goal has not reached the executor, but a missing answer to the
  /// hold-release request still leaves the actuator state unknown: the release
  /// may have taken effect and only its reply was lost.  Do not silently return
  /// to READY in that case.  Retire the unsent goal, re-engage hold, and make
  /// the operator explicitly verify the stop before allowing another Start.
  void abandonQueuedStart(const std::string & reason)
  {
    if (!queued_goal_) {
      return;
    }
    const auto task_id = queued_goal_->task.task_id;
    queued_goal_.reset();
    queued_goal_since_.reset();
    hold_release_confirmed_ = false;
    ++goal_generation_;
    // An inspection archive can have been prepared before the release request.
    // It belongs to this never-dispatched task and must not look like a usable
    // completed run after we discard the Goal.
    if (!status_.archive_run_id.empty()) {
      finalizeDetachedArchive(
        status_.archive_run_id, FinalizeArchive::Request::CANCELED,
        reason + " Goal was never dispatched.");
      status_.archive_state = ArchiveStatus::CANCELED;
      status_.archive_message = "Archive canceled before motion started.";
    }
    active_inspection_enabled_ = false;
    recovery_locked_ = true;
    // Do not use engageHold(): a stale true status cannot prove that the
    // preceding release request did not take effect.  Supersede it explicitly.
    maintainHoldRequest(true);
    publishStatus(
      Status::RECOVERY_LOCKED,
      reason + " " + task_id + " was not sent to the executor; speed hold was "
      "re-engaged. Verify the stop before rearming.");
  }

  void expireQueuedStart()
  {
    if (!queued_goal_ || !queued_goal_since_) {
      return;
    }
    if (std::chrono::duration<double>(
        std::chrono::steady_clock::now() - *queued_goal_since_).count() <
      start_response_timeout_s_)
    {
      return;
    }
    abandonQueuedStart("Timed out waiting for speed-hold release.");
  }

  bool requestStart(
    bool inspection_enabled, const std::string & output_root, std::string & response_message)
  {
    expireStalePending();
    if (!cached_task_) {
      response_message = "No valid coverage task is available.";
      return false;
    }
    if (busy()) {
      response_message = "A coverage task is already starting or executing.";
      return false;
    }
    if (!action_client_->action_server_is_ready()) {
      response_message = "Coverage executor Action server is unavailable.";
      return false;
    }
    if (inspection_enabled && !archive_prepare_client_->service_is_ready()) {
      response_message = "Inspection archive recorder is unavailable.";
      return false;
    }
    ExecuteCoverage::Goal goal;
    goal.task = *cached_task_;
    goal.inspection_enabled = inspection_enabled;
    const auto task_id = goal.task.task_id;
    const auto revision = goal.task.revision;

    // The identity always describes whatever the current state is about. A
    // preview cached during the previous run only becomes the reported task
    // here, when it is the one actually being started.
    status_.task_id = task_id;
    status_.revision = revision;
    status_.total_segments = static_cast<uint32_t>(goal.task.segment_types.size());
    status_.current_segment = -1;
    status_.progress = 0.0F;
    status_.planned_total_s = 0.0;
    status_.schedule_lag_s = 0.0;
    status_.estimated_remaining_s = 0.0;
    status_.executor_state = ExecuteCoverage::Feedback::WAITING;
    resetArchiveStatus(inspection_enabled);
    archive_failure_stop_ = false;
    if (!inspection_enabled) {
      beginActionStart(goal);
      response_message = "Motion-only start accepted for " + task_id + " revision " +
        std::to_string(revision) + ".";
      return true;
    }

    const auto generation = ++archive_start_generation_;
    archive_prepare_pending_ = true;
    pending_archive_goal_ = goal;
    status_.archive_state = ArchiveStatus::PREPARING;
    status_.archive_message = "Preparing task archive on the recorder host.";
    publishStatus(
      Status::STARTING,
      "Preparing the inspection archive before starting " + task_id + " revision " +
      std::to_string(revision) + ".");
    auto request = std::make_shared<PrepareArchive::Request>();
    request->task = goal.task;
    request->output_root = output_root;
    archive_prepare_client_->async_send_request(
      request, [this, generation](rclcpp::Client<PrepareArchive>::SharedFuture future) {
        std::shared_ptr<PrepareArchive::Response> prepared;
        try {
          prepared = future.get();
        } catch (const std::exception & error) {
          if (generation == archive_start_generation_) {
            archive_prepare_pending_ = false;
            pending_archive_goal_.reset();
            status_.archive_state = ArchiveStatus::FAILED;
            status_.archive_message = "Archive preparation transport failed: " +
            std::string(error.what());
            publishStatus(Status::READY, status_.archive_message);
          }
          return;
        }
        if (generation != archive_start_generation_) {
          if (prepared->success) {
            finalizeDetachedArchive(
              prepared->run_id, FinalizeArchive::Request::CANCELED,
              "Start was canceled while archive preparation was in flight.");
          }
          return;
        }
        archive_prepare_pending_ = false;
        auto goal_to_start = pending_archive_goal_;
        pending_archive_goal_.reset();
        if (!prepared->success || !goal_to_start) {
          status_.archive_state = ArchiveStatus::FAILED;
          status_.archive_message = prepared->success ?
          "Archive preparation lost its pending task." : prepared->message;
          publishStatus(Status::READY, "Archive preparation failed: " + status_.archive_message);
          return;
        }
        status_.archive_state = ArchiveStatus::READY;
        status_.archive_run_id = prepared->run_id;
        status_.archive_directory = prepared->task_directory;
        status_.archive_preflight_expected_images = prepared->expected_images;
        status_.archive_expected_images = prepared->expected_images;
        status_.archive_estimated_bytes = prepared->estimated_bytes;
        status_.archive_message = prepared->message;
        beginActionStart(*goal_to_start);
      });
    response_message = "Archive preparation accepted for " + task_id + " revision " +
      std::to_string(revision) + "; motion remains held until it succeeds.";
    return true;
  }

  void start(const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    response->success = requestStart(
      inspection_default_enabled_, inspection_output_root_, response->message);
  }

  void startConfigured(
    const StartCoverage::Request & request,
    const std::shared_ptr<StartCoverage::Response> & response)
  {
    const auto root = request.output_root.empty() ? inspection_output_root_ : request.output_root;
    response->success = requestStart(request.inspection_enabled, root, response->message);
    // Archive preparation is intentionally asynchronous: the manager must not
    // block a ROS service callback waiting for another service on the same
    // executor. The resolved run id and directory are published in status as
    // soon as preparation succeeds.
    response->run_id = status_.archive_run_id;
    response->task_directory = status_.archive_directory;
  }

  void beginActionStart(const ExecuteCoverage::Goal & goal)
  {
    const auto task_id = goal.task.task_id;
    const auto revision = goal.task.revision;
    const auto generation = ++goal_generation_;
    active_inspection_enabled_ = goal.inspection_enabled;
    // Whatever is held has to be confirmed released before the Action request
    // exists. Sending both asynchronously made EXECUTING mean only that the
    // controller had a goal, not that the actuator path could move, and a late
    // engage response could win after the new task started.
    // A newly observed watchdog state may arrive before DDS discovers its
    // service. Give that discovery a bounded grace interval; an older latched
    // false from a watchdog which has since gone away must not block a legacy
    // controller forever.
    if (hold_client_->service_is_ready() || holdStatusIsRecent()) {
      queued_goal_ = goal;
      queued_goal_generation_ = generation;
      queued_goal_since_ = std::chrono::steady_clock::now();
      hold_release_confirmed_ = false;
      maintainHoldRequest(false);
      publishStatus(
        Status::STARTING,
        "Releasing the speed hold before starting " + task_id + " revision " +
        std::to_string(revision) + ".");
    } else {
      start_pending_since_ = std::chrono::steady_clock::now();
      publishStatus(
        Status::STARTING,
        "Start requested for " + task_id + " revision " + std::to_string(revision));
      dispatchGoal(goal, generation);
    }
  }

  void finalizeDetachedArchive(
    const std::string & run_id, uint8_t outcome, const std::string & message)
  {
    if (run_id.empty() || !archive_finalize_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<FinalizeArchive::Request>();
    request->run_id = run_id;
    request->outcome = outcome;
    request->message = message;
    archive_finalize_client_->async_send_request(request);
  }

  void finalizeRun(uint16_t result_code, const std::string & message, uint8_t archive_outcome)
  {
    status_.result_code = result_code;
    status_.current_segment = -1;
    status_.executor_state = ExecuteCoverage::Feedback::STOPPED;
    if (!active_inspection_enabled_) {
      publishStatus(Status::FINISHED, message);
      return;
    }
    if (status_.archive_run_id.empty() || !archive_finalize_client_->service_is_ready()) {
      active_inspection_enabled_ = false;
      status_.archive_state = ArchiveStatus::FAILED;
      status_.archive_message = status_.archive_run_id.empty() ?
        "Archive run id was lost before finalization." :
        "Archive recorder is unavailable for finalization.";
      status_.result_code = ExecuteCoverage::Result::ARCHIVE_FAILED;
      publishStatus(Status::FINISHED, message + " Archive finalization failed: " +
        status_.archive_message);
      return;
    }
    archive_finalize_pending_ = true;
    archive_finalize_since_ = std::chrono::steady_clock::now();
    const auto generation = ++archive_finalize_generation_;
    archive_finalize_message_ = message;
    status_.archive_state = ArchiveStatus::FINALIZING;
    status_.archive_message = "Finalizing the archive manifest.";
    publishStatus(Status::FINISHED, message + " Finalizing inspection archive.");
    auto request = std::make_shared<FinalizeArchive::Request>();
    request->run_id = status_.archive_run_id;
    request->outcome = archive_outcome;
    request->message = message;
    archive_finalize_client_->async_send_request(
      request, [this, message, generation](rclcpp::Client<FinalizeArchive>::SharedFuture future) {
        if (generation != archive_finalize_generation_) {
          return;
        }
        archive_finalize_pending_ = false;
        archive_finalize_since_.reset();
        try {
          const auto response = future.get();
          if (!response->success) {
            status_.archive_state = ArchiveStatus::FAILED;
            status_.archive_message = response->message;
            status_.result_code = ExecuteCoverage::Result::ARCHIVE_FAILED;
            active_inspection_enabled_ = false;
            publishStatus(Status::FINISHED, message + " Archive finalization failed: " +
              response->message);
          } else {
            status_.archive_message = response->message;
            active_inspection_enabled_ = false;
            publishStatus(Status::FINISHED, message + " " + response->message);
          }
        } catch (const std::exception & error) {
          status_.archive_state = ArchiveStatus::FAILED;
          status_.archive_message = "Archive finalization transport failed: " +
          std::string(error.what());
          status_.result_code = ExecuteCoverage::Result::ARCHIVE_FAILED;
          active_inspection_enabled_ = false;
          publishStatus(Status::FINISHED, message + " " + status_.archive_message);
        }
      });
  }

  void continueQueuedStart()
  {
    if (!queued_goal_) {
      return;
    }
    if (!hold_release_confirmed_) {
      // Once a release request exists, only its causal service answer can
      // permit motion. Before any request can be sent, a short discovery grace
      // avoids racing a freshly observed watchdog; after it expires this is a
      // deployment with no active watchdog and the old direct path remains
      // available.
      if (!hold_client_->service_is_ready() &&
        (!hold_request_value_ || *hold_request_value_ != false) && !holdStatusIsRecent())
      {
        auto goal = *queued_goal_;
        const auto generation = queued_goal_generation_;
        queued_goal_.reset();
        queued_goal_since_.reset();
        dispatchGoal(goal, generation);
        return;
      }
      maintainHoldRequest(false);
      return;
    }
    if (!action_client_->action_server_is_ready()) {
      abandonQueuedStart("The executor disappeared after speed hold release.");
      return;
    }
    auto goal = *queued_goal_;
    const auto generation = queued_goal_generation_;
    queued_goal_.reset();
    queued_goal_since_.reset();
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
          const bool was_forcibly_abandoned =
            forced_abandoned_generation_ && *forced_abandoned_generation_ == generation;
          if (was_forcibly_abandoned) {
            forced_abandoned_generation_.reset();
          }
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
            if (was_forcibly_abandoned) {
              // The premise under which an operator rearmed was wrong: the
              // unknown request really was accepted. Stop any newer task too,
              // retire all of its callbacks, and require an explicit recovery
              // again rather than hiding the late acceptance in a warning.
              if (active_goal_) {
                action_client_->async_cancel_goal(active_goal_);
              }
              ++goal_generation_;
              start_pending_since_.reset();
              stopping_since_.reset();
              unresolved_goal_response_ = false;
              queued_goal_.reset();
              active_goal_.reset();
              recovery_locked_ = true;
              publishStatus(
                Status::RECOVERY_LOCKED,
                "A forcibly abandoned Goal was accepted late; speed hold is engaged. "
                "Verify the executor and hardware stop before rearming.");
            }
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
        const auto archive_outcome = archive_failure_stop_ ? FinalizeArchive::Request::FAILED :
          (result.result->result_code == ExecuteCoverage::Result::SUCCESS ?
          FinalizeArchive::Request::COMPLETED :
          (result.result->result_code == ExecuteCoverage::Result::CANCELED ?
          FinalizeArchive::Request::CANCELED : FinalizeArchive::Request::FAILED));
        const auto result_code = archive_failure_stop_ ?
          ExecuteCoverage::Result::ARCHIVE_FAILED : result.result->result_code;
        status_.progress = result.result->result_code == ExecuteCoverage::Result::SUCCESS ?
          1.0F : status_.progress;
        finalizeRun(
          result_code, "Execution finished: " + result.result->message, archive_outcome);
      };
    start_pending_since_ = std::chrono::steady_clock::now();
    action_client_->async_send_goal(goal, options);
  }

  void cancel(const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    expireStalePending();
    if (archive_prepare_pending_) {
      // No movement Goal exists yet, so retiring this request is safe. Its
      // eventual archive response is finalized as canceled by its generation
      // guard rather than being allowed to become an orphaned recording dir.
      ++archive_start_generation_;
      archive_prepare_pending_ = false;
      pending_archive_goal_.reset();
      status_.archive_state = ArchiveStatus::CANCELED;
      status_.archive_message = "Archive preparation canceled before motion started.";
      publishStatus(
        cached_task_ ? Status::READY : Status::IDLE,
        "Start canceled while archive preparation was in progress; robot never moved.");
      response->success = true;
      response->message = "Archive preparation canceled before motion started.";
      return;
    }
    if (queued_goal_) {
      abandonQueuedStart("Start canceled while waiting for speed-hold release.");
      response->success = true;
      response->message =
        "Queued start discarded and speed hold re-engaged; verification is required before "
        "rearming.";
      return;
    }
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

  void forceAbandon(const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    expireStalePending();
    if (!stopping_since_ || !unresolved_goal_response_) {
      response->success = false;
      response->message =
        "Force abandon is only available while a start response is unknown.";
      return;
    }

    // Retire the callbacks, but remember exactly which request was abandoned.
    // If its response later proves it was accepted, the callback re-engages
    // RECOVERY_LOCKED even after an operator has already rearmed.
    forced_abandoned_generation_ = goal_generation_;
    ++goal_generation_;
    start_pending_since_.reset();
    stopping_since_.reset();
    unresolved_goal_response_ = false;
    active_goal_.reset();
    recovery_locked_ = true;
    engageHold();
    publishStatus(
      Status::RECOVERY_LOCKED,
      "Unknown Goal supervision was forcibly abandoned. This does not prove the task "
      "stopped; speed hold remains requested. Verify hardware stop or executor shutdown "
      "before rearming.");
    response->success = true;
    response->message =
      "Recovery locked with speed hold requested; physical verification is required.";
  }

  void rearm(const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    if (!recovery_locked_) {
      response->success = false;
      response->message = "The manager is not recovery locked.";
      return;
    }
    recovery_locked_ = false;
    publishStatus(
      cached_task_ ? Status::READY : Status::IDLE,
      "Manager rearmed after operator verification; speed hold remains engaged until "
      "the next Start releases it.");
    response->success = true;
    response->message = "Manager rearmed; the next Start will release the speed hold.";
  }

  std::string frame_id_;
  double start_response_timeout_s_;
  double feedback_publish_period_s_;
  double executor_timeout_s_;
  double command_quiet_s_;
  double hold_response_timeout_s_;
  double hold_discovery_grace_s_;
  double archive_finalize_timeout_s_;
  bool inspection_default_enabled_;
  std::string inspection_output_root_;
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
  std::optional<std::chrono::steady_clock::time_point> last_hold_status_;
  std::optional<std::chrono::steady_clock::time_point> queued_goal_since_;
  std::optional<std::chrono::steady_clock::time_point> archive_finalize_since_;
  // No value until the speed watchdog has published: absent and false are
  // different answers, and only the second one is evidence of anything.
  std::optional<bool> hold_active_;
  std::optional<bool> hold_request_value_;
  bool hold_release_confirmed_{false};
  bool unresolved_goal_response_{false};
  bool recovery_locked_{false};
  bool active_inspection_enabled_{false};
  bool archive_prepare_pending_{false};
  bool archive_finalize_pending_{false};
  bool archive_failure_stop_{false};
  uint64_t hold_generation_{0};
  // Identifies callbacks from one Action request. A timed-out request keeps its
  // generation until its true outcome or a supervised stop retires it.
  uint64_t goal_generation_{0};
  std::optional<uint64_t> forced_abandoned_generation_;
  uint64_t queued_goal_generation_{0};
  uint64_t archive_start_generation_{0};
  uint64_t archive_finalize_generation_{0};
  std::string archive_finalize_message_;
  std::optional<climbot_interfaces::msg::CoverageTask> cached_task_;
  std::optional<ExecuteCoverage::Goal> queued_goal_;
  std::optional<ExecuteCoverage::Goal> pending_archive_goal_;
  GoalHandle::SharedPtr active_goal_;
  rclcpp::Subscription<climbot_interfaces::msg::CoverageTask>::SharedPtr task_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hold_subscription_;
  rclcpp::Subscription<ArchiveStatus>::SharedPtr archive_status_subscription_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr hold_client_;
  rclcpp::Client<PrepareArchive>::SharedPtr archive_prepare_client_;
  rclcpp::Client<FinalizeArchive>::SharedPtr archive_finalize_client_;
  rclcpp_action::Client<ExecuteCoverage>::SharedPtr action_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<StartCoverage>::SharedPtr configured_start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr force_abandon_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr rearm_service_;
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
