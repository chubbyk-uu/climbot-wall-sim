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

#ifndef CLIMBOT_RVIZ_PLUGINS__COVERAGE_PANEL_HPP_
#define CLIMBOT_RVIZ_PLUGINS__COVERAGE_PANEL_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <QComboBox>
#include <QCheckBox>
#include <QLabel>
#include <QLineEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QTimer>
#include <QWidget>

#include "climbot_interfaces/msg/coverage_config.hpp"
#include "climbot_interfaces/msg/coverage_status.hpp"
#include "climbot_interfaces/srv/configure_coverage.hpp"
#include "climbot_interfaces/srv/start_coverage.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace climbot_rviz_plugins
{

/// Text a narrow dock can break, whatever is in it.
///
/// Word wrapping only breaks at spaces, so an identifier such as
/// coverage_20260817_143512_rectangle is cut off mid-token in a dock narrower
/// than the id. This adds a break opportunity after each separator, which the
/// line breaker uses only when the line would otherwise overflow.
QString wrappableText(const QString & text);

/// How long a request may go unanswered before the panel stops waiting.
///
/// A service that passes service_is_ready() and then dies leaves its future
/// forever unfulfilled. Long enough that no answer from a live service is
/// mistaken for none - these are local services answering in well under a
/// tenth of it - and short enough that an operator whose planner died does not
/// sit in front of frozen controls wondering which of them to press.
std::chrono::milliseconds requestTimeout();

/// Whether a request sent at `sent` has waited past that point.
bool requestHasExpired(
  std::chrono::steady_clock::time_point sent,
  std::chrono::steady_clock::time_point now);

/// Decides whether an answer that has just arrived is still the one waited for.
///
/// Releasing a request that timed out does not retract it. The service can
/// still answer, minutes later, and its callback carries whatever it was going
/// to say: the configuration in force when it was asked, the tracking mode as
/// it was then. Applying that to a panel the operator has since driven
/// somewhere else walks the display backwards - the second request's pending
/// flag is cleared by the first request's answer, and the boxes repaint to a
/// state nobody chose.
///
/// Every request takes a generation on the way out and presents it on the way
/// back. Timing out, and sending again, each retire the one before.
class RequestGate
{
public:
  /// Claim the generation for a request about to be sent, retiring any before.
  uint64_t begin();
  /// Whether an answer stamped `generation` is the one still being waited for.
  bool isCurrent(uint64_t generation) const;
  /// Give up on the request in flight, so no later answer for it counts.
  void abandon();
  /// Whether a request is in flight and still wanted.
  bool waiting() const;
  /// Stop waiting because the current request has just been answered.
  void settle();

private:
  uint64_t generation_{0};
  bool waiting_{false};
};

/// Operator panel for planning, starting and stopping a coverage task.
class CoveragePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  using Status = climbot_interfaces::msg::CoverageStatus;
  using Config = climbot_interfaces::msg::CoverageConfig;

  explicit CoveragePanel(QWidget * parent = nullptr);
  ~CoveragePanel() override;

  void onInitialize() override;

  /// Paint one manager status. Public so the layout test can drive the real
  /// render path rather than a copy of it.
  void renderStatus(const Status & status);
  void renderDisconnected();
  /// Paint one planner configuration. Public for the same reason.
  void renderConfig(const Config & config);

private Q_SLOTS:
  void onReplan();
  void onClearPoints();
  void onStart();
  void onPause();
  void onResume();
  void onCancel();
  void onForceAbandon();
  void onRearm();
  void onConfigurationChosen();
  void onAlgorithmChosen();
  void refresh();

private:
  using Trigger = std_srvs::srv::Trigger;
  using Configure = climbot_interfaces::srv::ConfigureCoverage;
  using StartCoverage = climbot_interfaces::srv::StartCoverage;

  /// Everything the executor thread writes and the Qt thread reads, in one
  /// object held by shared_ptr and captured by value in every callback.
  ///
  /// RViz may remove a panel at any time, and the node this panel spins on
  /// belongs to RViz rather than to the panel, so there is no executor a
  /// destructor could stop first and no point at which an in-flight callback
  /// is known to be finished. A callback capturing the panel would then write
  /// through a destroyed object; a weak_ptr sentinel only narrows that window,
  /// because the panel can still be destroyed after the sentinel has been
  /// locked. Callbacks therefore keep the state alive themselves, and a
  /// callback that outlives its panel writes somewhere harmless.
  struct SharedState;

  void call(const rclcpp::Client<Trigger>::SharedPtr & client, const QString & label);
  void callConfiguredStart();
  void readTrackingMode();
  void note(const QString & text);
  /// Release a request whose answer never came. A service that passes
  /// service_is_ready() and then dies leaves its future forever unfulfilled,
  /// and the flags below are cleared only in the response callbacks, so the
  /// controls they disable would stay disabled until RViz itself restarts.
  void expireStalePendingRequests();

  QLabel * state_label_{nullptr};
  QLabel * task_label_{nullptr};
  QLabel * segment_label_{nullptr};
  QLabel * schedule_label_{nullptr};
  QProgressBar * progress_bar_{nullptr};
  QLabel * message_label_{nullptr};
  QLabel * planner_label_{nullptr};
  QLabel * response_label_{nullptr};
  QLabel * inspection_summary_label_{nullptr};
  QLabel * archive_count_label_{nullptr};
  QLabel * archive_directory_label_{nullptr};
  QLabel * archive_error_label_{nullptr};
  QComboBox * region_box_{nullptr};
  QComboBox * sweep_box_{nullptr};
  QComboBox * algorithm_box_{nullptr};
  QLabel * selection_label_{nullptr};
  QCheckBox * inspection_enabled_box_{nullptr};
  QLineEdit * archive_root_edit_{nullptr};
  QPushButton * browse_archive_root_button_{nullptr};
  QPushButton * default_archive_root_button_{nullptr};
  QPushButton * replan_button_{nullptr};
  QPushButton * clear_button_{nullptr};
  QPushButton * start_button_{nullptr};
  QPushButton * pause_button_{nullptr};
  QPushButton * resume_button_{nullptr};
  QPushButton * cancel_button_{nullptr};
  QWidget * recovery_controls_{nullptr};
  QPushButton * force_abandon_button_{nullptr};
  QPushButton * rearm_button_{nullptr};
  QTimer * refresh_timer_{nullptr};
  // Qt-thread only: the mode the last refresh painted with, so renderStatus
  // can label the schedule without reaching for the guarded copy.
  QString rendered_tracking_mode_;
  // Qt-thread only. The manager owns the resolved launch/env/home default;
  // an operator edit is a per-request override and never mutates that default.
  QString manager_archive_root_;
  bool archive_root_overridden_{false};
  // Qt-thread only. Set from the manager's operation permission bits rather
  // than from a state comparison here, so a recovery lock also freezes task
  // planning even though there is no cancellable Goal handle.
  bool task_running_{false};
  bool force_confirmation_armed_{false};
  std::chrono::steady_clock::time_point force_confirmation_deadline_{};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<Status>::SharedPtr status_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planner_subscription_;
  rclcpp::Subscription<Config>::SharedPtr config_subscription_;
  rclcpp::Client<Configure>::SharedPtr configure_client_;
  rclcpp::Client<Trigger>::SharedPtr replan_client_;
  rclcpp::Client<Trigger>::SharedPtr clear_client_;
  rclcpp::Client<StartCoverage>::SharedPtr start_client_;
  rclcpp::Client<Trigger>::SharedPtr pause_client_;
  rclcpp::Client<Trigger>::SharedPtr resume_client_;
  rclcpp::Client<Trigger>::SharedPtr cancel_client_;
  rclcpp::Client<Trigger>::SharedPtr force_abandon_client_;
  rclcpp::Client<Trigger>::SharedPtr rearm_client_;
  // The tracking mode is a parameter of the executor, not part of the planner's
  // configuration service, so it is read and written where it actually lives.
  rclcpp::AsyncParametersClient::SharedPtr tracking_client_;

  // Written by the ROS executor thread and read by the Qt thread, so every
  // access is guarded by the mutex inside it. Widgets are only ever touched
  // from the refresh timer.
  std::shared_ptr<SharedState> state_;
};

}  // namespace climbot_rviz_plugins

#endif  // CLIMBOT_RVIZ_PLUGINS__COVERAGE_PANEL_HPP_
