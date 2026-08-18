#ifndef CLIMBOT_RVIZ_PLUGINS__COVERAGE_PANEL_HPP_
#define CLIMBOT_RVIZ_PLUGINS__COVERAGE_PANEL_HPP_

#include <memory>
#include <mutex>
#include <string>

#include <QComboBox>
#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QTimer>
#include <QWidget>

#include "climbot_interfaces/msg/coverage_config.hpp"
#include "climbot_interfaces/msg/coverage_status.hpp"
#include "climbot_interfaces/srv/configure_coverage.hpp"
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

/// Operator panel for planning, starting and stopping a coverage task.
class CoveragePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  using Status = climbot_interfaces::msg::CoverageStatus;
  using Config = climbot_interfaces::msg::CoverageConfig;

  explicit CoveragePanel(QWidget * parent = nullptr);

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
  void onCancel();
  void onConfigurationChosen();
  void onAlgorithmChosen();
  void refresh();

private:
  using Trigger = std_srvs::srv::Trigger;
  using Configure = climbot_interfaces::srv::ConfigureCoverage;

  void call(const rclcpp::Client<Trigger>::SharedPtr & client, const QString & label);
  void readTrackingMode();
  void note(const QString & text);

  QLabel * state_label_{nullptr};
  QLabel * task_label_{nullptr};
  QLabel * segment_label_{nullptr};
  QLabel * schedule_label_{nullptr};
  QProgressBar * progress_bar_{nullptr};
  QLabel * message_label_{nullptr};
  QLabel * planner_label_{nullptr};
  QLabel * response_label_{nullptr};
  QComboBox * region_box_{nullptr};
  QComboBox * sweep_box_{nullptr};
  QComboBox * algorithm_box_{nullptr};
  QLabel * selection_label_{nullptr};
  QPushButton * replan_button_{nullptr};
  QPushButton * clear_button_{nullptr};
  QPushButton * start_button_{nullptr};
  QPushButton * cancel_button_{nullptr};
  QTimer * refresh_timer_{nullptr};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<Status>::SharedPtr status_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planner_subscription_;
  rclcpp::Subscription<Config>::SharedPtr config_subscription_;
  rclcpp::Client<Configure>::SharedPtr configure_client_;
  rclcpp::Client<Trigger>::SharedPtr replan_client_;
  rclcpp::Client<Trigger>::SharedPtr clear_client_;
  rclcpp::Client<Trigger>::SharedPtr start_client_;
  rclcpp::Client<Trigger>::SharedPtr cancel_client_;
  // The tracking mode is a parameter of the executor, not part of the planner's
  // configuration service, so it is read and written where it actually lives.
  rclcpp::AsyncParametersClient::SharedPtr tracking_client_;

  // Written by the ROS executor thread and read by the Qt thread, so every
  // access is guarded. Widgets are only ever touched from the refresh timer.
  std::mutex mutex_;
  std::unique_ptr<Status> status_;
  std::unique_ptr<Config> config_;
  QString planner_;
  QString response_;
  // Set while a configure request is in flight so a second one cannot be
  // launched from a control the first has not finished answering for.
  bool configure_pending_{false};
  // Empty until the executor has answered once; the box shows nothing
  // selectable until then rather than a guess that may be wrong.
  QString tracking_mode_;
  bool tracking_pending_{false};
};

}  // namespace climbot_rviz_plugins

#endif  // CLIMBOT_RVIZ_PLUGINS__COVERAGE_PANEL_HPP_
