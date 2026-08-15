#ifndef CLIMBOT_RVIZ_PLUGINS__COVERAGE_PANEL_HPP_
#define CLIMBOT_RVIZ_PLUGINS__COVERAGE_PANEL_HPP_

#include <memory>
#include <mutex>
#include <string>

#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QTimer>
#include <QWidget>

#include "climbot_interfaces/msg/coverage_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace climbot_rviz_plugins
{

/// Operator panel for planning, starting and stopping a coverage task.
class CoveragePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit CoveragePanel(QWidget * parent = nullptr);

  void onInitialize() override;

private Q_SLOTS:
  void onReplan();
  void onClearPoints();
  void onStart();
  void onCancel();
  void refresh();

private:
  using Status = climbot_interfaces::msg::CoverageStatus;
  using Trigger = std_srvs::srv::Trigger;

  void call(const rclcpp::Client<Trigger>::SharedPtr & client, const QString & label);
  void note(const QString & text);
  void renderStatus(const Status & status);
  void renderDisconnected();

  QLabel * state_label_{nullptr};
  QLabel * task_label_{nullptr};
  QLabel * segment_label_{nullptr};
  QProgressBar * progress_bar_{nullptr};
  QLabel * message_label_{nullptr};
  QLabel * response_label_{nullptr};
  QPushButton * replan_button_{nullptr};
  QPushButton * clear_button_{nullptr};
  QPushButton * start_button_{nullptr};
  QPushButton * cancel_button_{nullptr};
  QTimer * refresh_timer_{nullptr};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<Status>::SharedPtr status_subscription_;
  rclcpp::Client<Trigger>::SharedPtr replan_client_;
  rclcpp::Client<Trigger>::SharedPtr clear_client_;
  rclcpp::Client<Trigger>::SharedPtr start_client_;
  rclcpp::Client<Trigger>::SharedPtr cancel_client_;

  // Written by the ROS executor thread and read by the Qt thread, so every
  // access is guarded. Widgets are only ever touched from the refresh timer.
  std::mutex mutex_;
  std::unique_ptr<Status> status_;
  QString response_;
};

}  // namespace climbot_rviz_plugins

#endif  // CLIMBOT_RVIZ_PLUGINS__COVERAGE_PANEL_HPP_
