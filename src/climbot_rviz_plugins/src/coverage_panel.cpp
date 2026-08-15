#include "climbot_rviz_plugins/coverage_panel.hpp"

#include <memory>
#include <string>

#include <QGridLayout>
#include <QVBoxLayout>

#include "rviz_common/display_context.hpp"

namespace climbot_rviz_plugins
{

namespace
{

QString stateName(uint8_t state)
{
  switch (state) {
    case climbot_interfaces::msg::CoverageStatus::IDLE:
      return QObject::tr("Idle");
    case climbot_interfaces::msg::CoverageStatus::INVALID:
      return QObject::tr("Invalid preview");
    case climbot_interfaces::msg::CoverageStatus::READY:
      return QObject::tr("Ready");
    case climbot_interfaces::msg::CoverageStatus::STARTING:
      return QObject::tr("Starting");
    case climbot_interfaces::msg::CoverageStatus::EXECUTING:
      return QObject::tr("Executing");
    case climbot_interfaces::msg::CoverageStatus::FINISHED:
      return QObject::tr("Finished");
    default:
      return QObject::tr("Unknown (%1)").arg(state);
  }
}

}  // namespace

CoveragePanel::CoveragePanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  state_label_ = new QLabel(tr("Waiting for the coverage manager."));
  task_label_ = new QLabel("-");
  segment_label_ = new QLabel("-");
  message_label_ = new QLabel("-");
  message_label_->setWordWrap(true);
  response_label_ = new QLabel("-");
  response_label_->setWordWrap(true);

  progress_bar_ = new QProgressBar();
  progress_bar_->setRange(0, 100);
  progress_bar_->setValue(0);

  replan_button_ = new QPushButton(tr("Replan"));
  clear_button_ = new QPushButton(tr("Clear points"));
  start_button_ = new QPushButton(tr("Start"));
  cancel_button_ = new QPushButton(tr("Cancel / Stop"));

  auto * fields = new QGridLayout();
  fields->addWidget(new QLabel(tr("State")), 0, 0);
  fields->addWidget(state_label_, 0, 1);
  fields->addWidget(new QLabel(tr("Task")), 1, 0);
  fields->addWidget(task_label_, 1, 1);
  fields->addWidget(new QLabel(tr("Segment")), 2, 0);
  fields->addWidget(segment_label_, 2, 1);
  fields->addWidget(new QLabel(tr("Progress")), 3, 0);
  fields->addWidget(progress_bar_, 3, 1);
  fields->addWidget(new QLabel(tr("Manager")), 4, 0);
  fields->addWidget(message_label_, 4, 1);
  fields->addWidget(new QLabel(tr("Last request")), 5, 0);
  fields->addWidget(response_label_, 5, 1);
  fields->setColumnStretch(1, 1);

  auto * buttons = new QGridLayout();
  buttons->addWidget(replan_button_, 0, 0);
  buttons->addWidget(clear_button_, 0, 1);
  buttons->addWidget(start_button_, 1, 0);
  buttons->addWidget(cancel_button_, 1, 1);

  auto * layout = new QVBoxLayout();
  layout->addLayout(fields);
  layout->addLayout(buttons);
  layout->addStretch(1);
  setLayout(layout);

  connect(replan_button_, &QPushButton::clicked, this, &CoveragePanel::onReplan);
  connect(clear_button_, &QPushButton::clicked, this, &CoveragePanel::onClearPoints);
  connect(start_button_, &QPushButton::clicked, this, &CoveragePanel::onStart);
  connect(cancel_button_, &QPushButton::clicked, this, &CoveragePanel::onCancel);

  renderDisconnected();
}

void CoveragePanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  status_subscription_ = node_->create_subscription<Status>(
    "/coverage/manager_status", rclcpp::QoS(1).reliable().transient_local(),
    [this](const Status::SharedPtr status) {
      const std::lock_guard<std::mutex> lock(mutex_);
      status_ = std::make_unique<Status>(*status);
    });
  replan_client_ = node_->create_client<Trigger>("/coverage/replan");
  clear_client_ = node_->create_client<Trigger>("/coverage/clear_points");
  start_client_ = node_->create_client<Trigger>("/coverage/start");
  cancel_client_ = node_->create_client<Trigger>("/coverage/cancel");

  // Subscription and service callbacks run on the executor thread, which must
  // never touch widgets. The timer runs on the Qt thread and is the only place
  // this panel reads the shared state and repaints.
  refresh_timer_ = new QTimer(this);
  connect(refresh_timer_, &QTimer::timeout, this, &CoveragePanel::refresh);
  refresh_timer_->start(100);
}

void CoveragePanel::note(const QString & text)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  response_ = text;
}

void CoveragePanel::call(
  const rclcpp::Client<Trigger>::SharedPtr & client, const QString & label)
{
  if (!client || !client->service_is_ready()) {
    note(tr("%1: service unavailable.").arg(label));
    return;
  }
  note(tr("%1: sent.").arg(label));
  client->async_send_request(
    std::make_shared<Trigger::Request>(),
    [this, label](rclcpp::Client<Trigger>::SharedFuture future) {
      const auto response = future.get();
      note(
        tr("%1: %2 %3").arg(label)
        .arg(response->success ? tr("ok") : tr("refused"))
        .arg(QString::fromStdString(response->message)));
    });
}

void CoveragePanel::onReplan()
{
  call(replan_client_, tr("Replan"));
}

void CoveragePanel::onClearPoints()
{
  call(clear_client_, tr("Clear points"));
}

void CoveragePanel::onStart()
{
  call(start_client_, tr("Start"));
}

void CoveragePanel::onCancel()
{
  call(cancel_client_, tr("Cancel"));
}

void CoveragePanel::renderDisconnected()
{
  state_label_->setText(tr("No coverage manager status received."));
  task_label_->setText("-");
  segment_label_->setText("-");
  progress_bar_->setValue(0);
  message_label_->setText("-");
  // Every button stays available: the manager, not this panel, decides whether
  // a request is legal, and greying everything out would hide that decision.
  replan_button_->setEnabled(true);
  clear_button_->setEnabled(true);
  start_button_->setEnabled(false);
  cancel_button_->setEnabled(false);
}

void CoveragePanel::renderStatus(const Status & status)
{
  state_label_->setText(stateName(status.state));
  task_label_->setText(
    status.task_id.empty() ?
    tr("none") :
    tr("%1  revision %2")
    .arg(QString::fromStdString(status.task_id))
    .arg(status.revision));

  if (status.total_segments == 0U) {
    segment_label_->setText("-");
  } else if (status.current_segment < 0) {
    segment_label_->setText(tr("approach of %1").arg(status.total_segments));
  } else {
    segment_label_->setText(
      tr("%1 of %2").arg(status.current_segment + 1).arg(status.total_segments));
  }

  progress_bar_->setValue(static_cast<int>(status.progress * 100.0F + 0.5F));
  message_label_->setText(QString::fromStdString(status.message));

  // Enabling follows the published state only. It is a hint, not a guard: the
  // manager rejects an illegal request regardless of what this panel shows.
  const bool executing = status.state == Status::EXECUTING;
  const bool starting = status.state == Status::STARTING;
  start_button_->setEnabled(status.state == Status::READY);
  cancel_button_->setEnabled(executing);
  replan_button_->setEnabled(!executing && !starting);
  clear_button_->setEnabled(!executing && !starting);
}

void CoveragePanel::refresh()
{
  std::unique_ptr<Status> status;
  QString response;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (status_) {
      status = std::make_unique<Status>(*status_);
    }
    response = response_;
  }
  if (status) {
    renderStatus(*status);
  } else {
    renderDisconnected();
  }
  response_label_->setText(response.isEmpty() ? "-" : response);
}

}  // namespace climbot_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(climbot_rviz_plugins::CoveragePanel, rviz_common::Panel)
