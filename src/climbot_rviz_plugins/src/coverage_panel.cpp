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

#include "climbot_rviz_plugins/coverage_panel.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <QDir>
#include <QFileDialog>
#include <QFont>
#include <QFontMetrics>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QSignalBlocker>
#include <QScrollArea>
#include <QSizePolicy>
#include <QTabWidget>
#include <QVBoxLayout>

#include "climbot_interfaces/msg/inspection_archive_status.hpp"
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
    case climbot_interfaces::msg::CoverageStatus::STOPPING:
      // Not "Finished" and not "Executing": contact with the executor is gone
      // and the manager has not yet established that the robot has stopped.
      // The stop button stays enabled here, from can_cancel, because this is
      // the state in which it is most worth pressing.
      return QObject::tr("Stopping (executor lost)");
    case climbot_interfaces::msg::CoverageStatus::RECOVERY_LOCKED:
      return QObject::tr("Recovery locked");
    case climbot_interfaces::msg::CoverageStatus::PAUSING:
      return QObject::tr("Pausing");
    case climbot_interfaces::msg::CoverageStatus::PAUSED:
      return QObject::tr("Paused");
    case climbot_interfaces::msg::CoverageStatus::RESUMING:
      return QObject::tr("Resuming");
    default:
      return QObject::tr("Unknown (%1)").arg(state);
  }
}

QString defaultArchiveRoot()
{
  const QString environment = qEnvironmentVariable("CLIMBOT_DATA_ROOT");
  if (!environment.isEmpty()) {
    return environment;
  }
  return QDir::homePath() + QStringLiteral("/climbot_data");
}

QString archiveStateText(uint8_t state)
{
  using Archive = climbot_interfaces::msg::InspectionArchiveStatus;
  switch (state) {
    case Archive::IDLE:
      return QObject::tr("Off");
    case Archive::PREPARING:
      return QObject::tr("Preparing");
    case Archive::READY:
      return QObject::tr("Ready");
    case Archive::RECORDING:
      return QObject::tr("Recording");
    case Archive::FINALIZING:
      return QObject::tr("Finalizing");
    case Archive::COMPLETED:
      return QObject::tr("Completed");
    case Archive::CANCELED:
      return QObject::tr("Canceled");
    case Archive::FAILED:
      return QObject::tr("Failed");
    default:
      return QObject::tr("Unknown");
  }
}

/// A label that wraps instead of widening the dock it lives in.
///
/// The default policy makes a label demand its whole string on one line, and a
/// QGridLayout hands that demand straight to the dock. With a task id and three
/// full sentences on show that came to 566 px, far more than the dock beside the
/// render view gets, so RViz cut the text off rather than honour it. Ignored
/// means the label accepts whatever width the panel has and answers for its own
/// height through heightForWidth.
QLabel * makeValue(const char * name)
{
  auto * label = new QLabel("-");
  label->setObjectName(name);
  label->setWordWrap(true);
  label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Minimum);
  label->setTextInteractionFlags(Qt::TextSelectableByMouse);
  return label;
}

/// Caption for a message that is too long to sit beside its name.
QLabel * makeCaption(const QString & text)
{
  auto * caption = new QLabel(text);
  QFont font = caption->font();
  font.setBold(true);
  caption->setFont(font);
  caption->setWordWrap(true);
  caption->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Minimum);
  return caption;
}

/// A duration as an operator reads a clock, not as a count of seconds.
QString clockText(double seconds)
{
  const int rounded = static_cast<int>(seconds + 0.5);
  return QStringLiteral("%1:%2")
         .arg(rounded / 60)
         .arg(rounded % 60, 2, 10, QLatin1Char('0'));
}

/// The schedule row: how long the task was planned to take, how much is left,
/// and how far behind it is running. A dash before a task starts, when there
/// is nothing to report yet.
///
/// The duration model predicts both modes and its per-segment overheads were
/// in fact fitted on distance-mode runs, so the total is no less accurate
/// there. What differs is whether anything answers for it. In time mode the
/// robot is driven from this schedule and the residual is measured and
/// reported, so the figure is a commitment. In distance mode nothing enforces
/// or observes it: a robot that slips more than the model assumed simply runs
/// late and says nothing. That is worth telling an operator, and it is a
/// different statement from the number being imprecise.
QString scheduleText(
  const climbot_interfaces::msg::CoverageStatus & status, const QString & tracking_mode)
{
  if (!(status.planned_total_s > 0.0)) {
    return QStringLiteral("-");
  }
  QString text = QObject::tr("total %1  ·  left ~%2")
    .arg(clockText(status.planned_total_s))
    .arg(clockText(status.estimated_remaining_s));
  if (tracking_mode == QLatin1String("distance")) {
    return text + QObject::tr("  ·  estimate only");
  }
  // Always, and to two decimals. A threshold below which the lag was hidden
  // was set before the lag had been measured, and the acceleration feedforward
  // then brought its peak to 0.03-0.05 s - under any such threshold, so the
  // figure never appeared and there was no way to tell a run that was on
  // schedule from a feature that was not working. A live number that moves is
  // the evidence that the schedule is being followed, and it grows on its own
  // when it stops being.
  return text + QObject::tr("  ·  %1%2 s")
         .arg(status.schedule_lag_s < 0.0 ? QStringLiteral("-") : QStringLiteral("+"))
         .arg(std::abs(status.schedule_lag_s), 0, 'f', 2);
}

QFrame * makeSeparator()
{
  auto * line = new QFrame();
  line->setFrameShape(QFrame::HLine);
  line->setFrameShadow(QFrame::Sunken);
  return line;
}

}  // namespace

std::chrono::milliseconds requestTimeout()
{
  return std::chrono::seconds{3};
}

bool requestHasExpired(
  std::chrono::steady_clock::time_point sent,
  std::chrono::steady_clock::time_point now)
{
  // A clock that has not moved cannot have expired anything. sent is stamped
  // from the same steady clock, so now is never before it, but reading the
  // comparison as "elapsed" makes that assumption visible.
  return now > sent && now - sent > requestTimeout();
}

QString wrappableText(const QString & text)
{
  // A zero-width space is a break opportunity the line breaker takes only when
  // the line would otherwise overflow, so a wide dock still shows an id whole.
  QString broken;
  broken.reserve(text.size() * 2);
  for (const QChar character : text) {
    broken.append(character);
    if (character == '_' || character == '/' || character == '-') {
      broken.append(QChar(0x200B));
    }
  }
  return broken;
}

uint64_t RequestGate::begin()
{
  ++generation_;
  waiting_ = true;
  return generation_;
}

bool RequestGate::isCurrent(uint64_t generation) const
{
  return waiting_ && generation == generation_;
}

void RequestGate::abandon()
{
  // The generation moves even though nothing new was sent. The request that
  // was in flight keeps its old number, so its answer no longer matches.
  ++generation_;
  waiting_ = false;
}

bool RequestGate::waiting() const
{
  return waiting_;
}

void RequestGate::settle()
{
  waiting_ = false;
}

struct CoveragePanel::SharedState
{
  std::mutex mutex;
  std::unique_ptr<Status> status;
  std::unique_ptr<Config> config;
  QString planner;
  QString response;
  // Set while a configure request is in flight so a second one cannot be
  // launched from a control the first has not finished answering for, with the
  // instant it was sent so a request that is never answered can be released.
  // The gate beside it decides whether an answer that does arrive is still the
  // one being waited for; releasing a request does not retract it.
  RequestGate configure_gate;
  std::chrono::steady_clock::time_point configure_sent{};
  // Empty until the executor has answered once; the box shows nothing
  // selectable until then rather than a guess that may be wrong.
  QString tracking_mode;
  RequestGate tracking_gate;
  std::chrono::steady_clock::time_point tracking_sent{};
};

CoveragePanel::CoveragePanel(QWidget * parent)
: rviz_common::Panel(parent), state_(std::make_shared<SharedState>())
{
  state_label_ = makeValue("state_value");
  task_label_ = makeValue("task_value");
  segment_label_ = makeValue("segment_value");
  message_label_ = makeValue("manager_value");
  planner_label_ = makeValue("planner_value");
  response_label_ = makeValue("response_value");
  inspection_summary_label_ = makeValue("inspection_summary_value");
  archive_count_label_ = makeValue("archive_count_value");
  archive_directory_label_ = makeValue("archive_directory_value");
  archive_error_label_ = makeValue("archive_error_value");

  schedule_label_ = makeValue("schedule_value");

  progress_bar_ = new QProgressBar();
  progress_bar_->setObjectName("progress_value");
  progress_bar_->setRange(0, 100);
  progress_bar_->setValue(0);
  progress_bar_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);

  // The drop-downs carry the value, not the display text, so a translated
  // build still sends the planner the words it validates.
  region_box_ = new QComboBox();
  region_box_->setObjectName("region_box");
  region_box_->addItem(tr("Rectangle"), QStringLiteral("rectangle"));
  region_box_->addItem(tr("Trapezoid"), QStringLiteral("trapezoid"));
  sweep_box_ = new QComboBox();
  sweep_box_->setObjectName("sweep_box");
  sweep_box_->addItem(tr("Horizontal"), QStringLiteral("horizontal"));
  sweep_box_->addItem(tr("Vertical"), QStringLiteral("vertical"));
  // Belongs to the executor rather than the planner, and sits with the other
  // drop-downs anyway: an operator choosing what to sweep and how to drive it
  // is making one decision, and splitting the two across the panel would only
  // reflect which node happens to own each.
  algorithm_box_ = new QComboBox();
  algorithm_box_->setObjectName("algorithm_box");
  // Timed first, because it is the default the executor comes up in and the
  // box shows item zero for the moment before the executor answers with what
  // is actually in force. It is disabled until then, so this is what an
  // operator reads while it is greyed out.
  algorithm_box_->addItem(tr("Timed trajectory"), QStringLiteral("time"));
  algorithm_box_->addItem(tr("Position only"), QStringLiteral("distance"));
  selection_label_ = makeValue("selection_value");
  inspection_enabled_box_ = new QCheckBox(tr("Capture raw images"));
  inspection_enabled_box_->setObjectName("inspection_enabled_box");
  inspection_enabled_box_->setChecked(true);
  archive_root_edit_ = new QLineEdit();
  archive_root_edit_->setObjectName("archive_root_edit");
  archive_root_edit_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
  manager_archive_root_ = defaultArchiveRoot();
  archive_root_edit_->setText(manager_archive_root_);
  archive_root_edit_->setToolTip(tr(
    "Path interpreted by the archive recorder host. The manager resolves its launch argument, "
    "CLIMBOT_DATA_ROOT, or the recorder user's home default; editing this field overrides only "
    "coverage tasks started from this panel."));
  browse_archive_root_button_ = new QPushButton(tr("Browse..."));
  browse_archive_root_button_->setObjectName("browse_archive_root_button");
  default_archive_root_button_ = new QPushButton(tr("Default"));
  default_archive_root_button_->setObjectName("default_archive_root_button");

  replan_button_ = new QPushButton(tr("Replan"));
  clear_button_ = new QPushButton(tr("Clear points"));
  start_button_ = new QPushButton(tr("Start"));
  pause_button_ = new QPushButton(tr("Pause"));
  resume_button_ = new QPushButton(tr("Resume"));
  cancel_button_ = new QPushButton(tr("Stop"));
  force_abandon_button_ = new QPushButton(tr("Force abandon"));
  rearm_button_ = new QPushButton(tr("Rearm after verification"));
  // Named so a test can reach them. Without names the one test that already
  // checked a button's state had to guard against not finding it, which turned
  // the check into a no-op.
  replan_button_->setObjectName("replan_button");
  clear_button_->setObjectName("clear_button");
  start_button_->setObjectName("start_button");
  pause_button_->setObjectName("pause_button");
  resume_button_->setObjectName("resume_button");
  cancel_button_->setObjectName("cancel_button");
  force_abandon_button_->setObjectName("force_abandon_button");
  rearm_button_->setObjectName("rearm_button");
  // The buttons keep their default policy, unlike the labels. A label with no
  // room wraps; a button label with no room is simply cut, and "Cancel / Sto"
  // is not a control an operator should have to act on. So the buttons are
  // what sets the floor on how narrow this dock may go.

  // The compact status summary stays above every tab. An operator changing
  // inspection settings must still see whether the robot is stopping, and an
  // operator viewing planning must still see a recorder failure.
  auto * overview = new QGridLayout();
  overview->addWidget(new QLabel(tr("State")), 0, 0);
  overview->addWidget(state_label_, 0, 1);
  overview->addWidget(new QLabel(tr("Segment")), 1, 0);
  overview->addWidget(segment_label_, 1, 1);
  overview->addWidget(new QLabel(tr("Progress")), 2, 0);
  overview->addWidget(progress_bar_, 2, 1);
  overview->addWidget(new QLabel(tr("Inspection")), 3, 0);
  overview->addWidget(inspection_summary_label_, 3, 1);
  overview->setColumnStretch(1, 1);

  // Planning controls live on their own page, not beside long task and error
  // messages. This keeps the dock usable at its existing width.
  auto * planning_fields = new QGridLayout();
  planning_fields->addWidget(new QLabel(tr("Region")), 0, 0);
  planning_fields->addWidget(region_box_, 0, 1);
  planning_fields->addWidget(new QLabel(tr("Sweep")), 1, 0);
  planning_fields->addWidget(sweep_box_, 1, 1);
  planning_fields->addWidget(new QLabel(tr("Algorithm")), 2, 0);
  planning_fields->addWidget(algorithm_box_, 2, 1);
  planning_fields->addWidget(new QLabel(tr("Points")), 3, 0);
  planning_fields->addWidget(selection_label_, 3, 1);
  planning_fields->setColumnStretch(1, 1);
  // A combo box will happily grow to its widest item and drag the dock with
  // it; it may elide instead, like the labels around it.
  for (auto * box : {region_box_, sweep_box_, algorithm_box_}) {
    box->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
  }
  // A grid hands the value column whatever the name column leaves, which in a
  // narrow dock was less than the word "Executing" needs. State names have no
  // break opportunity inside them, so the floor is the widest one, measured in
  // the theme's own font rather than guessed in pixels.
  const QFontMetrics metrics(state_label_->font());
  int widest_state = 0;
  for (const auto & name : {tr("Executing"), tr("Finished"), tr("Starting"),
      tr("Invalid"), tr("preview"), tr("approach"), tr("connected")})
  {
    widest_state = std::max(widest_state, metrics.horizontalAdvance(name));
  }
  overview->setColumnMinimumWidth(1, widest_state);

  auto * planning_body = new QVBoxLayout();
  planning_body->addLayout(planning_fields);
  auto * planning_buttons = new QGridLayout();
  planning_buttons->addWidget(replan_button_, 0, 0);
  planning_buttons->addWidget(clear_button_, 0, 1);
  planning_body->addLayout(planning_buttons);
  planning_body->addStretch(1);
  auto * planning_page = new QWidget();
  planning_page->setLayout(planning_body);

  auto * inspection_body = new QVBoxLayout();
  inspection_body->addWidget(inspection_enabled_box_);
  inspection_body->addWidget(makeCaption(tr("Data root (recorder host)")));
  inspection_body->addWidget(archive_root_edit_);
  auto * root_buttons = new QHBoxLayout();
  root_buttons->addWidget(browse_archive_root_button_);
  root_buttons->addWidget(default_archive_root_button_);
  root_buttons->addStretch(1);
  inspection_body->addLayout(root_buttons);
  inspection_body->addWidget(makeCaption(tr("Nominal / frozen / saved / failed")));
  inspection_body->addWidget(archive_count_label_);
  inspection_body->addWidget(makeCaption(tr("Task archive directory")));
  inspection_body->addWidget(archive_directory_label_);
  inspection_body->addWidget(makeCaption(tr("Archive status")));
  inspection_body->addWidget(archive_error_label_);
  inspection_body->addStretch(1);
  auto * inspection_page = new QWidget();
  inspection_page->setLayout(inspection_body);

  // Long messages are useful for diagnosis but should not consume height in
  // the routine plan/capture workflow.
  auto * detail_body = new QVBoxLayout();
  detail_body->addWidget(makeCaption(tr("Task")));
  detail_body->addWidget(task_label_);
  detail_body->addWidget(makeCaption(tr("Schedule")));
  detail_body->addWidget(schedule_label_);
  detail_body->addWidget(makeCaption(tr("Manager")));
  detail_body->addWidget(message_label_);
  detail_body->addWidget(makeCaption(tr("Planner")));
  detail_body->addWidget(planner_label_);
  detail_body->addWidget(makeCaption(tr("Last request")));
  detail_body->addWidget(response_label_);
  detail_body->addStretch(1);
  auto * detail_page = new QWidget();
  detail_page->setLayout(detail_body);

  auto make_scroll_page = [](QWidget * page, const QString & name) {
      page->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
      page->setMinimumWidth(0);
      auto * scroll = new QScrollArea();
      scroll->setObjectName(name);
      scroll->setWidget(page);
      scroll->setWidgetResizable(true);
      scroll->setFrameShape(QFrame::NoFrame);
      scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
      return scroll;
    };
  auto * tabs = new QTabWidget();
  tabs->setObjectName("task_tabs");
  tabs->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Expanding);
  tabs->addTab(make_scroll_page(planning_page, "content_scroll"), tr("Plan"));
  tabs->addTab(make_scroll_page(inspection_page, "inspection_scroll"), tr("Capture"));
  tabs->addTab(make_scroll_page(detail_page, "detail_scroll"), tr("Details"));

  auto * safety_buttons = new QGridLayout();
  safety_buttons->addWidget(start_button_, 0, 0);
  safety_buttons->addWidget(pause_button_, 0, 1);
  safety_buttons->addWidget(resume_button_, 1, 0);
  safety_buttons->addWidget(cancel_button_, 1, 1);
  // These are real-robot recovery actions, not routine task controls. Keep
  // them out of the normal operator footprint and reveal only the one that
  // the manager has explicitly permitted in an exceptional state.
  recovery_controls_ = new QFrame();
  recovery_controls_->setObjectName("recovery_controls");
  auto * recovery_layout = new QVBoxLayout();
  recovery_layout->setContentsMargins(0, 0, 0, 0);
  recovery_layout->addWidget(force_abandon_button_);
  recovery_layout->addWidget(rearm_button_);
  recovery_controls_->setLayout(recovery_layout);
  recovery_controls_->setVisible(false);

  auto * layout = new QVBoxLayout();
  layout->addLayout(overview);
  layout->addWidget(tabs, 1);
  layout->addWidget(makeSeparator());
  // These controls stay outside every page scroll: an operator reaching for
  // Cancel must not have to switch tabs or scroll to find it.
  layout->addLayout(safety_buttons);
  layout->addWidget(recovery_controls_);
  setLayout(layout);
  // No explicit minimum width. An explicit one overrides the layout's own, so
  // naming a number below what the rows actually need lets a dock shrink the
  // panel into the clipping this layout exists to prevent. The layout knows
  // the real figure; RViz clamps the dock to it.

  // activated is overloaded in Qt 5, and it is deliberately not
  // currentIndexChanged: only a human picking an item may send a request,
  // while renderConfig writes these boxes on every refresh.
  for (auto * box : {region_box_, sweep_box_}) {
    connect(
      box, QOverload<int>::of(&QComboBox::activated), this,
      [this](int) {onConfigurationChosen();});
  }
  connect(
    algorithm_box_, QOverload<int>::of(&QComboBox::activated), this,
    [this](int) {onAlgorithmChosen();});
  connect(replan_button_, &QPushButton::clicked, this, &CoveragePanel::onReplan);
  connect(clear_button_, &QPushButton::clicked, this, &CoveragePanel::onClearPoints);
  connect(start_button_, &QPushButton::clicked, this, &CoveragePanel::onStart);
  connect(pause_button_, &QPushButton::clicked, this, &CoveragePanel::onPause);
  connect(resume_button_, &QPushButton::clicked, this, &CoveragePanel::onResume);
  connect(cancel_button_, &QPushButton::clicked, this, &CoveragePanel::onCancel);
  connect(
    force_abandon_button_, &QPushButton::clicked,
    this, &CoveragePanel::onForceAbandon);
  connect(rearm_button_, &QPushButton::clicked, this, &CoveragePanel::onRearm);
  connect(
    archive_root_edit_, &QLineEdit::textEdited, this,
    [this](const QString &) {archive_root_overridden_ = true;});
  connect(archive_root_edit_, &QLineEdit::editingFinished, this, [this]() {
      if (archive_root_edit_->text().trimmed().isEmpty()) {
        archive_root_overridden_ = false;
        archive_root_edit_->setText(manager_archive_root_);
      }
    });
  connect(browse_archive_root_button_, &QPushButton::clicked, this, [this]() {
      const QString selected = QFileDialog::getExistingDirectory(
        this, tr("Choose archive root on this computer"), archive_root_edit_->text());
      if (!selected.isEmpty()) {
        archive_root_overridden_ = true;
        archive_root_edit_->setText(selected);
      }
    });
  connect(default_archive_root_button_, &QPushButton::clicked, this, [this]() {
      archive_root_overridden_ = false;
      archive_root_edit_->setText(manager_archive_root_);
    });

  renderDisconnected();
}

// Out of line because SharedState is only complete here.
CoveragePanel::~CoveragePanel() = default;

void CoveragePanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // Every callback below captures the state rather than the panel: see
  // SharedState. None of them touch a widget or any other member.
  const auto state = state_;
  status_subscription_ = node_->create_subscription<Status>(
    "/coverage/manager_status", rclcpp::QoS(1).reliable().transient_local(),
    [state](const Status::SharedPtr status) {
      const std::lock_guard<std::mutex> lock(state->mutex);
      state->status = std::make_unique<Status>(*status);
    });
  config_subscription_ = node_->create_subscription<Config>(
    "/coverage/config", rclcpp::QoS(1).reliable().transient_local(),
    [state](const Config::SharedPtr config) {
      const std::lock_guard<std::mutex> lock(state->mutex);
      // A configure response in flight is newer than anything the topic can
      // carry, so it wins until it lands.
      if (!state->configure_gate.waiting()) {
        state->config = std::make_unique<Config>(*config);
      }
    });
  planner_subscription_ = node_->create_subscription<std_msgs::msg::String>(
    "/coverage/status", rclcpp::QoS(1).transient_local(),
    [state](const std_msgs::msg::String::SharedPtr message) {
      const std::lock_guard<std::mutex> lock(state->mutex);
      state->planner = QString::fromStdString(message->data);
    });
  replan_client_ = node_->create_client<Trigger>("/coverage/replan");
  clear_client_ = node_->create_client<Trigger>("/coverage/clear_points");
  start_client_ = node_->create_client<StartCoverage>("/coverage/start_configured");
  pause_client_ = node_->create_client<Trigger>("/coverage/pause");
  resume_client_ = node_->create_client<Trigger>("/coverage/resume");
  cancel_client_ = node_->create_client<Trigger>("/coverage/cancel");
  force_abandon_client_ = node_->create_client<Trigger>("/coverage/force_abandon");
  rearm_client_ = node_->create_client<Trigger>("/coverage/rearm");
  configure_client_ = node_->create_client<Configure>("/coverage/configure");
  tracking_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/line_tracker");

  // Subscription and service callbacks run on the executor thread, which must
  // never touch widgets. The timer runs on the Qt thread and is the only place
  // this panel reads the shared state and repaints.
  refresh_timer_ = new QTimer(this);
  connect(refresh_timer_, &QTimer::timeout, this, &CoveragePanel::refresh);
  refresh_timer_->start(100);
}

void CoveragePanel::note(const QString & text)
{
  const std::lock_guard<std::mutex> lock(state_->mutex);
  state_->response = text;
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
    [state = state_, label](rclcpp::Client<Trigger>::SharedFuture future) {
      const auto response = future.get();
      const std::lock_guard<std::mutex> lock(state->mutex);
      state->response = tr("%1: %2 %3").arg(label)
      .arg(response->success ? tr("ok") : tr("refused"))
      .arg(QString::fromStdString(response->message));
    });
}

void CoveragePanel::onConfigurationChosen()
{
  // activated() rather than currentIndexChanged(): only a human choosing an
  // item may send a request. renderConfig writes these boxes on every update,
  // and the change signal would turn each of those writes into another call.
  {
    const std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->configure_gate.waiting()) {
      state_->response = tr("Configuration: previous change still pending.");
      return;
    }
  }
  if (!configure_client_ || !configure_client_->service_is_ready()) {
    note(tr("Configuration: planner unavailable."));
    // The boxes now show something the planner never agreed to, so put them
    // back to the last configuration it published.
    refresh();
    return;
  }
  auto request = std::make_shared<Configure::Request>();
  request->region_type = region_box_->currentData().toString().toStdString();
  request->sweep_direction = sweep_box_->currentData().toString().toStdString();
  uint64_t generation = 0;
  {
    const std::lock_guard<std::mutex> lock(state_->mutex);
    generation = state_->configure_gate.begin();
    state_->configure_sent = std::chrono::steady_clock::now();
    state_->response = tr("Configuration: sent.");
  }
  region_box_->setEnabled(false);
  sweep_box_->setEnabled(false);
  configure_client_->async_send_request(
    request,
    [state = state_, generation](rclcpp::Client<Configure>::SharedFuture future) {
      const auto response = future.get();
      const std::lock_guard<std::mutex> lock(state->mutex);
      if (!state->configure_gate.isCurrent(generation)) {
        // This answer belongs to a request the panel stopped waiting for. It
        // carries the configuration as it was then, and the operator has since
        // asked for something else.
        return;
      }
      state->configure_gate.settle();
      // The response carries the configuration actually in force, accepted or
      // not, so a refused change repaints the boxes back to the truth without
      // waiting for the topic.
      state->config = std::make_unique<Config>(response->config);
      state->response = tr("Configuration: %1").arg(
        QString::fromStdString(response->message));
    });
}

void CoveragePanel::onAlgorithmChosen()
{
  {
    const std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->tracking_gate.waiting()) {
      state_->response = tr("Algorithm: previous change still pending.");
      return;
    }
  }
  if (!tracking_client_ || !tracking_client_->service_is_ready()) {
    note(tr("Algorithm: executor unavailable."));
    return;
  }
  const std::string wanted = algorithm_box_->currentData().toString().toStdString();
  uint64_t generation = 0;
  {
    const std::lock_guard<std::mutex> lock(state_->mutex);
    generation = state_->tracking_gate.begin();
    state_->tracking_sent = std::chrono::steady_clock::now();
    state_->response = tr("Algorithm: sent.");
  }
  algorithm_box_->setEnabled(false);
  tracking_client_->set_parameters(
    {rclcpp::Parameter("tracking_mode", wanted)},
    [state = state_, wanted, generation](
      std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
      const auto results = future.get();
      const bool accepted = !results.empty() && results.front().successful;
      const std::lock_guard<std::mutex> lock(state->mutex);
      if (!state->tracking_gate.isCurrent(generation)) {
        return;
      }
      state->tracking_gate.settle();
      // The executor refuses while a task is running, so a refused change has
      // to put the box back to what is actually in force rather than leave it
      // showing a mode nobody is driving in.
      state->response = accepted ?
      tr("Algorithm: set.") :
      tr("Algorithm: %1").arg(
        results.empty() ?
        tr("no answer from the executor.") :
        QString::fromStdString(results.front().reason));
      if (accepted) {
        state->tracking_mode = QString::fromStdString(wanted);
      } else {
        state->tracking_mode.clear();
      }
    });
}

/// Asks the executor which mode is in force. Called from the refresh timer
/// whenever the answer is unknown - at startup, and after a refused change -
/// so the box never shows a mode the executor did not confirm.
void CoveragePanel::readTrackingMode()
{
  if (!tracking_client_ || !tracking_client_->service_is_ready()) {
    return;
  }
  uint64_t generation = 0;
  {
    const std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->tracking_gate.waiting()) {
      return;
    }
    generation = state_->tracking_gate.begin();
    state_->tracking_sent = std::chrono::steady_clock::now();
  }
  tracking_client_->get_parameters(
    {"tracking_mode"},
    [state = state_, generation](std::shared_future<std::vector<rclcpp::Parameter>> future) {
      const auto parameters = future.get();
      const std::lock_guard<std::mutex> lock(state->mutex);
      if (!state->tracking_gate.isCurrent(generation)) {
        // A read that was given up on can still answer, and what it answers is
        // the mode as it was before whatever the operator did next.
        return;
      }
      state->tracking_gate.settle();
      if (!parameters.empty() &&
      parameters.front().get_type() == rclcpp::ParameterType::PARAMETER_STRING)
      {
        state->tracking_mode = QString::fromStdString(parameters.front().as_string());
      }
    });
}

void CoveragePanel::renderConfig(const Config & config)
{
  // Never currentIndexChanged: these writes must not look like a user choice.
  for (auto * box : {region_box_, sweep_box_}) {
    const QSignalBlocker blocker(box);
    const QString wanted = QString::fromStdString(
      box == region_box_ ? config.region_type : config.sweep_direction);
    const int index = box->findData(wanted);
    if (index >= 0) {
      box->setCurrentIndex(index);
    }
  }
  const bool clicking = config.input_mode == "rviz";
  selection_label_->setText(
    clicking ?
    tr("%1 of %2 selected").arg(config.selected_points).arg(config.required_points) :
    tr("from configuration file"));
  // Point selection is meaningless outside rviz input mode, and so is the
  // button that consumes it.
  clear_button_->setEnabled(clicking && !task_running_);
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
  callConfiguredStart();
}

void CoveragePanel::callConfiguredStart()
{
  if (!start_client_ || !start_client_->service_is_ready()) {
    note(tr("Start: service unavailable."));
    return;
  }
  auto request = std::make_shared<StartCoverage::Request>();
  request->inspection_enabled = inspection_enabled_box_->isChecked();
  request->output_root = archive_root_edit_->text().trimmed().toStdString();
  note(request->inspection_enabled ?
    tr("Start: archive preparation requested.") :
    tr("Start: task requested without image archive."));
  start_client_->async_send_request(
    request,
    [state = state_](rclcpp::Client<StartCoverage>::SharedFuture future) {
      const auto response = future.get();
      const std::lock_guard<std::mutex> lock(state->mutex);
      const QString detail = QString::fromStdString(response->message);
      state->response = response->success ?
      QObject::tr("Start: accepted. %1").arg(detail) :
      QObject::tr("Start: refused. %1").arg(detail);
    });
}

void CoveragePanel::onCancel()
{
  call(cancel_client_, tr("Stop"));
}

void CoveragePanel::onPause()
{
  call(pause_client_, tr("Pause"));
}

void CoveragePanel::onResume()
{
  call(resume_client_, tr("Resume"));
}

void CoveragePanel::onForceAbandon()
{
  const auto now = std::chrono::steady_clock::now();
  if (!force_confirmation_armed_ || now > force_confirmation_deadline_) {
    force_confirmation_armed_ = true;
    force_confirmation_deadline_ = now + std::chrono::seconds{5};
    force_abandon_button_->setText(tr("Confirm force abandon"));
    note(tr("Force abandon: this does not prove the task stopped. Verify the "
      "hardware stop or executor shutdown, then click again within 5 seconds."));
    return;
  }
  force_confirmation_armed_ = false;
  force_abandon_button_->setText(tr("Force abandon"));
  call(force_abandon_client_, tr("Force abandon"));
}

void CoveragePanel::onRearm()
{
  call(rearm_client_, tr("Rearm after verification"));
}

void CoveragePanel::renderDisconnected()
{
  // The State row is a narrow column beside its name, so it holds the state
  // and nothing else. The sentence explaining it belongs in the full-width
  // Manager row, which is where every other manager message already goes.
  task_running_ = false;
  state_label_->setText(tr("Not connected"));
  task_label_->setText("-");
  segment_label_->setText("-");
  progress_bar_->setValue(0);
  schedule_label_->setText("-");
  inspection_summary_label_->setText(tr("Not connected"));
  archive_count_label_->setText("-");
  archive_directory_label_->setText("-");
  archive_error_label_->setText("-");
  message_label_->setText(
    tr("No status received from the coverage manager yet."));
  // Replan and clear only change the preview, never the running task, so they
  // stay available. Start and cancel follow the manager's own permissions,
  // which are unknown until it publishes: offering them here would let a click
  // reach a manager that may not exist.
  replan_button_->setEnabled(true);
  clear_button_->setEnabled(true);
  start_button_->setEnabled(false);
  pause_button_->setEnabled(false);
  resume_button_->setEnabled(false);
  cancel_button_->setEnabled(false);
  force_abandon_button_->setEnabled(false);
  rearm_button_->setEnabled(false);
  recovery_controls_->setVisible(false);
  inspection_enabled_box_->setEnabled(true);
  archive_root_edit_->setEnabled(true);
  browse_archive_root_button_->setEnabled(true);
  default_archive_root_button_->setEnabled(true);
  force_confirmation_armed_ = false;
  force_abandon_button_->setText(tr("Force abandon"));
}

void CoveragePanel::renderStatus(const Status & status)
{
  if (!status.archive_default_root.empty()) {
    manager_archive_root_ = QString::fromStdString(status.archive_default_root);
    if (!archive_root_overridden_) {
      archive_root_edit_->setText(manager_archive_root_);
    }
  }
  state_label_->setText(stateName(status.state));
  task_label_->setText(
    status.task_id.empty() ?
    tr("none") :
    wrappableText(
      tr("%1  revision %2")
      .arg(QString::fromStdString(status.task_id))
      .arg(status.revision)));

  if (status.total_segments == 0U) {
    segment_label_->setText("-");
  } else if (status.current_segment < 0) {
    segment_label_->setText(tr("approach of %1").arg(status.total_segments));
  } else {
    segment_label_->setText(
      tr("%1 of %2").arg(status.current_segment + 1).arg(status.total_segments));
  }

  progress_bar_->setValue(static_cast<int>(status.progress * 100.0F + 0.5F));
  const QString archive_state = archiveStateText(status.archive_state);
  inspection_summary_label_->setText(
    status.inspection_enabled ?
    tr("%1 · %2 saved · %3 frozen / %4 nominal")
    .arg(archive_state)
    .arg(status.archive_saved_images)
    .arg(status.archive_expected_images)
    .arg(status.archive_preflight_expected_images) :
    tr("Off"));
  archive_count_label_->setText(
    status.inspection_enabled ?
    tr("%1 / %2 / %3 / %4")
    .arg(status.archive_preflight_expected_images)
    .arg(status.archive_expected_images)
    .arg(status.archive_saved_images)
    .arg(status.archive_failed_images) : tr("-"));
  archive_directory_label_->setText(status.archive_directory.empty() ?
    tr("-") : wrappableText(QString::fromStdString(status.archive_directory)));
  archive_error_label_->setText(status.archive_message.empty() ?
    tr("-") : wrappableText(QString::fromStdString(status.archive_message)));
  schedule_label_->setText(scheduleText(status, rendered_tracking_mode_));
  schedule_label_->setToolTip(
    rendered_tracking_mode_ == QLatin1String("distance") ?
    tr("Predicted from the segment geometry. Nothing measures whether the run "
    "keeps to it, so a robot losing ground runs late without reporting it. "
    "Switch the algorithm to Timed trajectory for a schedule the controller "
    "follows and reports against.") :
    tr("The schedule the controller is driving from. The figure after it is "
    "how far behind that schedule the robot currently is; a negative value "
    "means it is ahead. It stays within a few hundredths of a second while "
    "the run is going to plan."));
  // Manager sentences quote the task id, so they need the same treatment.
  message_label_->setText(wrappableText(QString::fromStdString(status.message)));

  // The manager publishes what it would accept, so this panel renders that
  // decision rather than deriving one from the state. Deriving it is how a
  // task that finished stayed unstartable although the manager still had it
  // cached. Replan and clear belong to the planner and only affect the
  // preview, so they are never withheld here.
  start_button_->setEnabled(status.can_start);
  pause_button_->setEnabled(status.can_pause);
  resume_button_->setEnabled(status.can_resume);
  cancel_button_->setEnabled(status.can_cancel);
  force_abandon_button_->setEnabled(status.can_force_abandon);
  rearm_button_->setEnabled(status.can_rearm);
  force_abandon_button_->setVisible(status.can_force_abandon);
  rearm_button_->setVisible(status.can_rearm);
  recovery_controls_->setVisible(status.can_force_abandon || status.can_rearm);
  if (!status.can_force_abandon) {
    force_confirmation_armed_ = false;
    force_abandon_button_->setText(tr("Force abandon"));
  }
  // Everything that shapes a task is frozen while one is running, so the only
  // thing left to do to a running task is stop it. These controls were left
  // live on the grounds that they only touch the preview and never the running
  // task, which is true of the messages they send and false of what an
  // operator sees: the preview is the trajectory drawn over the robot, and
  // changing the shape now withdraws it mid-drive, which reads as the mission
  // having been altered or lost.
  task_running_ = status.state == Status::STARTING ||
    status.state == Status::EXECUTING || status.state == Status::PAUSING ||
    status.state == Status::PAUSED || status.state == Status::RESUMING ||
    status.state == Status::STOPPING ||
    status.state == Status::RECOVERY_LOCKED ||
    status.archive_state == climbot_interfaces::msg::InspectionArchiveStatus::FINALIZING;
  replan_button_->setEnabled(!task_running_);
  clear_button_->setEnabled(!task_running_);
  inspection_enabled_box_->setEnabled(!task_running_);
  archive_root_edit_->setEnabled(!task_running_);
  browse_archive_root_button_->setEnabled(!task_running_);
  default_archive_root_button_->setEnabled(!task_running_);
  if (task_running_) {
    region_box_->setEnabled(false);
    sweep_box_->setEnabled(false);
    algorithm_box_->setEnabled(false);
  }
}

void CoveragePanel::expireStalePendingRequests()
{
  const auto now = std::chrono::steady_clock::now();
  const std::lock_guard<std::mutex> lock(state_->mutex);
  // abandon() rather than a flag: releasing the controls has never retracted
  // the request, and the answer that eventually arrives must not be applied to
  // whatever the operator has done in the meantime.
  if (state_->configure_gate.waiting() && requestHasExpired(state_->configure_sent, now)) {
    state_->configure_gate.abandon();
    state_->response = tr("Configuration: no answer; released the controls.");
  }
  if (state_->tracking_gate.waiting() && requestHasExpired(state_->tracking_sent, now)) {
    state_->tracking_gate.abandon();
    state_->response = tr("Algorithm: no answer; released the control.");
  }
}

void CoveragePanel::refresh()
{
  if (force_confirmation_armed_ &&
    std::chrono::steady_clock::now() > force_confirmation_deadline_)
  {
    force_confirmation_armed_ = false;
    force_abandon_button_->setText(tr("Force abandon"));
    note(tr("Force abandon: confirmation expired; no request was sent."));
  }
  expireStalePendingRequests();
  std::unique_ptr<Status> status;
  QString planner;
  QString response;
  std::unique_ptr<Config> config;
  bool pending = false;
  bool tracking_pending = false;
  QString tracking_mode;
  {
    const std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->status) {
      status = std::make_unique<Status>(*state_->status);
    }
    if (state_->config) {
      config = std::make_unique<Config>(*state_->config);
    }
    planner = state_->planner;
    response = state_->response;
    pending = state_->configure_gate.waiting();
    tracking_pending = state_->tracking_gate.waiting();
    tracking_mode = state_->tracking_mode;
  }
  if (tracking_mode.isEmpty()) {
    readTrackingMode();
  }
  // Handed to renderStatus through a member rather than an argument because
  // the layout test drives that function directly, with no refresh to pass it
  // through. Set before rendering, so the label never trails by a tick.
  rendered_tracking_mode_ = tracking_mode;
  if (status) {
    renderStatus(*status);
  } else {
    renderDisconnected();
  }
  if (config) {
    renderConfig(*config);
  } else {
    selection_label_->setText(tr("planner not connected"));
  }
  // Enabled only once the planner has answered, so a second choice cannot be
  // made against a configuration that is still being decided.
  region_box_->setEnabled(!task_running_ && !pending && config != nullptr);
  sweep_box_->setEnabled(!task_running_ && !pending && config != nullptr);
  // The executor refuses a change while a task is running, so the box says so
  // in advance instead of letting a click be turned down.
  algorithm_box_->setEnabled(
    !task_running_ && !tracking_pending && !tracking_mode.isEmpty());
  if (!tracking_mode.isEmpty()) {
    const QSignalBlocker blocker(algorithm_box_);
    const int index = algorithm_box_->findData(tracking_mode);
    if (index >= 0) {
      algorithm_box_->setCurrentIndex(index);
    }
  }
  // Replanning is refused without enough points; say so before it is pressed
  // rather than only in the response line afterwards.
  replan_button_->setEnabled(!task_running_ && (config == nullptr || config->can_plan));
  planner_label_->setText(planner.isEmpty() ? "-" : wrappableText(planner));
  response_label_->setText(response.isEmpty() ? "-" : wrappableText(response));
}

}  // namespace climbot_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(climbot_rviz_plugins::CoveragePanel, rviz_common::Panel)
