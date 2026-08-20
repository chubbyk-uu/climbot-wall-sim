#include "climbot_rviz_plugins/coverage_panel.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <QFont>
#include <QFontMetrics>
#include <QFrame>
#include <QGridLayout>
#include <QSignalBlocker>
#include <QScrollArea>
#include <QSizePolicy>
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
    case climbot_interfaces::msg::CoverageStatus::STOPPING:
      // Not "Finished" and not "Executing": contact with the executor is gone
      // and the manager has not yet established that the robot has stopped.
      // The stop button stays enabled here, from can_cancel, because this is
      // the state in which it is most worth pressing.
      return QObject::tr("Stopping (executor lost)");
    default:
      return QObject::tr("Unknown (%1)").arg(state);
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

  replan_button_ = new QPushButton(tr("Replan"));
  clear_button_ = new QPushButton(tr("Clear points"));
  start_button_ = new QPushButton(tr("Start"));
  cancel_button_ = new QPushButton(tr("Cancel / Stop"));
  // Named so a test can reach them. Without names the one test that already
  // checked a button's state had to guard against not finding it, which turned
  // the check into a no-op.
  replan_button_->setObjectName("replan_button");
  clear_button_->setObjectName("clear_button");
  start_button_->setObjectName("start_button");
  cancel_button_->setObjectName("cancel_button");
  // The buttons keep their default policy, unlike the labels. A label with no
  // room wraps; a button label with no room is simply cut, and "Cancel / Sto"
  // is not a control an operator should have to act on. So the buttons are
  // what sets the floor on how narrow this dock may go.

  // Short values keep their name beside them. The name column is sized from
  // its own text so it never grows with the value.
  auto * fields = new QGridLayout();
  fields->addWidget(new QLabel(tr("Region")), 0, 0);
  fields->addWidget(region_box_, 0, 1);
  fields->addWidget(new QLabel(tr("Sweep")), 1, 0);
  fields->addWidget(sweep_box_, 1, 1);
  fields->addWidget(new QLabel(tr("Algorithm")), 2, 0);
  fields->addWidget(algorithm_box_, 2, 1);
  fields->addWidget(new QLabel(tr("Points")), 3, 0);
  fields->addWidget(selection_label_, 3, 1);
  fields->addWidget(new QLabel(tr("State")), 4, 0);
  fields->addWidget(state_label_, 4, 1);
  fields->addWidget(new QLabel(tr("Segment")), 5, 0);
  fields->addWidget(segment_label_, 5, 1);
  fields->addWidget(new QLabel(tr("Progress")), 6, 0);
  fields->addWidget(progress_bar_, 6, 1);
  // Beneath the bar rather than inside it: the bar says how much of the work
  // is done, this row says whether it is on time. Folding the two together
  // would let a stuck robot show a bar that keeps filling.
  fields->addWidget(new QLabel(tr("Schedule")), 7, 0);
  fields->addWidget(schedule_label_, 7, 1);
  fields->setColumnStretch(1, 1);
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
  fields->setColumnMinimumWidth(1, widest_state);

  auto * buttons = new QGridLayout();
  buttons->addWidget(replan_button_, 0, 0);
  buttons->addWidget(clear_button_, 0, 1);
  buttons->addWidget(start_button_, 1, 0);
  buttons->addWidget(cancel_button_, 1, 1);

  // Task ids and manager sentences are longer than any dock is wide, so they
  // get the full width with the name above rather than a column beside them.
  auto * body = new QVBoxLayout();
  body->addLayout(fields);
  body->addWidget(makeCaption(tr("Task")));
  body->addWidget(task_label_);
  body->addWidget(makeCaption(tr("Manager")));
  body->addWidget(message_label_);
  // The manager cannot tell a cleared selection from a failed plan: both
  // arrive as an empty task. Only the planner knows which, so show it.
  body->addWidget(makeCaption(tr("Planner")));
  body->addWidget(planner_label_);
  body->addWidget(makeCaption(tr("Last request")));
  body->addWidget(response_label_);
  body->addStretch(1);

  auto * content = new QWidget();
  content->setLayout(body);

  // A dock can be short as easily as it can be narrow, and three sentences
  // stacked up outgrow it. Scrolling keeps the text reachable; without this
  // the bottom rows are simply cut away.
  auto * scroll = new QScrollArea();
  scroll->setObjectName("content_scroll");
  scroll->setWidget(content);
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

  auto * layout = new QVBoxLayout();
  layout->addWidget(scroll, 1);
  layout->addWidget(makeSeparator());
  // The buttons stay outside the scroll area: an operator reaching for Cancel
  // must not have to scroll to find it.
  layout->addLayout(buttons);
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
  connect(cancel_button_, &QPushButton::clicked, this, &CoveragePanel::onCancel);

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
  start_client_ = node_->create_client<Trigger>("/coverage/start");
  cancel_client_ = node_->create_client<Trigger>("/coverage/cancel");
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
  call(start_client_, tr("Start"));
}

void CoveragePanel::onCancel()
{
  call(cancel_client_, tr("Cancel"));
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
  message_label_->setText(
    tr("No status received from the coverage manager yet."));
  // Replan and clear only change the preview, never the running task, so they
  // stay available. Start and cancel follow the manager's own permissions,
  // which are unknown until it publishes: offering them here would let a click
  // reach a manager that may not exist.
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
  cancel_button_->setEnabled(status.can_cancel);
  // Everything that shapes a task is frozen while one is running, so the only
  // thing left to do to a running task is stop it. These controls were left
  // live on the grounds that they only touch the preview and never the running
  // task, which is true of the messages they send and false of what an
  // operator sees: the preview is the trajectory drawn over the robot, and
  // changing the shape now withdraws it mid-drive, which reads as the mission
  // having been altered or lost.
  task_running_ = status.can_cancel;
  replan_button_->setEnabled(!task_running_);
  clear_button_->setEnabled(!task_running_);
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
