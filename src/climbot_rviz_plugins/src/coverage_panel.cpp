#include "climbot_rviz_plugins/coverage_panel.hpp"

#include <algorithm>
#include <memory>
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

QFrame * makeSeparator()
{
  auto * line = new QFrame();
  line->setFrameShape(QFrame::HLine);
  line->setFrameShadow(QFrame::Sunken);
  return line;
}

}  // namespace

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

CoveragePanel::CoveragePanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  state_label_ = makeValue("state_value");
  task_label_ = makeValue("task_value");
  segment_label_ = makeValue("segment_value");
  message_label_ = makeValue("manager_value");
  planner_label_ = makeValue("planner_value");
  response_label_ = makeValue("response_value");

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
  selection_label_ = makeValue("selection_value");

  replan_button_ = new QPushButton(tr("Replan"));
  clear_button_ = new QPushButton(tr("Clear points"));
  start_button_ = new QPushButton(tr("Start"));
  cancel_button_ = new QPushButton(tr("Cancel / Stop"));
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
  fields->addWidget(new QLabel(tr("Points")), 2, 0);
  fields->addWidget(selection_label_, 2, 1);
  fields->addWidget(new QLabel(tr("State")), 3, 0);
  fields->addWidget(state_label_, 3, 1);
  fields->addWidget(new QLabel(tr("Segment")), 4, 0);
  fields->addWidget(segment_label_, 4, 1);
  fields->addWidget(new QLabel(tr("Progress")), 5, 0);
  fields->addWidget(progress_bar_, 5, 1);
  fields->setColumnStretch(1, 1);
  // A combo box will happily grow to its widest item and drag the dock with
  // it; it may elide instead, like the labels around it.
  for (auto * box : {region_box_, sweep_box_}) {
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
  config_subscription_ = node_->create_subscription<Config>(
    "/coverage/config", rclcpp::QoS(1).reliable().transient_local(),
    [this](const Config::SharedPtr config) {
      const std::lock_guard<std::mutex> lock(mutex_);
      // A configure response in flight is newer than anything the topic can
      // carry, so it wins until it lands.
      if (!configure_pending_) {
        config_ = std::make_unique<Config>(*config);
      }
    });
  planner_subscription_ = node_->create_subscription<std_msgs::msg::String>(
    "/coverage/status", rclcpp::QoS(1).transient_local(),
    [this](const std_msgs::msg::String::SharedPtr message) {
      const std::lock_guard<std::mutex> lock(mutex_);
      planner_ = QString::fromStdString(message->data);
    });
  replan_client_ = node_->create_client<Trigger>("/coverage/replan");
  clear_client_ = node_->create_client<Trigger>("/coverage/clear_points");
  start_client_ = node_->create_client<Trigger>("/coverage/start");
  cancel_client_ = node_->create_client<Trigger>("/coverage/cancel");
  configure_client_ = node_->create_client<Configure>("/coverage/configure");

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

void CoveragePanel::onConfigurationChosen()
{
  // activated() rather than currentIndexChanged(): only a human choosing an
  // item may send a request. renderConfig writes these boxes on every update,
  // and the change signal would turn each of those writes into another call.
  if (configure_pending_) {
    note(tr("Configuration: previous change still pending."));
    return;
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
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    configure_pending_ = true;
    response_ = tr("Configuration: sent.");
  }
  region_box_->setEnabled(false);
  sweep_box_->setEnabled(false);
  configure_client_->async_send_request(
    request,
    [this](rclcpp::Client<Configure>::SharedFuture future) {
      const auto response = future.get();
      const std::lock_guard<std::mutex> lock(mutex_);
      configure_pending_ = false;
      // The response carries the configuration actually in force, accepted or
      // not, so a refused change repaints the boxes back to the truth without
      // waiting for the topic.
      config_ = std::make_unique<Config>(response->config);
      response_ = tr("Configuration: %1").arg(
        QString::fromStdString(response->message));
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
  clear_button_->setEnabled(clicking);
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
  state_label_->setText(tr("Not connected"));
  task_label_->setText("-");
  segment_label_->setText("-");
  progress_bar_->setValue(0);
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
  // Manager sentences quote the task id, so they need the same treatment.
  message_label_->setText(wrappableText(QString::fromStdString(status.message)));

  // The manager publishes what it would accept, so this panel renders that
  // decision rather than deriving one from the state. Deriving it is how a
  // task that finished stayed unstartable although the manager still had it
  // cached. Replan and clear belong to the planner and only affect the
  // preview, so they are never withheld here.
  start_button_->setEnabled(status.can_start);
  cancel_button_->setEnabled(status.can_cancel);
  replan_button_->setEnabled(true);
  clear_button_->setEnabled(true);
}

void CoveragePanel::refresh()
{
  std::unique_ptr<Status> status;
  QString planner;
  QString response;
  std::unique_ptr<Config> config;
  bool pending = false;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (status_) {
      status = std::make_unique<Status>(*status_);
    }
    if (config_) {
      config = std::make_unique<Config>(*config_);
    }
    planner = planner_;
    response = response_;
    pending = configure_pending_;
  }
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
  region_box_->setEnabled(!pending && config != nullptr);
  sweep_box_->setEnabled(!pending && config != nullptr);
  // Replanning is refused without enough points; say so before it is pressed
  // rather than only in the response line afterwards.
  replan_button_->setEnabled(config == nullptr || config->can_plan);
  planner_label_->setText(planner.isEmpty() ? "-" : wrappableText(planner));
  response_label_->setText(response.isEmpty() ? "-" : wrappableText(response));
}

}  // namespace climbot_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(climbot_rviz_plugins::CoveragePanel, rviz_common::Panel)
