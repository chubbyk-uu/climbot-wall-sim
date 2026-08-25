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

// Layout regression test for the coverage panel.
//
// The panel lives in an RViz dock whose width the operator controls, and the
// manager publishes sentences, not words. A label that reports a taller size
// hint than the row it was given is text the operator cannot read, which is
// exactly how a refusal reason disappears at the moment it matters. This test
// lays the panel out at realistic dock widths and fails when any widget is
// asked to paint more than it was given room for.
//
// Set CLIMBOT_PANEL_SHOT to a directory to also dump the rendered panels as
// PNGs, which is how the layout is inspected by eye.

#include <gtest/gtest.h>

#include <cstdlib>
#include <string>
#include <vector>

#include <QApplication>
#include <QFontMetrics>
#include <QLabel>
#include <QLayout>
#include <QPixmap>
#include <QPoint>
#include <QPushButton>
#include <QRegularExpression>
#include <QScrollArea>
#include <QScrollBar>
#include <QString>
#include <QTabWidget>
#include <QWidget>

#include "climbot_interfaces/msg/inspection_archive_status.hpp"
#include "climbot_rviz_plugins/coverage_panel.hpp"

namespace
{

// The longest strings any of the three text rows can actually carry, taken
// from coverage_manager_node.cpp, coverage_planner_node.cpp and the panel's
// own service replies.
const char * kManagerMessage =
  "Executor disappeared while running coverage_20260817_143512_rectangle; "
  "released the task so it can be started again.";
const char * kPlannerMessage =
  "Planned 24 segments over a 6.00 x 6.00 m rectangle with 0.40 m line "
  "spacing from 2 clicked corners.";
const char * kResponseMessage =
  "Replan: refused Select 3 region points before replanning; 1 are set.";

QApplication & application()
{
  // The platform has to be chosen before the first QApplication exists, and
  // the test machine has no display.
  static int argc = 1;
  static char name[] = "test_coverage_panel_layout";
  static char * argv[] = {name, nullptr};
  static bool configured = []() {
      qputenv("QT_QPA_PLATFORM", "offscreen");
      return true;
    }();
  (void)configured;
  static QApplication app(argc, argv);
  return app;
}

QWidget * child(const QWidget & panel, const QString & name)
{
  auto * found = panel.findChild<QWidget *>(name);
  EXPECT_NE(found, nullptr) << "no widget named " << name.toStdString();
  return found;
}

// Drive the panel through the render path RViz uses, so the wrapping the
// panel applies to its own text is what this test measures.
void fill(climbot_rviz_plugins::CoveragePanel & panel)
{
  climbot_interfaces::msg::CoverageStatus status;
  status.state = climbot_interfaces::msg::CoverageStatus::EXECUTING;
  status.task_id = "coverage_20260817_143512_rectangle";
  status.revision = 7;
  status.current_segment = 11;
  status.total_segments = 24;
  status.progress = 0.47F;
  status.can_cancel = true;
  status.inspection_enabled = true;
  status.archive_state = climbot_interfaces::msg::InspectionArchiveStatus::RECORDING;
  status.archive_expected_images = 120U;
  status.archive_saved_images = 47U;
  status.archive_failed_images = 0U;
  status.archive_directory =
    "/srv/recorder/climbot_data/coverage_20260817_143512_rectangle/r7_run_001";
  status.archive_message =
    "Recording raw distorted mono8 images and immutable pose labels.";
  status.message = kManagerMessage;
  panel.renderStatus(status);

  // These two arrive from the planner topic and from service replies, which
  // only reach the labels through the panel's ROS callbacks.
  child(panel, "planner_value")->setProperty(
    "text", climbot_rviz_plugins::wrappableText(kPlannerMessage));
  child(panel, "response_value")->setProperty(
    "text", climbot_rviz_plugins::wrappableText(kResponseMessage));
}

// The longest run of characters with no break opportunity in it. A label can
// only be as narrow as this run is wide before it starts cutting text off.
QString longestUnbreakableRun(const QString & text)
{
  QString longest;
  for (const auto & run : text.split(QRegularExpression("[\\s\\x{200B}]"))) {
    if (run.size() > longest.size()) {
      longest = run;
    }
  }
  return longest;
}

// A widget is clipped when the layout gave it less than it needs. For a
// word-wrapped label the need depends on the width it ended up with, which is
// what heightForWidth answers.
int shortfall(const QWidget & widget)
{
  const int needed = widget.heightForWidth(widget.width()) > 0 ?
    widget.heightForWidth(widget.width()) :
    widget.sizeHint().height();
  return needed - widget.height();
}

void layOut(QWidget & panel, int width, int height)
{
  // Word wrapping only settles once the layout has run against the new width,
  // and a hidden widget never lays out at all.
  panel.show();
  panel.resize(width, height);
  QApplication::processEvents();
  panel.layout()->activate();
  QApplication::processEvents();
}

}  // namespace

class CoveragePanelLayout : public ::testing::TestWithParam<int>
{
protected:
  void SetUp() override
  {
    application();
  }
};

namespace
{

void expectNothingIsCut(
  QWidget & panel, int width, const std::vector<QString> & labels,
  const std::vector<QString> & buttons = {})
{
  for (const auto & name : labels) {
    auto * widget = child(panel, name);
    ASSERT_NE(widget, nullptr);
    EXPECT_LE(shortfall(*widget), 0)
      << name.toStdString() << " is clipped by " << shortfall(*widget)
      << " px at a dock width of " << width << " px";
    EXPECT_LE(widget->mapTo(&panel, QPoint(widget->width(), 0)).x(), panel.width())
      << name.toStdString() << " runs past the right edge at " << width << " px";

    // Wrapping cannot rescue a run with no break opportunity in it: whatever
    // sticks out past the label is painted over or simply lost.
    const QString run =
      longestUnbreakableRun(widget->property("text").toString());
    const int run_width = QFontMetrics(widget->font()).horizontalAdvance(run);
    EXPECT_LE(run_width, widget->width())
      << name.toStdString() << " cannot break \"" << run.toStdString()
      << "\" (" << run_width << " px) into " << widget->width() << " px";
  }

  // A button cannot wrap or scroll, so a label that does not fit is simply
  // cut, and "Cancel / Sto" is not a control anyone should have to act on.
  for (const auto & name : buttons) {
    auto * button = qobject_cast<QPushButton *>(child(panel, name));
    ASSERT_NE(button, nullptr);
    EXPECT_GE(button->width(), button->sizeHint().width())
      << button->text().toStdString() << " is cut at a dock width of "
      << width << " px";
  }
}

void selectTab(QWidget & panel, int index)
{
  auto * tabs = panel.findChild<QTabWidget *>("task_tabs");
  ASSERT_NE(tabs, nullptr);
  tabs->setCurrentIndex(index);
  panel.layout()->invalidate();
  panel.layout()->setGeometry(panel.rect());
  QApplication::processEvents();
  QApplication::processEvents();
}

void dumpIfRequested(QWidget & panel, const QString & label)
{
  const char * directory = std::getenv("CLIMBOT_PANEL_SHOT");
  if (directory == nullptr) {
    return;
  }
  const QString path = QString("%1/panel_%2.png").arg(directory).arg(label);
  ASSERT_TRUE(panel.grab().save(path, "PNG")) << path.toStdString();
}

}  // namespace

TEST_P(CoveragePanelLayout, showsEveryMessageInFullAtOperatorDockWidths)
{
  climbot_rviz_plugins::CoveragePanel panel;
  fill(panel);
  const int width = GetParam();
  layOut(panel, width, 640);

  // A widget that refuses to shrink is the failure this test exists for: the
  // dock cannot honour it, so RViz cuts the text off instead.
  ASSERT_EQ(panel.width(), width)
    << "the panel refused a dock " << width << " px wide";

  expectNothingIsCut(
    panel, width,
    {"state_value", "segment_value", "inspection_summary_value"},
    {"start_button", "cancel_button", "force_abandon_button", "rearm_button"});
  dumpIfRequested(panel, QString("%1_plan").arg(width));
  selectTab(panel, 1);
  layOut(panel, width, 640);
  expectNothingIsCut(
    panel, width,
    {"archive_count_value", "archive_directory_value", "archive_error_value"},
    {"browse_archive_root_button", "default_archive_root_button"});
  dumpIfRequested(panel, QString("%1_capture").arg(width));
  selectTab(panel, 2);
  layOut(panel, width, 640);
  expectNothingIsCut(
    panel, width,
    {"task_value", "schedule_value", "manager_value", "planner_value", "response_value"});
  dumpIfRequested(panel, QString("%1_details").arg(width));
}

// What the operator sees before anything is running. The compact overview
// remains readable even though diagnostic text is deliberately on Details.
TEST(CoveragePanelSizing, showsTheStartupNoticeInFull)
{
  application();
  climbot_rviz_plugins::CoveragePanel panel;
  panel.renderDisconnected();
  layOut(panel, 240, 640);
  ASSERT_EQ(panel.width(), 240);
  expectNothingIsCut(
    panel, 240,
    {"state_value", "segment_value", "inspection_summary_value"},
    {"start_button", "cancel_button", "force_abandon_button", "rearm_button"});
  dumpIfRequested(panel, "startup");
}

// The dock beside the render view is around 300 px in the shipped config, and
// an operator can drag it narrower still. 240 is the narrowest the panel
// accepts; below that RViz clamps the dock rather than cut the text.
INSTANTIATE_TEST_SUITE_P(
  DockWidths, CoveragePanelLayout, ::testing::Values(240, 300, 420, 560));

TEST(CoveragePanelSizing, fitsBesideTheRenderViewWithoutForcingTheDockWider)
{
  application();
  climbot_rviz_plugins::CoveragePanel panel;
  fill(panel);
  // The shipped config is 1200 px wide and the render view takes most of it.
  // Anything above this and the panel wins the argument with the dock, which
  // it loses as clipped text rather than as a wider dock.
  EXPECT_LE(panel.minimumSizeHint().width(), 240)
    << "the panel demands a dock wider than the config leaves for it";
  // RViz sizes a fresh dock from the preferred hint, so a hint that grows with
  // the longest message would open the panel wide enough to squeeze the render
  // view off the screen.
  EXPECT_LE(panel.sizeHint().width(), 340)
    << "the panel would open a dock " << panel.sizeHint().width() << " px wide";
  // An explicit minimum overrides the layout's, so one set below what the rows
  // need would let the dock shrink the panel back into clipping.
  EXPECT_EQ(panel.minimumWidth(), 0)
    << "an explicit minimum width would mask the layout's own";
}

TEST(CoveragePanelSizing, keepsTheButtonsReachableWhenTheDockIsTooShort)
{
  application();
  climbot_rviz_plugins::CoveragePanel panel;
  fill(panel);
  // A dock shorter than the messages must scroll them, not swallow the
  // controls: Cancel has to stay one click away while the robot is moving.
  layOut(panel, 300, 200);
  auto * tabs = panel.findChild<QTabWidget *>("task_tabs");
  ASSERT_NE(tabs, nullptr);
  tabs->setCurrentIndex(2);
  QApplication::processEvents();
  panel.layout()->activate();
  auto * scroll = panel.findChild<QScrollArea *>("detail_scroll");
  ASSERT_NE(scroll, nullptr);
  EXPECT_TRUE(scroll->verticalScrollBar()->maximum() > 0)
    << "the messages were cut off instead of made scrollable";
  for (auto * button : panel.findChildren<QPushButton *>()) {
    const QPoint corner = button->mapTo(&panel, QPoint(0, button->height()));
    EXPECT_LE(corner.y(), panel.height())
      << button->text().toStdString() << " was pushed out of the panel";
  }
}
