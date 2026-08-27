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

// Foolproofing test for the panel's region and sweep controls.
//
// These two drop-downs are the only widgets in the panel that send anything
// other than a fixed request, so they are the only place a stale or invented
// value can reach the planner. Everything below drives the render path an
// operator's clicking would drive, and checks the panel keeps showing what the
// planner published rather than what someone last picked.

#include <gtest/gtest.h>

#include <chrono>

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QString>

#include "climbot_interfaces/msg/inspection_archive_status.hpp"
#include "climbot_rviz_plugins/coverage_panel.hpp"

namespace
{

QApplication * application()
{
  static int argc = 1;
  static char name[] = "panel_config_test";
  static char * argv[] = {name, nullptr};
  static QApplication app(argc, argv);
  return &app;
}

climbot_rviz_plugins::CoveragePanel::Config makeConfig(
  const std::string & region, const std::string & sweep,
  uint8_t selected, uint8_t required, bool can_plan,
  const std::string & input_mode = "rviz")
{
  climbot_rviz_plugins::CoveragePanel::Config config;
  config.region_type = region;
  config.sweep_direction = sweep;
  config.input_mode = input_mode;
  config.selected_points = selected;
  config.required_points = required;
  config.can_plan = can_plan;
  return config;
}

QComboBox * box(climbot_rviz_plugins::CoveragePanel & panel, const char * name)
{
  return panel.findChild<QComboBox *>(name);
}

climbot_rviz_plugins::CoveragePanel::Status runningStatus(bool running)
{
  climbot_rviz_plugins::CoveragePanel::Status status;
  status.state = running ?
    climbot_rviz_plugins::CoveragePanel::Status::EXECUTING :
    climbot_rviz_plugins::CoveragePanel::Status::READY;
  status.total_segments = 4U;
  status.can_start = !running;
  status.can_cancel = running;
  return status;
}

}  // namespace

TEST(CoveragePanelConfig, showsWhatThePlannerPublished)
{
  application();
  climbot_rviz_plugins::CoveragePanel panel;
  panel.renderConfig(makeConfig("trapezoid", "vertical", 3, 3, true));
  EXPECT_EQ(box(panel, "region_box")->currentData().toString(), "trapezoid");
  EXPECT_EQ(box(panel, "sweep_box")->currentData().toString(), "vertical");
}

TEST(CoveragePanelConfig, aPublishedConfigIsNotMistakenForAnOperatorChoice)
{
  // The panel repaints these boxes on every refresh. If those writes emitted
  // the same signal a click does, each update from the planner would send
  // another configure request straight back to it.
  application();
  climbot_rviz_plugins::CoveragePanel panel;
  auto * region = box(panel, "region_box");
  int activations = 0;
  QObject::connect(
    region, QOverload<int>::of(&QComboBox::activated),
    [&activations](int) {++activations;});
  panel.renderConfig(makeConfig("trapezoid", "horizontal", 0, 3, false));
  panel.renderConfig(makeConfig("rectangle", "horizontal", 0, 2, false));
  EXPECT_EQ(activations, 0)
    << "repainting the box looked like the operator using it";
}

TEST(CoveragePanelConfig, aRefusedChangeSnapsBackToThePlannersValue)
{
  // A refused request still returns the configuration in force, so rendering
  // it must undo whatever the operator picked. Without this the panel keeps
  // claiming a shape the planner never accepted.
  application();
  climbot_rviz_plugins::CoveragePanel panel;
  panel.renderConfig(makeConfig("rectangle", "horizontal", 2, 2, true));
  auto * region = box(panel, "region_box");
  region->setCurrentIndex(region->findData("trapezoid"));
  ASSERT_EQ(region->currentData().toString(), "trapezoid");
  panel.renderConfig(makeConfig("rectangle", "horizontal", 2, 2, true));
  EXPECT_EQ(region->currentData().toString(), "rectangle");
}

TEST(CoveragePanelConfig, reportsHowManyPointsAreStillNeeded)
{
  application();
  climbot_rviz_plugins::CoveragePanel panel;
  panel.renderConfig(makeConfig("trapezoid", "vertical", 2, 3, false));
  auto * selection = panel.findChild<QLabel *>("selection_value");
  ASSERT_NE(selection, nullptr);
  EXPECT_TRUE(selection->text().contains("2")) << selection->text().toStdString();
  EXPECT_TRUE(selection->text().contains("3")) << selection->text().toStdString();
}

TEST(CoveragePanelConfig, hidesPointCountingWhenTheRegionComesFromAFile)
{
  // Clicks mean nothing in parameters mode, so neither does clearing them.
  application();
  climbot_rviz_plugins::CoveragePanel panel;
  panel.renderConfig(makeConfig("rectangle", "horizontal", 0, 2, true, "parameters"));
  auto * selection = panel.findChild<QLabel *>("selection_value");
  ASSERT_NE(selection, nullptr);
  EXPECT_FALSE(selection->text().contains("selected"));
  auto * clear = panel.findChild<QPushButton *>("clear_button");
  ASSERT_NE(clear, nullptr);
  EXPECT_FALSE(clear->isEnabled());
}

TEST(CoveragePanelConfig, showsAndRestoresTheHomeDataDirectory)
{
  application();
  climbot_rviz_plugins::CoveragePanel panel;
  auto * root = panel.findChild<QLineEdit *>("archive_root_edit");
  auto * defaults = panel.findChild<QPushButton *>("default_archive_root_button");
  ASSERT_NE(root, nullptr);
  ASSERT_NE(defaults, nullptr);
  const QString expected = QDir::homePath() + QStringLiteral("/climbot_data");
  EXPECT_EQ(root->text(), expected);
  root->setText(QStringLiteral("/another/data/root"));
  defaults->click();
  EXPECT_EQ(root->text(), expected);
}

// Every one of these sends a request that reshapes the task being previewed.
// They were left live during a run because none of them touches the running
// goal, which is true of the messages and false of what the operator sees: the
// preview is the trajectory drawn over the robot, so reshaping it mid-drive
// reads as the mission having changed.
TEST(CoveragePanelConfig, freezesEveryPlanningControlWhileATaskRuns)
{
  application();
  climbot_rviz_plugins::CoveragePanel panel;
  panel.renderStatus(runningStatus(true));
  panel.renderConfig(makeConfig("rectangle", "horizontal", 2, 2, true));
  EXPECT_FALSE(box(panel, "region_box")->isEnabled());
  EXPECT_FALSE(box(panel, "sweep_box")->isEnabled());
  EXPECT_FALSE(box(panel, "algorithm_box")->isEnabled());
  EXPECT_FALSE(panel.findChild<QPushButton *>("replan_button")->isEnabled());
  EXPECT_FALSE(panel.findChild<QPushButton *>("clear_button")->isEnabled());
  EXPECT_TRUE(panel.findChild<QPushButton *>("cancel_button")->isEnabled());
}

TEST(CoveragePanelConfig, releasesThePlanningControlsWhenTheTaskStops)
{
  application();
  climbot_rviz_plugins::CoveragePanel panel;
  panel.renderStatus(runningStatus(true));
  panel.renderStatus(runningStatus(false));
  panel.renderConfig(makeConfig("rectangle", "horizontal", 2, 2, true));
  EXPECT_TRUE(panel.findChild<QPushButton *>("replan_button")->isEnabled());
  EXPECT_TRUE(panel.findChild<QPushButton *>("clear_button")->isEnabled());
}

TEST(CoveragePanelConfig, freezesCaptureSettingsWhileArchivePreparationOrRunIsActive)
{
  application();
  climbot_rviz_plugins::CoveragePanel panel;
  climbot_rviz_plugins::CoveragePanel::Status status;
  status.state = climbot_rviz_plugins::CoveragePanel::Status::STARTING;
  status.inspection_enabled = true;
  status.archive_state =
    climbot_interfaces::msg::InspectionArchiveStatus::PREPARING;
  status.archive_preflight_expected_images = 132U;
  status.archive_expected_images = 30U;
  status.archive_saved_images = 2U;
  status.archive_directory = "/srv/recorder/climbot_data/task/r1_run";
  status.archive_message = "Archive root preflight succeeded.";
  status.can_cancel = true;
  panel.renderStatus(status);

  auto * enabled = panel.findChild<QCheckBox *>("inspection_enabled_box");
  auto * root = panel.findChild<QLineEdit *>("archive_root_edit");
  ASSERT_NE(enabled, nullptr);
  ASSERT_NE(root, nullptr);
  EXPECT_FALSE(enabled->isEnabled());
  EXPECT_FALSE(root->isEnabled());
  EXPECT_TRUE(panel.findChild<QLabel *>("inspection_summary_value")->text().contains("Preparing"));
  const auto archive_count = panel.findChild<QLabel *>("archive_count_value")->text();
  EXPECT_TRUE(archive_count.contains("132"));
  EXPECT_TRUE(archive_count.contains("30"));
  EXPECT_TRUE(panel.findChild<QLabel *>("archive_directory_value")->text().contains("task"));
}

TEST(CoveragePanelConfig, recoveryLockKeepsMotionAndPlanningControlsClosed)
{
  application();
  climbot_rviz_plugins::CoveragePanel panel;
  climbot_rviz_plugins::CoveragePanel::Status status;
  status.state = climbot_rviz_plugins::CoveragePanel::Status::RECOVERY_LOCKED;
  status.can_rearm = true;
  panel.renderStatus(status);
  panel.renderConfig(makeConfig("rectangle", "horizontal", 2, 2, true));

  EXPECT_FALSE(panel.findChild<QPushButton *>("start_button")->isEnabled());
  EXPECT_FALSE(panel.findChild<QPushButton *>("cancel_button")->isEnabled());
  EXPECT_FALSE(panel.findChild<QPushButton *>("force_abandon_button")->isEnabled());
  EXPECT_TRUE(panel.findChild<QPushButton *>("rearm_button")->isEnabled());
  EXPECT_TRUE(panel.findChild<QPushButton *>("force_abandon_button")->isHidden());
  EXPECT_FALSE(panel.findChild<QPushButton *>("rearm_button")->isHidden());
  EXPECT_FALSE(panel.findChild<QPushButton *>("replan_button")->isEnabled());
  EXPECT_FALSE(panel.findChild<QPushButton *>("clear_button")->isEnabled());
  EXPECT_FALSE(box(panel, "region_box")->isEnabled());
  EXPECT_FALSE(box(panel, "sweep_box")->isEnabled());
  EXPECT_TRUE(
    panel.findChild<QLabel *>("state_value")->text().contains("Recovery"));
}

TEST(CoveragePanelConfig, forceAbandonNeedsASecondClick)
{
  application();
  climbot_rviz_plugins::CoveragePanel panel;
  climbot_rviz_plugins::CoveragePanel::Status status;
  status.state = climbot_rviz_plugins::CoveragePanel::Status::STOPPING;
  status.can_cancel = true;
  status.can_force_abandon = true;
  panel.renderStatus(status);

  auto * button = panel.findChild<QPushButton *>("force_abandon_button");
  ASSERT_NE(button, nullptr);
  ASSERT_TRUE(button->isEnabled());
  EXPECT_EQ(button->text(), QStringLiteral("Force abandon"));
  button->click();
  EXPECT_EQ(button->text(), QStringLiteral("Confirm force abandon"));
  // The second click reaches an unavailable service in this widget-only test,
  // but it proves the first click did not send and the confirmation is retired.
  button->click();
  EXPECT_EQ(button->text(), QStringLiteral("Force abandon"));
}

// A configure or algorithm request disables the control that sent it and is
// re-enabled only by the response. rclcpp never completes the future of a
// service that died after service_is_ready() passed, so without an expiry the
// region, sweep and algorithm boxes stay disabled until RViz itself restarts.
TEST(CoveragePanelConfig, aRequestThatIsNeverAnsweredStopsBeingWaitedFor)
{
  const auto sent = std::chrono::steady_clock::now();
  EXPECT_FALSE(climbot_rviz_plugins::requestHasExpired(sent, sent));
  EXPECT_FALSE(
    climbot_rviz_plugins::requestHasExpired(
      sent, sent + climbot_rviz_plugins::requestTimeout()));
  EXPECT_TRUE(
    climbot_rviz_plugins::requestHasExpired(
      sent, sent + climbot_rviz_plugins::requestTimeout() +
      std::chrono::milliseconds{1}));
  // A local service answers in milliseconds; a window that could be crossed by
  // an ordinary answer would cancel live requests instead of dead ones.
  EXPECT_GE(climbot_rviz_plugins::requestTimeout(), std::chrono::seconds{1});
}

TEST(CoveragePanelConfig, theBoxesCarryValuesThePlannerValidates)
{
  // The visible text may be translated; the data must stay the words the
  // planner accepts, or a localised build silently stops working.
  application();
  climbot_rviz_plugins::CoveragePanel panel;
  auto * region = box(panel, "region_box");
  auto * sweep = box(panel, "sweep_box");
  ASSERT_NE(region, nullptr);
  ASSERT_NE(sweep, nullptr);
  EXPECT_GE(region->findData("rectangle"), 0);
  EXPECT_GE(region->findData("trapezoid"), 0);
  EXPECT_EQ(region->count(), 2);
  EXPECT_GE(sweep->findData("horizontal"), 0);
  EXPECT_GE(sweep->findData("vertical"), 0);
  EXPECT_EQ(sweep->count(), 2);
  auto * algorithm = box(panel, "algorithm_box");
  ASSERT_NE(algorithm, nullptr);
  EXPECT_GE(algorithm->findData("time"), 0);
  EXPECT_GE(algorithm->findData("distance"), 0);
  EXPECT_EQ(algorithm->count(), 2);
  // Item zero is what shows while the box is still disabled, before the
  // executor has answered with the mode actually in force, so it has to be
  // the mode the executor defaults to.
  EXPECT_EQ(algorithm->itemData(0).toString(), QStringLiteral("time"));
}

TEST(CoveragePanelConfig, anUnknownValueLeavesTheBoxAloneRatherThanClearingIt)
{
  // A planner from a newer version could name a shape this panel does not
  // know. Showing the previous one is wrong, but blanking the control and
  // then sending that blank back would be worse.
  application();
  climbot_rviz_plugins::CoveragePanel panel;
  panel.renderConfig(makeConfig("trapezoid", "vertical", 3, 3, true));
  panel.renderConfig(makeConfig("hexagon", "vertical", 3, 3, true));
  auto * region = box(panel, "region_box");
  EXPECT_EQ(region->currentData().toString(), "trapezoid");
  EXPECT_GE(region->currentIndex(), 0) << "the control was left with no value";
}
