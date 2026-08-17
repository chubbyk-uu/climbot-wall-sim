// Foolproofing test for the panel's region and sweep controls.
//
// These two drop-downs are the only widgets in the panel that send anything
// other than a fixed request, so they are the only place a stale or invented
// value can reach the planner. Everything below drives the render path an
// operator's clicking would drive, and checks the panel keeps showing what the
// planner published rather than what someone last picked.

#include <gtest/gtest.h>

#include <QApplication>
#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QString>

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
  if (clear != nullptr) {
    EXPECT_FALSE(clear->isEnabled());
  }
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
