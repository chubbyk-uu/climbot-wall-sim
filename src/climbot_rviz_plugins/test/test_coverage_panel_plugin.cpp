#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "pluginlib/class_loader.hpp"
#include "rviz_common/panel.hpp"

// A panel that RViz cannot load is invisible rather than broken: RViz logs a
// warning and starts without it, so a plain build success proves nothing. This
// loads the class exactly as RViz does, which catches a stale library name in
// plugins_description.xml, a missing export macro and an unresolved symbol.
TEST(CoveragePanelPlugin, is_declared_and_loadable_as_an_rviz_panel)
{
  // Discovered through the ament index, the same way RViz discovers panels,
  // so the export registration itself is part of what this covers.
  pluginlib::ClassLoader<rviz_common::Panel> loader(
    "rviz_common", "rviz_common::Panel");
  const std::string name = "climbot_rviz_plugins/Coverage";
  const auto declared = loader.getDeclaredClasses();
  EXPECT_NE(std::find(declared.begin(), declared.end(), name), declared.end())
    << "The panel is not declared for the rviz_common::Panel base class.";
  ASSERT_TRUE(loader.isClassAvailable(name));
  EXPECT_EQ(loader.getClassType(name), "climbot_rviz_plugins::CoveragePanel");
  // Loading resolves the library and its symbols without needing a display.
  ASSERT_NO_THROW(loader.loadLibraryForClass(name));
  EXPECT_TRUE(loader.isClassLoaded(name));
  loader.unloadLibraryForClass(name);
}
