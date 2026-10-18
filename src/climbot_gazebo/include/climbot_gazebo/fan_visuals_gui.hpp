// Copyright 2026 jerry
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CLIMBOT_GAZEBO__FAN_VISUALS_GUI_HPP_
#define CLIMBOT_GAZEBO__FAN_VISUALS_GUI_HPP_

#include <array>
#include <memory>
#include <mutex>
#include <optional>
#include <gz/gui/Plugin.hh>
#include <gz/math/Quaternion.hh>
#include <gz/rendering/RenderTypes.hh>
#include <gz/transport/Node.hh>

namespace climbot_gazebo
{
// Display animation only. No rotor rigid bodies, torque, collision or sensor
// pixels are changed. 0.6 rev/s at normalized speed 1 makes motion readable;
// actual motor RPM is not known and is not inferred from the display speed.
class ClimbotFanVisuals : public gz::gui::Plugin
{
  Q_OBJECT

public:
  ~ClimbotFanVisuals() override;

protected:
  void LoadConfig(const tinyxml2::XMLElement *) override;
  bool eventFilter(QObject * object, QEvent * event) override;

private:
  std::mutex mutex_;
  std::array<double, 3> angles_{};
  std::array<double, 3> previousSpeeds_{};
  std::optional<double> previousTime_, generation_;
  // Rendering objects are accessed exclusively from the PreRender callback.
  std::weak_ptr<gz::rendering::Scene> scene_;
  std::array<std::weak_ptr<gz::rendering::Visual>, 3> rotors_;
  std::array<gz::math::Quaterniond, 3> baseRotations_;
  gz::transport::Node node_;
};
}  // namespace climbot_gazebo
#endif  // CLIMBOT_GAZEBO__FAN_VISUALS_GUI_HPP_
