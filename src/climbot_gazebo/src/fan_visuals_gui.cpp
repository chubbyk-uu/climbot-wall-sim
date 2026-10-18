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

#include "climbot_gazebo/fan_visuals_gui.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <QJsonDocument>
#include <QJsonObject>

namespace climbot_gazebo
{
ClimbotFanVisuals::~ClimbotFanVisuals()
{
  node_.Unsubscribe("/suction/ground_truth/state");
}

void ClimbotFanVisuals::LoadConfig(const tinyxml2::XMLElement *)
{
  gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(this);
  node_.Subscribe<gz::msgs::StringMsg>("/suction/ground_truth/state",
    [this](const gz::msgs::StringMsg & message) {
      const auto doc = QJsonDocument::fromJson(QByteArray::fromStdString(message.data()));
      if (!doc.isObject()) {return;}
      const auto data = doc.object();
      if (!data["time_s"].isDouble() || !data["generation"].isDouble()) {return;}
      const double time = data["time_s"].toDouble();
      const double generation = data["generation"].toDouble();
      std::array<double, 3> speeds;
      for (int i = 0; i < 3; ++i) {
        const auto value = data[QString("fan_speed_%1").arg(i)];
        if (!value.isDouble() || !std::isfinite(value.toDouble())) {return;}
        speeds[i] = std::clamp(value.toDouble(), 0.0, 1.0);
      }
      if (!std::isfinite(time) || !std::isfinite(generation)) {return;}
      std::lock_guard<std::mutex> lock(mutex_);
      if (!previousTime_ || generation_ != generation || time < *previousTime_) {
        angles_ = {};
      } else {
        const double dt = time - *previousTime_;
        for (int i = 0; i < 3; ++i) {
          angles_[i] = std::fmod(angles_[i] + 0.6 * 2 * M_PI * dt *
          (previousSpeeds_[i] + speeds[i]) / 2, 2 * M_PI);
        }
      }
      previousTime_ = time;generation_ = generation;previousSpeeds_ = speeds;
    });
}

bool ClimbotFanVisuals::eventFilter(QObject * object, QEvent * event)
{
  if (event->type() == gz::gui::events::PreRender::kType) {
    const auto scene = gz::rendering::sceneFromFirstRenderEngine();
    if (scene) {
      if (scene_.lock() != scene) {scene_ = scene;rotors_ = {};}
      for (int i = 0; i < 3; ++i) {
        auto rotor = rotors_[i].lock();
        if (!rotor) {
          const std::string suffix = "::fan_" + std::to_string(i) + "_link_blades";
          for (unsigned int j = 0; j < scene->VisualCount(); ++j) {
            const auto candidate = scene->VisualByIndex(j);
            const auto & name = candidate->Name();
            if (name.size() >= suffix.size() &&
              name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0)
            {
              rotor = candidate;rotors_[i] = rotor;
              baseRotations_[i] = rotor->LocalRotation();
              break;
            }
          }
        }
        if (rotor) {
          double angle;
          {std::lock_guard<std::mutex> lock(mutex_);angle = angles_[i];}
          rotor->SetLocalRotation(baseRotations_[i] * gz::math::Quaterniond(0, 0, angle));
        }
      }
    }
  }
  return gz::gui::Plugin::eventFilter(object, event);
}
}  // namespace climbot_gazebo

GZ_ADD_PLUGIN(climbot_gazebo::ClimbotFanVisuals, gz::gui::Plugin)
