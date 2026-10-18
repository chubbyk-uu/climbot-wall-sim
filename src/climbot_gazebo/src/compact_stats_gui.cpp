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

#include "climbot_gazebo/compact_stats_gui.hpp"

#include <cmath>
#include <gz/msgs/world_stats.pb.h>
#include <gz/plugin/Register.hh>
#include <QMetaObject>

namespace climbot_gazebo
{
ClimbotWorldStats::~ClimbotWorldStats()
{
  node_.Unsubscribe("/world/climbot_wall/stats");
}

QString ClimbotWorldStats::Factor() const
{
  return factor_;
}

void ClimbotWorldStats::LoadConfig(const tinyxml2::XMLElement *)
{
  title = "Realtime factor";
  node_.Subscribe<gz::msgs::WorldStatistics>("/world/climbot_wall/stats",
    [this](const gz::msgs::WorldStatistics & message) {
      const double value = message.real_time_factor();
      const QString text = message.paused() ? "Paused" :
      (std::isfinite(value) && value >= 0.0 ?
      QString::number(value * 100.0, 'f', 1) + "%" : "--");
      // Transport callbacks never touch the QML item from a worker thread.
      QMetaObject::invokeMethod(this, [this, text]() {
        factor_ = text;
        emit FactorChanged();
        }, Qt::QueuedConnection);
    });
}
}  // namespace climbot_gazebo

GZ_ADD_PLUGIN(climbot_gazebo::ClimbotWorldStats, gz::gui::Plugin)
