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

#ifndef CLIMBOT_GAZEBO__COMPACT_STATS_GUI_HPP_
#define CLIMBOT_GAZEBO__COMPACT_STATS_GUI_HPP_

#include <gz/gui/Plugin.hh>
#include <gz/transport/Node.hh>
#include <QString>

namespace climbot_gazebo
{
class ClimbotWorldStats : public gz::gui::Plugin
{
  Q_OBJECT
  Q_PROPERTY(QString factor READ Factor NOTIFY FactorChanged)

public:
  ~ClimbotWorldStats() override;
  QString Factor() const;

signals:
  void FactorChanged();

protected:
  void LoadConfig(const tinyxml2::XMLElement * element) override;

private:
  QString factor_{"--"};
  gz::transport::Node node_;
};
}  // namespace climbot_gazebo
#endif  // CLIMBOT_GAZEBO__COMPACT_STATS_GUI_HPP_
