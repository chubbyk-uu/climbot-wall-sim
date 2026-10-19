// Copyright 2026 jerry
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// Optional acceptance-only observer. No force or controller modifications.
#include "climbot_gazebo/vacuum_state.hpp"
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <array>
#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace climbot_gazebo
{
class LoadRecorder : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPostUpdate,
  public gz::sim::ISystemReset
{
public:
  void Configure(
    const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm, gz::sim::EventManager &) override
  {
    entity_ = entity;gz::sim::Model model(entity);
    std::filesystem::path path = sdf->Get<std::string>("path");
    if (std::filesystem::exists(path)) {throw std::runtime_error("load trace already exists");}
    std::filesystem::create_directories(path.parent_path());
    out_.rdbuf()->pubsetbuf(buffer_.data(), buffer_.size());
    out_.open(path, std::ios::binary | std::ios::out);
    out_.exceptions(std::ios::badbit | std::ios::failbit);
    for (const auto & name : {"left_wheel_link", "right_wheel_link", "caster_ball_link"}) {
      gz::sim::Link link(model.LinkByName(ecm, name));
      collisions_[count_++] = link.Collisions(ecm).at(0);
    }
    body_ = gz::sim::Link(model.LinkByName(ecm, "chassis_link"));
    std::ofstream meta(path.string() + ".json");
    meta <<
      R"({"schema_version":1,"dtype":"<f8","columns":[
"sim_time_s","paused","vac_fixture_active","vac_release_time_s","vac_generation",
"vac_delta_pa","vac_runtime_failed","vac_startup_failed","vac_skirt_stop_previous_normal_n",
"pose_x","pose_y","pose_z",
"left_points","left_fx","left_fy","left_fz",
"right_points","right_fx","right_fy","right_fz",
"ball_points","ball_fx","ball_fy","ball_fz"]})";
  }

  void PostUpdate(
    const gz::sim::UpdateInfo & info,
    const gz::sim::EntityComponentManager & ecm) override
  {
    auto state = ecm.Component<vacuum::State>(entity_);
    auto pose = body_.WorldPose(ecm);
    if (!state || !pose || state->Data().size() != vacuum::Count) {return;}
    const auto & s = state->Data();
    std::array<double, 24> row{};
    row[0] = std::chrono::duration<double>(info.simTime).count();row[1] = info.paused;
    row[2] = s[vacuum::FixtureActive];row[3] = s[vacuum::ReleaseTime];
    row[4] = s[vacuum::Generation];row[5] = s[vacuum::DeltaPressure];
    row[6] = s[vacuum::RuntimeFailed];row[7] = s[vacuum::StartupFailed];
    row[8] = s[vacuum::SkirtStopNormal];
    row[9] = pose->X();row[10] = pose->Y();row[11] = pose->Z();
    for (int i = 0; i < 3; ++i) {
      auto data = ecm.Component<gz::sim::components::ContactSensorData>(collisions_[i]);
      if (!data) {continue;}
      for (const auto & c : data->Data().contact()) {
        bool first = c.collision1().id() == collisions_[i];
        if (!first && c.collision2().id() != collisions_[i]) {
          throw std::runtime_error("load recorder contact ownership mismatch");
        }
        for (const auto & w : c.wrench()) {
          const auto & force = first ? w.body_1_wrench().force() : w.body_2_wrench().force();
          ++row[12 + i * 4];
          row[13 + i * 4] += force.x();row[14 + i * 4] += force.y();
          row[15 + i * 4] += force.z();
        }
      }
    }
    out_.write(reinterpret_cast<const char *>(row.data()), sizeof(row));
    if (row[0] - flushed_ >= 1 || row[0] < flushed_) {
      out_.flush();flushed_ = row[0];
    }
  }

  void Reset(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &) override
  {
    // Preserve the open trace across clock rollback. Without ISystemReset,
    // Gazebo recreates this observer and Configure would reject its own file.
    out_.flush();flushed_ = -1;
  }

private:
  gz::sim::Entity entity_{};
  gz::sim::Link body_;
  std::array<gz::sim::Entity, 3> collisions_{};
  int count_{};
  std::array<char, 1 << 20> buffer_{};
  std::ofstream out_;
  double flushed_{};
};
}
GZ_ADD_PLUGIN(climbot_gazebo::LoadRecorder, gz::sim::System,
  gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate, gz::sim::ISystemReset)
