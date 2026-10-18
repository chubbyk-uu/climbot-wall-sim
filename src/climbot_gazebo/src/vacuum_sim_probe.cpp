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

// S3 independent native Gazebo observer: no force is inferred from vacuum demand.
#include "climbot_gazebo/vacuum_state.hpp"
#include <gz/common/Console.hh>
#include <gz/math/Matrix3.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/System.hh>
#include <functional>
#include <gz/sim/Util.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <yaml-cpp/yaml.h>
#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <set>
#include <string>
#include <vector>

namespace v = climbot_gazebo::vacuum;
using V3 = gz::math::Vector3d;
using ECM = gz::sim::EntityComponentManager;
namespace
{
// An in-memory system must implement Reset: gz-sim 8 retires systems without
// it, so TestFixture's helper cannot provide post-reset observation reliably.
class Observer : public gz::sim::System, public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate,
  public gz::sim::ISystemReset
{
public:
  std::function<void(ECM &)> configure;
  std::function<void(const gz::sim::UpdateInfo &, ECM &)> pre, reset;
  std::function<void(const gz::sim::UpdateInfo &, const ECM &)> post;
  void Configure(
    const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &,
    ECM & ecm, gz::sim::EventManager &) override {configure(ecm);}
  void PreUpdate(const gz::sim::UpdateInfo & info, ECM & ecm) override {pre(info, ecm);}
  void PostUpdate(const gz::sim::UpdateInfo & info, const ECM & ecm) override {post(info, ecm);}
  void Reset(const gz::sim::UpdateInfo & info, ECM & ecm) override {reset(info, ecm);}
};
V3 vector(const gz::msgs::Vector3d & p) {return {p.x(), p.y(), p.z()};}
void csv(std::ostream & out, const V3 & p) {out << ',' << p.X() << ',' << p.Y() << ',' << p.Z();}
double sec(std::chrono::steady_clock::duration t) {return std::chrono::duration<double>(t).count();}
void require(bool value, const char * text) {if (!value) {throw std::runtime_error(text);}}
}
int main(int argc, char ** argv)
{
  try {
    require(argc == 3, "usage: vacuum_sim_probe request.yaml NEW_OUTPUT_DIRECTORY");
    auto request = YAML::LoadFile(argv[1]);auto output = std::filesystem::path(argv[2]);
    std::vector<std::array<double, 3>> motion;
    if (request["motion_profile"]) {
      for (const auto & row : request["motion_profile"]) {
        std::array<double, 3> point{row["after_release_s"].as<double>(),
          row["linear_mps"].as<double>(), row["angular_rps"].as<double>()};
        require(std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2]) &&
          point[0] >= 0 && (motion.empty() || point[0] > motion.back()[0]),
          "invalid diagnostic motion time/value");
        motion.push_back(point);
      }
      require(!motion.empty(), "empty diagnostic motion profile");
    }
    require(!std::filesystem::exists(output), "output already exists");
    std::filesystem::create_directories(output);
    std::ofstream frames(output / "frames.csv"), contacts(output / "contacts.csv");
    frames << std::setprecision(17);contacts << std::setprecision(17);
    frames <<
      "iteration,sim_time_s,paused,pose_x,pose_y,pose_z,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz,mass_kg,cg_x,cg_y,cg_z,px,py,pz,hx,hy,hz";
    for (const auto & s : {"left", "right", "ball"}) {
      frames << ',' << s << "_valid," << s << "_stamp_s," << s << "_points," << s << "_fx," << s <<
        "_fy," << s << "_fz," << s << "_mx," << s << "_my," << s << "_mz";
    }
    for (auto name : v::kNames) {
      frames << ",vac_" << name;
    }
    frames << ",command_v_mps,command_omega_rps\n";
    contacts <<
      "iteration,sim_time_s,sensor,collision1_id,collision2_id,position_x,position_y,position_z,normal_x,normal_y,normal_z,depth,body1_fx,body1_fy,body1_fz,body2_fx,body2_fy,body2_fz,robot_fx,robot_fy,robot_fz,body1_tx,body1_ty,body1_tz,body2_tx,body2_ty,body2_tz\n";
    auto world = std::filesystem::path(argv[1]).parent_path() / request["world"].as<std::string>();
    double dt = request["step_s"].as<double>(), duration = request["duration_s"].as<double>();
    require(dt > 0 && duration > 0, "invalid probe interval");
    gz::sim::ServerConfig config;config.SetSdfFile(world.string());
    auto server = std::make_shared<gz::sim::Server>(config);
    auto observer = std::make_shared<Observer>();
    gz::sim::Model model;gz::sim::Link chassis;
    std::array<gz::sim::Entity, 3> sensors{}, collisions{};
    std::vector<gz::sim::Link> links;
    std::vector<double> lastState;
    bool diagnosticPoseApplied = false;
    double diagnosticPoseAt = -1;
    gz::math::Pose3d diagnosticPose;
    if (request["diagnostic_pose"]) {
      auto p = request["diagnostic_pose"];
      diagnosticPoseAt = p["time_s"].as<double>();
      auto xyz = p["xyz_m"].as<std::vector<double>>();
      auto rpy = p["rpy_rad"].as<std::vector<double>>();
      require(std::isfinite(diagnosticPoseAt) && diagnosticPoseAt > 0 &&
        xyz.size() == 3 && rpy.size() == 3 &&
        std::all_of(xyz.begin(), xyz.end(), [](double x) {return std::isfinite(x);}) &&
        std::all_of(rpy.begin(), rpy.end(), [](double x) {return std::isfinite(x);}),
        "invalid explicit diagnostic pose");
      diagnosticPose = gz::math::Pose3d(xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]);
    }
    double commandedV = 0, commandedW = 0;
    gz::math::Pose3d lastPose;
    unsigned rows = 0, pausedRows = 0, resetSnapshots = 0;bool valid = true;
    auto initialize = [&](ECM & ecm) {
        if (!chassis.Valid(ecm)) {
          auto id = ecm.EntityByComponents(gz::sim::components::Model(),
          gz::sim::components::Name("climbot"));
          require(id != gz::sim::kNullEntity, "climbot model not loaded");
          model = gz::sim::Model(id);
          chassis = gz::sim::Link(model.LinkByName(ecm, "chassis_link"));
          ecm.Each<gz::sim::components::Inertial, gz::sim::components::ParentEntity>(
            [&](const gz::sim::Entity & entity, const auto *, const auto * parent) {
              if (parent->Data() == id) {
                gz::sim::Link link(entity);link.EnableVelocityChecks(ecm);
                if (!ecm.Component<gz::sim::components::WorldPose>(entity)) {
                  ecm.CreateComponent(entity,
                gz::sim::components::WorldPose(gz::sim::worldPose(entity, ecm)));
                }
                links.push_back(link);
              }
              return true;
          });
          std::array<std::string, 3> names{"left_wheel_link", "right_wheel_link",
            "caster_ball_link"};
          for (int i = 0; i < 3; ++i) {
            gz::sim::Link link(model.LinkByName(ecm, names[i]));
            std::string side = i == 0 ? "left" : i == 1 ? "right" : "caster";
            sensors[i] = link.SensorByName(ecm, side + "_contact");
            collisions[i] = link.CollisionByName(ecm, side + "_collision");
            require(sensors[i] != gz::sim::kNullEntity && collisions[i] != gz::sim::kNullEntity,
            "native contact sensor missing");
            if (!ecm.Component<gz::sim::components::ContactSensorData>(collisions[i])) {
              ecm.CreateComponent(collisions[i], gz::sim::components::ContactSensorData());
            }
          }
        }
      };
    observer->configure = initialize;
    observer->pre = [&](const gz::sim::UpdateInfo & update, ECM & ecm) {
        if (!valid) {return;}
        try {
          initialize(ecm);
      // Reset restores the original ECM snapshot, including removing optional
      // feedback components requested by this observer after initial loading.
          for (auto & link : links) {
            link.EnableVelocityChecks(ecm);
            if (!ecm.Component<gz::sim::components::WorldPose>(link.Entity())) {
              ecm.CreateComponent(link.Entity(), gz::sim::components::WorldPose(
            gz::sim::worldPose(link.Entity(), ecm)));
            }
          }
      // Explicit actuator-level diagnostic only; normal launch uses DiffDrive.
      // The native observer is appended after model plugins, so these commands
      // replace the zero DiffDrive command for this isolated prescribed case.
          auto state = ecm.Component<v::State>(model.Entity());
          // An explicit test-only domain-exit input, never used by normal
          // motion/acceptance runs. Repeat it in each reset generation.
          if (!update.paused && !diagnosticPoseApplied && diagnosticPoseAt > 0 &&
            sec(update.simTime) >= diagnosticPoseAt)
          {
            model.SetWorldPoseCmd(ecm, diagnosticPose);diagnosticPoseApplied = true;
          }
          if (!update.paused && state && (request["command_v_mps"] || !motion.empty()) &&
            state->Data()[v::ReleaseTime] >= 0)
          {
            double after = state->Data()[v::Time] - state->Data()[v::ReleaseTime];
            double delay = request["command_delay_s"] ? request["command_delay_s"].as<double>() : 2;
            if (after >= delay || !motion.empty()) {
              double vel = 0, omega = 0;
              if (!motion.empty()) {
                auto next = std::upper_bound(motion.begin(), motion.end(), after,
                    [](double time, const auto & point) {return time < point[0];});
                if (next != motion.begin()) {
                  auto previous = next - 1;
                  double fraction = next == motion.end() ? 0 :
                    (after - (*previous)[0]) / ((*next)[0] - (*previous)[0]);
                  vel = (*previous)[1];omega = (*previous)[2];
                  if (next != motion.end()) {
                    vel += fraction * ((*next)[1] - vel);
                    omega += fraction * ((*next)[2] - omega);
                  }
                }
              } else {
                vel = request["command_v_mps"].as<double>();
                omega = request["command_omega_rps"].as<double>();
              }
              commandedV = vel;commandedW = omega;
              double track = request["track_m"].as<double>(),
                radius = request["radius_m"].as<double>();
              for (int i = 0; i < 2; ++i) {
                auto joint = model.JointByName(ecm,
                i == 0 ? "left_wheel_joint" : "right_wheel_joint");
                double value = (vel + (i == 0 ? -1 : 1) * track * omega / 2) / radius;
                if (!ecm.Component<gz::sim::components::JointVelocityCmd>(joint)) {
                  ecm.CreateComponent(joint, gz::sim::components::JointVelocityCmd({value}));
                } else {
                  ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(joint, {value});
                }
              }
            }
          }
        } catch (const std::exception & error) {
          valid = false;gzerr << "Native observer PreUpdate failed: " << error.what() << std::endl;
        }
      };observer->post = [&](const gz::sim::UpdateInfo & info, const ECM & ecm) {
        if (!valid) {return;}
        try {
          auto state = ecm.Component<v::State>(model.Entity());
          require(state && state->Data().size() == v::Count,
          "vacuum state missing: plugin not loaded or configuration rejected");
          bool resetSnapshot = !lastState.empty() &&
            state->Data()[v::Generation] > lastState[v::Generation];
          lastState = state->Data();
          if (resetSnapshot) {++resetSnapshots;return;}
          auto currentPose = chassis.WorldPose(ecm);
          require(currentPose.has_value(), "native world pose feedback missing");
          lastPose = *currentPose;
          if (info.paused) {++pausedRows;} else {++rows;}
          auto lin = chassis.WorldLinearVelocity(ecm), ang = chassis.WorldAngularVelocity(ecm);
          require(lin && ang, "observer velocity missing");
          auto q = lastPose.Rot();
          frames << info.iterations << ',' << sec(info.simTime) << ',' << info.paused;
          csv(frames, lastPose.Pos());
          frames << ',' << q.X() << ',' << q.Y() << ',' << q.Z() << ',' << q.W();
          csv(frames, *lin);csv(frames, *ang);
          double mass = 0;V3 first{}, momentum{}, angular{};
          for (const auto & link : links) {
            auto inertia = ecm.Component<gz::sim::components::Inertial>(link.Entity());
            auto observedPose = link.WorldPose(ecm);
            require(observedPose.has_value(), "inertial link world pose missing");
            auto pose = *observedPose;auto ip = inertia->Data().Pose();
            auto cg = pose.CoordPositionAdd(ip.Pos());
            auto vcom = link.WorldLinearVelocity(ecm, ip.Pos()), w = link.WorldAngularVelocity(ecm);
            require(vcom && w, ("whole-assembly momentum observation missing: " +
              link.Name(ecm).value_or("unknown") + " time=" + std::to_string(sec(info.simTime)) +
              " paused=" + std::to_string(info.paused) + " generation=" +
              std::to_string(lastState[v::Generation])).c_str());
            double m = inertia->Data().MassMatrix().Mass();mass += m;first += cg * m;
            momentum += *vcom * m;
            gz::math::Matrix3d rotation(pose.Rot() * ip.Rot());
            auto tensor = rotation * inertia->Data().MassMatrix().Moi() * rotation.Transposed();
            angular += tensor * (*w) + cg.Cross(*vcom * m);
          }
          frames << ',' << mass;csv(frames, first / mass);csv(frames, momentum);
          csv(frames, angular);
          for (int index = 0; index < 3; ++index) {
            auto data = ecm.Component<gz::sim::components::ContactSensorData>(collisions[index]);
            V3 force{}, torque{};int points = 0;double stamp = -1;
            if (data) {
              stamp = data->Data().header().stamp().sec() + data->Data().header().stamp().nsec() *
                1e-9;
              for (const auto & contact : data->Data().contact()) {
                bool robotFirst = contact.collision1().id() == collisions[index];
                require(robotFirst || contact.collision2().id() == collisions[index],
                "contact collision ownership mismatch");
                for (int j = 0; j < contact.wrench_size(); ++j) {
                  require(j < contact.position_size() && j < contact.normal_size(),
                  "incomplete native contact tuple");
                  const auto & wrench = contact.wrench(j);
                  V3 one = vector(wrench.body_1_wrench().force()),
                    two = vector(wrench.body_2_wrench().force());
                  V3 f = robotFirst ? one : two, p = vector(contact.position(j));
                  force += f;torque += (p - lastPose.Pos()).Cross(f);++points;
                  contacts << info.iterations << ',' << sec(info.simTime) << ',' << index << ',' <<
                    contact.collision1().id() << ',' << contact.collision2().id();
                  csv(contacts, p);csv(contacts, vector(contact.normal(j)));
                  contacts << ',' << (j < contact.depth_size() ? contact.depth(j) : -1);
                  csv(contacts, one);csv(contacts, two);csv(contacts, f);
                  csv(contacts, vector(wrench.body_1_wrench().torque()));
                  csv(contacts, vector(wrench.body_2_wrench().torque()));contacts << '\n';
                }
              }
            }
            frames << ',' << static_cast<bool>(data) << ',' << stamp << ',' << points;
            csv(frames, force);csv(frames, torque);
          }
          for (double value : lastState) {
            frames << ',' << value;
          }
          frames << ',' << commandedV << ',' << commandedW << '\n';
        } catch (const std::exception & error) {
          valid = false;gzerr << "Native observer PostUpdate failed: " << error.what() << std::endl;
        }
      };
    observer->reset = [&](const gz::sim::UpdateInfo &, ECM & ecm) {
        commandedV = 0;commandedW = 0;diagnosticPoseApplied = false;
        initialize(ecm);
        for (auto & link : links) {
          link.EnableVelocityChecks(ecm);
        }
        auto state = ecm.Component<v::State>(model.Entity());
        require(state && state->Data().size() == v::Count, "reset state missing");
        lastState = state->Data();++resetSnapshots;
      };
    require(server->AddSystem(observer).value_or(false), "native observer installation failed");
    // Record actual loaded objects, not merely requested environment values.
    // Paths stay out of the portable evidence; the runner freezes their hashes.
    std::ifstream maps("/proc/self/maps");std::string mapping;
    std::set<std::string> mappedMesa;unsigned privateObjects = 0;bool systemGallium = false;
    auto dri = std::getenv("LIBGL_DRIVERS_PATH");
    auto privateDirectory = dri ? std::filesystem::path(dri).parent_path().string() : "";
    std::set<std::string> privateNames;
    while (std::getline(maps, mapping)) {
      auto start = mapping.find('/');if (start == std::string::npos) {continue;}
      auto path = mapping.substr(start);auto name = std::filesystem::path(path).filename().string();
      bool mesa = name.find("libgallium") == 0 || name.find("libGLX_mesa") == 0 ||
        name.find("libEGL_mesa") == 0 || name.find("libgbm.so") == 0;
      if (!mesa) {continue;}
      mappedMesa.insert(name);
      bool selected = !privateDirectory.empty() && path.rfind(privateDirectory + "/", 0) == 0;
      if (selected) {privateNames.insert(name);}
      if (name.find("libgallium") == 0 && !selected) {systemGallium = true;}
    }
    privateObjects = privateNames.size();
    std::ofstream libraryRecord(output / "runtime_libraries.json");
    libraryRecord << "{\"libraries\":[";
    bool firstLibrary = true;
    for (const auto & name : mappedMesa) {
      if (!firstLibrary) {libraryRecord << ',';} firstLibrary = false;
      libraryRecord << '"' << name << '"';
    }
    libraryRecord << "],\"private_mapping_complete\":" << (privateObjects == 4 ? "true" : "false")
                  << ",\"system_gallium_mapped\":" << (systemGallium ? "true" : "false") << "}\n";
    libraryRecord.close();
    auto start = std::chrono::steady_clock::now();
    double pauseAt = request["pause_at_s"] ? request["pause_at_s"].as<double>() : 0;
    unsigned pauseIterations = request["pause_iterations"] ? request["pause_iterations"].as<unsigned>() : 0;
    double resetAt = request["reset_at_s"] ? request["reset_at_s"].as<double>() : 0;
    bool pauseEqual = true, posePauseEqual = true, resetOk = true;
    double pauseRefreshM = 0, pauseRefreshRad = 0;
    std::vector<double> beforeReset, afterReset;
    uint64_t ran = 0, total = static_cast<uint64_t>(std::llround(duration / dt));
    auto run = [&](uint64_t count) {
        require(server->Run(true, count, false), "Gazebo run failed");ran += count;
      };
    if (pauseAt > 0) {
      run(static_cast<uint64_t>(std::llround(pauseAt / dt)));
      require(pauseIterations >= 2, "pause requires a boundary refresh and a held interval");
      auto snapshot = lastState;auto before = lastPose;
      // Physics::Update skips Step while paused, but ChangedLinks without a
      // ChangedWorldPoses result refreshes the backend's cached link poses.
      // Record this boundary refresh separately; never call it a physics step.
      require(server->RunOnce(true), "paused boundary refresh failed");
      pauseRefreshM = (lastPose.Pos() - before.Pos()).Length();
      pauseRefreshRad = (before.Rot().Inverse() * lastPose.Rot()).Euler().Length();
      auto pose = lastPose;
      pauseEqual = lastState == snapshot;
      for (unsigned i = 1; i < pauseIterations; ++i) {
        require(server->RunOnce(true), "paused iteration failed");
        pauseEqual = pauseEqual && lastState == snapshot;
        posePauseEqual = posePauseEqual && lastPose == pose;
      }
    }
    if (resetAt > 0) {
      uint64_t target = static_cast<uint64_t>(std::llround(resetAt / dt));
      require(target > ran, "reset precedes pause");
      run(target - ran);beforeReset = lastState;
      server->ResetAll();require(server->RunOnce(true), "reset snapshot failed");
      afterReset = lastState;
      resetOk = afterReset[v::CavityPressure] == afterReset[v::AmbientPressure] &&
        afterReset[v::FixtureActive] == 1 &&
        afterReset[v::ReleaseTime] == -1 && afterReset[v::Speed0] == 0 &&
        afterReset[v::Speed1] == 0 && afterReset[v::Speed2] == 0 &&
        afterReset[v::Generation] > beforeReset[v::Generation];
    }
    require(total > ran, "duration must exceed inspection events");run(total - ran);
    frames.close();contacts.close();
    require(!lastState.empty(), "no vacuum state was observed; native run invalid");
    std::ofstream events(output / "run.json");events << std::setprecision(17);
    events << "{\"native_frames\":" << rows << ",\"paused_frames\":" << pausedRows
           << ",\"pause_state_equal\":" << (pauseEqual ? "true" : "false")
           << ",\"pause_pose_equal\":" << (posePauseEqual ? "true" : "false")
           << ",\"pause_boundary_refresh_m\":" << pauseRefreshM
           << ",\"pause_boundary_refresh_rad\":" << pauseRefreshRad
           << ",\"reset_restored\":" << (resetOk ? "true" : "false")
           << ",\"reset_requested\":" << (resetAt > 0 ? "true" : "false")
           << ",\"reset_snapshots\":" << resetSnapshots
           << ",\"state_present\":" << (valid ? "true" : "false")
           << ",\"last_runtime_failed\":" << (lastState[v::RuntimeFailed] != 0 ? "true" : "false")
           << ",\"last_startup_failed\":" << (lastState[v::StartupFailed] != 0 ? "true" : "false")
           << ",\"elapsed_wall_s\":" << sec(std::chrono::steady_clock::now() - start) << "}\n";
    require(valid && pauseEqual && posePauseEqual && resetOk, "native lifecycle check failed");
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';return 2;
  }
}
