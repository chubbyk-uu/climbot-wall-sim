// Copyright 2026 jerry
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// T0 observer; optional isolated sweep drives tangent motion and normal preload.
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

using ECM = gz::sim::EntityComponentManager;
class Observer : public gz::sim::System, public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
public:
  std::ofstream frames, contacts;
  std::vector<gz::sim::Link> links;
  std::vector<gz::sim::Entity> collisions;
  std::vector<std::string> names;
  std::vector<gz::sim::Entity> scanJoints;
  double scanSpeed = 0, settleSeconds = 0, normalForce = 0;
  void PreUpdate(const gz::sim::UpdateInfo & info, ECM & ecm) override
  {
    if (links.empty()) {
      ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
        [&](gz::sim::Entity entity, const auto *, const auto * name) {
          if (name->Data().rfind("probe_", 0) != 0) {return true;}
          gz::sim::Model model(entity);
          gz::sim::Link link(model.LinkByName(ecm, "body"));
          auto collision = link.CollisionByName(ecm, "collision");
          if (!collision) {throw std::runtime_error("probe collision missing");}
          ecm.CreateComponent(collision, gz::sim::components::ContactSensorData());
          links.push_back(link);collisions.push_back(collision);names.push_back(name->Data());
          if (scanSpeed > 0) {
            auto joint = model.JointByName(ecm, "scan_slide");
            if (!joint) {throw std::runtime_error("missing diagnostic scan joint");}
            scanJoints.push_back(joint);
          }
          return true;
      });
    }
    if (scanSpeed <= 0 || info.paused) {return;}
    const double time = std::chrono::duration<double>(info.simTime).count();
    for (std::size_t i = 0; i < links.size(); ++i) {
      const std::vector<double> command{time >= settleSeconds ? scanSpeed : 0.0};
      auto velocity = ecm.Component<gz::sim::components::JointVelocityCmd>(scanJoints[i]);
      if (velocity) {velocity->Data() = command;} else {
        ecm.CreateComponent(scanJoints[i], gz::sim::components::JointVelocityCmd(command));
      }
      // Isolated fixture only: normal axis remains free; tangent motion is imposed.
      links[i].AddWorldWrench(ecm, gz::math::Vector3d(-normalForce, 0, 0),
        gz::math::Vector3d::Zero);
    }
  }
  void PostUpdate(const gz::sim::UpdateInfo & info, const ECM & ecm) override
  {
    const double time = std::chrono::duration<double>(info.simTime).count();
    for (std::size_t i = 0; i < links.size(); ++i) {
      auto pose = links[i].WorldPose(ecm);
      auto data = ecm.Component<gz::sim::components::ContactSensorData>(collisions[i]);
      if (!pose || !data) {throw std::runtime_error("missing native contact/pose");}
      frames << time << ',' << names[i] << ',' << pose->Pos().X() << ',' <<
        pose->Pos().Y() << ',' << pose->Pos().Z() << ',' << data->Data().contact_size() << '\n';
      for (const auto & contact : data->Data().contact()) {
        for (int j = 0; j < contact.position_size(); ++j) {
          if (j >= contact.normal_size() || j >= contact.depth_size()) {
            throw std::runtime_error("incomplete native contact tuple");
          }
          const auto & p = contact.position(j);const auto & n = contact.normal(j);
          gz::math::Vector3d force;
          if (j < contact.wrench_size()) {
            const auto & w = contact.wrench(j);
            const auto & f = contact.collision1().id() == collisions[i] ?
              w.body_1_wrench().force() : w.body_2_wrench().force();
            force.Set(f.x(), f.y(), f.z());
          }
          contacts << time << ',' << names[i] << ',' << p.x() << ',' << p.y() << ',' <<
            p.z() << ',' << n.x() << ',' << n.y() << ',' << n.z() << ',' <<
            contact.depth(j) << ',' << force.X() << ',' << force.Y() << ',' << force.Z() << ',' <<
            contact.collision1().id() << ',' << contact.collision2().id() << ',' <<
            contact.collision1().name() << ',' << contact.collision2().name() << '\n';
        }
      }
    }
  }
};
int main(int argc, char ** argv)
{
  try {
    if (argc != 3) {throw std::runtime_error("usage: wall_surface_probe request.yaml NEW_DIR");}
    auto request = YAML::LoadFile(argv[1]);
    auto out = std::filesystem::path(argv[2]);
    if (!std::filesystem::create_directory(out)) {throw std::runtime_error("output exists");}
    const auto iterations = request["iterations"].as<unsigned int>();
    if (!iterations || iterations > 100000) {throw std::runtime_error("invalid iterations");}
    auto observer = std::make_shared<Observer>();
    if (request["sweep"]) {
      auto sweep = request["sweep"];
      observer->scanSpeed = sweep["speed_mps"].as<double>();
      observer->settleSeconds = sweep["settle_s"].as<double>();
      observer->normalForce = sweep["extra_normal_force_n"].as<double>();
      if (!(observer->scanSpeed > 0 && observer->scanSpeed <= .1) ||
        !(observer->settleSeconds >= 0) || !(observer->normalForce >= 0 &&
        observer->normalForce <= 300)) {throw std::runtime_error("invalid diagnostic sweep");}
    }
    observer->frames.open(out / "frames.csv");observer->contacts.open(out / "contacts.csv");
    observer->frames << std::setprecision(17) << "time_s,name,x,y,z,contacts\n";
    observer->contacts << std::setprecision(17) <<
      "time_s,name,x,y,z,nx,ny,nz,depth_m,force_x_n,force_y_n,force_z_n,"
      "collision1_id,collision2_id,collision1_name,collision2_name\n";
    gz::sim::ServerConfig config;
    config.SetSdfFile((std::filesystem::path(argv[1]).parent_path() /
      request["world"].as<std::string>()).string());
    const auto start = std::chrono::steady_clock::now();
    gz::sim::Server server(config);server.AddSystem(observer);
    if (!server.Run(true, iterations, false) || observer->links.empty()) {
      throw std::runtime_error("native run failed or no probes loaded");
    }
    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();
    std::set<std::string> odeLibraries;
    std::ifstream maps("/proc/self/maps");std::string line;
    while (std::getline(maps, line)) {
      auto slash = line.find('/');
      if (slash != std::string::npos && line.find("libode.so.", slash) != std::string::npos) {
        odeLibraries.insert(line.substr(slash));
      }
    }
    std::ofstream metrics(out / "run.yaml");
    metrics << "wall_seconds: " << std::setprecision(17) << elapsed << "\niterations: " <<
      iterations << "\nprobes: " << observer->links.size() << '\n';
    YAML::Node libraries;
    for (const auto & path : odeLibraries) {
      libraries.push_back(path);
    }
    metrics << "ode_libraries:\n" << YAML::Dump(libraries) << '\n';
    std::cout << "Native surface probe complete: " << observer->links.size() << " probes\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';return 1;
  }
}
