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

#include "climbot_gazebo/vacuum_core.hpp"
#include "climbot_gazebo/vacuum_io.hpp"
#include "climbot_gazebo/vacuum_state.hpp"
#include "climbot_gazebo/wall_surface.hpp"
#include <gz/common/Console.hh>
#include <gz/math/Matrix3.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/SystemPluginInfo.hh>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace climbot_gazebo
{
namespace v = vacuum;
using V3 = gz::math::Vector3d;
using Pose = gz::math::Pose3d;
using ECM = gz::sim::EntityComponentManager;
namespace
{
void require(bool valid, const char * message)
{if (!valid) {throw std::runtime_error(message);}}
V3 vector(v::Vec3 x) {return {x.x, x.y, x.z};}
double seconds(std::chrono::steady_clock::duration d)
{return std::chrono::duration<double>(d).count();}
double read(const std::shared_ptr<const sdf::Element> & s, const std::string & key)
{
  require(s->HasElement(key), ("missing parameter: " + key).c_str());
  double value = s->Get<double>(key);
  require(std::isfinite(value), ("nonfinite parameter: " + key).c_str());return value;
}
double coverage(double x, double size, double width)
{
  double t = std::clamp((std::min(x, size - x) + width / 2) / width, 0.0, 1.0);
  return t * t * (3 - 2 * t);
}
struct Applied {V3 force{}, moment{};};
}  // namespace

class VacuumSystem : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemReset
{
public:
  void Configure(
    const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & source,
    ECM & ecm, gz::sim::EventManager &) override
  {
    auto sdf = source->Clone();
    model_ = gz::sim::Model(entity);entity_ = entity;
    link_ = gz::sim::Link(model_.LinkByName(ecm, "chassis_link"));
    require(link_.Valid(ecm), "vacuum requires chassis_link");
    auto world = gz::sim::worldEntity(ecm);
    const auto * info = ecm.Component<gz::sim::components::SystemPluginInfo>(world);
    if (info) {
      for (const auto & p : info->Data().plugins()) {
        require(p.filename().find("apply-link-wrench") == std::string::npos,
          "vacuum and ApplyLinkWrench are mutually exclusive");
      }
    }
    auto worldGravity = ecm.Component<gz::sim::components::Gravity>(world);
    require(worldGravity != nullptr, "world gravity missing");gravity_ = worldGravity->Data();
    mass_ = read(sdf, "model_mass_kg");cg_ = sdf->Get<V3>("model_cg");
    track_ = read(sdf, "track_m");caster_ = read(sdf, "caster_x_m");
    driveMu_ = read(sdf, "drive_mu");margin_ = read(sdf, "margin");
    driveFloor_ = read(sdf, "drive_floor_n");ballFloor_ = read(sdf, "caster_floor_n");
    rollingCoefficient_ = read(sdf, "caster_rolling_coefficient");
    rollingEpsilon_ = read(sdf, "caster_rolling_regularization_mps");
    ballRadius_ = read(sdf, "caster_radius_m");
    inner_ = read(sdf, "inner_area_m2");volumeNominal_ = read(sdf, "nominal_volume_m3");
    cx_ = read(sdf, "centre_x_m");gapNominal_ = read(sdf, "nominal_gap_m");
    fixed_ = read(sdf, "fixed_delta_pa");substep_ = read(sdf, "maximum_pressure_substep_s");
    preload_ = read(sdf, "fixture_preload_n");readyMargin_ = read(sdf, "ready_margin");
    benchHold_ = sdf->Get<bool>("fixture_bench_hold");
    dwell_ = read(sdf, "ready_dwell_s");timeout_ = read(sdf, "startup_timeout_s");
    velocityTolerance_ = read(sdf, "fixture_velocity_tolerance_mps");
    angularTolerance_ = read(sdf, "fixture_angular_tolerance_rps");
    baseLeak_ = read(sdf, "residual_area_m2") + read(sdf, "reference_area_m2") *
      read(sdf, "controlled_opening_multiplier");
    std::string mode = sdf->Get<std::string>("pressure_mode");
    require(mode == "fan_ode" || mode == "fixed_pressure", "invalid vacuum source mode");
    ideal_ = mode == "fixed_pressure";
    if (sdf->HasElement("wall_surface_yaml")) {
      require(ideal_, "M2 contact patch requires fixed_pressure until channel validation");
      surface_ = std::make_unique<WallSurface>(sdf->Get<std::string>("wall_surface_yaml"));
    }
    require(mass_ > 0 && track_ > 0 && caster_ < 0 && inner_ > 0 && volumeNominal_ > 0 &&
      substep_ > 0 && substep_ <= .0001 && preload_ >= 0 && readyMargin_ >= 1 &&
      dwell_ > 0 && timeout_ > dwell_ && baseLeak_ >= 0 && fixed_ > 0 && fixed_ < 40000 &&
      rollingCoefficient_ >= 0 && rollingEpsilon_ > 0 && ballRadius_ > 0,
      "invalid vacuum runtime domain");
    auto gas = sdf->GetElement("gas");v::Gas g;
    g.ambient_pa = read(gas, "ambient_pa");g.temperature_k = read(gas, "temperature_k");
    g.specific_r = read(gas, "specific_r");g.gamma = read(gas, "gamma");
    g.discharge_coefficient = read(gas, "discharge_coefficient");
    g.regularization_pa = read(gas, "regularization_pa");
    std::vector<v::Fan> fans;
    for (auto f = sdf->GetElement("fan"); f; f = f->GetNextElement("fan")) {
      fans.push_back({read(f, "free_flow_m3_s"), read(f, "shutoff_pa"), read(f, "time_constant_s"),
          read(f, "port_area_m2"), read(f, "reverse_fraction")});
    }
    require(fans.size() == 3, "exactly three independent fans required");
    for (const auto & fan : fans) {fanHeads_.push_back(fan.shutoff_pa);}
    core_ = std::make_unique<v::Chamber>(g, fans);
    io_ = std::make_unique<v::VacuumIO>(sdf);
    auto sk = sdf->GetElement("skirt");
    parameters_.inner_area_m2 = inner_;
    parameters_.band_m = read(sk, "contact_band_m");
    parameters_.lip_weight = read(sk, "lip_pressure_weight");
    parameters_.stiffness_n_m = read(sk, "stiffness_n_m");
    parameters_.damping_ns_m = read(sk, "damping_ns_m");
    parameters_.free_height_m = read(sk, "free_height_m");
    parameters_.maximum_compression_m = read(sk, "maximum_compression_m");
    parameters_.transition_m = read(sk, "transition_m");
    parameters_.sliding_mu = read(sk, "sliding_mu");
    parameters_.velocity_regularization_mps = read(sk, "velocity_regularization_mps");
    for (auto pt = sdf->GetElement("sample"); pt; pt = pt->GetNextElement("sample")) {
      attachments_.push_back(pt->Get<V3>("attachment"));lengths_.push_back(read(pt, "length_m"));
    }
    require(attachments_.size() >= 8, "insufficient skirt quadrature");
    wall_ = sdf->Get<Pose>("wall_pose");size_ = sdf->Get<gz::math::Vector2d>("wall_size");
    require(size_.X() > 0 && size_.Y() > 0, "invalid finite wall");
    normal_ = wall_.Rot().RotateVector(V3::UnitZ);
    for (const auto & name : {"left_wheel_link", "right_wheel_link", "caster_ball_link"}) {
      auto id = model_.LinkByName(ecm, name);
      require(id != gz::sim::kNullEntity, "missing support wheel");
      wheelLinks_.push_back(gz::sim::Link(id));
      wheelLinks_.back().EnableVelocityChecks(ecm);
      auto collision = wheelLinks_.back().Collisions(ecm).at(0);
      wheelCollisions_.push_back(collision);
      if (!ecm.Component<gz::sim::components::ContactSensorData>(collision)) {
        ecm.CreateComponent(collision, gz::sim::components::ContactSensorData());
      }
    }
    for (auto id : link_.Collisions(ecm)) {
      auto name = ecm.Component<gz::sim::components::Name>(id);
      if (name && name->Data().rfind("skirt_stop_", 0) == 0) {
        stopCollisions_.push_back(id);
        if (!ecm.Component<gz::sim::components::ContactSensorData>(id)) {
          ecm.CreateComponent(id, gz::sim::components::ContactSensorData());
        }
      }
    }
    require(stopCollisions_.size() == 4, "missing unilateral skirt stop collisions");
    link_.EnableVelocityChecks(ecm);
    if (!ecm.Component<gz::sim::components::WorldPose>(link_.Entity())) {
      ecm.CreateComponent(link_.Entity(), gz::sim::components::WorldPose(
        gz::sim::worldPose(link_.Entity(), ecm)));
    }
    ecm.CreateComponent(entity_, v::State(std::vector<double>(v::Count, 0)));
    Clear(ecm);
  }

  void Reset(const gz::sim::UpdateInfo &, ECM & ecm) override
  {
    ++generation_;Clear(ecm);
    link_.EnableVelocityChecks(ecm);
    for (auto & wheel : wheelLinks_) {wheel.EnableVelocityChecks(ecm);}
    if (!ecm.Component<gz::sim::components::WorldPose>(link_.Entity())) {
      ecm.CreateComponent(link_.Entity(), gz::sim::components::WorldPose(
        gz::sim::worldPose(link_.Entity(), ecm)));
    }
  }

  void PreUpdate(const gz::sim::UpdateInfo & info, ECM & ecm) override
  {
    if (info.paused || info.dt.count() <= 0) {return;}
    auto start = std::chrono::steady_clock::now();
    double time = seconds(info.simTime), dt = seconds(info.dt);
    if (runtimeFailed_) {
      PublishFailure(time, dt, ecm);return;
    }
    try {
      auto targets = io_->apply(time);
      auto pose = link_.WorldPose(ecm);auto linear = link_.WorldLinearVelocity(ecm);
      auto angular = link_.WorldAngularVelocity(ecm);
      require(pose && linear && angular, "missing physical pose/velocity state");
      pose_ = *pose;velocity_ = *linear;omega_ = *angular;
      V3 bodyNormal = pose_.Rot().RotateVector(V3::UnitZ);
      double cosine = bodyNormal.Dot(normal_);
      require(cosine > .5, "pressure surface outside supported orientation domain");
      double originHeight = (pose_.Pos() - wall_.Pos()).Dot(normal_);
      projection_ = pose_.Pos() - normal_ * originHeight;
      auto innerPoint = pose_.CoordPositionAdd(V3(cx_, 0, gapNominal_));
      V3 innerWall = wall_.Rot().RotateVectorReverse(innerPoint - projection_);
      parameters_.centre_x_m = innerWall.X();parameters_.centre_y_m = innerWall.Y();
      parameters_.pressure_z_m = innerWall.Z();
      samples_.clear();double total = 0, mean = 0;bool inside = true;
      double minGap = 1e6, maxGap = -1e6;
      for (std::size_t i = 0; i < attachments_.size(); ++i) {
        V3 p = pose_.CoordPositionAdd(attachments_[i]);
        V3 wallPoint = wall_.Rot().RotateVectorReverse(p - wall_.Pos());
        V3 local = wall_.Rot().RotateVectorReverse(p - projection_);
        V3 speed = wall_.Rot().RotateVectorReverse(velocity_ + omega_.Cross(p - pose_.Pos()));
        double present = coverage(wallPoint.X(), size_.X(), parameters_.band_m) *
          coverage(wallPoint.Y(), size_.Y(), parameters_.band_m);
        inside = inside && present >= 1 - 1e-12;
        double gap = local.Z(), gapRate = speed.Z();
        if (surface_) {
          gap -= surface_->height(wallPoint.X(), wallPoint.Y());
          auto n = surface_->normal(wallPoint.X(), wallPoint.Y());
          gapRate += (n[0] * speed.X() + n[1] * speed.Y()) / n[2];
        }
        require(gap >= 0, "skirt attachment penetrated wall reference plane");
        minGap = std::min(minGap, gap);maxGap = std::max(maxGap, gap);
        samples_.push_back({local.X(), local.Y(), lengths_[i], gap, gapRate, speed.X(), speed.Y(),
            gap * (1 - present), present < 1, present});
        mean += lengths_[i] * gap;total += lengths_[i];
      }
      mean /= total;double volume = volumeNominal_ + inner_ * (mean - gapNominal_);
      require(volume > 0, "nonpositive physical cavity volume");
      auto shape = v::skirt(parameters_, samples_, 0);
      double area = baseLeak_ + shape.leak_area_m2 + io_->openingArea();
      int substeps = static_cast<int>(std::ceil(dt / substep_));
      double h = dt / substeps;
      if (ideal_) {
        speeds_ = core_->speedsAfter(speeds_, targets, dt);
        pressure_ = core_->gas().ambient_pa - fixed_;
        balance_ = core_->balance(pressure_, speeds_, area);
      } else {
        for (int i = 0; i < substeps; ++i) {
          auto mid = core_->speedsAfter(speeds_, targets, h / 2);
          speeds_ = core_->speedsAfter(speeds_, targets, h);
          double v0 = previousVolume_ + (volume - previousVolume_) * i / substeps;
          double v1 = previousVolume_ + (volume - previousVolume_) * (i + 1) / substeps;
          double a = previousArea_ + (area - previousArea_) * (i + .5) / substeps;
          balance_ = core_->implicitStep(pressure_, v0, v1, h, mid, a);
          pressure_ = balance_.pressure_pa;
        }
        balance_ = core_->balance(pressure_, speeds_, area);
      }
      previousVolume_ = volume;previousArea_ = area;
      auto contact = v::skirt(parameters_, samples_, balance_.delta_pa * cosine);
      auto applied = Forces(contact, balance_.delta_pa, bodyNormal);
      auto centre = PressureCentre(contact, bodyNormal);
      Applied fixture;
      double readyPressure = state_[v::ReadyPressure];
      if (fixture_ && !benchHold_) {
        // Threshold is recomputed from actual geometry / gravity at rest.
        readyPressure = readyMargin_ * MinimumPressure(bodyNormal);
        bool loaded = ContactsEstablished(ecm);
        bool ready = !startupFailed_ && balance_.delta_pa >= readyPressure && inside && loaded &&
          velocity_.Length() <= velocityTolerance_ && omega_.Length() <= angularTolerance_;
        readyDuration_ = ready ? readyDuration_ + dt : 0;
        if (readyDuration_ >= dwell_) {
          fixture_ = false;release_ = time;
        } else if (time >= timeout_) {
          startupFailed_ = true;
        }
      }
      // A dissipative rolling couple on the freely rotating sphere. Its
      // normal load is native feedback from the previous physics solve.
      // No force or static holding credit is added to the chassis.
      V3 rolling{};
      auto ballOmega = wheelLinks_[2].WorldAngularVelocity(ecm);
      auto ballData = ecm.Component<gz::sim::components::ContactSensorData>(wheelCollisions_[2]);
      require(ballOmega && ballData, "ball rolling feedback missing");
      double ballNormal = 0;
      for (const auto & c : ballData->Data().contact()) {
        bool first = c.collision1().id() == wheelCollisions_[2];
        require(first || c.collision2().id() == wheelCollisions_[2],
            "ball contact ownership mismatch");
        for (const auto & w : c.wrench()) {
          auto f = first ? w.body_1_wrench().force() : w.body_2_wrench().force();
          ballNormal += V3(f.x(), f.y(), f.z()).Dot(normal_);
        }
      }
      V3 spin = *ballOmega - normal_ * ballOmega->Dot(normal_);
      rolling = -rollingCoefficient_ *std::max(0.0, ballNormal) * ballRadius_ * spin /
        std::sqrt(spin.SquaredLength() + std::pow(rollingEpsilon_ / ballRadius_, 2));
      if (fixture_) {
        V3 gravitySupport = -mass_ * gravity_;
        V3 preload = -preload_ * normal_;
        V3 cgWorld = pose_.Rot().RotateVector(cg_);
        V3 pressureOffset = centre - pose_.Pos();
        fixture = {gravitySupport + preload,
          cgWorld.Cross(gravitySupport) + pressureOffset.Cross(preload)};
      }
      // Validate the complete step before any force mutation. Apply pressure
      // exactly once, with the SDK's link-relative offset.
      V3 offset = pose_.Rot().RotateVectorReverse(centre - pose_.Pos());
      link_.AddWorldWrench(ecm, applied.first.force, V3::Zero, offset);
      link_.AddWorldWrench(ecm, applied.second.force, applied.second.moment);
      wheelLinks_[2].AddWorldWrench(ecm, V3::Zero, rolling);
      if (fixture_) {link_.AddWorldWrench(ecm, fixture.force, fixture.moment);}
      state_[v::Time] = time;state_[v::CavityPressure] = pressure_;
      state_[v::AmbientPressure] = core_->gas().ambient_pa;
      state_[v::DeltaPressure] = balance_.delta_pa;
      state_[v::EffectiveArea] = contact.effective_area_m2;state_[v::Volume] = volume;
      state_[v::LipArea] = contact.weighted_lip_area_m2;state_[v::SkirtNormal] = contact.normal_n;
      state_[v::SkirtMechanical] = contact.mechanical_n;state_[v::LeakArea] = area;
      state_[v::FlowResidual] = balance_.residual_kg_s;state_[v::FixtureActive] = fixture_;
      state_[v::LeakMassFlow] = balance_.leakage_kg_s;state_[v::PumpMassFlow] = balance_.pump_kg_s;
      state_[v::ReverseMassFlow] = balance_.reverse_kg_s;state_[v::MaximumFanHead] = 0;
      for (int i = 0; i < 3; ++i) {
        state_[v::MaximumFanHead] = std::max(state_[v::MaximumFanHead],
          fanHeads_[i] * speeds_[i] * speeds_[i]);
      }
      state_[v::ReleaseTime] = release_;state_[v::ReadyDuration] = readyDuration_;
      state_[v::StartupFailed] = startupFailed_;state_[v::RuntimeFailed] = runtimeFailed_;
      state_[v::IdealSource] = ideal_;state_[v::Generation] = generation_;
      state_[v::ControlledOpeningArea] = io_->openingArea();
      state_[v::FixtureBenchHold] = benchHold_;
      state_[v::Substeps] = substeps;
      state_[v::MinimumGap] = minGap;state_[v::MaximumGap] = maxGap;
      state_[v::LinearSpeed] = velocity_.Length();state_[v::ReadyPressure] = readyPressure;
      for (int i = 0; i < 3; ++i) {state_[v::Speed0 + i] = speeds_[i];}
      Store(v::PressureFx, applied.first);Store(v::SkirtFx, applied.second);
      Store(v::FixtureFx, fixture);
      for (int i = 0; i < 3; ++i) {state_[v::RollingMx + i] = rolling[i];}
      state_[v::PressureCentreX] = centre.X();state_[v::PressureCentreY] = centre.Y();
      state_[v::PressureCentreZ] = centre.Z();
      double stopNormal = 0;
      for (auto id : stopCollisions_) {
        auto data = ecm.Component<gz::sim::components::ContactSensorData>(id);
        if (!data) {continue;}
        for (const auto & c : data->Data().contact()) {
          bool first = c.collision1().id() == id;
          for (const auto & wrench : c.wrench()) {
            auto f = first ? wrench.body_1_wrench().force() : wrench.body_2_wrench().force();
            stopNormal += V3(f.x(), f.y(), f.z()).Dot(normal_);
          }
        }
      }
      // Native reaction from the previous solve; never apply it a second time.
      state_[v::SkirtStopNormal] = std::max(0.0, stopNormal);
      state_[v::LastValidTime] = time;
      state_[v::CpuNanoseconds] = std::chrono::duration<double, std::nano>(
        std::chrono::steady_clock::now() - start).count();
      for (int i = 0; i < 3; ++i) {
        state_[v::ForceInputVx + i] = velocity_[i];state_[v::ForceInputWx + i] = omega_[i];
      }
      io_->publish(time, state_);
      ecm.SetComponentData<v::State>(entity_, state_);
    } catch (const std::exception & error) {
      runtimeFailed_ = true;
      PublishFailure(time, 0, ecm);
      gzerr << "VacuumSystem failed without force fallback: " << error.what() << std::endl;
    }
  }

private:
  void PublishFailure(double time, double dt, ECM & ecm)
  {
    // Outside the seal model's domain there is no pressure/contact solution
    // to extrapolate. Latch invalidity without a substitute wrench, and keep
    // diagnostic/fan-command state fresh until world reset.
    auto targets = io_->apply(time);
    if (dt > 0) {speeds_ = core_->speedsAfter(speeds_, targets, dt);}
    fixture_ = false;
    state_[v::Time] = time;state_[v::RuntimeFailed] = 1;state_[v::FixtureActive] = 0;
    Store(v::PressureFx, {});Store(v::SkirtFx, {});Store(v::FixtureFx, {});
    for (int i = 0; i < 3; ++i) {
      state_[v::RollingMx + i] = 0;state_[v::Speed0 + i] = speeds_[i];
    }
    io_->publish(time, state_);
    ecm.SetComponentData<v::State>(entity_, state_);
  }
  V3 PressureCentre(const v::SkirtResult & s, const V3 & bodyNormal) const
  {
    V3 n = wall_.Rot().RotateVectorReverse(bodyNormal);
    double z = parameters_.pressure_z_m -
      (n.X() * (s.pressure_centre.x - parameters_.centre_x_m) +
      n.Y() * (s.pressure_centre.y - parameters_.centre_y_m)) / n.Z();
    return projection_ + wall_.Rot().RotateVector({s.pressure_centre.x, s.pressure_centre.y, z});
  }
  std::pair<Applied, Applied> Forces(const v::SkirtResult & s, double dp, const V3 & n) const
  {
    V3 f = -dp * s.effective_area_m2 * n;
    V3 centre = PressureCentre(s, n);
    Applied pressure{f, (centre - pose_.Pos()).Cross(f)};
    V3 cf = wall_.Rot().RotateVector(vector(s.contact.force));
    V3 cm = wall_.Rot().RotateVector(vector(s.contact.moment)) +
      (projection_ - pose_.Pos()).Cross(cf);
    return {pressure, {cf, cm}};
  }
  double MinimumPressure(const V3 & bodyNormal)
  {
    auto rest = samples_;
    for (auto & sample : rest) {
      sample.velocity_x_mps = 0;sample.velocity_y_mps = 0;sample.gap_rate_mps = 0;
    }
    std::array<V3, 3> cp;
    for (int i = 0; i < 3; ++i) {
      V3 body = i == 2 ? V3(caster_, 0, 0) : V3(0, (i == 0 ? 1 : -1) * track_ / 2, 0);
      cp[i] = wall_.Rot().RotateVectorReverse(pose_.Rot().RotateVector(body));
    }
    gz::math::Matrix3d matrix(1, 1, 1, cp[0].Y(), cp[1].Y(), cp[2].Y(), -cp[0].X(), -cp[1].X(),
      -cp[2].X());
    require(std::abs(matrix.Determinant()) > 1e-6, "degenerate three-wheel support projection");
    auto inverse = matrix.Inverse();
    V3 g = mass_ * gravity_;
    V3 gm = (pose_.CoordPositionAdd(cg_) - projection_).Cross(g);
    auto valid = [&](double dp) {
        auto skirt = v::skirt(parameters_, rest, dp * bodyNormal.Dot(normal_));
        auto forces = Forces(skirt, dp, bodyNormal);
        V3 force = g + forces.first.force + forces.second.force;
        V3 moment = gm + forces.first.moment + forces.second.moment +
          (pose_.Pos() - projection_).Cross(forces.first.force + forces.second.force);
        force = wall_.Rot().RotateVectorReverse(force);
        moment = wall_.Rot().RotateVectorReverse(moment);
        V3 normal = inverse * V3(-force.Z(), -moment.X(), -moment.Y());
        if (normal.X() < driveFloor_ || normal.Y() < driveFloor_ || normal.Z() < ballFloor_) {
          return false;
        }
        // Wheel rolling direction follows the projected chassis +X, not a
        // hard-coded world axis. Re-express the yaw/tangent equations there.
        V3 drive = wall_.Rot().RotateVectorReverse(pose_.Rot().RotateVector(V3::UnitX));
        drive.Z(0);drive.Normalize();V3 lateral(-drive.Y(), drive.X(), 0);
        double tx = -force.Dot(drive), ty = -force.Dot(lateral);
        double yl = cp[0].Dot(lateral), yr = cp[1].Dot(lateral);
        double xl = cp[0].Dot(drive), xr = cp[1].Dot(drive);
        // Native rigid-wheel data show an almost equal lateral split. Until
        // motion validation closes the allocation model, require both equal
        // and normal-proportional hypotheses to satisfy the friction circles.
        for (double fraction : {0.5, normal.X() / (normal.X() + normal.Y())}) {
          double latl = ty * fraction, latr = ty - latl;
          double tl = (-moment.Z() - xl * latl - xr * latr + yr * tx) / (yr - yl), tr = tx - tl;
          if (driveMu_ * normal.X() < margin_ * std::hypot(tl, latl) ||
            driveMu_ * normal.Y() < margin_ * std::hypot(tr, latr)) {return false;}
        }
        return true;
      };
    double lo = 0, hi = 30000;
    require(valid(hi), "no startup traction/support solution below 30 kPa");
    for (int i = 0; i < 30; ++i) {
      double mid = (lo + hi) / 2;if (valid(mid)) {
        hi = mid;
      } else {
        lo = mid;
      }
    }
    return hi;
  }
  bool ContactsEstablished(const ECM & ecm) const
  {
    for (int i = 0; i < 3; ++i) {
      auto data = ecm.Component<gz::sim::components::ContactSensorData>(wheelCollisions_[i]);
      if (!data || data->Data().contact_size() == 0) {return false;}
      double load = 0;
      for (const auto & c : data->Data().contact()) {
        bool first = c.collision1().id() == wheelCollisions_[i];
        if (!first && c.collision2().id() != wheelCollisions_[i]) {return false;}
        for (const auto & w : c.wrench()) {
          auto f = first ? w.body_1_wrench().force() : w.body_2_wrench().force();
          load += V3(f.x(), f.y(), f.z()).Dot(normal_);
        }
      }
      if (load < (i == 2 ? ballFloor_ : driveFloor_)) {return false;}
    }
    return true;
  }
  void Store(std::size_t field, const Applied & w)
  {
    for (int i = 0; i < 3; ++i) {
      state_[field + i] = w.force[i];state_[field + 3 + i] = w.moment[i];
    }
  }
  void Clear(ECM & ecm)
  {
    io_->reset();
    fixture_ = true;startupFailed_ = false;runtimeFailed_ = false;readyDuration_ = 0;release_ = -1;
    pressure_ = core_->gas().ambient_pa;speeds_ = {0, 0, 0};
    previousVolume_ = volumeNominal_;previousArea_ = baseLeak_ + io_->openingArea();
    state_.assign(v::Count, 0);state_[v::CavityPressure] = pressure_;
    state_[v::AmbientPressure] = pressure_;state_[v::FixtureActive] = 1;state_[v::ReleaseTime] = -1;
    state_[v::Generation] = generation_;state_[v::IdealSource] = ideal_;
    state_[v::FixtureBenchHold] = benchHold_;
    state_[v::ControlledOpeningArea] = io_->openingArea();
    for (int i = 0; i < 3; ++i) {state_[v::Enabled0 + i] = 1;}
    state_[v::SensorStamp] = -1;state_[v::SensorDelivery] = -1;
    ecm.SetComponentData<v::State>(entity_, state_);
  }
  gz::sim::Entity entity_{};
  gz::sim::Model model_;
  gz::sim::Link link_;
  std::vector<gz::sim::Link> wheelLinks_;
  std::vector<gz::sim::Entity> wheelCollisions_;
  std::vector<double> fanHeads_;
  std::unique_ptr<v::Chamber> core_;
  std::unique_ptr<WallSurface> surface_;
  std::unique_ptr<v::VacuumIO> io_;
  v::SkirtParameters parameters_;
  v::Balance balance_;
  Pose wall_, pose_;
  gz::math::Vector2d size_;
  V3 normal_, gravity_, cg_, projection_, velocity_, omega_;
  std::vector<V3> attachments_;
  std::vector<double> lengths_, state_, speeds_;
  std::vector<v::SkirtSample> samples_;
  std::vector<gz::sim::Entity> stopCollisions_;
  double mass_{}, track_{}, caster_{}, driveMu_{}, margin_{}, driveFloor_{}, ballFloor_{};
  double inner_{}, volumeNominal_{}, cx_{}, gapNominal_{}, fixed_{}, substep_{}, preload_{};
  double readyMargin_{}, dwell_{}, timeout_{}, velocityTolerance_{}, angularTolerance_{},
    baseLeak_{};
  double pressure_{}, previousVolume_{}, previousArea_{}, readyDuration_{}, release_{-1};
  double rollingCoefficient_{}, rollingEpsilon_{}, ballRadius_{};
  unsigned generation_{};
  bool ideal_{}, fixture_{true}, startupFailed_{}, runtimeFailed_{}, benchHold_{};
};
}  // namespace climbot_gazebo
GZ_ADD_PLUGIN(climbot_gazebo::VacuumSystem, gz::sim::System,
  gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate, gz::sim::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(climbot_gazebo::VacuumSystem, "climbot_gazebo::VacuumSystem")
