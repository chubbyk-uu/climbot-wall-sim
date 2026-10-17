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

#ifndef CLIMBOT_GAZEBO__VACUUM_STATE_HPP_
#define CLIMBOT_GAZEBO__VACUUM_STATE_HPP_
#include <array>
#include <vector>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Serialization.hh>

namespace climbot_gazebo::vacuum
{
// PreUpdate physics truth, never a noisy pressure sensor or wheel-load estimate.
// Wrench moments are about the CURRENT chassis_link origin in world axes.
enum Field : std::size_t
{
  Time, CavityPressure, AmbientPressure, DeltaPressure, EffectiveArea, Volume,
  LipArea, SkirtNormal, SkirtMechanical, LeakArea, FlowResidual,
  FixtureActive, ReleaseTime, ReadyDuration, StartupFailed, RuntimeFailed,
  IdealSource, Speed0, Speed1, Speed2,
  PressureFx, PressureFy, PressureFz, PressureMx, PressureMy, PressureMz,
  SkirtFx, SkirtFy, SkirtFz, SkirtMx, SkirtMy, SkirtMz,
  FixtureFx, FixtureFy, FixtureFz, FixtureMx, FixtureMy, FixtureMz,
  MinimumGap, MaximumGap, LinearSpeed, Generation, Substeps, ReadyPressure,
  CpuNanoseconds, PressureCentreX, PressureCentreY, PressureCentreZ,
  RollingMx, RollingMy, RollingMz,
  Enabled0, Enabled1, Enabled2, Failed0, Failed1, Failed2,
  SensorFault, CommandRevision, RejectedCommands, SensorSequence,
  SensorStamp, SensorDelivery, SensorValue, SensorVariance, SensorSaturated,
  Count
};
inline constexpr std::array<const char *, Count> kNames{
  "time_s", "cavity_pa", "ambient_pa", "delta_pa", "area_m2", "volume_m3",
  "lip_area_m2", "skirt_normal_n", "skirt_mechanical_n", "leak_area_m2", "mass_flow_residual_kg_s",
  "fixture_active", "release_time_s", "ready_duration_s", "startup_failed", "runtime_failed",
  "ideal_source", "fan_speed_0", "fan_speed_1", "fan_speed_2",
  "pressure_fx", "pressure_fy", "pressure_fz", "pressure_mx", "pressure_my", "pressure_mz",
  "skirt_fx", "skirt_fy", "skirt_fz", "skirt_mx", "skirt_my", "skirt_mz",
  "fixture_fx", "fixture_fy", "fixture_fz", "fixture_mx", "fixture_my", "fixture_mz",
  "minimum_gap_m", "maximum_gap_m", "linear_speed_mps", "generation", "substeps", "ready_pa",
  "cpu_ns", "pressure_centre_x", "pressure_centre_y", "pressure_centre_z",
  "rolling_mx", "rolling_my", "rolling_mz",
  "fan_enabled_0", "fan_enabled_1", "fan_enabled_2", "fan_failed_0", "fan_failed_1", "fan_failed_2",
  "sensor_fault", "command_revision", "rejected_commands", "sensor_sequence",
  "sensor_stamp_s", "sensor_delivery_s", "sensor_value_pa", "sensor_variance_pa2",
  "sensor_saturated"
};
using State = gz::sim::components::Component<std::vector<double>, class VacuumStateTag,
    gz::sim::serializers::VectorDoubleSerializer>;
GZ_SIM_REGISTER_COMPONENT("climbot.vacuum.State", State)
}  // namespace climbot_gazebo::vacuum
#endif
