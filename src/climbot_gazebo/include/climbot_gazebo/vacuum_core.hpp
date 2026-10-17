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

#ifndef CLIMBOT_GAZEBO__VACUUM_CORE_HPP_
#define CLIMBOT_GAZEBO__VACUUM_CORE_HPP_

#include <array>
#include <vector>

namespace climbot_gazebo::vacuum
{
struct Vec3 {double x{}, y{}, z{};};
struct Wrench {Vec3 force, moment;};
struct Gas
{
  double ambient_pa{101325.0};
  double temperature_k{293.15};
  double specific_r{287.05};
  double gamma{1.4};
  double discharge_coefficient{0.65};
  double regularization_pa{0.1};
};
struct Fan
{
  // Free-flow volume is measured at cavity inlet density, at unit speed.
  double free_flow_m3_s{0.006};
  double shutoff_pa{9000.0};
  double time_constant_s{0.15};
  double port_area_m2{0.0025};
  // Also used above a running fan's available head. Zero is an explicit
  // ideal check-valve diagnostic, never the assumed stopped hardware.
  double reverse_fraction{0.05};
};
struct Balance
{
  double pressure_pa{}, delta_pa{}, leakage_kg_s{}, pump_kg_s{}, reverse_kg_s{};
  double residual_kg_s{}, pressure_slope_kg_s_pa{};
  bool sealed_retained{};
};
class Chamber
{
public:
  Chamber(Gas gas, std::vector<Fan> fans);
  double orifice(double upstream_pa, double downstream_pa, double area_m2) const;
  Balance balance(double cavity_pa, const std::vector<double> & speeds, double area_m2) const;
  Balance equilibrium(
    const std::vector<double> & speeds, double area_m2,
    double previous_pa, double previous_volume_m3, double volume_m3) const;
  Balance implicitStep(
    double previous_pa, double previous_volume_m3,
    double volume_m3, double dt, const std::vector<double> & speeds, double area_m2) const;
  std::vector<double> speedsAfter(
    const std::vector<double> & speeds,
    const std::vector<double> & targets, double dt) const;
  const Gas & gas() const {return gas_;}

private:
  void checkSpeeds(const std::vector<double> & speeds) const;
  Gas gas_;
  std::vector<Fan> fans_;
};
struct SkirtParameters
{
  double inner_area_m2{}, centre_x_m{}, centre_y_m{}, pressure_z_m{0.018};
  double band_m{0.020}, lip_weight{0.5}, stiffness_n_m{5000.0};
  double damping_ns_m{40.0}, free_height_m{0.022}, maximum_compression_m{0.010};
  double transition_m{0.001}, sliding_mu{0.1}, velocity_regularization_mps{0.001};
};
struct SkirtSample
{
  double x_m{}, y_m{}, length_m{}, gap_m{0.018}, gap_rate_mps{};
  double velocity_x_mps{}, velocity_y_mps{};
  // A declared through-channel is distinct from a local gap. An isolated pit
  // has connected=false and adds no through-flow merely by being deep.
  double channel_height_m{};
  bool connected{};
  // S3 finite-wall contact coverage, from the same surface as collisions.
  double support_fraction{1.0};
};
struct SkirtResult
{
  double effective_area_m2{}, weighted_lip_area_m2{}, leak_area_m2{};
  double mechanical_n{}, pressure_lip_n{}, normal_n{}, drag_power_w{};
  Vec3 pressure_centre;
  Wrench pressure, contact;
};
SkirtResult skirt(
  const SkirtParameters & parameters,
  const std::vector<SkirtSample> & samples, double delta_pa);
}  // namespace climbot_gazebo::vacuum
#endif  // CLIMBOT_GAZEBO__VACUUM_CORE_HPP_
