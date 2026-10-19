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

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace climbot_gazebo::vacuum
{
namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {throw std::invalid_argument(message);}
}
bool positive(double x) {return std::isfinite(x) && x > 0;}
bool nonnegative(double x) {return std::isfinite(x) && x >= 0;}
void add(Vec3 & a, const Vec3 & b) {a.x += b.x; a.y += b.y; a.z += b.z;}
Vec3 cross(const Vec3 & a, const Vec3 & b)
{
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
void addForce(Wrench & wrench, const Vec3 & point, const Vec3 & force)
{
  add(wrench.force, force); add(wrench.moment, cross(point, force));
}
double smoothOpening(double h, double width)
{
  if (h <= 0) {return 0;}
  if (h < width) {return h * h / (2 * width);}
  return h - width / 2;
}
}  // namespace

Chamber::Chamber(Gas gas, std::vector<Fan> fans)
: gas_(gas), fans_(std::move(fans))
{
  require(positive(gas_.ambient_pa) && positive(gas_.temperature_k) &&
    positive(gas_.specific_r) && std::isfinite(gas_.gamma) && gas_.gamma > 1 &&
    positive(gas_.regularization_pa) && positive(gas_.discharge_coefficient) &&
    gas_.discharge_coefficient <= 1, "invalid gas parameters");
  require(!fans_.empty(), "no fans configured");
  for (const auto & f : fans_) {
    require(positive(f.free_flow_m3_s) && positive(f.shutoff_pa) &&
      f.shutoff_pa < 0.45 * gas_.ambient_pa && positive(f.time_constant_s) &&
      positive(f.port_area_m2) && nonnegative(f.reverse_fraction) && f.reverse_fraction <= 1,
      "invalid fan curve or reverse channel");
  }
}

void Chamber::checkSpeeds(const std::vector<double> & speeds) const
{
  require(speeds.size() == fans_.size(), "fan state size mismatch");
  for (double n : speeds) {
    require(nonnegative(n) && n <= 1, "speed outside [0,1]");
  }
}

double Chamber::orifice(double a, double b, double area) const
{
  require(positive(a) && positive(b) && nonnegative(area), "invalid orifice state");
  double dp = a - b;
  if (area == 0 || dp == 0) {return 0;}
  double sign = dp > 0 ? 1 : -1;
  double high = std::max(a, b), low = std::min(a, b), d = std::abs(dp);
  double rt = gas_.specific_r * gas_.temperature_k;
  double ideal;
  if (d / high < 1e-8) {
    ideal = std::sqrt(2 * high / rt * d);  // stable zero-pressure asymptote
  } else {
    double gamma = gas_.gamma;
    double ratio = std::max(low / high, std::pow(2 / (gamma + 1), gamma / (gamma - 1)));
    double log_r = std::log(ratio);
    double term = std::exp(2 / gamma * log_r) * (-std::expm1((gamma - 1) / gamma * log_r));
    ideal = high * std::sqrt(2 * gamma / (rt * (gamma - 1)) * term);
  }
  double regular = std::sqrt(d) / std::pow(d * d + gas_.regularization_pa * gas_.regularization_pa,
      .25);
  return sign * gas_.discharge_coefficient * area * ideal * regular;
}

Balance Chamber::balance(double pc, const std::vector<double> & speeds, double area) const
{
  checkSpeeds(speeds);
  require(positive(pc) && nonnegative(area), "invalid chamber pressure/area");
  Balance result;
  result.pressure_pa = pc; result.delta_pa = gas_.ambient_pa - pc;
  result.leakage_kg_s = orifice(gas_.ambient_pa, pc, area);
  for (std::size_t i = 0; i < fans_.size(); ++i) {
    const auto & f = fans_[i]; double n = speeds[i], head = n * n * f.shutoff_pa;
    // Affinity-scaled linear forward branch; at negative gauge pressure the
    // free-flow value is the explicitly declared conservative extension.
    if (n > 0 && result.delta_pa < head) {
      double load = result.delta_pa <= 0 ? 0 : result.delta_pa / head;
      double q = n * f.free_flow_m3_s * (1 - load);
      result.pump_kg_s += pc / (gas_.specific_r * gas_.temperature_k) * q;
    }
    if (result.delta_pa > head && f.reverse_fraction > 0) {
      // Fan head subtracts from the driving pressure of the passive channel.
      result.reverse_kg_s += orifice(gas_.ambient_pa - head, pc,
        f.port_area_m2 * f.reverse_fraction);
    }
  }
  result.residual_kg_s = result.leakage_kg_s + result.reverse_kg_s - result.pump_kg_s;
  return result;
}

Balance Chamber::equilibrium(
  const std::vector<double> & speeds, double area,
  double previous, double old_volume, double volume) const
{
  checkSpeeds(speeds);
  require(positive(previous) && positive(old_volume) && positive(volume) && nonnegative(area),
    "invalid quasi-static state");
  bool closed = area == 0;
  for (std::size_t i = 0; i < fans_.size(); ++i) {
    closed = closed && speeds[i] == 0 && fans_[i].reverse_fraction == 0;
  }
  if (closed) {
    auto result = balance(previous * old_volume / volume, speeds, area);
    result.sealed_retained = true;
    return result;
  }
  double lo = gas_.ambient_pa * 1e-10, hi = gas_.ambient_pa;
  auto low = balance(lo, speeds, area), high = balance(hi, speeds, area);
  if (low.residual_kg_s <= 0 || high.residual_kg_s > 0) {
    throw std::runtime_error("no unique stable quasi-static pressure bracket");
  }
  for (int i = 0; i < 64 && hi - lo > 1e-7; ++i) {
    double mid = (lo + hi) / 2;
    if (balance(mid, speeds, area).residual_kg_s > 0) {lo = mid;} else {hi = mid;}
  }
  auto result = balance((lo + hi) / 2, speeds, area);
  double d = std::min(.01, result.pressure_pa / 4);
  result.pressure_slope_kg_s_pa = (balance(result.pressure_pa - d, speeds, area).residual_kg_s -
    balance(result.pressure_pa + d, speeds, area).residual_kg_s) / (2 * d);
  if (!(result.pressure_slope_kg_s_pa > 1e-14)) {
    throw std::runtime_error("non-unique or non-restoring pressure branch");
  }
  return result;
}

Balance Chamber::implicitStep(
  double previous, double old_volume, double volume,
  double dt, const std::vector<double> & speeds, double area) const
{
  checkSpeeds(speeds);
  require(positive(previous) && positive(old_volume) && positive(volume) && nonnegative(dt) &&
    nonnegative(area), "invalid ODE state");
  if (dt == 0) {
    require(old_volume == volume, "paused step cannot change volume");
    return balance(previous, speeds, area);
  }
  double rt = gas_.specific_r * gas_.temperature_k;
  auto residual = [&](double pc) {
      return pc * volume - previous * old_volume - rt * dt * balance(pc, speeds,
        area).residual_kg_s;
    };
  const double full_lo = std::min(previous, gas_.ambient_pa) * 1e-10;
  // The zero-flow upper endpoint can round just below the conserved mass.
  const double full_hi = std::nextafter(
    std::max(gas_.ambient_pa, previous * old_volume / volume),
    std::numeric_limits<double>::infinity());
  const double centre = std::clamp(previous * old_volume / volume, full_lo, full_hi);
  // Steady substeps need a small bracket. Prove its signs, expand as needed,
  // and retain the original physical interval as a bounded fallback.
  double radius = 1e-4;
  double lo = std::max(full_lo, centre - radius);
  double hi = std::min(full_hi, centre + radius);
  double low_residual = residual(lo), high_residual = residual(hi);
  for (int i = 0; (low_residual >= 0 || high_residual < 0) && i < 16; ++i) {
    radius *= 4;
    lo = std::max(full_lo, centre - radius);
    hi = std::min(full_hi, centre + radius);
    low_residual = residual(lo);high_residual = residual(hi);
    if (lo == full_lo && hi == full_hi) {break;}
  }
  if (low_residual >= 0 || high_residual < 0) {
    lo = full_lo;hi = full_hi;
    if (residual(lo) >= 0 || residual(hi) < 0) {
      throw std::runtime_error("implicit positive-pressure bracket failed");
    }
  }
  for (int i = 0; i < 80 && hi - lo > 1e-7; ++i) {
    double mid = (lo + hi) / 2;
    if (residual(mid) < 0) {lo = mid;} else {hi = mid;}
  }
  return balance((lo + hi) / 2, speeds, area);
}

std::vector<double> Chamber::speedsAfter(
  const std::vector<double> & speeds,
  const std::vector<double> & targets, double dt) const
{
  checkSpeeds(speeds); checkSpeeds(targets);
  require(nonnegative(dt), "invalid speed integration interval");
  std::vector<double> out(speeds.size());
  for (std::size_t i = 0; i < speeds.size(); ++i) {
    out[i] = targets[i] + (speeds[i] - targets[i]) * std::exp(-dt / fans_[i].time_constant_s);
  }
  return out;
}

SkirtResult skirt(
  const SkirtParameters & p, const std::vector<SkirtSample> & samples,
  double dp)
{
  require(positive(p.inner_area_m2) && positive(p.band_m) && nonnegative(p.lip_weight) &&
    p.lip_weight <= 1 && positive(p.stiffness_n_m) && nonnegative(p.damping_ns_m) &&
    positive(p.free_height_m) && positive(p.maximum_compression_m) &&
    p.maximum_compression_m < p.free_height_m && positive(p.transition_m) &&
    nonnegative(p.sliding_mu) && positive(p.velocity_regularization_mps) &&
    std::isfinite(dp) && std::isfinite(p.centre_x_m) && std::isfinite(p.centre_y_m) &&
    std::isfinite(p.pressure_z_m), "invalid skirt parameters");
  require(!samples.empty(), "empty skirt quadrature");
  double perimeter = 0;
  for (const auto & s : samples) {
    require(positive(s.length_m) && nonnegative(s.gap_m) && nonnegative(s.channel_height_m) &&
      std::isfinite(s.x_m) && std::isfinite(s.y_m) && std::isfinite(s.gap_rate_mps) &&
      std::isfinite(s.velocity_x_mps) && std::isfinite(s.velocity_y_mps), "invalid skirt sample");
    require(nonnegative(s.support_fraction) && s.support_fraction <= 1,
      "invalid finite-wall support fraction");
    perimeter += s.length_m;
  }
  SkirtResult out;
  double ax = p.inner_area_m2 * p.centre_x_m, ay = p.inner_area_m2 * p.centre_y_m;
  for (const auto & s : samples) {
    double z = s.gap_m - p.free_height_m;
    double t = std::clamp(z / p.transition_m, 0.0, 1.0);
    double contact = 1 - t * t * (3 - 2 * t);
    double lip = p.lip_weight * p.band_m * s.length_m * contact * s.support_fraction;
    out.weighted_lip_area_m2 += lip; ax += lip * s.x_m; ay += lip * s.y_m;
    // Through-channel and local lift share an aperture; max prevents counting
    // the same physical opening twice. Disconnected depressions do not leak.
    double opening = std::max(smoothOpening(z, p.transition_m),
      s.connected ? s.channel_height_m : 0.0);
    out.leak_area_m2 += s.length_m * opening;
    double mech = 0;
    if (z < 0) {
      double damping_weight = std::clamp(-z / p.transition_m, 0.0, 1.0);
      damping_weight = damping_weight * damping_weight * (3 - 2 * damping_weight);
      mech = std::max(0.0, (p.stiffness_n_m * std::min(-z, p.maximum_compression_m) -
          p.damping_ns_m * s.gap_rate_mps * damping_weight) * s.length_m / perimeter);
    }
    mech *= s.support_fraction;
    double lip_n = dp * lip, normal = std::max(0.0, mech + lip_n);
    out.mechanical_n += mech; out.pressure_lip_n += lip_n; out.normal_n += normal;
    double speed = std::sqrt(s.velocity_x_mps * s.velocity_x_mps + s.velocity_y_mps *
        s.velocity_y_mps +
      p.velocity_regularization_mps * p.velocity_regularization_mps);
    Vec3 force{-p.sliding_mu * normal * s.velocity_x_mps / speed,
      -p.sliding_mu * normal * s.velocity_y_mps / speed, normal};
    addForce(out.contact, {s.x_m, s.y_m, 0}, force);
    out.drag_power_w += force.x * s.velocity_x_mps + force.y * s.velocity_y_mps;
  }
  out.effective_area_m2 = p.inner_area_m2 + out.weighted_lip_area_m2;
  out.pressure_centre = {ax / out.effective_area_m2, ay / out.effective_area_m2, p.pressure_z_m};
  addForce(out.pressure, out.pressure_centre, {0, 0, -dp * out.effective_area_m2});
  return out;
}
}  // namespace climbot_gazebo::vacuum
