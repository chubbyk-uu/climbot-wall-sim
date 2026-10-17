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

// Offline acceptance adapter. The physics library itself has no YAML/ROS/Gazebo dependency.
#include "climbot_gazebo/vacuum_core.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace v = climbot_gazebo::vacuum;
namespace
{
void quoted(std::ostream & out, const std::string & value)
{
  out << '"';
  for (unsigned char ch : value) {
    if (ch == '"' || ch == '\\') {out << '\\' << ch;} else if (ch < 32) {
      out << "\\u00" << "0123456789abcdef"[ch / 16] << "0123456789abcdef"[ch % 16];
    } else {out << ch;}
  }
  out << '"';
}
void json(std::ostream & out, const YAML::Node & node, bool string = false)
{
  if (node.IsMap()) {
    out << '{';bool first = true;
    for (const auto & pair : node) {
      if (!first) {out << ',';} first = false;
      auto key = pair.first.as<std::string>();quoted(out, key);out << ':';
      json(out, pair.second, key == "id" || key == "status" || key == "error");
    }
    out << '}';
  } else if (node.IsSequence()) {
    out << '[';bool first = true;
    for (const auto & value : node) {
      if (!first) {
        out << ',';
      }
      first = false;json(out, value);
    }
    out << ']';
  } else if (node.IsNull()) {out << "null";} else {
    auto value = node.Scalar();char * end = nullptr;
    double number = std::strtod(value.c_str(), &end);
    if (!string && (value == "true" || value == "false")) {
      out << value;
    } else if (!string && end != value.c_str() && *end == '\0' && std::isfinite(number)) {
      out << value;
    } else {quoted(out, value);}
  }
}
double number(const YAML::Node & n, const char * key, double fallback)
{return n[key] ? n[key].as<double>() : fallback;}
YAML::Node vector(v::Vec3 a)
{YAML::Node n; n.push_back(a.x);n.push_back(a.y);n.push_back(a.z);return n;}
YAML::Node output(const v::Balance & r)
{
  YAML::Node n;
  n["pressure_pa"] = r.pressure_pa;n["delta_pa"] = r.delta_pa;
  n["leakage_kg_s"] = r.leakage_kg_s;n["pump_kg_s"] = r.pump_kg_s;
  n["reverse_kg_s"] = r.reverse_kg_s;n["residual_kg_s"] = r.residual_kg_s;
  n["pressure_slope_kg_s_pa"] = r.pressure_slope_kg_s_pa;
  n["sealed_retained"] = r.sealed_retained;
  return n;
}
std::vector<double> speeds(const YAML::Node & n, const char * key)
{return n[key].as<std::vector<double>>();}
std::vector<v::SkirtSample> rectangle(const YAML::Node & n, double gap)
{
  double L = n["length_m"].as<double>(), B = n["width_m"].as<double>();
  double cx = n["centre_x_m"].as<double>(), w = n["band_m"].as<double>();
  int count = n["samples_per_side"].as<int>();
  if (L <= 2 * w || B <= 2 * w || count < 1) {throw std::invalid_argument("invalid rectangle");}
  double x0 = cx - L / 2 + w / 2, x1 = cx + L / 2 - w / 2, y0 = -B / 2 + w / 2, y1 = B / 2 - w / 2;
  std::array<v::Vec3, 5> corners{{{x0, y0, 0}, {x1, y0, 0}, {x1, y1, 0}, {x0, y1, 0}, {x0, y0, 0}}};
  std::vector<v::SkirtSample> out;
  for (int edge = 0; edge < 4; ++edge) {
    auto a = corners[edge], b = corners[edge + 1];double dx = b.x - a.x, dy = b.y - a.y;
    for (int i = 0; i < count; ++i) {
      double x = a.x + dx * (i + .5) / count, y = a.y + dy * (i + .5) / count;
      // A smooth prescribed tilt/velocity field is an isolation probe, not M2 terrain.
      double local_gap = gap + number(n, "tilt_rad", 0) * (x - cx);
      double vx = number(n, "v_mps", 0) - number(n, "omega_rps", 0) * y;
      double vy = number(n, "omega_rps", 0) * x;
      out.push_back({x, y, std::hypot(dx, dy) / count, local_gap, number(n, "gap_rate_mps", 0), vx,
          vy,
          number(n, "channel_height_m", 0), n["connected"] ? n["connected"].as<bool>() : false});
    }
  }
  return out;
}
v::SkirtParameters skirtParameters(const YAML::Node & n)
{
  v::SkirtParameters p;
  double L = n["length_m"].as<double>(), B = n["width_m"].as<double>();
  p.band_m = n["band_m"].as<double>();p.inner_area_m2 = (L - 2 * p.band_m) * (B - 2 * p.band_m);
  p.centre_x_m = n["centre_x_m"].as<double>();
  p.pressure_z_m = number(n, "pressure_z_m", p.pressure_z_m);
  p.lip_weight = number(n, "lip_weight", p.lip_weight);
  p.stiffness_n_m = number(n, "stiffness_n_m", p.stiffness_n_m);
  p.damping_ns_m = number(n, "damping_ns_m", p.damping_ns_m);
  p.free_height_m = number(n, "free_height_m", p.free_height_m);
  p.maximum_compression_m = number(n, "maximum_compression_m", p.maximum_compression_m);
  p.transition_m = number(n, "transition_m", p.transition_m);
  p.sliding_mu = number(n, "sliding_mu", p.sliding_mu);
  p.velocity_regularization_mps = number(n, "velocity_regularization_mps",
      p.velocity_regularization_mps);
  return p;
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    if (argc != 3) {
      throw std::invalid_argument("usage: vacuum_core_probe request.yaml output.json");
    }
    auto request = YAML::LoadFile(argv[1]);auto config = request["physics"];
    v::Gas gas;auto gn = config["gas"];
    gas.ambient_pa = number(gn, "ambient_pa", gas.ambient_pa);
    gas.temperature_k = number(gn, "temperature_k", gas.temperature_k);
    gas.specific_r = number(gn, "specific_r", gas.specific_r);
    gas.gamma = number(gn, "gamma", gas.gamma);
    gas.regularization_pa = number(gn, "regularization_pa", gas.regularization_pa);
    gas.discharge_coefficient = number(gn, "discharge_coefficient", gas.discharge_coefficient);
    std::vector<v::Fan> fans;
    for (const auto & n : config["fans"]) {
      v::Fan f;f.free_flow_m3_s = n["free_flow_m3_s"].as<double>();
      f.shutoff_pa = n["shutoff_pa"].as<double>();f.time_constant_s = n["time_constant_s"].as<double>();
      f.port_area_m2 = n["port_area_m2"].as<double>();f.reverse_fraction = n["reverse_fraction"].as<double>();
      fans.push_back(f);
    }
    v::Chamber core(gas, fans);YAML::Node results(YAML::NodeType::Sequence);
    for (const auto & c : request["cases"]) {
      YAML::Node row;row["id"] = c["id"].as<std::string>();
      try {
        std::string kind = c["kind"].as<std::string>();
        double volume = number(c, "volume_m3", .003);
        if (kind == "steady") {
          auto r = core.equilibrium(speeds(c, "speeds"), c["area_m2"].as<double>(),
            number(c, "previous_pa", gas.ambient_pa), number(c, "previous_volume_m3", volume),
            volume);
          row["value"] = output(r);
        } else if (kind == "orifice") {
          row["flow_kg_s"] = core.orifice(c["a_pa"].as<double>(), c["b_pa"].as<double>(),
            c["area_m2"].as<double>());
        } else if (kind == "skirt") {
          auto geometry = c["geometry"];auto p = skirtParameters(geometry);
          auto pts = rectangle(geometry, c["gap_m"].as<double>());
          auto preliminary = v::skirt(p, pts, 0);double dp = number(c, "delta_pa", 0);
          if (c["speeds"]) {
            auto r = core.equilibrium(speeds(c, "speeds"),
              c["base_area_m2"].as<double>() + preliminary.leak_area_m2,
              gas.ambient_pa, volume, volume);
            row["balance"] = output(r);dp = r.delta_pa;
          }
          auto r = v::skirt(p, pts, dp);
          row["gap_m"] = c["gap_m"].as<double>();row["delta_pa"] = dp;
          row["area_m2"] = r.effective_area_m2;row["leak_area_m2"] = r.leak_area_m2;
          row["mechanical_n"] = r.mechanical_n;row["lip_n"] = r.pressure_lip_n;
          row["normal_n"] = r.normal_n;row["drag_power_w"] = r.drag_power_w;
          row["pressure_centre_m"] = vector(r.pressure_centre);
          row["pressure_force_n"] = vector(r.pressure.force);
          row["pressure_moment_nm"] = vector(r.pressure.moment);
          row["contact_force_n"] = vector(r.contact.force);
          row["contact_moment_nm"] = vector(r.contact.moment);
        } else if (kind == "skirt_ramp") {
          auto geometry = c["geometry"];auto p = skirtParameters(geometry);
          double dt = c["dt_s"].as<double>(), travel = c["travel_s"].as<double>();
          double start = c["gap_start_m"].as<double>(), end = c["gap_end_m"].as<double>();
          if (!(dt > 0 && travel > 0 && end > start && travel / dt < 100000)) {
            throw std::invalid_argument("invalid skirt ramp");
          }
          std::string mode = c["mode"].as<std::string>();
          if (mode != "ode" && mode != "quasi_static") {
            throw std::invalid_argument("unknown pressure mode");
          }
          double reference_volume = volume;
          double reference_gap = c["reference_gap_m"].as<double>();
          volume = reference_volume + p.inner_area_m2 * (start - reference_gap);
          double pc = number(c, "previous_pa", gas.ambient_pa),
            base = c["base_area_m2"].as<double>();
          auto ns = speeds(c, "speeds");YAML::Node trace(YAML::NodeType::Sequence);
          int steps = static_cast<int>(std::llround(2 * travel / dt));
          for (int i = 0; i <= steps; ++i) {
            double t = i * dt, phase = std::min(t / travel, 2.0);
            double gap = start + (end - start) * (phase <= 1 ? phase : 2 - phase);
            auto geo = YAML::Clone(geometry);
            geo["gap_rate_mps"] = (phase < 1 ? 1 : -1) * (end - start) / travel;
            auto pts = rectangle(geo, gap);auto r = v::skirt(p, pts, 0);
            double area = base + r.leak_area_m2;
            double new_volume = reference_volume + p.inner_area_m2 * (gap - reference_gap);
            auto b = mode == "ode" ? core.implicitStep(pc, volume, new_volume, i == 0 ? 0 : dt, ns,
              area) :
              core.equilibrium(ns, area, pc, volume, new_volume);
            pc = b.pressure_pa;volume = new_volume;r = v::skirt(p, pts, b.delta_pa);
            YAML::Node entry = output(b);entry["time_s"] = t;entry["gap_m"] = gap;
            entry["leak_area_m2"] = r.leak_area_m2;entry["normal_n"] = r.normal_n;
            entry["pressure_force_n"] = -r.pressure.force.z;
            trace.push_back(entry);
          }
          row["trace"] = trace;
        } else if (kind == "transient") {
          double dt = c["dt_s"].as<double>(), duration = c["duration_s"].as<double>();
          if (!(dt > 0 && duration > 0 && duration / dt < 2000000)) {
            throw std::invalid_argument("invalid trace interval");
          }
          std::string mode = c["mode"].as<std::string>();
          if (mode != "ode" && mode != "quasi_static") {
            throw std::invalid_argument("unknown pressure mode");
          }
          auto ns = speeds(c, "initial_speeds");
          double pc = number(c, "previous_pa", gas.ambient_pa);
          double area = c["area_m2"].as<double>();YAML::Node trace(YAML::NodeType::Sequence);
          auto emit = [&](double t, const v::Balance & r) {
              YAML::Node v = output(r);v["time_s"] = t;v["speeds"] = ns;trace.push_back(v);
            };
          emit(0, core.balance(pc, ns, area));
          int steps = static_cast<int>(std::llround(duration / dt));
          if (std::abs(steps * dt - duration) > 1e-9) {
            throw std::invalid_argument("duration must align to step");
          }
          for (int i = 0; i < steps; ++i) {
            double t = i * dt;auto targets = speeds(c, "targets");
            for (const auto & e : c["fan_events"]) {
              if (t + dt * 1e-7 >= e["time_s"].as<double>()) {targets = speeds(e, "targets");}
            }
            for (const auto & e : c["area_events"]) {
              if (t + dt * 1e-7 >= e["time_s"].as<double>()) {area = e["area_m2"].as<double>();}
            }
            double new_volume = volume;
            for (const auto & e : c["volume_events"]) {
              if (t + dt * 1e-7 >= e["time_s"].as<double>()) {
                new_volume = e["volume_m3"].as<double>();
              }
            }
            auto mid = core.speedsAfter(ns, targets, dt / 2);ns = core.speedsAfter(ns, targets, dt);
            auto r = mode == "ode" ? core.implicitStep(pc, volume, new_volume, dt, mid, area) :
              core.equilibrium(ns, area, pc, volume, new_volume);
            pc = r.pressure_pa;volume = new_volume;emit((i + 1) * dt, r);
          }
          row["trace"] = trace;
        } else {throw std::invalid_argument("unknown probe kind");}
        row["status"] = "ok";
      } catch (const std::exception & e) {
        row["status"] = "error";row["error"] = e.what();
      }
      results.push_back(row);
    }
    std::ofstream out(argv[2]);json(out, results);out << '\n';
    if (!out) {throw std::runtime_error("cannot write probe output");}
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';return 2;
  }
}
