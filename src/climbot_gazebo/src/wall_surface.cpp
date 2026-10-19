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
#include "climbot_gazebo/wall_surface.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
namespace climbot_gazebo
{
namespace
{
constexpr double tau = 6.2831853071795864769;
double smooth(double x) {x = std::clamp(x, 0.0, 1.0);return x * x * (3 - 2 * x);}
void require(bool b) {if (!b) {throw std::invalid_argument("invalid resolved wall surface");}}
double value(const YAML::Node & n, const char * key)
{
  double x = n[key].as<double>();require(std::isfinite(x));return x;
}
}
WallSurface::WallSurface(const std::string & text)
{
  const auto config = YAML::Load(text);
  const auto modes = config["roughness"]["modes"];
  require(modes.IsSequence() && modes.size() == 8);
  double total = 0;
  for (const auto & m : modes) {
    double period = value(m, "period_m"), angle = value(m, "angle_rad");
    double phase = value(m, "phase_rad"), weight = value(m, "weight_m");
    require(period > 0 && weight >= 0);total += weight;
    modes_.push_back({tau / period * std::cos(angle), tau / period * std::sin(angle), phase,
        weight});
  }
  require(total <= .010000000001);
  require(config["grooves"].IsSequence());
  for (const auto & n : config["grooves"]) {
    require(n["start_m"].size() == 2 && n["end_m"].size() == 2 &&
      n["depth_range_m"].size() == 2);
    Groove g{};
    g.u = n["start_m"][0].as<double>();g.v = n["start_m"][1].as<double>();
    double du = n["end_m"][0].as<double>() - g.u;
    double dv = n["end_m"][1].as<double>() - g.v;
    g.length = std::hypot(du, dv);require(std::isfinite(g.length) && g.length > 0);
    require(std::isfinite(g.u) && std::isfinite(g.v));
    g.du = du / g.length;g.dv = dv / g.length;
    g.width = value(n, "width_m");g.scale = value(n, "variation_scale_m");
    g.shoulder = value(n, "shoulder_fraction");g.taper = value(n, "end_taper_m");
    g.lo = n["depth_range_m"][0].as<double>();g.hi = n["depth_range_m"][1].as<double>();
    g.radius = n["rim_radius_m"] ? value(n, "rim_radius_m") : 0;
    g.chamfer = n["top_chamfer_m"] ? value(n, "top_chamfer_m") : 0;
    auto section = n["cross_section"] ? n["cross_section"].as<std::string>() : "smooth";
    require(section == "smooth" || section == "trapezoid");g.trapezoid = section == "trapezoid";
    require(g.width > 0 && g.width <= .05 && g.scale > 0 && g.shoulder > 0 &&
      g.shoulder < .5 && g.taper >= 0 && g.lo > 0 && g.lo <= g.hi && g.hi <= .02);
    double span = g.width * g.shoulder;
    require(g.radius >= 0 && g.radius < span / 2 && g.chamfer >= 0 &&
      g.chamfer < std::min(span, g.lo));
    require((g.radius == 0 && g.chamfer == 0) || (g.trapezoid && g.taper == 0));
    require(g.radius == 0 || g.chamfer == 0);grooves_.push_back(g);
  }
}
double WallSurface::roughness(double u, double v) const
{
  double h = 0;
  for (const auto & m : modes_) {
    h += m.weight * std::sin(m.ku * u + m.kv * v + m.phase);
  }
  return h;
}
double WallSurface::grooveDepth(double u, double v) const
{
  double depth = 0;
  for (const auto & g : grooves_) {
    double along = (u - g.u) * g.du + (v - g.v) * g.dv;
    double across = std::abs((u - g.u) * g.dv - (v - g.v) * g.du);
    double envelope = g.taper > 0 ? smooth(along / g.taper) * smooth((g.length - along) / g.taper) :
      static_cast<double>(along >= 0 && along <= g.length);
    if (envelope == 0 || across >= g.width * envelope / 2) {continue;}
    double lateral = (.5 - across / std::max(g.width * envelope, 1e-30)) / g.shoulder;
    double profile = g.trapezoid ? std::clamp(lateral, 0.0, 1.0) : smooth(lateral);
    double local = (g.lo + g.hi) / 2 + (g.hi - g.lo) / 2 * std::sin(tau * along / g.scale);
    double cut = profile * local, span = g.width * g.shoulder, inset = g.width / 2 - across;
    if (g.radius > 0) {
      double r = g.radius, vertical = local - 2 * r;
      double slope = (span * vertical + 2 * r *
        std::sqrt(vertical * vertical + span * span - 4 * r * r)) /
        (span * span - 4 * r * r);
      double cosine = 1 / std::sqrt(1 + slope * slope), tangent = r * slope * cosine;
      if (inset <= tangent) {
        cut = r - std::sqrt(std::max(0.0, r * r - inset * inset));
      } else if (inset >= span - tangent) {
        double offset = std::clamp(span - inset, 0.0, tangent);
        cut = local - r + std::sqrt(std::max(0.0, r * r - offset * offset));
      } else {cut = r * (1 - cosine) + slope * (inset - tangent);}
      if (inset >= span) {cut = local;}
    }
    if (g.chamfer > 0) {
      cut = inset <= g.chamfer ? inset :
        g.chamfer + (local - g.chamfer) * (inset - g.chamfer) / (span - g.chamfer);
      cut = std::min(cut, local);
    }
    depth = std::max(depth, envelope * cut);
  }
  return depth;
}
std::array<double, 3> WallSurface::normal(double u, double v, double epsilon) const
{
  require(std::isfinite(epsilon) && epsilon > 0);
  double x = -(height(u + epsilon, v) - height(u - epsilon, v)) / (2 * epsilon);
  double y = -(height(u, v + epsilon) - height(u, v - epsilon)) / (2 * epsilon);
  double length = std::sqrt(x * x + y * y + 1);return {x / length, y / length, 1 / length};
}
}  // namespace climbot_gazebo
