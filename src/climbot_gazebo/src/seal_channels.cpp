// Copyright 2026 jerry
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "climbot_gazebo/wall_surface.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace climbot_gazebo
{
namespace
{
void require(bool value)
{if (!value) {throw std::invalid_argument("invalid or over-budget seal channel query");}}
double opening(double x, double width)
{return x <= 0 ? 0 : (x < width ? x * x / (2 * width) : x - width / 2);}
double coverage(double x, double size, double width)
{
  double t = std::clamp((std::min(x, size - x) + width / 2) / width, 0.0, 1.0);
  return t * t * (3 - 2 * t);
}
}
SealLeak WallSurface::sealLeak(
  const std::array<std::array<double, 3>, 4> & corners,
  double band, double freeHeight, double drape, double transition,
  const std::array<double, 2> & size, int refinement) const
{
  require(std::isfinite(band) && band > 0 && std::isfinite(freeHeight) && freeHeight > 0 &&
    std::isfinite(drape) && drape >= 0 && drape <= .01 && std::isfinite(transition) &&
    transition > 0 && refinement >= 1 && refinement <= 8 &&
    std::isfinite(size[0]) && size[0] > 0 && std::isfinite(size[1]) && size[1] > 0);
  for (const auto & p : corners) {
    for (double x : p) {
      require(std::isfinite(x));
    }
  }
  constexpr double nodes[]{-.8611363115940526, -.3399810435848563,
    .3399810435848563, .8611363115940526};
  constexpr double weights[]{.3478548451374538, .6521451548625461,
    .6521451548625461, .3478548451374538};
  SealLeak result;
  for (int edge = 0; edge < 4; ++edge) {
    const auto & a = corners[edge];const auto & b = corners[(edge + 1) % 4];
    double length = std::hypot(b[0] - a[0], b[1] - a[1]);
    require(length > 1e-6 && length <= 4);
    double tu = (b[0] - a[0]) / length, tv = (b[1] - a[1]) / length;
    double nu = -tv, nv = tu;
    // Stack storage is bounded: no allocations or file IO in runtime queries.
    std::array<double, 2048> cuts{};std::size_t count = 0;
    auto add = [&](double s) {
        if (s >= 0 && s <= length) {require(count < cuts.size());cuts[count++] = s;}
      };
    int base = static_cast<int>(std::ceil(length / .02)) * refinement;
    for (int i = 0; i <= base; ++i) {
      add(length * i / base);
    }
    for (const auto & g : grooves_) {
      // Exact rim / shoulder crossings prevent narrow grooves falling between
      // coarse samples. Finite-end and band-crossing boundaries are split too.
      double across0 = (a[0] - g.u) * g.dv - (a[1] - g.v) * g.du;
      double acrossRate = tu * g.dv - tv * g.du;
      if (std::abs(acrossRate) > 1e-12) {
        for (double fraction : {-.5, -.5 + g.shoulder, .5 - g.shoulder, .5}) {
          add((g.width * fraction - across0) / acrossRate);
        }
      }
      double along0 = (a[0] - g.u) * g.du + (a[1] - g.v) * g.dv;
      double alongRate = tu * g.du + tv * g.dv;
      double cosine = std::abs(nu * g.du + nv * g.dv);
      if (std::abs(alongRate) > 1e-12 && cosine > 1e-8) {
        double half = band / (2 * cosine);
        for (double end : {0., g.taper, g.length - g.taper, g.length}) {
          add((end - half - along0) / alongRate);
          add((end + half - along0) / alongRate);
        }
      }
    }
    std::sort(cuts.begin(), cuts.begin() + count);
    count = std::unique(cuts.begin(), cuts.begin() + count) - cuts.begin();
    for (std::size_t interval = 1; interval < count; ++interval) {
      // Refinement changes geometry integration only, never a fan parameter.
      for (int sub = 0; sub < refinement; ++sub) {
        double lo = cuts[interval - 1] + (cuts[interval] - cuts[interval - 1]) * sub / refinement;
        double hi = cuts[interval - 1] +
          (cuts[interval] - cuts[interval - 1]) * (sub + 1) / refinement;
        for (int k = 0; k < 4; ++k) {
          ++result.evaluations;
          double s = (lo + hi) / 2 + (hi - lo) / 2 * nodes[k];
          double u = a[0] + s * tu, v = a[1] + s * tv;
          double gap = a[2] + s / length * (b[2] - a[2]) - roughness(u, v);
          require(gap >= 0);
          double lift = opening(gap - freeHeight, transition), channel = 0;
          double follow = std::min(drape, std::max(0.0, freeHeight - gap));
          for (const auto & g : grooves_) {
            double cosine = std::abs(nu * g.du + nv * g.dv);
            if (cosine <= 1e-8) {continue;}
            double along = (u - g.u) * g.du + (v - g.v) * g.dv;
            double across = std::abs((u - g.u) * g.dv - (v - g.v) * g.du);
            double half = band / (2 * cosine);
            if (across >= g.width / 2 || along < half || along > g.length - half) {continue;}
            double bottleneck = 1;
            for (int j = -2; j <= 2; ++j) {
              double offset = half * j / 2;
              bottleneck = std::min(bottleneck, grooveDepth(u + offset * g.du, v + offset * g.dv));
            }
            // A connected groove contributes its residual bottleneck section.
            // Projection cancels the increased crossing length at oblique angles.
            channel = std::max(channel, opening(bottleneck - follow, transition) * cosine);
          }
          double present = coverage(u, size[0], band) * coverage(v, size[1], band);
          double boundary = gap * (1 - present);
          double baseline = std::max(lift, boundary);
          double total = std::max(lift + present * channel, boundary);
          double weight = weights[k] * (hi - lo) / 2;
          result.area_m2 += weight * total;
          result.channel_area_m2 += weight * (total - baseline);
        }
      }
    }
  }
  return result;
}
}  // namespace climbot_gazebo
