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
#include <limits>
#include <stdexcept>
#include <utility>
namespace climbot_gazebo
{
SphereEnvelope::SphereEnvelope(
  std::vector<double> u, std::vector<double> v, std::vector<double> heights)
: u_(std::move(u)), v_(std::move(v)), heights_(std::move(heights))
{
  for (const auto * axis : {&u_, &v_}) {
    if (axis->size() < 2) {throw std::invalid_argument("envelope axis too short");}
    for (std::size_t i = 0; i < axis->size(); ++i) {
      if (!std::isfinite((*axis)[i]) || (i && (*axis)[i] <= (*axis)[i - 1])) {
        throw std::invalid_argument("envelope axes must be finite and increasing");
      }
    }
  }
  if (heights_.size() != u_.size() * v_.size() ||
    !std::all_of(heights_.begin(), heights_.end(), [](double h) {return std::isfinite(h);}))
  {
    throw std::invalid_argument("invalid envelope source grid");
  }
  maximum_ = *std::max_element(heights_.begin(), heights_.end());
}
double SphereEnvelope::height(double u, double v, double radius) const
{
  if (!std::isfinite(u) || !std::isfinite(v) || !std::isfinite(radius) || radius <= 0 ||
    u - radius < u_.front() - 1e-12 || u + radius > u_.back() + 1e-12 ||
    v - radius < v_.front() - 1e-12 || v + radius > v_.back() + 1e-12)
  {
    throw std::invalid_argument("envelope query footprint outside source grid");
  }
  const double squared = radius * radius;
  double best = -std::numeric_limits<double>::infinity();
  // Seed the lower bound at the nearest grid sample for row/column pruning.
  auto nearest = [](const auto & axis, double value) {
      auto it = std::lower_bound(axis.begin(), axis.end(), value);
      if (it == axis.end()) {return axis.size() - 1;}
      auto i = static_cast<std::size_t>(it - axis.begin());
      return i && value - axis[i - 1] < axis[i] - value ? i - 1 : i;
    };
  auto i = nearest(u_, u), j = nearest(v_, v);
  double distance = std::pow(u_[i] - u, 2) + std::pow(v_[j] - v, 2);
  if (distance < squared) {best = heights_[j * u_.size() + i] + std::sqrt(squared - distance);}
  if (best >= maximum_ + radius - 1e-15) {return best;}
  auto begin = std::lower_bound(u_.begin(), u_.end(), u - radius) - u_.begin();
  auto end = std::upper_bound(u_.begin(), u_.end(), u + radius) - u_.begin();
  auto vb = std::lower_bound(v_.begin(), v_.end(), v - radius) - v_.begin();
  auto ve = std::upper_bound(v_.begin(), v_.end(), v + radius) - v_.begin();
  for (auto row = vb; row < ve; ++row) {
    double section = squared - (v_[row] - v) * (v_[row] - v);
    if (section <= 0 || maximum_ + std::sqrt(section) <= best) {continue;}
    for (auto col = begin; col < end; ++col) {
      double remaining = section - (u_[col] - u) * (u_[col] - u);
      if (remaining <= 0 || heights_[row * u_.size() + col] + radius <= best) {continue;}
      best = std::max(best, heights_[row * u_.size() + col] + std::sqrt(remaining));
    }
  }
  if (!std::isfinite(best)) {throw std::invalid_argument("envelope grid misses footprint");}
  return best;
}
}  // namespace climbot_gazebo
