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
#ifndef CLIMBOT_GAZEBO__WALL_SURFACE_HPP_
#define CLIMBOT_GAZEBO__WALL_SURFACE_HPP_
#include <array>
#include <string>
#include <vector>
namespace climbot_gazebo
{
// SI wall coordinates. Immutable, precomputed coefficients; queries allocate nothing.
// Seed realization is an offline operation; serialized modes are shared with runtime.
class WallSurface
{
public:
  explicit WallSurface(const std::string & resolved_yaml);
  double roughness(double u, double v) const;
  double grooveDepth(double u, double v) const;
  double height(double u, double v) const {return roughness(u, v) - grooveDepth(u, v);}
  std::array<double, 3> normal(double u, double v, double epsilon = 1e-6) const;

private:
  struct Mode {double ku, kv, phase, weight;};
  struct Groove
  {
    double u, v, du, dv, length, width, lo, hi, scale, shoulder, taper, radius, chamfer;
    bool trapezoid;
  };
  std::vector<Mode> modes_;
  std::vector<Groove> grooves_;
};
}  // namespace climbot_gazebo
#endif  // CLIMBOT_GAZEBO__WALL_SURFACE_HPP_
