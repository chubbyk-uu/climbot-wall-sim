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

#include "climbot_description/geometry.hpp"

#include <cmath>

namespace climbot_description
{

double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

Quaternion quaternionMultiply(const Quaternion & first, const Quaternion & second)
{
  return Quaternion{
    first.w * second.x + first.x * second.w + first.y * second.z - first.z * second.y,
    first.w * second.y - first.x * second.z + first.y * second.w + first.z * second.x,
    first.w * second.z + first.x * second.y - first.y * second.x + first.z * second.w,
    first.w * second.w - first.x * second.x - first.y * second.y - first.z * second.z};
}

Quaternion quaternionConjugate(const Quaternion & quaternion)
{
  return Quaternion{-quaternion.x, -quaternion.y, -quaternion.z, quaternion.w};
}

Quaternion quaternionFromRpy(double roll, double pitch, double yaw)
{
  const double cosine_roll = std::cos(roll * 0.5);
  const double sine_roll = std::sin(roll * 0.5);
  const double cosine_pitch = std::cos(pitch * 0.5);
  const double sine_pitch = std::sin(pitch * 0.5);
  const double cosine_yaw = std::cos(yaw * 0.5);
  const double sine_yaw = std::sin(yaw * 0.5);
  return Quaternion{
    sine_roll * cosine_pitch * cosine_yaw - cosine_roll * sine_pitch * sine_yaw,
    cosine_roll * sine_pitch * cosine_yaw + sine_roll * cosine_pitch * sine_yaw,
    cosine_roll * cosine_pitch * sine_yaw - sine_roll * sine_pitch * cosine_yaw,
    cosine_roll * cosine_pitch * cosine_yaw + sine_roll * sine_pitch * sine_yaw};
}

Vector3 rotateVector(const Quaternion & quaternion, const Vector3 & vector)
{
  const double cross_x = 2.0 * (quaternion.y * vector.z - quaternion.z * vector.y);
  const double cross_y = 2.0 * (quaternion.z * vector.x - quaternion.x * vector.z);
  const double cross_z = 2.0 * (quaternion.x * vector.y - quaternion.y * vector.x);
  return Vector3{
    vector.x + quaternion.w * cross_x + quaternion.y * cross_z - quaternion.z * cross_y,
    vector.y + quaternion.w * cross_y + quaternion.z * cross_x - quaternion.x * cross_z,
    vector.z + quaternion.w * cross_z + quaternion.x * cross_y - quaternion.y * cross_x};
}

double yawFromQuaternion(const Quaternion & quaternion)
{
  return std::atan2(
    2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
    1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
}

}  // namespace climbot_description
