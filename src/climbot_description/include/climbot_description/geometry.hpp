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

#ifndef CLIMBOT_DESCRIPTION__GEOMETRY_HPP_
#define CLIMBOT_DESCRIPTION__GEOMETRY_HPP_

namespace climbot_description
{

/// Component order matches geometry_msgs, so a message maps field by field.
struct Quaternion
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct Vector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

/// Wrap an angle to [-pi, pi].
double wrapAngle(double angle);

/// Return the product of two quaternions.
Quaternion quaternionMultiply(const Quaternion & first, const Quaternion & second);

/// Return the conjugate, which inverts a unit quaternion.
Quaternion quaternionConjugate(const Quaternion & quaternion);

/// Return the quaternion for fixed-axis roll, pitch and yaw.
Quaternion quaternionFromRpy(double roll, double pitch, double yaw);

/// Rotate a vector by a unit quaternion.
Vector3 rotateVector(const Quaternion & quaternion, const Vector3 & vector);

/// Return the yaw angle of a quaternion.
double yawFromQuaternion(const Quaternion & quaternion);

}  // namespace climbot_description

#endif  // CLIMBOT_DESCRIPTION__GEOMETRY_HPP_
