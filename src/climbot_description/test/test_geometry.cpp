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

#include <gtest/gtest.h>

#include <cmath>

#include "climbot_description/geometry.hpp"

using climbot_description::Quaternion;
using climbot_description::Vector3;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1e-12;
}  // namespace

TEST(Geometry, WrapsAnAngleIntoASingleTurn)
{
  EXPECT_NEAR(climbot_description::wrapAngle(0.5), 0.5, kTolerance);
  EXPECT_NEAR(climbot_description::wrapAngle(2.0 * kPi + 0.5), 0.5, 1e-9);
  EXPECT_NEAR(climbot_description::wrapAngle(-2.0 * kPi - 0.5), -0.5, 1e-9);
  EXPECT_LE(std::abs(climbot_description::wrapAngle(100.0)), kPi);
}

TEST(Geometry, MultipliesByIdentityWithoutChange)
{
  const Quaternion value{0.1, 0.2, 0.3, 0.927};
  const Quaternion identity{};
  const Quaternion product = climbot_description::quaternionMultiply(value, identity);
  EXPECT_NEAR(product.x, value.x, kTolerance);
  EXPECT_NEAR(product.y, value.y, kTolerance);
  EXPECT_NEAR(product.z, value.z, kTolerance);
  EXPECT_NEAR(product.w, value.w, kTolerance);
}

TEST(Geometry, ConjugateInvertsAUnitRotation)
{
  const Quaternion rotation = climbot_description::quaternionFromRpy(0.3, -0.4, 1.1);
  const Quaternion product = climbot_description::quaternionMultiply(
    rotation, climbot_description::quaternionConjugate(rotation));
  EXPECT_NEAR(product.x, 0.0, 1e-12);
  EXPECT_NEAR(product.y, 0.0, 1e-12);
  EXPECT_NEAR(product.z, 0.0, 1e-12);
  EXPECT_NEAR(product.w, 1.0, 1e-12);
}

TEST(Geometry, RotatesAVectorAQuarterTurnAboutZ)
{
  const Quaternion yaw = climbot_description::quaternionFromRpy(0.0, 0.0, kPi / 2.0);
  const Vector3 rotated = climbot_description::rotateVector(yaw, Vector3{1.0, 0.0, 0.0});
  EXPECT_NEAR(rotated.x, 0.0, 1e-12);
  EXPECT_NEAR(rotated.y, 1.0, 1e-12);
  EXPECT_NEAR(rotated.z, 0.0, 1e-12);
}

TEST(Geometry, RecoversYawFromItsOwnQuaternion)
{
  for (const double yaw : {-3.0, -1.0, 0.0, 0.75, 2.5}) {
    const Quaternion rotation = climbot_description::quaternionFromRpy(0.0, 0.0, yaw);
    EXPECT_NEAR(climbot_description::yawFromQuaternion(rotation), yaw, 1e-12);
  }
}

TEST(Geometry, RotationPreservesVectorLength)
{
  const Quaternion rotation = climbot_description::quaternionFromRpy(0.2, 0.9, -1.3);
  const Vector3 source{0.3, -1.2, 4.5};
  const Vector3 rotated = climbot_description::rotateVector(rotation, source);
  const double before = std::sqrt(
    source.x * source.x + source.y * source.y + source.z * source.z);
  const double after = std::sqrt(
    rotated.x * rotated.x + rotated.y * rotated.y + rotated.z * rotated.z);
  EXPECT_NEAR(before, after, 1e-12);
}
