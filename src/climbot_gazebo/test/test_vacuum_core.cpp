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
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace v = climbot_gazebo::vacuum;

TEST(VacuumSkirt, OutsideWallCoverageCannotCarryLipOrMechanicalLoad)
{
  v::SkirtParameters p; p.inner_area_m2 = .1;
  std::vector<v::SkirtSample> points(1);
  points[0].length_m = 1; points[0].gap_m = .018;
  auto full = v::skirt(p, points, 6000);
  points[0].support_fraction = .5;
  auto half = v::skirt(p, points, 6000);
  EXPECT_NEAR(half.normal_n, full.normal_n / 2, 1e-12);
  EXPECT_NEAR(half.weighted_lip_area_m2, full.weighted_lip_area_m2 / 2, 1e-12);
  points[0].support_fraction = 0;
  EXPECT_DOUBLE_EQ(v::skirt(p, points, 6000).normal_n, 0);
  points[0].support_fraction = 1.01;
  EXPECT_THROW(v::skirt(p, points, 6000), std::invalid_argument);
}

TEST(VacuumCore, RejectsInvalidGasCurvesAndStates)
{
  v::Gas gas; gas.temperature_k = 0;
  EXPECT_THROW(v::Chamber(gas, {v::Fan{}}), std::invalid_argument);
  gas = {}; v::Fan fan; fan.shutoff_pa = 50000;
  EXPECT_THROW(v::Chamber(gas, {fan}), std::invalid_argument);
  v::Chamber core(gas, {v::Fan{}});
  EXPECT_THROW(core.balance(0, {1}, 0), std::invalid_argument);
  EXPECT_THROW(core.balance(100000, {1.1}, 0), std::invalid_argument);
  EXPECT_THROW(core.balance(100000, {}, 0), std::invalid_argument);
  EXPECT_THROW(core.balance(100000, {1}, -1), std::invalid_argument);
  EXPECT_THROW(core.balance(std::numeric_limits<double>::quiet_NaN(), {1}, 0),
    std::invalid_argument);
}
TEST(VacuumCore, OrificeHasFiniteZeroSlopeAndReversesWithPressure)
{
  v::Chamber core({}, {v::Fan{}});
  EXPECT_DOUBLE_EQ(core.orifice(101325, 101325, 1e-4), 0);
  EXPECT_NEAR(core.orifice(101325, 95000, 1e-4), -core.orifice(95000, 101325, 1e-4), 1e-14);
  double slope1 = core.orifice(101325, 101325 - 1e-3, 1e-4) / 1e-3;
  double slope2 = core.orifice(101325, 101325 - 1e-4, 1e-4) / 1e-4;
  EXPECT_GT(slope1, 0); EXPECT_NEAR(slope1 / slope2, 1, 1e-3);
  EXPECT_DOUBLE_EQ(core.orifice(101325, 95000, 0), 0);
}
TEST(VacuumCore, ChokedFlowMatchesIndependentSonicExpression)
{
  v::Gas gas;v::Chamber core(gas, {v::Fan{}});
  double ideal = gas.discharge_coefficient * 1e-4 * gas.ambient_pa *
    std::sqrt(gas.gamma / (gas.specific_r * gas.temperature_k)) *
    std::pow(2 / (gas.gamma + 1), (gas.gamma + 1) / (2 * (gas.gamma - 1)));
  EXPECT_NEAR(core.orifice(gas.ambient_pa, 10000, 1e-4), ideal, 1e-10);
  EXPECT_NEAR(core.orifice(gas.ambient_pa, 1000, 1e-4), ideal, 1e-10);
}
TEST(VacuumCore, ParallelFansAddFlowNotShutoffPressure)
{
  v::Fan fan;v::Chamber one({}, {fan}), three({}, {fan, fan, fan});
  auto a = one.balance(97000, {1}, 0), b = three.balance(97000, {1, 1, 1}, 0);
  EXPECT_NEAR(b.pump_kg_s, 3 * a.pump_kg_s, 1e-14);
  auto p1 = one.equilibrium({1}, 0, 101325, .003, .003);
  auto p3 = three.equilibrium({1, 1, 1}, 0, 101325, .003, .003);
  EXPECT_NEAR(p1.delta_pa, fan.shutoff_pa, 1e-6);
  EXPECT_NEAR(p3.delta_pa, p1.delta_pa, 1e-6);
}
TEST(VacuumCore, EquilibriumMassBalanceAndOpeningMonotonicity)
{
  v::Fan f;v::Chamber core({}, {f, f, f});double previous = 1e10;
  for (double area : {1e-5, 5e-5, 1e-4, 5e-4}) {
    auto r = core.equilibrium({1, 1, 1}, area, 101325, .003, .003);
    EXPECT_LT(r.delta_pa, previous);previous = r.delta_pa;
    EXPECT_NEAR(r.residual_kg_s, 0, 1e-10);
    EXPECT_GT(r.pressure_slope_kg_s_pa, 0);
  }
}
TEST(VacuumCore, StoppedChannelAndLowerHeadFanCanBackflow)
{
  v::Fan f;v::Chamber core({}, {f, f, f});
  EXPECT_GT(core.balance(95000, {1, 1, 0}, 1e-5).reverse_kg_s, 0);
  EXPECT_GT(core.balance(95000, {1, 1, .1}, 1e-5).reverse_kg_s, 0);
  f.reverse_fraction = 0;v::Chamber checked({}, {f, f, f});
  EXPECT_DOUBLE_EQ(checked.balance(95000, {1, 1, 0}, 1e-5).reverse_kg_s, 0);
}
TEST(VacuumCore, FullyClosedStoppedChamberRetainsMassIncludingCompression)
{
  v::Fan f;f.reverse_fraction = 0;v::Chamber core({}, {f});
  auto r = core.equilibrium({0}, 0, 95000, .003, .002);
  EXPECT_TRUE(r.sealed_retained);EXPECT_NEAR(r.pressure_pa, 142500, 1e-7);
  auto ode = core.implicitStep(95000, .003, .002, .001, {0}, 0);
  EXPECT_NEAR(ode.pressure_pa, 142500, 1e-6);
}
TEST(VacuumCore, NonUniqueRunningSealedCheckValveBranchIsRejected)
{
  v::Fan f;f.reverse_fraction = 0;v::Chamber core({}, {f});
  EXPECT_THROW(core.equilibrium({1}, 0, 101325, .003, .003), std::runtime_error);
  // Dynamic mode still has a well-defined trajectory from its initial gas mass.
  EXPECT_GT(core.implicitStep(101325, .003, .003, .001, {1}, 0).delta_pa, 0);
}
TEST(VacuumCore, ImplicitUpdateConservesMassAndConvergesToSteadyState)
{
  v::Gas gas;v::Fan f;v::Chamber core(gas, {f, f, f});double pc = 101325, V = .003, dt = .0001;
  for (int i = 0; i < 5000; ++i) {
    auto next = core.implicitStep(pc, V, V, dt, {1, 1, 1}, 1e-4);
    double dm = (next.pressure_pa - pc) * V / (gas.specific_r * gas.temperature_k);
    EXPECT_NEAR(dm, dt * next.residual_kg_s, 1e-14);pc = next.pressure_pa;
  }
  auto steady = core.equilibrium({1, 1, 1}, 1e-4, 101325, V, V);
  EXPECT_NEAR(pc, steady.pressure_pa, .01);
}
TEST(VacuumCore, CompressionCanVentToAmbientWithoutClipping)
{
  v::Chamber core({}, {v::Fan{}});
  auto r = core.implicitStep(101325, .003, .002, .0001, {0}, 1e-3);
  EXPECT_GT(r.pressure_pa, 101325);EXPECT_LT(r.pressure_pa, 151987.5);
  EXPECT_LT(r.leakage_kg_s, 0);
}
TEST(VacuumCore, PauseAndResetInputsHaveNoHiddenState)
{
  v::Chamber core({}, {v::Fan{}});
  EXPECT_DOUBLE_EQ(core.implicitStep(97000, .003, .003, 0, {.5}, 1e-5).pressure_pa, 97000);
  EXPECT_THROW(core.implicitStep(97000, .003, .002, 0, {.5}, 1e-5), std::invalid_argument);
  EXPECT_DOUBLE_EQ(core.speedsAfter({.2}, {1}, 0)[0], .2);
  EXPECT_NEAR(core.speedsAfter({0}, {1}, .15)[0], 1 - std::exp(-1), 1e-14);
  EXPECT_DOUBLE_EQ(core.speedsAfter({0}, {0}, 1)[0], 0);
  auto first = core.equilibrium({.5}, 1e-4, 101325, .003, .003);
  auto reset = core.equilibrium({.5}, 1e-4, 101325, .003, .003);
  EXPECT_DOUBLE_EQ(first.pressure_pa, reset.pressure_pa);
}

std::vector<v::SkirtSample> squareSamples()
{
  return {{-.2, -.1, .4, .018, 0, .1, 0, 0, false},
    {.2, -.1, .2, .018, 0, .1, 0, 0, false},
    {.2, .1, .4, .018, 0, .1, 0, 0, false},
    {-.2, .1, .2, .018, 0, .1, 0, 0, false}};
}
TEST(VacuumSkirt, CompressionLipAndPressureUseOneConsistentArea)
{
  v::SkirtParameters p;p.inner_area_m2 = .1;auto pts = squareSamples();
  auto r = v::skirt(p, pts, 6000);
  EXPECT_NEAR(r.mechanical_n, 20, 1e-12);
  EXPECT_NEAR(r.weighted_lip_area_m2, .012, 1e-14);
  EXPECT_NEAR(r.normal_n, 92, 1e-12);
  EXPECT_NEAR(r.effective_area_m2, .112, 1e-14);
  EXPECT_NEAR(-r.pressure.force.z - r.contact.force.z, 6000 * .1 - 20, 1e-12);
  EXPECT_LT(r.drag_power_w, 0);
  for (auto & pt : pts) {
    pt.velocity_x_mps = 0;
                                             }
  EXPECT_DOUBLE_EQ(v::skirt(p, pts, 6000).contact.force.x, 0);
}
TEST(VacuumSkirt, PreservesWeightedCentreAndMomentForAsymmetricContact)
{
  v::SkirtParameters p;p.inner_area_m2 = .1;p.centre_x_m = -.1;
  auto pts = squareSamples();pts[0].gap_m = .024;
  auto r = v::skirt(p, pts, 6000);
  EXPECT_NEAR(r.pressure.moment.y, r.pressure_centre.x * 6000 * r.effective_area_m2, 1e-12);
  EXPECT_NEAR(r.pressure.moment.x, -r.pressure_centre.y * 6000 * r.effective_area_m2, 1e-12);
  EXPECT_NE(r.pressure_centre.x, p.centre_x_m);
}
TEST(VacuumSkirt, OpeningAndDampingHaveContinuousContactBoundary)
{
  v::SkirtParameters p;p.inner_area_m2 = .1;auto pts = squareSamples();
  for (auto & pt : pts) {
    pt.gap_m = p.free_height_m - 1e-10;pt.gap_rate_mps = -.01;
                                                                             }
  auto a = v::skirt(p, pts, 6000);
  for (auto & pt : pts) {
    pt.gap_m = p.free_height_m + 1e-10;
                                                        }
  auto b = v::skirt(p, pts, 6000);
  EXPECT_NEAR(a.normal_n, b.normal_n, 1e-5);EXPECT_NEAR(a.leak_area_m2, b.leak_area_m2, 1e-12);
}
TEST(VacuumSkirt, LocalDepressionIsNotAutomaticallyAThroughChannel)
{
  v::SkirtParameters p;p.inner_area_m2 = .1;auto pts = squareSamples();
  pts[0].channel_height_m = .001;
  EXPECT_DOUBLE_EQ(v::skirt(p, pts, 0).leak_area_m2, 0);
  pts[0].connected = true;
  EXPECT_NEAR(v::skirt(p, pts, 0).leak_area_m2, .0004, 1e-14);
}
TEST(VacuumSkirt, DetachedLipHasNoResidualWallSupportAndHardLimitIsExplicit)
{
  v::SkirtParameters p;p.inner_area_m2 = .1;auto pts = squareSamples();
  for (auto & pt : pts) {
    pt.gap_m = .025;
                                       }
  auto r = v::skirt(p, pts, 6000);
  EXPECT_DOUBLE_EQ(r.normal_n, 0);EXPECT_DOUBLE_EQ(r.weighted_lip_area_m2, 0);
  EXPECT_GT(r.leak_area_m2, 0);
  pts[0].gap_m = .001;
  EXPECT_THROW(v::skirt(p, pts, 6000), std::domain_error);
}
