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

#include "climbot_gazebo/pressure_sensor.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <vector>
namespace v = climbot_gazebo::vacuum;

TEST(PressureSensor, ExactGridDelayBiasAndReset)
{
  v::SensorParameters p;p.noise_pa = 0;p.bias_pa = 12;p.delay_s = .02;
  v::PressureSensor sensor(p);std::vector<v::PressureSample> samples;
  for (int i = 0; i < 100; ++i) {
    double t = i * .001;auto s = sensor.update(t, 100000 - 1000 * t);
    EXPECT_FALSE(sensor.update(t, 0).has_value());  // paused timestamp
    if (s) {samples.push_back(*s);}
  }
  ASSERT_EQ(samples.size(), 8u);
  for (std::size_t i = 0; i < samples.size(); ++i) {
    EXPECT_NEAR(samples[i].stamp_s, .01 * i, 1e-12);
    EXPECT_NEAR(samples[i].delivery_s - samples[i].stamp_s, .02, 1e-12);
    EXPECT_NEAR(samples[i].pressure_pa, 100012 - 10 * i, 1e-9);
    EXPECT_EQ(samples[i].variance_pa2, 0);
  }
  sensor.reset();EXPECT_FALSE(sensor.update(0, 101325));
  EXPECT_THROW(sensor.update(-.001, 100000), std::invalid_argument);
}
TEST(PressureSensor, FixedSeedStatisticsAndSamplingStep)
{
  v::SensorParameters p;p.delay_s = 0;v::PressureSensor a(p), b(p), fine(p);
  double sum = 0, sum2 = 0;int n = 0;
  for (int i = 0; i < 100000; ++i) {
    auto x = a.update(i * .001, 100000);auto y = b.update(i * .001, 100000);
    auto z = fine.update(i * .001, 100000);
    fine.update(i * .001 + .0005, 100000);
    ASSERT_EQ(x.has_value(), y.has_value());ASSERT_EQ(x.has_value(), z.has_value());
    if (x) {
      EXPECT_EQ(x->pressure_pa, y->pressure_pa);EXPECT_EQ(x->pressure_pa, z->pressure_pa);
      EXPECT_EQ(x->variance_pa2, 100);
      double e = x->pressure_pa - 100000;sum += e;sum2 += e * e;++n;
    }
  }
  EXPECT_EQ(n, 10000);double mean = sum / n, variance = sum2 / n - mean * mean;
  EXPECT_NEAR(mean, 25, .4);EXPECT_NEAR(variance, 100, 5);
  a.reset();b.reset();EXPECT_EQ(a.update(0, 100000)->pressure_pa, b.update(0, 100000)->pressure_pa);
}
TEST(PressureSensor, FaultQueueIsolationStuckRecoveryAndSaturation)
{
  v::SensorParameters p;p.noise_pa = 0;p.bias_pa = 0;p.delay_s = .01;
  v::PressureSensor sensor(p);
  EXPECT_FALSE(sensor.update(0, 90000));sensor.fault(1);
  EXPECT_FALSE(sensor.update(.01, 91000));  // old queued sample flushed
  sensor.fault(0);EXPECT_FALSE(sensor.update(.02, 92000));
  auto sample = sensor.update(.03, 93000);ASSERT_TRUE(sample);EXPECT_EQ(sample->pressure_pa, 92000);
  sensor.fault(2);EXPECT_FALSE(sensor.update(.04, 94000));
  sample = sensor.update(.05, 95000);ASSERT_TRUE(sample);
  EXPECT_EQ(sample->pressure_pa, 93000);EXPECT_EQ(sample->stamp_s, .04);
  EXPECT_EQ(sample->variance_pa2, 0);
  sensor.fault(0);EXPECT_FALSE(sensor.update(.06, 999999));
  sample = sensor.update(.07, 0);ASSERT_TRUE(sample);
  EXPECT_EQ(sample->pressure_pa, 120000);EXPECT_TRUE(sample->saturated);
  sensor.reset();EXPECT_EQ(sensor.fault(), 0);EXPECT_EQ(sensor.samples(), 0u);
  EXPECT_THROW(sensor.fault(3), std::invalid_argument);
}
TEST(PressureSensor, InvalidParametersAndOffGridDoNotSilentlyDegrade)
{
  v::SensorParameters p;p.rate_hz = 0;
  EXPECT_THROW(v::PressureSensor{p}, std::invalid_argument);
  p.rate_hz = 100;p.noise_pa = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(v::PressureSensor{p}, std::invalid_argument);
  p.noise_pa = 0;v::PressureSensor sensor(p);
  sensor.update(0, 100000);EXPECT_THROW(sensor.update(.011, 100000), std::invalid_argument);
}
