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

// The control loop must never be timed by a clock that can be set backwards.
//
// A node's own clock is RCL_ROS_TIME, which falls back to the system clock
// while sim time is inactive. A backward step there -- WSL2 does it roughly
// every 30 s, NTP can do it anywhere -- stops a timer built on that clock for
// the length of the step, so the tracker published nothing for over a second
// while the robot kept moving on its last command. These cases fail if the
// choice is ever reverted to get_clock().

#include <gtest/gtest.h>

#include <memory>

#include "climbot_control/control_clock.hpp"
#include "rclcpp/rclcpp.hpp"

class ControlClock : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

TEST_F(ControlClock, isSteadyWhenSimTimeIsInactive)
{
  auto node = std::make_shared<rclcpp::Node>("control_clock_bare");
  ASSERT_FALSE(node->get_parameter("use_sim_time").as_bool());

  const auto clock = climbot_control::controlClock(node.get());
  ASSERT_NE(clock, nullptr);
  EXPECT_EQ(clock->get_clock_type(), RCL_STEADY_TIME)
    << "the control loop is timed by a clock that can be set backwards";
  // The node clock is the one that must not be used here.
  EXPECT_EQ(node->get_clock()->get_clock_type(), RCL_ROS_TIME);
  EXPECT_NE(clock, node->get_clock());
}

TEST_F(ControlClock, followsTheNodeClockWhenSimTimeIsActive)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  auto node = std::make_shared<rclcpp::Node>("control_clock_sim", options);
  ASSERT_TRUE(node->get_parameter("use_sim_time").as_bool());

  // Sim time is the plant's own timeline. Timing the loop off it would run the
  // controller at a cadence the simulation is not keeping.
  const auto clock = climbot_control::controlClock(node.get());
  ASSERT_NE(clock, nullptr);
  EXPECT_EQ(clock, node->get_clock());
  EXPECT_EQ(clock->get_clock_type(), RCL_ROS_TIME);
}

TEST_F(ControlClock, steadyTimeDoesNotFollowSystemTime)
{
  // Guards the assumption the whole fix rests on: RCL_STEADY_TIME is a
  // different timeline from the settable one, not an alias for it.
  rclcpp::Clock steady(RCL_STEADY_TIME);
  rclcpp::Clock system(RCL_SYSTEM_TIME);
  const auto steady_now = steady.now();
  const auto system_now = system.now();
  EXPECT_EQ(steady_now.get_clock_type(), RCL_STEADY_TIME);
  EXPECT_EQ(system_now.get_clock_type(), RCL_SYSTEM_TIME);
  // A steady clock counts from an arbitrary boot-relative origin, so it cannot
  // be within a day of a wall-clock epoch reading.
  EXPECT_GT(system_now.nanoseconds() - steady_now.nanoseconds(), 86400LL * 1000000000LL);
}
