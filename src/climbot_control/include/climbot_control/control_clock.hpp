#ifndef CLIMBOT_CONTROL__CONTROL_CLOCK_HPP_
#define CLIMBOT_CONTROL__CONTROL_CLOCK_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace climbot_control
{

/// The clock a control loop and its timers must run on.
///
/// A node's own clock is RCL_ROS_TIME, which falls back to the system clock
/// whenever sim time is inactive. The system clock is settable and can step
/// backwards: WSL2 resynchronises it against the host roughly every 30 s and
/// steps it back by more than a second. A timer built on that clock simply
/// does not fire for the length of the step, so the loop stops issuing
/// commands while the robot keeps moving on the last one it was given, and
/// every elapsed time measured against it shrinks, which hides staleness
/// instead of reporting it.
///
/// Sim time is the plant's own timeline and has to be followed while it is
/// active, otherwise a control loop would run at a cadence the simulation is
/// not keeping. Off sim time, a steady clock is the only one that cannot jump.
///
/// Message stamps are a separate question and still belong on ROS time, so
/// this is deliberately not a replacement for the node clock everywhere.
inline rclcpp::Clock::SharedPtr controlClock(rclcpp::Node * node)
{
  // rclcpp declares use_sim_time for every node, so this parameter always
  // exists and reflects what the node's own clock is about to follow.
  if (node->get_parameter("use_sim_time").as_bool()) {
    return node->get_clock();
  }
  return std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
}

}  // namespace climbot_control

#endif  // CLIMBOT_CONTROL__CONTROL_CLOCK_HPP_
