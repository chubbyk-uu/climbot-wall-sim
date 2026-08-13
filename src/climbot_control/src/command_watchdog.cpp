#include "climbot_control/command_watchdog.hpp"

#include <cmath>
#include <stdexcept>

namespace climbot_control
{

CommandWatchdog::CommandWatchdog(double timeout_s)
: timeout_s_(timeout_s)
{
  if (timeout_s_ <= 0.0) {
    throw std::invalid_argument("Watchdog timeout must be positive.");
  }
}

bool CommandWatchdog::accept(const Command & command, double received_time_s)
{
  if (!std::isfinite(command.linear) || !std::isfinite(command.angular) ||
    !std::isfinite(received_time_s))
  {
    command_ = {};
    have_command_ = false;
    return false;
  }
  command_ = command;
  received_time_s_ = received_time_s;
  have_command_ = true;
  return true;
}

bool CommandWatchdog::timedOut(double current_time_s) const
{
  return !std::isfinite(current_time_s) || !have_command_ ||
         current_time_s < received_time_s_ ||
         current_time_s - received_time_s_ > timeout_s_;
}

Command CommandWatchdog::commandAt(double current_time_s) const
{
  return timedOut(current_time_s) ? Command{} : command_;
}

}  // namespace climbot_control
