#include "climbot_control/command_watchdog.hpp"

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

void CommandWatchdog::accept(const Command & command, double received_time_s)
{
  command_ = command;
  received_time_s_ = received_time_s;
  have_command_ = true;
}

bool CommandWatchdog::timedOut(double current_time_s) const
{
  return !have_command_ || current_time_s < received_time_s_ ||
         current_time_s - received_time_s_ > timeout_s_;
}

Command CommandWatchdog::commandAt(double current_time_s) const
{
  return timedOut(current_time_s) ? Command{} : command_;
}

}  // namespace climbot_control
