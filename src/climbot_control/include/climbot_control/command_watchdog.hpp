#ifndef CLIMBOT_CONTROL__COMMAND_WATCHDOG_HPP_
#define CLIMBOT_CONTROL__COMMAND_WATCHDOG_HPP_

#include "climbot_control/line_tracker.hpp"

namespace climbot_control
{

class CommandWatchdog
{
public:
  explicit CommandWatchdog(double timeout_s);

  void accept(const Command & command, double received_time_s);
  Command commandAt(double current_time_s) const;
  bool timedOut(double current_time_s) const;

private:
  double timeout_s_;
  double received_time_s_{0.0};
  Command command_{};
  bool have_command_{false};
};

}  // namespace climbot_control

#endif  // CLIMBOT_CONTROL__COMMAND_WATCHDOG_HPP_
