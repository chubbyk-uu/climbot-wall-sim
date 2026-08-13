"""Stop the robot when a driving script is terminated."""

# Gazebo's DiffDrive latches the last velocity command indefinitely: there is
# no command timeout, so a script killed mid-drive leaves the robot running
# until it reaches the edge of the wall and falls off. Python's finally blocks
# do not run on SIGTERM, so the stop has to come from a signal handler.

import signal
import time


def install_stop_on_termination(publish_stop, repeats=5, interval_s=0.02):
    """Publish a stop command when the process receives SIGINT or SIGTERM."""
    def handler(signal_number, frame):
        # Repeated because publication is asynchronous and the process is
        # about to exit; a single call can be dropped before it is delivered.
        for _ in range(repeats):
            publish_stop()
            time.sleep(interval_s)
        raise SystemExit(128 + signal_number)

    for signal_number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signal_number, handler)
