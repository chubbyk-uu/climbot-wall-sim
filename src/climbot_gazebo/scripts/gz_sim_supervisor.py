#!/usr/bin/env python3
# Copyright 2026 jerry
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Supervise one Gazebo client without letting Ctrl+C kill it mid-frame."""

import argparse
import os
import shutil
import signal
import subprocess
import time


class GazeboSupervisor:
    """Forward launch shutdown to a separately grouped Gazebo process tree."""

    def __init__(self, command: list[str], environment: dict[str, str] | None = None) -> None:
        self._command = command
        self._environment = environment
        self._child: subprocess.Popen | None = None
        self._stopping = False
        self._deadline = 0.0

    def _stop(self, _signum, _frame) -> None:
        """Use TERM because Gazebo 8 can crash while handling SIGINT in Ogre."""
        # launch escalates SIGINT -> SIGTERM -> SIGKILL.  The child process
        # group must be reaped before launch kills this supervisor, so repeated
        # signals may never extend the first deadline.
        if not self._stopping:
            self._stopping = True
            self._deadline = time.monotonic() + 4.0
        if self._child is not None and self._child.poll() is None:
            try:
                os.killpg(self._child.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

    def run(self) -> int:
        for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(number, self._stop)
        self._child = subprocess.Popen(
            self._command, start_new_session=True, env=self._environment)
        if self._stopping:
            self._stop(None, None)
        while self._child.poll() is None:
            if self._stopping and time.monotonic() >= self._deadline:
                try:
                    os.killpg(self._child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self._deadline = float('inf')
            time.sleep(0.05)
        # A requested shutdown is successful even though Gazebo reports its
        # own signal exit code. A spontaneous non-zero exit remains visible to
        # launch and still shuts down the required physics server.
        return 0 if self._stopping else self._child.returncode


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('server', 'gui'))
    parser.add_argument('--world', help='Rendered SDF file; required for server mode.')
    parser.add_argument(
        '--software-rendering', action='store_true',
        help='Run this client with Mesa llvmpipe instead of the shared GPU.')
    arguments = parser.parse_args()
    if arguments.mode == 'server' and not arguments.world:
        parser.error('--world is required for server mode')
    executable = shutil.which('gz')
    if executable is None:
        raise RuntimeError('gz executable is not on PATH')
    command = [executable, 'sim']
    if arguments.mode == 'server':
        command += ['-s', '-r', '-v', '3', arguments.world]
    else:
        command += ['-g', '-v', '3']
    command += ['--force-version', '8']
    environment = None
    if arguments.software_rendering:
        if arguments.mode != 'gui':
            parser.error('--software-rendering is only valid for GUI mode')
        environment = os.environ.copy()
        environment['GALLIUM_DRIVER'] = 'llvmpipe'
        environment.pop('MESA_D3D12_DEFAULT_ADAPTER_NAME', None)
    return GazeboSupervisor(command, environment).run()


if __name__ == '__main__':
    raise SystemExit(main())
