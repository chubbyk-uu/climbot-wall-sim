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
import sys
import threading
import time


class GazeboSupervisor:
    """Forward launch shutdown to a separately grouped Gazebo process tree."""

    def __init__(self, command: list[str], environment: dict[str, str] | None = None,
                 wait_for_render_ready: bool = False, gui_startup_retries: int = 0) -> None:
        self._command = command
        self._environment = environment
        self._child: subprocess.Popen | None = None
        self._stopping = False
        self._deadline = 0.0
        self._wait_for_render_ready = wait_for_render_ready
        self._gui_startup_retries = gui_startup_retries
        self._gui_rendered = threading.Event()

    def _watch_gui(self):
        """Never restart a client that has already rendered its first camera."""
        from gz.msgs10.pose_pb2 import Pose
        from gz.transport13 import Node

        node = Node()
        if not node.subscribe(Pose, '/gui/camera/pose', lambda _: self._gui_rendered.set()):
            raise RuntimeError('could not observe GUI startup readiness')
        return node

    @staticmethod
    def _forward_output(stream, failures):
        """Keep the full child log visible while recognizing two exact errors."""
        for line in stream:
            sys.stdout.write(line)
            sys.stdout.flush()
            for marker in ('D3D12: Removing Device.', 'Failed to create OpenGL context'):
                if marker in line:
                    failures.add(marker)

    def _run_child(self):
        failures = set()
        capture = self._gui_startup_retries > 0
        options = ({'stdout': subprocess.PIPE, 'stderr': subprocess.STDOUT,
                    'text': True, 'errors': 'replace', 'bufsize': 1} if capture else {})
        self._child = subprocess.Popen(
            self._command, start_new_session=True, env=self._environment, **options)
        reader = None
        if capture:
            reader = threading.Thread(target=self._forward_output,
                                      args=(self._child.stdout, failures), daemon=True)
            reader.start()
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
        if reader is not None:
            reader.join(timeout=1.)
            if reader.is_alive():
                return set()
            self._child.stdout.close()
        return failures

    def _wait_for_server(self) -> bool:
        """Serialize graphics initialization behind advancing server physics."""
        from gz.msgs10.world_stats_pb2 import WorldStatistics
        from gz.transport13 import Node

        ready = threading.Event()
        node = Node()

        def receive(message):
            # The initial render population blocks the first few iterations.
            # A server past 100 steps has finished that stage; allow an
            # already-initialized paused scene to open its GUI as well.
            if message.iterations >= 100:
                ready.set()

        topic = '/world/climbot_wall/stats'
        if not node.subscribe(WorldStatistics, topic, receive):
            raise RuntimeError('could not observe server rendering readiness')
        started = time.monotonic()
        print('Waiting for initialized server before opening Gazebo GUI', flush=True)
        try:
            while not ready.wait(.1):
                if self._stopping:
                    return False
                if time.monotonic() - started > 180:
                    raise RuntimeError('server did not finish graphics initialization in 180s')
            print(f'Server ready; opening Gazebo GUI after '
                  f'{time.monotonic() - started:.3f}s', flush=True)
            return not self._stopping
        finally:
            node.unsubscribe(topic)

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
        if self._wait_for_render_ready and not self._wait_for_server():
            return 0
        watcher = self._watch_gui() if self._gui_startup_retries else None
        try:
            for attempt in range(self._gui_startup_retries + 1):
                started = time.monotonic()
                failures = self._run_child()
                retry = (not self._stopping and self._child.returncode != 0
                         and not self._gui_rendered.is_set()
                         and time.monotonic() - started <= 30.
                         and len(failures) == 2 and attempt < self._gui_startup_retries)
                if not retry:
                    break
                print(f'GUI D3D12/OpenGL startup failed before first frame; '
                      f'retrying unchanged GPU backend ({attempt + 1}/'
                      f'{self._gui_startup_retries})', flush=True)
                deadline = time.monotonic() + 1.
                while not self._stopping and time.monotonic() < deadline:
                    time.sleep(.05)
                if self._stopping:
                    break
        finally:
            if watcher is not None:
                watcher.unsubscribe('/gui/camera/pose')
        # A requested shutdown is successful even though Gazebo reports its
        # own signal exit code. A spontaneous non-zero exit remains visible to
        # launch and still shuts down the required physics server.
        return 0 if self._stopping else self._child.returncode


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('server', 'gui'))
    parser.add_argument('--world', help='Rendered SDF file; required for server mode.')
    parser.add_argument('--wait-for-render-ready', action='store_true',
                        help='Wait for initialized Climbot physics before opening the GUI.')
    parser.add_argument(
        '--software-rendering', action='store_true',
        help='Run this client with Mesa llvmpipe instead of the shared GPU.')
    arguments = parser.parse_args()
    if arguments.mode == 'server' and not arguments.world:
        parser.error('--world is required for server mode')
    if arguments.wait_for_render_ready and arguments.mode != 'gui':
        parser.error('--wait-for-render-ready is only valid for GUI mode')
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
    retries = 2 if arguments.mode == 'gui' and os.environ.get('GALLIUM_DRIVER') == 'd3d12' \
        and not arguments.software_rendering else 0
    return GazeboSupervisor(command, environment, arguments.wait_for_render_ready, retries).run()


if __name__ == '__main__':
    raise SystemExit(main())
