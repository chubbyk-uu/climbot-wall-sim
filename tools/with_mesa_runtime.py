#!/usr/bin/env python3
"""Run a GL client with the same validated rendering runtime as Gazebo."""
import argparse
import os
import sys
import threading

from climbot_gazebo.mesa_runtime import rendering_environment


def wait_for_gazebo_gui(timeout=180.):
    """Wait for a rendered GUI camera before initializing another GPU client."""
    from gz.msgs10.pose_pb2 import Pose
    from gz.transport13 import Node

    ready = threading.Event()
    node = Node()
    topic = '/gui/camera/pose'
    if not node.subscribe(Pose, topic, lambda _: ready.set()):
        raise RuntimeError('could not observe Gazebo GUI camera readiness')
    print('Waiting for rendered Gazebo GUI before initializing RViz',
          file=sys.stderr, flush=True)
    try:
        if not ready.wait(timeout):
            raise RuntimeError('Gazebo GUI did not render its camera in 180s')
    finally:
        node.unsubscribe(topic)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gpu-backend', default='auto',
                        choices=['auto', 'wsl_d3d12', 'native', 'software'])
    parser.add_argument('--mesa', default=os.environ.get('CLIMBOT_MESA', 'private'),
                        choices=['private', 'system'])
    parser.add_argument('--wait-for-gazebo-gui', action='store_true',
                        help='Serialize RViz GPU initialization after the Gazebo GUI.')
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command:
        parser.error('a child command is required')
    env, identity = rendering_environment(args.gpu_backend, args.mesa)
    print(f'Rendering runtime: {identity}', file=sys.stderr, flush=True)
    if args.wait_for_gazebo_gui:
        wait_for_gazebo_gui()
    os.execvpe(command[0], command, env)


if __name__ == '__main__':
    main()
