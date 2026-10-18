#!/usr/bin/env python3
"""Record actual ROS clock, GUI and Mesa evidence without controlling the robot."""
import argparse
from collections import defaultdict
import gzip
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from climbot_gazebo.mesa_runtime import default_prefix, libraries
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import FluidPressure, Imu, JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def runtime_processes():
    """Inspect only this lane's processes and avoid archiving host paths."""
    marker = ('GZ_PARTITION=' + os.environ['GZ_PARTITION']).encode()
    expected = libraries(default_prefix())
    result = []
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            if marker not in (proc / 'environ').read_bytes().split(b'\0'):
                continue
            cmd = (proc / 'cmdline').read_bytes().replace(b'\0', b' ').decode()
            if 'gz-sim' not in cmd and 'gz sim' not in cmd:
                continue
            maps = (proc / 'maps').read_text()
            mapped = [p for p in expected if str(p.resolve()) in maps or str(p) in maps]
            result.append({'pid': int(proc.name), 'gui': '--gui' in cmd or ' -g' in cmd,
                           'private_mapping_complete': len(mapped) == 4,
                           'libraries': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                         for p in mapped},
                           'system_gallium_mapped': any(
                               'libgallium' in line and str(default_prefix()) not in line
                               for line in maps.splitlines())})
        except (OSError, UnicodeError):
            continue
    return result


def screenshot(path, window_class=None):
    """Capture a visible native Gazebo window on the actual desktop."""
    # RViz also maps unnamed Qt helper windows with its class. Require the
    # titled main window rather than mistaking those helpers for extra GUIs.
    query = ['--class', window_class] if window_class else ['--name', 'Gazebo']
    windows = subprocess.check_output(
        ['xdotool', 'search', '--onlyvisible', *query], text=True).split()
    if window_class:
        windows = [wid for wid in windows if subprocess.check_output(
            ['xdotool', 'getwindowname', wid], text=True).strip().endswith('RViz')]
    if len(windows) != 1:
        raise RuntimeError(f'expected one visible {window_class or "Gazebo"}, '
                           f'found {len(windows)}')
    wid = windows[0]
    fields = dict(line.split('=', 1) for line in subprocess.check_output(
        ['xdotool', 'getwindowgeometry', '--shell', wid], text=True).splitlines())
    subprocess.run(['ffmpeg', '-v', 'error', '-f', 'x11grab', '-draw_mouse', '0',
                    '-window_id', wid, '-video_size', fields['WIDTH'] + 'x' + fields['HEIGHT'],
                    '-i', os.environ['DISPLAY'], '-frames:v', '1', str(path)],
                   check=True, timeout=20)
    return {'window_id': int(wid), 'width': int(fields['WIDTH']), 'height': int(fields['HEIGHT'])}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--geometry', choices=['legacy', 'vacuum_chassis'], required=True)
    parser.add_argument('--gui', choices=['required', 'disabled'], required=True)
    parser.add_argument('--rviz', choices=['required', 'disabled'], default='disabled')
    parser.add_argument('--duration', type=float, default=0.)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('s6_independent_gui_observer')
    buffer = Buffer()
    node.tf_listener = TransformListener(buffer, node)
    counts = defaultdict(int)
    points = []
    cpu = []
    states = []
    stop = False
    started = time.monotonic()
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    stream = gzip.open(args.output / 'vacuum.jsonl.gz', 'wt', compresslevel=1)

    def receive(name, message):
        counts[name] += 1
        if name == 'clock':
            if not points or time.monotonic() - points[-1][0] >= .1:
                points.append((time.monotonic(), message.clock.sec + message.clock.nanosec * 1e-9))
        elif name == 'vacuum':
            state = json.loads(message.data)
            stream.write(message.data + '\n')
            cpu.append(state['cpu_ns'])
            states.append((state['delta_pa'], state['fixture_active'], state['startup_failed'],
                           state['runtime_failed']))

    subs = []
    for name, topic, kind in [
            ('clock', '/clock', Clock), ('truth', '/model/climbot/ground_truth', Odometry),
            ('filtered', '/odometry/filtered', Odometry), ('imu', '/imu', Imu),
            ('imu_wall', '/imu_wall', Imu),
            ('joints', '/joint_states', JointState),
            ('pressure', '/suction/pressure', FluidPressure),
            ('vacuum', '/suction/ground_truth/state', String)]:
        subs.append(node.create_subscription(
            kind, topic, lambda msg, n=name: receive(n, msg), qos))

    def stopping(*_):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGTERM, stopping)
    signal.signal(signal.SIGINT, stopping)
    gui = None
    rviz = None
    processes = []
    error = None
    last_gui_check = started
    try:
        while not stop and (not args.duration or time.monotonic() - started < args.duration):
            rclpy.spin_once(node, timeout_sec=.05)
            if stop:
                break
            if processes and args.gui == 'required' and time.monotonic() - last_gui_check > 5:
                if not all(Path(f"/proc/{p['pid']}").exists() for p in processes if p['gui']):
                    raise RuntimeError('observed Gazebo GUI exited during the task')
                last_gui_check = time.monotonic()
                if rviz and not Path(f"/proc/{rviz['pid']}").exists():
                    raise RuntimeError('observed RViz exited during the task')
            if not processes and time.monotonic() - started > 12:
                processes = runtime_processes()
                if args.gui == 'required':
                    gui = screenshot(args.output / 'gui.png')
                    write(args.output / 'gui.json', gui)
                write(args.output / 'runtime_libraries.json', processes)
            if args.rviz == 'required' and rviz is None and time.monotonic()-started > 12:
                marker = ('GZ_PARTITION=' + os.environ['GZ_PARTITION']).encode()
                for proc in Path('/proc').glob('[0-9]*'):
                    try:
                        cmd = (proc / 'cmdline').read_bytes().split(b'\0')[0]
                        if (Path(os.fsdecode(cmd)).name != 'rviz2' or
                                marker not in (proc / 'environ').read_bytes().split(b'\0')):
                            continue
                        rviz = dict(screenshot(args.output / 'rviz.png', 'rviz2'),
                                    pid=int(proc.name),
                                    mapped_gl_libraries=[line.split()[-1] for line in
                                        (proc / 'maps').read_text().splitlines()
                                        if 'libgallium' in line or 'libGLX_mesa' in line])
                        write(args.output / 'rviz.json', rviz)
                        break
                    except (FileNotFoundError, ProcessLookupError):
                        continue
                if rviz is None and time.monotonic()-started > 60:
                    raise RuntimeError('required visible RViz did not become ready')
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        error = repr(exc)
    finally:
        stream.close()
        scored = [p for p in points if p[0] >= started + 20]
        bins = []
        if scored:
            anchor = scored[0]
            for point in scored[1:]:
                if point[0] - anchor[0] >= 10:
                    bins.append((point[1] - anchor[1]) / (point[0] - anchor[0]))
                    anchor = point
        tf = {frame: buffer.can_transform('odom', frame, rclpy.time.Time()) for frame in [
            'base_link', 'imu_link', 'inspection_camera_optical_frame',
            'left_wheel_link', 'right_wheel_link']}
        if args.geometry == 'vacuum_chassis':
            tf['chassis_link'] = buffer.can_transform('odom', 'chassis_link', rclpy.time.Time())
        rtf = ((scored[-1][1] - scored[0][1]) / (scored[-1][0] - scored[0][0])
               if len(scored) > 1 else None)
        endpoints = ([[p[0] - started, p[1]] for p in [scored[0], scored[-1]]]
                     if scored else [])
        value = {'geometry': args.geometry, 'gui_required': args.gui == 'required',
                 'rviz_required': args.rviz == 'required', 'rviz': rviz,
                 'gui': gui, 'error': error, 'counts': dict(counts), 'tf_connected': tf,
                 'clock_warmup_excluded_wall_s': 20, 'measured_realtime_factor': rtf,
                 'ten_wall_second_rtf': bins, 'wall_duration_s': time.monotonic() - started,
                 'clock_scored_endpoints': endpoints,
                 'plugin_cpu_us_mean': sum(cpu) / len(cpu) / 1000 if cpu else None,
                 'minimum_delta_pa': min((s[0] for s in states), default=None),
                 'fixture_released_observed': any(not s[1] for s in states),
                 'observed_startup_failure': any(s[2] for s in states),
                 'observed_runtime_failure': any(s[3] for s in states),
                 'runtime_libraries': processes}
        write(args.output / 'summary.json', value)
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps({'rtf': rtf, 'error': error, 'tf_connected': tf}))
    return 1 if (error or (args.gui == 'required' and gui is None) or
                 (args.rviz == 'required' and rviz is None)) else 0


if __name__ == '__main__':
    raise SystemExit(main())
