#!/usr/bin/env python3
"""Freeze the actual latched planner task and nominal frame estimate before dispatch."""
import argparse
import json
from pathlib import Path
import time

from climbot_inspection.archive_core import expected_image_count
from climbot_interfaces.msg import CoverageTask
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosidl_runtime_py.convert import message_to_ordereddict


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('freeze_capture_plan')
    tasks = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sub = node.create_subscription(CoverageTask, '/coverage/task', tasks.append, qos)
    deadline = time.monotonic()+90
    while not tasks and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.2)
    if not tasks:
        raise RuntimeError('no actual planner task to freeze')
    task = tasks[-1]
    result = {'task': message_to_ordereddict(task),
              'nominal_images': expected_image_count(task, task.detection_length, .2),
              'scan_segments': list(task.segment_types).count(task.SEGMENT_SCAN),
              'note': 'Nominal estimate only; final contract uses frozen dynamic scan references.'}
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k: result[k] for k in ['nominal_images', 'scan_segments']}))
    node.destroy_subscription(sub)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
