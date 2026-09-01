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

"""Executable CUDA OpenCV smoke test, intentionally imported only in a child."""

from __future__ import annotations

import json
import re
import sys


def _cuda_version(build_information: str) -> str | None:
    match = re.search(r'^\s*NVIDIA CUDA:\s+YES \(ver ([^)]+)\)', build_information,
                      flags=re.MULTILINE)
    return match.group(1) if match else None


def _probe() -> dict:
    # These imports must remain inside the child. The controller has no cv2 or
    # numpy import, so system OpenCV and the isolated CUDA build cannot mix.
    import cv2
    import numpy as np

    count = cv2.cuda.getCudaEnabledDeviceCount()
    if count <= 0:
        raise RuntimeError('OpenCV reports no CUDA-capable device.')
    cv2.cuda.setDevice(0)
    device = cv2.cuda.DeviceInfo(0)
    source = np.arange(16, dtype=np.uint8).reshape(4, 4)
    source_gpu = cv2.cuda_GpuMat()
    source_gpu.upload(source)
    if not np.array_equal(source_gpu.download(), source):
        raise RuntimeError('GpuMat upload/download round trip changed pixels.')
    identity = np.eye(3, dtype=np.float32)
    warped = cv2.cuda.warpPerspective(source_gpu, identity, (4, 4),
                                      flags=cv2.INTER_NEAREST)
    if not np.array_equal(warped.download(), source):
        raise RuntimeError('CUDA warpPerspective identity check failed.')
    coordinates = np.arange(4, dtype=np.float32)
    map_x, map_y = np.meshgrid(coordinates, coordinates)
    map_x_gpu, map_y_gpu = cv2.cuda_GpuMat(), cv2.cuda_GpuMat()
    map_x_gpu.upload(map_x)
    map_y_gpu.upload(map_y)
    remapped = cv2.cuda.remap(source_gpu, map_x_gpu, map_y_gpu, cv2.INTER_NEAREST)
    if not np.array_equal(remapped.download(), source):
        raise RuntimeError('CUDA remap identity check failed.')
    doubled = cv2.cuda.add(source_gpu, source_gpu).download()
    if not np.array_equal(doubled, (source * 2).astype(np.uint8)):
        raise RuntimeError('CUDA arithmetic check failed.')
    # Python's ``queryMemory`` binding takes output references, unlike the C++
    # method. Its two scalar accessors are the supported Python API.
    free_memory, total_memory = device.freeMemory(), device.totalMemory()
    from climbot_common.acceleration import opencv_provenance
    build_information = cv2.getBuildInformation()
    return {
        'status': 'completed', 'probe': 'cuda_opencv',
        'opencv': opencv_provenance(cv2, backend='cuda', cuda={
            'device_index': 0,
            # OpenCV's Python DeviceInfo wrapper exposes no device-name method
            # (unlike C++). A numeric index plus capability is unambiguous for
            # this process; do not shell out to nvidia-smi and make the probe
            # depend on a second optional installation.
            'device_name': None,
            'compute_capability': f'{device.majorVersion()}.{device.minorVersion()}',
            'total_memory_bytes': int(total_memory),
            'free_memory_bytes': int(free_memory),
            'cuda_build_version': _cuda_version(build_information),
            'device_count': int(count),
        }),
    }


def main() -> int:
    # Reading stdin proves the worker can participate in the controller
    # protocol, while keeping its request deliberately empty for this probe.
    try:
        request = json.loads(sys.stdin.read() or '{}')
        if request != {}:
            raise ValueError('CUDA probe accepts only an empty request object.')
        result = _probe()
    except Exception as error:  # The controller needs a machine-readable failure too.
        print(json.dumps({'status': 'failed', 'probe': 'cuda_opencv',
                          'error': {'message': str(error)}}, allow_nan=False,
                         sort_keys=True))
        return 2
    print(json.dumps(result, allow_nan=False, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
