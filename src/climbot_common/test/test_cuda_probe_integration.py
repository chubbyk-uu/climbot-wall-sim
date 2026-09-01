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

"""Explicit smoke coverage for a separately installed CUDA OpenCV build."""

import os

from climbot_common.acceleration import probe_cuda_opencv, resolve_cuda_opencv_root
import pytest


def test_explicit_cuda_opencv_probe():
    """Exercise real CUDA kernels only when the caller explicitly configured a prefix."""
    if not os.environ.get('CLIMBOT_CUDA_OPENCV_ROOT'):
        pytest.skip('CLIMBOT_CUDA_OPENCV_ROOT is not configured for a CUDA integration run.')
    result = probe_cuda_opencv(resolve_cuda_opencv_root())
    assert result['status'] == 'completed'
    assert result['probe'] == 'cuda_opencv'
    assert result['opencv']['backend'] == 'cuda'
    assert result['opencv']['cuda']['compute_capability']
    assert result['opencv']['cuda']['free_memory_bytes'] > 0
