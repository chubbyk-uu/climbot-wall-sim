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

"""Adapter from the tiled CPU fusion protocol to the custom CUDA extension."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import os
import time

from climbot_mosaic.fusion import _tile_transform, FusionError, RenderFrame, RenderGrid
import cv2
import numpy as np


class CudaUnavailableError(FusionError):
    """The custom extension or a usable CUDA device is unavailable."""


class CudaRuntimeError(FusionError):
    """CUDA failed after the fusion attempt had started."""


def _extension():
    try:
        from climbot_mosaic import _fusion_cuda
    except ImportError as error:
        raise CudaUnavailableError(
            'custom CUDA fusion extension is not installed; rebuild climbot_mosaic '
            'with CUDA Toolkit 12.8 or use --backend cpu.') from error
    return _fusion_cuda


def cuda_device_info() -> dict:
    """Probe the installed custom extension with one real CUDA runtime call."""
    try:
        return dict(_extension().device_info())
    except RuntimeError as error:
        raise CudaUnavailableError(f'custom CUDA fusion probe failed: {error}') from error


class CudaRenderPool:
    """
    Pool-compatible, single-context CUDA tile renderer.

    ``fusion._write_image_pass`` only requires ``map`` and context-manager
    semantics.  Matching that small protocol keeps publication, TIFF writing,
    seam extraction and all failure cleanup shared with the CPU backend.
    """

    def __init__(self, initial: tuple[RenderFrame, ...], optimized: tuple[RenderFrame, ...],
                 grid: RenderGrid, width: int, height: int):
        if len(initial) != len(optimized) or any(
                first.image_path != second.image_path
                for first, second in zip(initial, optimized, strict=True)):
            raise FusionError('CUDA pose variants do not share one stable frame sequence.')
        extension = _extension()
        started = time.perf_counter()
        try:
            self._session = extension.FusionCudaSession(
                len(initial), width, height, grid.tile_size_px)
        except (RuntimeError, ValueError) as error:
            raise CudaUnavailableError(f'cannot create CUDA fusion session: {error}') from error
        decode_started = time.perf_counter()
        upload_elapsed = 0.0
        decoder_workers = max(1, min(os.cpu_count() or 1, 8))
        cv2.setNumThreads(1)

        def decode(frame):
            image = cv2.imread(frame.image_path, cv2.IMREAD_UNCHANGED)
            if image is None or image.dtype != np.uint8 or image.shape != (height, width):
                raise FusionError(
                    f'CUDA fusion source is not expected mono8: {frame.image_path}.')
            return image

        # Keep decoding parallel but bounded. A single executor.map over all
        # frames can queue the complete 2.78 GiB archive in host memory when a
        # low-index PNG is slow; batches cap the decoded backlog at 64 MiB.
        with ThreadPoolExecutor(max_workers=decoder_workers) as executor:
            for batch_start in range(0, len(initial), 32):
                batch = initial[batch_start:batch_start + 32]
                for offset, image in enumerate(executor.map(decode, batch)):
                    upload_started = time.perf_counter()
                    try:
                        self._session.upload(batch_start + offset, image)
                    except (RuntimeError, ValueError) as error:
                        raise CudaRuntimeError(f'CUDA frame upload failed: {error}') from error
                    upload_elapsed += time.perf_counter() - upload_started
        decode_elapsed = time.perf_counter() - decode_started - upload_elapsed
        self._initial = initial
        self._optimized = optimized
        self._grid = grid
        self._render_elapsed = 0.0
        self._rendered_tiles = 0
        self._empty_tiles = 0
        self._summary = {
            'sampling': 'opencv-compatible-1/32-pixel',
            'frame_count': len(initial),
            'resident_image_bytes': int(self._session.image_bytes),
            'decode_elapsed_s': decode_elapsed,
            'decoder_workers': decoder_workers,
            'upload_elapsed_s': upload_elapsed,
            'initialization_elapsed_s': time.perf_counter() - started,
        }

    def __enter__(self):
        return self

    def __exit__(self, _type, _value, _traceback):
        self._session = None

    def _empty(self, mode: str, row: int, column: int):
        size = self._grid.tile_size_px
        image = np.zeros((size, size), np.uint8)
        owner = np.zeros((size, size), np.uint16)
        coverage = np.zeros((size, size), np.uint16)
        quality = np.full((size, size), np.nan, np.float32)
        products = [row, column, image, owner, coverage]
        if mode == 'optimized':
            products.append(np.full((size, size), np.nan, np.float32))
        products.append(quality)
        return (*products, 0, 0, 0.0, 0)

    def _render(self, task):
        mode, row, column, initial_candidates, optimized_candidates = task
        if mode == 'initial':
            frames, candidates, auxiliary = self._initial, initial_candidates, False
        elif mode == 'optimized':
            frames, candidates, auxiliary = self._optimized, optimized_candidates, True
        else:
            raise FusionError(f'unknown CUDA render mode: {mode}.')
        if not candidates:
            self._empty_tiles += 1
            return self._empty(mode, row, column)
        transform = _tile_transform(self._grid, row, column)
        matrices = np.stack([
            transform @ np.asarray(frames[index].homography, np.float64).reshape(3, 3)
            for index in candidates])
        inverse = np.ascontiguousarray(np.linalg.inv(matrices), dtype=np.float64)
        indices = np.ascontiguousarray(candidates, dtype=np.uint16)
        centers = np.ascontiguousarray(
            [frames[index].center_xy_m for index in candidates], dtype=np.float64)
        posterior = np.ascontiguousarray(
            [frames[index].posterior_std for index in candidates], dtype=np.float64)
        started = time.perf_counter()
        try:
            image, owner, coverage, uncertainty, quality = self._session.render(
                inverse, indices, centers, posterior,
                auxiliary, True,
                row, column, self._grid.min_x_m, self._grid.max_y_m,
                self._grid.resolution_m)
        except (RuntimeError, ValueError) as error:
            raise CudaRuntimeError(
                f'CUDA tile render failed at ({row}, {column}): {error}') from error
        self._render_elapsed += time.perf_counter() - started
        self._rendered_tiles += 1
        products = [row, column, image, owner, coverage]
        if auxiliary:
            products.append(uncertainty)
        products.append(quality)
        # CPU cache counters occupy the final four protocol fields. CUDA image
        # residency is recorded once in summary(), not misreported as an LRU.
        return (*products, 0, 0, 0.0, 0)

    def map(self, _function, tasks, chunksize=1):  # noqa: A003
        del _function, chunksize
        return map(self._render, tasks)

    def summary(self) -> dict:
        return {
            **self._summary,
            'rendered_tiles': self._rendered_tiles,
            'empty_tiles': self._empty_tiles,
            'render_and_download_elapsed_s': self._render_elapsed,
        }


def cuda_render_pool_factory():
    """Return the factory injected into the shared fusion publication path."""
    def factory(initial, optimized, grid, width, height):
        return CudaRenderPool(initial, optimized, grid, width, height)
    return factory
