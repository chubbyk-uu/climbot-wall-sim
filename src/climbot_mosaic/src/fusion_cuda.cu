// Copyright 2026 jerry
// Licensed under the Apache License, Version 2.0.

#include <cuda_runtime.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "pybind11/numpy.h"
#include "pybind11/pybind11.h"

namespace py = pybind11;

namespace
{

constexpr int kMaximumCandidates = 64;
constexpr std::size_t kMemoryReserveBytes = 2ULL * 1024ULL * 1024ULL * 1024ULL;

void checkCuda(cudaError_t result, const char * operation)
{
  if (result != cudaSuccess) {
    throw std::runtime_error(
            std::string(operation) + " failed: " + cudaGetErrorString(result));
  }
}

__device__ float sourceValue(
  const std::uint8_t * images, int image, int width, int height, int x, int y)
{
  if (x < 0 || x >= width || y < 0 || y >= height) {
    return 0.0F;
  }
  return static_cast<float>(
    images[(static_cast<std::size_t>(image) * height + y) * width + x]);
}

__device__ float interiorValue(int width, int height, int x, int y)
{
  if (x < 0 || x >= width || y < 0 || y >= height) {
    return 0.0F;
  }
  return static_cast<float>(min(min(x + 1, width - x), min(y + 1, height - y)));
}

template<typename Reader>
__device__ float bilinear(float x, float y, Reader reader)
{
  const int x0 = static_cast<int>(floorf(x));
  const int y0 = static_cast<int>(floorf(y));
  const float fx = x - static_cast<float>(x0);
  const float fy = y - static_cast<float>(y0);
  const float top = reader(x0, y0) * (1.0F - fx) + reader(x0 + 1, y0) * fx;
  const float bottom = reader(x0, y0 + 1) * (1.0F - fx) + reader(x0 + 1, y0 + 1) * fx;
  return top * (1.0F - fy) + bottom * fy;
}

__global__ void renderKernel(
  const std::uint8_t * images, const double * inverse, const std::uint16_t * frame_indices,
  const double * centers, const double * posterior, int candidate_count,
  int width, int height, int tile_size,
  bool auxiliary, bool quality, int tile_row, int tile_column,
  double grid_min_x, double grid_max_y, double resolution,
  std::uint8_t * output, std::uint16_t * owner, std::uint16_t * coverage,
  float * uncertainty, unsigned long long * quality_histogram,
  double * quality_sum, unsigned long long * quality_count)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= tile_size || y >= tile_size) {
    return;
  }
  const int pixel = y * tile_size + x;
  float best = 0.0F;
  std::uint8_t selected = 0;
  std::uint16_t selected_owner = 0;
  float selected_uncertainty = nanf("");
  std::uint16_t seen = 0;
  float value_sum = 0.0F;
  float value_square_sum = 0.0F;
  for (int candidate = 0; candidate < candidate_count; ++candidate) {
    const double * matrix = inverse + candidate * 9;
    const double denominator = matrix[6] * x + matrix[7] * y + matrix[8];
    if (denominator == 0.0) {
      continue;
    }
    const double source_x =
      (matrix[0] * x + matrix[1] * y + matrix[2]) / denominator;
    const double source_y =
      (matrix[3] * x + matrix[4] * y + matrix[5]) / denominator;
    const int nearest_x = __double2int_rn(source_x);
    const int nearest_y = __double2int_rn(source_y);
    if (nearest_x < 0 || nearest_x >= width || nearest_y < 0 || nearest_y >= height) {
      continue;
    }
    ++seen;
    // OpenCV INTER_LINEAR quantizes interpolation coordinates to 1/32 pixel.
    // Keep that contract so CUDA changes execution, not hard-cut ownership.
    const float sx = static_cast<float>(nearbyint(source_x * 32.0) / 32.0);
    const float sy = static_cast<float>(nearbyint(source_y * 32.0) / 32.0);
    const float value_float = bilinear(sx, sy, [=] __device__ (int px, int py) {
      return sourceValue(images, frame_indices[candidate], width, height, px, py);
    });
    const std::uint8_t value = static_cast<std::uint8_t>(
      min(255, max(0, __float2int_rn(value_float))));
    if (quality) {
      value_sum += value;
      value_square_sum += static_cast<float>(value) * value;
    }
    const float priority = bilinear(sx, sy, [=] __device__ (int px, int py) {
      return interiorValue(width, height, px, py);
    });
    if (priority > best) {
      best = priority;
      selected = value;
      selected_owner = static_cast<std::uint16_t>(frame_indices[candidate] + 1);
      if (auxiliary) {
        // The CPU reference intentionally builds these grids as float32.
        // Match that arithmetic before the shared 0.01 mm quantizer.
        const float world_x = static_cast<float>(grid_min_x) +
          (static_cast<float>(tile_column + x) + 0.5F) * static_cast<float>(resolution);
        const float world_y = static_cast<float>(grid_max_y) -
          (static_cast<float>(tile_row + y) + 0.5F) * static_cast<float>(resolution);
        const float dx = world_x - static_cast<float>(centers[candidate * 2]);
        const float dy = world_y - static_cast<float>(centers[candidate * 2 + 1]);
        const float sx_std = static_cast<float>(posterior[candidate * 3]);
        const float sy_std = static_cast<float>(posterior[candidate * 3 + 1]);
        const float yaw_std = static_cast<float>(posterior[candidate * 3 + 2]);
        selected_uncertainty = sqrtf(
          sx_std * sx_std + sy_std * sy_std + yaw_std * yaw_std * (dx * dx + dy * dy));
      }
    }
  }
  output[pixel] = selected;
  owner[pixel] = selected_owner;
  coverage[pixel] = seen;
  uncertainty[pixel] = auxiliary && selected_owner ? selected_uncertainty : nanf("");
  if (quality && seen >= 2) {
    const float mean = value_sum / seen;
    const float disagreement = sqrtf(max(0.0F, value_square_sum / seen - mean * mean));
    const int bin = min(4095, max(0, __float2int_rn(disagreement * (4095.0F / 255.0F))));
    atomicAdd(quality_histogram + bin, 1ULL);
    atomicAdd(quality_sum, static_cast<double>(disagreement));
    atomicAdd(quality_count, 1ULL);
  }
}

class FusionCudaSession
{
public:
  FusionCudaSession(int frame_count, int width, int height, int tile_size)
  : frame_count_(frame_count), width_(width), height_(height), tile_size_(tile_size)
  {
    if (frame_count <= 0 || frame_count > 65535 || width <= 0 || height <= 0 ||
      tile_size <= 0)
    {
      throw std::invalid_argument("invalid CUDA fusion session dimensions");
    }
    int device_count = 0;
    checkCuda(cudaGetDeviceCount(&device_count), "cudaGetDeviceCount");
    if (device_count <= 0) {
      throw std::runtime_error("no CUDA-capable device is available");
    }
    checkCuda(cudaSetDevice(0), "cudaSetDevice");
    const std::size_t image_bytes =
      static_cast<std::size_t>(frame_count_) * width_ * height_;
    const std::size_t pixels = static_cast<std::size_t>(tile_size_) * tile_size_;
    const std::size_t working_bytes =
      kMaximumCandidates * (9 + 2 + 3) * sizeof(double) +
      kMaximumCandidates * sizeof(std::uint16_t) +
      pixels * (sizeof(std::uint8_t) + 2 * sizeof(std::uint16_t) + sizeof(float)) +
      4096 * sizeof(unsigned long long) + sizeof(double) + sizeof(unsigned long long);
    std::size_t free_bytes = 0;
    std::size_t total_bytes = 0;
    checkCuda(cudaMemGetInfo(&free_bytes, &total_bytes), "cudaMemGetInfo");
    (void)total_bytes;
    const std::size_t required_bytes = image_bytes + working_bytes;
    const std::size_t eighty_percent = free_bytes / 5 * 4;
    const std::size_t reserve_limit =
      free_bytes > kMemoryReserveBytes ? free_bytes - kMemoryReserveBytes : 0;
    if (required_bytes > std::min(eighty_percent, reserve_limit)) {
      throw std::runtime_error(
              "insufficient free CUDA memory while preserving the 20% / 2 GiB reserve");
    }
    try {
      allocate(reinterpret_cast<void **>(&images_), image_bytes, "images");
      allocate(reinterpret_cast<void **>(&inverse_), kMaximumCandidates * 9 * sizeof(double), "inverse");
      allocate(reinterpret_cast<void **>(&frame_indices_), kMaximumCandidates * sizeof(std::uint16_t), "indices");
      allocate(reinterpret_cast<void **>(&centers_), kMaximumCandidates * 2 * sizeof(double), "centers");
      allocate(reinterpret_cast<void **>(&posterior_), kMaximumCandidates * 3 * sizeof(double), "posterior");
      allocate(reinterpret_cast<void **>(&output_), pixels, "output");
      allocate(reinterpret_cast<void **>(&owner_), pixels * sizeof(std::uint16_t), "owner");
      allocate(reinterpret_cast<void **>(&coverage_), pixels * sizeof(std::uint16_t), "coverage");
      allocate(reinterpret_cast<void **>(&uncertainty_), pixels * sizeof(float), "uncertainty");
      allocate(reinterpret_cast<void **>(&quality_histogram_), 4096 * sizeof(unsigned long long),
        "quality histogram");
      allocate(reinterpret_cast<void **>(&quality_sum_), sizeof(double), "quality sum");
      allocate(reinterpret_cast<void **>(&quality_count_), sizeof(unsigned long long),
        "quality count");
    } catch (...) {
      release();
      throw;
    }
    uploaded_ = new bool[frame_count_]{};
  }

  FusionCudaSession(const FusionCudaSession &) = delete;
  FusionCudaSession & operator=(const FusionCudaSession &) = delete;

  ~FusionCudaSession()
  {
    release();
    delete[] uploaded_;
  }

  void upload(int index, py::array_t<std::uint8_t, py::array::c_style> image)
  {
    if (index < 0 || index >= frame_count_) {
      throw std::invalid_argument("frame upload index is outside the session");
    }
    const auto values = image.request();
    if (values.ndim != 2 || values.shape[0] != height_ || values.shape[1] != width_) {
      throw std::invalid_argument("uploaded CUDA frame has an unexpected shape");
    }
    const std::size_t bytes = static_cast<std::size_t>(width_) * height_;
    py::gil_scoped_release release_gil;
    checkCuda(cudaMemcpy(images_ + static_cast<std::size_t>(index) * bytes,
      values.ptr, bytes, cudaMemcpyHostToDevice), "frame upload");
    uploaded_[index] = true;
  }

  py::tuple render(
    py::array_t<double, py::array::c_style> inverse,
    py::array_t<std::uint16_t, py::array::c_style> frame_indices,
    py::array_t<double, py::array::c_style> centers,
    py::array_t<double, py::array::c_style> posterior,
    bool auxiliary, bool quality,
    int tile_row, int tile_column, double grid_min_x, double grid_max_y, double resolution)
  {
    const auto inverse_values = inverse.request();
    const auto index_values = frame_indices.request();
    const auto center_values = centers.request();
    const auto posterior_values = posterior.request();
    if (inverse_values.ndim != 3 || inverse_values.shape[1] != 3 || inverse_values.shape[2] != 3) {
      throw std::invalid_argument("inverse transforms must have shape (N, 3, 3)");
    }
    const int count = static_cast<int>(inverse_values.shape[0]);
    if (count <= 0 || count > kMaximumCandidates || index_values.ndim != 1 ||
      index_values.shape[0] != count || center_values.ndim != 2 ||
      center_values.shape[0] != count || center_values.shape[1] != 2 ||
      posterior_values.ndim != 2 || posterior_values.shape[0] != count ||
      posterior_values.shape[1] != 3 || !std::isfinite(grid_min_x) ||
      !std::isfinite(grid_max_y) || !std::isfinite(resolution) || resolution <= 0.0)
    {
      throw std::invalid_argument("invalid CUDA tile metadata");
    }
    const auto * indices = static_cast<const std::uint16_t *>(index_values.ptr);
    for (int index = 0; index < count; ++index) {
      if (indices[index] >= frame_count_ || !uploaded_[indices[index]]) {
        throw std::invalid_argument("CUDA tile references a frame that was not uploaded");
      }
    }
    py::array_t<std::uint8_t> output({tile_size_, tile_size_});
    py::array_t<std::uint16_t> owner({tile_size_, tile_size_});
    py::array_t<std::uint16_t> coverage({tile_size_, tile_size_});
    py::array_t<float> uncertainty({tile_size_, tile_size_});
    py::array_t<std::uint64_t> histogram(4096);
    double quality_sum = 0.0;
    unsigned long long quality_count = 0;
    const std::size_t pixels = static_cast<std::size_t>(tile_size_) * tile_size_;
    {
      py::gil_scoped_release release_gil;
      checkCuda(cudaMemcpy(inverse_, inverse_values.ptr, count * 9 * sizeof(double),
        cudaMemcpyHostToDevice), "inverse upload");
      checkCuda(cudaMemcpy(frame_indices_, index_values.ptr, count * sizeof(std::uint16_t),
        cudaMemcpyHostToDevice), "index upload");
      checkCuda(cudaMemcpy(centers_, center_values.ptr, count * 2 * sizeof(double),
        cudaMemcpyHostToDevice), "center upload");
      checkCuda(cudaMemcpy(posterior_, posterior_values.ptr, count * 3 * sizeof(double),
        cudaMemcpyHostToDevice), "posterior upload");
      checkCuda(cudaMemset(quality_histogram_, 0, 4096 * sizeof(unsigned long long)),
        "quality histogram reset");
      checkCuda(cudaMemset(quality_sum_, 0, sizeof(double)), "quality sum reset");
      checkCuda(cudaMemset(quality_count_, 0, sizeof(unsigned long long)), "quality count reset");
      const dim3 block(16, 16);
      const dim3 grid((tile_size_ + 15) / 16, (tile_size_ + 15) / 16);
      renderKernel<<<grid, block>>>(
        images_, inverse_, frame_indices_, centers_, posterior_, count,
        width_, height_, tile_size_, auxiliary, quality,
        tile_row, tile_column, grid_min_x, grid_max_y, resolution,
        output_, owner_, coverage_, uncertainty_, quality_histogram_,
        quality_sum_, quality_count_);
      checkCuda(cudaGetLastError(), "CUDA fusion kernel launch");
      checkCuda(cudaDeviceSynchronize(), "CUDA fusion kernel execution");
      checkCuda(cudaMemcpy(output.mutable_data(), output_, pixels, cudaMemcpyDeviceToHost),
        "mosaic tile download");
      checkCuda(cudaMemcpy(owner.mutable_data(), owner_, pixels * sizeof(std::uint16_t),
        cudaMemcpyDeviceToHost), "owner tile download");
      checkCuda(cudaMemcpy(coverage.mutable_data(), coverage_, pixels * sizeof(std::uint16_t),
        cudaMemcpyDeviceToHost), "coverage tile download");
      checkCuda(cudaMemcpy(uncertainty.mutable_data(), uncertainty_, pixels * sizeof(float),
        cudaMemcpyDeviceToHost), "uncertainty tile download");
      checkCuda(cudaMemcpy(histogram.mutable_data(), quality_histogram_,
        4096 * sizeof(unsigned long long), cudaMemcpyDeviceToHost),
        "quality histogram download");
      checkCuda(cudaMemcpy(&quality_sum, quality_sum_, sizeof(double), cudaMemcpyDeviceToHost),
        "quality sum download");
      checkCuda(cudaMemcpy(&quality_count, quality_count_, sizeof(unsigned long long),
        cudaMemcpyDeviceToHost), "quality count download");
    }
    py::dict quality_result;
    quality_result["histogram"] = histogram;
    quality_result["sum"] = quality_sum;
    quality_result["count"] = quality_count;
    return py::make_tuple(output, owner, coverage, uncertainty, quality_result);
  }

  std::size_t imageBytes() const
  {
    return static_cast<std::size_t>(frame_count_) * width_ * height_;
  }

private:
  void allocate(void ** pointer, std::size_t bytes, const char * name)
  {
    const cudaError_t result = cudaMalloc(pointer, bytes);
    if (result != cudaSuccess) {
      throw std::runtime_error(
              std::string("CUDA allocation for ") + name + " failed: " + cudaGetErrorString(result));
    }
  }

  void release() noexcept
  {
    cudaFree(images_); cudaFree(inverse_); cudaFree(frame_indices_); cudaFree(centers_);
    cudaFree(posterior_); cudaFree(output_); cudaFree(owner_); cudaFree(coverage_);
    cudaFree(uncertainty_); cudaFree(quality_histogram_); cudaFree(quality_sum_);
    cudaFree(quality_count_);
    images_ = nullptr; inverse_ = nullptr; frame_indices_ = nullptr; centers_ = nullptr;
    posterior_ = nullptr; output_ = nullptr; owner_ = nullptr; coverage_ = nullptr;
    uncertainty_ = nullptr; quality_histogram_ = nullptr; quality_sum_ = nullptr;
    quality_count_ = nullptr;
  }

  std::uint8_t * images_{};
  double * inverse_{};
  std::uint16_t * frame_indices_{};
  double * centers_{};
  double * posterior_{};
  std::uint8_t * output_{};
  std::uint16_t * owner_{};
  std::uint16_t * coverage_{};
  float * uncertainty_{};
  unsigned long long * quality_histogram_{};
  double * quality_sum_{};
  unsigned long long * quality_count_{};
  bool * uploaded_{};
  int frame_count_{};
  int width_{};
  int height_{};
  int tile_size_{};
};

py::dict deviceInfo()
{
  int count = 0;
  checkCuda(cudaGetDeviceCount(&count), "cudaGetDeviceCount");
  if (count <= 0) {
    throw std::runtime_error("no CUDA-capable device is available");
  }
  cudaDeviceProp properties{};
  checkCuda(cudaGetDeviceProperties(&properties, 0), "cudaGetDeviceProperties");
  int runtime_version = 0;
  int driver_version = 0;
  checkCuda(cudaRuntimeGetVersion(&runtime_version), "cudaRuntimeGetVersion");
  checkCuda(cudaDriverGetVersion(&driver_version), "cudaDriverGetVersion");
  std::size_t free_memory = 0;
  std::size_t total_memory = 0;
  checkCuda(cudaMemGetInfo(&free_memory, &total_memory), "cudaMemGetInfo");
  py::dict result;
  result["device_count"] = count;
  result["device_index"] = 0;
  result["device_name"] = std::string(properties.name);
  result["compute_capability"] =
    std::to_string(properties.major) + "." + std::to_string(properties.minor);
  result["total_memory_bytes"] = properties.totalGlobalMem;
  result["free_memory_bytes"] = free_memory;
  result["cuda_runtime_version"] = runtime_version;
  result["cuda_driver_version"] = driver_version;
  result["maximum_candidates_per_tile"] = kMaximumCandidates;
  return result;
}

}  // namespace

PYBIND11_MODULE(_fusion_cuda, module)
{
  module.doc() = "Custom CUDA hard-cut wall-mosaic tile renderer";
  module.def("device_info", &deviceInfo);
  py::class_<FusionCudaSession>(module, "FusionCudaSession")
  .def(py::init<int, int, int, int>(), py::arg("frame_count"), py::arg("width"),
    py::arg("height"), py::arg("tile_size"))
  .def("upload", &FusionCudaSession::upload, py::arg("index"), py::arg("image"))
  .def("render", &FusionCudaSession::render, py::arg("inverse"), py::arg("frame_indices"),
    py::arg("centers"), py::arg("posterior"), py::arg("auxiliary"),
    py::arg("quality"), py::arg("tile_row"),
    py::arg("tile_column"), py::arg("grid_min_x"), py::arg("grid_max_y"),
    py::arg("resolution"))
  .def_property_readonly("image_bytes", &FusionCudaSession::imageBytes);
}
