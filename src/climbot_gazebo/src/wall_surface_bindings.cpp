// Copyright 2026 jerry
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "climbot_gazebo/wall_surface.hpp"
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
namespace py = pybind11;
using Surface = climbot_gazebo::WallSurface;
using Array = py::array_t<double, py::array::c_style | py::array::forcecast>;
PYBIND11_MODULE(_climbot_surface, module)
{
  py::class_<Surface>(module, "WallSurface")
  .def(py::init<const std::string &>())
  .def("evaluate", [](const Surface & surface, Array u, Array v, int kind) {
      auto a = u.request(), b = v.request();
      if (a.shape != b.shape || kind < 0 || kind > 2) {
        throw py::value_error("surface arrays must have matching shapes and a valid query kind");
      }
      py::array_t<double> out(a.shape);auto dst = out.mutable_data();
      auto x = static_cast<const double *>(a.ptr), y = static_cast<const double *>(b.ptr);
      {py::gil_scoped_release release;
        for (py::ssize_t i = 0; i < a.size; ++i) {
          dst[i] = kind == 0 ? surface.height(x[i], y[i]) :
          (kind == 1 ? surface.roughness(x[i], y[i]) : surface.grooveDepth(x[i], y[i]));
        }
      }
      return out;
  });
}
