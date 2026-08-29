// Copyright 2026 jerry
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Python bindings for the shared wall transform.
//
// The Python signatures below are the ones the package already published:
// quaternions are (x, y, z, w) tuples and vectors are (x, y, z) tuples, and
// every function returns a tuple, not a bound object. Existing callers are
// meant to keep working untouched -- the point of binding rather than
// reimplementing is that there stays exactly one copy of this convention.

#include <array>
#include <map>
#include <stdexcept>
#include <string>

#include "climbot_description/geometry.hpp"
#include "climbot_description/wall_frame.hpp"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

namespace py = pybind11;
using climbot_description::Quaternion;
using climbot_description::Vector3;
using climbot_description::WallFrame;

namespace
{

Quaternion toQuaternion(const py::sequence & value)
{
  if (py::len(value) != 4U) {
    throw std::invalid_argument("a quaternion needs four values");
  }
  return Quaternion{
    value[0].cast<double>(), value[1].cast<double>(),
    value[2].cast<double>(), value[3].cast<double>()};
}

Vector3 toVector(const py::sequence & value)
{
  if (py::len(value) != 3U) {
    throw std::invalid_argument("a vector needs three values");
  }
  return Vector3{
    value[0].cast<double>(), value[1].cast<double>(), value[2].cast<double>()};
}

py::tuple fromQuaternion(const Quaternion & value)
{
  return py::make_tuple(value.x, value.y, value.z, value.w);
}

py::tuple fromVector(const Vector3 & value)
{
  return py::make_tuple(value.x, value.y, value.z);
}

std::array<double, 3> toTriple(const py::sequence & value, const char * name)
{
  if (py::len(value) != 3U) {
    throw std::invalid_argument(
            std::string("Wall ") + name + " needs three values.");
  }
  return {value[0].cast<double>(), value[1].cast<double>(), value[2].cast<double>()};
}

}  // namespace

PYBIND11_MODULE(_climbot_description, module)
{
  module.doc() = "Shared wall transform and quaternion helpers.";

  module.def("wrap_angle", &climbot_description::wrapAngle, py::arg("angle"));
  module.def(
    "quaternion_multiply", [](const py::sequence & first, const py::sequence & second) {
      return fromQuaternion(
        climbot_description::quaternionMultiply(toQuaternion(first), toQuaternion(second)));
    }, py::arg("first"), py::arg("second"));
  module.def(
    "quaternion_conjugate", [](const py::sequence & quaternion) {
      return fromQuaternion(climbot_description::quaternionConjugate(toQuaternion(quaternion)));
    }, py::arg("quaternion"));
  module.def(
    "quaternion_from_rpy", [](double roll, double pitch, double yaw) {
      return fromQuaternion(climbot_description::quaternionFromRpy(roll, pitch, yaw));
    }, py::arg("roll"), py::arg("pitch"), py::arg("yaw"));
  module.def(
    "rotate_vector", [](const py::sequence & quaternion, const py::sequence & vector) {
      return fromVector(
        climbot_description::rotateVector(toQuaternion(quaternion), toVector(vector)));
    }, py::arg("quaternion"), py::arg("vector"));
  module.def(
    "yaw_from_quaternion", [](const py::sequence & quaternion) {
      return climbot_description::yawFromQuaternion(toQuaternion(quaternion));
    }, py::arg("quaternion"));

  py::class_<WallFrame>(module, "WallFrame")
  .def(
    py::init(
      [](const py::sequence & origin_xyz, const py::sequence & origin_rpy,
      const py::object & surface) {
        std::map<std::string, double> values;
        if (!surface.is_none()) {
          for (const auto & item : surface.cast<py::dict>()) {
            try {
              values[item.first.cast<std::string>()] = item.second.cast<double>();
            } catch (const py::cast_error &) {
              continue;
            }
          }
        }
        return WallFrame(
          toTriple(origin_xyz, "origin_xyz"), toTriple(origin_rpy, "origin_rpy"), values);
      }),
    py::arg("origin_xyz"), py::arg("origin_rpy"), py::arg("surface") = py::none())
  .def_static("from_yaml", &WallFrame::fromYaml, py::arg("path"))
  .def(
    "position_from_world", [](const WallFrame & self, const py::sequence & position) {
      return fromVector(self.positionFromWorld(toVector(position)));
    }, py::arg("position"))
  .def(
    "orientation_from_world", [](const WallFrame & self, const py::sequence & quaternion) {
      return fromQuaternion(self.orientationFromWorld(toQuaternion(quaternion)));
    }, py::arg("quaternion"))
  .def_property_readonly(
    "rotation_world_from_wall", [](const WallFrame & self) {
      return fromQuaternion(self.rotationWorldFromWall());
    })
  .def_property_readonly(
    "origin", [](const WallFrame & self) {
      const auto & value = self.origin();
      return py::make_tuple(value[0], value[1], value[2]);
    })
  .def_property_readonly(
    "roll_pitch_yaw", [](const WallFrame & self) {
      const auto & value = self.rollPitchYaw();
      return py::make_tuple(value[0], value[1], value[2]);
    })
  .def_property_readonly(
    "surface", [](const WallFrame & self) {return self.surface();});
}
