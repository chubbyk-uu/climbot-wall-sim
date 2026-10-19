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

// A diagnostic of ODE's face-normal rule at a triangle vertex, not a solver fix.
#include <ode/ode.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>

int main()
{
  if (!dInitODE2(0)) {return 2;}
  bool reproduced = true;
  std::cout << std::setprecision(17) << "{\"cases\":[";
  for (bool vertex : {false, true}) {
    std::array<dReal, 9> points = vertex ? std::array<dReal, 9>{
      0, 0, 0, .00001, 0, -.00008, 0, .00001, 0} : std::array<dReal, 9>{
      -.01, -.01, 0, .01, -.01, 0, 0, .01, 0};
    std::array<dTriIndex, 3> indices{0, 1, 2};
    auto data = dGeomTriMeshDataCreate();
    dGeomTriMeshDataBuildDouble(data, points.data(), 3*sizeof(dReal), 3,
      indices.data(), 3, 3*sizeof(dTriIndex));
    auto mesh = dCreateTriMesh(nullptr, data, nullptr, nullptr, nullptr);
    auto sphere = dCreateSphere(nullptr, .0001);
    dGeomSetPosition(sphere, 0, 0, .000099);
    dContactGeom contacts[8];
    const int count = dCollide(mesh, sphere, 8, contacts, sizeof(dContactGeom));
    double angle = -1;
    if (count) {
      const auto & normal = contacts[0].normal;
      const double length = std::sqrt(normal[0]*normal[0]+normal[1]*normal[1]+normal[2]*normal[2]);
      angle = std::acos(std::clamp(std::abs(normal[2])/length, 0.0, 1.0))*180/std::acos(-1.0);
    }
    reproduced = reproduced && count > 0 && (vertex ? angle > 80 : angle < 1e-6);
    if (vertex) {std::cout << ',';}
    std::cout << "{\"location\":\"" << (vertex ? "steep_face_vertex" : "flat_face_interior") <<
      "\",\"contacts\":" << count << ",\"normal_from_vertical_deg\":" << angle << '}';
    dGeomDestroy(sphere);dGeomDestroy(mesh);dGeomTriMeshDataDestroy(data);
  }
  std::cout << "],\"face_normal_rule_reproduced\":" << (reproduced ? "true" : "false") << "}\n";
  dCloseODE();return reproduced ? 0 : 1;
}
