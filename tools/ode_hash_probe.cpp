// Copyright 2026 jerry
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Two overlapping primitives isolate broad-phase failure without ROS or Gazebo.
#include <ode/ode.h>
#include <array>
#include <iostream>
#include <string>
struct Counts {int candidates = 0;int contacts = 0;};
void candidate(void * data, dGeomID a, dGeomID b)
{
  auto & counts = *static_cast<Counts *>(data);++counts.candidates;
  dContactGeom contact[8];
  counts.contacts += dCollide(a, b, 8, contact, sizeof(dContactGeom));
}
int main(int argc, char ** argv)
{
  if (!dInitODE2(0)) {return 2;}
  const bool diagnostic = argc == 2 && std::string(argv[1]) == "--diagnostic";
  bool passed = true;bool comma = false;
  std::cout << "{\"cases\":[";
  const std::array<std::array<double, 3>, 5> offsets{{
    {{0, 0, 0}}, {{2, 0, 0}}, {{0, 2, 0}}, {{0, 0, 2}}, {{-2, -2, -2}}}};
  for (std::size_t i = 0; i < offsets.size(); ++i) {
    for (bool simple : {false, true}) {
      auto space = simple ? dSimpleSpaceCreate(nullptr) : dHashSpaceCreate(nullptr);
      if (!simple) {dHashSpaceSetLevels(space, -2, 8);}
      const auto & offset = offsets[i];
      auto planeBox = dCreateBox(space, .8, .8, .052);
      dGeomSetPosition(planeBox, offset[0], offset[1], offset[2] - .0245);
      auto sphere = dCreateSphere(space, .0025);
      dGeomSetPosition(sphere, offset[0] + .375, offset[1] + .375, offset[2] + .0032);
      Counts counts;dSpaceCollide(space, &counts, candidate);
      passed = passed && counts.candidates == 1 && counts.contacts > 0;
      if (comma) {std::cout << ',';} comma = true;
      std::cout << "{\"offset_case\":" << i << ",\"space\":\"" << (simple ? "simple" : "hash") <<
        "\",\"candidates\":" << counts.candidates << ",\"contacts\":" << counts.contacts << '}';
      dSpaceDestroy(space);
    }
  }
  std::cout << "],\"passed\":" << (passed ? "true" : "false") << "}\n";
  dCloseODE();return passed || diagnostic ? 0 : 1;
}
