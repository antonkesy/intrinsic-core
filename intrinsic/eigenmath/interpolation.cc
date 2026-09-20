// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "intrinsic/eigenmath/interpolation.h"

#include <vector>

#include "intrinsic/eigenmath/types.h"

namespace intrinsic {
namespace eigenmath {

// Divides a series of segments using linear interpolation so that the norm
// between two successive points are smaller than `step`.
std::vector<eigenmath::VectorXd> Interpolate(
    const std::vector<eigenmath::VectorXd>& segments, double step) {
  if (segments.empty()) return {};
  std::vector<eigenmath::VectorXd> out = {segments.front()};
  for (int i = 1; i < segments.size(); i++) {
    const eigenmath::VectorXd& next = segments.at(i);
    while ((out.back() - next).norm() > step) {
      eigenmath::VectorXd delta = next - out.back();
      out.push_back(out.back() + delta * step / delta.norm());
    }
    if (!out.back().isApprox(next)) {
      out.push_back(next);
    }
  }
  return out;
}

}  // namespace eigenmath
}  // namespace intrinsic
