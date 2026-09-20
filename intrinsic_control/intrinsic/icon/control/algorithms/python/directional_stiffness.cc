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


#include "intrinsic/icon/control/algorithms/directional_stiffness.h"

#include <pybind11/cast.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "pybind11/pybind11.h"
#include "pybind11_abseil/status_casters.h"

namespace intrinsic::icon {

namespace {

absl::StatusOr<eigenmath::Matrix6d> InternalComputeDirectionalStiffness(
    const eigenmath::Vector3d& translational_motion_direction,
    double translational_stiffness_orthogonal_to_motion_direction,
    double rotational_stiffness_orthogonal_to_motion_direction) {
  return ComputeDirectionalStiffness(
      {translational_motion_direction}, /*rotational_directions=*/{},
      {.translational_stiffness_orthogonal_to_motion_direction =
           translational_stiffness_orthogonal_to_motion_direction,
       .rotational_stiffness_orthogonal_to_motion_direction =
           rotational_stiffness_orthogonal_to_motion_direction});
}

}  // namespace

PYBIND11_MODULE(directional_stiffness, m) {
  pybind11::google::ImportStatusModule();

  m.def("compute_directional_stiffness", &InternalComputeDirectionalStiffness,
        pybind11::arg("motion_direction"),
        pybind11::arg("translational_stiffness_orthogonal_to_motion_direction"),
        pybind11::arg("rotational_stiffness_orthogonal_to_motion_direction"));
}

}  // namespace intrinsic::icon
