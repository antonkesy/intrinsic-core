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


#include "intrinsic/icon/control/algorithms/overdamping.h"

#include <pybind11/cast.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "pybind11/pybind11.h"
#include "pybind11_abseil/status_casters.h"

namespace intrinsic::icon {

namespace {

absl::StatusOr<eigenmath::MatrixNd> ComputeOverdampingAbsl(
    const eigenmath::MatrixNd& inertia, double environment_stiffness) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      eigenmath::MatrixNd damping,
      ComputeOverdamping(inertia, environment_stiffness));
  return damping;
}

}  // namespace

PYBIND11_MODULE(overdamping, m) {
  pybind11::google::ImportStatusModule();

  m.def("compute_overdamping", &ComputeOverdampingAbsl,
        pybind11::arg("inertia"), pybind11::arg("environment_stiffness"));
}

}  // namespace intrinsic::icon
