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

#include "intrinsic/geometry/processing/processors/isotropic_remeshing_processor.h"

#include <memory>
#include <vector>

#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "intrinsic/geometry/api/construct_isotropic_remeshing.h"
#include "intrinsic/geometry/api/geometry.h"
#include "intrinsic/geometry/processing/pipeline_config.pb.h"
#include "intrinsic/longrunning/cc/operation_context.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::geo {
absl::StatusOr<std::unique_ptr<IsotropicRemeshingGeometryProcessor>>
IsotropicRemeshingGeometryProcessor::Create(
    const intrinsic_proto::geometry::IsotropicRemeshingProcessorConfig&
        config) {
  if (config.edge_reduction_factor() <= 0.0 ||
      config.edge_reduction_factor() > 1.0) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Edge reduction factor must be between 0.0 and 1.0 but "
                        "%f was provided.",
                        config.edge_reduction_factor()));
  }
  constexpr int kMaxNumIterations = 1000;
  if (config.num_iterations() <= 0 ||
      config.num_iterations() >= kMaxNumIterations) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Number of iterations must be positive and less than "
                        "%d but %d was provided.",
                        kMaxNumIterations, config.num_iterations()));
  }

  return absl::WrapUnique(new IsotropicRemeshingGeometryProcessor(
      config.edge_reduction_factor(), config.num_iterations()));
}

IsotropicRemeshingGeometryProcessor::IsotropicRemeshingGeometryProcessor(
    double edge_reduction_factor, int num_iterations)
    : edge_reduction_factor_(edge_reduction_factor),
      num_iterations_(num_iterations) {}

absl::StatusOr<std::vector<Geometry>>
IsotropicRemeshingGeometryProcessor::Process(
    longrunning::OperationContext& context, const Geometry& input) const {
  INTR_ASSIGN_OR_RETURN(Geometry result,
                        ConstructIsotropicRemeshing(
                            input, edge_reduction_factor_, num_iterations_));
  context.AddProgress(1.0);
  return std::vector<Geometry>{std::move(result)};
}

}  // namespace intrinsic::geo
