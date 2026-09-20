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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PARAMETRIC_CURVES_JOINT_PARAMETRIC_CURVE_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PARAMETRIC_CURVES_JOINT_PARAMETRIC_CURVE_H_

#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/motion_planning/path_planning/parametric_curves/parametric_curve.h"

namespace intrinsic {

// Holds the sampled `joint_configurations` at the
// `normalized_curve_parameters`.
struct JointConfigurationsAtNormalizedCurveParameters {
  std::vector<eigenmath::VectorNd> joint_configurations;
  std::vector<double> normalized_curve_parameters;
};

// An interface to define custom joint space curves as a function of a single
// curve parameter.
class JointParametricCurve
    : virtual public ParametricCurve<eigenmath::VectorNd> {
 public:
  ~JointParametricCurve() override = default;

  // Returns the length of the curve in radians.
  virtual double GetCurveLength() const = 0;

  // Samples the curve uniformly. The sampling step is normalized, i.e. the
  // sampling step is scaled by the curve length. The number of samples is
  // determined by the `normalized_sampling_step`, so that it produces an
  // integer number of samples. It returns the sampled joint configurations and
  // the normalized curve parameters. If `force_odd_number_of_samples` is true,
  // the total number of samples is forced to be odd.
  virtual absl::StatusOr<JointConfigurationsAtNormalizedCurveParameters>
  SampleUniformly(double normalized_sampling_step,
                  bool force_odd_number_of_samples) const = 0;

  // Samples the curve at the given `normalized_curve_parameter` in the
  // interval [0,1] and returns the curve sample and its derivatives.
  virtual absl::StatusOr<std::vector<eigenmath::VectorNd>>
  SampleWithDerivatives(double normalized_curve_parameter) const = 0;
};

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PARAMETRIC_CURVES_JOINT_PARAMETRIC_CURVE_H_
