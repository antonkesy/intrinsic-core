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

#include "intrinsic/math/spline/uniform_bspline_sampler.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <vector>

#include "absl/status/status.h"
#include "absl/status/status_matchers.h"
#include "internal/testing.h"
#include "intrinsic/math/spline/bspline.h"

namespace intrinsic {
namespace {

using ::absl_testing::StatusIs;
using ::testing::DoubleNear;
using ::testing::HasSubstr;
using ::testing::Pointwise;

constexpr double kNumericTolerance = 1.0e-10;

TEST(UniformBSplineSamplerTest, FailsForEmptyKnotVector) {
  BSplineNd spline_without_knots;
  UniformBSplineSampler sampler;

  EXPECT_THAT(sampler.GenerateCurveParameters(spline_without_knots,
                                              /*reference_sampling_step=*/1.0),
              StatusIs(absl::StatusCode::kInternal,
                       HasSubstr("Couldn't get the spline unique knots.")));
}

TEST(UniformBSplineSamplerTest, FailsForZeroLengthKnotVector) {
  constexpr int kDegree = 3;
  constexpr int kPointDim = 3;
  const std::vector<double> zero_length_knot_vector = {1.0, 1.0, 1.0, 1.0,
                                                       1.0, 1.0, 1.0, 1.0};
  BSplineNd spline;
  ASSERT_TRUE(spline.Init(kDegree, zero_length_knot_vector.size(), kPointDim));
  ASSERT_TRUE(spline.SetKnotVector(zero_length_knot_vector));

  UniformBSplineSampler sampler;

  EXPECT_THAT(
      sampler.GenerateCurveParameters(spline, /*reference_sampling_step=*/1.0),
      StatusIs(absl::StatusCode::kInvalidArgument,
               HasSubstr("at least two unique knots.")));
}

TEST(UniformBSplineSamplerTest, FailsForNonPositiveReferenceSamplingStep) {
  constexpr int kDegree = 3;
  constexpr int kPointDim = 3;
  const std::vector<double> knots = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0};
  BSplineNd spline;
  ASSERT_TRUE(spline.Init(kDegree, knots.size(), kPointDim));
  ASSERT_TRUE(spline.SetKnotVector(knots));

  UniformBSplineSampler sampler;

  EXPECT_THAT(
      sampler.GenerateCurveParameters(spline, /*reference_sampling_step=*/0.0),
      StatusIs(
          absl::StatusCode::kInvalidArgument,
          HasSubstr("The reference sampling step must be strictly positive.")));
  EXPECT_THAT(
      sampler.GenerateCurveParameters(spline, /*reference_sampling_step=*/-1.0),
      StatusIs(
          absl::StatusCode::kInvalidArgument,
          HasSubstr("The reference sampling step must be strictly positive.")));
}

TEST(UniformBSplineSamplerTest, EnforcesMinimumTwoSamples) {
  constexpr int kDegree = 3;
  constexpr int kPointDim = 3;
  const std::vector<double> knots = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0};
  BSplineNd spline;
  ASSERT_TRUE(spline.Init(kDegree, knots.size(), kPointDim));
  ASSERT_TRUE(spline.SetKnotVector(knots));

  // A sampling step larger than the knot domain size.
  constexpr double kLargeReferenceSamplingStep = 10.0;
  UniformBSplineSampler sampler;
  ASSERT_OK_AND_ASSIGN(
      const std::vector<double> samples,
      sampler.GenerateCurveParameters(spline, kLargeReferenceSamplingStep));

  // Should return exactly 2 samples: start and end knot.
  const std::vector<double> expected_samples = {0.0, 1.0};
  EXPECT_THAT(samples,
              Pointwise(DoubleNear(kNumericTolerance), expected_samples));
}

TEST(UniformBSplineSamplerTest, SamplesUniformlyWithIntegerNumberOfSamples) {
  constexpr int kDegree = 3;
  constexpr int kPointDim = 3;
  const std::vector<double> knots = {0.0, 0.0,  0.0,  0.0,  1.0, 2.0,
                                     3.0, 4.0,  5.0,  6.0,  7.0, 8.0,
                                     9.0, 10.0, 10.0, 10.0, 10.0};
  BSplineNd spline;
  ASSERT_TRUE(spline.Init(kDegree, knots.size(), kPointDim));
  ASSERT_TRUE(spline.SetKnotVector(knots));

  UniformBSplineSampler sampler;
  constexpr double kReferenceSamplingStep = 1.0;
  ASSERT_OK_AND_ASSIGN(
      const std::vector<double> samples,
      sampler.GenerateCurveParameters(spline, kReferenceSamplingStep));

  const std::vector<double> expected_samples = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0,
                                                6.0, 7.0, 8.0, 9.0, 10.0};
  EXPECT_THAT(samples,
              Pointwise(DoubleNear(kNumericTolerance), expected_samples));
}

TEST(UniformBSplineSamplerTest, SamplesUniformlyWhenStepNeedsAdjustment) {
  constexpr int kDegree = 3;
  constexpr int kPointDim = 3;
  const std::vector<double> knots = {0.0, 0.0,  0.0,  0.0,  1.0, 2.0,
                                     3.0, 4.0,  5.0,  6.0,  7.0, 8.0,
                                     9.0, 10.0, 10.0, 10.0, 10.0};
  BSplineNd spline;
  ASSERT_TRUE(spline.Init(kDegree, knots.size(), kPointDim));
  ASSERT_TRUE(spline.SetKnotVector(knots));

  UniformBSplineSampler sampler;
  constexpr double kReferenceSamplingStep = 3.0;
  ASSERT_OK_AND_ASSIGN(
      const std::vector<double> samples,
      sampler.GenerateCurveParameters(spline, kReferenceSamplingStep));

  // 10.0 / 3.0 = 3.333... rounded to 3 steps -> 4 samples
  const std::vector<double> expected_samples = {0.0, 10.0 / 3.0, 20.0 / 3.0,
                                                10.0};
  EXPECT_THAT(samples,
              Pointwise(DoubleNear(kNumericTolerance), expected_samples));
}

TEST(UniformBSplineSamplerTest, SucceedsForUnclampedSpline) {
  constexpr int kDegree = 3;
  constexpr int kPointDim = 3;
  const std::vector<double> knots = {0.0, 1.0, 2.0,  3.0,  4.0,
                                     5.0, 9.0, 10.0, 11.0, 12.0};
  BSplineNd spline;
  ASSERT_TRUE(spline.Init(kDegree, knots.size(), kPointDim));
  ASSERT_TRUE(spline.SetKnotVector(knots));

  UniformBSplineSampler sampler;
  constexpr double kReferenceSamplingStep = 3.0;
  ASSERT_OK_AND_ASSIGN(
      const std::vector<double> samples,
      sampler.GenerateCurveParameters(spline, kReferenceSamplingStep));

  // An unclamped spline has valid domain only in the range [`knots[kDegree]`,
  // `knots[knots.size() - kDegree` -1].
  const std::vector<double> expected_samples = {3.0, 6.0, 9.0};
  EXPECT_THAT(samples,
              Pointwise(DoubleNear(kNumericTolerance), expected_samples));
}

}  // namespace
}  // namespace intrinsic
