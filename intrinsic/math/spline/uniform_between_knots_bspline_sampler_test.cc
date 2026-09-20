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

#include "intrinsic/math/spline/uniform_between_knots_bspline_sampler.h"

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

TEST(UniformBetweenKnotsBSplineSamplerTest, FailsForEmptyKnotVector) {
  BSplineNd spline;
  UniformBetweenKnotsBSplineSampler sampler;

  EXPECT_THAT(
      sampler.GenerateCurveParameters(spline, /*reference_sampling_step=*/1.0),
      StatusIs(absl::StatusCode::kInternal,
               HasSubstr("Couldn't get the spline unique knots.")));
}

TEST(UniformBetweenKnotsBSplineSamplerTest, FailsForSingleUniqueKnots) {
  constexpr int kDegree = 1;
  constexpr int kPointDim = 3;
  const std::vector<double> knots = {1.0, 1.0, 1.0, 1.0};
  BSplineNd spline;
  ASSERT_TRUE(spline.Init(kDegree, knots.size(), kPointDim));
  ASSERT_TRUE(spline.SetKnotVector(knots));

  UniformBetweenKnotsBSplineSampler sampler;

  EXPECT_THAT(
      sampler.GenerateCurveParameters(spline, /*reference_sampling_step=*/1.0),
      StatusIs(absl::StatusCode::kInvalidArgument,
               HasSubstr("at least two unique knots")));
}

TEST(UniformBetweenKnotsBSplineSamplerTest,
     FailsForNonPositiveReferenceSamplingStep) {
  constexpr int kDegree = 3;
  constexpr int kPointDim = 3;
  const std::vector<double> knots = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0};
  BSplineNd spline;
  ASSERT_TRUE(spline.Init(kDegree, knots.size(), kPointDim));
  ASSERT_TRUE(spline.SetKnotVector(knots));

  UniformBetweenKnotsBSplineSampler sampler;

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

TEST(UniformBetweenKnotsBSplineSamplerTest,
     EnforcesMinimumSamplesPerKnotSpanWhenStepExceedsDomain) {
  constexpr int kDegree = 3;
  constexpr int kPointDim = 3;
  const std::vector<double> knots = {0.0, 0.0, 0.0, 0.0, 1.0,
                                     2.0, 2.0, 2.0, 2.0};
  BSplineNd spline;
  ASSERT_TRUE(spline.Init(kDegree, knots.size(), kPointDim));
  ASSERT_TRUE(spline.SetKnotVector(knots));

  UniformBetweenKnotsBSplineSampler sampler;
  constexpr double kLargeReferenceSamplingStep = 10.0;
  ASSERT_OK_AND_ASSIGN(
      const std::vector<double> samples,
      sampler.GenerateCurveParameters(spline, kLargeReferenceSamplingStep));

  // Every knot boundary must be sampled at least once (0.0, 1.0, 2.0).
  const std::vector<double> expected_samples = {0.0, 1.0, 2.0};
  EXPECT_THAT(samples,
              Pointwise(DoubleNear(kNumericTolerance), expected_samples));
}

TEST(UniformBetweenKnotsBSplineSamplerTest,
     SamplesUniformlyBetweenKnotsWithIntegerNumberOfSamples) {
  constexpr int kDegree = 3;
  constexpr int kPointDim = 3;
  const std::vector<double> knots = {0.0, 0.0, 0.0, 0.0, 2.0,
                                     4.0, 6.0, 6.0, 6.0, 6.0};
  BSplineNd spline;
  ASSERT_TRUE(spline.Init(kDegree, knots.size(), kPointDim));
  ASSERT_TRUE(spline.SetKnotVector(knots));

  UniformBetweenKnotsBSplineSampler sampler;
  constexpr double kReferenceSamplingStep = 1.0;
  ASSERT_OK_AND_ASSIGN(
      const std::vector<double> samples,
      sampler.GenerateCurveParameters(spline, kReferenceSamplingStep));

  const std::vector<double> expected_samples = {0.0, 1.0, 2.0, 3.0,
                                                4.0, 5.0, 6.0};
  EXPECT_THAT(samples,
              Pointwise(DoubleNear(kNumericTolerance), expected_samples));
}

TEST(UniformBetweenKnotsBSplineSamplerTest,
     SamplesUniformlyBetweenKnotsWhenStepNeedsAdjustment) {
  constexpr int kDegree = 3;
  constexpr int kPointDim = 3;
  // Non-uniform knot spans: [0.0, 2.0] (length 2.0) and [2.0, 8.0]
  // (length 6.0).
  const std::vector<double> knots = {0.0, 0.0, 0.0, 0.0, 2.0,
                                     8.0, 8.0, 8.0, 8.0};
  BSplineNd spline;
  ASSERT_TRUE(spline.Init(kDegree, knots.size(), kPointDim));
  ASSERT_TRUE(spline.SetKnotVector(knots));

  UniformBetweenKnotsBSplineSampler sampler;
  constexpr double kReferenceSamplingStep = 2.5;
  ASSERT_OK_AND_ASSIGN(
      const std::vector<double> samples,
      sampler.GenerateCurveParameters(spline, kReferenceSamplingStep));

  // Step is adjusted per span: span [0, 2] gets 1 step, span [2, 8] gets
  // 3 steps.
  const std::vector<double> expected_samples = {0.0, 2.0, 4.0, 6.0, 8.0};
  EXPECT_THAT(samples,
              Pointwise(DoubleNear(kNumericTolerance), expected_samples));
}

TEST(UniformBetweenKnotsBSplineSamplerTest,
     SamplesUniformlyInValidKnotSpansForUnclampedSpline) {
  constexpr int kDegree = 2;
  constexpr int kPointDim = 3;
  // Unclamped knot vector with degree 2:
  // Knots: {-2.0, -1.0, 0.0, 2.0, 4.0, 5.0, 6.0}
  // Valid domain: [knots[2], knots[4]] = [0.0, 4.0].
  // Unique knots in valid domain: {0.0, 2.0, 4.0}.
  const std::vector<double> knots = {-2.0, -1.0, 0.0, 2.0, 4.0, 5.0, 6.0};
  BSplineNd spline;
  ASSERT_TRUE(spline.Init(kDegree, knots.size(), kPointDim));
  ASSERT_TRUE(spline.SetKnotVector(knots));

  UniformBetweenKnotsBSplineSampler sampler;
  constexpr double kReferenceSamplingStep = 1.0;
  ASSERT_OK_AND_ASSIGN(
      const std::vector<double> samples,
      sampler.GenerateCurveParameters(spline, kReferenceSamplingStep));

  // Valid domain [0.0, 4.0] with spans [0.0, 2.0] and [2.0, 4.0], each
  // length 2.0. With reference step 1.0, each span produces samples at
  // step 1.0.
  const std::vector<double> expected_samples = {0.0, 1.0, 2.0, 3.0, 4.0};
  EXPECT_THAT(samples,
              Pointwise(DoubleNear(kNumericTolerance), expected_samples));
}

}  // namespace
}  // namespace intrinsic
