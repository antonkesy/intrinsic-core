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

#include "intrinsic/kinematics/ik_solvers/wrist_offset_geometric_ik_solver.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>  // Debug only
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/dh_params.h"
#include "intrinsic/kinematics/ik_solvers/geometric_algebra_utils.h"
#include "intrinsic/kinematics/state.h"
#include "intrinsic/kinematics/state_values.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"
#include "vsr/space/vsr_cga3D_op.h"
#include "vsr/space/vsr_cga3D_round.h"
#include "vsr/space/vsr_cga3D_types.h"

using ::intrinsic::eigenmath::Quaterniond;
using ::intrinsic::eigenmath::Vector3d;
using vsr::cga::Circle;
using vsr::cga::Construct;
using vsr::cga::DualPlane;
using vsr::cga::DualSphere;
using vsr::cga::Line;
using vsr::cga::Pair;
using vsr::cga::Plane;
using vsr::cga::Pnt;
using vsr::cga::Round;
using vsr::cga::Sca;
using vsr::cga::Vec;

// #define SAVE_ANGLE_VALUES

namespace intrinsic {
namespace kinematics {

namespace {

// The number of initial sample around the circle centered at p5w.
constexpr int kNbCircleSamples = 200;
constexpr double kDefaultStep = 2 * M_PI / (kNbCircleSamples - 2);

// Tolerances
constexpr double kCosWristAngleEpsilon = 1e-6;
constexpr double kCosWristAngleTangentEpsilon = 3e-4;
constexpr double kCircleAngleEpsilon = 1e-6;
constexpr double kPntEpsilon = 1e-10;
constexpr double kP4A1Epsilon = 2e-4;
constexpr double kP2Epsilon = 5e-6;

// Special constants
constexpr uint8_t kAllBranch = std::numeric_limits<uint8_t>::max();

// Useful unit vectors.
const auto no = Vec(0, 0, 0).null();
const auto e1 = Vec(1, 0, 0);
const auto e2 = Vec(0, 1, 0);
const auto e3 = Vec(0, 0, 1);
const auto ni = vsr::cga::Inf(1);

struct IKPntSolution {
  double circle_angle;
  uint8_t branch;
  Pnt p1;
  Pnt p2;
  Pnt p3;
  Pnt p4;
};

bool operator==(const IKPntSolution& lhs, const IKPntSolution& rhs) {
  return (fabs((lhs.p1 <= rhs.p1)[0]) < kPntEpsilon &&
          fabs((lhs.p2 <= rhs.p2)[0]) < kPntEpsilon &&
          fabs((lhs.p3 <= rhs.p3)[0]) < kPntEpsilon &&
          fabs((lhs.p4 <= rhs.p4)[0]) < kPntEpsilon);
}

struct PointAngle {
  PointAngle(double new_p4_angle, double new_wrist_angle)
      : p4_angle(new_p4_angle), wrist_angle(new_wrist_angle) {}
  explicit PointAngle(double new_p4_angle)
      : p4_angle(new_p4_angle),
        wrist_angle(std::numeric_limits<double>::quiet_NaN()) {}
  PointAngle()
      : p4_angle(std::numeric_limits<double>::quiet_NaN()),
        wrist_angle(std::numeric_limits<double>::quiet_NaN()) {}

  double p4_angle;
  double wrist_angle;

  bool IsInvalid() const {
    return std::isnan(p4_angle) || std::isnan(wrist_angle);
  }
};

struct AngleSolutions {
  double circle_angle;
  std::vector<IKPntSolution> solutions;
};

// Given an angle on the sampled circle, get a p4 candidate.
Pnt SelectP4(const WristOffsetGeometricIKSolver::Parameters& parameters,
             const Pnt& p5w, const Pnt p6, double circle_angle) {
  Line line_56 = p5w ^ p6 ^ ni;

  DualSphere dS5w = Round::dls(p5w, parameters.dh_d5_z);

  Plane plane_5w = p5w ^ -line_56.dual() ^ ni;

  Circle p4_circle = -(-plane_5w.dual() ^ dS5w).dual();

  // Generate new p4. Rotate a plan around line_56 and find new potential p4
  // around the circle

  // Pick e2 if line56 is parallel to e1
  vsr::cga::DirectionVector vec_56_e1 =
      -vsr::cga::Flat::direction(line_56 ^ e1).dual();
  Vec vector_hor =
      (DirectionVectorToVector3d(vec_56_e1).norm() < 1e-4) ? e2 : e1;

  Plane plane_hor_56 = -line_56 ^ vector_hor;

  auto R = LineAngleToRotor(circle_angle, line_56);

  Plane plane_4s = R * plane_hor_56 * ~R;

  Pair Q4s = -(-plane_4s.dual() ^ -p4_circle.dual()).dual();
  QCHECK_GT(Round::size(Q4s, false), 0);

  Pnt p4 = Round::split(Q4s, true);

  return p4;
}

// Knowing the orientation of the robot with plane_1, get the two possible
// values of p1 that correspond to facing forward and backward.
std::array<Pnt, 2> SolveP1(
    const WristOffsetGeometricIKSolver::Parameters& parameters,
    const Plane& plane_1) {
  // An horizontal plane going through p1
  auto plane_1_hor =
      Construct::point(Vec(0, 0, fabs(parameters.dh[1].d))) ^ e1 ^ e2 ^ ni;

  // An sphere with at p0 with a radius of |p1-p0|
  auto dS0 = Round::dls(no, parameters.r01);

  // The intersection of the circle formed by plane_1 and dS0 and the horizontal
  // plane going through p1.
  Pair Q1 = -((-plane_1.dual() ^ dS0) ^ -plane_1_hor.dual()).dual();
  CHECK_GT(Round::size(Q1, false), 0) << "p1 is infeasible";

  // The two solution are facing forward or backward
  std::array<Pnt, 2> p1;
  p1[0] = Round::split(Q1, true);
  p1[1] = Round::split(Q1, false);

  return p1;
}

// Given p1 and p4, the solutions of p2 are found at the intersection of the
// circles in plane_1 and centered at p1 and p4.
bool SolveP2(const WristOffsetGeometricIKSolver::Parameters& parameters,
             const Plane& plane_1, const Pnt& p1s, const Pnt& p4s,
             std::vector<Pnt>* p2) {
  const double r12 = parameters.dh[2].a;

  auto dS1 = Round::dls(p1s, r12);
  auto dS4 = Round::dls(p4s, parameters.r24);

  // The intersection of the circle from the intersection of the sphere around
  // p1 and p4 and the plane going through A1 and p1.
  // If infeasible, the robot can't reach.
  auto dC2 = dS1 ^ dS4;
  double p2_error = Round::size(dC2, true);
  if (p2_error < -kP2Epsilon) {
    DVLOG(3) << "p2: Infeasible solution. Link out of reach: " << p2_error;
    return false;
  }

  Pair Q2 = -(dC2 ^ -plane_1.dual()).dual();
  if (Round::size(Q2, false) < -kP2Epsilon) {
    // If the link are within reach. They are expected to be crossing plane 1.
    // TODO(jeanfrancoisd): Maybe change to CHECK_GT
    LOG(ERROR) << "p2: Infeasible solution:" << Round::size(Q2, true);
    return false;
  }

  // The two solutions are elbow up or down
  p2->resize(2);
  (*p2)[0] = Round::split(Q2, true);
  (*p2)[1] = Round::split(Q2, false);

  return true;
}

// Given p2 and p4, the solutions of p3 are found at the intersection of the
// circles in plane_1 and centered at p2 and p4.
Pnt SolveP3(const WristOffsetGeometricIKSolver::Parameters& parameters,
            const Plane& plane_1, int kfb, const Pnt& p2s, const Pnt& p4s) {
  const double r23 = parameters.dh[3].a;
  const double r34 = fabs(parameters.dh[4].d);

  auto dS4_r34 = Round::dls(p4s, r34);
  auto dS2 = Round::dls(p2s, r23);

  // The dual circle formed by the intersection of the sphere around p2 and p4.
  auto dC3 = dS2 ^ dS4_r34;
  CHECK_GT(Round::size(dC3, true), 0)
      << "p3 can't be determined, but p2 is valid";

  // The point pair from intersecting the circle with plane 1.
  Pair Q3 = dC3 <= plane_1;
  CHECK_GT(Round::size(Q3, false), 0) << "p3 circle should intersect plane 1";

  // Only one solution is not in self-conflict
  auto p3_1 = Round::split(Q3, true);
  auto p3_2 = Round::split(Q3, false);

  // The plane going through p2, p4 and perpendicular to plane 1.
  auto plane_234 = p2s ^ p4s ^ -plane_1.dual() ^ ni;

  // Check if p3_1 is above or below the plane going through p2 and p4. This
  // choice depends on the robot facing forward or backward. If forward facing
  // requires p3 to be above p2-p4. Backward facing requires p3 to be below
  // p2-p4.
  auto test_p3 = (Sca(kfb) * p3_1) <= -(plane_234).dual();
  Pnt p3;
  if (test_p3[0] > 0) {
    p3 = p3_1;
  } else {
    p3 = p3_2;
  }

  return p3;
}

// Return the cosine of the angle formed by p3s,p4s,p5
double WristAngle(const Pnt& p3s, const Pnt& p4s, const Pnt& p5) {
  // Get angle between line_34 and line_45
  auto dline_45 = -(p4s ^ p5 ^ ni).dual();
  auto dline_34 = -(p3s ^ p4s ^ ni).dual();

  auto cos_angle_34_45 =
      (dline_34 <= dline_45) / (dline_34.norm() * dline_45.norm());

  return cos_angle_34_45[0];
}

// Compute the circle on which p3 much be located. It computed from the cone
// formed by right triangle p3, p3w, p4s.
//                     p5
//                     *--*
//                p4s /    p6
//       p3 *--------*
//          | `     /
//       p2 *   `  /
//                * p3w
Circle GetP3WristCircle(
    const WristOffsetGeometricIKSolver::Parameters& parameters, const Pnt& p4s,
    const Pnt& p5, const Pnt& p6) {
  Line line_54 = p5 ^ p4s ^ ni;

  double p3_cone_radius =
      fabs(parameters.dh[4].d) * sin(parameters.dh[5].alpha);
  double p3_cone_height =
      fabs(parameters.dh[4].d) * cos(parameters.dh[5].alpha);

  auto T = LineMagnitudeToTranslator(p3_cone_height, line_54);

  Pnt p3w = T * p4s * ~T;
  DualSphere dS3w = Round::dls(p3w, p3_cone_radius);

  Plane plane_3 = p3w ^ -line_54.dual() ^ ni;
  Circle circle_p3 = -(-plane_3.dual() ^ dS3w).dual();

  return circle_p3;
}

// Find p2 knowing p3 and p4. This done by translating p3 in a direction normal
// to plane_3.
//                     p5
//                     *--* p6
//       p3       p4s /
//   -------*--------*----------- plane_3
//          |
//       p2 *
Pnt SolveP2WithP4P3(const WristOffsetGeometricIKSolver::Parameters& parameters,
                    const Pnt& p3, const Pnt& p4s, const Plane& plane_1) {
  Line line_43 = p4s ^ p3 ^ ni;

  // A plane that is through pt3 and pt4, and perpendicular to plane_1
  Plane plane_3 = line_43 ^ -plane_1.dual();
  Line line_23 = p3 ^ -plane_3.dual() ^ ni;

  auto T32 = LineMagnitudeToTranslator(parameters.dh[3].a, line_23);

  Pnt p2 = T32 * p3 * ~T32;

  return p2;
}

// Solve the position of the points given a p4 candidate p4s for the
// special case where p4s is aligned on the A1 axis.
bool SolveP1WithP4Aligned(
    const WristOffsetGeometricIKSolver::Parameters& parameters, const Pnt& p4s,
    const Pnt& p5, const Pnt& p6, uint8_t want_branch,
    std::vector<IKPntSolution>* solutions) {
  DVLOG(2) << "SolveP1WithP4Aligned";

  // Ignore branch selection on A1 as they are the same.
  want_branch = want_branch | 0x0001;

  Circle p3_wrist_circle = GetP3WristCircle(parameters, p4s, p5, p6);

  // Pick any robot plane. Here, ZX plane is choose.
  auto robot_plane = no ^ e3 ^ e1 ^ ni;

  std::array<Pnt, 2> p1_all = SolveP1(parameters, robot_plane);

  // Consider only forward as forward and backward loose meaning when p4 is on
  // a1.
  size_t p1_select = 1;
  auto p1c = p1_all[p1_select];
  int kfb = p1_select == 0 ? 1 : -1;  // -1 means facing backward

  // Find p2 in the chosen plane
  std::vector<Pnt> p2_all;
  if (!SolveP2(parameters, robot_plane, p1c, p4s, &p2_all)) {
    return false;
  }

  for (size_t p2_select = 0; p2_select < 2; ++p2_select) {
    // Select solution for p2, elbow up or down
    Pnt p2c = p2_all[p2_select];

    // Find p3 in the chosen plane
    Pnt p3c = SolveP3(parameters, robot_plane, kfb, p2c, p4s);

    // Horizontal plane through p3
    Plane plane_3c = p3c ^ -(e3 ^ ni).dual();

    // Valid solution for p3 are the intersection of base circle of the
    // p3-p3w-p4 cone.
    Pair Q3 = -(-p3_wrist_circle.dual() ^ -plane_3c.dual()).dual();
    DVLOG(3) << "Q3 size = " << Round::size(Q3, false);
    if (Round::size(Q3, false) < 0) {
      DVLOG(1) << "p3: Infeasible solution. Link out of reach. Cone don't "
                  "intersect horizontal plane. p2_select="
               << p2_select;

      if ((want_branch != kAllBranch) && ((want_branch >> 1) == p2_select)) {
        // Return here since the desired branch is invalid
        return false;
      }
      continue;
    }

    std::array<Pnt, 2> p3_all;
    p3_all[0] = Round::split(Q3, true);
    p3_all[1] = Round::split(Q3, false);

    // Solve p1 and p2 for valid solution of p3
    for (size_t p3_select = 0; p3_select < 2; ++p3_select) {
      Pnt p3s = p3_all[p3_select];

      Plane plane_1 = no ^ e3 ^ p3s ^ ni;
      Pnt p2s = SolveP2WithP4P3(parameters, p3s, p4s, plane_1);

      std::array<Pnt, 2> p1_all = SolveP1(parameters, plane_1);
      Pnt p1s = p1_all[p1_select];
      double p1_p2_squared_distance = -2 * (p1s <= p2s)[0];
      double r12_diff = fabs(sqrt(p1_p2_squared_distance) - parameters.dh[2].a);
      DVLOG(3) << "r12_diff: " << r12_diff << ", p2_select=" << p2_select
               << ", p3_select=" << p3_select;
      if (r12_diff <= 1e-4) {
        DVLOG(2) << "Found solution with a4 aligned";
        // p1_select is always equal to 1.
        uint8_t branch = 1 + (p2_select << 1);

        if ((want_branch != kAllBranch) && (branch != want_branch)) {
          DVLOG(2) << "Branch " << int(branch) << " is not on desired branch "
                   << int(want_branch);
          // TODO(jeanfrancoisd): Quit earlier
          continue;
        }
        IKPntSolution solution({NAN, branch, p1s, p2s, p3s, p4s});
        solutions->push_back(solution);
      } else {
        DVLOG(2) << "p1: Infeasible solution. Link out of reach, p2_select="
                 << p2_select << ", p3_select=" << p3_select;
        // Check that we didn't made a mistake here. To remove when certain.
        p1s = p1_all[p1_select == 1 ? 0 : 1];
        p1_p2_squared_distance = -2 * (p1s <= p2s)[0];
        r12_diff = fabs(sqrt(p1_p2_squared_distance) - parameters.dh[2].a);
        CHECK_GT(r12_diff, 1e-4);
        continue;
      }
    }
  }

  return !(solutions->empty());
}

// Solve the position of the points given a p4 candidate p4s.
bool SolveIKWithP4(const WristOffsetGeometricIKSolver::Parameters& parameters,
                   const Pnt& p4s, const Pnt& p5, const Pnt& p6,
                   uint8_t want_branch, std::vector<IKPntSolution>* solutions,
                   double* p4_a1_distance) {
  solutions->clear();

  *p4_a1_distance = fabs(PointSquaredDistanceToLine(p4s, no ^ e3 ^ ni));
  if (sqrt(*p4_a1_distance) < kP4A1Epsilon) {
    // p4 is aligned on A1 axis. The orientation of plane_1 is therefore
    // undefined.
    return SolveP1WithP4Aligned(parameters, p4s, p5, p6, want_branch,
                                solutions);
  }

  auto plane_1 = no ^ e3 ^ p4s ^ ni;

  // Solve p1, known p4
  std::array<Pnt, 2> p1_all = SolveP1(parameters, plane_1);

  for (size_t p1_select = 0; p1_select < 2; ++p1_select) {
    // Select solution for p1, facing forward or backward
    auto p1s = p1_all[p1_select];
    int kfb = p1_select == 0 ? 1 : -1;  // kfb==-1 means facing backward

    std::vector<Pnt> p2_all;
    if (!SolveP2(parameters, plane_1, p1s, p4s, &p2_all)) {
      DVLOG(2) << "p1_select " << p1_select << " can't solve p2";
      continue;
    }

    for (size_t p2_select = 0; p2_select < 2; ++p2_select) {
      // Select solution for p2, elbow up or down
      auto p2s = p2_all[p2_select];

      // Find p3
      Pnt p3s = SolveP3(parameters, plane_1, kfb, p2s, p4s);

      uint8_t branch = p1_select + (p2_select << 1);
      if (want_branch != kAllBranch && branch != want_branch) {
        // TODO(jeanfrancoisd): Quit earlier
        continue;
      }
      IKPntSolution solution({NAN, branch, p1s, p2s, p3s, p4s});
      solutions->push_back(solution);
    }
  }

  return !(solutions->empty());
}

// Check if the wrist angle is crossing the target value.
bool IsCrossing(PointAngle point_a, PointAngle point_b, double target) {
  if (point_a.IsInvalid() && point_b.IsInvalid()) {
    return false;
  }
  // Check if one of the two points is invalid.
  if (point_a.IsInvalid() ^ point_b.IsInvalid()) {
    return true;
  }

  double fa = (point_a.wrist_angle - target);
  double fb = (point_b.wrist_angle - target);

  // We look for difference in sign.
  return (fa >= 0 && fb < 0) || (fa < 0 && fb >= 0);
}

// Find the kinematic solution for a given branch and return the evaluation of
// the wrist angle at a given circle angle.
std::pair<bool, PointAngle> ComputeWristAngleAtCircleAngle(
    const WristOffsetGeometricIKSolver::Parameters& parameters, const Pnt& p5w,
    const Pnt& p5, const Pnt& p6, uint8_t branch, double circle_angle,
    IKPntSolution* solution) {
  // Get a p4 candidate at the circle_angle.
  Pnt p4s = SelectP4(parameters, p5w, p6, circle_angle);

  double p4_a1_distance;

  std::vector<IKPntSolution> solutions;
  if (!SolveIKWithP4(parameters, p4s, p5, p6, branch, &solutions,
                     &p4_a1_distance) ||
      solutions.empty()) {
    return std::make_pair(false, PointAngle(circle_angle));
  }
  if (p4_a1_distance >= kP4A1Epsilon) {
    CHECK_EQ(solutions.size(), 1)
        << "Expecting a solution only for desired branch";
  } else {
    // There could be a symmetrical solution on the same branch when aligned on
    // A1.
    CHECK_LE(solutions.size(), 2)
        << "Expecting at most two solutions for desired branch";
    // TODO(jeanfrancoisd): How to choose between the two solutions
  }

  *solution = solutions[0];
  solution->circle_angle = circle_angle;

  double cos_p3_p4_p5 = WristAngle(solution->p3, solution->p4, p5);
  return std::make_pair(true, PointAngle({circle_angle, cos_p3_p4_p5}));
}

// Find the kinematic solution for which the wrist angle is minimum or maximum
// in the bounded region defined by point_a and point_c using a golden section
// search
std::pair<bool, IKPntSolution> RefineP4Tangent(
    const WristOffsetGeometricIKSolver::Parameters& parameters, const Pnt& p5w,
    const Pnt& p5, const Pnt& p6, const uint8_t branch,
    const PointAngle point_a, const PointAngle point_b,
    const PointAngle point_c) {
  const double R = (sqrt(5) - 1) / 2.0;
  const double C = 1 - R;

  std::pair<bool, PointAngle> r1;
  std::pair<bool, PointAngle> r2;
  IKPntSolution solution1;
  IKPntSolution solution2;

  double ax = point_a.p4_angle;
  double bx = point_b.p4_angle;
  double cx = point_c.p4_angle;

  double fa = point_a.wrist_angle;
  double fb = point_b.wrist_angle;
  double fc = point_c.wrist_angle;

  double x0 = ax;
  double x3 = cx;

  double x1, x2;
  double f1, f2;

  bool minimize;
  if (fb > fa) {
    CHECK_GE(fb, fc - 1e-10);
    // We maximize
    minimize = false;
  } else {
    CHECK_LE(fb, fc + 1e-10);
    // We minimize
    minimize = true;
  }

  if (fabs(cx - bx) > fabs(bx - ax)) {
    x1 = bx;
    f1 = fb;
    x2 = bx + C * (cx - bx);

    r2 = ComputeWristAngleAtCircleAngle(parameters, p5w, p5, p6, branch, x2,
                                        &solution2);
    if (!r2.first) {
      DVLOG(2) << "No solution for branch " << branch;
      return std::make_pair(false, IKPntSolution());
    }
    f2 = r2.second.wrist_angle;
  } else {
    x2 = bx;
    f2 = fb;
    x1 = bx - C * (bx - ax);

    r1 = ComputeWristAngleAtCircleAngle(parameters, p5w, p5, p6, branch, x1,
                                        &solution1);
    if (!r1.first) {
      DVLOG(2) << "No solution for branch " << branch;
      return std::make_pair(false, IKPntSolution());
    }
    f1 = r1.second.wrist_angle;
  }

  while (fabs(x3 - x0) > kCircleAngleEpsilon) {
    if ((f2 < f1) && minimize) {
      x0 = x1;
      x1 = x2;
      x2 = R * x1 + C * x3;
      f1 = f2;
      r2 = ComputeWristAngleAtCircleAngle(parameters, p5w, p5, p6, branch, x2,
                                          &solution2);
      if (!r2.first) {
        DVLOG(2) << "No solution for branch " << branch;
        return std::make_pair(false, IKPntSolution());
      }
      f2 = r2.second.wrist_angle;
    } else {
      x3 = x2;
      x2 = x1;
      x1 = R * x2 + C * x0;
      f2 = f1;
      r1 = ComputeWristAngleAtCircleAngle(parameters, p5w, p5, p6, branch, x1,
                                          &solution1);
      if (!r1.first) {
        DVLOG(2) << "No solution for branch " << branch;
        return std::make_pair(false, IKPntSolution());
      }
      f1 = r1.second.wrist_angle;
    }
  }

  if ((f2 < f1) && minimize) {
    CHECK(r2.first);
    if (fabs(r2.second.wrist_angle - parameters.cos_wrist_angle) <=
        kCosWristAngleTangentEpsilon) {
      return std::make_pair(true, solution2);
    } else {
      DVLOG(2) << "No solution for branch " << branch;
      return std::make_pair(false, IKPntSolution());
    }
  } else {
    CHECK(r1.first);
    if (fabs(r1.second.wrist_angle - parameters.cos_wrist_angle) <=
        kCosWristAngleTangentEpsilon) {
      return std::make_pair(true, solution1);
    } else {
      DVLOG(2) << "No solution for branch " << int(branch);
      return std::make_pair(false, IKPntSolution());
    }
  }

  return std::make_pair(false, IKPntSolution());
}

std::pair<bool, IKPntSolution> RefineP4CrossingWithInvalid(
    const WristOffsetGeometricIKSolver::Parameters& parameters, const Pnt& p5w,
    const Pnt& p5, const Pnt& p6, const uint8_t branch, const PointAngle& prev,
    const PointAngle& next) {
  constexpr int kMaxIter = 1e4;

  // Make sure x values are valid
  CHECK(!std::isnan(prev.wrist_angle) || !std::isnan(next.wrist_angle));
  CHECK(!std::isnan(prev.p4_angle) && !std::isnan(next.p4_angle));

  double x1;
  double x2;
  // Make x2 the invalid side
  if (std::isnan(prev.wrist_angle)) {
    x2 = prev.p4_angle;
    x1 = next.p4_angle;
  } else {
    x1 = prev.p4_angle;
    x2 = next.p4_angle;
  }

  IKPntSolution solution;
  std::pair<bool, PointAngle> r;
  r = ComputeWristAngleAtCircleAngle(parameters, p5w, p5, p6, branch, x1,
                                     &solution);
  CHECK(r.first);
  double f1 = r.second.wrist_angle - parameters.cos_wrist_angle;

  if (fabs(f1) > 0.1) {
    DVLOG(2) << "Starting point is too far";
    return std::make_pair(false, solution);
  }

  Pnt previous_p4s = solution.p4;

  double p4_distance_squared;
  double x;
  double f;
  for (int i = 0; i < kMaxIter; ++i) {
    x = x1 + (x2 - x1) / 2;
    r = ComputeWristAngleAtCircleAngle(parameters, p5w, p5, p6, branch, x,
                                       &solution);
    if (!r.first) {
      x2 = x;
      continue;
    } else {
      f = r.second.wrist_angle - parameters.cos_wrist_angle;
      if (fabs(f) < fabs(f1)) {
        x1 = x;
        f1 = f;
      } else {
        x2 = x;
      }

      DVLOG(2) << "x: " << x << ": " << f;

      // Stopping criteria
      p4_distance_squared = -2 * ((solution.p4 <= previous_p4s)[0]);
      if (fabs(p4_distance_squared) < kPntEpsilon / 10) {
        if (fabs(f) <= kCosWristAngleTangentEpsilon) {
          return std::make_pair(true, solution);
        } else {
          DVLOG(2) << "p4 didn't change and still no solution, wrist_angle="
                   << r.second.wrist_angle;
          return std::make_pair(false, solution);
        }
      }
      previous_p4s = solution.p4;
    }
  }
  LOG(ERROR) << "Maximum number of iteration reached.";
  return std::make_pair(false, solution);
}

// Refine the circle angle that is crossing the target value of the wrist angle
// using the false-position method
std::pair<bool, IKPntSolution> RefineP4Crossing(
    const WristOffsetGeometricIKSolver::Parameters& parameters, const Pnt& p5w,
    const Pnt& p5, const Pnt& p6, const uint8_t branch, const PointAngle& prev,
    const PointAngle& next, Pnt previous_p4s) {
  constexpr int kMaxIter = 1e4;

  double p4_distance_squared;
  IKPntSolution solution;
  std::pair<bool, PointAngle> r;

  double x1 = prev.p4_angle;
  double x2 = next.p4_angle;
  double f1 = prev.wrist_angle - parameters.cos_wrist_angle;
  double f2 = next.wrist_angle - parameters.cos_wrist_angle;

  if (fabs(f1) < kCosWristAngleEpsilon) {
    r = ComputeWristAngleAtCircleAngle(parameters, p5w, p5, p6, branch, x1,
                                       &solution);
    CHECK(r.first);
    return std::make_pair(true, solution);
  }
  if (fabs(f2) < kCosWristAngleEpsilon) {
    r = ComputeWristAngleAtCircleAngle(parameters, p5w, p5, p6, branch, x2,
                                       &solution);
    CHECK(r.first);
    return std::make_pair(true, solution);
  }

  // Make x1 the negative side
  if (f1 < 0) {
    x1 = prev.p4_angle;
    x2 = next.p4_angle;
  } else {
    x2 = prev.p4_angle;
    x1 = next.p4_angle;
    std::swap(f1, f2);
  }

  double x = 0;
  double f = 0;
  double dx = x2 - x1;
  for (int i = 0; i < kMaxIter; ++i) {
    x = x1 + dx * f1 / (f1 - f2);
    r = ComputeWristAngleAtCircleAngle(parameters, p5w, p5, p6, branch, x,
                                       &solution);
    if (!r.first) {
      LOG(ERROR) << "No solution for p4 angle = " << x;
      return std::make_pair(false, solution);
    }
    f = r.second.wrist_angle - parameters.cos_wrist_angle;

    if (f < 0) {
      x1 = x;
      f1 = f;
    } else {
      x2 = x;
      f2 = f;
    }
    dx = x2 - x1;

    // Stopping criteria
    p4_distance_squared = -2 * ((solution.p4 <= previous_p4s)[0]);
    if (fabs(p4_distance_squared) < kPntEpsilon) {
      if (fabs(f) <= kCosWristAngleTangentEpsilon) {
        return std::make_pair(true, solution);
      } else {
        DVLOG(3) << "p4 didn't change and still no solution";
        return std::make_pair(false, solution);
      }
    }
    previous_p4s = solution.p4;
  }

  LOG(ERROR) << "Maximum number of iteration reached.";
  return std::make_pair(false, solution);
}

eigenmath::Vector6dAligned GetJointAngles(
    const WristOffsetGeometricIKSolver::Parameters& parameters,
    IKPntSolution solution, const Pnt& p5, const Pnt& p6, const Pnt& p6z) {
  eigenmath::Vector6dAligned theta(6);

  const Pnt p01 = Construct::point(Vec(0, 0, parameters.dh[1].d));

  const Line Lz = Construct::line(0, 0, 1);
  const Line Lx = Construct::line(1, 0, 0);

  const Line L01 = Construct::line(p01, solution.p1);
  const Line L12 = Construct::line(solution.p1, solution.p2);
  const Line L23 = Construct::line(solution.p2, solution.p3);
  const Line L34 = Construct::line(solution.p3, solution.p4);
  const Line L45 = Construct::line(solution.p4, p5);
  const Line L56 = Construct::line(p5, p6);
  const Line L6z = Construct::line(p6, p6z);

  const Line p4_x = LinePerpendicularToLines(L34, L45);
  const Line p5_x = LinePerpendicularToLines(L56, L45);

  const Line p1_z = LinePerpendicularToLines(L01, Lz);

  theta[0] = AngleBetweenLines(Lx, L01, Lz);
  theta[1] = AngleBetweenLines(L01, L12, p1_z);
  theta[2] = AngleBetweenLines(L12, L34, p1_z);

  theta[3] = AngleBetweenLines(L23, p4_x, L34);
  theta[4] = AngleBetweenLines(p4_x, p5_x, L45);
  theta[5] = AngleBetweenLines(p5_x, L6z, L56);

  return theta;
}

std::pair<bool, IKPntSolution> CheckForCrossingSolution(
    const WristOffsetGeometricIKSolver::Parameters& parameters, const Pnt& p5w,
    const Pnt& p5, const Pnt& p6, const uint8_t branch,
    const PointAngle& wrist_angles_1, const PointAngle& wrist_angles_0,
    const IKPntSolution& current_solution) {
  if (wrist_angles_1.IsInvalid() && wrist_angles_0.IsInvalid()) {
    return std::make_pair(false, current_solution);
  }

  bool is_crossing =
      IsCrossing(wrist_angles_1, wrist_angles_0, parameters.cos_wrist_angle);
  if (is_crossing) {
    std::pair<bool, IKPntSolution> p4_solved;
    if (wrist_angles_1.IsInvalid() || wrist_angles_0.IsInvalid()) {
      p4_solved = RefineP4CrossingWithInvalid(parameters, p5w, p5, p6, branch,
                                              wrist_angles_1, wrist_angles_0);
    } else {
      p4_solved =
          RefineP4Crossing(parameters, p5w, p5, p6, branch, wrist_angles_1,
                           wrist_angles_0, current_solution.p4);
    }

    if (!p4_solved.first) {
      DVLOG(2) << "Convergence not achieved for p4";
      return std::make_pair(false, IKPntSolution());
    } else {
      return std::make_pair(true, p4_solved.second);
    }
  }
  return std::make_pair(false, IKPntSolution());
}

std::pair<bool, IKPntSolution> CheckForTangentSolution(
    const WristOffsetGeometricIKSolver::Parameters& parameters, const Pnt& p5w,
    const Pnt& p5, const Pnt& p6, const uint8_t branch,
    const PointAngle& wrist_angles_0, const PointAngle& wrist_angles_1,
    const PointAngle& wrist_angles_2, const IKPntSolution& current_solution) {
  if (wrist_angles_2.IsInvalid() || wrist_angles_1.IsInvalid() ||
      wrist_angles_0.IsInvalid()) {
    return std::make_pair(false, current_solution);
  }

  bool refine_tangent = false;
  if ((wrist_angles_2.wrist_angle >= wrist_angles_1.wrist_angle) &&
      (wrist_angles_0.wrist_angle >= wrist_angles_1.wrist_angle)) {
    // We have a minimum
    refine_tangent = true;
  } else if ((wrist_angles_2.wrist_angle <= wrist_angles_1.wrist_angle) &&
             (wrist_angles_0.wrist_angle <= wrist_angles_1.wrist_angle)) {
    // We have a maximum
    refine_tangent = true;
  }

  // Check only when close to target value
  if (refine_tangent &&
      (fabs(wrist_angles_0.wrist_angle - parameters.cos_wrist_angle) < 4e-2)) {
    auto p4_solved =
        RefineP4Tangent(parameters, p5w, p5, p6, branch, wrist_angles_2,
                        wrist_angles_1, wrist_angles_0);
    if (!p4_solved.first) {
      DVLOG(2) << "Convergence not achieved for tangent p4";
      return std::make_pair(false, current_solution);
    } else {
      return std::make_pair(true, p4_solved.second);
    }
  }

  return std::make_pair(false, current_solution);
}

// When p4 gets close to A1, the derivative are getting larger. As a result, we
// increase sampling.
double ComputeAngleIncrement(double p4_a1_distance) {
  if (p4_a1_distance < 2 * kP4A1Epsilon) {
    return kDefaultStep / 10;
  }
  return kDefaultStep;
}

bool AppendSolution(const IKPntSolution& solution, uint8_t branch,
                    std::vector<std::vector<IKPntSolution>>* valid_solutions) {
  for (const auto& valid_solution : *valid_solutions) {
    for (const auto& existing_solution : valid_solution) {
      if (existing_solution == solution) {
        DVLOG(3) << "Duplicate solution";
        return false;
      }
    }
  }
  (*valid_solutions)[branch].push_back(solution);
  return true;
}

bool IsUniqueJointSolution(
    const eigenmath::Vector6dAligned& new_joint_solution,
    const std::array<eigenmath::Vector6dAligned,
                     WristOffsetGeometricIKSolver::kSolutionBufferSize>&
        existing_joint_solutions) {
  // TODO(jeanfrancoisd) : Use the measure wrist angle to select the best one
  for (const auto& existing_solution : existing_joint_solutions) {
    if ((existing_solution - new_joint_solution).norm() < 1e-2) {
      DVLOG(3) << "Duplicate solution";
      return false;
    }
  }
  return true;
}

std::pair<bool, std::vector<IKPntSolution>> GetSolutionForBranch(
    uint8_t branch, const std::vector<IKPntSolution>& solutions) {
  std::vector<IKPntSolution> branch_solutions;
  for (const auto& solution : solutions) {
    if (branch == solution.branch) {
      branch_solutions.push_back(solution);
    }
  }

  return std::make_pair((!branch_solutions.empty()), branch_solutions);
}

// Heavily inspired from "Inverse Kinematics for Industrial Robots using
// Conformal Geometric Algebra" https://doi.org/10.4173/mic.2016.1.6
//
// We first solve for the position of the joints. They are labelled as follow:
//                        + p6z
//                 p5w p5
//                   + *--*
//                 p4 /    p6
//       p3 *--------*
//          |
//       p2 *
//          |
//          |
//          |
//  p01 *---* p1
//      |
//   p0 *
//
// p5w is the extension of the p5-p6 line to form a right triangle with
// p4,p5,p5w. p6z is used to define the orientation of the end effector.
//
// The strategy used here is to pick a candidate p4 through sampling the circle
// centered at p5w with p5-p6 normal and radius p5w-p4. Then, given that p4
// candidate, the other points are solved. The solution is considered valid if
// the angle formed by p3-p4-p5 respect the robot geometry. This angle is
// referred as wrist_angle.
int GeometricSolve(
    const WristOffsetGeometricIKSolver::Parameters& parameters, const Pnt& p5w,
    const Pnt& p5, const Pnt& p6, const Pnt& p6z,
    std::array<eigenmath::Vector6dAligned,
               WristOffsetGeometricIKSolver::kSolutionBufferSize>*
        joint_solutions) {
  constexpr size_t kNbBranches = 4;

  std::vector<std::vector<PointAngle>> wrist_angles;
  std::vector<std::vector<IKPntSolution>> valid_solutions(kNbBranches);
#ifdef SAVE_ANGLE_VALUES
  std::vector<AngleSolutions> all_solutions;
#endif

  double circle_angle = 0;
  double p4_a1_distance = std::numeric_limits<double>::max();

  // We go around the circle with an extra step to cover the discontinuity
  while (circle_angle <= 2 * M_PI + kDefaultStep) {
    double angle_increments = ComputeAngleIncrement(p4_a1_distance);
    circle_angle += angle_increments;

    wrist_angles.emplace_back(kNbBranches, PointAngle(circle_angle));

    // TODO(jeanfrancoisd): We could get both points and only cover [0,pi]
    Pnt p4s = SelectP4(parameters, p5w, p6, circle_angle);

    std::vector<IKPntSolution> solutions;
    SolveIKWithP4(parameters, p4s, p5, p6, kAllBranch, &solutions,
                  &p4_a1_distance);

#ifdef SAVE_ANGLE_VALUES
    if (!solutions.empty()) {
      all_solutions.push_back(AngleSolutions({circle_angle, solutions}));
    }
#endif

    for (size_t branch = 0; branch < kNbBranches; ++branch) {
      auto solution_pair = GetSolutionForBranch(branch, solutions);
      if (solution_pair.first) {
        for (const auto& solution : solution_pair.second) {
          // Find the cosine between line p3 - p4s and line p4s - p5
          double cos_p3_p4_p5 = WristAngle(solution.p3, p4s, p5);

          (wrist_angles.back())[branch] =
              PointAngle({circle_angle, cos_p3_p4_p5});

          if (wrist_angles.size() > 1) {
            const PointAngle& wrist_angle_0 =
                (*(wrist_angles.rbegin()))[branch];
            const PointAngle& wrist_angle_1 =
                (*(wrist_angles.rbegin() + 1))[branch];

            auto crossing_solution = CheckForCrossingSolution(
                parameters, p5w, p5, p6, branch, wrist_angle_1, wrist_angle_0,
                solution);
            if (crossing_solution.first) {
              AppendSolution(crossing_solution.second, branch,
                             &valid_solutions);
            } else {
              if (wrist_angles.size() > 2) {
                const PointAngle& wrist_angle_2 =
                    (*(wrist_angles.rbegin() + 2))[branch];
                auto tangent_solution = CheckForTangentSolution(
                    parameters, p5w, p5, p6, branch, wrist_angle_0,
                    wrist_angle_1, wrist_angle_2, solution);
                if (tangent_solution.first) {
                  AppendSolution(tangent_solution.second, branch,
                                 &valid_solutions);
                }
              }
            }
          }
        }
      } else {
        // Check for transition from valid to invalid
        if (wrist_angles.size() > 1) {
          const PointAngle& wrist_angle_0 = (*(wrist_angles.rbegin()))[branch];
          const PointAngle& wrist_angle_1 =
              (*(wrist_angles.rbegin() + 1))[branch];

          bool is_crossing = IsCrossing(wrist_angle_1, wrist_angle_0,
                                        parameters.cos_wrist_angle);
          if (is_crossing) {
            auto crossing_solution = RefineP4CrossingWithInvalid(
                parameters, p5w, p5, p6, branch, wrist_angle_1, wrist_angle_0);
            if (crossing_solution.first) {
              AppendSolution(crossing_solution.second, branch,
                             &valid_solutions);
            } else {
              DVLOG(1) << "Failed to converge on invalid transition at "
                       << circle_angle;
            }
          }
        }
      }
    }
  }

#ifdef SAVE_ANGLE_VALUES
  // Write the angle values. Debugging only
  DVLOG(1) << "Saving angles values";
  std::ofstream angle_out("/tmp/angles.csv");
  for (size_t i = 0; i < wrist_angles.size(); ++i) {
    angle_out << wrist_angles[i][0].p4_angle << ",";
    for (size_t j = 0; j < kNbBranches; ++j) {
      angle_out << wrist_angles[i][j].wrist_angle;
      if (j < 3) {
        angle_out << ",";
      } else {
        angle_out << "\n";
      }
    }
  }
  angle_out.close();

  for (uint8 branch = 0; branch < kNbBranches; ++branch) {
    string filename = absl::StrFormat("/tmp/solution_branch_%d.csv", branch);
    DVLOG(1) << "Saving solution to " << filename;
    std::ofstream solution_out(filename);
    for (const auto& angle_solution : all_solutions) {
      for (const auto& branch_solution : angle_solution.solutions) {
        if (branch_solution.branch == branch) {
          solution_out << angle_solution.circle_angle << ",";
          solution_out << PointCoordinateToString(branch_solution.p1) << ",";
          solution_out << PointCoordinateToString(branch_solution.p2) << ",";
          solution_out << PointCoordinateToString(branch_solution.p3) << ",";
          solution_out << PointCoordinateToString(branch_solution.p4) << ",";
          solution_out << PointCoordinateToString(p5) << ",";
          solution_out << PointCoordinateToString(p6) << "\n";
        }
      }
    }
    solution_out.close();
  }
#endif

  int count = 0;
  for (const auto& solution_branch : valid_solutions) {
    count += solution_branch.size();
  }
  DVLOG(1) << "Total solutions found: " << count;

  // Get joint angles for the found solutions.
  // TODO(jeanfrancoisd): Here we lose branch information. Might be worth
  // keeping somehow in the return values.
  size_t joint_solutions_count = 0;
  for (const auto& solution_branch : valid_solutions) {
    for (const auto& solution : solution_branch) {
      const eigenmath::Vector6dAligned joint_solution =
          GetJointAngles(parameters, solution, p5, p6, p6z);

      if (IsUniqueJointSolution(joint_solution, *joint_solutions)) {
        CHECK_LT(joint_solutions_count, joint_solutions->size())
            << " There is " << count
            << " potential solutions. Current solution in the buffer:" <<
            [=]() -> std::string {
          std::stringstream ss;
          ss << "\n";
          for (const auto& joint_solution : *joint_solutions) {
            ss << joint_solution.transpose() << "\n";
          }
          return ss.str();
        }();

        (*joint_solutions)[joint_solutions_count] = joint_solution;

        DVLOG(2) << "Solutions at: " << solution.circle_angle << ": "
                 << (*joint_solutions)[joint_solutions_count].transpose();
        joint_solutions_count++;
      }
    }
  }

  // Apply joint offsets.
  // TODO(b/191714207): Offset should be made outside this solver since it
  // is tailored to special robot offset and joint directions.
  for (auto& solution_offset : *joint_solutions) {
    solution_offset[0] *= -1;
    solution_offset[1] *= -1;
    solution_offset[1] += M_PI / 2;
    solution_offset[2] = -(solution_offset[2] + M_PI / 2);
    solution_offset[3] -= M_PI / 2;
    solution_offset[5] += M_PI / 2;
  }

  return joint_solutions_count;
}

}  // namespace

absl::Status WristOffsetGeometricIKSolver::Init(const Chain& chain) {
  if (chain.GetNumberDegreesOfFreedom() != 6) {
    return absl::InvalidArgumentError("The robot should have 6-dof.");
  }

  INTR_ASSIGN_OR_RETURN(const bool has_dependent_joints,
                        chain.HasDependentJoints());
  if (has_dependent_joints) {
    // TODO(b/427524846): Add support for dependent joints.
    return absl::InvalidArgumentError(
        "WristOffsetGeometricIKSolver does not support chains with dependent "
        "joints.");
  }

  INTR_ASSIGN_OR_RETURN(parameters_, ExtractParameters(chain));

  is_initialized_ = true;
  return absl::OkStatus();
}

absl::StatusOr<WristOffsetGeometricIKSolver::Parameters>
WristOffsetGeometricIKSolver::ExtractParameters(const Chain& chain) {
  constexpr int kNbDof = 6;
  if (chain.GetNumberDegreesOfFreedom() != kNbDof) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "Chain should have 6 dof, got "
           << chain.GetNumberDegreesOfFreedom();
  }

  const auto tip_id = chain.GetTipId();
  INTRINSIC_RT_ASSIGN_OR_RETURN(auto dof_ids, chain.GetDofChainForTip(tip_id));
  State state(&chain);
  INTRINSIC_RT_ASSIGN_OR_RETURN(const StateValues default_state_values,
                                chain.GetDefaultStateValues());
  INTRINSIC_RT_RETURN_IF_ERROR(
      state.SetDofPositions(default_state_values.dof_positions));

  std::array<Pose3d, kNbDof> parent_t_j;
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      parent_t_j[0], state.GetTransform(chain.GetBaseId(), dof_ids[0]));
  for (size_t i = 1; i < dof_ids.size(); ++i) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        parent_t_j[i], state.GetTransform(dof_ids[i - 1], dof_ids[i]));
  }
  INTRINSIC_RT_ASSIGN_OR_RETURN(Pose3d parent_t_tip,
                                state.GetTransform(dof_ids.back(), tip_id));

  WristOffsetGeometricIKSolver::Parameters ret;
  ret.dh[1].d = parent_t_j[1].translation()[2];
  ret.dh[1].a = parent_t_j[1].translation()[0];
  ret.dh[2].a = parent_t_j[2].translation()[2];
  ret.dh[3].a = parent_t_j[3].translation()[2];
  ret.dh[4].d = parent_t_j[3].translation()[0];
  ret.dh_d5_x = parent_t_j[4].translation()[0];
  ret.dh_d5_z = parent_t_j[4].translation()[2];
  ret.dh[5].d = sqrt(ret.dh_d5_x * ret.dh_d5_x + ret.dh_d5_z * ret.dh_d5_z);
  ret.dh[6].d = parent_t_j[5].translation()[0];
  ret.dh[5].alpha = atan2(ret.dh_d5_z, ret.dh_d5_x);

  ret.r01 = sqrt(ret.dh[1].a * ret.dh[1].a + ret.dh[1].d * ret.dh[1].d);
  ret.r24 = sqrt(ret.dh[4].d * ret.dh[4].d + ret.dh[3].a * ret.dh[3].a);
  if (ret.dh[5].alpha > M_PI_2) {
    ret.cos_wrist_angle = cos(ret.dh[5].alpha);
  } else {
    ret.cos_wrist_angle = cos(M_PI - ret.dh[5].alpha);
  }

  // Verify the parameters
  static const double kEpsilon = 1e-6;

  if (parent_t_j[0].translation().squaredNorm() > kEpsilon) {
    return absl::InvalidArgumentError("Invalid parent pose translation.");
  }
  if (fabs(parent_t_j[1].translation().squaredNorm() -
           (ret.dh[1].d * ret.dh[1].d + ret.dh[1].a * ret.dh[1].a)) >
      kEpsilon) {
    return absl::InvalidArgumentError("Invalid value for d1 or a1.");
  }
  if (fabs(parent_t_j[2].translation().squaredNorm() -
           ret.dh[2].a * ret.dh[2].a) > kEpsilon) {
    return absl::InvalidArgumentError("Invalid value for a2.");
  }
  if (fabs(parent_t_j[3].translation().squaredNorm() -
           (ret.dh[3].a * ret.dh[3].a + ret.dh[4].d * ret.dh[4].d)) >
      kEpsilon) {
    return absl::InvalidArgumentError("Invalid value for a3 or d4.");
  }
  if (fabs(parent_t_j[4].translation().squaredNorm() -
           (ret.dh[5].d * ret.dh[5].d)) > kEpsilon) {
    return absl::InvalidArgumentError("Invalid value for d5.");
  }
  if (fabs(parent_t_j[5].translation().squaredNorm() -
           ret.dh[6].d * ret.dh[6].d) > kEpsilon) {
    return absl::InvalidArgumentError("The robot should have 6-dof.");
  }

  return ret;
}

WristOffsetGeometricIKSolver::WristOffsetGeometricIKSolver(
    const Parameters& parameters)
    : parameters_(parameters), is_initialized_(true) {}

int WristOffsetGeometricIKSolver::Solve(
    const Pose3d& base_t_tip,
    std::array<eigenmath::Vector6dAligned, kSolutionBufferSize>* solutions)
    const {
  CHECK(is_initialized_);
  solutions->fill(eigenmath::Vector6dAligned::Zero());

  const Vector3d& p6 = base_t_tip.translation();
  const Vector3d p6z = base_t_tip.translation() + base_t_tip.zAxis();

  const Vector3d p5 =
      base_t_tip.translation() - base_t_tip.xAxis() * parameters_.dh[6].d;
  const Vector3d p5w =
      base_t_tip.translation() -
      base_t_tip.xAxis() * (parameters_.dh_d5_x + parameters_.dh[6].d);

  return GeometricSolve(parameters_, PointFromVector3d(p5w),
                        PointFromVector3d(p5), PointFromVector3d(p6),
                        PointFromVector3d(p6z), solutions);
}

}  // namespace kinematics
}  // namespace intrinsic
