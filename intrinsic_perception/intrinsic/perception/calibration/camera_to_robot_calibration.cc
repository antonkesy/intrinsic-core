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

#include "intrinsic/perception/calibration/camera_to_robot_calibration.h"

#include <cmath>
#include <memory>
#include <vector>

#include "Eigen/Core"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "ceres/autodiff_cost_function.h"
#include "ceres/manifold.h"
#include "ceres/problem.h"
#include "ceres/solver.h"
#include "ceres/types.h"
#include "intrinsic/eigenmath/ceres_poses.h"
#include "intrinsic/eigenmath/pose3_utils.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/almost_equals.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/perception/calibration/metrics.h"
#include "intrinsic/perception/core/eigen_types.h"
#include "intrinsic/perception/core/opencv_wrapper.h"
#include "intrinsic/perception/proto/v1/camera_setup.pb.h"
#include "intrinsic/perception/proto/v1/camera_to_robot_calibration.pb.h"
#include "intrinsic/perception/proto_conversion/eigen_conversions.h"
#include "intrinsic/util/status/status_macros.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/core/mat.hpp"

namespace intrinsic::perception {

namespace {

using intrinsic_proto::perception::v1::CAMERA_SETUP_MOVING;
using intrinsic_proto::perception::v1::CAMERA_SETUP_UNSPECIFIED;
using intrinsic_proto::perception::v1::CameraToRobotCalibrationRequest;
using intrinsic_proto::perception::v1::CameraToRobotCalibrationResult;

cv::Mat ToCVRotation(const Eigen::Matrix3d& rotation) {
  cv::Mat rotation_cv;
  cv::eigen2cv(rotation, rotation_cv);
  return rotation_cv;
}

cv::Mat ToCVTranslation(const Eigen::Vector3d& translation) {
  cv::Mat translation_cv;
  cv::eigen2cv(translation, translation_cv);
  return translation_cv;
}

absl::Status IsValidPose(const cv::Mat& rotation, const cv::Mat& translation) {
  if (cv::sum(cv::Mat(rotation != rotation))[0] > 0 ||
      cv::sum(cv::Mat(translation != translation))[0]) {
    return absl::InternalError("Algorithm did not converge to a solution.");
  }
  return absl::OkStatus();
}

struct CostFunctor {
  Pose3d base_t_flange;
  Pose3d camera_t_object;
  // Scales the residuals of the translation error relative to the orientation
  // error.
  double translation_error_scale_factor;

  // Computes cost function for a pose pair. The variables to optimize
  // 'flange_t_object' and 'base_t_camera' are provided in two variable sets
  // each, one for rotation 'r' and one for translation 'p'.
  template <typename T>
  bool operator()(const T* const flange_r_object,
                  const T* const flange_p_object, const T* const base_r_camera,
                  const T* const base_p_camera, T* raw_residual) const {
    const Pose3<T> flange_t_object = eigenmath::ceresutils::MakePose3FromViews(
        Eigen::Map<const eigenmath::Vector4<T, Eigen::DontAlign>>(
            flange_r_object),
        Eigen::Map<const eigenmath::Vector3<T>>(flange_p_object));
    const Pose3<T> base_t_camera = eigenmath::ceresutils::MakePose3FromViews(
        Eigen::Map<const eigenmath::Vector4<T, Eigen::DontAlign>>(
            base_r_camera),
        Eigen::Map<const eigenmath::Vector3<T>>(base_p_camera));

    Eigen::Map<eigenmath::Vector6<T, Eigen::DontAlign>> residual(raw_residual);
    const Pose3<T> pose0 = base_t_flange.cast<T>() * flange_t_object;
    const Pose3<T> pose1 = base_t_camera * camera_t_object.cast<T>();
    residual = eigenmath::PoseResidual(pose0, pose1);
    residual.template tail<3>() *=
        static_cast<T>(1 / translation_error_scale_factor);

    return true;
  }

  static std::unique_ptr<ceres::CostFunction> Create(
      const Pose3d& base_t_flange, const Pose3d& camera_t_object,
      const double translation_error_scale_factor) {
    constexpr int kResidualDim = 6;
    constexpr int kQuaternionDim = 4;
    constexpr int kTranslationDim = 3;
    return std::make_unique<ceres::AutoDiffCostFunction<
        CostFunctor, kResidualDim, kQuaternionDim, kTranslationDim,
        kQuaternionDim, kTranslationDim>>
        // Ceres takes ownership.
        (new CostFunctor({
            .base_t_flange = base_t_flange,
            .camera_t_object = camera_t_object,
            .translation_error_scale_factor = translation_error_scale_factor,
        }));
  }
};

absl::StatusOr<CameraToRobotCalibrationResult> NonLinearOptimization(
    const CameraToRobotCalibrationRequest& request,
    const CameraToRobotCalibrationResult& initial_solution) {
  Pose3d base_t_camera;
  Pose3d flange_t_object;

  if (request.type() == CAMERA_SETUP_MOVING) {
    INTR_ASSIGN_OR_RETURN(
        base_t_camera,
        FromProto(
            initial_solution.moving_camera_result_poses().base_t_object()));
    INTR_ASSIGN_OR_RETURN(
        flange_t_object,
        FromProto(
            initial_solution.moving_camera_result_poses().flange_t_camera()));
  } else {
    INTR_ASSIGN_OR_RETURN(
        base_t_camera,
        FromProto(
            initial_solution.stationary_camera_result_poses().base_t_camera()));
    INTR_ASSIGN_OR_RETURN(
        flange_t_object,
        FromProto(initial_solution.stationary_camera_result_poses()
                      .flange_t_object()));
  }
  std::vector<Pose3d> base_t_flanges;
  base_t_flanges.reserve(request.input_pose_pairs_size());
  std::vector<Pose3d> camera_t_objects;
  camera_t_objects.reserve(request.input_pose_pairs_size());
  for (const auto& pose_pair : request.input_pose_pairs()) {
    INTR_ASSIGN_OR_RETURN(Pose3d base_t_flange,
                          FromProto(pose_pair.base_t_flange()));
    base_t_flanges.emplace_back(base_t_flange);
    INTR_ASSIGN_OR_RETURN(Pose3d camera_t_object,
                          FromProto(pose_pair.camera_t_object()));
    if (request.type() == CAMERA_SETUP_MOVING) {
      camera_t_object = camera_t_object.inverse();
    }
    camera_t_objects.push_back(camera_t_object);
  }

  // Compute optimal weighting between translation and rotation.
  double rotation_variance = 0.;
  double translation_variance = 0.;
  for (int i = 0; i < base_t_flanges.size(); ++i) {
    const Pose3d pose0 = base_t_flanges[i] * flange_t_object;
    const Pose3d pose1 = base_t_camera * camera_t_objects[i];
    const Vector6<double> residual = eigenmath::PoseResidual(pose0, pose1);
    translation_variance += std::pow(residual.tail<3>().norm(), 2);
    rotation_variance += std::pow(residual.head<3>().norm(), 2);
  }
  rotation_variance /= base_t_flanges.size();
  translation_variance /= base_t_flanges.size();
  const double translation_error_scale_factor =
      std::sqrt(translation_variance / rotation_variance);

  ceres::Problem::Options problem_options;
  problem_options.cost_function_ownership =
      ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem(problem_options);
  std::vector<std::unique_ptr<ceres::CostFunction>> cost_functors;
  cost_functors.reserve(request.input_pose_pairs().size());
  for (int i = 0; i < base_t_flanges.size(); ++i) {
    cost_functors.push_back(
        CostFunctor::Create(base_t_flanges[i], camera_t_objects[i],
                            translation_error_scale_factor));
    problem.AddResidualBlock(
        cost_functors.back()
            .get(),  // Note: The problem is configured not to take ownership.
        nullptr,     // Squared loss
        eigenmath::ceresutils::MutableRotationData(flange_t_object),
        flange_t_object.translation().data(),
        eigenmath::ceresutils::MutableRotationData(base_t_camera),
        base_t_camera.translation().data());
  }

  // ceres will take ownership.
  auto manifold = new ceres::EigenQuaternionManifold;

  problem.SetManifold(
      eigenmath::ceresutils::MutableRotationData(flange_t_object), manifold);
  problem.SetManifold(eigenmath::ceresutils::MutableRotationData(base_t_camera),
                      manifold);

  // Solve problem.
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  CameraToRobotCalibrationResult result;
  if (request.type() == CAMERA_SETUP_MOVING) {
    auto* result_poses = result.mutable_moving_camera_result_poses();
    *result_poses->mutable_flange_t_camera() = ToProto(flange_t_object);
    *result_poses->mutable_base_t_object() = ToProto(base_t_camera);
  } else {
    auto* result_poses = result.mutable_stationary_camera_result_poses();
    *result_poses->mutable_base_t_camera() = ToProto(base_t_camera);
    *result_poses->mutable_flange_t_object() = ToProto(flange_t_object);
  }
  return result;
}

bool IsOpenCVCalibrateRobotWorldCameraToRobotAlgorithm(
    const CameraToRobotCalibrationRequest::OptimizationAlgorithm& algorithm) {
  if (algorithm == CameraToRobotCalibrationRequest::SHAH ||
      algorithm == CameraToRobotCalibrationRequest::LI ||
      algorithm == CameraToRobotCalibrationRequest::NONLINEAR ||
      algorithm == CameraToRobotCalibrationRequest::AUTO) {
    return true;
  }
  return false;
}

absl::StatusOr<CameraToRobotCalibrationResult>
RunCalibrateRobotWorldCameraToRobot(
    const CameraToRobotCalibrationRequest& request) {
  int size = request.input_pose_pairs_size();
  std::vector<cv::Mat> camera_t_object_rotations;
  camera_t_object_rotations.reserve(size);
  std::vector<cv::Mat> camera_t_object_translations;
  camera_t_object_translations.reserve(size);
  std::vector<cv::Mat> flange_t_base_rotations;
  flange_t_base_rotations.reserve(size);
  std::vector<cv::Mat> flange_t_base_translations;
  flange_t_base_translations.reserve(size);
  for (const auto& pose_pair : request.input_pose_pairs()) {
    Eigen::Isometry3d camera_t_object = ToEigen(pose_pair.camera_t_object());
    Eigen::Isometry3d flange_t_base =
        ToEigen(pose_pair.base_t_flange()).inverse();
    flange_t_base_rotations.emplace_back(
        ToCVRotation(flange_t_base.rotation()));
    flange_t_base_translations.emplace_back(
        ToCVTranslation(flange_t_base.translation()));

    if (request.type() == CAMERA_SETUP_MOVING) {
      camera_t_object_rotations.emplace_back(
          ToCVRotation(camera_t_object.rotation()));
      camera_t_object_translations.emplace_back(
          ToCVTranslation(camera_t_object.translation()));
    } else {
      camera_t_object_rotations.emplace_back(
          ToCVRotation(camera_t_object.inverse().rotation()));
      camera_t_object_translations.emplace_back(
          ToCVTranslation(camera_t_object.inverse().translation()));
    }
  }

  cv::Mat world_t_base_rotation = cv::Mat_<float>(3, 3);
  cv::Mat world_t_base_translation = cv::Mat_<float>(3, 1);
  cv::Mat camera_t_flange_rotation = cv::Mat_<float>(3, 3);
  cv::Mat camera_t_flange_translation = cv::Mat_<float>(3, 1);
  // Note: The algorithm 'NONLINEAR' uses opencv SHAH as a preprocessor.
  auto algorithm = cv::CALIB_ROBOT_WORLD_HAND_EYE_SHAH;
  if (request.algorithm() == CameraToRobotCalibrationRequest::LI) {
    algorithm = cv::CALIB_ROBOT_WORLD_HAND_EYE_LI;
  }

  INTR_RETURN_IF_ERROR(TryCatchCvFunction([&]() {
    cv::calibrateRobotWorldHandEye(
        camera_t_object_rotations, camera_t_object_translations,
        flange_t_base_rotations, flange_t_base_translations,
        world_t_base_rotation, world_t_base_translation,
        camera_t_flange_rotation, camera_t_flange_translation, algorithm);
  }));
  INTR_RETURN_IF_ERROR(
      IsValidPose(world_t_base_rotation, world_t_base_translation));
  INTR_RETURN_IF_ERROR(
      IsValidPose(camera_t_flange_rotation, camera_t_flange_translation));

  INTR_ASSIGN_OR_RETURN(
      const Pose3d world_t_base,
      FromCVPose(world_t_base_rotation, world_t_base_translation));
  INTR_ASSIGN_OR_RETURN(
      const Pose3d camera_t_flange,
      FromCVPose(camera_t_flange_rotation, camera_t_flange_translation));

  CameraToRobotCalibrationResult result;
  if (request.type() == CAMERA_SETUP_MOVING) {
    auto* result_poses = result.mutable_moving_camera_result_poses();
    *result_poses->mutable_flange_t_camera() =
        ToProto(camera_t_flange.inverse());
    *result_poses->mutable_base_t_object() = ToProto(world_t_base.inverse());
  } else {
    auto* result_poses = result.mutable_stationary_camera_result_poses();
    *result_poses->mutable_base_t_camera() = ToProto(world_t_base.inverse());
    *result_poses->mutable_flange_t_object() =
        ToProto(camera_t_flange.inverse());
  }

  if (request.algorithm() == CameraToRobotCalibrationRequest::NONLINEAR ||
      request.algorithm() == CameraToRobotCalibrationRequest::AUTO) {
    INTR_ASSIGN_OR_RETURN(result, NonLinearOptimization(request, result));
  }

  return ComputeCameraToRobotCalibrationErrorMetrics(request, result);
}

absl::StatusOr<CameraToRobotCalibrationResult> RunCalibrateCameraToRobot(
    const CameraToRobotCalibrationRequest& request) {
  int size = request.input_pose_pairs_size();
  std::vector<cv::Mat> camera_t_object_rotations;
  camera_t_object_rotations.reserve(size);
  std::vector<cv::Mat> camera_t_object_translations;
  camera_t_object_translations.reserve(size);
  std::vector<cv::Mat> base_t_flange_rotations;
  base_t_flange_rotations.reserve(size);
  std::vector<cv::Mat> base_t_flange_translations;
  base_t_flange_translations.reserve(size);

  for (const auto& pose_pair : request.input_pose_pairs()) {
    Eigen::Isometry3d camera_t_object = ToEigen(pose_pair.camera_t_object());
    camera_t_object_rotations.emplace_back(
        ToCVRotation(camera_t_object.rotation()));
    camera_t_object_translations.emplace_back(
        ToCVTranslation(camera_t_object.translation()));
    if (request.type() == CAMERA_SETUP_MOVING) {
      Eigen::Isometry3d base_t_flange = ToEigen(pose_pair.base_t_flange());
      base_t_flange_rotations.emplace_back(
          ToCVRotation(base_t_flange.rotation()));
      base_t_flange_translations.emplace_back(
          ToCVTranslation(base_t_flange.translation()));
    } else {
      Eigen::Isometry3d flange_t_base =
          ToEigen(pose_pair.base_t_flange()).inverse();
      base_t_flange_rotations.emplace_back(
          ToCVRotation(flange_t_base.rotation()));
      base_t_flange_translations.emplace_back(
          ToCVTranslation(flange_t_base.translation()));
    }
  }

  cv::Mat rotation = cv::Mat_<float>(3, 3);
  cv::Mat translation = cv::Mat_<float>(3, 1);
  auto algorithm = cv::CALIB_HAND_EYE_TSAI;
  if (request.algorithm() == CameraToRobotCalibrationRequest::PARK) {
    algorithm = cv::CALIB_HAND_EYE_PARK;
  } else if (request.algorithm() == CameraToRobotCalibrationRequest::HORAUD) {
    algorithm = cv::CALIB_HAND_EYE_HORAUD;
  } else if (request.algorithm() == CameraToRobotCalibrationRequest::ANDREFF) {
    algorithm = cv::CALIB_HAND_EYE_ANDREFF;
  } else if (request.algorithm() ==
             CameraToRobotCalibrationRequest::DANIILIDIS) {
    algorithm = cv::CALIB_HAND_EYE_DANIILIDIS;
  }
  INTR_RETURN_IF_ERROR(TryCatchCvFunction([&]() {
    cv::calibrateHandEye(base_t_flange_rotations, base_t_flange_translations,
                         camera_t_object_rotations,
                         camera_t_object_translations, rotation, translation,
                         algorithm);
  }));
  INTR_RETURN_IF_ERROR(IsValidPose(rotation, translation));
  INTR_ASSIGN_OR_RETURN(const Pose3d resulting_pose,
                        FromCVPose(rotation, translation));
  if (!intrinsic::AlmostEquals(resulting_pose.quaternion().squaredNorm(),
                               1.0)) {
    return absl::InternalError(
        "cv::calibrateHandEye converged to a solution with an invalid "
        "rotation.");
  }

  INTR_ASSIGN_OR_RETURN(const Pose3d camera_t_object,
                        intrinsic_proto::FromProto(
                            request.input_pose_pairs(0).camera_t_object()));
  INTR_ASSIGN_OR_RETURN(
      const Pose3d base_t_flange,
      intrinsic_proto::FromProto(request.input_pose_pairs(0).base_t_flange()));
  CameraToRobotCalibrationResult result;
  if (request.type() == CAMERA_SETUP_MOVING) {
    auto* result_poses = result.mutable_moving_camera_result_poses();
    *result_poses->mutable_flange_t_camera() = ToProto(resulting_pose);
  } else {
    auto* result_poses = result.mutable_stationary_camera_result_poses();
    *result_poses->mutable_base_t_camera() = ToProto(resulting_pose);
  }
  return ComputeCameraToRobotCalibrationErrorMetrics(request, result);
}

}  // namespace

absl::StatusOr<CameraToRobotCalibrationResult> CalibrateCameraToRobot(
    const CameraToRobotCalibrationRequest& request) {
  if (request.type() == CAMERA_SETUP_UNSPECIFIED) {
    return absl::InvalidArgumentError("Invalid CameraSetup.");
  }
  if (request.input_pose_pairs_size() < 3) {
    return absl::InvalidArgumentError(
        "At least 3 pose pairs are required for calibration.");
  }

  LOG(INFO) << "Running hand eye calibration with "
            << request.input_pose_pairs_size() << " pose pairs.";
  if (IsOpenCVCalibrateRobotWorldCameraToRobotAlgorithm(request.algorithm())) {
    return RunCalibrateRobotWorldCameraToRobot(request);
  }
  return RunCalibrateCameraToRobot(request);
}

}  // namespace intrinsic::perception
