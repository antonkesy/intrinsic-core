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

#ifndef INTRINSIC_MOTION_PLANNING_SERVICE_MOTION_PLANNER_SERVICE_ASSET_UTILS_H_
#define INTRINSIC_MOTION_PLANNING_SERVICE_MOTION_PLANNER_SERVICE_ASSET_UTILS_H_

#include <memory>
#include <string_view>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/assets/proto/installed_assets.grpc.pb.h"
#include "intrinsic/assets/proto/v1/resolved_dependency.pb.h"
#include "intrinsic/motion_planning/motion_planner_client.h"

namespace intrinsic {

constexpr std::string_view kMotionPlannerServiceAssetVersionKey =
    "motion_planner_service_asset_version";

// The definition of `kMotionPlannerServiceAssetIdPackage` and
// `kMotionPlannerServiceInterface` below have to match the definition in the
// `MotionPlannerService` (MPS) Asset info from the corresponding
// `*_manifest.textproto` file, i.e. the `metadata.id.package` and
// `service_def.service_proto_prefixes`, respectively.
constexpr std::string_view kMotionPlannerServiceAssetIdPackage = "ai.intrinsic";
constexpr char kMotionPlannerServiceInterface[] =
    "grpc://intrinsic_proto.motion_planning.v1.MotionPlannerService";

// The default `MotionPlannerService` (MPS) name. This is also the default value
// of the `metadata.id.name` in the corresponding `*_manifest.textproto` file
// containing the `MotionPlannerService` (MPS) Asset info.
constexpr char kMotionPlannerServiceName[] = "motion_planner_service";

// Returns a `MotionPlannerClient` connected to the `MotionPlannerService` (MPS)
// Asset resolved via `mps_dependency` and initialized with `world_id`. Returns
// `nullptr` if `mps_dependency.interfaces()` is empty. Returns an error if
// connecting to the `MotionPlannerService` Asset interface fails, for example
// due to incorrect specification of the `mps_dependency`.
absl::StatusOr<std::unique_ptr<motion_planning::MotionPlannerClient>>
GetMotionPlannerServiceAssetClient(
    std::string_view world_id,
    const intrinsic_proto::assets::v1::ResolvedDependency& mps_dependency);

// Given an `InstalledAssetsReader` `stub` and
// `motion_planner_service_asset_id_name` (from the `MotionPlannerService` (MPS)
// Asset info from the corresponding
// `*_manifest.textproto` file, i.e. the `metadata.id.name` field), get the
// installed `MotionPlannerService` (MPS) Asset version.
absl::StatusOr<std::string> GetInstalledMotionPlannerServiceAssetVersion(
    intrinsic_proto::assets::v1::InstalledAssetsReader::StubInterface* stub,
    std::string_view motion_planner_service_asset_id_name =
        kMotionPlannerServiceName);

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_SERVICE_MOTION_PLANNER_SERVICE_ASSET_UTILS_H_
