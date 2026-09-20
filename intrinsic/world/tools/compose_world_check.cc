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

// Loads resource set and tries to assemble the world. This binary is intended
// to check whether a resource set was defined correctly and whether the
// resource instances are correctly integrated.
//
// Example usage:
// bazel run --config=intrinsic \
// //intrinsic/world/tools:compose_world_check -- \
// --resource_set resourceset.pb

#include <string>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/flags.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "intrinsic/config/proto/resource_set.pb.h"
#include "intrinsic/geometry/storage/dummy_storage.h"
#include "intrinsic/icon/release/file_helpers.h"
#include "intrinsic/icon/release/portable/init_intrinsic.h"
#include "intrinsic/resources/proto/geometric_resource_data.pb.h"
#include "intrinsic/resources/proto/resource_instance.pb.h"
#include "intrinsic/simulation/world/world_to_sdf.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/util/compose_world_util.h"

ABSL_FLAG(std::string, geometric_data, "", "Path to resource set pbtxt file.");

namespace intrinsic {

absl::Status MainImpl() {
  if (absl::GetFlag(FLAGS_geometric_data).empty()) {
    return absl::InvalidArgumentError("--geometric_data is required");
  }
  INTR_ASSIGN_OR_RETURN(
      const auto resource_set_data,
      intrinsic::GetBinaryProto<
          intrinsic_proto::resources::GeometricResourceSetData>(
          absl::GetFlag(FLAGS_geometric_data)));

  INTR_ASSIGN_OR_RETURN(ComposeWorldResult compose_result,
                        ComposeResourceSetIntoWorld(
                            resource_set_data, *GetDummyGeometryLibrary()));
  LOG(INFO) << "World successfully composed";

  INTR_ASSIGN_OR_RETURN(
      auto adapter, simulation::WorldSdfAdapter::Create(compose_result.world));
  if (adapter == nullptr) {
    return absl::InternalError("Failed to create WorldSdfAdapter");
  }

  return absl::OkStatus();
}

}  // namespace intrinsic

int main(int argc, char** argv) {
  InitIntrinsic(argv[0], argc, argv);
  QCHECK_OK(intrinsic::MainImpl());
  return 0;
}
