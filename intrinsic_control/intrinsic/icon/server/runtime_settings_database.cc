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

#include "intrinsic/icon/server/runtime_settings_database.h"

#include <filesystem>  // NOLINT
#include <string>
#include <system_error>  // NOLINT

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/server/runtime_settings.pb.h"
#include "intrinsic/util/status/status_macros.h"
#include "ortools/base/helpers.h"
#include "ortools/base/options.h"

namespace intrinsic::icon {

absl::StatusOr<RuntimeSettings> LoadRuntimeSettings(absl::string_view path) {
  intrinsic_proto::icon::RuntimeSettings proto;
  INTR_RETURN_IF_ERROR(file::GetBinaryProto(path, &proto, file::Defaults()));
  LOG(INFO) << "Loaded RuntimeSettings from " << path << ": " << proto;
  RuntimeSettings settings;
  if (proto.has_speed_override()) {
    settings.speed_override = proto.speed_override();
  }
  return settings;
}

absl::Status SaveRuntimeSettings(absl::string_view path,
                                 const RuntimeSettings& settings) {
  intrinsic_proto::icon::RuntimeSettings proto;
  proto.set_speed_override(settings.speed_override);
  std::string temp_path = absl::StrCat(path, ".temp");
  INTR_RETURN_IF_ERROR(
      file::SetBinaryProto(temp_path, proto, file::Defaults()));
  std::error_code error;
  // Rename reduces the risk of partial writes.
  // Avoid file::Rename because it is not available in insrc.
  std::filesystem::rename(temp_path, path, error);
  if (error) {
    return absl::InternalError(
        absl::StrCat("Failed to rename to ", path, ": ", error.message()));
  }
  return absl::OkStatus();
}

}  // namespace intrinsic::icon
