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

#include "intrinsic/perception/cameras/camera_factory.h"

#include <memory>
#include <string>
#include <variant>

#include "absl/functional/overload.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/perception/cameras/camera_identifier.h"
#include "intrinsic/perception/cameras/image_source.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::perception {
namespace {

absl::StatusOr<std::string> GetCameraDriverName(
    const CameraIdentifier& camera_identifier) {
  return std::visit(
      absl::Overload{
          [](const std::monostate&) -> absl::StatusOr<std::string> {
            return intrinsic::InvalidArgumentErrorBuilder()
                   << "Camera identifier has no driver.";
          },
          [](const CameraIdentifier::GenICam&) -> absl::StatusOr<std::string> {
            return "genicam";
          },
          [](const CameraIdentifier::Ros&) -> absl::StatusOr<std::string> {
            return "ros";
          },
          [](const CameraIdentifier::FileCamera&)
              -> absl::StatusOr<std::string> { return "file_camera"; },
          [](const CameraIdentifier::FakeGenICam&)
              -> absl::StatusOr<std::string> { return "fake_genicam"; }
      },
      camera_identifier.driver);
}

absl::StatusOr<std::unique_ptr<ImageSource>> CreateByName(
    absl::string_view name, const CameraIdentifier& camera_identifier) {
  INTR_ASSIGN_OR_RETURN(
      std::unique_ptr<ImageSource> interface,
      ImageSourceRegistry::Dispatch(std::string(name), camera_identifier));
  return interface;
}

absl::StatusOr<std::unique_ptr<ImageSource>> CreateByAlias(
    absl::string_view alias, const CameraIdentifier& camera_identifier) {
  auto it = GetImageSourceAliasRegistry().find(alias);
  if (it == GetImageSourceAliasRegistry().end()) {
    return absl::InvalidArgumentError(
        absl::StrCat("No image source registered with alias: ", alias));
  }
  return CreateByName(it->second, camera_identifier);
}

}  // namespace

absl::StatusOr<std::unique_ptr<ImageSource>> CreateImageSource(
    const CameraIdentifier& camera_identifier) {
  INTR_ASSIGN_OR_RETURN(const std::string camera_driver_alias,
                        GetCameraDriverName(camera_identifier), _.LogError());

  INTR_ASSIGN_OR_RETURN(std::unique_ptr<ImageSource> image_source,
                        CreateByAlias(camera_driver_alias, camera_identifier));

  return image_source;
}

}  // namespace intrinsic::perception
