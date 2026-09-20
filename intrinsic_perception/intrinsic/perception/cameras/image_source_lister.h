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

#ifndef INTRINSIC_PERCEPTION_CAMERAS_IMAGE_SOURCE_LISTER_H_
#define INTRINSIC_PERCEPTION_CAMERAS_IMAGE_SOURCE_LISTER_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/base/no_destructor.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "cppregpattern/registry.h"
#include "intrinsic/perception/cameras/camera_identifier.h"

namespace intrinsic {
namespace perception {

// An interface class which allows us to list supported cameras per driver.
class ImageSourceLister {
 public:
  virtual ~ImageSourceLister() = default;
  // List all cameras which are available in the system.
  virtual absl::StatusOr<std::vector<CameraIdentifier>>
  ListAvailableCameras() = 0;
};

using ImageSourceListerRegistry =
    registry::Registry<std::string,
                       absl::StatusOr<std::unique_ptr<ImageSourceLister>>(),
                       registry::MissingKeyPolicy::default_construct>;

inline absl::flat_hash_map<std::string, std::string>&
GetImageSourceListerAliasRegistry() {
  static absl::NoDestructor<absl::flat_hash_map<std::string, std::string>>
      registry;
  return *registry;
}

#define REGISTER_IMAGE_SOURCE_LISTER(name, alias)                      \
  [[maybe_unused]] const bool kUnused##name =                          \
      intrinsic::perception::ImageSourceListerRegistry::Register(      \
          alias, [] { return std::make_unique<name>(); });             \
  [[maybe_unused]] const bool kUnusedAlias##name = []() {              \
    intrinsic::perception::GetImageSourceListerAliasRegistry().insert( \
        {alias, #name});                                               \
    return true;                                                       \
  }();

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_CAMERAS_IMAGE_SOURCE_LISTER_H_
