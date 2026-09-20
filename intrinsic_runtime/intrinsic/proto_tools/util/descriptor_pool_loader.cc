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

#include "intrinsic_runtime/intrinsic/proto_tools/util/descriptor_pool_loader.h"

#include <memory>
#include <utility>

#include "absl/memory/memory.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/descriptor.pb.h"
#include "google/protobuf/descriptor_database.h"
#include "intrinsic/util/path_resolver/path_resolver.h"
#include "intrinsic/util/proto/descriptors.h"
#include "intrinsic/util/status/status_macros.h"
#include "ortools/base/file.h"

namespace intrinsic::executive {

namespace {

absl::StatusOr<google::protobuf::FileDescriptorSet>
LoadFileDescriptorSetFromPath(absl::string_view file_path) {
  google::protobuf::FileDescriptorSet fdset;
  INTR_RETURN_IF_ERROR(
      file::GetBinaryProto(file_path, &fdset, file::Defaults()));
  return fdset;
}

absl::StatusOr<google::protobuf::FileDescriptorSet>
LoadFileDescriptorSetFromRunfiles(absl::string_view runfiles_path) {
  return LoadFileDescriptorSetFromPath(
      PathResolver::ResolveRunfilesPath(runfiles_path));
}

}  // namespace

LoadedDescriptorPool::LoadedDescriptorPool(
    google::protobuf::FileDescriptorSet fdset,
    std::unique_ptr<google::protobuf::SimpleDescriptorDatabase> descriptor_db,
    std::unique_ptr<google::protobuf::DescriptorPool> descriptor_pool)
    : file_descriptor_set_(std::move(fdset)),
      descriptor_db_(std::move(descriptor_db)),
      descriptor_pool_(std::move(descriptor_pool)) {}

absl::StatusOr<std::unique_ptr<LoadedDescriptorPool>>
LoadedDescriptorPool::Create(google::protobuf::FileDescriptorSet fdset) {
  auto db = std::make_unique<google::protobuf::SimpleDescriptorDatabase>();
  INTR_RETURN_IF_ERROR(PopulateDescriptorDatabase(db.get(), fdset));

  auto pool = std::make_unique<google::protobuf::DescriptorPool>(db.get());

  return absl::WrapUnique(new LoadedDescriptorPool(
      std::move(fdset), std::move(db), std::move(pool)));
}

absl::StatusOr<std::unique_ptr<LoadedDescriptorPool>>
LoadedDescriptorPool::LoadFromPath(absl::string_view file_path) {
  INTR_ASSIGN_OR_RETURN(google::protobuf::FileDescriptorSet fdset,
                        LoadFileDescriptorSetFromPath(file_path));
  return Create(std::move(fdset));
}

absl::StatusOr<std::unique_ptr<LoadedDescriptorPool>>
LoadedDescriptorPool::LoadFromRunfiles(absl::string_view runfiles_path) {
  INTR_ASSIGN_OR_RETURN(google::protobuf::FileDescriptorSet fdset,
                        LoadFileDescriptorSetFromRunfiles(runfiles_path));
  return Create(std::move(fdset));
}

}  // namespace intrinsic::executive
