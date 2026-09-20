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

#ifndef INTRINSIC_SCENE_SERVICE_SCENE_OBJECT_IMPORT_H_
#define INTRINSIC_SCENE_SERVICE_SCENE_OBJECT_IMPORT_H_

#include <memory>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "google/longrunning/operations.pb.h"
#include "google/protobuf/any.pb.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "intrinsic/geometry/proto/geometry_service.grpc.pb.h"
#include "intrinsic/geometry/proto/geometry_service.pb.h"
#include "intrinsic/longrunning/cc/operation_scheduler_interface.h"
#include "intrinsic/longrunning/cc/operations_proxy.h"
#include "intrinsic/scene/instantiation/imported_scene_instantiator.h"
#include "intrinsic/scene/proto/v1/export.pb.h"
#include "intrinsic/scene/proto/v1/scene_object_export.grpc.pb.h"
#include "intrinsic/scene/proto/v1/scene_object_export.pb.h"
#include "intrinsic/scene/proto/v1/scene_object_import.grpc.pb.h"
#include "intrinsic/scene/proto/v1/scene_object_import.pb.h"
#include "intrinsic/scene/proto/v1/scene_object_internal.grpc.pb.h"
#include "intrinsic/scene/proto/v1/scene_object_internal.pb.h"
#include "intrinsic/world/proto/world_fragment.pb.h"

namespace intrinsic {

class SceneObjectImportImpl;
class SceneObjectImport;
class SceneObjectInternal;
class SceneObjectExport;

struct SceneObjectImportServices {
  std::unique_ptr<SceneObjectInternal> soii_service;
  std::unique_ptr<SceneObjectImport> soi_service;
  std::unique_ptr<SceneObjectExport> soe_service;
};

// Creates both the internal and external SceneObjectImport services in a single
// struct.
absl::StatusOr<SceneObjectImportServices> CreateSceneObjectImportServices(
    std::unique_ptr<intrinsic_proto::geometry::GeometryService::StubInterface>
        geometry_service_stub,
    std::shared_ptr<longrunning::OperationsProxy> geometry_operations_proxy,
    std::unique_ptr<ImportedSceneInstantiator> imported_scene_instantiator,
    std::shared_ptr<longrunning::OperationSchedulerInterface> scheduler);

class SceneObjectInternal
    : public intrinsic_proto::scene_object::v1::SceneObjectInternal::Service {
 public:
  SceneObjectInternal() = delete;
  explicit SceneObjectInternal(std::shared_ptr<SceneObjectImportImpl> pimpl);

  ~SceneObjectInternal() override;

  grpc::Status ImportScene(
      grpc::ServerContext* context,
      const intrinsic_proto::scene_object::v1::ImportSceneRequest* request,
      google::longrunning::Operation* response) override;

  grpc::Status InstantiateImportedScene(
      grpc::ServerContext* context,
      const intrinsic_proto::scene_object::v1::InstantiateImportedSceneRequest*
          request,
      google::longrunning::Operation* response) override;

  grpc::Status Export(
      grpc::ServerContext* context,
      const intrinsic_proto::scene_object::v1::ExportRequest* request,
      intrinsic_proto::scene_object::v1::ExportResponse* response) override;

 private:
  std::shared_ptr<SceneObjectImportImpl> pimpl_;
};

class SceneObjectImport
    : public intrinsic_proto::scene_object::v1::SceneObjectImport::Service {
 public:
  SceneObjectImport() = delete;
  explicit SceneObjectImport(std::shared_ptr<SceneObjectImportImpl> pimpl);

  ~SceneObjectImport() override = default;

  grpc::Status ImportSceneObject(
      grpc::ServerContext* context,
      const intrinsic_proto::scene_object::v1::ImportSceneObjectRequest*
          request,
      google::longrunning::Operation* response) override;

 private:
  std::shared_ptr<SceneObjectImportImpl> pimpl_;
};

class SceneObjectExport
    : public intrinsic_proto::scene_object::v1::SceneObjectExport::Service {
 public:
  SceneObjectExport() = delete;
  explicit SceneObjectExport(std::shared_ptr<SceneObjectImportImpl> pimpl);

  ~SceneObjectExport() override = default;

  grpc::Status Export(
      grpc::ServerContext* context,
      const intrinsic_proto::scene_object::v1::ExportRequest* request,
      intrinsic_proto::scene_object::v1::ExportResponse* response) override;

 private:
  std::shared_ptr<SceneObjectImportImpl> pimpl_;
};

namespace internal {
/**
 * Namespace for internal helper functions that we would like to expose for
 * testing.
 */

/**
 * The different high-level file types that we support for import
 */
enum class SceneFileTypes {
  SDF,
  USD,
  CAD,
  MESH,
};

// Returns all files in the directory that have the given extension.
// Does not look into subdirectories.
absl::StatusOr<std::vector<std::string>> FindFilesWithExtensionInDirectory(
    absl::string_view directory, absl::string_view ext);

// Returns all files in the directory that have any of the given extensions.
// Does not look into subdirectories.
absl::StatusOr<std::vector<std::string>> FindFilesWithExtensionsInDirectory(
    absl::string_view directory, const absl::flat_hash_set<std::string>& exts);

// Finds a scene file to import within the given directory. If we find a
// suitable scene file to import, returns {filepath, filetype}. Otherwise,
// returns an error. If there are multiple different file-types in the
// directory, attempts to select a "best" file / most-expected file
// to import, or fails with an error telling the user to resolve the ambiguity.
absl::StatusOr<std::pair<std::string, SceneFileTypes>>
FindSceneFileToImportInDirectory(absl::string_view scene_file_directory);

// Finds the topmost directory within the given directory that contains at least
// one relevant file (excluding metadata files and directories like __MACOSX
// and .DS_Store). This is used to skip redundant parent directories often found
// in zip files.
//
// Examples:
//
// Handles simple nesting:
// mydir/
//   mydir/
//     robot.sdf
// ==> mydir/mydir/
//
// Skips junk files:
// mydir/
//   .DS_Store
//   mydir/
//     robot.sdf
// ==> mydir/mydir/
//
// If it is ambigous which folder to traverse into, we return the parent:
// mydir/
//   folderA/
//      robot.sdf
//   folderB/
//      robot.sdf
// ==> mydir/
absl::StatusOr<std::string> FindTopmostDirectoryWithContent(
    absl::string_view directory_path);

// Returns a list of all supported file extensions for scene object import,
// in sorted order.
std::vector<std::string> GetAllSupportedExtensions();

};  // namespace internal

}  // namespace intrinsic
#endif  // INTRINSIC_SCENE_SERVICE_SCENE_OBJECT_IMPORT_H_
