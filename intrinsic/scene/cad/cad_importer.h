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

#ifndef INTRINSIC_SCENE_CAD_CAD_IMPORTER_H_
#define INTRINSIC_SCENE_CAD_CAD_IMPORTER_H_

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/scene/cad/cad_import_service.pb.h"
#include "intrinsic/scene/proto/v1/imported_scene.pb.h"

namespace intrinsic {
namespace world {

// Interface for importing CAD data inside a CAD file.
class CADImporter {
 public:
  virtual ~CADImporter() = default;

  virtual absl::StatusOr<intrinsic_proto::scene_object::v1::ImportedScene>
  ImportCADAsScene(absl::string_view cad_file_data,
                   const intrinsic_proto::scene::cad::CADImportConfig&
                       cad_import_config) = 0;
};

}  // namespace world
}  // namespace intrinsic

#endif  // INTRINSIC_SCENE_CAD_CAD_IMPORTER_H_
