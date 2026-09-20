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

// Checks a world for compatibility with the object-based view (see
// intrinsic/world/objects/object_world.h) and, if successful, prints
// a summary of the inferred objects and how they correspond to the entities in
// the world.
//
// Usage example:
//   bazel run --config=intrinsic \
//     //intrinsic/world/tools:check_objects -- \
//     --world_gzf_filename $PWD/bazel-genfiles/a/b/c/world.gzf

#include <iostream>
#include <memory>
#include <ostream>
#include <string>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/release/portable/init_intrinsic.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/gzfile/gzfile.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/objects/print_object_world.h"
#include "intrinsic/world/world.h"

ABSL_FLAG(bool, color, true,
          "Use font-weights and colors to structure output.");

ABSL_FLAG(bool, print_debug_info, false,
          "In case of a successful check print additional debug information "
          "about object-entity relationships.");
ABSL_FLAG(std::string, world_gzf_filename, "",
          "Path from which to read the world. In intrinsic process format");

namespace intrinsic {
namespace object_world {

absl::Status MainImpl() {
  const std::string world_gzf_filename =
      absl::GetFlag(FLAGS_world_gzf_filename);
  if (world_gzf_filename.empty()) {
    return absl::InvalidArgumentError("--world_gzf_filename not specified");
  }

  LOG(INFO) << "Loading the world from gzfile: " << world_gzf_filename;
  INTR_ASSIGN_OR_RETURN(auto gz_file, GZFile::Open(world_gzf_filename));
  INTR_ASSIGN_OR_RETURN(World entity_world, World::FromFile(*gz_file));

  INTR_ASSIGN_OR_RETURN(std::unique_ptr<ObjectWorld> world,
                        ObjectWorld::CreateView(entity_world));

  LOG(INFO) << "Check successful - given world is compatible with the "
               "object-world view!";

  INTR_ASSIGN_OR_RETURN(
      std::string summary,
      PrintObjectWorldSummary(*world, absl::GetFlag(FLAGS_color)));
  std::cout << "\nSummary of objects in the object-view of the given world:\n\n"
            << summary;

  if (absl::GetFlag(FLAGS_print_debug_info)) {
    INTR_ASSIGN_OR_RETURN(
        std::string mapping,
        PrintEntityToObjectMapping(*world, absl::GetFlag(FLAGS_color)));
    std::cout << "\n" << mapping << std::endl;
  }

  return absl::OkStatus();
}

}  // namespace object_world
}  // namespace intrinsic

int main(int argc, char** argv) {
  InitIntrinsic(argv[0], argc, argv);
  QCHECK_OK(intrinsic::object_world::MainImpl());
  return 0;
}
