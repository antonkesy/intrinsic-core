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

#include "intrinsic/world/objects/print_object_world.h"

#include <set>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "intrinsic/util/aggregate_type.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/entity.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/objects/frame_internal.h"
#include "intrinsic/world/objects/kinematic_object_internal.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/objects/physical_object.h"
#include "intrinsic/world/objects/root_object.h"
#include "intrinsic/world/objects/world_object_internal.h"
#include "intrinsic/world/world.h"

namespace intrinsic::object_world {

namespace {

const char* BeginColor(const char color[], bool enable_colors) { return ""; }

const char* EndColor(bool enable_colors) { return ""; }

}  // namespace

absl::StatusOr<std::string> PrintObjectWorldSummary(const ObjectWorld& world,
                                                    bool enable_colors) {
  struct : public WorldObjectConstVisitor {
    bool enable_colors;
    std::string info;
    std::string indent;

    void IncreaseIndent() { absl::StrAppend(&indent, "  "); }

    void DecreaseIndent() { indent.erase(indent.size() - 2); }

    void PrintFrames(const std::vector<const Frame*>& frames) {
      IncreaseIndent();
      for (const Frame* frame : frames) {
        absl::StrAppend(&info, indent, "-> ",
                        frame->GetParent()->GetName().value(), ".",
                        frame->GetName().value(), " (Frame)\n");
        std::vector<const Frame*> child_frames = frame->GetChildFramesSorted();
        if (!child_frames.empty()) {
          PrintFrames(child_frames);
        }
      }
      DecreaseIndent();
    }

    absl::Status PrintChildren(const WorldObject& parent) {
      IncreaseIndent();
      for (const WorldObject* child : parent.GetChildren()) {
        INTR_RETURN_IF_ERROR(child->Accept(*this));
      }
      DecreaseIndent();
      return absl::OkStatus();
    }

    absl::Status PrintWithTitle(const WorldObject& object,
                                absl::string_view title, const char color[]) {
      absl::StrAppend(&info, indent, BeginColor(color, enable_colors), "=> ",
                      object.GetName().value(), EndColor(enable_colors), " (",
                      title, ")\n");
      INTR_RETURN_IF_ERROR(PrintChildren(object));
      PrintFrames(object.GetChildFramesSorted());
      return absl::OkStatus();
    }

    absl::Status Visit(const RootObject& object) override {
      INTR_RETURN_IF_ERROR(PrintWithTitle(object, "RootObject", ""));
      return absl::OkStatus();
    }

    absl::Status Visit(const PhysicalObject& object) override {
      INTR_RETURN_IF_ERROR(PrintWithTitle(object, "PhysicalObject", ""));
      return absl::OkStatus();
    }

    absl::Status Visit(const KinematicObject& object) override {
      INTR_RETURN_IF_ERROR(PrintWithTitle(object, "KinematicObject", ""));
      return absl::OkStatus();
    }
  } print_visitor;
  print_visitor.enable_colors = enable_colors;

  // Print in DFS order, starting with the root object.
  INTR_ASSIGN_OR_RETURN(const WorldObject* object,
                        world.GetObject(RootObjectId()));
  INTR_RETURN_IF_ERROR(object->Accept(print_visitor));

  return print_visitor.info;
}

absl::Status PrintEntityRecursive(const ObjectWorld& world,
                                  AttachmentEntityId entity_id, int level,
                                  bool enable_colors, std::string& output) {
  INTR_ASSIGN_OR_RETURN(const WorldEntity* entity,
                        world.GetEntityWorld().GetEntityById(entity_id));

  // Find matching object/frame ids for given entity. There should be exactly
  // one match if everything is OK.
  std::vector<std::string> matching_names;
  for (const WorldObject* object : world.GetObjectsSorted()) {
    if (object->GetEntityIds().contains(entity_id)) {
      matching_names.push_back(object->GetName().value());
    }

    for (const Frame* frame : object->GetFramesSorted()) {
      if (entity_id == frame->GetEntityId()) {
        matching_names.push_back(absl::StrCat(object->GetName().value(), ".",
                                              frame->GetName().value()));
      }
    }
  }

  for (int i = 0; i < level; i++) {
    absl::StrAppend(&output, " ");
  }

  absl::StrAppend(&output, BeginColor("", enable_colors), "[",
                  absl::StrJoin(matching_names, ", "), "] ",
                  EndColor(enable_colors), entity_id.value());

  if (!entity->GetAlias().empty()) {
    absl::StrAppend(&output, ", alias=", entity->GetAlias());
  }
  if (!entity->GetLocalName().empty()) {
    absl::StrAppend(&output, ", local_name=", entity->GetLocalName());
  }
  if (!entity->GetLabels().empty()) {
    absl::StrAppend(
        &output, ", labels={",
        absl::StrJoin(entity->GetLabels(), ", ", absl::StreamFormatter()), "}");
  }
  absl::StrAppend(&output, "\n");

  for (const AttachmentEntityId& child_id :
       world.GetEntityWorld().GetChildrenOf(entity_id)) {
    INTR_RETURN_IF_ERROR(PrintEntityRecursive(world, child_id, level + 1,
                                              enable_colors, output));
  }

  return absl::OkStatus();
}

absl::StatusOr<std::string> PrintEntityToObjectMapping(const ObjectWorld& world,
                                                       bool enable_colors) {
  std::string output;
  INTR_RETURN_IF_ERROR(PrintEntityRecursive(world, kRootEntityId,
                                            /*level=*/0, enable_colors,
                                            output));
  return absl::StrCat(
      "Mapping from attachment entities to objects:\n\nFormat: ",
      BeginColor("", enable_colors), "[<object/frame name>*]",
      EndColor(enable_colors),
      " <entity id> <further entity details>\n  where <object/frame name> == "
      "<object name> or <object name>.<frame name>\n\n",
      output);
}

}  // namespace intrinsic::object_world
