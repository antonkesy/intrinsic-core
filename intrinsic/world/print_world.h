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

#ifndef INTRINSIC_WORLD_PRINT_WORLD_H_
#define INTRINSIC_WORLD_PRINT_WORLD_H_

#include <functional>
#include <ostream>
#include <string>

#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/world.h"

namespace intrinsic {

// Prints the world recursively, using `format` string to format the arguments.
//
// PrintfWorldOptions contains additional options for formatting the output.
struct PrintfWorldOptions {
  // This indentation string will be applied N times at the start of each print
  // and also before each newline in the format string.
  std::string indent_str = "  ";
};
// Supported options:
// "{id}": id of the entity,
// "{local_name}": local_name of the entity,
// "{alias}": alias,
// "{groups}": groups that the entity is part of,
// "{labels}": labels on the entity,
// "{udm}": user data map, prints the string to string map only,
// "{pose_a}": absolute pose of the entity (root_t_entity),
// "{pose_r}": relative pose of the entity (parent_t_entity),
void PrintfEntity(const World& world, const EntityId& id,
                  const std::string& format, const std::string& prefix,
                  std::ostream* output_stream,
                  const PrintfWorldOptions& options);
void PrintfWorld(const World& world, const std::string& format,
                 std::ostream* output_stream,
                 const PrintfWorldOptions& options = {});

// Outputs debug information about 'world'. Output contains the physical object
// tree with information about all shapes, Grouping, and which objects are
// joints.
void PrintAspectsTo(const World& world, std::ostream* output_stream);

// A more condensed version of the prints that focusses on displaying the
// Physical tree making it easier to see the connections between objects.
void PrintPhysicalTreeTo(const World& world, std::ostream* output_stream);

// Prints pairs of colliding objects to the stream. Allows the user to provide a
// function for printing the ids so that it can be customized for different
// surfaces.
//
// WARNING: this functions assumes a particular collision RuleSet which affects
// what is considered a collision. What is printed may not match your results if
// you are using a different RuleSet!
// TODO(b/141769701): Address the above at the source call.
void PrintCollisionsTo(
    const World& world, std::ostream* output_stream,
    std::function<std::string(const World& world, const EntityId& entity_id)>
        entity_id_to_string = nullptr);

// Same as above, but only prints collisions where at least one of the objects
// involved is in `objects`.
void PrintCollisionsForSpecifiedObjectsTo(
    const World& world, const WorldHashSet<PhysicalEntityId>& objects,
    std::ostream* output_stream,
    std::function<std::string(const World& world, const EntityId& entity_id)>
        entity_id_to_string = nullptr);

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_PRINT_WORLD_H_
