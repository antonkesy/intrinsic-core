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

#include "intrinsic/icon/control/collision/collision_world_loader.h"

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "intrinsic/icon/control/collision/collision_data_types.h"
#include "intrinsic/icon/control/collision/collision_error.h"
#include "intrinsic/icon/control/collision/collision_object.h"
#include "intrinsic/icon/control/collision/collision_utility_functions.h"
#include "intrinsic/icon/control/collision/collision_world.h"

namespace intrinsic {

namespace collision {

static constexpr double kRedundantSphereThreshold = 1e-3;

LoaderStatus CheckGeometryValidity(const CollisionModel& collision_model) {
  // Check for valid number of objects.
  size_t num_objects = collision_model.link.size();
  if (num_objects < 1) {
    return LoaderStatus::ERROR_INVALID_NUM_COLLISION_OBJECTS;
  }

  // Check for validity of geometry parameters.
  // Also check whether a single collision geometry has only one geometry
  // (either sphere or plane) defined.
  for (size_t obj_ind = 0; obj_ind < num_objects; obj_ind++) {
    for (size_t geom_ind = 0;
         geom_ind < collision_model.link[obj_ind].collision.size();
         geom_ind++) {
      // Check if position and orientation of collision geometry are valid.
      if (collision_model.link[obj_ind].collision[geom_ind].origin.xyz.size() !=
          3) {
        return LoaderStatus::ERROR_INVALID_POSITION;
      }
      if (collision_model.link[obj_ind].collision[geom_ind].origin.rpy.size() !=
          3) {
        return LoaderStatus::ERROR_INVALID_ORIENTATION;
      }
      // Check if sphere.
      if (collision_model.link[obj_ind].collision[geom_ind].geometry.sphere !=
          nullptr) {
        // Check if plane is also defined (error).
        if (collision_model.link[obj_ind].collision[geom_ind].geometry.box !=
            nullptr) {
          return LoaderStatus::ERROR_MULTIPLE_GEOMETRIES_DEFINED;
        }
        // Check if radius is valid.
        if (collision_model.link[obj_ind]
                .collision[geom_ind]
                .geometry.sphere->radius < 0.0) {
          return LoaderStatus::ERROR_INVALID_RADIUS;
        }
      } else if (collision_model.link[obj_ind]
                     .collision[geom_ind]
                     .geometry.cylinder != nullptr) {
        // Check if capsule.
        // Check if plane is also defined (error).
        if (collision_model.link[obj_ind].collision[geom_ind].geometry.box !=
            nullptr) {
          return LoaderStatus::ERROR_MULTIPLE_GEOMETRIES_DEFINED;
        }
        // check if radius is valid
        if (collision_model.link[obj_ind]
                .collision[geom_ind]
                .geometry.cylinder->radius < 0.0) {
          return LoaderStatus::ERROR_INVALID_RADIUS;
        }
        // check if length is valid
        if (collision_model.link[obj_ind]
                .collision[geom_ind]
                .geometry.cylinder->length < 0.0) {
          return LoaderStatus::ERROR_INVALID_LENGTH;
        }
      } else {
        // check if plane is defined, else error
        if (collision_model.link[obj_ind].collision[geom_ind].geometry.box ==
            nullptr) {
          return LoaderStatus::ERROR_NO_GEOMETRY_DEFINED;
        }
        // check if box size is valid
        if (collision_model.link[obj_ind]
                .collision[geom_ind]
                .geometry.box->size.size() != 3) {
          return LoaderStatus::ERROR_INVALID_BOX_SIZE;
        }
      }
    }
  }

  // Only the first object (base link or world link) is allowed to have
  // collision planes.
  // Check for invalid use of collision planes in other objects.
  for (size_t obj_ind = 1; obj_ind < num_objects; obj_ind++) {
    for (size_t geom_ind = 0;
         geom_ind < collision_model.link[obj_ind].collision.size();
         geom_ind++) {
      if (collision_model.link[obj_ind].collision[geom_ind].geometry.box !=
          nullptr) {
        return LoaderStatus::ERROR_INVALID_USE_COLLISION_PLANE;
      }
    }
  }
  return LoaderStatus::SUCCESS;
}

LoaderStatus LoadCollisionWorld(
    CollisionModel&& collision_model, CollisionCheckInfo&& collision_check_info,
    CollisionWorld& collision_world,
    std::vector<std::string>& link_names_in_index_order) {
  LoaderStatus validity = CheckGeometryValidity(collision_model);
  if (validity != LoaderStatus::SUCCESS) {
    return validity;
  }

  size_t num_objects = collision_model.link.size();
  link_names_in_index_order.resize(num_objects);

  // Check for valid number of links in the collision check info.
  if (collision_check_info.link_info.size() != num_objects) {
    return LoaderStatus::ERROR_INVALID_NUM_LINK_NAMES;
  }
  // Create a hashmap for the input link names.
  absl::flat_hash_map<std::string, size_t> link_name_hash_map;
  for (size_t link_ind = 0; link_ind < num_objects; link_ind++) {
    // Add if unique else report error.
    std::string link_name = collision_check_info.link_info[link_ind].name;
    if (bool inserted =
            link_name_hash_map.try_emplace(link_name, link_ind).second;
        !inserted) {
      return LoaderStatus::ERROR_DUPLICATE_LINK_NAMES;
    }
    // Add additional collision geometries, if any.
    if (collision_check_info.link_info[link_ind]
            .additional_collision_geometries != nullptr) {
      collision_model.link[link_ind].AddCollisionGeometries(
          collision_check_info.link_info[link_ind]
              .additional_collision_geometries->collision);
    }
  }

  // Set-up collision world

  // Vector of collision objects.
  std::vector<CollisionObject> collision_objects;
  collision_objects.resize(num_objects);
  // Get collision objects.
  for (size_t obj_ind = 0; obj_ind < num_objects; obj_ind++) {
    CollisionObject collision_obj;
    std::vector<SphereGeometry> spheres;
    std::vector<CapsuleGeometry> capsules;
    for (size_t geom_ind = 0;
         geom_ind < collision_model.link[obj_ind].collision.size();
         geom_ind++) {
      // Add sphere if needed.
      if (collision_model.link[obj_ind].collision[geom_ind].geometry.sphere !=
          nullptr) {
        SphereGeometry sphere_geom;
        sphere_geom.radius = collision_model.link[obj_ind]
                                 .collision[geom_ind]
                                 .geometry.sphere->radius;
        sphere_geom.center = TranslationVector(collision_model.link[obj_ind]
                                                   .collision[geom_ind]
                                                   .origin.xyz.data());
        spheres.push_back(sphere_geom);
      } else if (collision_model.link[obj_ind]
                     .collision[geom_ind]
                     .geometry.cylinder != nullptr) {
        // Add capsule if needed.
        CapsuleGeometry capsule_geom;
        capsule_geom.radius = collision_model.link[obj_ind]
                                  .collision[geom_ind]
                                  .geometry.cylinder->radius;
        double capsule_length = collision_model.link[obj_ind]
                                    .collision[geom_ind]
                                    .geometry.cylinder->length;
        TranslationVector capsule_center(collision_model.link[obj_ind]
                                             .collision[geom_ind]
                                             .origin.xyz.data());
        // Compute end points.
        // Get rotation of the capsule axis.
        RotationMatrix capsule_rotation = GetRPYRotationMatrix(
            collision_model.link[obj_ind].collision[geom_ind].origin.rpy[0],
            collision_model.link[obj_ind].collision[geom_ind].origin.rpy[1],
            collision_model.link[obj_ind].collision[geom_ind].origin.rpy[2]);
        capsule_geom.endpoint1 =
            capsule_center +
            capsule_rotation *
                TranslationVector(0.0, 0.0, -0.5 * capsule_length);
        capsule_geom.endpoint2 =
            capsule_center +
            capsule_rotation *
                TranslationVector(0.0, 0.0, 0.5 * capsule_length);
        capsules.push_back(capsule_geom);
      } else {
        // Add plane.
        // Planes can be used only in obj_ind = 0.
        // There is a check above that returns error
        // LoaderStatus::ERROR_INVALID_USE_COLLISION_PLANE if this is not the
        // case. Also assumes that the first object's frame and center aligns
        // with that of the world compute plane's frame transformation in
        // world frame.
        RotationMatrix plane_rotation = GetRPYRotationMatrix(
            collision_model.link[obj_ind].collision[geom_ind].origin.rpy[0],
            collision_model.link[obj_ind].collision[geom_ind].origin.rpy[1],
            collision_model.link[obj_ind].collision[geom_ind].origin.rpy[2]);
        // Create plane geometry.
        PlaneGeometry plane_geom;
        plane_geom.origin = TranslationVector(collision_model.link[obj_ind]
                                                  .collision[geom_ind]
                                                  .origin.xyz.data());
        // The normal is assumed to be +Z in the plane's frame and
        // it is transformed into world frame.
        plane_geom.unit_normal_vector = plane_rotation.col(2);
        collision_obj.AddPlaneGeometry(plane_geom);
      }
    }
    // Find and remove redundant spheres.
    for (size_t sphere_ind = 0; sphere_ind < spheres.size(); sphere_ind++) {
      const SphereGeometry& sphere_geom = spheres[sphere_ind];
      for (const auto& capsule_geom : capsules) {
        if ((((sphere_geom.center - capsule_geom.endpoint1).norm() <=
              kRedundantSphereThreshold) ||
             ((sphere_geom.center - capsule_geom.endpoint2).norm() <=
              kRedundantSphereThreshold)) &&
            ((sphere_geom.radius - capsule_geom.radius) <=
             kRedundantSphereThreshold)) {
          // Redundant sphere found.
          // Remove it and decrement the counter.
          spheres.erase(spheres.begin() + sphere_ind);
          sphere_ind--;
          break;
        }
      }
    }
    // Add the non-redundant sphere and capsule geometries.
    for (const auto& capsule_geom : capsules) {
      collision_obj.AddCapsuleGeometry(capsule_geom);
    }
    for (const auto& sphere_geom : spheres) {
      collision_obj.AddSphereGeometry(sphere_geom);
    }
    // Add to the vector in its correct place.
    std::string link_name = collision_model.link[obj_ind].name;
    auto found_link_ind = link_name_hash_map.find(link_name);
    if (found_link_ind == link_name_hash_map.end()) {
      return LoaderStatus::ERROR_LINK_NAME_NOT_FOUND;
    }
    // If found, add to its appropriate index.
    collision_objects[found_link_ind->second] = collision_obj;
    link_names_in_index_order[found_link_ind->second] = link_name;
  }

  // Create collision world.
  CollisionError collision_error =
      collision_world.CreateCollisionWorld(collision_objects);
  // Check for collision world error.
  if (collision_error != CollisionError::NO_ERROR) {
    return LoaderStatus::ERROR_CREATE_COLLISION_WORLD;
  }

  // Enable collision check for object-object pairs that have not been
  // disabled.
  for (size_t obj_ind1 = 0; obj_ind1 < collision_objects.size(); obj_ind1++) {
    // Create a hashmap of the links that need to be ignored for collision
    // checking.
    const ObjectInfo& link_info = collision_check_info.link_info[obj_ind1];
    absl::flat_hash_set<std::string> disabled_links;
    for (size_t link_ind = 0;
         link_ind < link_info.links_ignore_collision.size(); link_ind++) {
      // Add if unique else report error.
      std::string link_name = link_info.links_ignore_collision[link_ind];
      if (bool inserted = disabled_links.insert(link_name).second; !inserted) {
        return LoaderStatus::ERROR_DUPLICATE_LINK_NAMES;
      }
    }
    // Object-object pairs.
    for (size_t obj_ind2 = 0; obj_ind2 < collision_objects.size(); obj_ind2++) {
      // Ignore self-collision check.
      if (obj_ind1 == obj_ind2) {
        continue;
      }
      // Check if this link is to be ignored, if not enable collision check
      // between them.
      std::string link_name = collision_check_info.link_info[obj_ind2].name;
      if (!disabled_links.contains(link_name)) {
        collision_world.EnableObjectObjectCollisionCheck(obj_ind1, obj_ind2);
      }
    }
  }
  // Return success
  return LoaderStatus::SUCCESS;
}

const char* GetLoaderStatusMessage(const LoaderStatus& status) {
  switch (status) {
    case LoaderStatus::SUCCESS:
      return "LoaderStatus: No Error!";
    case LoaderStatus::ERROR_MULTIPLE_GEOMETRIES_DEFINED:
      return "LoaderStatus: Error: multiple geoms defined in the same "
             "collision geometry!";
    case LoaderStatus::ERROR_CREATE_COLLISION_WORLD:
      return "LoaderStatus: Error: creating CollisionWorld failed!";
    case LoaderStatus::ERROR_INVALID_BOX_SIZE:
      return "LoaderStatus: Error: invalid number of values in plane geom "
             "definition!";
    case LoaderStatus::ERROR_INVALID_NUM_COLLISION_OBJECTS:
      return "LoaderStatus: Error: invalid number of collision objects!";
    case LoaderStatus::ERROR_INVALID_ORIENTATION:
      return "LoaderStatus: Error: invalid value for orientation (rpy)!";
    case LoaderStatus::ERROR_INVALID_POSITION:
      return "LoaderStatus: Error: invalid value for position (xyz)!";
    case LoaderStatus::ERROR_INVALID_RADIUS:
      return "LoaderStatus: Error: invalid radius for sphere/capsule geometry!";
    case LoaderStatus::ERROR_INVALID_LENGTH:
      return "LoaderStatus: Error: invalid length for capsule geometry!";
    case LoaderStatus::ERROR_INVALID_USE_COLLISION_PLANE:
      return "LoaderStatus: Error: CollisionPlane can be used only in the base "
             "link!";
    case LoaderStatus::ERROR_INVALID_NUM_LINK_NAMES:
      return "LoaderStatus: Error: Invalid number of link names provided!";
    case LoaderStatus::ERROR_DUPLICATE_LINK_NAMES:
      return "LoaderStatus: Error: There are duplicate link names in the list "
             "provided!";
    case LoaderStatus::ERROR_LINK_NAME_NOT_FOUND:
      return "LoaderStatus: Error: Link name in the URDF is not found in the "
             "list provided!";
    case LoaderStatus::ERROR_NO_GEOMETRY_DEFINED:
      return "LoaderStatus: Error: No geometry defined!";
    case LoaderStatus::ERROR_CONFIG_NODE_INVALID:
      return "LoaderStatus: Error: Invalid ConfigNode structure";
  }
  LOG(FATAL)
      << "Bug in core::collision::GetLoaderStatusMessage: unhandled enum "
         "value "
      << static_cast<int>(status);
}

}  // namespace collision
}  // namespace intrinsic
