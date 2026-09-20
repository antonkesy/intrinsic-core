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

#ifndef INTRINSIC_ICON_CONTROL_COLLISION_COLLISION_WORLD_LOADER_H_
#define INTRINSIC_ICON_CONTROL_COLLISION_COLLISION_WORLD_LOADER_H_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "intrinsic/icon/control/collision/collision_world.h"

namespace intrinsic {

namespace collision {

// Enumeration class containing the status or error messages associated with
// loading CollisionWorld
enum class LoaderStatus {
  // Success
  SUCCESS,
  // Invalid number of collision objects
  ERROR_INVALID_NUM_COLLISION_OBJECTS,
  // Invalid position of collision geometry
  ERROR_INVALID_POSITION,
  // Invalid orientation of collision geometry
  ERROR_INVALID_ORIENTATION,
  // Multiple geometries defined in a single collision geometry
  ERROR_MULTIPLE_GEOMETRIES_DEFINED,
  // No geometries defined in a single collision geometry
  ERROR_NO_GEOMETRY_DEFINED,
  // Invalid radius for spherical/capsule geometry
  ERROR_INVALID_RADIUS,
  // Invalid length for capsule geometry
  ERROR_INVALID_LENGTH,
  // Invalid box size for collision plane
  ERROR_INVALID_BOX_SIZE,
  // Invalid use of collision plane
  ERROR_INVALID_USE_COLLISION_PLANE,
  // Error in creating collision world
  ERROR_CREATE_COLLISION_WORLD,
  // Invalid number of link names
  ERROR_INVALID_NUM_LINK_NAMES,
  // Duplicate link names
  ERROR_DUPLICATE_LINK_NAMES,
  // Link name from URDF is not found in the input list
  ERROR_LINK_NAME_NOT_FOUND,
  // ConfigNode invalid
  ERROR_CONFIG_NODE_INVALID
};

// Structure defining the transformation (position and rotation) of a collision
// geometry in object's frame in URDF
struct Transformation {
  std::vector<double> rpy;
  std::vector<double> xyz;
};
// Structure defining a collision plane as a box in URDF
struct Plane {
  std::vector<double> size;
  Plane() = default;
  explicit Plane(std::vector<double> size_) : size(size_) {}
};
// Structure defining a sphere collision geometry in URDF
struct Sphere {
  double radius;
  Sphere() : radius(0.0) {}
  explicit Sphere(double radius_) : radius(radius_) {}
};
// Structure defining a capsule collision geometry in URDF
struct Capsule {
  double radius;
  double length;
  Capsule() : radius(0.0), length(0.0) {}
  Capsule(double radius_, double length_) : radius(radius_), length(length_) {}
};
// Structure defining the geometries in URDF
struct Geometry {
  std::unique_ptr<Plane> box = nullptr;
  std::unique_ptr<Sphere> sphere = nullptr;
  std::unique_ptr<Capsule> cylinder = nullptr;
  void copy(const Geometry& geometry_to_copy) {
    if (geometry_to_copy.box != nullptr) {
      box = std::make_unique<Plane>(geometry_to_copy.box->size);
    }
    if (geometry_to_copy.sphere != nullptr) {
      sphere = std::make_unique<Sphere>(geometry_to_copy.sphere->radius);
    }
    if (geometry_to_copy.cylinder != nullptr) {
      cylinder = std::make_unique<Capsule>(geometry_to_copy.cylinder->radius,
                                           geometry_to_copy.cylinder->length);
    }
  }
};

// Structure defining a collision geometry in URDF
struct Collision {
  Transformation origin;
  Geometry geometry;
  void copy(const Collision& collision_to_copy) {
    origin = collision_to_copy.origin;
    geometry.copy(collision_to_copy.geometry);
  }
};

// Structure defining a collision object in URDF
struct Object {
  std::string name;
  std::vector<Collision> collision;
  void AddCollisionGeometries(const std::vector<Collision>& collision_to_add) {
    size_t new_ind = collision.size();
    collision.resize(collision.size() + collision_to_add.size());
    for (const auto& geom : collision_to_add) {
      collision[new_ind].copy(geom);
      new_ind++;
    }
  }
};

// Structure defining the list of collision objects in URDF
struct CollisionModel {
  std::vector<Object> link;
};

// Structure defining the collision geometries to be added in addition
// to the ones in the URDF
struct AdditionalCollisionGeometries {
  std::vector<Collision> collision;
};

// Structure defining the collision check information for a link
// in the XML
struct ObjectInfo {
  std::string name;
  std::vector<std::string> links_ignore_collision;
  std::unique_ptr<AdditionalCollisionGeometries>
      additional_collision_geometries;
};

// Structure defining the collision check information
// for each link in the XML
struct CollisionCheckInfo {
  std::vector<ObjectInfo> link_info;
};

// Initializes `collision_world` and `link_names_in_index_order` from
// `collision_model` and `collision_check_info`.
LoaderStatus LoadCollisionWorld(
    CollisionModel&& collision_model, CollisionCheckInfo&& collision_check_info,
    CollisionWorld& collision_world,
    std::vector<std::string>& link_names_in_index_order);

// Print loader status message
// Returns string containing the loader status message.
const char* GetLoaderStatusMessage(const LoaderStatus& status);

// Checks whether the given collision_model is valid or not and returns a
// corresponding LoaderStatus.
LoaderStatus CheckGeometryValidity(const CollisionModel& collision_model);

}  // namespace collision

}  // namespace intrinsic

#endif  // INTRINSIC_ICON_CONTROL_COLLISION_COLLISION_WORLD_LOADER_H_
