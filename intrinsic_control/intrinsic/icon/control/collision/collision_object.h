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

#ifndef INTRINSIC_ICON_CONTROL_COLLISION_COLLISION_OBJECT_H_
#define INTRINSIC_ICON_CONTROL_COLLISION_COLLISION_OBJECT_H_

#include <cstddef>
#include <limits>
#include <vector>

#include "intrinsic/icon/control/collision/collision_data_types.h"
#include "intrinsic/icon/control/collision/collision_error.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic {
namespace collision {

// Structure defining a plane geometry
struct PlaneGeometry {
  // Reference point on the plane in world frame
  TranslationVector origin = TranslationVector::Zero();
  // Unit vector normal to the plane at origin in world frame
  // The area above the plane in the direction of normal vector
  // is considered to be collision-free whereas the area below the plane
  // in the direction opposite of the normal vector is considered
  // to be in collision
  TranslationVector unit_normal_vector = TranslationVector::Zero();
};

// Structure defining a spherical geometry
struct SphereGeometry {
  // Radius of the sphere
  double radius = 0.0;
  // Center of the sphere w.r.t. object's frame
  TranslationVector center = TranslationVector::Zero();
};

// Structure defining a capsule geometry
struct CapsuleGeometry {
  // Radius of the capsule
  double radius = 0.0;
  // Endpoints of the capsule w.r.t. object's frame
  TranslationVector endpoint1 = TranslationVector::Zero();
  TranslationVector endpoint2 = TranslationVector::Zero();
};

// Structure containing the output information for collision check
struct CollisionResult {
  // Boolean flag indicating whether collision occurs or not
  bool collision_occurs = false;
  // Minimum distance from collision
  // A non-positive value indicates that collision has occurred
  double min_distance = std::numeric_limits<double>::infinity();
  // Contact normal
  // Unit vector representing the direction of escape in world frame
  TranslationVector world_unit_contact_normal_vector =
      TranslationVector::Zero();
  // Unit vector representing the direction of escape in object frame
  TranslationVector unit_contact_normal_vector = TranslationVector::Zero();
  // (Actual/Potential) Contact Point
  // Point of collision on the object in world frame
  TranslationVector world_contact_point = TranslationVector::Zero();
  // Point of collision on the object in object frame
  TranslationVector contact_point = TranslationVector::Zero();
};

// Enumeration of the collision pairs
// NOTE: Plane-Plane collision pairs are not supported here
enum class CollisionPair {
  PLANE_SPHERE = 0,
  PLANE_CAPSULE,
  SPHERE_SPHERE,
  SPHERE_CAPSULE,
  CAPSULE_CAPSULE,
  NUM_PAIRS
};

// Structure containing the collision geometry pair information
struct CollisionGeometryPairInfo {
  size_t object_index1;
  size_t object_index2;
  size_t geometry_index1;
  size_t geometry_index2;
  CollisionPair collision_pair;
  CollisionGeometryPairInfo(size_t obj_ind1, size_t obj_ind2, size_t geom_ind1,
                            size_t geom_ind2, CollisionPair pair)
      : object_index1(obj_ind1),
        object_index2(obj_ind2),
        geometry_index1(geom_ind1),
        geometry_index2(geom_ind2),
        collision_pair(pair) {}
};

// Class defining a collision object
class CollisionObject {
 public:
  // Constructor
  CollisionObject() = default;

  // Copy constructor of class CollisionObject
  // collision_object Object to be copied
  CollisionObject(const CollisionObject& collision_object) = default;

  // Destructor of class CollisionObject
  ~CollisionObject() = default;

  // Add a plane geometry
  // This function is NOT Realtime Safe!
  // plane_geometry PlaneGeometry to be added
  void AddPlaneGeometry(const PlaneGeometry& plane_geometry) {
    INTRINSIC_ASSERT_NON_REALTIME();
    plane_geometries_.push_back(plane_geometry);
  }

  // Add a spherical geometry
  // This function is NOT Realtime Safe!
  // sphere_geometry SphereGeometry to be added
  CollisionError AddSphereGeometry(const SphereGeometry& sphere_geometry) {
    INTRINSIC_ASSERT_NON_REALTIME();
    if (sphere_geometry.radius < 0.0) {
      return CollisionError::INVALID_RADIUS_FOR_COLLISION_GEOMETRY;
    }
    sphere_geometries_.push_back(sphere_geometry);
    return CollisionError::NO_ERROR;
  }

  // Add a capsule geometry
  // This function is NOT Realtime Safe!
  // capsule_geometry CapsuleGeometry to be added
  CollisionError AddCapsuleGeometry(const CapsuleGeometry& capsule_geometry) {
    INTRINSIC_ASSERT_NON_REALTIME();
    if (capsule_geometry.radius < 0.0) {
      return CollisionError::INVALID_RADIUS_FOR_COLLISION_GEOMETRY;
    }
    capsule_geometries_.push_back(capsule_geometry);
    return CollisionError::NO_ERROR;
  }

  // Set object transformation w.r.t. world frame
  // rotation_matrix 3x3 matrix representing the rotation of the
  // object's reference frame in world frame
  // translation_vector 3x1 vector representing the translation of the
  // object's reference frame in world frame
  void SetWorldObjectTransformation(
      const RotationMatrix& rotation_matrix,
      const TranslationVector& translation_vector) {
    world_object_rotation_ = rotation_matrix;
    world_object_translation_ = translation_vector;
  }

  // Get number of collision planes.
  size_t GetNumberOfPlaneGeometries() const { return plane_geometries_.size(); }

  // Get number of collision spheres.
  size_t GetNumberOfSphereGeometries() const {
    return sphere_geometries_.size();
  }

  // Get number of collision capsules.
  size_t GetNumberOfCapsuleGeometries() const {
    return capsule_geometries_.size();
  }

  // Get collision plane
  // plane_geom_index Index of the plane geometry that is to be accessed.
  // Returns PlaneGeometry object.
  icon::RealtimeStatusOr<PlaneGeometry> GetPlaneGeometry(
      const size_t plane_geom_index) const {
    if (plane_geom_index < GetNumberOfPlaneGeometries()) {
      return PlaneGeometry(plane_geometries_[plane_geom_index]);
    } else {
      return icon::OutOfRangeError("invalid plane geometry index");
    }
  }

  // Get collision sphere
  // sphere_geom_index Index of the spherical geometry that is to be
  // accessed.
  // Returns SphereGeometry object.
  icon::RealtimeStatusOr<SphereGeometry> GetSphereGeometry(
      const size_t sphere_geom_index) const {
    if (sphere_geom_index < GetNumberOfSphereGeometries()) {
      return SphereGeometry(sphere_geometries_[sphere_geom_index]);
    } else {
      return icon::OutOfRangeError("invalid sphere geometry index");
    }
  }

  // Get collision capsule
  // capsule_geom_index Index of the capsule geometry that is to be
  // accessed.
  // Returns CapsuleGeometry object if there is no error else the
  // CollisionError object
  icon::RealtimeStatusOr<CapsuleGeometry> GetCapsuleGeometry(
      const size_t capsule_geom_index) const {
    if (capsule_geom_index < GetNumberOfCapsuleGeometries()) {
      return CapsuleGeometry(capsule_geometries_[capsule_geom_index]);
    } else {
      return icon::OutOfRangeError("invalid capsule geometry index");
    }
  }

  // Get rotation of the object's reference frame w.r.t. world frame
  // Return Const reference to the rotation matrix w.r.t. world frame.
  const RotationMatrix& GetWorldObjectRotation() const {
    return world_object_rotation_;
  }

  // Get translation of the object's reference frame w.r.t. world frame
  // Return Const reference to the translation vector w.r.t. world frame.
  const TranslationVector& GetWorldObjectTranslation() const {
    return world_object_translation_;
  }

 private:
  // Vector of plane geometries
  std::vector<PlaneGeometry> plane_geometries_;
  // Vector of spherical geometries
  std::vector<SphereGeometry> sphere_geometries_;
  // Vector of capsule geometries
  std::vector<CapsuleGeometry> capsule_geometries_;
  // Rotation of the object's reference frame in world frame
  RotationMatrix world_object_rotation_ = RotationMatrix::Identity();
  // Position of the object's reference frame in world frame
  TranslationVector world_object_translation_ = TranslationVector::Zero();
};

}  // namespace collision
}  // namespace intrinsic

#endif  // INTRINSIC_ICON_CONTROL_COLLISION_COLLISION_OBJECT_H_
