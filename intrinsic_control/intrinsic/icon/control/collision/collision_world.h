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

#ifndef INTRINSIC_ICON_CONTROL_COLLISION_COLLISION_WORLD_H_
#define INTRINSIC_ICON_CONTROL_COLLISION_COLLISION_WORLD_H_

#include <array>
#include <cstddef>
#include <limits>
#include <ostream>
#include <vector>

#include "intrinsic/icon/control/collision/collision_data_types.h"
#include "intrinsic/icon/control/collision/collision_error.h"
#include "intrinsic/icon/control/collision/collision_object.h"

namespace intrinsic {

namespace collision {

static constexpr double kZeroThreshold = 1e-20;

// Class for a world of collision objects and collision planes
class CollisionWorld {
 public:
  // Constructor
  CollisionWorld() = default;

  // Copy constructor of class CollisionWorld
  // collision_world Object to be copied
  CollisionWorld(const CollisionWorld& collision_world) = default;

  // Destructor of class CollisionWorld
  ~CollisionWorld() = default;

  // Overloading operator to print contents of CollisionResult of all objects
  // out std::ostream object to which the data is to be output
  // Returns  std::ostream object
  std::ostream& operator<<(std::ostream& out);

  // Main collision check function for the collision world
  // collision_result Pointer to vector of CollisionResult where the collision
  // data for all objects are copied. Returns true if collision occurs else
  // false.
  bool CheckCollision(std::vector<CollisionResult>* collision_result);

  // Create collision world
  // This function is NOT Realtime Safe!
  // collision_objects Vector of collision objects defined in the world.
  // Returns CollisionError object.
  CollisionError CreateCollisionWorld(
      const std::vector<CollisionObject>& collision_objects);

  // Enable collision check for all pairs of geometries of objects
  // This excludes collision check of a geometry with itself and
  // also between the geometries of the same object
  void EnableAllObjectObjectCollisionChecks();

  // Enable collision check for all pairs of geometries for a pair of objects
  // using their indices object_index1 Index of the first collision
  // object in the pair object_index2 Index of the second collision
  // object in the pair.
  // Returns CollisionError object.
  CollisionError EnableObjectObjectCollisionCheck(size_t object_index1,
                                                  size_t object_index2);

  // Set object transformation w.r.t. world frame
  // collision_object_index Index of the collision object whose
  // world transformation are to be set
  // rotation_matrix 3x3 matrix representing the rotation of the
  // object's reference frame in world frame
  // translation_vector 3x1 vector representing the translation of the
  // object's reference frame in world frame.
  // Returns CollisionError object.
  CollisionError SetWorldObjectTransformation(
      size_t collision_object_index, const RotationMatrix& rotation_matrix,
      const TranslationVector& translation_vector);

  // Get number of collision objects.
  size_t GetNumberOfCollisionObjects() const {
    return collision_objects_.size();
  }

  // Get the minimum collision distance among all collision objects
  double GetMinCollisionDistance() const { return min_collision_distance_; }

  // Check whether CollisionWorld is valid or not
  bool IsValid() const { return is_valid_; }

 private:
  // Internal collision checking functions between different geometry pairs
  void ComputePlaneSphereCollisionResult(
      size_t plane_index, size_t sphere_index, double* collision_distance,
      TranslationVector* world_unit_contact_normal,
      TranslationVector* world_contact_point1,
      TranslationVector* world_contact_point2);
  void ComputePlaneCapsuleCollisionResult(
      size_t plane_index, size_t capsule_index, double* collision_distance,
      TranslationVector* world_unit_contact_normal,
      TranslationVector* world_contact_point1,
      TranslationVector* world_contact_point2);
  void ComputeSphereSphereCollisionResult(
      size_t sphere_index1, size_t sphere_index2, double* collision_distance,
      TranslationVector* world_unit_contact_normal,
      TranslationVector* world_contact_point1,
      TranslationVector* world_contact_point2);
  void ComputeSphereCapsuleCollisionResult(
      size_t sphere_index, size_t capsule_index, double* collision_distance,
      TranslationVector* world_unit_contact_normal,
      TranslationVector* world_contact_point1,
      TranslationVector* world_contact_point2);
  void ComputeCapsuleCapsuleCollisionResult(
      size_t capsule_index1, size_t capsule_index2, double* collision_distance,
      TranslationVector* world_unit_contact_normal,
      TranslationVector* world_contact_point1,
      TranslationVector* world_contact_point2);
  // Internal function to compute contact distance and contact normal between
  // a point and a line (capsule)
  void ComputeContactInfoPointLine(
      size_t capsule_index, const TranslationVector& point,
      double* contact_distance, TranslationVector* world_unit_contact_normal,
      TranslationVector* world_axial_contact_point);
  // Internal function to compute contact distance and contact normal between
  // two lines (capsule)
  // Returns true if a valid intersection point between the line segments is
  // found
  bool ComputeContactInfoLineLine(
      size_t capsule_index1, size_t capsule_index2, double* contact_distance,
      TranslationVector* world_unit_contact_normal,
      TranslationVector* world_axial_contact_point1,
      TranslationVector* world_axial_contact_point2);

  // Internal function to find a contact normal when capsule axes are in line
  // or the sphere center is on the capsule axis
  void ComputeValidCapsuleContactNormal(size_t capsule_index,
                                        TranslationVector* contact_normal);

  // Internal function to compute and update world geometry transformations
  // for all collision geometries
  void UpdateWorldGeometryTranslations(size_t obj_index);
  void UpdateWorldGeometryTranslations();

  // Internal function to transform contact points and unit contact normal
  // vectors for all collision objects from world frame to their respective
  // object frames
  void TransformContactDataToObjectFrame();

  // Internal function to update collision data if necessary
  // obj_index Index of the collision object whose collision data is
  // provided collision_distance Distance to collision \param
  // world_unit_contact_normal Unit vector representing the escape direction
  // for the input object to move away from collision \param
  // world_contact_point Closest actual/potential contact point on the surface
  // of the colliding geometry in world frame
  void UpdateCollisionResultIfNecessary(
      size_t obj_index, double collision_distance,
      const TranslationVector& world_unit_contact_normal,
      const TranslationVector& world_contact_point);

  // Internal Function to reset the vector of object collision data to default
  // values
  void ResetObjectCollisionResult();

  // Internal Function to reset collision geometries related info
  void ResetCollisionGeometries();

  // Collision Check Function Pointer definition
  typedef void (CollisionWorld::*collisionCheckFunction)(
      const size_t plane_index, const size_t sphere_index,
      double* collision_distance, TranslationVector* world_unit_contact_normal,
      TranslationVector* world_contact_point1,
      TranslationVector* world_contact_point2);

  // Array of collision checking function pointers
  std::array<collisionCheckFunction,
             static_cast<size_t>(CollisionPair::NUM_PAIRS)>
      collision_check_function_map_ = {
          {&CollisionWorld::ComputePlaneSphereCollisionResult,
           &CollisionWorld::ComputePlaneCapsuleCollisionResult,
           &CollisionWorld::ComputeSphereSphereCollisionResult,
           &CollisionWorld::ComputeSphereCapsuleCollisionResult,
           &CollisionWorld::ComputeCapsuleCapsuleCollisionResult}};

  // Boolean flag to indicate whether collision world is created and valid
  bool is_valid_ = false;
  // Boolean flag to indicate whether any collision occurs between valid
  // object-object pairs and object-plane pairs in the world
  bool collision_occurs_ = false;
  // Minimum collision distance among all collision objects
  double min_collision_distance_ = std::numeric_limits<double>::infinity();
  // Total number of collision geometries in the world
  size_t num_collision_geometries_ = 0;

  // Vector of collision objects
  std::vector<CollisionObject> collision_objects_;
  // Vector of collision data for objects
  std::vector<CollisionResult> object_collision_result_;

  // Vector of vectors containing the world indices of different geometries for
  // each object
  std::vector<std::vector<size_t>> world_plane_indices_per_object_map_;
  std::vector<std::vector<size_t>> world_sphere_indices_per_object_map_;
  std::vector<std::vector<size_t>> world_capsule_indices_per_object_map_;

  // Vector of the origins of the plane geometries in world frame
  std::vector<TranslationVector> world_plane_origins_;
  // Vector of the unit normal vectors of the plane geometries in world frame
  std::vector<TranslationVector> world_plane_unit_normal_vectors_;

  // Vector of the centers of the spherical geometries in world frame
  std::vector<TranslationVector> world_sphere_centers_;
  // Vector of the radii of the spherical geometries
  std::vector<double> world_sphere_radii_;

  // Vector of the endpoints of the capsule geometries in world frame
  std::vector<TranslationVector> world_capsule_endpoints_1_;
  std::vector<TranslationVector> world_capsule_endpoints_2_;
  // Vector of unit axial vectors of the capsule geometries in world frame
  std::vector<TranslationVector> world_capsule_unit_axial_vectors_;
  // Vector of the radii of the capsule geometries
  std::vector<double> world_capsule_radii_;
  // Vector of lengths of the capsule geometries
  std::vector<double> world_capsule_lengths_;

  // Vector of valid geometry-geometry collision pair info
  std::vector<CollisionGeometryPairInfo> valid_collision_geometry_pairs_;
};

}  // namespace collision

}  // namespace intrinsic

#endif  // INTRINSIC_ICON_CONTROL_COLLISION_COLLISION_WORLD_H_
