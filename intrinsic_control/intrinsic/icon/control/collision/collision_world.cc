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

#include "intrinsic/icon/control/collision/collision_world.h"

#include <cstddef>
#include <ios>
#include <limits>
#include <ostream>
#include <vector>

#include "Eigen/Core"
#include "absl/log/check.h"
#include "intrinsic/icon/control/collision/collision_data_types.h"
#include "intrinsic/icon/control/collision/collision_error.h"
#include "intrinsic/icon/control/collision/collision_object.h"
#include "intrinsic/icon/utils/realtime_guard.h"

namespace intrinsic {
namespace collision {

bool CollisionWorld::CheckCollision(
    std::vector<CollisionResult>* collision_result) {
  // Reset collision data
  ResetObjectCollisionResult();

  // Geometry-Geometry pairs
  for (const auto& collision_geom_pair : valid_collision_geometry_pairs_) {
    // pick an action based on collision pair
    size_t obj_ind1 = collision_geom_pair.object_index1;
    size_t obj_ind2 = collision_geom_pair.object_index2;
    size_t geom_ind1 = collision_geom_pair.geometry_index1;
    size_t geom_ind2 = collision_geom_pair.geometry_index2;
    CollisionPair collision_pair = collision_geom_pair.collision_pair;
    // compute collision distance and contact normals
    double collision_distance = 0.0;
    TranslationVector world_unit_contact_normal;
    TranslationVector world_contact_point1, world_contact_point2;
    // call the appropriate collision checking function
    (this->*(collision_check_function_map_[static_cast<size_t>(
                collision_pair)]))(
        geom_ind1, geom_ind2, &collision_distance, &world_unit_contact_normal,
        &world_contact_point1, &world_contact_point2);
    // Update collision data for the objects involved
    UpdateCollisionResultIfNecessary(obj_ind1, collision_distance,
                                     world_unit_contact_normal,
                                     world_contact_point1);
    UpdateCollisionResultIfNecessary(obj_ind2, collision_distance,
                                     -world_unit_contact_normal,
                                     world_contact_point2);
    // check for collision
    // any contact is considered collision here
    // therefore, only positive distances are considered collision free
    if (collision_distance <= 0.0) {
      // collision occurs
      object_collision_result_[obj_ind1].collision_occurs = true;
      object_collision_result_[obj_ind2].collision_occurs = true;
      // Update world collision occurs flag
      collision_occurs_ = true;
    }
  }

  // Update contact data for all objects
  // This call transforms the contact point and unit contact normal vectors
  // for each object from world frame to the object frame
  TransformContactDataToObjectFrame();

  // Copy the vector of collision data
  if (collision_result != nullptr) {
    *collision_result = object_collision_result_;
  }

  return collision_occurs_;
}

CollisionError CollisionWorld::CreateCollisionWorld(
    const std::vector<CollisionObject>& collision_objects) {
  INTRINSIC_ASSERT_NON_REALTIME();
  // reset collision geometries to begin
  ResetCollisionGeometries();
  // validity check for collision objects
  if (collision_objects.empty()) {
    return CollisionError::INVALID_NUM_COLLISION_OBJECTS;
  }
  // find the number of collision geometries
  size_t num_plane_geoms = 0;
  size_t num_sphere_geoms = 0;
  size_t num_capsule_geoms = 0;
  for (size_t obj_index = 0; obj_index < collision_objects.size();
       obj_index++) {
    std::vector<size_t> plane_indices;
    std::vector<size_t> sphere_indices;
    std::vector<size_t> capsule_indices;
    plane_indices.reserve(
        collision_objects[obj_index].GetNumberOfPlaneGeometries());
    for (size_t plane_index = 0;
         plane_index <
         collision_objects[obj_index].GetNumberOfPlaneGeometries();
         plane_index++) {
      plane_indices.push_back(num_plane_geoms++);
    }
    for (size_t sphere_index = 0;
         sphere_index <
         collision_objects[obj_index].GetNumberOfSphereGeometries();
         sphere_index++) {
      sphere_indices.push_back(num_sphere_geoms++);
      // add radii info as well
      world_sphere_radii_.push_back(collision_objects[obj_index]
                                        .GetSphereGeometry(sphere_index)
                                        .value()
                                        .radius);
    }
    for (size_t capsule_index = 0;
         capsule_index <
         collision_objects[obj_index].GetNumberOfCapsuleGeometries();
         capsule_index++) {
      capsule_indices.push_back(num_capsule_geoms++);
      // add radii and length info as well
      world_capsule_radii_.push_back(collision_objects[obj_index]
                                         .GetCapsuleGeometry(capsule_index)
                                         .value()
                                         .radius);
      world_capsule_lengths_.push_back((collision_objects[obj_index]
                                            .GetCapsuleGeometry(capsule_index)
                                            .value()
                                            .endpoint2 -
                                        collision_objects[obj_index]
                                            .GetCapsuleGeometry(capsule_index)
                                            .value()
                                            .endpoint1)
                                           .norm());
    }
    world_plane_indices_per_object_map_.push_back(plane_indices);
    world_sphere_indices_per_object_map_.push_back(sphere_indices);
    world_capsule_indices_per_object_map_.push_back(capsule_indices);
  }
  num_collision_geometries_ =
      num_plane_geoms + num_sphere_geoms + num_capsule_geoms;
  // set collision objects and collision planes
  collision_objects_ = collision_objects;
  // reserve and/or allocate memory
  world_plane_origins_.resize(num_plane_geoms);
  world_plane_unit_normal_vectors_.resize(num_plane_geoms);
  world_sphere_centers_.resize(num_sphere_geoms);
  world_capsule_endpoints_1_.resize(num_capsule_geoms);
  world_capsule_endpoints_2_.resize(num_capsule_geoms);
  world_capsule_unit_axial_vectors_.resize(num_capsule_geoms);
  valid_collision_geometry_pairs_.reserve(num_collision_geometries_ *
                                          num_collision_geometries_);
  object_collision_result_.resize(GetNumberOfCollisionObjects());
  // update the translations
  UpdateWorldGeometryTranslations();
  // Reset collision data
  ResetObjectCollisionResult();
  // set validity flag to true
  is_valid_ = true;
  // return no error
  return CollisionError::NO_ERROR;
}

std::ostream& CollisionWorld::operator<<(std::ostream& out) {
  out << "=====================" << std::endl;
  out << "Collision Data" << std::endl;
  out << "=====================" << std::endl;
  for (size_t obj_ind = 0; obj_ind < GetNumberOfCollisionObjects(); obj_ind++) {
    // print object index
    out << "--------------" << std::endl;
    out << "Object" << obj_ind << std::endl;
    out << "--------------" << std::endl;
    // print collision data
    out << "Collision? " << std::boolalpha
        << object_collision_result_[obj_ind].collision_occurs << std::endl;
    out << "Min Dist : " << object_collision_result_[obj_ind].min_distance
        << std::endl;
    out << "ClosePt (World Frame) : "
        << "[" << object_collision_result_[obj_ind].world_contact_point[0]
        << "," << object_collision_result_[obj_ind].world_contact_point[1]
        << "," << object_collision_result_[obj_ind].world_contact_point[2]
        << "]" << std::endl;
    out << "ClosePt (Obj Frame)   : "
        << "[" << object_collision_result_[obj_ind].contact_point[0] << ","
        << object_collision_result_[obj_ind].contact_point[1] << ","
        << object_collision_result_[obj_ind].contact_point[2] << "]"
        << std::endl;
    out << "Esc Dir (World Frame) : "
        << "["
        << object_collision_result_[obj_ind].world_unit_contact_normal_vector[0]
        << ","
        << object_collision_result_[obj_ind].world_unit_contact_normal_vector[1]
        << ","
        << object_collision_result_[obj_ind].world_unit_contact_normal_vector[2]
        << "]" << std::endl;
    out << "Esc Dir (Obj Frame)   : "
        << "["
        << object_collision_result_[obj_ind].unit_contact_normal_vector[0]
        << ","
        << object_collision_result_[obj_ind].unit_contact_normal_vector[1]
        << ","
        << object_collision_result_[obj_ind].unit_contact_normal_vector[2]
        << "]" << std::endl;
  }
  out << "=====================" << std::endl;
  out << "Collision? " << std::boolalpha << collision_occurs_ << std::endl;
  out << "=====================" << std::endl;
  return out;
}

void CollisionWorld::EnableAllObjectObjectCollisionChecks() {
  // Reset collision pairs
  valid_collision_geometry_pairs_.clear();
  for (size_t obj_ind1 = 0; obj_ind1 < GetNumberOfCollisionObjects();
       obj_ind1++) {
    // avoid adding a pair of geometries of the same object
    for (size_t obj_ind2 = obj_ind1 + 1;
         obj_ind2 < GetNumberOfCollisionObjects(); obj_ind2++) {
      EnableObjectObjectCollisionCheck(obj_ind1, obj_ind2);
    }
  }
}

CollisionError CollisionWorld::EnableObjectObjectCollisionCheck(
    const size_t object_index1, const size_t object_index2) {
  if ((object_index1 < GetNumberOfCollisionObjects()) &&
      (object_index2 < GetNumberOfCollisionObjects())) {
    // plane-x pairs
    for (auto plane_ind1 : world_plane_indices_per_object_map_[object_index1]) {
      // plane-sphere pairs
      for (auto sphere_ind2 :
           world_sphere_indices_per_object_map_[object_index2]) {
        // add pair
        valid_collision_geometry_pairs_.push_back(CollisionGeometryPairInfo(
            object_index1, object_index2, plane_ind1, sphere_ind2,
            CollisionPair::PLANE_SPHERE));
      }
      // plane-capsule pairs
      for (auto capsule_ind2 :
           world_capsule_indices_per_object_map_[object_index2]) {
        // add pair
        valid_collision_geometry_pairs_.push_back(CollisionGeometryPairInfo(
            object_index1, object_index2, plane_ind1, capsule_ind2,
            CollisionPair::PLANE_CAPSULE));
      }
    }
    // sphere-x pairs
    for (auto sphere_ind1 :
         world_sphere_indices_per_object_map_[object_index1]) {
      // sphere-plane pairs
      for (auto plane_ind2 :
           world_plane_indices_per_object_map_[object_index2]) {
        // add pair
        valid_collision_geometry_pairs_.push_back(CollisionGeometryPairInfo(
            object_index2, object_index1, plane_ind2, sphere_ind1,
            CollisionPair::PLANE_SPHERE));
      }
      // sphere-sphere pairs
      for (auto sphere_ind2 :
           world_sphere_indices_per_object_map_[object_index2]) {
        // add pair
        valid_collision_geometry_pairs_.push_back(CollisionGeometryPairInfo(
            object_index1, object_index2, sphere_ind1, sphere_ind2,
            CollisionPair::SPHERE_SPHERE));
      }
      // sphere-capsule pairs
      for (auto capsule_ind2 :
           world_capsule_indices_per_object_map_[object_index2]) {
        // add pair
        valid_collision_geometry_pairs_.push_back(CollisionGeometryPairInfo(
            object_index1, object_index2, sphere_ind1, capsule_ind2,
            CollisionPair::SPHERE_CAPSULE));
      }
    }
    // capsule-capsule pairs
    for (auto capsule_ind1 :
         world_capsule_indices_per_object_map_[object_index1]) {
      // capsule-plane pairs
      for (auto plane_ind2 :
           world_plane_indices_per_object_map_[object_index2]) {
        // add pair
        valid_collision_geometry_pairs_.push_back(CollisionGeometryPairInfo(
            object_index2, object_index1, plane_ind2, capsule_ind1,
            CollisionPair::PLANE_CAPSULE));
      }
      // capsule-sphere pairs
      for (auto sphere_ind2 :
           world_sphere_indices_per_object_map_[object_index2]) {
        // add pair
        valid_collision_geometry_pairs_.push_back(CollisionGeometryPairInfo(
            object_index2, object_index1, sphere_ind2, capsule_ind1,
            CollisionPair::SPHERE_CAPSULE));
      }
      // capsule-capsule pairs
      for (auto capsule_ind2 :
           world_capsule_indices_per_object_map_[object_index2]) {
        // add pair
        valid_collision_geometry_pairs_.push_back(CollisionGeometryPairInfo(
            object_index1, object_index2, capsule_ind1, capsule_ind2,
            CollisionPair::CAPSULE_CAPSULE));
      }
    }
    return CollisionError::NO_ERROR;
  } else {
    return CollisionError::INVALID_COLLISION_OBJECT_INDEX;
  }
}

CollisionError CollisionWorld::SetWorldObjectTransformation(
    const size_t collision_object_index, const RotationMatrix& rotation_matrix,
    const TranslationVector& translation_vector) {
  if (collision_object_index < GetNumberOfCollisionObjects()) {
    collision_objects_[collision_object_index].SetWorldObjectTransformation(
        rotation_matrix, translation_vector);
    // Update the world geometry translations for all geometries in this object
    UpdateWorldGeometryTranslations(collision_object_index);
    return CollisionError::NO_ERROR;
  } else {
    return CollisionError::INVALID_COLLISION_OBJECT_INDEX;
  }
}

void CollisionWorld::ComputeSphereSphereCollisionResult(
    const size_t sphere_index1, const size_t sphere_index2,
    double* collision_distance, TranslationVector* world_unit_contact_normal,
    TranslationVector* world_contact_point1,
    TranslationVector* world_contact_point2) {
  CHECK((collision_distance != nullptr) &&
        (world_unit_contact_normal != nullptr) &&
        (world_contact_point1 != nullptr) && (world_contact_point2 != nullptr))
      << "one or more output pointers are null";
  // compute distance between centers of the two spherical geometries
  // get vector between centers in world frame
  TranslationVector vector_between_centers =
      world_sphere_centers_[sphere_index1] -
      world_sphere_centers_[sphere_index2];
  double center_distance = vector_between_centers.norm();
  // compute unit contact normal for the first sphere
  if (center_distance > std::numeric_limits<double>::min()) {
    *world_unit_contact_normal = vector_between_centers / center_distance;
  } else {
    // center of the spheres coincide, hence any direction is a valid escape
    // direction choosing +Z in world frame as the escape direction here
    *world_unit_contact_normal = TranslationVector(0.0, 0.0, 1.0);
  }
  // distance before collision occurs
  *collision_distance = center_distance - world_sphere_radii_[sphere_index1] -
                        world_sphere_radii_[sphere_index2];
  // compute contact points on the two spheres
  *world_contact_point1 =
      world_sphere_centers_[sphere_index1] -
      world_sphere_radii_[sphere_index1] * (*world_unit_contact_normal);
  *world_contact_point2 =
      world_sphere_centers_[sphere_index2] +
      world_sphere_radii_[sphere_index2] * (*world_unit_contact_normal);
}

void CollisionWorld::ComputeSphereCapsuleCollisionResult(
    const size_t sphere_index, const size_t capsule_index,
    double* collision_distance, TranslationVector* world_unit_contact_normal,
    TranslationVector* world_contact_point1,
    TranslationVector* world_contact_point2) {
  CHECK((collision_distance != nullptr) &&
        (world_unit_contact_normal != nullptr) &&
        (world_contact_point1 != nullptr) && (world_contact_point2 != nullptr))
      << "one or more output pointers are null";
  // compute contact info
  double contact_distance;
  TranslationVector world_axial_contact_point;
  ComputeContactInfoPointLine(
      capsule_index, world_sphere_centers_[sphere_index], &contact_distance,
      world_unit_contact_normal, &world_axial_contact_point);
  // distance before collision occurs
  *collision_distance = contact_distance - world_sphere_radii_[sphere_index] -
                        world_capsule_radii_[capsule_index];
  // compute contact points on sphere and capsule
  *world_contact_point1 =
      world_sphere_centers_[sphere_index] -
      world_sphere_radii_[sphere_index] * (*world_unit_contact_normal);
  *world_contact_point2 =
      world_axial_contact_point +
      world_capsule_radii_[capsule_index] * (*world_unit_contact_normal);
}

void CollisionWorld::ComputeCapsuleCapsuleCollisionResult(
    const size_t capsule_index1, const size_t capsule_index2,
    double* collision_distance, TranslationVector* world_unit_contact_normal,
    TranslationVector* world_contact_point1,
    TranslationVector* world_contact_point2) {
  CHECK((collision_distance != nullptr) &&
        (world_unit_contact_normal != nullptr) &&
        (world_contact_point1 != nullptr) && (world_contact_point2 != nullptr))
      << "one or more output pointers are null";
  // compute contact info
  double min_contact_distance = std::numeric_limits<double>::infinity();
  TranslationVector min_world_unit_contact_normal, world_axial_contact_point1,
      world_axial_contact_point2;

  // line-line collision check
  if (!ComputeContactInfoLineLine(
          capsule_index1, capsule_index2, &min_contact_distance,
          &min_world_unit_contact_normal, &world_axial_contact_point1,
          &world_axial_contact_point2)) {
    // line-line collision check returned false indicating that no valid
    // intersection point within limits was found
    // therefore the closest point is one of the endpoints
    // point-line collision checks
    double temp_contact_distance = std::numeric_limits<double>::infinity();
    TranslationVector temp_world_unit_contact_normal,
        temp_world_axial_contact_point;
    // compute contact info for all combinations and find min
    size_t capsule_indices[4] = {capsule_index2, capsule_index2, capsule_index1,
                                 capsule_index1};
    TranslationVector points[4] = {world_capsule_endpoints_1_[capsule_index1],
                                   world_capsule_endpoints_2_[capsule_index1],
                                   world_capsule_endpoints_1_[capsule_index2],
                                   world_capsule_endpoints_2_[capsule_index2]};
    for (size_t i = 0; i < 4; i++) {
      ComputeContactInfoPointLine(
          capsule_indices[i], points[i], &temp_contact_distance,
          &temp_world_unit_contact_normal, &temp_world_axial_contact_point);
      if (temp_contact_distance < min_contact_distance) {
        min_contact_distance = temp_contact_distance;
        if (i > 1) {
          // reverse contact normal since the reported contact normal
          // is for the second capsule
          min_world_unit_contact_normal = -temp_world_unit_contact_normal;
          world_axial_contact_point1 = temp_world_axial_contact_point;
          world_axial_contact_point2 = points[i];
        } else {
          min_world_unit_contact_normal = temp_world_unit_contact_normal;
          world_axial_contact_point1 = points[i];

          world_axial_contact_point2 = temp_world_axial_contact_point;
        }
      }
    }
  }

  // distance before collision occurs
  *collision_distance = min_contact_distance -
                        world_capsule_radii_[capsule_index1] -
                        world_capsule_radii_[capsule_index2];
  // compute unit contact normal for the first capsule
  *world_unit_contact_normal = min_world_unit_contact_normal;
  // compute contact points on the two capsules
  *world_contact_point1 =
      world_axial_contact_point1 -
      world_capsule_radii_[capsule_index1] * min_world_unit_contact_normal;
  *world_contact_point2 =
      world_axial_contact_point2 +
      world_capsule_radii_[capsule_index2] * min_world_unit_contact_normal;
}

void CollisionWorld::ComputePlaneSphereCollisionResult(
    const size_t plane_index, const size_t sphere_index,
    double* collision_distance, TranslationVector* world_unit_contact_normal,
    TranslationVector* world_contact_point1,
    TranslationVector* world_contact_point2) {
  CHECK((collision_distance != nullptr) &&
        (world_unit_contact_normal != nullptr) &&
        (world_contact_point1 != nullptr) && (world_contact_point2 != nullptr))
      << "one or more output pointers are null";
  // Compute the vector from origin of plane to center of sphere in world frame
  TranslationVector vector_from_plane =
      world_sphere_centers_[sphere_index] - world_plane_origins_[plane_index];
  // Compute dot product of the above vector and plane's normal vector
  double distance_from_plane =
      vector_from_plane.dot(world_plane_unit_normal_vectors_[plane_index]);
  // distance before collision occurs
  *collision_distance = distance_from_plane - world_sphere_radii_[sphere_index];
  // Compute unit contact normal for the plane
  *world_unit_contact_normal = -world_plane_unit_normal_vectors_[plane_index];
  // Compute contact points on the plane and the sphere
  *world_contact_point1 =
      world_sphere_centers_[sphere_index] -
      distance_from_plane * world_plane_unit_normal_vectors_[plane_index];
  *world_contact_point2 = world_sphere_centers_[sphere_index] -
                          world_sphere_radii_[sphere_index] *
                              world_plane_unit_normal_vectors_[plane_index];
}

void CollisionWorld::ComputePlaneCapsuleCollisionResult(
    const size_t plane_index, const size_t capsule_index,
    double* collision_distance, TranslationVector* world_unit_contact_normal,
    TranslationVector* world_contact_point1,
    TranslationVector* world_contact_point2) {
  CHECK((collision_distance != nullptr) &&
        (world_unit_contact_normal != nullptr) &&
        (world_contact_point1 != nullptr) && (world_contact_point2 != nullptr))
      << "one or more output pointers are null";
  // Compute the vector from origin of plane to the capsule endpoints in world
  // frame
  TranslationVector vector_endpoint1_from_plane =
      world_capsule_endpoints_1_[capsule_index] -
      world_plane_origins_[plane_index];
  TranslationVector vector_endpoint2_from_plane =
      world_capsule_endpoints_2_[capsule_index] -
      world_plane_origins_[plane_index];
  // Compute dot product of the above vectors and plane's normal vector
  double distance_endpoint1_from_plane = vector_endpoint1_from_plane.dot(
      world_plane_unit_normal_vectors_[plane_index]);
  double distance_endpoint2_from_plane = vector_endpoint2_from_plane.dot(
      world_plane_unit_normal_vectors_[plane_index]);
  // line-plane collision check
  TranslationVector world_axial_contact_point;
  double min_distance_from_plane;
  if (distance_endpoint1_from_plane < distance_endpoint2_from_plane) {
    min_distance_from_plane = distance_endpoint1_from_plane;
    world_axial_contact_point = world_capsule_endpoints_1_[capsule_index];
  } else {
    min_distance_from_plane = distance_endpoint2_from_plane;
    world_axial_contact_point = world_capsule_endpoints_2_[capsule_index];
  }
  // distance before collision occurs
  *collision_distance =
      min_distance_from_plane - world_capsule_radii_[capsule_index];
  // Compute unit contact normal for the plane
  *world_unit_contact_normal = -world_plane_unit_normal_vectors_[plane_index];
  // Compute contact points on the plane and the capsule
  *world_contact_point1 =
      world_axial_contact_point -
      min_distance_from_plane * world_plane_unit_normal_vectors_[plane_index];
  *world_contact_point2 = world_axial_contact_point -
                          world_capsule_radii_[capsule_index] *
                              world_plane_unit_normal_vectors_[plane_index];
}

void CollisionWorld::ComputeContactInfoPointLine(
    const size_t capsule_index, const TranslationVector& point,
    double* contact_distance, TranslationVector* world_unit_contact_normal,
    TranslationVector* world_axial_contact_point) {
  CHECK((contact_distance != nullptr) &&
        (world_unit_contact_normal != nullptr) &&
        (world_axial_contact_point != nullptr))
      << "one or more output pointers are null";
  // Compute vector of capsule endpoint1 from the input point in world frame
  TranslationVector vector_from_endpoint1 =
      point - world_capsule_endpoints_1_[capsule_index];
  // Compute dot product with the unit axial vector
  double dot_product_capsule_contact_from_endpoint1 = vector_from_endpoint1.dot(
      world_capsule_unit_axial_vectors_[capsule_index]);
  // contact normal in world frame
  TranslationVector contact_normal, axial_contact_point, unit_contact_normal;
  // check if this potential contact point is within the capsule
  if ((dot_product_capsule_contact_from_endpoint1 >= 0) &&
      (dot_product_capsule_contact_from_endpoint1 <=
       world_capsule_lengths_[capsule_index])) {
    // Compute contact point on the capsule axis
    axial_contact_point = world_capsule_endpoints_1_[capsule_index] +
                          dot_product_capsule_contact_from_endpoint1 *
                              world_capsule_unit_axial_vectors_[capsule_index];
  } else {
    // one of the end points is the closest
    if (dot_product_capsule_contact_from_endpoint1 < 0.0) {
      // end point 1 is the closest
      axial_contact_point = world_capsule_endpoints_1_[capsule_index];
    } else {
      // end point 2 is the closest
      axial_contact_point = world_capsule_endpoints_2_[capsule_index];
    }
  }
  // Compute contact normal
  contact_normal = point - axial_contact_point;
  double min_distance = contact_normal.norm();
  if (min_distance < kZeroThreshold) {
    // this indicates that the sphere center is on the capsule axis
    // so contact normal can be any direction normal to the capsule axis
    // picking one here
    ComputeValidCapsuleContactNormal(capsule_index, &contact_normal);
  }
  unit_contact_normal = contact_normal / contact_normal.norm();
  // copy data
  *contact_distance = min_distance;
  *world_axial_contact_point = axial_contact_point;
  *world_unit_contact_normal = unit_contact_normal;
}

bool CollisionWorld::ComputeContactInfoLineLine(
    const size_t capsule_index1, const size_t capsule_index2,
    double* contact_distance, TranslationVector* world_unit_contact_normal,
    TranslationVector* world_axial_contact_point1,
    TranslationVector* world_axial_contact_point2) {
  CHECK((contact_distance != nullptr) &&
        (world_unit_contact_normal != nullptr) &&
        (world_axial_contact_point1 != nullptr) &&
        (world_axial_contact_point2 != nullptr))
      << "one or more output pointers are null";
  // compute vector between first endpoints in world frame
  // p2 - p1
  TranslationVector vector_between_endpoints1 =
      world_capsule_endpoints_1_[capsule_index2] -
      world_capsule_endpoints_1_[capsule_index1];
  // compute cross-product of two unit axial vectors
  // v1 x v2
  TranslationVector unit_cross_product_axial_vectors =
      world_capsule_unit_axial_vectors_[capsule_index1].cross(
          world_capsule_unit_axial_vectors_[capsule_index2]);
  // axial contact points
  TranslationVector axial_contact_point1, axial_contact_point2;
  // check if the axial vectors are parallel
  if (unit_cross_product_axial_vectors.norm() < kZeroThreshold) {
    // parallel lines
    // check if they can be projected within limits
    double dot_product1 = vector_between_endpoints1.dot(
        world_capsule_unit_axial_vectors_[capsule_index1]);
    double dot_product2 =
        (-vector_between_endpoints1)
            .dot(world_capsule_unit_axial_vectors_[capsule_index2]);
    // capsule 1
    if ((dot_product1 >= 0.0) &&
        (dot_product1 <= world_capsule_lengths_[capsule_index1])) {
      axial_contact_point1 =
          world_capsule_endpoints_1_[capsule_index1] +
          dot_product1 * world_capsule_unit_axial_vectors_[capsule_index1];
      axial_contact_point2 = world_capsule_endpoints_1_[capsule_index2];
    } else {
      // capsule 2
      if ((dot_product2 >= 0.0) &&
          (dot_product2 <= world_capsule_lengths_[capsule_index2])) {
        axial_contact_point1 = world_capsule_endpoints_1_[capsule_index1];
        axial_contact_point2 =
            world_capsule_endpoints_1_[capsule_index2] +
            dot_product2 * world_capsule_unit_axial_vectors_[capsule_index2];
      } else {
        // not within limits so one of the other endpoints is the closest
        return false;
      }
    }
  } else {
    // non-parallel lines
    // either intersecting or skew lines
    unit_cross_product_axial_vectors /= unit_cross_product_axial_vectors.norm();
    // compute closest point of contact
    // cp1 = p1 + a v1
    // cp2 = p2 + b v2
    // cp2 - cp1 = c (v1 x v2)
    // find scalars a,b,c to solve above equations
    // [v1, -v2, v1 x v2] [a; b; c] = p2 - p1
    Eigen::Matrix3d matrix_unit_vectors;
    matrix_unit_vectors.block(0, 0, 3, 1) =
        world_capsule_unit_axial_vectors_[capsule_index1];
    matrix_unit_vectors.block(0, 1, 3, 1) =
        -world_capsule_unit_axial_vectors_[capsule_index2];
    matrix_unit_vectors.block(0, 2, 3, 1) = unit_cross_product_axial_vectors;
    Eigen::Vector3d scalars =
        matrix_unit_vectors.inverse() * vector_between_endpoints1;
    // check if the closest points are within limits
    // capsule 1
    if ((scalars(0) >= 0.0) &&
        (scalars(0) <= world_capsule_lengths_[capsule_index1])) {
      axial_contact_point1 =
          world_capsule_endpoints_1_[capsule_index1] +
          scalars(0) * world_capsule_unit_axial_vectors_[capsule_index1];
    } else {
      // not within limits so one of the endpoints is the closest
      return false;
    }
    // capsule 2
    if ((scalars(1) >= 0.0) &&
        (scalars(1) <= world_capsule_lengths_[capsule_index2])) {
      axial_contact_point2 =
          world_capsule_endpoints_1_[capsule_index2] +
          scalars(1) * world_capsule_unit_axial_vectors_[capsule_index2];
    } else {
      // not within limits so one of the endpoints is the closest
      return false;
    }
  }
  // contact normal for capsule1
  TranslationVector contact_normal1, unit_contact_normal1;
  // cp1 - cp2
  contact_normal1 = axial_contact_point1 - axial_contact_point2;
  // min distance
  double min_distance = contact_normal1.norm();
  if (min_distance < kZeroThreshold) {
    // intersecting lines
    if (unit_cross_product_axial_vectors.norm() < kZeroThreshold) {
      // this indicates that the capsule axes are in line
      // so contact normal can be any direction normal to the capsule axis
      // picking one here
      ComputeValidCapsuleContactNormal(capsule_index1, &contact_normal1);
    } else {
      contact_normal1 = unit_cross_product_axial_vectors;
    }
  }
  unit_contact_normal1 = contact_normal1 / contact_normal1.norm();

  // copy contact data
  // min distance
  *contact_distance = min_distance;
  // contact normal / direction of escape for capsule1
  *world_unit_contact_normal = unit_contact_normal1;
  // contact points
  *world_axial_contact_point1 = axial_contact_point1;
  *world_axial_contact_point2 = axial_contact_point2;
  // a valid intersection point within limits was found
  return true;
}

void CollisionWorld::ComputeValidCapsuleContactNormal(
    const size_t capsule_index, TranslationVector* contact_normal) {
  double a = world_capsule_unit_axial_vectors_[capsule_index][0];
  double b = world_capsule_unit_axial_vectors_[capsule_index][1];
  double c = world_capsule_unit_axial_vectors_[capsule_index][2];
  if ((c != 0.0) && ((a + b) != 0.0)) {
    *contact_normal = TranslationVector(-b - c, a, a);
  } else {
    *contact_normal = TranslationVector(c, c, -a - b);
  }
}

void CollisionWorld::UpdateWorldGeometryTranslations() {
  for (size_t obj_index = 0; obj_index < GetNumberOfCollisionObjects();
       obj_index++) {
    UpdateWorldGeometryTranslations(obj_index);
  }
}

void CollisionWorld::UpdateWorldGeometryTranslations(const size_t obj_index) {
  // Update plane geometries
  for (size_t obj_plane_index = 0;
       obj_plane_index <
       collision_objects_[obj_index].GetNumberOfPlaneGeometries();
       obj_plane_index++) {
    size_t world_plane_index =
        world_plane_indices_per_object_map_[obj_index][obj_plane_index];
    world_plane_origins_[world_plane_index] =
        collision_objects_[obj_index].GetWorldObjectTranslation() +
        collision_objects_[obj_index].GetWorldObjectRotation() *
            collision_objects_[obj_index]
                .GetPlaneGeometry(obj_plane_index)
                .value()
                .origin;
    world_plane_unit_normal_vectors_[world_plane_index] =
        collision_objects_[obj_index].GetWorldObjectRotation() *
        collision_objects_[obj_index]
            .GetPlaneGeometry(obj_plane_index)
            .value()
            .unit_normal_vector;
    // ensure that the normal vector is a unit vector
    world_plane_unit_normal_vectors_[world_plane_index] /=
        world_plane_unit_normal_vectors_[world_plane_index].norm();
  }
  // Update sphere geometries
  for (size_t obj_sphere_index = 0;
       obj_sphere_index <
       collision_objects_[obj_index].GetNumberOfSphereGeometries();
       obj_sphere_index++) {
    world_sphere_centers_
        [world_sphere_indices_per_object_map_[obj_index][obj_sphere_index]] =
            collision_objects_[obj_index].GetWorldObjectTranslation() +
            collision_objects_[obj_index].GetWorldObjectRotation() *
                collision_objects_[obj_index]
                    .GetSphereGeometry(obj_sphere_index)
                    .value()
                    .center;
  }
  // Update capsule geometries
  for (size_t obj_capsule_index = 0;
       obj_capsule_index <
       collision_objects_[obj_index].GetNumberOfCapsuleGeometries();
       obj_capsule_index++) {
    size_t world_capsule_index =
        world_capsule_indices_per_object_map_[obj_index][obj_capsule_index];
    world_capsule_endpoints_1_[world_capsule_index] =
        collision_objects_[obj_index].GetWorldObjectTranslation() +
        collision_objects_[obj_index].GetWorldObjectRotation() *
            collision_objects_[obj_index]
                .GetCapsuleGeometry(obj_capsule_index)
                .value()
                .endpoint1;
    world_capsule_endpoints_2_[world_capsule_index] =
        collision_objects_[obj_index].GetWorldObjectTranslation() +
        collision_objects_[obj_index].GetWorldObjectRotation() *
            collision_objects_[obj_index]
                .GetCapsuleGeometry(obj_capsule_index)
                .value()
                .endpoint2;
    world_capsule_unit_axial_vectors_[world_capsule_index] =
        (world_capsule_endpoints_2_[world_capsule_index] -
         world_capsule_endpoints_1_[world_capsule_index]) /
        world_capsule_lengths_[world_capsule_index];
  }
}

void CollisionWorld::TransformContactDataToObjectFrame() {
  for (size_t obj_index = 0; obj_index < GetNumberOfCollisionObjects();
       obj_index++) {
    // transform contact point
    object_collision_result_[obj_index].contact_point =
        collision_objects_[obj_index].GetWorldObjectRotation().transpose() *
        (object_collision_result_[obj_index].world_contact_point -
         collision_objects_[obj_index].GetWorldObjectTranslation());
    // transform unit contact normal vector
    object_collision_result_[obj_index].unit_contact_normal_vector =
        collision_objects_[obj_index].GetWorldObjectRotation().transpose() *
        object_collision_result_[obj_index].world_unit_contact_normal_vector;
  }
}

void CollisionWorld::UpdateCollisionResultIfNecessary(
    const size_t obj_index, const double collision_distance,
    const TranslationVector& world_unit_contact_normal,
    const TranslationVector& world_contact_point) {
  // Update collision data of object, if necessary
  if (collision_distance < object_collision_result_[obj_index].min_distance) {
    // Update min distance to collision
    object_collision_result_[obj_index].min_distance = collision_distance;
    // Update direction of escape for the object in world frame
    object_collision_result_[obj_index].world_unit_contact_normal_vector =
        world_unit_contact_normal;
    // Update (actual/potential) contact point of object in world frame
    object_collision_result_[obj_index].world_contact_point =
        world_contact_point;
    // Update the world-wide min collision distance, if needed
    if (collision_distance < min_collision_distance_) {
      min_collision_distance_ = collision_distance;
    }
  }
}

void CollisionWorld::ResetObjectCollisionResult() {
  for (size_t obj_ind = 0; obj_ind < GetNumberOfCollisionObjects(); obj_ind++) {
    object_collision_result_[obj_ind].collision_occurs = false;
    object_collision_result_[obj_ind].min_distance =
        std::numeric_limits<double>::infinity();
    object_collision_result_[obj_ind].world_unit_contact_normal_vector =
        TranslationVector(0.0, 0.0, 0.0);
    object_collision_result_[obj_ind].unit_contact_normal_vector =
        TranslationVector(0.0, 0.0, 0.0);
    object_collision_result_[obj_ind].world_contact_point =
        TranslationVector(0.0, 0.0, 0.0);
    object_collision_result_[obj_ind].contact_point =
        TranslationVector(0.0, 0.0, 0.0);
  }
  collision_occurs_ = false;
  min_collision_distance_ = std::numeric_limits<double>::infinity();
}

void CollisionWorld::ResetCollisionGeometries() {
  num_collision_geometries_ = 0;
  world_plane_origins_.clear();
  world_plane_unit_normal_vectors_.clear();
  world_sphere_centers_.clear();
  world_sphere_radii_.clear();
  world_capsule_endpoints_1_.clear();
  world_capsule_endpoints_2_.clear();
  world_capsule_unit_axial_vectors_.clear();
  world_capsule_radii_.clear();
  world_capsule_lengths_.clear();
  valid_collision_geometry_pairs_.clear();
}

}  // namespace collision
}  // namespace intrinsic
