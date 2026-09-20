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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DATA_STRUCTURES_NEAREST_NEIGHBOR_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DATA_STRUCTURES_NEAREST_NEIGHBOR_H_

#include <functional>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/motion_planning/path_planning/data_structures/nearest_neighbor_configs.pb.h"

namespace intrinsic {
// Interface for nearest neighbor containers.
//
// The interface is set up to support different types of nearest neighbor
// strategies:
// * Returning the nearest neighbor
// * K-Nearest Neighbor
// * Radial search
// The type of nearest neighbor strategy needs to be defined during construction
// and cannot be changed during run time. If not defined, K-Nearest Neighbor
// is the default strategy. However, the interface is set up such that
// parameters like k or radius can be changed during run time to support
// path planners that use a star-algorithm.
//
// The nearest neighbors can be accessed through a single method:
// * absl::StatusOr<std::vector<T>> Nearest(const T& data).
// The method will return zero to n nearest neighbors depending on the
// nearest neighbor strategy set during construction.
//
// The distance function does not need to be set during construction and need
// to be set by the user during usage. While some nearest neighbor data
// structures allow adding elements without defining a cost function, some
// do not. An error status will be returned if the chosen action can  not
// be executed without setting the distance function.
template <typename T>
class NearestNeighbors {
 public:
  // A metric defining the distance between two instances of type T. Required
  // to define the nearest neighbors computation.
  using DistanceFunction = std::function<double(const T&, const T&)>;

  virtual ~NearestNeighbors() = default;

  // Sets a distance metric to evaluate the property 'near'.
  virtual absl::Status SetDistanceFunction(
      const DistanceFunction& distance_function) {
    distance_function_ = distance_function;
    return absl::OkStatus();
  }

  // Clears the internal data structure used to find the nearest neighbors.
  virtual absl::Status ClearDataStructure() = 0;

  // Adds a new element into the internal data structure.
  virtual absl::Status AddElement(const T& data) = 0;

  // Adds a vector of elements into the internal data structure.
  virtual absl::Status AddElements(const std::vector<T>& data) = 0;

  // Removes an element from the nearest neighbor data structure.
  virtual absl::Status RemoveElement(const T& data) = 0;

  // Returns the nearest neighbors of the data point defined by data. Uses
  // the definition of nearest as defined by the
  // NearestNeighbor::ConnectionStrategy.
  virtual absl::StatusOr<std::vector<T>> GetNearestNeighbors(
      const T& data) const = 0;

  // Returns the nearest neighbor of the data.
  virtual absl::StatusOr<T> GetNearestNeighbor(const T& data) const = 0;

  // Returns the k-nearest neighbors of data.
  virtual absl::StatusOr<std::vector<T>> GetKNearestNeighbors(const T& data,
                                                              int k) const = 0;

  // Returns the k-nearest neighbor of data, where k is the pre-defined value.
  virtual absl::StatusOr<std::vector<T>> GetKNearestNeighbors(
      const T& data) const = 0;

  // Returns the nearest neighbors of data within a radius of r. If r is not
  // defined by the user, it will use the pre-defined r from the configuration
  // file.
  virtual absl::StatusOr<std::vector<T>> GetNearestNeighborsWithinRadius(
      const T& data, double r) const = 0;

  // Returns the nearest neighbors of data within a radius of r, where r is
  // defined by the configuration file.
  virtual absl::StatusOr<std::vector<T>> GetNearestNeighborsWithinRadius(
      const T& data) const = 0;

  // Sets the radius in which we search for nearest neighbors.
  virtual void SetRadius(double radius) { radius_ = radius; }

  // Gets k (the maximum number of neighbors).
  virtual int GetK() { return k_; }

  // Gets the radius for radial nearest neighbor search.
  virtual double GetRadius() { return radius_; }

  // Gets the nearest neighbor strategy that is set as default;
  virtual proto::NearestNeighborConfig::NearestNeighborStrategy GetStrategy()
      const {
    return base_config_.strategy();
  }

  // Gets the number of elements currently stored in the internal data
  // structure.
  virtual absl::StatusOr<int> Size() const = 0;

 protected:
  // Distance function that defines what near means.
  DistanceFunction distance_function_;

  // The number of nearest neighbors returned for k-neares neighbor search
  int k_;

  // The radius for radia nearest neighbor search.
  double radius_;

  // Configuration file for the nearest neighbor.
  proto::NearestNeighborConfig base_config_;
};
}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DATA_STRUCTURES_NEAREST_NEIGHBOR_H_
