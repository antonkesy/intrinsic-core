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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DATA_STRUCTURES_LINEAR_NEAREST_NEIGHBOR_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DATA_STRUCTURES_LINEAR_NEAREST_NEIGHBOR_H_

#include <vector>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/motion_planning/path_planning/data_structures/nearest_neighbor.h"
#include "intrinsic/util/proto/parse_text_proto.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
// The class provides a nearest neighbor data structure and linear search.
// The class parameter are defined in nearest_neighbor_configs.proto.
// If non are provided the default configs will be used (i.e., k-nearest
// neighbor with k=15.)
template <typename T>
class LinearNearestNeighbor : public NearestNeighbors<T> {
 public:
  // The linear nearest neighbor class is initiated with a config file
  // as defined in nearest_neighbor.proto.
  explicit LinearNearestNeighbor(const proto::NearestNeighborConfig& config)
      : NearestNeighbors<T>() {
    // Set k_ and radius_ to default and replace if a different value is
    // defined in the config.
    NearestNeighbors<T>::base_config_ = *default_nn_config_;
    NearestNeighbors<T>::base_config_.MergeFrom(config);

    // Radius and k are stored in additional members to allow run time changes
    // to implement star planning algorithms.
    NearestNeighbors<T>::k_ = NearestNeighbors<T>::base_config_.k();
    NearestNeighbors<T>::radius_ = NearestNeighbors<T>::base_config_.radius();
  }

  absl::Status ClearDataStructure() override {
    data_.clear();
    return absl::OkStatus();
  }

  // Adds a new element into the internal data structure.
  absl::Status AddElement(const T& data) override {
    data_.push_back(data);
    return absl::OkStatus();
  }

  // Adds a vector of elements into the internal data structure.
  absl::Status AddElements(const std::vector<T>& data) override {
    data_.insert(data_.end(), data.begin(), data.end());
    return absl::OkStatus();
  }

  // Removes an element from the internal data structure if it exists.
  absl::Status RemoveElement(const T& data) override {
    auto itr = std::find(data_.begin(), data_.end(), data);
    if (itr != data_.end()) {
      data_.erase(itr);
    }

    return absl::OkStatus();
  }

  // Returns the nearest neighbor. Uses the strategy in base_config_.
  absl::StatusOr<std::vector<T>> GetNearestNeighbors(
      const T& data) const override {
    // Check if the distance function is set.
    if (NearestNeighbors<T>::distance_function_ == nullptr) {
      return absl::FailedPreconditionError(
          "No distance function set. Setting the distance function prior to "
          " calling Nearest() is required.");
    }

    // Trivial case
    std::vector<T> result;
    INTR_ASSIGN_OR_RETURN(const int number_elements, Size());
    if (number_elements < 1) {
      return result;
    }

    // Otherwise call nearest neighbor function according to set strategy.
    switch (NearestNeighbors<T>::base_config_.strategy()) {
      case proto::NearestNeighborConfig::NEAREST: {
        auto nn = nearest(data);
        if (nn.ok()) {
          result.push_back(*nn);
        }
        break;
      }
      case proto::NearestNeighborConfig::KNEAREST:
      case proto::NearestNeighborConfig::HNS:
        nearestK(data, NearestNeighbors<T>::k_, result);
        break;
      case proto::NearestNeighborConfig::RADIUS:
        nearestR(data, NearestNeighbors<T>::radius_, result);
        break;
      default:
        LOG(FATAL) << "Unknown nearest neighbor option";
    }

    return result;
  }

  absl::StatusOr<T> GetNearestNeighbor(const T& data) const override {
    // Check if the distance function is set.
    if (NearestNeighbors<T>::distance_function_ == nullptr) {
      return absl::FailedPreconditionError(
          "No distance function set. Setting the distance function prior to "
          " calling Nearest() is required.");
    }

    // Error case: No data available.
    INTR_ASSIGN_OR_RETURN(const int number_elements, Size());
    if (number_elements < 1) {
      return absl::UnavailableError(
          "Data structure is empty. No nearest neighbors available.");
    }

    return nearest(data);
  }

  absl::StatusOr<std::vector<T>> GetKNearestNeighbors(
      const T& data) const override {
    return GetKNearestNeighbors(data, NearestNeighbors<T>::k_);
  }

  absl::StatusOr<std::vector<T>> GetKNearestNeighbors(const T& data,
                                                      int k) const override {
    // Check if the distance function is set.
    if (NearestNeighbors<T>::distance_function_ == nullptr) {
      return absl::FailedPreconditionError(
          "No distance function set. Setting the distance function prior to "
          " calling Nearest() is required.");
    }

    if (k <= 0) {
      return absl::InvalidArgumentError("K must be a positive non zero value");
    }

    // Trivial case
    std::vector<T> result;
    INTR_ASSIGN_OR_RETURN(const int number_elements, Size());
    if (number_elements < 1) {
      return result;
    }

    nearestK(data, k, result);
    return result;
  }

  absl::StatusOr<std::vector<T>> GetNearestNeighborsWithinRadius(
      const T& data) const override {
    return GetNearestNeighborsWithinRadius(data, NearestNeighbors<T>::radius_);
  }

  absl::StatusOr<std::vector<T>> GetNearestNeighborsWithinRadius(
      const T& data, double r) const override {
    // Check if the distance function is set.
    if (NearestNeighbors<T>::distance_function_ == nullptr) {
      return absl::FailedPreconditionError(
          "No distance function set. Setting the distance function prior to "
          " calling Nearest() is required.");
    }

    if (r <= 0) {
      return absl::InvalidArgumentError(
          "The search radius must be a positive non zero value");
    }

    // Trivial case
    std::vector<T> result;
    INTR_ASSIGN_OR_RETURN(const int number_elements, Size());
    if (number_elements < 1) {
      return result;
    }

    nearestR(data, r, result);

    return result;
  }

  // Gets the number of elements stored in the internal data structure.
  absl::StatusOr<int> Size() const override { return data_.size(); }

 protected:
  // Stored data elements for which we compute the nearest neighbor.
  std::vector<T> data_;

  // Default settings for nearest neighbor configuration.
  static const proto::NearestNeighborConfig* default_nn_config_;

  // Returns the nearest neighbor using brute force linear search. If data_
  // is empty, it will return an internal error.
  absl::StatusOr<T> nearest(const T& data) const {
    const int size = data_.size();

    // Error case: No elements in the struct.
    if (size == 0) {
      return absl::InternalError(
          "Error using Nearest Neighbor: attempted to find nearest neighbor "
          "for empty data set.");
    }

    // General case: Find the minimum neighbor. Initialize with first element in
    // data_ to cover trivial case.
    int nearest_neighbor = 0;
    double min_distance =
        NearestNeighbors<T>::distance_function_(data_[nearest_neighbor], data);
    for (int i = 1; i < size; ++i) {
      const double current_distance =
          NearestNeighbors<T>::distance_function_(data_[i], data);
      if (min_distance > current_distance) {
        nearest_neighbor = i;
        min_distance = current_distance;
      }
    }

    return data_[nearest_neighbor];
  }

  // Return the k nearest neighbors in sorted order.
  void nearestK(const T& data, int k,
                std::vector<T>& k_nearest_neighbor) const {
    k_nearest_neighbor = data_;

    // Trivial case.
    if (k_nearest_neighbor.size() == 1) {
      return;
    }

    if (data_.size() > k) {
      // TODO(b/372953330): Test maintaining a distance map instead of
      // recomputing the values every time.
      std::partial_sort(
          k_nearest_neighbor.begin(), k_nearest_neighbor.begin() + k,
          k_nearest_neighbor.end(), [data, this](const T& a, const T& b) {
            return NearestNeighbors<T>::distance_function_(a, data) <
                   NearestNeighbors<T>::distance_function_(b, data);
          });
      k_nearest_neighbor.resize(k);
      return;
    }
    // TODO(b/372953330): Test maintaining a distance map instead of
    // recomputing the values every time.
    std::sort(k_nearest_neighbor.begin(), k_nearest_neighbor.end(),
              [data, this](const T& a, const T& b) {
                return NearestNeighbors<T>::distance_function_(a, data) <
                       NearestNeighbors<T>::distance_function_(b, data);
              });
  }

  // Returns all nearest neighbor within radius distance according to specified
  // distance function. The neighbors returned in the provided data structure
  // are sorted according to distance.
  void nearestR(const T& data, double radius,
                std::vector<T>& nearest_neighbors) const {
    nearest_neighbors.clear();
    for (const auto& element : data_) {
      if (NearestNeighbors<T>::distance_function_(element, data) <= radius) {
        nearest_neighbors.push_back(element);
      }
    }
    std::sort(nearest_neighbors.begin(), nearest_neighbors.end(),
              [data, this](const T& a, const T& b) {
                return NearestNeighbors<T>::distance_function_(a, data) <
                       NearestNeighbors<T>::distance_function_(b, data);
              });
  }
};

template <typename T>
const proto::NearestNeighborConfig*
    LinearNearestNeighbor<T>::default_nn_config_ =
        new proto::NearestNeighborConfig(ParseTextProtoOrDie(R"pb(
          k: 15
          radius: 0.5
          strategy: KNEAREST
        )pb"));

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DATA_STRUCTURES_LINEAR_NEAREST_NEIGHBOR_H_
