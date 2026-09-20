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

#ifndef INTRINSIC_MATH_TIME_SERIES_UTILS_H_
#define INTRINSIC_MATH_TIME_SERIES_UTILS_H_

#include <algorithm>
#include <functional>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/time/time.h"
#include "absl/types/span.h"

namespace intrinsic {

// Returns the index of the greatest element in a sorted `sequence` that is
// smaller or equal to `value`. Returns the index of the first element if no
// such element is found, i.e., if the first element of the `sequence` is
// greater than 'value'. A custom `comparator` can be used for performing
// comparisons other than the default `operator<`. Note that the `sequence`
// elements and the `value` might be of different types, as long as a suitable
// `comparator` is provided.
template <typename ElementType, typename ValueType,
          typename LessThan = std::less<>>
int GetLowerIndexForValue(const absl::Span<const ElementType> sequence,
                          const ValueType& value,
                          LessThan&& comparator = LessThan()) {
  // Returns an iterator pointing to the first element in the range
  // [ranges.begin(), ranges.end()) that is greater than 'value'.
  // Points to last entry if no such element is found. Complexity of this
  // operation is O(log(N)).
  const auto p =
      absl::c_upper_bound(sequence, value, std::forward<LessThan>(comparator));

  // Corner case left side of interval. Index should be zero between first and
  // second element.
  if (p == sequence.begin()) {
    return 0;
  }

  return std::distance(sequence.begin(), p) - 1;
}

// Returns a downsampled vector which contains every n-th element of the
// original vector, where n is the `downsampling_factor`. To maintain the
// target point of the input vector, the final datapoint is appended in any
// case, this means that the downsampling rate will be violated for the last
// interval. `downsampling_factor` must be >= 1, where 1 corresponds to a plain
// copy of the vector.
template <typename Data>
absl::StatusOr<std::vector<Data>> DownsampleVector(absl::Span<const Data> input,
                                                   int downsampling_factor) {
  if (downsampling_factor < 1) {
    return absl::InvalidArgumentError(absl::StrCat(
        "downsampling_factor must be >= 1, but got ", downsampling_factor));
  }

  if (downsampling_factor == 1) {
    return std::vector<Data>(input.begin(), input.end());
  }

  std::vector<Data> output;
  for (int i = 0; i < input.size(); i += downsampling_factor) {
    output.push_back(input.at(i));
  }
  if ((input.size() - 1) % downsampling_factor != 0) {
    output.push_back(input.back());
  }
  return output;
}

// Returns a copied segment of the `input` vector ranging from 'from_index' to
// 'to_index'. Returns 'kOutOfRangeError' if the indices are out of range.
template <typename Data>
absl::StatusOr<std::vector<Data>> ExtractVectorSegment(
    absl::Span<const Data> input, int from_index, int to_index) {
  if (from_index < 0) {
    return absl::OutOfRangeError("from_index must be >= 0.");
  }
  if (to_index > input.size() - 1) {
    return absl::OutOfRangeError("to_index exceeds the input vector size.");
  }
  if (to_index < from_index) {
    return absl::FailedPreconditionError(
        "from_index must be smaller or equal than to_index.");
  }
  return std::vector<Data>(input.begin() + from_index,
                           input.begin() + to_index + 1);
}

// Returns an Ok status if the sequence of provided `elements` are strictly
// monotonically increasing. Returns an error status otherwise.
template <typename Data>
absl::Status ElementsIncreaseStrictlyMonotonically(
    absl::Span<const Data> elements) {
  if (elements.empty()) {
    return absl::FailedPreconditionError("Elements must not be empty.");
  }

  for (int i = 1; i < elements.size(); i++) {
    if (elements[i - 1] >= elements[i]) {
      return absl::FailedPreconditionError(absl::StrCat(
          "Elements are not strictly monotonically increasing. "
          "Increment between elements at index ",
          i - 1, " and ", i, " is ", elements[i] - elements[i - 1], "."));
    }
  }

  return absl::OkStatus();
}

// Returns an Ok status if the sequence of provided `elements` are monotonically
// increasing. Returns an error status otherwise.
template <typename Data>
absl::Status ElementsIncreaseMonotonically(absl::Span<const Data> elements) {
  if (elements.empty()) {
    return absl::FailedPreconditionError("Elements must not be empty.");
  }

  for (int i = 1; i < elements.size(); i++) {
    if (elements[i - 1] > elements[i]) {
      return absl::FailedPreconditionError(absl::StrCat(
          "Elements are not monotonically increasing. "
          "Increment between elements at index ",
          i - 1, " and ", i, " is ", elements[i] - elements[i - 1], "."));
    }
  }

  return absl::OkStatus();
}

// Time shifts the input vector of `time_stamps` by a given `time_shift`.
inline void TimeShiftTimestampsVector(const absl::Duration time_shift,
                                      absl::Span<absl::Duration> time_stamps) {
  std::for_each(time_stamps.begin(), time_stamps.end(),
                [time_shift](absl::Duration& t) { t += time_shift; });
}

// Shifts the input vector `data` by a given `shift`.
inline void ShiftDataVector(const double shift, absl::Span<double> data) {
  std::for_each(data.begin(), data.end(), [shift](double& s) { s += shift; });
}

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_TIME_SERIES_UTILS_H_
