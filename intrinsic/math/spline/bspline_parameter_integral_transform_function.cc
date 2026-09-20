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

#include "intrinsic/math/spline/bspline_parameter_integral_transform_function.h"

#include <algorithm>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "intrinsic/math/almost_equals.h"

namespace intrinsic {
namespace internal {

std::vector<int> GetIndicesOfDuplicatedValues(absl::Span<const double> values) {
  if (values.size() < 2) {
    return {};
  }
  // We start the search from the second element and use a support index to
  // compare the elements. This allows us to always keep the first and last
  // elements of `values`, if they are not duplicated. Otherwise, we remove the
  // first element.
  int index_to_compare = 0;
  std::vector<int> indices_to_remove;
  indices_to_remove.reserve(values.size());
  for (int i = 1; i < values.size(); ++i) {
    if (AlmostEquals(values[i], values[index_to_compare])) {
      // We remove the current element if it is not the last element of
      // `values`, otherwise we remove the element to compare. This is to avoid
      // removing the last element of `values`.
      if (i == values.size() - 1) {
        indices_to_remove.push_back(index_to_compare);
      } else {
        indices_to_remove.push_back(i);
      }
    } else {
      index_to_compare = i;
    }
  }
  std::sort(indices_to_remove.begin(), indices_to_remove.end());
  return indices_to_remove;
}

absl::Status RemoveDuplicates(absl::Span<const int> indices_to_remove,
                              std::vector<double>& values) {
  if (indices_to_remove.empty()) {
    return absl::OkStatus();
  }
  for (auto index : indices_to_remove) {
    if (index >= values.size() || index < 0) {
      return absl::InvalidArgumentError(
          absl::StrCat("Index ", index, " is out of bounds for values of size ",
                       values.size(), "."));
    }
  }

  // We use two auxiliary indices to efficiently remove the elements at the
  // indices given in `indices_to_remove` from `values`. We modify the `values`
  // vector in place, such that the first `unique_elements_index` elements of
  // `values` are the non-duplicate elements, and the rest of the elements are
  // removed.
  int indices_to_remove_index = 0;
  int unique_elements_index = 0;
  for (int values_index = 0; values_index < values.size(); ++values_index) {
    if (indices_to_remove_index < indices_to_remove.size() &&
        values_index == indices_to_remove[indices_to_remove_index]) {
      indices_to_remove_index++;
    } else {
      if (unique_elements_index != values_index) {
        values[unique_elements_index] = values[values_index];
      }
      unique_elements_index++;
    }
  }
  values.erase(values.begin() + unique_elements_index, values.end());

  return absl::OkStatus();
}

}  // namespace internal
}  // namespace intrinsic
