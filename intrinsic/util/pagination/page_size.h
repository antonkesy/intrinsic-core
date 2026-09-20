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

#ifndef INTRINSIC_UTIL_PAGINATION_PAGE_SIZE_H_
#define INTRINSIC_UTIL_PAGINATION_PAGE_SIZE_H_

#include "absl/status/statusor.h"

namespace intrinsic {

// PageSizeValidator can validate the page size from a request following the
// guidelines for pagination in go/aip/158.
class PageSizeValidator {
 public:
  // Creates a new PageSizeValidator.
  //
  // Args:
  //   default_size: The default page size if the requested page size is 0.
  //   max_size: The maximum page size to allow.
  PageSizeValidator(int default_size, int max_size);

  // Validates the page size provided in a request following go/aip/158. It
  // returns the validated (and potentially modified) page size or an error.
  //
  // The AIP defines the following non-normal behavior:
  // * use default page size if not provided or provided as 0
  // * coerce down to maximum page size
  // * return error if negative
  absl::StatusOr<int> Run(int page_size) const;

 private:
  int default_size_;
  int max_size_;
};

}  // namespace intrinsic

#endif  // INTRINSIC_UTIL_PAGINATION_PAGE_SIZE_H_
