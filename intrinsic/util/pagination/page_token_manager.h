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

#ifndef INTRINSIC_UTIL_PAGINATION_PAGE_TOKEN_MANAGER_H_
#define INTRINSIC_UTIL_PAGINATION_PAGE_TOKEN_MANAGER_H_

#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/escaping.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/time/clock.h"
#include "absl/types/span.h"
#include "intrinsic/util/pagination/page_token.pb.h"
#include "intrinsic/util/proto_time.h"

namespace intrinsic::page_token {
// intrinsic::page_token provides functions for working with opaque page tokens
// (https://google.aip.dev/158#opacity).

// Opacify constructs an opaque page token. The original request MUST be used to
// validate that the passed pagination token matches the parameters of the
// previous request. In order to store the pagination boundary inside of the
// token, the library developers have two non-mutually exclusive possibilities:
// - "startAfterID" is the Firestore document ID of the pagination boundary
//   document.
// - "startAfter", the values of the order-by-fields which is the boundary
//   of the page.
//
// Special case: Empty "start_after_values" and empty "start_after_id" map to an
// empty opaque token because it has a special semantics of "start from the
// beginning".
//
// See also the documentation for
// [intrinsic/kubernetes/data_store/firestore_shim/firestoreshimtypes.ListParams]
// for the further explanation.
//
// The request used to create the page token should itself NOT contain a page
// token. Otherwise the page token will keep being re-encoded in every
// subsequent request. This would cause the request to unnecessarily keep
// growing in size. Simply setting the previous page token to an empty string
// before passing the request to this function is sufficient.
//
// Returns an opaque page token or absl::Status on error.
template <typename T>
absl::StatusOr<std::string> Opacify(
    const T& data, absl::string_view start_after_id = "",
    absl::Span<const std::string> start_after_values = {}) {
  if (start_after_values.empty() && start_after_id.empty()) {
    return "";
  }
  std::string data_bytes;
  if (!data.SerializeToString(&data_bytes)) {
    return absl::InvalidArgumentError(
        absl::StrFormat("cannot pack request data: %s", data.DebugString()));
  }

  intrinsic_proto::pagination::internal::PageToken page_token;
  *page_token.mutable_created() =
      intrinsic::FromAbslTimeClampToValidRange(absl::Now());
  page_token.set_request(std::move(data_bytes));
  page_token.mutable_start_after()->Assign(start_after_values.begin(),
                                           start_after_values.end());
  page_token.set_start_after_id(start_after_id);

  std::string page_token_bytes;
  if (!page_token.SerializeToString(&page_token_bytes)) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "cannot pack page token proto: %s", page_token.DebugString()));
  }
  std::string token_base64;
  absl::WebSafeBase64Escape(std::move(page_token_bytes), &token_base64);
  return token_base64;
}

// Deopacify converts an opaque page token to a page token proto.
//
// Special case: If an empty token is given, return no error and an empty
// token as well (see docstring for [Opacify]).
inline absl::StatusOr<intrinsic_proto::pagination::internal::PageToken>
Deopacify(absl::string_view token) {
  std::string page_token_bytes;
  if (!absl::WebSafeBase64Unescape(token, &page_token_bytes)) {
    return absl::InvalidArgumentError(
        absl::StrFormat("cannot decode base64 token: %s", token));
  }
  intrinsic_proto::pagination::internal::PageToken page_token;
  if (!page_token.ParseFromString(page_token_bytes)) {
    return absl::InvalidArgumentError(
        absl::StrFormat("cannot unpack page token proto: %s", token));
  }
  return page_token;
}

// RecoverRequest is a helper function that recovers the request proto of the
// specific type from the bytes in the [pb.PageToken] proto.
template <typename T>
absl::StatusOr<T> RecoverRequest(
    const intrinsic_proto::pagination::internal::PageToken& page_token) {
  T data;
  if (page_token.request().empty()) {
    return data;
  }
  if (!data.ParseFromString(page_token.request())) {
    return absl::InvalidArgumentError(
        absl::StrFormat("cannot unpack request: %s", page_token.DebugString()));
  }
  return data;
}

}  // namespace intrinsic::page_token

#endif  // INTRINSIC_UTIL_PAGINATION_PAGE_TOKEN_MANAGER_H_
