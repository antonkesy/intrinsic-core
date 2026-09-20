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

#ifndef INTRINSIC_UTIL_CGO_CPP_TYPES_H_
#define INTRINSIC_UTIL_CGO_CPP_TYPES_H_

#include <cstddef>
#include <string>
#include <vector>

#include "absl/log/die_if_null.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "google/protobuf/message.h"
#include "google/protobuf/message_lite.h"
#include "grpcpp/support/status.h"
#include "intrinsic/util/cgo/c_types.h"
#include "intrinsic/util/status/status_conversion_grpc.h"

namespace intrinsic {

void WriteStatus(go_c_Handle status, const char* data, size_t length);

template <typename Func, typename... Args>
absl::Status GetStatus(Func f, Args&&... args) {
  absl::Status status;
  go_c_StatusOut out = {
      .status = reinterpret_cast<go_c_Handle>(&status),
      .write_func = &WriteStatus,
      .write_if_ok = false,
  };
  f(std::forward<Args>(args)..., out);
  return status;
}

template <typename Func, typename... Args>
grpc::Status GetStatusGrpc(Func f, Args&&... args) {
  return ToGrpcStatus(GetStatus(f, std::forward<Args>(args)...));
}

go_c_StringIn ToStringIn(absl::string_view s);

go_c_SliceIn ToSliceIn(const std::string& arr);

go_c_StringSliceIn ToStringSliceIn(const std::vector<std::string>& arr);

class StringSliceIn {
 public:
  explicit StringSliceIn(const std::vector<std::string>& arr) {
    arr_ = arr;
    data_.reserve(arr_.size());
    for (const auto& s : arr_) {
      data_.push_back(ToStringIn(s));
    }
  }
  go_c_StringSliceIn ToGo() {
    return go_c_StringSliceIn{data_.data(), data_.size()};
  }

 private:
  std::vector<std::string> arr_;
  std::vector<go_c_StringIn> data_;
};

class ProtoIn {
 public:
  explicit ProtoIn(const google::protobuf::Message& msg)
      : data_(msg.SerializeAsString()), name_(msg.GetTypeName()) {}

  go_c_ProtoIn ToGo() {
    return go_c_ProtoIn{data_.data(), data_.size(), name_.data(), name_.size()};
  }

 private:
  const std::string data_;
  const std::string name_;
};

void ParseFromString(go_c_Handle message, const char* data, size_t length);

class ProtoOut {
 public:
  explicit ProtoOut(google::protobuf::Message* msg)
      : name_(ABSL_DIE_IF_NULL(msg)->GetTypeName()), msg_(msg) {}

  // NOTE: this is not marked as `explicit` because it is intended for automatic
  // conversion from `ProtoOut` to `go_c_ProtoOut`.
  // NOLINTNEXTLINE(google-explicit-constructor)
  operator go_c_ProtoOut() {
    return go_c_ProtoOut{reinterpret_cast<go_c_Handle>(msg_), name_.data(),
                         name_.size(), &ParseFromString};
  }

 private:
  // Explicitly store the message type to make sure message check works
  // correctly when unmarshalling the data.
  const std::string name_;
  google::protobuf::Message* const msg_;
};

// SliceIn passes the contents of a C++ container of a numeric type as a Go
// slice parameter as an immutable slice.
// The Go function must *not* write to the slice.
class SliceIn {
 public:
  template <typename C>
  explicit SliceIn(const C* arr) : SliceIn(absl::MakeConstSpan(*arr)) {}

  template <typename T>
  explicit SliceIn(absl::Span<const T> arr)
      : in_({const_cast<T*>(arr.data()), arr.size()}) {}

  go_c_SliceIn ToGo() { return in_; }

 private:
  const go_c_SliceIn in_;
};

// StringOut appends characters from a returned Go string to the end of a
// C++ string.
class StringOut {
 public:
  explicit StringOut(std::string* s) : s_(ABSL_DIE_IF_NULL(s)) {}

  // NOTE: this is not marked as `explicit` because it is intended for automatic
  // conversion from `StringOut` to `go_c_StringOut`.
  // NOLINTNEXTLINE(google-explicit-constructor)
  operator go_c_StringOut() {
    return go_c_StringOut{reinterpret_cast<go_c_Handle>(s_), &Resize};
  }

 private:
  // Returns a pointer to the insertion point in the new array.
  static char* Resize(go_c_Handle sp, size_t length) {
    std::string& s = *ABSL_DIE_IF_NULL(reinterpret_cast<std::string*>(sp));
    size_t old_size = s.size();
    s.resize(old_size + length);
    return s.empty() ? nullptr : &s[old_size];
  }

  std::string* const s_;
};

}  // namespace intrinsic

#endif  // INTRINSIC_UTIL_CGO_CPP_TYPES_H_
