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

// This header contains C types for marshalling Go data into and out of other
// languages via cgo.
// This file is a small subset of google3/go/c/c.h.

#ifndef INTRINSIC_UTIL_CGO_C_TYPES_H_
#define INTRINSIC_UTIL_CGO_C_TYPES_H_

#include <stdbool.h>  // for bool // IWYU pragma: keep
#include <stddef.h>   // for size_t
#include <stdint.h>   // for uintptr_t

#ifdef __cplusplus
extern "C" {
#endif

// go_c_ElementTypeEnum provides a way to automatically pass value type
// information for containers (e.g., vectors) into Go and to enforce
// restrictions on which value types are allowed.
typedef enum {
  GO_C_ELEMENT_TYPE_INT8,
  GO_C_ELEMENT_TYPE_INT16,
  GO_C_ELEMENT_TYPE_INT32,
  GO_C_ELEMENT_TYPE_INT64,
  GO_C_ELEMENT_TYPE_UINT8,
  GO_C_ELEMENT_TYPE_UINT16,
  GO_C_ELEMENT_TYPE_UINT32,
  GO_C_ELEMENT_TYPE_UINT64,
  GO_C_ELEMENT_TYPE_FLOAT,
  GO_C_ELEMENT_TYPE_DOUBLE,
} go_c_ElementTypeEnum;

// C structs representing possible input/output types:

// go_c_Handle is an opaque handle to an object - typically a pointer from C or
// a handle.H from Go.
typedef uintptr_t go_c_Handle;

typedef struct {
  void* data;
  size_t n_elts;  // Number of container elements.
  go_c_ElementTypeEnum element_type_id;
} go_c_SliceIn;

typedef struct {
  const char* data;
  size_t length;
  const char* name_data;
  size_t name_length;
} go_c_ProtoIn;

typedef struct {
  const uint8_t* data;
  size_t length;
} go_c_StringIn;

typedef struct {
  const go_c_StringIn* data;
  size_t n_elts;
} go_c_StringSliceIn;

typedef struct {
  go_c_Handle message;
  const char* name_data;
  size_t name_length;
  void (*unmarshal_func)(go_c_Handle message, const char*, size_t);
} go_c_ProtoOut;

typedef struct {
  go_c_Handle status;

  // write_func must be called at most once with a serialized StatusProto
  // message. A length of 0 indicates a canonical OK status.
  void (*write_func)(go_c_Handle status, const char*, size_t);

  // If true, write_func must be called even if the status is OK.
  bool write_if_ok;
} go_c_StatusOut;

typedef struct {
  go_c_Handle str;  // handle to a Go string.
  char* (*resize_func)(go_c_Handle str, size_t);
} go_c_StringOut;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // INTRINSIC_UTIL_CGO_C_TYPES_H_
