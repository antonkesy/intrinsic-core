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

// Package cgoconvert provides conversion functions between native Go types and C wrapper types.
package cgoconvert

// Stripped down version of google3/go/c/cgo_c.go

/*
#include "intrinsic/util/cgo/c_types.h"

// For Proto
static void unmarshal(go_c_ProtoOut dst, char* data, size_t len) {
  dst.unmarshal_func(dst.message, data, len);
}

// For Status
static void write_status(go_c_StatusOut dst, char* data, size_t len) {
  dst.write_func(dst.status, data, len);
}

// For string resize
static void* resize_string(go_c_StringOut dst, size_t len) {
  return dst.resize_func(dst.str, len);
}
*/
import (
	"C"
)

import (
	"bytes"
	"fmt"
	"reflect"
	"unsafe"

	"intrinsic/util/cgo/gotypes"
	"intrinsic/util/status/approximatestatus"

	"google.golang.org/grpc/status"
	"google.golang.org/protobuf/proto"
)

// TranslateStringIn returns a copy of C.go_c_StringIn (which wraps a C++ string) as a Go string.
func TranslateStringIn(in any) string {
	var ain C.go_c_StringIn
	gotypes.CopyStruct(&ain, in)
	return string(gotypes.AsBytes(unsafe.Pointer(ain.data), gotypes.CSize(ain.length)))
}

// protoName returns the non-empty fully-qualified name of the provided message.
// Will panic if the name is empty.
func protoName(msg proto.Message) (string, error) {
	name := string(proto.MessageName(msg))
	if len(name) == 0 {
		return "", fmt.Errorf("empty proto type name")
	}
	return name, nil
}

// TranslateProtoIn returns a proto message from the provided C.go_c_ProtoIn (which represents a
// serialized message).
func TranslateProtoIn(in any, msg proto.Message) error {
	var inp C.go_c_ProtoIn
	gotypes.CopyStruct(&inp, in)

	goName, err := protoName(msg)
	if err != nil {
		return err
	}
	cName := gotypes.AsBytes(unsafe.Pointer(inp.name_data), gotypes.CSize(inp.name_length))
	if bytes.Compare([]byte(goName), cName) != 0 {
		return fmt.Errorf("wrong proto destination type %q (expected %q)", goName, cName)
	}
	opts := proto.UnmarshalOptions{AllowPartial: true}
	err = opts.Unmarshal(gotypes.AsBytes(unsafe.Pointer(inp.data), gotypes.CSize(inp.length)), msg)
	if err != nil {
		return err
	}
	return nil
}

// bigArrayOf returns the type of a maximally-sized array of t.
//
// This works around the memory-stranding issue described in
// https://golang.org/issue/13656 by producing only one array type per element
// type (instead of one array type per length).
func bigArrayOf(t reflect.Type) reflect.Type {
	n := ^uintptr(0) / uintptr(t.Size())
	const maxInt = uintptr(^uint(0) >> 1)
	if n > maxInt {
		n = maxInt
	}
	return reflect.ArrayOf(int(n), t)
}

// TranslateSliceIn returns a slice over the C++-owned data in the form of a C.go_c_SliceIn.
func TranslateSliceIn(in any, t reflect.Type) reflect.Value {
	var ain C.go_c_SliceIn
	gotypes.CopyStruct(&ain, in)
	n := gotypes.MustIntOrPanic(gotypes.CSize(ain.n_elts))
	ptr := unsafe.Pointer(ain.data)
	elemType := t.Elem()

	// return a slice over the C++-owned data
	if ptr == nil && n == 0 {
		return reflect.Zero(reflect.SliceOf(elemType))
	}
	return reflect.NewAt(bigArrayOf(elemType), ptr).Elem().Slice3(0, n, n)
}

// TranslateStringSliceIn returns a copy of the provided C.go_c_StringSliceIn as a slice of strings.
func TranslateStringSliceIn(in any) []string {
	var sp C.go_c_StringSliceIn
	gotypes.CopyStruct(&sp, in)
	n := gotypes.MustIntOrPanic(gotypes.CSize(sp.n_elts))

	var result []string
	arr := unsafe.Slice((*C.go_c_StringIn)(unsafe.Pointer(sp.data)), n)
	for _, si := range arr {
		result = append(result, TranslateStringIn(si))
	}
	return result
}

// TranslateStringOut copies the string to the output pointer `dst` of type *C.go_c_StringOut.
// dst is C++ owned memory.
func TranslateStringOut(dst any, s string) {
	var dstp C.go_c_StringOut
	gotypes.CopyStruct(&dstp, dst)
	if len(s) == 0 {
		C.resize_string(dstp, 0)
		return
	}

	sz := C.size_t(gotypes.MustSizeOrPanic(len(s)))
	ptr := C.resize_string(dstp, sz)
	copy(gotypes.AsBytes(ptr, gotypes.CSize(sz)), s)
}

// TranslateProtoOut serializes the protobuf message to `dst` output of type *C.go_c_ProtoOut.
func TranslateProtoOut(dst any, msg proto.Message) error {
	var dstp C.go_c_ProtoOut
	gotypes.CopyStruct(&dstp, dst)

	// Check that the proto type matches.
	goName, err := protoName(msg)
	if err != nil {
		return err
	}
	cName := gotypes.AsBytes(unsafe.Pointer(dstp.name_data), gotypes.CSize(dstp.name_length))
	if bytes.Compare([]byte(goName), cName) != 0 {
		return fmt.Errorf("wrong proto source type %q (expected %q)", goName, cName)
	}

	if msg == nil || !msg.ProtoReflect().IsValid() {
		return nil
	}

	b, err := proto.MarshalOptions{AllowPartial: true}.Marshal(msg)
	if err != nil {
		return err
	}
	ptr := unsafe.Pointer(unsafe.SliceData(b))
	C.unmarshal(dstp, (*C.char)(ptr), C.size_t(gotypes.MustSizeOrPanic(len(b))))
	return nil
}

// TranslateStatusOut writes the error to `dst` output of type C.go_c_StatusOut.
// dst is C++ owned memory.
func TranslateStatusOut(dst any, err error) {
	var dstp C.go_c_StatusOut
	gotypes.CopyStruct(&dstp, dst)
	if err == nil && !dstp.write_if_ok {
		return
	}
	st, ok := status.FromError(err)
	if !ok {
		st = status.New(approximatestatus.Code(err), err.Error())
	}
	b, marshErr := proto.Marshal(st.Proto())
	if marshErr != nil {
		panic(fmt.Sprintf("Write: %v", marshErr))
	}

	C.write_status(dstp, (*C.char)(unsafe.Pointer(unsafe.SliceData(b))), C.size_t(gotypes.MustSizeOrPanic(len(b))))
}
