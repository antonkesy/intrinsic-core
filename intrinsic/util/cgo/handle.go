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

// Package handle provides functions for passing Go pointers through C++ code.
//
// Go does not permit arbitrary pointers to be passed to C++
// (the exact rules are at http://tip.golang.org/cmd/cgo/#hdr-Passing_pointers).
// If the C++ code needs to access the data in the pointers, you need to
// allocate C++ memory to hold that data.
//
// If you just need to pass Go pointers through C++ code that will pass
// them back to Go code, you can use this package.
// Multiple goroutines may call the functions in this package simultaneously.
//
// Copied from google3/go/c/handle.go.
package handle

// #include <stdint.h>
import "C"

import (
	"fmt"
	"sync"

	log "github.com/golang/glog"
)

// H is the type of a handle to a Go value.  The underlying type may change, but
// values are guaranteed to always fit in a C.uintptr_t.
//
// The zero H is never a valid handle, and is thus safe to use as a sentinel in
// C APIs.
type H C.uintptr_t

// Store is a slice of handles.
// It is used to manage the lifetime of handles passed to C++ code.
// When the Store is destroyed, all handles in it are deleted.
type Store []H

// Map from handle to value.
var m = make(map[H]any)

// Lock for m.
var mu sync.Mutex

// The next handle value to use.  The value doesn't matter, but we
// don't start at 0 to reduce the number of accidental C values treated as valid handles.
var next = H(100)

// New returns a handle for a Go value.
// The handle is valid until the program calls `Delete` on it.  The handle uses resources, and this
// package assumes that C++ code may hold on to the handle, so the program must explicitly call
// `Delete` when the handle is no longer needed.  The intended use is to pass the returned handle
// to C++ code, which passes it back to Go, which calls Value.
//
// To track ownership of these handles in C++, use the go::Handle type defined
// in google3/go/c/handle.h.
func New(v any) H {
	mu.Lock()
	defer mu.Unlock()
	for {
		if _, ok := m[next]; !ok && next != 0 {
			break
		}
		next++
	}

	h := next
	m[h] = v
	next++

	return h
}

// Delete removes a handle.
// This should be called when C++ code no longer has a copy of the handle, and
// the program no longer needs it. This returns an error if the handle is not valid.
func Delete(h H) error {
	mu.Lock()
	defer mu.Unlock()
	var err error
	if _, ok := m[h]; !ok {
		err = fmt.Errorf("handle.Delete called on invalid handle %d", h)
	} else {
		delete(m, h)
	}

	return err
}

// Value returns the Go value for a handle.
// If the handle is not valid, this returns an error.
func Value(h H) (any, error) {
	mu.Lock()
	defer mu.Unlock()
	v, ok := m[h]

	var err error
	if !ok {
		if h < next {
			err = fmt.Errorf("handle.Value called on deleted handle %d", h)
		} else {
			err = fmt.Errorf("handle.Value called on invalid handle %d", h)
		}
	}

	return v, err
}

// go_c_DeleteHandle deletes a Go handle.
// Calling go_c_DeleteHandle on the zero H is a no-op (like `delete` in C++ or `free` in C).
//
//export go_c_DeleteHandle
func go_c_DeleteHandle(h C.uintptr_t) {
	if h != 0 {
		err := Delete(H(h))
		if err != nil {
			log.Fatalf("handle.Delete(h) returned error: %v", err)
		}
	}
}
