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

// Package convertrunnables provides functions to serialize and deserialize
// runnables map into bytes, allowing us to store `Any` protos in Firestore.
package convertrunnables

import (
	"fmt"

	"github.com/pkg/errors"
	"google.golang.org/protobuf/proto"

	processpb "intrinsic/config/proto/process_go_proto"
	processinternalpb "intrinsic/config/proto/process_internal_go_proto"
)

// ErrInvalidState signals that a proto is in invalid state because both
// `runnables` and `runnables_as_bytes` fields are present.
var ErrInvalidState = errors.New("invalid state, proto contains both runnables and runnables_as_bytes")

// MoveRunnablesToBytes serializes the `runnables` map into bytes and stores
// this map into the `runnables_as_bytes` field. The `runnables` field is then
// cleared. This function is idempotent. Returns an error if the runnables
// cannot be marshaled to bytes or if the proto is in an invalid state, i.e.
// both the `runnables` and `runnables_as_bytes` fields are non-empty.
func MoveRunnablesToBytes(p *processpb.Process) error {
	hasRunnables := p.GetRunnables() != nil && len(p.GetRunnables()) != 0
	hasBytes := len(p.GetRunnablesAsBytes()) > 0 && len(p.GetRunnablesAsBytes()) != 0
	if !hasRunnables && hasBytes {
		// Runnables already moved to bytes, do nothing (be idempotent!).
		return nil
	}
	if hasRunnables && !hasBytes {
		ri := &processinternalpb.Runnables{}
		ri.Runnables = p.GetRunnables()
		bytes, err := proto.Marshal(ri)
		if err != nil {
			return fmt.Errorf("cannot marshal to bytes: %w", err)
		}
		p.RunnablesAsBytes = bytes
		p.Runnables = nil
		return nil
	}
	if hasRunnables && hasBytes {
		return ErrInvalidState
	}
	// Both `runnables` and `runnables_as_bytes` are empty: This is ok.
	return nil
}

// MoveBytesToRunnables deserializes the `runnables_as_bytes` field into the
// runnables map and stores this map into the `runnables` field.
// The `runnables_as_bytes` field is then cleared. This function is idempotent.
// Returns an error if the bytes cannot be unmarshaled from bytes or if the
// proto is in an invalid state, i.e. both the `runnables` and
// `runnables_as_bytes` fields are non-empty.
func MoveBytesToRunnables(p *processpb.Process) error {
	hasRunnables := p.GetRunnables() != nil && len(p.GetRunnables()) != 0
	hasBytes := len(p.GetRunnablesAsBytes()) > 0 && len(p.GetRunnablesAsBytes()) != 0
	if hasRunnables && !hasBytes {
		// Bytes already moved to runnables, do nothing (be idempotent!).
		return nil
	}
	if !hasRunnables && hasBytes {
		ri := &processinternalpb.Runnables{}
		if err := proto.Unmarshal(p.GetRunnablesAsBytes(), ri); err != nil {
			return fmt.Errorf("cannot unmarshal from bytes: %w", err)
		}
		p.Runnables = ri.GetRunnables()
		p.RunnablesAsBytes = nil
		return nil
	}
	if hasRunnables && hasBytes {
		return ErrInvalidState
	}
	// Both `runnables` and `runnables_as_bytes` are empty: This is ok.
	return nil
}
