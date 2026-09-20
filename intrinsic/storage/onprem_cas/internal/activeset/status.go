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

package activeset

import (
	"google.golang.org/grpc/codes"
)

// ObjectStatus holds the detailed availability status of a single object.
type ObjectStatus struct {
	Digest    Digest               `json:"digest"`
	Size      Optional[uint64]     `json:"size,omitzero"`
	Local     Optional[bool]       `json:"local,omitzero"`
	Upstream  Optional[bool]       `json:"upstream,omitzero"`
	ErrorMsg  Optional[string]     `json:"error,omitzero"`
	ErrorCode Optional[codes.Code] `json:"error_code,omitzero"`
}

// Copy creates a shallow copy of the ObjectStatus pointer. Because all fields are values
// or immutable optionals, a shallow copy produces a safe, independent copy.
func (o *ObjectStatus) Copy() *ObjectStatus {
	if o == nil {
		return nil
	}
	cp := *o
	return &cp
}
