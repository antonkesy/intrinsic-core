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
	"bytes"
	"encoding/json"
)

// Optional is a generic wrapper representing a value that may or may not be set.
type Optional[T any] struct {
	Set bool `json:"set"`
	Val T    `json:"val"`
}

// NewOptional creates a populated Optional[T].
func NewOptional[T any](val T) Optional[T] {
	return Optional[T]{Val: val, Set: true}
}

// IsZero returns true if the Optional is not set.
func (o Optional[T]) IsZero() bool {
	return !o.Set
}

// Get returns the value held by the Optional. If not set, it returns the zero value of T.
func (o Optional[T]) Get() T {
	return o.Val
}

// GetOr returns the value held by the Optional, or defaultVal if not set.
func (o Optional[T]) GetOr(defaultVal T) T {
	if !o.Set {
		return defaultVal
	}
	return o.Val
}

// MarshalJSON marshals the Optional value to JSON (null if not set).
func (o Optional[T]) MarshalJSON() ([]byte, error) {
	if !o.Set {
		return []byte("null"), nil
	}
	return json.Marshal(o.Val)
}

// UnmarshalJSON unmarshals the Optional value from JSON.
func (o *Optional[T]) UnmarshalJSON(data []byte) error {
	if len(data) == 0 || string(bytes.TrimSpace(data)) == "null" {
		o.Set = false
		return nil
	}
	var val T
	if err := json.Unmarshal(data, &val); err != nil {
		return err
	}
	o.Val = val
	o.Set = true
	return nil
}
