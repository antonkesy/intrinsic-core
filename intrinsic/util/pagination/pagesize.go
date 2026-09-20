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

// Package pagesize provides helpers to handling of page size for
// implementations of go/aip/158 compliant pagination.
package pagesize

import (
	"errors"
)

// ErrNegative is returned by [Validator.Run] when the page size in the request
// is negative.
var ErrNegative = errors.New("must be non-negative")

// Validator can validate the page size from a request following the guidelines
// for pagination in go/aip/158.
type Validator struct {
	Default int
	Max     int
}

// Run validates the page size provided in a request following go/aip/158. It
// returns the validated (and potentially modified) page size or an error. If
// the [Validator] has a Default > Max the Max will be used as the default.
//
// The AIP defines the following non-normal behavior:
// * use default page size if not provided or provided as 0
// * coerce down to maximum page size
// * return error if negative
func (v Validator) Run(pageSize int) (int, error) {
	if pageSize == 0 {
		if v.Default > v.Max {
			return v.Max, nil
		}
		return v.Default, nil
	}
	if pageSize < 0 {
		return 0, ErrNegative
	}
	if pageSize > v.Max {
		return v.Max, nil
	}
	return pageSize, nil
}
