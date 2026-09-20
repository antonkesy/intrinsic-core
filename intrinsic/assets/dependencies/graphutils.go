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

// Package graphutils provides utility functions that can be used for implementing
// graph construction for dependency validation.
package graphutils

import (
	"intrinsic/assets/interfaceutils"
	"intrinsic/util/proto/names"
)

// ExtractGRPCInterfacesFromPrefixes extracts GRPC interfaces from Service proto prefixes.
func ExtractGRPCInterfacesFromPrefixes(prefixes []string) (map[string]bool, error) {
	provides := make(map[string]bool)
	for _, prefix := range prefixes {
		if err := names.ValidateProtoPrefix(prefix); err != nil {
			return nil, err
		}
		provides[interfaceutils.GRPCURIPrefix+prefix[1:len(prefix)-1]] = true
	}
	return provides, nil
}
