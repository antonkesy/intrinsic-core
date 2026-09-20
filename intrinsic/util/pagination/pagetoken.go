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

// Package pagetoken provides functions for working with opaque page tokens
// (https://google.aip.dev/158#opacity).
package pagetoken

import (
	"encoding/base64"
	"fmt"
	"time"

	"google.golang.org/protobuf/proto"

	pb "intrinsic/util/pagination/page_token_go_proto"

	timestamppb "google.golang.org/protobuf/types/known/timestamppb"
)

var timeNow = time.Now // Stubbed out for testing.

// Opacify constructs an opaque page token. It consumes the original request, which MUST be used to
// validate that the passed pagination token matches the parameters of the previous request. In
// order to store the pagination boundary inside of the token, the library developers have two
// non-mutually exclusive possibilities:
//
//   - "startAfterID" is the Firestore document ID of the pagination boundary document.
//   - "startAfter", the values of the order-by-fields which is the boundary of the page.
//
// Special case: Empty or nil "startAfter" and empty "startAfterID" map to an empty opaque token
// because it has a special semantics of "start from the beginning".
//
// See also the documentation for
// [intrinsic/kubernetes/data_store/firestore_shim/firestoreshimtypes.ListParams] for the
// further explanation.
//
// The request used to create the page token should itself NOT contain a page token. Otherwise the
// page token will keep being re-encoded in every subsequent request. This would cause the request
// to unnecessarily keep growing in size. Simply setting the previous page token to an empty string
// before passing the request to this function is sufficient.
//
// Return an opaque page token and an error.
func Opacify(req proto.Message, startAfterID string, startAfterValues []string) (string, error) {
	if len(startAfterValues) == 0 && startAfterID == "" {
		return "", nil
	}

	reqAsBytes, err := proto.Marshal(req)
	if err != nil {
		return "", fmt.Errorf("cannot marshal request: %w", err)
	}

	pt := &pb.PageToken{
		Created:      timestamppb.New(timeNow()),
		StartAfter:   startAfterValues,
		StartAfterId: startAfterID,
		Request:      reqAsBytes,
	}

	b, err := proto.Marshal(pt)
	if err != nil {
		return "", fmt.Errorf("cannot marshal page token proto: %w", err)
	}
	return base64.URLEncoding.EncodeToString(b), nil
}

// Deopacify converts an opaque page token to a page token proto.
//
// Special case: If an empty token is given, return no error and an empty token
// as well (see docstring for [Opacify]).
func Deopacify(token string) (*pb.PageToken, error) {
	b, err := base64.URLEncoding.DecodeString(token)
	if err != nil {
		return nil, fmt.Errorf("cannot decode base64: %w", err)
	}
	pt := &pb.PageToken{}
	if err := proto.Unmarshal(b, pt); err != nil {
		return nil, fmt.Errorf("cannot unmarshal page token proto: %w", err)
	}
	return pt, nil
}

// RecoverRequest is a helper function that recovers the request proto of the
// specific type from the bytes in the [pb.PageToken] proto.
func RecoverRequest(pt *pb.PageToken, msg proto.Message) error {
	if len(pt.GetRequest()) == 0 {
		return nil
	}
	if err := proto.Unmarshal(pt.GetRequest(), msg); err != nil {
		return fmt.Errorf("cannot unmarshal request: %w", err)
	}
	return nil
}
