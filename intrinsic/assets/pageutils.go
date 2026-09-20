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

// Package pageutils provides paginations utils for assets.
package pageutils

import (
	"encoding/base64"
	"reflect"

	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"
	"google.golang.org/protobuf/proto"

	pagetokendatapb "intrinsic/assets/proto/page_token_data_go_proto"
)

// ResolvePageSize resolves a user-provided page size.
func ResolvePageSize(pageSize int) (int, error) {
	if pageSize == 0 {
		return 20, nil
	} else if pageSize > 200 {
		return 200, nil
	} else if pageSize < 0 {
		return 0, status.Errorf(codes.InvalidArgument, "page size must be non-negative")
	}
	return pageSize, nil
}

// EncodePageToken encodes a list of start after IDs into a page token for inclusion in a response
// to a list request.
func EncodePageToken(startAfterIDs []string, atEnd bool, params []string) (string, error) {
	if atEnd {
		return "", nil
	}

	tokenData := &pagetokendatapb.PageTokenData{
		StartAfterIds: startAfterIDs,
		Params:        params,
	}
	tokenBytes, err := proto.Marshal(tokenData)
	if err != nil {
		return "", status.Errorf(codes.Internal, "failed to encode page token: %v", err)
	}
	token := base64.URLEncoding.EncodeToString(tokenBytes)

	return token, nil
}

// DecodePageToken decodes a page token encoded by EncodePageToken.
//
// It also verifies that operation parameters are consistent with those that were used to encode the
// token.
func DecodePageToken(token string, expectedParams []string) ([]string, error) {
	if token == "" {
		return nil, nil
	}

	tokenBytes, err := base64.URLEncoding.DecodeString(token)
	if err != nil {
		return nil, status.Errorf(codes.Internal, "failed to decode page token string: %v", err)
	}
	tokenData := &pagetokendatapb.PageTokenData{}
	if err := proto.Unmarshal(tokenBytes, tokenData); err != nil {
		return nil, status.Errorf(codes.Internal, "failed to unmarshal page token: %v", err)
	}

	if !reflect.DeepEqual(tokenData.GetParams(), expectedParams) {
		return nil, status.Errorf(codes.InvalidArgument, "page token params do not match expected: (want: %v, got: %v)", expectedParams, tokenData.GetParams())
	}

	startAfterIDs := tokenData.GetStartAfterIds()
	if startAfterIDs == nil {
		startAfterIDs = []string{}
	}

	return startAfterIDs, nil
}
