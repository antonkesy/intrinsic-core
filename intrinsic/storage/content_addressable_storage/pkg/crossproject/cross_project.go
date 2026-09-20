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

// Package crossproject contains a function to copy objects from one content-addressable storage
// (CAS) service deployment (GCP project) to another efficiently.
package crossproject

import (
	"context"
	"fmt"
	"io"

	"intrinsic/storage/content_addressable_storage/pkg/clienthelpers"

	"go.opencensus.io/trace"

	casgrpcpb "intrinsic/storage/content_addressable_storage/proto/cas_service_go_proto"
	caspb "intrinsic/storage/content_addressable_storage/proto/cas_service_go_proto"
)

// Copy an object from one CAS project to another in a memory-efficient way, i.e. without allocating
// memory for the whole object, which can be potentially XX gigabytes large. Perform the usual
// checksumming and verify that the object ID in the destination project is the same as in the
// source project.
//
// This function does not check for the existence of the object in the destination project, you have
// to do it yourself. This function doesn't check if the source and destination clients are
// connected to the same project. This is safe to do, albeit very wasteful.
func Copy(ctx context.Context, from, to casgrpcpb.ContentAddressableStorageServiceClient, objectID string) error {
	ctx, span := trace.StartSpan(ctx, "crossproject.Copy")
	defer span.End()

	req := &caspb.GetRequest{ObjectId: objectID}
	readS, err := from.Get(ctx, req)
	if err != nil {
		return fmt.Errorf("creating read stream for %q: %w", req, err)
	}
	writeS, err := to.Create(ctx)
	if err != nil {
		return fmt.Errorf("creating write stream for %q: %w", req, err)
	}

	checksummer := clienthelpers.NewChecksummer()
	for {
		checksummer.Reset()
		res, err := readS.Recv()
		if err == io.EOF {
			break
		}
		if err != nil {
			return fmt.Errorf("receiving from read stream: %w", err)
		}
		if _, err := checksummer.Write(res.GetChecksummedData().GetContent()); err != nil {
			return fmt.Errorf("could not checksum content: %w", err)
		}
		if clientCRC, serverCRC := checksummer.Sum32(), res.GetChecksummedData().GetCrc32C(); clientCRC != serverCRC {
			return fmt.Errorf("checksum mismatch: client computed 0x%08x, server computed 0x%08x", clientCRC, serverCRC)
		}
		writeReq := &caspb.CreateRequest{ChecksummedData: res.GetChecksummedData()}
		if err := writeS.Send(writeReq); err != nil {
			return fmt.Errorf("sending to write stream: %w", writeS.RecvMsg(nil))
		}
	}
	res, err := writeS.CloseAndRecv()
	if err != nil {
		return fmt.Errorf("closing write stream: %w", err)
	}
	if fromID, toID := objectID, res.GetObjectId(); fromID != toID {
		return fmt.Errorf("object ID mismatch: origin has %q, newly uploaded has %q", fromID, toID)
	}
	return nil
}
