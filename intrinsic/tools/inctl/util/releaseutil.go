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

// Package releaseutil provides utilities to release from internal.
package releaseutil

import (
	"context"
	"fmt"
	"strings"

	"intrinsic/storage/content_addressable_storage/pkg/crossproject"
	"intrinsic/storage/content_addressable_storage/pkg/filetocas"
	"intrinsic/tools/inctl/util/color"

	log "github.com/golang/glog"
	"go.opencensus.io/trace"
	"golang.org/x/sync/errgroup"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"

	apipb "intrinsic/kubernetes/workcell_spec/proto/transfer_go_proto"
	casgrpcpb "intrinsic/storage/content_addressable_storage/proto/cas_service_go_proto"
	caspb "intrinsic/storage/content_addressable_storage/proto/cas_service_go_proto"
)

const (
	intrinsicCASPrefix = "intcas://"
	// CASUploadChunkSize is 2 MiB because we know that the data files can be large and it's more
	// efficient to use larger chunks. Default gRPC message limit is 4 MB, so we're well below it.
	CASUploadChunkSize = 2 * 1024 * 1024
)

func isAlreadyInCAS(ctx context.Context, casClient casgrpcpb.ContentAddressableStorageServiceClient, objectID string) (string, error) {
	_, err := casClient.Stat(ctx, &caspb.StatRequest{ObjectId: objectID})
	if c := status.Code(err); c == codes.OK {
		return objectID, nil
	} else if c == codes.NotFound {
		return "", nil
	}
	return "", fmt.Errorf("failed to stat object %q: %w", objectID, err)
}

// moveFileToCAS takes a pointer to a full FileReference and makes sure that the file is accessible
// on the project's content-addressable storage (CAS). It modifies the fileRef.spec.uri to point to
// the CAS object.
func moveFileToCAS(ctx context.Context, f2c *filetocas.Uploader, srcCASClient, dstCASClient casgrpcpb.ContentAddressableStorageServiceClient, fileRef *apipb.FileReference) error {
	ctx, span := trace.StartSpan(ctx, "releaseutil.moveFileToCAS")
	defer span.End()
	span.AddAttributes(
		trace.StringAttribute("name", fileRef.GetMetadata().GetName()),
		trace.StringAttribute("path", fileRef.GetSpec().GetPath()),
		trace.StringAttribute("uri", fileRef.GetSpec().GetUri()))

	sourceURI := fileRef.GetSpec().GetUri()
	log.V(1).InfoContextf(ctx, "Checking if %q is already in CAS", sourceURI)

	if strings.HasPrefix(sourceURI, intrinsicCASPrefix) {
		existingObjectID, err := isAlreadyInCAS(ctx, dstCASClient, sourceURI)
		if err != nil {
			return fmt.Errorf("failed to check if %q is already in CAS: %w", sourceURI, err)
		}
		if existingObjectID != "" {
			log.V(1).InfoContextf(ctx, "File %q is already in CAS, skipping upload.", sourceURI)
			fileRef.GetSpec().Uri = existingObjectID
			return nil
		}

		if err := crossproject.Copy(ctx, srcCASClient, dstCASClient, sourceURI); err != nil {
			return fmt.Errorf("failed to copy %q from source CAS to destination CAS: %w", sourceURI, err)
		}
		log.V(1).InfoContextf(ctx, "Copied %q from source CAS to destination CAS", sourceURI)
		return nil
	}

	objectID, _, err := f2c.Upload(ctx, sourceURI)
	if err != nil {
		return fmt.Errorf("failed to upload to CAS: %w", err)
	}

	log.InfoContextf(ctx, "Moved from %q to %q", fileRef.GetSpec().Uri, objectID)
	fileRef.GetSpec().Uri = objectID
	return nil
}

// MoveFilesToCAS uploads multiple [apipb.FileReference]s in parallel. It modifies each of the
// fileRef.spec.uri to point to the uploaded CAS object.
func MoveFilesToCAS(ctx context.Context, f2c *filetocas.Uploader, srcCASClient, dstCASClient casgrpcpb.ContentAddressableStorageServiceClient, fileRefs []*apipb.FileReference) error {
	ctx, span := trace.StartSpan(ctx, "releaseutil.MoveFilesToCAS")
	defer span.End()

	// This early return is just to avoid printing loud messages to the console every time.
	if len(fileRefs) == 0 {
		log.InfoContextf(ctx, "No file references to upload, skipping.")
		return nil
	}

	// Use Printf() so that it's always visible in the terminal!
	color.C.Blue().Printf("Uploading %d file references in parallel.\n", len(fileRefs))
	var g errgroup.Group
	for _, r := range fileRefs {
		r := r
		g.Go(func() error {
			if err := moveFileToCAS(ctx, f2c, srcCASClient, dstCASClient, r); err != nil {
				return fmt.Errorf("moving file %+v to CAS: %w", r, err)
			}
			return nil
		})
	}
	if err := g.Wait(); err != nil {
		return err
	}
	color.C.Blue().Printf("Upload finished!\n")
	return nil
}
