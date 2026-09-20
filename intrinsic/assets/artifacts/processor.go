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

// Package processor provides core logic for artifact processing.
package processor

import (
	"context"
	"crypto/tls"
	"errors"
	"fmt"
	"io"
	"path"
	"strings"
	"time"

	"intrinsic/assets/baseclientutils"
	"intrinsic/assets/referenceddata"
	"intrinsic/longrunning/go/operations"
	"intrinsic/storage/content_addressable_storage/pkg/crossproject"
	casobservability "intrinsic/storage/content_addressable_storage/pkg/observability"

	log "github.com/golang/glog"
	"github.com/pborman/uuid"
	"go.opencensus.io/trace"
	"google.golang.org/grpc"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/credentials"
	"google.golang.org/grpc/status"
	"google.golang.org/protobuf/proto"

	caspb "intrinsic/storage/content_addressable_storage/proto/cas_service_go_proto"

	lropb "cloud.google.com/go/longrunning/autogen/longrunningpb"
)

const (
	operationsPrefix = "operations/artifacts-v1/"
)

// CASClientFactory is a function interface that returns a CAS client for the specified GCP project
// and a closer to release resources when done.
type CASClientFactory func(context.Context, string) (caspb.ContentAddressableStorageServiceClient, io.Closer, error)

// DefaultCASClientFactory is a CASClientFactory that uses standard dial options, TLS, and
// casobservability interceptors.
func DefaultCASClientFactory(ctx context.Context, project string) (caspb.ContentAddressableStorageServiceClient, io.Closer, error) {
	unary, stream := casobservability.GRPCClientInterceptors("asset-artifacts")
	opts := append(baseclientutils.BaseDialOptions(),
		grpc.WithTransportCredentials(credentials.NewTLS(new(tls.Config))),
		grpc.WithUnaryInterceptor(unary),
		grpc.WithStreamInterceptor(stream),
	)

	domain := func() string {
		switch project {
		case "intrinsic-assets-dev":
			return "assets-dev.intrinsic.ai"
		case "intrinsic-assets-staging":
			return "assets-qa.intrinsic.ai"
		case "intrinsic-assets-prod":
			return "assets.intrinsic.ai"
		default:
			return fmt.Sprintf("www.endpoints.%s.cloud.goog", project)
		}
	}()

	conn, err := grpc.NewClient(fmt.Sprintf("%s:443", domain), opts...)
	if err != nil {
		return nil, nil, fmt.Errorf("cannot connect to CAS in project %q: %v", project, err)
	}
	return caspb.NewContentAddressableStorageServiceClient(conn), conn, nil
}

// MakeResponse creates the response proto that the Processor sets on its operation once finished.
type MakeResponse func(*referenceddata.ReferencedData) proto.Message

// StartRunner starts a background LRO operations runner with a queue and map.
//
// The returned cleanup function stops the worker queue.
func StartRunner(queueSize, workers int) (*operations.Runner, *operations.Map, func()) {
	opMap := operations.NewMap()
	opQueue := operations.NewQueue(queueSize)
	opQueue.Start(workers)
	return operations.NewRunner(opMap, opQueue), opMap, opQueue.Stop
}

// Config configures the [Processor].
type Config struct {
	// CASClient is a client for this project's CAS service.
	CASClient caspb.ContentAddressableStorageServiceClient
	// CASClientFactory returns clients for other projects' CAS services.
	CASClientFactory CASClientFactory
	// IgnoreMissingCASReferences specifies whether to ignore missing CAS data referenced by artifacts
	// during cross-project copy.
	IgnoreMissingCASReferences bool
	// Runner runs the service's LROs.
	Runner *operations.Runner
}

// Processor processes artifacts for inclusion in an Asset.
type Processor struct {
	cfg *Config
}

// New creates a new [Processor].
func New(cfg *Config) (*Processor, error) {
	if cfg.CASClient == nil {
		return nil, fmt.Errorf("CASClient is required")
	}
	if cfg.CASClientFactory == nil {
		return nil, fmt.Errorf("CASClientFactory is required")
	}
	if cfg.Runner == nil {
		return nil, fmt.Errorf("Runner is required")
	}
	return &Processor{
		cfg: cfg,
	}, nil
}

// Process processes artifacts for inclusion in an Asset.
//
// It processes the specified reference (which must be either inlined or a CAS reference),
// and returns an operation. When the operation is finished, its response is set using the provided
// MakeResponse function.
func (p *Processor) Process(ctx context.Context, ref *referenceddata.ReferencedData, makeResponse MakeResponse) (*operations.Operation, error) {
	// Ensure that the digest for CAS references is present and valid.
	if ref.Type() == referenceddata.CASReferenceType {
		refHash := ref.Reference()[9:]
		if ref.Digest() != "" || len(refHash) == 128 {
			if err := setDigest(ref, "sha512:", refHash); err != nil {
				return nil, err
			}
		}
	}

	op := operations.New(&lropb.Operation{
		Name: path.Join(operationsPrefix, "process", uuid.New()),
	})

	var finished bool
	switch rt := ref.Type(); rt {
	case referenceddata.CASReferenceType:
		var err error
		if finished, err = p.processCASReference(ctx, ref, op, makeResponse); err != nil {
			return nil, err
		}
	case referenceddata.InlinedReferenceType:
		// Inlined data must have been populated.
		if err := p.processInlinedData(ref); err != nil {
			return nil, err
		}
		finished = true
	case referenceddata.FileReferenceType:
		return nil, status.Errorf(codes.InvalidArgument, "file references are not supported; please upload the file first")
	default:
		return nil, status.Errorf(codes.InvalidArgument, "unknown reference type: %v", rt)
	}

	// If the operation is finished, set the response. Otherwise, the runner will set the response
	// later.
	if finished {
		if err := op.SetResponse(makeResponse(ref)); err != nil {
			return nil, wrapStatusErrorf(err, codes.Internal, "failed to set operation response")
		}
	}

	return op, nil
}

func (p *Processor) processCASReference(ctx context.Context, ref *referenceddata.ReferencedData, op *operations.Operation, makeResponse MakeResponse) (bool, error) {
	casURI := ref.Reference()
	_, err := p.cfg.CASClient.Stat(ctx, &caspb.StatRequest{ObjectId: casURI})
	code := status.Code(err)

	// CAS object already exists in the target project.
	if code == codes.OK {
		ref.ClearSourceProject()
		return true, nil
	}

	// Return early for anything other than a missing CAS object in the target project.
	if code != codes.NotFound {
		return false, wrapStatusErrorf(err, codes.Internal, "failed to stat CAS object %q", casURI)
	}

	sourceProject := ref.SourceProject()
	if sourceProject == "" {
		return false, status.Errorf(codes.InvalidArgument, "cannot copy CAS object %q without a source project", casURI)
	}

	// Create a clean background context with a 1-hour timeout.
	// We explicitly derive it from context.Background() rather than the incoming request context to
	// strip user credentials. The cross-project copy operation runs asynchronously and should use
	// the server's own service account credentials to access GCS/CAS, rather than trying to delegate
	// the user's incoming authentication tokens (which might not have permissions on the other
	// project, or might not be delegatable). We only preserve the trace context to link the
	// background operation's spans.
	bgCtx := trace.NewContext(context.Background(), trace.FromContext(ctx))

	// Transfer CAS object to the target project.
	// The runner will set the operation response later. We just need to return the response.
	if err := p.cfg.Runner.Schedule(bgCtx, op, func(opCtx context.Context) (proto.Message, error) {
		opCtx, cancel := context.WithTimeout(opCtx, 1*time.Hour)
		defer cancel()

		srcClient, closer, err := p.cfg.CASClientFactory(opCtx, sourceProject)
		if err != nil {
			return nil, wrapStatusErrorf(err, codes.Internal, "failed to create source CAS client")
		}
		if closer != nil {
			defer closer.Close()
		}
		if err := crossproject.Copy(opCtx, srcClient, p.cfg.CASClient, casURI); err != nil {
			if p.cfg.IgnoreMissingCASReferences && status.Code(err) == codes.NotFound {
				log.WarningContextf(opCtx, "failed to copy CAS object %q from source project %q: %v", casURI, sourceProject, err)
			} else {
				return nil, wrapStatusErrorf(err, codes.Internal, "failed to copy CAS object %q from project %q", casURI, sourceProject)
			}
		}

		ref.ClearSourceProject()

		return makeResponse(ref), nil
	}); err != nil {
		if errors.Is(err, operations.ErrQueueFull) {
			return false, status.Errorf(codes.ResourceExhausted, "queue is full: %v", err)
		}
		return false, wrapStatusErrorf(err, codes.Internal, "failed to schedule copy")
	}

	return false, nil
}

func (p *Processor) processInlinedData(ref *referenceddata.ReferencedData) error {
	// Do not set digest for inlined references to maintain backward compatibility with old platforms.
	ref.SetDigest("")

	return nil
}

func setDigest(ref *referenceddata.ReferencedData, prefix string, hash string) error {
	digest := prefix + hash
	if strings.HasPrefix(ref.Digest(), prefix) && digest != ref.Digest() {
		return status.Errorf(codes.InvalidArgument, "digest mismatch: client provided %q, computed %q", ref.Digest(), digest)
	}
	if ref.Type() == referenceddata.CASReferenceType {
		refHash := ref.Reference()[9:]
		if len(refHash) == 128 && strings.HasPrefix(digest, "sha512:") && hash != refHash {
			return status.Errorf(codes.InvalidArgument, "digest mismatch: CAS reference has %q, digest has %q", refHash, hash)
		}
	}

	ref.SetDigest(digest)

	return nil
}

func wrapStatusErrorf(err error, defaultCode codes.Code, format string, args ...any) error {
	if err == nil {
		return nil
	}

	st := status.Convert(err)
	code := st.Code()
	if code == codes.Unknown {
		code = defaultCode
	}

	return status.Errorf(code, "%s: %s", fmt.Sprintf(format, args...), st.Message())
}
