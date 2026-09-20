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

// Package preload helps manage a preload set onprem.
package preload

import (
	"context"
	"fmt"

	"intrinsic/assets/idutils"
	"intrinsic/config/datafiles"
	"intrinsic/kubernetes/acl/clientcontext"
	"intrinsic/storage/content_addressable_storage/pkg/idhandling"

	log "github.com/golang/glog"
	"go.opencensus.io/trace"
	"google.golang.org/grpc/status"

	apb "intrinsic/config/proto/application_go_proto"
	rtrpb "intrinsic/resources/proto/resource_type_runtime_go_proto"
	ocpsgrpcpb "intrinsic/storage/onprem_cas_preload/proto/v1/ocp_service_go_proto"
	ocpspb "intrinsic/storage/onprem_cas_preload/proto/v1/ocp_service_go_proto"
	pspb "intrinsic/storage/onprem_cas_preload/proto/v1/preload_set_go_proto"

	lrogrpcpb "cloud.google.com/go/longrunning/autogen/longrunningpb"
	lropb "cloud.google.com/go/longrunning/autogen/longrunningpb"
)

func waitOperation(ctx context.Context, lro lrogrpcpb.OperationsClient, op *lropb.Operation) (*ocpspb.UpsertPreloadSetResponse, error) {
	for !op.GetDone() {
		var err error
		op, err = lro.WaitOperation(ctx, &lropb.WaitOperationRequest{
			Name: op.GetName(),
		})
		if err != nil {
			return nil, fmt.Errorf("unable to check status of operation: %w", err)
		}
	}
	if err := status.ErrorProto(op.GetError()); err != nil {
		return nil, err
	}

	result := new(ocpspb.UpsertPreloadSetResponse)
	if err := op.GetResponse().UnmarshalTo(result); err != nil {
		return nil, fmt.Errorf("unable to check status of operation: %w", err)
	}
	return result, nil
}

func buildPreloadSet(app *apb.Application, rtrs []*rtrpb.ResourceTypeRuntime) (*pspb.PreloadSet, error) {
	fileRefs, err := datafiles.CollectFromApplication(app)
	if err != nil {
		return nil, fmt.Errorf("collecting data files failed: %w", err)
	}
	for _, rtr := range rtrs {
		if refs, err := datafiles.CollectFromRuntime(rtr); err != nil {
			return nil, fmt.Errorf("collecting data files from asset %q failed: %w", idutils.IDFromProtoUnchecked(rtr.GetMetadata().GetIdVersion().GetId()), err)
		} else {
			fileRefs = append(fileRefs, refs...)
		}
	}
	var validURIs []string
	for _, ref := range fileRefs {
		uri := ref.GetSpec().GetUri()
		// Skip everything that is not a valid CAS identifier.
		if _, err := idhandling.StripSchema(uri); err != nil {
			log.Warningf("Skipped invalid artifact: %v", err)
			continue
		}
		validURIs = append(validURIs, uri)
	}
	strategyByObject := make(map[string]*pspb.PreloadStrategy, len(validURIs))
	for _, uri := range validURIs {
		strategyByObject[uri] = &pspb.PreloadStrategy{
			PreloadDirective: pspb.PreloadDirective_DOWNLOAD_AND_KEEP,
		}
	}
	return &pspb.PreloadSet{
		// Name is currently output-only in OCPS.  There is a single
		// preload set and it is updated with each call.
		StrategyByObject: strategyByObject,
	}, nil
}

type client struct {
	ocps ocpsgrpcpb.OnpremCASPreloadServiceClient
	lro  lrogrpcpb.OperationsClient
}

func (c *client) Update(ctx context.Context, app *apb.Application, rtrs []*rtrpb.ResourceTypeRuntime) error {
	ctx, span := trace.StartSpan(ctx, "preload.Update")
	defer span.End()

	preloadSet, err := buildPreloadSet(app, rtrs)
	if err != nil {
		log.ErrorContextf(ctx, "Building preload set failed: %v", err)
		return fmt.Errorf("building preload set failed: %w", err)
	}
	span.AddAttributes(trace.Int64Attribute("requested_preload_num_files", int64(len(preloadSet.GetStrategyByObject()))))

	// Provide identity for data silos.
	ctxWithAuth, err := clientcontext.ToContextFromIncoming(ctx)
	if err != nil {
		log.ErrorContextf(ctx, "Adding identity information from incoming context to outgoing context failed: %v", err)
		return clientcontext.ErrGRPC(err)
	}
	log.InfoContextf(ctx, "Requesting preloaded of %d files", len(preloadSet.GetStrategyByObject()))
	op, err := c.ocps.UpsertPreloadSet(ctxWithAuth, &ocpspb.UpsertPreloadSetRequest{
		PreloadSet: preloadSet,
	})
	if err != nil {
		log.ErrorContextf(ctx, "Updating the preload set failed: %v", err)
		return fmt.Errorf("updating preload set failed: %w", err)
	}
	go func() {
		resp, err := waitOperation(ctx, c.lro, op)
		if err != nil {
			log.ErrorContextf(ctx, "Preload operation for the preload set failed: %v", err)
		}
		log.InfoContextf(ctx, "Successfully preloaded preload set %q", resp.GetPreloadSet().GetName())
	}()
	return nil
}

func (c *client) Clear(ctx context.Context) error {
	// Provide identity even though this won't trigger a download, as OCPS has
	// an option to enforce identity that could be enabled.
	ctxWithAuth, err := clientcontext.ToContextFromIncoming(ctx)
	if err != nil {
		log.ErrorContextf(ctx, "Adding identity information from incoming context to outgoing context failed: %v", err)
		return clientcontext.ErrGRPC(err)
	}
	// A clear could be the first call after the restart of a pod.  For
	// example, stopping a solution after being the IPC rebooted.  There is no
	// list method, so we go with an empty update.
	log.InfoContextf(ctx, "Clearing the preload set by requesting an empty update")
	op, err := c.ocps.UpsertPreloadSet(ctxWithAuth, &ocpspb.UpsertPreloadSetRequest{
		PreloadSet: &pspb.PreloadSet{
			// Name is currently output-only in OCPS.  There is a single
			// preload set and it is updated with each call.
			// Leave StrategyByObject unspecified.
		},
	})
	if err != nil {
		log.ErrorContextf(ctx, "Clearing the preload set failed: %v", err)
		return fmt.Errorf("clearing preload set failed: %w", err)
	}
	go func() {
		resp, err := waitOperation(ctx, c.lro, op)
		if err != nil {
			log.ErrorContextf(ctx, "Clear operation for the preload set failed: %v", err)
		}
		log.InfoContextf(ctx, "Successfully cleared preload set %q", resp.GetPreloadSet().GetName())
	}()
	return nil
}

// Client defines an interface to lazily manage a preload set onprem.
type Client interface {
	Update(context.Context, *apb.Application, []*rtrpb.ResourceTypeRuntime) error
	Clear(context.Context) error
}

// New creates a Client from the OCPS clients.
func New(ocps ocpsgrpcpb.OnpremCASPreloadServiceClient, lro lrogrpcpb.OperationsClient) Client {
	return &client{ocps, lro}
}

type noopClient struct{}

func (noopClient) Update(ctx context.Context, app *apb.Application, rtrs []*rtrpb.ResourceTypeRuntime) error {
	return nil
}

func (noopClient) Clear(ctx context.Context) error {
	return nil
}

// NewNoOp creates a preload Client that does nothing.
func NewNoOp() Client {
	return noopClient{}
}
