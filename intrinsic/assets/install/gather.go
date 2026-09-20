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

// Package gather provides functions to gather Assets from various sources.
package gather

import (
	"context"
	"fmt"
	"maps"
	"math"
	"slices"
	"strings"

	"intrinsic/assets/idutils"
	"intrinsic/assets/install/localconv"
	"intrinsic/assets/typeutils"
	"intrinsic/assets/viewutils"
	"intrinsic/kubernetes/acl/clientcontext"
	"intrinsic/util/go/xiter"

	log "github.com/golang/glog"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"

	acpb "intrinsic/assets/catalog/proto/v1/asset_catalog_go_proto"
	acipb "intrinsic/assets/catalog/proto/v1/asset_catalog_internal_go_proto"
	runtime "intrinsic/assets/conversion/runtime"
	atpb "intrinsic/assets/proto/asset_type_go_proto"
	idpb "intrinsic/assets/proto/id_go_proto"
	iopb "intrinsic/assets/proto/installation_origin_go_proto"
	vpb "intrinsic/assets/proto/view_go_proto"
	apppb "intrinsic/config/proto/application_go_proto"
	rcigrpcpb "intrinsic/resources/catalog/proto/resource_catalog_internal_go_proto"
	rcipb "intrinsic/resources/catalog/proto/resource_catalog_internal_go_proto"
	rtpb "intrinsic/resources/proto/resource_type_go_proto"
	rtrpb "intrinsic/resources/proto/resource_type_runtime_go_proto"
	svsgrpcpb "intrinsic/solution_versions/proto/v1/solution_version_service_go_proto"
)

const (
	// The request limit was previously at the default of 4MB, so this was used
	// to modulate the request size below that.  Errors would occur somewhere
	// between 16 to 33 skills, but asset size isn't uniform.  However, we've
	// seen that other services, such as the solution version service are able
	// to handle larger response sizes.  As a result, we've removed the 10
	// asset limit.  The chunking code is left as is until we see no further
	// problems with requests.
	// TODO: b/455587771: Remove chunking once having no max shows no issues.
	catalogChunkSize = math.MaxInt
)

var (
	metadataOnlyViews = map[vpb.AssetViewType]struct{}{
		vpb.AssetViewType_ASSET_VIEW_TYPE_BASIC:        {},
		vpb.AssetViewType_ASSET_VIEW_TYPE_DETAIL:       {},
		vpb.AssetViewType_ASSET_VIEW_TYPE_VERSIONS:     {},
		vpb.AssetViewType_ASSET_VIEW_TYPE_ALL_METADATA: {},
	}
)

// Gatherer is a function interface for gathering Assets from some source.
//
// It returns a list of the gathered Assets and a map from unavailable IDVersions to the reason they
// are unavailable (if SkipUnavailable is true).
type Gatherer = func(context.Context, []*idpb.IdVersion) ([]*rtrpb.ResourceTypeRuntime, map[string]string, error)

type fromAssetCatalogOptions struct {
	allowedAssetTypes []atpb.AssetType
	skipUnavailable   bool
	view              vpb.AssetViewType
}

// FromAssetCatalogOption is an option for FromAssetCatalog.
type FromAssetCatalogOption func(*fromAssetCatalogOptions)

// WithAllowedAssetTypes is an option for FromAssetCatalog that specifies the allowed asset types to
// gather.
func WithAllowedAssetTypes(ats []atpb.AssetType) FromAssetCatalogOption {
	return func(opts *fromAssetCatalogOptions) {
		opts.allowedAssetTypes = ats
	}
}

// WithSkipUnavailable is an option for FromAssetCatalog that specifies whether to skip unavailable
// assets.
//
// If false, an error is returned if any of the assets are not found.
// If true, unavailable assets are skipped.
func WithSkipUnavailable(skipUnavailable bool) FromAssetCatalogOption {
	return func(opts *fromAssetCatalogOptions) {
		opts.skipUnavailable = skipUnavailable
	}
}

// WithView specifies the Asset view to return.
func WithView(view vpb.AssetViewType) FromAssetCatalogOption {
	return func(opts *fromAssetCatalogOptions) {
		opts.view = view
	}
}

func fromAssetCatalog(ctx context.Context, c acipb.AssetCatalogInternalClient, assets []*idpb.IdVersion, opts *fromAssetCatalogOptions) ([]*rtrpb.ResourceTypeRuntime, map[string]string, error) {
	resp, err := c.BatchGetAssetsInternal(ctx, &acipb.BatchGetAssetsRequest{
		IdVersions:      assets,
		SkipUnavailable: opts.skipUnavailable,
		View:            opts.view,
	})
	if err != nil {
		log.ErrorContextf(ctx, "BatchGetAssetsInternal(%v) failed: %v", assets, err)
		return nil, nil, fmt.Errorf("unable to get asset from catalog: %w", err)
	}

	var rtrs []*rtrpb.ResourceTypeRuntime
	unavailable := make(map[string]string)
	for _, a := range resp.GetAssets() {
		idv := idutils.IDVersionFromProtoUnchecked(a.GetMetadata().GetIdVersion())

		if len(opts.allowedAssetTypes) > 0 && !slices.Contains(opts.allowedAssetTypes, a.GetMetadata().GetAssetType()) {
			var allowedTypes []string
			for _, t := range opts.allowedAssetTypes {
				allowedTypes = append(allowedTypes, t.String())
			}
			// We treat this as NotFound, because the user was requesting a (type, IDVersion) pair that
			// doesn't exist.
			err := status.Errorf(codes.NotFound, "%q is of disallowed type %v (allowed types: %v)", idv, a.GetMetadata().GetAssetType().String(), allowedTypes)
			if opts.skipUnavailable {
				unavailable[idv] = err.Error()
				continue
			}
			return nil, nil, err
		}

		rtr, err := catalogAssetToRuntimeView(ctx, c, a, opts.view)
		if err != nil {
			log.ErrorContextf(ctx, "catalogAssetToRuntimeView failed for %q from catalog: %v", idv, err)
			return nil, nil, err
		}
		rtrs = append(rtrs, rtr)
	}

	for _, un := range resp.GetUnavailableAssets() {
		unavailable[idutils.IDVersionFromProtoUnchecked(un.GetIdVersion())] = un.GetReason()
	}

	return rtrs, unavailable, nil
}

// FromAssetCatalog returns a gatherer that can be used to get Assets from the catalog.
func FromAssetCatalog(c acipb.AssetCatalogInternalClient, options ...FromAssetCatalogOption) Gatherer {
	opts := &fromAssetCatalogOptions{
		allowedAssetTypes: typeutils.AllAssetTypes(),
	}
	for _, opt := range options {
		opt(opts)
	}
	if opts.view == vpb.AssetViewType_ASSET_VIEW_TYPE_UNSPECIFIED {
		opts.view = vpb.AssetViewType_ASSET_VIEW_TYPE_ALL
	}

	return func(ctx context.Context, assets []*idpb.IdVersion) ([]*rtrpb.ResourceTypeRuntime, map[string]string, error) {
		// Forward the incoming identity so that we authenticate with the catalog.
		ctx, err := clientcontext.ToContextFromIncoming(ctx)
		if err != nil {
			return nil, nil, clientcontext.ErrGRPC(err)
		}

		var result []*rtrpb.ResourceTypeRuntime
		unavailable := make(map[string]string)
		for chunk := range slices.Chunk(assets, catalogChunkSize) {
			subset, unavailableSubset, err := fromAssetCatalog(ctx, c, chunk, opts)
			if err != nil {
				return nil, nil, err
			}
			result = append(result, subset...)
			for idv, reason := range unavailableSubset {
				unavailable[idv] = reason
			}
		}
		return result, unavailable, nil
	}
}

type fromSolutionOptions struct {
	aci              acipb.AssetCatalogInternalClient
	branchID         string
	returnAllIfEmpty bool
	svs              svsgrpcpb.SolutionVersionServiceClient
	view             vpb.AssetViewType
}

// FromSolutionOption is an option for FromSolution.
type FromSolutionOption func(*fromSolutionOptions)

// WithSolutionACIClient specifies the AssetCatalogInternal client to use.
func WithSolutionACIClient(aci acipb.AssetCatalogInternalClient) FromSolutionOption {
	return func(opts *fromSolutionOptions) {
		opts.aci = aci
	}
}

// WithSolutionBranchID specifies the branch ID of the target Solution.
func WithSolutionBranchID(branchID string) FromSolutionOption {
	return func(opts *fromSolutionOptions) {
		opts.branchID = branchID
	}
}

// WithSolutionReturnAllIfEmpty specifies whether the returned gatherer should interpret an empty
// input list of IDVersions as a signal to return -all- Assets in the Solution.
//
// This option can be used to list Assets in the specified Solution.
func WithSolutionReturnAllIfEmpty(returnAllIfEmpty bool) FromSolutionOption {
	return func(opts *fromSolutionOptions) {
		opts.returnAllIfEmpty = returnAllIfEmpty
	}
}

// WithSolutionSVSClient specifies the SolutionVersionService client to use.
func WithSolutionSVSClient(svs svsgrpcpb.SolutionVersionServiceClient) FromSolutionOption {
	return func(opts *fromSolutionOptions) {
		opts.svs = svs
	}
}

// WithSolutionView specifies the Asset view to return.
func WithSolutionView(view vpb.AssetViewType) FromSolutionOption {
	return func(opts *fromSolutionOptions) {
		opts.view = view
	}
}

// FromSolution returns a gatherer that can be used to get Assets from a Solution.
//
// The IDVersions passed to the gatherer can optionally omit the version. If omitted, the Asset with
// the matching ID portion is returned. If included and the version does not match the version of
// the Asset in the Solution, an error is returned.
func FromSolution(options ...FromSolutionOption) Gatherer {
	opts := &fromSolutionOptions{}
	for _, opt := range options {
		opt(opts)
	}
	if opts.view == vpb.AssetViewType_ASSET_VIEW_TYPE_UNSPECIFIED {
		opts.view = vpb.AssetViewType_ASSET_VIEW_TYPE_ALL
	}

	return func(ctx context.Context, assets []*idpb.IdVersion) ([]*rtrpb.ResourceTypeRuntime, map[string]string, error) {
		// Forward the incoming identity so that we authenticate with the cloud services.
		ctx, err := clientcontext.ToContextFromIncoming(ctx)
		if err != nil {
			return nil, nil, clientcontext.ErrGRPC(err)
		}

		// First gather all Assets.
		rtrMap, err := allFromSolution(ctx, opts)
		if err != nil {
			return nil, nil, err
		}

		// Return only what was requested.
		if opts.returnAllIfEmpty && len(assets) == 0 {
			return slices.Collect(maps.Values(rtrMap)), nil, nil
		}
		rtrs := make([]*rtrpb.ResourceTypeRuntime, len(assets))
		for i, idVersionProto := range assets {
			key := idutils.IDFromProtoUnchecked(idVersionProto.GetId())
			rtr, ok := rtrMap[key]
			if !ok {
				return nil, nil, status.Errorf(codes.NotFound, "Asset %q not found in Solution %q", key, opts.branchID)
			}
			if idVersionProto.GetVersion() != "" && rtr.GetMetadata().GetIdVersion().GetVersion() != idVersionProto.GetVersion() {
				idvWant := idutils.IDVersionFromProtoUnchecked(idVersionProto)
				idvGot := idutils.IDVersionFromProtoUnchecked(rtr.GetMetadata().GetIdVersion())
				return nil, nil, status.Errorf(codes.NotFound, "got unexpected Asset version in Solution %q (want: %s, got %s)", opts.branchID, idvWant, idvGot)
			}
			rtrs[i] = rtr
		}

		return rtrs, nil, nil
	}
}

func allFromSolution(ctx context.Context, opts *fromSolutionOptions) (map[string]*rtrpb.ResourceTypeRuntime, error) {
	dd, err := opts.svs.GetDeploymentData(ctx, &svsgrpcpb.GetDeploymentDataRequest{
		BranchId: opts.branchID,
	})
	if err != nil {
		return nil, err
	}

	rtrs := map[string]*rtrpb.ResourceTypeRuntime{}

	// Gather Assets from the Application proto.
	var catalogKeys []string
	var catalogIDVersions []*idpb.IdVersion
	for key, asset := range dd.GetModifiedSolution().GetApplication().GetAssets() {
		switch v := asset.GetVariant().(type) {
		case *apppb.Application_Asset_Catalog:
			// We'll get these from the catalog below.
			catalogKeys = append(catalogKeys, key)
			catalogIDVersions = append(catalogIDVersions, v.Catalog)
		default:
			rtr, err := localconv.LocalSolutionAssetToRuntime(ctx, asset,
				runtime.WithSkipValidation(),
				runtime.WithACIClient(opts.aci),
			)
			if err != nil {
				return nil, err
			}
			rtr, err = runtimeToView(rtr, opts.view)
			if err != nil {
				return nil, err
			}
			rtrs[key] = rtr
		}
	}

	// Add sideloaded ResourceTypeRuntimes.
	for _, rtr := range dd.GetModifiedSolution().GetSideloadedResourceTypes() {
		id := idutils.IDFromProtoUnchecked(rtr.GetMetadata().GetIdVersion().GetId())
		if _, exists := rtrs[id]; exists {
			return nil, status.Errorf(codes.Internal, "multiple Assets found for ID %q", id)
		}
		rtr, err = runtimeToView(rtr, opts.view)
		if err != nil {
			return nil, err
		}
		rtrs[id] = rtr
	}

	// Gather the catalog Assets.
	catalogGatherer := FromAssetCatalog(opts.aci,
		WithSkipUnavailable(false),
		WithView(opts.view),
	)
	catalogRTRs, _, err := catalogGatherer(ctx, catalogIDVersions)
	if err != nil {
		return nil, err
	}
	for _, rtr := range catalogRTRs {
		id := idutils.IDFromProtoUnchecked(rtr.GetMetadata().GetIdVersion().GetId())
		if _, exists := rtrs[id]; exists {
			return nil, status.Errorf(codes.Internal, "multiple Assets found for ID %q", id)
		}
		rtrs[id] = rtr
	}

	return rtrs, nil
}

// fromResourceCatalogOptions is an options struct for fromResourceCatalog.
type fromResourceCatalogOptions struct {
	skipUnavailable bool
}

// FromResourceCatalogOption is an option for FromResourceCatalog.
type FromResourceCatalogOption func(*fromResourceCatalogOptions)

// WithSkipUnavailableResources is an option for FromResourceCatalog that specifies whether to skip
// unavailable Resources.
//
// If false, an error is returned if any of the Resources are not found.
// If true, unavailable Resources are skipped.
func WithSkipUnavailableResources(skipUnavailable bool) FromResourceCatalogOption {
	return func(opts *fromResourceCatalogOptions) {
		opts.skipUnavailable = skipUnavailable
	}
}

func fromResourceCatalog(ctx context.Context, c rcigrpcpb.ResourceCatalogInternalClient, assets []*idpb.IdVersion) ([]*rtrpb.ResourceTypeRuntime, error) {
	resp, err := c.ListInternalResourceTypes(ctx, &rcipb.ListInternalResourceTypesRequest{
		IdVersions: slices.Collect(xiter.Map(idutils.IDVersionFromProtoUnchecked, slices.Values(assets))),
	})
	if err != nil {
		log.ErrorContextf(ctx, "s.rciClient.ListInternalResourceTypes(%v) failed: %v", assets, err)
		return nil, fmt.Errorf("unable to get resource from catalog: %w", err)
	}

	toCatalogRuntime := func(rt *rtpb.ResourceType) *rtrpb.ResourceTypeRuntime {
		return localconv.CatalogResourceToRuntime(rt)
	}
	return slices.Collect(xiter.Map(toCatalogRuntime, slices.Values(resp.GetResourceTypes()))), nil
}

// FromResourceCatalog gets runtime protos for the requested assets from the ResourceCatalog.
//
// The error returned is a gRPC status.
// This function handles forwarding user identity information to the catalog, so that should not be
// done for the context into this function.
func FromResourceCatalog(c rcigrpcpb.ResourceCatalogInternalClient, options ...FromResourceCatalogOption) Gatherer {
	opts := &fromResourceCatalogOptions{}
	for _, opt := range options {
		opt(opts)
	}

	return func(ctx context.Context, assets []*idpb.IdVersion) ([]*rtrpb.ResourceTypeRuntime, map[string]string, error) {
		// Forward the incoming identity so that we authenticate with the catalog.
		ctx, err := clientcontext.ToContextFromIncoming(ctx)
		if err != nil {
			return nil, nil, clientcontext.ErrGRPC(err)
		}

		var result []*rtrpb.ResourceTypeRuntime
		unavailable := make(map[string]string)
		for chunk := range slices.Chunk(assets, catalogChunkSize) {
			if subset, err := fromResourceCatalog(ctx, c, chunk); err == nil {
				result = append(result, subset...)
			} else if opts.skipUnavailable && isNotFoundError(err) {
				// Try again one Resource at a time so we can return the ones that were found.
				for _, idv := range chunk {
					if subsubset, err := fromResourceCatalog(ctx, c, []*idpb.IdVersion{idv}); err == nil {
						result = append(result, subsubset...)
					} else if isNotFoundError(err) {
						unavailable[idutils.IDVersionFromProtoUnchecked(idv)] = err.Error()
					} else {
						return nil, nil, err
					}
				}
			} else {
				return nil, nil, err
			}
		}
		return result, unavailable, nil
	}
}

// FromCatalogs returns a gatherer that attempts to gather from both the AssetCatalog and the
// ResourceCatalog.
//
// It first tries to find each item in the AssetCatalog (with skip_unavailable set to true). For
// missing Assets, it falls back to the ResourceCatalog. If not found there either and the caller
// does not want to skip unavailable assets, it tries the AssetCatalog once again with
// skip_unavailable set to false so that the proper error will be returned.
func FromCatalogs(ac acipb.AssetCatalogInternalClient, rc rcigrpcpb.ResourceCatalogInternalClient, options ...FromAssetCatalogOption) Gatherer {
	// Parse the options now, so we can tell whether or not we are ultimately going to skip
	// unavailable assets.
	opts := &fromAssetCatalogOptions{}
	for _, opt := range options {
		opt(opts)
	}

	// We always skip unavailable assets in the first two gatherers.
	var addedAssetCatalogOptions []FromAssetCatalogOption
	if !opts.skipUnavailable {
		addedAssetCatalogOptions = append(addedAssetCatalogOptions, WithSkipUnavailable(true))
	}
	gatherers := []Gatherer{
		FromAssetCatalog(ac, append(options, addedAssetCatalogOptions...)...),
		FromResourceCatalog(rc, WithSkipUnavailableResources(true)),
	}

	// If we're not skipping unavailable assets, we need a third gatherer that doesn't skip
	// unavailable assets.
	if !opts.skipUnavailable {
		gatherers = append(gatherers, FromAssetCatalog(ac, options...))
	}

	return func(ctx context.Context, assets []*idpb.IdVersion) ([]*rtrpb.ResourceTypeRuntime, map[string]string, error) {
		remaining := make(map[string]*idpb.IdVersion)
		for _, asset := range assets {
			remaining[idutils.IDVersionFromProtoUnchecked(asset)] = asset
		}

		var rtrs []*rtrpb.ResourceTypeRuntime
		unavailable := make(map[string]string)
		for _, gatherer := range gatherers {
			if len(remaining) == 0 {
				break
			}
			gathered, gatheredUnavailable, err := gatherer(ctx, slices.Collect(maps.Values(remaining)))
			if err != nil {
				return nil, nil, err
			}
			rtrs = append(rtrs, gathered...)
			for idv, reason := range gatheredUnavailable {
				unavailable[idv] = reason
			}
			for _, rtr := range gathered {
				delete(remaining, idutils.IDVersionFromProtoUnchecked(rtr.GetMetadata().GetIdVersion()))
			}
		}

		return rtrs, unavailable, nil
	}
}

func isNotFoundError(err error) bool {
	return status.Code(err) == codes.NotFound || strings.Contains(err.Error(), "not found")
}

func catalogAssetToRuntimeView(ctx context.Context, aci acipb.AssetCatalogInternalClient, a *acpb.Asset, view vpb.AssetViewType) (*rtrpb.ResourceTypeRuntime, error) {
	if _, exists := metadataOnlyViews[view]; exists {
		return &rtrpb.ResourceTypeRuntime{
			InstallationOrigin: iopb.InstallationOrigin_INSTALLATION_ORIGIN_CATALOG,
			Metadata:           a.GetMetadata(),
		}, nil
	}
	if view != vpb.AssetViewType_ASSET_VIEW_TYPE_ALL {
		return nil, status.Errorf(codes.Unimplemented, "Support for view not implemented: %v", view)
	}

	return localconv.CatalogAssetToRuntime(ctx, a, runtime.WithACIClient(aci))
}

func runtimeToView(rtr *rtrpb.ResourceTypeRuntime, view vpb.AssetViewType) (*rtrpb.ResourceTypeRuntime, error) {
	if _, exists := metadataOnlyViews[view]; exists {
		metadata, err := viewutils.MetadataToView(rtr.GetMetadata(), view)
		if err != nil {
			return nil, err
		}
		return &rtrpb.ResourceTypeRuntime{
			InstallationOrigin: rtr.GetInstallationOrigin(),
			Metadata:           metadata,
		}, nil
	}
	if view != vpb.AssetViewType_ASSET_VIEW_TYPE_ALL {
		return nil, status.Errorf(codes.Unimplemented, "Support for view not implemented: %v", view)
	}

	return rtr, nil
}
