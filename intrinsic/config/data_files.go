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

// Package datafiles provides functions operating on data files referenced
// within an application proto.
package datafiles

import (
	"errors"
	"fmt"
	"strings"

	datautils "intrinsic/assets/data/utils"
	"intrinsic/assets/referenceddata"

	log "github.com/golang/glog"

	dataassetpb "intrinsic/assets/data/proto/v1/data_asset_go_proto"
	applicationpb "intrinsic/config/proto/application_go_proto"
	transferpb "intrinsic/kubernetes/workcell_spec/proto/transfer_go_proto"
	rtrpb "intrinsic/resources/proto/resource_type_runtime_go_proto"
	sopb "intrinsic/scene/proto/v1/scene_object_go_proto"
)

// ErrDuplicatePaths is a sentinel error. It signalizes that a collection contains multiple file
// references with the same path specification.
var ErrDuplicatePaths = errors.New("duplicate file reference paths")

func collectDataAsset(data *dataassetpb.DataAsset) ([]*transferpb.FileReference, error) {
	var fileRefs []*transferpb.FileReference
	payload, err := datautils.ExtractPayload(data)
	if err != nil {
		return nil, fmt.Errorf("failed to extract payload: %w", err)
	}
	if _, err := referenceddata.WalkUnique(payload, func(ref *referenceddata.ReferencedData) error {
		if ref.Type() == referenceddata.CASReferenceType {
			var frDigest string
			if ref.Digest() != "" {
				digest, err := datautils.ParseDigest(ref.Digest())
				if err != nil {
					return fmt.Errorf("failed to parse digest %q: %w", ref.Digest(), err)
				}
				if digest.Algorithm == datautils.HighwayHash128 {
					frDigest = digest.Hash
				} else {
					log.Warningf("Ignoring non-HighwayHash128 digest %q", ref.Digest())
				}
			}

			fileRefs = append(fileRefs, &transferpb.FileReference{
				Spec: &transferpb.FileReference_Spec{
					Uri:    ref.Reference(),
					Digest: frDigest,
				},
			})
		}
		return nil
	}); err != nil {
		return nil, fmt.Errorf("failed to walk referenced data: %w", err)
	}
	return fileRefs, nil
}

// LINT.IfChange
func sceneObjectStorageRefs(so *sopb.SceneObject) []string {
	var refs []string
	appendNonEmpty := func(ref string) {
		if ref != "" {
			refs = append(refs, ref)
		}
	}
	for _, entity := range so.GetEntities() {
		if named := entity.GetLink().GetGeometryComponent().GetNamedGeometries(); named != nil {
			for _, geomSet := range named {
				for _, geom := range geomSet.GetGeometries() {
					appendNonEmpty(geom.GetGeometryStorageRefs().GetGeometryRef())
					appendNonEmpty(geom.GetGeometryStorageRefs().GetRenderableRef())
				}
				if namedTransformed := geomSet.GetNamedGeometries(); namedTransformed != nil {
					for _, transformed := range namedTransformed {
						appendNonEmpty(transformed.GetGeometry().GetGeoRef().GetExactGeometryRef())
						appendNonEmpty(transformed.GetGeometry().GetGeoRef().GetRenderableRef())
					}
				}
			}
		}
	}
	return refs
}

// LINT.ThenChange(//intrinsic/scene/util/scene_object_geo_refs.cc)

// CollectFromApplication returns a list of all data files within a given
// application. The results are in an arbitrary order and ARE NOT deduplicated.
func CollectFromApplication(app *applicationpb.Application) ([]*transferpb.FileReference, error) {
	var dfs []*transferpb.FileReference
	for _, i := range app.GetResources().GetResourceInstances() {
		dfs = append(dfs, i.GetDataFiles().GetFiles()...)
	}
	for _, i := range app.GetInstances() {
		dfs = append(dfs, i.GetConfig().GetService().GetDataFiles()...)
	}
	for name, a := range app.GetAssets() {
		switch v := a.GetVariant().(type) {
		case *applicationpb.Application_Asset_Data:
			if fileRefs, err := collectDataAsset(v.Data); err != nil {
				return nil, fmt.Errorf("failed to inspect Data asset %q: %w", name, err)
			} else {
				dfs = append(dfs, fileRefs...)
			}
		}
	}
	return dfs, nil
}

// CollectFromRuntime returns a non-deduplicated list of file references from
// the ResourceTypeRuntime message.  This collects from both Data Assets, and
// SceneObjects.
func CollectFromRuntime(rtr *rtrpb.ResourceTypeRuntime) ([]*transferpb.FileReference, error) {
	var fileRefs []*transferpb.FileReference
	for _, ref := range sceneObjectStorageRefs(rtr.GetSceneObject()) {
		fileRefs = append(fileRefs, &transferpb.FileReference{
			Spec: &transferpb.FileReference_Spec{Uri: ref},
		})
	}

	switch v := rtr.GetVariant().(type) {
	case *rtrpb.ResourceTypeRuntime_Data:
		if refs, err := collectDataAsset(v.Data); err != nil {
			return nil, fmt.Errorf("failed to inspect Data asset: %w", err)
		} else {
			fileRefs = append(fileRefs, refs...)
		}
	}
	return fileRefs, nil
}

// CheckDuplicatePaths takes a list of file references and checks if there are
// some of them which specify the same in-cluster paths. If there are no
// duplicates, return nil; otherwise, return an error message with the list of
// duplicate files.
func CheckDuplicatePaths(fileReferences []*transferpb.FileReference) error {
	filePathsCount := make(map[string]int, len(fileReferences))
	for _, file := range fileReferences {
		filePathsCount[file.GetSpec().GetPath()]++
	}

	var duplicates []string
	for p, n := range filePathsCount {
		if n > 1 {
			duplicates = append(duplicates, fmt.Sprintf("%q", p))
		}
	}
	if len(duplicates) == 0 {
		return nil
	}

	return fmt.Errorf("%w: %s", ErrDuplicatePaths, strings.Join(duplicates, ", "))
}
