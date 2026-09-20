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

package internal

import (
	"fmt"
	"io"
	"net/http"
	"strings"

	"github.com/containerd/containerd"
	"github.com/containerd/containerd/content"
	"github.com/containerd/containerd/errdefs"
	"github.com/containerd/containerd/namespaces"
	log "github.com/golang/glog"
	"github.com/opencontainers/go-digest"
	ocispec "github.com/opencontainers/image-spec/specs-go/v1"
)

// OCIRegistryHandler implements an HTTP handler compliant with the OCI Distribution Spec v1.0.1
// (Docker Registry HTTP API V2), backed by containerd's ContentStore and ImageService.
//
// Spec reference: https://github.com/opencontainers/distribution-spec/blob/v1.0.1/spec.md
type OCIRegistryHandler struct {
	client    *containerd.Client
	namespace string
}

// NewOCIRegistryHandler creates a new OCIRegistryHandler.
func NewOCIRegistryHandler(client *containerd.Client, namespace string) *OCIRegistryHandler {
	return &OCIRegistryHandler{
		client:    client,
		namespace: namespace,
	}
}

// ServeHTTP routes incoming registry requests according to the OCI Distribution Spec:
//   - GET/HEAD /v2/                          -> API Version Check
//   - GET/HEAD /v2/<name>/manifests/<ref>     -> Pulling Manifests
//   - GET/HEAD /v2/<name>/blobs/<digest>      -> Pulling Blobs
//
// Spec reference: https://github.com/opencontainers/distribution-spec/blob/v1.0.1/spec.md#endpoints
func (h *OCIRegistryHandler) ServeHTTP(w http.ResponseWriter, r *http.Request) {
	// All responses MUST include the Docker-Distribution-API-Version header.
	// Spec: https://github.com/opencontainers/distribution-spec/blob/v1.0.1/spec.md#api-version-check
	w.Header().Set("Docker-Distribution-API-Version", "registry/2.0")

	path := strings.TrimPrefix(r.URL.Path, "/v2")
	path = strings.TrimPrefix(path, "/")

	if path == "" {
		// Section: "API Version Check"
		// Spec: https://github.com/opencontainers/distribution-spec/blob/v1.0.1/spec.md#api-version-check
		// GET /v2/ returns 200 OK with an empty JSON object to indicate v2 support.
		w.Header().Set("Content-Type", "application/json")
		w.WriteHeader(http.StatusOK)
		_, _ = w.Write([]byte("{}\n"))
		return
	}

	if idx := strings.Index(path, "/manifests/"); idx != -1 {
		name := path[:idx]
		ref := path[idx+len("/manifests/"):]
		h.handleManifest(w, r, name, ref)
		return
	}

	if idx := strings.Index(path, "/blobs/"); idx != -1 {
		name := path[:idx]
		dig := path[idx+len("/blobs/"):]
		h.handleBlob(w, r, name, dig)
		return
	}

	http.NotFound(w, r)
}

// handleManifest serves GET and HEAD requests for manifests.
//
// Section: "Pulling Manifests"
// Spec: https://github.com/opencontainers/distribution-spec/blob/v1.0.1/spec.md#pulling-manifests
//
// Success Response:
//   - 200 OK
//   - Content-Type: <mediaType of manifest>
//   - Docker-Content-Digest: <canonical sha256 digest>
//   - Content-Length: <size in bytes>
//
// Error Response (404 Not Found):
//   - Code: MANIFEST_UNKNOWN
//   - Spec: https://github.com/opencontainers/distribution-spec/blob/v1.0.1/spec.md#error-codes
func (h *OCIRegistryHandler) handleManifest(w http.ResponseWriter, r *http.Request, name, ref string) {
	if r.Method != http.MethodGet && r.Method != http.MethodHead {
		http.Error(w, "Method Not Allowed", http.StatusMethodNotAllowed)
		return
	}

	ctx := namespaces.WithNamespace(r.Context(), h.namespace)
	cs := h.client.ContentStore()
	is := h.client.ImageService()

	var manifestDigest digest.Digest
	var mediaType string
	var manifestBytes []byte

	// 1. Check if ref is a direct canonical digest (e.g., sha256:...)
	if dig, err := digest.Parse(ref); err == nil {
		manifestDigest = dig
		info, err := cs.Info(ctx, dig)
		if err == nil {
			manifestBytes, err = content.ReadBlob(ctx, cs, ocispec.Descriptor{Digest: dig, Size: info.Size})
			if err == nil {
				if info.Labels != nil {
					mediaType = info.Labels["containerd.io/gc.ref.content.l"]
				}
			}
		}
	}

	// 2. If not resolved by direct digest, look up via containerd ImageService tags
	if len(manifestBytes) == 0 {
		lookupNames := []string{
			name + ":" + ref,
			name + "@" + ref,
			ref,
		}
		for _, lookupName := range lookupNames {
			img, err := is.Get(ctx, lookupName)
			if err == nil {
				manifestDigest = img.Target.Digest
				mediaType = img.Target.MediaType
				manifestBytes, _ = content.ReadBlob(ctx, cs, img.Target)
				if len(manifestBytes) > 0 {
					break
				}
			}
		}
	}

	if len(manifestBytes) == 0 {
		w.Header().Set("Content-Type", "application/json")
		w.WriteHeader(http.StatusNotFound)
		_, _ = w.Write([]byte(`{"errors":[{"code":"MANIFEST_UNKNOWN","message":"manifest not found"}]}` + "\n"))
		return
	}

	if mediaType == "" {
		mediaType = ocispec.MediaTypeImageManifest
	}

	w.Header().Set("Content-Type", mediaType)
	w.Header().Set("Docker-Content-Digest", manifestDigest.String())
	w.Header().Set("Content-Length", fmt.Sprintf("%d", len(manifestBytes)))
	w.WriteHeader(http.StatusOK)

	if r.Method == http.MethodGet {
		_, _ = w.Write(manifestBytes)
	}
}

// handleBlob serves GET and HEAD requests for layer and config blobs.
//
// Section: "Pulling Blobs"
// Spec: https://github.com/opencontainers/distribution-spec/blob/v1.0.1/spec.md#pulling-blobs
//
// Success Response:
//   - 200 OK
//   - Content-Type: application/octet-stream
//   - Docker-Content-Digest: <canonical sha256 digest>
//   - Content-Length: <size in bytes>
//
// Error Response (404 Not Found):
//   - Code: BLOB_UNKNOWN
//   - Spec: https://github.com/opencontainers/distribution-spec/blob/v1.0.1/spec.md#error-codes
func (h *OCIRegistryHandler) handleBlob(w http.ResponseWriter, r *http.Request, name, digStr string) {
	if r.Method != http.MethodGet && r.Method != http.MethodHead {
		http.Error(w, "Method Not Allowed", http.StatusMethodNotAllowed)
		return
	}

	dig, err := digest.Parse(digStr)
	if err != nil {
		http.Error(w, "Invalid digest", http.StatusBadRequest)
		return
	}

	ctx := namespaces.WithNamespace(r.Context(), h.namespace)
	cs := h.client.ContentStore()

	ra, err := cs.ReaderAt(ctx, ocispec.Descriptor{Digest: dig})
	if err != nil {
		if errdefs.IsNotFound(err) {
			w.Header().Set("Content-Type", "application/json")
			w.WriteHeader(http.StatusNotFound)
			_, _ = w.Write([]byte(`{"errors":[{"code":"BLOB_UNKNOWN","message":"blob not found"}]}` + "\n"))
			return
		}
		log.Errorf("failed to open blob reader for %s: %v", dig, err)
		http.Error(w, "Internal Server Error", http.StatusInternalServerError)
		return
	}
	defer ra.Close()

	w.Header().Set("Content-Type", "application/octet-stream")
	w.Header().Set("Docker-Content-Digest", dig.String())
	w.Header().Set("Content-Length", fmt.Sprintf("%d", ra.Size()))
	w.WriteHeader(http.StatusOK)

	if r.Method == http.MethodGet {
		reader := io.NewSectionReader(ra, 0, ra.Size())
		if _, err := io.Copy(w, reader); err != nil {
			log.Warningf("error streaming blob %s: %v", dig, err)
		}
	}
}
