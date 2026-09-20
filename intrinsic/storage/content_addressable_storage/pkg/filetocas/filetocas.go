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

// Package filetocas helps you upload a file to the content-addressable storage (CAS) service from a
// variety of sources.
package filetocas

import (
	"context"
	"encoding/hex"
	"fmt"
	"io"
	"io/fs"
	"net/url"
	"os"
	"path/filepath"
	"regexp"
	"strconv"
	"strings"

	"intrinsic/kubernetes/workcell_spec/author"
	"intrinsic/storage/content_addressable_storage/pkg/clienthelpers"
	casgrpcpb "intrinsic/storage/content_addressable_storage/proto/cas_service_go_proto"

	"cloud.google.com/go/storage"
	log "github.com/golang/glog"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"
)

const (
	// Data URLs are of the form: data:[<mediatype>][;base64],<data>
	// See https://datatracker.ietf.org/doc/html/rfc2397 for more details.
	dataPrefix         = "data:,"
	buildOutPrefix     = "build-out://"
	gcsPrefix          = "gs://"
	intrinsicCASPrefix = "intcas://"
)

var (
	// ErrMissingGCSCapability is a sentinel error signaling that user wants to read from GCS without
	// providing a GCS client.
	ErrMissingGCSCapability = errors.New("GCS client not provided, use the 'WithGCSClient()' option")
	// ErrMissingBuildOutCapability is a sentinel error signaling that user wants to read from the
	// build-out directory without providing a runfiles dir path.
	ErrMissingBuildOutCapability = errors.New("runfiles directory not provided, use the 'WithRunfilesFS()' option")

	gcsPathRegexp = regexp.MustCompile(`^gs://([\w-_]+)/(.*)#(\d+)$`)
)

// Uploader uploads to the content-addressable storage service.
type Uploader struct {
	casClient  casgrpcpb.ContentAddressableStorageServiceClient
	gcsClient  *storage.Client
	runfilesFS fs.FS
	chunkSize  int64
	dryRun     bool
}

// Option configures the [Uploader] via the Go functional options (go/go-functional-options)
// pattern.
type Option = func(*Uploader)

// Options is a list of multiple [Option]s.
type Options = []Option

// WithGCSClient adds the capability to upload from a GCS reference.
func WithGCSClient(gcsClient *storage.Client) Option {
	return func(u *Uploader) {
		u.gcsClient = gcsClient
	}
}

// WithRunfilesFS adds the capability to upload from a Bazel output directory.
func WithRunfilesFS(runfilesFS fs.FS) Option {
	return func(u *Uploader) {
		u.runfilesFS = runfilesFS
	}
}

// UseChunkSize configures the chunk size to be used during the upload. If unset,
// [clienthelpers.DefaultUploadChunkSize] will be used.
func UseChunkSize(chunkSize int64) Option {
	return func(u *Uploader) {
		u.chunkSize = chunkSize
	}
}

// DryRun sets the dry run mode. By default, the dry run mode is OFF.
func DryRun(b bool) Option {
	return func(u *Uploader) {
		u.dryRun = b
	}
}

// NewUploader creates a new [Uploader] which is ready to use for uploading from a file or from an
// inline data blob.
func NewUploader(casClient casgrpcpb.ContentAddressableStorageServiceClient, opts ...Option) *Uploader {
	u := &Uploader{
		casClient: casClient,
		chunkSize: clienthelpers.DefaultUploadChunkSize,
	}
	for _, opt := range opts {
		opt(u)
	}
	return u
}

// Upload a file to the content-addressable storage (CAS) service. The URI can be:
//
//   - A GCS object URI (requires [WithGCSClient] option, otherwise return the
//     [ErrMissingGCSCapability] sentinel error).
//   - An inlined data prefix as per https://datatracker.ietf.org/doc/html/rfc2397.
//   - A Bazel build-out (requires [WithRunfilesFS] option, otherwise return the
//     [ErrMissingBuildOutCapability] sentinel error).
//
// If neither of those prefixes match, the function will try to interpret the URI as a local file
// path. Another special case is the CAS prefix, in which case this function will return an error
// because reuploading an object to CAS does not make sense. You might want to use
// [intrinsic/storage/content_addressable_storage/pkg/crossproject.Copy] instead.
//
// If the user did not configure a dry run via the [DryRun] option, this function will always upload
// the full file to the CAS. It DOES NOT check if the object already exists in the CAS.
//
// If the upload was successful, return the Intrinsic CAS object ID and the HighwayHash128 hex
// digest. If a dry run was requested, return the input uri and a HighwayHash128 hex digest.
func (u *Uploader) Upload(ctx context.Context, uri string) (string, string, error) {
	ctx, span := trace.StartSpan(ctx, "filetocas.Upload")
	defer span.End()
	span.AddAttributes(
		trace.StringAttribute("uri", uri),
		trace.BoolAttribute("dry_run", u.dryRun),
	)

	var makeReader func(context.Context, string) (io.ReadCloser, error)
	if strings.HasPrefix(uri, dataPrefix) {
		makeReader = readerFromInlinedData
	} else if strings.HasPrefix(uri, buildOutPrefix) {
		makeReader = u.readerFromRunfiles
	} else if strings.HasPrefix(uri, gcsPrefix) {
		makeReader = u.readerFromGCS
	} else if strings.HasPrefix(uri, intrinsicCASPrefix) {
		return "", "", fmt.Errorf("%q is an Intrinsic CAS reference which does not make sense here", uri)
	} else {
		makeReader = readerFromFile
	}

	rc, err := makeReader(ctx, uri)
	if err != nil {
		return "", "", fmt.Errorf("failed to create a reader for %q: %w", uri, err)
	}
	defer rc.Close()

	hasher, err := author.NewHash128()
	if err != nil {
		return "", "", fmt.Errorf("could not create hasher: %w", err)
	}
	if u.dryRun {
		_, err := io.Copy(hasher, rc)
		if err != nil {
			return "", "", fmt.Errorf("failed to copy %q: %w", uri, err)
		}
		log.InfoContextf(ctx, "Dry run, would've uploaded %q", uri)
		return uri, hex.EncodeToString(hasher.Sum(nil)), nil
	}

	tr := io.TeeReader(rc, hasher)
	objectID, err := clienthelpers.Create(ctx, u.casClient, tr, u.chunkSize)
	if err != nil {
		return "", "", fmt.Errorf("failed to upload to CAS: %w", err)
	}
	log.V(1).InfoContextf(ctx, "Moved from %q to %q", uri, objectID)
	return objectID, hex.EncodeToString(hasher.Sum(nil)), nil
}

func readerFromInlinedData(_ context.Context, uri string) (io.ReadCloser, error) {
	b, err := url.QueryUnescape(strings.TrimPrefix(uri, dataPrefix))
	if err != nil {
		return nil, errors.Wrapf(err, "decoding data uri")
	}
	return io.NopCloser(strings.NewReader(b)), nil
}

func readerFromFile(ctx context.Context, path string) (io.ReadCloser, error) {
	file, err := os.Open(path)
	if err != nil {
		return nil, errors.Wrapf(err, "open %q", path)
	}
	return file, nil
}

func (u *Uploader) readerFromRunfiles(ctx context.Context, path string) (io.ReadCloser, error) {
	if u.runfilesFS == nil {
		return nil, ErrMissingBuildOutCapability
	}
	f, err := u.runfilesFS.Open(filepath.Join("_main", strings.TrimPrefix(path, buildOutPrefix)))
	if err != nil {
		return nil, errors.Wrapf(err, "open %q", path)
	}
	return f, nil
}

func (u *Uploader) readerFromGCS(ctx context.Context, uri string) (io.ReadCloser, error) {
	if u.gcsClient == nil {
		return nil, ErrMissingGCSCapability
	}
	return readerForPath(ctx, u.gcsClient, uri)
}

// objectForPath returns a handle for the given GCS object. It is assumed that the path
// is of the form "gs://bucket-name/object/name#1234"
func objectForPath(client *storage.Client, path string) (*storage.ObjectHandle, error) {
	// The first match returns the entire path but we are only interested in the individual capture groups.
	gcsName := gcsPathRegexp.FindStringSubmatch(path)
	if len(gcsName) != 4 {
		return nil, fmt.Errorf("couldn't extract bucket, name, and generation number from GCS file reference in %q (result of regexp: %v)", path, gcsPathRegexp)
	}
	gen, err := strconv.ParseInt(gcsName[3], 10, 64)
	if err != nil {
		return nil, errors.Wrapf(err, "parsing the gen id from %q", gcsName[3])
	}
	return client.Bucket(gcsName[1]).Object(gcsName[2]).Generation(gen), nil
}

// readerForPath returns a reader for the given GCS object. It is assumed that the path
// is of the form "gs://bucket-name/object/name#1234"
func readerForPath(ctx context.Context, client *storage.Client, path string) (*storage.Reader, error) {
	obj, err := objectForPath(client, path)
	if err != nil {
		return nil, err
	}
	return obj.NewReader(ctx)
}
