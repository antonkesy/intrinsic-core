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

// Package casgeometryuploader provides a CAS (go/intrinsic-cas) uploader for geometries and
// renderables during the release process.
package casgeometryuploader

import (
	"context"
	"fmt"
	"os"
	"path/filepath"
	"sync"

	"intrinsic/storage/content_addressable_storage/pkg/filetocas"

	log "github.com/golang/glog"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"
	"golang.org/x/sync/errgroup"
)

const (
	maxWorkers = 10
)

var (
	// ErrInvalidNumWorkers is a sentinel error signifying the passed parameter for the number of
	// workers is outside of the valid range.
	ErrInvalidNumWorkers = errors.New(fmt.Sprintf("invalid numWorkers, want 1 <= numWorkers <= %d", maxWorkers))
	// ErrNotADirectory is a sentinel error signifying the passed parameter was expected to be a
	// directory, but is not.
	ErrNotADirectory = errors.New("not a directory")
)

// TranslationTable maps geometry fingerprints to their CAS storage references.
type TranslationTable map[string]string

// CASGeometryUploader scans a local folder, uploads geometries and renderables to CAS, and returns
// a translation table from the geometry fingerprint to CAS IDs.
type CASGeometryUploader struct {
	f2c        *filetocas.Uploader
	numWorkers int

	muTT *sync.Mutex
	tt   TranslationTable
}

// New constructs a [CASGeometryUploader] which is ready to use. Return an error if the given local
// path cannot be converted to an absolute path or is not a directory.
func New(f2c *filetocas.Uploader, numWorkers int) (*CASGeometryUploader, error) {
	if numWorkers < 1 || numWorkers > maxWorkers {
		return nil, fmt.Errorf("numWorkers = %d: %w", numWorkers, ErrInvalidNumWorkers)
	}
	return &CASGeometryUploader{
		f2c:        f2c,
		numWorkers: numWorkers,
		muTT:       new(sync.Mutex),
		tt:         make(TranslationTable),
	}, nil
}

// ScanAndUpload scans the local directory (ignoring subfolders) and uploads geometries and the
// corresponding renderables to the CAS, updating the translation table. If a geometry doesn't have
// a renderable, returns [ErrMissingGLTF]. Files are assumed to be immutable and thus are only
// uploaded once. I.e. the second call of this function will not do anything if the fileset has not
// changed.
// Returns the translation table for the files found in the given local directory.
func (u *CASGeometryUploader) ScanAndUpload(ctx context.Context, localDir string) (TranslationTable, error) {
	ctx, span := trace.StartSpan(ctx, "CASGeometryUploader.ScanAndUpload")
	defer span.End()

	ld, err := filepath.Abs(localDir)
	if err != nil {
		return nil, fmt.Errorf("cannot get absolute path for %q: %v", localDir, err)
	}
	if info, err := os.Stat(ld); err != nil {
		return nil, fmt.Errorf("cannot stat %q: %v", ld, err)
	} else if !info.IsDir() {
		return nil, fmt.Errorf("%q: %w", ld, ErrNotADirectory)
	}

	des, err := os.ReadDir(ld)
	if err != nil {
		return nil, err
	}

	var filenames []string
	group, childCtx := errgroup.WithContext(ctx)
	group.SetLimit(u.numWorkers)
	for _, de := range des {
		if de.IsDir() {
			continue
		}
		f := de.Name()
		filenames = append(filenames, f)
		u.muTT.Lock()
		_, ok := u.tt[f]
		u.muTT.Unlock()
		if ok {
			log.V(1).InfoContextf(ctx, "Skipping %q because it's already in the translation table", f)
			continue
		}
		ctx := childCtx
		group.Go(func() error {
			// Upload the file to CAS and record the CAS ID in the translation table.
			casRef, _, err := u.f2c.Upload(ctx, filepath.Join(ld, f))
			if err != nil {
				return fmt.Errorf("could not upload %q to CAS: %w", f, err)
			}
			u.muTT.Lock()
			defer u.muTT.Unlock()
			u.tt[f] = casRef
			return nil
		})
	}
	if err := group.Wait(); err != nil {
		return nil, err
	}

	u.muTT.Lock()
	defer u.muTT.Unlock()
	relevantTT := make(TranslationTable, len(filenames))
	for _, f := range filenames {
		relevantTT[f] = u.tt[f]
	}
	return relevantTT, nil
}

// TranslationTable returns the currently available translation table from geometry fingerprint to
// the geometries' and renderables' CAS object IDs.
func (u *CASGeometryUploader) TranslationTable() TranslationTable {
	u.muTT.Lock()
	defer u.muTT.Unlock()

	// Create a copy of the map to prevent concurrent modifications after returning.
	ttCopy := make(TranslationTable, len(u.tt))
	for k, v := range u.tt {
		ttCopy[k] = v
	}
	return ttCopy
}
