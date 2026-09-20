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
	"context"
	"fmt"
	"io"
	"log/slog"
	"os"
	"path/filepath"
	"strings"
	"sync"
	"syscall"
	"time"

	"go.opencensus.io/stats"
	"go.opencensus.io/stats/view"
	"go.opencensus.io/tag"
)

var (
	MUsedBytes           = stats.Int64("used_bytes", "The number of bytes used by objects in the dir", stats.UnitBytes)
	MFreeBytes           = stats.Int64("free_bytes", "The number of free bytes in the dir folder", stats.UnitBytes)
	MObjectCount         = stats.Int64("object_count", "The number of objects in the dir", stats.UnitDimensionless)
	MInactiveUsedBytes   = stats.Int64("inactive_used_bytes", "The number of bytes used by inactive objects", stats.UnitBytes)
	MInactiveObjectCount = stats.Int64("inactive_object_count", "The number of inactive objects", stats.UnitDimensionless)

	DirKey  = tag.MustNewKey("dir")
	TypeKey = tag.MustNewKey("type")
)

var (
	UsedBytesView = &view.View{
		Name:        "onpremcas/storage/used_bytes",
		Measure:     MUsedBytes,
		Description: "The number of bytes used by objects in dir",
		Aggregation: view.LastValue(),
		TagKeys:     []tag.Key{DirKey, TypeKey},
	}
	FreeBytesView = &view.View{
		Name:        "onpremcas/storage/free_bytes",
		Measure:     MFreeBytes,
		Description: "The number of free bytes in dir",
		Aggregation: view.LastValue(),
		TagKeys:     []tag.Key{DirKey},
	}
	ObjectCountView = &view.View{
		Name:        "onpremcas/storage/object_count",
		Measure:     MObjectCount,
		Description: "The number of objects in dir",
		Aggregation: view.LastValue(),
		TagKeys:     []tag.Key{DirKey, TypeKey},
	}
	InactiveUsedBytesView = &view.View{
		Name:        "onpremcas/storage/inactive_used_bytes",
		Measure:     MInactiveUsedBytes,
		Description: "The number of bytes used by inactive objects",
		Aggregation: view.LastValue(),
	}
	InactiveObjectCountView = &view.View{
		Name:        "onpremcas/storage/inactive_object_count",
		Measure:     MInactiveObjectCount,
		Description: "The number of inactive objects",
		Aggregation: view.LastValue(),
	}

	// Views is a list of all views defined in this package.
	Views = []*view.View{
		UsedBytesView,
		FreeBytesView,
		ObjectCountView,
		InactiveUsedBytesView,
		InactiveObjectCountView,
	}
)

// ActiveObjectsProvider provides a snapshot of currently active object digests.
type ActiveObjectsProvider interface {
	ActiveObjects() map[string]struct{}
}

// Run runs metrics collection loop in background.
func Run(ctx context.Context, objectsDir, partialDir string, activeProvider ActiveObjectsProvider, interval time.Duration) (io.Closer, error) {
	if interval <= 0 {
		return nil, fmt.Errorf("Invalid interval %d for metrics collection", interval)
	}
	ctx, cancel := context.WithCancel(ctx)
	ticker := time.NewTicker(interval)
	var wg sync.WaitGroup
	wg.Go(func() {
		defer ticker.Stop()
		// Run once at start
		refreshMetrics(ctx, objectsDir, partialDir, activeProvider)
		for {
			select {
			case <-ctx.Done():
				return
			case <-ticker.C:
				refreshMetrics(ctx, objectsDir, partialDir, activeProvider)
			}
		}
	})
	return closeFunc(func() error {
		cancel()
		wg.Wait()
		return nil
	}), nil
}

type closeFunc func() error

func (f closeFunc) Close() error {
	return f()
}

type statsData struct {
	usedBytes   int64
	objectCount int64
}

// walk the directory tree and look for files with the given suffix
func makeWalkFn(suffix string, data *statsData) filepath.WalkFunc {
	return func(path string, info os.FileInfo, err error) error {
		if err != nil {
			if !os.IsNotExist(err) {
				slog.Warn("Failed to access path during walk", slog.String("path", path), slog.Any("error", err))
			}
			return nil
		}
		if info.IsDir() {
			return nil
		}
		if strings.HasSuffix(path, suffix) {
			data.usedBytes += info.Size()
			data.objectCount++
		}
		return nil
	}
}

func refreshMetrics(ctx context.Context, objectsDir, partialDir string, activeProvider ActiveObjectsProvider) {
	objectData := &statsData{}
	inactiveData := &statsData{}
	partialData := &statsData{}

	activeSet := activeProvider.ActiveObjects()

	// walkObjectsFn walks the objects directory to collect total object counts and byte sizes,
	// as well as counts and sizes of inactive objects not present in the active syncsets.
	walkObjectsFn := func(path string, info os.FileInfo, err error) error {
		if err != nil {
			if !os.IsNotExist(err) {
				slog.Warn("Failed to access path during walk", slog.String("path", path), slog.Any("error", err))
			}
			return nil
		}
		if info.IsDir() {
			return nil
		}
		size := info.Size()
		objectData.usedBytes += size
		objectData.objectCount++

		digest := info.Name()
		if _, isActive := activeSet[digest]; !isActive {
			inactiveData.usedBytes += size
			inactiveData.objectCount++
		}
		return nil
	}

	if err := filepath.Walk(objectsDir, walkObjectsFn); err != nil {
		slog.Error("Failed to calculate storage metrics for objects_dir",
			slog.String("directory", objectsDir), slog.Any("error", err))
	}

	if err := filepath.Walk(partialDir, makeWalkFn(PartialExt, partialData)); err != nil {
		slog.Error("Failed to calculate storage metrics for partial_dir",
			slog.String("directory", partialDir), slog.Any("error", err))
	}

	// Record object metrics (tagged with objectsDir and type="object")
	objectTags := []tag.Mutator{
		tag.Insert(DirKey, objectsDir),
		tag.Insert(TypeKey, "object"),
	}
	if err := stats.RecordWithTags(ctx, objectTags,
		MUsedBytes.M(objectData.usedBytes),
		MObjectCount.M(objectData.objectCount)); err != nil {
		slog.Error("Failed to record storage metrics for objects", slog.Any("error", err))
	}

	// Record inactive object metrics
	stats.Record(ctx,
		MInactiveUsedBytes.M(inactiveData.usedBytes),
		MInactiveObjectCount.M(inactiveData.objectCount))

	// Record partial metrics (tagged with partialDir and type="partial")
	partialTags := []tag.Mutator{
		tag.Insert(DirKey, partialDir),
		tag.Insert(TypeKey, "partial"),
	}
	if err := stats.RecordWithTags(ctx, partialTags,
		MUsedBytes.M(partialData.usedBytes),
		MObjectCount.M(partialData.objectCount)); err != nil {
		slog.Error("Failed to record storage metrics for partials", slog.Any("error", err))
	}

	// Free bytes (only uses objectsDir)
	var stat syscall.Statfs_t
	if err := syscall.Statfs(objectsDir, &stat); err != nil {
		slog.Error("Failed to calculate free bytes", slog.Any("error", err))
	} else {
		freeBytes := int64(stat.Bfree) * int64(stat.Bsize)
		freeTags := []tag.Mutator{tag.Insert(DirKey, objectsDir)}
		if err := stats.RecordWithTags(ctx, freeTags, MFreeBytes.M(freeBytes)); err != nil {
			slog.Error("Failed to record free bytes metric", slog.Any("error", err))
		}
	}
}
