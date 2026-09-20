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

package transfer

import (
	"context"
	"sync/atomic"

	"go.opencensus.io/stats"
	"go.opencensus.io/stats/view"
	"go.opencensus.io/tag"
)

var KeyOperation = tag.MustNewKey("operation")

const (
	OpDownload = "download"
	OpUpload   = "upload"
	OpStat     = "stat"
)

var (
	MTransferredBytes   = stats.Int64("transfer/transferred_bytes", "Total bytes successfully transferred", stats.UnitBytes)
	MWastedBytes        = stats.Int64("transfer/wasted_bytes", "Bytes transferred for operations that ultimately failed", stats.UnitBytes)
	MActiveOperations   = stats.Int64("transfer/active_operations", "Number of concurrent operations in progress", stats.UnitDimensionless)
	MFailedOperations   = stats.Int64("transfer/failed_operations", "Number of failed operations", stats.UnitDimensionless)
	MOperationAttempts  = stats.Int64("transfer/operation_attempts", "Total number of operation attempts", stats.UnitDimensionless)
	MOperationCompleted = stats.Int64("transfer/operation_completed", "Total number of completed operations", stats.UnitDimensionless)
)

var (
	TransferredBytesView = &view.View{
		Name:        "onpremcas/transfer/transferred_bytes_total",
		Measure:     MTransferredBytes,
		Description: "Total bytes successfully transferred",
		TagKeys:     []tag.Key{KeyOperation},
		Aggregation: view.Sum(),
	}
	WastedBytesView = &view.View{
		Name:        "onpremcas/transfer/wasted_bytes_total",
		Measure:     MWastedBytes,
		Description: "Bytes transferred for operations that ultimately failed",
		TagKeys:     []tag.Key{KeyOperation},
		Aggregation: view.Sum(),
	}
	ActiveOperationsView = &view.View{
		Name:        "onpremcas/transfer/active_operations",
		Measure:     MActiveOperations,
		Description: "Number of concurrent operations in progress",
		TagKeys:     []tag.Key{KeyOperation},
		Aggregation: view.LastValue(),
	}
	FailedOperationsView = &view.View{
		Name:        "onpremcas/transfer/failed_operations_total",
		Measure:     MFailedOperations,
		Description: "Number of failed operations",
		TagKeys:     []tag.Key{KeyOperation},
		Aggregation: view.Sum(),
	}
	OperationAttemptsView = &view.View{
		Name:        "onpremcas/transfer/operation_attempts_total",
		Measure:     MOperationAttempts,
		Description: "Total number of operation attempts",
		TagKeys:     []tag.Key{KeyOperation},
		Aggregation: view.Sum(),
	}
	OperationCompletedView = &view.View{
		Name:        "onpremcas/transfer/operation_completed_total",
		Measure:     MOperationCompleted,
		Description: "Total number of completed operations",
		TagKeys:     []tag.Key{KeyOperation},
		Aggregation: view.Sum(),
	}

	// Views is a list of all default views defined in this package.
	Views = []*view.View{
		TransferredBytesView,
		WastedBytesView,
		ActiveOperationsView,
		FailedOperationsView,
		OperationAttemptsView,
		OperationCompletedView,
	}
)

var (
	currentActiveDownloads atomic.Int64
	currentActiveUploads   atomic.Int64
	currentActiveStats     atomic.Int64
)

func recordWithOp(ctx context.Context, op string, m ...stats.Measurement) {
	ctx, err := tag.New(ctx, tag.Upsert(KeyOperation, op))
	if err == nil {
		stats.Record(ctx, m...)
	}
}

func recordAttempt(ctx context.Context, op string) {
	recordWithOp(ctx, op, MOperationAttempts.M(1))
}

func recordCompleted(ctx context.Context, op string) {
	recordWithOp(ctx, op, MOperationCompleted.M(1))
}

func recordFailed(ctx context.Context, op string) {
	recordWithOp(ctx, op, MFailedOperations.M(1))
}

func recordTransferredBytes(ctx context.Context, op string, bytes int64) {
	recordWithOp(ctx, op, MTransferredBytes.M(bytes))
}

func recordWastedBytes(ctx context.Context, op string, bytes int64) {
	recordWithOp(ctx, op, MWastedBytes.M(bytes))
}

func trackActiveOperations(ctx context.Context, op string, delta int64) {
	var val int64
	switch op {
	case OpDownload:
		val = currentActiveDownloads.Add(delta)
	case OpUpload:
		val = currentActiveUploads.Add(delta)
	case OpStat:
		val = currentActiveStats.Add(delta)
	}
	recordWithOp(ctx, op, MActiveOperations.M(val))
}
