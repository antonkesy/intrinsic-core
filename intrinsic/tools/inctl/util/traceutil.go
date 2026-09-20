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

// Package traceutil contains utilities for uploading traces from inctl.
package traceutil

import (
	"context"
	"flag"
	"fmt"

	"intrinsic/tools/inctl/util/orgutil"

	traceexporter "github.com/GoogleCloudPlatform/opentelemetry-operations-go/exporter/trace"
	"go.opentelemetry.io/otel"
	"go.opentelemetry.io/otel/bridge/opencensus"
	"go.opentelemetry.io/otel/sdk/resource"
	sdktrace "go.opentelemetry.io/otel/sdk/trace"
	semconv "go.opentelemetry.io/otel/semconv/v1.34.0"

	log "github.com/golang/glog"
)


var (
	flagTrace bool // The --trace flag.
)



func newTracerProvider(project string) (*sdktrace.TracerProvider, error) {
	exporter, err := traceexporter.New(traceexporter.WithProjectID(project))
	if err != nil {
		return nil, fmt.Errorf("create trace exporter: %w", err)
	}

	res, err := resource.New(context.Background(),
		resource.WithAttributes(semconv.ServiceName("inctl")),
	)
	if err != nil {
		return nil, fmt.Errorf("create resource: %w", err)
	}

	tp := sdktrace.NewTracerProvider(
		sdktrace.WithBatcher(exporter),
		sdktrace.WithResource(res),
		sdktrace.WithSampler(sdktrace.AlwaysSample()),
	)

	otel.SetTracerProvider(tp)
	opencensus.InstallTraceBridge()
	return tp, nil
}

// RegisterExporter registers a StackDriver for the given project that sends errors to the log
// rather than stderr, so as not to confuse users. It returns a flush function that can be called to
// wait for in-progress uploads.
func RegisterExporter(project string) func() {
	otel.SetErrorHandler(otel.ErrorHandlerFunc(func(err error) {
		log.V(1).Infof("Some traces have not been uploaded: %v", err)
	}))
	tp, err := newTracerProvider(project)
	if err != nil {
		log.V(1).Infof("Traces will not be uploaded: %v", err)
		return func() {}
	}
	return func() {
		tp.Shutdown(context.Background())
	}
}

// getProject determines cloud-project from flags or environment variables in order to set up
// the trace-exporter. The tracing setup needs to be done as early as possible..
//
// Note: We tried to use a PersistentPreRun() on the `rootCmd“ together with setting
// `cobra.EnableTraverseRunHooks = true`, but we still could not access the flag values.
// Retry when we left g3 and don't use gflags anymore.
func getProject(ctx context.Context, args []string) string {
	if project := orgutil.GetProject(nil, args); project != "" {
		log.V(1).InfoContextf(ctx, "Using project %q from args or environment.", project)
		return project
	}
	// TODO: hard-coding the project creates broken traces for all other projects
	log.V(1).InfoContextf(ctx, "Could not determine project. Using fallback %q", "giza-workcells")
	return "giza-workcells"
}

// SetUpTracing initializes the trace exporter.
// Should only be called from the root command.
func SetUpTracing(ctx context.Context, args []string) func() {
	if flagTrace {
		flush := RegisterExporter(getProject(ctx, args))
		return flush
	}
	return func() {}
}

func init() {
	// We use a global flag here to be able to access it before cobra has done the flag parsing.
	// This is okay as the commands don't need to access the flag.
	flag.BoolVar(&flagTrace, "trace", true, "Enable tracing.")
}
