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

// Package openapihandler contains HTTP Handlers that work without requiring any other gRPC services.
package openapihandler

import (
	"log/slog"

	oah "intrinsic/httpjson/openapi/handlers"

	"github.com/grpc-ecosystem/grpc-gateway/v2/runtime"
)

// RegisterOpenAPIHandler registers a handler for the /openapi.yaml endpoint.
func RegisterOpenAPIHandler(mux *runtime.ServeMux) {
	slog.Info("Registering OpenAPI handler")
	rlocationPath := "intrinsic-core/intrinsic_runtime/httpjson/_http_gateway_openapi/openapi.yaml"
	handler, err := oah.MakeOpenAPIHandlerFromRunfiles(rlocationPath)
	if err != nil {
		slog.Error("Failed to make OpenAPI Handler %v", err)
		return
	}
	mux.HandlePath("GET", "/openapi.yaml", handler)
}
