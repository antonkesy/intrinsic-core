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

// Package observability provides gRPC interceptors for adding and logging CAS peer metadata.
package observability

import (
	"context"
	"fmt"
	"os"
	"strings"

	log "github.com/golang/glog"
	"google.golang.org/grpc"
	"google.golang.org/grpc/metadata"
)

const (
	mdKey        = "x-icp" // _i_ntrinsic _C_AS _p_eer metadata
	logNoMD      = "!"
	logUnknown   = "-"
	logAvailable = "+"
	shortLen     = 4
)

func shorten(s string) string {
	l := len(s)
	if l > shortLen {
		l = shortLen
	}
	return s[:l]
}

func addPeerMetadata(ctx context.Context, hostname, name string) context.Context {
	md, ok := metadata.FromOutgoingContext(ctx)
	if !ok {
		md = metadata.New(make(map[string]string))
	}
	// If there was some peer data already, use it.
	if mdIn, ok := metadata.FromIncomingContext(ctx); ok {
		md.Append(mdKey, mdIn.Get(mdKey)...)
	}
	md.Append(mdKey, fmt.Sprintf("%s@%s", name, hostname))
	return metadata.NewOutgoingContext(ctx, md)
}

func addPeerMetadataReduced(ctx context.Context, hostname, name string) context.Context {
	return addPeerMetadata(ctx, shorten(hostname), shorten(name))
}

func noopPeerMetadata(ctx context.Context, _, _ string) context.Context {
	return ctx
}

var checkIdentity = func(ctx context.Context) (bool, bool) {
	return false, false
}

func extractPeerMetadata(ctx context.Context) string {
	peerInfo := logNoMD
	if md, ok := metadata.FromIncomingContext(ctx); ok {
		vals := md.Get(mdKey)
		if len(vals) == 0 {
			peerInfo = logUnknown
		} else {
			peerInfo = strings.Join(vals, ",")
		}
	}
	hasOrg, hasUser := checkIdentity(ctx)
	orgMarker := logUnknown
	if hasOrg {
		orgMarker = logAvailable
	}
	userMarker := logUnknown
	if hasUser {
		userMarker = logAvailable
	}
	return fmt.Sprintf("P:%s; O:%s; U:%s", peerInfo, orgMarker, userMarker)
}

type interceptorConfig struct {
	clientFunc func(context.Context, string, string) context.Context
}

// InterceptorOption is an option for configuring the CAS gRPC interceptors.
type InterceptorOption func(*interceptorConfig)

// WithCASPeerMetadata returns an interceptor option to enable CAS client tracing.
//
// We make this toggle-able as we were hit by max metadata size problem in b/313391059. We suggest
// off in prod, and on in dev until we have a better solution.
func WithCASPeerMetadata(enable bool) InterceptorOption {
	// N.B. we use a bool arg with the option to allow client code to consume a feature flag without
	// needing to branch.
	if !enable {
		return func(cfg *interceptorConfig) {
			cfg.clientFunc = noopPeerMetadata
		}
	}
	return func(cfg *interceptorConfig) {
		cfg.clientFunc = addPeerMetadata
	}
}

// WithCASPeerMetadataReduced returns an interceptor option to enable CAS client tracing.
//
// Reduces metadata size by shortening the hostname and service name, which can reduce clarity, so
// prefer to use WithCASPeerMetadata() instead.
// We make this toggle-able as we were hit by max metadata size problem in b/313391059. We suggest
// off in prod, and on in dev until we have a better solution.
func WithCASPeerMetadataReduced(enable bool) InterceptorOption {
	// N.B. we use a bool arg with the option to allow client code to consume a feature flag without
	// needing to branch.
	if !enable {
		return func(cfg *interceptorConfig) {
			cfg.clientFunc = noopPeerMetadata
		}
	}
	return func(cfg *interceptorConfig) {
		cfg.clientFunc = addPeerMetadataReduced
	}
}

// GRPCClientInterceptors returns grpc client interceptors for injecting CAS peer metadata.
func GRPCClientInterceptors(name string, opts ...InterceptorOption) (grpc.UnaryClientInterceptor, grpc.StreamClientInterceptor) {
	hostname, err := os.Hostname()
	if err != nil {
		hostname = ""
		log.Warningf("Failed to get hostname for CAS peer metadata: %v", err)
	}

	cfg := &interceptorConfig{
		clientFunc: noopPeerMetadata,
	}
	for _, opt := range opts {
		opt(cfg)
	}

	return func(ctx context.Context, method string, req, reply any, cc *grpc.ClientConn, invoker grpc.UnaryInvoker, opts ...grpc.CallOption) error {
			return invoker(cfg.clientFunc(ctx, hostname, name), method, req, reply, cc, opts...)
		}, func(ctx context.Context, desc *grpc.StreamDesc, cc *grpc.ClientConn, method string, streamer grpc.Streamer, opts ...grpc.CallOption) (grpc.ClientStream, error) {
			return streamer(cfg.clientFunc(ctx, hostname, name), desc, cc, method, opts...)
		}
}

// GRPCServerInterceptors returns grpc server interceptors for logging peer metadata.
func GRPCServerInterceptors() (grpc.UnaryServerInterceptor, grpc.StreamServerInterceptor) {
	return grpc.UnaryServerInterceptor(func(ctx context.Context, req any, info *grpc.UnaryServerInfo, handler grpc.UnaryHandler) (any, error) {
			m := extractPeerMetadata(ctx)
			res, err := handler(ctx, req)
			log.V(1).InfoContextf(ctx, "%s[%s]: %v", info.FullMethod, m, err)
			return res, err
		}), grpc.StreamServerInterceptor(func(srv any, ss grpc.ServerStream, info *grpc.StreamServerInfo, handler grpc.StreamHandler) error {
			m := extractPeerMetadata(ss.Context())
			err := handler(srv, ss)
			log.V(1).Infof("%s[%s]: %v", info.FullMethod, m, err)
			return err
		})
}
