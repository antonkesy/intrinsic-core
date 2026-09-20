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

// Package apputil contains utility functions common to inctl app functionality.
package apputil

import (
	"context"
	"net/http"
	"os/user"
	"strings"

	"intrinsic/tools/inctl/auth/auth"

	log "github.com/golang/glog"
	"github.com/pkg/errors"
	"google.golang.org/grpc/metadata"
)

// IsDevCluster return true if the given k8s context is for a developer cluster (local minikube).
func IsDevCluster(contextName string) bool {
	return contextName == "minikube"
}

// IsCloudCluster return true if the given k8s context is for a cloud cluster (e.g. "cloud-robotics", "ml-compute-cluster").
func IsCloudCluster(contextName string) bool {
	return strings.HasPrefix(contextName, "gke_")
}

// GetAccessToken gets an access token.
func GetAccessToken(ctx context.Context, authProject, flowstateAddr string, doFanOut bool) (string, error) {
	pc, err := auth.NewStore().GetConfiguration(authProject)
	if err != nil {
		return "", errors.Wrapf(err, "failed to get configuration for project %q", authProject)
	}
	key, err := pc.GetDefaultCredentials()
	if err != nil {
		return "", errors.Wrapf(err, "failed to get default API key for project %q", authProject)
	}
	resp, err := auth.GetIDToken(ctx, http.DefaultClient, flowstateAddr, &auth.GetIDTokenRequest{
		APIKey:   key.APIKey,
		DoFanOut: doFanOut,
	})
	if err != nil {
		return "", errors.Wrap(err, "failed to get ID token")
	}
	return resp.IDToken, nil
}

// AuthorizedContext provides a context update with the correct authorization
// metadata, if possible.  It will return the context unmodified and print a
// warning if it is unable to do so.  This is for use in situations where dial
// or call options are not feasible, such as using an insecure connection via
// port-forwarding or an SSH tunnel.
func AuthorizedContext(ctx context.Context, project string) context.Context {
	configuration, err := auth.NewStore().GetConfiguration(project)
	if err != nil {
		log.WarningContextf(ctx, "failed to get auth configuration: %v", err)
		return ctx
	}
	rpcCredentials, err := configuration.GetDefaultCredentials()
	if err != nil {
		log.WarningContextf(ctx, "failed to get credentials: %v", err)
		return ctx
	}
	credMD, err := rpcCredentials.GetRequestMetadata(ctx)
	if err != nil {
		log.WarningContextf(ctx, "failed to get credentials metadata: %v", err)
		return ctx
	}
	for k, v := range credMD {
		ctx = metadata.AppendToOutgoingContext(ctx, k, v)
	}
	return ctx
}

// CurrentUsername returns the current username.
func CurrentUsername() string {
	currentUser, err := user.Current()
	if err != nil {
		log.Warningf("failed to get user information: %v", err)
		return "unknown"
	}
	return currentUser.Username
}
