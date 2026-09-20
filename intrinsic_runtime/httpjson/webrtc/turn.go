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

// Package webrtc implements WebRTC signaling and data channel pub/sub transport for real-time client communication.
package webrtc

import (
	"context"
)

// ConfigurationProvider defines the interface for resolving TURN credentials and portal settings.
type ConfigurationProvider interface {
	FetchTURNAuthCredentials(ctx context.Context) (*TURNCredentials, error)
	PortalDomain() string
}

// TURNCredentials represents the transient credentials generated for coturn authentication.
type TURNCredentials struct {
	Username string `json:"username"`
	Password string `json:"password"`
}

// String implements fmt.Stringer to redact the password field.
func (c TURNCredentials) String() string {
	return "TURNCredentials{Username: " + c.Username + ", Password: <redacted>}"
}
