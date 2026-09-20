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
	"sync/atomic"

	webpubsubpb "intrinsic/httpjson/proto/v1alpha/webpubsub_go_proto"
)

type controlPlaneStatistics struct {
	messagesToClient   uint64
	messagesFromClient uint64
	errorsToClient     uint64
	errorsFromClient   uint64
}

// IncrementMessagesToClient increments the count of control plane messages sent to the client.
func (s *controlPlaneStatistics) IncrementMessagesToClient() {
	atomic.AddUint64(&s.messagesToClient, 1)
}

// IncrementMessagesFromClient increments the count of control plane messages received from the client.
func (s *controlPlaneStatistics) IncrementMessagesFromClient() {
	atomic.AddUint64(&s.messagesFromClient, 1)
}

// IncrementErrorsToClient increments the count of explicit error messages sent to the client (packets containing an error payload).
func (s *controlPlaneStatistics) IncrementErrorsToClient() { atomic.AddUint64(&s.errorsToClient, 1) }

// IncrementErrorsFromClient increments the count of explicit error messages received from the client (where the client populated the error payload).
func (s *controlPlaneStatistics) IncrementErrorsFromClient() {
	atomic.AddUint64(&s.errorsFromClient, 1)
}

type dataPlaneStatistics struct {
	messagesToClient   uint64
	messagesFromClient uint64
	errorsToClient     uint64
	errorsFromClient   uint64
}

// IncrementMessagesToClient increments the count of data plane messages sent to the client.
func (s *dataPlaneStatistics) IncrementMessagesToClient() { atomic.AddUint64(&s.messagesToClient, 1) }

// IncrementMessagesFromClient increments the count of data plane messages received from the client.
func (s *dataPlaneStatistics) IncrementMessagesFromClient() {
	atomic.AddUint64(&s.messagesFromClient, 1)
}

// IncrementErrorsToClient increments the count of explicit error messages sent to the client (packets containing an error payload).
func (s *dataPlaneStatistics) IncrementErrorsToClient() { atomic.AddUint64(&s.errorsToClient, 1) }

// IncrementErrorsFromClient increments the count of explicit error messages received from the client (where the client populated the error payload).
func (s *dataPlaneStatistics) IncrementErrorsFromClient() { atomic.AddUint64(&s.errorsFromClient, 1) }

// connectionStats encapsulates all statistics for a WebRTC session in a thread-safe manner.
type connectionStats struct {
	controlPlane controlPlaneStatistics
	dataPlane    dataPlaneStatistics
}

// FillStats populates the protobuf representation of the connection stats.
func (s *connectionStats) FillStats(pbStats *webpubsubpb.ConnectedClient) {
	pbStats.ControlPlaneStatistics = &webpubsubpb.ControlPlaneStatistics{
		MessagesToClientCount:   atomic.LoadUint64(&s.controlPlane.messagesToClient),
		MessagesFromClientCount: atomic.LoadUint64(&s.controlPlane.messagesFromClient),
		ErrorsToClientCount:     atomic.LoadUint64(&s.controlPlane.errorsToClient),
		ErrorsFromClientCount:   atomic.LoadUint64(&s.controlPlane.errorsFromClient),
	}
	pbStats.DataPlaneStatistics = &webpubsubpb.DataPlaneStatistics{
		MessagesToClientCount:   atomic.LoadUint64(&s.dataPlane.messagesToClient),
		MessagesFromClientCount: atomic.LoadUint64(&s.dataPlane.messagesFromClient),
		ErrorsToClientCount:     atomic.LoadUint64(&s.dataPlane.errorsToClient),
		ErrorsFromClientCount:   atomic.LoadUint64(&s.dataPlane.errorsFromClient),
	}
}
