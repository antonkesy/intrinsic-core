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
	"fmt"
	"log/slog"
	"sync"

	"github.com/gorilla/websocket"
	pion "github.com/pion/webrtc/v4"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"
	"google.golang.org/protobuf/types/known/anypb"

	"intrinsic/httpjson/proto/v1alpha/controlplane_go_proto"
	webpubsubpb "intrinsic/httpjson/proto/v1alpha/webpubsub_go_proto"
	pubsubpb "intrinsic/platform/pubsub/adapters/pubsub_go_proto"
	"intrinsic/platform/pubsub/golang/pubsubinterface"
)

// Connection orchestrates the session lifecycle of a single WebRTC client connection.
// It implements the Mediator design pattern, coordinating all interactions between the
// transport layers (ControlPlane, DataPlane) and the subcomponents (SignalingHandler, PubSubAdapter).
type Connection struct {
	pc        *pion.PeerConnection
	mediator  *connectionMediator
	onClose   func()
	closeOnce sync.Once
	stats     *connectionStats
}

// NewConnection instantiates a Connection and wires up its control plane, data plane, and callbacks.
func NewConnection(
	pc *pion.PeerConnection,
	ws *websocket.Conn,
	localIPs []string,
	ps pubsubinterface.PubSub,
	decoder *MessageDecoder,
	encoder *MessageEncoder,
	onClose func(),
) (*Connection, error) {
	stats := &connectionStats{}

	conn := &Connection{
		pc:      pc,
		onClose: onClose,
		stats:   stats,
	}

	mediator := &connectionMediator{
		pubSubAdapter: NewPubSubAdapter(ps),
	}
	conn.mediator = mediator

	dp, err := NewDataPlane(encoder, decoder, &stats.dataPlane, mediator)
	if err != nil {
		conn.Close(context.Background())
		return nil, fmt.Errorf("failed to create data plane: %w", err)
	}
	mediator.dataPlane = dp

	// Create ControlPlane, passing mediator as the ControlPlaneListener
	mediator.controlPlane = NewControlPlane(ws, encoder, decoder, &stats.controlPlane, mediator)

	// Create SignalingHandler, passing mediator as the SignalHandlingListener
	mediator.signaling = NewSignalingHandler(pc, localIPs, mediator)

	// Register DataChannel creation callback
	pc.OnDataChannel(func(dc *pion.DataChannel) {
		mediator.dataPlane.HandleDataChannel(dc)
	})

	return conn, nil
}

// Start runs the control plane loop and ensures clean resource teardown on exit.
func (c *Connection) Start(ctx context.Context) {
	ctx, cancel := context.WithCancel(ctx)
	defer cancel()

	// Start data plane cleanup loop
	go c.mediator.dataPlane.StartCleanupLoop(ctx)
	go c.mediator.controlPlane.WriteLoop(ctx)

	defer c.Close(ctx)
	c.mediator.controlPlane.ReadLoop(ctx)
}

// Close cleanly terminates the PeerConnection and the WebSocket signaling channel.
func (c *Connection) Close(ctx context.Context) {
	c.closeOnce.Do(func() {
		slog.InfoContext(ctx, "Teardown: closing PeerConnection and WebSocket")

		if c.onClose != nil {
			c.onClose()
		}

		if c.mediator != nil && c.mediator.pubSubAdapter != nil {
			c.mediator.pubSubAdapter.Close()
		}

		if c.pc != nil {
			if err := c.pc.Close(); err != nil {
				slog.ErrorContext(ctx, "Error closing PeerConnection", "error", err)
			}
		}
		if c.mediator != nil && c.mediator.controlPlane != nil {
			if err := c.mediator.controlPlane.Close(); err != nil {
				slog.ErrorContext(ctx, "Error closing ControlPlane", "error", err)
			}
		}
	})
}

// GetStats returns the ConnectedClient statistics for this connection.
func (c *Connection) GetStats() *webpubsubpb.ConnectedClient {
	stats := &webpubsubpb.ConnectedClient{}
	c.mediator.pubSubAdapter.PopulateStats(stats)
	c.stats.FillStats(stats)
	return stats
}

// connectionMediator routes internal subcomponent events and holds the subcomponents
// that it mediates interactions between. Unexported so its methods and fields do not
// pollute the public API of Connection.
type connectionMediator struct {
	controlPlane  *ControlPlane
	dataPlane     *DataPlane
	pubSubAdapter *PubSubAdapter
	signaling     *SignalingHandler
}

// OnLocalCandidate handles trickling local ICE candidates to the control plane WebSocket.
func (m *connectionMediator) OnLocalCandidate(candidate *pion.ICECandidate) {
	if err := m.controlPlane.SendIceCandidate(context.Background(), candidate); err != nil {
		slog.Error("Failed to send ICE candidate over WebSocket", "error", err)
	}
}

// OnRemoteSessionDescription delegates remote SDP offer/answer handling to SignalingHandler.
func (m *connectionMediator) OnRemoteSessionDescription(ctx context.Context, desc pion.SessionDescription) {
	answer, err := m.signaling.OnRemoteSessionDescription(desc)
	if err != nil {
		slog.ErrorContext(ctx, "Failed to process remote session description", "error", err)
		m.controlPlane.SendError(ctx, codes.InvalidArgument, "Failed to process remote session description")
		return
	}
	if answer != nil {
		if err := m.controlPlane.SendSessionDescription(ctx, *answer); err != nil {
			slog.ErrorContext(ctx, "Failed to send local answer over WebSocket", "error", err)
		}
	}
}

// OnRemoteIceCandidate delegates remote ICE candidate handling to SignalingHandler.
func (m *connectionMediator) OnRemoteIceCandidate(ctx context.Context, candidate pion.ICECandidateInit) {
	if err := m.signaling.OnRemoteIceCandidate(candidate); err != nil {
		slog.ErrorContext(ctx, "Failed to process remote ICE candidate", "error", err)
		m.controlPlane.SendError(ctx, codes.InvalidArgument, "Failed to process remote ICE candidate")
	}
}

// OnClientPublish handles incoming client published messages routed from DataPlane.
func (m *connectionMediator) OnClientPublish(ctx context.Context, dcLabel string, topic string, payload *anypb.Any) {
	if err := m.pubSubAdapter.Publish(ctx, topic, payload); err != nil {
		slog.ErrorContext(ctx, "Failed to publish client message", "error", err, "topic", topic)
		m.dataPlane.SendError(dcLabel, status.Code(err), "Failed to publish message")
	}
}

// OnSubscribe binds a topic subscription to a specific DataChannel.
func (m *connectionMediator) OnSubscribe(ctx context.Context, msg *controlplane_go_proto.Subscribe) {
	err := m.pubSubAdapter.Subscribe(ctx, msg, func(pubsubPacket *pubsubpb.PubSubPacket) {
		if err := m.dataPlane.SendPubSubMessage(msg.GetDataChannel(), msg.GetTopic(), pubsubPacket); err != nil {
			slog.ErrorContext(ctx, "Failed to dispatch publish message to client", "error", err, "label", msg.GetDataChannel(), "topic", msg.GetTopic())
		}
	})
	if err != nil {
		slog.ErrorContext(ctx, "Failed to subscribe to topic", "error", err, "topic", msg.GetTopic())
		m.controlPlane.SendError(ctx, status.Code(err), "Failed to subscribe to topic")
	}
}

// OnUnsubscribe cleans up a topic subscription.
func (m *connectionMediator) OnUnsubscribe(ctx context.Context, msg *controlplane_go_proto.Unsubscribe) {
	if err := m.pubSubAdapter.Unsubscribe(ctx, msg); err != nil {
		slog.ErrorContext(ctx, "Failed to unsubscribe from topic", "error", err, "topic", msg.GetTopic())
		m.controlPlane.SendError(ctx, status.Code(err), "Failed to unsubscribe from topic")
	}
}

// OnAdvertisePublisher registers a publisher for client-to-server publishing.
func (m *connectionMediator) OnAdvertisePublisher(ctx context.Context, msg *controlplane_go_proto.AdvertisePublisher) {
	if err := m.pubSubAdapter.AdvertisePublisher(ctx, msg); err != nil {
		slog.ErrorContext(ctx, "Failed to advertise publisher", "error", err, "topic", msg.GetTopic())
		m.controlPlane.SendError(ctx, status.Code(err), "Failed to advertise publisher")
	}
}

// OnUnadvertisePublisher removes a publisher registration.
func (m *connectionMediator) OnUnadvertisePublisher(ctx context.Context, msg *controlplane_go_proto.UnadvertisePublisher) {
	if err := m.pubSubAdapter.UnadvertisePublisher(ctx, msg); err != nil {
		slog.ErrorContext(ctx, "Failed to unadvertise publisher", "error", err, "topic", msg.GetTopic())
		m.controlPlane.SendError(ctx, status.Code(err), "Failed to unadvertise publisher")
	}
}
