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
	statuspb "google.golang.org/genproto/googleapis/rpc/status"
	"google.golang.org/grpc/codes"

	"intrinsic/httpjson/proto/v1alpha/controlplane_go_proto"
)

// ControlPlaneListener defines callbacks for incoming signaling and control events.
type ControlPlaneListener interface {
	OnRemoteSessionDescription(ctx context.Context, desc pion.SessionDescription)
	OnRemoteIceCandidate(ctx context.Context, candidate pion.ICECandidateInit)
	OnSubscribe(ctx context.Context, msg *controlplane_go_proto.Subscribe)
	OnUnsubscribe(ctx context.Context, msg *controlplane_go_proto.Unsubscribe)
	OnAdvertisePublisher(ctx context.Context, msg *controlplane_go_proto.AdvertisePublisher)
	OnUnadvertisePublisher(ctx context.Context, msg *controlplane_go_proto.UnadvertisePublisher)
}

// ControlPlane manages the WebSocket signaling connection.
type ControlPlane struct {
	ws        *websocket.Conn
	sendChan  chan *controlplane_go_proto.ControlPacket
	doneChan  chan struct{}
	closeOnce sync.Once
	listener  ControlPlaneListener
	encoder   *MessageEncoder
	decoder   *MessageDecoder
	stats     *controlPlaneStatistics
}

// NewControlPlane creates a new ControlPlane.
func NewControlPlane(
	ws *websocket.Conn,
	encoder *MessageEncoder,
	decoder *MessageDecoder,
	stats *controlPlaneStatistics,
	listener ControlPlaneListener,
) *ControlPlane {
	return &ControlPlane{
		ws: ws,
		// I picked this number arbitrarily. This needs to handle the http-gateway sending a
		// burst of ICE candidates, or, a burst of errors in response to invalid client requests.
		// I expect less than 10 ice candidates, and normal operation zero errors.
		sendChan: make(chan *controlplane_go_proto.ControlPacket, 64),
		doneChan: make(chan struct{}),
		encoder:  encoder,
		decoder:  decoder,
		stats:    stats,
		listener: listener,
	}
}

// Close signals the write loop to exit and closes the WebSocket connection.
func (cp *ControlPlane) Close() error {
	var err error
	cp.closeOnce.Do(func() {
		close(cp.doneChan)
		if cp.ws != nil {
			err = cp.ws.Close()
		}
	})
	return err
}

// Serializes outbound WebSocket writes onto a single goroutine to satisfy thread-safety requirements without mutex contention across callers.
func (cp *ControlPlane) WriteLoop(ctx context.Context) {
	defer cp.Close()

	for {
		select {
		case <-ctx.Done():
			return
		case <-cp.doneChan:
			return
		case pkt, ok := <-cp.sendChan:
			if !ok {
				return
			}
			bytes, err := cp.encoder.Encode(pkt)
			if err != nil {
				slog.ErrorContext(ctx, "Failed to marshal outbound ControlPacket", "error", err)
				continue
			}
			cp.stats.IncrementMessagesToClient()
			if err := cp.ws.WriteMessage(cp.encoder.WebsocketMessageType(), bytes); err != nil {
				slog.ErrorContext(ctx, "WebSocket write failure in WriteLoop", "error", err)
				return
			}
		}
	}
}

func (cp *ControlPlane) writePacket(ctx context.Context, pkt *controlplane_go_proto.ControlPacket) error {
	select {
	case <-ctx.Done():
		return fmt.Errorf("write packet cancelled: %w", ctx.Err())
	case <-cp.doneChan:
		return fmt.Errorf("websocket connection is closed")
	case cp.sendChan <- pkt:
		return nil
	}
}

// SendError sends an error packet to the client.
func (cp *ControlPlane) SendError(ctx context.Context, code codes.Code, errMsg string) {
	cp.stats.IncrementErrorsToClient()
	pkt := &controlplane_go_proto.ControlPacket{
		Payload: &controlplane_go_proto.ControlPacket_Error{
			Error: &statuspb.Status{
				Code:    int32(code),
				Message: errMsg,
			},
		},
	}
	if err := cp.writePacket(ctx, pkt); err != nil {
		slog.ErrorContext(ctx, "Failed to write error packet to WebSocket", "error", err)
	}
}

// SendSessionDescription sends an SDP session description to the client.
func (cp *ControlPlane) SendSessionDescription(ctx context.Context, pionDesc pion.SessionDescription) error {
	pbDesc, err := ToProtoSessionDescription(pionDesc)
	if err != nil {
		return fmt.Errorf("translate session description: %w", err)
	}
	pkt := &controlplane_go_proto.ControlPacket{
		Payload: &controlplane_go_proto.ControlPacket_SessionDescription{
			SessionDescription: pbDesc,
		},
	}
	return cp.writePacket(ctx, pkt)
}

// SendIceCandidate sends an ICE candidate to the client.
func (cp *ControlPlane) SendIceCandidate(ctx context.Context, pionCand *pion.ICECandidate) error {
	if pionCand == nil {
		return fmt.Errorf("local ICE candidate is nil")
	}
	pbCand, err := ToProtoIceCandidate(*pionCand)
	if err != nil {
		return fmt.Errorf("translate ICE candidate: %w", err)
	}
	pkt := &controlplane_go_proto.ControlPacket{
		Payload: &controlplane_go_proto.ControlPacket_IceCandidate{
			IceCandidate: pbCand,
		},
	}
	return cp.writePacket(ctx, pkt)
}

// ReadLoop runs the WebSocket read loop, processing incoming signaling packets.
func (cp *ControlPlane) ReadLoop(ctx context.Context) {
	for {
		messageType, bytes, err := cp.ws.ReadMessage()
		if err != nil {
			if websocket.IsUnexpectedCloseError(err, websocket.CloseNormalClosure, websocket.CloseGoingAway) {
				slog.ErrorContext(ctx, "WebSocket read error", "error", err)
			} else {
				slog.InfoContext(ctx, "WebSocket connection closed cleanly")
			}
			return
		}

		if messageType != cp.decoder.WebsocketMessageType() {
			slog.WarnContext(ctx, "Received unexpected message type over WebSocket, ignoring", "type", messageType)
			continue
		}

		cp.stats.IncrementMessagesFromClient()

		var pkt controlplane_go_proto.ControlPacket
		if err := cp.decoder.Decode(bytes, &pkt); err != nil {
			slog.ErrorContext(ctx, "Failed to unmarshal ControlPacket", "error", err)
			cp.SendError(ctx, codes.InvalidArgument, "Invalid WebSocket control packet format")
			continue
		}

		switch p := pkt.Payload.(type) {
		case *controlplane_go_proto.ControlPacket_SessionDescription:
			sdpProto := p.SessionDescription
			pionDesc, err := ToPionSessionDescription(sdpProto)
			if err != nil {
				slog.ErrorContext(ctx, "Failed to translate remote session description", "error", err)
				cp.SendError(ctx, codes.InvalidArgument, "Invalid session description format")
				continue
			}
			if cp.listener != nil {
				cp.listener.OnRemoteSessionDescription(ctx, pionDesc)
			}

		case *controlplane_go_proto.ControlPacket_IceCandidate:
			candProto := p.IceCandidate
			pionCandInit, err := ToPionIceCandidate(candProto)
			if err != nil {
				slog.ErrorContext(ctx, "Failed to translate remote ICE candidate", "error", err)
				continue
			}
			if cp.listener != nil {
				cp.listener.OnRemoteIceCandidate(ctx, pionCandInit)
			}

		case *controlplane_go_proto.ControlPacket_Subscribe:
			sub := p.Subscribe
			if sub == nil {
				slog.ErrorContext(ctx, "Subscribe payload is empty")
				continue
			}
			if cp.listener != nil {
				cp.listener.OnSubscribe(ctx, sub)
			}

		case *controlplane_go_proto.ControlPacket_Unsubscribe:
			unsub := p.Unsubscribe
			if unsub == nil {
				slog.ErrorContext(ctx, "Unsubscribe payload is empty")
				continue
			}
			if cp.listener != nil {
				cp.listener.OnUnsubscribe(ctx, unsub)
			}

		case *controlplane_go_proto.ControlPacket_AdvertisePublisher:
			adv := p.AdvertisePublisher
			if adv == nil {
				slog.ErrorContext(ctx, "AdvertisePublisher payload is empty")
				continue
			}
			if cp.listener != nil {
				cp.listener.OnAdvertisePublisher(ctx, adv)
			}

		case *controlplane_go_proto.ControlPacket_UnadvertisePublisher:
			unadv := p.UnadvertisePublisher
			if unadv == nil {
				slog.ErrorContext(ctx, "UnadvertisePublisher payload is empty")
				continue
			}
			if cp.listener != nil {
				cp.listener.OnUnadvertisePublisher(ctx, unadv)
			}

		case *controlplane_go_proto.ControlPacket_Error:
			slog.ErrorContext(ctx, "Received client-reported control plane error", "code", p.Error.GetCode(), "message", p.Error.GetMessage())
			cp.stats.IncrementErrorsFromClient()

		default:
			slog.WarnContext(ctx, "Received empty or unrecognized control plane packet payload")
		}
	}
}

// FillStats populates control plane statistics.
func (cp *ControlPlane) FillStats(stats *controlplane_go_proto.ControlPacket) {
	// Not used directly, but we can read from cp's atomic counters in Connection.GetStats()
}
