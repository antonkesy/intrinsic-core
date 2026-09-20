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

	pion "github.com/pion/webrtc/v4"
	statuspb "google.golang.org/genproto/googleapis/rpc/status"
	"google.golang.org/grpc/codes"
	"google.golang.org/protobuf/types/known/anypb"

	"intrinsic/httpjson/proto/v1alpha/dataplane_go_proto"
	pubsubpb "intrinsic/platform/pubsub/adapters/pubsub_go_proto"
)

// DataPlaneListener defines high-level callbacks for data plane events.
type DataPlaneListener interface {
	OnClientPublish(ctx context.Context, dcLabel string, topic string, payload *anypb.Any)
}

// DataPlane manages the active WebRTC DataChannels and bridges them with the listener.
type DataPlane struct {
	encoder    *MessageEncoder
	decoder    *MessageDecoder
	stats      *dataPlaneStatistics
	listener   DataPlaneListener
	channelsMu sync.RWMutex
	channels   map[string]*pion.DataChannel
	chunker    *DataPacketChunker
	assembler  *DataPacketAssembler
}

// NewDataPlane creates a new DataPlane.
func NewDataPlane(encoder *MessageEncoder, decoder *MessageDecoder, stats *dataPlaneStatistics, listener DataPlaneListener) (*DataPlane, error) {
	chunker, err := NewDataPacketChunker(encoder)
	if err != nil {
		return nil, err
	}
	return &DataPlane{
		encoder:   encoder,
		decoder:   decoder,
		stats:     stats,
		listener:  listener,
		channels:  make(map[string]*pion.DataChannel),
		chunker:   chunker,
		assembler: NewDataPacketAssembler(decoder),
	}, nil
}

// SendPubSubMessage packages, chunks, and transmits a pubsub message over a DataChannel.
func (dp *DataPlane) SendPubSubMessage(dcLabel string, topic string, pubsubPacket *pubsubpb.PubSubPacket) error {
	dp.channelsMu.RLock()
	dc, exists := dp.channels[dcLabel]
	dp.channelsMu.RUnlock()

	if !exists {
		return fmt.Errorf("data channel %q not found", dcLabel)
	}

	packet := &dataplane_go_proto.DataPacket{
		Payload: &dataplane_go_proto.DataPacket_Message{
			Message: &dataplane_go_proto.PubSubMessage{
				Topic:        topic,
				PubsubPacket: pubsubPacket,
			},
		},
	}

	return dp.sendDataPacket(dc, packet)
}

// SendError packages and transmits an error message over a DataChannel.
func (dp *DataPlane) SendError(dcLabel string, code codes.Code, errMsg string) {
	dp.channelsMu.RLock()
	dc, exists := dp.channels[dcLabel]
	dp.channelsMu.RUnlock()

	if !exists {
		dp.stats.IncrementErrorsToClient()
		slog.Error("Failed to send error on data channel: channel not found", "label", dcLabel)
		return
	}

	dp.sendError(dc, code, errMsg)
}

func (dp *DataPlane) sendError(dc *pion.DataChannel, code codes.Code, errMsg string) {
	dp.stats.IncrementErrorsToClient()
	packet := dataplane_go_proto.DataPacket{
		Payload: &dataplane_go_proto.DataPacket_Error{
			Error: &statuspb.Status{
				Code:    int32(code),
				Message: errMsg,
			},
		},
	}
	if err := dp.sendDataPacket(dc, &packet); err != nil {
		slog.Error("Failed to send error packet on DataChannel", "error", err, "label", dc.Label())
	}
}

func (dp *DataPlane) sendDataPacket(dc *pion.DataChannel, packet *dataplane_go_proto.DataPacket) error {
	dp.stats.IncrementMessagesToClient()

	packets, err := dp.chunker.Chunk(packet)
	if err != nil {
		return err
	}

	for _, p := range packets {
		packetBytes, err := dp.encoder.Encode(p)
		if err != nil {
			return err
		}
		if err := dc.Send(packetBytes); err != nil {
			return err
		}
	}
	return nil
}

// HandleDataChannel registers callbacks on the newly established WebRTC data channel.
func (dp *DataPlane) HandleDataChannel(dc *pion.DataChannel) {
	label := dc.Label()
	slog.Info("New WebRTC Data Channel discovered", "label", label)

	dp.channelsMu.Lock()
	if existing, ok := dp.channels[label]; ok && existing != dc {
		dp.channelsMu.Unlock()
		slog.Error("Duplicate Data Channel label rejected", "label", label)
		dp.sendError(dc, codes.AlreadyExists, fmt.Sprintf("Data channel label %q already exists", label))
		if err := dc.Close(); err != nil {
			slog.Error("Failed to close duplicate DataChannel", "error", err, "label", label)
		}
		return
	}
	dp.channels[label] = dc
	dp.channelsMu.Unlock()

	dc.OnOpen(func() {
		slog.Info("Data Channel opened", "label", label)
	})

	dc.OnClose(func() {
		slog.Info("Data Channel closed", "label", label)
		dp.channelsMu.Lock()
		// Prevent a race condition where a channel is quickly recreated with the same label:
		// if the old channel's OnClose fires after the replacement channel has registered, an
		// unconditional delete would remove the active replacement channel from the map.
		if dp.channels[label] == dc {
			delete(dp.channels, label)
		}
		dp.channelsMu.Unlock()
	})

	dc.OnMessage(func(msg pion.DataChannelMessage) {
		dp.handleIncomingData(dc, msg.Data)
	})
}

func (dp *DataPlane) handleIncomingData(dc *pion.DataChannel, data []byte) {
	dp.stats.IncrementMessagesFromClient()

	fullData, err := dp.assembler.Assemble(data)
	if err != nil {
		slog.Error("DataChannel packet assembly failure", "error", err)
		dp.sendError(dc, codes.InvalidArgument, err.Error())
		return
	}
	if fullData == nil {
		return // Waiting for remaining chunks
	}

	var packet dataplane_go_proto.DataPacket
	if err := dp.decoder.Decode(fullData, &packet); err != nil {
		slog.Error("Failed to parse DataPacket", "error", err)
		dp.sendError(dc, codes.InvalidArgument, "Invalid data packet format")
		return
	}

	if packet.GetError() != nil {
		slog.Error("Client reported an error on DataChannel", "code", packet.GetError().GetCode(), "message", packet.GetError().GetMessage(), "label", dc.Label())
		dp.stats.IncrementErrorsFromClient()
		return
	}

	if packet.GetChunk() != nil {
		slog.Warn("Dropped reassembled packet containing a chunk payload")
		dp.sendError(dc, codes.InvalidArgument, "Nested chunking is not allowed")
		return
	}

	if packet.GetMessage() != nil {
		msg := packet.GetMessage()
		topic := msg.GetTopic()
		pubsubPacket := msg.GetPubsubPacket()
		if pubsubPacket == nil || pubsubPacket.Payload == nil {
			slog.Warn("Discarded published message: missing payload", "topic", topic)
			dp.sendError(dc, codes.InvalidArgument, "Missing payload in PubSubMessage")
			return
		}

		if dp.listener != nil {
			dp.listener.OnClientPublish(context.Background(), dc.Label(), topic, pubsubPacket.Payload)
		}
	}
}

// StartCleanupLoop runs the chunk assembler timeout cleanup loop.
func (dp *DataPlane) StartCleanupLoop(ctx context.Context) {
	dp.assembler.StartCleanupLoop(ctx)
}
