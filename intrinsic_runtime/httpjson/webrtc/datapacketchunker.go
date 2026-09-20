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
	"encoding/base64"
	"fmt"
	"math/rand"
	"strings"
	"time"

	"intrinsic/httpjson/proto/v1alpha/dataplane_go_proto"
)

// maxChunksPerMessage is the maximum number of chunks allowed for a single message.
const maxChunksPerMessage = 3200

// maxChunkSizeBytes is the maximum size allowed for a single WebRTC data packet.
const maxChunkSizeBytes = 16 * 1024

// DataPacketChunker partitions large DataPacket payloads into small, transmittable chunks.
type DataPacketChunker struct {
	encoder   *MessageEncoder
	chunkSize int
}

// NewDataPacketChunker creates a new DataPacketChunker.
func NewDataPacketChunker(encoder *MessageEncoder) (*DataPacketChunker, error) {
	chunkSize, err := calculateOptimalChunkSize(encoder)
	if err != nil {
		return nil, err
	}
	return &DataPacketChunker{
		encoder:   encoder,
		chunkSize: chunkSize,
	}, nil
}

// Chunk partitions a DataPacket into one or more DataPacket chunks if it exceeds the 16 KiB limit.
// It returns a slice of DataPacket packets that are each guaranteed to serialize to <= 16 KiB.
func (mc *DataPacketChunker) Chunk(packet *dataplane_go_proto.DataPacket) ([]*dataplane_go_proto.DataPacket, error) {
	packetBytes, err := mc.encoder.Encode(packet)
	if err != nil {
		return nil, err
	}

	if len(packetBytes) <= maxChunkSizeBytes {
		return []*dataplane_go_proto.DataPacket{packet}, nil
	}

	b64Data := base64.StdEncoding.EncodeToString(packetBytes)
	b64Bytes := []byte(b64Data)

	id := fmt.Sprintf("pkt_%d_%d", time.Now().UnixNano(), rand.Int63())

	chunkSize := mc.chunkSize
	numChunks := (len(b64Bytes) + chunkSize - 1) / chunkSize

	if numChunks > maxChunksPerMessage {
		return nil, fmt.Errorf("message size too large to send: %d chunks exceeds limit of %d", numChunks, maxChunksPerMessage)
	}

	var packets []*dataplane_go_proto.DataPacket
	for i := 0; i < len(b64Bytes); i += chunkSize {
		end := i + chunkSize
		if end > len(b64Bytes) {
			end = len(b64Bytes)
		}
		chunkIndex := (i / chunkSize) + 1

		chunkPacket := &dataplane_go_proto.DataPacket{
			Payload: &dataplane_go_proto.DataPacket_Chunk{
				Chunk: &dataplane_go_proto.DataPacketChunk{
					Id:    id,
					Data:  string(b64Bytes[i:end]),
					Num:   uint32(chunkIndex),
					Total: uint32(numChunks),
				},
			},
		}
		packets = append(packets, chunkPacket)
	}

	return packets, nil
}

// calculateOptimalChunkSize calculates the optimal chunk size once based on the worst-case serialization overhead.
func calculateOptimalChunkSize(encoder *MessageEncoder) (int, error) {
	// Generate a worst-case ID matching the maximum allowed ID length.
	worstCaseID := strings.Repeat("a", maxChunkIDLength)
	dummyPacket := dataplane_go_proto.DataPacket{
		Payload: &dataplane_go_proto.DataPacket_Chunk{
			Chunk: &dataplane_go_proto.DataPacketChunk{
				Id:    worstCaseID,
				Num:   maxChunksPerMessage,
				Total: maxChunksPerMessage,
			},
		},
	}

	dummyBytes, err := encoder.Encode(&dummyPacket)
	if err != nil {
		return 0, fmt.Errorf("marshal dummy packet: %w", err)
	}

	// The "data" field key/tag overhead (e.g. `,"data":""` in JSON) is at most 16 bytes.
	calculatedSize := maxChunkSizeBytes - len(dummyBytes) - 16
	if calculatedSize <= 0 {
		return 0, fmt.Errorf("calculated chunk size %d is invalid (overhead %d is too large)", calculatedSize, len(dummyBytes))
	}

	return calculatedSize, nil
}
