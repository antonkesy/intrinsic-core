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
	"encoding/base64"
	"fmt"
	"log/slog"
	"strings"
	"sync"
	"time"

	"intrinsic/httpjson/proto/v1alpha/dataplane_go_proto"
)

// chunkAssemblyTimeout is the maximum time allowed to reassemble all chunks of a packet.
const chunkAssemblyTimeout = 3 * time.Second

// maxChunkIDLength is the maximum allowed length for a chunk ID.
const maxChunkIDLength = 64

// pendingPacket manages the fragments of a DataPacket being reassembled.
type pendingPacket struct {
	chunks    map[int]string
	total     int
	timestamp time.Time
}

// DataPacketAssembler aggregates incoming chunks, handles validation, and manages timeouts.
type DataPacketAssembler struct {
	decoder *MessageDecoder
	mu      sync.Mutex
	buffers map[string]*pendingPacket
}

// NewDataPacketAssembler creates a new DataPacketAssembler.
func NewDataPacketAssembler(decoder *MessageDecoder) *DataPacketAssembler {
	return &DataPacketAssembler{
		decoder: decoder,
		buffers: make(map[string]*pendingPacket),
	}
}

// Assemble accepts an incoming slice of bytes, parses it, and returns the fully reassembled packet if complete.
// If the incoming packet is not a chunk, it returns the input data directly.
// If it is a chunk and assembly is incomplete, it returns nil, nil.
func (ma *DataPacketAssembler) Assemble(data []byte) ([]byte, error) {
	var packet dataplane_go_proto.DataPacket
	if err := ma.decoder.Decode(data, &packet); err != nil {
		return nil, err
	}

	chunk := packet.GetChunk()
	if chunk == nil || chunk.GetId() == "" {
		return data, nil
	}

	if len(chunk.GetId()) > maxChunkIDLength {
		return nil, fmt.Errorf("chunk ID length %d exceeds maximum limit of %d characters", len(chunk.GetId()), maxChunkIDLength)
	}

	if chunk.GetTotal() > maxChunksPerMessage {
		return nil, fmt.Errorf("chunk total %d exceeds maximum limit of %d chunks", chunk.GetTotal(), maxChunksPerMessage)
	}

	if chunk.GetNum() < 1 || chunk.GetNum() > chunk.GetTotal() {
		return nil, fmt.Errorf("chunk number %d is out of range [1, %d]", chunk.GetNum(), chunk.GetTotal())
	}

	ma.mu.Lock()
	defer ma.mu.Unlock()

	pkt, exists := ma.buffers[chunk.GetId()]
	if !exists {
		pkt = &pendingPacket{
			chunks:    make(map[int]string),
			total:     int(chunk.GetTotal()),
			timestamp: time.Now(),
		}
		ma.buffers[chunk.GetId()] = pkt
	} else if pkt.total != int(chunk.GetTotal()) {
		return nil, fmt.Errorf("chunk total %d does not match previous total %d for packet %s", chunk.GetTotal(), pkt.total, chunk.GetId())
	}

	pkt.chunks[int(chunk.GetNum())] = chunk.GetData()

	if len(pkt.chunks) == pkt.total {
		var sb strings.Builder
		for i := 1; i <= pkt.total; i++ {
			sb.WriteString(pkt.chunks[i])
		}
		delete(ma.buffers, chunk.GetId())

		decoded, err := base64.StdEncoding.DecodeString(sb.String())
		if err != nil {
			return nil, fmt.Errorf("failed to decode base64 payloads: %w", err)
		}
		return decoded, nil
	}

	return nil, nil
}

// StartCleanupLoop runs the assembler timeout cleanup loop.
func (ma *DataPacketAssembler) StartCleanupLoop(ctx context.Context) {
	ticker := time.NewTicker(1 * time.Second)
	defer ticker.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
			ma.sweep(ctx, time.Now())
		}
	}
}

func (ma *DataPacketAssembler) sweep(ctx context.Context, now time.Time) {
	ma.mu.Lock()
	defer ma.mu.Unlock()
	for id, pkt := range ma.buffers {
		if now.Sub(pkt.timestamp) > chunkAssemblyTimeout {
			slog.WarnContext(ctx, "Dropped incomplete packet due to chunk reassembly timeout", "packet_id", id)
			delete(ma.buffers, id)
		}
	}
}
