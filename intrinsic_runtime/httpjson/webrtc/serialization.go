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
	"github.com/gorilla/websocket"
	"github.com/grpc-ecosystem/grpc-gateway/v2/runtime"
	"google.golang.org/protobuf/proto"

	"intrinsic/httpjson/serialization"
)

// MessageDecoder decodes inbound byte payloads into protobuf messages based on the configured format.
type MessageDecoder struct {
	msgType   int
	unmarshal func(data []byte, msg proto.Message) error
}

// NewDecoder instantiates a MessageDecoder with the requested format and an optional custom JSON marshaler.
func NewDecoder(format serialization.Format, marshaller *runtime.JSONPb) *MessageDecoder {
	if format == serialization.FormatProto {
		return &MessageDecoder{
			msgType:   websocket.BinaryMessage,
			unmarshal: proto.Unmarshal,
		}
	}
	if marshaller == nil {
		marshaller = &runtime.JSONPb{}
	}
	return &MessageDecoder{
		msgType: websocket.TextMessage,
		unmarshal: func(data []byte, msg proto.Message) error {
			return marshaller.Unmarshal(data, msg)
		},
	}
}

// WebsocketMessageType returns websocket.BinaryMessage or websocket.TextMessage corresponding to the configured encoding format.
func (d *MessageDecoder) WebsocketMessageType() int {
	return d.msgType
}

// Decode decodes bytes into a protobuf message based on the configured format.
func (d *MessageDecoder) Decode(data []byte, msg proto.Message) error {
	return d.unmarshal(data, msg)
}

// MessageEncoder encodes protobuf messages into outbound byte payloads based on the configured format.
type MessageEncoder struct {
	msgType int
	marshal func(msg proto.Message) ([]byte, error)
}

// NewEncoder instantiates a MessageEncoder with the requested format and an optional custom JSON marshaler.
func NewEncoder(format serialization.Format, marshaller *runtime.JSONPb) *MessageEncoder {
	if format == serialization.FormatProto {
		return &MessageEncoder{
			msgType: websocket.BinaryMessage,
			marshal: proto.Marshal,
		}
	}
	if marshaller == nil {
		marshaller = &runtime.JSONPb{}
	}
	return &MessageEncoder{
		msgType: websocket.TextMessage,
		marshal: func(msg proto.Message) ([]byte, error) {
			return marshaller.Marshal(msg)
		},
	}
}

// WebsocketMessageType returns websocket.BinaryMessage or websocket.TextMessage corresponding to the configured encoding format.
func (e *MessageEncoder) WebsocketMessageType() int {
	return e.msgType
}

// Encode encodes a protobuf message into bytes based on the configured format.
func (e *MessageEncoder) Encode(msg proto.Message) ([]byte, error) {
	return e.marshal(msg)
}
