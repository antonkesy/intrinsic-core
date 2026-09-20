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
	"net/http"
	"sync"

	"github.com/gorilla/websocket"
	"github.com/grpc-ecosystem/grpc-gateway/v2/runtime"
	ice "github.com/pion/ice/v4"
	pion "github.com/pion/webrtc/v4"

	"intrinsic/httpjson/serialization"

	webpubsubpb "intrinsic/httpjson/proto/v1alpha/webpubsub_go_proto"
	"intrinsic/platform/pubsub/golang/pubsubinterface"
)

// ConnectionManager manages the shared WebRTC resources and active client connections.
type ConnectionManager struct {
	api            *pion.API
	udpMux         *ice.MultiUDPMuxDefault
	configProvider ConfigurationProvider
	localIPs       []string
	ps             pubsubinterface.PubSub
	marshaller     *runtime.JSONPb
	connectionsMu  sync.RWMutex
	connections    map[*Connection]bool
}

// NewConnectionManager creates a new ConnectionManager.
func NewConnectionManager(
	ctx context.Context,
	provider ConfigurationProvider,
	udpMuxPort int,
	ps pubsubinterface.PubSub,
	marshaller *runtime.JSONPb,
) (*ConnectionManager, error) {
	slog.InfoContext(ctx, "Initializing shared UDP multiplexer", "port", udpMuxPort)
	mux, err := ice.NewMultiUDPMuxFromPort(udpMuxPort)
	if err != nil {
		return nil, fmt.Errorf("failed to create UDP multiplexer on port %d: %w", udpMuxPort, err)
	}

	var settingEngine pion.SettingEngine
	settingEngine.SetICEUDPMux(mux)
	settingEngine.SetAnsweringDTLSRole(pion.DTLSRoleServer)
	settingEngine.SetICEMulticastDNSMode(ice.MulticastDNSModeDisabled)

	api := pion.NewAPI(pion.WithSettingEngine(settingEngine))

	localIPs := DiscoverHostIPs(ctx)
	slog.InfoContext(ctx, "Discovered local host IPs for WebRTC candidates", "ips", localIPs)

	return &ConnectionManager{
		api:            api,
		udpMux:         mux,
		configProvider: provider,
		localIPs:       localIPs,
		ps:             ps,
		marshaller:     marshaller,
		connections:    make(map[*Connection]bool),
	}, nil
}

// Close releases any shared resources like the UDP multiplexer.
func (m *ConnectionManager) Close() error {
	if m.udpMux != nil {
		return m.udpMux.Close()
	}
	return nil
}

// ListClientStats returns a snapshot of statistics for all active client connections.
func (m *ConnectionManager) ListClientStats() []*webpubsubpb.ConnectedClient {
	m.connectionsMu.RLock()
	defer m.connectionsMu.RUnlock()

	var stats []*webpubsubpb.ConnectedClient
	for conn := range m.connections {
		stats = append(stats, conn.GetStats())
	}
	return stats
}

// HandleControlConnection upgrades the HTTP connection to WebSocket and initiates WebRTC signaling.
func (m *ConnectionManager) HandleControlConnection(w http.ResponseWriter, r *http.Request, _ map[string]string) {
	ctx := r.Context()

	if r.Method != http.MethodGet {
		w.WriteHeader(http.StatusMethodNotAllowed)
		return
	}

	var upgrader websocket.Upgrader
	ws, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		slog.ErrorContext(ctx, "Failed to upgrade connection to WebSocket", "error", err)
		return
	}

	slog.InfoContext(ctx, "Successfully upgraded control connection to WebSocket")

	// Limit maximum web socket message size. 256 KiB should be very generous.
	ws.SetReadLimit(256 * 1024) // 256 KiB

	decoder := NewDecoder(serialization.RequestFormat(r), m.marshaller)
	encoder := NewEncoder(serialization.ResponseFormat(r), m.marshaller)

	conn, err := m.NewConnection(ctx, ws, decoder, encoder)
	if err != nil {
		slog.ErrorContext(ctx, "Failed to create WebRTC connection", "error", err)
		_ = ws.Close()
		return
	}

	// Run the connection orchestrator synchronously using the request context.
	conn.Start(ctx)
}

// NewConnection initializes a new PeerConnection and delegates internal component wiring to Connection.
func (m *ConnectionManager) NewConnection(ctx context.Context, ws *websocket.Conn, decoder *MessageDecoder, encoder *MessageEncoder) (*Connection, error) {
	var iceServers []pion.ICEServer
	if m.configProvider != nil {
		creds, err := m.configProvider.FetchTURNAuthCredentials(ctx)
		if err != nil {
			slog.WarnContext(ctx, "Failed to retrieve TURN credentials, using STUN only", "error", err)
		}
		iceServers = GetICEServers(m.configProvider.PortalDomain(), creds)
	} else {
		iceServers = GetICEServers("", nil)
	}

	webrtcConfig := pion.Configuration{
		ICEServers: iceServers,
	}

	pc, err := m.api.NewPeerConnection(webrtcConfig)
	if err != nil {
		return nil, fmt.Errorf("failed to create PeerConnection: %w", err)
	}

	var conn *Connection
	onClose := func() {
		m.connectionsMu.Lock()
		delete(m.connections, conn)
		m.connectionsMu.Unlock()
	}

	conn, err = NewConnection(pc, ws, m.localIPs, m.ps, decoder, encoder, onClose)
	if err != nil {
		pc.Close()
		return nil, fmt.Errorf("failed to create Connection: %w", err)
	}

	m.connectionsMu.Lock()
	m.connections[conn] = true
	m.connectionsMu.Unlock()

	return conn, nil
}
