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
	"strings"
	"sync"
	"sync/atomic"

	"google.golang.org/protobuf/types/known/anypb"

	"intrinsic/httpjson/proto/v1alpha/controlplane_go_proto"
	webpubsubpb "intrinsic/httpjson/proto/v1alpha/webpubsub_go_proto"
	pubsubpb "intrinsic/platform/pubsub/adapters/pubsub_go_proto"
	"intrinsic/platform/pubsub/golang/pubsubinterface"
)

type activeSubscription struct {
	spec              *controlplane_go_proto.Subscribe
	typeMismatchDrops atomic.Uint64
	rateLimitDrops    atomic.Uint64
	messagesForwarded atomic.Uint64
}

// PubSubAdapter adapts Intrinsic's PubSub interface to a single WebRTC Connection.
// It manages topic subscriptions, active publishers, rate limiters, and payload type filtering.
type PubSubAdapter struct {
	ps         pubsubinterface.PubSub
	mu         sync.Mutex
	subs       map[string]pubsubinterface.Subscription // subKey -> Subscription
	publishers map[string]pubsubinterface.Publisher    // topic -> Publisher
	activeSubs map[string]*activeSubscription
	activePubs map[string]*controlplane_go_proto.AdvertisePublisher
}

// NewPubSubAdapter instantiates a new PubSubAdapter.
func NewPubSubAdapter(ps pubsubinterface.PubSub) *PubSubAdapter {
	return &PubSubAdapter{
		ps:         ps,
		subs:       make(map[string]pubsubinterface.Subscription),
		publishers: make(map[string]pubsubinterface.Publisher),
		activeSubs: make(map[string]*activeSubscription),
		activePubs: make(map[string]*controlplane_go_proto.AdvertisePublisher),
	}
}

// Subscribe registers a new subscription on the global PubSub service.
// It applies rate-limiting and payload-filtering, and invokes onMessageReceived when a message is received.
func (a *PubSubAdapter) Subscribe(
	ctx context.Context,
	msg *controlplane_go_proto.Subscribe,
	onMessageReceived func(*pubsubpb.PubSubPacket),
) error {
	a.mu.Lock()
	defer a.mu.Unlock()

	topic := msg.GetTopic()
	if topic == "" {
		return fmt.Errorf("invalid topic name: cannot be empty")
	}

	expectedType := msg.GetType()
	subKey := topic
	if expectedType != "" {
		subKey = topic + "|" + expectedType
	}
	if _, exists := a.subs[subKey]; exists {
		return fmt.Errorf("already subscribed to topic %q with type %q", topic, expectedType)
	}

	rateLimit := msg.GetRateLimit()
	if rateLimit != nil {
		mps := rateLimit.GetMessagesPerSecond()
		capVal := rateLimit.GetTokenBucketCapacity()
		if (mps == 0 && capVal > 0) || (mps > 0 && capVal == 0) {
			return fmt.Errorf("invalid rate limit: messages_per_second and token_bucket_capacity must either both be zero, or both be greater than zero")
		}
	}

	var tb *tokenBucket
	if rateLimit != nil && rateLimit.MessagesPerSecond > 0 && rateLimit.TokenBucketCapacity > 0 {
		tb = newTokenBucket(rateLimit.MessagesPerSecond, rateLimit.TokenBucketCapacity)
	}

	activeSub := &activeSubscription{
		spec: msg,
	}

	config := pubsubinterface.TopicConfig{Qos: pubsubinterface.HighReliability}
	rawSub, err := a.ps.NewRawSubscription(topic, config, func(pubsubPacket *pubsubpb.PubSubPacket) {
		if tb != nil && !tb.allow() {
			activeSub.rateLimitDrops.Add(1)
			return
		}

		// Filter by type if expectedType is specified
		if expectedType != "" {
			if pubsubPacket.Payload == nil || !matchesType(pubsubPacket.Payload.TypeUrl, expectedType) {
				activeSub.typeMismatchDrops.Add(1)
				return // Drop message due to type mismatch
			}
		}

		activeSub.messagesForwarded.Add(1)
		onMessageReceived(pubsubPacket)
	})
	if err != nil {
		return fmt.Errorf("failed to subscribe to topic %q: %w", topic, err)
	}

	a.subs[subKey] = rawSub
	a.activeSubs[subKey] = activeSub

	slog.InfoContext(ctx, "Subscribed client WebRTC connection to topic", "topic", topic, "channel", msg.GetDataChannel(), "type", expectedType)
	return nil
}

// Unsubscribe closes and removes an active topic subscription.
func (a *PubSubAdapter) Unsubscribe(ctx context.Context, msg *controlplane_go_proto.Unsubscribe) error {
	a.mu.Lock()
	defer a.mu.Unlock()

	topic := msg.GetTopic()
	if topic == "" {
		return fmt.Errorf("invalid topic name: cannot be empty")
	}

	expectedType := msg.GetType()
	subKey := topic
	if expectedType != "" {
		subKey = topic + "|" + expectedType
	}

	sub, exists := a.subs[subKey]
	if !exists {
		return fmt.Errorf("no subscription found for topic %q with type %q", topic, expectedType)
	}

	sub.Close()
	delete(a.subs, subKey)
	delete(a.activeSubs, subKey)

	slog.InfoContext(ctx, "Unsubscribed client WebRTC connection from topic", "topic", topic, "type", expectedType)
	return nil
}

// AdvertisePublisher creates a new publisher on the global PubSub service.
func (a *PubSubAdapter) AdvertisePublisher(ctx context.Context, msg *controlplane_go_proto.AdvertisePublisher) error {
	a.mu.Lock()
	defer a.mu.Unlock()

	topic := msg.GetTopic()
	if topic == "" {
		return fmt.Errorf("invalid topic name: cannot be empty")
	}

	if _, exists := a.publishers[topic]; exists {
		return fmt.Errorf("publisher already advertised for topic %q", topic)
	}

	config := pubsubinterface.TopicConfig{Qos: pubsubinterface.HighReliability}
	pub, err := a.ps.NewPublisher(topic, config)
	if err != nil {
		return fmt.Errorf("failed to advertise publisher for topic %q: %w", topic, err)
	}

	a.publishers[topic] = pub
	a.activePubs[topic] = msg

	slog.InfoContext(ctx, "Registered publisher for topic", "topic", topic)
	return nil
}

// UnadvertisePublisher closes and removes a registered publisher.
func (a *PubSubAdapter) UnadvertisePublisher(ctx context.Context, msg *controlplane_go_proto.UnadvertisePublisher) error {
	a.mu.Lock()
	defer a.mu.Unlock()

	topic := msg.GetTopic()
	if topic == "" {
		return fmt.Errorf("invalid topic name: cannot be empty")
	}

	pub, exists := a.publishers[topic]
	if !exists {
		return fmt.Errorf("no publisher advertised for topic %q", topic)
	}

	pub.Close()
	delete(a.publishers, topic)
	delete(a.activePubs, topic)

	slog.InfoContext(ctx, "Unregistered publisher for topic", "topic", topic)
	return nil
}

// Publish publishes a payload to a registered topic publisher.
func (a *PubSubAdapter) Publish(ctx context.Context, topic string, payload *anypb.Any) error {
	a.mu.Lock()
	pub := a.publishers[topic]
	a.mu.Unlock()

	if pub == nil {
		slog.WarnContext(ctx, "Discarded published message: no registered publisher for topic", "topic", topic)
		return fmt.Errorf("no registered publisher for topic %q", topic)
	}

	if err := pub.PublishAny(payload); err != nil {
		slog.ErrorContext(ctx, "Failed to publish Any message for topic", "error", err, "topic", topic)
		return fmt.Errorf("failed to publish message: %w", err)
	}

	return nil
}

// Close cleanly closes all active subscriptions and publishers.
func (a *PubSubAdapter) Close() {
	a.mu.Lock()
	defer a.mu.Unlock()

	for _, sub := range a.subs {
		sub.Close()
	}
	a.subs = nil

	for _, pub := range a.publishers {
		pub.Close()
	}
	a.publishers = nil
}

// PopulateStats appends active subscriptions and publishers to the client stats protobuf model.
func (a *PubSubAdapter) PopulateStats(stats *webpubsubpb.ConnectedClient) {
	a.mu.Lock()
	defer a.mu.Unlock()

	for _, sub := range a.activeSubs {
		stats.ActiveSubscriptions = append(stats.ActiveSubscriptions, &webpubsubpb.ActiveSubscription{
			Spec: sub.spec,
			Statistics: &webpubsubpb.SubscriptionStatistics{
				TypeMismatchDropsCount: sub.typeMismatchDrops.Load(),
				RateLimitDropsCount:    sub.rateLimitDrops.Load(),
				MessagesForwardedCount: sub.messagesForwarded.Load(),
			},
		})
	}

	for _, pub := range a.activePubs {
		stats.ActivePublishers = append(stats.ActivePublishers, pub)
	}
}

func matchesType(typeURL string, expectedType string) bool {
	if expectedType == "" {
		return true
	}
	return typeURL == expectedType || strings.HasSuffix(typeURL, "/"+expectedType)
}
