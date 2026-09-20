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

// Package collections lists the collection IDs used in Firestore.
package collections

const (
	// APIKeyNonce is the collection of (a single) nonce for API key generation.
	// keep-sorted start
	APIKeyNonce = "ApiKeyNonce"
	// APIKeys is the collection of generated API keys.
	APIKeys = "ApiKeys"
	// Applications is the collection containing applications (solutions).
	Applications = "Applications"
	// Clusters is the collection containing the compute-clusters.
	Clusters = "Clusters"
	// DHCPLeases is the collection containing the DHCP leases information about a cluster.
	DHCPLeases = "DHCPLeases"
	// Environments is the collection containing environments.
	Environments = "Environments"
	// Hardware is the collection containing the hardware information about a cluster.
	Hardware = "Hardware"
	// ResourceTypes is the collection containing resource types.
	ResourceTypes = "ResourceTypes"
	// Skills is the collection containing skills.
	Skills = "Skills"
	// keep-sorted end
)
