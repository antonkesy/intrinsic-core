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

// Package inspection contains logic around running Service Assets that expose
// service inspection topics.
package inspection

import (
	rtrpb "intrinsic/resources/proto/resource_type_runtime_go_proto"
)

const (
	// serviceInspectionTopicPrefix is the prefix for the service inspection topic.
	serviceInspectionTopicPrefix = "/service_inspection/services/"
)

// Topic returns the topic name for service inspection if the resource type
// runtime has a service inspection config.
func Topic(instanceName string, rtr *rtrpb.ResourceTypeRuntime) (string, bool) {
	if rtr.GetServiceDef().GetServiceInspectionConfig() != nil {
		return serviceInspectionTopicPrefix + instanceName, true
	}
	return "", false
}
