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

// Package handles contains functions that generate a resource handle from a
// resource instance.
package handles

import (
	"fmt"
	"slices"
	"strings"

	log "github.com/golang/glog"
	"github.com/pkg/errors"
	"google.golang.org/grpc/codes"
	grpcstatus "google.golang.org/grpc/status"

	ftppb "intrinsic/icon/control/parts/hal/force_torque_sensor_part/hal_force_torque_sensor_part_config_go_proto"
	fcpb "intrinsic/icon/equipment/force_control_settings_go_proto"
	iepb "intrinsic/icon/equipment/icon_equipment_go_proto"
	ipb "intrinsic/icon/server/config/icon_main_config_go_proto"
	ccv1pb "intrinsic/perception/proto/v1/camera_config_go_proto"
	rhpb "intrinsic/resources/proto/resource_handle_go_proto"
	rrpb "intrinsic/resources/proto/resource_registry_go_proto"

	anypb "google.golang.org/protobuf/types/known/anypb"
)

const (
	// InstanceHeader should be added to gRPC calls made through the ingress
	// service to reach the resource's service.
	InstanceHeader = `x-resource-instance-name`

	// Resource's service proto prefixes are expected in the format: '/proto.package.service_name/'
	serviceProtoPrefixMarker = "/"
)

// strippedServiceProtoPrefix returns a service proto prefix without the leading
// and trailing `serviceProtoPrefixMarker`.
func strippedServiceProtoPrefix(srv string) string {
	if strings.HasPrefix(srv, serviceProtoPrefixMarker) {
		srv = srv[len(serviceProtoPrefixMarker):]
	}
	if strings.HasSuffix(srv, serviceProtoPrefixMarker) {
		srv = srv[:len(srv)-len(serviceProtoPrefixMarker)]
	}
	return srv
}

// validServiceProtoPrefix returns a valid service proto prefix, i.e. with
// leading and trailing serviceProtoPrefixMarker.
func validServiceProtoPrefix(srv string) string {
	return serviceProtoPrefixMarker + strippedServiceProtoPrefix(srv) + serviceProtoPrefixMarker
}

// validateServiceProtoPrefix returns an error if the given service proto prefix does not have
// leading and trailing serviceProtoPrefixMarker.
func validateServiceProtoPrefix(srv string) error {
	if strings.HasPrefix(srv, serviceProtoPrefixMarker) && strings.HasSuffix(srv, serviceProtoPrefixMarker) {
		return nil
	}
	return fmt.Errorf("expected service definition like %q, got: %q", validServiceProtoPrefix(srv), srv)
}

// getServicePrefixesAsEquipmentData returns resource type's service proto prefixes as the resource data.
func getServicePrefixesAsEquipmentData(servicePrefixes []string) (map[string]*rhpb.ResourceHandle_ResourceData, error) {
	if len(servicePrefixes) == 0 {
		return nil, nil
	}

	equipData := make(map[string]*rhpb.ResourceHandle_ResourceData)
	for _, srv := range servicePrefixes {
		if err := validateServiceProtoPrefix(srv); err != nil {
			return nil, err
		}
		equipData[strippedServiceProtoPrefix(srv)] = nil
	}

	return equipData, nil
}

// extractResourceDataFromIconConfig extracts a resource handle map from the icon main config.
func extractResourceDataFromIconConfig(resourceName string, iconMainConfig *ipb.IconMainConfig) (map[string]*rhpb.ResourceHandle_ResourceData, error) {
	resourceData := map[string]*rhpb.ResourceHandle_ResourceData{
		"Icon2Connection": new(rhpb.ResourceHandle_ResourceData),
	}

	realtimeControlConfig := iconMainConfig.GetRealtimeControlConfig()
	// Parts that are deactivated will not be included in the resource data.
	disabledParts := iconMainConfig.GetDeactivatedHardwareConfiguration().GetPartNames()
	// Provide a stable output here.  See b/319606024 for more details.
	var keys []string
	for key := range realtimeControlConfig.GetPartsByName() {
		keys = append(keys, key)
	}
	// Sort lexicographically in ascending order such that "a" appears in the
	// fields related to a single part.
	slices.SortFunc(keys, strings.Compare)

	var adioPart *iepb.Icon2AdioPart
	var armPart *iepb.Icon2PositionPart
	var ftPart *iepb.Icon2ForceTorqueSensorPart
	var rfPart *iepb.Icon2RangefinderPart
	for _, partName := range keys {
		if slices.Contains(disabledParts, partName) {
			continue
		}
		partConfig, _ := realtimeControlConfig.GetPartsByName()[partName]
		switch partTypeName := partConfig.GetPartTypeName(); partTypeName {
		case "HalADIOPart":
			if adioPart == nil {
				adioPart = &iepb.Icon2AdioPart{
					Target: &iepb.Icon2AdioPart_IconTarget_{
						IconTarget: &iepb.Icon2AdioPart_IconTarget{
							PartName: partName,
						},
					},
					IconParts: []string{
						partName,
					},
				}
			} else {
				adioPart.IconParts = append(adioPart.IconParts, partName)
			}
		case "HalArmPart":
			objectName := resourceName
			if name := partConfig.GetHardwareResourceName(); name != "" {
				objectName = name
			}
			if armPart == nil {
				armPart = &iepb.Icon2PositionPart{
					PartName:                 partName,
					WorldRobotCollectionName: objectName,
					ObjectNames: map[string]string{
						partName: objectName,
					},
				}
			} else {
				armPart.ObjectNames[partName] = objectName
			}
		case "HalForceTorqueSensorPart":
			var settings *fcpb.ForceControlSettings
			if partConfig.GetConfig() != nil {
				ftPartConfig := &ftppb.HalForceTorqueSensorPartConfig{}
				if err := partConfig.Config.UnmarshalTo(ftPartConfig); err != nil {
					return nil, errors.Wrap(err, "cannot unpack part-config as ft-sensor-part config")
				}
				settings = ftPartConfig.GetForceControlSettings()
			}
			if ftPart == nil {
				ftPart = &iepb.Icon2ForceTorqueSensorPart{
					PartName:             partName,
					ForceControlSettings: settings,
					Settings: map[string]*fcpb.ForceControlSettings{
						partName: settings,
					},
				}
			} else {
				ftPart.Settings[partName] = settings
			}
		case "HalRangefinderPart":
			if rfPart == nil {
				rfPart = &iepb.Icon2RangefinderPart{
					PartName: partName,
					PartNames: []string{
						partName,
					},
				}
			} else {
				rfPart.PartNames = append(rfPart.PartNames, partName)
			}
		}
	}

	if adioPart != nil {
		target, err := anypb.New(adioPart)
		if err != nil {
			return nil, errors.Wrap(err, "any.New(adioPart)")
		}
		resourceData["Icon2AdioPart"] = &rhpb.ResourceHandle_ResourceData{
			Contents: target,
		}
	}
	if armPart != nil {
		target, err := anypb.New(armPart)
		if err != nil {
			return nil, errors.Wrap(err, "any.New(armPart)")
		}
		resourceData["Icon2PositionPart"] = &rhpb.ResourceHandle_ResourceData{
			Contents: target,
		}
	}
	if ftPart != nil {
		target, err := anypb.New(ftPart)
		if err != nil {
			return nil, errors.Wrap(err, "any.New(ftPart)")
		}
		resourceData["Icon2ForceTorqueSensorPart"] = &rhpb.ResourceHandle_ResourceData{
			Contents: target,
		}
	}
	if rfPart != nil {
		target, err := anypb.New(rfPart)
		if err != nil {
			return nil, errors.Wrap(err, "any.New(rfPart)")
		}
		resourceData["Icon2RangefinderPart"] = &rhpb.ResourceHandle_ResourceData{
			Contents: target,
		}
	}

	return resourceData, nil
}

func mergeResourceData(servicePrefixes []string, extraResourceData map[string]*rhpb.ResourceHandle_ResourceData) (map[string]*rhpb.ResourceHandle_ResourceData, error) {
	resourceData, err := getServicePrefixesAsEquipmentData(servicePrefixes)
	if err != nil {
		return nil, errors.Wrap(err, "mergeResourceData")
	}

	if resourceData == nil || len(resourceData) == 0 {
		return extraResourceData, nil
	}

	for eqKey, eqValue := range extraResourceData {
		if _, present := resourceData[eqKey]; present {
			// TODO(b/300273691): due to cl/561349521, resource types in catalog already contain service
			// prefixes as resource data (with nil or empty value). So ignore this error for now.
			if eqValue.GetContents() == nil {
				log.Warningf("User provided resource data key %q should not match a service prefix. Ignoring this error for now.", eqKey)
				continue
			}
			return nil, fmt.Errorf("(%q,%v) cannot be used in resource handle as it conflicts with resource's service prefix", eqKey, eqValue)
		}
		resourceData[eqKey] = eqValue
	}

	return resourceData, nil
}

// Convert attempts to convert a resource instance into a resource handle. If
// the resource instance doesn't specify resource handle, nil will be returned.
// TODO(b/233044332): Support more resource types and remove special casing.
func Convert(ri *rrpb.ResourceInstance, address string, servicePrefixes []string) (*rhpb.ResourceHandle, error) {
	connection := &rhpb.ResourceConnectionInfo{}
	if ri.GetHasServices() {
		connection = &rhpb.ResourceConnectionInfo{
			Target: &rhpb.ResourceConnectionInfo_Grpc{
				Grpc: &rhpb.ResourceGrpcConnectionInfo{
					Address:        address,
					ServerInstance: ri.GetName(),
					Header:         InstanceHeader,
				},
			},
		}
	}

	var extraResourceData map[string]*rhpb.ResourceHandle_ResourceData

	// Camera resource has a complex lifecycle - skills and the underlying
	// resources both consume information from the resource data and
	// config. To avoid parallel migrations, we leave this special-case
	// handling in place for cameras.
	cameraConfig := &ccv1pb.CameraConfig{}
	if err := ri.GetConfiguration().UnmarshalTo(cameraConfig); err == nil {
		extraResourceData = map[string]*rhpb.ResourceHandle_ResourceData{
			"CameraConfig": {
				Contents: ri.GetConfiguration(),
			},
		}
	}

	iconConfig := &ipb.IconMainConfig{}
	if err := ri.GetConfiguration().UnmarshalTo(iconConfig); err == nil {
		extraResourceData, err = extractResourceDataFromIconConfig(ri.GetName(), iconConfig)
		if err != nil {
			return nil, errors.Wrap(err, "extractResourceDataFromIconConfig")
		}
	}

	mergedResourceData, err := mergeResourceData(servicePrefixes, extraResourceData)
	if err != nil {
		return nil, grpcstatus.Error(codes.InvalidArgument, err.Error())
	}

	if mergedResourceData != nil {
		return &rhpb.ResourceHandle{
			Name:           ri.GetName(),
			ResourceData:   mergedResourceData,
			ConnectionInfo: connection,
		}, nil
	}

	// Some resources (like hw modules) can have services but no service prefix or resource data.
	if ri.GetHasServices() {
		return &rhpb.ResourceHandle{
			Name:           ri.GetName(),
			ResourceData:   map[string]*rhpb.ResourceHandle_ResourceData{},
			ConnectionInfo: connection,
		}, nil
	}

	return nil, nil
}
