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

// Package resolver provides functionalities for resolving dependencies of Assets.
package resolver

import (
	"fmt"
	"strings"

	"intrinsic/assets/idutils"
	"intrinsic/assets/interfaceutils"
	"intrinsic/util/proto/registryutil"
	"intrinsic/util/proto/walkmessages"

	log "github.com/golang/glog"
	"google.golang.org/protobuf/proto"
	"google.golang.org/protobuf/reflect/protoreflect"

	fieldmetadatapb "intrinsic/assets/proto/field_metadata_go_proto"
	grpcconnectionpb "intrinsic/assets/proto/v1/grpc_connection_go_proto"
	resolveddependencypb "intrinsic/assets/proto/v1/resolved_dependency_go_proto"

	descriptorpb "google.golang.org/protobuf/types/descriptorpb"
	anypb "google.golang.org/protobuf/types/known/anypb"
)

var (
	resolvedDependencyDescriptor = (&resolveddependencypb.ResolvedDependency{}).ProtoReflect().Descriptor()
	anyDescriptor                = (&anypb.Any{}).ProtoReflect().Descriptor()
)

// messageProcessor is a function that takes a message and processes it.
type messageProcessor func(proto.Message) (proto.Message, error)

// unpackAny uses the given file descriptor set to unmarshal the Any proto.
func unpackAny(config *anypb.Any, fds *descriptorpb.FileDescriptorSet) (proto.Message, error) {
	types, err := registryutil.NewTypesFromFileDescriptorSet(fds)
	if err != nil {
		log.Errorf("failed to create types from file descriptor set: %v", err)
		return nil, err
	}

	msgType, err := types.FindMessageByName(config.MessageName())
	if err != nil {
		log.Errorf("failed to find message by name: %v", err)
		return nil, err
	}
	msg := msgType.New().Interface()
	if err := config.UnmarshalTo(msg); err != nil {
		log.Errorf("failed to unmarshal Any: %v", err)
		return nil, err
	}
	return msg, nil
}

// deterministicAnyNew is an variant of anypb.New that uses deterministic
// marshalling.  This is at least consistent within the same binary, but may
// not match other languages or future versions of golang.
//
// Having a deterministic output is important because we presume we are able to
// generate a deterministic resources chart.  A change in an annotation can
// trigger a pod restart.  We deliberately include the serialized config for a
// service instance in the context_id annotation to restart pods so they pick
// up new config.  A randomized output can trigger unintended restarts.
func deterministicAnyNew(inner proto.Message) (*anypb.Any, error) {
	msg := new(anypb.Any)
	if err := anypb.MarshalFrom(msg, inner, proto.MarshalOptions{Deterministic: true}); err != nil {
		return nil, err
	}
	return msg, nil
}

// Resolver resolves interface-based asset dependencies for a given config.
type Resolver struct {
	address        string
	instanceHeader string
}

// New creates a new Resolver.
//
// The `address` is the address of the asset instance or the address of the ingress.
// The `instanceHeaderName“ is the name of the gRPC metadata header that contains the instance name
// of the asset.
func New(address, instanceHeaderName string) *Resolver {
	return &Resolver{
		address:        address,
		instanceHeader: instanceHeaderName,
	}
}

// ResolveConfigDependencies resolves the asset dependencies specified in the given config, and
// returns a new resolved config. The input config is not modified.
//
// This method recursively walks the proto message and resolves dependencies for
// [ResolvedDependency] message type that are annotated with the dependency field metadata.
// This [ResolvedDependency] type may be a single field, a repeated field, or a map with the values
// of [ResolvedDependency] type.
//
// This method unpacks the top level Any proto from the specified file descriptor set. Nested Any
// protos are packed iff its descriptors are provided in the file descriptor set. No errors will
// be returned for missing descriptors of nested Anys.
//
// If either of the input arguments is nil, this method returns the config unchanged.
func (r *Resolver) ResolveConfigDependencies(config *anypb.Any, fds *descriptorpb.FileDescriptorSet) (*anypb.Any, error) {
	if config == nil || fds == nil {
		return config, nil
	}

	resolved, err := r.resolveDependencies(config, fds)
	if err != nil {
		return nil, err
	}
	if resolved == nil {
		return config, err
	}
	return resolved, nil
}

// resolveDependencies will resolve fields marked as a dependency and return a
// new, updated Any proto.  If there are no dependencies, it returns nil.
func (r *Resolver) resolveDependencies(config *anypb.Any, fds *descriptorpb.FileDescriptorSet) (*anypb.Any, error) {
	msg, err := unpackAny(config, fds)
	if err != nil {
		return nil, err
	}

	// Keep track of whether we change state within the proto.  If we don't, then
	// we can return nil to indicate to the caller to just use the original value.
	// This avoids introducing any changes whatsoever and the overhead of
	// remarshalling the proto.  Ideally we could simply return config back to
	// the caller here, however, config is sometimes a copy of the original
	// obtained by marshalling and unmarshalling the proto, since the original is
	// a dynamic proto, rather than strictly an Any proto.
	var touched bool
	resolvedMsg, err := walkmessages.Recursively(msg, func(msg proto.Message) (proto.Message, bool, error) {
		fields := msg.ProtoReflect().Descriptor().Fields()
		for i := 0; i < fields.Len(); i++ {
			field := fields.Get(i)
			if field.Kind() != protoreflect.MessageKind {
				continue
			}

			// Get the message descriptor of the field. If the field is a map, get the message
			// descriptor of the map value.
			var msgDesc protoreflect.MessageDescriptor
			if field.IsMap() {
				if field.MapValue().Kind() != protoreflect.MessageKind {
					continue
				}
				msgDesc = field.MapValue().Message()
			} else {
				msgDesc = field.Message()
			}

			if msgDesc.FullName() == anyDescriptor.FullName() {
				if err := processField(field, msg, func(m proto.Message) (proto.Message, error) {
					// The message is likely a dynamic proto, so we need to marshal and unmarshal.
					data, err := proto.Marshal(m)
					if err != nil {
						return nil, err
					}
					if len(data) == 0 {
						return nil, nil
					}
					anyMsg := &anypb.Any{}
					if err := proto.Unmarshal(data, anyMsg); err != nil {
						return nil, err
					}
					resolvedAnyMsg, err := r.resolveDependencies(anyMsg, fds)
					if err != nil {
						log.Errorf("failed to resolve nested Any proto with type URL %q: %v. Ignoring this Any proto.", anyMsg.GetTypeUrl(), err)
						return anyMsg, nil
					}
					if resolvedAnyMsg == nil {
						return m, nil
					}
					touched = true
					return resolvedAnyMsg, nil
				}); err != nil {
					return nil, false, err
				}
				// If no error, either the Any proto was processed and any dependencies were resolved
				// successfully, or there was a missing FDS. In either case, we skip the rest of the
				// processing for this field.
				continue
			}

			if msgDesc.Name() != resolvedDependencyDescriptor.Name() {
				continue
			}

			options := field.Options().(*descriptorpb.FieldOptions)
			fieldMetadata := proto.GetExtension(options, fieldmetadatapb.E_FieldMetadata).(*fieldmetadatapb.FieldMetadata)
			requires := fieldMetadata.GetDependency().GetRequires()
			if fieldMetadata.GetDependency() == nil {
				// Since there is no dependency annotation, we assume here that the user does not want to
				// treat this field as a dependency specification.
				continue
			}
			requiresObject := fieldMetadata.GetDependency().GetRequiresObject() != nil

			if err := processField(field, msg, func(m proto.Message) (proto.Message, error) {
				return r.resolveStub(m, requires, requiresObject)
			}); err != nil {
				return nil, false, err
			}
			touched = true
		}
		return msg, true, nil
	})
	if err != nil {
		return nil, err
	}

	if !touched {
		return nil, nil
	}

	return deterministicAnyNew(resolvedMsg)
}

// processField processes a field of a proto message with a given processing function.
//
// If the field is a list or map, the function is called for each element.
// If the field is a message, the function is called for the message.
func processField(field protoreflect.FieldDescriptor, parentMsg proto.Message, f messageProcessor) error {
	if field.IsList() {
		list := parentMsg.ProtoReflect().Get(field).List()
		for i := 0; i < list.Len(); i++ {
			if list.Get(i).Message().Interface() != nil {
				val, err := f(list.Get(i).Message().Interface())
				if err != nil {
					return err
				}
				if val != nil {
					list.Set(i, protoreflect.ValueOfMessage(val.ProtoReflect()))
				}
			}
		}
	} else if field.IsMap() {
		m := parentMsg.ProtoReflect().Get(field).Map()
		var processErr error
		m.Range(func(key protoreflect.MapKey, value protoreflect.Value) bool {
			if value.Message().Interface() != nil {
				val, err := f(value.Message().Interface())
				if err != nil {
					processErr = err
					return false
				}
				if val != nil {
					m.Set(key, protoreflect.ValueOfMessage(val.ProtoReflect()))
				}
			}
			return true
		})
		if processErr != nil {
			return processErr
		}
	} else {
		if parentMsg.ProtoReflect().Has(field) {
			dynamicMsg := parentMsg.ProtoReflect().Get(field).Message().Interface()
			val, err := f(dynamicMsg)
			if err != nil {
				return err
			}
			if val != nil {
				parentMsg.ProtoReflect().Set(field, protoreflect.ValueOfMessage(val.ProtoReflect()))
			}
		}
	}
	return nil
}

// resolveStub returns a [ResolvedDependency] message with the interfaces field populated and the
// name field cleared.
func (r *Resolver) resolveStub(msg proto.Message, requires []string, requiresObject bool) (*resolveddependencypb.ResolvedDependency, error) {
	// The message is likely a dynamic proto, so we need to marshal and unmarshal.
	data, err := proto.Marshal(msg)
	if err != nil {
		return nil, err
	}
	if len(data) == 0 {
		return nil, nil
	}
	val := &resolveddependencypb.ResolvedDependency{}
	if err := proto.Unmarshal(data, val); err != nil {
		return nil, err
	}
	dependencyName := val.GetName()
	if dependencyName == "" {
		return val, nil
	}
	if len(val.GetInterfaces()) == 0 && len(requires) > 0 {
		val.Interfaces = make(map[string]*resolveddependencypb.ResolvedDependency_Interface, len(requires))
	}
	for _, require := range requires {
		if strings.HasPrefix(require, interfaceutils.GRPCURIPrefix) {
			val.Interfaces[require] = &resolveddependencypb.ResolvedDependency_Interface{
				Protocol: &resolveddependencypb.ResolvedDependency_Interface_Grpc_{
					Grpc: &resolveddependencypb.ResolvedDependency_Interface_Grpc{
						Connection: &grpcconnectionpb.GrpcConnection{
							Address: r.address,
							Metadata: []*grpcconnectionpb.GrpcConnection_Metadata{
								{
									Key:   r.instanceHeader,
									Value: dependencyName,
								},
							},
						},
					},
				},
			}
		} else if strings.HasPrefix(require, interfaceutils.DataURIPrefix) {
			if id, err := idutils.IDProtoFromString(dependencyName); err != nil {
				log.Errorf("failed to create data asset ID proto from dependency name: %v", err)
			} else {
				val.Interfaces[require] = &resolveddependencypb.ResolvedDependency_Interface{
					Protocol: &resolveddependencypb.ResolvedDependency_Interface_Data_{
						Data: &resolveddependencypb.ResolvedDependency_Interface_Data{
							Id: id,
						},
					},
				}
			}
		} else {
			return nil, fmt.Errorf("unsupported dependency type: %v", require)
		}
	}

	if requiresObject {
		val.Object = &resolveddependencypb.ResolvedDependency_Object{
			Name: dependencyName,
		}
	}

	// Clear the name field, as we have extracted all needed information.
	val.Name = ""

	return val, nil
}
