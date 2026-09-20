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

// Package descriptorcompatibility provides utilities to correctly handle file
// descriptor sets built in external or google3 environments regardless of the
// build environment of handling code.
package descriptorcompatibility

import (
	"fmt"

	"google.golang.org/protobuf/proto"

	descpb "google.golang.org/protobuf/types/descriptorpb"
)

// Reconcile updates the file descriptor set so that it is compatible with the
// well-known protos (e.g. `descriptor.proto`) used to build this code.
//
// Some file descriptor sets may be built internally (google3) while others are
// built externally. There are well-known types that have different package
// names in these two cases (e.g. those in `descriptor.proto`). This is an issue
// when runtime code compares the names of well-known types from arbitrary file
// descriptor sets to the names of the same protos built into the service during
// compilation. For example, if this code is built in google3, its proto tooling
// expects `proto2.FieldOptions`, but if someone builds a skill with field
// options externally (e.g. in insrc), the proto will be called
// `google.protobuf.FieldOptions` in the skill's file descriptor set. This
// function replaces the affected package names in the file descriptor set
// (through questionable means). In practice, this replaces `proto2` with
// `google.protobuf` when the service is built externally and `google.protobuf`
// with `proto2` when the service is built in google3.
//
// This function intentionally uses as little reflection as possible. Proto
// reflection code might perform some kind of validation, which could fail if
// there is a build environment mismatch.
//
// See b/382250177 for fallout of this issue and more details.
func Reconcile(fds *descpb.FileDescriptorSet) error {
	if fds == nil {
		return nil
	}
	replacements := make(map[string]string)
	// Iterate each file in the descriptor to see if any of them are known to be
	// mismatched between google3 and external builds.
	for _, file := range fds.GetFile() {
		// Update the package name if the reconciled package name is different from
		// the current package name in the file.
		if wantPackage := reconciledPackage(file); wantPackage != file.GetPackage() {
			// We will need to update all references to types in the file to use the
			// new package name. This is necessary because all type references in file
			// descriptors are absolute (e.g. `.google.protobuf.FileDescriptorProto`).
			// Changing the package name in a file (`package` field) renders all of
			// these references invalid.
			for _, name := range typeNames(file) {
				// The type name is the fully qualified name of the type including the
				// leading period. Only exact matches must be replaced.
				// Example: `.google.protobuf.FileDescriptorProto`
				oldName := fmt.Sprintf(".%s.%s", file.GetPackage(), name)
				newName := fmt.Sprintf(".%s.%s", wantPackage, name)
				replacements[oldName] = newName
			}
			// This changes the package name of the file. At this moment all
			// references to types in this file are invalid until the replacements
			// returned from this function are applied to the entire descriptor set.
			file.Package = proto.String(wantPackage)
		}
	}
	// If no package names were changed in the file descriptor set, there is
	// nothing further to do and we can skip the type reference update.
	if len(replacements) > 0 {
		updateTypeReferences(fds, replacements)
	}
	return nil
}

func updateTypeReferences(fds *descpb.FileDescriptorSet, replacements map[string]string) {
	for _, file := range fds.GetFile() {
		for _, msg := range file.GetMessageType() {
			updateMessageTypeReferences(msg, replacements)
		}
		for _, ext := range file.GetExtension() {
			updateExtensionTypeReferences(ext, replacements)
		}
	}
}

func updateMessageTypeReferences(msg *descpb.DescriptorProto, replacements map[string]string) {
	for _, field := range msg.GetField() {
		if newTypeName, ok := replacements[field.GetTypeName()]; ok {
			field.TypeName = proto.String(newTypeName)
		}
	}
	for _, ext := range msg.GetExtension() {
		updateExtensionTypeReferences(ext, replacements)
	}
	for _, nested := range msg.GetNestedType() {
		updateMessageTypeReferences(nested, replacements)
	}
}

func updateExtensionTypeReferences(ext *descpb.FieldDescriptorProto, replacements map[string]string) {
	if newTypeName, ok := replacements[ext.GetExtendee()]; ok {
		ext.Extendee = proto.String(newTypeName)
	}
	if newType, ok := replacements[ext.GetTypeName()]; ok {
		ext.TypeName = proto.String(newType)
	}
}

// typeNames returns the names of all types (including recursively nested types)
// in the given file descriptor.
func typeNames(file *descpb.FileDescriptorProto) []string {
	var names []string
	for _, msg := range file.GetMessageType() {
		names = append(names, messageTypeNames(msg, "")...)
	}
	for _, enum := range file.GetEnumType() {
		names = append(names, enum.GetName())
	}
	return names
}

// messageTypeNames returns the names of all recursively nested types in the
// given message. Includes the name of the message itself.
func messageTypeNames(msg *descpb.DescriptorProto, parentName string) []string {
	name := msg.GetName()
	if parentName != "" {
		name = fmt.Sprintf("%s.%s", parentName, name)
	}
	names := []string{name}
	for _, nested := range msg.GetNestedType() {
		names = append(names, messageTypeNames(nested, name)...)
	}
	for _, enum := range msg.GetEnumType() {
		names = append(names, fmt.Sprintf("%s.%s", name, enum.GetName()))
	}
	return names
}

// reconciledPackage returns the correct package for the given file descriptor
// in the current build environment (google3 or external).
func reconciledPackage(file *descpb.FileDescriptorProto) string {
	// Matches `descriptor.proto`. We can't match the full name (path) because it
	// would be different between google3 and external builds.
	if file.GetOptions().GetGoPackage() == "google.golang.org/protobuf/types/descriptorpb" {
		// The package name of the `descriptor.proto` file that was used to build
		// this code.
		buildPkg := (&descpb.FileDescriptorSet{}).ProtoReflect().Descriptor().ParentFile().Package()
		return string(buildPkg)
	}
	// All other files should stick with the package they have.
	return file.GetPackage()
}
