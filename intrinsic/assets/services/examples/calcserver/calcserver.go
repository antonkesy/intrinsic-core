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

// Package calcserver contains a golang implementation of the calculator service
package calcserver

import (
	"context"
	"fmt"

	log "github.com/golang/glog"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"

	deputils "intrinsic/assets/dependencies/utils"

	rdpb "intrinsic/assets/proto/v1/resolved_dependency_go_proto"
	calcpb "intrinsic/assets/services/examples/calcserver/calc_server_go_proto"
	drpb "intrinsic/assets/services/proto/v1/dynamic_reconfiguration_go_proto"

	"google.golang.org/protobuf/types/known/anypb"
)

// PayloadFetcher is a function that retrieves a data payload from a dependency.
type PayloadFetcher func(ctx context.Context, dep *rdpb.ResolvedDependency, iface string, options ...deputils.GetDataOption) (*anypb.Any, error)

// CalcServer implements the calculator service.
type CalcServer struct {
	config  *serviceConfig
	fetcher PayloadFetcher
}

type serviceConfig struct {
	configpb    *calcpb.CalculatorConfig
	conversions map[string]conversionCategory
}

type linearConversion struct {
	factor float64
	offset float64
}

type conversionCategory struct {
	name     string
	baseUnit string
	units    map[string]linearConversion
}

// NewCalcServerOptions contains options for NewCalcServer.
type NewCalcServerOptions struct {
	Config         *calcpb.CalculatorConfig
	DatasetFetcher PayloadFetcher
}

// NewCalcServer creates a new calculator server.
func NewCalcServer(opts *NewCalcServerOptions) (*CalcServer, error) {
	s := &CalcServer{
		fetcher: opts.DatasetFetcher,
	}
	if s.fetcher == nil {
		s.fetcher = deputils.GetDataPayload
	}

	config := opts.Config
	if config == nil {
		config = &calcpb.CalculatorConfig{}
	}
	err := s.applyConfig(context.Background(), config)
	if err != nil {
		return nil, err
	}
	return s, nil
}

func (s *CalcServer) Calculate(ctx context.Context, request *calcpb.CalculatorRequest) (*calcpb.CalculatorResponse, error) {
	var result int64

	var a, b int64
	if s.config.configpb.GetReverseOrder() {
		a = request.GetY()
		b = request.GetX()
	} else {
		a = request.GetX()
		b = request.GetY()
	}

	switch request.GetOperation() {
	case calcpb.CalculatorOperation_CALCULATOR_OPERATION_ADD:
		result = a + b
		log.InfoContextf(ctx, "%d + %d = %d", a, b, result)
	case calcpb.CalculatorOperation_CALCULATOR_OPERATION_MULTIPLY:
		result = a * b
		log.InfoContextf(ctx, "%d * %d = %d", a, b, result)
	case calcpb.CalculatorOperation_CALCULATOR_OPERATION_SUBTRACT:
		result = a - b
		log.InfoContextf(ctx, "%d - %d = %d", a, b, result)
	case calcpb.CalculatorOperation_CALCULATOR_OPERATION_DIVIDE:
		if b == 0 {
			return nil, status.Error(codes.InvalidArgument, "cannot divide by 0")
		}
		result = a / b
		log.InfoContextf(ctx, "%d / %d = %d", a, b, result)
	default:
		return nil, status.Errorf(codes.Unimplemented, "unsupported operation: %v", request.GetOperation())
	}

	return &calcpb.CalculatorResponse{
		Result: result,
	}, nil
}

func (s *CalcServer) Convert(ctx context.Context, req *calcpb.ConvertRequest) (*calcpb.ConvertResponse, error) {
	config := s.config
	if config.conversions == nil {
		return nil, status.Error(codes.FailedPrecondition, "conversion dataset not loaded")
	}

	if req.GetCategory() == "" {
		return nil, status.Error(codes.InvalidArgument, "category not specified")
	}
	category, ok := config.conversions[req.GetCategory()]
	if !ok {
		return nil, status.Errorf(codes.InvalidArgument, "category %q not found", req.GetCategory())
	}

	if req.GetFromUnit() == "" {
		return nil, status.Error(codes.InvalidArgument, "from unit not specified")
	}
	if req.GetToUnit() == "" {
		return nil, status.Error(codes.InvalidArgument, "to unit not specified")
	}

	fromUnit, hasFrom := category.units[req.GetFromUnit()]
	toUnit, hasTo := category.units[req.GetToUnit()]

	if !hasFrom {
		return nil, status.Errorf(codes.NotFound, "unit %q not found in category %q", req.GetFromUnit(), req.GetCategory())
	}
	if !hasTo {
		return nil, status.Errorf(codes.NotFound, "unit %q not found in category %q", req.GetToUnit(), req.GetCategory())
	}

	from, to := req.GetFromUnit(), req.GetToUnit()
	if config.configpb.GetReverseOrder() {
		from, to = to, from
		fromUnit, toUnit = toUnit, fromUnit
	}

	baseVal := (req.GetValue() * fromUnit.factor) + fromUnit.offset

	// The dataset is already verified to ensure that toUnit.factor != 0.
	result := (baseVal - toUnit.offset) / toUnit.factor

	return &calcpb.ConvertResponse{
		Result: result,
	}, nil
}

func (s *CalcServer) ApplyConfiguration(ctx context.Context, request *drpb.ApplyConfigurationRequest) (*drpb.ApplyConfigurationResponse, error) {
	config := &calcpb.CalculatorConfig{}
	if err := request.GetConfiguration().UnmarshalTo(config); err != nil {
		return nil, status.Errorf(codes.InvalidArgument, "failed to unmarshal configuration: %v", err)
	}

	log.InfoContextf(ctx, "Applying configuration: %v", config)
	if err := s.applyConfig(ctx, config); err != nil {
		return nil, fmt.Errorf("failed to apply configuration: %w", err)
	}
	return &drpb.ApplyConfigurationResponse{}, nil
}

// processDataset processes the given conversion dataset and returns a map of conversion categories
// for internal representation and easy lookup.
func processDataset(dataset *calcpb.ConversionDataset) (map[string]conversionCategory, error) {
	if dataset == nil {
		return nil, nil
	}

	result := make(map[string]conversionCategory)
	for _, category := range dataset.GetCategories() {
		if category.GetName() == "" {
			return nil, status.Error(codes.InvalidArgument, "a category has no name specified")
		}
		if _, exists := result[category.GetName()]; exists {
			return nil, status.Errorf(codes.InvalidArgument, "duplicate category name: %s", category.GetName())
		}
		if category.GetBaseUnit() == "" {
			return nil, status.Errorf(codes.InvalidArgument, "category %s has no base unit", category.GetName())
		}
		cc := conversionCategory{
			name:     category.GetName(),
			baseUnit: category.GetBaseUnit(),
			units: map[string]linearConversion{
				category.GetBaseUnit(): {factor: 1.0, offset: 0.0},
			},
		}
		for _, unit := range category.GetUnits() {
			if _, exists := cc.units[unit.GetName()]; exists {
				return nil, status.Errorf(codes.InvalidArgument, "duplicate unit name %q in category %q", unit.GetName(), category.GetName())
			}
			if unit.GetLinear() == nil {
				return nil, status.Errorf(codes.InvalidArgument, "category %q: unit %q: conversion type not supported", category.GetName(), unit.GetName())
			}
			if unit.GetLinear().GetFactor() == 0 {
				return nil, status.Errorf(codes.InvalidArgument, "category %q: unit %q: factor cannot be 0", category.GetName(), unit.GetName())
			}
			cc.units[unit.GetName()] = linearConversion{
				factor: unit.GetLinear().GetFactor(),
				offset: unit.GetLinear().GetOffset(),
			}
		}
		result[category.GetName()] = cc
	}

	return result, nil
}

// applyConfig applies the given configuration to the service.
//
// Any conversion datasets present in the config are processed in a background goroutine.
func (s *CalcServer) applyConfig(ctx context.Context, inputConfig *calcpb.CalculatorConfig) error {
	if inputConfig.GetConversionDataset() == nil {
		s.config = &serviceConfig{
			configpb:    inputConfig,
			conversions: nil,
		}
		return nil
	}

	payload, err := s.fetcher(ctx, inputConfig.GetConversionDataset(), "data://intrinsic_proto.services.ConversionDataset")
	if err != nil {
		return err
	}
	dataset := &calcpb.ConversionDataset{}
	if err := payload.UnmarshalTo(dataset); err != nil {
		return err
	}
	conversions, err := processDataset(dataset)
	if err != nil {
		return err
	}
	s.config = &serviceConfig{
		configpb:    inputConfig,
		conversions: conversions,
	}
	return nil
}
