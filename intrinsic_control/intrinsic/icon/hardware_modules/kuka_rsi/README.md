<!--
Copyright 2026 Intrinsic Innovation LLC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    https://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
-->
# KUKA RSI Hardware Module

> [!NOTE]
> Developed and maintained by Intrinsic. Not an official product of KUKA.

## Overview

This directory contains the ICON Hardware Module (HWM) implementation for KUKA robots communicating via the RobotSensorInterface (RSI) real-time data exchange protocol.

## Supported Robot Models & Device Targets

| Robot Model | `intrinsic_hardware_device` Bazel Target | Asset ID |
| :--- | :--- | :--- |
| **KR 6 R900-2** | `:kr6_r900_2_hardware_module` | `ai.intrinsic.kuka_kr6_hardware_module` |
| **KR 10 R1100-2** | `:kr10_r1100_2_hardware_module` | `ai.intrinsic.kuka_kr10_hardware_module` |
| **KR 16 R2010-2** | `:kr16_r2010_2_hardware_module` | `ai.intrinsic.kuka_kr16_hardware_module` |
| **KR 20 R1810-2** | `:kr20_r1810_2_hardware_module` | `ai.intrinsic.kuka_kr20_hardware_module` |
| **KR 50 R2500** | `:kr50_r2500_hardware_module` | `ai.intrinsic.kuka_kr50_hardware_module` |

## Documentation

For documentation on setting up KUKA RSI with Intrinsic Flowstate, see the [KUKA RSI Setup Guide](https://flowstate.intrinsic.ai/docs/guides/commission_solution/set_up_robot/kuka_rsi_setup).
