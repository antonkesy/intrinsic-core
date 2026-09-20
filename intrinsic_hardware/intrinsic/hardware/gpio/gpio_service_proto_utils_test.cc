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

#include "intrinsic/hardware/gpio/gpio_service_proto_utils.h"

#include <gtest/gtest.h>

#include <vector>

#include "absl/strings/str_cat.h"

namespace intrinsic_proto::gpio::v1 {

TEST(OpenWriteSessionRequest, Equality) {
  OpenWriteSessionRequest req1;
  EXPECT_EQ(req1, req1);

  OpenWriteSessionRequest req2;

  EXPECT_EQ(req1, req2);

  for (const auto& name : {"port.01", "port.03", "port.04"}) {
    req1.mutable_initial_session_data()->add_signal_names(name);
    req2.mutable_initial_session_data()->add_signal_names(name);
  }

  EXPECT_EQ(req1, req2);
  EXPECT_EQ(req1, req1);  // self check

  intrinsic_proto::gpio::v1::SignalValueSet write_signals;
  write_signals.mutable_values()->insert(
      {"port.01", ::intrinsic::gpio::SignalTrueValue()});
  write_signals.mutable_values()->insert(
      {"port.04", ::intrinsic::gpio::SignalFalseValue()});

  *req1.mutable_write_signals()->mutable_signal_values() = write_signals;
  EXPECT_NE(req1, req2);

  *req2.mutable_write_signals()->mutable_signal_values() = write_signals;
  EXPECT_EQ(req1, req2);

  req1.clear_initial_session_data();
  EXPECT_NE(req1, req2);
}

TEST(ReadSignalsRequest, SelfCheck) {
  ReadSignalsRequest req;
  EXPECT_EQ(req, req);

  req.add_signal_names("electrical");
  EXPECT_EQ(req, req);

  req.add_signal_names("magnetic");
  EXPECT_EQ(req, req);
}

TEST(ReadSignalsRequest, Equality) {
  ReadSignalsRequest req1, req2;
  EXPECT_EQ(req1, req2);

  req1.add_signal_names("high");
  EXPECT_NE(req1, req2);

  req2.add_signal_names("high");
  EXPECT_EQ(req1, req2);

  req2.add_signal_names("low");
  EXPECT_NE(req1, req2);

  req1.add_signal_names("low");
  EXPECT_EQ(req1, req2);
}

TEST(SignalValue, BoolValues) {
  EXPECT_EQ(::intrinsic::gpio::SignalFalseValue().bool_value(), false);
  EXPECT_EQ(::intrinsic::gpio::SignalTrueValue().bool_value(), true);
}

TEST(GpioServiceProtoUtils, SignalValueSetEquality) {
  // TODO(dhirajgoel): Move these helper functions to gpio_service_proto_utils
  // library.
  auto ToBoolValue = [](const bool v) {
    intrinsic_proto::gpio::v1::SignalValue value;
    value.set_bool_value(v);
    return value;
  };

  auto ToIntValue = [](const int32_t v) {
    intrinsic_proto::gpio::v1::SignalValue value;
    value.set_int_value(v);
    return value;
  };

  auto ToUnsignedIntValue = [](const uint32_t v) {
    intrinsic_proto::gpio::v1::SignalValue value;
    value.set_unsigned_int_value(v);
    return value;
  };

  auto ToFloatValue = [](const float v) {
    intrinsic_proto::gpio::v1::SignalValue value;
    value.set_float_value(v);
    return value;
  };

  auto ToDoubleValue = [](const double v) {
    intrinsic_proto::gpio::v1::SignalValue value;
    value.set_double_value(v);
    return value;
  };

  auto ToInt8Value = [](const int8_t v) {
    intrinsic_proto::gpio::v1::SignalValue value;
    value.mutable_int8_value()->set_value(v);
    return value;
  };

  auto ToUnsignedInt8Value = [](const uint8_t v) {
    intrinsic_proto::gpio::v1::SignalValue value;
    value.mutable_unsigned_int8_value()->set_value(v);
    return value;
  };

  const std::vector<intrinsic_proto::gpio::v1::SignalValue> values = {
      ToBoolValue(true),   ToIntValue(-981),        ToUnsignedIntValue(911007),
      ToFloatValue(1.25f), ToBoolValue(false),      ToDoubleValue(-4.50),
      ToInt8Value(-120),   ToFloatValue(-3.14157f), ToUnsignedInt8Value(108)};

  intrinsic_proto::gpio::v1::SignalValueSet outer;
  for (size_t i = 0; i < values.size(); i++) {
    outer.mutable_values()->insert({absl::StrCat("signal_", i), values[i]});
    intrinsic_proto::gpio::v1::SignalValueSet inner;
    intrinsic_proto::gpio::v1::SignalValueSet just_different;
    for (size_t j = 0; j < values.size(); j++) {
      inner.mutable_values()->insert({absl::StrCat("signal_", j), values[j]});
      just_different.mutable_values()->insert(
          {absl::StrCat("just_different_", j), values[j]});

      // Values on the diagonal of this ixj matrix should be equal.
      EXPECT_EQ(outer == inner, i == j);

      // These should never be equal since the signal names are different.
      EXPECT_NE(outer, just_different);
      EXPECT_NE(inner, just_different);
    }
  }
}

}  // namespace intrinsic_proto::gpio::v1

namespace intrinsic::gpio {

TEST(GpioServiceProtoUtils, CheckSignalEqualityBoolean) {
  intrinsic_proto::gpio::v1::SignalValue true_signal;
  true_signal.set_bool_value(true);
  intrinsic_proto::gpio::v1::SignalValue false_signal;
  false_signal.set_bool_value(false);
  EXPECT_TRUE(SignalValuesAreApproxEqual(true_signal, true_signal));
  EXPECT_EQ(true_signal, true_signal);
  EXPECT_TRUE(SignalValuesAreApproxEqual(false_signal, false_signal));
  EXPECT_EQ(false_signal, false_signal);
  EXPECT_FALSE(SignalValuesAreApproxEqual(false_signal, true_signal));
  EXPECT_NE(false_signal, true_signal);
  EXPECT_FALSE(SignalValuesAreApproxEqual(true_signal, false_signal));
  EXPECT_NE(true_signal, false_signal);
}

TEST(GpioServiceProtoUtils, CheckSignalEqualityInt) {
  intrinsic_proto::gpio::v1::SignalValue x;
  intrinsic_proto::gpio::v1::SignalValue y;
  EXPECT_FALSE(SignalValuesAreApproxEqual(x, y));
  EXPECT_NE(x, y);

  x.set_int_value(1);
  y.set_int_value(1);
  EXPECT_TRUE(SignalValuesAreApproxEqual(x, y));
  EXPECT_EQ(x, y);

  y.set_int_value(-2);
  EXPECT_FALSE(SignalValuesAreApproxEqual(x, y));
  EXPECT_NE(x, y);

  x.set_int_value(-2);
  EXPECT_TRUE(SignalValuesAreApproxEqual(x, y));
  EXPECT_EQ(x, y);
}

TEST(GpioServiceProtoUtils, CheckSignalEqualityUnsignedInt) {
  intrinsic_proto::gpio::v1::SignalValue x;
  intrinsic_proto::gpio::v1::SignalValue y;
  EXPECT_FALSE(SignalValuesAreApproxEqual(x, y));

  x.set_unsigned_int_value(1);
  y.set_unsigned_int_value(1);
  EXPECT_TRUE(SignalValuesAreApproxEqual(x, y));
  EXPECT_EQ(x, y);

  y.set_unsigned_int_value(5);
  EXPECT_FALSE(SignalValuesAreApproxEqual(x, y));
  EXPECT_NE(x, y);

  x.set_unsigned_int_value(5);
  EXPECT_TRUE(SignalValuesAreApproxEqual(x, y));
  EXPECT_EQ(x, y);
}

TEST(GpioServiceProtoUtils, CheckSignalEqualityUnsignedInt8) {
  intrinsic_proto::gpio::v1::SignalValue x;
  intrinsic_proto::gpio::v1::SignalValue y;
  EXPECT_FALSE(SignalValuesAreApproxEqual(x, y));

  x.mutable_unsigned_int8_value()->set_value(10);
  y.mutable_unsigned_int8_value()->set_value(10);
  EXPECT_TRUE(SignalValuesAreApproxEqual(x, y));
  EXPECT_EQ(x, y);

  y.mutable_unsigned_int8_value()->set_value(51);
  EXPECT_FALSE(SignalValuesAreApproxEqual(x, y));
  EXPECT_NE(x, y);

  x.mutable_unsigned_int8_value()->set_value(51);
  EXPECT_TRUE(SignalValuesAreApproxEqual(x, y));
  EXPECT_EQ(x, y);
}

TEST(GpioServiceProtoUtils, CheckSignalEqualityInt8) {
  intrinsic_proto::gpio::v1::SignalValue x;
  intrinsic_proto::gpio::v1::SignalValue y;
  EXPECT_FALSE(SignalValuesAreApproxEqual(x, y));

  x.mutable_int8_value()->set_value(10);
  y.mutable_int8_value()->set_value(10);
  EXPECT_TRUE(SignalValuesAreApproxEqual(x, y));
  EXPECT_EQ(x, y);

  y.mutable_int8_value()->set_value(-51);
  EXPECT_FALSE(SignalValuesAreApproxEqual(x, y));
  EXPECT_NE(x, y);

  x.mutable_int8_value()->set_value(-51);
  EXPECT_TRUE(SignalValuesAreApproxEqual(x, y));
  EXPECT_EQ(x, y);
}

TEST(GpioServiceProtoUtils, CheckSignalEqualityMistmatch) {
  intrinsic_proto::gpio::v1::SignalValue bool_val;
  bool_val.set_bool_value(true);

  intrinsic_proto::gpio::v1::SignalValue int_val;
  int_val.set_int_value(-1);

  intrinsic_proto::gpio::v1::SignalValue uint_val;
  uint_val.set_unsigned_int_value(10);

  intrinsic_proto::gpio::v1::SignalValue float_val;
  float_val.set_float_value(3.14f);

  intrinsic_proto::gpio::v1::SignalValue double_val;
  double_val.set_double_value(2.71828);

  intrinsic_proto::gpio::v1::SignalValue int8_val;
  int8_val.mutable_int8_value()->set_value(-10);

  intrinsic_proto::gpio::v1::SignalValue unsigned_int8_val;
  unsigned_int8_val.mutable_unsigned_int8_value()->set_value(113);

  const std::vector<const intrinsic_proto::gpio::v1::SignalValue*> values = {
      &bool_val,   &int_val,  &uint_val,         &float_val,
      &double_val, &int8_val, &unsigned_int8_val};

  for (size_t i = 0; i < values.size(); i++) {
    for (size_t j = 0; j < values.size(); j++) {
      // Values should be approx. equal.
      EXPECT_EQ(SignalValuesAreApproxEqual(*values[i], *values[j]), i == j);
      // Value should also be exactly equal.
      EXPECT_EQ(*values[i] == *values[j], i == j);
    }
  }
}

}  // namespace intrinsic::gpio
