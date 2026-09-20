
#include "third_party/abb_hardware_module/abb_hwm/rws_egm_utils.h"

#include <gtest/gtest.h>

#include <string>

#include "absl/log/log.h"
#include "absl/strings/str_format.h"
#include "google/protobuf/descriptor.h"
#include "third_party/abb_hardware_module/abb_hwm/abb_config.pb.h"

using namespace abb_hardware_module::rapid;

namespace {

std::string ValueToStr(float value) {
  return absl::StrFormat("%.*f", kMaxDecimalPoints, value);
}

std::string ValueToStr(bool value) {
  if (value) {
    return "TRUE";
  }
  return "FALSE";
}

std::string ValueToStr(int value) { return absl::StrFormat("%d", value); }

class EgmSettingsTest : public ::testing::Test {
 protected:
  const std::string egm_settings_string =
      "[TRUE,FALSE,[TRUE,1],[[TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,"
      "0,0],0,0,0]],[FALSE,TRUE,\"\",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]],["
      "[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]],0.5,20,8,1],[60,1,[[0,0,0],[1,0,"
      "0,0]],1],[1]]";

  // Helper function to apply config and check results in RAPID string
  template <typename T>
  void ApplyAndCheck(::intrinsic_proto::icon::EgmConfig& config,
                     const std::string& original_value, T value, int begin) {
    auto result =
        ApplyEgmConfigToEgmSettingsRapidString(egm_settings_string, config);
    ASSERT_TRUE(result.ok());
    LOG(ERROR) << result.value();
    // String length of the value
    int length = ValueToStr(value).size();
    // String length of the original value, before we changed it
    int original_length = original_value.size();

    ASSERT_NE(result.value(), egm_settings_string);
    ASSERT_EQ(result.value().substr(0, begin),
              egm_settings_string.substr(0, begin));
    ASSERT_EQ(result.value().substr(begin, length), ValueToStr(value));
    ASSERT_EQ(
        result.value().substr(begin + length, std::string::npos),
        egm_settings_string.substr(begin + original_length, std::string::npos));
  }

  // Helper function to apply config and check results in json
  void ApplyAndCheckJson(::intrinsic_proto::icon::EgmConfig& config) {
    auto current_json =
        ParseEgmSettingsJsonFromRapidString(egm_settings_string);
    ASSERT_TRUE(current_json.ok());
    json current_json_value = current_json.value();
    auto result = UpdateEgmSettingsJson(config, &current_json_value);
    ASSERT_TRUE(result.ok());
    const google::protobuf::Descriptor* descriptor = config.GetDescriptor();
    for (int i = 0; i < descriptor->field_count(); i++) {
      const google::protobuf::FieldDescriptor* field = descriptor->field(i);
      auto [section, index] = EgmSettingsFieldPositionInJson(field->number());
      ASSERT_TRUE(section >= 0) << "Field not implemented";
      ASSERT_TRUE(index >= 0) << "Field not implemented";

      switch (field->type()) {
        {
          case google::protobuf::FieldDescriptor::Type::TYPE_FLOAT:
            auto value = config.GetReflection()->GetFloat(config, field);
            ASSERT_FLOAT_EQ(current_json_value[section][index], value)
                << "Field: " << field->name() << " Section: " << section
                << " Index: " << index;
            break;
        }
        {
          case google::protobuf::FieldDescriptor::Type::TYPE_BOOL:
            auto value = config.GetReflection()->GetBool(config, field);
            ASSERT_EQ(current_json_value[section][index], value);
            break;
        }
        {
          case google::protobuf::FieldDescriptor::Type::TYPE_UINT32:
            auto value = config.GetReflection()->GetUInt32(config, field);
            ASSERT_EQ(current_json_value[section][index], value);
            break;
        }
        {
          default:
            ASSERT_TRUE(false) << "Field type not implemented!";
        }
      }
    }
  }
};

TEST_F(EgmSettingsTest, ApplyEmptyConfigMakesNoChanges) {
  ::intrinsic_proto::icon::EgmConfig config;
  auto result =
      ApplyEgmConfigToEgmSettingsRapidString(egm_settings_string, config);
  ASSERT_TRUE(result.ok());
  ASSERT_EQ(result.value(), egm_settings_string);
}

TEST_F(EgmSettingsTest, ApplyConfigChangeUseFiltering) {
  bool value = false;
  // The string encoding of the original value
  std::string original_value = "TRUE";
  // String index where updated value is located
  // This is hardcoded based on egm_settings_string, and is only correct
  // for a single changed value. If multiple values are changed, this will
  // no longer be true.
  int begin = 13;

  ::intrinsic_proto::icon::EgmConfig config;
  config.set_setup_uc_use_filtering(value);

  ApplyAndCheck(config, original_value, value, begin);
}

// test for setuo_uc_comm_timeout
TEST_F(EgmSettingsTest, ApplyConfigChangeCommTimeout) {
  float value = 3.2;
  std::string original_value = "1";
  // String index where updated value is located
  // This is hardcoded based on egm_settings_string, and is only correct
  // for a single changed value. If multiple values are changed, this will
  // no longer be true.
  int begin = 18;

  ::intrinsic_proto::icon::EgmConfig config;
  config.set_setup_uc_comm_timeout(value);
  ApplyAndCheck(config, original_value, value, begin);
}

// test for activate_cond_min_max
TEST_F(EgmSettingsTest, ApplyConfigChangeCondMinMax) {
  float value = 0.042;
  std::string original_value = "0.5";
  // String index where updated value is located
  // This is hardcoded based on egm_settings_string, and is only correct
  // for a single changed value. If multiple values are changed, this will
  // no longer be true.
  int begin = 181;

  ::intrinsic_proto::icon::EgmConfig config;
  config.set_activate_cond_min_max(value);
  ApplyAndCheck(config, original_value, value, begin);
}

// test for activate_lp_filter
TEST_F(EgmSettingsTest, ApplyConfigChangeLpFilter) {
  float value = 16.2;
  std::string original_value = "20";
  // String index where updated value is located
  // This is hardcoded based on egm_settings_string, and is only correct
  // for a single changed value. If multiple values are changed, this will
  // no longer be true.
  int begin = 185;

  ::intrinsic_proto::icon::EgmConfig config;
  config.set_activate_lp_filter(value);
  ApplyAndCheck(config, original_value, value, begin);
}

// test for activate_sample_time
TEST_F(EgmSettingsTest, ApplyConfigChangeSampleTime) {
  int value = 4;
  std::string original_value = "4";
  // String index where updated value is located
  // This is hardcoded based on egm_settings_string, and is only correct
  // for a single changed value. If multiple values are changed, this will
  // no longer be true.
  int begin = 188;

  ::intrinsic_proto::icon::EgmConfig config;
  config.set_activate_sample_time(value);
  ApplyAndCheck(config, original_value, value, begin);

  // if the value is not a multiple of 4, the function should return an error
  value = 3;
  config.set_activate_sample_time(value);
  auto result =
      ApplyEgmConfigToEgmSettingsRapidString(egm_settings_string, config);
  ASSERT_FALSE(result.ok());
}

// test for run_cond_time
TEST_F(EgmSettingsTest, ApplyConfigChangeRunCondTime) {
  float value = 200.1;
  std::string original_value = "60";
  // String index where updated value is located
  // This is hardcoded based on egm_settings_string, and is only correct
  // for a single changed value. If multiple values are changed, this will
  // no longer be true.
  int begin = 194;

  ::intrinsic_proto::icon::EgmConfig config;
  config.set_run_cond_time(value);
  ApplyAndCheck<float>(config, original_value, value, begin);
}

// test for ramp_in_time
TEST_F(EgmSettingsTest, ApplyConfigChangeRunRampInTime) {
  float value = 2.03;
  std::string original_value = "1";
  // String index where updated value is located
  // This is hardcoded based on egm_settings_string, and is only correct
  // for a single changed value. If multiple values are changed, this will
  // no longer be true.
  int begin = 197;

  ::intrinsic_proto::icon::EgmConfig config;
  config.set_run_ramp_in_time(value);
  ApplyAndCheck<float>(config, original_value, value, begin);
}

// test run_pos_corr_gain
TEST_F(EgmSettingsTest, ApplyConfigChangeRunPosCorrGain) {
  float value = 3.2;
  std::string original_value = "1";
  // String index where updated value is located
  // This is hardcoded based on egm_settings_string, and is only correct
  // for a single changed value. If multiple values are changed, this will
  // no longer be true.
  int begin = 219;

  ::intrinsic_proto::icon::EgmConfig config;
  config.set_run_pos_corr_gain(value);
  ApplyAndCheck<float>(config, original_value, value, begin);
}

// test stop_ramp_out_time
TEST_F(EgmSettingsTest, ApplyConfigChangeStopRampOutTime) {
  float value = 2.1;
  std::string original_value = "1";
  // String index where updated value is located
  // This is hardcoded based on egm_settings_string, and is only correct
  // for a single changed value. If multiple values are changed, this will
  // no longer be true.
  int begin = 223;

  ::intrinsic_proto::icon::EgmConfig config;
  config.set_stop_ramp_out_time(value);
  ApplyAndCheck<float>(config, original_value, value, begin);
}

TEST_F(EgmSettingsTest, ApplyConfigJson) {
  ::intrinsic_proto::icon::EgmConfig config;
  config.set_setup_uc_use_filtering(false);
  config.set_setup_uc_comm_timeout(3.2);
  config.set_activate_cond_min_max(0.042);
  config.set_activate_lp_filter(16.2);
  config.set_activate_sample_time(4);
  config.set_activate_max_speed_deviation(30.0);
  config.set_run_cond_time(200.1);
  config.set_run_ramp_in_time(2.03);
  config.set_run_pos_corr_gain(3.2);
  config.set_stop_ramp_out_time(2.1);

  ApplyAndCheckJson(config);
}

}  // namespace
