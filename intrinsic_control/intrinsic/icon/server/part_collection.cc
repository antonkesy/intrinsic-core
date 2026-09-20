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

#include "intrinsic/icon/server/part_collection.h"

#include <iterator>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/icon/common/part_properties.h"
#include "intrinsic/icon/proto/generic_part_config.pb.h"
#include "intrinsic/icon/proto/part_status.pb.h"
#include "intrinsic/icon/proto/safety_status.pb.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/server/parts_manager.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

namespace {
absl::flat_hash_set<std::string> GetPartNames(
    absl::Span<const intrinsic_proto::icon::v1::PartConfig> part_configs) {
  absl::flat_hash_set<std::string> names;
  absl::c_transform(part_configs, std::inserter(names, names.end()),
                    [](const auto& config) { return config.name(); });
  return names;
}

// Returns the set of strings that are in `lhs` but not `rhs`;
absl::flat_hash_set<std::string> SetDifference(
    const absl::flat_hash_set<std::string>& lhs,
    const absl::flat_hash_set<std::string>& rhs) {
  absl::flat_hash_set<std::string> output_set;
  absl::c_copy_if(lhs, std::inserter(output_set, output_set.end()),
                  [&rhs](const auto& s) { return !rhs.contains(s); });
  return output_set;
}

// Adjusts and returns the provided PartConfigs with the correct limits based on
// `mode` and the contained `safety_limits_config`. Returns a
// FailedPreconditionError if a PartConfig contains a GenericSafetyConfig, but
// no joint/Cartesian limits for 'MODE_OF_SAFE_OPERATION_UNKNOWN' (the
// fallback).
static absl::StatusOr<std::vector<intrinsic_proto::icon::v1::PartConfig>>
CreatePartConfigsForMode(
    intrinsic_proto::icon::ModeOfSafeOperation mode,
    absl::Span<const intrinsic_proto::icon::v1::PartConfig> plain_configs) {
  std::vector<intrinsic_proto::icon::v1::PartConfig> part_configs;
  part_configs.reserve(plain_configs.size());

  for (const auto& part_config : plain_configs) {
    if (!part_config.has_generic_config()) {
      LOG(WARNING) << "Part '" << part_config.name()
                   << "' doesn't have a generic_config.";
      part_configs.push_back(part_config);
      continue;
    }

    if (!part_config.generic_config().has_safety_limits_config()) {
      LOG(INFO) << "Part '" << part_config.name()
                << "' doesn't have a safety_limits_config. Using default "
                   "config for "
                << intrinsic_proto::icon::ModeOfSafeOperation_Name(mode);
      part_configs.push_back(part_config);
      continue;
    }

    if (part_config.generic_config().has_joint_limits_config() ||
        part_config.generic_config().has_cartesian_limits_config()) {
      LOG(WARNING) << "Part '" << part_config.name()
                   << " defines 'safety_limits_config' and "
                      "'joint_limits_config'/'cartesian_limits_config'. Will "
                      "use 'safety_limits_config'.";
    }

    intrinsic_proto::icon::GenericJointLimitsConfig
        adjusted_joint_limits_config;
    intrinsic_proto::icon::GenericCartesianLimitsConfig
        adjusted_cartesian_limits_config;
    {
      // Lookup joint limits.
      const auto& joint_limits_map = part_config.generic_config()
                                         .safety_limits_config()
                                         .joint_limits_map();
      const auto& jlimit_it = joint_limits_map.find(
          intrinsic_proto::icon::ModeOfSafeOperation_Name(mode));
      if (jlimit_it != joint_limits_map.end()) {
        LOG(INFO) << "Part '" << part_config.name()
                  << "' contains joint limits for '"
                  << intrinsic_proto::icon::ModeOfSafeOperation_Name(mode)
                  << "'.";
        adjusted_joint_limits_config = jlimit_it->second;
      } else if (const auto& jlimit_fallback_it = joint_limits_map.find(
                     intrinsic_proto::icon::ModeOfSafeOperation_Name(
                         intrinsic_proto::icon::
                             MODE_OF_SAFE_OPERATION_UNKNOWN));
                 jlimit_fallback_it != joint_limits_map.end()) {
        LOG(WARNING)
            << "Part '" << part_config.name()
            << "' doesn't define joint limits for '"
            << intrinsic_proto::icon::ModeOfSafeOperation_Name(mode)
            << "' using limits for '"
            << intrinsic_proto::icon::ModeOfSafeOperation_Name(
                   intrinsic_proto::icon::MODE_OF_SAFE_OPERATION_UNKNOWN)
            << "'.";
        adjusted_joint_limits_config = jlimit_fallback_it->second;
      } else {
        // Every part with a safety_limits_config needs to define limits for
        // MODE_OF_SAFE_OPERATION_UNKNOWN, as they are used as fallback.
        return absl::FailedPreconditionError(absl::StrCat(
            "Part '", part_config.name(), "' doesn't define joint limits for '",
            intrinsic_proto::icon::ModeOfSafeOperation_Name(mode),
            "' or MODE_OF_SAFE_OPERATION_UNKNOWN."));
      }
    }
    {
      // Lookup Cartesian limits.
      const auto& cartesian_limits_map = part_config.generic_config()
                                             .safety_limits_config()
                                             .cartesian_limits_map();
      const auto& cart_it = cartesian_limits_map.find(
          intrinsic_proto::icon::ModeOfSafeOperation_Name(mode));
      if (cart_it != cartesian_limits_map.end()) {
        LOG(INFO) << "Part '" << part_config.name()
                  << "' contains cartesian limits for '"
                  << intrinsic_proto::icon::ModeOfSafeOperation_Name(mode)
                  << "'.";
        adjusted_cartesian_limits_config = cart_it->second;
      } else if (const auto& cart_limit_fallback_it = cartesian_limits_map.find(
                     intrinsic_proto::icon::ModeOfSafeOperation_Name(
                         intrinsic_proto::icon::
                             MODE_OF_SAFE_OPERATION_UNKNOWN));
                 cart_limit_fallback_it != cartesian_limits_map.end()) {
        LOG(WARNING)
            << "Part '" << part_config.name()
            << "' doesn't define Cartesian limits for '"
            << intrinsic_proto::icon::ModeOfSafeOperation_Name(mode)
            << "' using limits for '"
            << intrinsic_proto::icon::ModeOfSafeOperation_Name(
                   intrinsic_proto::icon::MODE_OF_SAFE_OPERATION_UNKNOWN)
            << "'.";
        adjusted_cartesian_limits_config = cart_limit_fallback_it->second;
      } else {
        // Every part with a safety_limits_config needs to define limits for
        // MODE_OF_SAFE_OPERATION_UNKNOWN, as they are used as fallback.
        return absl::FailedPreconditionError(
            absl::StrCat("Part '", part_config.name(),
                         "' doesn't define Cartesian limits for '",
                         intrinsic_proto::icon::ModeOfSafeOperation_Name(mode),
                         "' or MODE_OF_SAFE_OPERATION_UNKNOWN."));
      }
    }

    intrinsic_proto::icon::v1::PartConfig adjusted_part_config = part_config;
    *adjusted_part_config.mutable_generic_config()
         ->mutable_joint_limits_config() = adjusted_joint_limits_config;
    *adjusted_part_config.mutable_generic_config()
         ->mutable_cartesian_limits_config() = adjusted_cartesian_limits_config;

    part_configs.push_back(adjusted_part_config);
  }
  return part_configs;
}

// Creates and maps part_configs to all ModeOfSafeOperation.
// Returns an error if adjusting a PartConfig to a ModeOfSafeOperation fails.
static absl::StatusOr<absl::flat_hash_map<
    intrinsic_proto::icon::ModeOfSafeOperation,
    const std::vector<intrinsic_proto::icon::v1::PartConfig>>>
CreatePartConfigsMap(
    absl::Span<const intrinsic_proto::icon::v1::PartConfig> plain_configs) {
  absl::flat_hash_map<intrinsic_proto::icon::ModeOfSafeOperation,
                      const std::vector<intrinsic_proto::icon::v1::PartConfig>>
      configs_map;
  // Fill the configs map with a set of PartConfigs for every
  // ModeOfSafeOperation.
  // LINT.IfChange(ModeOfSafeOperation)
  auto modes = std::vector<intrinsic_proto::icon::ModeOfSafeOperation>({
      intrinsic_proto::icon::ModeOfSafeOperation::
          MODE_OF_SAFE_OPERATION_UNKNOWN,
      intrinsic_proto::icon::ModeOfSafeOperation::
          MODE_OF_SAFE_OPERATION_AUTOMATIC,
      intrinsic_proto::icon::ModeOfSafeOperation::
          MODE_OF_SAFE_OPERATION_TEACHING_1,
      intrinsic_proto::icon::ModeOfSafeOperation::
          MODE_OF_SAFE_OPERATION_TEACHING_2,
      intrinsic_proto::icon::ModeOfSafeOperation::
          MODE_OF_SAFE_OPERATION_CONFIGURATION,
      intrinsic_proto::icon::ModeOfSafeOperation::
          MODE_OF_SAFE_OPERATION_UPDATING,
  });
  // LINT.ThenChange()
  for (const auto mode : modes) {
    INTR_ASSIGN_OR_RETURN(const auto adjusted_configs,
                          CreatePartConfigsForMode(mode, plain_configs));
    configs_map.emplace(mode, adjusted_configs);
  }
  return configs_map;
}

}  // namespace

// static
absl::StatusOr<PartCollection> PartCollection::Create(
    absl::Span<const intrinsic_proto::icon::v1::PartConfig> part_configs,
    std::unique_ptr<StatusGetter> status_getter) {
  absl::flat_hash_set<std::string> config_part_names;
  for (const auto& part_config : part_configs) {
    if (bool inserted = config_part_names.insert(part_config.name()).second;
        !inserted) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Duplicate part name '", part_config.name(), "' in configuration"));
    }
  }

  const auto part_names = GetPartNames(part_configs);
  INTR_ASSIGN_OR_RETURN(const auto part_configs_map,
                        CreatePartConfigsMap(part_configs));

  return PartCollection(std::move(part_names), std::move(config_part_names),
                        std::move(part_configs_map), std::move(status_getter));
}

std::vector<intrinsic_proto::icon::v1::PartConfig>
PartCollection::GetPartConfigs() const {
  intrinsic_proto::icon::ModeOfSafeOperation current_mode_of_safe_operation =
      intrinsic_proto::icon::MODE_OF_SAFE_OPERATION_UNKNOWN;
  // Reads the SafetyStatus using a 'zero' deadline and returns the correct
  // part_configs. Calling this before the SafetyStatus is populated logs a
  // Warning and returns PartConfigs for MODE_OF_SAFE_OPERATION_TEACHING_1, or
  // MODE_OF_SAFE_OPERATION_UNKNOWN if teaching is not present.
  const auto safety_status_or = GetSafetyStatus(absl::Now());
  if (!safety_status_or.ok()) {
    if (mode_of_safe_operation_part_configs_map_.contains(
            intrinsic_proto::icon::MODE_OF_SAFE_OPERATION_TEACHING_1)) {
      current_mode_of_safe_operation =
          intrinsic_proto::icon::MODE_OF_SAFE_OPERATION_TEACHING_1;
    }
    LOG(WARNING) << "Failed to GetSafetyStatus. Using part_configs for: "
                 << intrinsic_proto::icon::ModeOfSafeOperation_Name(
                        current_mode_of_safe_operation)
                 << " Error: " << safety_status_or.status();
  } else {
    current_mode_of_safe_operation = safety_status_or->mode_of_safe_operation();
  }
  auto it = mode_of_safe_operation_part_configs_map_.find(
      current_mode_of_safe_operation);
  QCHECK(it != mode_of_safe_operation_part_configs_map_.end())
      << "Failed to find part_configs for: "
      << intrinsic_proto::icon::ModeOfSafeOperation_Name(
             current_mode_of_safe_operation);
  LOG_EVERY_N_SEC(INFO, /*seconds=*/1)
      << "Found part_configs for: "
      << intrinsic_proto::icon::ModeOfSafeOperation_Name(
             current_mode_of_safe_operation);
  return it->second;
}

absl::StatusOr<
    absl::flat_hash_map<std::string, intrinsic_proto::icon::PartStatus>>
PartCollection::GetPartStatuses(absl::Time deadline) const {
  INTR_ASSIGN_OR_RETURN(
      (absl::flat_hash_map<std::string, intrinsic_proto::icon::PartStatus>
           statuses),
      status_getter_->GetPartStatuses(deadline));
  absl::flat_hash_set<std::string> status_part_names;
  for (const auto& [name, status] : statuses) {
    if (!status_part_names.insert(name).second) {
      return absl::InternalError(
          absl::StrCat("Duplicate part name '", name, "' in statuses"));
    }
  }
  absl::flat_hash_set<std::string> missing_from_parts =
      SetDifference(status_part_names, config_part_names_);
  absl::flat_hash_set<std::string> missing_from_status =
      SetDifference(config_part_names_, status_part_names);
  if (!(missing_from_parts.empty() && missing_from_status.empty())) {
    return absl::FailedPreconditionError(absl::StrCat(
        "Inconsistent part data:\n", "  Names that don't appear in parts: [",
        absl::StrJoin(missing_from_parts, ", "), "]\n",
        "  Names that don't appear in statuses: [",
        absl::StrJoin(missing_from_status, ", "), "].\n",
        "Must provide `part_configs` to PartCollection that includes all "
        "parts for which a status will be received."));
  }

  return statuses;
}

absl::StatusOr<TimestampedPartProperties> PartCollection::GetPartProperties() {
  return status_getter_->GetPartProperties();
}

absl::Status PartCollection::SetPartProperties(
    const PartPropertyMap& properties) {
  return status_getter_->SetPartProperties(properties);
}

absl::StatusOr<intrinsic_proto::icon::SafetyStatus>
PartCollection::GetSafetyStatus(absl::Time deadline) const {
  return status_getter_->GetSafetyStatus(deadline);
}

internal::PartsManager& PartCollection::GetPartsManager() {
  return *parts_manager_;
}

const internal::PartsManager& PartCollection::GetPartsManager() const {
  return *parts_manager_;
}

PartCollection::PartCollection(
    absl::flat_hash_set<std::string> part_names,
    absl::flat_hash_set<std::string> config_part_names,
    absl::flat_hash_map<
        intrinsic_proto::icon::ModeOfSafeOperation,
        const std::vector<intrinsic_proto::icon::v1::PartConfig>>
        part_configs_map,
    std::unique_ptr<StatusGetter> status_getter)
    : mode_of_safe_operation_part_configs_map_(std::move(part_configs_map)),
      config_part_names_(std::move(config_part_names)),
      status_getter_(std::move(status_getter)),
      parts_manager_(std::make_unique<internal::PartsManager>(part_names)) {}

}  // namespace intrinsic::icon
