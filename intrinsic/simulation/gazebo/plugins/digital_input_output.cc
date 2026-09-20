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

#include "intrinsic/simulation/gazebo/plugins/digital_input_output.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/cleanup/cleanup.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "gz/msgs/MessageTypes.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Types.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/transport/Node.hh"
#include "intrinsic/simulation/gazebo/components/digital_io_components.h"
#include "sdf/Element.hh"
#include "sdf/Param.hh"

namespace intrinsic::simulation {

namespace {
constexpr char kInputBlockTag[] = "input_block";
constexpr char kOutputBlockTag[] = "output_block";
constexpr char kNameAttribute[] = "name";
constexpr char kNumBitsAttribute[] = "num_bits";
constexpr char kTopicNameAttribute[] = "topic_name";
constexpr char kIsLegacyAttribute[] = "is_legacy";

absl::StatusOr<DigitalInputOutput::DigitalInputDescription>
GetDigitalInputDescription(const ::sdf::Element& input_element,
                           absl::string_view model_name) {
  DigitalInputOutput::DigitalInputDescription input_data;

  ::sdf::ParamPtr name_attr = input_element.GetAttribute(kNameAttribute);
  if (name_attr == nullptr) {
    return absl::InvalidArgumentError(absl::StrCat(
        "<", kInputBlockTag, "> in DigitalInputOutput system for '", model_name,
        "' is missing '", kNameAttribute, "' attribute"));
  }
  input_data.name = name_attr->GetAsString();
  if (input_data.name.empty()) {
    return absl::InvalidArgumentError(
        absl::StrCat("'", kNameAttribute, "' attribute of <", kInputBlockTag,
                     "> must not be empty (in DigitalInputOutput system for '",
                     model_name, "')"));
  }
  ::sdf::ParamPtr num_bits_attr = input_element.GetAttribute(kNumBitsAttribute);
  if (num_bits_attr == nullptr) {
    return absl::InvalidArgumentError(absl::StrCat(
        "<", kInputBlockTag, "> in DigitalInputOutput system for '", model_name,
        "' is missing '", kNumBitsAttribute, "' attribute"));
  }

  if (!num_bits_attr->Get<int>(input_data.num_bits) ||
      input_data.num_bits <= 0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "<", kInputBlockTag, "> of DigitalInputOutput system for '", model_name,
        "' has invalid '", kNumBitsAttribute,
        "' attribute. Must be a positive number, but is ",
        num_bits_attr->GetAsString()));
  }

  ::sdf::ParamPtr topic_name_attr =
      input_element.GetAttribute(kTopicNameAttribute);
  if (topic_name_attr != nullptr) {
    input_data.topic_name = topic_name_attr->GetAsString();
    if (input_data.topic_name->empty()) {
      return absl::InvalidArgumentError(absl::StrCat(
          "'", kTopicNameAttribute, "' attribute in <", kInputBlockTag,
          "> must not be empty (in DigitalInputOutput system for '", model_name,
          "')"));
    }
  }
  ::sdf::ParamPtr is_legacy_attr =
      input_element.GetAttribute(kIsLegacyAttribute);
  if (is_legacy_attr != nullptr) {
    if (!is_legacy_attr->Get(input_data.is_legacy)) {
      return absl::InvalidArgumentError(absl::StrCat(
          "'", kIsLegacyAttribute, "' attribute in <", kInputBlockTag,
          "> must be a boolean value (in DigitalInputOutput system for '",
          model_name, "')"));
    }
  }
  return input_data;
}

absl::StatusOr<DigitalInputOutput::DigitalOutputDescription>
GetDigitalOutputDescription(const ::sdf::Element& output_element,
                            absl::string_view model_name) {
  DigitalInputOutput::DigitalOutputDescription output_data;

  ::sdf::ParamPtr name_attr = output_element.GetAttribute(kNameAttribute);
  if (name_attr == nullptr) {
    return absl::InvalidArgumentError(absl::StrCat(
        "<", kOutputBlockTag, "> in DigitalInputOutput system for '",
        model_name, "' is missing '", kNameAttribute, "' attribute"));
  }
  output_data.name = name_attr->GetAsString();
  if (output_data.name.empty()) {
    return absl::InvalidArgumentError(
        absl::StrCat("'", kNameAttribute, "' attribute of <", kOutputBlockTag,
                     "> must not be empty (in DigitalInputOutput system for '",
                     model_name, "')"));
  }
  ::sdf::ParamPtr num_bits_attr =
      output_element.GetAttribute(kNumBitsAttribute);
  if (num_bits_attr == nullptr) {
    return absl::InvalidArgumentError(absl::StrCat(
        "<", kOutputBlockTag, "> in DigitalInputOutput system for '",
        model_name, "' is missing '", kNumBitsAttribute, "' attribute"));
  }

  if (!num_bits_attr->Get<int>(output_data.num_bits) ||
      output_data.num_bits <= 0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "<", kOutputBlockTag, "> of DigitalInputOutput system for '",
        model_name, "' has invalid '", kNumBitsAttribute,
        "' attribute. Must be a positive number, but is ",
        num_bits_attr->GetAsString()));
  }

  ::sdf::ParamPtr topic_name_attr =
      output_element.GetAttribute(kTopicNameAttribute);
  if (topic_name_attr != nullptr) {
    output_data.topic_name = topic_name_attr->GetAsString();
    if (output_data.topic_name->empty()) {
      return absl::InvalidArgumentError(absl::StrCat(
          "'", kTopicNameAttribute, "' attribute in <", kOutputBlockTag,
          "> must not be empty (in DigitalInputOutput system for '", model_name,
          "')"));
    }
  }
  ::sdf::ParamPtr is_legacy_attr =
      output_element.GetAttribute(kIsLegacyAttribute);
  if (is_legacy_attr != nullptr) {
    if (!is_legacy_attr->Get(output_data.is_legacy)) {
      return absl::InvalidArgumentError(absl::StrCat(
          "'", kIsLegacyAttribute, "' attribute in <", kOutputBlockTag,
          "> must be a boolean value (in DigitalInputOutput system for '",
          model_name, "')"));
    }
  }
  return output_data;
}

gz::msgs::UInt32 GetOutputMessage(const std::vector<bool>& bit_vector) {
  uint32_t data = 0;
  for (size_t i = 0; i < std::min(bit_vector.size(), 8 * sizeof(data)); ++i) {
    if (bit_vector.at(i)) {
      data |= (1 << i);
    }
  }
  gz::msgs::UInt32 msg;
  msg.set_data(data);
  return msg;
}

}  // namespace

void DigitalInputOutput::Configure(
    const gz::sim::Entity& entity,
    const std::shared_ptr<const ::sdf::Element>& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager& event_manager) {
  model_ = gz::sim::Model{entity};
  auto remove_components_on_error = absl::MakeCleanup([&]() {
    for (const auto& [entity, component] : input_components_) {
      ecm.RequestRemoveEntity(entity);
    }
    input_components_.clear();
    for (const auto& [entity, component] : output_components_) {
      ecm.RequestRemoveEntity(entity);
    }
    output_components_.clear();
    // Clearing this deletes the Publishers and should unadvertise the
    // output topics.
    output_entity_to_publisher_.clear();
    for (const auto& input_topic : subscribed_input_topics_) {
      node_.Unsubscribe(input_topic);
    }
    subscribed_input_topics_.clear();
    // Now that all of the subscription callbacks should be gone, we can clear
    // the input data buffer.
    absl::MutexLock l(input_data_buffer_->mutex);
    input_data_buffer_->data.clear();
  });

  absl::flat_hash_set<std::string> input_names;
  for (::sdf::ElementConstPtr input_element = sdf->FindElement(kInputBlockTag);
       input_element != nullptr;
       input_element = input_element->GetNextElement(kInputBlockTag)) {
    absl::StatusOr<DigitalInputDescription> input_data =
        GetDigitalInputDescription(*input_element, model_.Name(ecm));
    if (!input_data.ok()) {
      LOG(ERROR) << input_data.status();
      return;
    }
    absl::Status add_input_status =
        AddInputBlock(*input_data, ecm, input_names);
    if (!add_input_status.ok()) {
      LOG(ERROR) << add_input_status;
      return;
    }
  }

  absl::flat_hash_set<std::string> output_names;
  for (::sdf::ElementConstPtr output_element =
           sdf->FindElement(kOutputBlockTag);
       output_element != nullptr;
       output_element = output_element->GetNextElement(kOutputBlockTag)) {
    absl::StatusOr<DigitalOutputDescription> output_data =
        GetDigitalOutputDescription(*output_element, model_.Name(ecm));
    if (!output_data.ok()) {
      LOG(ERROR) << output_data.status();
      return;
    }
    absl::Status add_output_status =
        AddOutputBlock(*output_data, ecm, output_names);
    if (!add_output_status.ok()) {
      LOG(ERROR) << add_output_status;
      return;
    }
  }
  std::move(remove_components_on_error).Cancel();
}

void DigitalInputOutput::PreUpdate(const gz::sim::UpdateInfo& info,
                                   gz::sim::EntityComponentManager& ecm) {
  {
    // Copy Input Bits from buffer map to each DigitalInput Component
    absl::MutexLock l(input_data_buffer_->mutex);
    for (auto& [entity, input_from_topic] : input_data_buffer_->data) {
      if (!input_from_topic.has_value()) {
        continue;
      }
      auto& input_bits = input_components_.at(entity)->Data().data;
      for (size_t i = 0; i < std::min(input_bits.size(),
                                      8 * sizeof(input_from_topic->data()));
           ++i) {
        input_bits.at(i) = input_from_topic->data() & (1 << i);
      }
      // Set to nullopt, so we can avoid writing the same update again and
      // again.
      input_from_topic = std::nullopt;
    };
  }

  // Publish Outputs from DigitalOutput Components to gz-transport topics
  for (auto& [entity, publisher] : output_entity_to_publisher_) {
    auto output_data = output_components_.at(entity)->Data();
    if (output_data.data.size() > 32) {
      LOG_EVERY_N_SEC(ERROR, 5)
          << "Output block  '"
          << ecm.ComponentData<gz::sim::components::Name>(entity).value_or(
                 "UNKNOWN")
          << "' in model '" << model_.Name(ecm)
          << "' is larger than maximum size for topic publishing. "
             "Maybe some other system modified the block's size?";
      continue;
    }
    publisher.Publish(GetOutputMessage(output_data.data));
  }
}

absl::Status DigitalInputOutput::AddInputBlock(
    const DigitalInputDescription& input_block,
    gz::sim::EntityComponentManager& ecm,
    absl::flat_hash_set<std::string>& input_names) {
  {
    bool inserted = input_names.emplace(input_block.name).second;
    if (!inserted) {
      return absl::AlreadyExistsError(absl::StrCat(
          "Model '", model_.Name(ecm), "' has multiple <", kInputBlockTag,
          "> tags with the same name ('", input_block.name, "')"));
    }
  }
  // Add child entity and create DigitalInput component on that entity.
  gz::sim::Entity child_entity = ecm.CreateEntity();
  auto remove_child_entity_on_error =
      absl::MakeCleanup([&]() { ecm.RequestRemoveEntity(child_entity); });
  if (!ecm.SetParentEntity(child_entity, model_.Entity())) {
    return absl::InternalError(
        absl::StrCat("Failed to set parent/child relation for digital input '",
                     input_block.name, "' of model '", model_.Name(ecm), "'"));
  }
  DigitalInput* input_component = ecm.CreateComponent<DigitalInput>(
      child_entity, DigitalInput{DigitalIoData{
                        .data = std::vector<bool>(input_block.num_bits, false),
                        .is_legacy = input_block.is_legacy,
                    }});

  if (input_component == nullptr) {
    return absl::InternalError(
        absl::StrCat("Failed to add DigitalInput Component for digital input '",
                     input_block.name, "' of model '", model_.Name(ecm), "'"));
  }
  // Add ParentEntity and Name Components. These are important for consumers
  // that want to use the ECM, rather than the gz-transport topic, to interact
  // with the input.
  ecm.SetComponentData<gz::sim::components::ParentEntity>(child_entity,
                                                          model_.Entity());
  ecm.SetComponentData<gz::sim::components::Name>(child_entity,
                                                  input_block.name);

  // Subscribe to gz-transport topic (if configured)
  if (input_block.topic_name.has_value()) {
    // If we have a topic name, we must limit the number of bits to 32 to be
    // backwards compatible (since the topic uses a UInt32 message)
    if (input_block.num_bits > 32) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Input block '", input_block.name, "' in model '", model_.Name(ecm),
          "' has a 'topic_name', but more than 32 bits. If you need more than "
          "32 bits, make sure all consumers of this input block use the "
          "DigitalInput component rather than publishing to a topic, and then "
          "remove the `topic_name` from your configuration"));
    }

    if (bool inserted =
            subscribed_input_topics_.emplace(input_block.topic_name.value())
                .second;
        !inserted) {
      return absl::AlreadyExistsError(absl::StrCat(
          "Will not subscribe to digital input topic '",
          input_block.topic_name.value(), "' for input block '",
          input_block.name, "' in model '", model_.Name(ecm),
          "'. There is another block using that same topic, which could lead "
          "to conflicts. Please check your configuration"));
    }
    std::function<void(const gz::msgs::UInt32&)> on_dio_input =
        // Make a **copy** of input_data_buffer_, so that the callback keeps
        // alive the shared_ptr until it exits scope (callbacks might execute
        // after DigitalInputOutput's destructor returns!)
        [input_data_buffer = input_data_buffer_,
         child_entity](const gz::msgs::UInt32& dio_msg) {
          absl::MutexLock lock(input_data_buffer->mutex);
          input_data_buffer->data[child_entity] = dio_msg;
        };
    if (!node_.Subscribe(input_block.topic_name.value(), on_dio_input)) {
      return absl::InternalError(
          absl::StrCat("Failed to subscribe to digital input topic '",
                       input_block.topic_name.value(), "' for input block '",
                       input_block.name, "' in model '", model_.Name(ecm),
                       "'. This could be because another input block is using "
                       "the same topic name, please check your configuration"));
    }
  }

  // Don't check the insertion here – if we can't trust the ECM that
  // child_entity is unique, then things are very bad anyway.
  input_components_.emplace(child_entity, input_component);
  std::move(remove_child_entity_on_error).Cancel();
  return absl::OkStatus();
}

absl::Status DigitalInputOutput::AddOutputBlock(
    const DigitalOutputDescription& output_block,
    gz::sim::EntityComponentManager& ecm,
    absl::flat_hash_set<std::string>& output_names) {
  {
    bool inserted = output_names.emplace(output_block.name).second;
    if (!inserted) {
      return absl::AlreadyExistsError(absl::StrCat(
          "Model '", model_.Name(ecm), "' has multiple <", kOutputBlockTag,
          "> tags with the same name ('", output_block.name, "')"));
    }
  }

  // Add child entity and create DigitalOutput component on that entity.
  gz::sim::Entity child_entity = ecm.CreateEntity();
  auto remove_child_entity_on_error =
      absl::MakeCleanup([&]() { ecm.RequestRemoveEntity(child_entity); });
  if (!ecm.SetParentEntity(child_entity, model_.Entity())) {
    return absl::InternalError(
        absl::StrCat("Failed to set parent/child relation for digital output '",
                     output_block.name, "' of model '", model_.Name(ecm), "'"));
  }
  DigitalOutput* output_component = ecm.CreateComponent<DigitalOutput>(
      child_entity, DigitalOutput{DigitalIoData{
                        .data = std::vector<bool>(output_block.num_bits, false),
                        .is_legacy = output_block.is_legacy,
                    }});

  if (output_component == nullptr) {
    return absl::InternalError(absl::StrCat(
        "Failed to add DigitalOutput Component for digital output '",
        output_block.name, "' of model '", model_.Name(ecm), "'"));
  }
  // Add ParentEntity and Name Components. These are important for consumers
  // that want to use the ECM, rather than the gz-transport topic, to interact
  // with the output.
  ecm.SetComponentData<gz::sim::components::ParentEntity>(child_entity,
                                                          model_.Entity());
  ecm.SetComponentData<gz::sim::components::Name>(child_entity,
                                                  output_block.name);
  // Advertise gz-transport topic (if configured)
  if (output_block.topic_name.has_value()) {
    // If we have a topic name, we must limit the number of bits to 32 to be
    // backwards compatible (since the topic uses a UInt32 message)
    if (output_block.num_bits > 32) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Output block '", output_block.name, "' in model '", model_.Name(ecm),
          "' has a 'topic_name', but more than 32 bits. If you need more than "
          "32 bits, make sure all consumers of this output block use the "
          "DigitalOutputs component rather than subscribing to a topic, and "
          "then remove the `topic_name` from your configuration"));
    }
    gz::transport::Node::Publisher publisher =
        node_.Advertise<gz::msgs::UInt32>(output_block.topic_name.value());
    if (!publisher) {
      return absl::InternalError(absl::StrCat(
          "Failed to advertise digital output topic '",
          output_block.topic_name.value(), "' for output block '",
          output_block.name, "' in model '", model_.Name(ecm), "'"));
    }
    // Don't check insertion here - if this output had a duplicate name, we
    // would have bailed out above.
    output_entity_to_publisher_.emplace(child_entity, std::move(publisher));
  }

  {
    // Insert the child into the output_components_ map.
    bool inserted =
        output_components_.try_emplace(child_entity, output_component).second;
    if (!inserted) {
      return absl::InternalError(absl::StrCat(
          "Failed to save data for digital output '", output_block.name,
          "' of model '", model_.Name(ecm), "'"));
    }
  }

  std::move(remove_child_entity_on_error).Cancel();
  return absl::OkStatus();
}

}  // namespace intrinsic::simulation
