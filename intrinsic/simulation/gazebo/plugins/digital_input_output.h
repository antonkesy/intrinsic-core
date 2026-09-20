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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_DIGITAL_INPUT_OUTPUT_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_DIGITAL_INPUT_OUTPUT_H_

#include <memory>
#include <optional>
#include <string>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "gz/msgs/MessageTypes.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/System.hh"
#include "gz/sim/Types.hh"
#include "gz/transport/Node.hh"
#include "intrinsic/simulation/gazebo/components/digital_io_components.h"
#include "sdf/Element.hh"

namespace intrinsic::simulation {

// This system is a crude simulation of digital inputs and outputs that can be
// attached to a Gazebo model.
// The DigitalInputOutput system
// * Adds entities with DigitalInput or DigitalOutput components to the system's
//   model entity. Other systems that are aware of the DigitalInput/Output
//   components can use this to efficiently control and react to DIO values.
//   For example:
//   The following code gets a reference to the model entity using its full
//   (scoped) name "world::boopbeep" (though some other unique identifier, like
//   the Intrinsic WorldObject name, would also work). It then finds the model's
//   child entity with the name "beep_emitters", accesses its DigitalOutput
//   component, and reads the first output bit.
//   ```c++
//   auto entity_set = gz::sim::entitiesFromScopedName("world::boopbeep", ecm);
//   if (entity_set.size() == 1) {
//     gz::sim::Entity model_entity = entity_set.front();
//     auto io_block_entities =
//         ecm.ChildrenByComponents(model_entity, Name("beep_emitters"));
//     if (io_block_entities != 1) {
//       // Didn't find DIO block in device, or found more than one, bail out
//       return;
//     }
//     DigitalOutput* output_component = ecm.Component<DigitalOutput>(
//         io_block_entities.front());
//     if (output_component == nullptr) {
//       // Missing DigitalOutput Component, bail out.
//       return;
//     }
//     // Each input/output block is a vector<bool>.
//     // Read the first output bit.
//     this->close_gripper_ = output_component->Data()->at(0);
//   }
//   ```
// * Creates gz-transport topics with the message type `gz::msgs::UInt32` to
//   report outputs and receive inputs. Systems that _aren't_ aware of the
//   DigitalInput/Output components can use this to control and react to DIO
//   values, but it comes with drawbacks. Most obviously, gz-transport does not
//   make timing guarantees about when a message gets delivered. In the worst
//   case, a DIO command could "slip" by one or more simulation steps, which
//   cannot happen when using the Components directly.
//
// The DigitalInputOutput system accepts `<input_block>` and `<output_block>`
// sub-tags for configuration, and both have the same attributes:
// * name: Name of the block. This becomes the map key in the
//         DigitalInputs/DigitalOutputs component.
// * num_bits: The number of bits in this block. Must be greater than zero.
// * topic_name: Name of the `gz::msgs::UInt32` gz-transport topic for this
//               input/output block.
//               This is optional. If not present, the system does not advertise
//               any topic! If this is set, num_bits cannot be greater than 32
// * is_legacy: This marks blocks that were automatically created from World
//              objects that still use timeslicer devices. If true, then the
//              simulated hardware module appends a suffix to the name of the
//              input/output. We can't just append the suffix to the name when
//              we convert from World object to SDF, because
//              SimBusHardwareModuleConfig uses the un-suffixed name as a key in
//              the map that prescribes the final hardware interface names, and
//              we might want to look that up.
//
// Example SDF fragment:
// <model name="boopbeep">
//   <plugin filename="static://intrinsic::simulation::DigitalInputOutput"
//           name="DigitalInputOutput">
//     <input_block name="boop_detectors"
//                  num_bits="16"
//                  topic_name="boop_detectors_dio_in" />
//     <output_block name="beep_emitters"
//                   num_bits="8"
//                   topic_name="beep_emitters_dio_out" />
//   </plugin>
// </model>
// <model name="grasp_when_beep">
//   <!-- This grips when the beep_emitter topic above sends a 1 -->
//   <plugin filename="static://intrinsic::simulation::FixedJointGripperPlugin">
//     <dio_command_topic>"beep_emitters_dio_out"</dio_command_topic>
//     <dio_command_grip>1</dio_command_grip>
//     <dio_command_release>0</dio_command_release>
//   </plugin>
// </model>
class DigitalInputOutput final : public gz::sim::System,
                                 public gz::sim::ISystemConfigure,
                                 public gz::sim::ISystemPreUpdate {
 public:
  struct DigitalInputDescription {
    std::string name;
    int num_bits = 0;
    std::optional<std::string> topic_name;
    bool is_legacy = false;
  };

  struct DigitalOutputDescription {
    std::string name;
    int num_bits = 0;
    std::optional<std::string> topic_name;
    bool is_legacy = false;
  };

  void Configure(const gz::sim::Entity& entity,
                 const std::shared_ptr<const ::sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm,
                 gz::sim::EventManager& event_manager) override;

  void PreUpdate(const gz::sim::UpdateInfo& info,
                 gz::sim::EntityComponentManager& ecm) override;

 private:
  // We can only safely access the components while in our own
  // PreUpdate() function, but gz-transport callbacks want to write new
  // input data too.
  //
  // This mutex and buffer allow the callbacks to write new data at any time,
  // and the PreUpdate() function to copy that data into the DigitalInput
  // components in `input_components_` when we have exclusive access to those.
  struct InputDataBuffer {
    absl::Mutex mutex;
    // This map buffers the latest `UInt32` message from each DigitalInput
    // gz-transport topic.
    // The map values are `std::optional`s to indicate whether or not there's
    // fresh data from the topic. This is important in the (admittedly
    // hypothetical) case where inputs receive new data from both the ECM *and*
    // gz-transport. If we just unconditionally copied data from the
    // gz-transport topic to the ECM, we'd stomp the ECM updates! So,
    // std::nullopt means "no new data since the last time `PreUpdate()` read
    // the buffer". That is, `PreUpdate()` is responsible for setting the value
    // to std::nullopt, and the gz-transport callback is responsible for writing
    // a new `UInt32` whenever it receives one.
    absl::flat_hash_map<gz::sim::Entity, std::optional<gz::msgs::UInt32>> data
        ABSL_GUARDED_BY(mutex);
  };

  // Adds a child entity to `model_` and sets up several Components according to
  // `input_block`:
  //
  // * `intrinsic::simulation::DigitalInput`:
  //   This carries the actual digital input data, and its payload has
  //   `input_block.num_bits` entries.
  // * `gz::sim::components::ParentEntity` and `gz::sim::components::Name`:
  //   The ECM has no direct concept of names, and while it does have an
  //   internal graph of parent/child relations, it offers no way to directly
  //   query it.
  //   So a consumer that wants to implement "Set bit 1 of input block
  //   'holy_diver' on model 'ronnie_james_dio' has to do something like this:
  //
  //   Entity parent = ecm.EntityByName("ronnie_james_dio");
  //   Entity input_block = ecm.EntityByComponent(
  //       ParentEntity(parent),
  //       Name(holy_diver));
  //   ecm.ComponentData<DigitalInput>(input_block)[1] = true;
  //
  //   But to ensure this works, we need to actually add those components
  //   ourself, because the ECM doesn't do it automatically.
  //
  // If `input_block` has a `topic_name`, this subscribes to that topic with a
  // callback that expects a gz::msgs::UInt32 message and updates the data in
  // the corresponding child Entity's DigitalInput Component.
  // NOTE: Because the topic is a gz::msgs::UInt32, setting
  // `input_block.num_bits` to a value greater than 32 *and* providing a topic
  // name is an error.
  //
  // Returns an error if
  // * `input_block.name` is already in `input_names` (i.e. if the name is not
  //   unique)
  // * `input_block.topic_name` is set *and* `input_name.num_bits` is greater
  //   than 32
  // * any of the ECM interactions (adding a child entity and setting the
  //   Components on it) fails. Note that this function takes care of removing
  //   the child entity (if it already exists) before returning an error.
  absl::Status AddInputBlock(const DigitalInputDescription& input_block,
                             gz::sim::EntityComponentManager& ecm,
                             absl::flat_hash_set<std::string>& input_names);

  // Sets up the DigitalOutputs data entry for `output_block`, creating a child
  // entity and adding components in a similar manner to `AddInputBlock()`.
  //
  // If `output_block`has a `topic_name`, this advertises that topic with
  // gz::msgs::UInt32 as its message type. It then adds that publisher to
  // `output_name_to_publisher_`, so that the PreUpdate() function can use it to
  // publish DIO data.
  absl::Status AddOutputBlock(const DigitalOutputDescription& output_block,
                              gz::sim::EntityComponentManager& ecm,
                              absl::flat_hash_set<std::string>& output_names);

  gz::sim::Model model_;
  // Map from the ID for a DigitalInput child entity to its DigitalInput*
  // Component.
  absl::flat_hash_map<gz::sim::Entity, DigitalInput*> input_components_;

  // This is a shared_ptr to avoid lifetime issues when destroying the
  // DigitalInputOutput system. There's a (slim but real) chance that threads
  // are interleaved in a way that, despite our best efforts, a DigitalInput
  // gz-transport callback tries to access its buffer after the
  // DigitalInputOutput destructor finishes.
  std::shared_ptr<InputDataBuffer> input_data_buffer_ =
      std::make_shared<InputDataBuffer>();

  // Map from the ID for a DigitalOutput child entity to its DigitalOutput*
  // Component.
  // Note that there isn't a buffer like for the inputs, because we only write
  // to the DigitalOutput components in PreUpdate().
  absl::flat_hash_map<gz::sim::Entity, DigitalOutput*> output_components_;
  absl::flat_hash_map<gz::sim::Entity, gz::transport::Node::Publisher>
      output_entity_to_publisher_;
  absl::flat_hash_set<std::string> subscribed_input_topics_;
  // Gazebo transport node used to advertise/subscribe to the I/O topics
  // Declare this last so that it is destroyed first, since the callbacks
  // reference other members of this class.
  // TODO(b/363247945): Remove once all consumers of simulated IOs use the
  // Components
  gz::transport::Node node_;
};
}  // namespace intrinsic::simulation

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_DIGITAL_INPUT_OUTPUT_H_
