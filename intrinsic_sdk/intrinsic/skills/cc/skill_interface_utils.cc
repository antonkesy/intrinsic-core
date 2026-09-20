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

#include "intrinsic/skills/cc/skill_interface_utils.h"

#include <algorithm>
#include <memory>
#include <optional>
#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/time/time.h"
#include "google/protobuf/message.h"
#include "intrinsic/skills/cc/equipment_pack.h"
#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/internal/execute_context_view.h"
#include "intrinsic/skills/internal/predict_context_view.h"  
#include "intrinsic/skills/proto/prediction.pb.h"  
#include "intrinsic/skills/proto/skill_service.pb.h"
#include "intrinsic/util/proto_time.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/proto/object_world_updates.pb.h"

namespace intrinsic {
namespace skills {

absl::StatusOr<std::unique_ptr<::google::protobuf::Message>> PreviewViaExecute(
    SkillExecuteInterface& skill, const PreviewRequest& request,
    PreviewContext& context) {
  EquipmentPack equipment;

  INTR_ASSIGN_OR_RETURN(ExecuteRequest execute_request,
                        PreviewToExecuteRequest(request));
  INTR_ASSIGN_OR_RETURN(ExecuteContextView execute_context,
                        PreviewToExecuteContext(context, equipment));

  return skill.Execute(execute_request, execute_context);
}


absl::Status PreviewViaPredict(SkillInterface& skill,
                               const PreviewRequest& request,
                               PreviewContext& context,
                               ::google::protobuf::Any* result_any) {
  INTR_ASSIGN_OR_RETURN(PredictRequest predict_request,
                        PreviewToPredictRequest(request));
  INTR_ASSIGN_OR_RETURN(PredictContextView predict_context,
                        PreviewToPredictContext(context));
  INTR_ASSIGN_OR_RETURN(intrinsic_proto::skills::PredictResult predict_result,
                        skill.Predict(predict_request, predict_context));

  if (predict_result.outcomes().empty()) {
    return absl::InternalError("Skill produced no predictions.");
  }

  const intrinsic_proto::skills::Prediction& most_likely_outcome =
      *std::max_element(predict_result.outcomes().begin(),
                        predict_result.outcomes().end(),
                        [](const intrinsic_proto::skills::Prediction& a,
                           const intrinsic_proto::skills::Prediction& b) {
                          return a.probability() < b.probability();
                        });

  // Record the world updates from the most likely outcome.
  absl::Time previous_start_time;
  absl::Duration previous_duration;
  for (int i = 0; i < most_likely_outcome.expected_states_size(); ++i) {
    intrinsic_proto::skills::TimedWorldUpdate expected_state =
        most_likely_outcome.expected_states(i);
    absl::Time start_time;
    if (expected_state.has_start_time()) {
      INTR_ASSIGN_OR_RETURN(start_time,
                            ToAbslTime(expected_state.start_time()));
    } else {
      start_time = previous_start_time + previous_duration;
    }
    INTR_ASSIGN_OR_RETURN(absl::Duration duration,
                          ToAbslDuration(expected_state.time_until_update()));

    absl::Duration elapsed = start_time - previous_start_time;
    if (elapsed < absl::ZeroDuration()) {
      return absl::InvalidArgumentError(absl::StrFormat(
          "Expected state start times must be monotonically increasing "
          "(state[%d] start time: %s, state[%d] start time: %s).",
          i, absl::FormatTime(start_time), i - 1,
          absl::FormatTime(previous_start_time)));
    }

    for (const intrinsic_proto::world::ObjectWorldUpdate& update :
         expected_state.world_updates().updates()) {
      INTR_RETURN_IF_ERROR(
          context.RecordWorldUpdate(update, elapsed, duration));

      // All updates in this expected state started at the same time, so all but
      // the first update should have `elapsed = 0`.
      elapsed = absl::ZeroDuration();
    }

    previous_start_time = start_time;
    previous_duration = duration;
  }

  if (result_any != nullptr) {
    *result_any = most_likely_outcome.result();
  }

  return absl::OkStatus();
}


absl::StatusOr<ExecuteRequest> PreviewToExecuteRequest(
    const PreviewRequest& request) {
  return ExecuteRequest(

      std::string(request.internal_data()),

      /*params=*/request.params_any(),
      /*param_defaults=*/std::nullopt);
}

absl::StatusOr<ExecuteContextView> PreviewToExecuteContext(
    PreviewContext& context, const EquipmentPack& equipment) {
  return ExecuteContextView(context.canceller(), equipment,
                            context.logging_context(), context.motion_planner(),
                            context.object_world()

                            ,
                            context.geometry_library()

                            ,
                            std::string(context.context_id()));
}


absl::StatusOr<PredictRequest> PreviewToPredictRequest(
    const PreviewRequest& request) {
  return PredictRequest(/*internal_data=*/std::string(request.internal_data()),
                        /*params=*/request.params_any(),
                        /*param_defaults=*/std::nullopt);
}

absl::StatusOr<PredictContextView> PreviewToPredictContext(
    PreviewContext& context) {
  PredictContextView predict_context(context.equipment(),
                                     context.motion_planner(),
                                     context.object_world()

                                     ,
                                     context.geometry_library()

  );

  return predict_context;
}


}  // namespace skills
}  // namespace intrinsic
